// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 BitScrambler memory loopback. Programs use the SDK bsasm v1
 * format; native DMAengine owns descriptors, cache maintenance and IRQs.
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/compat.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <uapi/linux/esp32s31-accelerators.h>

#define BS_CTRL 0x28
#define BS_STATE 0x30
#define BS_HALT BIT(2)
struct s31_bs {
	struct miscdevice misc;
	struct device *dev;
	void __iomem *regs, *route, *memory;
	struct clk *clk;
	struct reset_control *reset;
	struct mutex lock;
	struct kref refs;
	bool removed;
};
struct s31_bs_wait {
	struct completion done;
	struct dmaengine_result result;
};
static void s31_bs_done(void *arg, const struct dmaengine_result *result)
{
	struct s31_bs_wait *w = arg;

	w->result =
		result ? *result :
			 (struct dmaengine_result){ .result =
							    DMA_TRANS_ABORTED };
	complete(&w->done);
}
static int s31_bs_submit(struct dma_chan *ch, struct sg_table *sg,
			 enum dma_transfer_direction dir, struct s31_bs_wait *w)
{
	struct dma_async_tx_descriptor *d;
	dma_cookie_t cookie;

	init_completion(&w->done);
	d = dmaengine_prep_slave_sg(ch, sg->sgl, sg->nents, dir,
				    DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!d)
		return -ENOMEM;
	d->callback_result = s31_bs_done;
	d->callback_param = w;
	cookie = dmaengine_submit(d);
	return dma_submit_error(cookie);
}
static int s31_bs_wait(struct s31_bs_wait *w, unsigned long deadline)
{
	long ret = wait_for_completion_interruptible_timeout(
		&w->done,
		time_after(deadline, jiffies) ? deadline - jiffies : 1);
	if (ret <= 0)
		return ret ? ret : -ETIMEDOUT;
	return w->result.result == DMA_TRANS_NOERROR ? 0 : -EIO;
}
static void s31_bs_free(struct kref *ref)
{
	kfree(container_of(ref, struct s31_bs, refs));
}
static int s31_bs_open(struct inode *inode, struct file *file)
{
	struct s31_bs *b =
		container_of(file->private_data, struct s31_bs, misc);
	kref_get(&b->refs);
	file->private_data = b;
	return nonseekable_open(inode, file);
}
static int s31_bs_close(struct inode *inode, struct file *file)
{
	struct s31_bs *b = file->private_data;

	kref_put(&b->refs, s31_bs_free);
	return 0;
}
/* Aligned userspace streams can be mapped directly. Pinning bounds their
 * lifetime even if a different thread unmaps the virtual address. Unaligned
 * streams use owned bounce storage so cache maintenance cannot touch a
 * neighbouring user's allocation. Overlapping streams also need a snapshot.
 */
struct s31_bs_buffer {
	struct sg_table sg;
	struct page **pages;
	unsigned int nr_pages;
	void *bounce;
	struct device *dev;
	enum dma_data_direction direction;
	bool mapped, dirty;
};
static void s31_bs_buffer_release(struct s31_bs_buffer *buf)
{
	if (buf->mapped)
		dma_unmap_sgtable(buf->dev, &buf->sg, buf->direction, 0);
	if (buf->pages) {
		unpin_user_pages_dirty_lock(buf->pages, buf->nr_pages,
					    buf->dirty);
		kfree(buf->pages);
	}
	if (buf->sg.sgl)
		sg_free_table(&buf->sg);
	kfree(buf->bounce);
	memset(buf, 0, sizeof(*buf));
}

static bool s31_bs_buffers_overlap(struct s31_bs_buffer *input,
				   struct s31_bs_buffer *output)
{
	struct scatterlist *a, *b;
	unsigned int i, j;

	for_each_sg(input->sg.sgl, a, input->sg.orig_nents, i) {
		u64 start = sg_phys(a), end = start + a->length;

		for_each_sg(output->sg.sgl, b, output->sg.orig_nents, j)
			if (start < (u64)sg_phys(b) + b->length && sg_phys(b) < end)
				return true;
	}
	return false;
}

static int s31_bs_buffer_prepare(struct s31_bs_buffer *buf, u64 pointer,
				 u32 bytes, bool output, bool direct)
{
	void __user *user = u64_to_user_ptr(pointer);
	unsigned long address = (unsigned long)user;
	unsigned int pages;
	long pinned;
	int ret;

	buf->direction = output ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
	if (pointer > ULONG_MAX || !access_ok(user, bytes))
		return -EFAULT;
	if (!direct) {
		buf->bounce = output ? kzalloc(bytes, GFP_KERNEL) :
				       memdup_user(user, bytes);
		if (IS_ERR(buf->bounce)) {
			ret = PTR_ERR(buf->bounce);
			buf->bounce = NULL;
			return ret;
		}
		if (!buf->bounce)
			return -ENOMEM;
		ret = sg_alloc_table(&buf->sg, 1, GFP_KERNEL);
		if (!ret)
			sg_set_buf(buf->sg.sgl, buf->bounce, bytes);
		return ret;
	}
	pages = DIV_ROUND_UP(offset_in_page(address) + bytes, PAGE_SIZE);
	buf->pages = kcalloc(pages, sizeof(*buf->pages), GFP_KERNEL);
	if (!buf->pages)
		return -ENOMEM;
	pinned = pin_user_pages_fast(address, pages, output ? FOLL_WRITE : 0,
				     buf->pages);
	if (pinned < 0)
		return pinned;
	buf->nr_pages = pinned;
	if (pinned != pages)
		return -EFAULT;
	return sg_alloc_table_from_pages(&buf->sg, buf->pages, pages,
					 offset_in_page(address), bytes,
					 GFP_KERNEL);
}
static int s31_bs_buffer_map(struct s31_bs_buffer *buf,
			     struct dma_chan *channel)
{
	int ret;

	buf->dev = dmaengine_get_dma_device(channel);
	ret = dma_map_sgtable(buf->dev, &buf->sg, buf->direction, 0);
	if (!ret)
		buf->mapped = true;
	return ret;
}
static long s31_bs_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct s31_bs *b = file->private_data;
	struct s31_bitscrambler_job j;
	struct s31_bs_wait tw, rw;
	struct dma_slave_config conf = {
		.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.src_maxburst = 4,
		.dst_maxburst = 4
	};
	struct dma_chan *tx = NULL, *rx = NULL;
	struct s31_bs_buffer input = {}, output = {};
	u8 *program = NULL;
	bool direct, powered = false, route_owned = false;
	unsigned long deadline;
	u32 control, saved_route = 0, saved_memory = 0;
	unsigned int n, w, words, offset;
	int ret;

	if (cmd != S31_BITSCRAMBLER_RUN)
		return -ENOTTY;
	if (copy_from_user(&j, (void __user *)arg, sizeof(j)))
		return -EFAULT;
	if (j.version != S31_ACCEL_ABI_VERSION || j.reserved ||
	    j.program_bytes < 12 || j.program_bytes > 12 + 8 * 36 + 2048 ||
	    !j.input_bytes || j.input_bytes > S31_BS_MAX_BYTES ||
	    !j.output_bytes || j.output_bytes > S31_BS_MAX_BYTES ||
	    (j.input_bytes & 3) || (j.output_bytes & 3))
		return -EINVAL;
	if (j.program > ULONG_MAX)
		return -EFAULT;
	j.written = 0;
	direct = j.input_bytes >= PAGE_SIZE && IS_ALIGNED(j.input, 64) &&
		 IS_ALIGNED(j.output, 64) && IS_ALIGNED(j.input_bytes, 64) &&
		 IS_ALIGNED(j.output_bytes, 64) &&
		 (j.input + j.input_bytes <= j.output ||
		  j.output + j.output_bytes <= j.input);
	program = memdup_user(u64_to_user_ptr(j.program), j.program_bytes);
	if (IS_ERR(program))
		return PTR_ERR(program);
	words = get_unaligned_le16(program + 4);
	if (program[0] != 1 || program[1] || program[2] != 3 || !program[3] ||
	    program[3] > 8 || words > 512 || program[6] > 2 || program[7] > 1 ||
	    program[10] > 1 || program[11] ||
	    j.program_bytes != 12 + program[3] * 36 + words * 4) {
		ret = -EINVAL;
		goto free;
	}
	ret = s31_bs_buffer_prepare(&input, j.input, j.input_bytes, false,
				    direct);
	if (ret)
		goto free;
	ret = s31_bs_buffer_prepare(&output, j.output, j.output_bytes, true,
				    direct);
	if (ret)
		goto free;
	/* Disjoint virtual ranges can still map the same physical pages. Use
	 * the existing snapshot path before either alias is mapped for DMA.
	 */
	if (direct && s31_bs_buffers_overlap(&input, &output)) {
		s31_bs_buffer_release(&output);
		s31_bs_buffer_release(&input);
		ret = s31_bs_buffer_prepare(&input, j.input, j.input_bytes, false, false);
		if (!ret)
			ret = s31_bs_buffer_prepare(&output, j.output, j.output_bytes,
						    true, false);
		if (ret)
			goto free;
	}
	ret = mutex_lock_interruptible(&b->lock);
	if (ret)
		goto free;
	if (b->removed) {
		ret = -ENODEV;
		goto unlock;
	}
	/* The DT reserves the otherwise unused LCD route. Never intercept AES,
	 * SHA or the radio; both borrowed channels are released after each job.
	 */
	tx = of_dma_request_slave_channel(b->dev->of_node, "tx");
	if (IS_ERR(tx)) {
		ret = PTR_ERR(tx);
		tx = NULL;
		goto unlock;
	}
	rx = of_dma_request_slave_channel(b->dev->of_node, "rx");
	if (IS_ERR(rx)) {
		ret = PTR_ERR(rx);
		rx = NULL;
		goto cleanup;
	}
	ret = dmaengine_slave_config(tx, &conf);
	if (!ret)
		ret = dmaengine_slave_config(rx, &conf);
	if (ret)
		goto cleanup;
	ret = s31_bs_buffer_map(&input, tx);
	if (ret)
		goto cleanup;
	ret = s31_bs_buffer_map(&output, rx);
	if (ret)
		goto cleanup;
	ret = clk_prepare_enable(b->clk);
	if (ret)
		goto cleanup;
	powered = true;
	ret = reset_control_reset(b->reset);
	if (ret)
		goto cleanup;
	saved_route = readl(b->route);
	saved_memory = readl(b->memory);
	/* LUT is powered while programmed, without changing unrelated SRAM modes. */
	writel((saved_memory & ~BIT(2)) | BIT(3), b->memory);
	writel((saved_route & ~GENMASK(9, 0)) | 1 | (1 << 5), b->route);
	writel(1, b->regs + 0xf8); /* TX->RX loopback */
	route_owned = true;
	/* LUT SRAM accepts register accesses only while the core is enabled.
	 * Keep it halted throughout programming, before any DMA is submitted.
	 */
	writel(BIT(0) | BS_HALT, b->regs + BS_CTRL);
	for (n = 0; n < 8; n++)
		for (w = 0; w < 9; w++) {
			writel(n | (w << 3), b->regs);
			writel(n < program[3] ?
				       get_unaligned_le32(program + 12 +
							  n * 36 + w * 4) :
				       0,
			       b->regs + 4);
		}
	offset = 12 + program[3] * 36;
	/* Clear the complete LUT so one application's program cannot see another's. */
	for (w = 0; w < 512; w++) {
		writel(w | (2 << 11), b->regs + 0x10);
		writel(w < words ?
			       get_unaligned_le32(program + offset + w * 4) :
			       0,
		       b->regs + 0x14);
	}
	writel(program[6] << 11, b->regs + 0x10);
	writel(get_unaligned_le16(program + 8), b->regs + 0x20);
	control = BIT(0) | BS_HALT | BIT(6) | BIT(7) |
		  (program[7] ? 0 : BIT(5)) | (program[10] ? 0 : BIT(3));
	writel(control | BIT(8), b->regs + BS_CTRL);
	writel(control, b->regs + BS_CTRL);
	/* Pulse trace clear without retaining stale status bits. */
	writel(BIT(31), b->regs + BS_STATE);
	writel(0, b->regs + BS_STATE);
	ret = s31_bs_submit(rx, &output.sg, DMA_DEV_TO_MEM, &rw);
	if (!ret)
		ret = s31_bs_submit(tx, &input.sg, DMA_MEM_TO_DEV, &tw);
	if (ret)
		goto cleanup;
	output.dirty = true;
	dma_async_issue_pending(rx);
	dma_async_issue_pending(tx);
	writel(control & ~BS_HALT, b->regs + BS_CTRL);
	deadline =
		jiffies +
		msecs_to_jiffies(100 + (j.input_bytes + j.output_bytes) / 1000);
	ret = s31_bs_wait(&rw, deadline);
	if (!ret)
		ret = s31_bs_wait(&tw, deadline);
	if (!ret && rw.result.residue <= j.output_bytes)
		j.written = j.output_bytes - rw.result.residue;
cleanup:
	if (route_owned)
		writel(control | BS_HALT, b->regs + BS_CTRL);
	if (rx)
		dmaengine_terminate_sync(rx);
	if (tx)
		dmaengine_terminate_sync(tx);
	if (route_owned) {
		writel(0, b->regs + BS_CTRL);
		writel(0, b->regs + 0xf8);
		writel(saved_route, b->route);
		writel(saved_memory, b->memory);
	}
	if (powered)
		clk_disable_unprepare(b->clk);
	/* Join IRQ callbacks before unmapping, dirtying or unpinning any pages. */
	if (output.mapped) {
		dma_unmap_sgtable(output.dev, &output.sg, output.direction, 0);
		output.mapped = false;
	}
	if (input.mapped) {
		dma_unmap_sgtable(input.dev, &input.sg, input.direction, 0);
		input.mapped = false;
	}
	if (rx)
		dma_release_channel(rx);
	if (tx)
		dma_release_channel(tx);
unlock:
	mutex_unlock(&b->lock);
	if (!ret && output.bounce &&
	    copy_to_user(u64_to_user_ptr(j.output), output.bounce, j.written))
		ret = -EFAULT;
	if (copy_to_user((void __user *)arg, &j, sizeof(j)))
		ret = -EFAULT;
free:
	s31_bs_buffer_release(&output);
	s31_bs_buffer_release(&input);
	kfree(program);
	return ret;
}
static const struct file_operations s31_bs_fops = {
	.owner = THIS_MODULE,
	.open = s31_bs_open,
	.release = s31_bs_close,
	.unlocked_ioctl = s31_bs_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};
static int s31_bs_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s31_bs *b = kzalloc(sizeof(*b), GFP_KERNEL);
	int ret;

	if (!b)
		return -ENOMEM;
	b->dev = dev;
	mutex_init(&b->lock);
	kref_init(&b->refs);
	b->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(b->regs)) {
		ret = PTR_ERR(b->regs);
		goto free;
	}
	b->route = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(b->route)) {
		ret = PTR_ERR(b->route);
		goto free;
	}
	b->memory = devm_platform_ioremap_resource(pdev, 2);
	if (IS_ERR(b->memory)) {
		ret = PTR_ERR(b->memory);
		goto free;
	}
	b->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(b->clk)) {
		ret = PTR_ERR(b->clk);
		goto free;
	}
	b->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(b->reset)) {
		ret = PTR_ERR(b->reset);
		goto free;
	}
	platform_set_drvdata(pdev, b);
	b->misc = (struct miscdevice){ .minor = MISC_DYNAMIC_MINOR,
				       .name = "s31-bitscrambler",
				       .fops = &s31_bs_fops,
				       .parent = dev,
				       .mode = 0600 };
	ret = misc_register(&b->misc);
	if (!ret)
		return 0;
free:
	kref_put(&b->refs, s31_bs_free);
	return dev_err_probe(dev, ret, "BitScrambler registration failed\n");
}
static void s31_bs_remove(struct platform_device *pdev)
{
	struct s31_bs *b = platform_get_drvdata(pdev);

	misc_deregister(&b->misc);
	mutex_lock(&b->lock);
	b->removed = true;
	mutex_unlock(&b->lock);
	kref_put(&b->refs, s31_bs_free);
}
static const struct of_device_id s31_bs_match[] = {
	{ .compatible = "espressif,esp32s31-bitscrambler" },
	{}
};
MODULE_DEVICE_TABLE(of, s31_bs_match);
static struct platform_driver s31_bs_driver = {
	.probe = s31_bs_probe,
	.remove = s31_bs_remove,
	.driver = { .name = "esp32s31-bitscrambler",
		    .of_match_table = s31_bs_match },
};
module_platform_driver(s31_bs_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ESP32-S31 BitScrambler bounded memory transformation");
