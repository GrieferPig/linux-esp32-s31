// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/esp32s31-gdma.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/sizes.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>

#include <uapi/linux/esp32s31-bitscrambler.h>

/*
 * S31 TRM ch.5 lists the BitScrambler address. The BitScrambler chapter
 * is pending. Register semantics follow IDF bitscrambler_ll.h and
 * instruction and LUT formatting follows bitscrambler_load_program().
 */
#define TX_INST_SELECT		0x00
#define TX_INST_WRITE		0x04
#define TX_LUT_SELECT		0x10
#define TX_LUT_WRITE		0x14
#define TX_TRAILING_BITS	0x20
#define TX_CONTROL		0x28
#define TX_STATE		0x30
#define SYSTEM			0xf8

#define TX_ENABLE		BIT(0)
#define TX_HALT			BIT(2)
#define TX_EOF_ON_WRITE		BIT(3)
#define TX_NO_PREFETCH		BIT(5)
#define TX_HALT_IGNORE_WRITES	BIT(6)
#define TX_READ_DUMMY		BIT(7)
#define TX_FIFO_RESET		BIT(8)
#define TX_IDLE			BIT(0)
#define TX_EOF_TRACE_CLEAR	BIT(31)
#define SYSTEM_LOOPBACK		BIT(0)
#define SYSTEM_REG_CLOCK	BIT(31)

#define SELECT_RX		GENMASK(4, 0)
#define SELECT_TX		GENMASK(9, 5)
#define SELECT_GPSPI3		3
#define LUT_WIDTH		GENMASK(12, 11)
#define MEM_LP_ENABLE		BIT(2)
#define MEM_LP_FORCE		BIT(3)

#define MAX_INSTRUCTIONS	8
#define INST_WORDS		9
/* The IDF assembler describes a 2048 x 8-bit LUT, written as 512 words. */
#define MAX_LUT_WORDS		512
#define HEADER_SIZE		12
#define MAX_PROGRAM_SIZE	(HEADER_SIZE + (MAX_INSTRUCTIONS * INST_WORDS + MAX_LUT_WORDS) * 4)

struct bs_header {
	u8 version;
	u8 hw_revision;
	u8 header_words;
	u8 instructions;
	__le16 lut_words;
	u8 lut_width;
	u8 prefetch;
	__le16 trailing_bits;
	u8 eof_on_read;
	u8 reserved;
};

struct s31_bitscrambler {
	void __iomem *base;
	void __iomem *select;
	void __iomem *memory;
	u32 saved_memory;
	struct dma_chan *dma;
	struct miscdevice misc;
	struct mutex lock;
	struct kref refs;
	bool removed;
};

struct s31_bs_transfer {
	struct completion done;
	struct dmaengine_result result;
};

static int s31_bs_load(struct s31_bitscrambler *bs, const u8 *program, size_t size,
		       u32 *control)
{
	const struct bs_header *header = (const void *)program;
	const u8 *words = program + HEADER_SIZE;
	unsigned int count, lut, i, j;
	u32 value;

	if (size < HEADER_SIZE || header->version != 1 || header->hw_revision ||
	    header->header_words != HEADER_SIZE / 4 || header->reserved ||
	    !header->instructions || header->instructions > MAX_INSTRUCTIONS ||
	    header->lut_width > 2 || header->prefetch > 1 || header->eof_on_read > 1)
		return -EINVAL;

	count = header->instructions;
	lut = le16_to_cpu(header->lut_words);
	if (lut > MAX_LUT_WORDS || size != HEADER_SIZE + (count * INST_WORDS + lut) * 4)
		return -EINVAL;

	/* Scrub unused instruction slots before executing an unrelated program. */
	for (i = 0; i < MAX_INSTRUCTIONS; i++) {
		for (j = 0; j < INST_WORDS; j++) {
			value = i < count ? get_unaligned_le32(words) : 0;
			writel(i | (j << 3), bs->base + TX_INST_SELECT);
			writel(value, bs->base + TX_INST_WRITE);
			if (i < count)
				words += 4;
		}
	}

	/* The APB writes 32-bit LUT words independent of the program's lookup width. */
	for (i = 0; i < MAX_LUT_WORDS; i++) {
		writel(FIELD_PREP(LUT_WIDTH, 2) | i, bs->base + TX_LUT_SELECT);
		writel(i < lut ? get_unaligned_le32(words + i * 4) : 0,
		       bs->base + TX_LUT_WRITE);
	}
	writel(FIELD_PREP(LUT_WIDTH, header->lut_width), bs->base + TX_LUT_SELECT);
	writel(le16_to_cpu(header->trailing_bits), bs->base + TX_TRAILING_BITS);

	*control = TX_ENABLE | TX_HALT_IGNORE_WRITES | TX_READ_DUMMY;
	if (!header->prefetch)
		*control |= TX_NO_PREFETCH;
	if (!header->eof_on_read)
		*control |= TX_EOF_ON_WRITE;
	return 0;
}

static void s31_bs_dma_done(void *arg, const struct dmaengine_result *result)
{
	struct s31_bs_transfer *transfer = arg;

	transfer->result = *result;
	complete(&transfer->done);
}

static int s31_bs_execute(struct s31_bitscrambler *bs, const u8 *program,
			  struct esp32s31_bitscrambler_run *request,
			  dma_addr_t dst, dma_addr_t src)
{
	struct dma_async_tx_descriptor *desc;
	struct s31_bs_transfer transfer;
	u32 control, status, saved_select;
	dma_cookie_t cookie;
	long wait;
	int ret, stop;

	ret = s31_bs_load(bs, program, request->program_len, &control);
	if (ret)
		return ret;

	/* Match IDF bitscrambler_loopback_run(): reset, arm both DMA halves, run. */
	writel(control | TX_HALT, bs->base + TX_CONTROL);
	ret = readl_poll_timeout(bs->base + TX_STATE, status, status & TX_IDLE, 1, 1000);
	if (ret)
		return ret;
	writel(control | TX_HALT | TX_FIFO_RESET, bs->base + TX_CONTROL);
	writel(control | TX_HALT, bs->base + TX_CONTROL);
	writel(TX_EOF_TRACE_CLEAR, bs->base + TX_STATE);

	desc = esp32s31_gdma_prep_bitscrambler(bs->dma, dst, request->dst_len,
					     src, request->src_len,
					     DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc)
		return -ENOMEM;

	init_completion(&transfer.done);
	desc->callback_result = s31_bs_dma_done;
	desc->callback_param = &transfer;
	cookie = dmaengine_submit(desc);
	ret = dma_submit_error(cookie);
	if (ret)
		return ret;

	saved_select = readl(bs->select);
	writel((saved_select & ~(SELECT_RX | SELECT_TX)) |
	       FIELD_PREP(SELECT_RX, SELECT_GPSPI3) |
	       FIELD_PREP(SELECT_TX, SELECT_GPSPI3), bs->select);
	writel(SYSTEM_REG_CLOCK | SYSTEM_LOOPBACK, bs->base + SYSTEM);
	dma_async_issue_pending(bs->dma);
	writel(control, bs->base + TX_CONTROL);

	/* Bound execution of a microcode image that never emits EOF. */
	wait = wait_for_completion_interruptible_timeout(&transfer.done,
							 msecs_to_jiffies(25));
	if (wait < 0) {
		ret = wait;
	} else if (!wait) {
		ret = -ETIMEDOUT;
	} else if (transfer.result.result != DMA_TRANS_NOERROR ||
		   transfer.result.residue > request->dst_len) {
		ret = -EIO;
	} else {
		request->dst_len -= transfer.result.residue;
	}
	writel(control | TX_HALT, bs->base + TX_CONTROL);
	stop = dmaengine_terminate_sync(bs->dma);
	writel(saved_select, bs->select);
	writel(SYSTEM_REG_CLOCK, bs->base + SYSTEM);
	return ret ? ret : stop;
}

static int s31_bs_run(struct s31_bitscrambler *bs, struct esp32s31_bitscrambler_run *r)
{
	struct device *dev;
	dma_addr_t src_dma, dst_dma;
	void *src, *dst;
	u8 *program;
	size_t dst_len = r->dst_len;
	int ret;

	if (!r->src || !r->dst || !r->program || r->reserved ||
	    !r->src_len || !dst_len || r->src_len > SZ_4K || dst_len > SZ_4K ||
	    r->program_len < HEADER_SIZE || r->program_len > MAX_PROGRAM_SIZE)
		return -EINVAL;
	if (r->src > ULONG_MAX || r->dst > ULONG_MAX || r->program > ULONG_MAX)
		return -EFAULT;

	program = memdup_user(u64_to_user_ptr(r->program), r->program_len);
	if (IS_ERR(program))
		return PTR_ERR(program);
	ret = mutex_lock_interruptible(&bs->lock);
	if (ret)
		goto out_program;
	if (bs->removed) {
		ret = -ENODEV;
		goto out_unlock;
	}

	dev = dmaengine_get_dma_device(bs->dma);
	src = dma_alloc_noncoherent(dev, r->src_len, &src_dma, DMA_TO_DEVICE,
				    GFP_KERNEL);
	if (!src) {
		ret = -ENOMEM;
		goto out_unlock;
	}
	if (copy_from_user(src, u64_to_user_ptr(r->src), r->src_len)) {
		ret = -EFAULT;
		goto out_src;
	}
	dst = dma_alloc_noncoherent(dev, dst_len, &dst_dma, DMA_FROM_DEVICE, GFP_KERNEL);
	if (!dst) {
		ret = -ENOMEM;
		goto out_src;
	}
	memset(dst, 0, dst_len);

	dma_sync_single_for_device(dev, src_dma, r->src_len, DMA_TO_DEVICE);
	dma_sync_single_for_device(dev, dst_dma, dst_len, DMA_FROM_DEVICE);
	ret = s31_bs_execute(bs, program, r, dst_dma, src_dma);
	dma_sync_single_for_cpu(dev, dst_dma, dst_len, DMA_FROM_DEVICE);
	dma_sync_single_for_cpu(dev, src_dma, r->src_len, DMA_TO_DEVICE);
	if (!ret && copy_to_user(u64_to_user_ptr(r->dst), dst, r->dst_len))
		ret = -EFAULT;
	dma_free_noncoherent(dev, dst_len, dst, dst_dma, DMA_FROM_DEVICE);
out_src:
	dma_free_noncoherent(dev, r->src_len, src, src_dma, DMA_TO_DEVICE);
out_unlock:
	mutex_unlock(&bs->lock);
out_program:
	kfree(program);
	return ret;
}

static long s31_bs_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct esp32s31_bitscrambler_run request;
	void __user *user = (void __user *)arg;
	int ret;

	if (cmd != ESP32S31_BITSCRAMBLER_IOC_RUN)
		return -ENOTTY;
	if (copy_from_user(&request, user, sizeof(request)))
		return -EFAULT;
	ret = s31_bs_run(file->private_data, &request);
	if (ret)
		return ret;
	return copy_to_user(user, &request, sizeof(request)) ? -EFAULT : 0;
}

static void s31_bs_free(struct kref *refs)
{
	kfree(container_of(refs, struct s31_bitscrambler, refs));
}

static void s31_bs_put(void *data)
{
	struct s31_bitscrambler *bs = data;

	kref_put(&bs->refs, s31_bs_free);
}

static int s31_bs_open(struct inode *inode, struct file *file)
{
	struct s31_bitscrambler *bs = container_of(file->private_data,
						  struct s31_bitscrambler, misc);

	kref_get(&bs->refs);
	file->private_data = bs;
	return nonseekable_open(inode, file);
}

static int s31_bs_release(struct inode *inode, struct file *file)
{
	s31_bs_put(file->private_data);
	return 0;
}

static const struct file_operations s31_bs_fops = {
	.owner = THIS_MODULE,
	.open = s31_bs_open,
	.release = s31_bs_release,
	.unlocked_ioctl = s31_bs_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

static void s31_bs_restore_memory(void *data)
{
	struct s31_bitscrambler *bs = data;

	writel(bs->saved_memory, bs->memory);
}

static int s31_bs_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dma_slave_config config = {
		.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.src_maxburst = 8,
		.dst_maxburst = 8,
	};
	struct s31_bitscrambler *bs;
	struct reset_control *rst;
	struct clk *clk;
	int ret;

	bs = kzalloc(sizeof(*bs), GFP_KERNEL);
	if (!bs)
		return -ENOMEM;
	kref_init(&bs->refs);
	ret = devm_add_action_or_reset(dev, s31_bs_put, bs);
	if (ret)
		return ret;
	mutex_init(&bs->lock);

	bs->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(bs->base))
		return PTR_ERR(bs->base);
	bs->select = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(bs->select))
		return PTR_ERR(bs->select);
	bs->memory = devm_platform_ioremap_resource(pdev, 2);
	if (IS_ERR(bs->memory))
		return PTR_ERR(bs->memory);
	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "clock unavailable\n");
	rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "reset unavailable\n");
	ret = reset_control_reset(rst);
	if (ret)
		return ret;

	/* IDF bitscrambler_ll_mem_power_by_pmu() enables the instruction/LUT SRAM. */
	bs->saved_memory = readl(bs->memory);
	writel(bs->saved_memory & ~(MEM_LP_ENABLE | MEM_LP_FORCE), bs->memory);
	ret = devm_add_action_or_reset(dev, s31_bs_restore_memory, bs);
	if (ret)
		return ret;
	writel(SYSTEM_REG_CLOCK, bs->base + SYSTEM);
	writel(TX_HALT, bs->base + TX_CONTROL);

	bs->dma = devm_dma_request_chan(dev, "loopback");
	if (IS_ERR(bs->dma))
		return dev_err_probe(dev, PTR_ERR(bs->dma), "AXI DMA channel unavailable\n");
	ret = dmaengine_slave_config(bs->dma, &config);
	if (ret)
		return ret;

	bs->misc.minor = MISC_DYNAMIC_MINOR;
	bs->misc.name = "esp32s31-bitscrambler";
	bs->misc.fops = &s31_bs_fops;
	bs->misc.parent = dev;
	bs->misc.mode = 0600;
	platform_set_drvdata(pdev, bs);
	return misc_register(&bs->misc);
}

static void s31_bs_remove(struct platform_device *pdev)
{
	struct s31_bitscrambler *bs = platform_get_drvdata(pdev);

	misc_deregister(&bs->misc);
	mutex_lock(&bs->lock);
	bs->removed = true;
	writel(TX_HALT, bs->base + TX_CONTROL);
	writel(0, bs->base + SYSTEM);
	mutex_unlock(&bs->lock);
}

static const struct of_device_id s31_bs_of_match[] = {
	{ .compatible = "espressif,esp32s31-bitscrambler" },
	{ }
};
MODULE_DEVICE_TABLE(of, s31_bs_of_match);

static struct platform_driver s31_bs_driver = {
	.probe = s31_bs_probe,
	.remove = s31_bs_remove,
	.driver = {
		.name = "esp32s31-bitscrambler",
		.of_match_table = s31_bs_of_match,
	},
};
module_platform_driver(s31_bs_driver);

MODULE_DESCRIPTION("Espressif ESP32-S31 BitScrambler");
MODULE_LICENSE("GPL");
