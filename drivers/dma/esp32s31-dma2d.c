// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 DMA2D: native DMAengine copies and shared JPEG/PPA transport.
 * One serialized engine queue bounds SRAM and avoids competing clients
 * assigning the same peripheral handshake to multiple channels.
 */
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_dma.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/soc/espressif/esp32s31-dma2d.h>
#include <linux/soc/espressif/esp32s31-regfield.h>
#include "virt-dma.h"

struct s31_dma2d {
	struct device *dev;
	dma2d_dev_t __iomem *hw;
	void __iomem *descs, *memory;
	phys_addr_t desc_phys;
	struct clk *clk;
	struct reset_control *reset;
	struct mutex lock;
	struct completion done;
	atomic_t status;
	struct dma_device engine;
	struct virt_dma_chan chan;
	struct work_struct work;
	struct workqueue_struct *wq;
	bool removing;
	struct s31_dma2d_job *active_job;
	struct virt_dma_desc *running;
	dma_cookie_t error_cookie;
};
struct s31_dma2d_copy {
	struct virt_dma_desc vd;
	struct s31_dma2d_job job;
};
static dma_addr_t s31_dma2d_bus_address(dma_addr_t address)
{
	/* DMA2D uses the S31 PSRAM address window, as in the native SDK. */
	return address;
}
void s31_dma2d_desc_linear(struct s31_dma2d_desc *d, dma_addr_t buffer,
			   size_t length, bool tx)
{
	memset(d, 0, sizeof(*d));
	d->words[0] = cpu_to_le32((length & 0x3fff) |
				  (tx ? (length & 0x3fff) << 14 : 0) | BIT(31) |
				  (tx ? BIT(30) : 0));
	d->words[1] =
		cpu_to_le32((length >> 14) | (tx ? (length >> 14) << 14 : 0));
	d->words[3] = cpu_to_le32(s31_dma2d_bus_address(buffer));
}
EXPORT_SYMBOL_GPL(s31_dma2d_desc_linear);
void s31_dma2d_desc_image(struct s31_dma2d_desc *d, dma_addr_t buffer,
			  u16 width, u16 height, u16 block_w, u16 block_h,
			  u8 pbyte, bool multiple)
{
	memset(d, 0, sizeof(*d));
	d->words[0] = cpu_to_le32(block_h | (block_w << 14) | BIT(29) |
				  BIT(30) | BIT(31));
	d->words[1] = cpu_to_le32(height | (width << 14) | (pbyte << 28));
	d->words[2] = cpu_to_le32(multiple ? BIT(28) : 0);
	d->words[3] = cpu_to_le32(s31_dma2d_bus_address(buffer));
}
EXPORT_SYMBOL_GPL(s31_dma2d_desc_image);
struct device *s31_dma2d_device(struct s31_dma2d *d)
{
	return d->dev;
}
EXPORT_SYMBOL_GPL(s31_dma2d_device);
struct s31_dma2d *s31_dma2d_get(struct device *consumer)
{
	struct device_node *node =
		of_parse_phandle(consumer->of_node, "espressif,dma2d", 0);
	struct platform_device *supplier;
	struct s31_dma2d *d;

	if (!node)
		return ERR_PTR(-EINVAL);
	supplier = of_find_device_by_node(node);
	of_node_put(node);
	if (!supplier)
		return ERR_PTR(-EPROBE_DEFER);
	d = device_is_bound(&supplier->dev) ? platform_get_drvdata(supplier) :
					      NULL;
	if (!d ||
	    !device_link_add(consumer, &supplier->dev,
			     DL_FLAG_AUTOREMOVE_CONSUMER | DL_FLAG_PM_RUNTIME))
		d = ERR_PTR(d ? -ENOMEM : -EPROBE_DEFER);
	put_device(&supplier->dev);
	return d;
}
EXPORT_SYMBOL_GPL(s31_dma2d_get);
static irqreturn_t s31_dma2d_irq(int irq, void *arg)
{
	struct s31_dma2d *d = arg;
	dma2d_dev_t __iomem *h = d->hw;
	u32 rx = readl(&h->in_channel[0].in_int_st.val);
	u32 tx = readl(&h->out_channel[0].out_int_st.val);
	u32 fg = readl(&h->out_channel[1].out_int_st.val);

	if (!(rx | tx | fg))
		return IRQ_NONE;
	writel(rx, &h->in_channel[0].in_int_clr.val);
	writel(tx, &h->out_channel[0].out_int_clr.val);
	writel(fg, &h->out_channel[1].out_int_clr.val);
	atomic_or((rx << 16) | tx | fg, &d->status);
	if ((rx & (BIT(1) | GENMASK(13, 2))) || ((tx | fg) & GENMASK(12, 2)))
		complete(&d->done);
	return IRQ_HANDLED;
}
int s31_dma2d_transfer(struct s31_dma2d *d, struct s31_dma2d_job *job)
{
	dma2d_dev_t __iomem *h = d->hw;
	dma2d_out_conf0_chn_reg_t tx = {};
	dma2d_in_conf0_chn_reg_t rx = {};
	u32 rx0, rx1, status;
	int ret = mutex_lock_interruptible(&d->lock);

	if (ret)
		return ret;
	if (READ_ONCE(d->removing)) {
		ret = -ESHUTDOWN;
		goto unlock;
	}
	if (READ_ONCE(job->cancelled)) {
		ret = -ECANCELED;
		goto unlock;
	}
	ret = pm_runtime_resume_and_get(d->dev);
	if (ret < 0)
		goto unlock;
	/* Restore access limits and AXI FIFOs after the preceding transaction reset. */
	writel(0x2f000000, &h->intr_mem_start_addr.val);
	writel(0x2f080000, &h->intr_mem_end_addr.val);
	writel(0x50000000, &h->extr_mem_start_addr.val);
	writel(0x51000000, &h->extr_mem_end_addr.val);
	s31_reg_set(h, rst_conf, clk_en, 1);
	s31_reg_set(h, rst_conf, axim_rd_rst, 1);
	s31_reg_set(h, rst_conf, axim_rd_rst, 0);
	s31_reg_set(h, rst_conf, axim_wr_rst, 1);
	s31_reg_set(h, rst_conf, axim_wr_rst, 0);
	reinit_completion(&d->done);
	atomic_set(&d->status, 0);
	memcpy_toio(d->descs, &job->tx, sizeof(job->tx));
	memcpy_toio(d->descs + 32, &job->rx, sizeof(job->rx));
	if (job->blend)
		memcpy_toio(d->descs + 64, &job->foreground, sizeof(job->foreground));
	/* Peripheral descriptor ports may revisit the source descriptor. The
	 * native SDK leaves TX writeback disabled; RX still reports completion.
	 */
	tx.out_auto_wrback_chn = 0;
	tx.out_reorder_en_chn = job->tx_reorder;
	tx.out_eof_mode_chn = 1;
	tx.outdscr_burst_en_chn = 1;
	tx.out_mem_burst_length_chn = 4;
	tx.out_page_bound_en_chn = 1;
	tx.out_macro_block_size_chn = job->tx_macroblock;
	tx.out_dscr_port_en_chn = job->descriptor_port;
	rx.in_mem_trans_en_chn = job->peripheral == 7;
	rx.indscr_burst_en_chn = 1;
	rx.in_mem_burst_length_chn = 4;
	rx.in_page_bound_en_chn = 1;
	rx.in_macro_block_size_chn = job->rx_macroblock;
	rx.in_reorder_en_chn = job->rx_reorder;
	rx.in_dscr_port_en_chn = job->descriptor_port;
	writel(tx.val | BIT(24), &h->out_channel[0].out_conf0.val);
	writel(rx.val | BIT(24), &h->in_channel[0].in_conf0.val);
	writel(tx.val, &h->out_channel[0].out_conf0.val);
	writel(rx.val, &h->in_channel[0].in_conf0.val);
	writel(job->peripheral, &h->out_channel[0].out_peri_sel.val);
	writel(job->peripheral, &h->in_channel[0].in_peri_sel.val);
	/* Disable CSC, matching the LL NONE settings. */
	writel((7 << 3) | 2, &h->out_channel[0].out_color_convert.val);
	writel((7 << 3) | 1, &h->in_channel[0].in_color_convert.val);
	writel(0, &h->out_channel[0].out_scramble.val);
	writel(0, &h->in_channel[0].in_scramble.val);
	if (job->blend) {
		/* PPA blend consumes BG on TX0 and FG on TX1, using different
		 * peripheral handshakes. All three descriptors share this lease.
		 */
		writel(tx.val | BIT(24), &h->out_channel[1].out_conf0.val);
		writel(tx.val, &h->out_channel[1].out_conf0.val);
		writel(3, &h->out_channel[1].out_peri_sel.val);
		writel((7 << 3) | 2, &h->out_channel[1].out_color_convert.val);
		writel(0, &h->out_channel[1].out_scramble.val);
		writel(d->desc_phys + 64, &h->out_channel[1].out_link_addr.val);
	}
	if (job->descriptor_port)
		writel(job->block_w | (job->block_h << 14),
		       &h->out_channel[0].out_dscr_port_blk.val);
	if (job->configure)
		job->configure(job->arg, h);
	writel(0x3fff, &h->in_channel[0].in_int_clr.val);
	writel(0x1fff, &h->out_channel[0].out_int_clr.val);
	writel(0x1fff, &h->out_channel[1].out_int_clr.val);
	writel(BIT(1) | GENMASK(13, 2), &h->in_channel[0].in_int_ena.val);
	/* TX_TOTAL_EOF is normal completion, not an error. */
	writel(job->rx_only ? 0 : BIT(2) | GENMASK(12, 4),
	       &h->out_channel[0].out_int_ena.val);
	writel(job->blend ? BIT(2) | GENMASK(12, 4) : 0,
	       &h->out_channel[1].out_int_ena.val);
	writel(d->desc_phys, &h->out_channel[0].out_link_addr.val);
	writel(d->desc_phys + 32, &h->in_channel[0].in_link_addr.val);
	s31_reg_set(h, in_channel[0].in_link_conf, inlink_auto_ret_chn, 0);
	dma_wmb();
	if (!job->rx_only)
		s31_reg_set(h, out_channel[0].out_link_conf, outlink_start_chn, 1);
	if (job->blend)
		s31_reg_set(h, out_channel[1].out_link_conf, outlink_start_chn, 1);
	s31_reg_set(h, in_channel[0].in_link_conf, inlink_start_chn, 1);
	WRITE_ONCE(d->active_job, job);
	if (job->start)
		job->start(job->arg);
	if (READ_ONCE(job->cancelled))
		complete(&d->done);
	ret = wait_for_completion_interruptible_timeout(&d->done,
							msecs_to_jiffies(1000));
	status = atomic_read(&d->status);
	WRITE_ONCE(d->active_job, NULL);
	if (READ_ONCE(job->cancelled))
		ret = -ECANCELED;
	else if (ret <= 0)
		ret = ret ? ret : -ETIMEDOUT;
	else
		ret = ((status >> 16) & GENMASK(13, 2)) ||
				      (status & (BIT(2) | GENMASK(12, 4))) ?
			      -EIO :
			      0;
	writel(0, &h->in_channel[0].in_int_ena.val);
	writel(0, &h->out_channel[0].out_int_ena.val);
	writel(0, &h->out_channel[1].out_int_ena.val);
	s31_reg_set(h, in_channel[0].in_link_conf, inlink_stop_chn, 1);
	s31_reg_set(h, out_channel[0].out_link_conf, outlink_stop_chn, 1);
	if (job->blend)
		s31_reg_set(h, out_channel[1].out_link_conf, outlink_stop_chn, 1);
	if (job->stop)
		job->stop(job->arg);
	/* Reset the dedicated engine before buffers may be unmapped. */
	reset_control_reset(d->reset);
	dma_rmb();
	rx0 = readl(d->descs + 32);
	rx1 = readl(d->descs + 36);
	job->received = ((rx0 >> 14) & 0x3fff) | (((rx1 >> 14) & 0x3fff) << 14);
	pm_runtime_mark_last_busy(d->dev);
	pm_runtime_put_autosuspend(d->dev);
unlock:
	mutex_unlock(&d->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(s31_dma2d_transfer);
static void s31_dma2d_free_desc(struct virt_dma_desc *vd)
{
	kfree(container_of(vd, struct s31_dma2d_copy, vd));
}
static void s31_dma2d_work(struct work_struct *work)
{
	struct s31_dma2d *d = container_of(work, struct s31_dma2d, work);
	struct virt_dma_desc *vd;
	struct s31_dma2d_copy *copy;
	unsigned long flags;
	int ret;

	for (;;) {
		spin_lock_irqsave(&d->chan.lock, flags);
		vd = vchan_next_desc(&d->chan);
		if (!vd || d->removing) {
			spin_unlock_irqrestore(&d->chan.lock, flags);
			break;
		}
		list_del(&vd->node);
		d->running = vd;
		spin_unlock_irqrestore(&d->chan.lock, flags);
		copy = container_of(vd, struct s31_dma2d_copy, vd);
		ret = s31_dma2d_transfer(d, &copy->job);
		spin_lock_irqsave(&d->chan.lock, flags);
		vd->tx_result.result = ret ? DMA_TRANS_ABORTED :
					     DMA_TRANS_NOERROR;
		vd->tx_result.residue = 0;
		if (ret)
			d->error_cookie = vd->tx.cookie;
		d->running = NULL;
		if (READ_ONCE(copy->job.cancelled))
			vchan_terminate_vdesc(vd);
		else
			vchan_cookie_complete(vd);
		spin_unlock_irqrestore(&d->chan.lock, flags);
	}
}
static struct dma_async_tx_descriptor *
s31_dma2d_prep_copy(struct dma_chan *ch, dma_addr_t dst, dma_addr_t src,
		    size_t len, unsigned long flags)
{
	struct s31_dma2d *d = container_of(ch, struct s31_dma2d, chan.chan);
	struct s31_dma2d_copy *c;

	if (!len || len >= BIT(28))
		return NULL;
	c = kzalloc(sizeof(*c), GFP_NOWAIT);
	if (!c)
		return NULL;
	c->job.peripheral = 7;
	c->job.tx_macroblock = 3;
	c->job.rx_macroblock = 3;
	s31_dma2d_desc_linear(&c->job.tx, src, len, true);
	s31_dma2d_desc_linear(&c->job.rx, dst, len, false);
	return vchan_tx_prep(&d->chan, &c->vd, flags);
}
static struct dma_async_tx_descriptor *
s31_dma2d_prep_interleaved(struct dma_chan *ch,
			   struct dma_interleaved_template *xt,
			   unsigned long flags)
{
	struct s31_dma2d *d = container_of(ch, struct s31_dma2d, chan.chan);
	struct s31_dma2d_copy *c;
	size_t width, src_stride, dst_stride;

	if (xt->dir != DMA_MEM_TO_MEM || xt->frame_size != 1 || !xt->numf ||
	    xt->numf > 0x3fff || !xt->src_inc || !xt->dst_inc)
		return NULL;
	width = xt->sgl[0].size;
	src_stride = width + dmaengine_get_src_icg(xt, &xt->sgl[0]);
	dst_stride = width + dmaengine_get_dst_icg(xt, &xt->sgl[0]);
	if (!width || width > 0x3fff || src_stride > 0x3fff ||
	    dst_stride > 0x3fff)
		return NULL;
	c = kzalloc(sizeof(*c), GFP_NOWAIT);
	if (!c)
		return NULL;
	c->job.peripheral = 7;
	c->job.tx_macroblock = 3;
	c->job.rx_macroblock = 3;
	s31_dma2d_desc_image(&c->job.tx, xt->src_start, src_stride, xt->numf,
			     width, xt->numf, 1, false);
	s31_dma2d_desc_image(&c->job.rx, xt->dst_start, dst_stride, xt->numf,
			     width, xt->numf, 1, false);
	return vchan_tx_prep(&d->chan, &c->vd, flags);
}
static void s31_dma2d_issue(struct dma_chan *ch)
{
	struct s31_dma2d *d = container_of(ch, struct s31_dma2d, chan.chan);
	unsigned long flags;

	spin_lock_irqsave(&d->chan.lock, flags);
	if (vchan_issue_pending(&d->chan))
		queue_work(d->wq, &d->work);
	spin_unlock_irqrestore(&d->chan.lock, flags);
}
static enum dma_status s31_dma2d_status(struct dma_chan *ch,
					dma_cookie_t cookie,
					struct dma_tx_state *state)
{
	struct s31_dma2d *d = container_of(ch, struct s31_dma2d, chan.chan);

	if (cookie == READ_ONCE(d->error_cookie))
		return DMA_ERROR;
	return dma_cookie_status(ch, cookie, state);
}
static int s31_dma2d_terminate(struct dma_chan *ch)
{
	struct s31_dma2d *d = container_of(ch, struct s31_dma2d, chan.chan);
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&d->chan.lock, flags);
	if (d->running) {
		struct s31_dma2d_copy *copy =
			container_of(d->running, struct s31_dma2d_copy, vd);
		WRITE_ONCE(copy->job.cancelled, true);
		if (READ_ONCE(d->active_job) == &copy->job)
			complete(&d->done);
	}
	vchan_get_all_descriptors(&d->chan, &head);
	spin_unlock_irqrestore(&d->chan.lock, flags);
	vchan_dma_desc_free_list(&d->chan, &head);
	return 0;
}
static void s31_dma2d_sync(struct dma_chan *ch)
{
	struct s31_dma2d *d = container_of(ch, struct s31_dma2d, chan.chan);

	flush_work(&d->work);
	vchan_synchronize(&d->chan);
}
static int s31_dma2d_alloc(struct dma_chan *ch)
{
	return 0;
}
static void s31_dma2d_release(struct dma_chan *ch)
{
	struct s31_dma2d *d = container_of(ch, struct s31_dma2d, chan.chan);

	s31_dma2d_terminate(ch);
	s31_dma2d_sync(ch);
	vchan_free_chan_resources(&d->chan);
}
static struct dma_chan *s31_dma2d_xlate(struct of_phandle_args *spec,
					struct of_dma *of)
{
	struct s31_dma2d *d = of->of_dma_data;

	if (spec->args_count != 1 || spec->args[0])
		return NULL;
	return dma_get_slave_channel(&d->chan.chan);
}
static int s31_dma2d_resume(struct device *dev)
{
	struct s31_dma2d *d = dev_get_drvdata(dev);
	dma2d_dev_t __iomem *h = d->hw;
	int ret = clk_prepare_enable(d->clk);

	if (ret)
		return ret;
	ret = reset_control_reset(d->reset);
	if (ret) {
		clk_disable_unprepare(d->clk);
		return ret;
	}
	writel((readl(d->memory) & ~BIT(2)) | BIT(3), d->memory);
	s31_reg_set(h, rst_conf, clk_en, 1);
	s31_reg_set(h, rst_conf, axim_rd_rst, 1);
	s31_reg_set(h, rst_conf, axim_rd_rst, 0);
	s31_reg_set(h, rst_conf, axim_wr_rst, 1);
	s31_reg_set(h, rst_conf, axim_wr_rst, 0);
	writel(0, &h->out_arb_config.val);
	writel(0, &h->in_arb_config.val);
	writel(0x2f000000, &h->intr_mem_start_addr.val);
	writel(0x2f080000, &h->intr_mem_end_addr.val);
	writel(0x50000000, &h->extr_mem_start_addr.val);
	writel(0x51000000, &h->extr_mem_end_addr.val);
	return 0;
}
static int s31_dma2d_suspend(struct device *dev)
{
	struct s31_dma2d *d = dev_get_drvdata(dev);

	writel(readl(d->memory) | BIT(2) | BIT(3), d->memory);
	clk_disable_unprepare(d->clk);
	return 0;
}
static const struct dev_pm_ops s31_dma2d_pm = {
	SET_RUNTIME_PM_OPS(s31_dma2d_suspend, s31_dma2d_resume, NULL)
		SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
					pm_runtime_force_resume)
};
static int s31_dma2d_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s31_dma2d *d;
	struct resource *res;
	int ret, irq;

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	d->dev = dev;
	d->hw = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(d->hw))
		return PTR_ERR(d->hw);
	d->descs = devm_platform_get_and_ioremap_resource(pdev, 1, &res);
	if (IS_ERR(d->descs))
		return PTR_ERR(d->descs);
	if (resource_size(res) < 96)
		return -EINVAL;
	d->desc_phys = res->start;
	d->memory = devm_platform_ioremap_resource(pdev, 2);
	if (IS_ERR(d->memory))
		return PTR_ERR(d->memory);
	d->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(d->clk))
		return dev_err_probe(dev, PTR_ERR(d->clk), "clock\n");
	d->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(d->reset))
		return dev_err_probe(dev, PTR_ERR(d->reset), "reset\n");
	mutex_init(&d->lock);
	init_completion(&d->done);
	platform_set_drvdata(pdev, d);
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	for (irq = 0; irq < 3; irq++) {
		ret = platform_get_irq(pdev, irq);
		if (ret < 0)
			return ret;
		ret = devm_request_irq(dev, ret, s31_dma2d_irq, 0,
				       dev_name(dev), d);
		if (ret)
			return ret;
	}
	d->wq = alloc_ordered_workqueue("s31-dma2d",
					WQ_MEM_RECLAIM | WQ_FREEZABLE);
	if (!d->wq)
		return -ENOMEM;
	INIT_WORK(&d->work, s31_dma2d_work);
	INIT_LIST_HEAD(&d->engine.channels);
	d->engine.dev = dev;
	dma_cap_set(DMA_MEMCPY, d->engine.cap_mask);
	dma_cap_set(DMA_INTERLEAVE, d->engine.cap_mask);
	d->engine.device_alloc_chan_resources = s31_dma2d_alloc;
	d->engine.device_free_chan_resources = s31_dma2d_release;
	d->engine.device_prep_dma_memcpy = s31_dma2d_prep_copy;
	d->engine.device_prep_interleaved_dma = s31_dma2d_prep_interleaved;
	d->engine.device_issue_pending = s31_dma2d_issue;
	d->engine.device_tx_status = s31_dma2d_status;
	d->engine.device_terminate_all = s31_dma2d_terminate;
	d->engine.device_synchronize = s31_dma2d_sync;
	d->engine.directions = BIT(DMA_MEM_TO_MEM);
	d->chan.desc_free = s31_dma2d_free_desc;
	vchan_init(&d->chan, &d->engine);
	pm_runtime_set_autosuspend_delay(dev, 100);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);
	ret = dma_async_device_register(&d->engine);
	if (ret)
		goto disable;
	ret = of_dma_controller_register(dev->of_node, s31_dma2d_xlate, d);
	if (!ret)
		return 0;
	dma_async_device_unregister(&d->engine);
disable:
	pm_runtime_disable(dev);
	tasklet_kill(&d->chan.task);
	destroy_workqueue(d->wq);
	return ret;
}
static void s31_dma2d_remove(struct platform_device *pdev)
{
	struct s31_dma2d *d = platform_get_drvdata(pdev);

	WRITE_ONCE(d->removing, true);
	of_dma_controller_free(pdev->dev.of_node);
	dma_async_device_unregister(&d->engine);
	s31_dma2d_release(&d->chan.chan);
	destroy_workqueue(d->wq);
	tasklet_kill(&d->chan.task);
	pm_runtime_disable(d->dev);
	if (!pm_runtime_status_suspended(d->dev))
		s31_dma2d_suspend(d->dev);
}
static const struct of_device_id s31_dma2d_match[] = {
	{ .compatible = "espressif,esp32s31-dma2d" },
	{}
};
MODULE_DEVICE_TABLE(of, s31_dma2d_match);
static struct platform_driver s31_dma2d_driver = {
	.probe = s31_dma2d_probe,
	.remove = s31_dma2d_remove,
	.driver = { .name = "esp32s31-dma2d",
		    .of_match_table = s31_dma2d_match,
		    .pm = &s31_dma2d_pm },
};
module_platform_driver(s31_dma2d_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ESP32-S31 shared image DMA and DMAengine copies");
