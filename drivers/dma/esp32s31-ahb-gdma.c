// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S31 AHB GDMA dmaengine driver.
 *
 * Five coupled TX/RX pairs are exported as memcpy channels.  Descriptor
 * storage is carved from loader-reserved HP SRAM because this DMA cannot
 * fetch linked-list descriptors from cached PSRAM.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/genalloc.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "virt-dma.h"

#define AHB_CHANNELS			5
#define AHB_DESC_MAX			4095U
#define AHB_DESC_SIZE			12U
#define AHB_RX_INT(ch)			((ch) * 0x10)
#define AHB_RX_RAW			0x00
#define AHB_RX_ST			0x04
#define AHB_RX_ENA			0x08
#define AHB_RX_CLR			0x0c
#define AHB_CH_BASE(ch)			(0x100 + (ch) * 0x100)
#define AHB_RX_CONF0			0x00
#define AHB_RX_CONF1			0x04
#define AHB_RX_LINK			0x10
#define AHB_RX_LINK_ADDR		0x14
#define AHB_RX_PERI_SEL			0x38
#define AHB_TX_CONF0			0x80
#define AHB_TX_CONF1			0x84
#define AHB_TX_LINK			0x90
#define AHB_TX_LINK_ADDR		0x94
#define AHB_TX_PERI_SEL			0xb8
#define AHB_MISC_CONF			0x0a4
#define AHB_DATE			0x0a8
#define AHB_MEM_START			0x600
#define AHB_MEM_END			0x604
#define AHB_MODULE_CLK			0x618
#define AHB_RX_RST			BIT(0)
#define AHB_RX_DESC_BURST		BIT(2)
#define AHB_RX_MEM_TRANS		BIT(4)
#define AHB_TX_RST			BIT(0)
#define AHB_TX_DESC_BURST		BIT(4)
#define AHB_TX_AUTO_WRBACK		BIT(2)
#define AHB_TX_EOF_MODE			BIT(3)
#define AHB_CHECK_OWNER			BIT(12)
#define AHB_RX_STOP			BIT(1)
#define AHB_RX_START			BIT(2)
#define AHB_TX_STOP			BIT(0)
#define AHB_TX_START			BIT(1)
#define AHB_RX_SUC_EOF			BIT(1)
#define AHB_RX_ERR_EOF			BIT(2)
#define AHB_RX_DESC_ERR			BIT(3)
#define AHB_RX_DESC_EMPTY		BIT(4)
#define AHB_RX_RESP_ERR			BIT(7)
#define AHB_RX_ERROR			(AHB_RX_ERR_EOF | AHB_RX_DESC_ERR | \
					 AHB_RX_DESC_EMPTY | AHB_RX_RESP_ERR)
#define AHB_MISC_RESET			BIT(0)
#define AHB_MISC_CLK			BIT(3)
#define AHB_DESC_BUF_SIZE		GENMASK(11, 0)
#define AHB_DESC_DATA_LEN		GENMASK(23, 12)
#define AHB_DESC_EOF			BIT(30)
#define AHB_DESC_OWNER			BIT(31)
#define AHB_M2M_DUMMY			9

struct esp32s31_ahb_hw_desc {
	__le32 control;
	__le32 buffer;
	__le32 next;
};

struct esp32s31_ahb_desc {
	struct virt_dma_desc vd;
	void __iomem *pool;
	size_t pool_len;
	size_t len;
	dma_addr_t tx_dma;
	dma_addr_t rx_dma;
};

struct esp32s31_ahb;

struct esp32s31_ahb_chan {
	struct virt_dma_chan vc;
	struct esp32s31_ahb *gdma;
	struct esp32s31_ahb_desc *active;
	unsigned int id;
};

struct esp32s31_ahb {
	struct dma_device dma_dev;
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct gen_pool *pool;
	struct esp32s31_ahb_chan chans[AHB_CHANNELS];
};

static inline struct esp32s31_ahb_chan *to_ahb_chan(struct dma_chan *chan)
{
	return container_of(chan, struct esp32s31_ahb_chan, vc.chan);
}

static inline struct esp32s31_ahb_desc *to_ahb_desc(struct virt_dma_desc *vd)
{
	return container_of(vd, struct esp32s31_ahb_desc, vd);
}

static inline void __iomem *ahb_ch_reg(struct esp32s31_ahb_chan *chan,
				       u32 reg)
{
	return chan->gdma->base + AHB_CH_BASE(chan->id) + reg;
}

static void esp32s31_ahb_free_desc(struct virt_dma_desc *vd)
{
	struct esp32s31_ahb_desc *desc = to_ahb_desc(vd);
	struct esp32s31_ahb_chan *chan = to_ahb_chan(vd->tx.chan);

	gen_pool_free(chan->gdma->pool, (unsigned long)desc->pool,
		      desc->pool_len);
	kfree(desc);
}

static void esp32s31_ahb_reset(struct esp32s31_ahb_chan *chan)
{
	u32 val;

	writel(AHB_RX_STOP, ahb_ch_reg(chan, AHB_RX_LINK));
	writel(AHB_TX_STOP, ahb_ch_reg(chan, AHB_TX_LINK));
	writel(0, chan->gdma->base + AHB_RX_INT(chan->id) + AHB_RX_ENA);
	writel(GENMASK(7, 0),
	       chan->gdma->base + AHB_RX_INT(chan->id) + AHB_RX_CLR);
	val = readl(ahb_ch_reg(chan, AHB_RX_CONF0));
	writel(val | AHB_RX_RST, ahb_ch_reg(chan, AHB_RX_CONF0));
	writel(val & ~AHB_RX_RST, ahb_ch_reg(chan, AHB_RX_CONF0));
	val = readl(ahb_ch_reg(chan, AHB_TX_CONF0));
	writel(val | AHB_TX_RST, ahb_ch_reg(chan, AHB_TX_CONF0));
	writel(val & ~AHB_TX_RST, ahb_ch_reg(chan, AHB_TX_CONF0));
}

static void esp32s31_ahb_start(struct esp32s31_ahb_chan *chan,
			       struct esp32s31_ahb_desc *desc)
{
	u32 clk_mask = BIT(chan->id) | BIT(5 + chan->id) |
		       BIT(10 + chan->id) | BIT(15 + chan->id) |
		       BIT(20 + chan->id) | BIT(27) | BIT(28);

	writel(readl(chan->gdma->base + AHB_MODULE_CLK) | clk_mask,
	       chan->gdma->base + AHB_MODULE_CLK);
	esp32s31_ahb_reset(chan);
	writel(AHB_RX_DESC_BURST | AHB_RX_MEM_TRANS,
	       ahb_ch_reg(chan, AHB_RX_CONF0));
	writel(AHB_CHECK_OWNER, ahb_ch_reg(chan, AHB_RX_CONF1));
	writel(AHB_M2M_DUMMY, ahb_ch_reg(chan, AHB_RX_PERI_SEL));
	writel(AHB_TX_AUTO_WRBACK | AHB_TX_EOF_MODE | AHB_TX_DESC_BURST,
	       ahb_ch_reg(chan, AHB_TX_CONF0));
	writel(AHB_CHECK_OWNER, ahb_ch_reg(chan, AHB_TX_CONF1));
	writel(AHB_M2M_DUMMY, ahb_ch_reg(chan, AHB_TX_PERI_SEL));
	dma_wmb();
	writel(lower_32_bits(desc->rx_dma), ahb_ch_reg(chan, AHB_RX_LINK_ADDR));
	writel(lower_32_bits(desc->tx_dma), ahb_ch_reg(chan, AHB_TX_LINK_ADDR));
	writel(AHB_RX_SUC_EOF | AHB_RX_ERROR,
	       chan->gdma->base + AHB_RX_INT(chan->id) + AHB_RX_ENA);
	writel(AHB_RX_START, ahb_ch_reg(chan, AHB_RX_LINK));
	writel(AHB_TX_START, ahb_ch_reg(chan, AHB_TX_LINK));
}

static void esp32s31_ahb_start_pending(struct esp32s31_ahb_chan *chan)
{
	struct virt_dma_desc *vd;

	if (chan->active)
		return;
	vd = vchan_next_desc(&chan->vc);
	if (!vd)
		return;
	list_del(&vd->node);
	chan->active = to_ahb_desc(vd);
	esp32s31_ahb_start(chan, chan->active);
}

static struct dma_async_tx_descriptor *
esp32s31_ahb_prep_memcpy(struct dma_chan *dchan, dma_addr_t dst,
			 dma_addr_t src, size_t len, unsigned long flags)
{
	struct esp32s31_ahb_chan *chan = to_ahb_chan(dchan);
	struct esp32s31_ahb_desc *desc;
	unsigned long pool;
	unsigned int count, i;
	dma_addr_t pool_dma;
	size_t remaining = len;

	if (!len || upper_32_bits(src) || upper_32_bits(dst))
		return NULL;
	count = DIV_ROUND_UP(len, AHB_DESC_MAX);
	desc = kzalloc(sizeof(*desc), GFP_NOWAIT);
	if (!desc)
		return NULL;
	desc->pool_len = count * AHB_DESC_SIZE * 2;
	pool = gen_pool_alloc(chan->gdma->pool, desc->pool_len);
	if (!pool) {
		kfree(desc);
		return NULL;
	}
	desc->pool = (void __iomem *)pool;
	desc->len = len;
	pool_dma = gen_pool_virt_to_phys(chan->gdma->pool, pool);
	desc->tx_dma = pool_dma;
	desc->rx_dma = pool_dma + count * AHB_DESC_SIZE;
	memset_io(desc->pool, 0, desc->pool_len);

	for (i = 0; i < count; i++) {
		void __iomem *tx = desc->pool + i * AHB_DESC_SIZE;
		void __iomem *rx = desc->pool + (count + i) * AHB_DESC_SIZE;
		size_t chunk = min_t(size_t, remaining, AHB_DESC_MAX);
		u32 tx_control = FIELD_PREP(AHB_DESC_BUF_SIZE, chunk) |
				 FIELD_PREP(AHB_DESC_DATA_LEN, chunk) |
				 AHB_DESC_OWNER;
		u32 rx_control = FIELD_PREP(AHB_DESC_BUF_SIZE, chunk) |
				 FIELD_PREP(AHB_DESC_DATA_LEN, chunk) |
				 AHB_DESC_OWNER;
		dma_addr_t next_tx = i + 1 == count ? 0 :
				     desc->tx_dma + (i + 1) * AHB_DESC_SIZE;
		dma_addr_t next_rx = i + 1 == count ? 0 :
				     desc->rx_dma + (i + 1) * AHB_DESC_SIZE;

		writel(tx_control | (i + 1 == count ? AHB_DESC_EOF : 0), tx);
		writel(lower_32_bits(src), tx + 4);
		writel(lower_32_bits(next_tx), tx + 8);
		writel(rx_control, rx);
		writel(lower_32_bits(dst), rx + 4);
		writel(lower_32_bits(next_rx), rx + 8);
		src += chunk;
		dst += chunk;
		remaining -= chunk;
	}
	return vchan_tx_prep(&chan->vc, &desc->vd, flags);
}

static irqreturn_t esp32s31_ahb_irq(int irq, void *data)
{
	struct esp32s31_ahb_chan *chan = data;
	struct esp32s31_ahb_desc *desc;
	unsigned long flags;
	u32 status;

	status = readl(chan->gdma->base + AHB_RX_INT(chan->id) + AHB_RX_ST);
	if (!status)
		return IRQ_NONE;
	writel(status, chan->gdma->base + AHB_RX_INT(chan->id) + AHB_RX_CLR);
	spin_lock_irqsave(&chan->vc.lock, flags);
	desc = chan->active;
	if (desc) {
		chan->active = NULL;
		if (status & AHB_RX_ERROR)
			desc->vd.tx_result.result = DMA_TRANS_WRITE_FAILED;
		else
			desc->vd.tx_result.result = DMA_TRANS_NOERROR;
		desc->vd.tx_result.residue = 0;
		vchan_cookie_complete(&desc->vd);
	}
	esp32s31_ahb_start_pending(chan);
	spin_unlock_irqrestore(&chan->vc.lock, flags);
	return IRQ_HANDLED;
}

static void esp32s31_ahb_issue_pending(struct dma_chan *dchan)
{
	struct esp32s31_ahb_chan *chan = to_ahb_chan(dchan);
	unsigned long flags;

	spin_lock_irqsave(&chan->vc.lock, flags);
	if (vchan_issue_pending(&chan->vc))
		esp32s31_ahb_start_pending(chan);
	spin_unlock_irqrestore(&chan->vc.lock, flags);
}

static enum dma_status esp32s31_ahb_tx_status(struct dma_chan *dchan,
					      dma_cookie_t cookie,
					      struct dma_tx_state *state)
{
	struct esp32s31_ahb_chan *chan = to_ahb_chan(dchan);
	enum dma_status status = dma_cookie_status(dchan, cookie, state);
	unsigned long flags;

	if (status == DMA_COMPLETE || !state)
		return status;
	spin_lock_irqsave(&chan->vc.lock, flags);
	if (chan->active && chan->active->vd.tx.cookie == cookie)
		dma_set_residue(state, chan->active->len);
	spin_unlock_irqrestore(&chan->vc.lock, flags);
	return status;
}

static int esp32s31_ahb_terminate_all(struct dma_chan *dchan)
{
	struct esp32s31_ahb_chan *chan = to_ahb_chan(dchan);
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&chan->vc.lock, flags);
	if (chan->active)
		dev_err(chan->gdma->dev,
			"ch%u timeout: raw=%#x st=%#x rx_conf=%#x/%#x rx_link=%#x rx_addr=%#x tx_conf=%#x/%#x tx_link=%#x tx_addr=%#x tx_desc=%#x rx_desc=%#x\n",
			chan->id,
			readl(chan->gdma->base + AHB_RX_INT(chan->id) + AHB_RX_RAW),
			readl(chan->gdma->base + AHB_RX_INT(chan->id) + AHB_RX_ST),
			readl(ahb_ch_reg(chan, AHB_RX_CONF0)),
			readl(ahb_ch_reg(chan, AHB_RX_CONF1)),
			readl(ahb_ch_reg(chan, AHB_RX_LINK)),
			readl(ahb_ch_reg(chan, AHB_RX_LINK_ADDR)),
			readl(ahb_ch_reg(chan, AHB_TX_CONF0)),
			readl(ahb_ch_reg(chan, AHB_TX_CONF1)),
			readl(ahb_ch_reg(chan, AHB_TX_LINK)),
			readl(ahb_ch_reg(chan, AHB_TX_LINK_ADDR)),
			readl(chan->active->pool),
			readl(chan->active->pool + chan->active->pool_len / 2));
	esp32s31_ahb_reset(chan);
	if (chan->active) {
		vchan_terminate_vdesc(&chan->active->vd);
		chan->active = NULL;
	}
	vchan_get_all_descriptors(&chan->vc, &head);
	spin_unlock_irqrestore(&chan->vc.lock, flags);
	vchan_dma_desc_free_list(&chan->vc, &head);
	return 0;
}

static void esp32s31_ahb_free_resources(struct dma_chan *dchan)
{
	struct esp32s31_ahb_chan *chan = to_ahb_chan(dchan);

	esp32s31_ahb_terminate_all(dchan);
	vchan_free_chan_resources(&chan->vc);
}

static int esp32s31_ahb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp32s31_ahb *gdma;
	struct dma_device *dma_dev;
	struct resource *pool_res;
	unsigned int i;
	u32 val;
	int ret;

	gdma = devm_kzalloc(dev, sizeof(*gdma), GFP_KERNEL);
	if (!gdma)
		return -ENOMEM;
	gdma->dev = dev;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "32-bit DMA mask unavailable\n");
	gdma->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gdma->base))
		return PTR_ERR(gdma->base);
	gdma->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(gdma->clk))
		return dev_err_probe(dev, PTR_ERR(gdma->clk), "clock unavailable\n");
	gdma->pool = devm_gen_pool_create(dev, 4, -1, NULL);
	if (IS_ERR(gdma->pool))
		return PTR_ERR(gdma->pool);
	{
		void __iomem *pool_base;

		pool_base = devm_platform_get_and_ioremap_resource(pdev, 1,
								   &pool_res);
		if (IS_ERR(pool_base))
			return PTR_ERR(pool_base);
		ret = gen_pool_add_virt(gdma->pool, (unsigned long)pool_base,
					pool_res->start, resource_size(pool_res), -1);
		if (ret)
			return ret;
	}

	writel(0x2f000000, gdma->base + AHB_MEM_START);
	writel(0x53ffffff, gdma->base + AHB_MEM_END);
	/*
	 * Use the reset-safe channel-0 synchronizer state.  The remaining
	 * per-channel clocks are enabled lazily immediately before a transfer.
	 */
	writel(BIT(0), gdma->base + AHB_MODULE_CLK);
	val = readl(gdma->base + AHB_MISC_CONF) | AHB_MISC_CLK;
	writel(val | AHB_MISC_RESET, gdma->base + AHB_MISC_CONF);
	writel(val, gdma->base + AHB_MISC_CONF);

	dma_dev = &gdma->dma_dev;
	dma_dev->dev = dev;
	INIT_LIST_HEAD(&dma_dev->channels);
	dma_cap_set(DMA_MEMCPY, dma_dev->cap_mask);
	dma_dev->copy_align = DMAENGINE_ALIGN_1_BYTE;
	dma_dev->directions = BIT(DMA_MEM_TO_MEM);
	dma_dev->residue_granularity = DMA_RESIDUE_GRANULARITY_DESCRIPTOR;
	dma_dev->device_free_chan_resources = esp32s31_ahb_free_resources;
	dma_dev->device_prep_dma_memcpy = esp32s31_ahb_prep_memcpy;
	dma_dev->device_issue_pending = esp32s31_ahb_issue_pending;
	dma_dev->device_tx_status = esp32s31_ahb_tx_status;
	dma_dev->device_terminate_all = esp32s31_ahb_terminate_all;

	for (i = 0; i < AHB_CHANNELS; i++) {
		struct esp32s31_ahb_chan *chan = &gdma->chans[i];
		int irq = platform_get_irq(pdev, i);

		if (irq < 0)
			return irq;
		chan->gdma = gdma;
		chan->id = i;
		chan->vc.desc_free = esp32s31_ahb_free_desc;
		vchan_init(&chan->vc, dma_dev);
		ret = devm_request_irq(dev, irq, esp32s31_ahb_irq, 0,
				       dev_name(dev), chan);
		if (ret)
			return ret;
	}
	ret = dma_async_device_register(dma_dev);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, gdma);
	dev_info(dev, "AHB GDMA ready: 5 memcpy pairs, version %#x\n",
		 readl(gdma->base + AHB_DATE));
	return 0;
}

static void esp32s31_ahb_remove(struct platform_device *pdev)
{
	struct esp32s31_ahb *gdma = platform_get_drvdata(pdev);
	unsigned int i;

	dma_async_device_unregister(&gdma->dma_dev);
	for (i = 0; i < AHB_CHANNELS; i++) {
		esp32s31_ahb_terminate_all(&gdma->chans[i].vc.chan);
		tasklet_kill(&gdma->chans[i].vc.task);
	}
}

static const struct of_device_id esp32s31_ahb_of_match[] = {
	{ .compatible = "espressif,esp32s31-ahb-gdma" }, { }
};
MODULE_DEVICE_TABLE(of, esp32s31_ahb_of_match);

static struct platform_driver esp32s31_ahb_driver = {
	.probe = esp32s31_ahb_probe,
	.remove = esp32s31_ahb_remove,
	.driver = {
		.name = "esp32s31-ahb-gdma",
		.of_match_table = esp32s31_ahb_of_match,
	},
};
module_platform_driver(esp32s31_ahb_driver);
MODULE_DESCRIPTION("ESP32-S31 AHB GDMA driver");
MODULE_LICENSE("GPL");
