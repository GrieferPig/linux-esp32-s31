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
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/reset.h>

#include "virt-dma.h"
#include "esp32s31-ahb-gdma.h"

#define AHB_CHANNELS			5
#define AHB_DESC_MAX			4095U
/* The M2M datapath ignores the low two buffer-address bits. */
#define AHB_MEMCPY_DESC_MAX		4092U
#define AHB_DESC_SIZE			12U
#define AHB_RX_INT(ch)			((ch) * 0x10)
#define AHB_TX_INT(ch)			(0x50 + (ch) * 0x10)
#define AHB_RX_RAW			0x00
#define AHB_RX_ST			0x04
#define AHB_RX_ENA			0x08
#define AHB_RX_CLR			0x0c
#define AHB_CH_BASE(ch)			(0x100 + (ch) * 0x100)
#define AHB_RX_CONF0			0x00
#define AHB_RX_CONF1			0x04
#define AHB_RX_LINK			0x10
#define AHB_RX_LINK_ADDR		0x14
#define AHB_RX_STATE			0x18
#define AHB_RX_PERI_SEL			0x38
#define AHB_TX_CONF0			0x80
#define AHB_TX_CONF1			0x84
#define AHB_TX_LINK			0x90
#define AHB_TX_LINK_ADDR		0x94
#define AHB_TX_STATE			0x98
#define AHB_TX_DSCR			0xa8
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
#define AHB_RX_DONE			BIT(0)
#define AHB_RX_SUC_EOF			BIT(1)
#define AHB_RX_ERR_EOF			BIT(2)
#define AHB_RX_DESC_ERR			BIT(3)
#define AHB_RX_DESC_EMPTY		BIT(4)
#define AHB_RX_RESP_ERR			BIT(7)
#define AHB_TX_DSCR_ERR			BIT(2)
#define AHB_TX_RESP_ERR			BIT(6)
#define AHB_RX_ERROR			(AHB_RX_ERR_EOF | AHB_RX_DESC_ERR | \
					 AHB_RX_DESC_EMPTY | AHB_RX_RESP_ERR)
#define AHB_MISC_RESET			BIT(0)
#define AHB_MISC_CLK			BIT(3)
#define AHB_DESC_BUF_SIZE		GENMASK(11, 0)
#define AHB_DESC_DATA_LEN		GENMASK(23, 12)
#define AHB_DESC_EOF			BIT(30)
#define AHB_DESC_OWNER			BIT(31)
#define AHB_M2M_DUMMY			9
#define AHB_RX_SUC_EOF_DESC		0x1c

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
	unsigned int ndesc;
	dma_addr_t tx_dma;
	dma_addr_t rx_dma;
	enum dma_transfer_direction direction;
	struct esp32s31_ahb_rx_result rx_result;
	bool cyclic;
};

struct esp32s31_ahb;

struct esp32s31_ahb_chan {
	struct virt_dma_chan vc;
	struct esp32s31_ahb *gdma;
	struct esp32s31_ahb_desc *active_tx;
	struct esp32s31_ahb_desc *active_rx;
	struct esp32s31_ahb_desc *cyclic_tx;
	struct esp32s31_ahb_desc *cyclic_rx;
	u32 cyclic_generation;
	unsigned int tx_node;
	unsigned int rx_node;
	unsigned int tx_irqs;
	unsigned int rx_irqs;
	struct dma_slave_config config;
	u32 request_id;
	unsigned int id;
	int rx_irq;
	int tx_irq;
	unsigned int users;
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

static void esp32s31_ahb_reset(struct esp32s31_ahb_chan *chan,
			       enum dma_transfer_direction direction)
{
	u32 val;

	if (direction == DMA_DEV_TO_MEM) {
		writel(AHB_RX_STOP, ahb_ch_reg(chan, AHB_RX_LINK));
		writel(0, chan->gdma->base + AHB_RX_INT(chan->id) + AHB_RX_ENA);
		writel(GENMASK(7, 0),
		       chan->gdma->base + AHB_RX_INT(chan->id) + AHB_RX_CLR);
		val = readl(ahb_ch_reg(chan, AHB_RX_CONF0));
		writel(val | AHB_RX_RST, ahb_ch_reg(chan, AHB_RX_CONF0));
		writel(val & ~AHB_RX_RST, ahb_ch_reg(chan, AHB_RX_CONF0));
	} else {
		writel(AHB_TX_STOP, ahb_ch_reg(chan, AHB_TX_LINK));
		writel(0, chan->gdma->base + AHB_TX_INT(chan->id) + AHB_RX_ENA);
		writel(GENMASK(7, 0),
		       chan->gdma->base + AHB_TX_INT(chan->id) + AHB_RX_CLR);
		val = readl(ahb_ch_reg(chan, AHB_TX_CONF0));
		writel(val | AHB_TX_RST, ahb_ch_reg(chan, AHB_TX_CONF0));
		writel(val & ~AHB_TX_RST, ahb_ch_reg(chan, AHB_TX_CONF0));
	}
}

static void esp32s31_ahb_start(struct esp32s31_ahb_chan *chan,
			       struct esp32s31_ahb_desc *desc)
{
	u32 clk_mask = BIT(chan->id) | BIT(5 + chan->id) |
		       BIT(10 + chan->id) | BIT(15 + chan->id) |
		       BIT(20 + chan->id) | BIT(27) | BIT(28);

	writel(readl(chan->gdma->base + AHB_MODULE_CLK) | clk_mask,
	       chan->gdma->base + AHB_MODULE_CLK);
	/*
	 * Do not reset a slave TX channel between adjacent descriptors.  The
	 * TX EOF interrupt is configured for "data popped" mode, so the link
	 * FSM is already parked and a new address/start is sufficient.  A
	 * reset here races the UHCI handshake and can leave it unable to accept
	 * another request after a few transfers.  ESP-IDF's uhci_do_transmit()
	 * follows the same sequence: mount, set descriptor address, start.
	 * Resets remain confined to terminate/recovery paths and cyclic RX EOF.
	 */
	if (desc->direction == DMA_MEM_TO_MEM)
		esp32s31_ahb_reset(chan, desc->direction);
	dma_wmb();
	if (desc->direction == DMA_DEV_TO_MEM ||
	    desc->direction == DMA_MEM_TO_MEM) {
		writel(AHB_RX_DESC_BURST |
		       (desc->direction == DMA_MEM_TO_MEM ? AHB_RX_MEM_TRANS : 0),
		       ahb_ch_reg(chan, AHB_RX_CONF0));
		writel(AHB_CHECK_OWNER, ahb_ch_reg(chan, AHB_RX_CONF1));
		writel(desc->direction == DMA_MEM_TO_MEM ? AHB_M2M_DUMMY : chan->request_id,
		       ahb_ch_reg(chan, AHB_RX_PERI_SEL));
		writel(lower_32_bits(desc->rx_dma), ahb_ch_reg(chan, AHB_RX_LINK_ADDR));
		writel(AHB_RX_SUC_EOF | AHB_RX_ERROR,
		       chan->gdma->base + AHB_RX_INT(chan->id) + AHB_RX_ENA);
		writel(AHB_RX_START, ahb_ch_reg(chan, AHB_RX_LINK));
	}
	if (desc->direction == DMA_MEM_TO_DEV ||
	    desc->direction == DMA_MEM_TO_MEM) {
		writel(AHB_TX_AUTO_WRBACK | AHB_TX_EOF_MODE | AHB_TX_DESC_BURST,
		       ahb_ch_reg(chan, AHB_TX_CONF0));
		writel(AHB_CHECK_OWNER, ahb_ch_reg(chan, AHB_TX_CONF1));
		writel(desc->direction == DMA_MEM_TO_MEM ? AHB_M2M_DUMMY : chan->request_id,
		       ahb_ch_reg(chan, AHB_TX_PERI_SEL));
		writel(lower_32_bits(desc->tx_dma), ahb_ch_reg(chan, AHB_TX_LINK_ADDR));
		writel(BIT(1) | AHB_TX_DSCR_ERR | AHB_TX_RESP_ERR,
		       chan->gdma->base + AHB_TX_INT(chan->id) + AHB_RX_ENA);
		writel(AHB_TX_START, ahb_ch_reg(chan, AHB_TX_LINK));
	}
}

static void esp32s31_ahb_start_pending(struct esp32s31_ahb_chan *chan)
{
	struct virt_dma_desc *vd;
	struct virt_dma_desc *next;
	struct esp32s31_ahb_desc *desc;

	list_for_each_entry_safe(vd, next, &chan->vc.desc_issued, node) {
		desc = to_ahb_desc(vd);
		if (desc->direction == DMA_DEV_TO_MEM && chan->cyclic_rx)
			continue;
		if ((desc->direction == DMA_DEV_TO_MEM && chan->active_rx) ||
		    (desc->direction == DMA_MEM_TO_DEV && chan->active_tx))
			continue;
		list_del(&vd->node);
		if (desc->direction == DMA_DEV_TO_MEM)
			chan->active_rx = desc;
		else if (desc->direction == DMA_MEM_TO_DEV)
			chan->active_tx = desc;
		else
			chan->active_tx = chan->active_rx = desc;
		esp32s31_ahb_start(chan, desc);
		if (chan->active_tx && chan->active_rx)
			break;
	}
}

/* Start a persistent cyclic slave ring (IDF: gdma_start). */
static void esp32s31_ahb_start_cyclic(struct esp32s31_ahb_chan *chan,
				      struct esp32s31_ahb_desc *desc)
{
	u32 clk_mask = BIT(chan->id) | BIT(5 + chan->id) |
		       BIT(10 + chan->id) | BIT(15 + chan->id) |
		       BIT(20 + chan->id) | BIT(27) | BIT(28);

	writel(readl(chan->gdma->base + AHB_MODULE_CLK) | clk_mask,
	       chan->gdma->base + AHB_MODULE_CLK);
	dev_info(chan->gdma->dev,
		 "cyclic ch%u request=%u direction=%s bytes=%zu periods=%u first-control=%#08x first-buffer=%#08x\n",
		 chan->id, chan->request_id,
		 desc->direction == DMA_MEM_TO_DEV ? "tx" : "rx",
		 desc->len, desc->ndesc, readl(desc->pool),
		 readl(desc->pool + 4));
	esp32s31_ahb_reset(chan, desc->direction);
	if (desc->direction == DMA_MEM_TO_DEV) {
		chan->tx_irqs = 0;
		writel(AHB_TX_AUTO_WRBACK | AHB_TX_EOF_MODE |
		       AHB_TX_DESC_BURST, ahb_ch_reg(chan, AHB_TX_CONF0));
		writel(0, ahb_ch_reg(chan, AHB_TX_CONF1));
		writel(chan->request_id, ahb_ch_reg(chan, AHB_TX_PERI_SEL));
		writel(lower_32_bits(desc->tx_dma),
		       ahb_ch_reg(chan, AHB_TX_LINK_ADDR));
		writel(BIT(1) | AHB_TX_DSCR_ERR | AHB_TX_RESP_ERR,
		       chan->gdma->base + AHB_TX_INT(chan->id) + AHB_RX_ENA);
		writel(AHB_TX_START, ahb_ch_reg(chan, AHB_TX_LINK));
		return;
	}

	chan->rx_irqs = 0;
	writel(AHB_RX_DESC_BURST, ahb_ch_reg(chan, AHB_RX_CONF0));
	/*
	 * Match ESP-IDF: RX circular links do not enable owner checking.  The
	 * engine writes OWNER back after each node, and with owner checking set
	 * a circular link stops when it reaches that node again.  Re-arming the
	 * node while the link is live also races the descriptor prefetcher.
	 */
	writel(0, ahb_ch_reg(chan, AHB_RX_CONF1));
	writel(chan->request_id, ahb_ch_reg(chan, AHB_RX_PERI_SEL));
	writel(lower_32_bits(desc->rx_dma +
			     chan->rx_node * AHB_DESC_SIZE),
	       ahb_ch_reg(chan, AHB_RX_LINK_ADDR));
	writel(AHB_RX_DONE | AHB_RX_SUC_EOF | AHB_RX_ERROR,
	       chan->gdma->base + AHB_RX_INT(chan->id) + AHB_RX_ENA);
	writel(AHB_RX_START, ahb_ch_reg(chan, AHB_RX_LINK));
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

	if (!len || !IS_ALIGNED(src, 4) || !IS_ALIGNED(dst, 4) ||
	    !IS_ALIGNED(len, 4) || upper_32_bits(src) || upper_32_bits(dst))
		return NULL;
	count = DIV_ROUND_UP(len, AHB_MEMCPY_DESC_MAX);
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
	desc->direction = DMA_MEM_TO_MEM;
	pool_dma = gen_pool_virt_to_phys(chan->gdma->pool, pool);
	desc->tx_dma = pool_dma;
	desc->rx_dma = pool_dma + count * AHB_DESC_SIZE;
	memset_io(desc->pool, 0, desc->pool_len);

	for (i = 0; i < count; i++) {
		void __iomem *tx = desc->pool + i * AHB_DESC_SIZE;
		void __iomem *rx = desc->pool + (count + i) * AHB_DESC_SIZE;
		size_t chunk = min_t(size_t, remaining, AHB_MEMCPY_DESC_MAX);
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

static struct dma_async_tx_descriptor *
esp32s31_ahb_prep_slave_sg(struct dma_chan *dchan, struct scatterlist *sgl,
			   unsigned int sg_len,
			   enum dma_transfer_direction direction,
			   unsigned long flags, void *context)
{
	struct esp32s31_ahb_chan *chan = to_ahb_chan(dchan);
	struct esp32s31_ahb_desc *desc;
	struct scatterlist *sg;
	unsigned long pool;
	dma_addr_t pool_dma;
	size_t total = 0, remaining;
	unsigned int count = 0, i, j;

	if (!sg_len || (direction != DMA_MEM_TO_DEV &&
		       direction != DMA_DEV_TO_MEM) ||
	    chan->request_id >= 32)
		return NULL;
	for_each_sg(sgl, sg, sg_len, i) {
		if (!sg_dma_len(sg) || upper_32_bits(sg_dma_address(sg)))
			return NULL;
		count += DIV_ROUND_UP(sg_dma_len(sg), AHB_DESC_MAX);
		total += sg_dma_len(sg);
	}
	if (!count || !total)
		return NULL;
	desc = kzalloc(sizeof(*desc), GFP_NOWAIT);
	if (!desc)
		return NULL;
	desc->pool_len = count * AHB_DESC_SIZE;
	pool = gen_pool_alloc(chan->gdma->pool, desc->pool_len);
	if (!pool) {
		kfree(desc);
		return NULL;
	}
	desc->pool = (void __iomem *)pool;
	desc->len = total;
	desc->ndesc = count;
	desc->direction = direction;
	pool_dma = gen_pool_virt_to_phys(chan->gdma->pool, pool);
	if (direction == DMA_MEM_TO_DEV)
		desc->tx_dma = pool_dma;
	else
		desc->rx_dma = pool_dma;
	memset_io(desc->pool, 0, desc->pool_len);

	j = 0;
	for_each_sg(sgl, sg, sg_len, i) {
		dma_addr_t addr = sg_dma_address(sg);
		remaining = sg_dma_len(sg);
		while (remaining) {
			void __iomem *hw = desc->pool + j * AHB_DESC_SIZE;
			size_t chunk = min_t(size_t, remaining, AHB_DESC_MAX);
			dma_addr_t next = j + 1 == count ? 0 :
				pool_dma + (j + 1) * AHB_DESC_SIZE;
			u32 control = FIELD_PREP(AHB_DESC_BUF_SIZE, chunk) |
				AHB_DESC_OWNER;

			/*
			 * RX descriptors advertise their capacity in BUF_SIZE, while
			 * DATA_LEN is hardware writeback and must start at zero.  If it
			 * is pre-filled with the capacity, an early EOF/error callback
			 * reports untouched receive buffers as valid data.
			 */
			if (direction == DMA_MEM_TO_DEV)
				control |= FIELD_PREP(AHB_DESC_DATA_LEN, chunk);

			if (j + 1 == count)
				control |= AHB_DESC_EOF;
			writel(control, hw);
			writel(lower_32_bits(addr), hw + 4);
			writel(lower_32_bits(next), hw + 8);
			addr += chunk;
			remaining -= chunk;
			j++;
		}
	}
	return vchan_tx_prep(&chan->vc, &desc->vd, flags);
}

static struct dma_async_tx_descriptor *
esp32s31_ahb_prep_dma_cyclic(struct dma_chan *dchan, dma_addr_t buf_addr,
			     size_t buf_len, size_t period_len,
			     enum dma_transfer_direction direction,
			     unsigned long flags)
{
	struct esp32s31_ahb_chan *chan = to_ahb_chan(dchan);
	struct esp32s31_ahb_desc *desc;
	unsigned long pool;
	dma_addr_t pool_dma;
	unsigned int count, i;

	if ((direction != DMA_DEV_TO_MEM && direction != DMA_MEM_TO_DEV) ||
	    !buf_len || !period_len ||
	    buf_len % period_len || period_len > AHB_DESC_MAX ||
	    chan->request_id >= 32)
		return NULL;
	count = buf_len / period_len;
	desc = kzalloc(sizeof(*desc), GFP_NOWAIT);
	if (!desc)
		return NULL;
	desc->pool_len = count * AHB_DESC_SIZE;
	pool = gen_pool_alloc(chan->gdma->pool, desc->pool_len);
	if (!pool) {
		kfree(desc);
		return NULL;
	}
	desc->pool = (void __iomem *)pool;
	desc->len = buf_len;
	desc->ndesc = count;
	desc->direction = direction;
	desc->cyclic = true;
	pool_dma = gen_pool_virt_to_phys(chan->gdma->pool, pool);
	if (direction == DMA_DEV_TO_MEM)
		desc->rx_dma = pool_dma;
	else
		desc->tx_dma = pool_dma;
	memset_io(desc->pool, 0, desc->pool_len);

	/*
	 * IDF mounts the RX descriptors once as a *circular* link
	 * (GDMA_FINAL_LINK_TO_DEFAULT): the last descriptor points back to
	 * the head, so the DMA never stops at the end of the ring and an
	 * idle-EOF just marks the end of the current burst.  Normal EOF events
	 * advance node_index without stopping the link; only an error remounts
	 * and restarts the ring.
	 */
	for (i = 0; i < count; i++) {
		void __iomem *hw = desc->pool + i * AHB_DESC_SIZE;
		dma_addr_t next = pool_dma +
			((i + 1) % count) * AHB_DESC_SIZE;
		u32 control = FIELD_PREP(AHB_DESC_BUF_SIZE, period_len) |
			      FIELD_PREP(AHB_DESC_DATA_LEN, period_len) |
			      AHB_DESC_OWNER;

		if (direction == DMA_MEM_TO_DEV)
			control |= AHB_DESC_EOF;
		writel(control, hw);
		writel(lower_32_bits(buf_addr + i * period_len), hw + 4);
		writel(lower_32_bits(next), hw + 8);
	}
	/* RX EOF is written back by the peripheral; TX marks every period. */
	return vchan_tx_prep(&chan->vc, &desc->vd, flags);
}

static irqreturn_t esp32s31_ahb_irq(int irq, void *data)
{
	struct esp32s31_ahb_chan *chan = data;
	struct esp32s31_ahb_desc *desc;
	struct esp32s31_ahb_rx_result rx_result;
	struct dmaengine_desc_callback cb;
	unsigned long flags;
	u32 status;
	u32 cyclic_generation;
	bool tx = irq == chan->tx_irq;

	if (tx)
		chan->tx_irqs++;
	else
		chan->rx_irqs++;

	status = readl(chan->gdma->base + (tx ? AHB_TX_INT(chan->id) :
					     AHB_RX_INT(chan->id)) + AHB_RX_ST);
	if (!status)
		return IRQ_NONE;
	writel(status, chan->gdma->base + (tx ? AHB_TX_INT(chan->id) :
					     AHB_RX_INT(chan->id)) + AHB_RX_CLR);
	spin_lock_irqsave(&chan->vc.lock, flags);
	if (tx && chan->cyclic_tx) {
		desc = chan->cyclic_tx;
		chan->tx_node = (chan->tx_node + 1) % desc->ndesc;
		dmaengine_desc_get_callback(&desc->vd.tx, &cb);
		spin_unlock_irqrestore(&chan->vc.lock, flags);
		dmaengine_desc_callback_invoke(&cb, NULL);
		return IRQ_HANDLED;
	}
	if (!tx && chan->cyclic_rx) {
		/*
		 * Persistent circular ring, IDF uhci_gdma_rx_callback_done
		 * model with the node_index bookkeeping:
		 *  - RX_DONE  (bit 0): a node filled without EOF.  Deliver the
		 *    node (totally_received=false), advance node_index, and
		 *    let the DMA keep running so a burst longer than the ring
		 *    drains continuously instead of wrapping and losing data.
		 *  - SUC_EOF  (bit 1): the UHCI idle-EOF landed.  Count bytes
		 *    through the hardware-reported EOF node and keep the link live.
		 */
		struct esp32s31_ahb_rx_result *rxres;
		unsigned int actual = 0, i, done = 0;
		bool eof = !!(status & AHB_RX_SUC_EOF);
		bool restart = !!(status & AHB_RX_ERROR);
		u32 ctrl;

		desc = chan->cyclic_rx;
		cyclic_generation = chan->cyclic_generation;
		rxres = &desc->rx_result;
		if (status & AHB_RX_ERROR) {
			rxres->res.result = DMA_TRANS_WRITE_FAILED;
			eof = true;
		} else {
			rxres->res.result = DMA_TRANS_NOERROR;
		}
		rxres->eof = eof;
		rxres->pos = chan->rx_node * (desc->len / desc->ndesc);
		/*
		 * Follow ESP-IDF's node_index model.  A non-EOF RX_DONE completes
		 * exactly the current node.  At EOF, use the hardware's successful
		 * EOF descriptor address instead of guessing from OWNER writeback.
		 */
		if (!eof) {
			ctrl = readl(desc->pool +
				     chan->rx_node * AHB_DESC_SIZE);
			actual = FIELD_GET(AHB_DESC_DATA_LEN, ctrl);
			done = 1;
		} else if (status & AHB_RX_SUC_EOF) {
			dma_addr_t eof_dma = readl(ahb_ch_reg(chan,
							AHB_RX_SUC_EOF_DESC));

			if (eof_dma >= desc->rx_dma &&
			    eof_dma < desc->rx_dma + desc->pool_len &&
			    !((eof_dma - desc->rx_dma) % AHB_DESC_SIZE)) {
				unsigned int eof_node =
					(eof_dma - desc->rx_dma) / AHB_DESC_SIZE;

				for (i = 0; i < desc->ndesc; i++) {
					unsigned int node = (chan->rx_node + i) %
							   desc->ndesc;

					ctrl = readl(desc->pool +
						     node * AHB_DESC_SIZE);
					actual += FIELD_GET(AHB_DESC_DATA_LEN,
							    ctrl);
					done++;
					if (node == eof_node)
						break;
				}
			}
		}
		rxres->res.residue = desc->len - actual;
		chan->rx_node = (chan->rx_node + done) % desc->ndesc;
		dmaengine_desc_get_callback(&desc->vd.tx, &cb);
		/* The descriptor may be terminated while its callback runs. */
		rx_result = *rxres;
		spin_unlock_irqrestore(&chan->vc.lock, flags);
		dmaengine_desc_callback_invoke(&cb, &rx_result.res);
		spin_lock_irqsave(&chan->vc.lock, flags);
		if (chan->cyclic_rx != desc ||
		    chan->cyclic_generation != cyclic_generation) {
			spin_unlock_irqrestore(&chan->vc.lock, flags);
			return IRQ_HANDLED;
		}
		if (restart) {
			unsigned int k;

			/*
			 * Keep a normal UART idle EOF on the live circular link so the
			 * next byte cannot arrive in a stop/reset window.  Reset and
			 * remount only after a real DMA error.
			 */
			for (k = 0; k < desc->ndesc; k++) {
				u32 size = FIELD_GET(AHB_DESC_BUF_SIZE,
						     readl(desc->pool +
							   k * AHB_DESC_SIZE));

				writel(FIELD_PREP(AHB_DESC_BUF_SIZE, size) |
				       FIELD_PREP(AHB_DESC_DATA_LEN, size) |
				       AHB_DESC_OWNER,
				       desc->pool + k * AHB_DESC_SIZE);
			}
			chan->rx_node = 0;
			dma_wmb();
			esp32s31_ahb_start_cyclic(chan, desc);
		}
		spin_unlock_irqrestore(&chan->vc.lock, flags);
		return IRQ_HANDLED;
	}
	desc = tx ? chan->active_tx : chan->active_rx;
	if (desc) {
		/*
		 * RX one-shot links need an explicit stop before completion.  A TX
		 * link is already parked at EOF and must remain untouched so the next
		 * descriptor can be started directly from its completion callback.
		 */
		if (desc->direction == DMA_DEV_TO_MEM && !tx)
			writel(AHB_RX_STOP, ahb_ch_reg(chan, AHB_RX_LINK));
		if (desc->direction == DMA_MEM_TO_MEM)
			chan->active_tx = chan->active_rx = NULL;
		else if (tx)
			chan->active_tx = NULL;
		else
			chan->active_rx = NULL;
		if (status & (tx ? (AHB_TX_DSCR_ERR | AHB_TX_RESP_ERR) :
			       AHB_RX_ERROR))
			desc->vd.tx_result.result = DMA_TRANS_WRITE_FAILED;
		else
			desc->vd.tx_result.result = DMA_TRANS_NOERROR;
		if (desc->direction == DMA_DEV_TO_MEM) {
			unsigned int actual = 0, prev;
			u32 ctrl;
			int retry = 0, i;

			for (;;) {
				prev = actual;
				actual = 0;
				for (i = 0; i < desc->ndesc; i++) {
					ctrl = readl(desc->pool +
						    i * AHB_DESC_SIZE);
					if (ctrl & AHB_DESC_OWNER)
						break;
					actual += FIELD_GET(AHB_DESC_DATA_LEN,
							   ctrl);
				}
				if (actual == prev)
					break;
				if (++retry > 1000)
					break;
				cpu_relax();
			}
			desc->vd.tx_result.residue = desc->len - actual;
			/*
			 * Re-arm the ring before the UART RX FIFO overflows:
			 * the tasklet is too slow when a burst is fragmented
			 * by idle EOFs, so invoke the client callback in IRQ
			 * context like IDF does in its gdma ISR.
			 */
			dmaengine_desc_get_callback(&desc->vd.tx, &cb);
		} else {
			desc->vd.tx_result.residue = 0;
			/*
			 * TX completions re-fill the UART TX FIFO.  Doing
			 * that from the tasklet leaves a gap after every slot
			 * and caps sustained throughput well below the link
			 * rate; arm the next slot directly from the IRQ.
			 */
			dmaengine_desc_get_callback(&desc->vd.tx, &cb);
		}
		if (desc->direction != DMA_MEM_TO_MEM)
			dma_cookie_complete(&desc->vd.tx);
		if (desc->direction == DMA_MEM_TO_MEM) {
			vchan_cookie_complete(&desc->vd);
			desc = NULL;
		}
	}
	spin_unlock_irqrestore(&chan->vc.lock, flags);
	if (desc) {
		dmaengine_desc_callback_invoke(&cb, &desc->vd.tx_result);
		vchan_vdesc_fini(&desc->vd);
	}
	spin_lock_irqsave(&chan->vc.lock, flags);
	esp32s31_ahb_start_pending(chan);
	spin_unlock_irqrestore(&chan->vc.lock, flags);
	return IRQ_HANDLED;
}

static int esp32s31_ahb_slave_config(struct dma_chan *dchan,
				     struct dma_slave_config *config)
{
	struct esp32s31_ahb_chan *chan = to_ahb_chan(dchan);

	if (chan->request_id >= 32)
		return -EINVAL;
	memcpy(&chan->config, config, sizeof(*config));
	return 0;
}

static void esp32s31_ahb_issue_pending(struct dma_chan *dchan)
{
	struct esp32s31_ahb_chan *chan = to_ahb_chan(dchan);
	struct virt_dma_desc *vd;
	unsigned long flags;

	spin_lock_irqsave(&chan->vc.lock, flags);
	list_for_each_entry(vd, &chan->vc.desc_submitted, node) {
		struct esp32s31_ahb_desc *desc = to_ahb_desc(vd);

		if (desc->cyclic) {
			struct esp32s31_ahb_desc **cyclic =
				desc->direction == DMA_DEV_TO_MEM ?
				&chan->cyclic_rx : &chan->cyclic_tx;

			if (*cyclic)
				break;
			*cyclic = desc;
			chan->cyclic_generation++;
			if (desc->direction == DMA_MEM_TO_DEV)
				chan->tx_node = 0;
			else
				chan->rx_node = 0;
			list_del(&vd->node);
			esp32s31_ahb_start_cyclic(chan, desc);
			break;
		}
	}
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
	if (chan->cyclic_tx && chan->cyclic_tx->vd.tx.cookie == cookie) {
		size_t period = chan->cyclic_tx->len / chan->cyclic_tx->ndesc;

		dma_set_residue(state, chan->cyclic_tx->len -
				 chan->tx_node * period);
	}
	if (chan->cyclic_rx && chan->cyclic_rx->vd.tx.cookie == cookie) {
		size_t period = chan->cyclic_rx->len / chan->cyclic_rx->ndesc;

		dma_set_residue(state, chan->cyclic_rx->len -
				 chan->rx_node * period);
	}
	if (chan->active_tx && chan->active_tx->vd.tx.cookie == cookie)
		dma_set_residue(state, chan->active_tx->len);
	if (chan->active_rx && chan->active_rx->vd.tx.cookie == cookie)
		dma_set_residue(state, chan->active_rx->len);
	spin_unlock_irqrestore(&chan->vc.lock, flags);
	return status;
}

static int esp32s31_ahb_terminate_all(struct dma_chan *dchan)
{
	struct esp32s31_ahb_chan *chan = to_ahb_chan(dchan);
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&chan->vc.lock, flags);
	if (chan->cyclic_tx) {
		dev_info(chan->gdma->dev,
			 "cyclic-stop ch%u tx irqs=%u node=%u raw=%#x st=%#x link=%#x addr=%#x state=%#x dscr=%#x first=%#x\n",
			 chan->id, chan->tx_irqs, chan->tx_node,
			 readl(chan->gdma->base + AHB_TX_INT(chan->id) +
			       AHB_RX_RAW),
			 readl(chan->gdma->base + AHB_TX_INT(chan->id) +
			       AHB_RX_ST),
			 readl(ahb_ch_reg(chan, AHB_TX_LINK)),
			 readl(ahb_ch_reg(chan, AHB_TX_LINK_ADDR)),
			 readl(ahb_ch_reg(chan, AHB_TX_STATE)),
			 readl(ahb_ch_reg(chan, AHB_TX_DSCR)),
			 readl(chan->cyclic_tx->pool));
		esp32s31_ahb_reset(chan, DMA_MEM_TO_DEV);
		vchan_terminate_vdesc(&chan->cyclic_tx->vd);
		chan->cyclic_tx = NULL;
		chan->cyclic_generation++;
		chan->tx_node = 0;
	}
	if (chan->cyclic_rx) {
		dev_info(chan->gdma->dev,
			 "cyclic-stop ch%u rx irqs=%u node=%u raw=%#x st=%#x link=%#x addr=%#x state=%#x eof=%#x first=%#x\n",
			 chan->id, chan->rx_irqs, chan->rx_node,
			 readl(chan->gdma->base + AHB_RX_INT(chan->id) +
			       AHB_RX_RAW),
			 readl(chan->gdma->base + AHB_RX_INT(chan->id) +
			       AHB_RX_ST),
			 readl(ahb_ch_reg(chan, AHB_RX_LINK)),
			 readl(ahb_ch_reg(chan, AHB_RX_LINK_ADDR)),
			 readl(ahb_ch_reg(chan, AHB_RX_STATE)),
			 readl(ahb_ch_reg(chan, AHB_RX_SUC_EOF_DESC)),
			 readl(chan->cyclic_rx->pool));
		esp32s31_ahb_reset(chan, DMA_DEV_TO_MEM);
		vchan_terminate_vdesc(&chan->cyclic_rx->vd);
		chan->cyclic_rx = NULL;
		chan->cyclic_generation++;
		chan->rx_node = 0;
	}
	if (chan->active_tx || chan->active_rx)
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
			readl(chan->active_tx ? chan->active_tx->pool : chan->active_rx->pool),
			readl(chan->active_rx ? chan->active_rx->pool : chan->active_tx->pool));
	if (chan->active_rx)
		esp32s31_ahb_reset(chan, DMA_DEV_TO_MEM);
	if (chan->active_tx)
		esp32s31_ahb_reset(chan, DMA_MEM_TO_DEV);
	if (chan->active_tx && chan->active_tx == chan->active_rx) {
		vchan_terminate_vdesc(&chan->active_tx->vd);
		chan->active_tx = chan->active_rx = NULL;
	} else {
		if (chan->active_tx) {
			vchan_terminate_vdesc(&chan->active_tx->vd);
			chan->active_tx = NULL;
		}
		if (chan->active_rx) {
			vchan_terminate_vdesc(&chan->active_rx->vd);
			chan->active_rx = NULL;
		}
	}
	vchan_get_all_descriptors(&chan->vc, &head);
	spin_unlock_irqrestore(&chan->vc.lock, flags);
	vchan_dma_desc_free_list(&chan->vc, &head);
	return 0;
}

int esp32s31_ahb_terminate_direction(struct dma_chan *dchan,
				     enum dma_transfer_direction direction)
{
	struct esp32s31_ahb_chan *chan = to_ahb_chan(dchan);
	struct virt_dma_desc *vd, *next;
	struct esp32s31_ahb_desc *desc;
	unsigned long flags;
	LIST_HEAD(head);

	if (direction != DMA_MEM_TO_DEV && direction != DMA_DEV_TO_MEM)
		return -EINVAL;

	spin_lock_irqsave(&chan->vc.lock, flags);
	if (direction == DMA_MEM_TO_DEV && chan->cyclic_tx) {
		esp32s31_ahb_reset(chan, direction);
		list_add_tail(&chan->cyclic_tx->vd.node, &head);
		chan->cyclic_tx = NULL;
		chan->cyclic_generation++;
		chan->tx_node = 0;
	}
	if (direction == DMA_DEV_TO_MEM && chan->cyclic_rx) {
		esp32s31_ahb_reset(chan, direction);
		list_add_tail(&chan->cyclic_rx->vd.node, &head);
		chan->cyclic_rx = NULL;
		chan->cyclic_generation++;
		chan->rx_node = 0;
	}
	if (direction == DMA_MEM_TO_DEV && chan->active_tx &&
	    chan->active_tx->direction == direction) {
		esp32s31_ahb_reset(chan, direction);
		list_add_tail(&chan->active_tx->vd.node, &head);
		chan->active_tx = NULL;
	} else if (direction == DMA_DEV_TO_MEM && chan->active_rx &&
		   chan->active_rx->direction == direction) {
		esp32s31_ahb_reset(chan, direction);
		list_add_tail(&chan->active_rx->vd.node, &head);
		chan->active_rx = NULL;
	}

	list_for_each_entry_safe(vd, next, &chan->vc.desc_submitted, node) {
		desc = to_ahb_desc(vd);
		if (desc->direction == direction)
			list_move_tail(&vd->node, &head);
	}
	list_for_each_entry_safe(vd, next, &chan->vc.desc_issued, node) {
		desc = to_ahb_desc(vd);
		if (desc->direction == direction)
			list_move_tail(&vd->node, &head);
	}
	spin_unlock_irqrestore(&chan->vc.lock, flags);

	vchan_dma_desc_free_list(&chan->vc, &head);
	return 0;
}
EXPORT_SYMBOL_GPL(esp32s31_ahb_terminate_direction);

static void esp32s31_ahb_synchronize(struct dma_chan *dchan)
{
	struct esp32s31_ahb_chan *chan = to_ahb_chan(dchan);

	synchronize_irq(chan->rx_irq);
	synchronize_irq(chan->tx_irq);
	vchan_synchronize(&chan->vc);
}

static void esp32s31_ahb_free_resources(struct dma_chan *dchan)
{
	struct esp32s31_ahb_chan *chan = to_ahb_chan(dchan);

	esp32s31_ahb_terminate_all(dchan);
	vchan_free_chan_resources(&chan->vc);
	chan->request_id = U32_MAX;
	chan->users = 0;
}

static struct dma_chan *esp32s31_ahb_of_xlate(struct of_phandle_args *spec,
					       struct of_dma *ofdma)
{
	struct esp32s31_ahb *gdma = ofdma->of_dma_data;
	struct esp32s31_ahb_chan *chan;

	if (spec->args_count != 2 || spec->args[0] >= AHB_CHANNELS ||
	    spec->args[1] >= 32)
		return NULL;
	chan = &gdma->chans[spec->args[0]];
	if (chan->users >= 2)
		return NULL;
	chan->request_id = spec->args[1];
	chan->users++;
	return dma_get_slave_channel(&chan->vc.chan);
}

static int esp32s31_ahb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp32s31_ahb *gdma;
	struct dma_device *dma_dev;
	struct reset_control *rst;
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
	rst = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "reset unavailable\n");
	ret = reset_control_reset(rst);
	if (ret)
		return dev_err_probe(dev, ret, "reset failed\n");
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
	dma_cap_set(DMA_SLAVE, dma_dev->cap_mask);
	dma_cap_set(DMA_CYCLIC, dma_dev->cap_mask);
	dma_dev->copy_align = DMAENGINE_ALIGN_4_BYTES;
	dma_dev->directions = BIT(DMA_MEM_TO_MEM) | BIT(DMA_MEM_TO_DEV) |
			       BIT(DMA_DEV_TO_MEM);
	dma_dev->src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |
		BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
		BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	dma_dev->dst_addr_widths = dma_dev->src_addr_widths;
	dma_dev->residue_granularity = DMA_RESIDUE_GRANULARITY_DESCRIPTOR;
	dma_dev->device_free_chan_resources = esp32s31_ahb_free_resources;
	dma_dev->device_prep_dma_memcpy = esp32s31_ahb_prep_memcpy;
	dma_dev->device_config = esp32s31_ahb_slave_config;
	dma_dev->device_prep_slave_sg = esp32s31_ahb_prep_slave_sg;
	dma_dev->device_prep_dma_cyclic = esp32s31_ahb_prep_dma_cyclic;
	dma_dev->device_issue_pending = esp32s31_ahb_issue_pending;
	dma_dev->device_tx_status = esp32s31_ahb_tx_status;
	dma_dev->device_terminate_all = esp32s31_ahb_terminate_all;
	dma_dev->device_synchronize = esp32s31_ahb_synchronize;

	for (i = 0; i < AHB_CHANNELS; i++) {
		struct esp32s31_ahb_chan *chan = &gdma->chans[i];
		int irq = platform_get_irq(pdev, i);

		if (irq < 0)
			return irq;
		chan->gdma = gdma;
		chan->id = i;
		chan->request_id = U32_MAX;
		chan->rx_irq = irq;
		chan->vc.desc_free = esp32s31_ahb_free_desc;
		vchan_init(&chan->vc, dma_dev);
		ret = devm_request_irq(dev, irq, esp32s31_ahb_irq, 0,
				       dev_name(dev), chan);
		if (ret)
			return ret;
		chan->tx_irq = platform_get_irq(pdev, AHB_CHANNELS + i);
		if (chan->tx_irq < 0)
			return chan->tx_irq;
		ret = devm_request_irq(dev, chan->tx_irq, esp32s31_ahb_irq, 0,
				       dev_name(dev), chan);
		if (ret)
			return ret;
	}
	ret = dma_async_device_register(dma_dev);
	if (ret)
		return ret;
	ret = of_dma_controller_register(dev->of_node, esp32s31_ahb_of_xlate,
					 gdma);
	if (ret) {
		dma_async_device_unregister(dma_dev);
		return ret;
	}
	platform_set_drvdata(pdev, gdma);
	dev_info(dev, "AHB GDMA ready: 5 memcpy/slave pairs, version %#x\n",
		 readl(gdma->base + AHB_DATE));
	return 0;
}

static void esp32s31_ahb_remove(struct platform_device *pdev)
{
	struct esp32s31_ahb *gdma = platform_get_drvdata(pdev);
	unsigned int i;

	dma_async_device_unregister(&gdma->dma_dev);
	of_dma_controller_free(pdev->dev.of_node);
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
