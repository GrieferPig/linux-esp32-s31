// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif ESP32-S31 AXI GDMA dmaengine driver
 *
 * Hardware reference: ESP-IDF esp32s31 AXI GDMA HAL/LL.  The block contains
 * three coupled TX/RX pairs.  A Linux channel represents one pair and can run
 * one of memory-to-memory, memory-to-device or device-to-memory at a time.
 */

#include <linux/bitfield.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/genalloc.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>

#include "virt-dma.h"

#define ESP32S31_AXI_GDMA_CHANNELS	3
#define ESP32S31_AXI_GDMA_DESC_MAX	4095U
#define ESP32S31_AXI_GDMA_DESC_ALIGNED_MAX 4032U
#define ESP32S31_AXI_GDMA_BURST_SIZE	128U
#define ESP32S31_AXI_GDMA_CACHE_LINE	64U
#define ESP32S31_AXI_GDMA_PATTERN_SIZE	512U
#define ESP32S31_AXI_GDMA_PERIPH_MAX	5U

/* Cached and direct (uncached) views of the 16-MiB MSPI PSRAM window. */
#define ESP32S31_PSRAM_CACHED_BASE	0x50000000U
#define ESP32S31_PSRAM_DIRECT_BASE	0xc0000000U
#define ESP32S31_PSRAM_SIZE		0x01000000U

#define ESP32S31_AXI_RX_BASE(ch)		((ch) * 0x68)
#define ESP32S31_AXI_TX_BASE(ch)		(0x138 + (ch) * 0x68)

#define ESP32S31_AXI_INT_RAW		0x00
#define ESP32S31_AXI_INT_ST		0x04
#define ESP32S31_AXI_INT_ENA		0x08
#define ESP32S31_AXI_INT_CLR		0x0c

#define ESP32S31_AXI_RX_CONF0		0x10
#define ESP32S31_AXI_RX_CONF1		0x14
#define ESP32S31_AXI_RX_LINK1		0x20
#define ESP32S31_AXI_RX_LINK2		0x24
#define ESP32S31_AXI_RX_SUC_EOF_DESC	0x2c
#define ESP32S31_AXI_RX_DSCR		0x34
#define ESP32S31_AXI_RX_DSCR_BF0		0x38
#define ESP32S31_AXI_RX_PERI_SEL		0x44

#define ESP32S31_AXI_TX_CONF0		0x10
#define ESP32S31_AXI_TX_CONF1		0x14
#define ESP32S31_AXI_TX_LINK1		0x20
#define ESP32S31_AXI_TX_LINK2		0x24
#define ESP32S31_AXI_TX_EOF_DESC		0x2c
#define ESP32S31_AXI_TX_DSCR		0x34
#define ESP32S31_AXI_TX_DSCR_BF0		0x38
#define ESP32S31_AXI_TX_PERI_SEL		0x44

#define ESP32S31_AXI_RX_RST		BIT(0)
#define ESP32S31_AXI_RX_MEM_TRANS_EN	BIT(2)
#define ESP32S31_AXI_RX_BURST_SIZE	GENMASK(6, 4)
#define ESP32S31_AXI_RX_DSCR_BURST_EN	BIT(9)
#define ESP32S31_AXI_RX_CHECK_OWNER	BIT(12)
#define ESP32S31_AXI_RX_LINK_STOP	BIT(1)
#define ESP32S31_AXI_RX_LINK_START	BIT(2)
#define ESP32S31_AXI_RX_LINK_RESTART	BIT(3)

#define ESP32S31_AXI_TX_RST		BIT(0)
#define ESP32S31_AXI_TX_AUTO_WRBACK	BIT(2)
#define ESP32S31_AXI_TX_EOF_MODE		BIT(3)
#define ESP32S31_AXI_TX_BURST_SIZE	GENMASK(7, 5)
#define ESP32S31_AXI_TX_DSCR_BURST_EN	BIT(10)
#define ESP32S31_AXI_TX_CHECK_OWNER	BIT(12)
#define ESP32S31_AXI_TX_LINK_STOP	BIT(0)
#define ESP32S31_AXI_TX_LINK_START	BIT(1)
#define ESP32S31_AXI_TX_LINK_RESTART	BIT(2)

#define ESP32S31_AXI_RX_DONE		BIT(0)
#define ESP32S31_AXI_RX_SUC_EOF		BIT(1)
#define ESP32S31_AXI_RX_ERR_EOF		BIT(2)
#define ESP32S31_AXI_RX_DSCR_ERR		BIT(3)
#define ESP32S31_AXI_RX_DSCR_EMPTY	BIT(4)
#define ESP32S31_AXI_RX_ERROR_MASK	(ESP32S31_AXI_RX_ERR_EOF | \
					 ESP32S31_AXI_RX_DSCR_ERR | \
					 ESP32S31_AXI_RX_DSCR_EMPTY)

#define ESP32S31_AXI_TX_EOF		BIT(1)
#define ESP32S31_AXI_TX_DSCR_ERR		BIT(2)

#define ESP32S31_AXI_INTR_MEM_START	0x27c
#define ESP32S31_AXI_INTR_MEM_END	0x280
#define ESP32S31_AXI_EXTR_MEM_START	0x284
#define ESP32S31_AXI_EXTR_MEM_END	0x288
#define ESP32S31_AXI_MISC_CONF		0x2a8
#define ESP32S31_AXI_DATE		0x2d8

#define ESP32S31_AXI_MISC_RST_WR	BIT(0)
#define ESP32S31_AXI_MISC_RST_RD	BIT(1)
#define ESP32S31_AXI_MISC_CLK_EN	BIT(4)
#define ESP32S31_AXI_CLK_EN		BIT(0)
#define ESP32S31_AXI_RESET		BIT(1)

#define ESP32S31_DESC_SIZE		GENMASK(11, 0)
#define ESP32S31_DESC_LENGTH		GENMASK(23, 12)
#define ESP32S31_DESC_SUC_EOF		BIT(30)
#define ESP32S31_DESC_OWNER_DMA		BIT(31)

struct esp32s31_axi_hw_desc {
	__le32 control;
	__le32 buffer;
	__le32 next;
	__le32 reserved;
} __aligned(8);

struct esp32s31_axi_seg {
	dma_addr_t addr;
	dma_addr_t recovery;
	size_t len;
	unsigned int period;
	bool eof;
	bool stash;
};

struct esp32s31_axi_desc_node {
	void __iomem *hw;
	dma_addr_t dma;
	size_t len;
	unsigned int period;
	bool period_end;
};

struct esp32s31_axi_stash {
	void __iomem *hw;
	dma_addr_t dma;
	dma_addr_t recovery;
	size_t len;
	unsigned int period;
};

struct esp32s31_axi_desc {
	struct virt_dma_desc vd;
	struct list_head running_node;
	enum dma_transfer_direction direction;
	size_t len;
	size_t residue;
	size_t period_len;
	unsigned int periods;
	unsigned int current_period;
	unsigned int cyclic_node;
	unsigned int tx_count;
	unsigned int rx_count;
	unsigned int stash_count;
	bool cyclic;
	void __iomem *pool_base;
	size_t pool_len;
	struct esp32s31_axi_desc_node *tx;
	struct esp32s31_axi_desc_node *rx;
	struct esp32s31_axi_stash *stash;
};

struct esp32s31_axi_gdma;

struct esp32s31_axi_chan {
	struct virt_dma_chan vc;
	struct esp32s31_axi_gdma *gdma;
	struct list_head running;
	struct dma_slave_config config;
	dma_cookie_t error_cookie;
	u32 request_id;
	unsigned int id;
	int rx_irq;
	int tx_irq;
};

struct esp32s31_axi_gdma {
	struct dma_device dma_dev;
	struct device *dev;
	void __iomem *base;
	void __iomem *clkrst;
	void __iomem *desc_base;
	void __iomem *psram_direct;
	struct gen_pool *desc_pool;
	struct esp32s31_axi_chan chans[ESP32S31_AXI_GDMA_CHANNELS];
};

static inline struct esp32s31_axi_chan *to_esp32s31_axi_chan(struct dma_chan *chan)
{
	return container_of(chan, struct esp32s31_axi_chan, vc.chan);
}

static inline struct esp32s31_axi_desc *to_esp32s31_axi_desc(struct virt_dma_desc *vd)
{
	return container_of(vd, struct esp32s31_axi_desc, vd);
}

static inline void __iomem *esp32s31_axi_rx_reg(struct esp32s31_axi_chan *chan,
						unsigned int reg)
{
	return chan->gdma->base + ESP32S31_AXI_RX_BASE(chan->id) + reg;
}

static inline void __iomem *esp32s31_axi_tx_reg(struct esp32s31_axi_chan *chan,
						unsigned int reg)
{
	return chan->gdma->base + ESP32S31_AXI_TX_BASE(chan->id) + reg;
}

static bool esp32s31_axi_is_psram(dma_addr_t addr, size_t len)
{
	return addr >= ESP32S31_PSRAM_CACHED_BASE &&
	       len <= ESP32S31_PSRAM_SIZE &&
	       addr - ESP32S31_PSRAM_CACHED_BASE <= ESP32S31_PSRAM_SIZE - len;
}

static void esp32s31_axi_free_desc(struct virt_dma_desc *vd)
{
	struct esp32s31_axi_desc *desc = to_esp32s31_axi_desc(vd);
	struct esp32s31_axi_chan *chan = to_esp32s31_axi_chan(vd->tx.chan);

	if (desc->pool_base)
		gen_pool_free(chan->gdma->desc_pool,
			      (unsigned long)desc->pool_base, desc->pool_len);
	kfree(desc->stash);
	kfree(desc->tx);
	kfree(desc->rx);
	kfree(desc);
}

static void esp32s31_axi_reset_pair(struct esp32s31_axi_chan *chan)
{
	u32 val;

	writel(0, esp32s31_axi_rx_reg(chan, ESP32S31_AXI_INT_ENA));
	writel(0, esp32s31_axi_tx_reg(chan, ESP32S31_AXI_INT_ENA));
	writel(GENMASK(10, 0), esp32s31_axi_rx_reg(chan, ESP32S31_AXI_INT_CLR));
	writel(GENMASK(10, 0), esp32s31_axi_tx_reg(chan, ESP32S31_AXI_INT_CLR));

	val = readl(esp32s31_axi_rx_reg(chan, ESP32S31_AXI_RX_CONF0));
	writel(val | ESP32S31_AXI_RX_RST,
	       esp32s31_axi_rx_reg(chan, ESP32S31_AXI_RX_CONF0));
	writel(val & ~ESP32S31_AXI_RX_RST,
	       esp32s31_axi_rx_reg(chan, ESP32S31_AXI_RX_CONF0));
	val = readl(esp32s31_axi_tx_reg(chan, ESP32S31_AXI_TX_CONF0));
	writel(val | ESP32S31_AXI_TX_RST,
	       esp32s31_axi_tx_reg(chan, ESP32S31_AXI_TX_CONF0));
	writel(val & ~ESP32S31_AXI_TX_RST,
	       esp32s31_axi_tx_reg(chan, ESP32S31_AXI_TX_CONF0));
}

static void esp32s31_axi_config_pair(struct esp32s31_axi_chan *chan,
				     enum dma_transfer_direction direction)
{
	u32 periph = direction == DMA_MEM_TO_MEM ? 6 + chan->id : chan->request_id;
	u32 burst_sel = __ffs(ESP32S31_AXI_GDMA_BURST_SIZE) - 3;
	u32 rx_conf = FIELD_PREP(ESP32S31_AXI_RX_BURST_SIZE, burst_sel) |
		      ESP32S31_AXI_RX_DSCR_BURST_EN;

	if (direction == DMA_MEM_TO_MEM)
		rx_conf |= ESP32S31_AXI_RX_MEM_TRANS_EN;

	writel(rx_conf, esp32s31_axi_rx_reg(chan, ESP32S31_AXI_RX_CONF0));
	writel(ESP32S31_AXI_RX_CHECK_OWNER,
	       esp32s31_axi_rx_reg(chan, ESP32S31_AXI_RX_CONF1));
	writel(periph, esp32s31_axi_rx_reg(chan, ESP32S31_AXI_RX_PERI_SEL));
	writel(ESP32S31_AXI_TX_AUTO_WRBACK | ESP32S31_AXI_TX_EOF_MODE |
	       FIELD_PREP(ESP32S31_AXI_TX_BURST_SIZE, burst_sel) |
	       ESP32S31_AXI_TX_DSCR_BURST_EN,
	       esp32s31_axi_tx_reg(chan, ESP32S31_AXI_TX_CONF0));
	writel(ESP32S31_AXI_TX_CHECK_OWNER,
	       esp32s31_axi_tx_reg(chan, ESP32S31_AXI_TX_CONF1));
	writel(periph, esp32s31_axi_tx_reg(chan, ESP32S31_AXI_TX_PERI_SEL));
}

static void esp32s31_axi_stop_pair(struct esp32s31_axi_chan *chan)
{
	writel(ESP32S31_AXI_RX_LINK_STOP,
	       esp32s31_axi_rx_reg(chan, ESP32S31_AXI_RX_LINK1));
	writel(ESP32S31_AXI_TX_LINK_STOP,
	       esp32s31_axi_tx_reg(chan, ESP32S31_AXI_TX_LINK1));
	esp32s31_axi_reset_pair(chan);
}

static u32 esp32s31_axi_ack(struct esp32s31_axi_chan *chan, bool tx)
{
	void __iomem *raw = tx ? esp32s31_axi_tx_reg(chan, ESP32S31_AXI_INT_RAW) :
				 esp32s31_axi_rx_reg(chan, ESP32S31_AXI_INT_RAW);
	void __iomem *clr = tx ? esp32s31_axi_tx_reg(chan, ESP32S31_AXI_INT_CLR) :
				 esp32s31_axi_rx_reg(chan, ESP32S31_AXI_INT_CLR);
	void __iomem *st = tx ? esp32s31_axi_tx_reg(chan, ESP32S31_AXI_INT_ST) :
				esp32s31_axi_rx_reg(chan, ESP32S31_AXI_INT_ST);
	u32 pending, status = 0;
	unsigned int i;

	for (i = 0; i < 4; i++) {
		pending = readl(raw);
		if (!pending)
			break;
		status |= pending;
		writel(pending, clr);
	}
	readl(st);
	return status;
}

static int esp32s31_axi_build_linear(struct esp32s31_axi_seg *segs,
				      unsigned int capacity, dma_addr_t addr,
				      size_t len, unsigned int period,
				      bool rx, unsigned int *used)
{
	unsigned int n = *used;

	if (upper_32_bits(addr) || !len)
		return -EINVAL;

	if (rx && esp32s31_axi_is_psram(addr, len)) {
		size_t head = addr & (ESP32S31_AXI_GDMA_CACHE_LINE - 1);

		if (head) {
			head = min_t(size_t, len,
					     ESP32S31_AXI_GDMA_CACHE_LINE - head);
			if (n == capacity)
				return -ENOSPC;
			segs[n++] = (struct esp32s31_axi_seg) {
				.recovery = addr, .len = head, .period = period,
				.stash = true,
			};
			addr += head;
			len -= head;
		}
	}

	while (len >= ESP32S31_AXI_GDMA_CACHE_LINE) {
		size_t chunk = min_t(size_t, len & ~(ESP32S31_AXI_GDMA_CACHE_LINE - 1),
				     ESP32S31_AXI_GDMA_DESC_ALIGNED_MAX);

		if (!chunk)
			break;
		if (n == capacity)
			return -ENOSPC;
		segs[n++] = (struct esp32s31_axi_seg) {
			.addr = addr, .len = chunk, .period = period,
		};
		addr += chunk;
		len -= chunk;
	}

	while (len) {
		size_t chunk = min_t(size_t, len, ESP32S31_AXI_GDMA_DESC_MAX);
		bool stash = rx && esp32s31_axi_is_psram(addr, chunk) &&
			     (chunk < ESP32S31_AXI_GDMA_CACHE_LINE ||
			      (addr & (ESP32S31_AXI_GDMA_CACHE_LINE - 1)));

		if (n == capacity)
			return -ENOSPC;
		segs[n++] = (struct esp32s31_axi_seg) {
			.addr = stash ? 0 : addr,
			.recovery = stash ? addr : 0,
			.len = chunk, .period = period, .stash = stash,
		};
		addr += chunk;
		len -= chunk;
	}
	*used = n;
	return 0;
}

static struct esp32s31_axi_desc *
esp32s31_axi_alloc_desc(struct esp32s31_axi_chan *chan,
			struct esp32s31_axi_seg *tx_seg, unsigned int tx_count,
			struct esp32s31_axi_seg *rx_seg, unsigned int rx_count,
			size_t pattern_len)
{
	struct esp32s31_axi_desc *desc;
	unsigned int i, stash_count = 0, stash_index = 0;
	unsigned int node_count = tx_count + rx_count;
	unsigned long pool;
	unsigned long cursor;
	size_t pool_len;
	dma_addr_t pattern_dma = 0;

	for (i = 0; i < rx_count; i++)
		stash_count += rx_seg[i].stash;

	desc = kzalloc(sizeof(*desc), GFP_NOWAIT);
	if (!desc)
		return NULL;
	INIT_LIST_HEAD(&desc->running_node);
	desc->tx = kcalloc(tx_count, sizeof(*desc->tx), GFP_NOWAIT);
	desc->rx = kcalloc(rx_count, sizeof(*desc->rx), GFP_NOWAIT);
	desc->stash = kcalloc(stash_count, sizeof(*desc->stash), GFP_NOWAIT);
	if ((tx_count && !desc->tx) || (rx_count && !desc->rx) ||
	    (stash_count && !desc->stash))
		goto err;

	/* Padding permits cache-line aligned stash/pattern buffers in one allocation. */
	pool_len = node_count * sizeof(struct esp32s31_axi_hw_desc) +
		   stash_count * ESP32S31_AXI_GDMA_CACHE_LINE + pattern_len + 127;
	pool = gen_pool_alloc(chan->gdma->desc_pool, pool_len);
	if (!pool)
		goto err;
	desc->pool_base = (void __iomem *)pool;
	desc->pool_len = pool_len;
	desc->tx_count = tx_count;
	desc->rx_count = rx_count;
	desc->stash_count = stash_count;
	memset_io(desc->pool_base, 0, pool_len);

	for (i = 0; i < tx_count; i++) {
		desc->tx[i].hw = desc->pool_base + i * sizeof(struct esp32s31_axi_hw_desc);
		desc->tx[i].dma = gen_pool_virt_to_phys(chan->gdma->desc_pool,
							(unsigned long)desc->tx[i].hw);
	}
	for (i = 0; i < rx_count; i++) {
		desc->rx[i].hw = desc->pool_base + (tx_count + i) *
						 sizeof(struct esp32s31_axi_hw_desc);
		desc->rx[i].dma = gen_pool_virt_to_phys(chan->gdma->desc_pool,
							(unsigned long)desc->rx[i].hw);
	}

	cursor = ALIGN(pool + node_count * sizeof(struct esp32s31_axi_hw_desc),
		       ESP32S31_AXI_GDMA_CACHE_LINE);
	for (i = 0; i < stash_count; i++) {
		desc->stash[i].hw = (void __iomem *)cursor;
		desc->stash[i].dma = gen_pool_virt_to_phys(chan->gdma->desc_pool, cursor);
		cursor += ESP32S31_AXI_GDMA_CACHE_LINE;
	}
	if (pattern_len) {
		cursor = ALIGN(cursor, ESP32S31_AXI_GDMA_BURST_SIZE);
		pattern_dma = gen_pool_virt_to_phys(chan->gdma->desc_pool, cursor);
	}

	for (i = 0; i < tx_count; i++) {
		u32 control;
		dma_addr_t addr = tx_seg[i].addr ? tx_seg[i].addr : pattern_dma;

		control = FIELD_PREP(ESP32S31_DESC_SIZE, tx_seg[i].len) |
			  FIELD_PREP(ESP32S31_DESC_LENGTH, tx_seg[i].len) |
			  ESP32S31_DESC_OWNER_DMA;
		if (tx_seg[i].eof)
			control |= ESP32S31_DESC_SUC_EOF;
		writel(control, desc->tx[i].hw);
		writel(lower_32_bits(addr), desc->tx[i].hw + 4);
		writel(i + 1 == tx_count ? 0 : lower_32_bits(desc->tx[i + 1].dma),
		       desc->tx[i].hw + 8);
		desc->tx[i].len = tx_seg[i].len;
		desc->tx[i].period = tx_seg[i].period;
		desc->tx[i].period_end = tx_seg[i].eof;
	}

	for (i = 0; i < rx_count; i++) {
		u32 control = FIELD_PREP(ESP32S31_DESC_SIZE, rx_seg[i].len) |
			      FIELD_PREP(ESP32S31_DESC_LENGTH, rx_seg[i].len) |
			      ESP32S31_DESC_OWNER_DMA;
		dma_addr_t addr = rx_seg[i].addr;

		if (rx_seg[i].stash) {
			struct esp32s31_axi_stash *stash = &desc->stash[stash_index++];

			addr = stash->dma;
			stash->recovery = rx_seg[i].recovery;
			stash->len = rx_seg[i].len;
			stash->period = rx_seg[i].period;
		}
		writel(control, desc->rx[i].hw);
		writel(lower_32_bits(addr), desc->rx[i].hw + 4);
		writel(i + 1 == rx_count ? 0 : lower_32_bits(desc->rx[i + 1].dma),
		       desc->rx[i].hw + 8);
		desc->rx[i].len = rx_seg[i].len;
		desc->rx[i].period = rx_seg[i].period;
		desc->rx[i].period_end = rx_seg[i].eof;
	}

	return desc;
err:
	kfree(desc->stash);
	kfree(desc->tx);
	kfree(desc->rx);
	kfree(desc);
	return NULL;
}

static void __iomem *esp32s31_axi_pattern(struct esp32s31_axi_desc *desc)
{
	unsigned long cursor;
	unsigned int stash_count = desc->stash_count;

	cursor = (unsigned long)desc->pool_base +
		 (desc->tx_count + desc->rx_count) * sizeof(struct esp32s31_axi_hw_desc);
	cursor = ALIGN(cursor, ESP32S31_AXI_GDMA_CACHE_LINE);
	cursor += stash_count * ESP32S31_AXI_GDMA_CACHE_LINE;
	return (void __iomem *)ALIGN(cursor, ESP32S31_AXI_GDMA_BURST_SIZE);
}

static void esp32s31_axi_recover_stash(struct esp32s31_axi_chan *chan,
				       struct esp32s31_axi_desc *desc,
				       int period)
{
	u8 data[ESP32S31_AXI_GDMA_CACHE_LINE];
	unsigned int i, count = desc->stash_count;

	for (i = 0; i < count; i++) {
		struct esp32s31_axi_stash *stash = &desc->stash[i];
		void __iomem *dst;

		if (period >= 0 && stash->period != period)
			continue;
		if (!esp32s31_axi_is_psram(stash->recovery, stash->len))
			continue;
		dst = chan->gdma->psram_direct +
		      (stash->recovery - ESP32S31_PSRAM_CACHED_BASE);
		memcpy_fromio(data, stash->hw, stash->len);
		memcpy_toio(dst, data, stash->len);
	}
	/* Publish uncached alias recovery writes before signaling completion. */
	wmb();
}

static bool esp32s31_axi_compatible(struct esp32s31_axi_desc *a,
				    struct esp32s31_axi_desc *b)
{
	return !a->cyclic && !b->cyclic && a->direction == b->direction;
}

static void esp32s31_axi_start_desc(struct esp32s31_axi_chan *chan,
				    struct esp32s31_axi_desc *desc)
{
	u32 rx_mask = ESP32S31_AXI_RX_ERROR_MASK;
	u32 tx_mask = ESP32S31_AXI_TX_DSCR_ERR;

	esp32s31_axi_reset_pair(chan);
	esp32s31_axi_config_pair(chan, desc->direction);
	if (desc->direction != DMA_MEM_TO_DEV)
		rx_mask |= ESP32S31_AXI_RX_SUC_EOF;
	if (desc->cyclic && desc->direction == DMA_DEV_TO_MEM)
		rx_mask |= ESP32S31_AXI_RX_DONE;
	if (desc->direction == DMA_MEM_TO_DEV)
		tx_mask |= ESP32S31_AXI_TX_EOF;

	dma_wmb();
	writel(rx_mask, esp32s31_axi_rx_reg(chan, ESP32S31_AXI_INT_ENA));
	writel(tx_mask, esp32s31_axi_tx_reg(chan, ESP32S31_AXI_INT_ENA));
	if (desc->rx_count)
		writel(lower_32_bits(desc->rx[0].dma),
		       esp32s31_axi_rx_reg(chan, ESP32S31_AXI_RX_LINK2));
	if (desc->tx_count)
		writel(lower_32_bits(desc->tx[0].dma),
		       esp32s31_axi_tx_reg(chan, ESP32S31_AXI_TX_LINK2));
	if (desc->rx_count)
		writel(ESP32S31_AXI_RX_LINK_START,
		       esp32s31_axi_rx_reg(chan, ESP32S31_AXI_RX_LINK1));
	if (desc->tx_count)
		writel(ESP32S31_AXI_TX_LINK_START,
		       esp32s31_axi_tx_reg(chan, ESP32S31_AXI_TX_LINK1));
}

static void esp32s31_axi_append_desc(struct esp32s31_axi_chan *chan,
				     struct esp32s31_axi_desc *tail,
				     struct esp32s31_axi_desc *desc)
{
	if (tail->rx_count) {
		writel(lower_32_bits(desc->rx[0].dma),
		       tail->rx[tail->rx_count - 1].hw + 8);
	}
	if (tail->tx_count) {
		writel(lower_32_bits(desc->tx[0].dma),
		       tail->tx[tail->tx_count - 1].hw + 8);
	}
	dma_wmb();
	if (tail->rx_count)
		writel(ESP32S31_AXI_RX_LINK_RESTART,
		       esp32s31_axi_rx_reg(chan, ESP32S31_AXI_RX_LINK1));
	if (tail->tx_count)
		writel(ESP32S31_AXI_TX_LINK_RESTART,
		       esp32s31_axi_tx_reg(chan, ESP32S31_AXI_TX_LINK1));
}

static void esp32s31_axi_start_pending(struct esp32s31_axi_chan *chan)
{
	struct esp32s31_axi_desc *tail = NULL;
	struct virt_dma_desc *vd;

	lockdep_assert_held(&chan->vc.lock);
	if (!list_empty(&chan->running))
		tail = list_last_entry(&chan->running, struct esp32s31_axi_desc,
				       running_node);

	while ((vd = vchan_next_desc(&chan->vc))) {
		struct esp32s31_axi_desc *desc = to_esp32s31_axi_desc(vd);

		if (tail && !esp32s31_axi_compatible(tail, desc))
			break;
		list_del(&vd->node);
		list_add_tail(&desc->running_node, &chan->running);
		if (!tail)
			esp32s31_axi_start_desc(chan, desc);
		else
			esp32s31_axi_append_desc(chan, tail, desc);
		tail = desc;
		if (desc->cyclic)
			break;
	}
}

static void esp32s31_axi_complete_desc(struct esp32s31_axi_chan *chan,
				       struct esp32s31_axi_desc *desc,
				       enum dmaengine_tx_result result)
{
	list_del_init(&desc->running_node);
	desc->residue = 0;
	desc->vd.tx_result.result = result;
	desc->vd.tx_result.residue = 0;
	if (result != DMA_TRANS_NOERROR)
		chan->error_cookie = desc->vd.tx.cookie;
	else
		esp32s31_axi_recover_stash(chan, desc, -1);
	vchan_cookie_complete(&desc->vd);
}

static void esp32s31_axi_cyclic_advance(struct esp32s31_axi_chan *chan,
					struct esp32s31_axi_desc *desc,
					dma_addr_t eof, bool tx)
{
	struct esp32s31_axi_desc_node *nodes = tx ? desc->tx : desc->rx;
	unsigned int count = tx ? desc->tx_count : desc->rx_count;
	unsigned int end, pos, walked;

	for (end = 0; end < count; end++)
		if (nodes[end].dma == eof)
			break;
	if (end == count)
		return;

	pos = desc->cyclic_node;
	for (walked = 0; walked < count; walked++) {
		u32 control = readl(nodes[pos].hw);

		/* AUTO_WRBACK/RX completion returns ownership to the CPU. */
		if (!tx)
			control &= ~ESP32S31_DESC_SUC_EOF;
		writel(control | ESP32S31_DESC_OWNER_DMA, nodes[pos].hw);
		if (nodes[pos].period_end) {
			esp32s31_axi_recover_stash(chan, desc, nodes[pos].period);
			desc->current_period = (nodes[pos].period + 1) % desc->periods;
			desc->residue = desc->len -
					desc->current_period * desc->period_len;
			if (!desc->residue)
				desc->residue = desc->len;
			vchan_cyclic_callback(&desc->vd);
		}
		if (pos == end) {
			desc->cyclic_node = (pos + 1) % count;
			break;
		}
		pos = (pos + 1) % count;
	}
	dma_wmb();
	writel(tx ? ESP32S31_AXI_TX_LINK_RESTART : ESP32S31_AXI_RX_LINK_RESTART,
	       tx ? esp32s31_axi_tx_reg(chan, ESP32S31_AXI_TX_LINK1) :
		    esp32s31_axi_rx_reg(chan, ESP32S31_AXI_RX_LINK1));
}

static void esp32s31_axi_complete_eof(struct esp32s31_axi_chan *chan,
				      dma_addr_t eof, bool tx)
{
	struct esp32s31_axi_desc *desc, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&chan->vc.lock, flags);
	list_for_each_entry_safe(desc, tmp, &chan->running, running_node) {
		dma_addr_t tail = tx ? desc->tx[desc->tx_count - 1].dma :
					 desc->rx[desc->rx_count - 1].dma;

		if (desc->cyclic) {
			esp32s31_axi_cyclic_advance(chan, desc, eof, tx);
			break;
		}
		esp32s31_axi_complete_desc(chan, desc, DMA_TRANS_NOERROR);
		if (!eof || tail == eof)
			break;
	}
	esp32s31_axi_start_pending(chan);
	spin_unlock_irqrestore(&chan->vc.lock, flags);
}

static void esp32s31_axi_fail_running(struct esp32s31_axi_chan *chan,
				      enum dmaengine_tx_result result)
{
	struct esp32s31_axi_desc *desc, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&chan->vc.lock, flags);
	esp32s31_axi_stop_pair(chan);
	list_for_each_entry_safe(desc, tmp, &chan->running, running_node)
		esp32s31_axi_complete_desc(chan, desc, result);
	esp32s31_axi_start_pending(chan);
	spin_unlock_irqrestore(&chan->vc.lock, flags);
}

static irqreturn_t esp32s31_axi_rx_irq(int irq, void *data)
{
	struct esp32s31_axi_chan *chan = data;
	u32 status = esp32s31_axi_ack(chan, false);

	if (status & ESP32S31_AXI_RX_ERROR_MASK) {
		dev_err_ratelimited(chan->gdma->dev,
				    "channel %u RX error, status=%#x\n",
				    chan->id, status);
		esp32s31_axi_fail_running(chan, DMA_TRANS_WRITE_FAILED);
	} else if (status & ESP32S31_AXI_RX_SUC_EOF) {
		esp32s31_axi_complete_eof(chan,
			readl(esp32s31_axi_rx_reg(chan, ESP32S31_AXI_RX_SUC_EOF_DESC)),
			false);
	} else if (status & ESP32S31_AXI_RX_DONE) {
		/* RX cyclic peripherals may signal descriptor-done without frame EOF. */
		esp32s31_axi_complete_eof(chan,
			readl(esp32s31_axi_rx_reg(chan, ESP32S31_AXI_RX_DSCR_BF0)),
			false);
	}
	return IRQ_HANDLED;
}

static irqreturn_t esp32s31_axi_tx_irq(int irq, void *data)
{
	struct esp32s31_axi_chan *chan = data;
	u32 status = esp32s31_axi_ack(chan, true);

	if (status & ESP32S31_AXI_TX_DSCR_ERR) {
		dev_err_ratelimited(chan->gdma->dev,
				    "channel %u TX descriptor error, status=%#x\n",
				    chan->id, status);
		esp32s31_axi_fail_running(chan, DMA_TRANS_READ_FAILED);
	} else if (status & ESP32S31_AXI_TX_EOF) {
		esp32s31_axi_complete_eof(chan,
			readl(esp32s31_axi_tx_reg(chan, ESP32S31_AXI_TX_EOF_DESC)),
			true);
	}
	return IRQ_HANDLED;
}

static struct dma_async_tx_descriptor *
esp32s31_axi_prep_memcpy(struct dma_chan *dchan, dma_addr_t dst,
			 dma_addr_t src, size_t len, unsigned long flags)
{
	struct esp32s31_axi_chan *chan = to_esp32s31_axi_chan(dchan);
	struct esp32s31_axi_desc *desc;
	struct esp32s31_axi_seg *tx, *rx;
	unsigned int cap, tx_count = 0, rx_count = 0;

	if (!len || !IS_ALIGNED(src, 4) || !IS_ALIGNED(dst, 4) ||
	    !IS_ALIGNED(len, 4) || upper_32_bits(src) || upper_32_bits(dst))
		return NULL;
	cap = DIV_ROUND_UP(len, ESP32S31_AXI_GDMA_DESC_ALIGNED_MAX) + 3;
	tx = kcalloc(cap, sizeof(*tx), GFP_NOWAIT);
	rx = kcalloc(cap, sizeof(*rx), GFP_NOWAIT);
	if (!tx || !rx)
		goto out;
	if (esp32s31_axi_build_linear(tx, cap, src, len, 0, false, &tx_count) ||
	    esp32s31_axi_build_linear(rx, cap, dst, len, 0, true, &rx_count))
		goto out;
	tx[tx_count - 1].eof = true;
	rx[rx_count - 1].eof = true;
	desc = esp32s31_axi_alloc_desc(chan, tx, tx_count, rx, rx_count, 0);
	if (!desc)
		goto out;
	desc->direction = DMA_MEM_TO_MEM;
	desc->len = len;
	desc->residue = len;
	kfree(rx);
	kfree(tx);
	return vchan_tx_prep(&chan->vc, &desc->vd, flags);
out:
	kfree(rx);
	kfree(tx);
	return NULL;
}

static struct dma_async_tx_descriptor *
esp32s31_axi_prep_memset(struct dma_chan *dchan, dma_addr_t dst, int value,
			 size_t len, unsigned long flags)
{
	struct esp32s31_axi_chan *chan = to_esp32s31_axi_chan(dchan);
	struct esp32s31_axi_desc *desc;
	struct esp32s31_axi_seg *tx, *rx;
	unsigned int tx_cap, rx_cap, tx_count = 0, rx_count = 0, i;
	size_t remaining = len;

	if (!len || !IS_ALIGNED(dst, 4) || !IS_ALIGNED(len, 4) ||
	    upper_32_bits(dst))
		return NULL;
	tx_cap = DIV_ROUND_UP(len, ESP32S31_AXI_GDMA_PATTERN_SIZE);
	rx_cap = DIV_ROUND_UP(len, ESP32S31_AXI_GDMA_DESC_ALIGNED_MAX) + 3;
	tx = kcalloc(tx_cap, sizeof(*tx), GFP_NOWAIT);
	rx = kcalloc(rx_cap, sizeof(*rx), GFP_NOWAIT);
	if (!tx || !rx)
		goto out;
	for (i = 0; i < tx_cap; i++) {
		size_t chunk = min_t(size_t, remaining, ESP32S31_AXI_GDMA_PATTERN_SIZE);

		tx[tx_count++] = (struct esp32s31_axi_seg) { .len = chunk };
		remaining -= chunk;
	}
	tx[tx_count - 1].eof = true;
	if (esp32s31_axi_build_linear(rx, rx_cap, dst, len, 0, true, &rx_count))
		goto out;
	rx[rx_count - 1].eof = true;
	desc = esp32s31_axi_alloc_desc(chan, tx, tx_count, rx, rx_count,
					ESP32S31_AXI_GDMA_PATTERN_SIZE);
	if (!desc)
		goto out;
	memset_io(esp32s31_axi_pattern(desc), value,
		  ESP32S31_AXI_GDMA_PATTERN_SIZE);
	desc->direction = DMA_MEM_TO_MEM;
	desc->len = len;
	desc->residue = len;
	kfree(rx);
	kfree(tx);
	return vchan_tx_prep(&chan->vc, &desc->vd, flags);
out:
	kfree(rx);
	kfree(tx);
	return NULL;
}

static int esp32s31_axi_slave_config(struct dma_chan *dchan,
				     struct dma_slave_config *config)
{
	struct esp32s31_axi_chan *chan = to_esp32s31_axi_chan(dchan);

	if (chan->request_id > ESP32S31_AXI_GDMA_PERIPH_MAX)
		return -EINVAL;
	memcpy(&chan->config, config, sizeof(*config));
	return 0;
}

static struct dma_async_tx_descriptor *
esp32s31_axi_prep_slave_sg(struct dma_chan *dchan, struct scatterlist *sgl,
			   unsigned int sg_len,
			   enum dma_transfer_direction direction,
			   unsigned long flags, void *context)
{
	struct esp32s31_axi_chan *chan = to_esp32s31_axi_chan(dchan);
	struct esp32s31_axi_desc *desc;
	struct esp32s31_axi_seg *segs;
	struct scatterlist *sg;
	unsigned int i, cap = 2, count = 0;
	size_t len = 0;
	bool rx;

	if (!sg_len || (direction != DMA_MEM_TO_DEV && direction != DMA_DEV_TO_MEM) ||
	    chan->request_id > ESP32S31_AXI_GDMA_PERIPH_MAX)
		return NULL;
	rx = direction == DMA_DEV_TO_MEM;
	for_each_sg(sgl, sg, sg_len, i) {
		cap += DIV_ROUND_UP(sg_dma_len(sg),
				    ESP32S31_AXI_GDMA_DESC_ALIGNED_MAX) + 2;
		len += sg_dma_len(sg);
	}
	segs = kcalloc(cap, sizeof(*segs), GFP_NOWAIT);
	if (!segs)
		return NULL;
	for_each_sg(sgl, sg, sg_len, i) {
		if (esp32s31_axi_build_linear(segs, cap, sg_dma_address(sg),
					      sg_dma_len(sg), 0, rx, &count))
			goto out;
	}
	segs[count - 1].eof = true;
	desc = rx ? esp32s31_axi_alloc_desc(chan, NULL, 0, segs, count, 0) :
		    esp32s31_axi_alloc_desc(chan, segs, count, NULL, 0, 0);
	if (!desc)
		goto out;
	desc->direction = direction;
	desc->len = len;
	desc->residue = len;
	kfree(segs);
	return vchan_tx_prep(&chan->vc, &desc->vd, flags);
out:
	kfree(segs);
	return NULL;
}

static struct dma_async_tx_descriptor *
esp32s31_axi_prep_cyclic(struct dma_chan *dchan, dma_addr_t buf_addr,
			 size_t buf_len, size_t period_len,
			 enum dma_transfer_direction direction,
			 unsigned long flags)
{
	struct esp32s31_axi_chan *chan = to_esp32s31_axi_chan(dchan);
	struct esp32s31_axi_desc *desc;
	struct esp32s31_axi_seg *segs;
	unsigned int periods, cap, count = 0, i;
	bool rx;

	if (!buf_len || !period_len || buf_len % period_len ||
	    (direction != DMA_MEM_TO_DEV && direction != DMA_DEV_TO_MEM) ||
	    chan->request_id > ESP32S31_AXI_GDMA_PERIPH_MAX)
		return NULL;
	periods = buf_len / period_len;
	cap = periods * (DIV_ROUND_UP(period_len,
				    ESP32S31_AXI_GDMA_DESC_ALIGNED_MAX) + 3);
	segs = kcalloc(cap, sizeof(*segs), GFP_NOWAIT);
	if (!segs)
		return NULL;
	rx = direction == DMA_DEV_TO_MEM;
	for (i = 0; i < periods; i++) {
		if (esp32s31_axi_build_linear(segs, cap, buf_addr + i * period_len,
					      period_len, i, rx, &count))
			goto out;
		segs[count - 1].eof = true;
	}
	desc = rx ? esp32s31_axi_alloc_desc(chan, NULL, 0, segs, count, 0) :
		    esp32s31_axi_alloc_desc(chan, segs, count, NULL, 0, 0);
	if (!desc)
		goto out;
	if (rx)
		writel(lower_32_bits(desc->rx[0].dma), desc->rx[count - 1].hw + 8);
	else
		writel(lower_32_bits(desc->tx[0].dma), desc->tx[count - 1].hw + 8);
	desc->direction = direction;
	desc->len = buf_len;
	desc->residue = buf_len;
	desc->period_len = period_len;
	desc->periods = periods;
	desc->cyclic = true;
	kfree(segs);
	return vchan_tx_prep(&chan->vc, &desc->vd, flags);
out:
	kfree(segs);
	return NULL;
}

static void esp32s31_axi_issue_pending(struct dma_chan *dchan)
{
	struct esp32s31_axi_chan *chan = to_esp32s31_axi_chan(dchan);
	unsigned long flags;

	spin_lock_irqsave(&chan->vc.lock, flags);
	if (vchan_issue_pending(&chan->vc))
		esp32s31_axi_start_pending(chan);
	spin_unlock_irqrestore(&chan->vc.lock, flags);
}

static size_t esp32s31_axi_active_residue(struct esp32s31_axi_chan *chan,
					 struct esp32s31_axi_desc *desc)
{
	struct esp32s31_axi_desc_node *nodes;
	dma_addr_t current_desc;
	unsigned int count, i;
	size_t residue = 0;

	if (desc->cyclic)
		return desc->residue;
	if (desc->direction == DMA_MEM_TO_DEV) {
		nodes = desc->tx;
		count = desc->tx_count;
		current_desc = readl(esp32s31_axi_tx_reg(chan, ESP32S31_AXI_TX_DSCR));
	} else {
		nodes = desc->rx;
		count = desc->rx_count;
		current_desc = readl(esp32s31_axi_rx_reg(chan, ESP32S31_AXI_RX_DSCR));
	}
	for (i = 0; i < count; i++) {
		if (nodes[i].dma == current_desc)
			break;
	}
	if (i == count)
		return desc->residue;
	for (; i < count; i++)
		residue += nodes[i].len;
	return residue;
}

static enum dma_status esp32s31_axi_tx_status(struct dma_chan *dchan,
					      dma_cookie_t cookie,
					      struct dma_tx_state *state)
{
	struct esp32s31_axi_chan *chan = to_esp32s31_axi_chan(dchan);
	struct esp32s31_axi_desc *desc;
	struct virt_dma_desc *vd;
	enum dma_status status;
	unsigned long flags;

	status = dma_cookie_status(dchan, cookie, state);
	if (status == DMA_COMPLETE && cookie == READ_ONCE(chan->error_cookie))
		return DMA_ERROR;
	if (status == DMA_COMPLETE || !state)
		return status;

	spin_lock_irqsave(&chan->vc.lock, flags);
	list_for_each_entry(desc, &chan->running, running_node) {
		if (desc->vd.tx.cookie == cookie) {
			dma_set_residue(state, esp32s31_axi_active_residue(chan, desc));
			goto out;
		}
	}
	vd = vchan_find_desc(&chan->vc, cookie);
	if (vd)
		dma_set_residue(state, to_esp32s31_axi_desc(vd)->len);
out:
	spin_unlock_irqrestore(&chan->vc.lock, flags);
	return status;
}

static int esp32s31_axi_terminate_all(struct dma_chan *dchan)
{
	struct esp32s31_axi_chan *chan = to_esp32s31_axi_chan(dchan);
	struct esp32s31_axi_desc *desc, *tmp;
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&chan->vc.lock, flags);
	esp32s31_axi_stop_pair(chan);
	list_for_each_entry_safe(desc, tmp, &chan->running, running_node) {
		list_del_init(&desc->running_node);
		if (desc->cyclic)
			vchan_terminate_vdesc(&desc->vd);
		else
			list_add_tail(&desc->vd.node, &head);
	}
	vchan_get_all_descriptors(&chan->vc, &head);
	spin_unlock_irqrestore(&chan->vc.lock, flags);
	vchan_dma_desc_free_list(&chan->vc, &head);
	return 0;
}

static int esp32s31_axi_alloc_chan_resources(struct dma_chan *dchan)
{
	return 0;
}

static void esp32s31_axi_free_chan_resources(struct dma_chan *dchan)
{
	struct esp32s31_axi_chan *chan = to_esp32s31_axi_chan(dchan);

	esp32s31_axi_terminate_all(dchan);
	vchan_free_chan_resources(&chan->vc);
	chan->request_id = U32_MAX;
}

static void esp32s31_axi_synchronize(struct dma_chan *dchan)
{
	vchan_synchronize(&to_esp32s31_axi_chan(dchan)->vc);
}

static void esp32s31_axi_hw_init(struct esp32s31_axi_gdma *gdma)
{
	u32 val;
	unsigned int i;

	val = readl(gdma->clkrst);
	writel(val | ESP32S31_AXI_CLK_EN, gdma->clkrst);
	writel((val | ESP32S31_AXI_CLK_EN) | ESP32S31_AXI_RESET, gdma->clkrst);
	writel((val | ESP32S31_AXI_CLK_EN) & ~ESP32S31_AXI_RESET, gdma->clkrst);
	writel(0x2f000000, gdma->base + ESP32S31_AXI_INTR_MEM_START);
	writel(0x2f07ffff, gdma->base + ESP32S31_AXI_INTR_MEM_END);
	writel(0x40000000, gdma->base + ESP32S31_AXI_EXTR_MEM_START);
	writel(0x53ffffff, gdma->base + ESP32S31_AXI_EXTR_MEM_END);
	val = readl(gdma->base + ESP32S31_AXI_MISC_CONF) |
	      ESP32S31_AXI_MISC_CLK_EN;
	writel(val | ESP32S31_AXI_MISC_RST_RD, gdma->base + ESP32S31_AXI_MISC_CONF);
	writel(val & ~ESP32S31_AXI_MISC_RST_RD, gdma->base + ESP32S31_AXI_MISC_CONF);
	writel(val | ESP32S31_AXI_MISC_RST_WR, gdma->base + ESP32S31_AXI_MISC_CONF);
	writel(val & ~ESP32S31_AXI_MISC_RST_WR, gdma->base + ESP32S31_AXI_MISC_CONF);
	for (i = 0; i < ESP32S31_AXI_GDMA_CHANNELS; i++) {
		esp32s31_axi_reset_pair(&gdma->chans[i]);
		esp32s31_axi_config_pair(&gdma->chans[i], DMA_MEM_TO_MEM);
	}
}

static struct dma_chan *esp32s31_axi_of_xlate(struct of_phandle_args *spec,
					      struct of_dma *ofdma)
{
	struct esp32s31_axi_gdma *gdma = ofdma->of_dma_data;
	struct dma_chan *dchan;
	struct esp32s31_axi_chan *chan;

	if (spec->args_count != 2 || spec->args[0] >= ESP32S31_AXI_GDMA_CHANNELS ||
	    spec->args[1] > ESP32S31_AXI_GDMA_PERIPH_MAX)
		return NULL;
	chan = &gdma->chans[spec->args[0]];
	dchan = dma_get_slave_channel(&chan->vc.chan);
	if (dchan)
		chan->request_id = spec->args[1];
	return dchan;
}

static int esp32s31_axi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp32s31_axi_gdma *gdma;
	struct dma_device *dma_dev;
	struct resource *desc_res;
	unsigned int i;
	int ret;

	gdma = devm_kzalloc(dev, sizeof(*gdma), GFP_KERNEL);
	if (!gdma)
		return -ENOMEM;
	gdma->dev = dev;
	gdma->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gdma->base))
		return PTR_ERR(gdma->base);
	gdma->clkrst = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(gdma->clkrst))
		return PTR_ERR(gdma->clkrst);
	gdma->desc_base = devm_platform_get_and_ioremap_resource(pdev, 2, &desc_res);
	if (IS_ERR(gdma->desc_base))
		return PTR_ERR(gdma->desc_base);
	gdma->psram_direct = devm_ioremap(dev, ESP32S31_PSRAM_DIRECT_BASE,
					 ESP32S31_PSRAM_SIZE);
	if (!gdma->psram_direct)
		return -ENOMEM;
	gdma->desc_pool = devm_gen_pool_create(dev, 3, -1, NULL);
	if (IS_ERR(gdma->desc_pool))
		return PTR_ERR(gdma->desc_pool);
	ret = gen_pool_add_virt(gdma->desc_pool, (unsigned long)gdma->desc_base,
				 desc_res->start, resource_size(desc_res), -1);
	if (ret)
		return ret;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set DMA mask\n");

	dma_dev = &gdma->dma_dev;
	dma_dev->dev = dev;
	INIT_LIST_HEAD(&dma_dev->channels);
	dma_cap_set(DMA_MEMCPY, dma_dev->cap_mask);
	dma_cap_set(DMA_MEMSET, dma_dev->cap_mask);
	dma_cap_set(DMA_SLAVE, dma_dev->cap_mask);
	dma_cap_set(DMA_CYCLIC, dma_dev->cap_mask);
	/* M2M buffer addresses and lengths are hardware word-granular. */
	dma_dev->copy_align = DMAENGINE_ALIGN_4_BYTES;
	dma_dev->fill_align = DMAENGINE_ALIGN_4_BYTES;
	dma_dev->directions = BIT(DMA_MEM_TO_MEM) | BIT(DMA_MEM_TO_DEV) |
			      BIT(DMA_DEV_TO_MEM);
	dma_dev->src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |
				   BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
				   BIT(DMA_SLAVE_BUSWIDTH_4_BYTES) |
				   BIT(DMA_SLAVE_BUSWIDTH_8_BYTES);
	dma_dev->dst_addr_widths = dma_dev->src_addr_widths;
	dma_dev->min_burst = 1;
	dma_dev->max_burst = ESP32S31_AXI_GDMA_BURST_SIZE;
	dma_dev->residue_granularity = DMA_RESIDUE_GRANULARITY_SEGMENT;
	dma_dev->device_alloc_chan_resources = esp32s31_axi_alloc_chan_resources;
	dma_dev->device_free_chan_resources = esp32s31_axi_free_chan_resources;
	dma_dev->device_config = esp32s31_axi_slave_config;
	dma_dev->device_prep_dma_memcpy = esp32s31_axi_prep_memcpy;
	dma_dev->device_prep_dma_memset = esp32s31_axi_prep_memset;
	dma_dev->device_prep_slave_sg = esp32s31_axi_prep_slave_sg;
	dma_dev->device_prep_dma_cyclic = esp32s31_axi_prep_cyclic;
	dma_dev->device_issue_pending = esp32s31_axi_issue_pending;
	dma_dev->device_tx_status = esp32s31_axi_tx_status;
	dma_dev->device_terminate_all = esp32s31_axi_terminate_all;
	dma_dev->device_synchronize = esp32s31_axi_synchronize;

	for (i = 0; i < ESP32S31_AXI_GDMA_CHANNELS; i++) {
		struct esp32s31_axi_chan *chan = &gdma->chans[i];

		chan->gdma = gdma;
		chan->id = i;
		chan->request_id = U32_MAX;
		chan->error_cookie = -EBUSY;
		INIT_LIST_HEAD(&chan->running);
		chan->vc.desc_free = esp32s31_axi_free_desc;
		vchan_init(&chan->vc, dma_dev);
	}
	esp32s31_axi_hw_init(gdma);

	for (i = 0; i < ESP32S31_AXI_GDMA_CHANNELS; i++) {
		struct esp32s31_axi_chan *chan = &gdma->chans[i];
		char irq_name[8];

		snprintf(irq_name, sizeof(irq_name), "rx%u", i);
		chan->rx_irq = platform_get_irq_byname(pdev, irq_name);
		if (chan->rx_irq < 0) {
			ret = chan->rx_irq;
			goto err_vchans;
		}
		ret = devm_request_irq(dev, chan->rx_irq, esp32s31_axi_rx_irq,
				       0, dev_name(dev), chan);
		if (ret)
			goto err_vchans;
		snprintf(irq_name, sizeof(irq_name), "tx%u", i);
		chan->tx_irq = platform_get_irq_byname(pdev, irq_name);
		if (chan->tx_irq < 0) {
			ret = chan->tx_irq;
			goto err_vchans;
		}
		ret = devm_request_irq(dev, chan->tx_irq, esp32s31_axi_tx_irq,
				       0, dev_name(dev), chan);
		if (ret)
			goto err_vchans;
	}

	ret = dma_async_device_register(dma_dev);
	if (ret)
		goto err_vchans;
	ret = of_dma_controller_register(dev->of_node, esp32s31_axi_of_xlate, gdma);
	if (ret)
		goto err_unregister;
	platform_set_drvdata(pdev, gdma);
	dev_info(dev,
		 "AXI GDMA ready: %u pairs, memcpy/memset/slave-sg/cyclic, burst %u, version %#x\n",
		 ESP32S31_AXI_GDMA_CHANNELS, ESP32S31_AXI_GDMA_BURST_SIZE,
		 readl(gdma->base + ESP32S31_AXI_DATE));
	return 0;

err_unregister:
	dma_async_device_unregister(dma_dev);
err_vchans:
	for (i = 0; i < ESP32S31_AXI_GDMA_CHANNELS; i++)
		tasklet_kill(&gdma->chans[i].vc.task);
	return ret;
}

static void esp32s31_axi_remove(struct platform_device *pdev)
{
	struct esp32s31_axi_gdma *gdma = platform_get_drvdata(pdev);
	unsigned int i;

	of_dma_controller_free(pdev->dev.of_node);
	dma_async_device_unregister(&gdma->dma_dev);
	for (i = 0; i < ESP32S31_AXI_GDMA_CHANNELS; i++) {
		esp32s31_axi_terminate_all(&gdma->chans[i].vc.chan);
		tasklet_kill(&gdma->chans[i].vc.task);
	}
}

static const struct of_device_id esp32s31_axi_of_match[] = {
	{ .compatible = "espressif,esp32s31-axi-gdma" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s31_axi_of_match);

static struct platform_driver esp32s31_axi_driver = {
	.probe = esp32s31_axi_probe,
	.remove = esp32s31_axi_remove,
	.driver = {
		.name = "esp32s31-axi-gdma",
		.of_match_table = esp32s31_axi_of_match,
	},
};
module_platform_driver(esp32s31_axi_driver);

MODULE_DESCRIPTION("Espressif ESP32-S31 AXI GDMA dmaengine driver");
MODULE_LICENSE("GPL");
