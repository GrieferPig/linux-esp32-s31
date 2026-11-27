// SPDX-License-Identifier: GPL-2.0-only
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/of_device.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>

#include "dmaengine.h"
#include "espressif-dma2d.h"
#include "virt-dma.h"

/* ESP32-P4 TRM ch.8 2D-DMA — channel registers and linked descriptors. */
#define D2D_RX_BASE		0x500
#define D2D_CH_STRIDE		0x100
#define D2D_CONF0		0x00
#define D2D_INT_ENA		0x08
#define D2D_INT_ST		0x0c
#define D2D_INT_CLR		0x10
#define D2D_LINK_CONF		0x1c
#define D2D_LINK_ADDR		0x20
#define D2D_STATE		0x24
#define D2D_TX_PERI		0x38
#define D2D_RX_PERI		0x3c
#define D2D_RX_COLOR_CONV	0x4c
#define D2D_RX_SCRAMBLE		0x50
#define D2D_RX_COLOR_PARAM0	0x54
#define D2D_TX_PORT_BLK		0x6c

#define D2D_RST_CONF		0xa04
#define D2D_MEM_START		0xa08
#define D2D_MEM_END		0xa0c
#define D2D_EXT_START		0xa10
#define D2D_EXT_END		0xa14

#define D2D_CH_RST		BIT(24)
#define D2D_CMD_DISABLE		BIT(25)
#define D2D_TX_RESET_AVAIL	BIT(24)
#define D2D_RX_RESET_AVAIL	BIT(23)
#define D2D_RX_M2M		BIT(0)
#define D2D_DSCR_BURST_EN	BIT(2)
#define D2D_MEM_BURST_128	(4U << 6)
#define D2D_MB_SIZE_SHIFT	9
#define D2D_REORDER_EN		BIT(16)
#define D2D_TX_START		BIT(21)
#define D2D_RX_START		BIT(22)
#define D2D_REG_CLK_EN		BIT(2)
#define D2D_PORT_ENABLE		BIT(11)

#define D2D_TX_EOF		BIT(1)
#define D2D_TX_ERROR		BIT(2)
#define D2D_RX_EOF		BIT(1)
#define D2D_RX_ERRORS		(BIT(2) | BIT(3) | BIT(10))

#define D2D_DESC_OWNER		BIT(31)
#define D2D_DESC_EOF		BIT(30)
#define D2D_DESC_2D		BIT(29)
#define D2D_DESC_PIXEL		GENMASK(31, 28)
#define D2D_DESC_HORIZ		GENMASK(27, 14)
#define D2D_DESC_VERT		GENMASK(13, 0)
#define D2D_COUNT_MAX		FIELD_MAX(D2D_DESC_VERT)
#define D2D_COPY_MAX		GENMASK(27, 0)
/* ESP32-P4 rev1.3 TRM ch.6 and 33: JPEG linked list and MCU layout. */
#define D2D_DESC_MULTIPLE	BIT(28)
#define D2D_JPEG		0
#define D2D_JPEG_GRAY_MCU	8
#define D2D_M2M_DUMMY		4
/* TRM ch.40 PPA: the SRM and blend engines use dedicated peripheral selectors. */
#define D2D_PPA_SRM		1
#define D2D_PPA_BLEND_BG_TX	2
#define D2D_PPA_BLEND_FG_TX	3
#define D2D_PPA_BLEND_RX	2
#define D2D_SRM_RGB565		3
#define D2D_SRM_RGB_PADDING	18
#define D2D_SRM_RX_BLOCK	2
#define D2D_MAX_PAIRS		2

struct esp_dma2d_hw_desc {
	__le32 control;
	__le32 frame;
	__le32 position;
	__le32 buffer;
	__le32 next;
} __aligned(8);

struct esp_dma2d_desc {
	struct virt_dma_desc vd;
	struct esp_dma2d_hw_desc *hw;
	dma_addr_t dma;
	enum {
		D2D_MEMCPY,
		D2D_PPA,
		D2D_PPA_FILL,
		D2D_PPA_BLEND_BG,
		D2D_PPA_BLEND_FG,
		D2D_JPEG_GRAY,
		D2D_JPEG_XFER,
	} mode;
	/* S31 JPEG decode/encode transfer options (D2D_JPEG_XFER only). */
	bool jpeg_encode;
	u8 mb_size;
	u8 rx_csc;
	u8 rx_post_scramble;
	u32 *rx_bytes_out;
};

struct esp_dma2d_soc {
	u32 sram_start;
	u32 sram_end;
	u32 ext_start;
	u32 ext_end;
	bool has_srm;
	bool noncoherent_desc;
};

struct esp_dma2d_chan {
	struct virt_dma_chan vc;
	struct esp_dma2d_desc *active;
	struct dma_pool *pool;
	void __iomem *tx;
	void __iomem *rx;
	dma_cookie_t error_cookie;
	int tx_irq;
	int rx_irq;
	bool has_srm;
	bool noncoherent_desc;
};

struct esp_dma2d {
	struct dma_device engine;
	struct esp_dma2d_chan chan[D2D_MAX_PAIRS];
};

static struct esp_dma2d_chan *to_dma2d_chan(struct dma_chan *chan)
{
	return container_of(chan, struct esp_dma2d_chan, vc.chan);
}

static void esp_dma2d_desc_free(struct virt_dma_desc *vd)
{
	struct esp_dma2d_desc *desc = container_of(vd, struct esp_dma2d_desc, vd);
	struct esp_dma2d_chan *chan = to_dma2d_chan(vd->tx.chan);

	if (chan->noncoherent_desc) {
		if (desc->dma)
			dma_unmap_single(chan->vc.chan.device->dev, desc->dma,
					 2 * sizeof(*desc->hw), DMA_BIDIRECTIONAL);
		kfree(desc->hw);
	} else {
		dma_pool_free(chan->pool, desc->hw, desc->dma);
	}
	kfree(desc);
}

static struct esp_dma2d_desc *esp_dma2d_alloc(struct esp_dma2d_chan *chan)
{
	struct esp_dma2d_desc *desc;

	desc = kzalloc(sizeof(*desc), GFP_ATOMIC);
	if (!desc)
		return NULL;

	if (chan->noncoherent_desc)
		desc->hw = kzalloc(ALIGN(2 * sizeof(*desc->hw), ARCH_DMA_MINALIGN),
				   GFP_ATOMIC);
	else
		desc->hw = dma_pool_zalloc(chan->pool, GFP_ATOMIC, &desc->dma);
	if (!desc->hw) {
		kfree(desc);
		return NULL;
	}

	return desc;
}

static int esp_dma2d_map_desc(struct esp_dma2d_chan *chan,
			      struct esp_dma2d_desc *desc)
{
	struct device *dev = chan->vc.chan.device->dev;

	if (!chan->noncoherent_desc)
		return 0;

	desc->dma = dma_map_single(dev, desc->hw, 2 * sizeof(*desc->hw),
				   DMA_BIDIRECTIONAL);
	if (dma_mapping_error(dev, desc->dma)) {
		desc->dma = 0;
		kfree(desc->hw);
		kfree(desc);
		return -ENOMEM;
	}

	return 0;
}

static struct dma_async_tx_descriptor *
esp_dma2d_prep_memcpy(struct dma_chan *dchan, dma_addr_t dst,
		     dma_addr_t src, size_t len, unsigned long flags)
{
	struct esp_dma2d_chan *chan = to_dma2d_chan(dchan);
	struct esp_dma2d_desc *desc;
	u32 lo = len & D2D_COUNT_MAX;
	u32 hi = len >> 14;

	if (!len || len > D2D_COPY_MAX)
		return NULL;

	desc = esp_dma2d_alloc(chan);
	if (!desc)
		return NULL;

	desc->hw[0].control = cpu_to_le32(D2D_DESC_OWNER | D2D_DESC_EOF |
					 FIELD_PREP(D2D_DESC_HORIZ, lo) | lo);
	desc->hw[1].control = cpu_to_le32(D2D_DESC_OWNER | lo);
	desc->hw[0].frame = cpu_to_le32(FIELD_PREP(D2D_DESC_HORIZ, hi) | hi);
	desc->hw[1].frame = cpu_to_le32(hi);
	desc->hw[0].buffer = cpu_to_le32(src);
	desc->hw[1].buffer = cpu_to_le32(dst);

	if (esp_dma2d_map_desc(chan, desc))
		return NULL;

	return vchan_tx_prep(&chan->vc, &desc->vd, flags);
}

static struct dma_async_tx_descriptor *
esp_dma2d_prep_interleaved(struct dma_chan *dchan,
			  struct dma_interleaved_template *xt,
			  unsigned long flags)
{
	struct esp_dma2d_chan *chan = to_dma2d_chan(dchan);
	struct esp_dma2d_desc *desc;
	size_t width, src_stride, dst_stride;
	u32 rows, block;

	if (xt->dir != DMA_MEM_TO_MEM || xt->frame_size != 1 ||
	    !xt->src_inc || !xt->dst_inc || !xt->numf ||
	    xt->numf > D2D_COUNT_MAX || (flags & DMA_PREP_REPEAT))
		return NULL;

	width = xt->sgl[0].size;
	rows = xt->numf;
	if (!width ||
	    check_add_overflow(width, dmaengine_get_src_icg(xt, xt->sgl), &src_stride) ||
	    check_add_overflow(width, dmaengine_get_dst_icg(xt, xt->sgl), &dst_stride) ||
	    src_stride > D2D_COUNT_MAX || dst_stride > D2D_COUNT_MAX)
		return NULL;
	/* S31 PSRAM writes have 2-byte minimum granularity; odd widths overwrite the first gap byte. */
	if (chan->noncoherent_desc &&
	    !IS_ALIGNED(xt->dst_start | width | dst_stride, 2))
		return NULL;

	desc = esp_dma2d_alloc(chan);
	if (!desc)
		return NULL;

	/* 2D point (0, 0) is each client's start. Use one byte per pixel. */
	block = D2D_DESC_OWNER | D2D_DESC_2D |
		FIELD_PREP(D2D_DESC_HORIZ, width) | rows;
	desc->hw[0].control = cpu_to_le32(block | D2D_DESC_EOF);
	desc->hw[1].control = cpu_to_le32(block);
	desc->hw[0].frame = cpu_to_le32(FIELD_PREP(D2D_DESC_PIXEL, 1) |
					FIELD_PREP(D2D_DESC_HORIZ, src_stride) | rows);
	desc->hw[1].frame = cpu_to_le32(FIELD_PREP(D2D_DESC_PIXEL, 1) |
					FIELD_PREP(D2D_DESC_HORIZ, dst_stride) | rows);
	desc->hw[0].buffer = cpu_to_le32(xt->src_start);
	desc->hw[1].buffer = cpu_to_le32(xt->dst_start);

	if (esp_dma2d_map_desc(chan, desc))
		return NULL;

	return vchan_tx_prep(&chan->vc, &desc->vd, flags);
}

static void esp_dma2d_srm_desc(struct esp_dma2d_hw_desc *hw,
			      const struct esp_dma2d_image *image,
			      unsigned int width, unsigned int height)
{
	hw->control = cpu_to_le32(D2D_DESC_OWNER | D2D_DESC_EOF | D2D_DESC_2D |
				  FIELD_PREP(D2D_DESC_HORIZ, width) | height);
	hw->frame = cpu_to_le32(FIELD_PREP(D2D_DESC_PIXEL, D2D_SRM_RGB565) |
				FIELD_PREP(D2D_DESC_HORIZ, image->width) | image->height);
	hw->buffer = cpu_to_le32(image->address);
}

struct dma_async_tx_descriptor *
esp_dma2d_prep_ppa_srm(struct dma_chan *dchan, const struct esp_dma2d_image *src,
		      const struct esp_dma2d_image *dst, unsigned long flags)
{
	struct esp_dma2d_chan *chan;
	struct esp_dma2d_desc *desc;

	if (dchan->device->device_prep_interleaved_dma != esp_dma2d_prep_interleaved)
		return NULL;
	chan = to_dma2d_chan(dchan);
	if (!chan->has_srm || !IS_ALIGNED(src->address | dst->address, 4) ||
	    !IS_ALIGNED(src->width | dst->width, 2) ||
	    !src->width || !src->height || !dst->width || !dst->height ||
	    src->width > D2D_COUNT_MAX || src->height > D2D_COUNT_MAX ||
	    dst->width > D2D_COUNT_MAX || dst->height > D2D_COUNT_MAX ||
	    upper_32_bits(src->address) || upper_32_bits(dst->address))
		return NULL;

	desc = esp_dma2d_alloc(chan);
	if (!desc)
		return NULL;

	desc->mode = D2D_PPA;
	esp_dma2d_srm_desc(&desc->hw[0], src, src->width, src->height);
	/* With the descriptor port enabled, SRM supplies actual RX blocks. */
	esp_dma2d_srm_desc(&desc->hw[1], dst, D2D_SRM_RX_BLOCK, D2D_SRM_RX_BLOCK);

	if (esp_dma2d_map_desc(chan, desc)) {
		esp_dma2d_desc_free(&desc->vd);
		return NULL;
	}

	return vchan_tx_prep(&chan->vc, &desc->vd, flags);
}
EXPORT_SYMBOL_GPL(esp_dma2d_prep_ppa_srm);

static bool esp_dma2d_ppa_rect_valid(const struct esp_dma2d_ppa_rect *r)
{
	return r && !upper_32_bits(r->address) &&
	       r->pic_w && r->pic_h && r->block_w && r->block_h &&
	       r->pic_w <= D2D_COUNT_MAX && r->pic_h <= D2D_COUNT_MAX &&
	       r->block_w <= D2D_COUNT_MAX && r->block_h <= D2D_COUNT_MAX &&
	       r->x + r->block_w <= r->pic_w && r->y + r->block_h <= r->pic_h &&
	       r->pbyte <= FIELD_MAX(D2D_DESC_PIXEL);
}

static void esp_dma2d_fill_rect_desc(struct esp_dma2d_hw_desc *hw,
				     const struct esp_dma2d_ppa_rect *r)
{
	hw->control = cpu_to_le32(D2D_DESC_OWNER | D2D_DESC_EOF | D2D_DESC_2D |
				  FIELD_PREP(D2D_DESC_HORIZ, r->block_w) |
				  r->block_h);
	hw->frame = cpu_to_le32(FIELD_PREP(D2D_DESC_PIXEL, r->pbyte) |
				FIELD_PREP(D2D_DESC_HORIZ, r->pic_w) | r->pic_h);
	hw->position = cpu_to_le32(FIELD_PREP(D2D_DESC_HORIZ, r->x) | r->y);
	hw->buffer = cpu_to_le32(r->address);
}

struct dma_async_tx_descriptor *
esp_dma2d_prep_ppa_fill(struct dma_chan *dchan,
			const struct esp_dma2d_ppa_rect *dst,
			unsigned long flags)
{
	struct esp_dma2d_chan *chan;
	struct esp_dma2d_desc *desc;

	if (dchan->device->device_prep_interleaved_dma != esp_dma2d_prep_interleaved)
		return NULL;
	chan = to_dma2d_chan(dchan);
	if (!chan->has_srm || !esp_dma2d_ppa_rect_valid(dst))
		return NULL;

	desc = esp_dma2d_alloc(chan);
	if (!desc)
		return NULL;

	desc->mode = D2D_PPA_FILL;
	esp_dma2d_fill_rect_desc(&desc->hw[1], dst);

	if (esp_dma2d_map_desc(chan, desc)) {
		esp_dma2d_desc_free(&desc->vd);
		return NULL;
	}

	return vchan_tx_prep(&chan->vc, &desc->vd, flags);
}
EXPORT_SYMBOL_GPL(esp_dma2d_prep_ppa_fill);

struct dma_async_tx_descriptor *
esp_dma2d_prep_ppa_blend_bg(struct dma_chan *dchan,
			    const struct esp_dma2d_ppa_rect *bg,
			    const struct esp_dma2d_ppa_rect *dst,
			    unsigned long flags)
{
	struct esp_dma2d_chan *chan;
	struct esp_dma2d_desc *desc;

	if (dchan->device->device_prep_interleaved_dma != esp_dma2d_prep_interleaved)
		return NULL;
	chan = to_dma2d_chan(dchan);
	if (!chan->has_srm || !esp_dma2d_ppa_rect_valid(bg) ||
	    !esp_dma2d_ppa_rect_valid(dst))
		return NULL;

	desc = esp_dma2d_alloc(chan);
	if (!desc)
		return NULL;

	desc->mode = D2D_PPA_BLEND_BG;
	esp_dma2d_fill_rect_desc(&desc->hw[0], bg);
	esp_dma2d_fill_rect_desc(&desc->hw[1], dst);

	if (esp_dma2d_map_desc(chan, desc)) {
		esp_dma2d_desc_free(&desc->vd);
		return NULL;
	}

	return vchan_tx_prep(&chan->vc, &desc->vd, flags);
}
EXPORT_SYMBOL_GPL(esp_dma2d_prep_ppa_blend_bg);

struct dma_async_tx_descriptor *
esp_dma2d_prep_ppa_blend_fg(struct dma_chan *dchan,
			    const struct esp_dma2d_ppa_rect *fg,
			    unsigned long flags)
{
	struct esp_dma2d_chan *chan;
	struct esp_dma2d_desc *desc;

	if (dchan->device->device_prep_interleaved_dma != esp_dma2d_prep_interleaved)
		return NULL;
	chan = to_dma2d_chan(dchan);
	if (!chan->has_srm || !esp_dma2d_ppa_rect_valid(fg))
		return NULL;

	desc = esp_dma2d_alloc(chan);
	if (!desc)
		return NULL;

	desc->mode = D2D_PPA_BLEND_FG;
	esp_dma2d_fill_rect_desc(&desc->hw[0], fg);

	if (esp_dma2d_map_desc(chan, desc)) {
		esp_dma2d_desc_free(&desc->vd);
		return NULL;
	}

	return vchan_tx_prep(&chan->vc, &desc->vd, flags);
}
EXPORT_SYMBOL_GPL(esp_dma2d_prep_ppa_blend_fg);

struct dma_async_tx_descriptor *
esp_dma2d_prep_jpeg_gray(struct dma_chan *dchan, dma_addr_t src, size_t len,
		       const struct esp_dma2d_image *dst, unsigned long flags)
{
	struct esp_dma2d_chan *chan;
	struct esp_dma2d_desc *desc;
	u32 high = len >> 14, low = len & D2D_COUNT_MAX;

	if (dchan->device->device_prep_interleaved_dma != esp_dma2d_prep_interleaved)
		return NULL;
	chan = to_dma2d_chan(dchan);
	if (!chan->has_srm || !len || len > D2D_COPY_MAX ||
	    upper_32_bits(src) || upper_32_bits(dst->address) ||
	    !IS_ALIGNED(dst->address, 4) ||
	    !dst->width || !dst->height ||
	    !IS_ALIGNED(dst->width | dst->height, D2D_JPEG_GRAY_MCU) ||
	    dst->width > D2D_COUNT_MAX || dst->height > D2D_COUNT_MAX)
		return NULL;

	desc = esp_dma2d_alloc(chan);
	if (!desc)
		return NULL;

	/* JPEG consumes a linear scan; the RX stream walks 8x8 blocks. */
	desc->mode = D2D_JPEG_GRAY;
	desc->hw[0].control = cpu_to_le32(D2D_DESC_OWNER |
					 FIELD_PREP(D2D_DESC_HORIZ, low) | low);
	desc->hw[0].frame = cpu_to_le32(FIELD_PREP(D2D_DESC_PIXEL, 1) |
				       FIELD_PREP(D2D_DESC_HORIZ, high) | high);
	desc->hw[0].buffer = cpu_to_le32(src);
	desc->hw[1].control = cpu_to_le32(D2D_DESC_OWNER | D2D_DESC_2D |
					 FIELD_PREP(D2D_DESC_HORIZ, D2D_JPEG_GRAY_MCU) |
					 D2D_JPEG_GRAY_MCU);
	desc->hw[1].frame = cpu_to_le32(FIELD_PREP(D2D_DESC_PIXEL, 1) |
				       FIELD_PREP(D2D_DESC_HORIZ, dst->width) | dst->height);
	desc->hw[1].position = cpu_to_le32(D2D_DESC_MULTIPLE);
	desc->hw[1].buffer = cpu_to_le32(dst->address);

	return vchan_tx_prep(&chan->vc, &desc->vd, flags);
}
EXPORT_SYMBOL_GPL(esp_dma2d_prep_jpeg_gray);

/*
 * S31 TRM ch.35 JPEG / IDF dma2d_ll.h: full 2D-DMA JPEG decode & encode
 * descriptor builder with macro-block reorder and optional RX CSC.
 */
struct dma_async_tx_descriptor *
esp_dma2d_prep_jpeg_xfer(struct dma_chan *dchan,
			 const struct esp_dma2d_jpeg_cfg *cfg,
			 unsigned long flags)
{
	struct esp_dma2d_chan *chan;
	struct esp_dma2d_desc *desc;
	u32 high, low;

	if (!cfg || dchan->device->device_prep_interleaved_dma != esp_dma2d_prep_interleaved)
		return NULL;
	chan = to_dma2d_chan(dchan);
	if (!chan->has_srm || !cfg->len_1d || cfg->len_1d > D2D_COPY_MAX ||
	    upper_32_bits(cfg->buf_1d) || upper_32_bits(cfg->buf_2d) ||
	    !cfg->width || !cfg->height || !cfg->block_w || !cfg->block_h ||
	    cfg->width > D2D_COUNT_MAX || cfg->height > D2D_COUNT_MAX ||
	    cfg->block_w > D2D_COUNT_MAX || cfg->block_h > D2D_COUNT_MAX)
		return NULL;

	desc = esp_dma2d_alloc(chan);
	if (!desc)
		return NULL;

	desc->mode = D2D_JPEG_XFER;
	desc->jpeg_encode = cfg->encode;
	desc->mb_size = cfg->mb_size;
	desc->rx_csc = cfg->rx_csc;
	desc->rx_post_scramble = cfg->rx_post_scramble;
	desc->rx_bytes_out = cfg->rx_bytes_out;

	high = cfg->len_1d >> 14;
	low = cfg->len_1d & D2D_COUNT_MAX;

	if (cfg->encode) {
		/* Encode: TX = 2D pixel image, RX = 1D compressed bitstream. */
		desc->hw[0].control = cpu_to_le32(D2D_DESC_OWNER | D2D_DESC_2D |
						  FIELD_PREP(D2D_DESC_HORIZ, cfg->block_w) |
						  cfg->block_h);
		desc->hw[0].frame = cpu_to_le32(FIELD_PREP(D2D_DESC_PIXEL, cfg->pbyte) |
						FIELD_PREP(D2D_DESC_HORIZ, cfg->width) |
						cfg->height);
		desc->hw[0].position = cpu_to_le32(D2D_DESC_MULTIPLE);
		desc->hw[0].buffer = cpu_to_le32(cfg->buf_2d);

		desc->hw[1].control = cpu_to_le32(D2D_DESC_OWNER | low);
		desc->hw[1].frame = cpu_to_le32(FIELD_PREP(D2D_DESC_PIXEL, 1) | high);
		desc->hw[1].position = 0;
		desc->hw[1].buffer = cpu_to_le32(cfg->buf_1d);
	} else {
		/* Decode: TX = 1D bitstream, RX = 2D decoded pixel image. */
		desc->hw[0].control = cpu_to_le32(D2D_DESC_OWNER |
						  FIELD_PREP(D2D_DESC_HORIZ, low) | low);
		desc->hw[0].frame = cpu_to_le32(FIELD_PREP(D2D_DESC_PIXEL, 1) |
						FIELD_PREP(D2D_DESC_HORIZ, high) | high);
		desc->hw[0].position = 0;
		desc->hw[0].buffer = cpu_to_le32(cfg->buf_1d);

		desc->hw[1].control = cpu_to_le32(D2D_DESC_OWNER | D2D_DESC_2D |
						  FIELD_PREP(D2D_DESC_HORIZ, cfg->block_w) |
						  cfg->block_h);
		desc->hw[1].frame = cpu_to_le32(FIELD_PREP(D2D_DESC_PIXEL, cfg->pbyte) |
						FIELD_PREP(D2D_DESC_HORIZ, cfg->width) |
						cfg->height);
		desc->hw[1].position = cpu_to_le32(D2D_DESC_MULTIPLE);
		desc->hw[1].buffer = cpu_to_le32(cfg->buf_2d);
	}

	if (esp_dma2d_map_desc(chan, desc)) {
		esp_dma2d_desc_free(&desc->vd);
		return NULL;
	}

	return vchan_tx_prep(&chan->vc, &desc->vd, flags);
}
EXPORT_SYMBOL_GPL(esp_dma2d_prep_jpeg_xfer);

/* TRM ch.8.7.7 — CMD_DISABLE then RESET_AVAIL before reset or freeing memory. */
static int esp_dma2d_stop(struct esp_dma2d_chan *chan)
{
	u32 tx, rx;
	int ret;

	writel(0, chan->tx + D2D_INT_ENA);
	writel(0, chan->rx + D2D_INT_ENA);
	writel(readl(chan->tx + D2D_CONF0) | D2D_CMD_DISABLE,
	       chan->tx + D2D_CONF0);
	writel(readl(chan->rx + D2D_CONF0) | D2D_CMD_DISABLE,
	       chan->rx + D2D_CONF0);

	ret = read_poll_timeout_atomic(readl, tx,
				       (tx & D2D_TX_RESET_AVAIL) &&
				       ((rx = readl(chan->rx + D2D_STATE)) & D2D_RX_RESET_AVAIL),
				       1, 1000, false, chan->tx + D2D_STATE);
	if (ret)
		return ret;

	writel(D2D_CH_RST | D2D_CMD_DISABLE, chan->tx + D2D_CONF0);
	writel(D2D_CH_RST | D2D_CMD_DISABLE, chan->rx + D2D_CONF0);
	writel(0, chan->tx + D2D_CONF0);
	writel(chan->has_srm ? 0 : D2D_RX_M2M, chan->rx + D2D_CONF0);
	writel(U32_MAX, chan->tx + D2D_INT_CLR);
	writel(U32_MAX, chan->rx + D2D_INT_CLR);
	return 0;
}

static void esp_dma2d_start(struct esp_dma2d_chan *chan)
{
	struct virt_dma_desc *vd = vchan_next_desc(&chan->vc);
	struct esp_dma2d_desc *desc;

	if (!vd)
		return;

	list_del(&vd->node);
	desc = container_of(vd, struct esp_dma2d_desc, vd);
	chan->active = desc;
	if (desc->vd.tx.cookie == chan->error_cookie)
		WRITE_ONCE(chan->error_cookie, 0);
	if (desc->mode == D2D_PPA) {
		/* TRM ch.8.7.3–4: reset the channel FSMs before configuring a port. */
		writel(D2D_CH_RST, chan->tx + D2D_CONF0);
		writel(D2D_CH_RST, chan->rx + D2D_CONF0);
		writel(D2D_PORT_ENABLE, chan->tx + D2D_CONF0);
		writel(D2D_PORT_ENABLE, chan->rx + D2D_CONF0);
		writel(D2D_SRM_RGB_PADDING | (D2D_SRM_RGB_PADDING << 14),
		       chan->tx + D2D_TX_PORT_BLK);
		writel(D2D_PPA_SRM, chan->tx + D2D_TX_PERI);
		writel(D2D_PPA_SRM, chan->rx + D2D_RX_PERI);
	} else if (desc->mode == D2D_PPA_FILL) {
		/* IDF ppa_fill.c: RX-only single-block 2D transfer from PPA_BLEND. */
		writel(D2D_CH_RST, chan->rx + D2D_CONF0);
		writel(D2D_DSCR_BURST_EN | D2D_MEM_BURST_128, chan->rx + D2D_CONF0);
		writel(D2D_PPA_BLEND_RX, chan->rx + D2D_RX_PERI);
		dma_wmb();
		writel(desc->dma + sizeof(*desc->hw), chan->rx + D2D_LINK_ADDR);
		writel(0, chan->tx + D2D_INT_ENA);
		writel(D2D_RX_EOF | D2D_RX_ERRORS, chan->rx + D2D_INT_ENA);
		writel(D2D_RX_START, chan->rx + D2D_LINK_CONF);
		return;
	} else if (desc->mode == D2D_PPA_BLEND_BG) {
		/* IDF ppa_blend.c: BG TX + output RX on PPA_BLEND. */
		writel(D2D_CH_RST, chan->tx + D2D_CONF0);
		writel(D2D_CH_RST, chan->rx + D2D_CONF0);
		writel(D2D_DSCR_BURST_EN | D2D_MEM_BURST_128, chan->tx + D2D_CONF0);
		writel(D2D_DSCR_BURST_EN | D2D_MEM_BURST_128, chan->rx + D2D_CONF0);
		writel(D2D_PPA_BLEND_BG_TX, chan->tx + D2D_TX_PERI);
		writel(D2D_PPA_BLEND_RX, chan->rx + D2D_RX_PERI);
	} else if (desc->mode == D2D_PPA_BLEND_FG) {
		/* IDF ppa_blend.c: FG TX-only on PPA_BLEND. */
		writel(D2D_CH_RST, chan->tx + D2D_CONF0);
		writel(D2D_DSCR_BURST_EN | D2D_MEM_BURST_128, chan->tx + D2D_CONF0);
		writel(D2D_PPA_BLEND_FG_TX, chan->tx + D2D_TX_PERI);
		dma_wmb();
		writel(desc->dma, chan->tx + D2D_LINK_ADDR);
		writel(D2D_TX_EOF | D2D_TX_ERROR, chan->tx + D2D_INT_ENA);
		writel(0, chan->rx + D2D_INT_ENA);
		writel(D2D_TX_START, chan->tx + D2D_LINK_CONF);
		return;
	} else if (desc->mode == D2D_JPEG_GRAY) {
		writel(D2D_CH_RST, chan->tx + D2D_CONF0);
		writel(D2D_CH_RST, chan->rx + D2D_CONF0);
		/* JPEG uses linked descriptors, with DSCR_PORT_EN clear. */
		writel(0, chan->tx + D2D_CONF0);
		writel(0, chan->rx + D2D_CONF0);
		writel(D2D_JPEG, chan->tx + D2D_TX_PERI);
		writel(D2D_JPEG, chan->rx + D2D_RX_PERI);
	} else if (desc->mode == D2D_JPEG_XFER) {
		/* IDF jpeg_decode.c / jpeg_encode.c + dma2d_ll.h RX CSC & reorder. */
		static const u32 bt601_yuv2rgb[6] = {
			298U, 210164121U, 1995050U, 35540784U, 528682U, 195899392U,
		};
		u32 tx_conf = D2D_DSCR_BURST_EN | D2D_MEM_BURST_128;
		u32 rx_conf = D2D_DSCR_BURST_EN | D2D_MEM_BURST_128;
		u32 csc = (7U << 3) | 1U; /* input_sel=7 (disabled), output_sel=1 */
		bool load_bt601 = false;

		writel(D2D_CH_RST, chan->tx + D2D_CONF0);
		writel(D2D_CH_RST, chan->rx + D2D_CONF0);
		if (desc->jpeg_encode) {
			tx_conf |= ((u32)desc->mb_size << D2D_MB_SIZE_SHIFT) | D2D_REORDER_EN;
			rx_conf |= ((u32)ESP_DMA2D_MB_NONE << D2D_MB_SIZE_SHIFT);
		} else {
			tx_conf |= ((u32)ESP_DMA2D_MB_NONE << D2D_MB_SIZE_SHIFT);
			rx_conf |= ((u32)desc->mb_size << D2D_MB_SIZE_SHIFT) | D2D_REORDER_EN;
		}
		writel(tx_conf, chan->tx + D2D_CONF0);
		writel(rx_conf, chan->rx + D2D_CONF0);

		switch (desc->rx_csc) {
		case ESP_DMA2D_RX_CSC_YUV420_TO_RGB565_601:
			csc = (0U << 3) | BIT(2) | 0U;
			load_bt601 = true;
			break;
		case ESP_DMA2D_RX_CSC_YUV420_TO_RGB888_601:
			csc = (0U << 3) | BIT(2) | 1U;
			load_bt601 = true;
			break;
		case ESP_DMA2D_RX_CSC_YUV444_TO_RGB565_601:
			csc = (2U << 3) | BIT(2) | 0U;
			load_bt601 = true;
			break;
		case ESP_DMA2D_RX_CSC_YUV444_TO_RGB888_601:
			csc = (2U << 3) | BIT(2) | 1U;
			load_bt601 = true;
			break;
		default:
			break;
		}
		writel(csc, chan->rx + D2D_RX_COLOR_CONV);
		writel((u32)desc->rx_post_scramble << 3, chan->rx + D2D_RX_SCRAMBLE);
		if (load_bt601) {
			int k;

			for (k = 0; k < 6; k++)
				writel(bt601_yuv2rgb[k],
				       chan->rx + D2D_RX_COLOR_PARAM0 + k * 4);
		}
		writel(D2D_JPEG, chan->tx + D2D_TX_PERI);
		writel(D2D_JPEG, chan->rx + D2D_RX_PERI);
	} else if (chan->has_srm) {
		writel(D2D_RX_M2M, chan->rx + D2D_CONF0);
		writel(D2D_M2M_DUMMY, chan->tx + D2D_TX_PERI);
		writel(D2D_M2M_DUMMY, chan->rx + D2D_RX_PERI);
	}
	dma_wmb();
	writel(desc->dma, chan->tx + D2D_LINK_ADDR);
	writel(desc->dma + sizeof(*desc->hw), chan->rx + D2D_LINK_ADDR);
	writel(D2D_TX_ERROR, chan->tx + D2D_INT_ENA);
	writel(D2D_RX_EOF | D2D_RX_ERRORS, chan->rx + D2D_INT_ENA);
	writel(D2D_TX_START, chan->tx + D2D_LINK_CONF);
	writel(D2D_RX_START, chan->rx + D2D_LINK_CONF);
}

static void esp_dma2d_issue_pending(struct dma_chan *dchan)
{
	struct esp_dma2d_chan *chan = to_dma2d_chan(dchan);
	unsigned long flags;

	spin_lock_irqsave(&chan->vc.lock, flags);
	if (vchan_issue_pending(&chan->vc) && !chan->active)
		esp_dma2d_start(chan);
	spin_unlock_irqrestore(&chan->vc.lock, flags);
}

static irqreturn_t esp_dma2d_irq(int irq, void *data)
{
	struct esp_dma2d_chan *chan = data;
	struct esp_dma2d_desc *desc;
	unsigned long flags;
	u32 tx, rx;

	spin_lock_irqsave(&chan->vc.lock, flags);
	tx = readl(chan->tx + D2D_INT_ST);
	rx = readl(chan->rx + D2D_INT_ST);
	if (!tx && !rx) {
		spin_unlock_irqrestore(&chan->vc.lock, flags);
		return IRQ_NONE;
	}

	desc = chan->active;
	if (esp_dma2d_stop(chan)) {
		dev_err_ratelimited(chan->vc.chan.device->dev,
				    "channel %d did not quiesce\n", chan->vc.chan.chan_id);
		goto unlock;
	}

	if (desc) {
		dma_rmb();
		if (desc->rx_bytes_out) {
			if (chan->noncoherent_desc)
				dma_sync_single_for_cpu(chan->vc.chan.device->dev,
							desc->dma,
							2 * sizeof(*desc->hw),
							DMA_FROM_DEVICE);
			*desc->rx_bytes_out =
				(FIELD_GET(D2D_DESC_HORIZ, le32_to_cpu(desc->hw[1].frame)) << 14) |
				FIELD_GET(D2D_DESC_HORIZ, le32_to_cpu(desc->hw[1].control));
		}
		if (tx & D2D_TX_ERROR)
			desc->vd.tx_result.result = DMA_TRANS_READ_FAILED;
		else if (rx & D2D_RX_ERRORS)
			desc->vd.tx_result.result = DMA_TRANS_WRITE_FAILED;
		if (desc->vd.tx_result.result != DMA_TRANS_NOERROR)
			WRITE_ONCE(chan->error_cookie, desc->vd.tx.cookie);
		chan->active = NULL;
		vchan_cookie_complete(&desc->vd);
	}
	esp_dma2d_start(chan);
unlock:
	spin_unlock_irqrestore(&chan->vc.lock, flags);
	return IRQ_HANDLED;
}

static enum dma_status esp_dma2d_tx_status(struct dma_chan *dchan,
					  dma_cookie_t cookie,
					  struct dma_tx_state *state)
{
	struct esp_dma2d_chan *chan = to_dma2d_chan(dchan);
	enum dma_status status = dma_cookie_status(dchan, cookie, state);

	if (status == DMA_COMPLETE && cookie == READ_ONCE(chan->error_cookie))
		return DMA_ERROR;
	return status;
}

static int esp_dma2d_terminate_all(struct dma_chan *dchan)
{
	struct esp_dma2d_chan *chan = to_dma2d_chan(dchan);
	unsigned long flags;
	LIST_HEAD(head);
	int ret = 0;

	spin_lock_irqsave(&chan->vc.lock, flags);
	if (chan->active) {
		ret = esp_dma2d_stop(chan);
		if (!ret) {
			vchan_terminate_vdesc(&chan->active->vd);
			chan->active = NULL;
		}
	}
	if (!ret)
		vchan_get_all_descriptors(&chan->vc, &head);
	spin_unlock_irqrestore(&chan->vc.lock, flags);
	vchan_dma_desc_free_list(&chan->vc, &head);
	return ret;
}

static void esp_dma2d_synchronize(struct dma_chan *dchan)
{
	struct esp_dma2d_chan *chan = to_dma2d_chan(dchan);

	synchronize_irq(chan->tx_irq);
	synchronize_irq(chan->rx_irq);
	vchan_synchronize(&chan->vc);
}

static void esp_dma2d_free_chan_resources(struct dma_chan *dchan)
{
	if (WARN_ON_ONCE(esp_dma2d_terminate_all(dchan)))
		return;

	esp_dma2d_synchronize(dchan);
	vchan_free_chan_resources(&to_dma2d_chan(dchan)->vc);
}

static void esp_dma2d_unregister_of(void *data)
{
	of_dma_controller_free(data);
}

static int esp_dma2d_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct esp_dma2d_soc *soc = of_device_get_match_data(dev);
	struct reset_control *rst;
	struct dma_device *engine;
	struct esp_dma2d *dma;
	void __iomem *base;
	struct dma_pool *pool;
	struct clk *clk;
	unsigned int i;
	int ret;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "cannot enable clock\n");

	rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "cannot acquire reset\n");

	ret = reset_control_reset(rst);
	if (ret)
		return dev_err_probe(dev, ret, "cannot reset controller\n");

	writel(D2D_REG_CLK_EN, base + D2D_RST_CONF);
	writel(soc->sram_start, base + D2D_MEM_START);
	writel(soc->sram_end, base + D2D_MEM_END);
	writel(soc->ext_start, base + D2D_EXT_START);
	writel(soc->ext_end, base + D2D_EXT_END);

	dma = devm_kzalloc(dev, sizeof(*dma), GFP_KERNEL);
	if (!dma)
		return -ENOMEM;

	if (soc->noncoherent_desc) {
		pool = NULL;
	} else {
		pool = dmam_pool_create("esp-dma2d", dev,
					2 * sizeof(struct esp_dma2d_hw_desc),
					__alignof__(struct esp_dma2d_hw_desc), 0);
		if (!pool)
			return -ENOMEM;
	}

	engine = &dma->engine;
	engine->dev = dev;
	engine->device_free_chan_resources = esp_dma2d_free_chan_resources;
	engine->device_prep_dma_memcpy = esp_dma2d_prep_memcpy;
	engine->device_prep_interleaved_dma = esp_dma2d_prep_interleaved;
	engine->device_issue_pending = esp_dma2d_issue_pending;
	engine->device_tx_status = esp_dma2d_tx_status;
	engine->device_terminate_all = esp_dma2d_terminate_all;
	engine->device_synchronize = esp_dma2d_synchronize;
	engine->directions = BIT(DMA_MEM_TO_MEM);
	engine->residue_granularity = DMA_RESIDUE_GRANULARITY_DESCRIPTOR;
	dma_cap_set(DMA_MEMCPY, engine->cap_mask);
	dma_cap_set(DMA_INTERLEAVE, engine->cap_mask);
	INIT_LIST_HEAD(&engine->channels);

	for (i = 0; i < ARRAY_SIZE(dma->chan); i++) {
		struct esp_dma2d_chan *chan = &dma->chan[i];

		chan->pool = pool;
		chan->has_srm = soc->has_srm;
		chan->noncoherent_desc = soc->noncoherent_desc;
		chan->tx = base + i * D2D_CH_STRIDE;
		chan->rx = base + D2D_RX_BASE + i * D2D_CH_STRIDE;
		chan->tx_irq = platform_get_irq(pdev, 2 * i);
		if (chan->tx_irq < 0)
			return chan->tx_irq;
		chan->rx_irq = platform_get_irq(pdev, 2 * i + 1);
		if (chan->rx_irq < 0)
			return chan->rx_irq;

		chan->vc.desc_free = esp_dma2d_desc_free;
		vchan_init(&chan->vc, engine);
		if (!chan->has_srm) {
			ret = esp_dma2d_stop(chan);
			if (ret)
				return dev_err_probe(dev, ret, "channel %u did not reset\n", i);
			writel(D2D_M2M_DUMMY, chan->tx + D2D_TX_PERI);
			writel(D2D_M2M_DUMMY, chan->rx + D2D_RX_PERI);
		}

		ret = devm_request_irq(dev, chan->tx_irq, esp_dma2d_irq, 0,
				       dev_name(dev), chan);
		if (ret)
			return ret;
		ret = devm_request_irq(dev, chan->rx_irq, esp_dma2d_irq, 0,
				       dev_name(dev), chan);
		if (ret)
			return ret;
	}

	ret = dmaenginem_async_device_register(engine);
	if (ret || !of_property_present(dev->of_node, "#dma-cells"))
		return ret;
	ret = of_dma_controller_register(dev->of_node, of_dma_xlate_by_chan_id, engine);
	if (ret)
		return ret;
	return devm_add_action_or_reset(dev, esp_dma2d_unregister_of, dev->of_node);
}

/* TRM ch.8.4.11 Accessing Memory and ch.9 System and Memory. */
static const struct esp_dma2d_soc esp32p4_dma2d = {
	.sram_start = 0x4fc00000,
	.sram_end = 0x4ffc0000,
	.ext_start = 0x48000000,
	.ext_end = 0x4c000000,
	.has_srm = true,
};

/* The S31 uses the same descriptor and channel registers with its bus apertures.
 * S31 PSRAM is non-coherent without Svpbmt, so descriptors must be streamed via
 * dma_map_single() to flush D-cache before starting 2D-DMA.
 */
static const struct esp_dma2d_soc esp32s31_dma2d = {
	.sram_start = 0x2f000000,
	.sram_end = 0x2f07ffff,
	.ext_start = 0x50000000,
	.ext_end = 0x53ffffff,
	.has_srm = true,
	.noncoherent_desc = true,
};

static const struct of_device_id esp_dma2d_of_match[] = {
	{ .compatible = "espressif,esp32p4-dma2d", .data = &esp32p4_dma2d },
	{ .compatible = "espressif,esp32s31-dma2d", .data = &esp32s31_dma2d },
	{ }
};
MODULE_DEVICE_TABLE(of, esp_dma2d_of_match);

static struct platform_driver esp_dma2d_driver = {
	.probe = esp_dma2d_probe,
	.driver = {
		.name = "esp-dma2d",
		.of_match_table = esp_dma2d_of_match,
	},
};
module_platform_driver(esp_dma2d_driver);

MODULE_DESCRIPTION("Espressif 2D DMA memory-to-memory controller");
MODULE_LICENSE("GPL");
