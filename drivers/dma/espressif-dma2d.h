/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef ESPRESSIF_DMA2D_H
#define ESPRESSIF_DMA2D_H

#include <linux/dmaengine.h>

struct esp_dma2d_image {
	dma_addr_t address;
	u16 width;
	u16 height;
};

/* ESP32-P4 TRM ch.40, Table 40.5-4 and Table 40.5-5. */
struct dma_async_tx_descriptor *
esp_dma2d_prep_ppa_srm(struct dma_chan *chan, const struct esp_dma2d_image *src,
		      const struct esp_dma2d_image *dst, unsigned long flags);

/* ESP32-P4 rev1.3 TRM ch.33.7.2: bitstream to an 8x8 grayscale image. */
struct dma_async_tx_descriptor *
esp_dma2d_prep_jpeg_gray(struct dma_chan *chan, dma_addr_t src, size_t len,
		       const struct esp_dma2d_image *dst, unsigned long flags);

/*
 * S31 TRM ch.35 JPEG / IDF dma2d_ll.h: full 2D-DMA JPEG decode & encode
 * descriptor configuration (macro-block reorder, RX CSC, 1D/2D modes).
 */
enum esp_dma2d_mb_size {
	ESP_DMA2D_MB_8X8 = 0,
	ESP_DMA2D_MB_8X16 = 1,
	ESP_DMA2D_MB_16X16 = 2,
	ESP_DMA2D_MB_NONE = 3,
};

enum esp_dma2d_rx_csc {
	ESP_DMA2D_RX_CSC_NONE = 0,
	ESP_DMA2D_RX_CSC_YUV420_TO_RGB565_601,
	ESP_DMA2D_RX_CSC_YUV420_TO_RGB888_601,
	ESP_DMA2D_RX_CSC_YUV444_TO_RGB565_601,
	ESP_DMA2D_RX_CSC_YUV444_TO_RGB888_601,
};

struct esp_dma2d_jpeg_cfg {
	bool encode;			/* true: 2D TX -> 1D RX; false: 1D TX -> 2D RX */
	dma_addr_t buf_1d;		/* bitstream buffer (TX for decode, RX for encode) */
	size_t len_1d;			/* bitstream length (or max capacity for encode) */
	dma_addr_t buf_2d;		/* pixel buffer (RX for decode, TX for encode) */
	u16 width;			/* picture width in pixels (ha_length) */
	u16 height;			/* picture height in pixels (va_size) */
	u16 block_w;			/* macro-block width (hb_length) */
	u16 block_h;			/* macro-block height (vb_size) */
	u8 pbyte;			/* DMA2D_DESCRIPTOR_PBYTE_* (1=1B, 3=2B, 4=3B) */
	enum esp_dma2d_mb_size mb_size;
	enum esp_dma2d_rx_csc rx_csc;
	u8 rx_post_scramble;		/* 0=210 (BGR), 1=201 (RGB565), 5=012 (RGB888) */
	u32 *rx_bytes_out;		/* optional: filled on RX EOF with actual 1D RX bytes */
};

struct dma_async_tx_descriptor *
esp_dma2d_prep_jpeg_xfer(struct dma_chan *chan,
			 const struct esp_dma2d_jpeg_cfg *cfg,
			 unsigned long flags);

/*
 * S31 TRM ch.34 PPA / P4 TRM ch.40 PPA: 2D-DMA helpers for PPA fill
 * (RX-only) and PPA alpha-blend (BG TX + RX on one pair, FG TX on second).
 */
struct esp_dma2d_ppa_rect {
	dma_addr_t address;
	u16 pic_w;
	u16 pic_h;
	u16 block_w;
	u16 block_h;
	u16 x;
	u16 y;
	u8 pbyte;
};

struct dma_async_tx_descriptor *
esp_dma2d_prep_ppa_fill(struct dma_chan *chan,
			const struct esp_dma2d_ppa_rect *dst,
			unsigned long flags);

struct dma_async_tx_descriptor *
esp_dma2d_prep_ppa_blend_bg(struct dma_chan *chan,
			    const struct esp_dma2d_ppa_rect *bg,
			    const struct esp_dma2d_ppa_rect *dst,
			    unsigned long flags);

struct dma_async_tx_descriptor *
esp_dma2d_prep_ppa_blend_fg(struct dma_chan *chan,
			    const struct esp_dma2d_ppa_rect *fg,
			    unsigned long flags);

#endif
