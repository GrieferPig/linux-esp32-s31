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

#endif
