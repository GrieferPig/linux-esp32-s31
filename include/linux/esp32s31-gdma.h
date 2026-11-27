/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_ESP32S31_GDMA_H
#define _LINUX_ESP32S31_GDMA_H

#include <linux/dmaengine.h>

/*
 * Run both halves of a dedicated AXI DMA pair through the S31 BitScrambler.
 * The descriptor result reports unused bytes in the output buffer.
 */
struct dma_async_tx_descriptor *
esp32s31_gdma_prep_bitscrambler(struct dma_chan *chan, dma_addr_t dst, size_t dst_len,
				dma_addr_t src, size_t src_len, unsigned long flags);

#endif
