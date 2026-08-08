/* SPDX-License-Identifier: GPL-2.0-only */
/* Shared ABI between esp32s31-ahb-gdma and its UART client. */

#ifndef ESP32S31_AHB_GDMA_H
#define ESP32S31_AHB_GDMA_H

#include <linux/dmaengine.h>

/*
 * Result handed to the RX callback of the persistent circular ring.  The
 * generic dmaengine_result tells the client how many bytes arrived since the
 * previous callback (residue = ring_len - new_bytes); `pos` is the byte
 * offset inside the ring where the new data starts (the DMA restarts the
 * ring at the head after an idle EOF, so DONE events land at pos != 0 while
 * EOF events land at pos == 0).
 */
struct esp32s31_ahb_rx_result {
	struct dmaengine_result res;
	u32 pos;
	bool eof;
};

int esp32s31_ahb_terminate_direction(struct dma_chan *dchan,
				     enum dma_transfer_direction direction);

#endif /* ESP32S31_AHB_GDMA_H */
