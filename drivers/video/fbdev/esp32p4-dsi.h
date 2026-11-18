/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * ESP32-P4 MIPI DSI display bring-up.
 *
 * The implementation is a Linux port of the ESP-IDF MIPI DSI host/bridge and
 * DW-GDMA bring-up used by the ESP32-P4 Raspberry Pi Touch Display 2 board.
 * The framebuffer itself is owned by the caller; this driver only programs the
 * display pipeline and the VDMA descriptor that reads it.
 */
#ifndef _VIDEO_ESP32P4_DSI_H
#define _VIDEO_ESP32P4_DSI_H

#include <linux/device.h>
#include <linux/types.h>

struct esp32p4_dsi;

/* Resource description filled by the platform glue (fb driver). */
struct esp32p4_dsi_config {
	phys_addr_t fb_phys;	/* physical address of the RGB565 framebuffer */
	u32 width;		/* active width in pixels */
	u32 height;		/* active height in lines */
};

struct esp32p4_dsi *esp32p4_dsi_create(struct device *dev,
				       const struct esp32p4_dsi_config *cfg);
int esp32p4_dsi_probe(struct esp32p4_dsi *dsi);
int esp32p4_dsi_start(struct esp32p4_dsi *dsi);
/* Release the panel reset line and set brightness after the stream is on. */
int esp32p4_dsi_late_init(struct esp32p4_dsi *dsi);

/*
 * The IRQ below is the DW-GDMA channel interrupt, which is shared by all
 * DW-GDMA channels: request it with IRQF_SHARED.  Both rearm() and irq() are
 * safe to call from hard IRQ context and never sleep.
 *
 * rearm()/irq() return 1 when a frame-done interrupt was handled and the
 * channel was re-armed, 0 when the interrupt was not ours, and a negative
 * errno after a fatal VDMA error (the stream is stopped in that case).
 */
int esp32p4_dsi_rearm(struct esp32p4_dsi *dsi);
void esp32p4_dsi_stop(struct esp32p4_dsi *dsi);
int esp32p4_dsi_irq(struct esp32p4_dsi *dsi);
unsigned int esp32p4_dsi_irq_number(struct esp32p4_dsi *dsi);
void esp32p4_dsi_destroy(struct esp32p4_dsi *dsi);

#endif /* _VIDEO_ESP32P4_DSI_H */
