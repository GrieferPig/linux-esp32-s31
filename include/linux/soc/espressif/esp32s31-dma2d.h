/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_ESP32S31_DMA2D_H
#define _LINUX_ESP32S31_DMA2D_H
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/soc/espressif/regs/dma2d-regs.h>

struct s31_dma2d;
struct s31_dma2d_desc {
	__le32 words[6];
} __aligned(8);
struct s31_dma2d_job {
	struct s31_dma2d_desc tx, rx, foreground;
	u8 peripheral; /* 0 JPEG, 1 PPA SRM, 2 PPA fill/blend, 7 memory */
	u8 tx_macroblock; /* same hardware encoding as rx_macroblock */
	u8 rx_macroblock; /* 0:8x8, 1:8x16, 2:16x16, 3:none */
	bool tx_reorder, rx_reorder, descriptor_port, rx_only, blend;
	u16 block_w, block_h;
	void (*configure)(void *arg, dma2d_dev_t __iomem *dma);
	void (*start)(void *arg);
	void (*stop)(void *arg);
	void *arg;
	size_t received;
	bool cancelled;
};
struct s31_dma2d *s31_dma2d_get(struct device *consumer);
struct device *s31_dma2d_device(struct s31_dma2d *dma);
int s31_dma2d_transfer(struct s31_dma2d *dma, struct s31_dma2d_job *job);
void s31_dma2d_desc_linear(struct s31_dma2d_desc *desc, dma_addr_t buffer,
			   size_t length, bool tx);
void s31_dma2d_desc_image(struct s31_dma2d_desc *desc, dma_addr_t buffer,
			  u16 width, u16 height, u16 block_w, u16 block_h,
			  u8 pbyte, bool multiple);
#endif
