/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_ESP_PPA_H
#define _UAPI_LINUX_ESP_PPA_H

#include <linux/ioctl.h>
#include <linux/types.h>

/* Alpha override modes matching TRM PPA_BLEND_FIX_ALPHA_REG. */
#define ESP_PPA_ALPHA_NO_CHANGE	0
#define ESP_PPA_ALPHA_FIX_VALUE	1
#define ESP_PPA_ALPHA_SCALE	2
#define ESP_PPA_ALPHA_INVERT	3

struct esp_ppa_fill_req {
	__u64 dst_ptr;		/* user pointer to destination image buffer */
	__u32 pixfmt;		/* V4L2_PIX_FMT_RGB565 / RGB24 / ABGR32 / GREY */
	__u32 color;		/* 0xAARRGGBB for RGB/ARGB formats (IDF fill_argb_color), or gray8 */
	__u16 width;		/* full image width in pixels */
	__u16 height;		/* full image height in pixels */
	__u16 x;		/* ROI x offset in pixels */
	__u16 y;		/* ROI y offset in pixels */
	__u16 block_w;		/* ROI width in pixels */
	__u16 block_h;		/* ROI height in pixels */
};

struct esp_ppa_blend_req {
	__u64 bg_ptr;		/* user pointer to background image buffer */
	__u64 fg_ptr;		/* user pointer to foreground image buffer */
	__u64 dst_ptr;		/* user pointer to output image buffer */
	__u32 bg_pixfmt;	/* V4L2_PIX_FMT_RGB565 / RGB24 / ABGR32 */
	__u32 fg_pixfmt;	/* V4L2_PIX_FMT_RGB565 / RGB24 / ABGR32 */
	__u32 dst_pixfmt;	/* V4L2_PIX_FMT_RGB565 / RGB24 / ABGR32 */
	__u16 width;		/* image/block width in pixels */
	__u16 height;		/* image/block height in pixels */
	__u8 bg_alpha_mode;	/* ESP_PPA_ALPHA_* */
	__u8 bg_alpha;		/* 0..255 */
	__u8 fg_alpha_mode;	/* ESP_PPA_ALPHA_* */
	__u8 fg_alpha;		/* 0..255 */
};

#define ESP_PPA_IOC_FILL	_IOW('P', 0x01, struct esp_ppa_fill_req)
#define ESP_PPA_IOC_BLEND	_IOW('P', 0x02, struct esp_ppa_blend_req)

#endif /* _UAPI_LINUX_ESP_PPA_H */
