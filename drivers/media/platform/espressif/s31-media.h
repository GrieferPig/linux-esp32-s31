/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _S31_MEDIA_H
#define _S31_MEDIA_H
#include <linux/clk.h>
#include <linux/kref.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/workqueue.h>
#include <linux/soc/espressif/esp32s31-dma2d.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

struct s31_media;
struct s31_fill;
struct s31_blend;
struct s31_blend_image {
	const struct v4l2_pix_format *format;
	const struct v4l2_rect *rect;
	struct vb2_v4l2_buffer *buffer;
	u8 alpha;
};
struct s31_media_ctx {
	struct v4l2_fh fh;
	struct s31_media *dev;
	struct v4l2_pix_format fmt[2]; /* output/source, capture/destination */
	struct v4l2_rect rect[2]; /* source crop, destination compose */
	struct v4l2_ctrl_handler controls;
	struct v4l2_ctrl *quality, *rotate, *hflip, *vflip, *alpha;
	struct v4l2_ctrl *colorfx, *fill_rgb;
	struct work_struct work;
	u32 sequence[2];
	bool aborting;
};
struct s31_media {
	struct device *dev;
	void __iomem *regs, *memory;
	struct clk *clk;
	struct reset_control *reset;
	struct s31_dma2d *dma;
	struct v4l2_device v4l2;
	struct video_device video;
	struct v4l2_m2m_dev *m2m;
	struct s31_fill *fill;
	struct s31_blend *blend;
	struct mutex lock;
	struct mutex hardware_lock;
	struct kref refs;
	bool removed;
	bool jpeg;
};

static inline bool s31_media_buffers_overlap(struct vb2_v4l2_buffer *a, size_t a_size,
					     struct vb2_v4l2_buffer *b, size_t b_size)
{
	u64 a_start = vb2_dma_contig_plane_dma_addr(&a->vb2_buf, 0);
	u64 b_start = vb2_dma_contig_plane_dma_addr(&b->vb2_buf, 0);

	return a_start < b_start + b_size && b_start < a_start + a_size;
}

void s31_media_get(struct s31_media *d);
void s31_media_put(struct s31_media *d);
void s31_normalize_format(struct s31_media *d, struct v4l2_pix_format *f);
int s31_enum_format(struct s31_media *d, struct v4l2_fmtdesc *f);
int s31_select_axis(s32 *position, u32 *size, u32 limit, u32 alignment, u32 flags);
int s31_fill_register(struct s31_media *d);
void s31_fill_unregister(struct s31_media *d);
int s31_blend_register(struct s31_media *d);
void s31_blend_unregister(struct s31_media *d);
int s31_ppa_blend_validate(const struct s31_blend_image image[3]);
int s31_ppa_blend_run(struct s31_media *d, const struct s31_blend_image image[3]);
int s31_ppa_fill_image(struct s31_media *d, const struct v4l2_pix_format *f,
		       const struct v4l2_rect *compose, u32 rgb, u32 alpha,
		       struct vb2_v4l2_buffer *dst, size_t *bytes);
int s31_jpeg_run(struct s31_media_ctx *ctx, struct vb2_v4l2_buffer *src,
		 struct vb2_v4l2_buffer *dst, size_t *bytes);
int s31_ppa_run(struct s31_media_ctx *ctx, struct vb2_v4l2_buffer *src,
		struct vb2_v4l2_buffer *dst, size_t *bytes);
#endif
