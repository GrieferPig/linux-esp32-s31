// SPDX-License-Identifier: GPL-2.0-only
#include <linux/soc/espressif/esp32s31-regfield.h>
#include <linux/soc/espressif/regs/ppa-regs.h>
#include "s31-media.h"

struct s31_ppa_run {
	struct s31_media_ctx *ctx;
	u32 rotate, hflip, vflip, scale_x, scale_y;
};
static bool s31_ppa_has_alpha(u32 fourcc)
{
	return fourcc == V4L2_PIX_FMT_ABGR32 || fourcc == V4L2_PIX_FMT_RGBA32;
}
static bool s31_ppa_red_first(u32 fourcc)
{
	return fourcc == V4L2_PIX_FMT_RGB24 || fourcc == V4L2_PIX_FMT_RGBA32;
}
static u32 s31_ppa_color(u32 fourcc)
{
	switch (fourcc) {
	case V4L2_PIX_FMT_ABGR32:
	case V4L2_PIX_FMT_RGBA32:
		return 0;
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_BGR24:
		return 1;
	case V4L2_PIX_FMT_RGB565:
		return 2;
	case V4L2_PIX_FMT_UYVY:
		return 9;
	default:
		return 12;
	}
}
static u32 s31_ppa_pbyte(u32 fourcc)
{
	switch (fourcc) {
	case V4L2_PIX_FMT_ABGR32:
	case V4L2_PIX_FMT_RGBA32:
		return 5;
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_BGR24:
		return 4;
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_UYVY:
		return 3;
	default:
		return 1;
	}
}
static void s31_ppa_configure(void *arg, dma2d_dev_t __iomem *dma)
{
	struct s31_ppa_run *run = arg;
	struct s31_media_ctx *ctx = run->ctx;
	ppa_dev_t __iomem *h = ctx->dev->regs;

	s31_reg_set(h, srm_scal_rotate, scal_rotate_rst, 1);
	s31_reg_set(h, srm_scal_rotate, scal_rotate_rst, 0);
}
static void s31_ppa_start(void *arg)
{
	struct s31_ppa_run *run = arg;
	struct s31_media_ctx *ctx = run->ctx;
	ppa_dev_t __iomem *h = ctx->dev->regs;
	ppa_srm_scal_rotate_reg_t scale = {};
	ppa_srm_color_mode_reg_t color = {};
	ppa_srm_byte_order_reg_t order = {};

	color.srm_rx_cm = s31_ppa_color(ctx->fmt[0].pixelformat);
	color.srm_tx_cm = s31_ppa_color(ctx->fmt[1].pixelformat);
	color.yuv_rx_range = 1;
	color.yuv_tx_range = 1;
	writel(color.val, &h->srm_color_mode.val);
	/* SRM transforms RGB channels independently. Account for both memory
	 * layouts at its input instead of adding a receive-side DMA CSC stage.
	 * Gray has identical RGB components and needs no channel swap.
	 */
	order.srm_rx_rgb_swap_en =
		ctx->fmt[0].pixelformat != V4L2_PIX_FMT_GREY &&
		(s31_ppa_red_first(ctx->fmt[0].pixelformat) ^
		 s31_ppa_red_first(ctx->fmt[1].pixelformat));
	order.srm_bk_size_sel = 0; /* Native SDK's 32x32 blocks. */
	writel(order.val, &h->srm_byte_order.val);
	/* Preserve an input alpha channel. Otherwise the standard alpha control
	 * supplies the component for a destination with alpha.
	 */
	writel(s31_ppa_has_alpha(ctx->fmt[1].pixelformat) &&
		       !s31_ppa_has_alpha(ctx->fmt[0].pixelformat) ?
		       BIT(8) | v4l2_ctrl_g_ctrl(ctx->alpha) : 0,
	       &h->srm_fix_alpha.val);
	/* BT.601 gray coefficients sum to 256. */
	writel(29 | (150 << 8) | (77 << 16), &h->rgb2gray.val);
	scale.srm_scal_x_int = run->scale_x >> 4;
	scale.srm_scal_x_frag = run->scale_x & 15;
	scale.srm_scal_y_int = run->scale_y >> 4;
	scale.srm_scal_y_frag = run->scale_y & 15;
	scale.srm_rotate_angle = run->rotate;
	scale.srm_mirror_x = run->hflip;
	scale.srm_mirror_y = run->vflip;
	writel(scale.val, &h->srm_scal_rotate.val);
	s31_reg_set(h, srm_scal_rotate, scal_rotate_start, 1);
}
static void s31_ppa_stop(void *arg)
{
	struct s31_ppa_run *run = arg;
	ppa_dev_t __iomem *h = run->ctx->dev->regs;

	s31_reg_set(h, srm_scal_rotate, scal_rotate_rst, 1);
	s31_reg_set(h, srm_scal_rotate, scal_rotate_rst, 0);
}

struct s31_ppa_fill {
	struct s31_media *dev;
	const struct v4l2_pix_format *format;
	const struct v4l2_rect *compose;
	u32 pixel;
};

static void s31_ppa_fill_configure(void *arg, dma2d_dev_t __iomem *dma)
{
	struct s31_ppa_fill *fill = arg;
	ppa_dev_t __iomem *h = fill->dev->regs;

	s31_reg_set(h, blend_trans_mode, blend_rst, 1);
	s31_reg_set(h, blend_trans_mode, blend_rst, 0);
}

static void s31_ppa_fill_start(void *arg)
{
	struct s31_ppa_fill *fill = arg;
	ppa_dev_t __iomem *h = fill->dev->regs;
	ppa_blend_color_mode_reg_t color = {};
	ppa_blend_tx_size_reg_t size = {};
	ppa_blend_trans_mode_reg_t mode = {};

	color.blend_tx_cm = s31_ppa_color(fill->format->pixelformat);
	writel(color.val, &h->blend_color_mode.val);
	writel(fill->pixel, &h->blend_fix_pixel.val);
	size.blend_hb = fill->compose->width;
	size.blend_vb = fill->compose->height;
	writel(size.val, &h->blend_tx_size.val);
	/* Latch fill mode before enabling, then commit the transaction. Combining
	 * these writes leaves the S31 fill engine waiting without producing data.
	 */
	mode.blend_fix_pixel_fill_en = 1;
	writel(mode.val, &h->blend_trans_mode.val);
	mode.blend_en = 1;
	writel(mode.val, &h->blend_trans_mode.val);
	mode.blend_trans_mode_update = 1;
	writel(mode.val, &h->blend_trans_mode.val);
}

static void s31_ppa_fill_stop(void *arg)
{
	struct s31_ppa_fill *fill = arg;
	ppa_dev_t __iomem *h = fill->dev->regs;

	s31_reg_set(h, blend_trans_mode, blend_rst, 1);
	s31_reg_set(h, blend_trans_mode, blend_rst, 0);
}

int s31_ppa_fill_image(struct s31_media *d, const struct v4l2_pix_format *f,
		       const struct v4l2_rect *compose, u32 rgb, u32 alpha,
		       struct vb2_v4l2_buffer *dst, size_t *bytes)
{
	struct s31_ppa_fill fill = { .dev = d, .format = f, .compose = compose };
	struct s31_dma2d_job job = { .peripheral = 2,
		.tx_macroblock = 3, .rx_macroblock = 3, .rx_only = true,
		.configure = s31_ppa_fill_configure, .start = s31_ppa_fill_start,
		.stop = s31_ppa_fill_stop, .arg = &fill };
	u32 format = f->pixelformat;
	s32 r = (rgb >> 16) & 255, g = (rgb >> 8) & 255, b = rgb & 255;
	int ret;

	switch (format) {
	case V4L2_PIX_FMT_RGB565:
		/* The fill register takes ARGB8888 even for RGB565 output. */
		fill.pixel = rgb;
		break;
	case V4L2_PIX_FMT_GREY:
		fill.pixel = ((77 * r + 150 * g + 29 * b) >> 8) * 0x010101;
		break;
	case V4L2_PIX_FMT_UYVY: {
		s32 y = (77 * r + 150 * g + 29 * b) >> 8;
		s32 u = clamp_t(s32, ((-43 * r - 85 * g + 128 * b) >> 8) + 128,
				0, 255);
		s32 v = clamp_t(s32, ((128 * r - 107 * g - 21 * b) >> 8) + 128,
				0, 255);

		fill.pixel = u | (y << 8) | (v << 16) | (y << 24);
		break;
	}
	default:
		fill.pixel = s31_ppa_red_first(format) ?
			r | (g << 8) | (b << 16) : b | (g << 8) | (r << 16);
		if (s31_ppa_has_alpha(format))
			fill.pixel |= alpha << 24;
		break;
	}
	s31_dma2d_desc_image(&job.rx,
			     vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0),
			     f->width, f->height,
			     compose->width, compose->height, s31_ppa_pbyte(format), false);
	job.rx.words[2] = cpu_to_le32(compose->top | (compose->left << 14));
	ret = s31_dma2d_transfer(d->dma, &job);
	if (!ret)
		*bytes = f->sizeimage;
	return ret;
}

static int s31_ppa_fill_run(struct s31_media_ctx *ctx,
			    struct vb2_v4l2_buffer *dst, size_t *bytes)
{
	u32 format = ctx->fmt[1].pixelformat;
	/* SET_RGB changes only RGB. Receive-only fill cannot retain a varying
	 * alpha component from a 32-bit source image.
	 */
	if (s31_ppa_has_alpha(format) &&
	    s31_ppa_has_alpha(ctx->fmt[0].pixelformat))
		return -EINVAL;

	return s31_ppa_fill_image(ctx->dev, &ctx->fmt[1], &ctx->rect[1],
			  v4l2_ctrl_g_ctrl(ctx->fill_rgb),
			  v4l2_ctrl_g_ctrl(ctx->alpha), dst, bytes);
}

struct s31_ppa_blend_run {
	struct s31_media *dev;
	const struct s31_blend_image *image;
};

int s31_ppa_blend_validate(const struct s31_blend_image image[3])
{
	u32 out = image[2].format->pixelformat;
	unsigned int i;

	if (image[1].format->pixelformat == V4L2_PIX_FMT_GREY ||
	    image[1].format->pixelformat == V4L2_PIX_FMT_UYVY)
		return -EINVAL;
	for (i = 0; i < 2; i++) {
		u32 in = image[i].format->pixelformat;

		if (image[i].rect->width != image[2].rect->width ||
		    image[i].rect->height != image[2].rect->height)
			return -EPIPE;
		/* The four-byte swap also moves alpha. Never claim to retain
		 * per-pixel alpha while exchanging only red and blue channels.
		 */
		if ((s31_ppa_has_alpha(in) || in == V4L2_PIX_FMT_UYVY) &&
		    (s31_ppa_red_first(in) ^ s31_ppa_red_first(out)))
			return -EINVAL;
	}
	return 0;
}

static void s31_ppa_blend_configure(void *arg, dma2d_dev_t __iomem *dma)
{
	struct s31_ppa_blend_run *run = arg;
	ppa_dev_t __iomem *h = run->dev->regs;

	s31_reg_set(h, blend_trans_mode, blend_rst, 1);
	s31_reg_set(h, blend_trans_mode, blend_rst, 0);
}

static void s31_ppa_blend_start(void *arg)
{
	struct s31_ppa_blend_run *run = arg;
	const struct s31_blend_image *im = run->image;
	ppa_dev_t __iomem *h = run->dev->regs;
	ppa_blend_color_mode_reg_t color = {};
	ppa_blend_byte_order_reg_t order = {};
	ppa_blend_fix_alpha_reg_t alpha = {};
	ppa_blend_tx_size_reg_t size = {};
	ppa_blend_trans_mode_reg_t mode = {};
	u32 bg = im[0].format->pixelformat, fg = im[1].format->pixelformat;
	u32 out = im[2].format->pixelformat;

	color.blend0_rx_cm = s31_ppa_color(bg);
	color.blend1_rx_cm = s31_ppa_color(fg);
	color.blend_tx_cm = s31_ppa_color(out);
	color.blend0_rx_yuv_range = 1;
	color.blend_tx_yuv_range = 1;
	writel(color.val, &h->blend_color_mode.val);
	order.blend0_rx_rgb_swap_en = bg != V4L2_PIX_FMT_GREY &&
		(s31_ppa_red_first(bg) ^ s31_ppa_red_first(out));
	order.blend1_rx_rgb_swap_en =
		s31_ppa_red_first(fg) ^ s31_ppa_red_first(out);
	writel(order.val, &h->blend_byte_order.val);
	alpha.blend0_rx_fix_alpha = im[0].alpha;
	alpha.blend1_rx_fix_alpha = im[1].alpha;
	/* Preserve per-pixel alpha at full opacity. Otherwise the hardware
	 * scales it by alpha/256. Sources without alpha use the fixed value.
	 */
	alpha.blend0_rx_alpha_mod = s31_ppa_has_alpha(bg) ?
		(im[0].alpha == 255 ? 0 : 2) : 1;
	alpha.blend1_rx_alpha_mod = s31_ppa_has_alpha(fg) ?
		(im[1].alpha == 255 ? 0 : 2) : 1;
	writel(alpha.val, &h->blend_fix_alpha.val);
	writel(29 | (150 << 8) | (77 << 16), &h->rgb2gray.val);
	/* Disable both color-key ranges by setting low above high. */
	writel(0xffffff, &h->ck_fg_low.val);
	writel(0, &h->ck_fg_high.val);
	writel(0xffffff, &h->ck_bg_low.val);
	writel(0, &h->ck_bg_high.val);
	writel(0, &h->ck_default.val);
	size.blend_hb = im[2].rect->width;
	size.blend_vb = im[2].rect->height;
	writel(size.val, &h->blend_tx_size.val);
	writel(mode.val, &h->blend_trans_mode.val);
	mode.blend_en = 1;
	writel(mode.val, &h->blend_trans_mode.val);
	mode.blend_trans_mode_update = 1;
	writel(mode.val, &h->blend_trans_mode.val);
}

static void s31_ppa_blend_stop(void *arg)
{
	struct s31_ppa_blend_run *run = arg;
	ppa_dev_t __iomem *h = run->dev->regs;

	s31_reg_set(h, blend_trans_mode, blend_rst, 1);
	s31_reg_set(h, blend_trans_mode, blend_rst, 0);
}

int s31_ppa_blend_run(struct s31_media *d, const struct s31_blend_image image[3])
{
	struct s31_ppa_blend_run run = { .dev = d, .image = image };
	struct s31_dma2d_job job = { .peripheral = 2, .blend = true,
		.tx_macroblock = 3, .rx_macroblock = 3,
		.configure = s31_ppa_blend_configure, .start = s31_ppa_blend_start,
		.stop = s31_ppa_blend_stop, .arg = &run };
	struct s31_dma2d_desc *desc[] = { &job.tx, &job.foreground, &job.rx };
	unsigned int i;
	int ret = s31_ppa_blend_validate(image);

	if (ret)
		return ret;
	/* DMA reads and writes run concurrently. An imported output must not
	 * alias either source canvas, even when the three queue owners differ.
	 */
	for (i = 0; i < 2; i++) {
		if (s31_media_buffers_overlap(image[i].buffer, image[i].format->sizeimage,
					      image[2].buffer, image[2].format->sizeimage))
			return -EINVAL;
	}
	for (i = 0; i < 3; i++) {
		const struct v4l2_rect *r = image[i].rect;
		const struct v4l2_pix_format *f = image[i].format;

		dma_addr_t address =
			vb2_dma_contig_plane_dma_addr(&image[i].buffer->vb2_buf, 0);

		s31_dma2d_desc_image(desc[i], address, f->width, f->height,
				     r->width, r->height,
				     s31_ppa_pbyte(f->pixelformat), false);
		desc[i]->words[2] = cpu_to_le32(r->top | (r->left << 14));
	}
	return s31_dma2d_transfer(d->dma, &job);
}

int s31_ppa_run(struct s31_media_ctx *ctx, struct vb2_v4l2_buffer *src,
		struct vb2_v4l2_buffer *dst, size_t *bytes)
{
	struct s31_ppa_run run = { .ctx = ctx };
	struct s31_dma2d_job job = { .peripheral = 1,
				     .tx_macroblock = 3,
				     .rx_macroblock = 3,
				     .descriptor_port = true,
				     .configure = s31_ppa_configure,
				     .start = s31_ppa_start,
				     .stop = s31_ppa_stop,
				     .arg = &run };
	const struct v4l2_rect *crop = &ctx->rect[0];
	const struct v4l2_rect *compose = &ctx->rect[1];
	u32 w = compose->width, h = compose->height;
	int ret;

	if (v4l2_ctrl_g_ctrl(ctx->colorfx) == V4L2_COLORFX_SET_RGB)
		return s31_ppa_fill_run(ctx, dst, bytes);
	/* PPA's input RGB swap does not apply to YUV. Applications can request
	 * native BGR24 for this conversion; do not enter the unsupported DMA
	 * receive-side shuffle while using the PPA descriptor port.
	 */
	if (ctx->fmt[0].pixelformat == V4L2_PIX_FMT_UYVY &&
	    s31_ppa_red_first(ctx->fmt[1].pixelformat))
		return -EINVAL;
	/* The 32-bit input swap reverses all four bytes, including alpha. It
	 * cannot exchange red/blue while retaining the alpha component. Keep
	 * alpha-bearing sources in their input channel order rather than
	 * silently discarding a color channel during format conversion.
	 */
	if (s31_ppa_has_alpha(ctx->fmt[0].pixelformat) &&
	    (s31_ppa_red_first(ctx->fmt[0].pixelformat) ^
	     s31_ppa_red_first(ctx->fmt[1].pixelformat)))
		return -EINVAL;
	run.rotate = v4l2_ctrl_g_ctrl(ctx->rotate) / 90;
	run.hflip = v4l2_ctrl_g_ctrl(ctx->hflip);
	run.vflip = v4l2_ctrl_g_ctrl(ctx->vflip);
	if (run.rotate & 1)
		swap(w, h);
	run.scale_x = DIV_ROUND_CLOSEST(w * 16, crop->width);
	run.scale_y = DIV_ROUND_CLOSEST(h * 16, crop->height);
	if (!run.scale_x || !run.scale_y || run.scale_x > 4095 ||
	    run.scale_y > 4095 || (crop->width * run.scale_x) / 16 != w ||
	    (crop->height * run.scale_y) / 16 != h)
		return -EINVAL;
	job.block_w = ctx->fmt[0].pixelformat == V4L2_PIX_FMT_UYVY ? 36 : 34;
	job.block_h = 34;
	s31_dma2d_desc_image(&job.tx,
			     vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0),
			     ctx->fmt[0].width, ctx->fmt[0].height,
			     crop->width, crop->height,
			     s31_ppa_pbyte(ctx->fmt[0].pixelformat), false);
	s31_dma2d_desc_image(
		&job.rx, vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0),
		ctx->fmt[1].width, ctx->fmt[1].height, 2, 2, 0,
		false); /* PPA supplies the actual receive block and pixel size. */
	job.tx.words[2] = cpu_to_le32(crop->top | (crop->left << 14));
	job.rx.words[2] = cpu_to_le32(compose->top | (compose->left << 14));
	ret = s31_dma2d_transfer(ctx->dev->dma, &job);
	if (!ret)
		*bytes = ctx->fmt[1].sizeimage;
	return ret;
}
