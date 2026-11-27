// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S31 V4L2 M2M JPEG encoder and decoder (grayscale and color).
 * S31 TRM ch.35 JPEG Codec, ch.6 2D-DMA, and ESP-IDF esp_driver_jpeg /
 * esp_hal_jpeg/esp32s31/jpeg_ll.h.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/sizes.h>
#include <linux/workqueue.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-jpeg.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "../../../dma/espressif-dma2d.h"

/* S31 TRM ch.35 JPEG Codec / IDF soc/esp32s31/register/soc/jpeg_reg.h. */
#define JPEG_CONFIG		0x00
#define JPEG_DQT_INFO		0x04
#define JPEG_PIC_SIZE		0x08
#define JPEG_T0QNR		0x10
#define JPEG_T1QNR		0x14
#define JPEG_T2QNR		0x18
#define JPEG_T3QNR		0x1c
#define JPEG_DEC_CONF		0x20
#define JPEG_C0			0x24
#define JPEG_C1			0x28
#define JPEG_C2			0x2c
#define JPEG_DHT_INFO		0x34
#define JPEG_IRQ_ENA		0x3c
#define JPEG_IRQ_ST		0x40
#define JPEG_IRQ_CLR		0x44
#define JPEG_DC0_COUNT		0x58
#define JPEG_AC0_COUNT		0x60
#define JPEG_DC1_COUNT		0x68
#define JPEG_AC1_COUNT		0x70
#define JPEG_DC0_MIN		0x78
#define JPEG_AC0_MIN		0x7c
#define JPEG_DC1_MIN		0x80
#define JPEG_AC1_MIN		0x84

#define JPEG_START		BIT(1)
#define JPEG_FF_CHECK_EN	BIT(3)
#define JPEG_SAMPLE_SEL		GENMASK(5, 4)
#define JPEG_QNR_FIFO_EN	BIT(7)
#define JPEG_LQNR_TBL_SEL	GENMASK(9, 8)
#define JPEG_CQNR_TBL_SEL	GENMASK(11, 10)
#define JPEG_COLOR_SPACE	GENMASK(14, 12)
#define JPEG_DHT_FIFO_EN	BIT(15)
#define JPEG_DEC_TIMEOUT_THRES	GENMASK(22, 17)
#define JPEG_DEC_TIMEOUT_DEF	FIELD_PREP(JPEG_DEC_TIMEOUT_THRES, 32)
#define JPEG_SOFT_RST		BIT(24)
#define JPEG_TAILER_EN		BIT(27)
#define JPEG_DECODE		BIT(31)

#define JPEG_DONE		BIT(0)
#define JPEG_ENC_ERRORS		(BIT(1) | BIT(16))
#define JPEG_DEC_ERRORS		(GENMASK(8, 2) | GENMASK(15, 13) | GENMASK(24, 18))
#define JPEG_DEC_IRQS		(JPEG_DONE | JPEG_DEC_ERRORS)
#define JPEG_ENC_IRQS		(JPEG_ENC_ERRORS)

#define JPEG_TIMEOUT_MS		500
#define JPEG_MIN_EDGE		8
#define JPEG_MAX_EDGE		1024
#define JPEG_MAX_BS		SZ_2M

/* Standard JPEG Annex K.1 / K.2 quantization and K.3 / K.4 / K.5 Huffman tables. */
static const u8 zigzag_order[64] = {
	 0,  1,  8, 16,  9,  2,  3, 10,
	17, 24, 32, 25, 18, 11,  4,  5,
	12, 19, 26, 33, 40, 48, 41, 34,
	27, 20, 13,  6,  7, 14, 21, 28,
	35, 42, 49, 56, 57, 50, 43, 36,
	29, 22, 15, 23, 30, 37, 44, 51,
	58, 59, 52, 45, 38, 31, 39, 46,
	53, 60, 61, 54, 47, 55, 62, 63,
};

static const u8 std_luma_qtable[64] = {
	16, 11, 10, 16, 24, 40, 51, 61,
	12, 12, 14, 19, 26, 58, 60, 55,
	14, 13, 16, 24, 40, 57, 69, 56,
	14, 17, 22, 29, 51, 87, 80, 62,
	18, 22, 37, 56, 68, 109, 103, 77,
	24, 35, 55, 64, 81, 104, 113, 92,
	49, 64, 78, 87, 103, 121, 120, 101,
	72, 92, 95, 98, 112, 100, 103, 99,
};

static const u8 std_chroma_qtable[64] = {
	17, 18, 24, 47, 99, 99, 99, 99,
	18, 21, 26, 66, 99, 99, 99, 99,
	24, 26, 56, 99, 99, 99, 99, 99,
	47, 66, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
};

static const u8 std_dc_luma[16 + 12] = {
	0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
};

static const u8 std_dc_chroma[16 + 12] = {
	0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
};

static const u8 std_ac_luma[16 + 162] = {
	0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d,
	0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
	0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
	0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
	0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
	0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
	0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
	0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
	0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
	0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
	0xf9, 0xfa,
};

static const u8 std_ac_chroma[16 + 162] = {
	0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77,
	0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
	0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
	0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
	0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
	0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
	0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
	0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
	0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
	0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
	0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
	0xf9, 0xfa,
};

static const u32 s31_jpeg_formats[] = {
	V4L2_PIX_FMT_JPEG,
	V4L2_PIX_FMT_GREY,
	V4L2_PIX_FMT_RGB565,
	V4L2_PIX_FMT_RGB24,
};

struct s31_jpeg {
	void __iomem *base;
	struct device *dev;
	struct v4l2_device v4l2;
	struct video_device video;
	struct v4l2_m2m_dev *m2m;
	struct mutex lock;
	struct work_struct work;
	struct s31_jpeg_ctx *running;
	struct completion codec_done;
	struct completion dma_done;
	enum dmaengine_tx_result dma_result;
	u32 codec_status;
	int irq;
	bool disconnected;
};

struct s31_jpeg_ctx {
	struct v4l2_fh fh;
	struct v4l2_ctrl_handler ctrls;
	struct s31_jpeg *jpeg;
	struct v4l2_pix_format source;
	struct v4l2_pix_format target;
	u32 quality;
};

static struct s31_jpeg_ctx *to_s31_ctx(struct file *file)
{
	return container_of(file->private_data, struct s31_jpeg_ctx, fh);
}

static struct v4l2_pix_format *s31_jpeg_fmt(struct s31_jpeg_ctx *ctx,
					    enum v4l2_buf_type type)
{
	return V4L2_TYPE_IS_OUTPUT(type) ? &ctx->source : &ctx->target;
}

static u32 s31_jpeg_bpp(u32 fourcc)
{
	switch (fourcc) {
	case V4L2_PIX_FMT_GREY:
		return 1;
	case V4L2_PIX_FMT_RGB565:
		return 2;
	case V4L2_PIX_FMT_RGB24:
		return 3;
	default:
		return 0;
	}
}

static void s31_jpeg_bound_fmt(struct v4l2_pix_format *pix, bool output)
{
	u32 bpp;

	if (pix->pixelformat != V4L2_PIX_FMT_JPEG &&
	    pix->pixelformat != V4L2_PIX_FMT_GREY &&
	    pix->pixelformat != V4L2_PIX_FMT_RGB565 &&
	    pix->pixelformat != V4L2_PIX_FMT_RGB24)
		pix->pixelformat = output ? V4L2_PIX_FMT_JPEG : V4L2_PIX_FMT_RGB565;

	pix->width = round_up(clamp(pix->width, (u32)JPEG_MIN_EDGE, (u32)JPEG_MAX_EDGE), 8);
	pix->height = round_up(clamp(pix->height, (u32)JPEG_MIN_EDGE, (u32)JPEG_MAX_EDGE), 8);
	pix->field = V4L2_FIELD_NONE;
	bpp = s31_jpeg_bpp(pix->pixelformat);
	if (bpp) {
		pix->bytesperline = pix->width * bpp;
		pix->sizeimage = ALIGN(pix->bytesperline * pix->height, ARCH_DMA_MINALIGN);
	} else {
		pix->bytesperline = 0;
		pix->sizeimage = ALIGN(clamp(pix->sizeimage ?: pix->width * pix->height * 2 + SZ_4K,
					     (u32)SZ_4K, (u32)JPEG_MAX_BS),
				       ARCH_DMA_MINALIGN);
	}
	pix->colorspace = V4L2_COLORSPACE_JPEG;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static int s31_jpeg_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	struct s31_jpeg *jpeg = video_drvdata(file);

	strscpy(cap->driver, "esp32s31-jpeg", sizeof(cap->driver));
	strscpy(cap->card, jpeg->video.name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s", dev_name(jpeg->dev));
	return 0;
}

static int s31_jpeg_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *fmt)
{
	if (fmt->index >= ARRAY_SIZE(s31_jpeg_formats))
		return -EINVAL;
	fmt->pixelformat = s31_jpeg_formats[fmt->index];
	return 0;
}

static int s31_jpeg_try_fmt(struct file *file, void *fh, struct v4l2_format *fmt)
{
	s31_jpeg_bound_fmt(&fmt->fmt.pix, V4L2_TYPE_IS_OUTPUT(fmt->type));
	return 0;
}

static int s31_jpeg_get_fmt(struct file *file, void *fh, struct v4l2_format *fmt)
{
	fmt->fmt.pix = *s31_jpeg_fmt(to_s31_ctx(file), fmt->type);
	return 0;
}

static int s31_jpeg_set_fmt(struct file *file, void *fh, struct v4l2_format *fmt)
{
	struct s31_jpeg_ctx *ctx = to_s31_ctx(file);

	if (vb2_is_busy(v4l2_m2m_get_vq(ctx->fh.m2m_ctx, fmt->type)))
		return -EBUSY;
	s31_jpeg_bound_fmt(&fmt->fmt.pix, V4L2_TYPE_IS_OUTPUT(fmt->type));
	*s31_jpeg_fmt(ctx, fmt->type) = fmt->fmt.pix;
	return 0;
}

static int s31_jpeg_reqbufs(struct file *file, void *fh, struct v4l2_requestbuffers *req)
{
	if (req->count && req->memory == V4L2_MEMORY_MMAP)
		req->flags |= V4L2_MEMORY_FLAG_NON_COHERENT;
	return v4l2_m2m_ioctl_reqbufs(file, fh, req);
}

static const struct v4l2_ioctl_ops s31_jpeg_ioctls = {
	.vidioc_querycap = s31_jpeg_querycap,
	.vidioc_enum_fmt_vid_cap = s31_jpeg_enum_fmt,
	.vidioc_enum_fmt_vid_out = s31_jpeg_enum_fmt,
	.vidioc_try_fmt_vid_cap = s31_jpeg_try_fmt,
	.vidioc_try_fmt_vid_out = s31_jpeg_try_fmt,
	.vidioc_g_fmt_vid_cap = s31_jpeg_get_fmt,
	.vidioc_g_fmt_vid_out = s31_jpeg_get_fmt,
	.vidioc_s_fmt_vid_cap = s31_jpeg_set_fmt,
	.vidioc_s_fmt_vid_out = s31_jpeg_set_fmt,
	.vidioc_reqbufs = s31_jpeg_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
};

static int s31_jpeg_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s31_jpeg_ctx *ctx = container_of(ctrl->handler, struct s31_jpeg_ctx, ctrls);

	if (ctrl->id == V4L2_CID_JPEG_COMPRESSION_QUALITY) {
		ctx->quality = ctrl->val;
		return 0;
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops s31_jpeg_ctrl_ops = {
	.s_ctrl = s31_jpeg_s_ctrl,
};

static int s31_jpeg_queue_setup(struct vb2_queue *q, unsigned int *buffers,
				unsigned int *planes, unsigned int sizes[],
				struct device *alloc_devs[])
{
	struct s31_jpeg_ctx *ctx = vb2_get_drv_priv(q);
	unsigned int size = s31_jpeg_fmt(ctx, q->type)->sizeimage;

	if (*planes)
		return *planes == 1 && sizes[0] >= size ? 0 : -EINVAL;
	*planes = 1;
	sizes[0] = size;
	return 0;
}

static int s31_jpeg_buf_prepare(struct vb2_buffer *buf)
{
	struct vb2_queue *q = buf->vb2_queue;
	struct s31_jpeg_ctx *ctx = vb2_get_drv_priv(q);
	unsigned int size = s31_jpeg_fmt(ctx, q->type)->sizeimage;
	unsigned int payload = vb2_get_plane_payload(buf, 0);

	if (vb2_plane_size(buf, 0) < size)
		return -EINVAL;
	if (V4L2_TYPE_IS_OUTPUT(q->type) && (payload < 4 || payload > size))
		return -EINVAL;
	return 0;
}

static void s31_jpeg_buf_queue(struct vb2_buffer *buf)
{
	struct s31_jpeg_ctx *ctx = vb2_get_drv_priv(buf->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(buf));
}

static void s31_jpeg_stop_streaming(struct vb2_queue *q)
{
	struct s31_jpeg_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *buf;

	for (;;) {
		buf = V4L2_TYPE_IS_OUTPUT(q->type) ?
		      v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
		      v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!buf)
			break;
		v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
	}
}

static const struct vb2_ops s31_jpeg_vb2_ops = {
	.queue_setup = s31_jpeg_queue_setup,
	.buf_prepare = s31_jpeg_buf_prepare,
	.buf_queue = s31_jpeg_buf_queue,
	.stop_streaming = s31_jpeg_stop_streaming,
};

static int s31_jpeg_queue_init(void *data, struct vb2_queue *src, struct vb2_queue *dst)
{
	struct s31_jpeg_ctx *ctx = data;
	int ret;

	*src = (struct vb2_queue) {
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
		.io_modes = VB2_MMAP,
		.drv_priv = ctx,
		.buf_struct_size = sizeof(struct v4l2_m2m_buffer),
		.ops = &s31_jpeg_vb2_ops,
		.mem_ops = &vb2_dma_contig_memops,
		.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY,
		.lock = &ctx->jpeg->lock,
		.dev = ctx->jpeg->dev,
		.allow_cache_hints = 1,
	};
	*dst = *src;
	dst->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ret = vb2_queue_init(src);
	if (ret)
		return ret;
	return vb2_queue_init(dst);
}

static int s31_jpeg_open(struct file *file)
{
	struct s31_jpeg *jpeg = video_drvdata(file);
	struct s31_jpeg_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->jpeg = jpeg;
	ctx->quality = 80;
	ctx->source.width = 64;
	ctx->source.height = 64;
	ctx->source.pixelformat = V4L2_PIX_FMT_JPEG;
	ctx->target = ctx->source;
	ctx->target.pixelformat = V4L2_PIX_FMT_RGB565;
	s31_jpeg_bound_fmt(&ctx->source, true);
	s31_jpeg_bound_fmt(&ctx->target, false);

	v4l2_ctrl_handler_init(&ctx->ctrls, 1);
	v4l2_ctrl_new_std(&ctx->ctrls, &s31_jpeg_ctrl_ops,
			  V4L2_CID_JPEG_COMPRESSION_QUALITY, 1, 100, 1, 80);
	if (ctx->ctrls.error) {
		ret = ctx->ctrls.error;
		goto err_ctrl;
	}

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	ctx->fh.ctrl_handler = &ctx->ctrls;
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(jpeg->m2m, ctx, s31_jpeg_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		v4l2_fh_exit(&ctx->fh);
		goto err_ctrl;
	}
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh, file);
	return 0;

err_ctrl:
	v4l2_ctrl_handler_free(&ctx->ctrls);
	kfree(ctx);
	return ret;
}

static int s31_jpeg_release(struct file *file)
{
	struct s31_jpeg_ctx *ctx = to_s31_ctx(file);

	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	v4l2_ctrl_handler_free(&ctx->ctrls);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations s31_jpeg_fops = {
	.owner = THIS_MODULE,
	.open = s31_jpeg_open,
	.release = s31_jpeg_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static void s31_jpeg_write_huff(void __iomem *base, const u8 *table,
				u32 count_reg, u32 min_reg)
{
	unsigned int i, total = 0, code = 0;

	for (i = 0; i < 16; i++) {
		total += table[i];
		writel(total, base + count_reg);
		writel(table[i] ? code << (15 - i) : 0xffff, base + min_reg);
		code = (code + table[i]) << 1;
	}
	for (i = 0; i < total; i++)
		writel(table[16 + i], base + count_reg + 4);
}

static int s31_jpeg_parse_dht(const struct v4l2_jpeg_header *header,
			      const u8 *tables[4])
{
	unsigned int i;

	if (header->num_dht > ARRAY_SIZE(header->dht))
		return -EOPNOTSUPP;
	for (i = 0; i < header->num_dht; i++) {
		const struct v4l2_jpeg_reference *ref = &header->dht[i];
		const u8 *p = ref->start + 2, *end = ref->start + ref->length;

		while (p < end) {
			unsigned int n, total = 0, code = 0;
			u8 id;

			if (end - p < 17)
				return -EINVAL;
			id = *p++;
			if (id & ~0x11)
				return -EINVAL;
			for (n = 0; n < 16; n++) {
				total += p[n];
				code += p[n];
				if (code >= (1U << (n + 1)))
					return -EINVAL;
				code <<= 1;
			}
			if (!total || total > (id & 0x10 ? 256 : 16) ||
			    total > end - p - 16)
				return -EINVAL;
			tables[(id >> 3) | (id & 1)] = p;
			p += 16 + total;
		}
	}
	return 0;
}

static irqreturn_t s31_jpeg_irq(int irq, void *data)
{
	struct s31_jpeg *jpeg = data;
	u32 status = readl(jpeg->base + JPEG_IRQ_ST);

	if (!status)
		return IRQ_NONE;
	jpeg->codec_status |= status;
	writel(status, jpeg->base + JPEG_IRQ_CLR);
	complete(&jpeg->codec_done);
	if (status & (JPEG_DEC_ERRORS | JPEG_ENC_ERRORS))
		complete(&jpeg->dma_done);
	return IRQ_HANDLED;
}

static void s31_jpeg_dma_cb(void *data, const struct dmaengine_result *result)
{
	struct s31_jpeg *jpeg = data;

	jpeg->dma_result = result->result;
	complete(&jpeg->dma_done);
}

static int s31_jpeg_decode(struct s31_jpeg_ctx *ctx, struct vb2_buffer *src,
			   struct vb2_buffer *dst, u32 *out_bytes)
{
	struct s31_jpeg *jpeg = ctx->jpeg;
	void __iomem *base = jpeg->base;
	struct v4l2_jpeg_reference quant[4] = {};
	struct v4l2_jpeg_scan_header scan = {};
	struct v4l2_jpeg_header header = {
		.scan = &scan,
		.quantization_tables = quant,
	};
	const u8 *tables[4] = {};
	struct esp_dma2d_jpeg_cfg dcfg = {};
	struct dma_async_tx_descriptor *desc;
	struct dma_chan *dma;
	dma_cookie_t cookie;
	u8 *input = vb2_plane_vaddr(src, 0);
	size_t bytes = vb2_get_plane_payload(src, 0), offset;
	u16 mcu_w, mcu_h, proc_w, proc_h;
	unsigned int i, j, nf;
	u32 config, dqt_info = 0;
	int ret;

	if (!input)
		return -EFAULT;
	ret = v4l2_jpeg_parse_header(input, bytes, &header);
	if (ret)
		return ret;

	nf = header.frame.num_components;
	if (header.frame.precision != 8 || (nf != 1 && nf != 3) ||
	    scan.num_components != nf || !header.sos.start ||
	    header.sos.start < input + 2)
		return -EINVAL;

	if (nf == 1) {
		if (ctx->target.pixelformat != V4L2_PIX_FMT_GREY ||
		    header.frame.component[0].horizontal_sampling_factor != 1 ||
		    header.frame.component[0].vertical_sampling_factor != 1)
			return -EINVAL;
		mcu_w = 8;
		mcu_h = 8;
		dcfg.mb_size = ESP_DMA2D_MB_8X8;
		dcfg.block_w = 96;
		dcfg.block_h = 8;
		dcfg.pbyte = 1;
		dcfg.rx_csc = ESP_DMA2D_RX_CSC_NONE;
		dcfg.rx_post_scramble = 0;
	} else {
		u8 h0 = header.frame.component[0].horizontal_sampling_factor;
		u8 v0 = header.frame.component[0].vertical_sampling_factor;

		if (ctx->target.pixelformat != V4L2_PIX_FMT_RGB565 &&
		    ctx->target.pixelformat != V4L2_PIX_FMT_RGB24)
			return -EINVAL;
		if (h0 == 1 && v0 == 1) {
			/* YUV444 */
			mcu_w = 8;
			mcu_h = 8;
			dcfg.mb_size = ESP_DMA2D_MB_8X8;
			dcfg.block_w = 40;
			dcfg.block_h = 8;
			dcfg.rx_csc = (ctx->target.pixelformat == V4L2_PIX_FMT_RGB565) ?
				      ESP_DMA2D_RX_CSC_YUV444_TO_RGB565_601 :
				      ESP_DMA2D_RX_CSC_YUV444_TO_RGB888_601;
		} else if (h0 == 2 && v0 == 1) {
			/* YUV422 */
			mcu_w = 16;
			mcu_h = 8;
			dcfg.mb_size = ESP_DMA2D_MB_8X16;
			dcfg.block_w = 32;
			dcfg.block_h = 8;
			dcfg.rx_csc = (ctx->target.pixelformat == V4L2_PIX_FMT_RGB565) ?
				      ESP_DMA2D_RX_CSC_YUV420_TO_RGB565_601 :
				      ESP_DMA2D_RX_CSC_YUV420_TO_RGB888_601;
		} else if (h0 == 2 && v0 == 2) {
			/* YUV420 */
			mcu_w = 16;
			mcu_h = 16;
			dcfg.mb_size = ESP_DMA2D_MB_16X16;
			dcfg.block_w = 32;
			dcfg.block_h = 16;
			dcfg.rx_csc = (ctx->target.pixelformat == V4L2_PIX_FMT_RGB565) ?
				      ESP_DMA2D_RX_CSC_YUV420_TO_RGB565_601 :
				      ESP_DMA2D_RX_CSC_YUV420_TO_RGB888_601;
		} else {
			return -EINVAL;
		}
		if (ctx->target.pixelformat == V4L2_PIX_FMT_RGB565) {
			dcfg.pbyte = 3;
			dcfg.rx_post_scramble = 0; /* BYTE2_1_0 -> little-endian V4L2_PIX_FMT_RGB565 */
		} else {
			dcfg.pbyte = 4;
			dcfg.rx_post_scramble = 0; /* BYTE2_1_0 -> V4L2_PIX_FMT_RGB24 (R,G,B) */
		}
	}

	proc_w = round_up(header.frame.width, mcu_w);
	proc_h = round_up(header.frame.height, mcu_h);
	if (proc_w != ctx->target.width || proc_h != ctx->target.height)
		return -EINVAL;

	ret = s31_jpeg_parse_dht(&header, tables);
	if (ret)
		return ret;
	/* Fall back to standard Annex K Huffman tables when DHT is omitted. */
	if (!tables[0])
		tables[0] = std_dc_luma;
	if (!tables[1])
		tables[1] = std_dc_chroma;
	if (!tables[2])
		tables[2] = std_ac_luma;
	if (!tables[3])
		tables[3] = std_ac_chroma;

	offset = header.sos.start - input - 2;
	if (offset >= bytes || header.ecs_offset >= bytes)
		return -EINVAL;

	dma = dma_request_chan(jpeg->dev, "codec");
	if (IS_ERR(dma))
		return PTR_ERR(dma);

	dcfg.encode = false;
	dcfg.buf_1d = vb2_dma_contig_plane_dma_addr(src, 0) + offset;
	dcfg.len_1d = bytes - offset;
	dcfg.buf_2d = vb2_dma_contig_plane_dma_addr(dst, 0);
	dcfg.width = proc_w;
	dcfg.height = proc_h;
	if (dcfg.block_w > proc_w)
		dcfg.block_w = proc_w;

	desc = esp_dma2d_prep_jpeg_xfer(dma, &dcfg, DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
	if (!desc) {
		ret = -ENOMEM;
		goto release_dma;
	}
	desc->callback_result = s31_jpeg_dma_cb;
	desc->callback_param = jpeg;
	cookie = dmaengine_submit(desc);
	ret = dma_submit_error(cookie);
	if (ret)
		goto release_dma;

	config = readl(base + JPEG_CONFIG) & ~(JPEG_SOFT_RST | JPEG_START);
	writel(config | JPEG_SOFT_RST, base + JPEG_CONFIG);
	/*
	 * S31 TRM ch.35 JPEG_CONFIG_REG: preserve JPEG_DECODE_TIMEOUT_THRES=32
	 * (0 sets timeout to 2^0 - 1 = 0 cycles and immediately triggers
	 * DECODE_TIMEOUT). Clear PIC_SIZE before loading new dimensions
	 * (IDF jpeg_decode.c:820 digital issue).
	 */
	config = JPEG_DECODE | JPEG_DEC_TIMEOUT_DEF | JPEG_QNR_FIFO_EN |
		 JPEG_DHT_FIFO_EN | JPEG_FF_CHECK_EN;
	writel(config, base + JPEG_CONFIG);
	writel(0, base + JPEG_PIC_SIZE);

	/* IDF jpeg_parse_marker.c:110 — un-zigzag DQT to raster order for T0..T3QNR. */
	for (i = 0; i < 4; i++) {
		if (quant[i].length == 64) {
			u8 qtbl[64];

			for (j = 0; j < 64; j++)
				qtbl[zigzag_order[j]] = quant[i].start[j];
			dqt_info |= (i << (i * 8));
			writel(dqt_info, base + JPEG_DQT_INFO);
			for (j = 0; j < 64; j++)
				writel(qtbl[j], base + JPEG_T0QNR + i * 4);
		}
	}

	writel((proc_w << 16) | proc_h, base + JPEG_PIC_SIZE);
	/* Preserve default DEZIGZAG_READY_CTL / SOS_CHECK / RST_CHECK bits [31:24]. */
	writel((readl(base + JPEG_DEC_CONF) & ~GENMASK(23, 0)) |
	       (nf << 16) | header.restart_interval, base + JPEG_DEC_CONF);
	for (i = 0; i < nf; i++) {
		const struct v4l2_jpeg_frame_component_spec *c = &header.frame.component[i];
		u32 val = ((u32)c->component_identifier << 16) |
			  ((u32)c->horizontal_sampling_factor << 12) |
			  ((u32)c->vertical_sampling_factor << 8) |
			  c->quantization_table_selector;

		writel(val, base + JPEG_C0 + i * 4);
	}

	writel(0 | (1 << 4) | (0 << 8) | (1 << 12), base + JPEG_DHT_INFO);
	s31_jpeg_write_huff(base, tables[0], JPEG_DC0_COUNT, JPEG_DC0_MIN);
	s31_jpeg_write_huff(base, tables[2], JPEG_AC0_COUNT, JPEG_AC0_MIN);
	if (nf == 3) {
		s31_jpeg_write_huff(base, tables[1], JPEG_DC1_COUNT, JPEG_DC1_MIN);
		s31_jpeg_write_huff(base, tables[3], JPEG_AC1_COUNT, JPEG_AC1_MIN);
	}

	reinit_completion(&jpeg->codec_done);
	reinit_completion(&jpeg->dma_done);
	jpeg->codec_status = 0;
	/* IDF jpeg_decode.c: enable error IRQs; completion is signaled by 2D-DMA RX EOF. */
	writel(JPEG_DEC_IRQS, base + JPEG_IRQ_CLR);
	writel(JPEG_DEC_ERRORS, base + JPEG_IRQ_ENA);
	dma_async_issue_pending(dma);
	writel(config | JPEG_START, base + JPEG_CONFIG);

	if (!wait_for_completion_timeout(&jpeg->dma_done, msecs_to_jiffies(JPEG_TIMEOUT_MS)))
		ret = -ETIMEDOUT;
	else if (jpeg->codec_status & JPEG_DEC_ERRORS)
		ret = -EIO;
	else if (jpeg->dma_result != DMA_TRANS_NOERROR)
		ret = -EIO;

	writel(0, base + JPEG_IRQ_ENA);
	synchronize_irq(jpeg->irq);
	if (ret) {
		dev_err_ratelimited(jpeg->dev,
				    "decode failed: %d, status=%#x, dma=%d\n",
				    ret, jpeg->codec_status, jpeg->dma_result);
		writel(config | JPEG_SOFT_RST, base + JPEG_CONFIG);
		writel(config, base + JPEG_CONFIG);
		dmaengine_terminate_sync(dma);
	} else {
		*out_bytes = proc_w * proc_h * s31_jpeg_bpp(ctx->target.pixelformat);
	}

release_dma:
	dma_release_channel(dma);
	return ret;
}

static void s31_jpeg_scale_qtable(u8 dst[64], const u8 base_tbl[64], u32 quality)
{
	int scale = (quality < 50) ? (5000 / quality) : (200 - quality * 2);
	unsigned int i;

	for (i = 0; i < 64; i++) {
		int v = (base_tbl[i] * scale + 50) / 100;

		dst[i] = clamp(v, 1, 255);
	}
}

static size_t s31_jpeg_emit_header(u8 *buf, size_t max_len, u16 w, u16 h,
				   bool gray, const u8 luma_q[64], const u8 chroma_q[64])
{
	size_t pos = 0;
	unsigned int i, nf = gray ? 1 : 3;
	size_t sos_len = 2 + 2 + 1 + 2 * nf + 3;
	size_t align_target, com_pad;

#define EMIT_B(v) do { if (pos >= max_len) return 0; buf[pos++] = (u8)(v); } while (0)
#define EMIT_W(v) do { EMIT_B((v) >> 8); EMIT_B((v) & 0xff); } while (0)
#define EMIT_DHT(id, tbl, vlen) do { \
	EMIT_B(0xff); EMIT_B(0xc4); EMIT_W(2 + 1 + 16 + (vlen)); EMIT_B(id); \
	for (i = 0; i < 16 + (vlen); i++) EMIT_B((tbl)[i]); \
} while (0)

	/* SOI */
	EMIT_B(0xff); EMIT_B(0xd8);
	/* APP0 JFIF 1.1 */
	EMIT_B(0xff); EMIT_B(0xe0); EMIT_W(16);
	EMIT_B('J'); EMIT_B('F'); EMIT_B('I'); EMIT_B('F'); EMIT_B(0);
	EMIT_B(1); EMIT_B(1); EMIT_B(0); EMIT_W(1); EMIT_W(1); EMIT_B(0); EMIT_B(0);

	/* DQT (zigzag order in JFIF bitstream) */
	EMIT_B(0xff); EMIT_B(0xdb); EMIT_W(67); EMIT_B(0);
	for (i = 0; i < 64; i++)
		EMIT_B(luma_q[zigzag_order[i]]);
	if (!gray) {
		EMIT_B(0xff); EMIT_B(0xdb); EMIT_W(67); EMIT_B(1);
		for (i = 0; i < 64; i++)
			EMIT_B(chroma_q[zigzag_order[i]]);
	}

	/* SOF0 */
	EMIT_B(0xff); EMIT_B(0xc0); EMIT_W(8 + 3 * nf); EMIT_B(8);
	EMIT_W(h); EMIT_W(w); EMIT_B(nf);
	if (gray) {
		EMIT_B(1); EMIT_B(0x11); EMIT_B(0);
	} else {
		/* YUV420 (2x2 luma, 1x1 Cb/Cr) */
		EMIT_B(1); EMIT_B(0x22); EMIT_B(0);
		EMIT_B(2); EMIT_B(0x11); EMIT_B(1);
		EMIT_B(3); EMIT_B(0x11); EMIT_B(1);
	}

	/* DHT */
	EMIT_DHT(0x00, std_dc_luma, 12);
	EMIT_DHT(0x10, std_ac_luma, 162);
	if (!gray) {
		EMIT_DHT(0x01, std_dc_chroma, 12);
		EMIT_DHT(0x11, std_ac_chroma, 162);
	}

	/* COM padding so (header_len) is ARCH_DMA_MINALIGN (64B) aligned per IDF emit_com_marker(). */
	align_target = ALIGN(pos + 4 + sos_len, ARCH_DMA_MINALIGN);
	com_pad = align_target - pos - 4 - sos_len;
	EMIT_B(0xff); EMIT_B(0xfe); EMIT_W(com_pad + 2);
	for (i = 0; i < com_pad; i++)
		EMIT_B(0);

	/* SOS */
	EMIT_B(0xff); EMIT_B(0xda); EMIT_W(sos_len - 2); EMIT_B(nf);
	for (i = 0; i < nf; i++) {
		EMIT_B(i + 1);
		EMIT_B(i == 0 ? 0x00 : 0x11);
	}
	EMIT_B(0); EMIT_B(63); EMIT_B(0);

#undef EMIT_DHT
#undef EMIT_W
#undef EMIT_B
	return pos;
}

static int s31_jpeg_encode(struct s31_jpeg_ctx *ctx, struct vb2_buffer *src,
			   struct vb2_buffer *dst, u32 *out_bytes)
{
	struct s31_jpeg *jpeg = ctx->jpeg;
	void __iomem *base = jpeg->base;
	struct esp_dma2d_jpeg_cfg dcfg = {};
	struct dma_async_tx_descriptor *desc;
	struct dma_chan *dma;
	dma_cookie_t cookie;
	u8 luma_q[64], chroma_q[64];
	u8 *out_vaddr = vb2_plane_vaddr(dst, 0);
	dma_addr_t out_dma = vb2_dma_contig_plane_dma_addr(dst, 0);
	size_t dst_cap = vb2_plane_size(dst, 0);
	size_t hdr_len;
	u32 rx_bytes = 0;
	u32 config, cs, sample_sel;
	bool gray = (ctx->source.pixelformat == V4L2_PIX_FMT_GREY);
	unsigned int i;
	int ret = 0;

	if (!out_vaddr)
		return -EFAULT;

	if (gray) {
		cs = 3; /* JPEG_ENC_SRC_GRAY */
		sample_sel = 0;
		dcfg.mb_size = ESP_DMA2D_MB_8X8;
		dcfg.block_w = 128;
		dcfg.block_h = 8;
		dcfg.pbyte = 1;
	} else if (ctx->source.pixelformat == V4L2_PIX_FMT_RGB565) {
		if (!IS_ALIGNED(ctx->source.width | ctx->source.height, 16))
			return -EINVAL;
		cs = 2; /* JPEG_ENC_SRC_RGB565 */
		sample_sel = 2; /* YUV420 */
		dcfg.mb_size = ESP_DMA2D_MB_16X16;
		dcfg.block_w = 48;
		dcfg.block_h = 16;
		dcfg.pbyte = 3;
	} else if (ctx->source.pixelformat == V4L2_PIX_FMT_RGB24) {
		if (!IS_ALIGNED(ctx->source.width | ctx->source.height, 16))
			return -EINVAL;
		cs = 0; /* JPEG_ENC_SRC_RGB888 */
		sample_sel = 2; /* YUV420 */
		dcfg.mb_size = ESP_DMA2D_MB_16X16;
		dcfg.block_w = 32;
		dcfg.block_h = 16;
		dcfg.pbyte = 4;
	} else {
		return -EINVAL;
	}

	s31_jpeg_scale_qtable(luma_q, std_luma_qtable, ctx->quality);
	s31_jpeg_scale_qtable(chroma_q, std_chroma_qtable, ctx->quality);
	hdr_len = s31_jpeg_emit_header(out_vaddr, dst_cap, ctx->source.width,
				       ctx->source.height, gray, luma_q, chroma_q);
	if (!hdr_len || hdr_len >= dst_cap)
		return -ENOSPC;

	/* Flush CPU-written JFIF header before DQBUF invalidates the capture buffer. */
	dma_sync_single_for_device(jpeg->dev, out_dma, hdr_len, DMA_TO_DEVICE);

	dma = dma_request_chan(jpeg->dev, "codec");
	if (IS_ERR(dma))
		return PTR_ERR(dma);

	dcfg.encode = true;
	dcfg.buf_2d = vb2_dma_contig_plane_dma_addr(src, 0);
	dcfg.buf_1d = out_dma + hdr_len;
	dcfg.len_1d = dst_cap - hdr_len;
	dcfg.width = ctx->source.width;
	dcfg.height = ctx->source.height;
	if (dcfg.block_w > dcfg.width)
		dcfg.block_w = dcfg.width;
	dcfg.rx_csc = ESP_DMA2D_RX_CSC_NONE;
	dcfg.rx_bytes_out = &rx_bytes;

	desc = esp_dma2d_prep_jpeg_xfer(dma, &dcfg, DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
	if (!desc) {
		ret = -ENOMEM;
		goto release_dma;
	}
	desc->callback_result = s31_jpeg_dma_cb;
	desc->callback_param = jpeg;
	cookie = dmaengine_submit(desc);
	ret = dma_submit_error(cookie);
	if (ret)
		goto release_dma;

	/* IDF jpeg_encode.c: soft reset, encoder mode, color space, sample mode, QNR tables. */
	writel(JPEG_SOFT_RST, base + JPEG_CONFIG);
	config = JPEG_FF_CHECK_EN | JPEG_QNR_FIFO_EN | JPEG_TAILER_EN |
		 FIELD_PREP(JPEG_SAMPLE_SEL, sample_sel) |
		 FIELD_PREP(JPEG_LQNR_TBL_SEL, 0) |
		 FIELD_PREP(JPEG_CQNR_TBL_SEL, 1) |
		 FIELD_PREP(JPEG_COLOR_SPACE, cs);
	writel(config, base + JPEG_CONFIG);
	writel(((u32)ctx->source.width << 16) | ctx->source.height, base + JPEG_PIC_SIZE);

	/* Hardware T0QNR/T1QNR take raster-order quantization coefficients (IDF jpeg_hal.c). */
	writel(0 | (1 << 8), base + JPEG_DQT_INFO);
	for (i = 0; i < 64; i++)
		writel(luma_q[i], base + JPEG_T0QNR);
	if (!gray) {
		for (i = 0; i < 64; i++)
			writel(chroma_q[i], base + JPEG_T1QNR);
	}

	reinit_completion(&jpeg->codec_done);
	reinit_completion(&jpeg->dma_done);
	jpeg->codec_status = 0;
	writel(JPEG_ENC_IRQS | JPEG_DONE, base + JPEG_IRQ_CLR);
	writel(JPEG_ENC_IRQS, base + JPEG_IRQ_ENA);
	dma_async_issue_pending(dma);
	writel(config | JPEG_START, base + JPEG_CONFIG);

	if (!wait_for_completion_timeout(&jpeg->dma_done, msecs_to_jiffies(JPEG_TIMEOUT_MS)))
		ret = -ETIMEDOUT;
	else if (jpeg->codec_status & JPEG_ENC_ERRORS)
		ret = -EIO;
	else if (jpeg->dma_result != DMA_TRANS_NOERROR || !rx_bytes)
		ret = -EIO;

	writel(0, base + JPEG_IRQ_ENA);
	synchronize_irq(jpeg->irq);
	if (ret) {
		dev_err_ratelimited(jpeg->dev,
				    "encode failed: %d, status=%#x, dma=%d\n",
				    ret, jpeg->codec_status, jpeg->dma_result);
		writel(config | JPEG_SOFT_RST, base + JPEG_CONFIG);
		writel(config, base + JPEG_CONFIG);
		dmaengine_terminate_sync(dma);
	} else {
		*out_bytes = hdr_len + rx_bytes;
	}

release_dma:
	dma_release_channel(dma);
	return ret;
}

static void s31_jpeg_worker(struct work_struct *work)
{
	struct s31_jpeg *jpeg = container_of(work, struct s31_jpeg, work);
	struct s31_jpeg_ctx *ctx = jpeg->running;
	struct v4l2_m2m_ctx *m2m = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(m2m);
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(m2m);
	u32 out_bytes = 0;
	int ret = -ENODEV;

	if (!READ_ONCE(jpeg->disconnected)) {
		if (ctx->source.pixelformat == V4L2_PIX_FMT_JPEG &&
		    ctx->target.pixelformat != V4L2_PIX_FMT_JPEG)
			ret = s31_jpeg_decode(ctx, &src->vb2_buf, &dst->vb2_buf, &out_bytes);
		else if (ctx->source.pixelformat != V4L2_PIX_FMT_JPEG &&
			 ctx->target.pixelformat == V4L2_PIX_FMT_JPEG)
			ret = s31_jpeg_encode(ctx, &src->vb2_buf, &dst->vb2_buf, &out_bytes);
		else
			ret = -EINVAL;
	}

	vb2_set_plane_payload(&dst->vb2_buf, 0, ret ? 0 : out_bytes);
	if (!ret)
		v4l2_m2m_buf_copy_metadata(src, dst, true);
	v4l2_m2m_buf_done_and_job_finish(jpeg->m2m, m2m,
					 ret ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
}

static void s31_jpeg_device_run(void *data)
{
	struct s31_jpeg_ctx *ctx = data;

	ctx->jpeg->running = ctx;
	schedule_work(&ctx->jpeg->work);
}

static const struct v4l2_m2m_ops s31_jpeg_m2m_ops = {
	.device_run = s31_jpeg_device_run,
};

static void s31_jpeg_video_release(struct video_device *video)
{
	struct s31_jpeg *jpeg = video_get_drvdata(video);

	v4l2_m2m_release(jpeg->m2m);
	v4l2_device_unregister(&jpeg->v4l2);
	kfree(jpeg);
}

static int s31_jpeg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *reset;
	struct s31_jpeg *jpeg;
	void __iomem *base;
	struct dma_chan *dma;
	struct clk *clk;
	int ret, irq;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "JPEG clock unavailable\n");
	reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset), "JPEG reset unavailable\n");
	ret = reset_control_reset(reset);
	if (ret)
		return ret;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	dma = dma_request_chan(dev, "codec");
	if (IS_ERR(dma))
		return dev_err_probe(dev, PTR_ERR(dma), "JPEG 2D-DMA unavailable\n");
	dma_release_channel(dma);

	jpeg = kzalloc(sizeof(*jpeg), GFP_KERNEL);
	if (!jpeg)
		return -ENOMEM;
	jpeg->dev = dev;
	jpeg->base = base;
	jpeg->irq = irq;
	mutex_init(&jpeg->lock);
	init_completion(&jpeg->codec_done);
	init_completion(&jpeg->dma_done);
	INIT_WORK(&jpeg->work, s31_jpeg_worker);

	ret = request_irq(irq, s31_jpeg_irq, 0, dev_name(dev), jpeg);
	if (ret)
		goto free_jpeg;
	ret = v4l2_device_register(dev, &jpeg->v4l2);
	if (ret)
		goto free_irq;
	jpeg->m2m = v4l2_m2m_init(&s31_jpeg_m2m_ops);
	if (IS_ERR(jpeg->m2m)) {
		ret = PTR_ERR(jpeg->m2m);
		goto unregister;
	}
	jpeg->video = (struct video_device) {
		.name = "esp32s31-jpeg",
		.v4l2_dev = &jpeg->v4l2,
		.fops = &s31_jpeg_fops,
		.ioctl_ops = &s31_jpeg_ioctls,
		.release = s31_jpeg_video_release,
		.lock = &jpeg->lock,
		.vfl_dir = VFL_DIR_M2M,
		.device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING,
	};
	video_set_drvdata(&jpeg->video, jpeg);
	ret = video_register_device(&jpeg->video, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto release_m2m;
	platform_set_drvdata(pdev, jpeg);
	dev_info(dev, "registered V4L2 M2M JPEG encoder/decoder as /dev/video%d\n",
		 jpeg->video.num);
	return 0;

release_m2m:
	v4l2_m2m_release(jpeg->m2m);
unregister:
	v4l2_device_unregister(&jpeg->v4l2);
free_irq:
	free_irq(irq, jpeg);
free_jpeg:
	kfree(jpeg);
	return ret;
}

static void s31_jpeg_remove(struct platform_device *pdev)
{
	struct s31_jpeg *jpeg = platform_get_drvdata(pdev);

	mutex_lock(&jpeg->lock);
	WRITE_ONCE(jpeg->disconnected, true);
	mutex_unlock(&jpeg->lock);
	v4l2_m2m_suspend(jpeg->m2m);
	flush_work(&jpeg->work);
	free_irq(jpeg->irq, jpeg);
	video_unregister_device(&jpeg->video);
}

static const struct of_device_id s31_jpeg_of_match[] = {
	{ .compatible = "espressif,esp32s31-jpeg" },
	{ }
};
MODULE_DEVICE_TABLE(of, s31_jpeg_of_match);

static struct platform_driver s31_jpeg_driver = {
	.probe = s31_jpeg_probe,
	.remove = s31_jpeg_remove,
	.driver = {
		.name = "esp32s31-jpeg",
		.of_match_table = s31_jpeg_of_match,
	},
};
module_platform_driver(s31_jpeg_driver);

MODULE_DESCRIPTION("ESP32-S31 V4L2 M2M JPEG encoder and decoder");
MODULE_LICENSE("GPL");
