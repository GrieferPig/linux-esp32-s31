// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-P4 rev1.3 JPEG hardware encoder and decoder (V4L2 M2M).
 *
 * Supports baseline JPEG decode and encode for GREY, RGB565, and RGB24
 * using 2D-DMA macro-block reorder and RX BT.601 YUV->RGB CSC.
 *
 * Hardware reference: ESP32-P4 rev1.3 TRM ch.33 JPEG + ESP-IDF
 * soc/esp32p4/register/hw_ver1/soc/jpeg_reg.h and esp_hal_jpeg/esp32p4/jpeg_ll.h.
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

/* ESP32-P4 rev1.3 TRM ch.33 JPEG / IDF hw_ver1/soc/jpeg_reg.h */
#define JPEG_CONFIG		0x00
#define JPEG_DQT_INFO		0x04
#define JPEG_PIC_SIZE		0x08
#define JPEG_QNR0		0x10
#define JPEG_QNR1		0x14
#define JPEG_QNR2		0x18
#define JPEG_QNR3		0x1c
#define JPEG_DEC_CONF		0x20
#define JPEG_C0			0x24
#define JPEG_C1			0x28
#define JPEG_C2			0x2c
#define JPEG_DHT_INFO		0x34
#define JPEG_IRQ_ENA		0x3c
#define JPEG_IRQ_ST		0x40
#define JPEG_IRQ_CLR		0x44
#define JPEG_DC0_COUNT		0x58
#define JPEG_DC1_COUNT		0x5c
#define JPEG_AC0_COUNT		0x60
#define JPEG_AC1_COUNT		0x64
#define JPEG_DC0_MIN		0x78
#define JPEG_DC1_MIN		0x7c
#define JPEG_AC0_MIN		0x80
#define JPEG_AC1_MIN		0x84

#define JPEG_FSM_RST		BIT(0)
#define JPEG_START		BIT(1)
#define JPEG_FF_CHECK_EN	BIT(3)
#define JPEG_SAMPLE_SEL		GENMASK(5, 4)
#define JPEG_QNR_FIFO		BIT(8)	/* P4 hw_ver1: bit 8 (S31 is bit 7) */
#define JPEG_LQNR_TBL_SEL	GENMASK(10, 9)
#define JPEG_CQNR_TBL_SEL	GENMASK(12, 11)
#define JPEG_COLOR_SPACE	GENMASK(14, 13)
#define JPEG_HUFF_FIFO		BIT(15)
#define JPEG_SOFT_RST		BIT(24)
#define JPEG_FIFO_RST		BIT(25)
#define JPEG_PIXEL_REV		BIT(26)
#define JPEG_TAILER_EN		BIT(27)
#define JPEG_DECODE		BIT(31)

#define JPEG_DONE		BIT(0)
#define JPEG_ENC_ERRORS		(BIT(1) | BIT(16))
#define JPEG_DECODE_ERRORS	(GENMASK(8, 2) | GENMASK(15, 13) | GENMASK(24, 18))

#define JPEG_TIMEOUT_MS		500
#define JPEG_MIN_EDGE		16
#define JPEG_MAX_EDGE		1920
#define JPEG_MAX_INPUT		SZ_2M

/* Standard Annex K luminance and chrominance quantization tables (zigzag). */
static const u8 std_luma_qt[64] = {
	16, 11, 12, 14, 12, 10, 16, 14, 13, 14, 18, 17, 16, 19, 24, 40,
	26, 24, 22, 22, 24, 49, 35, 37, 29, 40, 58, 51, 61, 60, 57, 51,
	56, 55, 64, 72, 92, 78, 64, 68, 87, 69, 55, 56, 80, 109, 81, 87,
	95, 98, 103, 104, 103, 62, 77, 113, 121, 112, 100, 120, 92, 101, 103, 99,
};

static const u8 std_chroma_qt[64] = {
	17, 18, 18, 24, 21, 24, 47, 26, 26, 47, 99, 66, 56, 66, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
};

static const u8 std_dc_luma_bits[16] = {
	0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0
};
static const u8 std_dc_luma_vals[12] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

static const u8 std_dc_chroma_bits[16] = {
	0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0
};
static const u8 std_dc_chroma_vals[12] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

static const u8 std_ac_luma_bits[16] = {
	0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 125
};
static const u8 std_ac_luma_vals[162] = {
	0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
	0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
	0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
	0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
	0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16,
	0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
	0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
	0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
	0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
	0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
	0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
	0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
	0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
	0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
	0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
	0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
	0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
	0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
	0xf9, 0xfa,
};

static const u8 std_ac_chroma_bits[16] = {
	0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 119
};
static const u8 std_ac_chroma_vals[162] = {
	0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21,
	0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
	0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
	0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
	0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34,
	0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
	0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38,
	0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
	0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
	0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
	0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
	0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
	0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96,
	0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
	0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
	0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
	0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2,
	0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
	0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9,
	0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
	0xf9, 0xfa,
};

struct p4_jpeg {
	void __iomem *base;
	struct device *dev;
	struct v4l2_device v4l2;
	struct video_device video;
	struct v4l2_m2m_dev *m2m;
	struct mutex lock;
	struct work_struct work;
	struct jpeg_context *running;
	struct completion codec_done;
	struct completion dma_done;
	enum dmaengine_tx_result dma_result;
	u32 codec_status;
	int irq;
	bool disconnected;
};

struct jpeg_context {
	struct v4l2_fh fh;
	struct p4_jpeg *jpeg;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_pix_format source;
	struct v4l2_pix_format target;
	u32 quality;
};

static const u32 p4_jpeg_fmts[] = {
	V4L2_PIX_FMT_JPEG,
	V4L2_PIX_FMT_GREY,
	V4L2_PIX_FMT_RGB565,
	V4L2_PIX_FMT_RGB24,
};

static struct jpeg_context *jpeg_context(struct file *file)
{
	return container_of(file->private_data, struct jpeg_context, fh);
}

static struct v4l2_pix_format *jpeg_format(struct jpeg_context *ctx,
					   enum v4l2_buf_type type)
{
	return V4L2_TYPE_IS_OUTPUT(type) ? &ctx->source : &ctx->target;
}

static unsigned int jpeg_bpp(u32 pixfmt)
{
	switch (pixfmt) {
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

static void jpeg_bound_format(struct v4l2_pix_format *pix, bool output)
{
	u32 bpp;

	if (pix->pixelformat != V4L2_PIX_FMT_JPEG && !jpeg_bpp(pix->pixelformat))
		pix->pixelformat = output ? V4L2_PIX_FMT_JPEG : V4L2_PIX_FMT_RGB565;

	pix->width = round_up(clamp_t(u32, pix->width ?: 64,
				      JPEG_MIN_EDGE, JPEG_MAX_EDGE), 16);
	pix->height = round_up(clamp_t(u32, pix->height ?: 64,
				       JPEG_MIN_EDGE, JPEG_MAX_EDGE), 16);
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_JPEG;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	bpp = jpeg_bpp(pix->pixelformat);
	if (!bpp) {
		pix->bytesperline = 0;
		pix->sizeimage = clamp_t(u32, pix->sizeimage ?: pix->width * pix->height * 3 + SZ_4K,
					 SZ_4K, JPEG_MAX_INPUT);
	} else {
		pix->bytesperline = pix->width * bpp;
		pix->sizeimage = pix->bytesperline * pix->height;
	}
}

static int jpeg_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	struct p4_jpeg *jpeg = video_drvdata(file);

	strscpy(cap->driver, "esp32p4-jpeg", sizeof(cap->driver));
	strscpy(cap->card, "ESP32-P4 JPEG Codec", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(jpeg->dev));
	return 0;
}

static int jpeg_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *fmt)
{
	if (fmt->index >= ARRAY_SIZE(p4_jpeg_fmts))
		return -EINVAL;
	fmt->pixelformat = p4_jpeg_fmts[fmt->index];
	return 0;
}

static int jpeg_try_fmt(struct file *file, void *fh, struct v4l2_format *fmt)
{
	jpeg_bound_format(&fmt->fmt.pix, V4L2_TYPE_IS_OUTPUT(fmt->type));
	return 0;
}

static int jpeg_get_fmt(struct file *file, void *fh, struct v4l2_format *fmt)
{
	fmt->fmt.pix = *jpeg_format(jpeg_context(file), fmt->type);
	return 0;
}

static int jpeg_set_fmt(struct file *file, void *fh, struct v4l2_format *fmt)
{
	struct jpeg_context *ctx = jpeg_context(file);

	if (vb2_is_busy(v4l2_m2m_get_vq(ctx->fh.m2m_ctx, fmt->type)))
		return -EBUSY;
	jpeg_bound_format(&fmt->fmt.pix, V4L2_TYPE_IS_OUTPUT(fmt->type));
	*jpeg_format(ctx, fmt->type) = fmt->fmt.pix;
	return 0;
}

static int jpeg_reqbufs(struct file *file, void *fh, struct v4l2_requestbuffers *req)
{
	if (req->count && req->memory == V4L2_MEMORY_MMAP)
		req->flags |= V4L2_MEMORY_FLAG_NON_COHERENT;
	return v4l2_m2m_ioctl_reqbufs(file, fh, req);
}

static const struct v4l2_ioctl_ops jpeg_ioctls = {
	.vidioc_querycap = jpeg_querycap,
	.vidioc_enum_fmt_vid_cap = jpeg_enum_fmt,
	.vidioc_enum_fmt_vid_out = jpeg_enum_fmt,
	.vidioc_try_fmt_vid_cap = jpeg_try_fmt,
	.vidioc_try_fmt_vid_out = jpeg_try_fmt,
	.vidioc_g_fmt_vid_cap = jpeg_get_fmt,
	.vidioc_g_fmt_vid_out = jpeg_get_fmt,
	.vidioc_s_fmt_vid_cap = jpeg_set_fmt,
	.vidioc_s_fmt_vid_out = jpeg_set_fmt,
	.vidioc_reqbufs = jpeg_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
};

static int jpeg_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct jpeg_context *ctx = container_of(ctrl->handler, struct jpeg_context, ctrl_handler);

	if (ctrl->id == V4L2_CID_JPEG_COMPRESSION_QUALITY)
		ctx->quality = ctrl->val;
	return 0;
}

static const struct v4l2_ctrl_ops jpeg_ctrl_ops = {
	.s_ctrl = jpeg_s_ctrl,
};

static int jpeg_queue_setup(struct vb2_queue *q, unsigned int *buffers,
			    unsigned int *planes, unsigned int sizes[],
			    struct device *alloc_devs[])
{
	struct jpeg_context *ctx = vb2_get_drv_priv(q);
	unsigned int size = jpeg_format(ctx, q->type)->sizeimage;

	if (*planes)
		return *planes == 1 && sizes[0] >= size ? 0 : -EINVAL;
	*planes = 1;
	sizes[0] = size;
	return 0;
}

static int jpeg_buffer_prepare(struct vb2_buffer *buf)
{
	struct vb2_queue *q = buf->vb2_queue;
	struct jpeg_context *ctx = vb2_get_drv_priv(q);
	unsigned int size = jpeg_format(ctx, q->type)->sizeimage;
	unsigned int payload = vb2_get_plane_payload(buf, 0);

	if (vb2_plane_size(buf, 0) < size)
		return -EINVAL;
	if (V4L2_TYPE_IS_OUTPUT(q->type) && (payload < 4 || payload > size))
		return -EINVAL;
	return 0;
}

static void jpeg_buffer_queue(struct vb2_buffer *buf)
{
	struct jpeg_context *ctx = vb2_get_drv_priv(buf->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(buf));
}

static void jpeg_stop_streaming(struct vb2_queue *q)
{
	struct jpeg_context *ctx = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *buf;

	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(q->type))
			buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!buf)
			break;
		v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
	}
}

static const struct vb2_ops jpeg_queue_ops = {
	.queue_setup = jpeg_queue_setup,
	.buf_prepare = jpeg_buffer_prepare,
	.buf_queue = jpeg_buffer_queue,
	.stop_streaming = jpeg_stop_streaming,
};

static int jpeg_queue_init(void *data, struct vb2_queue *src, struct vb2_queue *dst)
{
	struct jpeg_context *ctx = data;
	int ret;

	*src = (struct vb2_queue) {
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
		.io_modes = VB2_MMAP,
		.drv_priv = ctx,
		.buf_struct_size = sizeof(struct v4l2_m2m_buffer),
		.ops = &jpeg_queue_ops,
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

static int jpeg_open(struct file *file)
{
	struct p4_jpeg *jpeg = video_drvdata(file);
	struct jpeg_context *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->jpeg = jpeg;
	ctx->quality = 80;
	ctx->source.width = 320;
	ctx->source.height = 240;
	ctx->source.pixelformat = V4L2_PIX_FMT_JPEG;
	ctx->target.width = 320;
	ctx->target.height = 240;
	ctx->target.pixelformat = V4L2_PIX_FMT_RGB565;
	jpeg_bound_format(&ctx->source, true);
	jpeg_bound_format(&ctx->target, false);

	v4l2_ctrl_handler_init(&ctx->ctrl_handler, 1);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &jpeg_ctrl_ops,
			  V4L2_CID_JPEG_COMPRESSION_QUALITY, 1, 100, 1, 80);
	if (ctx->ctrl_handler.error) {
		ret = ctx->ctrl_handler.error;
		goto free_ctrl;
	}

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	ctx->fh.ctrl_handler = &ctx->ctrl_handler;
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(jpeg->m2m, ctx, jpeg_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		v4l2_fh_exit(&ctx->fh);
		goto free_ctrl;
	}
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh, file);
	return 0;

free_ctrl:
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	kfree(ctx);
	return ret;
}

static int jpeg_release(struct file *file)
{
	struct jpeg_context *ctx = jpeg_context(file);

	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations jpeg_fops = {
	.owner = THIS_MODULE,
	.open = jpeg_open,
	.release = jpeg_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
#ifndef CONFIG_MMU
	.get_unmapped_area = v4l2_m2m_get_unmapped_area,
#endif
};

static int jpeg_huffman_tables(const struct v4l2_jpeg_header *header,
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

static void jpeg_write_huffman_idx(void __iomem *base, const u8 *table,
				   bool ac, unsigned int idx)
{
	u32 count = ac ? (idx ? JPEG_AC1_COUNT : JPEG_AC0_COUNT)
		       : (idx ? JPEG_DC1_COUNT : JPEG_DC0_COUNT);
	u32 min = ac ? (idx ? JPEG_AC1_MIN : JPEG_AC0_MIN)
		     : (idx ? JPEG_DC1_MIN : JPEG_DC0_MIN);
	u32 val = count + 4;
	unsigned int i, total = 0, code = 0;

	for (i = 0; i < 16; i++) {
		total += table[i];
		writel(total, base + count);
		writel(table[i] ? code << (15 - i) : 0xffff, base + min);
		code = (code + table[i]) << 1;
	}
	for (i = 0; i < total; i++)
		writel(table[16 + i], base + val);
}

static irqreturn_t jpeg_interrupt(int irq, void *data)
{
	struct p4_jpeg *jpeg = data;
	u32 status = readl(jpeg->base + JPEG_IRQ_ST);

	if (!status)
		return IRQ_NONE;
	jpeg->codec_status |= status;
	writel(status, jpeg->base + JPEG_IRQ_CLR);
	complete(&jpeg->codec_done);
	return IRQ_HANDLED;
}

static void jpeg_dma_complete(void *data, const struct dmaengine_result *result)
{
	struct p4_jpeg *jpeg = data;

	jpeg->dma_result = result->result;
	complete(&jpeg->dma_done);
}

static int jpeg_decode(struct jpeg_context *ctx, struct vb2_buffer *src,
		       struct vb2_buffer *dst, u32 *out_bytes)
{
	struct p4_jpeg *jpeg = ctx->jpeg;
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
	unsigned int i, c, ncomp;
	u32 config, dht_info = 0;
	int ret;

	if (!input)
		return -EFAULT;
	ret = v4l2_jpeg_parse_header(input, bytes, &header);
	if (ret)
		return ret;
	ncomp = header.frame.num_components;
	if (header.frame.precision != 8 || (ncomp != 1 && ncomp != 3) ||
	    scan.num_components != ncomp ||
	    header.frame.width != ctx->target.width ||
	    header.frame.height != ctx->target.height)
		return -EINVAL;

	if (ncomp == 1 && ctx->target.pixelformat != V4L2_PIX_FMT_GREY)
		return -EINVAL;
	if (ncomp == 3 && ctx->target.pixelformat != V4L2_PIX_FMT_RGB565 &&
	    ctx->target.pixelformat != V4L2_PIX_FMT_RGB24)
		return -EINVAL;

	ret = jpeg_huffman_tables(&header, tables);
	if (ret)
		return ret;
	if (!header.sos.start || header.sos.start < input + 2)
		return -EINVAL;
	offset = header.sos.start - input - 2;
	if (offset >= bytes || header.ecs_offset >= bytes)
		return -EINVAL;

	dcfg.encode = false;
	dcfg.buf_1d = vb2_dma_contig_plane_dma_addr(src, 0) + offset;
	dcfg.len_1d = bytes - offset;
	dcfg.buf_2d = vb2_dma_contig_plane_dma_addr(dst, 0);
	dcfg.width = ctx->target.width;
	dcfg.height = ctx->target.height;

	if (ncomp == 1) {
		dcfg.block_w = 8;
		dcfg.block_h = 8;
		dcfg.pbyte = 1;
		dcfg.mb_size = ESP_DMA2D_MB_8X8;
		dcfg.rx_csc = ESP_DMA2D_RX_CSC_NONE;
	} else {
		u8 h0 = header.frame.component[0].horizontal_sampling_factor;
		u8 v0 = header.frame.component[0].vertical_sampling_factor;
		bool yuv420 = (h0 == 2 && v0 == 2);
		bool yuv444 = (h0 == 1 && v0 == 1);

		if (!yuv420 && !yuv444)
			return -EOPNOTSUPP;
		dcfg.block_w = yuv420 ? 16 : 8;
		dcfg.block_h = yuv420 ? 16 : 8;
		dcfg.mb_size = yuv420 ? ESP_DMA2D_MB_16X16 : ESP_DMA2D_MB_8X8;
		if (ctx->target.pixelformat == V4L2_PIX_FMT_RGB565) {
			dcfg.pbyte = 3;
			dcfg.rx_csc = yuv420 ? ESP_DMA2D_RX_CSC_YUV420_TO_RGB565_601
					     : ESP_DMA2D_RX_CSC_YUV444_TO_RGB565_601;
		} else {
			dcfg.pbyte = 4;
			dcfg.rx_csc = yuv420 ? ESP_DMA2D_RX_CSC_YUV420_TO_RGB888_601
					     : ESP_DMA2D_RX_CSC_YUV444_TO_RGB888_601;
		}
		dcfg.rx_post_scramble = 0;
	}

	dma = dma_request_chan(jpeg->dev, "codec");
	if (IS_ERR(dma))
		return PTR_ERR(dma);

	desc = esp_dma2d_prep_jpeg_xfer(dma, &dcfg, DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
	if (!desc) {
		ret = -ENOMEM;
		goto release_dma;
	}
	desc->callback_result = jpeg_dma_complete;
	desc->callback_param = jpeg;
	cookie = dmaengine_submit(desc);
	ret = dma_submit_error(cookie);
	if (ret)
		goto release_dma;

	config = readl(base + JPEG_CONFIG) & ~(JPEG_SOFT_RST | JPEG_START);
	writel(config | JPEG_SOFT_RST, base + JPEG_CONFIG);
	config |= JPEG_DECODE | JPEG_QNR_FIFO | JPEG_HUFF_FIFO;
	writel(config, base + JPEG_CONFIG);

	writel(0x03020100, base + JPEG_DQT_INFO);
	for (c = 0; c < 4; c++) {
		if (quant[c].length == 64) {
			for (i = 0; i < 64; i++)
				writel(quant[c].start[i], base + JPEG_QNR0 + c * 4);
		}
	}

	writel((dcfg.width << 16) | dcfg.height, base + JPEG_PIC_SIZE);
	writel((ncomp << 16) | header.restart_interval, base + JPEG_DEC_CONF);
	for (c = 0; c < ncomp; c++) {
		const struct v4l2_jpeg_frame_component_spec *comp = &header.frame.component[c];
		u32 sf = (comp->horizontal_sampling_factor << 4) | comp->vertical_sampling_factor;

		writel((comp->component_identifier << 16) | (sf << 8) |
		       comp->quantization_table_selector, base + JPEG_C0 + c * 4);
	}

	dht_info = (0 << 0) | (1 << 4) | (0 << 8) | (1 << 12);
	writel(dht_info, base + JPEG_DHT_INFO);
	if (tables[0])
		jpeg_write_huffman_idx(base, tables[0], false, 0);
	if (tables[1])
		jpeg_write_huffman_idx(base, tables[1], false, 1);
	if (tables[2])
		jpeg_write_huffman_idx(base, tables[2], true, 0);
	if (tables[3])
		jpeg_write_huffman_idx(base, tables[3], true, 1);

	reinit_completion(&jpeg->codec_done);
	reinit_completion(&jpeg->dma_done);
	jpeg->codec_status = 0;
	writel(JPEG_DONE | JPEG_DECODE_ERRORS, base + JPEG_IRQ_CLR);
	writel(JPEG_DONE | JPEG_DECODE_ERRORS, base + JPEG_IRQ_ENA);
	dma_async_issue_pending(dma);
	writel(config | JPEG_START, base + JPEG_CONFIG);

	if (!wait_for_completion_timeout(&jpeg->dma_done, msecs_to_jiffies(JPEG_TIMEOUT_MS)))
		ret = -ETIMEDOUT;
	else if (jpeg->codec_status & JPEG_DECODE_ERRORS)
		ret = -EIO;
	else if (jpeg->dma_result != DMA_TRANS_NOERROR)
		ret = -EIO;

	writel(0, base + JPEG_IRQ_ENA);
	synchronize_irq(jpeg->irq);
	if (ret) {
		writel(config | JPEG_SOFT_RST, base + JPEG_CONFIG);
		writel(config, base + JPEG_CONFIG);
		dmaengine_terminate_sync(dma);
	} else {
		*out_bytes = ctx->target.sizeimage;
	}

release_dma:
	dma_release_channel(dma);
	return ret;
}

static void jpeg_scale_qt(u8 out[64], const u8 base_qt[64], u32 quality)
{
	u32 scale = (quality < 50) ? (5000 / quality) : (200 - 2 * quality);
	unsigned int i;

	for (i = 0; i < 64; i++) {
		u32 v = (base_qt[i] * scale + 50) / 100;
		out[i] = clamp_t(u32, v, 1, 255);
	}
}

static size_t jpeg_emit_header(u8 *buf, u16 w, u16 h, bool gray,
			       const u8 qt_y[64], const u8 qt_c[64])
{
	u8 *p = buf;
	size_t hdr_len, pad;

	/* SOI */
	*p++ = 0xff; *p++ = 0xd8;
	/* DQT Y */
	*p++ = 0xff; *p++ = 0xdb; *p++ = 0x00; *p++ = 67; *p++ = 0x00;
	memcpy(p, qt_y, 64); p += 64;
	if (!gray) {
		*p++ = 0xff; *p++ = 0xdb; *p++ = 0x00; *p++ = 67; *p++ = 0x01;
		memcpy(p, qt_c, 64); p += 64;
	}
	/* SOF0 */
	*p++ = 0xff; *p++ = 0xc0; *p++ = 0x00; *p++ = gray ? 11 : 17;
	*p++ = 8; *p++ = h >> 8; *p++ = h & 0xff; *p++ = w >> 8; *p++ = w & 0xff;
	*p++ = gray ? 1 : 3;
	*p++ = 1; *p++ = gray ? 0x11 : 0x22; *p++ = 0;
	if (!gray) {
		*p++ = 2; *p++ = 0x11; *p++ = 1;
		*p++ = 3; *p++ = 0x11; *p++ = 1;
	}
	/* DHT DC0, AC0, and chroma DC1/AC1 */
	*p++ = 0xff; *p++ = 0xc4; *p++ = 0x00; *p++ = 31; *p++ = 0x00;
	memcpy(p, std_dc_luma_bits, 16); p += 16;
	memcpy(p, std_dc_luma_vals, 12); p += 12;
	*p++ = 0xff; *p++ = 0xc4; *p++ = 0x00; *p++ = 181; *p++ = 0x10;
	memcpy(p, std_ac_luma_bits, 16); p += 16;
	memcpy(p, std_ac_luma_vals, 162); p += 162;
	if (!gray) {
		*p++ = 0xff; *p++ = 0xc4; *p++ = 0x00; *p++ = 31; *p++ = 0x01;
		memcpy(p, std_dc_chroma_bits, 16); p += 16;
		memcpy(p, std_dc_chroma_vals, 12); p += 12;
		*p++ = 0xff; *p++ = 0xc4; *p++ = 0x00; *p++ = 181; *p++ = 0x11;
		memcpy(p, std_ac_chroma_bits, 16); p += 16;
		memcpy(p, std_ac_chroma_vals, 162); p += 162;
	}
	/* Pad with COM marker so ECS starts on a 64-byte DMA alignment boundary. */
	hdr_len = (p - buf) + (gray ? 8 : 12);
	pad = ALIGN(hdr_len, 64) - hdr_len;
	if (pad < 4)
		pad += 64;
	*p++ = 0xff; *p++ = 0xfe;
	*p++ = (pad - 2) >> 8; *p++ = (pad - 2) & 0xff;
	memset(p, 0, pad - 4); p += pad - 4;

	/* SOS */
	*p++ = 0xff; *p++ = 0xda; *p++ = 0x00; *p++ = gray ? 8 : 12;
	*p++ = gray ? 1 : 3;
	*p++ = 1; *p++ = 0x00;
	if (!gray) {
		*p++ = 2; *p++ = 0x11;
		*p++ = 3; *p++ = 0x11;
	}
	*p++ = 0; *p++ = 63; *p++ = 0;
	return p - buf;
}

static int jpeg_encode(struct jpeg_context *ctx, struct vb2_buffer *src,
		       struct vb2_buffer *dst, u32 *out_bytes)
{
	struct p4_jpeg *jpeg = ctx->jpeg;
	void __iomem *base = jpeg->base;
	struct esp_dma2d_jpeg_cfg dcfg = {};
	struct dma_async_tx_descriptor *desc;
	struct dma_chan *dma;
	dma_cookie_t cookie;
	u8 *out_vaddr = vb2_plane_vaddr(dst, 0);
	dma_addr_t out_dma = vb2_dma_contig_plane_dma_addr(dst, 0);
	u8 qt_y[64], qt_c[64];
	bool gray = (ctx->source.pixelformat == V4L2_PIX_FMT_GREY);
	size_t hdr_len, max_ecs;
	u32 rx_bytes = 0, config, cs;
	unsigned int i;
	int ret;

	if (!out_vaddr)
		return -EFAULT;

	jpeg_scale_qt(qt_y, std_luma_qt, ctx->quality);
	jpeg_scale_qt(qt_c, std_chroma_qt, ctx->quality);
	hdr_len = jpeg_emit_header(out_vaddr, ctx->source.width,
				   ctx->source.height, gray, qt_y, qt_c);
	if (hdr_len + 64 >= vb2_plane_size(dst, 0))
		return -ENOSPC;
	dma_sync_single_for_device(jpeg->dev, out_dma, hdr_len, DMA_TO_DEVICE);
	max_ecs = vb2_plane_size(dst, 0) - hdr_len;

	dcfg.encode = true;
	dcfg.buf_2d = vb2_dma_contig_plane_dma_addr(src, 0);
	dcfg.buf_1d = out_dma + hdr_len;
	dcfg.len_1d = max_ecs;
	dcfg.width = ctx->source.width;
	dcfg.height = ctx->source.height;
	dcfg.rx_bytes_out = &rx_bytes;

	if (gray) {
		cs = 3;
		dcfg.pbyte = 1;
		dcfg.block_w = 8;
		dcfg.block_h = 8;
		dcfg.mb_size = ESP_DMA2D_MB_8X8;
	} else if (ctx->source.pixelformat == V4L2_PIX_FMT_RGB565) {
		cs = 2;
		dcfg.pbyte = 3;
		dcfg.block_w = 16;
		dcfg.block_h = 16;
		dcfg.mb_size = ESP_DMA2D_MB_16X16;
	} else {
		cs = 0;
		dcfg.pbyte = 4;
		dcfg.block_w = 16;
		dcfg.block_h = 16;
		dcfg.mb_size = ESP_DMA2D_MB_16X16;
	}

	dma = dma_request_chan(jpeg->dev, "codec");
	if (IS_ERR(dma))
		return PTR_ERR(dma);

	desc = esp_dma2d_prep_jpeg_xfer(dma, &dcfg, DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
	if (!desc) {
		ret = -ENOMEM;
		goto release_dma;
	}
	desc->callback_result = jpeg_dma_complete;
	desc->callback_param = jpeg;
	cookie = dmaengine_submit(desc);
	ret = dma_submit_error(cookie);
	if (ret)
		goto release_dma;

	config = JPEG_FF_CHECK_EN | JPEG_QNR_FIFO | JPEG_TAILER_EN |
		 FIELD_PREP(JPEG_COLOR_SPACE, cs) |
		 FIELD_PREP(JPEG_SAMPLE_SEL, gray ? 0 : 2) |
		 FIELD_PREP(JPEG_LQNR_TBL_SEL, 0) |
		 FIELD_PREP(JPEG_CQNR_TBL_SEL, 1);
	writel(config | JPEG_SOFT_RST | JPEG_FIFO_RST, base + JPEG_CONFIG);
	writel(config, base + JPEG_CONFIG);

	for (i = 0; i < 64; i++)
		writel(qt_y[i], base + JPEG_QNR0);
	if (!gray) {
		for (i = 0; i < 64; i++)
			writel(qt_c[i], base + JPEG_QNR1);
	}
	writel((dcfg.width << 16) | dcfg.height, base + JPEG_PIC_SIZE);

	reinit_completion(&jpeg->codec_done);
	reinit_completion(&jpeg->dma_done);
	jpeg->codec_status = 0;
	writel(JPEG_DONE | JPEG_ENC_ERRORS, base + JPEG_IRQ_CLR);
	writel(JPEG_DONE | JPEG_ENC_ERRORS, base + JPEG_IRQ_ENA);
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

static void jpeg_worker(struct work_struct *work)
{
	struct p4_jpeg *jpeg = container_of(work, struct p4_jpeg, work);
	struct jpeg_context *ctx = jpeg->running;
	struct v4l2_m2m_ctx *m2m = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(m2m);
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(m2m);
	u32 out_bytes = 0;
	int ret = -ENODEV;

	if (!READ_ONCE(jpeg->disconnected)) {
		if (ctx->source.pixelformat == V4L2_PIX_FMT_JPEG &&
		    ctx->target.pixelformat != V4L2_PIX_FMT_JPEG)
			ret = jpeg_decode(ctx, &src->vb2_buf, &dst->vb2_buf, &out_bytes);
		else if (ctx->source.pixelformat != V4L2_PIX_FMT_JPEG &&
			 ctx->target.pixelformat == V4L2_PIX_FMT_JPEG)
			ret = jpeg_encode(ctx, &src->vb2_buf, &dst->vb2_buf, &out_bytes);
		else
			ret = -EINVAL;
	}
	vb2_set_plane_payload(&dst->vb2_buf, 0, ret ? 0 : out_bytes);
	if (!ret)
		v4l2_m2m_buf_copy_metadata(src, dst, true);
	v4l2_m2m_buf_done_and_job_finish(jpeg->m2m, m2m,
					 ret ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
}

static void jpeg_run(void *data)
{
	struct jpeg_context *ctx = data;

	ctx->jpeg->running = ctx;
	schedule_work(&ctx->jpeg->work);
}

static const struct v4l2_m2m_ops jpeg_m2m_ops = {
	.device_run = jpeg_run,
};

static void jpeg_video_release(struct video_device *video)
{
	struct p4_jpeg *jpeg = video_get_drvdata(video);

	v4l2_m2m_release(jpeg->m2m);
	v4l2_device_unregister(&jpeg->v4l2);
	kfree(jpeg);
}

static int jpeg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *reset;
	struct p4_jpeg *jpeg;
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
		return PTR_ERR(clk);
	reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(reset))
		return PTR_ERR(reset);
	ret = reset_control_reset(reset);
	if (ret)
		return ret;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	dma = dma_request_chan(dev, "codec");
	if (IS_ERR(dma))
		return dev_err_probe(dev, PTR_ERR(dma), "could not request JPEG DMA\n");
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
	INIT_WORK(&jpeg->work, jpeg_worker);
	ret = request_irq(irq, jpeg_interrupt, 0, dev_name(dev), jpeg);
	if (ret)
		goto free_jpeg;
	ret = v4l2_device_register(dev, &jpeg->v4l2);
	if (ret)
		goto free_irq;
	jpeg->m2m = v4l2_m2m_init(&jpeg_m2m_ops);
	if (IS_ERR(jpeg->m2m)) {
		ret = PTR_ERR(jpeg->m2m);
		goto unregister;
	}
	jpeg->video = (struct video_device) {
		.name = "esp32p4-jpeg",
		.v4l2_dev = &jpeg->v4l2,
		.fops = &jpeg_fops,
		.ioctl_ops = &jpeg_ioctls,
		.release = jpeg_video_release,
		.lock = &jpeg->lock,
		.vfl_dir = VFL_DIR_M2M,
		.device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING,
	};
	video_set_drvdata(&jpeg->video, jpeg);
	ret = video_register_device(&jpeg->video, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto release_m2m;
	platform_set_drvdata(pdev, jpeg);
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

static void jpeg_remove(struct platform_device *pdev)
{
	struct p4_jpeg *jpeg = platform_get_drvdata(pdev);

	mutex_lock(&jpeg->lock);
	WRITE_ONCE(jpeg->disconnected, true);
	mutex_unlock(&jpeg->lock);
	v4l2_m2m_suspend(jpeg->m2m);
	flush_work(&jpeg->work);
	free_irq(jpeg->irq, jpeg);
	video_unregister_device(&jpeg->video);
}

static const struct of_device_id jpeg_match[] = {
	{ .compatible = "espressif,esp32p4-jpeg" },
	{}
};
MODULE_DEVICE_TABLE(of, jpeg_match);

static struct platform_driver jpeg_driver = {
	.probe = jpeg_probe,
	.remove = jpeg_remove,
	.driver = {
		.name = "esp32p4-jpeg",
		.of_match_table = jpeg_match,
	},
};
module_platform_driver(jpeg_driver);

MODULE_DESCRIPTION("ESP32-P4 V4L2 JPEG encoder and decoder");
MODULE_LICENSE("GPL");
