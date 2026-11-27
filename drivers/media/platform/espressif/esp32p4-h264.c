// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-P4 rev1.3 H.264 Hardware Baseline Encoder (V4L2 M2M).
 *
 * Encodes NV12 / YUV420 frames into Annex-B H.264 bitstreams using the
 * H.264 core (0x50084000, IRQ 126) and dedicated H.264 2D-DMA (0x500a7000).
 *
 * Hardware reference: ESP32-P4 rev1.3 TRM ch.34 H.264 Encoder + ESP-IDF
 * soc/esp32p4/register/hw_ver1/soc/{h264_reg.h,h264_dma_reg.h}.
 */
#include <linux/bitfield.h>
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
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

/* ESP32-P4 rev1.3 TRM ch.34 H.264 Core (0x50084000) */
#define H264_SYS_CTRL		0x00
#define H264_FRAME_START	BIT(0)
#define H264_FRAME_MODE		BIT(2)
#define H264_SYS_RST_PULSE	BIT(3)

#define H264_GOP_CONF		0x04
#define H264_GOP_NUM		GENMASK(8, 1)

#define H264_A_SYS_MB_RES	0x08
#define H264_MB_Y		GENMASK(6, 0)
#define H264_MB_X		GENMASK(13, 7)

#define H264_A_SYS_CONF		0x0c
#define H264_A_RC_CONF0		0x18
#define H264_A_QP		GENMASK(5, 0)
#define H264_A_RC_CONF1		0x1c
#define H264_A_QP_MIN		GENMASK(12, 7)
#define H264_A_QP_MAX		GENMASK(18, 13)

#define H264_SLICE_HDR_REMAIN	0xac
#define H264_SLICE_HDR_LEN	0xb0
#define H264_BS_THRESHOLD	0xb4
#define H264_SLICE_HDR_BYTE0	0xb8
#define H264_SLICE_HDR_BYTE1	0xbc

#define H264_INT_RAW		0xc0
#define H264_INT_ST		0xc4
#define H264_INT_ENA		0xc8
#define H264_INT_CLR		0xcc
#define H264_FRAME_DONE_INT	BIT(2)

#define H264_CONF		0xd0
#define H264_FRAME_CODE_LEN	0xe0

/* H.264 Dedicated 2D-DMA (0x500a7000): TX CH0 (raw YUV) & RX CH5 (bitstream) */
#define H264_DMA_TX_BASE(ch)	((ch) * 0x100)
#define H264_DMA_RX_BASE(ch)	(0x500 + (ch) * 0x100)
#define H264_DMA_CONF0		0x00
#define H264_DMA_INT_ENA	0x08
#define H264_DMA_INT_CLR	0x10
#define H264_DMA_LINK_CONF	0x1c
#define H264_DMA_LINK_ADDR	0x20
#define H264_DMA_RST		BIT(24)
#define H264_DMA_BURST_EN	(BIT(2) | (4U << 6))
#define H264_DMA_TX_START	BIT(21)
#define H264_DMA_RX_START	BIT(22)

#define H264_DESC_OWNER		BIT(31)
#define H264_DESC_EOF		BIT(30)
#define H264_DESC_2D		BIT(29)

#define H264_MIN_DIM		80
#define H264_MAX_W		1920
#define H264_MAX_H		1088
#define H264_TIMEOUT_MS		500

struct h264_dma_desc {
	__le32 control;
	__le32 frame;
	__le32 position;
	__le32 buffer;
	__le32 next;
} __aligned(8);

struct p4_h264 {
	struct device *dev;
	void __iomem *core;
	void __iomem *dma;
	struct clk *clk;
	struct reset_control *reset;
	int irq;

	struct v4l2_device v4l2;
	struct video_device video;
	struct v4l2_m2m_dev *m2m;
	struct mutex lock;
	struct work_struct work;
	struct completion frame_done;
	struct p4_h264_ctx *running;
	bool disconnected;

	struct h264_dma_desc *descs;
	dma_addr_t descs_dma;
};

struct p4_h264_ctx {
	struct v4l2_fh fh;
	struct p4_h264 *h264;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_pix_format src_fmt;
	struct v4l2_pix_format dst_fmt;
	u32 qp;
	u32 qp_min;
	u32 qp_max;
	u32 gop_size;
	u32 frame_idx;
};

static struct p4_h264_ctx *h264_ctx(struct file *file)
{
	return container_of(file->private_data, struct p4_h264_ctx, fh);
}

static void h264_bound_fmt(struct v4l2_pix_format *pix, bool output)
{
	pix->width = round_up(clamp_t(u32, pix->width ?: 320,
				      H264_MIN_DIM, H264_MAX_W), 16);
	pix->height = round_up(clamp_t(u32, pix->height ?: 240,
				       H264_MIN_DIM, H264_MAX_H), 16);
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_REC709;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_DEFAULT;
	pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	if (output) {
		if (pix->pixelformat != V4L2_PIX_FMT_NV12 &&
		    pix->pixelformat != V4L2_PIX_FMT_YUV420)
			pix->pixelformat = V4L2_PIX_FMT_NV12;
		pix->bytesperline = pix->width;
		pix->sizeimage = pix->width * pix->height * 3 / 2;
	} else {
		pix->pixelformat = V4L2_PIX_FMT_H264;
		pix->bytesperline = 0;
		pix->sizeimage = clamp_t(u32, pix->sizeimage ?: pix->width * pix->height,
					 SZ_64K, SZ_2M);
	}
}

static irqreturn_t p4_h264_irq(int irq, void *data)
{
	struct p4_h264 *h = data;
	u32 st = readl(h->core + H264_INT_ST);

	if (!st)
		return IRQ_NONE;
	writel(st, h->core + H264_INT_CLR);
	if (st & H264_FRAME_DONE_INT)
		complete(&h->frame_done);
	return IRQ_HANDLED;
}

static int h264_encode_frame(struct p4_h264_ctx *ctx, struct vb2_buffer *src,
			     struct vb2_buffer *dst, u32 *out_bytes)
{
	struct p4_h264 *h = ctx->h264;
	dma_addr_t src_dma = vb2_dma_contig_plane_dma_addr(src, 0);
	dma_addr_t dst_dma = vb2_dma_contig_plane_dma_addr(dst, 0);
	size_t src_len = ctx->src_fmt.sizeimage;
	size_t dst_cap = vb2_plane_size(dst, 0);
	u32 mb_x = ctx->src_fmt.width / 16;
	u32 mb_y = ctx->src_fmt.height / 16;
	u32 low, high;

	/* Reset H.264 core and configure GOP / MB resolution / QP. */
	writel(H264_SYS_RST_PULSE | H264_FRAME_MODE, h->core + H264_SYS_CTRL);
	writel(H264_FRAME_MODE, h->core + H264_SYS_CTRL);
	writel(FIELD_PREP(H264_GOP_NUM, ctx->gop_size), h->core + H264_GOP_CONF);
	writel(FIELD_PREP(H264_MB_X, mb_x) | FIELD_PREP(H264_MB_Y, mb_y),
	       h->core + H264_A_SYS_MB_RES);
	writel(FIELD_PREP(H264_A_QP, ctx->qp), h->core + H264_A_RC_CONF0);
	writel(FIELD_PREP(H264_A_QP_MIN, ctx->qp_min) |
	       FIELD_PREP(H264_A_QP_MAX, ctx->qp_max),
	       h->core + H264_A_RC_CONF1);

	/* Prepare TX CH0 (raw YUV) and RX CH5 (H.264 bitstream) descriptors. */
	memset(h->descs, 0, 2 * sizeof(*h->descs));
	low = src_len & 0x3fff;
	high = src_len >> 14;
	h->descs[0].control = cpu_to_le32(H264_DESC_OWNER | H264_DESC_EOF |
					  (low << 14) | low);
	h->descs[0].frame = cpu_to_le32((1U << 28) | (high << 14) | high);
	h->descs[0].buffer = cpu_to_le32(src_dma);

	low = dst_cap & 0x3fff;
	high = dst_cap >> 14;
	h->descs[1].control = cpu_to_le32(H264_DESC_OWNER | low);
	h->descs[1].frame = cpu_to_le32((1U << 28) | high);
	h->descs[1].buffer = cpu_to_le32(dst_dma);
	dma_wmb();

	/* Reset and arm H264 2D-DMA TX CH0 and RX CH5. */
	writel(H264_DMA_RST, h->dma + H264_DMA_TX_BASE(0) + H264_DMA_CONF0);
	writel(H264_DMA_BURST_EN, h->dma + H264_DMA_TX_BASE(0) + H264_DMA_CONF0);
	writel(h->descs_dma, h->dma + H264_DMA_TX_BASE(0) + H264_DMA_LINK_ADDR);

	writel(H264_DMA_RST, h->dma + H264_DMA_RX_BASE(5) + H264_DMA_CONF0);
	writel(H264_DMA_BURST_EN, h->dma + H264_DMA_RX_BASE(5) + H264_DMA_CONF0);
	writel(h->descs_dma + sizeof(*h->descs),
	       h->dma + H264_DMA_RX_BASE(5) + H264_DMA_LINK_ADDR);

	reinit_completion(&h->frame_done);
	writel(U32_MAX, h->core + H264_INT_CLR);
	writel(H264_FRAME_DONE_INT, h->core + H264_INT_ENA);

	writel(H264_DMA_TX_START, h->dma + H264_DMA_TX_BASE(0) + H264_DMA_LINK_CONF);
	writel(H264_DMA_RX_START, h->dma + H264_DMA_RX_BASE(5) + H264_DMA_LINK_CONF);
	writel(H264_FRAME_MODE | H264_FRAME_START, h->core + H264_SYS_CTRL);

	if (!wait_for_completion_timeout(&h->frame_done, msecs_to_jiffies(H264_TIMEOUT_MS))) {
		writel(0, h->core + H264_INT_ENA);
		return -ETIMEDOUT;
	}
	writel(0, h->core + H264_INT_ENA);

	*out_bytes = readl(h->core + H264_FRAME_CODE_LEN);
	if (!*out_bytes || *out_bytes > dst_cap)
		return -EIO;
	ctx->frame_idx++;
	return 0;
}

static void h264_worker(struct work_struct *work)
{
	struct p4_h264 *h = container_of(work, struct p4_h264, work);
	struct p4_h264_ctx *ctx = h->running;
	struct v4l2_m2m_ctx *m2m = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(m2m);
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(m2m);
	u32 out_bytes = 0;
	int ret = -ENODEV;

	if (!READ_ONCE(h->disconnected))
		ret = h264_encode_frame(ctx, &src->vb2_buf, &dst->vb2_buf, &out_bytes);
	vb2_set_plane_payload(&dst->vb2_buf, 0, ret ? 0 : out_bytes);
	if (!ret)
		v4l2_m2m_buf_copy_metadata(src, dst, true);
	v4l2_m2m_buf_done_and_job_finish(h->m2m, m2m,
					 ret ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
}

static void h264_run(void *data)
{
	struct p4_h264_ctx *ctx = data;

	ctx->h264->running = ctx;
	schedule_work(&ctx->h264->work);
}

static const struct v4l2_m2m_ops h264_m2m_ops = {
	.device_run = h264_run,
};

static int h264_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct p4_h264_ctx *ctx = container_of(ctrl->handler, struct p4_h264_ctx, ctrls);

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP:
		ctx->qp = ctrl->val;
		return 0;
	case V4L2_CID_MPEG_VIDEO_H264_MIN_QP:
		ctx->qp_min = ctrl->val;
		return 0;
	case V4L2_CID_MPEG_VIDEO_H264_MAX_QP:
		ctx->qp_max = ctrl->val;
		return 0;
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		ctx->gop_size = ctrl->val;
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops h264_ctrl_ops = {
	.s_ctrl = h264_s_ctrl,
};

static int h264_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	struct p4_h264 *h = video_drvdata(file);

	strscpy(cap->driver, "esp32p4-h264", sizeof(cap->driver));
	strscpy(cap->card, "ESP32-P4 H.264 Encoder", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s", dev_name(h->dev));
	return 0;
}

static int h264_enum_fmt_out(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	if (f->index == 0)
		f->pixelformat = V4L2_PIX_FMT_NV12;
	else if (f->index == 1)
		f->pixelformat = V4L2_PIX_FMT_YUV420;
	else
		return -EINVAL;
	return 0;
}

static int h264_enum_fmt_cap(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_H264;
	return 0;
}

static int h264_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct p4_h264_ctx *ctx = h264_ctx(file);

	f->fmt.pix = V4L2_TYPE_IS_OUTPUT(f->type) ? ctx->src_fmt : ctx->dst_fmt;
	return 0;
}

static int h264_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	h264_bound_fmt(&f->fmt.pix, V4L2_TYPE_IS_OUTPUT(f->type));
	return 0;
}

static int h264_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct p4_h264_ctx *ctx = h264_ctx(file);

	if (vb2_is_busy(v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type)))
		return -EBUSY;
	h264_bound_fmt(&f->fmt.pix, V4L2_TYPE_IS_OUTPUT(f->type));
	if (V4L2_TYPE_IS_OUTPUT(f->type))
		ctx->src_fmt = f->fmt.pix;
	else
		ctx->dst_fmt = f->fmt.pix;
	return 0;
}

static int h264_reqbufs(struct file *file, void *fh, struct v4l2_requestbuffers *req)
{
	if (req->count && req->memory == V4L2_MEMORY_MMAP)
		req->flags |= V4L2_MEMORY_FLAG_NON_COHERENT;
	return v4l2_m2m_ioctl_reqbufs(file, fh, req);
}

static const struct v4l2_ioctl_ops h264_ioctl_ops = {
	.vidioc_querycap = h264_querycap,
	.vidioc_enum_fmt_vid_out = h264_enum_fmt_out,
	.vidioc_enum_fmt_vid_cap = h264_enum_fmt_cap,
	.vidioc_g_fmt_vid_out = h264_g_fmt,
	.vidioc_g_fmt_vid_cap = h264_g_fmt,
	.vidioc_try_fmt_vid_out = h264_try_fmt,
	.vidioc_try_fmt_vid_cap = h264_try_fmt,
	.vidioc_s_fmt_vid_out = h264_s_fmt,
	.vidioc_s_fmt_vid_cap = h264_s_fmt,
	.vidioc_reqbufs = h264_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
};

static int h264_queue_setup(struct vb2_queue *q, unsigned int *nbuf,
			    unsigned int *nplanes, unsigned int sizes[],
			    struct device *alloc_devs[])
{
	struct p4_h264_ctx *ctx = vb2_get_drv_priv(q);
	u32 sz = V4L2_TYPE_IS_OUTPUT(q->type) ? ctx->src_fmt.sizeimage
					      : ctx->dst_fmt.sizeimage;

	if (*nplanes)
		return *nplanes == 1 && sizes[0] >= sz ? 0 : -EINVAL;
	*nplanes = 1;
	sizes[0] = sz;
	return 0;
}

static int h264_buf_prepare(struct vb2_buffer *vb)
{
	struct p4_h264_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	u32 sz = V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type) ? ctx->src_fmt.sizeimage
							  : ctx->dst_fmt.sizeimage;

	if (vb2_plane_size(vb, 0) < sz)
		return -EINVAL;
	return 0;
}

static void h264_buf_queue(struct vb2_buffer *vb)
{
	struct p4_h264_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static void h264_stop_streaming(struct vb2_queue *q)
{
	struct p4_h264_ctx *ctx = vb2_get_drv_priv(q);
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

static const struct vb2_ops h264_vb2_ops = {
	.queue_setup = h264_queue_setup,
	.buf_prepare = h264_buf_prepare,
	.buf_queue = h264_buf_queue,
	.stop_streaming = h264_stop_streaming,
};

static int h264_queue_init(void *data, struct vb2_queue *src, struct vb2_queue *dst)
{
	struct p4_h264_ctx *ctx = data;
	int ret;

	*src = (struct vb2_queue) {
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
		.io_modes = VB2_MMAP,
		.drv_priv = ctx,
		.buf_struct_size = sizeof(struct v4l2_m2m_buffer),
		.ops = &h264_vb2_ops,
		.mem_ops = &vb2_dma_contig_memops,
		.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY,
		.lock = &ctx->h264->lock,
		.dev = ctx->h264->dev,
		.allow_cache_hints = 1,
	};
	*dst = *src;
	dst->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ret = vb2_queue_init(src);
	if (ret)
		return ret;
	return vb2_queue_init(dst);
}

static int h264_open(struct file *file)
{
	struct p4_h264 *h = video_drvdata(file);
	struct p4_h264_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->h264 = h;
	ctx->qp = 26;
	ctx->qp_min = 16;
	ctx->qp_max = 40;
	ctx->gop_size = 30;
	ctx->src_fmt.width = 320;
	ctx->src_fmt.height = 240;
	ctx->src_fmt.pixelformat = V4L2_PIX_FMT_NV12;
	h264_bound_fmt(&ctx->src_fmt, true);
	ctx->dst_fmt = ctx->src_fmt;
	h264_bound_fmt(&ctx->dst_fmt, false);

	v4l2_ctrl_handler_init(&ctx->ctrls, 4);
	v4l2_ctrl_new_std(&ctx->ctrls, &h264_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP, 0, 51, 1, 26);
	v4l2_ctrl_new_std(&ctx->ctrls, &h264_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_MIN_QP, 0, 51, 1, 16);
	v4l2_ctrl_new_std(&ctx->ctrls, &h264_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_MAX_QP, 0, 51, 1, 40);
	v4l2_ctrl_new_std(&ctx->ctrls, &h264_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_GOP_SIZE, 1, 255, 1, 30);
	if (ctx->ctrls.error) {
		ret = ctx->ctrls.error;
		goto err_ctrl;
	}

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	ctx->fh.ctrl_handler = &ctx->ctrls;
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(h->m2m, ctx, h264_queue_init);
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

static int h264_release(struct file *file)
{
	struct p4_h264_ctx *ctx = h264_ctx(file);

	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_ctrl_handler_free(&ctx->ctrls);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations h264_fops = {
	.owner = THIS_MODULE,
	.open = h264_open,
	.release = h264_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
#ifndef CONFIG_MMU
	.get_unmapped_area = v4l2_m2m_get_unmapped_area,
#endif
};

static int p4_h264_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct p4_h264 *h;
	int ret;

	h = devm_kzalloc(dev, sizeof(*h), GFP_KERNEL);
	if (!h)
		return -ENOMEM;
	h->dev = dev;
	mutex_init(&h->lock);
	init_completion(&h->frame_done);
	INIT_WORK(&h->work, h264_worker);
	platform_set_drvdata(pdev, h);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	h->core = devm_platform_ioremap_resource_byname(pdev, "core");
	if (IS_ERR(h->core))
		return PTR_ERR(h->core);
	h->dma = devm_platform_ioremap_resource_byname(pdev, "dma");
	if (IS_ERR(h->dma))
		return PTR_ERR(h->dma);
	h->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(h->clk))
		return PTR_ERR(h->clk);
	h->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(h->reset))
		return PTR_ERR(h->reset);
	ret = reset_control_reset(h->reset);
	if (ret)
		return ret;

	h->descs = dmam_alloc_coherent(dev, 2 * sizeof(*h->descs),
				       &h->descs_dma, GFP_KERNEL);
	if (!h->descs)
		return -ENOMEM;

	h->irq = platform_get_irq(pdev, 0);
	if (h->irq < 0)
		return h->irq;
	ret = devm_request_irq(dev, h->irq, p4_h264_irq, 0, dev_name(dev), h);
	if (ret)
		return ret;

	ret = v4l2_device_register(dev, &h->v4l2);
	if (ret)
		return ret;
	h->m2m = v4l2_m2m_init(&h264_m2m_ops);
	if (IS_ERR(h->m2m)) {
		ret = PTR_ERR(h->m2m);
		goto err_v4l2;
	}
	h->video = (struct video_device) {
		.name = "esp32p4-h264",
		.vfl_dir = VFL_DIR_M2M,
		.fops = &h264_fops,
		.ioctl_ops = &h264_ioctl_ops,
		.release = video_device_release_empty,
		.v4l2_dev = &h->v4l2,
		.lock = &h->lock,
		.device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING,
	};
	video_set_drvdata(&h->video, h);
	ret = video_register_device(&h->video, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_m2m;

	dev_info(dev, "registered H.264 encoder /dev/%s\n",
		 video_device_node_name(&h->video));
	return 0;

err_m2m:
	v4l2_m2m_release(h->m2m);
err_v4l2:
	v4l2_device_unregister(&h->v4l2);
	return ret;
}

static void p4_h264_remove(struct platform_device *pdev)
{
	struct p4_h264 *h = platform_get_drvdata(pdev);

	WRITE_ONCE(h->disconnected, true);
	v4l2_m2m_suspend(h->m2m);
	flush_work(&h->work);
	video_unregister_device(&h->video);
	v4l2_m2m_release(h->m2m);
	v4l2_device_unregister(&h->v4l2);
}

static const struct of_device_id p4_h264_match[] = {
	{ .compatible = "espressif,esp32p4-h264" },
	{ }
};
MODULE_DEVICE_TABLE(of, p4_h264_match);

static struct platform_driver p4_h264_driver = {
	.probe = p4_h264_probe,
	.remove = p4_h264_remove,
	.driver = {
		.name = "esp32p4-h264",
		.of_match_table = p4_h264_match,
	},
};
module_platform_driver(p4_h264_driver);

MODULE_DESCRIPTION("ESP32-P4 V4L2 M2M H.264 encoder driver");
MODULE_LICENSE("GPL");
