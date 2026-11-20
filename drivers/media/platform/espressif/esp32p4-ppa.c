// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-P4 Pixel Processing Accelerator. */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/workqueue.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "../../../dma/espressif-dma2d.h"

/* TRM ch.40 PPA — SRM color mode, conversion parameters and status. */
#define PPA_IRQ_CLEAR		0x1c
#define PPA_SRM_IRQS		(BIT(0) | BIT(2))
#define PPA_SRM_COLOR		0x20
#define PPA_SRM_RX_RGB565	FIELD_PREP(GENMASK(3, 0), 2)
#define PPA_SRM_TX_RGB565	FIELD_PREP(GENMASK(7, 4), 2)
#define PPA_SRM_CTRL		0x64
#define PPA_SCALE_X_INT		GENMASK(7, 0)
#define PPA_SCALE_X_FRAC	GENMASK(11, 8)
#define PPA_SCALE_Y_INT		GENMASK(19, 12)
#define PPA_SCALE_Y_FRAC	GENMASK(23, 20)
#define PPA_ROTATE		GENMASK(25, 24)
#define PPA_SRM_RESET		BIT(26)
#define PPA_SRM_START		BIT(27)
#define PPA_MIRROR_X		BIT(28)
#define PPA_MIRROR_Y		BIT(29)
#define PPA_REG_CONF		0x6c
#define PPA_REG_CLK		BIT(0)
#define PPA_SRM_ERROR		0x78
#define PPA_SRM_STATUS		0x7c

#define PPA_MAX_DIMENSION	1280
#define PPA_MIN_DIMENSION	2
#define PPA_SCALE_FRAC		16
#define PPA_TIMEOUT_MS		100

struct ppa_device {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct reset_control *reset;
	struct v4l2_device v4l2;
	struct video_device video;
	struct v4l2_m2m_dev *m2m;
	struct mutex lock;
	struct work_struct work;
	struct completion dma_done;
	enum dmaengine_tx_result dma_result;
	struct ppa_context *running;
	bool disconnected;
};

struct ppa_context {
	struct v4l2_fh fh;
	struct ppa_device *ppa;
	struct v4l2_ctrl_handler controls;
	struct v4l2_ctrl *rotate;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
	struct v4l2_pix_format source;
	struct v4l2_pix_format target;
};

static struct ppa_context *ppa_context(struct file *file)
{
	return container_of(file->private_data, struct ppa_context, fh);
}

static struct v4l2_pix_format *ppa_format(struct ppa_context *ctx,
					  enum v4l2_buf_type type)
{
	return V4L2_TYPE_IS_OUTPUT(type) ? &ctx->source : &ctx->target;
}

static void ppa_bound_format(struct v4l2_pix_format *pix)
{
	pix->width = ALIGN_DOWN(clamp_t(u32, pix->width, PPA_MIN_DIMENSION,
				      PPA_MAX_DIMENSION), 2);
	pix->height = clamp_t(u32, pix->height, PPA_MIN_DIMENSION, PPA_MAX_DIMENSION);
	pix->pixelformat = V4L2_PIX_FMT_RGB565;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width * sizeof(u16);
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_DEFAULT;
	pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static int ppa_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	struct ppa_device *ppa = video_drvdata(file);

	strscpy(cap->driver, "esp32p4-ppa", sizeof(cap->driver));
	strscpy(cap->card, "ESP32-P4 PPA", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s", dev_name(ppa->dev));
	return 0;
}

static int ppa_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *fmt)
{
	if (fmt->index)
		return -EINVAL;
	fmt->pixelformat = V4L2_PIX_FMT_RGB565;
	return 0;
}

static int ppa_try_fmt(struct file *file, void *fh, struct v4l2_format *fmt)
{
	ppa_bound_format(&fmt->fmt.pix);
	return 0;
}

static int ppa_get_fmt(struct file *file, void *fh, struct v4l2_format *fmt)
{
	fmt->fmt.pix = *ppa_format(ppa_context(file), fmt->type);
	return 0;
}

static int ppa_set_fmt(struct file *file, void *fh, struct v4l2_format *fmt)
{
	struct ppa_context *ctx = ppa_context(file);
	struct vb2_queue *q = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, fmt->type);

	if (vb2_is_busy(q))
		return -EBUSY;
	ppa_bound_format(&fmt->fmt.pix);
	*ppa_format(ctx, fmt->type) = fmt->fmt.pix;
	return 0;
}

static int ppa_reqbufs(struct file *file, void *fh, struct v4l2_requestbuffers *req)
{
	/* Allocate images in PSRAM and leave the coherent SRAM for DMA descriptors. */
	if (req->count && req->memory == V4L2_MEMORY_MMAP)
		req->flags |= V4L2_MEMORY_FLAG_NON_COHERENT;

	return v4l2_m2m_ioctl_reqbufs(file, fh, req);
}

static const struct v4l2_ioctl_ops ppa_ioctl_ops = {
	.vidioc_querycap = ppa_querycap,
	.vidioc_enum_fmt_vid_cap = ppa_enum_fmt,
	.vidioc_enum_fmt_vid_out = ppa_enum_fmt,
	.vidioc_try_fmt_vid_cap = ppa_try_fmt,
	.vidioc_try_fmt_vid_out = ppa_try_fmt,
	.vidioc_g_fmt_vid_cap = ppa_get_fmt,
	.vidioc_g_fmt_vid_out = ppa_get_fmt,
	.vidioc_s_fmt_vid_cap = ppa_set_fmt,
	.vidioc_s_fmt_vid_out = ppa_set_fmt,
	.vidioc_reqbufs = ppa_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
};

static int ppa_queue_setup(struct vb2_queue *q, unsigned int *buffers,
			   unsigned int *planes, unsigned int sizes[],
			   struct device *alloc_devs[])
{
	struct ppa_context *ctx = vb2_get_drv_priv(q);
	unsigned int size = ppa_format(ctx, q->type)->sizeimage;

	if (*planes)
		return *planes == 1 && sizes[0] >= size ? 0 : -EINVAL;
	*planes = 1;
	sizes[0] = size;
	return 0;
}

static int ppa_buffer_prepare(struct vb2_buffer *buf)
{
	struct vb2_queue *q = buf->vb2_queue;
	struct ppa_context *ctx = vb2_get_drv_priv(q);
	const struct v4l2_pix_format *pix = ppa_format(ctx, q->type);

	if (vb2_plane_size(buf, 0) < pix->sizeimage)
		return -EINVAL;
	if (V4L2_TYPE_IS_OUTPUT(q->type) && vb2_get_plane_payload(buf, 0) < pix->sizeimage)
		return -EINVAL;
	vb2_set_plane_payload(buf, 0, pix->sizeimage);
	return 0;
}

static void ppa_buffer_queue(struct vb2_buffer *buf)
{
	struct ppa_context *ctx = vb2_get_drv_priv(buf->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(buf));
}

static void ppa_stop_streaming(struct vb2_queue *q)
{
	struct ppa_context *ctx = vb2_get_drv_priv(q);
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

static const struct vb2_ops ppa_queue_ops = {
	.queue_setup = ppa_queue_setup,
	.buf_prepare = ppa_buffer_prepare,
	.buf_queue = ppa_buffer_queue,
	.stop_streaming = ppa_stop_streaming,
};

static int ppa_queue_init(void *data, struct vb2_queue *src, struct vb2_queue *dst)
{
	struct ppa_context *ctx = data;
	struct ppa_device *ppa = ctx->ppa;
	int ret;

	*src = (struct vb2_queue) {
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
		.io_modes = VB2_MMAP | VB2_USERPTR,
		.drv_priv = ctx,
		.buf_struct_size = sizeof(struct v4l2_m2m_buffer),
		.ops = &ppa_queue_ops,
		.mem_ops = &vb2_dma_contig_memops,
		.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY,
		.lock = &ppa->lock,
		.dev = ppa->dev,
		.allow_cache_hints = 1,
	};
	*dst = *src;
	dst->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	ret = vb2_queue_init(src);
	if (ret)
		return ret;
	return vb2_queue_init(dst);
}

static int ppa_open(struct file *file)
{
	struct ppa_device *ppa = video_drvdata(file);
	struct ppa_context *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->ppa = ppa;
	ctx->source.width = 320;
	ctx->source.height = 240;
	ppa_bound_format(&ctx->source);
	ctx->target = ctx->source;
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;

	v4l2_ctrl_handler_init(&ctx->controls, 3);
	ctx->rotate = v4l2_ctrl_new_std(&ctx->controls, NULL, V4L2_CID_ROTATE, 0, 270, 90, 0);
	ctx->hflip = v4l2_ctrl_new_std(&ctx->controls, NULL, V4L2_CID_HFLIP, 0, 1, 1, 0);
	ctx->vflip = v4l2_ctrl_new_std(&ctx->controls, NULL, V4L2_CID_VFLIP, 0, 1, 1, 0);
	ret = ctx->controls.error;
	if (ret)
		goto free_ctx;
	ctx->fh.ctrl_handler = &ctx->controls;
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(ppa->m2m, ctx, ppa_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto free_ctx;
	}

	v4l2_fh_add(&ctx->fh, file);
	return 0;

free_ctx:
	v4l2_ctrl_handler_free(&ctx->controls);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int ppa_release(struct file *file)
{
	struct ppa_context *ctx = ppa_context(file);

	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_ctrl_handler_free(&ctx->controls);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations ppa_fops = {
	.owner = THIS_MODULE,
	.open = ppa_open,
	.release = ppa_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
#ifndef CONFIG_MMU
	.get_unmapped_area = v4l2_m2m_get_unmapped_area,
#endif
};

static int ppa_scale_ctrl(struct ppa_context *ctx, u32 *config)
{
	unsigned int rotation = v4l2_ctrl_g_ctrl(ctx->rotate) / 90;
	unsigned int width = ctx->target.width, height = ctx->target.height;
	unsigned int x, y;

	if (rotation & 1)
		swap(width, height);
	x = DIV_ROUND_UP(width * PPA_SCALE_FRAC, ctx->source.width);
	y = DIV_ROUND_UP(height * PPA_SCALE_FRAC, ctx->source.height);
	if (!FIELD_FIT(PPA_SCALE_X_INT, x / PPA_SCALE_FRAC) ||
	    !FIELD_FIT(PPA_SCALE_Y_INT, y / PPA_SCALE_FRAC) ||
	    x * ctx->source.width / PPA_SCALE_FRAC != width ||
	    y * ctx->source.height / PPA_SCALE_FRAC != height)
		return -ERANGE;

	/* TRM reg.40.19: integer bits come before fractional bits. */
	*config = FIELD_PREP(PPA_SCALE_X_INT, x / PPA_SCALE_FRAC) |
		  FIELD_PREP(PPA_SCALE_X_FRAC, x % PPA_SCALE_FRAC) |
		  FIELD_PREP(PPA_SCALE_Y_INT, y / PPA_SCALE_FRAC) |
		  FIELD_PREP(PPA_SCALE_Y_FRAC, y % PPA_SCALE_FRAC) |
		  FIELD_PREP(PPA_ROTATE, rotation);
	if (v4l2_ctrl_g_ctrl(ctx->hflip))
		*config |= PPA_MIRROR_X;
	if (v4l2_ctrl_g_ctrl(ctx->vflip))
		*config |= PPA_MIRROR_Y;
	return 0;
}

static void ppa_dma_complete(void *data, const struct dmaengine_result *result)
{
	struct ppa_device *ppa = data;

	ppa->dma_result = result->result;
	complete(&ppa->dma_done);
}

static int ppa_process(struct ppa_context *ctx, struct vb2_v4l2_buffer *src,
		       struct vb2_v4l2_buffer *dst)
{
	struct ppa_device *ppa = ctx->ppa;
	struct esp_dma2d_image source = {
		.address = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0),
		.width = ctx->source.width,
		.height = ctx->source.height,
	};
	struct esp_dma2d_image target = {
		.address = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0),
		.width = ctx->target.width,
		.height = ctx->target.height,
	};
	struct dma_async_tx_descriptor *desc;
	struct dma_chan *chan;
	dma_cookie_t cookie;
	u32 config, error;
	int ret;

	ret = ppa_scale_ctrl(ctx, &config);
	if (ret)
		return ret;
	chan = dma_request_chan(ppa->dev, "srm");
	if (IS_ERR(chan))
		return PTR_ERR(chan);
	ret = clk_prepare_enable(ppa->clk);
	if (ret)
		goto release_channel;
	/*
	 * TRM ch.40.7.1–2: initialize the accelerator before configuring
	 * SRM. In particular, rotation changes must clear its prior block
	 * dimensions before associating the next pair of DMA descriptors.
	 */
	ret = reset_control_reset(ppa->reset);
	if (ret)
		goto release;
	writel(PPA_REG_CLK, ppa->base + PPA_REG_CONF);

	desc = esp_dma2d_prep_ppa_srm(chan, &source, &target,
				     DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
	if (!desc) {
		ret = -EIO;
		goto release;
	}
	reinit_completion(&ppa->dma_done);
	desc->callback_result = ppa_dma_complete;
	desc->callback_param = ppa;
	cookie = dmaengine_submit(desc);
	ret = dma_submit_error(cookie);
	if (ret)
		goto release;

	/* TRM ch.40.7.2: mount both descriptor ports before starting SRM. */
	writel(PPA_SRM_IRQS, ppa->base + PPA_IRQ_CLEAR);
	dma_async_issue_pending(chan);
	writel(PPA_SRM_RX_RGB565 | PPA_SRM_TX_RGB565, ppa->base + PPA_SRM_COLOR);
	writel(config, ppa->base + PPA_SRM_CTRL);
	writel(config | PPA_SRM_START, ppa->base + PPA_SRM_CTRL);
	if (!wait_for_completion_timeout(&ppa->dma_done, msecs_to_jiffies(PPA_TIMEOUT_MS)))
		ret = -ETIMEDOUT;
	else if (ppa->dma_result != DMA_TRANS_NOERROR)
		ret = -EIO;

	error = readl(ppa->base + PPA_SRM_ERROR);
	if (error)
		ret = -EIO;
	if (ret) {
		int stop;

		dev_err_ratelimited(ppa->dev, "SRM failed: %d, parameter error %#x, state %#x\n",
				    ret, error, readl(ppa->base + PPA_SRM_STATUS));
		writel(PPA_SRM_RESET, ppa->base + PPA_SRM_CTRL);
		writel(0, ppa->base + PPA_SRM_CTRL);
		stop = dmaengine_terminate_sync(chan);
		if (stop)
			dev_err(ppa->dev, "failed to stop 2D DMA: %d\n", stop);
	}

release:
	clk_disable_unprepare(ppa->clk);
release_channel:
	dma_release_channel(chan);
	return ret;
}

static void ppa_worker(struct work_struct *work)
{
	struct ppa_device *ppa = container_of(work, struct ppa_device, work);
	struct ppa_context *ctx = ppa->running;
	struct v4l2_m2m_ctx *m2m = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(m2m);
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(m2m);
	int ret = -ENODEV;

	if (!READ_ONCE(ppa->disconnected))
		ret = ppa_process(ctx, src, dst);
	if (!ret)
		v4l2_m2m_buf_copy_metadata(src, dst, true);
	else
		vb2_set_plane_payload(&dst->vb2_buf, 0, 0);
	v4l2_m2m_buf_done_and_job_finish(ppa->m2m, m2m,
					ret ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
}

static void ppa_run(void *data)
{
	struct ppa_context *ctx = data;

	ctx->ppa->running = ctx;
	schedule_work(&ctx->ppa->work);
}

static const struct v4l2_m2m_ops ppa_m2m_ops = {
	.device_run = ppa_run,
};

static void ppa_video_release(struct video_device *video)
{
	struct ppa_device *ppa = video_get_drvdata(video);

	v4l2_m2m_release(ppa->m2m);
	v4l2_device_unregister(&ppa->v4l2);
	kfree(ppa);
}

static int ppa_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *reset;
	struct ppa_device *ppa;
	void __iomem *base;
	struct clk *clk;
	int ret;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);
	clk = devm_clk_get(dev, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);
	reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(reset))
		return PTR_ERR(reset);
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	ppa = kzalloc(sizeof(*ppa), GFP_KERNEL);
	if (!ppa)
		return -ENOMEM;
	ppa->dev = dev;
	ppa->base = base;
	ppa->clk = clk;
	ppa->reset = reset;
	mutex_init(&ppa->lock);
	init_completion(&ppa->dma_done);
	INIT_WORK(&ppa->work, ppa_worker);

	ret = v4l2_device_register(dev, &ppa->v4l2);
	if (ret)
		goto free_ppa;
	ppa->m2m = v4l2_m2m_init(&ppa_m2m_ops);
	if (IS_ERR(ppa->m2m)) {
		ret = PTR_ERR(ppa->m2m);
		goto unregister;
	}

	ppa->video = (struct video_device) {
		.name = "esp32p4-ppa",
		.v4l2_dev = &ppa->v4l2,
		.fops = &ppa_fops,
		.ioctl_ops = &ppa_ioctl_ops,
		.release = ppa_video_release,
		.lock = &ppa->lock,
		.vfl_dir = VFL_DIR_M2M,
		.device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING,
	};
	video_set_drvdata(&ppa->video, ppa);
	ret = video_register_device(&ppa->video, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto release_m2m;
	platform_set_drvdata(pdev, ppa);
	return 0;

release_m2m:
	v4l2_m2m_release(ppa->m2m);
unregister:
	v4l2_device_unregister(&ppa->v4l2);
free_ppa:
	kfree(ppa);
	return ret;
}

static void ppa_remove(struct platform_device *pdev)
{
	struct ppa_device *ppa = platform_get_drvdata(pdev);

	mutex_lock(&ppa->lock);
	WRITE_ONCE(ppa->disconnected, true);
	mutex_unlock(&ppa->lock);
	v4l2_m2m_suspend(ppa->m2m);
	flush_work(&ppa->work);
	video_unregister_device(&ppa->video);
}

static const struct of_device_id ppa_match[] = {
	{ .compatible = "espressif,esp32p4-ppa" },
	{ }
};
MODULE_DEVICE_TABLE(of, ppa_match);

static struct platform_driver ppa_driver = {
	.probe = ppa_probe,
	.remove = ppa_remove,
	.driver = {
		.name = "esp32p4-ppa",
		.of_match_table = ppa_match,
	},
};
module_platform_driver(ppa_driver);

MODULE_DESCRIPTION("Espressif ESP32-P4 V4L2 Pixel Processing Accelerator");
MODULE_LICENSE("GPL");
