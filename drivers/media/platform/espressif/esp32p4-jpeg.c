// SPDX-License-Identifier: GPL-2.0-only
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
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-jpeg.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "../../../dma/espressif-dma2d.h"

/* ESP32-P4 rev1.3 TRM ch.33 JPEG — configuration and FIFO table ports. */
#define JPEG_CONFIG		0x00
#define JPEG_DQT_INFO		0x04
#define JPEG_PIC_SIZE		0x08
#define JPEG_QNR0		0x10
#define JPEG_DEC_CONF		0x20
#define JPEG_C0			0x24
#define JPEG_DHT_INFO		0x34
#define JPEG_IRQ_ENA		0x3c
#define JPEG_IRQ_ST		0x40
#define JPEG_IRQ_CLR		0x44
#define JPEG_DC0_COUNT		0x58
#define JPEG_AC0_COUNT		0x60
#define JPEG_DC0_MIN		0x78
#define JPEG_AC0_MIN		0x7c

#define JPEG_START		BIT(1)
#define JPEG_QNR_FIFO		BIT(8)
#define JPEG_HUFF_FIFO		BIT(15)
#define JPEG_SOFT_RST		BIT(24)
#define JPEG_DECODE		BIT(31)
#define JPEG_DONE		BIT(0)
#define JPEG_DECODE_ERRORS	(GENMASK(8, 2) | GENMASK(15, 13) | GENMASK(24, 18))
#define JPEG_IRQS		(JPEG_DONE | JPEG_DECODE_ERRORS)

#define JPEG_TIMEOUT_MS		100
#define JPEG_GRAY_MCU		8
#define JPEG_MAX_EDGE		1024
#define JPEG_MAX_INPUT		SZ_2M

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
	struct v4l2_pix_format source;
	struct v4l2_pix_format target;
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

static void jpeg_bound_format(struct v4l2_pix_format *pix, bool source)
{
	pix->width = round_up(clamp(pix->width, (u32)JPEG_GRAY_MCU,
				   (u32)JPEG_MAX_EDGE), JPEG_GRAY_MCU);
	pix->height = round_up(clamp(pix->height, (u32)JPEG_GRAY_MCU,
				    (u32)JPEG_MAX_EDGE), JPEG_GRAY_MCU);
	pix->pixelformat = source ? V4L2_PIX_FMT_JPEG : V4L2_PIX_FMT_GREY;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = source ? 0 : pix->width;
	pix->sizeimage = source ?
		clamp(pix->sizeimage ?: pix->width * pix->height + SZ_4K,
		      (u32)SZ_4K, (u32)JPEG_MAX_INPUT) : pix->width * pix->height;
	pix->colorspace = V4L2_COLORSPACE_JPEG;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static int jpeg_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	struct p4_jpeg *jpeg = video_drvdata(file);

	strscpy(cap->driver, "esp32p4-jpeg", sizeof(cap->driver));
	strscpy(cap->card, jpeg->video.name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(jpeg->dev));
	return 0;
}

static int jpeg_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *fmt)
{
	if (fmt->index)
		return -EINVAL;
	fmt->pixelformat = V4L2_TYPE_IS_OUTPUT(fmt->type) ?
			  V4L2_PIX_FMT_JPEG : V4L2_PIX_FMT_GREY;
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
	jpeg_try_fmt(file, fh, fmt);
	*jpeg_format(ctx, fmt->type) = fmt->fmt.pix;
	return 0;
}

static int jpeg_reqbufs(struct file *file, void *fh, struct v4l2_requestbuffers *req)
{
	/* Keep DMA images in PSRAM and the descriptor pool in coherent SRAM. */
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
	ctx->source.width = 320;
	ctx->source.height = 240;
	ctx->target = ctx->source;
	jpeg_bound_format(&ctx->source, true);
	jpeg_bound_format(&ctx->target, false);
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(jpeg->m2m, ctx, jpeg_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		v4l2_fh_exit(&ctx->fh);
		kfree(ctx);
		return ret;
	}
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh, file);
	return 0;
}

static int jpeg_release(struct file *file)
{
	struct jpeg_context *ctx = jpeg_context(file);

	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
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

static void jpeg_write_huffman(void __iomem *base, const u8 *table, bool ac)
{
	u32 count = ac ? JPEG_AC0_COUNT : JPEG_DC0_COUNT;
	u32 min = ac ? JPEG_AC0_MIN : JPEG_DC0_MIN;
	unsigned int i, total = 0, code = 0;

	for (i = 0; i < 16; i++) {
		/* FIFO uses cumulative counts, see jpeg_ll_dht_dc0_write_codeword(). */
		total += table[i];
		writel(total, base + count);
		writel(table[i] ? code << (15 - i) : 0xffff, base + min);
		code = (code + table[i]) << 1;
	}
	for (i = 0; i < total; i++)
		writel(table[16 + i], base + count + 4);
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
		       struct vb2_buffer *dst)
{
	struct p4_jpeg *jpeg = ctx->jpeg;
	void __iomem *base = jpeg->base;
	struct v4l2_jpeg_reference quant[4] = {};
	struct v4l2_jpeg_scan_header scan = {};
	struct v4l2_jpeg_header header = {
		.scan = &scan,
		.quantization_tables = quant,
	};
	const struct v4l2_jpeg_frame_component_spec *component = &header.frame.component[0];
	const u8 *tables[4] = {};
	struct esp_dma2d_image image = {
		.address = vb2_dma_contig_plane_dma_addr(dst, 0),
		.width = ctx->target.width,
		.height = ctx->target.height,
	};
	struct dma_async_tx_descriptor *desc;
	struct dma_chan *dma;
	dma_cookie_t cookie;
	u8 *input = vb2_plane_vaddr(src, 0);
	size_t bytes = vb2_get_plane_payload(src, 0), offset;
	unsigned int i, q, dc, ac;
	u32 config;
	int ret;

	if (!input)
		return -EFAULT;
	ret = v4l2_jpeg_parse_header(input, bytes, &header);
	if (ret)
		return ret;
	if (header.frame.precision != 8 || header.frame.num_components != 1 ||
	    scan.num_components != 1 || component->horizontal_sampling_factor != 1 ||
	    component->vertical_sampling_factor != 1 ||
	    scan.component[0].component_selector != component->component_identifier ||
	    header.frame.width != image.width || header.frame.height != image.height)
		return -EINVAL;

	q = component->quantization_table_selector;
	dc = scan.component[0].dc_entropy_coding_table_selector;
	ac = scan.component[0].ac_entropy_coding_table_selector;
	if (q >= ARRAY_SIZE(quant) || quant[q].length != 64 || dc > 1 || ac > 1)
		return -EINVAL;
	ret = jpeg_huffman_tables(&header, tables);
	if (ret || !tables[dc] || !tables[2 + ac])
		return ret ?: -EINVAL;
	if (!header.sos.start || header.sos.start < input + 2 || header.sos.length != 8)
		return -EINVAL;
	offset = header.sos.start - input - 2;
	if (offset >= bytes || header.ecs_offset >= bytes)
		return -EINVAL;

	dma = dma_request_chan(jpeg->dev, "codec");
	if (IS_ERR(dma))
		return PTR_ERR(dma);

	desc = esp_dma2d_prep_jpeg_gray(dma,
				       vb2_dma_contig_plane_dma_addr(src, 0) + offset,
				       bytes - offset, &image,
				       DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
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
	writel(q, base + JPEG_DQT_INFO);
	for (i = 0; i < quant[q].length; i++)
		writel(quant[q].start[i], base + JPEG_QNR0);
	writel(image.width << 16 | image.height, base + JPEG_PIC_SIZE);
	writel(BIT(16) | header.restart_interval, base + JPEG_DEC_CONF);
	writel(component->component_identifier << 16 | 0x1100 | q, base + JPEG_C0);
	writel(dc | (dc ^ 1) << 4 | ac << 8 | (ac ^ 1) << 12, base + JPEG_DHT_INFO);
	jpeg_write_huffman(base, tables[dc], false);
	jpeg_write_huffman(base, tables[2 + ac], true);
	reinit_completion(&jpeg->codec_done);
	reinit_completion(&jpeg->dma_done);
	jpeg->codec_status = 0;
	writel(JPEG_IRQS, base + JPEG_IRQ_CLR);
	writel(JPEG_IRQS, base + JPEG_IRQ_ENA);
	dma_async_issue_pending(dma);
	writel(config | JPEG_START, base + JPEG_CONFIG);

	if (!wait_for_completion_timeout(&jpeg->codec_done, msecs_to_jiffies(JPEG_TIMEOUT_MS)))
		ret = -ETIMEDOUT;
	else if (!(jpeg->codec_status & JPEG_DONE) || jpeg->codec_status & JPEG_DECODE_ERRORS)
		ret = -EIO;
	else if (!wait_for_completion_timeout(&jpeg->dma_done, msecs_to_jiffies(JPEG_TIMEOUT_MS)))
		ret = -ETIMEDOUT;
	else if (jpeg->dma_result != DMA_TRANS_NOERROR)
		ret = -EIO;

	writel(0, base + JPEG_IRQ_ENA);
	synchronize_irq(jpeg->irq);
	if (ret) {
		int stop;

		writel(config | JPEG_SOFT_RST, base + JPEG_CONFIG);
		writel(config, base + JPEG_CONFIG);
		stop = dmaengine_terminate_sync(dma);
		if (stop)
			dev_err(jpeg->dev, "failed to quiesce JPEG DMA: %d\n", stop);
		dev_dbg(jpeg->dev, "decode failed: %d, codec interrupt %#x\n",
			ret, jpeg->codec_status);
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
	int ret = -ENODEV;

	if (!READ_ONCE(jpeg->disconnected))
		ret = jpeg_decode(ctx, &src->vb2_buf, &dst->vb2_buf);
	vb2_set_plane_payload(&dst->vb2_buf, 0, ret ? 0 : ctx->target.sizeimage);
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

MODULE_DESCRIPTION("ESP32-P4 V4L2 JPEG decoder");
MODULE_LICENSE("GPL");
