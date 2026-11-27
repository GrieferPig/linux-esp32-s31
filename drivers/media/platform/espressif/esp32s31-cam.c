// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S31 LCD_CAM DVP Camera V4L2 Capture Driver.
 *
 * Hardware reference: S31 TRM ch.39 LCD_CAM + ESP-IDF
 * esp_hal_cam/esp32s31/include/hal/cam_ll.h and
 * esp_driver_cam/dvp/src/{esp_cam_ctlr_dvp_cam.c,esp_cam_ctlr_dvp_gdma.c}.
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/sizes.h>
#include <linux/spinlock.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

/* S31 TRM ch.39 LCD_CAM / IDF soc/esp32s31/register/soc/lcd_cam_reg.h */
#define LCDCAM_CAM_CTRL		0x04
#define CAM_STOP_EN		BIT(0)
#define CAM_UPDATE_REG		BIT(4)
#define CAM_BYTE_ORDER		BIT(5)
#define CAM_BIT_ORDER		BIT(6)
#define CAM_VS_EOF_EN		BIT(8)

#define LCDCAM_CAM_CTRL1	0x08
#define CAM_REC_DATA_BYTELEN	GENMASK(15, 0)
#define CAM_CLK_INV		BIT(22)
#define CAM_VSYNC_FILTER_EN	BIT(23)
#define CAM_2BYTE_EN		BIT(24)
#define CAM_DE_INV		BIT(25)
#define CAM_HSYNC_INV		BIT(26)
#define CAM_VSYNC_INV		BIT(27)
#define CAM_VH_DE_MODE_EN	BIT(28)
#define CAM_START		BIT(29)
#define CAM_RESET		BIT(30)
#define CAM_AFIFO_RESET		BIT(31)

#define LCDCAM_CAM_RGB_YUV	0x0c

#define LCDCAM_INT_ENA		0x64
#define LCDCAM_INT_RAW		0x68
#define LCDCAM_INT_ST		0x6c
#define LCDCAM_INT_CLR		0x70
#define CAM_VSYNC_INT		BIT(2)
#define CAM_HS_INT		BIT(3)
#define CAM_OVERFLOW_INT	BIT(5)
#define CAM_ALL_INTS		(CAM_VSYNC_INT | CAM_HS_INT | CAM_OVERFLOW_INT)

#define CAM_MIN_DIM		16
#define CAM_MAX_DIM		1280

struct esp32s31_cam_buf {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

struct esp32s31_cam {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct reset_control *reset;
	struct dma_chan *rx;
	int irq;

	struct v4l2_device v4l2;
	struct video_device video;
	struct vb2_queue queue;
	struct mutex lock;
	spinlock_t slock;

	struct v4l2_pix_format fmt;
	struct list_head buf_list;
	struct esp32s31_cam_buf *active;
	bool streaming;
	u32 sequence;
	u32 bus_width;
	u32 vsync_count;
	u32 overflow_count;
};

static const u32 esp32s31_cam_fmts[] = {
	V4L2_PIX_FMT_RGB565,
	V4L2_PIX_FMT_YUYV,
	V4L2_PIX_FMT_UYVY,
	V4L2_PIX_FMT_JPEG,
};

static void esp32s31_cam_fill_fmt(struct v4l2_pix_format *pix)
{
	bool found = false;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(esp32s31_cam_fmts); i++) {
		if (pix->pixelformat == esp32s31_cam_fmts[i]) {
			found = true;
			break;
		}
	}
	if (!found)
		pix->pixelformat = V4L2_PIX_FMT_RGB565;

	pix->width = ALIGN_DOWN(clamp_t(u32, pix->width ?: 64,
					CAM_MIN_DIM, CAM_MAX_DIM), 2);
	pix->height = ALIGN_DOWN(clamp_t(u32, pix->height ?: 64,
					 CAM_MIN_DIM, CAM_MAX_DIM), 2);
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_DEFAULT;
	pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	if (pix->pixelformat == V4L2_PIX_FMT_JPEG) {
		pix->bytesperline = 0;
		pix->sizeimage = max_t(u32, pix->width * pix->height, SZ_64K);
	} else {
		pix->bytesperline = pix->width * 2;
		pix->sizeimage = pix->bytesperline * pix->height;
	}
}

static void esp32s31_cam_hw_stop(struct esp32s31_cam *cam)
{
	u32 ctrl1 = readl(cam->base + LCDCAM_CAM_CTRL1);

	writel(ctrl1 & ~CAM_START, cam->base + LCDCAM_CAM_CTRL1);
	writel(CAM_VS_EOF_EN | CAM_UPDATE_REG, cam->base + LCDCAM_CAM_CTRL);
}

static int esp32s31_cam_arm_next_locked(struct esp32s31_cam *cam);

static void esp32s31_cam_dma_cb(void *data,
				const struct dmaengine_result *result)
{
	struct esp32s31_cam *cam = data;
	struct esp32s31_cam_buf *buf;
	unsigned long flags;
	enum vb2_buffer_state state;
	u32 bytesused;

	spin_lock_irqsave(&cam->slock, flags);
	buf = cam->active;
	cam->active = NULL;
	if (!buf) {
		spin_unlock_irqrestore(&cam->slock, flags);
		return;
	}

	if (result && result->result != DMA_TRANS_NOERROR) {
		state = VB2_BUF_STATE_ERROR;
		bytesused = 0;
	} else {
		state = VB2_BUF_STATE_DONE;
		bytesused = cam->fmt.sizeimage;
		if (result && result->residue && result->residue < bytesused)
			bytesused -= result->residue;
	}

	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	buf->vb.sequence = cam->sequence++;
	buf->vb.field = V4L2_FIELD_NONE;
	vb2_set_plane_payload(&buf->vb.vb2_buf, 0, bytesused);
	vb2_buffer_done(&buf->vb.vb2_buf, state);

	if (cam->streaming && !list_empty(&cam->buf_list))
		esp32s31_cam_arm_next_locked(cam);
	else
		esp32s31_cam_hw_stop(cam);

	spin_unlock_irqrestore(&cam->slock, flags);
}

static int esp32s31_cam_arm_next_locked(struct esp32s31_cam *cam)
{
	struct esp32s31_cam_buf *buf;
	struct dma_async_tx_descriptor *desc;
	dma_addr_t dma_addr;
	dma_cookie_t cookie;
	u32 ctrl1 = (cam->bus_width == 16) ? CAM_2BYTE_EN : 0;

	lockdep_assert_held(&cam->slock);
	if (cam->active || list_empty(&cam->buf_list))
		return 0;

	buf = list_first_entry(&cam->buf_list, struct esp32s31_cam_buf, list);
	list_del_init(&buf->list);

	dma_addr = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);
	desc = dmaengine_prep_slave_single(cam->rx, dma_addr,
					   cam->fmt.sizeimage, DMA_DEV_TO_MEM,
					   DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc) {
		list_add(&buf->list, &cam->buf_list);
		return -EIO;
	}

	desc->callback_result = esp32s31_cam_dma_cb;
	desc->callback_param = cam;
	cookie = dmaengine_submit(desc);
	if (dma_submit_error(cookie)) {
		list_add(&buf->list, &cam->buf_list);
		return -EIO;
	}

	cam->active = buf;
	/* S31 TRM ch.39 / IDF cam_ll_reset(), cam_ll_fifo_reset(), cam_ll_start(). */
	writel(ctrl1 | CAM_RESET | CAM_AFIFO_RESET, cam->base + LCDCAM_CAM_CTRL1);
	writel(0, cam->base + LCDCAM_CAM_RGB_YUV);
	writel(CAM_VS_EOF_EN | CAM_UPDATE_REG, cam->base + LCDCAM_CAM_CTRL);
	dma_async_issue_pending(cam->rx);
	writel(ctrl1 | CAM_START, cam->base + LCDCAM_CAM_CTRL1);
	return 0;
}

static irqreturn_t esp32s31_cam_irq(int irq, void *data)
{
	struct esp32s31_cam *cam = data;
	u32 st = readl(cam->base + LCDCAM_INT_ST) & CAM_ALL_INTS;

	if (!st)
		return IRQ_NONE;

	writel(st, cam->base + LCDCAM_INT_CLR);
	if (st & CAM_VSYNC_INT)
		cam->vsync_count++;
	if (st & CAM_OVERFLOW_INT) {
		cam->overflow_count++;
		dev_warn_ratelimited(cam->dev, "CAM RX FIFO overflow\n");
	}
	return IRQ_HANDLED;
}

/* ---------- VB2 Ops ---------- */

static int cam_queue_setup(struct vb2_queue *q, unsigned int *nbuffers,
			   unsigned int *nplanes, unsigned int sizes[],
			   struct device *alloc_devs[])
{
	struct esp32s31_cam *cam = vb2_get_drv_priv(q);

	if (*nplanes)
		return *nplanes == 1 && sizes[0] >= cam->fmt.sizeimage ? 0 : -EINVAL;
	*nplanes = 1;
	sizes[0] = cam->fmt.sizeimage;
	return 0;
}

static int cam_buf_prepare(struct vb2_buffer *vb)
{
	struct esp32s31_cam *cam = vb2_get_drv_priv(vb->vb2_queue);

	if (vb2_plane_size(vb, 0) < cam->fmt.sizeimage)
		return -EINVAL;
	vb2_set_plane_payload(vb, 0, cam->fmt.sizeimage);
	return 0;
}

static void cam_buf_queue(struct vb2_buffer *vb)
{
	struct esp32s31_cam *cam = vb2_get_drv_priv(vb->vb2_queue);
	struct esp32s31_cam_buf *buf =
		container_of(to_vb2_v4l2_buffer(vb), struct esp32s31_cam_buf, vb);
	unsigned long flags;

	spin_lock_irqsave(&cam->slock, flags);
	list_add_tail(&buf->list, &cam->buf_list);
	if (cam->streaming && !cam->active)
		esp32s31_cam_arm_next_locked(cam);
	spin_unlock_irqrestore(&cam->slock, flags);
}

static void cam_return_all_buffers(struct esp32s31_cam *cam,
				   enum vb2_buffer_state state)
{
	struct esp32s31_cam_buf *buf, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&cam->slock, flags);
	if (cam->active) {
		vb2_buffer_done(&cam->active->vb.vb2_buf, state);
		cam->active = NULL;
	}
	list_for_each_entry_safe(buf, tmp, &cam->buf_list, list) {
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	spin_unlock_irqrestore(&cam->slock, flags);
}

static int cam_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct esp32s31_cam *cam = vb2_get_drv_priv(q);
	struct dma_slave_config cfg = {
		.direction = DMA_DEV_TO_MEM,
		.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
	};
	unsigned long flags;
	int ret;

	ret = clk_prepare_enable(cam->clk);
	if (ret)
		goto err_return;

	ret = reset_control_reset(cam->reset);
	if (ret)
		goto err_clk;

	ret = dmaengine_slave_config(cam->rx, &cfg);
	if (ret)
		goto err_clk;

	cam->sequence = 0;
	writel(CAM_ALL_INTS, cam->base + LCDCAM_INT_CLR);
	writel(CAM_VSYNC_INT | CAM_OVERFLOW_INT, cam->base + LCDCAM_INT_ENA);

	spin_lock_irqsave(&cam->slock, flags);
	cam->streaming = true;
	ret = esp32s31_cam_arm_next_locked(cam);
	if (ret)
		cam->streaming = false;
	spin_unlock_irqrestore(&cam->slock, flags);

	if (ret)
		goto err_irq;
	return 0;

err_irq:
	writel(0, cam->base + LCDCAM_INT_ENA);
err_clk:
	clk_disable_unprepare(cam->clk);
err_return:
	cam_return_all_buffers(cam, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void cam_stop_streaming(struct vb2_queue *q)
{
	struct esp32s31_cam *cam = vb2_get_drv_priv(q);
	unsigned long flags;

	spin_lock_irqsave(&cam->slock, flags);
	cam->streaming = false;
	esp32s31_cam_hw_stop(cam);
	writel(0, cam->base + LCDCAM_INT_ENA);
	spin_unlock_irqrestore(&cam->slock, flags);

	dmaengine_terminate_sync(cam->rx);
	cam_return_all_buffers(cam, VB2_BUF_STATE_ERROR);
	clk_disable_unprepare(cam->clk);
}

static const struct vb2_ops esp32s31_cam_vb2_ops = {
	.queue_setup = cam_queue_setup,
	.buf_prepare = cam_buf_prepare,
	.buf_queue = cam_buf_queue,
	.start_streaming = cam_start_streaming,
	.stop_streaming = cam_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

/* ---------- V4L2 IOCTLs ---------- */

static int cam_querycap(struct file *file, void *fh,
			struct v4l2_capability *cap)
{
	struct esp32s31_cam *cam = video_drvdata(file);

	strscpy(cap->driver, "esp32s31-cam", sizeof(cap->driver));
	strscpy(cap->card, "ESP32-S31 LCD_CAM DVP", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(cam->dev));
	return 0;
}

static int cam_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(esp32s31_cam_fmts))
		return -EINVAL;
	f->pixelformat = esp32s31_cam_fmts[f->index];
	return 0;
}

static int cam_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct esp32s31_cam *cam = video_drvdata(file);

	f->fmt.pix = cam->fmt;
	return 0;
}

static int cam_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	esp32s31_cam_fill_fmt(&f->fmt.pix);
	return 0;
}

static int cam_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct esp32s31_cam *cam = video_drvdata(file);

	if (vb2_is_busy(&cam->queue))
		return -EBUSY;
	esp32s31_cam_fill_fmt(&f->fmt.pix);
	cam->fmt = f->fmt.pix;
	return 0;
}

static int cam_reqbufs(struct file *file, void *fh,
		       struct v4l2_requestbuffers *req)
{
	if (req->count && req->memory == V4L2_MEMORY_MMAP)
		req->flags |= V4L2_MEMORY_FLAG_NON_COHERENT;
	return vb2_ioctl_reqbufs(file, fh, req);
}

static const struct v4l2_ioctl_ops esp32s31_cam_ioctl_ops = {
	.vidioc_querycap = cam_querycap,
	.vidioc_enum_fmt_vid_cap = cam_enum_fmt,
	.vidioc_g_fmt_vid_cap = cam_g_fmt,
	.vidioc_try_fmt_vid_cap = cam_try_fmt,
	.vidioc_s_fmt_vid_cap = cam_s_fmt,
	.vidioc_reqbufs = cam_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations esp32s31_cam_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.read = vb2_fop_read,
	.poll = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
#ifndef CONFIG_MMU
	.get_unmapped_area = vb2_fop_get_unmapped_area,
#endif
};

static int esp32s31_cam_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp32s31_cam *cam;
	int ret;

	cam = devm_kzalloc(dev, sizeof(*cam), GFP_KERNEL);
	if (!cam)
		return -ENOMEM;

	cam->dev = dev;
	mutex_init(&cam->lock);
	spin_lock_init(&cam->slock);
	INIT_LIST_HEAD(&cam->buf_list);
	platform_set_drvdata(pdev, cam);

	cam->bus_width = 8;
	device_property_read_u32(dev, "bus-width", &cam->bus_width);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	cam->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cam->base))
		return PTR_ERR(cam->base);

	cam->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(cam->clk))
		return dev_err_probe(dev, PTR_ERR(cam->clk), "cannot get clock\n");

	cam->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(cam->reset))
		return dev_err_probe(dev, PTR_ERR(cam->reset), "cannot get reset\n");

	cam->rx = dma_request_chan(dev, "rx");
	if (IS_ERR(cam->rx))
		return dev_err_probe(dev, PTR_ERR(cam->rx), "cannot get RX DMA\n");

	cam->irq = platform_get_irq(pdev, 0);
	if (cam->irq < 0) {
		ret = cam->irq;
		goto release_dma;
	}
	ret = devm_request_irq(dev, cam->irq, esp32s31_cam_irq, IRQF_SHARED,
			       dev_name(dev), cam);
	if (ret)
		goto release_dma;

	cam->fmt.width = 64;
	cam->fmt.height = 64;
	cam->fmt.pixelformat = V4L2_PIX_FMT_RGB565;
	esp32s31_cam_fill_fmt(&cam->fmt);

	ret = v4l2_device_register(dev, &cam->v4l2);
	if (ret)
		goto release_dma;

	cam->queue = (struct vb2_queue) {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.io_modes = VB2_MMAP | VB2_READ,
		.drv_priv = cam,
		.buf_struct_size = sizeof(struct esp32s31_cam_buf),
		.ops = &esp32s31_cam_vb2_ops,
		.mem_ops = &vb2_dma_contig_memops,
		.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC,
		.lock = &cam->lock,
		.dev = dev,
		.allow_cache_hints = 1,
	};
	ret = vb2_queue_init(&cam->queue);
	if (ret)
		goto unreg_v4l2;

	cam->video = (struct video_device) {
		.name = "esp32s31-cam",
		.vfl_dir = VFL_DIR_RX,
		.fops = &esp32s31_cam_fops,
		.ioctl_ops = &esp32s31_cam_ioctl_ops,
		.release = video_device_release_empty,
		.v4l2_dev = &cam->v4l2,
		.queue = &cam->queue,
		.lock = &cam->lock,
		.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			       V4L2_CAP_READWRITE,
	};
	video_set_drvdata(&cam->video, cam);

	ret = video_register_device(&cam->video, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto unreg_v4l2;

	dev_info(dev, "registered /dev/%s (%u-bit DVP)\n",
		 video_device_node_name(&cam->video), cam->bus_width);
	return 0;

unreg_v4l2:
	v4l2_device_unregister(&cam->v4l2);
release_dma:
	dma_release_channel(cam->rx);
	return ret;
}

static void esp32s31_cam_remove(struct platform_device *pdev)
{
	struct esp32s31_cam *cam = platform_get_drvdata(pdev);

	video_unregister_device(&cam->video);
	v4l2_device_unregister(&cam->v4l2);
	dma_release_channel(cam->rx);
}

static const struct of_device_id esp32s31_cam_of_match[] = {
	{ .compatible = "espressif,esp32s31-cam" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s31_cam_of_match);

static struct platform_driver esp32s31_cam_driver = {
	.probe = esp32s31_cam_probe,
	.remove = esp32s31_cam_remove,
	.driver = {
		.name = "esp32s31-cam",
		.of_match_table = esp32s31_cam_of_match,
	},
};
module_platform_driver(esp32s31_cam_driver);

MODULE_DESCRIPTION("ESP32-S31 LCD_CAM DVP V4L2 capture driver");
MODULE_LICENSE("GPL");
