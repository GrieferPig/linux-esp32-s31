// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-P4 rev1.3 LCD_CAM driver: Parallel RGB LCD (fbdev) and DVP camera
 * capture (V4L2) on 0x500dc000 (IRQ 36, AXI-GDMA LCD0/CAM0 trigger 0).
 *
 * Hardware reference: ESP32-P4 rev1.3 TRM ch.37 LCD_CAM + ESP-IDF
 * esp_hal_lcd/esp32p4/include/hal/lcd_ll.h and
 * esp_hal_cam/esp32p4/include/hal/cam_ll.h.
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/sizes.h>
#include <linux/spinlock.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

/* ESP32-P4 rev1.3 TRM ch.37 LCD_CAM registers. */
#define LCD_CLOCK		0x00
#define LCD_PCLK_DIV		GENMASK(5, 0)
#define LCD_PCLK_NEGATIVE	BIT(8)

#define CAM_CTRL		0x04
#define CAM_UPDATE_REG		BIT(4)
#define CAM_VS_EOF_EN		BIT(8)

#define CAM_CTRL1		0x08
#define CAM_2BYTE_EN		BIT(24)
#define CAM_START		BIT(29)
#define CAM_RESET		BIT(30)
#define CAM_AFIFO_RESET		BIT(31)

#define CAM_RGB_YUV		0x0c

#define LCD_USER		0x14
#define LCD_ALWAYS_OUT		BIT(13)
#define LCD_BYTE_MODE		GENMASK(20, 19)
#define LCD_UPDATE		BIT(21)
#define LCD_DOUT		BIT(24)
#define LCD_START		BIT(27)
#define LCD_RESET		BIT(28)

#define LCD_MISC		0x18
#define LCD_RGB_MODE		BIT(3)
#define LCD_WIRE_MODE		GENMASK(5, 4)
#define LCD_NEXT_FRAME		BIT(25)
#define LCD_BLANK_ENABLE	BIT(26)
#define LCD_FIFO_RESET		BIT(27)

#define LCD_RGB_VERTICAL	0x1c
#define LCD_RGB_HORIZONTAL	0x20
#define LCD_RGB_BLANK		0x24
#define LCD_ACTIVE		GENMASK(15, 0)
#define LCD_TOTAL		GENMASK(31, 16)
#define LCD_HBLANK		GENMASK(15, 0)
#define LCD_VBLANK		GENMASK(31, 16)

#define LCD_RGB_CTRL		0x28
#define LCD_VSYNC_WIDTH		GENMASK(9, 0)
#define LCD_VSYNC_IDLE_HIGH	BIT(10)
#define LCD_DE_IDLE_HIGH	BIT(11)
#define LCD_HSYNC_IN_PORCH	BIT(12)
#define LCD_HSYNC_WIDTH		GENMASK(22, 16)
#define LCD_HSYNC_IDLE_HIGH	BIT(23)

#define LCDCAM_INT_ENA		0x64
#define LCDCAM_INT_ST		0x6c
#define LCDCAM_INT_CLR		0x70
#define LCD_VSYNC_INT		BIT(0)
#define CAM_VSYNC_INT		BIT(2)
#define CAM_OVERFLOW_INT	BIT(5)

#define CAM_MIN_DIM		16
#define CAM_MAX_DIM		1280

struct p4_cam_buf {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

struct p4_lcdcam {
	struct device *dev;
	void __iomem *base;
	struct clk *lcd_clk;
	struct clk *cam_clk;
	struct reset_control *reset;
	int irq;
	spinlock_t slock;

	/* Parallel RGB LCD (fbdev) */
	bool has_lcd;
	struct fb_info *fb;
	struct dma_chan *tx;
	void *fb_virt;
	dma_addr_t fb_dma;
	size_t fb_size;
	u32 palette[16];
	struct completion lcd_vsync;

	/* DVP Camera Capture (V4L2) */
	bool has_cam;
	struct dma_chan *rx;
	struct v4l2_device v4l2;
	struct video_device video;
	struct vb2_queue queue;
	struct mutex lock;
	struct v4l2_pix_format fmt;
	struct list_head buf_list;
	struct p4_cam_buf *active;
	bool streaming;
	u32 sequence;
	u32 bus_width;
};

static const u32 p4_cam_fmts[] = {
	V4L2_PIX_FMT_RGB565,
	V4L2_PIX_FMT_YUYV,
	V4L2_PIX_FMT_UYVY,
	V4L2_PIX_FMT_JPEG,
};

static void p4_cam_fill_fmt(struct v4l2_pix_format *pix)
{
	unsigned int i;
	bool ok = false;

	for (i = 0; i < ARRAY_SIZE(p4_cam_fmts); i++) {
		if (pix->pixelformat == p4_cam_fmts[i]) {
			ok = true;
			break;
		}
	}
	if (!ok)
		pix->pixelformat = V4L2_PIX_FMT_RGB565;

	pix->width = ALIGN_DOWN(clamp_t(u32, pix->width ?: 320,
					CAM_MIN_DIM, CAM_MAX_DIM), 2);
	pix->height = ALIGN_DOWN(clamp_t(u32, pix->height ?: 240,
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

static void p4_cam_hw_stop(struct p4_lcdcam *lc)
{
	u32 c1 = readl(lc->base + CAM_CTRL1);

	writel(c1 & ~CAM_START, lc->base + CAM_CTRL1);
	writel(CAM_VS_EOF_EN | CAM_UPDATE_REG, lc->base + CAM_CTRL);
}

static int p4_cam_arm_next_locked(struct p4_lcdcam *lc);

static void p4_cam_dma_cb(void *data, const struct dmaengine_result *result)
{
	struct p4_lcdcam *lc = data;
	struct p4_cam_buf *buf;
	unsigned long flags;
	enum vb2_buffer_state state;
	u32 bytesused;

	spin_lock_irqsave(&lc->slock, flags);
	buf = lc->active;
	lc->active = NULL;
	if (!buf) {
		spin_unlock_irqrestore(&lc->slock, flags);
		return;
	}

	if (result && result->result != DMA_TRANS_NOERROR) {
		state = VB2_BUF_STATE_ERROR;
		bytesused = 0;
	} else {
		state = VB2_BUF_STATE_DONE;
		bytesused = lc->fmt.sizeimage;
		if (result && result->residue && result->residue < bytesused)
			bytesused -= result->residue;
	}

	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	buf->vb.sequence = lc->sequence++;
	buf->vb.field = V4L2_FIELD_NONE;
	vb2_set_plane_payload(&buf->vb.vb2_buf, 0, bytesused);
	vb2_buffer_done(&buf->vb.vb2_buf, state);

	if (lc->streaming && !list_empty(&lc->buf_list))
		p4_cam_arm_next_locked(lc);
	else
		p4_cam_hw_stop(lc);
	spin_unlock_irqrestore(&lc->slock, flags);
}

static int p4_cam_arm_next_locked(struct p4_lcdcam *lc)
{
	struct p4_cam_buf *buf;
	struct dma_async_tx_descriptor *desc;
	dma_addr_t addr;
	dma_cookie_t cookie;
	u32 ctrl1 = (lc->bus_width == 16) ? CAM_2BYTE_EN : 0;

	if (lc->active || list_empty(&lc->buf_list))
		return 0;

	buf = list_first_entry(&lc->buf_list, struct p4_cam_buf, list);
	list_del_init(&buf->list);

	addr = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);
	desc = dmaengine_prep_slave_single(lc->rx, addr, lc->fmt.sizeimage,
					   DMA_DEV_TO_MEM,
					   DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc) {
		list_add(&buf->list, &lc->buf_list);
		return -EIO;
	}
	desc->callback_result = p4_cam_dma_cb;
	desc->callback_param = lc;
	cookie = dmaengine_submit(desc);
	if (dma_submit_error(cookie)) {
		list_add(&buf->list, &lc->buf_list);
		return -EIO;
	}

	lc->active = buf;
	writel(ctrl1 | CAM_RESET | CAM_AFIFO_RESET, lc->base + CAM_CTRL1);
	writel(0, lc->base + CAM_RGB_YUV);
	writel(CAM_VS_EOF_EN | CAM_UPDATE_REG, lc->base + CAM_CTRL);
	dma_async_issue_pending(lc->rx);
	writel(ctrl1 | CAM_START, lc->base + CAM_CTRL1);
	return 0;
}

static irqreturn_t p4_lcdcam_irq(int irq, void *data)
{
	struct p4_lcdcam *lc = data;
	u32 st = readl(lc->base + LCDCAM_INT_ST);

	if (!st)
		return IRQ_NONE;
	writel(st, lc->base + LCDCAM_INT_CLR);
	if (st & LCD_VSYNC_INT)
		complete(&lc->lcd_vsync);
	if (st & CAM_OVERFLOW_INT)
		dev_warn_ratelimited(lc->dev, "CAM RX FIFO overflow\n");
	return IRQ_HANDLED;
}

/* ---------- VB2 / V4L2 DVP Capture ---------- */

static int cam_queue_setup(struct vb2_queue *q, unsigned int *nbuffers,
			   unsigned int *nplanes, unsigned int sizes[],
			   struct device *alloc_devs[])
{
	struct p4_lcdcam *lc = vb2_get_drv_priv(q);

	if (*nplanes)
		return *nplanes == 1 && sizes[0] >= lc->fmt.sizeimage ? 0 : -EINVAL;
	*nplanes = 1;
	sizes[0] = lc->fmt.sizeimage;
	return 0;
}

static int cam_buf_prepare(struct vb2_buffer *vb)
{
	struct p4_lcdcam *lc = vb2_get_drv_priv(vb->vb2_queue);

	if (vb2_plane_size(vb, 0) < lc->fmt.sizeimage)
		return -EINVAL;
	vb2_set_plane_payload(vb, 0, lc->fmt.sizeimage);
	return 0;
}

static void cam_buf_queue(struct vb2_buffer *vb)
{
	struct p4_lcdcam *lc = vb2_get_drv_priv(vb->vb2_queue);
	struct p4_cam_buf *buf =
		container_of(to_vb2_v4l2_buffer(vb), struct p4_cam_buf, vb);
	unsigned long flags;

	spin_lock_irqsave(&lc->slock, flags);
	list_add_tail(&buf->list, &lc->buf_list);
	if (lc->streaming && !lc->active)
		p4_cam_arm_next_locked(lc);
	spin_unlock_irqrestore(&lc->slock, flags);
}

static void cam_return_all(struct p4_lcdcam *lc, enum vb2_buffer_state state)
{
	struct p4_cam_buf *buf, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&lc->slock, flags);
	if (lc->active) {
		vb2_buffer_done(&lc->active->vb.vb2_buf, state);
		lc->active = NULL;
	}
	list_for_each_entry_safe(buf, tmp, &lc->buf_list, list) {
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	spin_unlock_irqrestore(&lc->slock, flags);
}

static int cam_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct p4_lcdcam *lc = vb2_get_drv_priv(q);
	struct dma_slave_config cfg = {
		.direction = DMA_DEV_TO_MEM,
		.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
	};
	unsigned long flags;
	u32 ena;
	int ret;

	ret = clk_prepare_enable(lc->cam_clk);
	if (ret)
		goto err_ret;
	ret = dmaengine_slave_config(lc->rx, &cfg);
	if (ret)
		goto err_clk;

	lc->sequence = 0;
	writel(CAM_VSYNC_INT | CAM_OVERFLOW_INT, lc->base + LCDCAM_INT_CLR);
	ena = readl(lc->base + LCDCAM_INT_ENA) | CAM_VSYNC_INT | CAM_OVERFLOW_INT;
	writel(ena, lc->base + LCDCAM_INT_ENA);

	spin_lock_irqsave(&lc->slock, flags);
	lc->streaming = true;
	ret = p4_cam_arm_next_locked(lc);
	if (ret)
		lc->streaming = false;
	spin_unlock_irqrestore(&lc->slock, flags);
	if (ret)
		goto err_clk;
	return 0;

err_clk:
	clk_disable_unprepare(lc->cam_clk);
err_ret:
	cam_return_all(lc, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void cam_stop_streaming(struct vb2_queue *q)
{
	struct p4_lcdcam *lc = vb2_get_drv_priv(q);
	unsigned long flags;
	u32 ena;

	spin_lock_irqsave(&lc->slock, flags);
	lc->streaming = false;
	p4_cam_hw_stop(lc);
	ena = readl(lc->base + LCDCAM_INT_ENA) & ~(CAM_VSYNC_INT | CAM_OVERFLOW_INT);
	writel(ena, lc->base + LCDCAM_INT_ENA);
	spin_unlock_irqrestore(&lc->slock, flags);

	dmaengine_terminate_sync(lc->rx);
	cam_return_all(lc, VB2_BUF_STATE_ERROR);
	clk_disable_unprepare(lc->cam_clk);
}

static const struct vb2_ops p4_cam_vb2_ops = {
	.queue_setup = cam_queue_setup,
	.buf_prepare = cam_buf_prepare,
	.buf_queue = cam_buf_queue,
	.start_streaming = cam_start_streaming,
	.stop_streaming = cam_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int cam_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	struct p4_lcdcam *lc = video_drvdata(file);

	strscpy(cap->driver, "esp32p4-lcdcam", sizeof(cap->driver));
	strscpy(cap->card, "ESP32-P4 LCD_CAM DVP", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s", dev_name(lc->dev));
	return 0;
}

static int cam_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(p4_cam_fmts))
		return -EINVAL;
	f->pixelformat = p4_cam_fmts[f->index];
	return 0;
}

static int cam_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct p4_lcdcam *lc = video_drvdata(file);

	f->fmt.pix = lc->fmt;
	return 0;
}

static int cam_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	p4_cam_fill_fmt(&f->fmt.pix);
	return 0;
}

static int cam_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct p4_lcdcam *lc = video_drvdata(file);

	if (vb2_is_busy(&lc->queue))
		return -EBUSY;
	p4_cam_fill_fmt(&f->fmt.pix);
	lc->fmt = f->fmt.pix;
	return 0;
}

static int cam_reqbufs(struct file *file, void *fh, struct v4l2_requestbuffers *req)
{
	if (req->count && req->memory == V4L2_MEMORY_MMAP)
		req->flags |= V4L2_MEMORY_FLAG_NON_COHERENT;
	return vb2_ioctl_reqbufs(file, fh, req);
}

static const struct v4l2_ioctl_ops p4_cam_ioctl_ops = {
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

static const struct v4l2_file_operations p4_cam_fops = {
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

/* ---------- Parallel RGB LCD (fbdev) ---------- */

static int p4_lcd_setcolreg(unsigned int regno, unsigned int red,
			    unsigned int green, unsigned int blue,
			    unsigned int transp, struct fb_info *info)
{
	struct p4_lcdcam *lc = info->par;

	if (regno >= ARRAY_SIZE(lc->palette))
		return -EINVAL;
	lc->palette[regno] = ((red >> 11) << 11) | ((green >> 10) << 5) | (blue >> 11);
	return 0;
}

static const struct fb_ops p4_lcd_fbops = {
	.owner = THIS_MODULE,
	FB_DEFAULT_IOMEM_OPS,
	.fb_setcolreg = p4_lcd_setcolreg,
};

static int p4_lcd_init(struct p4_lcdcam *lc)
{
	struct display_timing timing;
	struct videomode mode;
	struct dma_slave_config cfg = {
		.direction = DMA_MEM_TO_DEV,
		.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES,
	};
	struct dma_async_tx_descriptor *desc;
	struct fb_info *info;
	u64 hblank, vblank, htotal, vtotal;
	u32 sync = LCD_HSYNC_IN_PORCH;
	int ret;

	if (of_get_display_timing(lc->dev->of_node, "panel-timing", &timing))
		return 0;
	videomode_from_timing(&timing, &mode);
	if (!mode.hactive || !mode.vactive)
		return -EINVAL;

	lc->tx = dma_request_chan(lc->dev, "tx");
	if (IS_ERR(lc->tx))
		return PTR_ERR(lc->tx);
	ret = dmaengine_slave_config(lc->tx, &cfg);
	if (ret)
		goto err_tx;
	ret = clk_prepare_enable(lc->lcd_clk);
	if (ret)
		goto err_tx;

	lc->fb_size = (size_t)mode.hactive * mode.vactive * 2;
	lc->fb_virt = dma_alloc_coherent(lc->dev, lc->fb_size, &lc->fb_dma, GFP_KERNEL);
	if (!lc->fb_virt) {
		ret = -ENOMEM;
		goto err_clk;
	}

	info = framebuffer_alloc(0, lc->dev);
	if (!info) {
		ret = -ENOMEM;
		goto err_fb_mem;
	}
	info->par = lc;
	info->fbops = &p4_lcd_fbops;
	info->screen_base = lc->fb_virt;
	info->screen_size = lc->fb_size;
	info->pseudo_palette = lc->palette;
	strscpy(info->fix.id, "esp32p4-lcd", sizeof(info->fix.id));
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.smem_start = lc->fb_dma;
	info->fix.smem_len = lc->fb_size;
	info->fix.line_length = mode.hactive * 2;
	info->var.xres = info->var.xres_virtual = mode.hactive;
	info->var.yres = info->var.yres_virtual = mode.vactive;
	info->var.bits_per_pixel = 16;
	info->var.red = (struct fb_bitfield) { 11, 5, 0 };
	info->var.green = (struct fb_bitfield) { 5, 6, 0 };
	info->var.blue = (struct fb_bitfield) { 0, 5, 0 };

	hblank = (u64)mode.hsync_len + mode.hback_porch;
	vblank = (u64)mode.vsync_len + mode.vback_porch;
	htotal = hblank + mode.hactive + mode.hfront_porch;
	vtotal = vblank + mode.vactive + mode.vfront_porch;
	if (!(mode.flags & DISPLAY_FLAGS_HSYNC_HIGH))
		sync |= LCD_HSYNC_IDLE_HIGH;
	if (!(mode.flags & DISPLAY_FLAGS_VSYNC_HIGH))
		sync |= LCD_VSYNC_IDLE_HIGH;
	if (mode.flags & DISPLAY_FLAGS_DE_LOW)
		sync |= LCD_DE_IDLE_HIGH;

	writel(LCD_RESET, lc->base + LCD_USER);
	writel(LCD_FIFO_RESET, lc->base + LCD_MISC);
	writel(FIELD_PREP(LCD_ACTIVE, mode.hactive - 1) | FIELD_PREP(LCD_TOTAL, htotal - 1),
	       lc->base + LCD_RGB_HORIZONTAL);
	writel(FIELD_PREP(LCD_ACTIVE, mode.vactive - 1) | FIELD_PREP(LCD_TOTAL, vtotal - 1),
	       lc->base + LCD_RGB_VERTICAL);
	writel(FIELD_PREP(LCD_HBLANK, hblank - 1) | FIELD_PREP(LCD_VBLANK, vblank - 1),
	       lc->base + LCD_RGB_BLANK);
	writel(sync | FIELD_PREP(LCD_HSYNC_WIDTH, mode.hsync_len - 1) |
	       FIELD_PREP(LCD_VSYNC_WIDTH, mode.vsync_len - 1), lc->base + LCD_RGB_CTRL);
	writel(LCD_RGB_MODE | FIELD_PREP(LCD_WIRE_MODE, 1) | LCD_NEXT_FRAME | LCD_BLANK_ENABLE,
	       lc->base + LCD_MISC);
	writel(LCD_ALWAYS_OUT | FIELD_PREP(LCD_BYTE_MODE, 1) | LCD_DOUT | LCD_UPDATE,
	       lc->base + LCD_USER);

	desc = dmaengine_prep_dma_cyclic(lc->tx, lc->fb_dma, lc->fb_size, lc->fb_size,
					 DMA_MEM_TO_DEV, DMA_CTRL_ACK);
	if (!desc || dma_submit_error(dmaengine_submit(desc))) {
		ret = -EIO;
		goto err_fb;
	}
	writel(LCD_VSYNC_INT, lc->base + LCDCAM_INT_CLR);
	writel(readl(lc->base + LCDCAM_INT_ENA) | LCD_VSYNC_INT, lc->base + LCDCAM_INT_ENA);
	dma_async_issue_pending(lc->tx);
	writel(LCD_ALWAYS_OUT | FIELD_PREP(LCD_BYTE_MODE, 1) | LCD_DOUT | LCD_UPDATE | LCD_START,
	       lc->base + LCD_USER);

	ret = register_framebuffer(info);
	if (ret)
		goto err_stop;

	lc->fb = info;
	lc->has_lcd = true;
	dev_info(lc->dev, "registered parallel RGB LCD %ux%u as fb%d\n",
		 mode.hactive, mode.vactive, info->node);
	return 0;

err_stop:
	dmaengine_terminate_sync(lc->tx);
err_fb:
	framebuffer_release(info);
err_fb_mem:
	dma_free_coherent(lc->dev, lc->fb_size, lc->fb_virt, lc->fb_dma);
err_clk:
	clk_disable_unprepare(lc->lcd_clk);
err_tx:
	dma_release_channel(lc->tx);
	return ret;
}

static int p4_cam_init(struct p4_lcdcam *lc)
{
	int ret;

	lc->rx = dma_request_chan(lc->dev, "rx");
	if (IS_ERR(lc->rx))
		return PTR_ERR(lc->rx);

	lc->fmt.width = 320;
	lc->fmt.height = 240;
	lc->fmt.pixelformat = V4L2_PIX_FMT_RGB565;
	p4_cam_fill_fmt(&lc->fmt);

	ret = v4l2_device_register(lc->dev, &lc->v4l2);
	if (ret)
		goto err_rx;

	lc->queue = (struct vb2_queue) {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.io_modes = VB2_MMAP | VB2_READ,
		.drv_priv = lc,
		.buf_struct_size = sizeof(struct p4_cam_buf),
		.ops = &p4_cam_vb2_ops,
		.mem_ops = &vb2_dma_contig_memops,
		.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC,
		.lock = &lc->lock,
		.dev = lc->dev,
		.allow_cache_hints = 1,
	};
	ret = vb2_queue_init(&lc->queue);
	if (ret)
		goto err_v4l2;

	lc->video = (struct video_device) {
		.name = "esp32p4-cam",
		.vfl_dir = VFL_DIR_RX,
		.fops = &p4_cam_fops,
		.ioctl_ops = &p4_cam_ioctl_ops,
		.release = video_device_release_empty,
		.v4l2_dev = &lc->v4l2,
		.queue = &lc->queue,
		.lock = &lc->lock,
		.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			       V4L2_CAP_READWRITE,
	};
	video_set_drvdata(&lc->video, lc);
	ret = video_register_device(&lc->video, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_v4l2;

	lc->has_cam = true;
	dev_info(lc->dev, "registered DVP camera as /dev/%s\n",
		 video_device_node_name(&lc->video));
	return 0;

err_v4l2:
	v4l2_device_unregister(&lc->v4l2);
err_rx:
	dma_release_channel(lc->rx);
	return ret;
}

static int p4_lcdcam_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct p4_lcdcam *lc;
	int ret;

	lc = devm_kzalloc(dev, sizeof(*lc), GFP_KERNEL);
	if (!lc)
		return -ENOMEM;
	lc->dev = dev;
	spin_lock_init(&lc->slock);
	mutex_init(&lc->lock);
	INIT_LIST_HEAD(&lc->buf_list);
	init_completion(&lc->lcd_vsync);
	lc->bus_width = 8;
	device_property_read_u32(dev, "bus-width", &lc->bus_width);
	platform_set_drvdata(pdev, lc);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	lc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(lc->base))
		return PTR_ERR(lc->base);
	lc->lcd_clk = devm_clk_get(dev, "lcd");
	if (IS_ERR(lc->lcd_clk))
		return PTR_ERR(lc->lcd_clk);
	lc->cam_clk = devm_clk_get(dev, "cam");
	if (IS_ERR(lc->cam_clk))
		return PTR_ERR(lc->cam_clk);
	lc->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(lc->reset))
		return PTR_ERR(lc->reset);
	ret = reset_control_reset(lc->reset);
	if (ret)
		return ret;

	lc->irq = platform_get_irq(pdev, 0);
	if (lc->irq < 0)
		return lc->irq;
	ret = devm_request_irq(dev, lc->irq, p4_lcdcam_irq, IRQF_SHARED,
			       dev_name(dev), lc);
	if (ret)
		return ret;

	ret = p4_lcd_init(lc);
	if (ret)
		return ret;

	if (device_property_read_bool(dev, "espressif,enable-cam") || !lc->has_lcd) {
		ret = p4_cam_init(lc);
		if (ret)
			return ret;
	}
	return 0;
}

static void p4_lcdcam_remove(struct platform_device *pdev)
{
	struct p4_lcdcam *lc = platform_get_drvdata(pdev);

	if (lc->has_cam) {
		video_unregister_device(&lc->video);
		v4l2_device_unregister(&lc->v4l2);
		dma_release_channel(lc->rx);
	}
	if (lc->has_lcd) {
		unregister_framebuffer(lc->fb);
		dmaengine_terminate_sync(lc->tx);
		framebuffer_release(lc->fb);
		dma_free_coherent(lc->dev, lc->fb_size, lc->fb_virt, lc->fb_dma);
		clk_disable_unprepare(lc->lcd_clk);
		dma_release_channel(lc->tx);
	}
}

static const struct of_device_id p4_lcdcam_match[] = {
	{ .compatible = "espressif,esp32p4-lcdcam" },
	{ }
};
MODULE_DEVICE_TABLE(of, p4_lcdcam_match);

static struct platform_driver p4_lcdcam_driver = {
	.probe = p4_lcdcam_probe,
	.remove = p4_lcdcam_remove,
	.driver = {
		.name = "esp32p4-lcdcam",
		.of_match_table = p4_lcdcam_match,
	},
};
module_platform_driver(p4_lcdcam_driver);

MODULE_DESCRIPTION("ESP32-P4 LCD_CAM parallel LCD and DVP capture driver");
MODULE_LICENSE("GPL");
