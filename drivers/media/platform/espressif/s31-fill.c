// SPDX-License-Identifier: GPL-2.0-only
/* Demand-driven PPA solid-color capture, without an unused M2M source queue. */
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <media/v4l2-ioctl.h>
#include "s31-media.h"

struct s31_fill_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head entry;
	u32 rgb, alpha;
};

struct s31_fill {
	struct s31_media *dev;
	struct video_device video;
	struct vb2_queue queue;
	struct v4l2_ctrl_handler controls;
	struct v4l2_ctrl *red, *green, *blue, *alpha;
	struct v4l2_pix_format format;
	struct v4l2_rect compose;
	struct work_struct work;
	/* Protects ready, stopping and running; the worker does not take q->lock. */
	spinlock_t queue_lock;
	struct list_head ready;
	u32 sequence;
	bool stopping, running;
};

static struct s31_fill_buffer *s31_fill_buffer(struct vb2_buffer *vb)
{
	return container_of(to_vb2_v4l2_buffer(vb), struct s31_fill_buffer, vb);
}

static void s31_fill_schedule(struct s31_fill *fill)
{
	lockdep_assert_held(&fill->queue_lock);
	if (!fill->stopping && !fill->running && !list_empty(&fill->ready)) {
		fill->running = true;
		queue_work(system_freezable_wq, &fill->work);
	}
}

static void s31_fill_work(struct work_struct *work)
{
	struct s31_fill *fill = container_of(work, struct s31_fill, work);
	struct s31_media *d = fill->dev;
	struct s31_fill_buffer *buf;
	unsigned long flags;
	size_t bytes;
	int ret;

	for (;;) {
		spin_lock_irqsave(&fill->queue_lock, flags);
		if (fill->stopping || list_empty(&fill->ready)) {
			fill->running = false;
			spin_unlock_irqrestore(&fill->queue_lock, flags);
			return;
		}
		buf = list_first_entry(&fill->ready, struct s31_fill_buffer, entry);
		list_del(&buf->entry);
		spin_unlock_irqrestore(&fill->queue_lock, flags);

		bytes = 0;
		ret = -ENODEV;
		mutex_lock(&d->hardware_lock);
		if (!d->removed && !READ_ONCE(fill->stopping)) {
			ret = pm_runtime_resume_and_get(d->dev);
			if (ret >= 0) {
				ret = s31_ppa_fill_image(d, &fill->format, &fill->compose,
							 buf->rgb, buf->alpha, &buf->vb, &bytes);
				pm_runtime_mark_last_busy(d->dev);
				pm_runtime_put_autosuspend(d->dev);
			}
		}
		mutex_unlock(&d->hardware_lock);
		if (READ_ONCE(fill->stopping))
			ret = -ECANCELED;
		buf->vb.sequence = fill->sequence++;
		buf->vb.field = V4L2_FIELD_NONE;
		buf->vb.vb2_buf.timestamp = ktime_get_ns();
		vb2_set_plane_payload(&buf->vb.vb2_buf, 0, ret ? 0 : bytes);
		vb2_buffer_done(&buf->vb.vb2_buf,
				ret ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
	}
}

static int s31_fill_queue_setup(struct vb2_queue *q, unsigned int *count,
				unsigned int *planes, unsigned int sizes[],
				struct device *alloc_devs[])
{
	struct s31_fill *fill = vb2_get_drv_priv(q);

	if (vb2_get_num_buffers(q) >= 2)
		return -ENOSPC;
	*count = min_t(unsigned int, *count, 2 - vb2_get_num_buffers(q));
	if (*planes)
		return *planes == 1 && sizes[0] >= fill->format.sizeimage ? 0 : -EINVAL;
	*planes = 1;
	sizes[0] = fill->format.sizeimage;
	return 0;
}

static int s31_fill_prepare(struct vb2_buffer *vb)
{
	struct s31_fill *fill = vb2_get_drv_priv(vb->vb2_queue);
	struct s31_fill_buffer *buf = s31_fill_buffer(vb);

	if (READ_ONCE(fill->dev->removed))
		return -ENODEV;
	if (vb2_plane_size(vb, 0) < fill->format.sizeimage)
		return -EINVAL;
	/* The capture queue lock keeps geometry fixed. Snapshot all color
	 * components under their own handler lock, never from the worker that
	 * STREAMOFF/close must join while holding the capture queue lock.
	 */
	mutex_lock(fill->controls.lock);
	buf->rgb = (fill->red->cur.val << 16) | (fill->green->cur.val << 8) |
		   fill->blue->cur.val;
	buf->alpha = fill->alpha->cur.val;
	mutex_unlock(fill->controls.lock);
	return 0;
}

static void s31_fill_queue(struct vb2_buffer *vb)
{
	struct s31_fill *fill = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long flags;

	spin_lock_irqsave(&fill->queue_lock, flags);
	list_add_tail(&s31_fill_buffer(vb)->entry, &fill->ready);
	s31_fill_schedule(fill);
	spin_unlock_irqrestore(&fill->queue_lock, flags);
}

static void s31_fill_return_buffers(struct s31_fill *fill,
				    enum vb2_buffer_state state)
{
	struct s31_fill_buffer *buf;
	unsigned long flags;

	spin_lock_irqsave(&fill->queue_lock, flags);
	while (!list_empty(&fill->ready)) {
		buf = list_first_entry(&fill->ready, struct s31_fill_buffer, entry);
		list_del(&buf->entry);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	fill->running = false;
	spin_unlock_irqrestore(&fill->queue_lock, flags);
}

static int s31_fill_start(struct vb2_queue *q, unsigned int count)
{
	struct s31_fill *fill = vb2_get_drv_priv(q);
	unsigned long flags;

	if (READ_ONCE(fill->dev->removed)) {
		s31_fill_return_buffers(fill, VB2_BUF_STATE_QUEUED);
		return -ENODEV;
	}
	spin_lock_irqsave(&fill->queue_lock, flags);
	fill->sequence = 0;
	fill->stopping = false;
	s31_fill_schedule(fill);
	spin_unlock_irqrestore(&fill->queue_lock, flags);
	return 0;
}

static void s31_fill_stop(struct vb2_queue *q)
{
	struct s31_fill *fill = vb2_get_drv_priv(q);
	unsigned long flags;

	spin_lock_irqsave(&fill->queue_lock, flags);
	fill->stopping = true;
	spin_unlock_irqrestore(&fill->queue_lock, flags);
	/* A running transfer is bounded by the shared DMA engine's timeout.
	 * Join it before returning buffers or releasing their backing memory.
	 */
	cancel_work_sync(&fill->work);
	s31_fill_return_buffers(fill, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops s31_fill_queue_ops = {
	.queue_setup = s31_fill_queue_setup,
	.buf_prepare = s31_fill_prepare,
	.buf_queue = s31_fill_queue,
	.start_streaming = s31_fill_start,
	.stop_streaming = s31_fill_stop,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int s31_fill_querycap(struct file *file, void *priv,
			     struct v4l2_capability *cap)
{
	struct s31_fill *fill = video_drvdata(file);

	strscpy(cap->driver, "esp32s31-media", sizeof(cap->driver));
	strscpy(cap->card, "S31 PPA solid color", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s:fill",
		 dev_name(fill->dev->dev));
	return 0;
}

static int s31_fill_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	struct s31_fill *fill = video_drvdata(file);

	return s31_enum_format(fill->dev, f);
}

static int s31_fill_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct s31_fill *fill = video_drvdata(file);

	s31_normalize_format(fill->dev, &f->fmt.pix);
	return 0;
}

static int s31_fill_get_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct s31_fill *fill = video_drvdata(file);

	f->fmt.pix = fill->format;
	return 0;
}

static int s31_fill_set_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct s31_fill *fill = video_drvdata(file);

	if (vb2_is_busy(&fill->queue))
		return -EBUSY;
	s31_normalize_format(fill->dev, &f->fmt.pix);
	fill->format = f->fmt.pix;
	fill->compose = (struct v4l2_rect){ .width = f->fmt.pix.width,
					 .height = f->fmt.pix.height };
	return 0;
}

static int s31_fill_get_selection(struct file *file, void *priv,
				  struct v4l2_selection *s)
{
	struct s31_fill *fill = video_drvdata(file);

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE:
		s->r = fill->compose;
		return 0;
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		s->r = (struct v4l2_rect){ .width = fill->format.width,
					 .height = fill->format.height };
		return 0;
	default:
		return -EINVAL;
	}
}

static int s31_fill_set_selection(struct file *file, void *priv,
				  struct v4l2_selection *s)
{
	struct s31_fill *fill = video_drvdata(file);
	struct v4l2_rect rect = s->r;
	u32 code = fill->format.pixelformat;
	u32 align = code == V4L2_PIX_FMT_RGB565 || code == V4L2_PIX_FMT_ABGR32 ||
		    code == V4L2_PIX_FMT_RGBA32 ? 1 : 2;
	int ret;

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
	    s->target != V4L2_SEL_TGT_COMPOSE ||
	    s->flags & ~(V4L2_SEL_FLAG_GE | V4L2_SEL_FLAG_LE))
		return -EINVAL;
	if (vb2_is_busy(&fill->queue))
		return -EBUSY;
	ret = s31_select_axis(&rect.left, &rect.width, fill->format.width,
			      align, s->flags);
	if (!ret)
		ret = s31_select_axis(&rect.top, &rect.height, fill->format.height,
				      1, s->flags);
	if (ret)
		return ret;
	fill->compose = rect;
	s->r = rect;
	return 0;
}

static int s31_fill_reqbufs(struct file *file, void *priv,
			    struct v4l2_requestbuffers *r)
{
	if (r->memory == V4L2_MEMORY_MMAP)
		r->flags |= V4L2_MEMORY_FLAG_NON_COHERENT;
	return vb2_ioctl_reqbufs(file, priv, r);
}

static int s31_fill_create_bufs(struct file *file, void *priv,
				struct v4l2_create_buffers *r)
{
	if (r->memory == V4L2_MEMORY_MMAP)
		r->flags |= V4L2_MEMORY_FLAG_NON_COHERENT;
	return vb2_ioctl_create_bufs(file, priv, r);
}

static const struct v4l2_ioctl_ops s31_fill_ioctl_ops = {
	.vidioc_querycap = s31_fill_querycap,
	.vidioc_enum_fmt_vid_cap = s31_fill_enum_fmt,
	.vidioc_try_fmt_vid_cap = s31_fill_try_fmt,
	.vidioc_g_fmt_vid_cap = s31_fill_get_fmt,
	.vidioc_s_fmt_vid_cap = s31_fill_set_fmt,
	.vidioc_g_selection = s31_fill_get_selection,
	.vidioc_s_selection = s31_fill_set_selection,
	.vidioc_reqbufs = s31_fill_reqbufs,
	.vidioc_create_bufs = s31_fill_create_bufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations s31_fill_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.poll = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
};

static void s31_fill_release(struct video_device *video)
{
	struct s31_fill *fill = container_of(video, struct s31_fill, video);
	struct s31_media *d = fill->dev;

	vb2_queue_release(&fill->queue);
	v4l2_ctrl_handler_free(&fill->controls);
	kfree(fill);
	s31_media_put(d);
}

int s31_fill_register(struct s31_media *d)
{
	static const char * const patterns[] = { "Solid Color", NULL };
	struct s31_fill *fill;
	struct vb2_queue *q;
	int ret;

	fill = kzalloc(sizeof(*fill), GFP_KERNEL);
	if (!fill)
		return -ENOMEM;
	fill->dev = d;
	fill->stopping = true;
	INIT_WORK(&fill->work, s31_fill_work);
	INIT_LIST_HEAD(&fill->ready);
	spin_lock_init(&fill->queue_lock);
	fill->format = (struct v4l2_pix_format){ .width = 80, .height = 64,
		.pixelformat = V4L2_PIX_FMT_RGB24 };
	s31_normalize_format(d, &fill->format);
	fill->compose = (struct v4l2_rect){ .width = fill->format.width,
					 .height = fill->format.height };
	v4l2_ctrl_handler_init(&fill->controls, 5);
	v4l2_ctrl_new_std_menu_items(&fill->controls, NULL, V4L2_CID_TEST_PATTERN,
				     0, 0, 0, patterns);
	fill->red = v4l2_ctrl_new_std(&fill->controls, NULL,
				      V4L2_CID_TEST_PATTERN_RED, 0, 255, 1, 0);
	fill->green = v4l2_ctrl_new_std(&fill->controls, NULL,
					V4L2_CID_TEST_PATTERN_GREENR, 0, 255, 1, 0);
	fill->blue = v4l2_ctrl_new_std(&fill->controls, NULL,
				       V4L2_CID_TEST_PATTERN_BLUE, 0, 255, 1, 0);
	fill->alpha = v4l2_ctrl_new_std(&fill->controls, NULL,
					V4L2_CID_ALPHA_COMPONENT, 0, 255, 1, 255);
	ret = fill->controls.error;
	if (ret)
		goto controls;
	q = &fill->queue;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->allow_cache_hints = 1;
	q->drv_priv = fill;
	q->buf_struct_size = sizeof(struct s31_fill_buffer);
	q->ops = &s31_fill_queue_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &d->lock;
	q->dev = s31_dma2d_device(d->dma);
	ret = vb2_queue_init(q);
	if (ret)
		goto controls;
	strscpy(fill->video.name, "s31-ppa-fill", sizeof(fill->video.name));
	fill->video.v4l2_dev = &d->v4l2;
	fill->video.fops = &s31_fill_fops;
	fill->video.ioctl_ops = &s31_fill_ioctl_ops;
	fill->video.release = s31_fill_release;
	fill->video.lock = &d->lock;
	fill->video.queue = q;
	fill->video.ctrl_handler = &fill->controls;
	fill->video.vfl_dir = VFL_DIR_RX;
	fill->video.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	video_set_drvdata(&fill->video, fill);
	s31_media_get(d);
	ret = video_register_device(&fill->video, VFL_TYPE_VIDEO, -1);
	if (ret) {
		s31_media_put(d);
		vb2_queue_release(q);
		goto controls;
	}
	d->fill = fill;
	dev_info(d->dev, "%s registered as /dev/video%d\n", fill->video.name,
		 fill->video.num);
	return 0;

controls:
	v4l2_ctrl_handler_free(&fill->controls);
	kfree(fill);
	return ret;
}

void s31_fill_unregister(struct s31_media *d)
{
	struct s31_fill *fill = d->fill;

	if (!fill)
		return;
	mutex_lock(&d->lock);
	vb2_queue_error(&fill->queue);
	s31_fill_stop(&fill->queue);
	d->fill = NULL;
	video_unregister_device(&fill->video);
	mutex_unlock(&d->lock);
}
