// SPDX-License-Identifier: GPL-2.0-only
/* Two independent image inputs and one output, using standard media queues. */
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <media/media-device.h>
#include <media/media-devnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include "s31-media.h"

#define S31_BLEND_PORTS 3
#define S31_BLEND_ALL GENMASK(2, 0)

struct s31_blend_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head entry;
	u8 alpha;
};

struct s31_blend_port {
	struct s31_blend *blend;
	struct video_device video;
	struct vb2_queue queue;
	struct media_pad pad;
	struct v4l2_pix_format format;
	struct v4l2_rect rect;
	struct v4l2_ctrl_handler controls;
	struct v4l2_ctrl *alpha;
	struct list_head ready;
	unsigned int index;
	bool queue_initialized, pad_initialized;
};

struct s31_blend {
	struct s31_media *dev;
	struct media_device media;
	struct v4l2_device v4l2;
	struct v4l2_subdev subdev;
	struct media_pad pads[S31_BLEND_PORTS];
	struct media_pipeline pipeline;
	struct s31_blend_port port[S31_BLEND_PORTS];
	struct work_struct work;
	/* Protects ready lists, streaming and running; work never takes d->lock. */
	spinlock_t queue_lock;
	u32 streaming, sequence;
	bool running, ready, removed, subdev_pads;
};

static struct s31_blend_buffer *s31_blend_buffer(struct vb2_buffer *vb)
{
	return container_of(to_vb2_v4l2_buffer(vb), struct s31_blend_buffer, vb);
}

static bool s31_blend_busy(struct s31_blend *b)
{
	unsigned int i;

	for (i = 0; i < S31_BLEND_PORTS; i++)
		if (vb2_is_busy(&b->port[i].queue))
			return true;
	return false;
}

static void s31_blend_schedule(struct s31_blend *b)
{
	unsigned int i;

	lockdep_assert_held(&b->queue_lock);
	if (b->streaming != S31_BLEND_ALL || b->running)
		return;
	for (i = 0; i < S31_BLEND_PORTS; i++)
		if (list_empty(&b->port[i].ready))
			return;
	b->running = true;
	queue_work(system_freezable_wq, &b->work);
}

static void s31_blend_work(struct work_struct *work)
{
	struct s31_blend *b = container_of(work, struct s31_blend, work);
	struct s31_media *d = b->dev;
	struct s31_blend_buffer *buf[S31_BLEND_PORTS];
	struct s31_blend_image im[S31_BLEND_PORTS];
	unsigned long flags;
	u64 timestamp;
	unsigned int i;
	int ret;

	for (;;) {
		spin_lock_irqsave(&b->queue_lock, flags);
		for (i = 0; i < S31_BLEND_PORTS; i++)
			if (list_empty(&b->port[i].ready))
				break;
		if (b->streaming != S31_BLEND_ALL || i != S31_BLEND_PORTS) {
			b->running = false;
			spin_unlock_irqrestore(&b->queue_lock, flags);
			return;
		}
		for (i = 0; i < S31_BLEND_PORTS; i++) {
			buf[i] = list_first_entry(&b->port[i].ready,
						  struct s31_blend_buffer, entry);
			list_del(&buf[i]->entry);
			im[i] = (struct s31_blend_image){
				.format = &b->port[i].format, .rect = &b->port[i].rect,
				.buffer = &buf[i]->vb, .alpha = buf[i]->alpha,
			};
		}
		spin_unlock_irqrestore(&b->queue_lock, flags);

		ret = -ENODEV;
		mutex_lock(&d->hardware_lock);
		if (!d->removed && !READ_ONCE(b->removed) &&
		    READ_ONCE(b->streaming) == S31_BLEND_ALL) {
			ret = pm_runtime_resume_and_get(d->dev);
			if (ret >= 0) {
				ret = s31_ppa_blend_run(d, im);
				pm_runtime_mark_last_busy(d->dev);
				pm_runtime_put_autosuspend(d->dev);
			}
		}
		mutex_unlock(&d->hardware_lock);
		if (READ_ONCE(b->streaming) != S31_BLEND_ALL)
			ret = -ECANCELED;
		timestamp = ktime_get_ns();
		for (i = 0; i < S31_BLEND_PORTS; i++) {
			buf[i]->vb.sequence = b->sequence;
			buf[i]->vb.field = V4L2_FIELD_NONE;
			buf[i]->vb.vb2_buf.timestamp = timestamp;
			if (i == 2)
				vb2_set_plane_payload(&buf[i]->vb.vb2_buf, 0,
						      ret ? 0 : im[i].format->sizeimage);
			vb2_buffer_done(&buf[i]->vb.vb2_buf,
					ret ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
		}
		b->sequence++;
	}
}

static void s31_blend_return(struct s31_blend_port *port,
			     enum vb2_buffer_state state)
{
	struct s31_blend_buffer *buf;

	lockdep_assert_held(&port->blend->queue_lock);
	while (!list_empty(&port->ready)) {
		buf = list_first_entry(&port->ready, struct s31_blend_buffer, entry);
		list_del(&buf->entry);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
}

static int s31_blend_queue_setup(struct vb2_queue *q, unsigned int *count,
				 unsigned int *planes, unsigned int sizes[],
				 struct device *alloc_devs[])
{
	struct s31_blend_port *p = vb2_get_drv_priv(q);

	if (vb2_get_num_buffers(q) >= 2)
		return -ENOSPC;
	*count = min_t(unsigned int, *count, 2 - vb2_get_num_buffers(q));
	if (*planes)
		return *planes == 1 && sizes[0] >= p->format.sizeimage ? 0 : -EINVAL;
	*planes = 1;
	sizes[0] = p->format.sizeimage;
	return 0;
}

static int s31_blend_prepare(struct vb2_buffer *vb)
{
	struct s31_blend_port *p = vb2_get_drv_priv(vb->vb2_queue);
	struct s31_blend_buffer *buf = s31_blend_buffer(vb);

	if (!READ_ONCE(p->blend->ready) || READ_ONCE(p->blend->removed) ||
	    READ_ONCE(p->blend->dev->removed))
		return -ENODEV;
	if (vb2_plane_size(vb, 0) < p->format.sizeimage ||
	    (p->index < 2 && vb2_get_plane_payload(vb, 0) < p->format.sizeimage))
		return -EINVAL;
	if (p->alpha) {
		mutex_lock(p->controls.lock);
		buf->alpha = p->alpha->cur.val;
		mutex_unlock(p->controls.lock);
	}
	return 0;
}

static void s31_blend_queue(struct vb2_buffer *vb)
{
	struct s31_blend_port *p = vb2_get_drv_priv(vb->vb2_queue);
	struct s31_blend *b = p->blend;
	unsigned long flags;

	spin_lock_irqsave(&b->queue_lock, flags);
	list_add_tail(&s31_blend_buffer(vb)->entry, &p->ready);
	s31_blend_schedule(b);
	spin_unlock_irqrestore(&b->queue_lock, flags);
}

static int s31_blend_start(struct vb2_queue *q, unsigned int count)
{
	struct s31_blend_port *p = vb2_get_drv_priv(q);
	struct s31_blend *b = p->blend;
	struct s31_blend_image im[S31_BLEND_PORTS];
	unsigned long flags;
	unsigned int i;
	int ret;

	for (i = 0; i < S31_BLEND_PORTS; i++)
		im[i] = (struct s31_blend_image){
			.format = &b->port[i].format, .rect = &b->port[i].rect,
		};
	ret = !b->ready || b->removed || b->dev->removed ?
		-ENODEV : s31_ppa_blend_validate(im);
	if (!ret)
		ret = media_pipeline_start(&p->pad, &b->pipeline);
	spin_lock_irqsave(&b->queue_lock, flags);
	if (ret) {
		s31_blend_return(p, VB2_BUF_STATE_QUEUED);
	} else {
		if (!b->streaming)
			b->sequence = 0;
		b->streaming |= BIT(p->index);
		s31_blend_schedule(b);
	}
	spin_unlock_irqrestore(&b->queue_lock, flags);
	return ret;
}

static void s31_blend_stop(struct vb2_queue *q)
{
	struct s31_blend_port *p = vb2_get_drv_priv(q);
	struct s31_blend *b = p->blend;
	unsigned long flags;
	bool streaming;
	unsigned int i;

	spin_lock_irqsave(&b->queue_lock, flags);
	streaming = b->streaming & BIT(p->index);
	b->streaming &= ~BIT(p->index);
	spin_unlock_irqrestore(&b->queue_lock, flags);
	/* Stopping any member invalidates an entire pending triple. Join the
	 * bounded DMA job before returning any of its buffers to a closing file.
	 */
	cancel_work_sync(&b->work);
	spin_lock_irqsave(&b->queue_lock, flags);
	for (i = 0; i < S31_BLEND_PORTS; i++)
		s31_blend_return(&b->port[i], VB2_BUF_STATE_ERROR);
	b->running = false;
	spin_unlock_irqrestore(&b->queue_lock, flags);
	if (streaming)
		media_pipeline_stop(&p->pad);
}

static const struct vb2_ops s31_blend_queue_ops = {
	.queue_setup = s31_blend_queue_setup,
	.buf_prepare = s31_blend_prepare,
	.buf_queue = s31_blend_queue,
	.start_streaming = s31_blend_start,
	.stop_streaming = s31_blend_stop,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int s31_blend_querycap(struct file *file, void *priv,
			      struct v4l2_capability *cap)
{
	struct s31_blend_port *p = video_drvdata(file);

	strscpy(cap->driver, "esp32s31-media", sizeof(cap->driver));
	strscpy(cap->card, p->video.name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s:blend%u",
		 dev_name(p->blend->dev->dev), p->index);
	return 0;
}

static bool s31_blend_foreground_format(u32 code)
{
	return code == V4L2_PIX_FMT_RGB24 || code == V4L2_PIX_FMT_BGR24 ||
		code == V4L2_PIX_FMT_RGB565 || code == V4L2_PIX_FMT_ABGR32 ||
		code == V4L2_PIX_FMT_RGBA32;
}

static int s31_blend_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	struct s31_blend_port *p = video_drvdata(file);
	struct v4l2_fmtdesc candidate = *f;
	unsigned int index = 0, wanted = f->index;
	int ret;

	for (candidate.index = 0; ; candidate.index++) {
		ret = s31_enum_format(p->blend->dev, &candidate);
		if (ret)
			return ret;
		if (p->index == 1 &&
		    !s31_blend_foreground_format(candidate.pixelformat))
			continue;
		if (index++ == wanted) {
			f->pixelformat = candidate.pixelformat;
			return 0;
		}
	}
}

static int s31_blend_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct s31_blend_port *p = video_drvdata(file);

	if (p->index == 1 && !s31_blend_foreground_format(f->fmt.pix.pixelformat))
		f->fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
	s31_normalize_format(p->blend->dev, &f->fmt.pix);
	return 0;
}

static int s31_blend_get_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct s31_blend_port *p = video_drvdata(file);

	f->fmt.pix = p->format;
	return 0;
}

static int s31_blend_set_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct s31_blend_port *p = video_drvdata(file);

	if (s31_blend_busy(p->blend))
		return -EBUSY;
	if (f->fmt.pix.flags & V4L2_PIX_FMT_FLAG_PREMUL_ALPHA)
		return -EINVAL;
	s31_blend_try_fmt(file, priv, f);
	p->format = f->fmt.pix;
	p->rect = (struct v4l2_rect){ .width = p->format.width,
				    .height = p->format.height };
	return 0;
}

static int s31_blend_get_selection(struct file *file, void *priv,
				   struct v4l2_selection *s)
{
	struct s31_blend_port *p = video_drvdata(file);
	u32 target = p->index == 2 ? V4L2_SEL_TGT_COMPOSE : V4L2_SEL_TGT_CROP;

	if (s->type != p->queue.type)
		return -EINVAL;
	if (s->target == target)
		s->r = p->rect;
	else if (s->target == target + 1 || s->target == target + 2)
		s->r = (struct v4l2_rect){ .width = p->format.width,
					 .height = p->format.height };
	else
		return -EINVAL;
	return 0;
}

static int s31_blend_set_selection(struct file *file, void *priv,
				   struct v4l2_selection *s)
{
	struct s31_blend_port *p = video_drvdata(file);
	struct v4l2_rect rect = s->r;
	u32 code = p->format.pixelformat;
	u32 align = code == V4L2_PIX_FMT_RGB565 || code == V4L2_PIX_FMT_ABGR32 ||
		code == V4L2_PIX_FMT_RGBA32 ? 1 : 2;
	int ret;

	if (s->type != p->queue.type ||
	    s->target != (p->index == 2 ? V4L2_SEL_TGT_COMPOSE : V4L2_SEL_TGT_CROP) ||
	    s->flags & ~(V4L2_SEL_FLAG_GE | V4L2_SEL_FLAG_LE))
		return -EINVAL;
	if (s31_blend_busy(p->blend))
		return -EBUSY;
	ret = s31_select_axis(&rect.left, &rect.width, p->format.width, align, s->flags);
	if (!ret)
		ret = s31_select_axis(&rect.top, &rect.height, p->format.height, 1, s->flags);
	if (ret)
		return ret;
	p->rect = rect;
	s->r = rect;
	return 0;
}

static int s31_blend_reqbufs(struct file *file, void *priv,
			     struct v4l2_requestbuffers *r)
{
	if (r->memory == V4L2_MEMORY_MMAP)
		r->flags |= V4L2_MEMORY_FLAG_NON_COHERENT;
	return vb2_ioctl_reqbufs(file, priv, r);
}

static int s31_blend_create_bufs(struct file *file, void *priv,
				 struct v4l2_create_buffers *r)
{
	if (r->memory == V4L2_MEMORY_MMAP)
		r->flags |= V4L2_MEMORY_FLAG_NON_COHERENT;
	return vb2_ioctl_create_bufs(file, priv, r);
}

static const struct v4l2_ioctl_ops s31_blend_ioctl_ops = {
	.vidioc_querycap = s31_blend_querycap,
	.vidioc_enum_fmt_vid_cap = s31_blend_enum_fmt,
	.vidioc_enum_fmt_vid_out = s31_blend_enum_fmt,
	.vidioc_try_fmt_vid_cap = s31_blend_try_fmt,
	.vidioc_try_fmt_vid_out = s31_blend_try_fmt,
	.vidioc_g_fmt_vid_cap = s31_blend_get_fmt,
	.vidioc_g_fmt_vid_out = s31_blend_get_fmt,
	.vidioc_s_fmt_vid_cap = s31_blend_set_fmt,
	.vidioc_s_fmt_vid_out = s31_blend_set_fmt,
	.vidioc_g_selection = s31_blend_get_selection,
	.vidioc_s_selection = s31_blend_set_selection,
	.vidioc_reqbufs = s31_blend_reqbufs,
	.vidioc_create_bufs = s31_blend_create_bufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations s31_blend_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.poll = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
};

static u32 s31_blend_mbus_code(u32 fourcc)
{
	switch (fourcc) {
	case V4L2_PIX_FMT_RGB565:
		return MEDIA_BUS_FMT_RGB565_1X16;
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_BGR24:
		return MEDIA_BUS_FMT_RGB888_1X24;
	case V4L2_PIX_FMT_GREY:
		return MEDIA_BUS_FMT_Y8_1X8;
	case V4L2_PIX_FMT_UYVY:
		return MEDIA_BUS_FMT_UYVY8_1X16;
	default:
		return MEDIA_BUS_FMT_ARGB8888_1X32;
	}
}

static int s31_blend_pad_get_fmt(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *f)
{
	struct s31_blend *b = container_of(sd, struct s31_blend, subdev);
	struct s31_blend_port *p;

	if (f->pad >= S31_BLEND_PORTS || f->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;
	mutex_lock(&b->dev->lock);
	p = &b->port[f->pad];
	f->format = (struct v4l2_mbus_framefmt){
		.width = p->rect.width, .height = p->rect.height,
		.code = s31_blend_mbus_code(p->format.pixelformat),
		.field = V4L2_FIELD_NONE, .colorspace = V4L2_COLORSPACE_SRGB,
		.ycbcr_enc = V4L2_YCBCR_ENC_601,
		.quantization = V4L2_QUANTIZATION_FULL_RANGE,
		.xfer_func = V4L2_XFER_FUNC_SRGB,
	};
	mutex_unlock(&b->dev->lock);
	return 0;
}

static const struct v4l2_subdev_pad_ops s31_blend_pad_ops = {
	.get_fmt = s31_blend_pad_get_fmt,
};

static const struct v4l2_subdev_ops s31_blend_subdev_ops = {
	.pad = &s31_blend_pad_ops,
};

static int s31_blend_link_validate(struct media_link *link)
{
	/* Immutable links connect these three queues directly to the compositor.
	 * Its START admission validates both cropped inputs against the composed
	 * output, not the larger memory canvases or unlike packed pixel formats.
	 */
	return 0;
}

static const struct media_entity_operations s31_blend_entity_ops = {
	.link_validate = s31_blend_link_validate,
};

static void s31_blend_release(struct v4l2_device *v4l2)
{
	struct s31_blend *b = container_of(v4l2, struct s31_blend, v4l2);
	struct s31_media *d = b->dev;
	unsigned int i;

	for (i = 0; i < S31_BLEND_PORTS; i++) {
		if (b->port[i].queue_initialized)
			vb2_queue_release(&b->port[i].queue);
		if (b->port[i].pad_initialized)
			media_entity_cleanup(&b->port[i].video.entity);
		v4l2_ctrl_handler_free(&b->port[i].controls);
	}
	if (b->subdev_pads)
		media_entity_cleanup(&b->subdev.entity);
	media_device_cleanup(&b->media);
	kfree(b);
	s31_media_put(d);
}

static void s31_blend_media_release(struct media_devnode *node)
{
	struct s31_blend *b = container_of(node->media_dev, struct s31_blend, media);

	v4l2_device_put(&b->v4l2);
}

static void s31_blend_remove(struct s31_blend *b)
{
	unsigned int i;

	mutex_lock(&b->dev->lock);
	WRITE_ONCE(b->removed, true);
	for (i = 0; i < S31_BLEND_PORTS; i++) {
		if (b->port[i].queue_initialized) {
			vb2_queue_error(&b->port[i].queue);
			s31_blend_stop(&b->port[i].queue);
		}
	}
	media_device_unregister(&b->media);
	for (i = 0; i < S31_BLEND_PORTS; i++)
		if (video_is_registered(&b->port[i].video))
			video_unregister_device(&b->port[i].video);
	v4l2_device_unregister(&b->v4l2);
	mutex_unlock(&b->dev->lock);
	/* Core video/subdev references and the media devnode keep the entire
	 * graph alive until all open files and video-backed VMAs are gone.
	 */
	v4l2_device_put(&b->v4l2);
}

static int s31_blend_port_register(struct s31_blend *b, unsigned int index)
{
	static const char * const names[] = {
		"s31-ppa-background", "s31-ppa-foreground", "s31-ppa-composed",
	};
	struct s31_blend_port *p = &b->port[index];
	struct vb2_queue *q = &p->queue;
	int ret;

	p->blend = b;
	p->index = index;
	p->format = (struct v4l2_pix_format){ .width = 80, .height = 64,
		.pixelformat = V4L2_PIX_FMT_BGR24 };
	s31_normalize_format(b->dev, &p->format);
	p->rect = (struct v4l2_rect){ .width = p->format.width,
				    .height = p->format.height };
	v4l2_ctrl_handler_init(&p->controls, index < 2 ? 1 : 0);
	if (index < 2)
		p->alpha = v4l2_ctrl_new_std(&p->controls, NULL,
					     V4L2_CID_ALPHA_COMPONENT, 0, 255, 1, 255);
	if (p->controls.error)
		return p->controls.error;
	q->type = index == 2 ? V4L2_BUF_TYPE_VIDEO_CAPTURE : V4L2_BUF_TYPE_VIDEO_OUTPUT;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->allow_cache_hints = 1;
	q->drv_priv = p;
	q->buf_struct_size = sizeof(struct s31_blend_buffer);
	q->ops = &s31_blend_queue_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &b->dev->lock;
	q->dev = s31_dma2d_device(b->dev->dma);
	ret = vb2_queue_init(q);
	if (ret)
		return ret;
	p->queue_initialized = true;
	strscpy(p->video.name, names[index], sizeof(p->video.name));
	p->video.v4l2_dev = &b->v4l2;
	p->video.fops = &s31_blend_fops;
	p->video.ioctl_ops = &s31_blend_ioctl_ops;
	p->video.release = video_device_release_empty;
	p->video.lock = &b->dev->lock;
	p->video.queue = q;
	p->video.ctrl_handler = &p->controls;
	p->video.vfl_dir = index == 2 ? VFL_DIR_RX : VFL_DIR_TX;
	p->video.device_caps = V4L2_CAP_STREAMING | V4L2_CAP_IO_MC |
		(index == 2 ? V4L2_CAP_VIDEO_CAPTURE : V4L2_CAP_VIDEO_OUTPUT);
	p->pad.flags = index == 2 ? MEDIA_PAD_FL_SINK : MEDIA_PAD_FL_SOURCE;
	p->video.entity.function = MEDIA_ENT_F_IO_V4L;
	ret = media_entity_pads_init(&p->video.entity, 1, &p->pad);
	if (ret)
		return ret;
	p->pad_initialized = true;
	video_set_drvdata(&p->video, p);
	return video_register_device(&p->video, VFL_TYPE_VIDEO, -1);
}

int s31_blend_register(struct s31_media *d)
{
	struct s31_blend *b;
	unsigned int i;
	int ret;

	b = kzalloc(sizeof(*b), GFP_KERNEL);
	if (!b)
		return -ENOMEM;
	b->dev = d;
	INIT_WORK(&b->work, s31_blend_work);
	spin_lock_init(&b->queue_lock);
	for (i = 0; i < S31_BLEND_PORTS; i++)
		INIT_LIST_HEAD(&b->port[i].ready);
	b->media.dev = d->dev;
	strscpy(b->media.model, "S31 PPA compositor", sizeof(b->media.model));
	media_device_init(&b->media);
	b->v4l2.mdev = &b->media;
	b->v4l2.release = s31_blend_release;
	ret = v4l2_device_register(d->dev, &b->v4l2);
	if (ret) {
		media_device_cleanup(&b->media);
		kfree(b);
		return ret;
	}
	s31_media_get(d);
	for (i = 0; i < S31_BLEND_PORTS; i++) {
		ret = s31_blend_port_register(b, i);
		if (ret)
			goto fail;
	}
	v4l2_subdev_init(&b->subdev, &s31_blend_subdev_ops);
	b->subdev.owner = THIS_MODULE;
	b->subdev.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	b->subdev.entity.function = MEDIA_ENT_F_PROC_VIDEO_COMPOSER;
	b->subdev.entity.ops = &s31_blend_entity_ops;
	strscpy(b->subdev.name, "s31-ppa-blend", sizeof(b->subdev.name));
	for (i = 0; i < S31_BLEND_PORTS; i++)
		b->pads[i].flags = i == 2 ? MEDIA_PAD_FL_SOURCE : MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&b->subdev.entity, S31_BLEND_PORTS, b->pads);
	if (ret)
		goto fail;
	b->subdev_pads = true;
	ret = v4l2_device_register_subdev(&b->v4l2, &b->subdev);
	if (ret)
		goto fail;
	for (i = 0; i < S31_BLEND_PORTS; i++) {
		struct media_entity *from = i == 2 ? &b->subdev.entity :
			&b->port[i].video.entity;
		struct media_entity *to = i == 2 ? &b->port[i].video.entity :
			&b->subdev.entity;

		ret = media_create_pad_link(from, i == 2 ? 2 : 0, to, i == 2 ? 0 : i,
					    MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
		if (ret)
			goto fail;
	}
	ret = v4l2_device_register_ro_subdev_nodes(&b->v4l2);
	if (ret)
		goto fail;
	v4l2_device_get(&b->v4l2);
	ret = media_device_register(&b->media);
	if (ret) {
		v4l2_device_put(&b->v4l2);
		goto fail;
	}
	b->media.devnode->release = s31_blend_media_release;
	WRITE_ONCE(b->ready, true);
	d->blend = b;
	dev_info(d->dev, "PPA blend: two OUTPUT queues and one CAPTURE queue\n");
	return 0;

fail:
	s31_blend_remove(b);
	return ret;
}

void s31_blend_unregister(struct s31_media *d)
{
	struct s31_blend *b = d->blend;

	if (b) {
		d->blend = NULL;
		s31_blend_remove(b);
	}
}
