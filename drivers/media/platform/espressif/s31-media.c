// SPDX-License-Identifier: GPL-2.0-only
/* On-demand JPEG/PPA memory operations, without a permanent frame pool. */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ioctl.h>
#include "s31-media.h"

#define S31_IMAGE_MAX_WIDTH 1024
#define S31_IMAGE_MAX_HEIGHT 768
#define S31_JPEG_MAX_BYTES (1024 * 1024)
static const u32 s31_jpeg_formats[] = {
	V4L2_PIX_FMT_JPEG,
	V4L2_PIX_FMT_GREY,
	V4L2_PIX_FMT_RGB24,
	V4L2_PIX_FMT_BGR24,
};
static const u32 s31_ppa_formats[] = {
	V4L2_PIX_FMT_RGB24, V4L2_PIX_FMT_RGB565, V4L2_PIX_FMT_ABGR32,
	V4L2_PIX_FMT_RGBA32,
	V4L2_PIX_FMT_GREY,  V4L2_PIX_FMT_BGR24,	 V4L2_PIX_FMT_UYVY,
};
static struct s31_media_ctx *s31_ctx(void *priv)
{
	return container_of(priv, struct s31_media_ctx, fh);
}
static int s31_queue_index(enum v4l2_buf_type type)
{
	return V4L2_TYPE_IS_OUTPUT(type) ? 0 : 1;
}
static bool s31_has_format(struct s31_media *d, u32 code)
{
	const u32 *list = d->jpeg ? s31_jpeg_formats : s31_ppa_formats;
	unsigned int i, count = d->jpeg ? ARRAY_SIZE(s31_jpeg_formats) :
					  ARRAY_SIZE(s31_ppa_formats);
	for (i = 0; i < count; i++)
		if (code == list[i])
			return true;
	return false;
}

void s31_normalize_format(struct s31_media *d, struct v4l2_pix_format *f)
{
	u32 bpp;

	if (!s31_has_format(d, f->pixelformat))
		f->pixelformat = d->jpeg ? V4L2_PIX_FMT_GREY :
						  V4L2_PIX_FMT_RGB24;
	f->width = clamp_t(u32, ALIGN(f->width, 16), 16, S31_IMAGE_MAX_WIDTH);
	f->height =
		clamp_t(u32, ALIGN(f->height, 16), 16, S31_IMAGE_MAX_HEIGHT);
	f->field = V4L2_FIELD_NONE;
	f->flags = 0;
	f->priv = 0;
	f->colorspace = f->pixelformat == V4L2_PIX_FMT_JPEG ?
				V4L2_COLORSPACE_JPEG :
				V4L2_COLORSPACE_SRGB;
	f->ycbcr_enc = V4L2_YCBCR_ENC_601;
	f->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	f->xfer_func = V4L2_XFER_FUNC_SRGB;
	if (f->pixelformat == V4L2_PIX_FMT_JPEG) {
		f->bytesperline = 0;
		f->sizeimage =
			clamp_t(u32, f->sizeimage, 4096, S31_JPEG_MAX_BYTES);
		return;
	}
	switch (f->pixelformat) {
	case V4L2_PIX_FMT_ABGR32:
	case V4L2_PIX_FMT_RGBA32:
		bpp = 4;
		break;
	case V4L2_PIX_FMT_BGR24:
	case V4L2_PIX_FMT_RGB24:
		bpp = 3;
		break;
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_UYVY:
		bpp = 2;
		break;
	default:
		bpp = 1;
		break;
	}
	f->bytesperline = f->width * bpp;
	f->sizeimage = f->bytesperline * f->height;
	f->sizeimage = ALIGN(f->sizeimage, 64);
}
static int s31_querycap(struct file *file, void *priv,
			struct v4l2_capability *cap)
{
	struct s31_media *d = s31_ctx(file_to_v4l2_fh(file))->dev;

	strscpy(cap->driver, "esp32s31-media", sizeof(cap->driver));
	strscpy(cap->card, d->jpeg ? "S31 JPEG" : "S31 PPA", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(d->dev));
	return 0;
}

int s31_enum_format(struct s31_media *d, struct v4l2_fmtdesc *f)
{
	const u32 *list = d->jpeg ? s31_jpeg_formats : s31_ppa_formats;
	unsigned int count = d->jpeg ? ARRAY_SIZE(s31_jpeg_formats) :
				       ARRAY_SIZE(s31_ppa_formats);
	if (f->index >= count)
		return -EINVAL;
	f->pixelformat = list[f->index];
	if (f->pixelformat == V4L2_PIX_FMT_JPEG)
		f->flags = V4L2_FMT_FLAG_COMPRESSED;
	return 0;
}

static int s31_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	return s31_enum_format(s31_ctx(file_to_v4l2_fh(file))->dev, f);
}
static int s31_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	s31_normalize_format(s31_ctx(file_to_v4l2_fh(file))->dev, &f->fmt.pix);
	return 0;
}
static int s31_get_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	f->fmt.pix =
		s31_ctx(file_to_v4l2_fh(file))->fmt[s31_queue_index(f->type)];
	return 0;
}
static int s31_set_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct s31_media_ctx *ctx = s31_ctx(file_to_v4l2_fh(file));
	struct vb2_queue *q = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);

	if (vb2_is_busy(q))
		return -EBUSY;
	s31_normalize_format(ctx->dev, &f->fmt.pix);
	ctx->fmt[s31_queue_index(f->type)] = f->fmt.pix;
	ctx->rect[s31_queue_index(f->type)] = (struct v4l2_rect){
		.width = f->fmt.pix.width, .height = f->fmt.pix.height
	};
	return 0;
}
/* Selection is configured before allocation, so no queued job observes a
 * partially changed source/destination rectangle. GE/LE constrain the entire
 * rectangle, including position, as required by the selection API.
 */
int s31_select_axis(s32 *position, u32 *size, u32 limit,
		    u32 alignment, u32 flags)
{
	s64 left = *position, right = left + *size;
	s64 lo, hi, pos;
	u32 length;

	if (flags & V4L2_SEL_FLAG_GE) {
		if (left < 0 || right > limit || !*size)
			return -ERANGE;
		pos = round_down((u32)left, alignment);
		length = ALIGN(right - pos, 16);
		if (length > limit)
			return -ERANGE;
		pos = min_t(s64, pos, limit - length);
	} else if (flags & V4L2_SEL_FLAG_LE) {
		lo = ALIGN(clamp_t(s64, left, 0, limit), alignment);
		hi = clamp_t(s64, right, 0, limit);
		if (hi - lo < 16)
			return -ERANGE;
		length = round_down((u32)(hi - lo), 16);
		pos = clamp_t(s64, left + ((s64)*size - length) / 2,
			      lo, hi - length);
		pos = round_down((u32)pos, alignment);
	} else {
		length = clamp_t(u32, *size, 16, limit);
		length = min_t(u32, round_down(length + 8, 16), limit);
		pos = clamp_t(s64, left + ((s64)*size - length) / 2,
			      0, limit - length);
		pos = round_down((u32)pos, alignment);
	}
	if ((flags & V4L2_SEL_FLAG_LE) &&
	    (pos < left || pos + length > right))
		return -ERANGE;
	*position = pos;
	*size = length;
	return 0;
}

static int s31_get_selection(struct file *file, void *priv,
			     struct v4l2_selection *s)
{
	struct s31_media_ctx *ctx = s31_ctx(file_to_v4l2_fh(file));
	int idx;

	if (ctx->dev->jpeg)
		return -ENODATA;
	if (s->type == V4L2_BUF_TYPE_VIDEO_OUTPUT &&
	    (s->target == V4L2_SEL_TGT_CROP ||
	     s->target == V4L2_SEL_TGT_CROP_DEFAULT ||
	     s->target == V4L2_SEL_TGT_CROP_BOUNDS))
		idx = 0;
	else if (s->type == V4L2_BUF_TYPE_VIDEO_CAPTURE &&
		 (s->target == V4L2_SEL_TGT_COMPOSE ||
		  s->target == V4L2_SEL_TGT_COMPOSE_DEFAULT ||
		  s->target == V4L2_SEL_TGT_COMPOSE_BOUNDS))
		idx = 1;
	else
		return -EINVAL;
	if (s->target == V4L2_SEL_TGT_CROP ||
	    s->target == V4L2_SEL_TGT_COMPOSE)
		s->r = ctx->rect[idx];
	else
		s->r = (struct v4l2_rect){ .width = ctx->fmt[idx].width,
					 .height = ctx->fmt[idx].height };
	return 0;
}

static int s31_set_selection(struct file *file, void *priv,
			     struct v4l2_selection *s)
{
	struct s31_media_ctx *ctx = s31_ctx(file_to_v4l2_fh(file));
	struct v4l2_rect rect = s->r;
	u32 alignment;
	int idx, ret;

	if (ctx->dev->jpeg)
		return -ENODATA;
	if (s->flags & ~(V4L2_SEL_FLAG_GE | V4L2_SEL_FLAG_LE))
		return -EINVAL;
	if (s->type == V4L2_BUF_TYPE_VIDEO_OUTPUT &&
	    s->target == V4L2_SEL_TGT_CROP)
		idx = 0;
	else if (s->type == V4L2_BUF_TYPE_VIDEO_CAPTURE &&
		 s->target == V4L2_SEL_TGT_COMPOSE)
		idx = 1;
	else
		return -EINVAL;
	if (vb2_is_busy(v4l2_m2m_get_vq(ctx->fh.m2m_ctx, s->type)))
		return -EBUSY;
	/* The receive port writes at halfword granularity, including 8/24-bit
	 * formats. Keep a partial destination from touching its left neighbor.
	 * All negotiated strides and rectangle widths are multiples of 16.
	 */
	alignment = ctx->fmt[idx].pixelformat == V4L2_PIX_FMT_UYVY ||
		    (idx && (ctx->fmt[idx].pixelformat == V4L2_PIX_FMT_GREY ||
			     ctx->fmt[idx].pixelformat == V4L2_PIX_FMT_RGB24 ||
			     ctx->fmt[idx].pixelformat == V4L2_PIX_FMT_BGR24)) ?
			    2 : 1;
	ret = s31_select_axis(&rect.left, &rect.width, ctx->fmt[idx].width,
			      alignment, s->flags);
	if (!ret)
		ret = s31_select_axis(&rect.top, &rect.height,
				      ctx->fmt[idx].height, 1, s->flags);
	if (ret)
		return ret;
	ctx->rect[idx] = rect;
	s->r = rect;
	return 0;
}

static int s31_queue_setup(struct vb2_queue *q, unsigned int *count,
			   unsigned int *planes, unsigned int sizes[],
			   struct device *alloc_devs[])
{
	struct s31_media_ctx *ctx = vb2_get_drv_priv(q);
	const struct v4l2_pix_format *f = &ctx->fmt[s31_queue_index(q->type)];
	/* One pair is enough for streaming. Limit requested queue depth on 16 MiB. */
	if (vb2_get_num_buffers(q) >= 2)
		return -ENOSPC;
	*count = min_t(unsigned int, *count, 2 - vb2_get_num_buffers(q));
	if (*planes)
		return *planes == 1 && sizes[0] >= f->sizeimage ? 0 : -EINVAL;
	*planes = 1;
	sizes[0] = f->sizeimage;
	return 0;
}
static int s31_buf_prepare(struct vb2_buffer *vb)
{
	struct s31_media_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	int idx = s31_queue_index(vb->type);

	if (vb2_plane_size(vb, 0) < ctx->fmt[idx].sizeimage)
		return -EINVAL;
	if (!idx && ctx->fmt[0].pixelformat != V4L2_PIX_FMT_JPEG)
		vb2_set_plane_payload(vb, 0, ctx->fmt[0].sizeimage);
	if (!idx && (!vb2_get_plane_payload(vb, 0) ||
		     vb2_get_plane_payload(vb, 0) > vb2_plane_size(vb, 0)))
		return -EINVAL;
	return 0;
}
static void s31_buf_queue(struct vb2_buffer *vb)
{
	struct s31_media_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}
static int s31_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct s31_media_ctx *ctx = vb2_get_drv_priv(q);

	ctx->sequence[s31_queue_index(q->type)] = 0;
	ctx->aborting = false;
	return 0;
}
static void s31_stop_streaming(struct vb2_queue *q)
{
	struct s31_media_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *b;

	for (;;) {
		b = V4L2_TYPE_IS_OUTPUT(q->type) ?
			    v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
			    v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!b)
			break;
		v4l2_m2m_buf_done(b, VB2_BUF_STATE_ERROR);
	}
}
static const struct vb2_ops s31_queue_ops = {
	.queue_setup = s31_queue_setup,
	.buf_prepare = s31_buf_prepare,
	.buf_queue = s31_buf_queue,
	.start_streaming = s31_start_streaming,
	.stop_streaming = s31_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};
static int s31_queue_init(void *priv, struct vb2_queue *src,
			  struct vb2_queue *dst)
{
	struct s31_media_ctx *ctx = priv;
	struct vb2_queue *qs[] = { src, dst };
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(qs); i++) {
		struct vb2_queue *q = qs[i];

		q->type = i ? V4L2_BUF_TYPE_VIDEO_CAPTURE :
			      V4L2_BUF_TYPE_VIDEO_OUTPUT;
		q->io_modes = VB2_MMAP | VB2_DMABUF;
		q->allow_cache_hints = 1;
		/* Only the encoder destination has a CPU-written interchange header.
		 * Input is read-only to the device and must remain DMA_TO_DEVICE.
		 */
		q->bidirectional = i && ctx->dev->jpeg;
		q->drv_priv = ctx;
		q->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
		q->ops = &s31_queue_ops;
		q->mem_ops = &vb2_dma_contig_memops;
		q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
		q->lock = &ctx->dev->lock;
		q->dev = s31_dma2d_device(ctx->dev->dma);
		/* The M2M core requires one ready buffer on each queue. Leaving
		 * min_queued_buffers zero permits an actual single-buffer allocation.
		 */
		ret = vb2_queue_init(q);
		if (ret)
			return ret;
	}
	return 0;
}
static void s31_job_work(struct work_struct *work)
{
	struct s31_media_ctx *ctx =
		container_of(work, struct s31_media_ctx, work);
	struct s31_media *d = ctx->dev;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	size_t bytes = 0;
	enum vb2_buffer_state state = VB2_BUF_STATE_ERROR;
	int ret = -ECANCELED;

	mutex_lock(&d->hardware_lock);
	if (!d->removed && !READ_ONCE(ctx->aborting)) {
		/* Separate queue slots may import the same backing allocation.
		 * Neither codec nor SRM supports overlapping source/output DMA.
		 */
		if (s31_media_buffers_overlap(src, ctx->fmt[0].sizeimage,
					      dst, ctx->fmt[1].sizeimage)) {
			ret = -EINVAL;
			goto unlock;
		}
		ret = pm_runtime_resume_and_get(d->dev);
		if (ret >= 0) {
			ret = d->jpeg ? s31_jpeg_run(ctx, src, dst, &bytes) :
					s31_ppa_run(ctx, src, dst, &bytes);
			pm_runtime_mark_last_busy(d->dev);
			pm_runtime_put_autosuspend(d->dev);
		}
	}
unlock:
	mutex_unlock(&d->hardware_lock);
	if (!ret && !READ_ONCE(ctx->aborting))
		state = VB2_BUF_STATE_DONE;
	v4l2_m2m_buf_copy_metadata(src, dst, true);
	src->sequence = ctx->sequence[0]++;
	dst->sequence = ctx->sequence[1]++;
	vb2_set_plane_payload(&dst->vb2_buf, 0,
			      state == VB2_BUF_STATE_DONE ? bytes : 0);
	dst->field = V4L2_FIELD_NONE;
	v4l2_m2m_buf_done_and_job_finish(d->m2m, ctx->fh.m2m_ctx, state);
}
static void s31_device_run(void *priv)
{
	struct s31_media_ctx *ctx = priv;

	queue_work(system_freezable_wq, &ctx->work);
}
static void s31_job_abort(void *priv)
{
	struct s31_media_ctx *ctx = priv;

	WRITE_ONCE(ctx->aborting, true);
}
static const struct v4l2_m2m_ops s31_m2m_ops = {
	.device_run = s31_device_run,
	.job_abort = s31_job_abort,
};
static void s31_media_free(struct kref *ref)
{
	struct s31_media *d = container_of(ref, struct s31_media, refs);

	v4l2_m2m_release(d->m2m);
	v4l2_device_unregister(&d->v4l2);
	kfree(d);
}

void s31_media_get(struct s31_media *d)
{
	kref_get(&d->refs);
}

void s31_media_put(struct s31_media *d)
{
	kref_put(&d->refs, s31_media_free);
}

static void s31_video_release(struct video_device *video)
{
	struct s31_media *d = video_get_drvdata(video);

	kref_put(&d->refs, s31_media_free);
}
static int s31_open(struct file *file)
{
	struct s31_media *d = video_drvdata(file);
	struct s31_media_ctx *ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	int ret;

	if (!ctx)
		return -ENOMEM;
	ctx->dev = d;
	INIT_WORK(&ctx->work, s31_job_work);
	v4l2_fh_init(&ctx->fh, &d->video);
	file->private_data = &ctx->fh;
	v4l2_ctrl_handler_init(&ctx->controls, d->jpeg ? 1 : 6);
	if (d->jpeg)
		ctx->quality = v4l2_ctrl_new_std(
			&ctx->controls, NULL, V4L2_CID_JPEG_COMPRESSION_QUALITY,
			1, 100, 1, 75);
	else {
		ctx->rotate = v4l2_ctrl_new_std(&ctx->controls, NULL,
						V4L2_CID_ROTATE, 0, 270, 90, 0);
		ctx->hflip = v4l2_ctrl_new_std(&ctx->controls, NULL,
					       V4L2_CID_HFLIP, 0, 1, 1, 0);
		ctx->vflip = v4l2_ctrl_new_std(&ctx->controls, NULL,
					       V4L2_CID_VFLIP, 0, 1, 1, 0);
		ctx->alpha = v4l2_ctrl_new_std(&ctx->controls, NULL,
				V4L2_CID_ALPHA_COMPONENT, 0, 255, 1, 255);
		ctx->colorfx = v4l2_ctrl_new_std_menu(&ctx->controls, NULL,
						      V4L2_CID_COLORFX,
						      V4L2_COLORFX_SET_RGB,
						      GENMASK_ULL(V4L2_COLORFX_SET_RGB - 1, 1),
						      V4L2_COLORFX_NONE);
		ctx->fill_rgb = v4l2_ctrl_new_std(&ctx->controls, NULL,
						  V4L2_CID_COLORFX_RGB, 0, 0xffffff, 1, 0);
	}
	ret = ctx->controls.error;
	if (ret)
		goto free;
	ctx->fh.ctrl_handler = &ctx->controls;
	ctx->fmt[0] = (struct v4l2_pix_format){
		.width = 320,
		.height = 240,
		.pixelformat = d->jpeg ? V4L2_PIX_FMT_GREY : V4L2_PIX_FMT_RGB24
	};
	ctx->fmt[1] = ctx->fmt[0];
	if (d->jpeg) {
		ctx->fmt[1].pixelformat = V4L2_PIX_FMT_JPEG;
		ctx->fmt[1].sizeimage = 128 * 1024;
	}
	s31_normalize_format(ctx->dev, &ctx->fmt[0]);
	s31_normalize_format(ctx->dev, &ctx->fmt[1]);
	ctx->rect[0] = (struct v4l2_rect){ .width = ctx->fmt[0].width,
					.height = ctx->fmt[0].height };
	ctx->rect[1] = ctx->rect[0];
	mutex_lock(&d->lock);
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(d->m2m, ctx, s31_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		mutex_unlock(&d->lock);
		goto free;
	}
	kref_get(&d->refs);
	v4l2_fh_add(&ctx->fh, file);
	mutex_unlock(&d->lock);
	return 0;
free:
	v4l2_ctrl_handler_free(&ctx->controls);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}
static int s31_release(struct file *file)
{
	struct s31_media_ctx *ctx = s31_ctx(file->private_data);

	mutex_lock(&ctx->dev->lock);
	/* Stop new jobs and join the last completion before the M2M queues and
	 * context are freed. job_finish can wake cancellation before returning.
	 */
	v4l2_m2m_streamoff(file, ctx->fh.m2m_ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT);
	v4l2_m2m_streamoff(file, ctx->fh.m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE);
	flush_work(&ctx->work);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	mutex_unlock(&ctx->dev->lock);
	v4l2_ctrl_handler_free(&ctx->controls);
	kref_put(&ctx->dev->refs, s31_media_free);
	kfree(ctx);
	return 0;
}
static const struct v4l2_file_operations s31_fops = {
	.owner = THIS_MODULE,
	.open = s31_open,
	.release = s31_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};
/* S31 RV32 has address-based cache attributes, not PTE cacheability bits.
 * Keep MMAP buffers cached and use vb2's DMA ownership synchronization.
 */
static int s31_reqbufs(struct file *file, void *priv,
		       struct v4l2_requestbuffers *r)
{
	if (r->memory == V4L2_MEMORY_MMAP)
		r->flags |= V4L2_MEMORY_FLAG_NON_COHERENT;
	return v4l2_m2m_ioctl_reqbufs(file, priv, r);
}
static int s31_create_bufs(struct file *file, void *priv,
			   struct v4l2_create_buffers *r)
{
	if (r->memory == V4L2_MEMORY_MMAP)
		r->flags |= V4L2_MEMORY_FLAG_NON_COHERENT;
	return v4l2_m2m_ioctl_create_bufs(file, priv, r);
}
static const struct v4l2_ioctl_ops s31_ioctl_ops = {
	.vidioc_querycap = s31_querycap,
	.vidioc_enum_fmt_vid_cap = s31_enum_fmt,
	.vidioc_enum_fmt_vid_out = s31_enum_fmt,
	.vidioc_g_fmt_vid_cap = s31_get_fmt,
	.vidioc_g_fmt_vid_out = s31_get_fmt,
	.vidioc_try_fmt_vid_cap = s31_try_fmt,
	.vidioc_try_fmt_vid_out = s31_try_fmt,
	.vidioc_s_fmt_vid_cap = s31_set_fmt,
	.vidioc_s_fmt_vid_out = s31_set_fmt,
	.vidioc_g_selection = s31_get_selection,
	.vidioc_s_selection = s31_set_selection,
	.vidioc_reqbufs = s31_reqbufs,
	.vidioc_create_bufs = s31_create_bufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
};
static int s31_media_resume(struct device *dev)
{
	struct s31_media *d = dev_get_drvdata(dev);
	int ret = clk_prepare_enable(d->clk);

	if (ret)
		return ret;
	ret = reset_control_reset(d->reset);
	if (ret) {
		clk_disable_unprepare(d->clk);
		return ret;
	}
	writel((readl(d->memory) & ~BIT(2)) | BIT(3), d->memory);
	return 0;
}
static int s31_media_suspend(struct device *dev)
{
	struct s31_media *d = dev_get_drvdata(dev);

	writel(readl(d->memory) | BIT(2) | BIT(3), d->memory);
	clk_disable_unprepare(d->clk);
	return 0;
}
static const struct dev_pm_ops s31_media_pm = {
	SET_RUNTIME_PM_OPS(s31_media_suspend, s31_media_resume, NULL)
		SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
					pm_runtime_force_resume)
};
static int s31_media_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s31_media *d;
	int ret;

	d = kzalloc(sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	kref_init(&d->refs);
	d->dev = dev;
	d->jpeg = of_device_is_compatible(dev->of_node,
					  "espressif,esp32s31-jpeg");
	d->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(d->regs)) {
		ret = PTR_ERR(d->regs);
		goto free;
	}
	d->memory = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(d->memory)) {
		ret = PTR_ERR(d->memory);
		goto free;
	}
	d->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(d->clk)) {
		ret = PTR_ERR(d->clk);
		goto free;
	}
	d->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(d->reset)) {
		ret = PTR_ERR(d->reset);
		goto free;
	}
	d->dma = s31_dma2d_get(dev);
	if (IS_ERR(d->dma)) {
		ret = PTR_ERR(d->dma);
		goto free;
	}
	mutex_init(&d->lock);
	mutex_init(&d->hardware_lock);
	platform_set_drvdata(pdev, d);
	ret = v4l2_device_register(dev, &d->v4l2);
	if (ret)
		goto free;
	d->m2m = v4l2_m2m_init(&s31_m2m_ops);
	if (IS_ERR(d->m2m)) {
		ret = PTR_ERR(d->m2m);
		goto v4l2;
	}
	strscpy(d->video.name, d->jpeg ? "s31-jpeg" : "s31-ppa",
		sizeof(d->video.name));
	d->video.v4l2_dev = &d->v4l2;
	d->video.fops = &s31_fops;
	d->video.ioctl_ops = &s31_ioctl_ops;
	d->video.release = s31_video_release;
	d->video.lock = &d->lock;
	d->video.vfl_dir = VFL_DIR_M2M;
	d->video.device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	video_set_drvdata(&d->video, d);
	pm_runtime_set_autosuspend_delay(dev, 100);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);
	ret = video_register_device(&d->video, VFL_TYPE_VIDEO, -1);
	if (!ret) {
		kref_get(&d->refs);
		if (!d->jpeg) {
			ret = s31_fill_register(d);
			if (!ret)
				ret = s31_blend_register(d);
			if (ret) {
				s31_fill_unregister(d);
				video_unregister_device(&d->video);
				pm_runtime_disable(dev);
				s31_media_put(d);
				return ret;
			}
		}
		dev_info(dev, "%s registered as /dev/video%d\n", d->video.name,
			 d->video.num);
		return 0;
	}
	pm_runtime_disable(dev);
	kref_put(&d->refs, s31_media_free);
	return ret;
	/* Before video registration, only the platform reference exists. */

v4l2:
	v4l2_device_unregister(&d->v4l2);
free:
	kfree(d);
	return dev_err_probe(dev, ret, "media registration failed\n");
}
static void s31_media_remove(struct platform_device *pdev)
{
	struct s31_media *d = platform_get_drvdata(pdev);

	mutex_lock(&d->hardware_lock);
	d->removed = true;
	mutex_unlock(&d->hardware_lock);
	s31_fill_unregister(d);
	s31_blend_unregister(d);
	video_unregister_device(&d->video);
	v4l2_device_disconnect(&d->v4l2);
	pm_runtime_disable(d->dev);
	if (!pm_runtime_status_suspended(d->dev))
		s31_media_suspend(d->dev);
	kref_put(&d->refs, s31_media_free);
}
static const struct of_device_id s31_media_match[] = {
	{ .compatible = "espressif,esp32s31-jpeg" },
	{ .compatible = "espressif,esp32s31-ppa" },
	{}
};
MODULE_DEVICE_TABLE(of, s31_media_match);
static struct platform_driver s31_media_driver = {
	.probe = s31_media_probe,
	.remove = s31_media_remove,
	.driver = { .name = "esp32s31-media",
		    .of_match_table = s31_media_match,
		    .pm = &s31_media_pm,
		    .suppress_bind_attrs = true },
};
module_platform_driver(s31_media_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ESP32-S31 compact JPEG and PPA V4L2 memory engines");
