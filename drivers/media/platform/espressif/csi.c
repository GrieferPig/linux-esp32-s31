// SPDX-License-Identifier: GPL-2.0-only
/* P4 rev103 OV5647 CSI/ISP capture. Fixed 800x640 RGB565, IRQ-driven VB2.
 * Two reserved 1MiB slots plus an independent discard slot prevent DMA from
 * overwriting dequeued buffers. Read-only NOMMU mmap and blocking read/DQBUF.
 * Sensor settings: Espressif esp-video-components 48c1892af113 (Apache-2.0).
 */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/of_address.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/refcount.h>
#include <linux/slab.h>
#include <linux/soc/espressif/esp32p4.h>
#include <linux/soc/espressif/esp32p4.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-memops.h>
#include <media/videobuf2-v4l2.h>

#define WIDTH 800
#define HEIGHT 640
#define BYTES (WIDTH * HEIGHT * 2)
#define BASE 0x49300000U
#define SLOT 0x100000U
#define DISCARD 2
struct p4_sensor_reg { u16 reg; u8 value; };
#include "ov5647_800x640.h"
struct p4_cam;
struct p4_mem {
    struct p4_cam *cam;
    unsigned int slot;
    refcount_t refs;
    struct vb2_vmarea_handler handler;
};
struct p4_buf {
    struct vb2_v4l2_buffer vb;
    struct list_head list;
};
struct p4_cam {
    struct v4l2_device v4l2;
    struct video_device video;
    struct vb2_queue queue;
    struct i2c_client *client;
    struct mutex mutex;
    spinlock_t lock;
    struct list_head pending;
    struct p4_buf *active;
    unsigned long allocated;
    u32 sequence, dropped, errors, frames;
    bool running, disconnected;
    int irq;
    int (*service)(u32, u32);
};
static inline struct p4_buf *to_buf(struct vb2_buffer *vb)
{ return container_of(to_vb2_v4l2_buffer(vb), struct p4_buf, vb); }
static u32 buffer_address(struct p4_buf *buf)
{
    struct p4_mem *m = buf->vb.vb2_buf.planes[0].mem_priv;
    return BASE + m->slot * SLOT;
}
static int sensor_write(struct p4_cam *cam, u16 reg, u8 val)
{
    u8 data[] = {reg >> 8, reg, val};
    int ret = i2c_master_send(cam->client, data, sizeof(data));
    return ret == sizeof(data) ? 0 : ret < 0 ? ret : -EIO;
}
static int sensor_read(struct p4_cam *cam, u16 reg)
{
    u8 addr[] = {reg >> 8, reg}, value;
    struct i2c_msg msgs[] = {
        {.addr=cam->client->addr, .len=2, .buf=addr},
        {.addr=cam->client->addr, .flags=I2C_M_RD, .len=1, .buf=&value},
    };
    int ret = i2c_transfer(cam->client->adapter, msgs, 2);
    return ret == 2 ? value : ret < 0 ? ret : -EIO;
}
static int sensor_configure(struct p4_cam *cam)
{
    unsigned int i;
    int ret;
    ret = sensor_write(cam, 0x0100, 0);
    if (ret) return ret;
    ret = sensor_write(cam, 0x0103, 1);
    if (ret) return ret;
    msleep(10);
    ret = sensor_write(cam, 0x4800, 1); /* clock lane LP11 while stopped */
    if (ret) return ret;
    for (i=0; i<ARRAY_SIZE(ov5647_mipi_2lane_24Minput_800x640_raw8_50fps)-1; i++) {
        const struct p4_sensor_reg *r=&ov5647_mipi_2lane_24Minput_800x640_raw8_50fps[i];
        ret = sensor_write(cam, r->reg, r->value);
        if (ret) return ret;
    }
    /* Enable the sensor's exposure/gain and white balance, retain RAW output.
     * No host timer or I2C polling loop is needed for exposure control. */
    ret = sensor_write(cam, 0x3503, 0);
    if (ret) return ret;
    ret = sensor_write(cam, 0x5001, 1);
    if (ret) return ret;
    return sensor_write(cam, 0x4800, 0x25);
}
static void mem_put(void *priv)
{
    struct p4_mem *m=priv;
    if (refcount_dec_and_test(&m->refs)) {
        clear_bit(m->slot, &m->cam->allocated);
        kfree(m);
    }
}
static void *mem_alloc(struct vb2_buffer *vb, struct device *dev, unsigned long size)
{
    struct p4_cam *cam=vb2_get_drv_priv(vb->vb2_queue);
    struct p4_mem *m;
    unsigned int slot;
    if (size != BYTES) return ERR_PTR(-EINVAL);
    m=kzalloc(sizeof(*m), GFP_KERNEL);
    if (!m) return ERR_PTR(-ENOMEM);
    for (slot=0; slot<2; slot++)
        if (!test_and_set_bit(slot, &cam->allocated)) break;
    if (slot==2) { kfree(m); return ERR_PTR(-ENOMEM); }
    m->cam=cam; m->slot=slot;
    refcount_set(&m->refs, 1);
    m->handler.refcount=&m->refs;
    m->handler.put=mem_put; m->handler.arg=m;
    return m;
}
static void *mem_vaddr(struct vb2_buffer *vb, void *priv)
{
    struct p4_mem *m=priv;
    return (void *)(unsigned long)(BASE+m->slot*SLOT);
}
static unsigned int mem_users(void *priv)
{ return refcount_read(&((struct p4_mem *)priv)->refs); }
static int mem_mmap(void *priv, struct vm_area_struct *vma)
{
    struct p4_mem *m=priv;
    if ((vma->vm_flags & (VM_WRITE|VM_EXEC)) || vma->vm_pgoff ||
        !(vma->vm_flags & VM_SHARED) || vma->vm_end-vma->vm_start != BYTES ||
        vma->vm_start != BASE+m->slot*SLOT)
        return -EINVAL;
    vm_flags_clear(vma, VM_MAYWRITE|VM_MAYEXEC);
    vm_flags_set(vma, VM_DONTEXPAND|VM_DONTDUMP);
    vma->vm_ops=&vb2_common_vm_ops;
    vma->vm_private_data=&m->handler;
    vma->vm_ops->open(vma);
    return 0;
}
static const struct vb2_mem_ops memory_ops = {
    .alloc=mem_alloc, .put=mem_put, .vaddr=mem_vaddr,
    .num_users=mem_users, .mmap=mem_mmap,
};
static int setup(struct vb2_queue *q, unsigned int *nb, unsigned int *np,
                 unsigned int sizes[], struct device *devs[])
{
    if (*np) return *np==1 && sizes[0]>=BYTES ? 0 : -EINVAL;
    *nb=clamp_t(unsigned int, *nb, 1, 2);
    *np=1; sizes[0]=BYTES;
    return 0;
}
static int prepare(struct vb2_buffer *vb)
{
    int ret;
    if (vb2_plane_size(vb, 0)<BYTES) return -EINVAL;
    ret=esp32p4_camera_cache_invalidate(buffer_address(to_buf(vb)), BYTES);
    if (ret) return ret;
    vb2_set_plane_payload(vb, 0, BYTES);
    return 0;
}
static void finish(struct vb2_buffer *vb)
{
    if (esp32p4_camera_cache_invalidate(buffer_address(to_buf(vb)), BYTES)) {
        vb2_set_plane_payload(vb, 0, 0);
        to_vb2_v4l2_buffer(vb)->flags |= V4L2_BUF_FLAG_ERROR;
    }
}
static void enqueue(struct vb2_buffer *vb)
{
    struct p4_cam *cam=vb2_get_drv_priv(vb->vb2_queue);
    unsigned long flags;
    spin_lock_irqsave(&cam->lock, flags);
    list_add_tail(&to_buf(vb)->list, &cam->pending);
    spin_unlock_irqrestore(&cam->lock, flags);
}
static unsigned int next_slot(struct p4_cam *cam)
{
    struct p4_mem *m;
    cam->active=NULL;
    if (list_empty(&cam->pending)) return DISCARD;
    cam->active=list_first_entry(&cam->pending, struct p4_buf, list);
    list_del(&cam->active->list);
    m=cam->active->vb.vb2_buf.planes[0].mem_priv;
    return m->slot;
}
static void return_buffers(struct p4_cam *cam, enum vb2_buffer_state state)
{
    struct p4_buf *buf, *tmp;
    if (cam->active) {
        vb2_buffer_done(&cam->active->vb.vb2_buf, state);
        cam->active=NULL;
    }
    list_for_each_entry_safe(buf, tmp, &cam->pending, list) {
        list_del(&buf->list);
        vb2_buffer_done(&buf->vb.vb2_buf, state);
    }
}
static int start(struct vb2_queue *q, unsigned int count)
{
    struct p4_cam *cam=vb2_get_drv_priv(q);
    unsigned long flags;
    int ret;
    if (cam->disconnected) {
        spin_lock_irqsave(&cam->lock, flags);
        return_buffers(cam, VB2_BUF_STATE_QUEUED);
        spin_unlock_irqrestore(&cam->lock, flags);
        return -ENODEV;
    }
    ret=sensor_configure(cam);
    if (ret) goto fail;
    spin_lock_irqsave(&cam->lock, flags);
    cam->sequence=0;
    ret=cam->service(0, next_slot(cam));
    if (!ret) cam->running=true;
    spin_unlock_irqrestore(&cam->lock, flags);
    if (ret) goto fail;
    ret=sensor_write(cam, 0x4800, 0x24);
    if (!ret) ret=sensor_write(cam, 0x0100, 1);
    if (!ret) return 0;
fail:
    spin_lock_irqsave(&cam->lock, flags);
    cam->running=false;
    cam->service(2, 0);
    return_buffers(cam, VB2_BUF_STATE_QUEUED);
    spin_unlock_irqrestore(&cam->lock, flags);
    sensor_write(cam, 0x0100, 0);
    return ret;
}
static void stop(struct vb2_queue *q)
{
    struct p4_cam *cam=vb2_get_drv_priv(q);
    unsigned long flags;
    if (!cam->disconnected) {
        sensor_write(cam, 0x0100, 0);
        sensor_write(cam, 0x4800, 0x25);
    }
    spin_lock_irqsave(&cam->lock, flags);
    cam->running=false;
    cam->service(2, 0);
    return_buffers(cam, VB2_BUF_STATE_ERROR);
    spin_unlock_irqrestore(&cam->lock, flags);
    synchronize_irq(cam->irq);
}
static const struct vb2_ops queue_ops = {
    .queue_setup=setup, .buf_prepare=prepare, .buf_finish=finish,
    .buf_queue=enqueue, .start_streaming=start, .stop_streaming=stop,
    .wait_prepare=vb2_ops_wait_prepare, .wait_finish=vb2_ops_wait_finish,
};
static irqreturn_t camera_irq(int irq, void *data)
{
    struct p4_cam *cam=data;
    struct p4_buf *done;
    unsigned long flags;
    int ret;
    spin_lock_irqsave(&cam->lock, flags);
    if (!cam->running) { spin_unlock_irqrestore(&cam->lock, flags); return IRQ_NONE; }
    ret=cam->service(1, 0);
    if (!ret) { spin_unlock_irqrestore(&cam->lock, flags); return IRQ_NONE; }
    done=cam->active;
    cam->active=NULL;
    if (ret<0) goto error;
    cam->frames++;
    if (!done) cam->dropped++;
    else {
        done->vb.sequence=cam->sequence;
        done->vb.field=V4L2_FIELD_NONE;
        done->vb.vb2_buf.timestamp=ktime_get_ns();
    }
    cam->sequence++;
    ret=cam->service(3, next_slot(cam));
    if (ret<0) goto error;
    if (done) vb2_buffer_done(&done->vb.vb2_buf, VB2_BUF_STATE_DONE);
    spin_unlock_irqrestore(&cam->lock, flags);
    return IRQ_HANDLED;
error:
    cam->errors++;
    cam->running=false;
    cam->service(2, 0);
    if (done) vb2_buffer_done(&done->vb.vb2_buf, VB2_BUF_STATE_ERROR);
    return_buffers(cam, VB2_BUF_STATE_ERROR);
    vb2_queue_error(&cam->queue);
    spin_unlock_irqrestore(&cam->lock, flags);
    return IRQ_HANDLED;
}
static int querycap(struct file *file, void *fh, struct v4l2_capability *c)
{
    strscpy(c->driver, "espressif-csi", sizeof(c->driver));
    strscpy(c->card, "P4 OV5647 CSI ISP", sizeof(c->card));
    strscpy(c->bus_info, "platform:espressif-csi", sizeof(c->bus_info));
    return 0;
}
static int enumfmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
    if (f->index) return -EINVAL;
    f->pixelformat=V4L2_PIX_FMT_RGB565;
    return 0;
}
static int format(struct file *file, void *fh, struct v4l2_format *f)
{
    f->fmt.pix=(struct v4l2_pix_format){
        .width=WIDTH, .height=HEIGHT, .pixelformat=V4L2_PIX_FMT_RGB565,
        .field=V4L2_FIELD_NONE, .bytesperline=WIDTH*2, .sizeimage=BYTES,
        .colorspace=V4L2_COLORSPACE_SRGB,
    };
    return 0;
}
static int setfmt(struct file *file, void *fh, struct v4l2_format *f)
{
    struct p4_cam *cam=video_drvdata(file);
    return vb2_is_busy(&cam->queue) ? -EBUSY : format(file, fh, f);
}
static int framesize(struct file *file, void *fh, struct v4l2_frmsizeenum *f)
{
    if (f->index || f->pixel_format!=V4L2_PIX_FMT_RGB565) return -EINVAL;
    f->type=V4L2_FRMSIZE_TYPE_DISCRETE;
    f->discrete.width=WIDTH; f->discrete.height=HEIGHT;
    return 0;
}
static int input_enum(struct file *file, void *fh, struct v4l2_input *in)
{
    if (in->index) return -EINVAL;
    in->type=V4L2_INPUT_TYPE_CAMERA;
    strscpy(in->name, "OV5647", sizeof(in->name)); return 0;
}
static int input_get(struct file *file, void *fh, unsigned int *i) { *i=0; return 0; }
static int input_set(struct file *file, void *fh, unsigned int i) { return i ? -EINVAL : 0; }
static const struct v4l2_ioctl_ops ioctl_ops = {
    .vidioc_querycap=querycap, .vidioc_enum_fmt_vid_cap=enumfmt,
    .vidioc_g_fmt_vid_cap=format, .vidioc_try_fmt_vid_cap=format,
    .vidioc_s_fmt_vid_cap=setfmt, .vidioc_enum_framesizes=framesize,
    .vidioc_enum_input=input_enum, .vidioc_g_input=input_get, .vidioc_s_input=input_set,
    .vidioc_reqbufs=vb2_ioctl_reqbufs, .vidioc_querybuf=vb2_ioctl_querybuf,
    .vidioc_qbuf=vb2_ioctl_qbuf, .vidioc_dqbuf=vb2_ioctl_dqbuf,
    .vidioc_streamon=vb2_ioctl_streamon, .vidioc_streamoff=vb2_ioctl_streamoff,
};
static const struct v4l2_file_operations file_ops = {
    .owner=THIS_MODULE, .open=v4l2_fh_open, .release=vb2_fop_release,
    .read=vb2_fop_read, .poll=vb2_fop_poll, .unlocked_ioctl=video_ioctl2,
    .mmap=vb2_fop_mmap, .get_unmapped_area=vb2_fop_get_unmapped_area,
};
static void release_video(struct video_device *vdev)
{
    struct p4_cam *cam=container_of(vdev, struct p4_cam, video);
    v4l2_device_unregister(&cam->v4l2);
    release_mem_region(BASE, SLOT*3);
    kfree(cam);
}
static int camera_probe(struct i2c_client *client)
{
    struct device_node *chosen, *pool;
    struct resource res;
    struct p4_cam *cam;
    u32 entry;
    int high, low, ret;
    cam=kzalloc(sizeof(*cam), GFP_KERNEL);
    if (!cam) return -ENOMEM;
    cam->client=client;
    mutex_init(&cam->mutex); spin_lock_init(&cam->lock); INIT_LIST_HEAD(&cam->pending);
    chosen=of_find_node_by_path("/chosen");
    ret=of_property_read_u32(chosen, "espressif,camera-service-entry", &entry);
    of_node_put(chosen);
    if (ret || !esp32p4_retained_entry_valid(entry)) { ret=-ENODEV; goto free; }
    cam->service=(void *)(unsigned long)entry;
    cam->irq=of_irq_get(client->dev.of_node, 0);
    if (cam->irq<0) { ret=cam->irq; goto free; }
    high=sensor_read(cam, 0x300a); low=sensor_read(cam, 0x300b);
    if (high<0 || low<0) { ret=high<0 ? high : low; goto free; }
    if ((high<<8 | low)!=0x5647) { ret=-ENODEV; goto free; }
    ret=sensor_write(cam, 0x0100, 0);
    if (ret) goto free;
    pool=of_find_node_by_path("/reserved-memory/camera-buffers@49300000");
    ret=pool ? of_address_to_resource(pool, 0, &res) : -EINVAL;
    of_node_put(pool);
    if (ret || res.start!=BASE || resource_size(&res)!=3*SLOT) { ret=-EINVAL; goto free; }
    if (!request_mem_region(BASE, SLOT*3, "espressif-csi")) { ret=-EBUSY; goto free; }
    ret=v4l2_device_register(&client->dev, &cam->v4l2);
    if (ret) goto release_region;
    cam->queue=(struct vb2_queue){
        .type=V4L2_BUF_TYPE_VIDEO_CAPTURE, .io_modes=VB2_MMAP|VB2_READ,
        .drv_priv=cam, .buf_struct_size=sizeof(struct p4_buf),
        .ops=&queue_ops, .mem_ops=&memory_ops, .lock=&cam->mutex,
        .timestamp_flags=V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC,
        .dev=&client->dev, .min_queued_buffers=1,
    };
    ret=vb2_queue_init(&cam->queue);
    if (ret) goto unregister;
    cam->video=(struct video_device){
        .fops=&file_ops, .ioctl_ops=&ioctl_ops, .v4l2_dev=&cam->v4l2,
        .queue=&cam->queue, .lock=&cam->mutex, .release=release_video,
        .device_caps=V4L2_CAP_VIDEO_CAPTURE|V4L2_CAP_STREAMING|V4L2_CAP_READWRITE,
    };
    strscpy(cam->video.name, "P4 OV5647", sizeof(cam->video.name));
    video_set_drvdata(&cam->video, cam);
    ret=request_irq(cam->irq, camera_irq, IRQF_SHARED, "espressif-csi", cam);
    if (ret) goto unregister;
    ret=video_register_device(&cam->video, VFL_TYPE_VIDEO, -1);
    if (ret) { free_irq(cam->irq, cam); goto unregister; }
    i2c_set_clientdata(client, cam);
    dev_info(&client->dev, "OV5647 ID5647: /dev/video%d 800x640 RGB565, DMA IRQ%d\n", cam->video.num, cam->irq);
    return 0;
unregister:
    v4l2_device_unregister(&cam->v4l2);
release_region:
    release_mem_region(BASE, SLOT*3);
free:
    kfree(cam); return dev_err_probe(&client->dev, ret, "camera probe failed\n");
}
static void camera_remove(struct i2c_client *client)
{
    struct p4_cam *cam=i2c_get_clientdata(client);
    mutex_lock(&cam->mutex);
    stop(&cam->queue);
    cam->disconnected=true;
    vb2_queue_error(&cam->queue);
    free_irq(cam->irq, cam);
    v4l2_device_disconnect(&cam->v4l2);
    mutex_unlock(&cam->mutex);
    video_unregister_device(&cam->video);
}
static const struct of_device_id match[] = {
    {.compatible="espressif,esp32p4-ov5647"}, {},
};
MODULE_DEVICE_TABLE(of, match);
static struct i2c_driver camera_driver = {
    .driver={.name="espressif-csi", .of_match_table=match},
    .probe=camera_probe, .remove=camera_remove,
};
module_i2c_driver(camera_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ESP32-P4 IRQ-driven OV5647 CSI ISP capture");
