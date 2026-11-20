// SPDX-License-Identifier: GPL-2.0-only
/* P4 rev103 OV5647 CSI/ISP capture. Fixed 800x640 RGB565, IRQ-driven VB2.
 * Two reserved 1MiB slots plus an independent discard slot prevent DMA from
 * overwriting dequeued buffers. Read-only NOMMU mmap and blocking read/DQBUF.
 * Sensor settings: Espressif esp-video-components 48c1892af113 (Apache-2.0).
 */
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/of_address.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/refcount.h>
#include <linux/slab.h>
#include <linux/soc/espressif/esp32p4.h>
#include <video/esp32p4-dsi/dw_gdma_struct.h>
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
    void __iomem *host, *bridge, *isp, *clkrst;
    dw_gdma_dev_t __iomem *dma;
    bool receiver_on;
};
/*
 * TRM ch.43 MIPI CSI - CSI_HOST_PHY_TEST_CTRL*, CSI_HOST_PHY_STOPSTATE.
 * TRM ch.39 ISP - HP_SYS_CLKRST_ISP_CLK_*, CSI_BRIG_DMA_BURST_LEN,
 * ISP_CNTL and ISP_FRAME_CFG. CSI_BRG is the rev1.x register prefix.
 * TRM ch.7 VDMA - DMAC_CHEN write enables and per-channel interrupt status.
 * See ESP-IDF mipi_csi_hal_init(), isp_ll_set_output_data_color_format()
 * and dw_gdma_channel_config_transfer() for the rev1.x programming sequence.
 */
#define P4_CSI_HOST_BASE         0x5009f000
#define P4_CSI_BRIDGE_BASE       0x5009f800
#define P4_CSI_ISP_BASE          0x500a1000
#define P4_CSI_CLKRST_BASE       0x500e6000
#define P4_CSI_VDMA_BASE         0x50081000
#define P4_CSI_FIFO              0x50104000

#define P4_CSI_BUS_CLOCKS        0x18
#define P4_CSI_DPHY_CLOCK        0x3c
#define P4_CSI_ISP_CLOCK         0xa8
#define P4_CSI_ISP_DIV           0xac
#define P4_CSI_RESETS            0xc0
#define P4_CSI_RESET_MASK        (BIT(27) | BIT(28) | BIT(29))

#define CSI_N_LANES              0x04
#define CSI_RESETN               0x08
#define CSI_PHY_SHUTDOWNZ        0x40
#define CSI_DPHY_RSTZ            0x44
#define CSI_PHY_STOPSTATE        0x4c
#define CSI_PHY_TEST_CTRL0       0x50
#define CSI_PHY_TEST_CTRL1       0x54
#define CSI_VC_EXTENSION         0xc8
#define CSI_SCRAMBLING           0x300
#define CSI_LP11                 (BIT(16) | BIT(1) | BIT(0))

#define CSI_BRG_ENABLE           0x04
#define CSI_BRG_DMA_REQ_CFG      0x08
#define CSI_BRG_BUF_FLOW_CTL     0x0c
#define CSI_BRG_DATA_TYPE_CFG    0x10
#define CSI_BRG_FRAME_CFG        0x14
#define CSI_BRG_ENDIAN_MODE      0x18
#define CSI_BRG_INT_CLR          0x20
#define CSI_BRG_INT_ENA          0x28
#define CSI_BRG_HOST_CTRL        0x40

#define ISP_CLK_EN               0x04
#define ISP_CNTL                 0x08
#define ISP_FRAME_CFG            0x10
#define ISP_INT_ENA              0x6c
#define ISP_INT_CLR              0x70
#define ISP_RGB565               (FIELD_PREP(GENMASK(31, 29), 4) | \
                                  BIT(13) | BIT(10) | BIT(6))
#define ISP_MIPI_DATA_EN         BIT(0)
#define ISP_ENABLE               BIT(1)

/* VDMA channel 0 is reserved for display. */
#define CSI_DMA_CH               1
#define CSI_DMA_CH_EN            BIT(CSI_DMA_CH)
#define CSI_DMA_CH_WE            BIT(CSI_DMA_CH + 8)
#define CSI_DMA_DONE             BIT(1)
#define CSI_DMA_ERRORS           GENMASK(13, 5)
#define CSI_DMA_EVENTS           (CSI_DMA_DONE | CSI_DMA_ERRORS)

static void csi_update(void __iomem *reg, u32 mask, u32 val)
{
    writel((readl(reg) & ~mask) | val, reg);
}

static void csi_dma_configure(struct p4_cam *cam)
{
    volatile dmac_channel_reg_t __iomem *ch = &cam->dma->ch[CSI_DMA_CH];
    dmac_chn_cfg1_reg_t cfg = { .tt_fc = 4, .src_per = 1, .ch_prior = 1,
                               .src_osr_lmt = 4, .dst_osr_lmt = 4 };
    dmac_chn_ctl0_reg_t ctl = { .dms = 1, .sinc = 1,
                               .src_tr_width = 3, .dst_tr_width = 3,
                               .src_msize = 8, .dst_msize = 8 };
    dmac_chn_ctl1_reg_t burst = { .arlen_en = 1, .arlen = 16,
                                 .awlen_en = 1, .awlen = 16 };

    /* Fixed CSI FIFO at master port 0, incrementing PSRAM on port 1. */
    writel(P4_CSI_FIFO, &ch->sar0.val);
    writel(0, &ch->sar1.val);
    writel(P4_CSI_FIFO, &ch->sstatar0.val);
    writel(0, &ch->cfg0.val); /* single contiguous block */
    writel(cfg.val, &ch->cfg1.val);
    writel(ctl.val, &ch->ctl0.val);
    writel(burst.val, &ch->ctl1.val);
    writel(CSI_DMA_EVENTS, &ch->int_st_ena0.val);
    writel(U32_MAX, &ch->int_clr0.val);
    writel(CSI_DMA_EVENTS, &ch->int_sig_ena0.val);
}

static void csi_dma_arm(struct p4_cam *cam, unsigned int slot)
{
    volatile dmac_channel_reg_t __iomem *ch = &cam->dma->ch[CSI_DMA_CH];

    writel(BASE + slot * SLOT, &ch->dar0.val);
    writel(0, &ch->dar1.val);
    /*
     * The bridge is the flow controller. Its EOF ends the RGB565 transfer.
     * Program the RAW8 input count, as in the IDF CSI controller.
     */
    writel(WIDTH * HEIGHT / 8 - 1, &ch->block_ts0.val);
    writel(CSI_DMA_CH_WE | CSI_DMA_CH_EN, &cam->dma->chen0.val);
}

static int csi_receiver_init(struct p4_cam *cam)
{
    void __iomem *host = cam->host;
    void __iomem *bridge = cam->bridge;
    void __iomem *isp = cam->isp;
    u32 status;
    int ret;

    /* Configure only CSI clocks and resets. The DSI driver owns VDMA global
     * configuration and the shared 2.5 V MIPI supply on this board.
     */
    csi_update(cam->clkrst + P4_CSI_BUS_CLOCKS, BIT(10) | BIT(11),
               BIT(10) | BIT(11));
    csi_update(cam->clkrst + P4_CSI_DPHY_CLOCK, GENMASK(4, 2), BIT(4));
    /* ISP functional clock is PLL_F240M / 3. */
    csi_update(cam->clkrst + P4_CSI_ISP_CLOCK, GENMASK(25, 23), BIT(23));
    csi_update(cam->clkrst + P4_CSI_ISP_DIV, GENMASK(7, 0), 2);
    csi_update(cam->clkrst + P4_CSI_ISP_CLOCK, BIT(25), BIT(25));
    csi_update(cam->clkrst + P4_CSI_RESETS, P4_CSI_RESET_MASK,
               P4_CSI_RESET_MASK);
    csi_update(cam->clkrst + P4_CSI_RESETS, P4_CSI_RESET_MASK, 0);
    cam->receiver_on = true;

    writel(BIT(0) | BIT(1), bridge + CSI_BRG_HOST_CTRL);
    writel(0, host + CSI_PHY_SHUTDOWNZ);
    writel(0, host + CSI_DPHY_RSTZ);
    writel(0, host + CSI_RESETN);
    writel(BIT(16), host + CSI_PHY_TEST_CTRL1);
    writel(BIT(0), host + CSI_PHY_TEST_CTRL0);
    writel(0, host + CSI_PHY_TEST_CTRL0);
    /* D-PHY test register 0x44, range 400–449 Mbps per lane. */
    writel(BIT(16) | 0x44, host + CSI_PHY_TEST_CTRL1);
    writel(BIT(1), host + CSI_PHY_TEST_CTRL0);
    writel(0, host + CSI_PHY_TEST_CTRL0);
    writel(0x4a, host + CSI_PHY_TEST_CTRL1);
    writel(BIT(1), host + CSI_PHY_TEST_CTRL0);
    writel(0, host + CSI_PHY_TEST_CTRL0);
    writel(1, host + CSI_PHY_SHUTDOWNZ);
    writel(1, host + CSI_DPHY_RSTZ);
    writel(1, host + CSI_RESETN);
    writel(1, host + CSI_N_LANES); /* 2 active data lanes */
    writel(1, host + CSI_VC_EXTENSION); /* VCX disabled */
    writel(0, host + CSI_SCRAMBLING);
    ret = readl_poll_timeout(host + CSI_PHY_STOPSTATE, status,
                            (status & CSI_LP11) == CSI_LP11, 10, 10000);
    if (ret) {
        dev_err(&cam->client->dev, "CSI lanes did not enter LP-11: %#x\n",
                status);
        return ret;
    }

    writel(0, bridge + CSI_BRG_ENABLE);
    writel(512, bridge + CSI_BRG_DMA_REQ_CFG);
    csi_update(bridge + CSI_BRG_BUF_FLOW_CTL, GENMASK(13, 0), 960);
    writel((0x2f << 8) | 0x12, bridge + CSI_BRG_DATA_TYPE_CFG);
    writel((WIDTH << 12) | HEIGHT, bridge + CSI_BRG_FRAME_CFG);
    writel(0, bridge + CSI_BRG_ENDIAN_MODE);
    writel(0, bridge + CSI_BRG_INT_ENA);
    writel(U32_MAX, bridge + CSI_BRG_INT_CLR);

    csi_update(isp + ISP_CLK_EN, BIT(0), BIT(0));
    writel(0, isp + ISP_CNTL);
    writel(0, isp + ISP_INT_ENA);
    writel(U32_MAX, isp + ISP_INT_CLR);
    writel(BIT(27) | ((WIDTH - 1) << 12) | (HEIGHT - 1),
           isp + ISP_FRAME_CFG); /* GBRG */
    writel(ISP_RGB565 | ISP_MIPI_DATA_EN | ISP_ENABLE, isp + ISP_CNTL);

    csi_dma_configure(cam);
    return 0;
}

static void csi_receiver_stop(struct p4_cam *cam)
{
    volatile dmac_channel_reg_t __iomem *ch = &cam->dma->ch[CSI_DMA_CH];
    u32 status;
    int ret;

    if (!cam->receiver_on)
        return;
    writel(0, &ch->int_sig_ena0.val);
    writel(CSI_DMA_CH_WE, &cam->dma->chen0.val);
    ret = readl_poll_timeout_atomic(&cam->dma->chen0.val, status,
                                    !(status & CSI_DMA_CH_EN), 1, 1000);
    if (ret)
        dev_err_ratelimited(&cam->client->dev, "CSI VDMA channel did not stop\n");
    writel(U32_MAX, &ch->int_clr0.val);
    writel(0, cam->bridge + CSI_BRG_ENABLE);
    writel(0, cam->isp + ISP_CNTL);
    writel(0, cam->host + CSI_RESETN);
    cam->receiver_on = false;
}

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
/* See ov5647_stream_off() in the Linux OV5647 driver: keep the
 * transmitter out of software standby so that it actively drives LP-11.
 */
static int sensor_output(struct p4_cam *cam, bool enable)
{
    int ret;

    ret = sensor_write(cam, 0x4800, enable ? 0x24 : 0x25);
    if (ret)
        return ret;
    ret = sensor_write(cam, 0x4202, enable ? 0 : 0x0f);
    if (ret)
        return ret;
    return sensor_write(cam, 0x300d, enable ? 0 : 1);
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
    ret = sensor_output(cam, false);
    if (ret)
        return ret;
    return sensor_write(cam, 0x0100, 1);
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
    ret=csi_receiver_init(cam);
    if (ret) goto fail;
    spin_lock_irqsave(&cam->lock, flags);
    cam->sequence=0;
    cam->running=true;
    csi_dma_arm(cam, next_slot(cam));
    writel(1, cam->bridge + CSI_BRG_ENABLE);
    spin_unlock_irqrestore(&cam->lock, flags);
    ret=sensor_output(cam, true);
    if (!ret) return 0;
fail:
    spin_lock_irqsave(&cam->lock, flags);
    cam->running=false;
    csi_receiver_stop(cam);
    return_buffers(cam, VB2_BUF_STATE_QUEUED);
    spin_unlock_irqrestore(&cam->lock, flags);
    sensor_write(cam, 0x0100, 0);
    return ret;
}
static void stop(struct vb2_queue *q)
{
    struct p4_cam *cam=vb2_get_drv_priv(q);
    unsigned long flags;
    int ret;

    if (!cam->disconnected) {
        ret = sensor_output(cam, false);
        if (ret)
            dev_warn(&cam->client->dev, "could not stop sensor output: %d\n", ret);
    }
    spin_lock_irqsave(&cam->lock, flags);
    cam->running=false;
    csi_receiver_stop(cam);
    return_buffers(cam, VB2_BUF_STATE_ERROR);
    spin_unlock_irqrestore(&cam->lock, flags);
    if (!cam->disconnected)
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
    volatile dmac_channel_reg_t __iomem *ch = &cam->dma->ch[CSI_DMA_CH];
    u32 status;

    spin_lock_irqsave(&cam->lock, flags);
    if (!cam->running) { spin_unlock_irqrestore(&cam->lock, flags); return IRQ_NONE; }
    status=readl(&ch->int_st0.val);
    if (!(status & CSI_DMA_EVENTS)) {
        spin_unlock_irqrestore(&cam->lock, flags);
        return IRQ_NONE;
    }
    writel(status, &ch->int_clr0.val);
    done=cam->active;
    cam->active=NULL;
    if (status & CSI_DMA_ERRORS) {
        dev_err_ratelimited(&cam->client->dev, "CSI VDMA failure: %#x\n", status);
        goto error;
    }
    cam->frames++;
    if (!done) cam->dropped++;
    else {
        done->vb.sequence=cam->sequence;
        done->vb.field=V4L2_FIELD_NONE;
        done->vb.vb2_buf.timestamp=ktime_get_ns();
    }
    cam->sequence++;
    csi_dma_arm(cam, next_slot(cam));
    if (done) vb2_buffer_done(&done->vb.vb2_buf, VB2_BUF_STATE_DONE);
    spin_unlock_irqrestore(&cam->lock, flags);
    return IRQ_HANDLED;
error:
    cam->errors++;
    cam->running=false;
    csi_receiver_stop(cam);
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
/*
 * The Linux display device owns the shared MIPI rail and VDMA controller.
 * Keep it bound until the camera IRQ and stream are stopped on unbind.
 */
static int camera_link_display(struct device *dev)
{
    struct device_node *np;
    struct platform_device *supplier;
    struct device_link *link;

    np = of_parse_phandle(dev->of_node, "espressif,mipi-supplier", 0);
    if (!np)
        return -EINVAL;
    supplier = of_find_device_by_node(np);
    of_node_put(np);
    if (!supplier)
        return -EPROBE_DEFER;

    link = device_link_add(dev, &supplier->dev, DL_FLAG_AUTOREMOVE_CONSUMER);
    put_device(&supplier->dev);
    if (!link)
        return -EINVAL;
    if (READ_ONCE(link->status) != DL_STATE_CONSUMER_PROBE)
        return -EPROBE_DEFER;
    return 0;
}

static int camera_probe(struct i2c_client *client)
{
    struct device_node *pool;
    struct resource res;
    struct p4_cam *cam;
    int high, low, ret;
    cam=kzalloc(sizeof(*cam), GFP_KERNEL);
    if (!cam) return -ENOMEM;
    cam->client=client;
    mutex_init(&cam->mutex); spin_lock_init(&cam->lock); INIT_LIST_HEAD(&cam->pending);
    ret=camera_link_display(&client->dev);
    if (ret) goto free;
    cam->host=devm_ioremap(&client->dev, P4_CSI_HOST_BASE, 0x800);
    cam->bridge=devm_ioremap(&client->dev, P4_CSI_BRIDGE_BASE, 0x800);
    cam->isp=devm_ioremap(&client->dev, P4_CSI_ISP_BASE, 0x1000);
    cam->clkrst=devm_ioremap(&client->dev, P4_CSI_CLKRST_BASE, 0x1000);
    cam->dma=devm_ioremap(&client->dev, P4_CSI_VDMA_BASE, sizeof(*cam->dma));
    if (!cam->host || !cam->bridge || !cam->isp || !cam->clkrst || !cam->dma) {
        ret=-ENOMEM;
        goto free;
    }
    cam->irq=of_irq_get(client->dev.of_node, 0);
    if (cam->irq<0) { ret=cam->irq; goto free; }
    high=sensor_read(cam, 0x300a); low=sensor_read(cam, 0x300b);
    if (high<0 || low<0) { ret=high<0 ? high : low; goto free; }
    if ((high<<8 | low)!=0x5647) { ret=-ENODEV; goto free; }
    ret=sensor_write(cam, 0x0100, 0);
    if (ret) goto free;
    pool=of_parse_phandle(client->dev.of_node, "memory-region", 0);
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
