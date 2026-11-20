// SPDX-License-Identifier: GPL-2.0-only
/*
 * Reconstructed ESP32-P4 framebuffer, not recovered original source.
 * The display pipeline belongs to Linux: this driver owns the RGB565 PSRAM
 * framebuffer and delegates DSI host, panel and VDMA programming to
 * esp32p4-dsi.c. The loader only publishes the shared cache service, which
 * writeback of the pixels the VDMA reads still goes through.
 */
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/sizes.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/soc/espressif/esp32p4.h>
#include <linux/uaccess.h>

#include "esp32p4-dsi.h"

#define P4_FB_BASE 0x49000000U
#define P4_FB_SIZE 0x001c2000U
#define P4_FB_WIDTH 720U
#define P4_FB_HEIGHT 1280U
#define P4_FB_STRIDE 1440U
#define P4_FB_CHUNK SZ_16K
#define P4_FB_COMPAT "espressif,esp32p4-idf-framebuffer"

struct p4_fb {
	struct fb_info *info;
	raw_spinlock_t service_lock;
	struct esp32p4_dsi *dsi;
	int irq;
	bool mem_claimed;
	bool running;
	bool failed;
	bool removed;
	u32 palette[16];
};

/* This lock never nests the platform's shared cache-engine lock. */
static void p4_fb_stop(struct p4_fb *fb)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&fb->service_lock, flags);
	if (fb->running) {
		esp32p4_dsi_stop(fb->dsi);
		fb->running = false;
	}
	raw_spin_unlock_irqrestore(&fb->service_lock, flags);
}

static irqreturn_t p4_fb_irq(int irq, void *data)
{
	struct p4_fb *fb = data;
	unsigned long flags;
	int ret = 0;

	raw_spin_lock_irqsave(&fb->service_lock, flags);
	if (fb->running) {
		ret = esp32p4_dsi_irq(fb->dsi);
		if (ret < 0) {
			esp32p4_dsi_stop(fb->dsi);
			fb->running = false;
			WRITE_ONCE(fb->failed, true);
		}
	}
	raw_spin_unlock_irqrestore(&fb->service_lock, flags);
	if (ret < 0)
		pr_err_ratelimited("esp32p4-fb: DMA service failed; scanout stopped\n");
	return ret ? IRQ_HANDLED : IRQ_NONE;
}

static bool p4_fb_unavailable(struct p4_fb *fb)
{
	return READ_ONCE(fb->removed) || READ_ONCE(fb->failed);
}

/*
 * Cache callbacks serialize BOTH harts and executable synchronization in
 * platform code. Each transaction is <=16KiB, 64-byte aligned, writeback only.
 * No service lock, allocation, user access or whole-frame lock in this loop.
 * Callers such as fbcon may already have IRQs disabled; do not enable them.
 */
static int p4_fb_flush(struct fb_info *info, u32 offset, u32 bytes)
{
	struct p4_fb *fb = info->par;
	u32 end, size;
	int ret;

	if (p4_fb_unavailable(fb))
		return -EIO;
	if (offset > P4_FB_SIZE || bytes > P4_FB_SIZE - offset)
		return -EINVAL;
	if (!bytes)
		return 0;
	end = ALIGN(offset + bytes, 64);
	offset &= ~63U;
	while (offset < end) {
		size = min_t(u32, end - offset, P4_FB_CHUNK);
		ret = esp32p4_display_cache_writeback(P4_FB_BASE + offset, size);
		if (ret) {
			WRITE_ONCE(fb->failed, true);
			/* Cache lock has already been released by the wrapper. */
			p4_fb_stop(fb);
			pr_err_ratelimited("esp32p4-fb: writeback failed; scanout stopped\n");
			return -EIO;
		}
		offset += size;
	}
	return 0;
}

static ssize_t p4_fb_read(struct fb_info *info, char __user *buf,
			  size_t count, loff_t *ppos)
{
	if (*ppos < 0)
		return -EINVAL;
	if (p4_fb_unavailable(info->par))
		return -EIO;
	return fb_sys_read(info, buf, count, ppos);
}

static ssize_t p4_fb_write(struct fb_info *info, const char __user *buf,
			   size_t count, loff_t *ppos)
{
	size_t done = 0, n, copied;
	unsigned long missing;
	u32 pos;

	if (*ppos < 0)
		return -EINVAL;
	if (p4_fb_unavailable(info->par))
		return -EIO;
	if (*ppos > P4_FB_SIZE)
		return -EFBIG;
	pos = *ppos;
	if (!count)
		return 0;
	if (pos == P4_FB_SIZE)
		return -ENOSPC;
	count = min_t(size_t, count, P4_FB_SIZE - pos);
	while (done < count) {
		n = min_t(size_t, count - done, P4_FB_CHUNK);
		missing = copy_from_user(info->screen_buffer + pos, buf + done, n);
		copied = n - missing;
		/* copy_from_user may zero an uncopied tail: flush that too. */
		if (p4_fb_flush(info, pos, n))
			return -EIO;
		pos += copied;
		done += copied;
		*ppos += copied;
		if (missing)
			return done ? done : -EFAULT;
	}
	return done;
}

static bool p4_fb_rect_valid(u32 x, u32 y, u32 width, u32 height)
{
	return x < P4_FB_WIDTH && y < P4_FB_HEIGHT &&
		width <= P4_FB_WIDTH - x && height <= P4_FB_HEIGHT - y;
}

static void p4_fb_damage(struct fb_info *info, u32 x, u32 y, u32 w, u32 h)
{
	u32 row;

	if (!w || !h)
		return;
	/* A glyph dirties only its destination lines, not the full framebuffer. */
	for (row = 0; row < h; row++)
		if (p4_fb_flush(info, (y + row) * P4_FB_STRIDE + x * 2, w * 2))
			break;
}

static void p4_fb_fillrect(struct fb_info *info, const struct fb_fillrect *r)
{
	if (p4_fb_unavailable(info->par) ||
	    !p4_fb_rect_valid(r->dx, r->dy, r->width, r->height))
		return;
	sys_fillrect(info, r);
	p4_fb_damage(info, r->dx, r->dy, r->width, r->height);
}

static void p4_fb_copyarea(struct fb_info *info, const struct fb_copyarea *a)
{
	if (p4_fb_unavailable(info->par) ||
	    !p4_fb_rect_valid(a->sx, a->sy, a->width, a->height) ||
	    !p4_fb_rect_valid(a->dx, a->dy, a->width, a->height))
		return;
	sys_copyarea(info, a);
	p4_fb_damage(info, a->dx, a->dy, a->width, a->height);
}

static void p4_fb_imageblit(struct fb_info *info, const struct fb_image *im)
{
	if (p4_fb_unavailable(info->par) ||
	    !p4_fb_rect_valid(im->dx, im->dy, im->width, im->height))
		return;
	sys_imageblit(info, im);
	p4_fb_damage(info, im->dx, im->dy, im->width, im->height);
}

static int p4_fb_setcolreg(unsigned int reg, unsigned int red,
			 unsigned int green, unsigned int blue,
			 unsigned int transp, struct fb_info *info)
{
	struct p4_fb *fb = info->par;

	if (reg >= ARRAY_SIZE(fb->palette))
		return -EINVAL;
	fb->palette[reg] = ((red >> 11) << 11) |
			  ((green >> 10) << 5) | (blue >> 11);
	return 0;
}

static int p4_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	/* Userspace stores would bypass dirty-range writeback. */
	return -ENODEV;
}

static void p4_fb_destroy(struct fb_info *info)
{
	struct p4_fb *fb = info->par;

	esp32p4_dsi_destroy(fb->dsi);
	if (fb->mem_claimed)
		release_mem_region(P4_FB_BASE, P4_FB_SIZE);
	framebuffer_release(info);
}

static const struct fb_ops p4_fb_ops = {
	.owner = THIS_MODULE,
	.fb_read = p4_fb_read,
	.fb_write = p4_fb_write,
	.fb_fillrect = p4_fb_fillrect,
	.fb_copyarea = p4_fb_copyarea,
	.fb_imageblit = p4_fb_imageblit,
	.fb_setcolreg = p4_fb_setcolreg,
	.fb_mmap = p4_fb_mmap,
	.fb_destroy = p4_fb_destroy,
};

static int p4_fb_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct fb_info *info;
	struct p4_fb *fb;
	const char *format;
	u32 reg[2], interrupts[2], width, height, stride, cache;
	unsigned long flags;
	bool mem_claimed;
	int ret;

	if (of_n_addr_cells(node) != 1 || of_n_size_cells(node) != 1) {
		dev_err(&pdev->dev, "node needs one address and size cell\n");
		return -EINVAL;
	}
	if (of_property_read_u32_array(node, "reg", reg, 2) ||
	    of_property_read_u32_array(node, "interrupts", interrupts, 2)) {
		dev_err(&pdev->dev, "node needs reg and interrupts\n");
		return -EINVAL;
	}
	if (of_property_read_u32(node, "width", &width) ||
	    of_property_read_u32(node, "height", &height) ||
	    of_property_read_u32(node, "stride", &stride) ||
	    of_property_read_string(node, "format", &format) ||
	    of_property_read_u32(node, "espressif,cache-writeback-entry", &cache)) {
		dev_err(&pdev->dev, "node is missing geometry or cache entry\n");
		return -EINVAL;
	}
	if (reg[0] != P4_FB_BASE || reg[1] != P4_FB_SIZE ||
	    width != P4_FB_WIDTH || height != P4_FB_HEIGHT ||
	    stride != P4_FB_STRIDE || strcmp(format, "r5g6b5")) {
		dev_err(&pdev->dev, "unexpected geometry: %ux%u stride %u %s reg %x+%x\n",
			width, height, stride, format, reg[0], reg[1]);
		return -EINVAL;
	}
	if (interrupts[0] != 24 || interrupts[1] != IRQ_TYPE_LEVEL_HIGH) {
		dev_err(&pdev->dev, "unexpected interrupt spec %u %u\n",
			interrupts[0], interrupts[1]);
		return -EINVAL;
	}
	if (!esp32p4_retained_entry_valid(cache)) {
		dev_err(&pdev->dev, "cache writeback entry 0x%08x is not retained SRAM\n",
			cache);
		return -EINVAL;
	}
	/*
	 * The framebuffer is described by /reserved-memory, so the OF layer has
	 * already inserted the range into iomem_resource and request_mem_region()
	 * reports it busy. That is success for us: the memory is ours by DT.
	 */
	mem_claimed = request_mem_region(P4_FB_BASE, P4_FB_SIZE, "esp32p4-fb") != NULL;
	info = framebuffer_alloc(sizeof(*fb), &pdev->dev);
	if (!info) {
		if (mem_claimed)
			release_mem_region(P4_FB_BASE, P4_FB_SIZE);
		return -ENOMEM;
	}
	fb = info->par;
	fb->info = info;
	fb->mem_claimed = mem_claimed;
	raw_spin_lock_init(&fb->service_lock);
	info->fbops = &p4_fb_ops;
	info->flags = FBINFO_VIRTFB;
	info->screen_buffer = (void *)(unsigned long)P4_FB_BASE;
	info->screen_size = P4_FB_SIZE;
	info->pseudo_palette = fb->palette;
	strscpy(info->fix.id, "esp32p4-idf", sizeof(info->fix.id));
	info->fix.smem_start = P4_FB_BASE;
	info->fix.smem_len = P4_FB_SIZE;
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.line_length = P4_FB_STRIDE;
	info->fix.accel = FB_ACCEL_NONE;
	info->var.xres = info->var.xres_virtual = P4_FB_WIDTH;
	info->var.yres = info->var.yres_virtual = P4_FB_HEIGHT;
	info->var.bits_per_pixel = 16;
	info->var.red = (struct fb_bitfield){ 11, 5, 0 };
	info->var.green = (struct fb_bitfield){ 5, 6, 0 };
	info->var.blue = (struct fb_bitfield){ 0, 5, 0 };
	info->var.width = info->var.height = -1;
	info->var.activate = FB_ACTIVATE_NOW;
	info->var.vmode = FB_VMODE_NONINTERLACED;
	{
		struct esp32p4_dsi_config cfg = {
			.fb_phys = P4_FB_BASE,
			.width = P4_FB_WIDTH,
			.height = P4_FB_HEIGHT,
		};

		fb->dsi = esp32p4_dsi_create(&pdev->dev, &cfg);
		if (IS_ERR(fb->dsi)) {
			ret = PTR_ERR(fb->dsi);
			fb->dsi = NULL;
			if (ret != -EPROBE_DEFER)
				dev_err(&pdev->dev, "DSI setup failed: %d\n", ret);
			goto release;
		}
	}
	ret = esp32p4_dsi_probe(fb->dsi);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "DSI bring-up failed: %d\n", ret);
		goto release;
	}
	fb->irq = esp32p4_dsi_irq_number(fb->dsi);
	if ((int)fb->irq < 0) {
		ret = (int)fb->irq;
		goto release;
	}
	ret = p4_fb_flush(info, 0, P4_FB_SIZE);
	if (ret)
		goto release;
	/* The DW-GDMA channel IRQ is shared between all channels. */
	ret = request_irq(fb->irq, p4_fb_irq, IRQF_SHARED, "esp32p4-fb", fb);
	if (ret)
		goto release;
	/* Nothing runs before Linux starts the stream. */
	raw_spin_lock_irqsave(&fb->service_lock, flags);
	ret = esp32p4_dsi_start(fb->dsi);
	if (!ret)
		fb->running = true;
	raw_spin_unlock_irqrestore(&fb->service_lock, flags);
	if (ret) {
		ret = -EIO;
		goto free_irq;
	}
	/* Turn on the backlight once the stream is running. */
	ret = esp32p4_dsi_late_init(fb->dsi);
	if (ret) {
		p4_fb_stop(fb);
		goto free_irq;
	}
	ret = register_framebuffer(info);
	if (ret) {
		p4_fb_stop(fb);
		goto free_irq;
	}
	platform_set_drvdata(pdev, info);
	dev_info(&pdev->dev, "fb%d: %ux%u RGB565 kernel DSI scanout\n",
		 info->node, P4_FB_WIDTH, P4_FB_HEIGHT);
	return 0;
free_irq:
	free_irq(fb->irq, fb);
release:
	p4_fb_destroy(info);
	return ret;
}

static void p4_fb_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);
	struct p4_fb *fb = info->par;

	WRITE_ONCE(fb->removed, true);
	p4_fb_stop(fb);
	free_irq(fb->irq, fb);
	/* Existing file references keep info alive through fb_destroy. */
	unregister_framebuffer(info);
}

static void p4_fb_shutdown(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);

	p4_fb_stop(info->par);
}

static const struct of_device_id p4_fb_match[] = {
	{ .compatible = P4_FB_COMPAT },
	{ }
};
MODULE_DEVICE_TABLE(of, p4_fb_match);

static struct platform_driver p4_fb_driver = {
	.probe = p4_fb_probe,
	.remove = p4_fb_remove,
	.shutdown = p4_fb_shutdown,
	.driver = {
		.name = "esp32p4-fb",
		.of_match_table = p4_fb_match,
	},
};

static int __init p4_fb_init(void)
{
	struct device_node *node;
	struct platform_device *pdev;
	int ret;

	ret = platform_driver_register(&p4_fb_driver);
	if (ret)
		return ret;
	/* /chosen is not a normal OF-populated platform bus. */
	node = of_get_compatible_child(of_chosen, P4_FB_COMPAT);
	if (!node)
		return 0;
	if (of_device_is_available(node)) {
		pdev = of_platform_device_create(node, NULL, NULL);
		if (!pdev) {
			of_node_put(node);
			platform_driver_unregister(&p4_fb_driver);
			return -ENODEV;
		}
	}
	of_node_put(node);
	return 0;
}
device_initcall(p4_fb_init);

MODULE_DESCRIPTION("ESP32-P4 kernel DSI framebuffer (reconstruction)");
MODULE_LICENSE("GPL");

