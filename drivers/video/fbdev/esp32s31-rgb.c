// SPDX-License-Identifier: GPL-2.0-only
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/fb.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/rational.h>
#include <linux/sizes.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>

/* ESP32-S31 TRM ch.24 LCD_CAM — LCD_CLOCK, LCD_USER, LCD_MISC, LCD_RGB_*. */
#define LCD_CLOCK		0x00
#define LCD_PCLK_DIV		GENMASK(5, 0)
#define LCD_PCLK_NEGATIVE	BIT(8)
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
#define LCD_TRANS_BUFF_CFG	0x3c
#define LCD_TRANS_BUFFER_EN	BIT(0)
#define LCD_INT_ENA		0x64
#define LCD_INT_STATUS		0x6c
#define LCD_INT_CLR		0x70
#define LCD_VSYNC		BIT(0)

/* ESP32-S31 TRM ch.24 LCD_CAM — HP_SYS_CLKRST_LCDCAM_*CTRL0. */
#define LCDCAM_CTRL		0x00
#define LCDCAM_APB_CLK_EN	BIT(0)
#define LCDCAM_APB_RST		BIT(1)
#define LCDCAM_SYS_CLK_EN	BIT(2)
#define LCDCAM_RST		BIT(3)
#define LCD_CLK_CTRL		0x04
#define LCDCAM_CLK_CTRL		0x08
#define LCD_CLK_SRC		GENMASK(1, 0)
#define LCD_CLK_EN		BIT(2)
#define LCD_CLK_DIV		GENMASK(10, 3)
#define LCD_CLK_NUM		GENMASK(18, 11)
#define LCD_CLK_DEN		GENMASK(26, 19)
#define LCD_PLL_F160		1
#define LCDCAM_PLL_F120		1

/* ESP32-S31 TRM ch.5 System and Memory — cached and direct PSRAM windows. */
#define S31_PSRAM_BASE		0x50000000UL
#define S31_PSRAM_DIRECT	0xc0000000UL
#define S31_PSRAM_SIZE		SZ_64M

#define RGB565_BYTES		2
#define LCD_PIXEL_DIV		2
#define LCD_RGB_USER		(LCD_ALWAYS_OUT | FIELD_PREP(LCD_BYTE_MODE, 1) | LCD_DOUT)

struct s31_rgb {
	void __iomem *base;
	void __iomem *clkrst;
	struct gpio_desc *enable;
	struct dma_chan *tx;
	struct device *dma_dev;
	struct completion first_vsync;
	void *pages;
	dma_addr_t dma_addr;
	size_t size;
	u32 palette[16];
	int irq;
};

static irqreturn_t s31_rgb_irq(int irq, void *data)
{
	struct s31_rgb *rgb = data;

	if (!(readl(rgb->base + LCD_INT_STATUS) & LCD_VSYNC))
		return IRQ_NONE;

	writel(LCD_VSYNC, rgb->base + LCD_INT_CLR);
	complete(&rgb->first_vsync);
	return IRQ_HANDLED;
}

static int s31_rgb_setcolreg(unsigned int regno, unsigned int red,
			    unsigned int green, unsigned int blue,
			    unsigned int transp, struct fb_info *info)
{
	struct s31_rgb *rgb = info->par;

	if (regno >= ARRAY_SIZE(rgb->palette))
		return -EINVAL;
	rgb->palette[regno] = ((red >> 11) << 11) | ((green >> 10) << 5) | (blue >> 11);

	return 0;
}

static void s31_rgb_destroy(struct fb_info *info)
{
	struct s31_rgb *rgb = info->par;

	iounmap(info->screen_base);
	dma_unmap_single(rgb->dma_dev, rgb->dma_addr, rgb->size, DMA_TO_DEVICE);
	put_device(rgb->dma_dev);
	free_pages_exact(rgb->pages, rgb->size);
	framebuffer_release(info);
}

static const struct fb_ops s31_rgb_ops = {
	.owner = THIS_MODULE,
	FB_DEFAULT_IOMEM_OPS,
	.fb_setcolreg = s31_rgb_setcolreg,
	.fb_destroy = s31_rgb_destroy,
};

static int s31_rgb_alloc_fb(struct fb_info *info, const struct videomode *mode)
{
	struct s31_rgb *rgb = info->par;
	phys_addr_t phys, direct;
	size_t stride = mode->hactive * RGB565_BYTES;

	if (check_mul_overflow(stride, (size_t)mode->vactive, &rgb->size) ||
	    rgb->size > S31_PSRAM_SIZE)
		return -EINVAL;

	rgb->pages = alloc_pages_exact(rgb->size, GFP_KERNEL | __GFP_ZERO);
	if (!rgb->pages)
		return -ENOMEM;

	phys = virt_to_phys(rgb->pages);
	if (phys < S31_PSRAM_BASE ||
	    phys - S31_PSRAM_BASE > S31_PSRAM_SIZE - rgb->size)
		goto free_pages;

	rgb->dma_dev = dmaengine_get_dma_device(rgb->tx);
	rgb->dma_addr = dma_map_single(rgb->dma_dev, rgb->pages, rgb->size, DMA_TO_DEVICE);
	if (dma_mapping_error(rgb->dma_dev, rgb->dma_addr))
		goto free_pages;

	/*
	 * Sv32 cannot mark the cached PSRAM window uncached. Draw through the
	 * direct PSRAM window while GDMA reads the same pages via the bus window.
	 */
	direct = S31_PSRAM_DIRECT + (phys - S31_PSRAM_BASE);
	info->screen_base = ioremap(direct, rgb->size);
	if (!info->screen_base)
		goto unmap;

	get_device(rgb->dma_dev);
	info->fbops = &s31_rgb_ops;
	info->screen_size = rgb->size;
	info->pseudo_palette = rgb->palette;
	strscpy(info->fix.id, "ESP32-S31 RGB", sizeof(info->fix.id));
	info->fix.smem_start = direct;
	info->fix.smem_len = rgb->size;
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.line_length = stride;
	fb_videomode_to_var(&info->var, &(struct fb_videomode) {
		.xres = mode->hactive,
		.yres = mode->vactive,
		.pixclock = KHZ2PICOS(mode->pixelclock / 1000),
		.left_margin = mode->hback_porch,
		.right_margin = mode->hfront_porch,
		.upper_margin = mode->vback_porch,
		.lower_margin = mode->vfront_porch,
		.hsync_len = mode->hsync_len,
		.vsync_len = mode->vsync_len,
		.sync = (mode->flags & DISPLAY_FLAGS_HSYNC_HIGH ? FB_SYNC_HOR_HIGH_ACT : 0) |
			(mode->flags & DISPLAY_FLAGS_VSYNC_HIGH ? FB_SYNC_VERT_HIGH_ACT : 0),
	});
	info->var.bits_per_pixel = 16;
	info->var.red = (struct fb_bitfield){ 11, 5, 0 };
	info->var.green = (struct fb_bitfield){ 5, 6, 0 };
	info->var.blue = (struct fb_bitfield){ 0, 5, 0 };
	info->var.width = info->var.height = -1;
	return 0;

unmap:
	dma_unmap_single(rgb->dma_dev, rgb->dma_addr, rgb->size, DMA_TO_DEVICE);
free_pages:
	free_pages_exact(rgb->pages, rgb->size);
	return -EIO;
}

static int s31_rgb_set_clocks(struct s31_rgb *rgb, unsigned long rate,
			      const struct videomode *mode)
{
	const u32 gates = LCDCAM_APB_CLK_EN | LCDCAM_SYS_CLK_EN;
	unsigned long target, div, numerator, denominator;

	if (check_mul_overflow(mode->pixelclock, (unsigned long)LCD_PIXEL_DIV, &target) ||
	    !target || target > rate)
		return -EINVAL;

	div = rate / target;
	rational_best_approximation(rate % target, target, FIELD_MAX(LCD_CLK_NUM),
				    FIELD_MAX(LCD_CLK_DEN), &numerator, &denominator);
	if (numerator == denominator) {
		div++;
		numerator = 0;
		denominator = 1;
	}
	if (!FIELD_FIT(LCD_CLK_DIV, div - 1))
		return -ERANGE;

	writel(LCD_CLK_EN | FIELD_PREP(LCD_CLK_SRC, LCDCAM_PLL_F120) |
	       FIELD_PREP(LCD_CLK_DIV, 1), rgb->clkrst + LCDCAM_CLK_CTRL);
	writel(LCD_CLK_EN | FIELD_PREP(LCD_CLK_SRC, LCD_PLL_F160) |
	       FIELD_PREP(LCD_CLK_DIV, div - 1) | FIELD_PREP(LCD_CLK_NUM, numerator) |
	       FIELD_PREP(LCD_CLK_DEN, denominator), rgb->clkrst + LCD_CLK_CTRL);
	writel(gates | LCDCAM_APB_RST, rgb->clkrst + LCDCAM_CTRL);
	writel(gates, rgb->clkrst + LCDCAM_CTRL);
	writel(gates | LCDCAM_RST, rgb->clkrst + LCDCAM_CTRL);
	writel(gates, rgb->clkrst + LCDCAM_CTRL);

	writel(FIELD_PREP(LCD_PCLK_DIV, LCD_PIXEL_DIV - 1) |
	       (mode->flags & DISPLAY_FLAGS_PIXDATA_NEGEDGE ? LCD_PCLK_NEGATIVE : 0),
	       rgb->base + LCD_CLOCK);
	return 0;
}

static int s31_rgb_program_mode(struct s31_rgb *rgb, const struct videomode *mode)
{
	u64 hblank = (u64)mode->hsync_len + mode->hback_porch;
	u64 vblank = (u64)mode->vsync_len + mode->vback_porch;
	u64 htotal = hblank + mode->hactive + mode->hfront_porch;
	u64 vtotal = vblank + mode->vactive + mode->vfront_porch;
	u32 sync = LCD_HSYNC_IN_PORCH;

	if (!mode->hactive || !mode->vactive || !mode->hsync_len || !mode->vsync_len ||
	    htotal > FIELD_MAX(LCD_TOTAL) + 1 || vtotal > FIELD_MAX(LCD_TOTAL) + 1 ||
	    mode->hsync_len > FIELD_MAX(LCD_HSYNC_WIDTH) + 1 ||
	    mode->vsync_len > FIELD_MAX(LCD_VSYNC_WIDTH) + 1 ||
	    mode->flags & (DISPLAY_FLAGS_INTERLACED | DISPLAY_FLAGS_DOUBLESCAN |
			   DISPLAY_FLAGS_DOUBLECLK))
		return -EINVAL;

	/* Follow IDF lcd_ll_set_{horizontal,vertical}_timing() for count encoding. */
	if (!(mode->flags & DISPLAY_FLAGS_HSYNC_HIGH))
		sync |= LCD_HSYNC_IDLE_HIGH;
	if (!(mode->flags & DISPLAY_FLAGS_VSYNC_HIGH))
		sync |= LCD_VSYNC_IDLE_HIGH;
	if (mode->flags & DISPLAY_FLAGS_DE_LOW)
		sync |= LCD_DE_IDLE_HIGH;

	writel(LCD_RESET, rgb->base + LCD_USER);
	writel(LCD_FIFO_RESET, rgb->base + LCD_MISC);
	writel(FIELD_PREP(LCD_ACTIVE, mode->hactive - 1) | FIELD_PREP(LCD_TOTAL, htotal - 1),
	       rgb->base + LCD_RGB_HORIZONTAL);
	writel(FIELD_PREP(LCD_ACTIVE, mode->vactive - 1) | FIELD_PREP(LCD_TOTAL, vtotal - 1),
	       rgb->base + LCD_RGB_VERTICAL);
	writel(FIELD_PREP(LCD_HBLANK, hblank - 1) | FIELD_PREP(LCD_VBLANK, vblank - 1),
	       rgb->base + LCD_RGB_BLANK);
	writel(sync | FIELD_PREP(LCD_HSYNC_WIDTH, mode->hsync_len - 1) |
	       FIELD_PREP(LCD_VSYNC_WIDTH, mode->vsync_len - 1), rgb->base + LCD_RGB_CTRL);
	writel(LCD_RGB_MODE | FIELD_PREP(LCD_WIRE_MODE, 1) | LCD_NEXT_FRAME |
	       LCD_BLANK_ENABLE, rgb->base + LCD_MISC);
	writel(LCD_TRANS_BUFFER_EN, rgb->base + LCD_TRANS_BUFF_CFG);
	writel(LCD_RGB_USER | LCD_UPDATE, rgb->base + LCD_USER);
	return 0;
}

static int s31_rgb_start(struct s31_rgb *rgb)
{
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;

	desc = dmaengine_prep_dma_cyclic(rgb->tx, rgb->dma_addr, rgb->size, rgb->size,
					DMA_MEM_TO_DEV, DMA_CTRL_ACK);
	if (!desc)
		return -EIO;

	cookie = dmaengine_submit(desc);
	if (dma_submit_error(cookie))
		return cookie;

	writel(LCD_VSYNC, rgb->base + LCD_INT_CLR);
	writel(LCD_VSYNC, rgb->base + LCD_INT_ENA);
	/* TRM ch.24 LCD_CAM — start the GDMA outlink before asserting LCD_START. */
	dma_async_issue_pending(rgb->tx);
	writel(LCD_RGB_USER | LCD_UPDATE | LCD_START, rgb->base + LCD_USER);
	if (!wait_for_completion_timeout(&rgb->first_vsync, msecs_to_jiffies(1000)))
		return -ETIMEDOUT;
	gpiod_set_value_cansleep(rgb->enable, 1);
	return 0;
}

static void s31_rgb_stop(struct s31_rgb *rgb)
{
	gpiod_set_value_cansleep(rgb->enable, 0);
	writel(0, rgb->base + LCD_INT_ENA);
	writel(LCD_RGB_USER | LCD_UPDATE, rgb->base + LCD_USER);
	free_irq(rgb->irq, rgb);
	dmaengine_terminate_sync(rgb->tx);
}

static void s31_rgb_disable_clocks(struct s31_rgb *rgb)
{
	writel(0, rgb->clkrst + LCDCAM_CTRL);
	writel(0, rgb->clkrst + LCD_CLK_CTRL);
	writel(0, rgb->clkrst + LCDCAM_CLK_CTRL);
}

static int s31_rgb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dma_slave_config dma_config = {
		.direction = DMA_MEM_TO_DEV,
		.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES,
	};
	struct display_timing timing;
	struct videomode mode;
	struct fb_info *info;
	struct s31_rgb *rgb;
	struct clk *pll;
	u32 bus_width;
	int ret;

	ret = device_property_read_u32(dev, "bus-width", &bus_width);
	if (ret || bus_width != 16)
		return dev_err_probe(dev, -EINVAL, "expected a 16-bit RGB bus\n");
	ret = of_get_display_timing(dev->of_node, "panel-timing", &timing);
	if (ret)
		return dev_err_probe(dev, ret, "missing RGB panel timing\n");
	videomode_from_timing(&timing, &mode);
	if (mode.pixelclock < 1000 || !mode.hactive || mode.hactive > U16_MAX ||
	    !mode.vactive || mode.vactive > U16_MAX)
		return -EINVAL;

	info = framebuffer_alloc(sizeof(*rgb), dev);
	if (!info)
		return -ENOMEM;
	rgb = info->par;
	init_completion(&rgb->first_vsync);

	rgb->base = devm_platform_ioremap_resource_byname(pdev, "lcdcam");
	rgb->clkrst = devm_platform_ioremap_resource_byname(pdev, "clkrst");
	if (IS_ERR(rgb->base) || IS_ERR(rgb->clkrst)) {
		ret = IS_ERR(rgb->base) ? PTR_ERR(rgb->base) : PTR_ERR(rgb->clkrst);
		goto release;
	}
	pll = devm_clk_get_enabled(dev, "core");
	if (IS_ERR(pll)) {
		ret = PTR_ERR(pll);
		goto release;
	}
	pll = devm_clk_get_enabled(dev, "lcd");
	if (IS_ERR(pll)) {
		ret = PTR_ERR(pll);
		goto release;
	}
	rgb->enable = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(rgb->enable)) {
		ret = PTR_ERR(rgb->enable);
		goto release;
	}
	rgb->irq = platform_get_irq(pdev, 0);
	if (rgb->irq < 0) {
		ret = rgb->irq;
		goto release;
	}

	rgb->tx = dma_request_chan(dev, "tx");
	if (IS_ERR(rgb->tx)) {
		ret = PTR_ERR(rgb->tx);
		goto release;
	}
	ret = dmaengine_slave_config(rgb->tx, &dma_config);
	if (ret)
		goto release_dma;
	ret = s31_rgb_alloc_fb(info, &mode);
	if (ret)
		goto release_dma;
	ret = s31_rgb_set_clocks(rgb, clk_get_rate(pll), &mode);
	if (ret)
		goto release_dma;
	ret = s31_rgb_program_mode(rgb, &mode);
	if (ret)
		goto disable_clocks;
	writel(0, rgb->base + LCD_INT_ENA);
	ret = request_irq(rgb->irq, s31_rgb_irq, 0, "esp32s31-rgb-vsync", rgb);
	if (ret)
		goto disable_clocks;
	ret = s31_rgb_start(rgb);
	if (ret)
		goto stop;
	ret = register_framebuffer(info);
	if (ret)
		goto stop;

	platform_set_drvdata(pdev, info);
	dev_info(dev, "%ux%u RGB565 scanout at %lu Hz, fb%d\n",
		 mode.hactive, mode.vactive, mode.pixelclock, info->node);
	return 0;

stop:
	s31_rgb_stop(rgb);
disable_clocks:
	s31_rgb_disable_clocks(rgb);
release_dma:
	dma_release_channel(rgb->tx);
release:
	if (info->fbops)
		s31_rgb_destroy(info);
	else
		framebuffer_release(info);
	return dev_err_probe(dev, ret, "failed to initialize RGB framebuffer\n");
}

static void s31_rgb_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);
	struct s31_rgb *rgb = info->par;

	s31_rgb_stop(rgb);
	s31_rgb_disable_clocks(rgb);
	dma_release_channel(rgb->tx);
	unregister_framebuffer(info);
}

static const struct of_device_id s31_rgb_of_match[] = {
	{ .compatible = "espressif,esp32s31-lcd-rgb" },
	{ }
};
MODULE_DEVICE_TABLE(of, s31_rgb_of_match);

static struct platform_driver s31_rgb_driver = {
	.probe = s31_rgb_probe,
	.remove = s31_rgb_remove,
	.driver = {
		.name = "esp32s31-rgb",
		.of_match_table = s31_rgb_of_match,
	},
};
module_platform_driver(s31_rgb_driver);

MODULE_DESCRIPTION("ESP32-S31 LCD_CAM RGB framebuffer");
MODULE_LICENSE("GPL");
