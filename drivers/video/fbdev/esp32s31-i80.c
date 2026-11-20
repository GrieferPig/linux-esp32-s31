// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S31 LCD_CAM 8080-I framebuffer for the ILI9341 panel.
 */
#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/fb.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/consumer.h>
#include <linux/input.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <video/mipi_display.h>

#define S31_LCD_WIDTH			320
#define S31_LCD_HEIGHT			240
#define S31_LCD_STRIDE			(S31_LCD_WIDTH * sizeof(u16))
#define S31_LCD_FRAME_BYTES		(S31_LCD_STRIDE * S31_LCD_HEIGHT)
#define S31_LCD_MAX_WR_HZ		15000000

/* ESP32-S31 TRM ch.24 LCD_CAM — LCD_CLOCK, LCD_USER, LCD_MISC, LC_DMA_INT. */
#define S31_LCD_CLOCK			0x00
#define S31_LCD_CLKCNT_N		GENMASK(5, 0)
#define S31_LCD_PCLK_IDLE_HIGH		BIT(7)
#define S31_LCD_USER			0x14
#define S31_LCD_ALWAYS_OUT_EN		BIT(13)
#define S31_LCD_UPDATE_REG		BIT(21)
#define S31_LCD_DOUT_SWIZZLE		BIT(17)
#define S31_LCD_DOUT			BIT(24)
#define S31_LCD_CMD			BIT(26)
#define S31_LCD_START			BIT(27)
#define S31_LCD_RESET			BIT(28)
#define S31_LCD_MISC			0x18
#define S31_LCD_BK_EN			BIT(26)
#define S31_LCD_AFIFO_RESET		BIT(27)
#define S31_LCD_CD_DATA_SET		BIT(28)
#define S31_LCD_FIRST_CMD_VAL		0x2c
#define S31_LCD_INT_ENA			0x64
#define S31_LCD_INT_RAW			0x68
#define S31_LCD_INT_ST			0x6c
#define S31_LCD_INT_CLR			0x70
#define S31_LCD_TRANS_DONE		BIT(1)

#define S31_DONE_DMA			BIT(0)
#define S31_DONE_LCD			BIT(1)
#define S31_DONE_ALL			(S31_DONE_DMA | S31_DONE_LCD)

/* ESP32-S31 TRM ch.24 LCD_CAM — HP_SYS_CLKRST_LCDCAM_{,LCD_,LCDCAM_}CTRL0. */
#define S31_LCDCAM_CTRL			0x00
#define S31_LCDCAM_APB_CLK_EN		BIT(0)
#define S31_LCDCAM_APB_RST_EN		BIT(1)
#define S31_LCDCAM_SYS_CLK_EN		BIT(2)
#define S31_LCDCAM_RST_EN		BIT(3)
#define S31_LCD_CLK_CTRL		0x04
#define S31_LCDCAM_CLK_CTRL		0x08
#define S31_CLK_SRC_SEL			GENMASK(1, 0)
#define S31_CLK_EN			BIT(2)
#define S31_CLK_DIV_NUM			GENMASK(10, 3)
#define S31_LCD_CLK_NUMERATOR		GENMASK(18, 11)
#define S31_LCD_CLK_DENOMINATOR		GENMASK(26, 19)
#define S31_LCD_CLK_PLL_F160M		1
#define S31_LCDCAM_CLK_PLL_F120M		1
#define S31_CLK_DIV_TWO			1

enum s31_touch_phase {
	S31_TOUCH_DETECT,
	S31_TOUCH_X,
	S31_TOUCH_X_REVERSE,
	S31_TOUCH_Y,
	S31_TOUCH_Y_REVERSE,
	S31_TOUCH_PHASES,
};

struct s31_lcd_touch {
	struct input_dev *input;
	struct iio_channel *sense[2];
	struct pinctrl *pinctrl;
	struct pinctrl_state *display;
	struct pinctrl_state *phase[S31_TOUCH_PHASES];
	struct touchscreen_properties props;
	bool pressed;
};

struct s31_lcd {
	struct device *dev;
	void __iomem *base;
	void __iomem *clkctrl;
	struct dma_chan *tx;
	struct device *dma_dev;
	struct completion tx_done;
	atomic_t tx_events;
	int irq;
	enum dmaengine_tx_result tx_result;
	void *frame;
	dma_addr_t fb_dma;
	void *transfer[2];
	struct mutex lock;
	struct mutex stage_lock;
	wait_queue_head_t stage_wait;
	int active;
	int pending;
	dma_addr_t pending_dma;
	atomic_t dirty;
	atomic_long_t frames_completed;
	bool stopping;
	bool clocks_on;
	u32 palette[16];
	struct fb_deferred_io defio;
	struct s31_lcd_touch touch;
};

static void s31_lcd_complete(struct s31_lcd *lcd, int events)
{
	int old = atomic_fetch_or(events, &lcd->tx_events);

	if (old != S31_DONE_ALL && (old | events) == S31_DONE_ALL)
		complete(&lcd->tx_done);
}

static irqreturn_t s31_lcd_irq(int irq, void *data)
{
	struct s31_lcd *lcd = data;

	/* TRM ch.24 §24.4 LCD_CAM — LCD_TRANS_DONE after transmission. */
	if (!(readl(lcd->base + S31_LCD_INT_ST) & S31_LCD_TRANS_DONE))
		return IRQ_NONE;
	writel(S31_LCD_TRANS_DONE, lcd->base + S31_LCD_INT_CLR);
	s31_lcd_complete(lcd, S31_DONE_LCD);
	return IRQ_HANDLED;
}

static void s31_lcd_dma_done(void *data, const struct dmaengine_result *result)
{
	struct s31_lcd *lcd = data;

	lcd->tx_result = result->result;
	s31_lcd_complete(lcd, lcd->irq && result->result == DMA_TRANS_NOERROR ?
			 S31_DONE_DMA : S31_DONE_ALL);
}

static void s31_lcd_stop_clocks(struct s31_lcd *lcd)
{
	writel(0, lcd->clkctrl + S31_LCD_CLK_CTRL);
	writel(0, lcd->clkctrl + S31_LCDCAM_CTRL);
	writel(0, lcd->clkctrl + S31_LCDCAM_CLK_CTRL);
}

static int s31_lcd_start_clocks(struct s31_lcd *lcd, unsigned long rate, u32 wr_hz)
{
	const u32 gates = S31_LCDCAM_APB_CLK_EN | S31_LCDCAM_SYS_CLK_EN;
	const u32 ctrl = FIELD_PREP(S31_CLK_DIV_NUM, S31_CLK_DIV_TWO) | S31_CLK_EN;
	unsigned int div;

	/*
	 * LCD functional clock 160 MHz / 2 and shared core clock 120 MHz / 2
	 * follow IDF lcd_i80_select_periph_clock() and LCD_CAM allocation.
	 */
	if (!rate || !wr_hz || wr_hz > S31_LCD_MAX_WR_HZ)
		return -EINVAL;
	div = DIV_ROUND_UP(rate / 2, wr_hz);
	if (div < 2 || div > FIELD_MAX(S31_LCD_CLKCNT_N) + 1)
		return -EINVAL;
	if (wr_hz == S31_LCD_MAX_WR_HZ && rate != 160000000)
		return dev_err_probe(lcd->dev, -EINVAL,
				     "15 MHz i80 requires 160 MHz parent, got %lu Hz\n",
				     rate);

	writel(ctrl | FIELD_PREP(S31_CLK_SRC_SEL, S31_LCDCAM_CLK_PLL_F120M),
	       lcd->clkctrl + S31_LCDCAM_CLK_CTRL);
	writel(ctrl | FIELD_PREP(S31_CLK_SRC_SEL, S31_LCD_CLK_PLL_F160M),
	       lcd->clkctrl + S31_LCD_CLK_CTRL);
	writel(gates | S31_LCDCAM_APB_RST_EN, lcd->clkctrl + S31_LCDCAM_CTRL);
	writel(gates, lcd->clkctrl + S31_LCDCAM_CTRL);
	writel(gates | S31_LCDCAM_RST_EN, lcd->clkctrl + S31_LCDCAM_CTRL);
	writel(gates, lcd->clkctrl + S31_LCDCAM_CTRL);
	writel(S31_LCD_PCLK_IDLE_HIGH | FIELD_PREP(S31_LCD_CLKCNT_N, div - 1),
	       lcd->base + S31_LCD_CLOCK);
	if (wr_hz != S31_LCD_MAX_WR_HZ)
		dev_info(lcd->dev, "i80 WR clock %lu Hz\n", rate / 2 / div);
	lcd->clocks_on = true;
	return 0;
}

static int s31_lcd_set_fast_wr_clock(struct s31_lcd *lcd)
{
	const u32 group = FIELD_PREP(S31_CLK_SRC_SEL, S31_LCD_CLK_PLL_F160M) |
			  S31_CLK_EN | FIELD_PREP(S31_CLK_DIV_NUM, S31_CLK_DIV_TWO) |
			  FIELD_PREP(S31_LCD_CLK_NUMERATOR, 2) |
			  FIELD_PREP(S31_LCD_CLK_DENOMINATOR, 3);
	const u32 pixel = S31_LCD_PCLK_IDLE_HIGH | FIELD_PREP(S31_LCD_CLKCNT_N, 3);
	u32 group_readback, pixel_readback;

	/*
	 * TRM ch.24 §24.3.3: 160 MHz / (2 + 2/3) / 4 = nominal 15 MHz.
	 * Change the dividers with LCD_CAM idle after the DCS setup transfers.
	 */
	writel(group, lcd->clkctrl + S31_LCD_CLK_CTRL);
	writel(pixel, lcd->base + S31_LCD_CLOCK);
	group_readback = readl(lcd->clkctrl + S31_LCD_CLK_CTRL);
	pixel_readback = readl(lcd->base + S31_LCD_CLOCK);
	if (group_readback != group || pixel_readback != pixel) {
		dev_err(lcd->dev, "i80 clock readback %#x/%#x differs from %#x/%#x\n",
			group_readback, pixel_readback, group, pixel);
		return -EIO;
	}
	dev_info(lcd->dev, "i80 WR clock 15000000 Hz (clkrst %#x, lcd_clock %#x)\n",
		 group_readback, pixel_readback);
	return 0;
}

static int s31_lcd_send_mapped(struct s31_lcd *lcd, dma_addr_t address,
			      u8 cmd, size_t len)
{
	const u32 misc = S31_LCD_BK_EN | S31_LCD_CD_DATA_SET;
	u32 user = S31_LCD_CMD | S31_LCD_ALWAYS_OUT_EN;
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;
	u32 raw;
	int ret;

	if (len)
		user |= S31_LCD_DOUT;
	/* TRM ch.24 LCD_CAM, Table 24.3-3 — LCD_DOUT_BYTE_SWIZZLE_MODE 0.
	 * Swap adjacent bytes in native RGB565 pixel data on the 8-bit bus.
	 * IDF uses lcd_ll_enable_swizzle() for color data only.
	 */
	if (cmd == MIPI_DCS_WRITE_MEMORY_START)
		user |= S31_LCD_DOUT_SWIZZLE;

	desc = dmaengine_prep_slave_single(lcd->tx, address, len ?: sizeof(u32),
					  DMA_MEM_TO_DEV,
					  DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc)
		return -EIO;
	reinit_completion(&lcd->tx_done);
	atomic_set(&lcd->tx_events, 0);
	desc->callback_result = s31_lcd_dma_done;
	desc->callback_param = lcd;
	cookie = dmaengine_submit(desc);
	ret = dma_submit_error(cookie);
	if (ret)
		return ret;

	writel(S31_LCD_TRANS_DONE, lcd->base + S31_LCD_INT_CLR);
	writel(cmd, lcd->base + S31_LCD_FIRST_CMD_VAL);
	writel(user | S31_LCD_RESET, lcd->base + S31_LCD_USER);
	writel(misc | S31_LCD_AFIFO_RESET, lcd->base + S31_LCD_MISC);
	dma_wmb();
	dma_async_issue_pending(lcd->tx);
	writel(user | S31_LCD_UPDATE_REG, lcd->base + S31_LCD_USER);
	writel(user | S31_LCD_START, lcd->base + S31_LCD_USER);

	if (!wait_for_completion_timeout(&lcd->tx_done, msecs_to_jiffies(50)))
		ret = -ETIMEDOUT;
	else if (lcd->tx_result != DMA_TRANS_NOERROR)
		ret = -EIO;
	else if (!lcd->irq) /* DTs published before LCD_CAM IRQ support. */
		ret = readl_poll_timeout(lcd->base + S31_LCD_INT_RAW, raw,
					 raw & S31_LCD_TRANS_DONE, 10, 20000);
	if (ret) {
		writel(0, lcd->base + S31_LCD_INT_ENA);
		writel(user | S31_LCD_RESET, lcd->base + S31_LCD_USER);
		writel(user | S31_LCD_UPDATE_REG, lcd->base + S31_LCD_USER);
		dmaengine_terminate_sync(lcd->tx);
		writel(S31_LCD_TRANS_DONE, lcd->base + S31_LCD_INT_CLR);
		if (lcd->irq) {
			synchronize_irq(lcd->irq);
			writel(S31_LCD_TRANS_DONE, lcd->base + S31_LCD_INT_ENA);
		}
	}
	return ret;
}

static int s31_lcd_cmd(struct s31_lcd *lcd, u8 cmd, const u8 *data, size_t len)
{
	size_t bytes = len ?: sizeof(u32);

	/* Panel setup precedes publication of the zeroed framebuffer. */
	if (len)
		memcpy(lcd->frame, data, len);
	else
		memset(lcd->frame, 0, bytes);
	dma_sync_single_range_for_device(lcd->dma_dev, lcd->fb_dma, 0, bytes,
					 DMA_TO_DEVICE);
	return s31_lcd_send_mapped(lcd, lcd->fb_dma, cmd, len);
}

static int s31_lcd_panel_init(struct s31_lcd *lcd, struct gpio_desc *reset)
{
	/* ILI9341 datasheet §8.2.12, §8.2.29, §8.2.33. The shield uses
	 * landscape orientation (MV | BGR) and RGB565, as verified with IDF.
	 */
	const u8 madctl = BIT(5) | BIT(3);
	const u8 rgb565 = 0x55;
	const u8 cols[] = { 0, 0, (S31_LCD_WIDTH - 1) >> 8, (S31_LCD_WIDTH - 1) & 0xff };
	const u8 rows[] = { 0, 0, 0, S31_LCD_HEIGHT - 1 };
	int ret;

	gpiod_set_value_cansleep(reset, 1);
	fsleep(10000);
	gpiod_set_value_cansleep(reset, 0);
	fsleep(10000);
	ret = s31_lcd_cmd(lcd, MIPI_DCS_EXIT_SLEEP_MODE, NULL, 0);
	if (ret)
		return ret;
	msleep(120);
	ret = s31_lcd_cmd(lcd, MIPI_DCS_SET_ADDRESS_MODE, &madctl, 1);
	if (ret)
		return ret;
	ret = s31_lcd_cmd(lcd, MIPI_DCS_SET_PIXEL_FORMAT, &rgb565, 1);
	if (ret)
		return ret;
	ret = s31_lcd_cmd(lcd, MIPI_DCS_SET_COLUMN_ADDRESS, cols, sizeof(cols));
	if (ret)
		return ret;
	ret = s31_lcd_cmd(lcd, MIPI_DCS_SET_PAGE_ADDRESS, rows, sizeof(rows));
	if (ret)
		return ret;
	return s31_lcd_cmd(lcd, MIPI_DCS_SET_DISPLAY_ON, NULL, 0);
}

static int s31_lcd_send_frame(struct s31_lcd *lcd, bool queued, dma_addr_t address)
{
	int ret;

	if (!queued) {
		/*
		 * Sv32 PSRAM is cached. Publish live fbcon/mmap pixels to GDMA.
		 * This is a live framebuffer and concurrent rendering may tear.
		 */
		address = lcd->fb_dma;
		dma_sync_single_for_device(lcd->dma_dev, address, S31_LCD_FRAME_BYTES,
					   DMA_TO_DEVICE);
	}

	/* TRM ch.24 LCD_CAM §24.5.2: TRANS_DONE follows the last write strobe. */
	ret = s31_lcd_send_mapped(lcd, address, MIPI_DCS_WRITE_MEMORY_START,
				  S31_LCD_FRAME_BYTES);
	if (queued)
		dma_unmap_single(lcd->dma_dev, address, S31_LCD_FRAME_BYTES,
				 DMA_TO_DEVICE);
	else
		dma_sync_single_for_cpu(lcd->dma_dev, address, S31_LCD_FRAME_BYTES,
					DMA_TO_DEVICE);
	if (!ret)
		atomic_long_inc(&lcd->frames_completed);
	return ret;
}

/*
 * TRM ch.36 SAR ADC — ADC_SAR1_DATA. The ADC provider leaves the
 * hardware weight accumulator enabled (IDF ADC_LL_RAW_DATA_WEIGHTED_SUM).
 * Use nominal zero and slope from IDF's S31 line-fitting calibration.
 */
static int s31_touch_read_mv(struct s31_lcd_touch *touch, unsigned int channel,
			     enum s31_touch_phase phase, int *mv)
{
	int code, ret;

	ret = pinctrl_select_state(touch->pinctrl, touch->phase[phase]);
	if (ret)
		return ret;
	ret = iio_read_channel_raw(touch->sense[channel], &code);
	if (ret < 0)
		return ret;

	if (code < 0 || code > 4393)
		return -ERANGE;
	/* GPIO42 is ADC1 channel 0 (N); GPIO43 is channel 1 (P). */
	code = channel ? code - 2197 : 2197 - code;
	*mv = max(0, DIV_ROUND_CLOSEST(code * 4000, 4393));
	return 0;
}

static int s31_touch_axis(struct s31_lcd_touch *touch, unsigned int sense,
			  enum s31_touch_phase phase, int *value)
{
	int forward, reverse, ret;

	ret = s31_touch_read_mv(touch, sense, phase, &forward);
	if (ret)
		return ret;
	ret = s31_touch_read_mv(touch, sense, phase + 1, &reverse);
	if (ret)
		return ret;

	/*
	 * The pad supply is 3.3 V and the ADC clips near 2 V. In either
	 * drive polarity the lower voltage is within the measurable range.
	 */
	if (forward < 1850 && reverse < 1850)
		*value = (forward + 3300 - reverse) / 2;
	else
		*value = forward < reverse ? forward : 3300 - reverse;
	return 0;
}

static void s31_touch_poll(struct input_dev *input)
{
	struct s31_lcd *lcd = input_get_drvdata(input);
	struct s31_lcd_touch *touch = &lcd->touch;
	int detect, x, y, ret, restore;
	bool down = false;

	mutex_lock(&lcd->lock);
	if (lcd->stopping)
		goto out;

	ret = s31_touch_read_mv(touch, 0, S31_TOUCH_DETECT, &detect);
	if (!ret && detect < 500) {
		ret = s31_touch_axis(touch, 0, S31_TOUCH_X, &x);
		if (!ret)
			ret = s31_touch_axis(touch, 1, S31_TOUCH_Y, &y);
		if (!ret)
			ret = s31_touch_read_mv(touch, 0, S31_TOUCH_DETECT, &detect);
		down = !ret && detect < 500;
	}

	restore = pinctrl_select_state(touch->pinctrl, touch->display);
	if (restore) {
		WRITE_ONCE(lcd->stopping, true);
		dev_err(lcd->dev, "failed to restore LCD pinmux: %d\n", restore);
	}
	if (ret)
		dev_err_ratelimited(lcd->dev, "touch conversion failed: %d\n", ret);
	if (down && !restore) {
		input_report_key(input, BTN_TOUCH, 1);
		touchscreen_report_pos(input, &touch->props,
				       clamp(x, 0, 3300), clamp(y, 0, 3300), false);
		input_sync(input);
	} else if (touch->pressed) {
		input_report_key(input, BTN_TOUCH, 0);
		input_sync(input);
	}
	touch->pressed = down && !restore;
out:
	mutex_unlock(&lcd->lock);
}

static int s31_touch_init(struct s31_lcd *lcd)
{
	static const char * const names[] = {
		"touch-detect", "touch-x", "touch-x-reverse", "touch-y", "touch-y-reverse",
	};
	struct s31_lcd_touch *touch = &lcd->touch;
	struct device *dev = lcd->dev;
	struct input_dev *input;
	unsigned int i;
	int ret;

	if (!device_property_present(dev, "io-channels"))
		return 0;

	touch->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(touch->pinctrl))
		return PTR_ERR(touch->pinctrl);
	touch->display = pinctrl_lookup_state(touch->pinctrl, PINCTRL_STATE_DEFAULT);
	if (IS_ERR(touch->display))
		return PTR_ERR(touch->display);
	for (i = 0; i < ARRAY_SIZE(names); i++) {
		touch->phase[i] = pinctrl_lookup_state(touch->pinctrl, names[i]);
		if (IS_ERR(touch->phase[i]))
			return PTR_ERR(touch->phase[i]);
	}
	touch->sense[0] = devm_iio_channel_get(dev, "gpio42");
	if (IS_ERR(touch->sense[0]))
		return PTR_ERR(touch->sense[0]);
	touch->sense[1] = devm_iio_channel_get(dev, "gpio43");
	if (IS_ERR(touch->sense[1]))
		return PTR_ERR(touch->sense[1]);

	input = input_allocate_device();
	if (!input)
		return -ENOMEM;
	input->name = "HiLetgo ILI9341 resistive touchscreen";
	input->id.bustype = BUS_HOST;
	input->dev.parent = dev;
	input_set_drvdata(input, lcd);
	input_set_capability(input, EV_KEY, BTN_TOUCH);
	__set_bit(INPUT_PROP_DIRECT, input->propbit);
	input_set_abs_params(input, ABS_X, 0, 3300, 12, 0);
	input_set_abs_params(input, ABS_Y, 0, 3300, 12, 0);
	touchscreen_parse_properties(input, false, &touch->props);
	ret = input_setup_polling(input, s31_touch_poll);
	if (ret)
		goto free_input;
	input_set_poll_interval(input, 20);
	ret = input_register_device(input);
	if (ret)
		goto free_input;

	touch->input = input;
	return 0;

free_input:
	input_free_device(input);
	return ret;
}

static void s31_lcd_deferred_io(struct fb_info *info, struct list_head *pagerefs)
{
	struct s31_lcd *lcd = info->par;
	dma_addr_t address;
	bool queued;
	int index, ret;

	mutex_lock(&lcd->lock);
	mutex_lock(&lcd->stage_lock);
	if (lcd->stopping)
		goto unlock;
	index = lcd->pending;
	queued = index >= 0;
	address = lcd->pending_dma;
	if (!queued && !atomic_xchg(&lcd->dirty, 0))
		goto unlock;
	lcd->active = index;
	WRITE_ONCE(lcd->pending, -1);
	mutex_unlock(&lcd->stage_lock);
	wake_up_interruptible(&lcd->stage_wait);

	ret = s31_lcd_send_frame(lcd, queued, address);
	if (ret)
		dev_err_ratelimited(lcd->dev, "LCD_CAM frame transfer failed: %d\n", ret);

	mutex_lock(&lcd->stage_lock);
	lcd->active = -1;
	if (lcd->pending >= 0 || atomic_read(&lcd->dirty))
		schedule_delayed_work(&info->deferred_work, 0);

unlock:
	mutex_unlock(&lcd->stage_lock);
	mutex_unlock(&lcd->lock);
}

static void s31_lcd_damage(struct fb_info *info)
{
	struct s31_lcd *lcd = info->par;

	if (!READ_ONCE(lcd->stopping)) {
		atomic_set(&lcd->dirty, 1);
		schedule_delayed_work(&info->deferred_work, 0);
	}
}

static void s31_lcd_damage_area(struct fb_info *info, u32 x, u32 y, u32 w, u32 h)
{
	if (w && h)
		s31_lcd_damage(info);
}

static ssize_t s31_lcd_write(struct fb_info *info, const char __user *buf,
			     size_t len, loff_t *pos)
{
	struct s31_lcd *lcd = info->par;
	void *buffer;
	dma_addr_t address;
	ssize_t ret;
	int index;

	/*
	 * Keep one completed CPU snapshot queued behind the active DMA.
	 * The worker owns the active buffer until LCD_CAM reports TRANS_DONE.
	 */
	if (!len)
		return 0;

	mutex_lock(&lcd->stage_lock);
	while (lcd->pending >= 0 && !lcd->stopping) {
		mutex_unlock(&lcd->stage_lock);
		ret = wait_event_interruptible(lcd->stage_wait,
					       READ_ONCE(lcd->pending) < 0 ||
					       READ_ONCE(lcd->stopping));
		if (ret)
			return ret;
		mutex_lock(&lcd->stage_lock);
	}
	if (lcd->stopping) {
		ret = -ENODEV;
		goto unlock;
	}

	index = lcd->active == 0 ? 1 : 0;
	buffer = lcd->transfer[index];
	if (!buffer) {
		buffer = alloc_pages_exact(S31_LCD_FRAME_BYTES, GFP_KERNEL);
		if (!buffer) {
			ret = -ENOMEM;
			goto unlock;
		}
		lcd->transfer[index] = buffer;
	}
	if (*pos || len != S31_LCD_FRAME_BYTES) {
		ret = fb_sys_write(info, buf, len, pos);
		if (ret <= 0)
			goto unlock;
		memcpy(buffer, info->screen_buffer, S31_LCD_FRAME_BYTES);
	} else {
		if (copy_from_user(buffer, buf, len)) {
			ret = -EFAULT;
			goto unlock;
		}
		memcpy(info->screen_buffer, buffer, len);
		*pos += len;
		ret = len;
	}
	/*
	 * Sv32 PSRAM is cached. Map the next buffer here so cache writeback
	 * can run while GDMA is reading the active buffer.
	 */
	address = dma_map_single(lcd->dma_dev, buffer, S31_LCD_FRAME_BYTES,
				 DMA_TO_DEVICE);
	if (dma_mapping_error(lcd->dma_dev, address)) {
		ret = -EIO;
		goto unlock;
	}
	lcd->pending_dma = address;
	WRITE_ONCE(lcd->pending, index);
	schedule_delayed_work(&info->deferred_work, 0);
unlock:
	mutex_unlock(&lcd->stage_lock);
	return ret;
}

static int s31_lcd_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	struct s31_lcd *lcd = info->par;
	unsigned long pages = PAGE_ALIGN(S31_LCD_FRAME_BYTES) >> PAGE_SHIFT;

	if (!(vma->vm_flags & VM_SHARED))
		return -EINVAL;
	if (vma->vm_pgoff >= pages || vma_pages(vma) > pages - vma->vm_pgoff)
		return -ENXIO;

	/* The dedicated DMA pages and the CPU mapping have the same cacheability. */
	return remap_pfn_range(vma, vma->vm_start,
			       page_to_pfn(virt_to_page(lcd->frame)) + vma->vm_pgoff,
			       vma->vm_end - vma->vm_start, vma->vm_page_prot);
}

static int s31_lcd_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct s31_lcd *lcd = info->par;
	int ret;

	flush_delayed_work(&info->deferred_work);
	mutex_lock(&lcd->lock);
	if (READ_ONCE(lcd->stopping))
		ret = -ENODEV;
	else
		ret = s31_lcd_send_frame(lcd, false, 0);
	mutex_unlock(&lcd->lock);
	return ret;
}

static int s31_lcd_setcolreg(unsigned int regno, unsigned int r, unsigned int g,
			    unsigned int b, unsigned int transp, struct fb_info *info)
{
	struct s31_lcd *lcd = info->par;

	if (regno >= ARRAY_SIZE(lcd->palette))
		return -EINVAL;
	lcd->palette[regno] = ((r >> 11) << 11) | ((g >> 10) << 5) | (b >> 11);
	return 0;
}

static void s31_lcd_destroy(struct fb_info *info)
{
	struct s31_lcd *lcd = info->par;

	if (info->pagerefs)
		fb_deferred_io_cleanup(info);
	if (lcd->frame) {
		dma_unmap_single(lcd->dma_dev, lcd->fb_dma, S31_LCD_FRAME_BYTES,
				 DMA_TO_DEVICE);
		free_pages_exact(lcd->frame, S31_LCD_FRAME_BYTES);
		put_device(lcd->dma_dev);
	}
	framebuffer_release(info);
}

__FB_GEN_DEFAULT_DEFERRED_OPS_DRAW(s31_lcd, s31_lcd_damage_area, sys)

static const struct fb_ops s31_lcd_ops = {
	.owner = THIS_MODULE,
	.fb_read = fb_sys_read,
	.fb_write = s31_lcd_write,
	__FB_DEFAULT_DEFERRED_OPS_DRAW(s31_lcd),
	.fb_mmap = s31_lcd_mmap,
	.fb_pan_display = s31_lcd_pan_display,
	.fb_setcolreg = s31_lcd_setcolreg,
	.fb_destroy = s31_lcd_destroy,
};

static void s31_lcd_release_hw(struct s31_lcd *lcd)
{
	if (lcd->irq > 0) {
		writel(0, lcd->base + S31_LCD_INT_ENA);
		free_irq(lcd->irq, lcd);
	}
	if (lcd->clocks_on)
		s31_lcd_stop_clocks(lcd);
	if (lcd->transfer[0])
		free_pages_exact(lcd->transfer[0], S31_LCD_FRAME_BYTES);
	if (lcd->transfer[1])
		free_pages_exact(lcd->transfer[1], S31_LCD_FRAME_BYTES);
	if (!IS_ERR_OR_NULL(lcd->tx))
		dma_release_channel(lcd->tx);
}

static int s31_lcd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dma_slave_config dma_config = { };
	struct gpio_desc *reset, *rd;
	struct fb_info *info;
	struct s31_lcd *lcd;
	struct clk *pll;
	u32 wr_hz;
	int irq, ret;

	info = framebuffer_alloc(sizeof(*lcd), dev);
	if (!info)
		return -ENOMEM;
	lcd = info->par;
	lcd->dev = dev;
	mutex_init(&lcd->lock);
	mutex_init(&lcd->stage_lock);
	init_waitqueue_head(&lcd->stage_wait);
	lcd->active = -1;
	lcd->pending = -1;
	atomic_set(&lcd->dirty, 0);
	atomic_long_set(&lcd->frames_completed, 0);
	init_completion(&lcd->tx_done);

	lcd->base = devm_platform_ioremap_resource_byname(pdev, "lcdcam");
	lcd->clkctrl = devm_platform_ioremap_resource_byname(pdev, "clkrst");
	if (IS_ERR(lcd->base) || IS_ERR(lcd->clkctrl)) {
		ret = IS_ERR(lcd->base) ? PTR_ERR(lcd->base) : PTR_ERR(lcd->clkctrl);
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
	ret = device_property_read_u32(dev, "clock-frequency", &wr_hz);
	if (ret)
		goto release;

	rd = devm_gpiod_get(dev, "rd", GPIOD_OUT_LOW);
	if (IS_ERR(rd)) {
		ret = PTR_ERR(rd);
		goto release;
	}
	reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset)) {
		ret = PTR_ERR(reset);
		goto release;
	}
	lcd->tx = dma_request_chan(dev, "tx");
	if (IS_ERR(lcd->tx)) {
		ret = PTR_ERR(lcd->tx);
		goto release;
	}
	dma_config.direction = DMA_MEM_TO_DEV;
	dma_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	ret = dmaengine_slave_config(lcd->tx, &dma_config);
	if (ret)
		goto release;
	lcd->dma_dev = dmaengine_get_dma_device(lcd->tx);
	/*
	 * PSRAM is DMA-addressable on S31. Retain only the pages used by
	 * the LCD frame and keep the streaming mapping for its lifetime.
	 */
	lcd->frame = alloc_pages_exact(S31_LCD_FRAME_BYTES, GFP_KERNEL | __GFP_ZERO);
	if (!lcd->frame) {
		ret = -ENOMEM;
		goto release;
	}
	lcd->fb_dma = dma_map_single(lcd->dma_dev, lcd->frame, S31_LCD_FRAME_BYTES,
				     DMA_TO_DEVICE);
	if (dma_mapping_error(lcd->dma_dev, lcd->fb_dma)) {
		free_pages_exact(lcd->frame, S31_LCD_FRAME_BYTES);
		lcd->frame = NULL;
		ret = -EIO;
		goto release;
	}
	get_device(lcd->dma_dev);
	info->screen_buffer = lcd->frame;
	info->flags = FBINFO_VIRTFB;
	info->fbops = &s31_lcd_ops;
	info->screen_size = S31_LCD_FRAME_BYTES;
	info->pseudo_palette = lcd->palette;
	strscpy(info->fix.id, "ESP32-S31 i80", sizeof(info->fix.id));
	info->fix.smem_len = S31_LCD_FRAME_BYTES;
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.line_length = S31_LCD_STRIDE;
	info->var.xres = info->var.xres_virtual = S31_LCD_WIDTH;
	info->var.yres = info->var.yres_virtual = S31_LCD_HEIGHT;
	info->var.bits_per_pixel = 16;
	info->var.red = (struct fb_bitfield){ 11, 5, 0 };
	info->var.green = (struct fb_bitfield){ 5, 6, 0 };
	info->var.blue = (struct fb_bitfield){ 0, 5, 0 };
	info->var.width = info->var.height = -1;

	ret = s31_lcd_start_clocks(lcd, clk_get_rate(pll), wr_hz);
	if (ret)
		goto release;
	irq = platform_get_irq_optional(pdev, 0);
	if (irq < 0 && irq != -ENXIO) {
		ret = irq;
		goto release;
	}
	writel(0, lcd->base + S31_LCD_INT_ENA);
	writel(S31_LCD_TRANS_DONE, lcd->base + S31_LCD_INT_CLR);
	if (irq > 0) {
		ret = request_irq(irq, s31_lcd_irq, 0, dev_name(dev), lcd);
		if (ret)
			goto release;
		lcd->irq = irq;
		writel(S31_LCD_TRANS_DONE, lcd->base + S31_LCD_INT_ENA);
	}

	ret = s31_lcd_panel_init(lcd, reset);
	if (ret)
		goto release;
	memset(lcd->frame, 0, sizeof(u32));
	if (wr_hz == S31_LCD_MAX_WR_HZ) {
		ret = s31_lcd_set_fast_wr_clock(lcd);
		if (ret)
			goto release;
	}
	ret = s31_lcd_send_frame(lcd, false, 0);
	if (ret)
		goto release;

	lcd->defio.delay = 1;
	lcd->defio.deferred_io = s31_lcd_deferred_io;
	info->fbdefio = &lcd->defio;
	ret = fb_deferred_io_init(info);
	if (ret)
		goto release;
	ret = s31_touch_init(lcd);
	if (ret)
		goto release;
	ret = register_framebuffer(info);
	if (ret)
		goto release;

	platform_set_drvdata(pdev, info);
	return 0;

release:
	if (lcd->touch.input)
		input_unregister_device(lcd->touch.input);
	s31_lcd_release_hw(lcd);
	s31_lcd_destroy(info);
	return dev_err_probe(dev, ret, "failed to initialize ILI9341 framebuffer\n");
}

static void s31_lcd_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);
	struct s31_lcd *lcd = info->par;

	if (lcd->touch.input)
		input_unregister_device(lcd->touch.input);
	mutex_lock(&lcd->lock);
	mutex_lock(&lcd->stage_lock);
	WRITE_ONCE(lcd->stopping, true);
	mutex_unlock(&lcd->stage_lock);
	mutex_unlock(&lcd->lock);
	wake_up_interruptible(&lcd->stage_wait);
	cancel_delayed_work_sync(&info->deferred_work);
	if (lcd->pending >= 0)
		dma_unmap_single(lcd->dma_dev, lcd->pending_dma, S31_LCD_FRAME_BYTES,
				 DMA_TO_DEVICE);
	s31_lcd_release_hw(lcd);
	unregister_framebuffer(info);
}

static ssize_t frames_completed_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct fb_info *info = dev_get_drvdata(dev);
	struct s31_lcd *lcd = info->par;

	return sysfs_emit(buf, "%lu\n", atomic_long_read(&lcd->frames_completed));
}
static DEVICE_ATTR_RO(frames_completed);

static struct attribute *s31_lcd_attrs[] = {
	&dev_attr_frames_completed.attr,
	NULL,
};
ATTRIBUTE_GROUPS(s31_lcd);

static const struct of_device_id s31_lcd_match[] = {
	{ .compatible = "espressif,esp32s31-i80-ili9341" },
	{ }
};
MODULE_DEVICE_TABLE(of, s31_lcd_match);

static struct platform_driver s31_lcd_driver = {
	.probe = s31_lcd_probe,
	.remove = s31_lcd_remove,
	.driver = {
		.name = "esp32s31-i80",
		.of_match_table = s31_lcd_match,
		.dev_groups = s31_lcd_groups,
	},
};
module_platform_driver(s31_lcd_driver);

MODULE_DESCRIPTION("ESP32-S31 LCD_CAM Intel 8080 framebuffer");
MODULE_LICENSE("GPL");
