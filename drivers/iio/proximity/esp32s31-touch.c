// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 14-pad capacitive touch sensor, ESP-IDF oneshot sequence. */

#include <linux/bitfield.h>
#include <linux/iio/iio.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define TOUCH_STATUS			0x10
#define TOUCH_DATA(pad)			(0x14 + (pad) * 4)
#define TOUCH_DATE			0x100
#define TOUCH_MEAS_DONE			BIT(15)
#define TOUCH_AON_SCAN_CTRL1		0x04
#define TOUCH_AON_WORK			0x0c
#define TOUCH_AON_WORK_MEAS		0x10
#define TOUCH_AON_SCAN_CTRL2		0x08
#define TOUCH_AON_FILTER2		0x18
#define TOUCH_AON_FREQ0			0x30
#define TOUCH_AON_MUX0			0x40
#define TOUCH_AON_MUX1			0x44
#define TOUCH_AON_DATE			0xfc
#define TOUCH_SCAN_MAP			GENMASK(16, 2)
#define TOUCH_XPD_WAIT			GENMASK(31, 17)
#define TOUCH_OUT_EN			GENMASK(14, 0)
#define TOUCH_OUT_GATE			BIT(27)
#define TOUCH_OUT_AS_CLOCK		BIT(25)
#define TOUCH_DIV0			GENMASK(24, 22)
#define TOUCH_FREQ_RISE			GENMASK(1, 0)
#define TOUCH_FREQ_SCAN_EN		BIT(27)
#define TOUCH_FREQ_SCAN_LIMIT		GENMASK(29, 28)
#define TOUCH_FREQ_CAP			GENMASK(6, 0)
#define TOUCH_FREQ_RES			GENMASK(8, 7)
#define TOUCH_FREQ_DRV_LS		GENMASK(12, 9)
#define TOUCH_FREQ_DRV_HS		GENMASK(17, 13)
#define TOUCH_FREQ_BIAS			GENMASK(22, 18)
#define TOUCH_FREQ_BUF_EN		BIT(23)
#define TOUCH_DATA_SEL			GENMASK(9, 8)
#define TOUCH_DATA_SMOOTH		3
#define TOUCH_DONE_FORCE		BIT(28)
#define TOUCH_START_EN			BIT(30)
#define TOUCH_START_FORCE		BIT(31)
#define TOUCH_AON_CLK			BIT(31)
#define LP_TOUCH_CLK_EN			BIT(30)
#define LP_TOUCH_RST			BIT(31)

struct esp32s31_touch {
	void __iomem *base;
	void __iomem *aon;
	void __iomem *clkrst;
	struct mutex lock;
};

#define ESP32S31_TOUCH_CH(n) {					\
	.type = IIO_CAPACITANCE, .indexed = 1, .channel = (n),	\
	.address = (n),						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
}

static const struct iio_chan_spec esp32s31_touch_channels[] = {
	ESP32S31_TOUCH_CH(0), ESP32S31_TOUCH_CH(1),
	ESP32S31_TOUCH_CH(2), ESP32S31_TOUCH_CH(3),
	ESP32S31_TOUCH_CH(4), ESP32S31_TOUCH_CH(5),
	ESP32S31_TOUCH_CH(6), ESP32S31_TOUCH_CH(7),
	ESP32S31_TOUCH_CH(8), ESP32S31_TOUCH_CH(9),
	ESP32S31_TOUCH_CH(10), ESP32S31_TOUCH_CH(11),
	ESP32S31_TOUCH_CH(12), ESP32S31_TOUCH_CH(13),
};

static int esp32s31_touch_read_raw(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan,
				   int *val, int *val2, long mask)
{
	struct esp32s31_touch *touch = iio_priv(indio_dev);
	u32 pad_mask = BIT(chan->address), status, mux;
	int ret;

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;
	mutex_lock(&touch->lock);
	writel(FIELD_PREP(TOUCH_SCAN_MAP, pad_mask) |
	       FIELD_PREP(TOUCH_XPD_WAIT, 4096),
	       touch->aon + TOUCH_AON_SCAN_CTRL1);
	writel((readl(touch->aon + TOUCH_AON_FILTER2) & ~TOUCH_OUT_EN) |
	       pad_mask, touch->aon + TOUCH_AON_FILTER2);
	/* Power and start only the selected physical touch pad. */
	writel(pad_mask | (pad_mask << 15), touch->aon + TOUCH_AON_MUX1);
	mux = FIELD_PREP(TOUCH_DATA_SEL, TOUCH_DATA_SMOOTH) |
	      TOUCH_DONE_FORCE | TOUCH_START_FORCE;
	writel(mux | TOUCH_START_EN, touch->aon + TOUCH_AON_MUX0);
	writel(mux, touch->aon + TOUCH_AON_MUX0);
	ret = readl_poll_timeout(touch->base + TOUCH_STATUS, status,
				 status & TOUCH_MEAS_DONE, 0, 100000);
	if (!ret)
		*val = readl(touch->base + TOUCH_DATA(chan->address)) & 0xffff;
	else
		dev_err(indio_dev->dev.parent,
			"timeout: raw=%#x status=%#x scan1=%#x scan2=%#x work=%#x freq0=%#x mux0=%#x mux1=%#x clk=%#x\n",
			readl(touch->base), status,
			readl(touch->aon + TOUCH_AON_SCAN_CTRL1),
			readl(touch->aon + TOUCH_AON_SCAN_CTRL2),
			readl(touch->aon + TOUCH_AON_WORK),
			readl(touch->aon + TOUCH_AON_FREQ0),
			readl(touch->aon + TOUCH_AON_MUX0),
			readl(touch->aon + TOUCH_AON_MUX1),
			readl(touch->clkrst));
	writel(0, touch->aon + TOUCH_AON_MUX1);
	writel(readl(touch->aon + TOUCH_AON_FILTER2) & ~TOUCH_OUT_EN,
	       touch->aon + TOUCH_AON_FILTER2);
	mutex_unlock(&touch->lock);
	return ret ? ret : IIO_VAL_INT;
}

static const struct iio_info esp32s31_touch_info = {
	.read_raw = esp32s31_touch_read_raw,
};

static int esp32s31_touch_probe(struct platform_device *pdev)
{
	struct esp32s31_touch *touch;
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*touch));
	if (!indio_dev)
		return -ENOMEM;
	touch = iio_priv(indio_dev);
	mutex_init(&touch->lock);
	touch->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(touch->base))
		return PTR_ERR(touch->base);
	touch->aon = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(touch->aon))
		return PTR_ERR(touch->aon);
	touch->clkrst = devm_platform_ioremap_resource(pdev, 2);
	if (IS_ERR(touch->clkrst))
		return PTR_ERR(touch->clkrst);

	writel(readl(touch->clkrst) | LP_TOUCH_CLK_EN | LP_TOUCH_RST,
	       touch->clkrst);
	writel(readl(touch->clkrst) & ~LP_TOUCH_RST, touch->clkrst);
	writel(readl(touch->base + TOUCH_DATE) | TOUCH_AON_CLK,
	       touch->base + TOUCH_DATE);
	writel(readl(touch->aon + TOUCH_AON_DATE) | TOUCH_AON_CLK,
	       touch->aon + TOUCH_AON_DATE);
	/* ESP-IDF V3 defaults: 256 us at 16 MHz, divide by 8, 500 cycles. */
	writel(500 | (500 << 10) | (500 << 20),
	       touch->aon + TOUCH_AON_WORK_MEAS);
	writel(TOUCH_OUT_GATE | TOUCH_OUT_AS_CLOCK |
	       FIELD_PREP(TOUCH_DIV0, 7), touch->aon + TOUCH_AON_WORK);
	writel(FIELD_PREP(TOUCH_FREQ_RISE, 1) | TOUCH_FREQ_SCAN_EN |
	       FIELD_PREP(TOUCH_FREQ_SCAN_LIMIT, 1),
	       touch->aon + TOUCH_AON_SCAN_CTRL2);
	writel(FIELD_PREP(TOUCH_FREQ_CAP, 29) |
	       FIELD_PREP(TOUCH_FREQ_RES, 3) |
	       FIELD_PREP(TOUCH_FREQ_DRV_LS, 3) |
	       FIELD_PREP(TOUCH_FREQ_DRV_HS, 8) |
	       FIELD_PREP(TOUCH_FREQ_BIAS, 5) | TOUCH_FREQ_BUF_EN,
	       touch->aon + TOUCH_AON_FREQ0);
	writel(GENMASK(28, 15), touch->aon + TOUCH_AON_FILTER2);

	indio_dev->name = "esp32s31-touch";
	indio_dev->info = &esp32s31_touch_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = esp32s31_touch_channels;
	indio_dev->num_channels = ARRAY_SIZE(esp32s31_touch_channels);
	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static const struct of_device_id esp32s31_touch_of_match[] = {
	{ .compatible = "espressif,esp32s31-touch" }, { }
};
MODULE_DEVICE_TABLE(of, esp32s31_touch_of_match);

static struct platform_driver esp32s31_touch_driver = {
	.probe = esp32s31_touch_probe,
	.driver = {
		.name = "esp32s31-touch",
		.of_match_table = esp32s31_touch_of_match,
	},
};
module_platform_driver(esp32s31_touch_driver);
MODULE_DESCRIPTION("ESP32-S31 capacitive touch sensor");
MODULE_LICENSE("GPL");
