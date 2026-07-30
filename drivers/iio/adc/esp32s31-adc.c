// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 dual 8-channel SAR ADC, following ESP-IDF adc_ll oneshot flow. */

#include <linux/bitfield.h>
#include <linux/iio/iio.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define ESP32S31_PMU_ANA_PWR		0x20704208
#define PMU_XPD_PERIF_I2C		BIT(30)
#define PMU_RSTB_PERIF_I2C		BIT(31)
#define ADC_CTRL(u)			((u) * 4)
#define ADC_PATTERN(u)			(0x1c + (u) * 0x10)
#define ADC_DATA(u)			(0x40 + (u) * 4)
#define ADC_CTRL2			0x08
#define ADC_INT_RAW			0x68
#define ADC_INT_CLR			0x70
#define ADC_REF_CONTROL			0x78
#define ADC_CTRL_DATE			0x3fc
#define ADC_TRIGGER_STOP		BIT(0)
#define ADC_TRIGGER_START		BIT(1)
#define ADC_TRIGGER_MODE		GENMASK(3, 2)
#define ADC_TRIGGER_SW			2
#define ADC_PATTERN_LEN			GENMASK(17, 14)
#define ADC_PATTERN_CLEAR		BIT(22)
#define ADC_PATTERN_TYPE		BIT(23)
#define ADC_CONTINUE_MODE		BIT(25)
#define ADC_POWER			GENMASK(27, 26)
#define ADC_POWER_UP			3
#define ADC_DONE(u)			BIT(31 - (u))
#define ADC_REF_DELAY			BIT(29)
#define ADC_REF_PRECHARGE		BIT(30)
#define ADC_REF_POWER			BIT(31)
#define ADC_BLOCK_CLK			BIT(31)
#define ADC_TIMER_EN			BIT(24)
#define LP_ADC_DIV			GENMASK(7, 0)
#define LP_ADC_CLK_SEL			GENMASK(29, 28)
#define LP_ADC_CLK_XTAL			1
#define LP_ADC_CLK_EN			BIT(30)
#define LP_ADC_RST			BIT(31)

struct esp32s31_adc {
	void __iomem *base;
	void __iomem *clkrst;
	struct mutex lock;
};

#define ESP32S31_ADC_CH(u, c) {						\
	.type = IIO_VOLTAGE, .indexed = 1, .channel = (u) * 8 + (c),	\
	.address = ((u) << 8) | (c),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),		\
}

static const struct iio_chan_spec esp32s31_adc_channels[] = {
	ESP32S31_ADC_CH(0, 0), ESP32S31_ADC_CH(0, 1),
	ESP32S31_ADC_CH(0, 2), ESP32S31_ADC_CH(0, 3),
	ESP32S31_ADC_CH(0, 4), ESP32S31_ADC_CH(0, 5),
	ESP32S31_ADC_CH(0, 6), ESP32S31_ADC_CH(0, 7),
	ESP32S31_ADC_CH(1, 0), ESP32S31_ADC_CH(1, 1),
	ESP32S31_ADC_CH(1, 2), ESP32S31_ADC_CH(1, 3),
	ESP32S31_ADC_CH(1, 4), ESP32S31_ADC_CH(1, 5),
	ESP32S31_ADC_CH(1, 6), ESP32S31_ADC_CH(1, 7),
};

static int esp32s31_adc_read_raw(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 int *val, int *val2, long mask)
{
	struct esp32s31_adc *adc = iio_priv(indio_dev);
	unsigned int unit = chan->address >> 8;
	unsigned int channel = chan->address & 0xff;
	u32 ctrl, status;
	int ret;

	if (mask == IIO_CHAN_INFO_SCALE) {
		*val = 1100;
		*val2 = 17;
		return IIO_VAL_FRACTIONAL_LOG2;
	}
	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;

	mutex_lock(&adc->lock);
	ctrl = readl(adc->base + ADC_CTRL(unit));
	ctrl &= ~(ADC_TRIGGER_MODE | ADC_PATTERN_LEN | ADC_CONTINUE_MODE |
		  ADC_POWER);
	ctrl |= FIELD_PREP(ADC_TRIGGER_MODE, ADC_TRIGGER_SW) |
		FIELD_PREP(ADC_POWER, ADC_POWER_UP) | ADC_PATTERN_TYPE;
	writel(ctrl | ADC_TRIGGER_STOP, adc->base + ADC_CTRL(unit));
	writel((channel & 0xf) << 20, adc->base + ADC_PATTERN(unit));
	writel(ctrl | ADC_PATTERN_CLEAR, adc->base + ADC_CTRL(unit));
	writel(ctrl, adc->base + ADC_CTRL(unit));
	writel(ADC_DONE(unit), adc->base + ADC_INT_CLR);
	/*
	 * Match adc_oneshot_ll_enable() followed by adc_oneshot_ll_start():
	 * trigger_stop remains asserted when the one-shot start pulse is set.
	 */
	writel(ctrl | ADC_TRIGGER_STOP, adc->base + ADC_CTRL(unit));
	writel(ctrl | ADC_TRIGGER_STOP | ADC_TRIGGER_START,
	       adc->base + ADC_CTRL(unit));
	ret = readl_poll_timeout(adc->base + ADC_INT_RAW, status,
				 status & ADC_DONE(unit), 1, 10000);
	if (!ret)
		*val = readl(adc->base + ADC_DATA(unit)) & GENMASK(16, 0);
	writel(ADC_DONE(unit), adc->base + ADC_INT_CLR);
	writel((ctrl & ~ADC_TRIGGER_MODE) | ADC_TRIGGER_STOP,
	       adc->base + ADC_CTRL(unit));
	mutex_unlock(&adc->lock);
	return ret ? ret : IIO_VAL_INT;
}

static const struct iio_info esp32s31_adc_info = {
	.read_raw = esp32s31_adc_read_raw,
};

static int esp32s31_adc_probe(struct platform_device *pdev)
{
	struct esp32s31_adc *adc;
	struct iio_dev *indio_dev;
	void __iomem *ana_pwr;
	u32 val;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;
	adc = iio_priv(indio_dev);
	mutex_init(&adc->lock);
	adc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(adc->base))
		return PTR_ERR(adc->base);
	adc->clkrst = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(adc->clkrst))
		return PTR_ERR(adc->clkrst);

	/*
	 * IDF regi2c_saradc_enable(): release the analog SAR/TSENS I2C
	 * power-domain reset.  It is shared and intentionally left enabled.
	 */
	ana_pwr = devm_ioremap(&pdev->dev, ESP32S31_PMU_ANA_PWR, 4);
	if (!ana_pwr)
		return -ENOMEM;
	val = readl(ana_pwr);
	writel(val & ~PMU_RSTB_PERIF_I2C, ana_pwr);
	udelay(1);
	writel(val | PMU_XPD_PERIF_I2C | PMU_RSTB_PERIF_I2C, ana_pwr);

	val = FIELD_PREP(LP_ADC_DIV, 4) |
	      FIELD_PREP(LP_ADC_CLK_SEL, LP_ADC_CLK_XTAL) | LP_ADC_CLK_EN;
	writel(val | LP_ADC_RST, adc->clkrst);
	writel(val, adc->clkrst);
	writel(readl(adc->base + ADC_CTRL_DATE) | ADC_BLOCK_CLK,
	       adc->base + ADC_CTRL_DATE);
	writel(readl(adc->base + ADC_CTRL2) & ~ADC_TIMER_EN,
	       adc->base + ADC_CTRL2);
	writel(ADC_REF_POWER | ADC_REF_PRECHARGE | ADC_REF_DELAY,
	       adc->base + ADC_REF_CONTROL);

	indio_dev->name = "esp32s31-adc";
	indio_dev->info = &esp32s31_adc_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = esp32s31_adc_channels;
	indio_dev->num_channels = ARRAY_SIZE(esp32s31_adc_channels);
	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static const struct of_device_id esp32s31_adc_of_match[] = {
	{ .compatible = "espressif,esp32s31-adc" }, { }
};
MODULE_DEVICE_TABLE(of, esp32s31_adc_of_match);

static struct platform_driver esp32s31_adc_driver = {
	.probe = esp32s31_adc_probe,
	.driver = {
		.name = "esp32s31-adc",
		.of_match_table = esp32s31_adc_of_match,
	},
};
module_platform_driver(esp32s31_adc_driver);
MODULE_DESCRIPTION("ESP32-S31 SAR ADC driver");
MODULE_LICENSE("GPL");
