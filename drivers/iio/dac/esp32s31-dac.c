// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 dual-channel 12-bit low-power DAC. */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/iio.h>

#define DAC_PAD_CFG		0x00
#define DAC_DATA_OUTPUT_CFG	0x10
#define DAC_SINTX_CFG		0x14
#define DAC_SINTX_DATA		0x1c
#define DAC_DATE		0x3fc

#define DAC_PAD_POWER(ch)	BIT(1 + (ch) * 7)
#define DAC_PAD_BUFFER(ch)	BIT(5 + (ch) * 7)
#define DAC_PAD_FULL_RANGE(ch)	BIT(30 + (ch))
#define DAC_DC_MASK(ch)		(GENMASK(11, 0) << ((ch) * 12))
#define DAC_CLK_EN		BIT(31)

#define LP_DAC_CLK_EN		BIT(30)
#define LP_DAC_RST		BIT(31)

struct esp32s31_dac {
	void __iomem *base;
	void __iomem *clkrst;
	struct mutex lock;
	u16 value[2];
};

#define ESP32S31_DAC_CHANNEL(_channel) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.output = 1, \
	.channel = (_channel), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.scan_type = { .sign = 'u', .realbits = 12, .storagebits = 16 }, \
}

static const struct iio_chan_spec esp32s31_dac_channels[] = {
	ESP32S31_DAC_CHANNEL(0),
	ESP32S31_DAC_CHANNEL(1),
};

static int esp32s31_dac_read_raw(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 int *val, int *val2, long mask)
{
	struct esp32s31_dac *dac = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		*val = dac->value[chan->channel];
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int esp32s31_dac_write_raw(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan,
				  int val, int val2, long mask)
{
	struct esp32s31_dac *dac = iio_priv(indio_dev);
	u32 cfg, data;
	unsigned int shift = chan->channel * 12;

	if (mask != IIO_CHAN_INFO_RAW || val2 || val < 0 || val > 0xfff)
		return -EINVAL;

	mutex_lock(&dac->lock);
	data = readl(dac->base + DAC_SINTX_DATA);
	data &= ~DAC_DC_MASK(chan->channel);
	data |= val << shift;
	writel(data, dac->base + DAC_SINTX_DATA);
	/* DC path, buffered full-range output; writing RAW powers the pad on. */
	writel(readl(dac->base + DAC_DATA_OUTPUT_CFG) & ~BIT(chan->channel),
	       dac->base + DAC_DATA_OUTPUT_CFG);
	cfg = readl(dac->base + DAC_PAD_CFG);
	cfg |= DAC_PAD_POWER(chan->channel) |
	       DAC_PAD_BUFFER(chan->channel) |
	       DAC_PAD_FULL_RANGE(chan->channel);
	writel(cfg, dac->base + DAC_PAD_CFG);
	dac->value[chan->channel] = val;
	mutex_unlock(&dac->lock);
	return 0;
}

static const struct iio_info esp32s31_dac_info = {
	.read_raw = esp32s31_dac_read_raw,
	.write_raw = esp32s31_dac_write_raw,
};

static int esp32s31_dac_probe(struct platform_device *pdev)
{
	struct esp32s31_dac *dac;
	struct iio_dev *indio_dev;
	u32 clock;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*dac));
	if (!indio_dev)
		return -ENOMEM;
	dac = iio_priv(indio_dev);
	mutex_init(&dac->lock);
	dac->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dac->base))
		return PTR_ERR(dac->base);
	dac->clkrst = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(dac->clkrst))
		return PTR_ERR(dac->clkrst);

	ret = devm_regulator_get_enable(&pdev->dev, "analog");
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "analog power unavailable\n");

	/* Preserve the boot-selected source/divider; only gate and reset here. */
	clock = readl(dac->clkrst) | LP_DAC_CLK_EN;
	writel(clock | LP_DAC_RST, dac->clkrst);
	writel(clock, dac->clkrst);
	writel(readl(dac->base + DAC_DATE) | DAC_CLK_EN, dac->base + DAC_DATE);
	writel(0, dac->base + DAC_SINTX_CFG);
	writel(0, dac->base + DAC_DATA_OUTPUT_CFG);
	writel(0, dac->base + DAC_SINTX_DATA);
	/* Leave both analog outputs powered down until userspace writes RAW. */
	writel(readl(dac->base + DAC_PAD_CFG) &
	       ~(DAC_PAD_POWER(0) | DAC_PAD_BUFFER(0) |
		 DAC_PAD_POWER(1) | DAC_PAD_BUFFER(1)),
	       dac->base + DAC_PAD_CFG);

	indio_dev->name = "esp32s31-dac";
	indio_dev->info = &esp32s31_dac_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = esp32s31_dac_channels;
	indio_dev->num_channels = ARRAY_SIZE(esp32s31_dac_channels);
	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static const struct of_device_id esp32s31_dac_of_match[] = {
	{ .compatible = "espressif,esp32s31-dac" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s31_dac_of_match);

static struct platform_driver esp32s31_dac_driver = {
	.probe = esp32s31_dac_probe,
	.driver = {
		.name = "esp32s31-dac",
		.of_match_table = esp32s31_dac_of_match,
	},
};
module_platform_driver(esp32s31_dac_driver);

MODULE_DESCRIPTION("ESP32-S31 dual-channel DAC");
MODULE_LICENSE("GPL");
