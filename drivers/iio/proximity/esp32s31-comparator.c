// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 three-channel analog comparator (ZERO_DET) IIO events. */

#include <linux/bitfield.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define CMPR_CONF			0x00
#define CMPR_FILTER			0x04
#define CMPR_POLL			0x08
#define CMPR_INT_ENA			0x10
#define CMPR_INT_CLR			0x18
#define CMPR_INT_ST			0x1c
#define CMPR_PAD_CFG			0x3c
#define CMPR_START			0x40
#define CMPR_DATE			0x3fc
#define CMPR_SCAN_MASK			GENMASK(30, 28)
#define CMPR_DREF			GENMASK(6, 4)
#define CMPR_HYSTERESIS			BIT(3)
#define CMPR_POWER			BIT(8)
#define CMPR_REG_CLK			BIT(28)
#define CMPR_CLK_APB			BIT(0)
#define CMPR_RST_APB			BIT(1)
#define CMPR_CLK_FUNC			BIT(2)
#define CMPR_RST_CORE			BIT(3)
#define CMPR_FORCE_NORST		BIT(4)
#define CMPR_CLK_DIV			GENMASK(14, 7)

struct esp32s31_cmpr {
	void __iomem *base;
};

static const struct iio_event_spec esp32s31_cmpr_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	},
};

#define ESP32S31_CMPR_CH(n) {					\
	.type = IIO_VOLTAGE, .indexed = 1, .channel = (n),	\
	.event_spec = esp32s31_cmpr_events,			\
	.num_event_specs = ARRAY_SIZE(esp32s31_cmpr_events),	\
}

static const struct iio_chan_spec esp32s31_cmpr_channels[] = {
	ESP32S31_CMPR_CH(0), ESP32S31_CMPR_CH(1), ESP32S31_CMPR_CH(2),
};

static u32 esp32s31_cmpr_event_bit(unsigned int channel,
				   enum iio_event_direction dir)
{
	unsigned int base = (2 - channel) * 3;

	return BIT(base + (dir == IIO_EV_DIR_RISING));
}

static int esp32s31_cmpr_read_event_config(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *chan,
					   enum iio_event_type type,
					   enum iio_event_direction dir)
{
	struct esp32s31_cmpr *cmpr = iio_priv(indio_dev);

	return !!(readl(cmpr->base + CMPR_INT_ENA) &
		  esp32s31_cmpr_event_bit(chan->channel, dir));
}

static int esp32s31_cmpr_write_event_config(struct iio_dev *indio_dev,
					    const struct iio_chan_spec *chan,
					    enum iio_event_type type,
					    enum iio_event_direction dir,
					    bool state)
{
	struct esp32s31_cmpr *cmpr = iio_priv(indio_dev);
	u32 bit = esp32s31_cmpr_event_bit(chan->channel, dir);
	u32 val = readl(cmpr->base + CMPR_INT_ENA);

	writel(state ? val | bit : val & ~bit, cmpr->base + CMPR_INT_ENA);
	return 0;
}

static const struct iio_info esp32s31_cmpr_info = {
	.read_event_config = esp32s31_cmpr_read_event_config,
	.write_event_config = esp32s31_cmpr_write_event_config,
};

static irqreturn_t esp32s31_cmpr_irq(int irq, void *data)
{
	struct iio_dev *indio_dev = data;
	struct esp32s31_cmpr *cmpr = iio_priv(indio_dev);
	u32 status = readl(cmpr->base + CMPR_INT_ST) & 0x1ff;
	unsigned int ch;

	if (!status)
		return IRQ_NONE;
	writel(status, cmpr->base + CMPR_INT_CLR);
	for (ch = 0; ch < 3; ch++) {
		if (status & esp32s31_cmpr_event_bit(ch, IIO_EV_DIR_RISING))
			iio_push_event(indio_dev,
				IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE, ch,
					IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING),
				iio_get_time_ns(indio_dev));
		if (status & esp32s31_cmpr_event_bit(ch, IIO_EV_DIR_FALLING))
			iio_push_event(indio_dev,
				IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE, ch,
					IIO_EV_TYPE_THRESH, IIO_EV_DIR_FALLING),
				iio_get_time_ns(indio_dev));
	}
	return IRQ_HANDLED;
}

static int esp32s31_cmpr_probe(struct platform_device *pdev)
{
	struct esp32s31_cmpr *cmpr;
	struct iio_dev *indio_dev;
	void __iomem *clkrst;
	u32 reference = 3, val;
	int irq, ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*cmpr));
	if (!indio_dev)
		return -ENOMEM;
	cmpr = iio_priv(indio_dev);
	cmpr->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cmpr->base))
		return PTR_ERR(cmpr->base);
	clkrst = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(clkrst))
		return PTR_ERR(clkrst);
	of_property_read_u32(pdev->dev.of_node, "espressif,reference-level",
			     &reference);
	if (reference > 7)
		return -EINVAL;

	val = CMPR_CLK_APB | CMPR_CLK_FUNC | CMPR_FORCE_NORST |
	      FIELD_PREP(CMPR_CLK_DIV, 39);
	writel(val | CMPR_RST_APB | CMPR_RST_CORE, clkrst);
	writel(val, clkrst);
	writel(readl(cmpr->base + CMPR_DATE) | CMPR_REG_CLK,
	       cmpr->base + CMPR_DATE);
	/* Internal reference, pads 0/1/2 as independent source channels. */
	writel((BIT(3)) | (BIT(2) << 4) | (BIT(1) << 8) |
	       (BIT(0) << 12) | FIELD_PREP(CMPR_SCAN_MASK, 7),
	       cmpr->base + CMPR_CONF);
	writel(255, cmpr->base + CMPR_FILTER);
	writel(15, cmpr->base + CMPR_POLL);
	writel(CMPR_POWER | CMPR_HYSTERESIS |
	       FIELD_PREP(CMPR_DREF, reference), cmpr->base + CMPR_PAD_CFG);
	writel(1, cmpr->base + CMPR_START);

	indio_dev->name = "esp32s31-comparator";
	indio_dev->info = &esp32s31_cmpr_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = esp32s31_cmpr_channels;
	indio_dev->num_channels = ARRAY_SIZE(esp32s31_cmpr_channels);
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ret = devm_request_irq(&pdev->dev, irq, esp32s31_cmpr_irq, 0,
			       dev_name(&pdev->dev), indio_dev);
	if (ret)
		return ret;
	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static const struct of_device_id esp32s31_cmpr_of_match[] = {
	{ .compatible = "espressif,esp32s31-comparator" }, { }
};
MODULE_DEVICE_TABLE(of, esp32s31_cmpr_of_match);

static struct platform_driver esp32s31_cmpr_driver = {
	.probe = esp32s31_cmpr_probe,
	.driver = {
		.name = "esp32s31-comparator",
		.of_match_table = esp32s31_cmpr_of_match,
	},
};
module_platform_driver(esp32s31_cmpr_driver);
MODULE_DESCRIPTION("ESP32-S31 analog comparator");
MODULE_LICENSE("GPL");
