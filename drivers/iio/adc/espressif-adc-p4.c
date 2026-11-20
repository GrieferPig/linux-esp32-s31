// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-P4 low-power SAR ADC one-shot conversions. */

#include <linux/bitfield.h>
#include <linux/iio/iio.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/soc/espressif/esp32p4-analog.h>

/* TRM ch.67 ADC Controller — LPADC_MEASx_CTRL2, MEASx_MUX and ATTENx. */
#define ADC_SAMPLE_RESULT	GENMASK(11, 0)
#define ADC_SAMPLE_DONE		BIT(16)
#define ADC_SAMPLE_START	BIT(17)
#define ADC_START_FORCE		BIT(18)
#define ADC_PAD_SELECT		GENMASK(30, 19)
#define ADC_PAD_FORCE		BIT(31)
#define ADC_MUX_FORCE		BIT(31)
#define ADC_READER_DIV		GENMASK(7, 0)
#define ADC_READER_DIV_8	8

/*
 * TRM ch.67 ADC Controller and ch.49 Analog I2C Controller.
 * The internal register fields are specified by the P4
 * adc_ll_calibration_init() and adc_ll_set_calibration_param().
 */
#define ADC_INIT_HIGH		GENMASK(3, 0)
#define ADC_DREF_MASK		GENMASK(6, 4)
#define ADC_DREF_CODE		4
#define ADC_OFFSET_BIAS		1400
#define ADC_OFFSET_MASK		GENMASK(9, 0)

#define ADC_CHANNELS		14
#define ADC1_CHANNELS		8

/* TRM ch.12 Reset and Clock — LPPERI_CLK_EN, RESET_EN and ADC_CTRL. */
#define LP_CLOCK_EN		0x00
#define LP_RESET_EN		0x08
#define LP_ADC_CLK_DIV		0x2c
#define LP_ADC_GATE		BIT(23)
#define LP_ADC_RESET		BIT(24)
#define LP_ADC_DIV_MASK		GENMASK(31, 8)
#define LP_ADC1_DIV		GENMASK(31, 24)
#define LP_ADC2_DIV		GENMASK(23, 16)
#define LP_ADC_FUNC_DIV		GENMASK(15, 8)

struct p4_adc_unit {
	u8 reader;
	u8 measure;
	u8 mux;
	u8 atten;
	u8 pad_shift;
	u8 analog_offset;
	u32 mux_select;
};

static const struct p4_adc_unit p4_adc_units[] = {
	{ .reader = 0x00, .measure = 0x0c, .mux = 0x10, .atten = 0x14 },
	{ .reader = 0x24, .measure = 0x30, .mux = 0x34, .atten = 0x38,
	  .pad_shift = 2, .analog_offset = 3, .mux_select = ADC_MUX_FORCE },
};

struct p4_adc {
	struct device *dev;
	void __iomem *base;
	struct regmap *lpperi;
	struct mutex lock;
};

static int p4_adc_read_raw(struct iio_dev *indio_dev,
			  const struct iio_chan_spec *channel,
			  int *val, int *val2, long mask)
{
	const struct p4_adc_unit *unit;
	struct p4_adc *adc = iio_priv(indio_dev);
	void __iomem *measure;
	unsigned int input = channel->channel;
	u32 start, raw, attenuation;
	int ret;

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;
	if (input < ADC1_CHANNELS) {
		unit = &p4_adc_units[0];
	} else {
		unit = &p4_adc_units[1];
		input -= ADC1_CHANNELS;
	}
	input += unit->pad_shift;
	measure = adc->base + unit->measure;
	start = ADC_PAD_FORCE | ADC_START_FORCE |
		FIELD_PREP(ADC_PAD_SELECT, BIT(input));

	mutex_lock(&adc->lock);
	/* Use 0 dB and leave every other pad's attenuation unchanged. */
	attenuation = readl(adc->base + unit->atten);
	writel(attenuation & ~(3U << (input * 2)), adc->base + unit->atten);
	writel(start, measure);
	writel(start | ADC_SAMPLE_START, measure);
	ret = readl_poll_timeout(measure, raw, raw & ADC_SAMPLE_DONE, 0, 1000);
	writel(0, measure);
	mutex_unlock(&adc->lock);

	if (ret)
		return ret;
	*val = FIELD_GET(ADC_SAMPLE_RESULT, raw);
	return IIO_VAL_INT;
}

static const struct iio_info p4_adc_info = {
	.read_raw = p4_adc_read_raw,
};

static void p4_adc_disable(void *data)
{
	struct p4_adc *adc = data;
	int ret;

	ret = regmap_clear_bits(adc->lpperi, LP_CLOCK_EN, LP_ADC_GATE);
	if (ret)
		dev_err(adc->dev, "failed to disable LP ADC clock: %d\n", ret);
}

static int p4_adc_configure_offset(struct device *dev, unsigned long channels)
{
	static const char * const cells[] = { "adc1-offset", "adc2-offset" };
	struct regmap *analog;
	unsigned int i;
	u8 version;
	u16 code;
	int ret;

	if (!of_property_present(dev->of_node, "espressif,analog-i2c"))
		return 0;
	analog = esp32p4_analog_get(dev);
	if (IS_ERR(analog))
		return PTR_ERR(analog);

	/* TRM ch.10 eFuse Controller — BLOCK1 and BLOCK2 read shadows.
	 * P4 esp_efuse_rtc_calib_get_ver() and _get_init_code() describe
	 * the chip-specific offset encoding for nonzero block versions.
	 */
	ret = nvmem_cell_read_u8(dev, "otp-version", &version);
	if (ret)
		return ret;
	if (!version)
		return -ENODATA;
	for (i = 0; i < ARRAY_SIZE(p4_adc_units); i++) {
		const struct p4_adc_unit *unit = &p4_adc_units[i];

		if (!(channels & (GENMASK(7, 0) << (ADC1_CHANNELS * i))))
			continue;
		ret = nvmem_cell_read_u16(dev, cells[i], &code);
		if (ret)
			return ret;
		if (code > ADC_OFFSET_MASK)
			return -ERANGE;
		code += ADC_OFFSET_BIAS;
		ret = regmap_update_bits(analog, unit->analog_offset + 2, ADC_DREF_MASK,
					 FIELD_PREP(ADC_DREF_MASK, ADC_DREF_CODE));
		if (ret)
			return ret;
		ret = regmap_write(analog, unit->analog_offset, code & 0xff);
		if (ret)
			return ret;
		ret = regmap_update_bits(analog, unit->analog_offset + 1,
					 ADC_INIT_HIGH, code >> 8);
		if (ret)
			return ret;
	}
	return 0;
}

static int p4_adc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_chan_spec *channels;
	struct iio_dev *indio_dev;
	struct p4_adc *adc;
	int i;
	unsigned long seen = 0;
	int count, ret;
	u32 input;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;
	adc = iio_priv(indio_dev);
	adc->dev = dev;
	mutex_init(&adc->lock);

	count = of_property_count_u32_elems(dev->of_node, "espressif,adc-inputs");
	if (count < 1 || count > ADC_CHANNELS)
		return dev_err_probe(dev, -EINVAL, "invalid ADC input list\n");
	channels = devm_kcalloc(dev, count, sizeof(*channels), GFP_KERNEL);
	if (!channels)
		return -ENOMEM;
	for (i = 0; i < count; i++) {
		ret = of_property_read_u32_index(dev->of_node, "espressif,adc-inputs",
						 i, &input);
		if (ret)
			return ret;
		if (input >= ADC_CHANNELS || test_and_set_bit(input, &seen))
			return dev_err_probe(dev, -EINVAL, "invalid ADC input %u\n", input);
		channels[i] = (struct iio_chan_spec) {
			.type = IIO_VOLTAGE,
			.indexed = 1,
			.channel = input,
			.scan_index = -1,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		};
	}

	adc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(adc->base))
		return PTR_ERR(adc->base);
	adc->lpperi = syscon_regmap_lookup_by_phandle(dev->of_node, "espressif,lpperi");
	if (IS_ERR(adc->lpperi))
		return PTR_ERR(adc->lpperi);
	ret = devm_regulator_get_enable(dev, "analog");
	if (ret)
		return dev_err_probe(dev, ret, "analog supply unavailable\n");

	ret = regmap_set_bits(adc->lpperi, LP_CLOCK_EN, LP_ADC_GATE);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, p4_adc_disable, adc);
	if (ret)
		return ret;
	ret = regmap_set_bits(adc->lpperi, LP_RESET_EN, LP_ADC_RESET);
	if (ret)
		return ret;
	ret = regmap_clear_bits(adc->lpperi, LP_RESET_EN, LP_ADC_RESET);
	if (ret)
		return ret;

	/* The input clock is at most 40 MHz. The SAR clocks must be <= 5 MHz. */
	ret = regmap_update_bits(adc->lpperi, LP_ADC_CLK_DIV, LP_ADC_DIV_MASK,
				 FIELD_PREP(LP_ADC1_DIV, 4) |
				 FIELD_PREP(LP_ADC2_DIV, 4) |
				 FIELD_PREP(LP_ADC_FUNC_DIV, 4));
	if (ret)
		return ret;

	/* ADC1's bit selects the digital controller. ADC2's selects LP. */
	for (i = 0; i < ARRAY_SIZE(p4_adc_units); i++) {
		const struct p4_adc_unit *unit = &p4_adc_units[i];
		u32 value = readl(adc->base + unit->mux);

		writel((value & ~ADC_MUX_FORCE) | unit->mux_select,
		       adc->base + unit->mux);
		value = readl(adc->base + unit->reader);
		writel((value & ~ADC_READER_DIV) |
		       FIELD_PREP(ADC_READER_DIV, ADC_READER_DIV_8),
		       adc->base + unit->reader);
	}
	ret = p4_adc_configure_offset(dev, seen);
	if (ret)
		return dev_err_probe(dev, ret, "ADC offset calibration failed\n");

	indio_dev->name = "esp32p4-lp-adc";
	indio_dev->info = &p4_adc_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = channels;
	indio_dev->num_channels = count;
	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id p4_adc_of_match[] = {
	{ .compatible = "espressif,esp32p4-lp-adc" },
	{ }
};
MODULE_DEVICE_TABLE(of, p4_adc_of_match);

static struct platform_driver p4_adc_driver = {
	.probe = p4_adc_probe,
	.driver = {
		.name = "esp32p4-lp-adc",
		.of_match_table = p4_adc_of_match,
	},
};
module_platform_driver(p4_adc_driver);

MODULE_DESCRIPTION("Espressif ESP32-P4 low-power SAR ADC driver");
MODULE_LICENSE("GPL");
