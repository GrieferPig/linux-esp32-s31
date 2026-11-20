// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-P4 LP one-shot conversions and HP SAR ADC scans. */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
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
#include <linux/reset.h>
#include <linux/soc/espressif/esp32p4-analog.h>

#include "../../dma/espressif-gdma-ahb.h"

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

/* TRM ch.67 ADC Controller — ADC_CTRL, CTRL2, FSM_WAIT and DMA_CONF. */
#define HP_ADC_CTRL		0x00
#define HP_ADC_CLK_GATE		BIT(5)
#define HP_ADC_SAR_DIV		GENMASK(13, 6)
#define HP_ADC_PATTERN_LEN	GENMASK(17, 14)
#define HP_ADC_PATTERN_RESET	BIT(22)
#define HP_ADC_DATA_TAG		BIT(24)
#define HP_ADC_CTRL2		0x04
#define HP_ADC_TIMER_SELECT	BIT(11)
#define HP_ADC_TIMER_TARGET	GENMASK(23, 12)
#define HP_ADC_TIMER_ENABLE	BIT(24)
#define HP_ADC_FSM_WAIT		0x0c
#define HP_ADC_STANDBY_WAIT	GENMASK(23, 16)
#define HP_ADC_RESET_WAIT	GENMASK(15, 8)
#define HP_ADC_POWER_WAIT	GENMASK(7, 0)
#define HP_ADC_PATTERN		0x18
#define HP_ADC_PATTERN_BITS	6
#define HP_ADC_PATTERN_SIZE	4
#define HP_ADC_DMA_CONF		0x60
#define HP_ADC_DMA_ENABLE	BIT(31)
#define HP_ADC_FSM_RESET	BIT(30)
#define HP_ADC_DMA_EOF		GENMASK(15, 0)
#define HP_ADC_SAMPLE_UNIT	BIT(17)
#define HP_ADC_SAMPLE_CHAN	GENMASK(16, 13)

#define HP_ADC_SCANS_PER_SEC	1000
#define HP_ADC_SCAN_DELAY_MIN	30
#define HP_ADC_PERIOD_SAMPLES	256
#define HP_ADC_PERIOD_BYTES	(HP_ADC_PERIOD_SAMPLES * sizeof(__le32))
#define HP_ADC_RING_BYTES	(4 * HP_ADC_PERIOD_BYTES)

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
	void __iomem *hp;
	struct dma_chan *rx;
	__le32 *ring;
	dma_addr_t ring_dma;
	unsigned long hp_rate;
	u8 scan_channels[ADC1_CHANNELS];
	u8 scan_count;
	u8 scan_pos;
	u16 scan[ADC1_CHANNELS];
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
	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;
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
	iio_device_release_direct(indio_dev);

	if (ret)
		return ret;
	*val = FIELD_GET(ADC_SAMPLE_RESULT, raw);
	return IIO_VAL_INT;
}

static void p4_adc_hp_select(struct p4_adc *adc, bool enable)
{
	void __iomem *mux = adc->base + p4_adc_units[0].mux;
	u32 value = readl(mux);

	writel(enable ? value | ADC_MUX_FORCE : value & ~ADC_MUX_FORCE, mux);
}

static void p4_adc_dma_complete(void *data, const struct dmaengine_result *res)
{
	struct iio_dev *indio_dev = data;
	struct p4_adc *adc = iio_priv(indio_dev);
	const struct esp32s31_ahb_rx_result *rx;
	unsigned int i, count, pos;

	if (res->result != DMA_TRANS_NOERROR || res->residue > HP_ADC_RING_BYTES) {
		dev_err_ratelimited(adc->dev, "ADC DMA error %u, residue %u\n",
				    res->result, res->residue);
		adc->scan_pos = 0;
		return;
	}
	rx = container_of(res, struct esp32s31_ahb_rx_result, res);
	count = HP_ADC_RING_BYTES - res->residue;
	if (!IS_ALIGNED(count | rx->pos, sizeof(*adc->ring)) ||
	    rx->pos >= HP_ADC_RING_BYTES) {
		dev_err_ratelimited(adc->dev, "invalid ADC DMA span %u + %u\n",
				    rx->pos, count);
		adc->scan_pos = 0;
		return;
	}

	dma_rmb();
	for (i = 0; i < count; i += sizeof(*adc->ring)) {
		u32 sample, channel;

		pos = (rx->pos + i) % HP_ADC_RING_BYTES;
		sample = le32_to_cpu(adc->ring[pos / sizeof(*adc->ring)]);
		channel = FIELD_GET(HP_ADC_SAMPLE_CHAN, sample);
		if ((sample & HP_ADC_SAMPLE_UNIT) ||
		    channel != adc->scan_channels[adc->scan_pos]) {
			dev_err_ratelimited(adc->dev, "unexpected ADC DMA sample %#x at scan index %u\n",
					    sample, adc->scan_pos);
			adc->scan_pos = 0;
			if ((sample & HP_ADC_SAMPLE_UNIT) || channel != adc->scan_channels[0])
				continue;
		}
		adc->scan[adc->scan_pos++] = FIELD_GET(ADC_SAMPLE_RESULT, sample);
		if (adc->scan_pos == adc->scan_count) {
			iio_push_to_buffers(indio_dev, adc->scan);
			adc->scan_pos = 0;
		}
	}
}

static int p4_adc_buffer_start(struct iio_dev *indio_dev)
{
	struct p4_adc *adc = iio_priv(indio_dev);
	struct dma_async_tx_descriptor *desc;
	u32 pattern[ADC1_CHANNELS / HP_ADC_PATTERN_SIZE] = { };
	unsigned int i, count = 0, interval;
	dma_cookie_t cookie;
	u32 ctrl, timer;

	for_each_set_bit(i, indio_dev->active_scan_mask, indio_dev->masklength) {
		unsigned int shift = (HP_ADC_PATTERN_SIZE - 1 -
				      count % HP_ADC_PATTERN_SIZE) * HP_ADC_PATTERN_BITS;
		u8 channel = indio_dev->channels[i].channel;

		adc->scan_channels[count] = channel;
		/* ADC1 channel plus 0 dB attenuation. */
		pattern[count / HP_ADC_PATTERN_SIZE] |= (channel << 2) << shift;
		count++;
	}
	if (!count)
		return -EINVAL;
	interval = DIV_ROUND_CLOSEST(adc->hp_rate, 2 * HP_ADC_SCANS_PER_SEC * count);
	if (interval < HP_ADC_SCAN_DELAY_MIN || !FIELD_FIT(HP_ADC_TIMER_TARGET, interval))
		return -ERANGE;

	desc = dmaengine_prep_dma_cyclic(adc->rx, adc->ring_dma, HP_ADC_RING_BYTES,
					 HP_ADC_PERIOD_BYTES, DMA_DEV_TO_MEM,
					 DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc)
		return -EIO;
	desc->callback_result = p4_adc_dma_complete;
	desc->callback_param = indio_dev;
	cookie = dmaengine_submit(desc);
	if (dma_submit_error(cookie))
		return cookie;

	adc->scan_count = count;
	adc->scan_pos = 0;

	p4_adc_hp_select(adc, true);
	writel(HP_ADC_FSM_RESET, adc->hp + HP_ADC_DMA_CONF);
	writel(0, adc->hp + HP_ADC_DMA_CONF);
	/* P4 adc_ll_digi_set_fsm_time(): reset, power and standby cycles. */
	writel(FIELD_PREP(HP_ADC_RESET_WAIT, 8) | FIELD_PREP(HP_ADC_POWER_WAIT, 5) |
	       FIELD_PREP(HP_ADC_STANDBY_WAIT, 100), adc->hp + HP_ADC_FSM_WAIT);
	for (i = 0; i < ARRAY_SIZE(pattern); i++)
		writel(pattern[i], adc->hp + HP_ADC_PATTERN + i * sizeof(u32));

	ctrl = HP_ADC_CLK_GATE | HP_ADC_DATA_TAG | FIELD_PREP(HP_ADC_SAR_DIV, 1) |
	       FIELD_PREP(HP_ADC_PATTERN_LEN, count - 1);
	writel(ctrl | HP_ADC_PATTERN_RESET, adc->hp + HP_ADC_CTRL);
	writel(ctrl, adc->hp + HP_ADC_CTRL);
	timer = HP_ADC_TIMER_SELECT | FIELD_PREP(HP_ADC_TIMER_TARGET, interval);
	writel(timer, adc->hp + HP_ADC_CTRL2);
	writel(HP_ADC_DMA_ENABLE | FIELD_PREP(HP_ADC_DMA_EOF, HP_ADC_PERIOD_SAMPLES),
	       adc->hp + HP_ADC_DMA_CONF);
	dma_async_issue_pending(adc->rx);
	writel(timer | HP_ADC_TIMER_ENABLE, adc->hp + HP_ADC_CTRL2);
	return 0;
}

static int p4_adc_buffer_stop(struct iio_dev *indio_dev)
{
	struct p4_adc *adc = iio_priv(indio_dev);
	int ret;

	writel(0, adc->hp + HP_ADC_CTRL2);
	writel(HP_ADC_FSM_RESET, adc->hp + HP_ADC_DMA_CONF);
	writel(0, adc->hp + HP_ADC_DMA_CONF);
	ret = dmaengine_terminate_sync(adc->rx);
	p4_adc_hp_select(adc, false);
	return ret;
}

static const struct iio_buffer_setup_ops p4_adc_buffer_ops = {
	.postenable = p4_adc_buffer_start,
	.predisable = p4_adc_buffer_stop,
};

static void p4_adc_release_dma(void *data)
{
	dma_release_channel(data);
}

static int p4_adc_configure_hp(struct platform_device *pdev, struct iio_dev *indio_dev)
{
	struct device *dev = &pdev->dev;
	struct p4_adc *adc = iio_priv(indio_dev);
	struct reset_control *reset;
	struct clk *clk;
	int ret;

	adc->hp = devm_platform_ioremap_resource_byname(pdev, "hp");
	if (IS_ERR(adc->hp))
		return PTR_ERR(adc->hp);
	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);
	adc->hp_rate = clk_get_rate(clk);
	if (!adc->hp_rate || adc->hp_rate > 5000000)
		return -EINVAL;
	reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(reset))
		return PTR_ERR(reset);
	ret = reset_control_reset(reset);
	if (ret)
		return ret;

	adc->rx = dma_request_chan(dev, "rx");
	if (IS_ERR(adc->rx))
		return PTR_ERR(adc->rx);
	ret = devm_add_action_or_reset(dev, p4_adc_release_dma, adc->rx);
	if (ret)
		return ret;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	adc->ring = dmam_alloc_coherent(dev, HP_ADC_RING_BYTES, &adc->ring_dma,
					GFP_KERNEL);
	if (!adc->ring)
		return -ENOMEM;
	return devm_iio_kfifo_buffer_setup(dev, indio_dev, &p4_adc_buffer_ops);
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
	bool has_hp;
	u32 input;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;
	adc = iio_priv(indio_dev);
	adc->dev = dev;
	mutex_init(&adc->lock);
	has_hp = platform_get_resource_byname(pdev, IORESOURCE_MEM, "hp");

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
			.scan_index = has_hp && input < ADC1_CHANNELS ? i : -1,
			.scan_type = {
				.sign = 'u',
				.realbits = 12,
				.storagebits = 16,
				.endianness = IIO_CPU,
			},
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
	if (has_hp && (seen & GENMASK(ADC1_CHANNELS - 1, 0))) {
		ret = p4_adc_configure_hp(pdev, indio_dev);
		if (ret)
			return dev_err_probe(dev, ret, "HP ADC buffer setup failed\n");
	}
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

MODULE_DESCRIPTION("Espressif ESP32-P4 SAR ADC driver");
MODULE_LICENSE("GPL");
