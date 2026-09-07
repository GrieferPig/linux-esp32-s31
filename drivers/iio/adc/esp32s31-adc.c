// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 dual 8-channel SAR ADC, following ESP-IDF adc_ll flows. */

#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/iio.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#define ADC_CTRL(u)			((u) * 4)
#define ADC_PATTERN(u)			(0x1c + (u) * 0x10)
#define ADC_DATA(u)			(0x40 + (u) * 4)
#define ADC_FILTER_CTRL1		0x0c
#define ADC_FILTER_CTRL0		0x3c
#define ADC_THRES_HIGH(m)		(0x48 + (m) * 0x0c)
#define ADC_THRES_LOW(m)		(0x4c + (m) * 0x0c)
#define ADC_THRES_CONFIG(m)		(0x50 + (m) * 0x0c)
#define ADC_THRES_CTRL			0x60
#define ADC_CTRL2			0x08
#define ADC_INT_ENA			0x64
#define ADC_INT_RAW			0x68
#define ADC_INT_CLR			0x70
#define ADC_REF_CONTROL			0x78
#define ADC_CTRL_DATE			0x3fc
#define ADC_TRIGGER_STOP		BIT(0)
#define ADC_TRIGGER_START		BIT(1)
#define ADC_TRIGGER_MODE		GENMASK(3, 2)
#define ADC_TRIGGER_SCAN		1
#define ADC_TRIGGER_SW			2
#define ADC_PATTERN_LEN			GENMASK(17, 14)
#define ADC_PATTERN_CLEAR		BIT(22)
#define ADC_PATTERN_TYPE		BIT(23)
#define ADC_CONTINUE_MODE		BIT(25)
#define ADC_POWER			GENMASK(27, 26)
#define ADC_POWER_UP			3
#define ADC_DONE(u)			BIT(31 - (u))
#define ADC_THRES_HIGH_INT(m)		BIT(29 - (m))
#define ADC_THRES_LOW_INT(m)		BIT(27 - (m))
#define ADC_THRES_CHANNEL		GENMASK(4, 0)
#define ADC_THRES_ENABLE		BIT(5)
#define ADC_THRES_ALL_ENABLE		BIT(27)
#define ADC_FILTER_CHANNEL0		GENMASK(23, 19)
#define ADC_FILTER_CHANNEL1		GENMASK(18, 14)
#define ADC_FILTER_FACTOR0		GENMASK(31, 29)
#define ADC_FILTER_FACTOR1		GENMASK(28, 26)
#define ADC_FILTER_RESET		BIT(31)
#define ADC_REF_DELAY			BIT(29)
#define ADC_REF_PRECHARGE		BIT(30)
#define ADC_REF_POWER			BIT(31)
#define ADC_BLOCK_CLK			BIT(31)
#define ADC_TIMER_EN			BIT(24)
#define ADC_TIMER_TARGET		GENMASK(23, 12)
#define LP_ADC_DIV			GENMASK(7, 0)
#define LP_ADC_CLK_SEL			GENMASK(29, 28)
#define LP_ADC_CLK_XTAL			1
#define LP_ADC_CLK_EN			BIT(30)
#define LP_ADC_RST			BIT(31)

struct esp32s31_adc {
	void __iomem *base;
	void __iomem *clkrst;
	struct iio_dev *indio_dev;
	struct mutex lock;
	u32 sample_freq;
	u32 active_units;
	u32 pending_units;
	u8 unit_channels[2][8];
	u8 unit_length[2];
	u8 unit_position[2];
	u32 samples[16];
	struct {
		int channel;
		u32 high;
		u32 low;
		bool high_enabled;
		bool low_enabled;
	} monitor[2];
	struct {
		u32 values[16];
		aligned_s64 timestamp;
	} scan;
};

static const struct iio_event_spec esp32s31_adc_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
				 BIT(IIO_EV_INFO_ENABLE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
				 BIT(IIO_EV_INFO_ENABLE),
	},
};

#define ESP32S31_ADC_CH(u, c) {						\
	.type = IIO_VOLTAGE, .indexed = 1, .channel = (u) * 8 + (c),	\
	.address = ((u) << 8) | (c),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),		\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = (u) * 8 + (c),					\
	.scan_type = {							\
		.sign = 'u', .realbits = 17, .storagebits = 32,		\
		.endianness = IIO_CPU,					\
	},								\
	.event_spec = esp32s31_adc_events,				\
	.num_event_specs = ARRAY_SIZE(esp32s31_adc_events),		\
}

#define ESP32S31_ADC_DIFF(u, p, n) {					\
	.type = IIO_VOLTAGE, .indexed = 1, .differential = 1,		\
	.channel = (u) * 8 + (p), .channel2 = (u) * 8 + (n),		\
	.address = BIT(16) | ((u) << 8) | ((p) << 4) | (n),		\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),		\
	.scan_index = -1,						\
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
	/* Paired, back-to-back conversions; no undocumented HW model bits. */
	ESP32S31_ADC_DIFF(0, 0, 1), ESP32S31_ADC_DIFF(0, 2, 3),
	ESP32S31_ADC_DIFF(0, 4, 5), ESP32S31_ADC_DIFF(0, 6, 7),
	ESP32S31_ADC_DIFF(1, 0, 1), ESP32S31_ADC_DIFF(1, 2, 3),
	ESP32S31_ADC_DIFF(1, 4, 5), ESP32S31_ADC_DIFF(1, 6, 7),
	IIO_CHAN_SOFT_TIMESTAMP(16),
};

static int esp32s31_adc_convert_locked(struct esp32s31_adc *adc,
				       unsigned int unit,
				       unsigned int channel, int *value)
{
	u32 ctrl, status;
	int ret;

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
	/* adc_oneshot_ll_enable() followed by adc_oneshot_ll_start(). */
	writel(ctrl | ADC_TRIGGER_STOP, adc->base + ADC_CTRL(unit));
	writel(ctrl | ADC_TRIGGER_STOP | ADC_TRIGGER_START,
	       adc->base + ADC_CTRL(unit));
	ret = readl_poll_timeout(adc->base + ADC_INT_RAW, status,
				 status & ADC_DONE(unit), 1, 10000);
	if (!ret)
		*value = readl(adc->base + ADC_DATA(unit)) & GENMASK(16, 0);
	writel(ADC_DONE(unit), adc->base + ADC_INT_CLR);
	writel((ctrl & ~ADC_TRIGGER_MODE) | ADC_TRIGGER_STOP,
	       adc->base + ADC_CTRL(unit));

	return ret;
}

static int esp32s31_adc_read_raw(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 int *val, int *val2, long mask)
{
	struct esp32s31_adc *adc = iio_priv(indio_dev);
	unsigned int unit = (chan->address >> 8) & 1;
	unsigned int channel = chan->address & 0xf;
	int positive, negative;
	int ret;

	if (mask == IIO_CHAN_INFO_SCALE) {
		*val = 1100;
		*val2 = 4393;
		return IIO_VAL_FRACTIONAL;
	}
	if (mask == IIO_CHAN_INFO_SAMP_FREQ) {
		*val = adc->sample_freq;
		return IIO_VAL_INT;
	}
	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;
	if (iio_buffer_enabled(indio_dev))
		return -EBUSY;

	mutex_lock(&adc->lock);
	if (chan->differential) {
		ret = esp32s31_adc_convert_locked(adc, unit,
						 (chan->address >> 4) & 0xf,
						 &positive);
		if (!ret)
			ret = esp32s31_adc_convert_locked(adc, unit, channel,
							 &negative);
		if (!ret)
			*val = positive - negative;
	} else {
		ret = esp32s31_adc_convert_locked(adc, unit, channel, val);
	}
	mutex_unlock(&adc->lock);
	return ret ? ret : IIO_VAL_INT;
}

static int esp32s31_adc_write_raw(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan,
				  int val, int val2, long mask)
{
	struct esp32s31_adc *adc = iio_priv(indio_dev);

	if (mask != IIO_CHAN_INFO_SAMP_FREQ || val2 || val < 2000 || val > 20000)
		return -EINVAL;
	if (iio_buffer_enabled(indio_dev))
		return -EBUSY;
	adc->sample_freq = val;
	return 0;
}

static bool esp32s31_adc_validate_scan_mask(struct iio_dev *indio_dev,
					    const unsigned long *mask)
{
	return !bitmap_empty(mask, 16);
}

static void esp32s31_adc_select_scan_channel(struct esp32s31_adc *adc,
					      unsigned int unit)
{
	u32 ctrl = readl(adc->base + ADC_CTRL(unit));
	u8 channel = adc->unit_channels[unit][adc->unit_position[unit]];

	writel((channel & 0xf) << 20, adc->base + ADC_PATTERN(unit));
	writel(ctrl | ADC_PATTERN_CLEAR, adc->base + ADC_CTRL(unit));
	writel(ctrl, adc->base + ADC_CTRL(unit));
}

static int esp32s31_adc_monitor_slot(struct esp32s31_adc *adc,
				     unsigned int channel, bool allocate)
{
	int free_slot = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(adc->monitor); i++) {
		if (adc->monitor[i].channel == channel)
			return i;
		if (adc->monitor[i].channel < 0)
			free_slot = i;
	}
	if (!allocate || free_slot < 0)
		return -ENOSPC;
	adc->monitor[free_slot].channel = channel;
	adc->monitor[free_slot].high = GENMASK(16, 0);
	adc->monitor[free_slot].low = 0;
	return free_slot;
}

static void esp32s31_adc_program_monitor(struct esp32s31_adc *adc,
					 unsigned int slot)
{
	u32 interrupts = ADC_THRES_HIGH_INT(slot) | ADC_THRES_LOW_INT(slot);
	u32 enabled = readl(adc->base + ADC_INT_ENA) & ~interrupts;
	bool active = adc->monitor[slot].high_enabled ||
		      adc->monitor[slot].low_enabled;

	writel(adc->monitor[slot].high, adc->base + ADC_THRES_HIGH(slot));
	writel(adc->monitor[slot].low, adc->base + ADC_THRES_LOW(slot));
	writel(active ? FIELD_PREP(ADC_THRES_CHANNEL,
				   adc->monitor[slot].channel) |
			      ADC_THRES_ENABLE : 0,
	       adc->base + ADC_THRES_CONFIG(slot));
	if (adc->monitor[slot].high_enabled)
		enabled |= ADC_THRES_HIGH_INT(slot);
	if (adc->monitor[slot].low_enabled)
		enabled |= ADC_THRES_LOW_INT(slot);
	writel(interrupts, adc->base + ADC_INT_CLR);
	writel(enabled, adc->base + ADC_INT_ENA);
	writel(ADC_THRES_ALL_ENABLE, adc->base + ADC_THRES_CTRL);
}

static int esp32s31_adc_read_event_config(struct iio_dev *indio_dev,
					  const struct iio_chan_spec *chan,
					  enum iio_event_type type,
					  enum iio_event_direction dir)
{
	struct esp32s31_adc *adc = iio_priv(indio_dev);
	int slot = esp32s31_adc_monitor_slot(adc, chan->channel, false);

	if (slot < 0)
		return 0;
	return dir == IIO_EV_DIR_RISING ? adc->monitor[slot].high_enabled :
		adc->monitor[slot].low_enabled;
}

static int esp32s31_adc_write_event_config(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *chan,
					   enum iio_event_type type,
					   enum iio_event_direction dir,
					   bool state)
{
	struct esp32s31_adc *adc = iio_priv(indio_dev);
	int slot;

	mutex_lock(&adc->lock);
	slot = esp32s31_adc_monitor_slot(adc, chan->channel, state);
	if (slot < 0) {
		mutex_unlock(&adc->lock);
		return state ? slot : 0;
	}
	if (dir == IIO_EV_DIR_RISING)
		adc->monitor[slot].high_enabled = state;
	else
		adc->monitor[slot].low_enabled = state;
	esp32s31_adc_program_monitor(adc, slot);
	if (!adc->monitor[slot].high_enabled &&
	    !adc->monitor[slot].low_enabled)
		adc->monitor[slot].channel = -1;
	mutex_unlock(&adc->lock);
	return 0;
}

static int esp32s31_adc_read_event_value(struct iio_dev *indio_dev,
					 const struct iio_chan_spec *chan,
					 enum iio_event_type type,
					 enum iio_event_direction dir,
					 enum iio_event_info info,
					 int *val, int *val2)
{
	struct esp32s31_adc *adc = iio_priv(indio_dev);
	int slot = esp32s31_adc_monitor_slot(adc, chan->channel, false);

	if (slot < 0)
		return -ENOENT;
	*val = dir == IIO_EV_DIR_RISING ? adc->monitor[slot].high :
		adc->monitor[slot].low;
	return IIO_VAL_INT;
}

static int esp32s31_adc_write_event_value(struct iio_dev *indio_dev,
					  const struct iio_chan_spec *chan,
					  enum iio_event_type type,
					  enum iio_event_direction dir,
					  enum iio_event_info info,
					  int val, int val2)
{
	struct esp32s31_adc *adc = iio_priv(indio_dev);
	int slot;

	if (val2 || val < 0 || val > GENMASK(16, 0))
		return -ERANGE;
	mutex_lock(&adc->lock);
	slot = esp32s31_adc_monitor_slot(adc, chan->channel, true);
	if (slot >= 0) {
		if (dir == IIO_EV_DIR_RISING)
			adc->monitor[slot].high = val;
		else
			adc->monitor[slot].low = val;
		esp32s31_adc_program_monitor(adc, slot);
	}
	mutex_unlock(&adc->lock);
	return slot < 0 ? slot : 0;
}

static irqreturn_t esp32s31_adc_irq(int irq, void *data)
{
	struct esp32s31_adc *adc = data;
	struct iio_dev *indio_dev = adc->indio_dev;
	u32 status, done;
	unsigned int bit, unit, n = 0;

	status = readl(adc->base + ADC_INT_RAW);
	for (unit = 0; unit < ARRAY_SIZE(adc->monitor); unit++) {
		u32 events = status & (ADC_THRES_HIGH_INT(unit) |
				       ADC_THRES_LOW_INT(unit));
		int channel = adc->monitor[unit].channel;

		if (!events || channel < 0)
			continue;
		if (events & ADC_THRES_HIGH_INT(unit))
			iio_push_event(indio_dev,
				IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE, channel,
					IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING),
				iio_get_time_ns(indio_dev));
		if (events & ADC_THRES_LOW_INT(unit))
			iio_push_event(indio_dev,
				IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE, channel,
					IIO_EV_TYPE_THRESH, IIO_EV_DIR_FALLING),
				iio_get_time_ns(indio_dev));
		writel(events, adc->base + ADC_INT_CLR);
	}
	done = status & (ADC_DONE(0) | ADC_DONE(1)) & adc->active_units;
	if (!done)
		return status & GENMASK(29, 26) ? IRQ_HANDLED : IRQ_NONE;

	for (unit = 0; unit < 2; unit++) {
		u8 position;

		if (!(done & ADC_DONE(unit)))
			continue;
		position = adc->unit_position[unit];
		bit = unit * 8 + adc->unit_channels[unit][position];
		adc->samples[bit] = readl(adc->base + ADC_DATA(unit)) &
				    GENMASK(16, 0);
		position++;
		if (position == adc->unit_length[unit]) {
			position = 0;
			adc->pending_units |= ADC_DONE(unit);
		}
		adc->unit_position[unit] = position;
		esp32s31_adc_select_scan_channel(adc, unit);
	}
	writel(done, adc->base + ADC_INT_CLR);

	if (adc->pending_units != adc->active_units)
		return IRQ_HANDLED;

	for_each_set_bit(bit, indio_dev->active_scan_mask, 16)
		adc->scan.values[n++] = adc->samples[bit];
	adc->pending_units = 0;
	iio_push_to_buffers_with_timestamp(indio_dev, &adc->scan,
					   iio_get_time_ns(indio_dev));

	return IRQ_HANDLED;
}

static int esp32s31_adc_buffer_postenable(struct iio_dev *indio_dev)
{
	struct esp32s31_adc *adc = iio_priv(indio_dev);
	u32 ctrl, timer, irq_mask = 0;
	unsigned int bit, unit, max_length;

	mutex_lock(&adc->lock);
	adc->active_units = 0;
	adc->pending_units = 0;
	memset(adc->unit_length, 0, sizeof(adc->unit_length));
	memset(adc->unit_position, 0, sizeof(adc->unit_position));
	for_each_set_bit(bit, indio_dev->active_scan_mask, 16) {
		unit = bit / 8;
		adc->unit_channels[unit][adc->unit_length[unit]++] = bit % 8;
		irq_mask |= ADC_DONE(unit);
	}
	max_length = max(adc->unit_length[0], adc->unit_length[1]);
	if (adc->sample_freq * max_length > 83333) {
		mutex_unlock(&adc->lock);
		return -ERANGE;
	}
	for (unit = 0; unit < 2; unit++) {
		if (!adc->unit_length[unit])
			continue;
		ctrl = readl(adc->base + ADC_CTRL(unit));
		ctrl &= ~(ADC_TRIGGER_MODE | ADC_PATTERN_LEN |
			  ADC_CONTINUE_MODE | ADC_POWER | ADC_PATTERN_TYPE);
		ctrl |= FIELD_PREP(ADC_TRIGGER_MODE, ADC_TRIGGER_SCAN) |
			FIELD_PREP(ADC_POWER, ADC_POWER_UP);
		writel(ctrl, adc->base + ADC_CTRL(unit));
		esp32s31_adc_select_scan_channel(adc, unit);
	}
	adc->active_units = irq_mask;
	writel(irq_mask, adc->base + ADC_INT_CLR);
	writel(readl(adc->base + ADC_INT_ENA) | irq_mask,
	       adc->base + ADC_INT_ENA);
	timer = readl(adc->base + ADC_CTRL2);
	timer &= ~(ADC_TIMER_EN | ADC_TIMER_TARGET);
	timer |= FIELD_PREP(ADC_TIMER_TARGET,
			    DIV_ROUND_CLOSEST(8000000U,
					      adc->sample_freq * max_length));
	writel(timer | ADC_TIMER_EN, adc->base + ADC_CTRL2);
	mutex_unlock(&adc->lock);

	return 0;
}

static int esp32s31_adc_buffer_predisable(struct iio_dev *indio_dev)
{
	struct esp32s31_adc *adc = iio_priv(indio_dev);
	u32 ctrl;
	unsigned int unit;

	mutex_lock(&adc->lock);
	writel(readl(adc->base + ADC_CTRL2) & ~ADC_TIMER_EN,
	       adc->base + ADC_CTRL2);
	writel(readl(adc->base + ADC_INT_ENA) & ~adc->active_units,
	       adc->base + ADC_INT_ENA);
	writel(adc->active_units, adc->base + ADC_INT_CLR);
	for (unit = 0; unit < 2; unit++) {
		if (!(adc->active_units & ADC_DONE(unit)))
			continue;
		ctrl = readl(adc->base + ADC_CTRL(unit));
		ctrl &= ~ADC_TRIGGER_MODE;
		writel(ctrl | ADC_TRIGGER_STOP, adc->base + ADC_CTRL(unit));
	}
	adc->active_units = 0;
	adc->pending_units = 0;
	mutex_unlock(&adc->lock);

	return 0;
}

static const struct iio_buffer_setup_ops esp32s31_adc_buffer_ops = {
	.postenable = esp32s31_adc_buffer_postenable,
	.predisable = esp32s31_adc_buffer_predisable,
	.validate_scan_mask = esp32s31_adc_validate_scan_mask,
};

static const struct iio_info esp32s31_adc_info = {
	.read_raw = esp32s31_adc_read_raw,
	.write_raw = esp32s31_adc_write_raw,
	.read_event_config = esp32s31_adc_read_event_config,
	.write_event_config = esp32s31_adc_write_event_config,
	.read_event_value = esp32s31_adc_read_event_value,
	.write_event_value = esp32s31_adc_write_event_value,
};

static int esp32s31_adc_configure_filters(struct platform_device *pdev,
					  struct esp32s31_adc *adc)
{
	u32 channels[2], factors[2], ctrl0, ctrl1;
	int count, factor_count, i;

	count = device_property_count_u32(&pdev->dev,
					  "espressif,filter-channels");
	if (count < 0) {
		writel(FIELD_PREP(ADC_FILTER_CHANNEL0, 0x1f) |
		       FIELD_PREP(ADC_FILTER_CHANNEL1, 0x1f),
		       adc->base + ADC_FILTER_CTRL0);
		return 0;
	}
	factor_count = device_property_count_u32(&pdev->dev,
						 "espressif,filter-factors");
	if (count > 2 || factor_count != count)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "filters need one or two channel/factor pairs\n");
	if (device_property_read_u32_array(&pdev->dev,
					   "espressif,filter-channels",
					   channels, count) ||
	    device_property_read_u32_array(&pdev->dev,
					   "espressif,filter-factors",
					   factors, count))
		return -EINVAL;

	ctrl0 = FIELD_PREP(ADC_FILTER_CHANNEL0, 0x1f) |
		FIELD_PREP(ADC_FILTER_CHANNEL1, 0x1f);
	ctrl1 = readl(adc->base + ADC_FILTER_CTRL1) &
		~(ADC_FILTER_FACTOR0 | ADC_FILTER_FACTOR1);
	for (i = 0; i < count; i++) {
		u32 encoded, factor;

		if (channels[i] >= 16 || factors[i] < 2 || factors[i] > 64 ||
		    !is_power_of_2(factors[i]))
			return dev_err_probe(&pdev->dev, -EINVAL,
					     "invalid ADC filter channel/factor\n");
		encoded = ((channels[i] / 8 + 1) << 3) | (channels[i] % 8);
		factor = ilog2(factors[i]);
		if (!i) {
			ctrl0 &= ~ADC_FILTER_CHANNEL0;
			ctrl0 |= FIELD_PREP(ADC_FILTER_CHANNEL0, encoded);
			ctrl1 |= FIELD_PREP(ADC_FILTER_FACTOR0, factor);
		} else {
			ctrl0 &= ~ADC_FILTER_CHANNEL1;
			ctrl0 |= FIELD_PREP(ADC_FILTER_CHANNEL1, encoded);
			ctrl1 |= FIELD_PREP(ADC_FILTER_FACTOR1, factor);
		}
	}
	writel(ctrl1, adc->base + ADC_FILTER_CTRL1);
	writel(ctrl0 | ADC_FILTER_RESET, adc->base + ADC_FILTER_CTRL0);
	writel(ctrl0, adc->base + ADC_FILTER_CTRL0);
	return 0;
}

static int esp32s31_adc_probe(struct platform_device *pdev)
{
	struct esp32s31_adc *adc;
	struct iio_dev *indio_dev;
	int irq, ret;
	u32 val;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;
	adc = iio_priv(indio_dev);
	adc->indio_dev = indio_dev;
	adc->sample_freq = 2000;
	mutex_init(&adc->lock);
	adc->monitor[0].channel = -1;
	adc->monitor[1].channel = -1;
	adc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(adc->base))
		return PTR_ERR(adc->base);
	adc->clkrst = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(adc->clkrst))
		return PTR_ERR(adc->clkrst);
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ret = devm_request_irq(&pdev->dev, irq, esp32s31_adc_irq, 0,
			       dev_name(&pdev->dev), adc);
	if (ret)
		return ret;

	ret = devm_regulator_get_enable(&pdev->dev, "analog");
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "analog I2C power unavailable\n");

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
	ret = esp32s31_adc_configure_filters(pdev, adc);
	if (ret)
		return ret;

	indio_dev->name = "esp32s31-adc";
	indio_dev->info = &esp32s31_adc_info;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	indio_dev->channels = esp32s31_adc_channels;
	indio_dev->num_channels = ARRAY_SIZE(esp32s31_adc_channels);
	ret = devm_iio_kfifo_buffer_setup(&pdev->dev, indio_dev,
					  &esp32s31_adc_buffer_ops);
	if (ret)
		return ret;
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
