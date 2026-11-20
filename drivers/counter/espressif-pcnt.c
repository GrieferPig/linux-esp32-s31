// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif pulse counter driver.
 *
 * Each controller contains four independent signed 16-bit counters.  This
 * initial Counter framework mapping counts rising edges on channel 0; GPIO
 * matrix routing is described by the selected pinctrl test profile.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/counter.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#define PCNT_CH0_POS_MODE		GENMASK(19, 18)
#define PCNT_EDGE_INCREASE		1
#define PCNT_STEP_FORWARD		GENMASK(15, 0)
#define PCNT_STEP_BACKWARD		GENMASK(31, 16)
#define PCNT_STEP_FORWARD_EVENT		BIT(7)
#define PCNT_STEP_BACKWARD_EVENT	BIT(8)
#define PCNT_STEP_CHANNEL(unit, backward)	((unit) * 2 + (backward))

struct espressif_pcnt_layout {
	u8 stride;
	u8 threshold;
	u8 count;
	u8 int_st;
	u8 int_ena;
	u8 int_clr;
	u8 status;
	u8 ctrl;
	u16 event[2];
	bool step_events;
};

static const struct espressif_pcnt_layout esp32s31_pcnt_layout = {
	.stride = 0x10, .threshold = 0x0c, .count = 0x40,
	.int_st = 0x54, .int_ena = 0x58, .int_clr = 0x5c,
	.status = 0x60, .ctrl = 0x70,
	.event = { PCNT_STEP_FORWARD_EVENT, PCNT_STEP_BACKWARD_EVENT },
	.step_events = true,
};

/* TRM ch.53 Pulse Count Controller - CONF0/1/2 and THR_THRESn. */
static const struct espressif_pcnt_layout esp32p4_pcnt_layout = {
	.stride = 0x0c, .threshold = 0x04, .count = 0x30,
	.int_st = 0x44, .int_ena = 0x48, .int_clr = 0x4c,
	.status = 0x50, .ctrl = 0x60, .event = { BIT(3), BIT(2) },
};

struct esp32s31_pcnt {
	const struct espressif_pcnt_layout *layout;
	void __iomem *base;
	struct clk *clk;
	struct mutex lock;
	struct counter_signal signals[4];
	struct counter_synapse synapses[4];
	struct counter_count counts[4];
	struct counter_comp count_ext[4][2];
	struct counter_device *counter;
};

static const enum counter_function esp32s31_pcnt_functions[] = {
	COUNTER_FUNCTION_INCREASE,
};

static const enum counter_synapse_action esp32s31_pcnt_actions[] = {
	COUNTER_SYNAPSE_ACTION_RISING_EDGE,
};

static int esp32s31_pcnt_count_read(struct counter_device *counter,
				    struct counter_count *count, u64 *value)
{
	struct esp32s31_pcnt *pcnt = counter_priv(counter);

	*value = (u16)readl(pcnt->base + pcnt->layout->count + 4 * count->id);
	return 0;
}

static int esp32s31_pcnt_count_write(struct counter_device *counter,
				     struct counter_count *count, u64 value)
{
	struct esp32s31_pcnt *pcnt = counter_priv(counter);
	u32 ctrl;

	/* Hardware only supports an atomic clear, not an arbitrary preload. */
	if (value)
		return -EINVAL;
	mutex_lock(&pcnt->lock);
	ctrl = readl(pcnt->base + pcnt->layout->ctrl);
	writel(ctrl | BIT(count->id * 2), pcnt->base + pcnt->layout->ctrl);
	writel(ctrl & ~BIT(count->id * 2), pcnt->base + pcnt->layout->ctrl);
	mutex_unlock(&pcnt->lock);
	return 0;
}

static int esp32s31_pcnt_function_read(struct counter_device *counter,
				       struct counter_count *count,
				       enum counter_function *function)
{
	*function = COUNTER_FUNCTION_INCREASE;
	return 0;
}

static int esp32s31_pcnt_action_read(struct counter_device *counter,
				     struct counter_count *count,
				     struct counter_synapse *synapse,
				     enum counter_synapse_action *action)
{
	*action = COUNTER_SYNAPSE_ACTION_RISING_EDGE;
	return 0;
}

static int esp32s31_pcnt_step_read(struct counter_device *counter,
				   struct counter_count *count, u64 *value,
				   bool backward)
{
	struct esp32s31_pcnt *pcnt = counter_priv(counter);
	void __iomem *reg = pcnt->base + pcnt->layout->threshold +
			    count->id * pcnt->layout->stride;
	u32 conf = readl(reg);

	*value = (conf >> (backward ? 16 : 0)) & U16_MAX;
	return 0;
}

static int esp32s31_pcnt_step_write(struct counter_device *counter,
				    struct counter_count *count, u64 value,
				    bool backward)
{
	struct esp32s31_pcnt *pcnt = counter_priv(counter);
	void __iomem *reg = pcnt->base + pcnt->layout->threshold +
			    count->id * pcnt->layout->stride;
	u32 conf, mask = backward ? PCNT_STEP_BACKWARD : PCNT_STEP_FORWARD;

	/* PCNT is a signed 16-bit counter; a larger interval is unreachable. */
	if (value > SHRT_MAX)
		return -ERANGE;

	mutex_lock(&pcnt->lock);
	conf = readl(reg);
	conf &= ~mask;
	conf |= (u32)value << (backward ? 16 : 0);
	writel(conf, reg);
	mutex_unlock(&pcnt->lock);
	return 0;
}

static int esp32s31_pcnt_forward_step_read(struct counter_device *counter,
					   struct counter_count *count,
					   u64 *value)
{
	return esp32s31_pcnt_step_read(counter, count, value, false);
}

static int esp32s31_pcnt_forward_step_write(struct counter_device *counter,
					    struct counter_count *count,
					    u64 value)
{
	return esp32s31_pcnt_step_write(counter, count, value, false);
}

static int esp32s31_pcnt_backward_step_read(struct counter_device *counter,
					    struct counter_count *count,
					    u64 *value)
{
	return esp32s31_pcnt_step_read(counter, count, value, true);
}

static int esp32s31_pcnt_backward_step_write(struct counter_device *counter,
					     struct counter_count *count,
					     u64 value)
{
	return esp32s31_pcnt_step_write(counter, count, value, true);
}

static int esp32s31_pcnt_events_configure(struct counter_device *counter)
{
	struct esp32s31_pcnt *pcnt = counter_priv(counter);
	struct counter_event_node *event_node;
	u32 enabled_units = 0;
	u32 thresholds[4] = { };
	u32 ctrl;
	unsigned int unit;

	list_for_each_entry(event_node, &counter->events_list, l) {
		bool backward = event_node->channel & 1;
		u32 conf;

		unit = event_node->channel / 2;
		if (event_node->event != COUNTER_EVENT_THRESHOLD || unit >= 4)
			continue;
		conf = readl(pcnt->base + pcnt->layout->threshold +
			     unit * pcnt->layout->stride);
		if ((conf >> (backward ? 16 : 0)) & U16_MAX) {
			enabled_units |= BIT(unit);
			thresholds[unit] |= BIT(14 + backward);
		}
	}

	mutex_lock(&pcnt->lock);
	writel(0, pcnt->base + pcnt->layout->int_ena);
	writel(GENMASK(3, 0), pcnt->base + pcnt->layout->int_clr);
	if (pcnt->layout->step_events) {
		ctrl = readl(pcnt->base + pcnt->layout->ctrl);
		ctrl &= ~GENMASK(11, 8);
		ctrl |= enabled_units << 8;
		writel(ctrl, pcnt->base + pcnt->layout->ctrl);
	} else {
		for (unit = 0; unit < ARRAY_SIZE(thresholds); unit++) {
			void __iomem *conf = pcnt->base + unit * pcnt->layout->stride;

			ctrl = readl(conf) & ~GENMASK(15, 14);
			writel(ctrl | thresholds[unit], conf);
		}
	}
	writel(enabled_units, pcnt->base + pcnt->layout->int_ena);
	mutex_unlock(&pcnt->lock);
	return 0;
}

static int esp32s31_pcnt_watch_validate(struct counter_device *counter,
					const struct counter_watch *watch)
{
	if (watch->event != COUNTER_EVENT_THRESHOLD || watch->channel >= 8)
		return -EINVAL;
	return 0;
}

static irqreturn_t esp32s31_pcnt_irq(int irq, void *data)
{
	struct counter_device *counter = data;
	struct esp32s31_pcnt *pcnt = counter_priv(counter);
	u32 pending = readl(pcnt->base + pcnt->layout->int_st) & GENMASK(3, 0);
	u32 unit;

	if (!pending)
		return IRQ_NONE;

	for (unit = 0; unit < 4; unit++) {
		u32 status;

		if (!(pending & BIT(unit)))
			continue;
		status = readl(pcnt->base + pcnt->layout->status + unit * 4);
		if (status & pcnt->layout->event[0])
			counter_push_event(counter, COUNTER_EVENT_THRESHOLD,
					   PCNT_STEP_CHANNEL(unit, 0));
		if (status & pcnt->layout->event[1])
			counter_push_event(counter, COUNTER_EVENT_THRESHOLD,
					   PCNT_STEP_CHANNEL(unit, 1));
	}
	writel(pending, pcnt->base + pcnt->layout->int_clr);
	return IRQ_HANDLED;
}

static const struct counter_ops esp32s31_pcnt_ops = {
	.action_read = esp32s31_pcnt_action_read,
	.count_read = esp32s31_pcnt_count_read,
	.count_write = esp32s31_pcnt_count_write,
	.events_configure = esp32s31_pcnt_events_configure,
	.function_read = esp32s31_pcnt_function_read,
	.watch_validate = esp32s31_pcnt_watch_validate,
};

static int esp32s31_pcnt_probe(struct platform_device *pdev)
{
	struct counter_device *counter;
	struct esp32s31_pcnt *pcnt;
	struct reset_control *rst;
	unsigned int i;
	int irq, ret;

	counter = devm_counter_alloc(&pdev->dev, sizeof(*pcnt));
	if (!counter)
		return -ENOMEM;
	pcnt = counter_priv(counter);
	pcnt->counter = counter;
	pcnt->layout = device_get_match_data(&pdev->dev) ?: &esp32s31_pcnt_layout;
	mutex_init(&pcnt->lock);
	pcnt->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pcnt->base))
		return PTR_ERR(pcnt->base);
	pcnt->clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(pcnt->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(pcnt->clk),
				     "clock unavailable\n");
	rst = devm_reset_control_get_optional_exclusive(&pdev->dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(&pdev->dev, PTR_ERR(rst),
				     "reset unavailable\n");
	ret = reset_control_reset(rst);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "reset failed\n");

	for (i = 0; i < ARRAY_SIZE(pcnt->counts); i++) {
		pcnt->signals[i].id = i;
		pcnt->signals[i].name = devm_kasprintf(&pdev->dev, GFP_KERNEL,
						      "unit%u pulse", i);
		pcnt->synapses[i].actions_list = esp32s31_pcnt_actions;
		pcnt->synapses[i].num_actions =
			ARRAY_SIZE(esp32s31_pcnt_actions);
		pcnt->synapses[i].signal = &pcnt->signals[i];
		pcnt->counts[i].id = i;
		pcnt->counts[i].name = devm_kasprintf(&pdev->dev, GFP_KERNEL,
						     "unit%u count", i);
		pcnt->counts[i].functions_list = esp32s31_pcnt_functions;
		pcnt->counts[i].num_functions =
			ARRAY_SIZE(esp32s31_pcnt_functions);
		pcnt->counts[i].synapses = &pcnt->synapses[i];
		pcnt->counts[i].num_synapses = 1;
		pcnt->count_ext[i][0] = (struct counter_comp)
			COUNTER_COMP_COUNT_U64(pcnt->layout->step_events ?
				"forward_step_interval" : "threshold0",
				esp32s31_pcnt_forward_step_read,
				esp32s31_pcnt_forward_step_write);
		pcnt->count_ext[i][1] = (struct counter_comp)
			COUNTER_COMP_COUNT_U64(pcnt->layout->step_events ?
				"backward_step_interval" : "threshold1",
				esp32s31_pcnt_backward_step_read,
				esp32s31_pcnt_backward_step_write);
		pcnt->counts[i].ext = pcnt->count_ext[i];
		pcnt->counts[i].num_ext = ARRAY_SIZE(pcnt->count_ext[i]);
		if (!pcnt->signals[i].name || !pcnt->counts[i].name)
			return -ENOMEM;

		writel(FIELD_PREP(PCNT_CH0_POS_MODE, PCNT_EDGE_INCREASE),
		       pcnt->base + i * pcnt->layout->stride);
		if (!pcnt->layout->step_events)
			writel(SHRT_MAX, pcnt->base + i * pcnt->layout->stride + 0x08);
	}
	/* Clear then start all four units: clear bits 0/2/4/6, pause 1/3/5/7. */
	writel(0x55, pcnt->base + pcnt->layout->ctrl);
	writel(0, pcnt->base + pcnt->layout->ctrl);

	counter->name = dev_name(&pdev->dev);
	counter->parent = &pdev->dev;
	counter->ops = &esp32s31_pcnt_ops;
	counter->signals = pcnt->signals;
	counter->num_signals = ARRAY_SIZE(pcnt->signals);
	counter->counts = pcnt->counts;
	counter->num_counts = ARRAY_SIZE(pcnt->counts);
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ret = devm_request_irq(&pdev->dev, irq, esp32s31_pcnt_irq, 0,
			       dev_name(&pdev->dev), counter);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "interrupt unavailable\n");
	ret = devm_counter_add(&pdev->dev, counter);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "counter registration failed\n");
	return 0;
}

static const struct of_device_id esp32s31_pcnt_of_match[] = {
	{ .compatible = "espressif,esp32s31-pcnt", .data = &esp32s31_pcnt_layout },
	{ .compatible = "espressif,esp32p4-pcnt", .data = &esp32p4_pcnt_layout },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s31_pcnt_of_match);

static struct platform_driver esp32s31_pcnt_driver = {
	.probe = esp32s31_pcnt_probe,
	.driver = {
		.name = "esp32s31-pcnt",
		.of_match_table = esp32s31_pcnt_of_match,
	},
};
module_platform_driver(esp32s31_pcnt_driver);

MODULE_DESCRIPTION("Espressif pulse counter driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("COUNTER");
