// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S31 pulse counter driver.
 *
 * Each controller contains four independent signed 16-bit counters.  This
 * initial Counter framework mapping counts rising edges on channel 0; GPIO
 * matrix routing is described by the selected pinctrl test profile.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/counter.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define PCNT_CONF0(unit)		((unit) * 0x10)
#define PCNT_COUNT(unit)		(0x40 + (unit) * 4)
#define PCNT_CTRL			0x70
#define PCNT_CH0_POS_MODE		GENMASK(19, 18)
#define PCNT_EDGE_INCREASE		1

struct esp32s31_pcnt {
	void __iomem *base;
	struct clk *clk;
	struct mutex lock;
	struct counter_signal signals[4];
	struct counter_synapse synapses[4];
	struct counter_count counts[4];
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

	*value = (u16)readl(pcnt->base + PCNT_COUNT(count->id));
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
	ctrl = readl(pcnt->base + PCNT_CTRL);
	writel(ctrl | BIT(count->id * 2), pcnt->base + PCNT_CTRL);
	writel(ctrl & ~BIT(count->id * 2), pcnt->base + PCNT_CTRL);
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

static const struct counter_ops esp32s31_pcnt_ops = {
	.action_read = esp32s31_pcnt_action_read,
	.count_read = esp32s31_pcnt_count_read,
	.count_write = esp32s31_pcnt_count_write,
	.function_read = esp32s31_pcnt_function_read,
};

static int esp32s31_pcnt_probe(struct platform_device *pdev)
{
	struct counter_device *counter;
	struct esp32s31_pcnt *pcnt;
	unsigned int i;
	int ret;

	counter = devm_counter_alloc(&pdev->dev, sizeof(*pcnt));
	if (!counter)
		return -ENOMEM;
	pcnt = counter_priv(counter);
	mutex_init(&pcnt->lock);
	pcnt->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pcnt->base))
		return PTR_ERR(pcnt->base);
	pcnt->clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(pcnt->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(pcnt->clk),
				     "clock unavailable\n");

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
		if (!pcnt->signals[i].name || !pcnt->counts[i].name)
			return -ENOMEM;

		writel(FIELD_PREP(PCNT_CH0_POS_MODE, PCNT_EDGE_INCREASE),
		       pcnt->base + PCNT_CONF0(i));
	}
	/* Clear then start all four units: clear bits 0/2/4/6, pause 1/3/5/7. */
	writel(0x55, pcnt->base + PCNT_CTRL);
	writel(0, pcnt->base + PCNT_CTRL);

	counter->name = dev_name(&pdev->dev);
	counter->parent = &pdev->dev;
	counter->ops = &esp32s31_pcnt_ops;
	counter->signals = pcnt->signals;
	counter->num_signals = ARRAY_SIZE(pcnt->signals);
	counter->counts = pcnt->counts;
	counter->num_counts = ARRAY_SIZE(pcnt->counts);
	ret = devm_counter_add(&pdev->dev, counter);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "counter registration failed\n");
	return 0;
}

static const struct of_device_id esp32s31_pcnt_of_match[] = {
	{ .compatible = "espressif,esp32s31-pcnt" },
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

MODULE_DESCRIPTION("ESP32-S31 pulse counter driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(COUNTER);
