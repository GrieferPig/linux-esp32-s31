// SPDX-License-Identifier: GPL-2.0-only
/*
 * Read-only Counter framework access to ESP32-S31 SYSTIMER and RTC_TIMER.
 * Linux scheduling continues to use the architectural RISC-V timer.
 */

#include <linux/clk.h>
#include <linux/counter.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

enum esp32s31_timer_kind {
	ESP32S31_SYSTIMER,
	ESP32S31_RTC_TIMER,
};

struct esp32s31_system_timer {
	void __iomem *base;
	enum esp32s31_timer_kind kind;
	struct clk *clk;
	struct counter_count counts[2];
};

static const enum counter_function esp32s31_timer_functions[] = {
	COUNTER_FUNCTION_INCREASE,
};

static int esp32s31_timer_count_read(struct counter_device *counter,
				     struct counter_count *count, u64 *value)
{
	struct esp32s31_system_timer *timer = counter_priv(counter);
	u32 hi, lo, status;
	int ret;

	if (timer->kind == ESP32S31_SYSTIMER) {
		/* Snapshot unit n, then read its 52-bit synchronized value. */
		writel(BIT(30), timer->base + 0x04 + count->id * 4);
		ret = readl_poll_timeout(timer->base + 0x04 + count->id * 4,
					 status, status & BIT(29), 0, 1000);
		if (ret)
			return ret;
		hi = readl(timer->base + 0x40 + count->id * 8) & GENMASK(19, 0);
		lo = readl(timer->base + 0x44 + count->id * 8);
	} else {
		/* RTC_TIMER update snapshots both 48-bit counters. */
		writel(BIT(27), timer->base + 0x10);
		lo = readl(timer->base + 0x14 + count->id * 8);
		hi = readl(timer->base + 0x18 + count->id * 8) & GENMASK(15, 0);
	}
	*value = ((u64)hi << 32) | lo;
	return 0;
}

static int esp32s31_timer_function_read(struct counter_device *counter,
					struct counter_count *count,
					enum counter_function *function)
{
	*function = COUNTER_FUNCTION_INCREASE;
	return 0;
}

static const struct counter_ops esp32s31_timer_ops = {
	.count_read = esp32s31_timer_count_read,
	.function_read = esp32s31_timer_function_read,
};

static int esp32s31_system_timer_probe(struct platform_device *pdev)
{
	struct counter_device *counter;
	struct esp32s31_system_timer *timer;
	unsigned int i;

	counter = devm_counter_alloc(&pdev->dev, sizeof(*timer));
	if (!counter)
		return -ENOMEM;
	timer = counter_priv(counter);
	timer->kind = (uintptr_t)of_device_get_match_data(&pdev->dev);
	timer->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(timer->base))
		return PTR_ERR(timer->base);
	timer->clk = devm_clk_get_optional_enabled(&pdev->dev, NULL);
	if (IS_ERR(timer->clk))
		return PTR_ERR(timer->clk);
	if (timer->kind == ESP32S31_SYSTIMER)
		writel(readl(timer->base) | BIT(31) | BIT(30) | BIT(29),
		       timer->base);
	else
		writel(readl(timer->base + 0x3fc) | BIT(31),
		       timer->base + 0x3fc);

	for (i = 0; i < ARRAY_SIZE(timer->counts); i++) {
		timer->counts[i].id = i;
		timer->counts[i].name = devm_kasprintf(&pdev->dev, GFP_KERNEL,
						      "counter%u", i);
		timer->counts[i].functions_list = esp32s31_timer_functions;
		timer->counts[i].num_functions =
			ARRAY_SIZE(esp32s31_timer_functions);
		if (!timer->counts[i].name)
			return -ENOMEM;
	}
	counter->name = dev_name(&pdev->dev);
	counter->parent = &pdev->dev;
	counter->ops = &esp32s31_timer_ops;
	counter->counts = timer->counts;
	counter->num_counts = ARRAY_SIZE(timer->counts);
	return devm_counter_add(&pdev->dev, counter);
}

static const struct of_device_id esp32s31_system_timer_of_match[] = {
	{ .compatible = "espressif,esp32s31-systimer",
	  .data = (void *)ESP32S31_SYSTIMER },
	{ .compatible = "espressif,esp32s31-rtc-timer",
	  .data = (void *)ESP32S31_RTC_TIMER },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s31_system_timer_of_match);

static struct platform_driver esp32s31_system_timer_driver = {
	.probe = esp32s31_system_timer_probe,
	.driver = {
		.name = "esp32s31-system-timers",
		.of_match_table = esp32s31_system_timer_of_match,
	},
};
module_platform_driver(esp32s31_system_timer_driver);
MODULE_DESCRIPTION("ESP32-S31 SYSTIMER and RTC timer counter driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(COUNTER);
