// SPDX-License-Identifier: GPL-2.0-only
/*
 * Read-only access to Espressif SYSTIMER and RTC timer snapshots.
 * The kernel clocksource remains owned by the platform timer driver.
 */

#include <linux/clk.h>
#include <linux/counter.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>

enum esp32s31_timer_kind {
	ESP32S31_SYSTIMER,
	ESP32S31_RTC_TIMER,
	ESP32P4_RTC_TIMER,
};

/* TRM ch.20 RTC Timer: UPDATE, MAIN_BUF0 and DATE.CLK_EN. */
#define RTC_TIMER_UPDATE	0x010
#define RTC_TIMER_BUF_LO	0x014
#define RTC_TIMER_BUF_HI	0x018
#define RTC_TIMER_DATE		0x3fc
#define RTC_TIMER_CLK_EN	BIT(31)
#define P4_RTC_TIMER_LATCH	BIT(28)
#define S31_RTC_TIMER_LATCH	BIT(27)

struct esp32s31_system_timer {
	void __iomem *base;
	enum esp32s31_timer_kind kind;
	struct clk *clk;
	struct mutex lock;
	struct counter_count counts[2];
};

static const enum counter_function esp32s31_timer_functions[] = {
	COUNTER_FUNCTION_INCREASE,
};

static int esp32s31_timer_count_read(struct counter_device *counter,
				     struct counter_count *count, u64 *value)
{
	struct esp32s31_system_timer *timer = counter_priv(counter);
	u32 hi, lo, status, latch;
	int ret = 0;

	mutex_lock(&timer->lock);
	if (timer->kind == ESP32S31_SYSTIMER) {
		/* Snapshot unit n, then read its 52-bit synchronized value. */
		writel(BIT(30), timer->base + 0x04 + count->id * 4);
		ret = readl_poll_timeout(timer->base + 0x04 + count->id * 4,
					 status, status & BIT(29), 0, 1000);
		if (ret)
			goto unlock;
		hi = readl(timer->base + 0x40 + count->id * 8) & GENMASK(19, 0);
		lo = readl(timer->base + 0x44 + count->id * 8);
	} else {
		if (timer->kind == ESP32P4_RTC_TIMER) {
			/* Preserve the hardware-triggered event latching enables. */
			latch = readl(timer->base + RTC_TIMER_UPDATE) |
				P4_RTC_TIMER_LATCH;
		} else {
			latch = S31_RTC_TIMER_LATCH;
		}
		writel(latch, timer->base + RTC_TIMER_UPDATE);
		lo = readl(timer->base + RTC_TIMER_BUF_LO + count->id * 8);
		hi = readl(timer->base + RTC_TIMER_BUF_HI + count->id * 8) &
		     GENMASK(15, 0);
	}
	*value = ((u64)hi << 32) | lo;
unlock:
	mutex_unlock(&timer->lock);
	return ret;
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
	unsigned int i, num_counts;

	counter = devm_counter_alloc(&pdev->dev, sizeof(*timer));
	if (!counter)
		return -ENOMEM;
	timer = counter_priv(counter);
	mutex_init(&timer->lock);
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
		writel(readl(timer->base + RTC_TIMER_DATE) | RTC_TIMER_CLK_EN,
		       timer->base + RTC_TIMER_DATE);

	/* On P4 BUF1 is the previous snapshot of the same 48-bit counter. */
	num_counts = timer->kind == ESP32P4_RTC_TIMER ? 1 : ARRAY_SIZE(timer->counts);
	for (i = 0; i < num_counts; i++) {
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
	counter->num_counts = num_counts;
	return devm_counter_add(&pdev->dev, counter);
}

static const struct of_device_id esp32s31_system_timer_of_match[] = {
	{ .compatible = "espressif,esp32s31-systimer",
	  .data = (void *)ESP32S31_SYSTIMER },
	{ .compatible = "espressif,esp32s31-rtc-timer",
	  .data = (void *)ESP32S31_RTC_TIMER },
	{ .compatible = "espressif,esp32p4-rtc-timer",
	  .data = (void *)ESP32P4_RTC_TIMER },
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
MODULE_DESCRIPTION("Espressif SYSTIMER and RTC timer counter driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("COUNTER");
