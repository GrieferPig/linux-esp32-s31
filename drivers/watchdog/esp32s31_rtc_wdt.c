// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S31 low-power RTC watchdog.
 *
 * Register programming follows ESP-IDF's ESP32-S31 rwdt_ll implementation.
 */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/watchdog.h>

#define RWDT_CONFIG0			0x00
#define RWDT_CONFIG1			0x04
#define RWDT_FEED			0x14
#define RWDT_WPROTECT			0x18
#define RWDT_DATE			0x3fc

#define RWDT_WKEY			0x50d83aa1
#define RWDT_SLOW_HZ			32768U
#define RWDT_DEFAULT_TIMEOUT		30U

#define RWDT_PAUSE_IN_SLEEP		BIT(9)
#define RWDT_APPCPU_RESET_EN		BIT(10)
#define RWDT_PROCPU_RESET_EN		BIT(11)
#define RWDT_FLASHBOOT_EN		BIT(12)
#define RWDT_SYS_RESET_LENGTH		GENMASK(15, 13)
#define RWDT_CPU_RESET_LENGTH		GENMASK(18, 16)
#define RWDT_STAGE3			GENMASK(21, 19)
#define RWDT_STAGE2			GENMASK(24, 22)
#define RWDT_STAGE1			GENMASK(27, 25)
#define RWDT_STAGE0			GENMASK(30, 28)
#define RWDT_ENABLE			BIT(31)
#define RWDT_CLOCK_ENABLE		BIT(31)

#define RWDT_STAGE_RESET_SYSTEM		3

struct esp32s31_rwdt {
	void __iomem *base;
	spinlock_t lock;
	struct watchdog_device wdd;
};

static void rwdt_unlock(struct esp32s31_rwdt *rwdt)
{
	writel(RWDT_WKEY, rwdt->base + RWDT_WPROTECT);
}

static void rwdt_lock(struct esp32s31_rwdt *rwdt)
{
	writel(0, rwdt->base + RWDT_WPROTECT);
}

static u32 rwdt_config(bool enable)
{
	u32 val;

	val = RWDT_PAUSE_IN_SLEEP | RWDT_APPCPU_RESET_EN |
	      RWDT_PROCPU_RESET_EN |
	      FIELD_PREP(RWDT_SYS_RESET_LENGTH, 7) |
	      FIELD_PREP(RWDT_CPU_RESET_LENGTH, 7) |
	      FIELD_PREP(RWDT_STAGE0, RWDT_STAGE_RESET_SYSTEM);
	if (enable)
		val |= RWDT_ENABLE;

	return val & ~(RWDT_FLASHBOOT_EN | RWDT_STAGE1 |
		       RWDT_STAGE2 | RWDT_STAGE3);
}

static void rwdt_program(struct esp32s31_rwdt *rwdt, bool enable)
{
	u64 ticks = (u64)rwdt->wdd.timeout * RWDT_SLOW_HZ;

	/*
	 * Stage 0 has an eFuse-selected x2/x4/x8/x16 implicit multiplier.
	 * Use the minimum multiplier so a timeout can never fire early.
	 */
	writel(min_t(u64, DIV_ROUND_UP_ULL(ticks, 2), U32_MAX),
	       rwdt->base + RWDT_CONFIG1);
	writel(rwdt_config(enable), rwdt->base + RWDT_CONFIG0);
	writel(BIT(31), rwdt->base + RWDT_FEED);
}

static int rwdt_start(struct watchdog_device *wdd)
{
	struct esp32s31_rwdt *rwdt = watchdog_get_drvdata(wdd);
	unsigned long flags;

	spin_lock_irqsave(&rwdt->lock, flags);
	rwdt_unlock(rwdt);
	writel(readl(rwdt->base + RWDT_DATE) | RWDT_CLOCK_ENABLE,
	       rwdt->base + RWDT_DATE);
	rwdt_program(rwdt, true);
	rwdt_lock(rwdt);
	spin_unlock_irqrestore(&rwdt->lock, flags);
	return 0;
}

static int rwdt_stop(struct watchdog_device *wdd)
{
	struct esp32s31_rwdt *rwdt = watchdog_get_drvdata(wdd);
	unsigned long flags;

	spin_lock_irqsave(&rwdt->lock, flags);
	rwdt_unlock(rwdt);
	writel(rwdt_config(false), rwdt->base + RWDT_CONFIG0);
	rwdt_lock(rwdt);
	spin_unlock_irqrestore(&rwdt->lock, flags);
	return 0;
}

static int rwdt_ping(struct watchdog_device *wdd)
{
	struct esp32s31_rwdt *rwdt = watchdog_get_drvdata(wdd);
	unsigned long flags;

	spin_lock_irqsave(&rwdt->lock, flags);
	rwdt_unlock(rwdt);
	writel(BIT(31), rwdt->base + RWDT_FEED);
	rwdt_lock(rwdt);
	spin_unlock_irqrestore(&rwdt->lock, flags);
	return 0;
}

static int rwdt_set_timeout(struct watchdog_device *wdd, unsigned int timeout)
{
	struct esp32s31_rwdt *rwdt = watchdog_get_drvdata(wdd);
	unsigned long flags;

	wdd->timeout = timeout;
	if (!watchdog_active(wdd))
		return 0;
	spin_lock_irqsave(&rwdt->lock, flags);
	rwdt_unlock(rwdt);
	rwdt_program(rwdt, true);
	rwdt_lock(rwdt);
	spin_unlock_irqrestore(&rwdt->lock, flags);
	return 0;
}

static const struct watchdog_info rwdt_info = {
	.identity = "ESP32-S31 RTC watchdog",
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops rwdt_ops = {
	.owner = THIS_MODULE,
	.start = rwdt_start,
	.stop = rwdt_stop,
	.ping = rwdt_ping,
	.set_timeout = rwdt_set_timeout,
};

static int esp32s31_rwdt_probe(struct platform_device *pdev)
{
	struct esp32s31_rwdt *rwdt;
	int ret;

	rwdt = devm_kzalloc(&pdev->dev, sizeof(*rwdt), GFP_KERNEL);
	if (!rwdt)
		return -ENOMEM;
	rwdt->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(rwdt->base))
		return PTR_ERR(rwdt->base);

	spin_lock_init(&rwdt->lock);
	rwdt->wdd.info = &rwdt_info;
	rwdt->wdd.ops = &rwdt_ops;
	rwdt->wdd.parent = &pdev->dev;
	rwdt->wdd.min_timeout = 1;
	rwdt->wdd.max_timeout = U32_MAX / RWDT_SLOW_HZ;
	rwdt->wdd.timeout = RWDT_DEFAULT_TIMEOUT;
	watchdog_set_drvdata(&rwdt->wdd, rwdt);
	watchdog_set_nowayout(&rwdt->wdd, WATCHDOG_NOWAYOUT);
	watchdog_stop_on_reboot(&rwdt->wdd);
	watchdog_stop_on_unregister(&rwdt->wdd);

	ret = watchdog_init_timeout(&rwdt->wdd, 0, &pdev->dev);
	if (ret)
		return ret;
	rwdt_stop(&rwdt->wdd);
	return devm_watchdog_register_device(&pdev->dev, &rwdt->wdd);
}

static const struct of_device_id esp32s31_rwdt_of_match[] = {
	{ .compatible = "espressif,esp32s31-rtc-wdt" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s31_rwdt_of_match);

static struct platform_driver esp32s31_rwdt_driver = {
	.probe = esp32s31_rwdt_probe,
	.driver = {
		.name = "esp32s31-rtc-wdt",
		.of_match_table = esp32s31_rwdt_of_match,
	},
};
module_platform_driver(esp32s31_rwdt_driver);

MODULE_DESCRIPTION("Espressif ESP32-S31 RTC watchdog");
MODULE_LICENSE("GPL");
