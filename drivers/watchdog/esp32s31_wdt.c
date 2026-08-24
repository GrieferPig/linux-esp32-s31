// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S31 timer-group main watchdog
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/watchdog.h>

#define ESP32S31_WDT_CONFIG0		0x48
#define ESP32S31_WDT_CONFIG1		0x4c
#define ESP32S31_WDT_CONFIG2		0x50
#define ESP32S31_WDT_FEED		0x60
#define ESP32S31_WDT_WPROTECT		0x64

#define ESP32S31_WDT_WKEY		0x50d83aa1
#define ESP32S31_WDT_PRESCALER		20000
#define ESP32S31_WDT_TICKS_PER_SEC	2000

#define ESP32S31_WDT_APPCPU_RESET_EN	BIT(12)
#define ESP32S31_WDT_PROCPU_RESET_EN	BIT(13)
#define ESP32S31_WDT_FLASHBOOT_EN	BIT(14)
#define ESP32S31_WDT_SYS_RESET_LENGTH	GENMASK(17, 15)
#define ESP32S31_WDT_CPU_RESET_LENGTH	GENMASK(20, 18)
#define ESP32S31_WDT_CONF_UPDATE_EN	BIT(22)
#define ESP32S31_WDT_STG3		GENMASK(24, 23)
#define ESP32S31_WDT_STG2		GENMASK(26, 25)
#define ESP32S31_WDT_STG1		GENMASK(28, 27)
#define ESP32S31_WDT_STG0		GENMASK(30, 29)
#define ESP32S31_WDT_EN			BIT(31)

#define ESP32S31_WDT_STAGE_OFF		0
#define ESP32S31_WDT_STAGE_RESET_SYSTEM	3

#define ESP32S31_WDT_PRESCALE		GENMASK(31, 16)

#define ESP32S31_WDT_DEFAULT_TIMEOUT	30
#define ESP32S31_WDT_MAX_TIMEOUT	(UINT_MAX / ESP32S31_WDT_TICKS_PER_SEC)

struct esp32s31_wdt {
	void __iomem *base;
	struct clk *clk;
	spinlock_t lock;
	struct watchdog_device wdd;
};

static void esp32s31_wdt_unlock(struct esp32s31_wdt *priv)
{
	writel(ESP32S31_WDT_WKEY, priv->base + ESP32S31_WDT_WPROTECT);
}

static void esp32s31_wdt_lock(struct esp32s31_wdt *priv)
{
	writel(0, priv->base + ESP32S31_WDT_WPROTECT);
}

static u32 esp32s31_wdt_config0(bool enable)
{
	u32 val;

	val = FIELD_PREP(ESP32S31_WDT_SYS_RESET_LENGTH, 7) |
	      FIELD_PREP(ESP32S31_WDT_CPU_RESET_LENGTH, 7) |
	      FIELD_PREP(ESP32S31_WDT_STG0,
			 ESP32S31_WDT_STAGE_RESET_SYSTEM) |
	      FIELD_PREP(ESP32S31_WDT_STG1, ESP32S31_WDT_STAGE_OFF) |
	      FIELD_PREP(ESP32S31_WDT_STG2, ESP32S31_WDT_STAGE_OFF) |
	      FIELD_PREP(ESP32S31_WDT_STG3, ESP32S31_WDT_STAGE_OFF) |
	      ESP32S31_WDT_APPCPU_RESET_EN |
	      ESP32S31_WDT_PROCPU_RESET_EN |
	      ESP32S31_WDT_CONF_UPDATE_EN;

	if (enable)
		val |= ESP32S31_WDT_EN;

	/* Flash-boot mode is independent of WDT_EN and must stay disabled. */
	return val & ~ESP32S31_WDT_FLASHBOOT_EN;
}

static void esp32s31_wdt_program(struct esp32s31_wdt *priv, bool enable)
{
	writel(FIELD_PREP(ESP32S31_WDT_PRESCALE,
			  ESP32S31_WDT_PRESCALER),
	       priv->base + ESP32S31_WDT_CONFIG1);
	writel(priv->wdd.timeout * ESP32S31_WDT_TICKS_PER_SEC,
	       priv->base + ESP32S31_WDT_CONFIG2);
	writel(esp32s31_wdt_config0(enable),
	       priv->base + ESP32S31_WDT_CONFIG0);
	writel(1, priv->base + ESP32S31_WDT_FEED);
}

static int esp32s31_wdt_start(struct watchdog_device *wdd)
{
	struct esp32s31_wdt *priv = watchdog_get_drvdata(wdd);
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	esp32s31_wdt_unlock(priv);
	esp32s31_wdt_program(priv, true);
	esp32s31_wdt_lock(priv);
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int esp32s31_wdt_stop(struct watchdog_device *wdd)
{
	struct esp32s31_wdt *priv = watchdog_get_drvdata(wdd);
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	esp32s31_wdt_unlock(priv);
	writel(esp32s31_wdt_config0(false),
	       priv->base + ESP32S31_WDT_CONFIG0);
	esp32s31_wdt_lock(priv);
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int esp32s31_wdt_ping(struct watchdog_device *wdd)
{
	struct esp32s31_wdt *priv = watchdog_get_drvdata(wdd);
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	esp32s31_wdt_unlock(priv);
	writel(1, priv->base + ESP32S31_WDT_FEED);
	esp32s31_wdt_lock(priv);
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int esp32s31_wdt_set_timeout(struct watchdog_device *wdd,
				    unsigned int timeout)
{
	struct esp32s31_wdt *priv = watchdog_get_drvdata(wdd);
	unsigned long flags;

	wdd->timeout = timeout;

	if (!watchdog_active(wdd))
		return 0;

	spin_lock_irqsave(&priv->lock, flags);
	esp32s31_wdt_unlock(priv);
	writel(timeout * ESP32S31_WDT_TICKS_PER_SEC,
	       priv->base + ESP32S31_WDT_CONFIG2);
	writel(readl(priv->base + ESP32S31_WDT_CONFIG0) |
	       ESP32S31_WDT_CONF_UPDATE_EN,
	       priv->base + ESP32S31_WDT_CONFIG0);
	writel(1, priv->base + ESP32S31_WDT_FEED);
	esp32s31_wdt_lock(priv);
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static const struct watchdog_info esp32s31_wdt_info = {
	.identity = "ESP32-S31 timer-group watchdog",
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops esp32s31_wdt_ops = {
	.owner = THIS_MODULE,
	.start = esp32s31_wdt_start,
	.stop = esp32s31_wdt_stop,
	.ping = esp32s31_wdt_ping,
	.set_timeout = esp32s31_wdt_set_timeout,
};

static int esp32s31_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp32s31_wdt *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(priv->clk))
		return dev_err_probe(dev, PTR_ERR(priv->clk),
				     "failed to enable watchdog clock\n");

	spin_lock_init(&priv->lock);

	priv->wdd.info = &esp32s31_wdt_info;
	priv->wdd.ops = &esp32s31_wdt_ops;
	priv->wdd.parent = dev;
	priv->wdd.min_timeout = 1;
	priv->wdd.max_timeout = ESP32S31_WDT_MAX_TIMEOUT;
	priv->wdd.timeout = ESP32S31_WDT_DEFAULT_TIMEOUT;

	watchdog_set_drvdata(&priv->wdd, priv);
	watchdog_set_nowayout(&priv->wdd, WATCHDOG_NOWAYOUT);
	watchdog_stop_on_reboot(&priv->wdd);
	watchdog_stop_on_unregister(&priv->wdd);

	ret = watchdog_init_timeout(&priv->wdd, 0, dev);
	if (ret)
		return ret;

	/*
	 * Boot ROM enables flash-boot protection independently of WDT_EN.
	 * Linux owns TIMERG1, so leave it disabled until /dev/watchdog is
	 * explicitly opened.
	 */
	esp32s31_wdt_stop(&priv->wdd);

	return devm_watchdog_register_device(dev, &priv->wdd);
}

static const struct of_device_id esp32s31_wdt_of_match[] = {
	{ .compatible = "espressif,esp32s31-timg-wdt" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s31_wdt_of_match);

static struct platform_driver esp32s31_wdt_driver = {
	.probe = esp32s31_wdt_probe,
	.driver = {
		.name = "esp32s31-wdt",
		.of_match_table = esp32s31_wdt_of_match,
	},
};
module_platform_driver(esp32s31_wdt_driver);

MODULE_DESCRIPTION("Espressif ESP32-S31 timer-group watchdog");
MODULE_LICENSE("GPL");
