// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 on-chip temperature sensor, register flow from ESP-IDF. */

#include <linux/bitfield.h>
#include <linux/hwmon.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#define ESP32S31_MODEM_SYSCON_CLK	0x20109c04
#define ESP32S31_MODEM_LPCON_CLK	0x2010f018
#define ESP32S31_ANA_I2C		0x20111800
#define MODEM_I2C_CLK_EN		BIT(2)
#define MODEM_I2C_CLK_160M		BIT(12)
#define REGI2C_BUSY			BIT(25)
#define REGI2C_WRITE			BIT(24)
#define REGI2C_PERIF_ID			0x69
#define REGI2C_PERIF_SEL			BIT(9)
#define REGI2C_CONF1			0x1c
#define REGI2C_CONF2			0x20
#define REGI2C_TSENS_DAC			0x06
#define REGI2C_TSENS_ENABLE		0x07
#define TSENS_CTRL			0x00
#define TSENS_INT_ST			0x0c
#define TSENS_INT_CLR			0x14
#define TSENS_CLK_CONF			0x18
#define TSENS_INT_ENA_W1TS		0x1c
#define TSENS_INT_ENA_W1TC		0x20
#define TSENS_WAKEUP_CTRL		0x24
#define TSENS_SAMPLE_RATE		0x28
#define TSENS_READY			BIT(8)
#define TSENS_CLK_DIV			GENMASK(21, 14)
#define TSENS_SAMPLE			BIT(9)
#define TSENS_INT_ENABLE		BIT(12)
#define TSENS_POWER			BIT(22)
#define TSENS_REG_CLK			BIT(0)
#define TSENS_WAKE_INT			BIT(0)
#define TSENS_WAKE_LOW			GENMASK(7, 0)
#define TSENS_WAKE_HIGH		GENMASK(21, 14)
#define TSENS_WAKE_OVER_HIGH		BIT(29)
#define TSENS_WAKE_ENABLE		BIT(30)
#define TSENS_WAKE_RELATIVE		BIT(31)
#define LP_TSENS_CLK_EN			BIT(30)
#define LP_TSENS_RST			BIT(31)

struct esp32s31_tsens {
	void __iomem *base;
	void __iomem *clkrst;
	struct device *hwmon;
	struct mutex lock;
	long low_mc;
	long high_mc;
	int irq;
};

static long esp32s31_tsens_raw_to_mc(u8 raw)
{
	/* IDF range with offset 0: C = 0.4386 * raw - 20.52. */
	return ((long)raw * 4386 - 205200) / 10;
}

static u8 esp32s31_tsens_mc_to_raw(long value)
{
	long raw = DIV_ROUND_CLOSEST(value * 10 + 205200, 4386);

	return clamp_val(raw, 0, U8_MAX);
}

static void esp32s31_tsens_program_thresholds(struct esp32s31_tsens *tsens)
{
	u32 ctrl = FIELD_PREP(TSENS_WAKE_LOW,
			      esp32s31_tsens_mc_to_raw(tsens->low_mc)) |
		   FIELD_PREP(TSENS_WAKE_HIGH,
			      esp32s31_tsens_mc_to_raw(tsens->high_mc)) |
		   TSENS_WAKE_ENABLE;

	/* Absolute comparison mode; relative/delta mode is not a hwmon ABI. */
	ctrl &= ~TSENS_WAKE_RELATIVE;
	writel(ctrl, tsens->base + TSENS_WAKEUP_CTRL);
}

static irqreturn_t esp32s31_tsens_irq(int irq, void *data)
{
	struct esp32s31_tsens *tsens = data;
	u32 status = readl(tsens->base + TSENS_INT_ST);
	u32 wake;

	if (!(status & TSENS_WAKE_INT))
		return IRQ_NONE;
	wake = readl(tsens->base + TSENS_WAKEUP_CTRL);
	/*
	 * The wake comparator is level-sensitive.  Leave it masked after the
	 * notification so a sustained over/under-temperature condition cannot
	 * monopolize the CPU.  Reprogramming either limit acknowledges the event
	 * and arms the comparator again.
	 */
	writel(TSENS_WAKE_INT, tsens->base + TSENS_INT_ENA_W1TC);
	writel(TSENS_WAKE_INT, tsens->base + TSENS_INT_CLR);
	hwmon_notify_event(tsens->hwmon, hwmon_temp,
			   wake & TSENS_WAKE_OVER_HIGH ?
			   hwmon_temp_max_alarm : hwmon_temp_min_alarm, 0);
	return IRQ_HANDLED;
}

static int esp32s31_tsens_set_range(struct device *dev, u8 range)
{
	void __iomem *i2c, *lpcon, *syscon;
	u32 ctrl, reg;
	unsigned int bus;
	int ret;

	i2c = devm_ioremap(dev, ESP32S31_ANA_I2C, 0x40);
	lpcon = devm_ioremap(dev, ESP32S31_MODEM_LPCON_CLK, 4);
	syscon = devm_ioremap(dev, ESP32S31_MODEM_SYSCON_CLK, 4);
	if (!i2c || !lpcon || !syscon)
		return -ENOMEM;

	writel(readl(lpcon) | MODEM_I2C_CLK_EN, lpcon);
	writel(readl(syscon) | MODEM_I2C_CLK_160M, syscon);
	bus = !!(readl(i2c + REGI2C_CONF2) & (REGI2C_PERIF_SEL << 4));
	/* Same slave routing operation as IDF regi2c_enable_block(). */
	writel((~(REGI2C_PERIF_SEL << 2)) & GENMASK(23, 0),
	       i2c + REGI2C_CONF1);
	ret = readl_poll_timeout(i2c + bus * 4, ctrl,
				 !(ctrl & REGI2C_BUSY), 1, 10000);
	if (ret)
		return ret;
	writel(REGI2C_PERIF_ID | (REGI2C_TSENS_DAC << 8), i2c + bus * 4);
	ret = readl_poll_timeout(i2c + bus * 4, ctrl,
				 !(ctrl & REGI2C_BUSY), 1, 10000);
	if (ret)
		return ret;
	reg = (ctrl >> 16) & 0xff;
	reg = (reg & ~GENMASK(3, 0)) | (range & GENMASK(3, 0));
	writel(REGI2C_PERIF_ID | (REGI2C_TSENS_DAC << 8) |
	       (reg << 16) | REGI2C_WRITE, i2c + bus * 4);
	ret = readl_poll_timeout(i2c + bus * 4, ctrl,
				 !(ctrl & REGI2C_BUSY), 1, 10000);
	if (ret)
		return ret;
	/* Route the analog temperature sensor output into the SAR peripheral. */
	writel(REGI2C_PERIF_ID | (REGI2C_TSENS_ENABLE << 8),
	       i2c + bus * 4);
	ret = readl_poll_timeout(i2c + bus * 4, ctrl,
				 !(ctrl & REGI2C_BUSY), 1, 10000);
	if (ret)
		return ret;
	reg = ((ctrl >> 16) & 0xff) | BIT(2);
	writel(REGI2C_PERIF_ID | (REGI2C_TSENS_ENABLE << 8) |
	       (reg << 16) | REGI2C_WRITE, i2c + bus * 4);
	return readl_poll_timeout(i2c + bus * 4, ctrl,
				  !(ctrl & REGI2C_BUSY), 1, 10000);
}

static umode_t esp32s31_tsens_is_visible(const void *data,
					 enum hwmon_sensor_types type,
					 u32 attr, int channel)
{
	if (type != hwmon_temp)
		return 0;
	switch (attr) {
	case hwmon_temp_input:
	case hwmon_temp_min_alarm:
	case hwmon_temp_max_alarm:
		return 0444;
	case hwmon_temp_min:
	case hwmon_temp_max:
		return 0644;
	default:
		return 0;
	}
}

static int esp32s31_tsens_read(struct device *dev,
			       enum hwmon_sensor_types type,
			       u32 attr, int channel, long *value)
{
	struct esp32s31_tsens *tsens = dev_get_drvdata(dev);
	u32 ctrl, raw;

	if (type != hwmon_temp)
		return -EOPNOTSUPP;
	ctrl = readl(tsens->base + TSENS_CTRL);
	raw = ctrl & 0xff;
	switch (attr) {
	case hwmon_temp_input:
		*value = esp32s31_tsens_raw_to_mc(raw);
		return 0;
	case hwmon_temp_min:
		*value = READ_ONCE(tsens->low_mc);
		return 0;
	case hwmon_temp_max:
		*value = READ_ONCE(tsens->high_mc);
		return 0;
	case hwmon_temp_min_alarm:
		*value = raw < esp32s31_tsens_mc_to_raw(READ_ONCE(tsens->low_mc));
		return 0;
	case hwmon_temp_max_alarm:
		*value = raw > esp32s31_tsens_mc_to_raw(READ_ONCE(tsens->high_mc));
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int esp32s31_tsens_write(struct device *dev,
				enum hwmon_sensor_types type,
				u32 attr, int channel, long value)
{
	struct esp32s31_tsens *tsens = dev_get_drvdata(dev);
	int ret = 0;

	if (type != hwmon_temp ||
	    (attr != hwmon_temp_min && attr != hwmon_temp_max))
		return -EOPNOTSUPP;
	if (value < esp32s31_tsens_raw_to_mc(0) ||
	    value > esp32s31_tsens_raw_to_mc(U8_MAX))
		return -ERANGE;

	mutex_lock(&tsens->lock);
	if ((attr == hwmon_temp_min && value >= tsens->high_mc) ||
	    (attr == hwmon_temp_max && value <= tsens->low_mc)) {
		ret = -EINVAL;
		goto out;
	}
	if (attr == hwmon_temp_min)
		tsens->low_mc = value;
	else
		tsens->high_mc = value;
	esp32s31_tsens_program_thresholds(tsens);
	writel(TSENS_WAKE_INT, tsens->base + TSENS_INT_CLR);
	writel(TSENS_WAKE_INT, tsens->base + TSENS_INT_ENA_W1TS);
out:
	mutex_unlock(&tsens->lock);
	return ret;
}

static const struct hwmon_ops esp32s31_tsens_ops = {
	.is_visible = esp32s31_tsens_is_visible,
	.read = esp32s31_tsens_read,
	.write = esp32s31_tsens_write,
};

static const struct hwmon_channel_info * const esp32s31_tsens_info[] = {
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT | HWMON_T_MIN | HWMON_T_MAX |
			   HWMON_T_MIN_ALARM | HWMON_T_MAX_ALARM),
	NULL
};

static const struct hwmon_chip_info esp32s31_tsens_chip_info = {
	.ops = &esp32s31_tsens_ops,
	.info = esp32s31_tsens_info,
};

static int esp32s31_tsens_probe(struct platform_device *pdev)
{
	struct esp32s31_tsens *tsens;
	void __iomem *clkrst;
	struct device *hwmon;
	int ret;

	tsens = devm_kzalloc(&pdev->dev, sizeof(*tsens), GFP_KERNEL);
	if (!tsens)
		return -ENOMEM;
	mutex_init(&tsens->lock);
	tsens->low_mc = -10000;
	tsens->high_mc = 80000;
	tsens->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(tsens->base))
		return PTR_ERR(tsens->base);
	clkrst = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(clkrst))
		return PTR_ERR(clkrst);
	tsens->clkrst = clkrst;

	ret = devm_regulator_get_enable(&pdev->dev, "analog");
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "analog I2C power unavailable\n");

	writel(readl(clkrst) | LP_TSENS_CLK_EN | LP_TSENS_RST, clkrst);
	writel(readl(clkrst) & ~LP_TSENS_RST, clkrst);
	writel(TSENS_REG_CLK, tsens->base + TSENS_CLK_CONF);
	/* IDF's middle range: DAC 15, -10..80 C, nominal 1 C error. */
	ret = esp32s31_tsens_set_range(&pdev->dev, 15);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "analog range setup timed out\n");
	/* IDF only sets power_up; power_up_force is not used by its driver. */
	writel(FIELD_PREP(TSENS_CLK_DIV, 6) | TSENS_SAMPLE | TSENS_INT_ENABLE |
	       TSENS_POWER,
	       tsens->base + TSENS_CTRL);
	writel(20, tsens->base + TSENS_SAMPLE_RATE);
	esp32s31_tsens_program_thresholds(tsens);
	writel(TSENS_WAKE_INT, tsens->base + TSENS_INT_CLR);
	udelay(300);
	platform_set_drvdata(pdev, tsens);
	hwmon = devm_hwmon_device_register_with_info(&pdev->dev,
						     "esp32s31_tsens", tsens,
						     &esp32s31_tsens_chip_info,
						     NULL);
	if (IS_ERR(hwmon))
		return PTR_ERR(hwmon);
	tsens->hwmon = hwmon;
	tsens->irq = platform_get_irq(pdev, 0);
	if (tsens->irq < 0)
		return tsens->irq;
	ret = devm_request_irq(&pdev->dev, tsens->irq, esp32s31_tsens_irq,
			       0, dev_name(&pdev->dev), tsens);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "threshold interrupt unavailable\n");
	writel(TSENS_WAKE_INT, tsens->base + TSENS_INT_ENA_W1TS);
	return 0;
}

static void esp32s31_tsens_remove(struct platform_device *pdev)
{
	struct esp32s31_tsens *tsens = platform_get_drvdata(pdev);

	writel(TSENS_WAKE_INT, tsens->base + TSENS_INT_ENA_W1TC);
	writel(TSENS_WAKE_INT, tsens->base + TSENS_INT_CLR);
}

static const struct of_device_id esp32s31_tsens_of_match[] = {
	{ .compatible = "espressif,esp32s31-tsens" }, { }
};
MODULE_DEVICE_TABLE(of, esp32s31_tsens_of_match);

static struct platform_driver esp32s31_tsens_driver = {
	.probe = esp32s31_tsens_probe,
	.remove = esp32s31_tsens_remove,
	.driver = {
		.name = "esp32s31-tsens",
		.of_match_table = esp32s31_tsens_of_match,
	},
};
module_platform_driver(esp32s31_tsens_driver);
MODULE_DESCRIPTION("ESP32-S31 temperature sensor");
MODULE_LICENSE("GPL");
