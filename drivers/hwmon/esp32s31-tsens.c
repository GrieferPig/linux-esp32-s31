// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 on-chip temperature sensor, register flow from ESP-IDF. */

#include <linux/bitfield.h>
#include <linux/hwmon.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define ESP32S31_PMU_ANA_PWR		0x20704208
#define ESP32S31_MODEM_SYSCON_CLK	0x20109c04
#define ESP32S31_MODEM_LPCON_CLK	0x2010f018
#define ESP32S31_ANA_I2C		0x20111800
#define PMU_XPD_PERIF_I2C		BIT(30)
#define PMU_RSTB_PERIF_I2C		BIT(31)
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
#define TSENS_CLK_CONF			0x18
#define TSENS_READY			BIT(8)
#define TSENS_CLK_DIV			GENMASK(21, 14)
#define TSENS_SAMPLE			BIT(9)
#define TSENS_POWER			BIT(22)
#define TSENS_REG_CLK			BIT(0)
#define LP_TSENS_CLK_EN			BIT(30)
#define LP_TSENS_RST			BIT(31)

struct esp32s31_tsens {
	void __iomem *base;
	void __iomem *clkrst;
};

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
	return type == hwmon_temp && attr == hwmon_temp_input ? 0444 : 0;
}

static int esp32s31_tsens_read(struct device *dev,
			       enum hwmon_sensor_types type,
			       u32 attr, int channel, long *value)
{
	struct esp32s31_tsens *tsens = dev_get_drvdata(dev);
	u32 ctrl;

	/*
	 * IDF reads OUT directly after the 300 us settling delay. READY is a
	 * pulse and is not a persistent "new sample" flag on S31.
	 */
	ctrl = readl(tsens->base + TSENS_CTRL);
	/* IDF range with offset 0: C = 0.4386 * raw - 20.52. */
	*value = ((long)(ctrl & 0xff) * 4386 - 205200) / 10;
	return 0;
}

static const struct hwmon_ops esp32s31_tsens_ops = {
	.is_visible = esp32s31_tsens_is_visible,
	.read = esp32s31_tsens_read,
};

static const struct hwmon_channel_info * const esp32s31_tsens_info[] = {
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT),
	NULL
};

static const struct hwmon_chip_info esp32s31_tsens_chip_info = {
	.ops = &esp32s31_tsens_ops,
	.info = esp32s31_tsens_info,
};

static int esp32s31_tsens_probe(struct platform_device *pdev)
{
	struct esp32s31_tsens *tsens;
	void __iomem *ana_pwr;
	void __iomem *clkrst;
	struct device *hwmon;
	u32 val;
	int ret;

	tsens = devm_kzalloc(&pdev->dev, sizeof(*tsens), GFP_KERNEL);
	if (!tsens)
		return -ENOMEM;
	tsens->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(tsens->base))
		return PTR_ERR(tsens->base);
	clkrst = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(clkrst))
		return PTR_ERR(clkrst);
	tsens->clkrst = clkrst;

	/* IDF regi2c_saradc_enable(); shared with the ADC analog block. */
	ana_pwr = devm_ioremap(&pdev->dev, ESP32S31_PMU_ANA_PWR, 4);
	if (!ana_pwr)
		return -ENOMEM;
	val = readl(ana_pwr);
	writel(val & ~PMU_RSTB_PERIF_I2C, ana_pwr);
	udelay(1);
	writel(val | PMU_XPD_PERIF_I2C | PMU_RSTB_PERIF_I2C, ana_pwr);

	writel(readl(clkrst) | LP_TSENS_CLK_EN | LP_TSENS_RST, clkrst);
	writel(readl(clkrst) & ~LP_TSENS_RST, clkrst);
	writel(TSENS_REG_CLK, tsens->base + TSENS_CLK_CONF);
	/* IDF's middle range: DAC 15, -10..80 C, nominal 1 C error. */
	ret = esp32s31_tsens_set_range(&pdev->dev, 15);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "analog range setup timed out\n");
	/* IDF only sets power_up; power_up_force is not used by its driver. */
	writel(FIELD_PREP(TSENS_CLK_DIV, 6) | TSENS_SAMPLE | TSENS_POWER,
	       tsens->base + TSENS_CTRL);
	udelay(300);
	platform_set_drvdata(pdev, tsens);
	hwmon = devm_hwmon_device_register_with_info(&pdev->dev,
						     "esp32s31_tsens", tsens,
						     &esp32s31_tsens_chip_info,
						     NULL);
	return PTR_ERR_OR_ZERO(hwmon);
}

static const struct of_device_id esp32s31_tsens_of_match[] = {
	{ .compatible = "espressif,esp32s31-tsens" }, { }
};
MODULE_DEVICE_TABLE(of, esp32s31_tsens_of_match);

static struct platform_driver esp32s31_tsens_driver = {
	.probe = esp32s31_tsens_probe,
	.driver = {
		.name = "esp32s31-tsens",
		.of_match_table = esp32s31_tsens_of_match,
	},
};
module_platform_driver(esp32s31_tsens_driver);
MODULE_DESCRIPTION("ESP32-S31 temperature sensor");
MODULE_LICENSE("GPL");
