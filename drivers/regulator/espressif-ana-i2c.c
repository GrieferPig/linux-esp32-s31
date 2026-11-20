// SPDX-License-Identifier: GPL-2.0-only
/* Shared analog register-I2C power switches. */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define ANA_I2C_XPD	BIT(30)
#define ANA_I2C_RSTB	BIT(31)

/* ESP32-P4 TRM ch.16 Low-Power Management — PMU_RF_PWC. */
#define P4_PERIF_I2C_RSTB	BIT(26)
#define P4_XPD_PERIF_I2C	BIT(27)

struct esp_ana_i2c_soc {
	struct regulator_desc desc;
	u32 power;
	u32 reset_release;
	bool separate_power_on;
};

struct esp32s31_ana_i2c {
	void __iomem *reg;
	const struct esp_ana_i2c_soc *soc;
};

static int esp32s31_ana_i2c_enable(struct regulator_dev *rdev)
{
	struct esp32s31_ana_i2c *ana = rdev_get_drvdata(rdev);
	u32 value = readl(ana->reg);
	u32 enable = ana->soc->power | ana->soc->reset_release;

	/* IDF regi2c_saradc_enable(): reset, power up, release reset. */
	if (ana->soc->separate_power_on && (value & enable) == enable)
		return 0;
	writel(value & ~ana->soc->reset_release, ana->reg);
	udelay(1);
	if (ana->soc->separate_power_on)
		writel((value | ana->soc->power) & ~ana->soc->reset_release, ana->reg);
	writel(value | enable, ana->reg);
	readl(ana->reg);

	return 0;
}

static int esp32s31_ana_i2c_disable(struct regulator_dev *rdev)
{
	struct esp32s31_ana_i2c *ana = rdev_get_drvdata(rdev);
	u32 value = readl(ana->reg);

	writel(value & ~(ana->soc->power | ana->soc->reset_release), ana->reg);
	readl(ana->reg);

	return 0;
}

static int esp32s31_ana_i2c_is_enabled(struct regulator_dev *rdev)
{
	struct esp32s31_ana_i2c *ana = rdev_get_drvdata(rdev);
	u32 value = readl(ana->reg);
	u32 mask = ana->soc->power | ana->soc->reset_release;

	return (value & mask) == mask;
}

static const struct regulator_ops esp32s31_ana_i2c_ops = {
	.enable = esp32s31_ana_i2c_enable,
	.disable = esp32s31_ana_i2c_disable,
	.is_enabled = esp32s31_ana_i2c_is_enabled,
};

static const struct esp_ana_i2c_soc esp32s31_ana_i2c_soc = {
	.desc = {
		.name = "esp32s31-ana-i2c",
		.of_match = "ana-i2c",
		.ops = &esp32s31_ana_i2c_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.n_voltages = 1,
		.fixed_uV = 1,
	},
	.power = ANA_I2C_XPD,
	.reset_release = ANA_I2C_RSTB,
};

static const struct esp_ana_i2c_soc esp32p4_ana_i2c_soc = {
	.desc = {
		.name = "esp32p4-ana-i2c",
		.ops = &esp32s31_ana_i2c_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	.power = P4_XPD_PERIF_I2C,
	.reset_release = P4_PERIF_I2C_RSTB,
	.separate_power_on = true,
};

static int esp32s31_ana_i2c_probe(struct platform_device *pdev)
{
	struct regulator_config config = { .dev = &pdev->dev };
	struct esp32s31_ana_i2c *ana;
	struct regulator_dev *rdev;

	ana = devm_kzalloc(&pdev->dev, sizeof(*ana), GFP_KERNEL);
	if (!ana)
		return -ENOMEM;
	ana->soc = of_device_get_match_data(&pdev->dev);
	ana->reg = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ana->reg))
		return PTR_ERR(ana->reg);

	config.driver_data = ana;
	config.of_node = pdev->dev.of_node;
	config.init_data = of_get_regulator_init_data(&pdev->dev,
						      pdev->dev.of_node,
						      &ana->soc->desc);
	rdev = devm_regulator_register(&pdev->dev, &ana->soc->desc,
					&config);
	return PTR_ERR_OR_ZERO(rdev);
}

static const struct of_device_id esp32s31_ana_i2c_of_match[] = {
	{ .compatible = "espressif,esp32s31-ana-i2c", .data = &esp32s31_ana_i2c_soc },
	{ .compatible = "espressif,esp32p4-ana-i2c", .data = &esp32p4_ana_i2c_soc },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s31_ana_i2c_of_match);

static struct platform_driver esp32s31_ana_i2c_driver = {
	.probe = esp32s31_ana_i2c_probe,
	.driver = {
		.name = "esp32s31-ana-i2c",
		.of_match_table = esp32s31_ana_i2c_of_match,
	},
};
builtin_platform_driver(esp32s31_ana_i2c_driver);

MODULE_DESCRIPTION("Espressif shared analog I2C power regulator");
MODULE_LICENSE("GPL");
