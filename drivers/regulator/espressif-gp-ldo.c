// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 general-purpose LDO regulator. */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define LDO_TIE_HIGH		BIT(0)
#define LDO_RIPPLE_SUPPRESS	BIT(1)
#define LDO_CURRENT_LIMIT	BIT(2)
#define LDO_MUL		GENMASK(5, 3)
#define LDO_DREF		GENMASK(9, 6)
#define LDO_RAIL_SELECTOR	128

struct esp32s31_ldo {
	void __iomem *base;
};

static int esp32s31_ldo_voltage_uv(unsigned int selector)
{
	unsigned int dref, mul, vref20;

	if (selector == LDO_RAIL_SELECTOR)
		return 3300000;
	if (selector > LDO_RAIL_SELECTOR)
		return 0;
	dref = selector / 8;
	mul = selector % 8;
	vref20 = dref < 9 ? 10 + dref : 20 + (dref - 9) * 2;
	return DIV_ROUND_CLOSEST(vref20 * (4 + mul) * 1000000, 80);
}

static int esp32s31_ldo_list_voltage(struct regulator_dev *rdev,
				     unsigned int selector)
{
	return esp32s31_ldo_voltage_uv(selector);
}

static int esp32s31_ldo_get_voltage_sel(struct regulator_dev *rdev)
{
	struct esp32s31_ldo *ldo = rdev_get_drvdata(rdev);
	u32 val = readl(ldo->base);

	if (val & LDO_TIE_HIGH)
		return LDO_RAIL_SELECTOR;
	return FIELD_GET(LDO_DREF, val) * 8 + FIELD_GET(LDO_MUL, val);
}

static int esp32s31_ldo_is_enabled(struct regulator_dev *rdev)
{
	/* S31 exposes voltage control but no software enable bit for GP LDO1. */
	return 1;
}

static const struct regulator_ops esp32s31_ldo_ops = {
	.get_voltage_sel = esp32s31_ldo_get_voltage_sel,
	.is_enabled = esp32s31_ldo_is_enabled,
	.list_voltage = esp32s31_ldo_list_voltage,
};

static const struct regulator_desc esp32s31_ldo_desc = {
	.name = "esp32s31-gp-ldo1",
	.of_match = "gp-ldo1",
	.ops = &esp32s31_ldo_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.n_voltages = LDO_RAIL_SELECTOR + 1,
};

static int esp32s31_ldo_probe(struct platform_device *pdev)
{
	struct regulator_config config = { .dev = &pdev->dev };
	struct esp32s31_ldo *ldo;
	struct regulator_dev *rdev;
	u32 val;

	ldo = devm_kzalloc(&pdev->dev, sizeof(*ldo), GFP_KERNEL);
	if (!ldo)
		return -ENOMEM;
	ldo->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ldo->base))
		return PTR_ERR(ldo->base);

	val = readl(ldo->base);
	if (device_property_read_bool(&pdev->dev,
				      "espressif,ripple-suppression"))
		val |= LDO_RIPPLE_SUPPRESS;
	if (device_property_read_bool(&pdev->dev,
				      "espressif,current-limit"))
		val |= LDO_CURRENT_LIMIT;
	writel(val, ldo->base);

	config.driver_data = ldo;
	config.of_node = pdev->dev.of_node;
	config.init_data = of_get_regulator_init_data(&pdev->dev,
						      pdev->dev.of_node,
						      &esp32s31_ldo_desc);
	rdev = devm_regulator_register(&pdev->dev, &esp32s31_ldo_desc, &config);
	return PTR_ERR_OR_ZERO(rdev);
}

static const struct of_device_id esp32s31_ldo_of_match[] = {
	{ .compatible = "espressif,esp32s31-gp-ldo" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s31_ldo_of_match);

static struct platform_driver esp32s31_ldo_driver = {
	.probe = esp32s31_ldo_probe,
	.driver = {
		.name = "esp32s31-gp-ldo",
		.of_match_table = esp32s31_ldo_of_match,
	},
};
module_platform_driver(esp32s31_ldo_driver);

MODULE_DESCRIPTION("ESP32-S31 general-purpose LDO regulator driver");
MODULE_LICENSE("GPL");
