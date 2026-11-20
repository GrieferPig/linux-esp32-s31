// SPDX-License-Identifier: GPL-2.0-only
#include <linux/bits.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

/* TRM ch.16 Low-Power Management - PMU_EXT_LDO_P1_0P2A_REG.
 * Software selects the 3.3 V bypass rail before asserting XPD.
 */
#define VO4_SW_MODE	BIT(7)
#define VO4_XPD		BIT(8)
#define VO4_MODE	GENMASK(11, 9)
#define VO4_BYPASS	BIT(10)

static int esp32p4_vo4_enable(struct regulator_dev *rdev)
{
	void __iomem *ctrl = rdev_get_drvdata(rdev);
	u32 val = readl(ctrl);

	val &= ~VO4_MODE;
	val |= VO4_SW_MODE | VO4_BYPASS;
	writel(val, ctrl);
	writel(val | VO4_XPD, ctrl);
	return 0;
}

static int esp32p4_vo4_disable(struct regulator_dev *rdev)
{
	void __iomem *ctrl = rdev_get_drvdata(rdev);

	writel(readl(ctrl) & ~VO4_XPD, ctrl);
	return 0;
}

static int esp32p4_vo4_is_enabled(struct regulator_dev *rdev)
{
	void __iomem *ctrl = rdev_get_drvdata(rdev);

	return !!(readl(ctrl) & VO4_XPD);
}

static const struct regulator_ops esp32p4_vo4_ops = {
	.enable = esp32p4_vo4_enable,
	.disable = esp32p4_vo4_disable,
	.is_enabled = esp32p4_vo4_is_enabled,
	.list_voltage = regulator_list_voltage_linear,
};

static const struct regulator_desc esp32p4_vo4_desc = {
	.name = "esp32p4-vo4",
	.ops = &esp32p4_vo4_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.n_voltages = 1,
	.min_uV = 3300000,
	.fixed_uV = 3300000,
};

static int esp32p4_vo4_probe(struct platform_device *pdev)
{
	struct regulator_config config = {
		.dev = &pdev->dev,
		.of_node = pdev->dev.of_node,
	};
	void __iomem *ctrl;

	ctrl = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ctrl))
		return PTR_ERR(ctrl);

	config.init_data = of_get_regulator_init_data(&pdev->dev, config.of_node,
						     &esp32p4_vo4_desc);
	if (!config.init_data)
		return -EINVAL;

	config.driver_data = ctrl;
	return PTR_ERR_OR_ZERO(devm_regulator_register(&pdev->dev,
						       &esp32p4_vo4_desc, &config));
}

static const struct of_device_id esp32p4_vo4_match[] = {
	{ .compatible = "espressif,esp32p4-vo4-bypass" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32p4_vo4_match);

static struct platform_driver esp32p4_vo4_driver = {
	.probe = esp32p4_vo4_probe,
	.driver = {
		.name = "esp32p4-vo4",
		.of_match_table = esp32p4_vo4_match,
	},
};
module_platform_driver(esp32p4_vo4_driver);

MODULE_DESCRIPTION("Espressif ESP32-P4 VO4 bypass regulator");
MODULE_LICENSE("GPL");
