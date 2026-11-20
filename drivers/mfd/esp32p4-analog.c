// SPDX-License-Identifier: GPL-2.0-only
/* Shared SAR ADC and temperature sensor register bus on ESP32-P4. */

#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/soc/espressif/esp32p4-analog.h>

/* TRM ch.49 Analog I2C Controller — I2C0_CTRL and ANA_CONF2. */
#define ANA_I2C_CTRL		0x00
#define ANA_I2C_CONF1		0x1c
#define ANA_I2C_CONF2		0x20
#define ANA_I2C_CLK		0x34
#define ANA_I2C_DATA		GENMASK(23, 16)
#define ANA_I2C_WRITE		BIT(24)
#define ANA_I2C_BUSY		BIT(25)
#define ANA_I2C_CONF_MASK	GENMASK(23, 0)
#define ANA_I2C_SENSOR_MST0	BIT(7)
#define ANA_I2C_160M		BIT(0)
#define ANA_I2C_SAR_ADDR	0x69

/* TRM ch.12 Reset and Clock — LPPERI_CK_EN_LP_I2CMST. */
#define LPPERI_CLK_EN		0x00
#define LPPERI_ANA_I2C_CLK	BIT(27)

struct esp32p4_analog {
	struct device *dev;
	void __iomem *base;
	struct regmap *lpperi;
	unsigned int old_clk;
};

static int esp32p4_analog_transfer(struct esp32p4_analog *ana,
				    unsigned int reg, unsigned int *value,
				    bool write)
{
	void __iomem *base = ana->base;
	u32 conf1, conf2, clock, ctrl, cmd;
	int ret;

	ret = readl_poll_timeout(base + ANA_I2C_CTRL, ctrl,
				 !(ctrl & ANA_I2C_BUSY), 1, 10000);
	if (ret)
		return ret;

	/* Match the rev1 sensor master routing in IDF regi2c_enable_block(). */
	conf1 = readl(base + ANA_I2C_CONF1) & ANA_I2C_CONF_MASK;
	conf2 = readl(base + ANA_I2C_CONF2) & ANA_I2C_CONF_MASK;
	clock = readl(base + ANA_I2C_CLK);
	writel(clock | ANA_I2C_160M, base + ANA_I2C_CLK);
	writel(0, base + ANA_I2C_CONF1);
	writel(conf2 | ANA_I2C_SENSOR_MST0, base + ANA_I2C_CONF2);

	/* rev1 RTL uses bit 24 set for writes, as in the IDF LL. */
	cmd = ANA_I2C_SAR_ADDR | (reg << 8);
	if (write)
		cmd |= ANA_I2C_WRITE | FIELD_PREP(ANA_I2C_DATA, *value);
	writel(cmd, base + ANA_I2C_CTRL);
	ret = readl_poll_timeout(base + ANA_I2C_CTRL, ctrl,
				 !(ctrl & ANA_I2C_BUSY), 1, 10000);
	if (!ret && !write)
		*value = FIELD_GET(ANA_I2C_DATA, ctrl);

	writel(conf1, base + ANA_I2C_CONF1);
	writel(conf2, base + ANA_I2C_CONF2);
	writel(clock, base + ANA_I2C_CLK);
	return ret;
}

static int esp32p4_analog_read(void *context, unsigned int reg,
				unsigned int *value)
{
	return esp32p4_analog_transfer(context, reg, value, false);
}

static int esp32p4_analog_write(void *context, unsigned int reg,
				 unsigned int value)
{
	return esp32p4_analog_transfer(context, reg, &value, true);
}

static const struct regmap_config esp32p4_analog_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 9,
	.reg_read = esp32p4_analog_read,
	.reg_write = esp32p4_analog_write,
	.can_sleep = true,
};

struct regmap *esp32p4_analog_get(struct device *consumer)
{
	struct platform_device *supplier;
	struct device_node *np;
	struct regmap *map;

	np = of_parse_phandle(consumer->of_node, "espressif,analog-i2c", 0);
	if (!np)
		return ERR_PTR(-EINVAL);
	supplier = of_find_device_by_node(np);
	of_node_put(np);
	if (!supplier)
		return ERR_PTR(-EPROBE_DEFER);

	map = NULL;
	if (device_link_add(consumer, &supplier->dev, DL_FLAG_AUTOREMOVE_CONSUMER))
		map = dev_get_regmap(&supplier->dev, NULL);
	put_device(&supplier->dev);
	return map ?: ERR_PTR(-EPROBE_DEFER);
}
EXPORT_SYMBOL_GPL(esp32p4_analog_get);

static void esp32p4_analog_restore_clock(void *data)
{
	struct esp32p4_analog *ana = data;
	int ret;

	ret = regmap_update_bits(ana->lpperi, LPPERI_CLK_EN,
				 LPPERI_ANA_I2C_CLK, ana->old_clk);
	if (ret)
		dev_err(ana->dev, "failed to restore analog I2C clock: %d\n", ret);
}

static int esp32p4_analog_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp32p4_analog *ana;
	struct regmap *map;
	int ret;

	ana = devm_kzalloc(dev, sizeof(*ana), GFP_KERNEL);
	if (!ana)
		return -ENOMEM;
	ana->dev = dev;
	ana->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ana->base))
		return PTR_ERR(ana->base);
	ana->lpperi = syscon_regmap_lookup_by_phandle(dev->of_node, "espressif,lpperi");
	if (IS_ERR(ana->lpperi))
		return PTR_ERR(ana->lpperi);

	ret = devm_regulator_get_enable(dev, "analog");
	if (ret)
		return dev_err_probe(dev, ret, "analog supply unavailable\n");
	ret = regmap_read(ana->lpperi, LPPERI_CLK_EN, &ana->old_clk);
	if (ret)
		return ret;
	ret = regmap_set_bits(ana->lpperi, LPPERI_CLK_EN, LPPERI_ANA_I2C_CLK);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, esp32p4_analog_restore_clock, ana);
	if (ret)
		return ret;

	map = devm_regmap_init(dev, NULL, ana, &esp32p4_analog_regmap);
	return PTR_ERR_OR_ZERO(map);
}

static const struct of_device_id esp32p4_analog_of_match[] = {
	{ .compatible = "espressif,esp32p4-analog-i2c" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32p4_analog_of_match);

static struct platform_driver esp32p4_analog_driver = {
	.probe = esp32p4_analog_probe,
	.driver = {
		.name = "esp32p4-analog-i2c",
		.of_match_table = esp32p4_analog_of_match,
	},
};
module_platform_driver(esp32p4_analog_driver);

MODULE_DESCRIPTION("Espressif ESP32-P4 analog sensor I2C regmap");
MODULE_LICENSE("GPL");
