// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-P4 hw_ver1 peripheral clock/reset subset, using only the XTAL. */
#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <dt-bindings/clock/esp32p4-peripheral-clock.h>

#define SOC_CLK1		0x18
#define SOC_CLK2		0x1c
#define SOC_CLK3		0x20
#define I2C_CLK10		0x40
#define I2C_CLK11		0x44
#define GPSPI_CLK116		0x80
#define IOMUX_CLK26		0xac
#define HP_RST1			0xc4
#define HP_RST2			0xc8

#define I2C1_APB_EN		BIT(13)
#define I2C1_XTAL_MASK		BIT(26)
#define I2C1_FUNC_EN		BIT(27)
#define I2C1_PREDIV		GENMASK(23, 0)
#define I2C1_RST		BIT(21)

#define GPSPI2_SYS_EN		BIT(0)
#define GPSPI2_APB_EN		BIT(19)
#define GPSPI2_SRC		GENMASK(2, 0)
#define GPSPI2_HS_EN		BIT(3)
#define GPSPI2_HS_DIV		GENMASK(11, 4)
#define GPSPI2_MST_DIV		GENMASK(19, 12)
#define GPSPI2_MST_EN		BIT(20)
#define GPSPI2_RST		BIT(7)

#define IOMUX_APB_EN		BIT(3)
#define IOMUX_FUNC_EN		BIT(9)

struct p4_cc {
	void __iomem *base;
	spinlock_t lock;
	struct reset_controller_dev reset;
};

struct p4_clk {
	struct clk_hw hw;
	struct p4_cc *cc;
	unsigned int id;
};

static void p4_update(struct p4_cc *cc, u32 reg, u32 mask, u32 val)
{
	writel((readl(cc->base + reg) & ~mask) | val, cc->base + reg);
}

static int p4_clk_enable(struct clk_hw *hw)
{
	struct p4_clk *clk = container_of(hw, struct p4_clk, hw);
	struct p4_cc *cc = clk->cc;
	unsigned long flags;

	spin_lock_irqsave(&cc->lock, flags);
	switch (clk->id) {
	case ESP32P4_CLK_I2C1:
		p4_update(cc, SOC_CLK2, I2C1_APB_EN, I2C1_APB_EN);
		p4_update(cc, I2C_CLK10, I2C1_FUNC_EN, 0);
		p4_update(cc, I2C_CLK11, I2C1_PREDIV, 0);
		p4_update(cc, I2C_CLK10, I2C1_XTAL_MASK | I2C1_FUNC_EN,
			  I2C1_FUNC_EN);
		break;
	case ESP32P4_CLK_GPSPI2:
		p4_update(cc, SOC_CLK1, GPSPI2_SYS_EN, GPSPI2_SYS_EN);
		p4_update(cc, SOC_CLK2, GPSPI2_APB_EN, GPSPI2_APB_EN);
		p4_update(cc, GPSPI_CLK116, GPSPI2_HS_EN | GPSPI2_MST_EN, 0);
		/* XTAL / 1 (HS) / 2 (master). Leave SPI3's fields alone. */
		p4_update(cc, GPSPI_CLK116, GPSPI2_SRC | GPSPI2_HS_DIV |
			  GPSPI2_MST_DIV | GPSPI2_HS_EN | GPSPI2_MST_EN,
			  FIELD_PREP(GPSPI2_MST_DIV, 1) | GPSPI2_HS_EN |
			  GPSPI2_MST_EN);
		break;
	case ESP32P4_CLK_IOMUX:
		/* Other pads are firmware-owned. Keep their existing source/divider. */
		p4_update(cc, SOC_CLK3, IOMUX_APB_EN, IOMUX_APB_EN);
		p4_update(cc, IOMUX_CLK26, IOMUX_FUNC_EN, IOMUX_FUNC_EN);
		break;
	}
	spin_unlock_irqrestore(&cc->lock, flags);

	return 0;
}

static void p4_clk_disable(struct clk_hw *hw)
{
	struct p4_clk *clk = container_of(hw, struct p4_clk, hw);
	struct p4_cc *cc = clk->cc;
	unsigned long flags;

	spin_lock_irqsave(&cc->lock, flags);
	switch (clk->id) {
	case ESP32P4_CLK_I2C1:
		p4_update(cc, I2C_CLK10, I2C1_FUNC_EN, 0);
		p4_update(cc, SOC_CLK2, I2C1_APB_EN, 0);
		break;
	case ESP32P4_CLK_GPSPI2:
		p4_update(cc, GPSPI_CLK116, GPSPI2_HS_EN | GPSPI2_MST_EN, 0);
		p4_update(cc, SOC_CLK2, GPSPI2_APB_EN, 0);
		p4_update(cc, SOC_CLK1, GPSPI2_SYS_EN, 0);
		break;
	}
	spin_unlock_irqrestore(&cc->lock, flags);
}

static unsigned long p4_clk_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct p4_clk *clk = container_of(hw, struct p4_clk, hw);

	return clk->id == ESP32P4_CLK_GPSPI2 ? parent_rate / 2 : parent_rate;
}

static const struct clk_ops p4_clk_ops = {
	.enable = p4_clk_enable,
	.disable = p4_clk_disable,
	.recalc_rate = p4_clk_rate,
};

static int p4_reset(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct p4_cc *cc = container_of(rcdev, struct p4_cc, reset);
	unsigned long flags;
	u32 reg, bit;

	switch (id) {
	case ESP32P4_RST_I2C1:
		reg = HP_RST1;
		bit = I2C1_RST;
		break;
	case ESP32P4_RST_GPSPI2:
		reg = HP_RST2;
		bit = GPSPI2_RST;
		break;
	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&cc->lock, flags);
	p4_update(cc, reg, bit, bit);
	readl(cc->base + reg);
	udelay(1);
	p4_update(cc, reg, bit, 0);
	readl(cc->base + reg);
	spin_unlock_irqrestore(&cc->lock, flags);

	return 0;
}

static const struct reset_control_ops p4_reset_ops = {
	.reset = p4_reset,
};

static int p4_peripheral_probe(struct platform_device *pdev)
{
	static const char * const names[] = { "p4-i2c1", "p4-gpspi2", "p4-iomux" };
	const struct clk_parent_data parent = { .index = 0 };
	struct device *dev = &pdev->dev;
	struct clk_hw_onecell_data *data;
	struct p4_cc *cc;
	int i, ret;

	cc = devm_kzalloc(dev, sizeof(*cc), GFP_KERNEL);
	if (!cc)
		return -ENOMEM;
	cc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cc->base))
		return PTR_ERR(cc->base);
	spin_lock_init(&cc->lock);

	data = devm_kzalloc(dev, struct_size(data, hws, ESP32P4_CLK_NR),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->num = ESP32P4_CLK_NR;
	for (i = 0; i < data->num; i++) {
		const struct clk_init_data init = {
			.name = names[i], .ops = &p4_clk_ops, .parent_data = &parent,
			.num_parents = 1,
		};
		struct p4_clk *clk;

		clk = devm_kzalloc(dev, sizeof(*clk), GFP_KERNEL);
		if (!clk)
			return -ENOMEM;
		clk->cc = cc;
		clk->id = i;
		clk->hw.init = &init;
		ret = devm_clk_hw_register(dev, &clk->hw);
		if (ret)
			return ret;
		data->hws[i] = &clk->hw;
	}

	cc->reset.owner = THIS_MODULE;
	cc->reset.ops = &p4_reset_ops;
	cc->reset.of_node = dev->of_node;
	cc->reset.nr_resets = ESP32P4_RST_NR;
	ret = devm_reset_controller_register(dev, &cc->reset);
	if (ret)
		return ret;
	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, data);
}

static const struct of_device_id p4_peripheral_of_match[] = {
	{ .compatible = "espressif,esp32p4-rev1-peripheral-clock" },
	{ }
};
MODULE_DEVICE_TABLE(of, p4_peripheral_of_match);

static struct platform_driver p4_peripheral_driver = {
	.probe = p4_peripheral_probe,
	.driver = {
		.name = "esp32p4-peripheral-clock",
		.of_match_table = p4_peripheral_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(p4_peripheral_driver);
