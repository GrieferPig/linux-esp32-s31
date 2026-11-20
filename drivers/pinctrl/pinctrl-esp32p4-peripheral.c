// SPDX-License-Identifier: GPL-2.0-only
/*
 * Waveshare ESP32-P4-WIFI6-DEV-KIT peripheral groups on ESP32-P4 hw_ver1.
 * Deliberately expose neither arbitrary GPIO matrix writes nor a GPIO chip.
 */
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "pinctrl-utils.h"

#define GPIO_ENABLE_CLR		0x28
#define GPIO_PIN_CTRL(p)	(0x74 + 4 * (p))
#define GPIO_IN_SEL(s)		(0x158 + 4 * (s))
#define GPIO_OUT_SEL(p)		(0x558 + 4 * (p))
#define GPIO_CLOCK_GATE		0x64c
#define GPIO_CLK_EN		BIT(0)
#define GPIO_IN_MATRIX		BIT(7)
#define GPIO_OD			BIT(2)
#define GPIO_OUT_SW_EN		BIT(10)
#define GPIO_OUT_SW		256

#define IO_MUX_PAD(p)		(4 + 4 * (p))
#define IO_MUX_SLEEP_SEL	BIT(1)
#define IO_MUX_PU		BIT(8)
#define IO_MUX_PD		BIT(7)
#define IO_MUX_IE		BIT(9)
#define IO_MUX_FUNC_MASK	GENMASK(14, 12)
#define IO_MUX_FUNC_MATRIX	BIT(12)

#define P4_SIGNAL_NONE		(-1)

struct p4_route {
	int in, out;
	bool open_drain, pull_up;
};

struct p4_group {
	const char *name;
	const unsigned int *pins;
	const struct p4_route *routes;
	unsigned int npins;
};

static const unsigned int i2c1_pins[] = { 7, 8 };	/* SDA, SCL */
static const struct p4_route i2c1_routes[] = {
	{ 71, 71, true, true },
	{ 70, 70, true, true },
};

static const unsigned int gpspi2_pins[] = { 20, 21, 22, 23 };
static const struct p4_route gpspi2_routes[] = {
	{ P4_SIGNAL_NONE, 53 },			/* SCK */
	{ P4_SIGNAL_NONE, 55 },			/* MOSI */
	{ 54, P4_SIGNAL_NONE },			/* MISO */
	{ P4_SIGNAL_NONE, 62, false, true },	/* CS0 */
};

static const struct p4_group p4_groups[] = {
	{ "i2c1", i2c1_pins, i2c1_routes, ARRAY_SIZE(i2c1_pins) },
	{ "gpspi2", gpspi2_pins, gpspi2_routes, ARRAY_SIZE(gpspi2_pins) },
};

static const struct pinctrl_pin_desc p4_pins[] = {
	PINCTRL_PIN(7, "GPIO7"), PINCTRL_PIN(8, "GPIO8"),
	PINCTRL_PIN(20, "GPIO20"), PINCTRL_PIN(21, "GPIO21"),
	PINCTRL_PIN(22, "GPIO22"), PINCTRL_PIN(23, "GPIO23"),
};

struct p4_pinctrl {
	void __iomem *gpio;
	void __iomem *iomux;
};

static int p4_group_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(p4_groups);
}

static const char *p4_group_name(struct pinctrl_dev *pctldev,
				 unsigned int selector)
{
	return p4_groups[selector].name;
}

static int p4_group_pins(struct pinctrl_dev *pctldev, unsigned int selector,
			 const unsigned int **pins, unsigned int *npins)
{
	*pins = p4_groups[selector].pins;
	*npins = p4_groups[selector].npins;
	return 0;
}

static int p4_function_groups(struct pinctrl_dev *pctldev, unsigned int selector,
			      const char * const **groups, unsigned int *ngroups)
{
	*groups = &p4_groups[selector].name;
	*ngroups = 1;
	return 0;
}

static int p4_dt_node_to_map(struct pinctrl_dev *pctldev, struct device_node *np,
			     struct pinctrl_map **map, unsigned int *nmaps)
{
	const char *name;
	unsigned int reserved = 0;
	int i, ret;

	*map = NULL;
	*nmaps = 0;
	ret = of_property_read_string(np, "function", &name);
	if (ret)
		return ret;
	for (i = 0; i < ARRAY_SIZE(p4_groups); i++)
		if (!strcmp(name, p4_groups[i].name)) {
			ret = pinctrl_utils_reserve_map(pctldev, map, &reserved,
							nmaps, 1);
			if (ret)
				return ret;
			return pinctrl_utils_add_map_mux(pctldev, map, &reserved,
							 nmaps, name, name);
		}
	return -EINVAL;
}

static int p4_set_mux(struct pinctrl_dev *pctldev, unsigned int function,
		      unsigned int group)
{
	struct p4_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	const struct p4_group *pg = &p4_groups[group];
	u32 val;
	int i;

	if (function != group)
		return -EINVAL;

	val = readl(pc->gpio + GPIO_CLOCK_GATE);
	writel(val | GPIO_CLK_EN, pc->gpio + GPIO_CLOCK_GATE);

	for (i = 0; i < pg->npins; i++) {
		const struct p4_route *route = &pg->routes[i];
		unsigned int pin = pg->pins[i];
		void __iomem *mux = pc->iomux + IO_MUX_PAD(pin);
		void __iomem *pad = pc->gpio + GPIO_PIN_CTRL(pin);

		/* Park in high impedance while routing this board-owned pad. */
		writel(BIT(pin), pc->gpio + GPIO_ENABLE_CLR);
		writel(GPIO_OUT_SW_EN | GPIO_OUT_SW, pc->gpio + GPIO_OUT_SEL(pin));
		val = readl(pad) & ~GPIO_OD;
		writel(val | (route->open_drain ? GPIO_OD : 0), pad);

		val = readl(mux) & ~(IO_MUX_FUNC_MASK | IO_MUX_IE | IO_MUX_PU |
				     IO_MUX_PD | IO_MUX_SLEEP_SEL);
		val |= IO_MUX_FUNC_MATRIX;
		if (route->in != P4_SIGNAL_NONE) {
			val |= IO_MUX_IE;
			writel(GPIO_IN_MATRIX | pin, pc->gpio + GPIO_IN_SEL(route->in));
		}
		if (route->pull_up)
			val |= IO_MUX_PU;
		writel(val, mux);

		if (route->out != P4_SIGNAL_NONE)
			writel(route->out, pc->gpio + GPIO_OUT_SEL(pin));
	}
	return 0;
}

static const struct pinctrl_ops p4_pctl_ops = {
	.get_groups_count = p4_group_count,
	.get_group_name = p4_group_name,
	.get_group_pins = p4_group_pins,
	.dt_node_to_map = p4_dt_node_to_map,
	.dt_free_map = pinctrl_utils_free_map,
};

static const struct pinmux_ops p4_pmx_ops = {
	.get_functions_count = p4_group_count,
	.get_function_name = p4_group_name,
	.get_function_groups = p4_function_groups,
	.set_mux = p4_set_mux,
	.strict = true,
};

static struct pinctrl_desc p4_pin_desc = {
	.name = "esp32p4-waveshare-peripheral",
	.pins = p4_pins,
	.npins = ARRAY_SIZE(p4_pins),
	.pctlops = &p4_pctl_ops,
	.pmxops = &p4_pmx_ops,
	.owner = THIS_MODULE,
};

static int p4_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct p4_pinctrl *pc;
	struct pinctrl_dev *pctldev;
	struct clk *clk;
	int ret;

	pc = devm_kzalloc(dev, sizeof(*pc), GFP_KERNEL);
	if (!pc)
		return -ENOMEM;
	pc->gpio = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pc->gpio))
		return PTR_ERR(pc->gpio);
	pc->iomux = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(pc->iomux))
		return PTR_ERR(pc->iomux);
	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "IOMUX clock unavailable\n");

	ret = devm_pinctrl_register_and_init(dev, &p4_pin_desc, pc, &pctldev);
	if (ret)
		return ret;
	return pinctrl_enable(pctldev);
}

static const struct of_device_id p4_pinctrl_of_match[] = {
	{ .compatible = "waveshare,esp32p4-wifi6-dev-kit-peripheral-pinctrl" },
	{ }
};
MODULE_DEVICE_TABLE(of, p4_pinctrl_of_match);

static struct platform_driver p4_pinctrl_driver = {
	.probe = p4_pinctrl_probe,
	.driver = {
		.name = "esp32p4-peripheral-pinctrl",
		.of_match_table = p4_pinctrl_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(p4_pinctrl_driver);
