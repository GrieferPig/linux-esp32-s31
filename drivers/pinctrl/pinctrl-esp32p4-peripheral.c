// SPDX-License-Identifier: GPL-2.0-only
/*
 * Waveshare ESP32-P4-WIFI6-DEV-KIT peripheral groups on ESP32-P4 hw_ver1.
 * Deliberately expose neither arbitrary GPIO matrix writes nor a GPIO chip.
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "pinctrl-utils.h"

/* TRM ch.11 GPIO Matrix and IO MUX - GPIO_OUT, GPIO_ENABLE and GPIO_IN. */
#define GPIO_OUT_SET		0x08
#define GPIO_OUT_CLR		0x0c
#define GPIO_OUT1_SET		0x14
#define GPIO_OUT1_CLR		0x18
#define GPIO_ENABLE		0x20
#define GPIO_ENABLE_SET		0x24
#define GPIO_ENABLE_CLR		0x28
#define GPIO_ENABLE1		0x2c
#define GPIO_ENABLE1_SET	0x30
#define GPIO_ENABLE1_CLR	0x34
#define GPIO_IN			0x3c
#define GPIO_IN1		0x40
#define P4_GPIO_NR		57
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
	bool input;
	unsigned int in_alt; /* Second input signal, or zero if unused. */
};

struct p4_group {
	const char *name;
	const unsigned int *pins;
	const struct p4_route *routes;
	unsigned int npins;
	bool iomux;
	u8 iomux_func;
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

/* TRM ch.59 SD/MMC Host Controller and ch.11 GPIO Matrix and IO MUX: slot 0 pads. */
static const unsigned int sdmmc0_pins[] = { 43, 44, 39, 40, 41, 42 };
static const struct p4_route sdmmc0_routes[] = {
	{ P4_SIGNAL_NONE, P4_SIGNAL_NONE },	/* SD_CLK0 */
	{ P4_SIGNAL_NONE, P4_SIGNAL_NONE, false, true },	/* SD_CMD0 */
	{ P4_SIGNAL_NONE, P4_SIGNAL_NONE, false, true },
	{ P4_SIGNAL_NONE, P4_SIGNAL_NONE, false, true },
	{ P4_SIGNAL_NONE, P4_SIGNAL_NONE, false, true },
	{ P4_SIGNAL_NONE, P4_SIGNAL_NONE, false, true },
};

/* TRM ch.11 - GPIO matrix SD_CARD_CCLK/CCMD/CDATAx_2 signals. */
static const unsigned int sdmmc1_pins[] = { 18, 19, 14, 15, 16, 17 };
static const struct p4_route sdmmc1_routes[] = {
	{ P4_SIGNAL_NONE, 0 },
	{ 1, 1, false, true },
	{ 2, 2, false, true },
	{ 3, 3, false, true },
	{ 4, 4, false, true },
	{ 5, 5, false, true },
};

/* TRM ch.11 GPIO Matrix and IO MUX - native RMII, matrix SMI. */
static const unsigned int emac_rmii_pins[] = { 50, 28, 29, 30, 49, 34, 35 };
static const struct p4_route emac_rmii_routes[] = {
	{ P4_SIGNAL_NONE, P4_SIGNAL_NONE, .input = true }, /* REF_CLK */
	{ P4_SIGNAL_NONE, P4_SIGNAL_NONE, .input = true }, /* CRS_DV */
	{ P4_SIGNAL_NONE, P4_SIGNAL_NONE, .input = true }, /* RXD0 */
	{ P4_SIGNAL_NONE, P4_SIGNAL_NONE, .input = true }, /* RXD1 */
	{ P4_SIGNAL_NONE, P4_SIGNAL_NONE }, /* TX_EN */
	{ P4_SIGNAL_NONE, P4_SIGNAL_NONE }, /* TXD0 */
	{ P4_SIGNAL_NONE, P4_SIGNAL_NONE }, /* TXD1 */
};

static const unsigned int emac_mdio_pins[] = { 31, 52 };
static const struct p4_route emac_mdio_routes[] = {
	{ P4_SIGNAL_NONE, 108 }, /* MII_MDC */
	{ 107, 109 }, /* MII_MDI / MII_MDO, external pull-up */
};

/* TRM ch.11 GPIO Matrix and IO MUX - I2S0 TX/RX BCK and WS inputs.
 * The codec drives BCK/WS; the controller provides MCLK to the codec.
 */
static const unsigned int i2s0_pins[] = { 13, 12, 10, 11, 9 };
static const struct p4_route i2s0_routes[] = {
	{ P4_SIGNAL_NONE, 26 },			/* MCLK */
	{ 25, P4_SIGNAL_NONE, .in_alt = 29 },	/* TX/RX BCK */
	{ 27, P4_SIGNAL_NONE, .in_alt = 30 },	/* TX/RX WS */
	{ 28, P4_SIGNAL_NONE },			/* SD from codec */
	{ P4_SIGNAL_NONE, 28 },			/* SD to codec */
};

/* TRM ch.11 - PCNT_SIG_CH0_PAD_IN0 on the board's GPIO2 header pin. */
static const unsigned int pcnt0_pins[] = { 2 };
static const struct p4_route pcnt0_routes[] = {
	{ 141, P4_SIGNAL_NONE },
};

/* TRM ch.11 GPIO Matrix - LEDC output 0 on the GPIO3 header pad. */
static const unsigned int ledc0_pins[] = { 3 };
static const struct p4_route ledc0_routes[] = {
	{ P4_SIGNAL_NONE, 126 },
};

/* TRM ch.11 GPIO Matrix - alternative MCPWM outputs on the GPIO3 header. */
static const unsigned int mcpwm_pins[] = { 3 };
static const struct p4_route mcpwm0_routes[] = { { P4_SIGNAL_NONE, 89 } };
static const struct p4_route mcpwm1_routes[] = { { P4_SIGNAL_NONE, 95 } };

/* TRM ch.11 GPIO Matrix - RMT shared line on GPIO3. */
static const unsigned int rmt_pins[] = { 3 };
static const struct p4_route rmt_routes[] = { { 246, 246 } };

static const struct p4_group p4_groups[] = {
	{ "i2c1", i2c1_pins, i2c1_routes, ARRAY_SIZE(i2c1_pins) },
	{ "gpspi2", gpspi2_pins, gpspi2_routes, ARRAY_SIZE(gpspi2_pins) },
	{ "sdmmc0", sdmmc0_pins, sdmmc0_routes, ARRAY_SIZE(sdmmc0_pins), true },
	{ "sdmmc1", sdmmc1_pins, sdmmc1_routes, ARRAY_SIZE(sdmmc1_pins) },
	{ "emac-rmii", emac_rmii_pins, emac_rmii_routes,
	  ARRAY_SIZE(emac_rmii_pins), true, 3 },
	{ "emac-mdio", emac_mdio_pins, emac_mdio_routes,
	  ARRAY_SIZE(emac_mdio_pins) },
	{ "i2s0", i2s0_pins, i2s0_routes, ARRAY_SIZE(i2s0_pins) },
	{ "pcnt0", pcnt0_pins, pcnt0_routes, ARRAY_SIZE(pcnt0_pins) },
	{ "ledc0", ledc0_pins, ledc0_routes, ARRAY_SIZE(ledc0_pins) },
	{ "mcpwm0", mcpwm_pins, mcpwm0_routes, ARRAY_SIZE(mcpwm_pins) },
	{ "mcpwm1", mcpwm_pins, mcpwm1_routes, ARRAY_SIZE(mcpwm_pins) },
	{ "rmt-loopback", rmt_pins, rmt_routes, ARRAY_SIZE(rmt_pins) },
};

#define P4_PIN(n) PINCTRL_PIN(n, "GPIO" #n)
static const struct pinctrl_pin_desc p4_pins[] = {
	P4_PIN(0), P4_PIN(1), P4_PIN(2), P4_PIN(3), P4_PIN(4),
	P4_PIN(5), P4_PIN(6), P4_PIN(7), P4_PIN(8), P4_PIN(9),
	P4_PIN(10), P4_PIN(11), P4_PIN(12), P4_PIN(13), P4_PIN(14),
	P4_PIN(15), P4_PIN(16), P4_PIN(17), P4_PIN(18), P4_PIN(19),
	P4_PIN(20), P4_PIN(21), P4_PIN(22), P4_PIN(23), P4_PIN(24),
	P4_PIN(25), P4_PIN(26), P4_PIN(27), P4_PIN(28), P4_PIN(29),
	P4_PIN(30), P4_PIN(31), P4_PIN(32), P4_PIN(33), P4_PIN(34),
	P4_PIN(35), P4_PIN(36), P4_PIN(37), P4_PIN(38), P4_PIN(39),
	P4_PIN(40), P4_PIN(41), P4_PIN(42), P4_PIN(43), P4_PIN(44),
	P4_PIN(45), P4_PIN(46), P4_PIN(47), P4_PIN(48), P4_PIN(49),
	P4_PIN(50), P4_PIN(51), P4_PIN(52), P4_PIN(53), P4_PIN(54),
	P4_PIN(55), P4_PIN(56),
};

struct p4_pinctrl {
	struct gpio_chip gpiochip;
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
		writel(BIT(pin % 32), pc->gpio +
		       (pin >= 32 ? GPIO_ENABLE1_CLR : GPIO_ENABLE_CLR));
		writel(GPIO_OUT_SW_EN | GPIO_OUT_SW, pc->gpio + GPIO_OUT_SEL(pin));
		val = readl(pad) & ~GPIO_OD;
		writel(val | (route->open_drain ? GPIO_OD : 0), pad);

		val = readl(mux) & ~(IO_MUX_FUNC_MASK | IO_MUX_IE | IO_MUX_PU |
				     IO_MUX_PD | IO_MUX_SLEEP_SEL);
		if (pg->iomux) {
			val |= FIELD_PREP(IO_MUX_FUNC_MASK, pg->iomux_func);
			if (route->input || route->pull_up)
				val |= IO_MUX_IE;
		} else {
			val |= IO_MUX_FUNC_MATRIX;
		}
		if (route->in != P4_SIGNAL_NONE) {
			val |= IO_MUX_IE;
			writel(GPIO_IN_MATRIX | pin, pc->gpio + GPIO_IN_SEL(route->in));
		}
		if (route->in_alt)
			writel(GPIO_IN_MATRIX | pin, pc->gpio + GPIO_IN_SEL(route->in_alt));
		if (route->pull_up)
			val |= IO_MUX_PU;
		writel(val, mux);

		if (route->out != P4_SIGNAL_NONE)
			writel(route->out, pc->gpio + GPIO_OUT_SEL(pin));
	}
	return 0;
}

static int p4_gpio_request_enable(struct pinctrl_dev *pctldev,
				  struct pinctrl_gpio_range *range,
				  unsigned int pin)
{
	struct p4_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	void __iomem *mux = pc->iomux + IO_MUX_PAD(pin);
	u32 val;

	writel(BIT(pin % 32), pc->gpio +
	       (pin >= 32 ? GPIO_ENABLE1_CLR : GPIO_ENABLE_CLR));
	writel(GPIO_OUT_SW_EN | GPIO_OUT_SW, pc->gpio + GPIO_OUT_SEL(pin));
	val = readl(pc->gpio + GPIO_PIN_CTRL(pin)) & ~GPIO_OD;
	writel(val, pc->gpio + GPIO_PIN_CTRL(pin));
	val = readl(mux) & ~(IO_MUX_FUNC_MASK | IO_MUX_SLEEP_SEL);
	writel(val | IO_MUX_FUNC_MATRIX | IO_MUX_IE, mux);
	return 0;
}

static int p4_gpio_get(struct gpio_chip *gc, unsigned int pin)
{
	struct p4_pinctrl *pc = gpiochip_get_data(gc);

	return !!(readl(pc->gpio + (pin >= 32 ? GPIO_IN1 : GPIO_IN)) &
		  BIT(pin % 32));
}

static int p4_gpio_set(struct gpio_chip *gc, unsigned int pin, int value)
{
	struct p4_pinctrl *pc = gpiochip_get_data(gc);
	unsigned int reg;

	if (pin >= 32)
		reg = value ? GPIO_OUT1_SET : GPIO_OUT1_CLR;
	else
		reg = value ? GPIO_OUT_SET : GPIO_OUT_CLR;
	writel(BIT(pin % 32), pc->gpio + reg);
	return 0;
}

static int p4_gpio_get_direction(struct gpio_chip *gc, unsigned int pin)
{
	struct p4_pinctrl *pc = gpiochip_get_data(gc);
	u32 val = readl(pc->gpio + (pin >= 32 ? GPIO_ENABLE1 : GPIO_ENABLE));

	return val & BIT(pin % 32) ? GPIO_LINE_DIRECTION_OUT :
				     GPIO_LINE_DIRECTION_IN;
}

static int p4_gpio_direction_input(struct gpio_chip *gc, unsigned int pin)
{
	struct p4_pinctrl *pc = gpiochip_get_data(gc);

	writel(BIT(pin % 32), pc->gpio +
	       (pin >= 32 ? GPIO_ENABLE1_CLR : GPIO_ENABLE_CLR));
	return 0;
}

static int p4_gpio_direction_output(struct gpio_chip *gc, unsigned int pin,
				    int value)
{
	struct p4_pinctrl *pc = gpiochip_get_data(gc);

	p4_gpio_set(gc, pin, value);
	writel(BIT(pin % 32), pc->gpio +
	       (pin >= 32 ? GPIO_ENABLE1_SET : GPIO_ENABLE_SET));
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
	.gpio_request_enable = p4_gpio_request_enable,
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
	u32 val;
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
	ret = pinctrl_enable(pctldev);
	if (ret || !of_property_read_bool(dev->of_node, "gpio-controller"))
		return ret;

	val = readl(pc->gpio + GPIO_CLOCK_GATE);
	writel(val | GPIO_CLK_EN, pc->gpio + GPIO_CLOCK_GATE);

	pc->gpiochip = (struct gpio_chip) {
		.label = dev_name(dev),
		.parent = dev,
		.owner = THIS_MODULE,
		.request = gpiochip_generic_request,
		.free = gpiochip_generic_free,
		.get = p4_gpio_get,
		.set = p4_gpio_set,
		.get_direction = p4_gpio_get_direction,
		.direction_input = p4_gpio_direction_input,
		.direction_output = p4_gpio_direction_output,
		.base = -1,
		.ngpio = P4_GPIO_NR,
	};
	return devm_gpiochip_add_data(dev, &pc->gpiochip, pc);
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
