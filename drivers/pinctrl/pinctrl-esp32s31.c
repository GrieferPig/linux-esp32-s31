// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S31 pin controller, GPIO and GPIO interrupt driver
 *
 * Register definitions and behaviour are derived from the ESP-IDF S31 GPIO
 * low-level implementation and generated S31 GPIO/IO_MUX register headers.
 */

#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>

#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>

#include <dt-bindings/pinctrl/esp32s31-pinmux.h>

#include "core.h"
#include "pinconf.h"
#include "pinctrl-utils.h"
#include "pinmux.h"

#define ESP32S31_GPIO_NR		62
#define ESP32S31_GPIO_VALID_MASK	(GENMASK_ULL(61, 0) & ~BIT_ULL(29) & \
					 ~BIT_ULL(33) & ~BIT_ULL(34) & \
					 ~BIT_ULL(41))

/* GPIO register block. */
#define GPIO_OUT		0x004
#define GPIO_OUT_W1TS		0x008
#define GPIO_OUT_W1TC		0x00c
#define GPIO_OUT1		0x010
#define GPIO_OUT1_W1TS		0x014
#define GPIO_OUT1_W1TC		0x018
#define GPIO_ENABLE		0x034
#define GPIO_ENABLE_W1TS	0x038
#define GPIO_ENABLE_W1TC	0x03c
#define GPIO_ENABLE1		0x040
#define GPIO_ENABLE1_W1TS	0x044
#define GPIO_ENABLE1_W1TC	0x048
#define GPIO_IN			0x064
#define GPIO_IN1		0x068
#define GPIO_STATUS		0x074
#define GPIO_STATUS_W1TC	0x07c
#define GPIO_STATUS1		0x080
#define GPIO_STATUS1_W1TC	0x088
#define GPIO_INT0		0x0a4
#define GPIO_INT0_1		0x0b4
#define GPIO_PIN_BASE		0x0f4
#define GPIO_PIN_STRIDE		4
#define GPIO_FUNC_IN_BASE	0x2f4
#define GPIO_FUNC_OUT_BASE	0xaf4
#define GPIO_CLOCK_GATE		0xdf8
#define GPIO_CLOCK_GATE_EN	BIT(0)

#define GPIO_PIN_PAD_DRIVER	BIT(2)
#define GPIO_PIN_INT_TYPE	GENMASK(9, 7)
#define GPIO_PIN_WAKEUP_ENABLE	BIT(10)
#define GPIO_PIN_INT_ENA		GENMASK(17, 13)
#define GPIO_PIN_INT0_ENA	BIT(13)

#define GPIO_FUNC_IN_SEL		GENMASK(7, 0)
#define GPIO_FUNC_IN_INV		BIT(8)
#define GPIO_FUNC_IN_MATRIX	BIT(9)

#define GPIO_FUNC_OUT_SEL	GENMASK(8, 0)
#define GPIO_FUNC_OUT_INV	BIT(9)
#define GPIO_FUNC_OUT_OEN_SEL	BIT(10)
#define GPIO_FUNC_OUT_OEN_INV	BIT(11)

/* IO_MUX has one 32-bit register per GPIO. */
#define IOMUX_SLP_OE		BIT(0)
#define IOMUX_SLP_SEL		BIT(1)
#define IOMUX_SLP_PD		BIT(2)
#define IOMUX_SLP_PU		BIT(3)
#define IOMUX_SLP_IE		BIT(4)
#define IOMUX_SLP_DRV		GENMASK(6, 5)
#define IOMUX_FUN_PD		BIT(7)
#define IOMUX_FUN_PU		BIT(8)
#define IOMUX_FUN_IE		BIT(9)
#define IOMUX_FUN_DRV		GENMASK(11, 10)
#define IOMUX_MCU_SEL		GENMASK(14, 12)
#define IOMUX_FILTER_EN		BIT(15)
#define IOMUX_HYS_EN		BIT(16)
#define IOMUX_HYS_SEL		BIT(17)
#define IOMUX_GPIO_FUNC		1

/* Dedicated control selection for the connectivity-domain pads. */
#define CNNT_PAD_CTRL			0x3f4
#define CNNT_PAD_CLOCK_GATE		0x3f8
#define CNNT_PAD_SDIO_DEDICATED		BIT(0)
#define CNNT_PAD_GMAC_DEDICATED		BIT(1)
#define CNNT_PAD_CLOCK_GATE_EN		BIT(0)

enum esp32s31_gpio_irq_type {
	ESP32S31_GPIO_INTR_DISABLE,
	ESP32S31_GPIO_INTR_POSEDGE,
	ESP32S31_GPIO_INTR_NEGEDGE,
	ESP32S31_GPIO_INTR_ANYEDGE,
	ESP32S31_GPIO_INTR_LOW_LEVEL,
	ESP32S31_GPIO_INTR_HIGH_LEVEL,
};

enum esp32s31_pinmux_type {
	ESP32S31_MUX_IOMUX = ESP32S31_PINMUX_TYPE_IOMUX,
	ESP32S31_MUX_MATRIX_OUT = ESP32S31_PINMUX_TYPE_MATRIX_OUT,
	ESP32S31_MUX_MATRIX_IN = ESP32S31_PINMUX_TYPE_MATRIX_IN,
	ESP32S31_MUX_IOMUX_IN = ESP32S31_PINMUX_TYPE_IOMUX_IN,
};

struct esp32s31_pinmux_entry {
	u8 pin;
	u8 type;
	u8 function;
	u16 value;
	bool invert;
	bool oen_invert;
	bool gpio_oen;
};

struct esp32s31_pin_group {
	struct esp32s31_pinmux_entry *entries;
	unsigned int nentries;
};

struct esp32s31_pinctrl {
	struct device *dev;
	void __iomem *gpio_base;
	void __iomem *iomux_base;
	void __iomem *cnnt_pad_base;
	raw_spinlock_t lock;
	struct mutex groups_lock;

	struct pinctrl_dev *pctldev;
	struct pinctrl_desc pdesc;
	struct pinctrl_pin_desc *pins;

	struct gpio_chip gc;
	u8 irq_types[ESP32S31_GPIO_NR];
};

static u64 esp32s31_bitmap_to_u64(const unsigned long *bitmap)
{
#if BITS_PER_LONG == 64
	return bitmap[0];
#else
	return bitmap[0] | (u64)bitmap[1] << 32;
#endif
}

static bool esp32s31_valid_pin(unsigned int pin)
{
	return pin < ESP32S31_GPIO_NR &&
	       (ESP32S31_GPIO_VALID_MASK & BIT_ULL(pin));
}

static void __iomem *esp32s31_pin_reg(struct esp32s31_pinctrl *pctl,
				      unsigned int pin)
{
	return pctl->gpio_base + GPIO_PIN_BASE + pin * GPIO_PIN_STRIDE;
}

static void __iomem *esp32s31_iomux_reg(struct esp32s31_pinctrl *pctl,
					unsigned int pin)
{
	return pctl->iomux_base + pin * sizeof(u32);
}

static void __iomem *esp32s31_out_sel_reg(struct esp32s31_pinctrl *pctl,
					  unsigned int pin)
{
	return pctl->gpio_base + GPIO_FUNC_OUT_BASE + pin * sizeof(u32);
}

static void esp32s31_update_bits(struct esp32s31_pinctrl *pctl,
				 void __iomem *reg, u32 mask, u32 value)
{
	unsigned long flags;
	u32 tmp;

	raw_spin_lock_irqsave(&pctl->lock, flags);
	tmp = readl_relaxed(reg);
	tmp = (tmp & ~mask) | (value & mask);
	writel_relaxed(tmp, reg);
	raw_spin_unlock_irqrestore(&pctl->lock, flags);
}

static int esp32s31_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	return pinctrl_generic_get_group_count(pctldev);
}

static const char *esp32s31_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
						    unsigned int selector)
{
	return pinctrl_generic_get_group_name(pctldev, selector);
}

static int esp32s31_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
					   unsigned int selector,
					   const unsigned int **pins,
					   unsigned int *npins)
{
	return pinctrl_generic_get_group_pins(pctldev, selector, pins, npins);
}

static void esp32s31_pinctrl_pin_dbg_show(struct pinctrl_dev *pctldev,
					  struct seq_file *s,
					  unsigned int pin)
{
	struct esp32s31_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	seq_printf(s, " iomux=%08x out_sel=%08x pin=%08x",
		   readl_relaxed(esp32s31_iomux_reg(pctl, pin)),
		   readl_relaxed(esp32s31_out_sel_reg(pctl, pin)),
		   readl_relaxed(esp32s31_pin_reg(pctl, pin)));
}

static int esp32s31_decode_pinmux(struct device *dev, u32 cell,
				  struct esp32s31_pinmux_entry *entry)
{
	u32 type = (cell >> ESP32S31_PINMUX_TYPE_SHIFT) & 0x3;
	u32 value = (cell >> 8) & 0x1ff;
	u32 pin = cell & 0xff;

	if (!esp32s31_valid_pin(pin) &&
	    !(type == ESP32S31_MUX_MATRIX_IN &&
	      (pin == ESP32S31_MATRIX_CONST_ONE ||
	       pin == ESP32S31_MATRIX_CONST_ZERO))) {
		dev_err(dev, "invalid GPIO%u in pinmux cell %#x\n", pin, cell);
		return -EINVAL;
	}

	if (type > ESP32S31_MUX_IOMUX_IN ||
	    (type == ESP32S31_MUX_IOMUX && value > 7) ||
	    (type == ESP32S31_MUX_IOMUX_IN && value > 255) ||
	    (type == ESP32S31_MUX_MATRIX_IN && value > 255) ||
	    (type == ESP32S31_MUX_MATRIX_OUT && value > 256)) {
		dev_err(dev, "invalid pinmux cell %#x\n", cell);
		return -EINVAL;
	}

	entry->pin = pin;
	entry->type = type;
	entry->value = value;
	entry->function = type == ESP32S31_MUX_IOMUX_IN ?
			  (cell >> 20) & 0x7 : 0;
	entry->invert = !!(cell & ESP32S31_PINMUX_INVERT);
	entry->oen_invert = !!(cell & ESP32S31_PINMUX_OEN_INVERT);
	entry->gpio_oen = !!(cell & ESP32S31_PINMUX_GPIO_OEN);

	if (type == ESP32S31_MUX_IOMUX &&
	    cell & (ESP32S31_PINMUX_INVERT | ESP32S31_PINMUX_OEN_INVERT |
		    ESP32S31_PINMUX_GPIO_OEN))
		return -EINVAL;

	return 0;
}

static int esp32s31_pinctrl_dt_node_to_map(struct pinctrl_dev *pctldev,
					   struct device_node *np,
					   struct pinctrl_map **maps,
					   unsigned int *num_maps)
{
	struct esp32s31_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	struct device_node *child;
	struct pinctrl_map *map;
	const char **group_names;
	unsigned int ngroups = 0, nmaps = 0;
	int ret = 0;

	for_each_available_child_of_node(np, child)
		ngroups++;
	if (!ngroups)
		return -EINVAL;

	group_names = devm_kcalloc(pctl->dev, ngroups, sizeof(*group_names),
				   GFP_KERNEL);
	if (!group_names)
		return -ENOMEM;

	map = kcalloc(ngroups * 2, sizeof(*map), GFP_KERNEL);
	if (!map)
		return -ENOMEM;

	ngroups = 0;
	mutex_lock(&pctl->groups_lock);
	for_each_available_child_of_node(np, child) {
		struct esp32s31_pin_group *group;
		unsigned int *pins;
		unsigned long seen[BITS_TO_LONGS(ESP32S31_GPIO_NR)] = {};
		const char *name;
		int nentries, npins = 0, i;

		nentries = of_property_count_u32_elems(child, "pinmux");
		if (nentries <= 0) {
			dev_err(pctl->dev, "%pOF: pinmux is required\n", child);
			ret = -EINVAL;
			goto err_put_child;
		}

		name = devm_kasprintf(pctl->dev, GFP_KERNEL, "%pOFn.%pOFn",
				       np, child);
		group = devm_kzalloc(pctl->dev, sizeof(*group), GFP_KERNEL);
		pins = devm_kcalloc(pctl->dev, nentries, sizeof(*pins), GFP_KERNEL);
		if (!name || !group || !pins) {
			ret = -ENOMEM;
			goto err_put_child;
		}

		group->entries = devm_kcalloc(pctl->dev, nentries,
					      sizeof(*group->entries), GFP_KERNEL);
		if (!group->entries) {
			ret = -ENOMEM;
			goto err_put_child;
		}
		group->nentries = nentries;

		for (i = 0; i < nentries; i++) {
			u32 cell;

			ret = of_property_read_u32_index(child, "pinmux", i, &cell);
			if (ret)
				goto err_put_child;
			ret = esp32s31_decode_pinmux(pctl->dev, cell,
						      &group->entries[i]);
			if (ret)
				goto err_put_child;
			if (esp32s31_valid_pin(group->entries[i].pin) &&
			    !test_and_set_bit(group->entries[i].pin, seen))
				pins[npins++] = group->entries[i].pin;
		}

		ret = pinctrl_generic_add_group(pctldev, name, pins, npins, group);
		if (ret < 0)
			goto err_put_child;

		group_names[ngroups++] = name;
		map[nmaps].type = PIN_MAP_TYPE_MUX_GROUP;
		map[nmaps].data.mux.function = np->name;
		map[nmaps].data.mux.group = name;
		nmaps++;

		ret = pinconf_generic_parse_dt_config(child, pctldev,
				&map[nmaps].data.configs.configs,
				&map[nmaps].data.configs.num_configs);
		if (ret)
			goto err_put_child;
		if (map[nmaps].data.configs.num_configs) {
			map[nmaps].type = PIN_MAP_TYPE_CONFIGS_GROUP;
			map[nmaps].data.configs.group_or_pin = name;
			nmaps++;
		}
	}

	ret = pinmux_generic_add_function(pctldev, np->name, group_names,
					  ngroups, NULL);
	if (ret < 0)
		goto err_free;

	*maps = map;
	*num_maps = nmaps;
	mutex_unlock(&pctl->groups_lock);
	return 0;

err_put_child:
	of_node_put(child);
err_free:
	pinctrl_utils_free_map(pctldev, map, nmaps);
	mutex_unlock(&pctl->groups_lock);
	return ret;
}

static const struct pinctrl_ops esp32s31_pinctrl_ops = {
	.get_groups_count = esp32s31_pinctrl_get_groups_count,
	.get_group_name = esp32s31_pinctrl_get_group_name,
	.get_group_pins = esp32s31_pinctrl_get_group_pins,
	.pin_dbg_show = esp32s31_pinctrl_pin_dbg_show,
	.dt_node_to_map = esp32s31_pinctrl_dt_node_to_map,
	.dt_free_map = pinctrl_utils_free_map,
};

static void esp32s31_set_iomux(struct esp32s31_pinctrl *pctl,
			       unsigned int pin, unsigned int function)
{
	/* GPIO8..19 function 2 is the native RGMII group. */
	if (pin >= 8 && pin <= 19 && function == 2)
		esp32s31_update_bits(pctl,
				       pctl->cnnt_pad_base + CNNT_PAD_CTRL,
				       CNNT_PAD_GMAC_DEDICATED,
				       CNNT_PAD_GMAC_DEDICATED);

	/* S31 SDMMC_LL_IOMUX_FUNC is function 4 for the native slot-0 group. */
	if (pin >= 20 && pin <= 25 && function == 4)
		esp32s31_update_bits(pctl,
				       pctl->cnnt_pad_base + CNNT_PAD_CTRL,
				       CNNT_PAD_SDIO_DEDICATED,
				       CNNT_PAD_SDIO_DEDICATED);

	esp32s31_update_bits(pctl, esp32s31_iomux_reg(pctl, pin),
			       IOMUX_MCU_SEL, FIELD_PREP(IOMUX_MCU_SEL, function));
}

static void esp32s31_set_matrix_output(struct esp32s31_pinctrl *pctl,
				       const struct esp32s31_pinmux_entry *entry)
{
	u32 mask = GPIO_FUNC_OUT_SEL | GPIO_FUNC_OUT_INV |
		   GPIO_FUNC_OUT_OEN_SEL | GPIO_FUNC_OUT_OEN_INV;
	u32 val = FIELD_PREP(GPIO_FUNC_OUT_SEL, entry->value);

	if (entry->invert)
		val |= GPIO_FUNC_OUT_INV;
	if (entry->gpio_oen)
		val |= GPIO_FUNC_OUT_OEN_SEL;
	if (entry->oen_invert)
		val |= GPIO_FUNC_OUT_OEN_INV;

	esp32s31_set_iomux(pctl, entry->pin, IOMUX_GPIO_FUNC);
	esp32s31_update_bits(pctl, esp32s31_out_sel_reg(pctl, entry->pin),
			       mask, val);
}

static void esp32s31_set_matrix_input(struct esp32s31_pinctrl *pctl,
				      const struct esp32s31_pinmux_entry *entry)
{
	void __iomem *reg = pctl->gpio_base + GPIO_FUNC_IN_BASE +
			    entry->value * sizeof(u32);
	u32 val = FIELD_PREP(GPIO_FUNC_IN_SEL, entry->pin) |
		  GPIO_FUNC_IN_MATRIX;

	if (entry->invert)
		val |= GPIO_FUNC_IN_INV;
	esp32s31_update_bits(pctl, reg,
			       GPIO_FUNC_IN_SEL | GPIO_FUNC_IN_INV |
			       GPIO_FUNC_IN_MATRIX, val);
	if (esp32s31_valid_pin(entry->pin))
		esp32s31_update_bits(pctl,
				       esp32s31_iomux_reg(pctl, entry->pin),
				       IOMUX_FUN_IE, IOMUX_FUN_IE);
}

static void esp32s31_set_iomux_input(struct esp32s31_pinctrl *pctl,
				     const struct esp32s31_pinmux_entry *entry)
{
	void __iomem *reg = pctl->gpio_base + GPIO_FUNC_IN_BASE +
			    entry->value * sizeof(u32);

	esp32s31_update_bits(pctl, reg, GPIO_FUNC_IN_MATRIX, 0);
	esp32s31_set_iomux(pctl, entry->pin, entry->function);
	esp32s31_update_bits(pctl, esp32s31_iomux_reg(pctl, entry->pin),
			       IOMUX_FUN_IE, IOMUX_FUN_IE);
}

static int esp32s31_pinmux_set_mux(struct pinctrl_dev *pctldev,
				   unsigned int function,
				   unsigned int group_selector)
{
	struct esp32s31_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	const struct group_desc *desc;
	struct esp32s31_pin_group *group;
	unsigned int i;

	desc = pinctrl_generic_get_group(pctldev, group_selector);
	if (!desc)
		return -EINVAL;
	group = desc->data;

	for (i = 0; i < group->nentries; i++) {
		struct esp32s31_pinmux_entry *entry = &group->entries[i];

		switch (entry->type) {
		case ESP32S31_MUX_IOMUX:
			esp32s31_set_iomux(pctl, entry->pin, entry->value);
			break;
		case ESP32S31_MUX_MATRIX_OUT:
			esp32s31_set_matrix_output(pctl, entry);
			break;
		case ESP32S31_MUX_MATRIX_IN:
			esp32s31_set_matrix_input(pctl, entry);
			break;
		case ESP32S31_MUX_IOMUX_IN:
			esp32s31_set_iomux_input(pctl, entry);
			break;
		default:
			return -EINVAL;
		}
	}

	return 0;
}

static int esp32s31_pinmux_gpio_request(struct pinctrl_dev *pctldev,
					struct pinctrl_gpio_range *range,
					unsigned int pin)
{
	struct esp32s31_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	struct esp32s31_pinmux_entry entry = {
		.pin = pin,
		.type = ESP32S31_MUX_MATRIX_OUT,
		.value = ESP32S31_MATRIX_GPIO_OUT,
		.gpio_oen = true,
	};

	if (!esp32s31_valid_pin(pin))
		return -EINVAL;
	esp32s31_set_matrix_output(pctl, &entry);
	return 0;
}

static const struct pinmux_ops esp32s31_pinmux_ops = {
	.get_functions_count = pinmux_generic_get_function_count,
	.get_function_name = pinmux_generic_get_function_name,
	.get_function_groups = pinmux_generic_get_function_groups,
	.set_mux = esp32s31_pinmux_set_mux,
	.gpio_request_enable = esp32s31_pinmux_gpio_request,
	.strict = true,
};

static int esp32s31_pinconf_get(struct pinctrl_dev *pctldev, unsigned int pin,
				unsigned long *config)
{
	struct esp32s31_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param = pinconf_to_config_param(*config);
	u32 iomux, pinreg, arg;

	if (!esp32s31_valid_pin(pin))
		return -EINVAL;

	iomux = readl_relaxed(esp32s31_iomux_reg(pctl, pin));
	pinreg = readl_relaxed(esp32s31_pin_reg(pctl, pin));
	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		if (iomux & (IOMUX_FUN_PU | IOMUX_FUN_PD))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		if (!(iomux & IOMUX_FUN_PU))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (!(iomux & IOMUX_FUN_PD))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		if (!(iomux & IOMUX_FUN_IE))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
		if ((iomux & (IOMUX_HYS_SEL | IOMUX_HYS_EN)) !=
		    (IOMUX_HYS_SEL | IOMUX_HYS_EN))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		if (!(pinreg & GPIO_PIN_PAD_DRIVER))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		if (pinreg & GPIO_PIN_PAD_DRIVER)
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		switch (FIELD_GET(IOMUX_FUN_DRV, iomux)) {
		case 0:
			arg = 5;
			break;
		case 1:
			arg = 10;
			break;
		case 2:
			arg = 20;
			break;
		default:
			arg = 40;
			break;
		}
		break;
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);
	return 0;
}

static int esp32s31_pinconf_set(struct pinctrl_dev *pctldev, unsigned int pin,
				unsigned long *configs, unsigned int nconfigs)
{
	struct esp32s31_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	unsigned int i;

	if (!esp32s31_valid_pin(pin))
		return -EINVAL;

	for (i = 0; i < nconfigs; i++) {
		enum pin_config_param param = pinconf_to_config_param(configs[i]);
		u32 arg = pinconf_to_config_argument(configs[i]);
		u32 mask, val;
		void __iomem *reg = esp32s31_iomux_reg(pctl, pin);

		switch (param) {
		case PIN_CONFIG_BIAS_DISABLE:
			mask = IOMUX_FUN_PU | IOMUX_FUN_PD;
			val = 0;
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			mask = IOMUX_FUN_PU | IOMUX_FUN_PD;
			val = arg ? IOMUX_FUN_PU : 0;
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			mask = IOMUX_FUN_PU | IOMUX_FUN_PD;
			val = arg ? IOMUX_FUN_PD : 0;
			break;
		case PIN_CONFIG_INPUT_ENABLE:
			mask = IOMUX_FUN_IE;
			val = arg ? IOMUX_FUN_IE : 0;
			break;
		case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
			mask = IOMUX_HYS_SEL | IOMUX_HYS_EN;
			val = IOMUX_HYS_SEL | (arg ? IOMUX_HYS_EN : 0);
			break;
		case PIN_CONFIG_INPUT_DEBOUNCE:
			/* S31 provides a fixed pin glitch filter, not a timer. */
			mask = IOMUX_FILTER_EN;
			val = arg ? IOMUX_FILTER_EN : 0;
			break;
		case PIN_CONFIG_DRIVE_STRENGTH:
			switch (arg) {
			case 5:
				val = 0;
				break;
			case 10:
				val = 1;
				break;
			case 20:
				val = 2;
				break;
			case 40:
				val = 3;
				break;
			default:
				return -EINVAL;
			}
			mask = IOMUX_FUN_DRV;
			val = FIELD_PREP(IOMUX_FUN_DRV, val);
			break;
		case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		case PIN_CONFIG_DRIVE_PUSH_PULL:
			reg = esp32s31_pin_reg(pctl, pin);
			mask = GPIO_PIN_PAD_DRIVER;
			val = param == PIN_CONFIG_DRIVE_OPEN_DRAIN ? mask : 0;
			break;
		case PIN_CONFIG_OUTPUT:
			pctl->gc.direction_output(&pctl->gc, pin, arg);
			continue;
		default:
			return -ENOTSUPP;
		}
		esp32s31_update_bits(pctl, reg, mask, val);
	}

	return 0;
}

static int esp32s31_pinconf_group_get(struct pinctrl_dev *pctldev,
				      unsigned int selector,
				      unsigned long *config)
{
	const struct group_desc *group = pinctrl_generic_get_group(pctldev,
								  selector);

	if (!group || !group->grp.npins)
		return -EINVAL;
	return esp32s31_pinconf_get(pctldev, group->grp.pins[0], config);
}

static int esp32s31_pinconf_group_set(struct pinctrl_dev *pctldev,
				      unsigned int selector,
				      unsigned long *configs,
				      unsigned int nconfigs)
{
	const struct group_desc *group = pinctrl_generic_get_group(pctldev,
								  selector);
	unsigned int i;
	int ret;

	if (!group)
		return -EINVAL;
	for (i = 0; i < group->grp.npins; i++) {
		ret = esp32s31_pinconf_set(pctldev, group->grp.pins[i], configs,
					   nconfigs);
		if (ret)
			return ret;
	}
	return 0;
}

static const struct pinconf_ops esp32s31_pinconf_ops = {
	.is_generic = true,
	.pin_config_get = esp32s31_pinconf_get,
	.pin_config_set = esp32s31_pinconf_set,
	.pin_config_group_get = esp32s31_pinconf_group_get,
	.pin_config_group_set = esp32s31_pinconf_group_set,
};

static int esp32s31_gpio_init_valid_mask(struct gpio_chip *gc,
					 unsigned long *valid_mask,
					 unsigned int ngpios)
{
	bitmap_from_u64(valid_mask, ESP32S31_GPIO_VALID_MASK);
	return 0;
}

static int esp32s31_gpio_get_direction(struct gpio_chip *gc,
				       unsigned int offset)
{
	struct esp32s31_pinctrl *pctl = gpiochip_get_data(gc);
	u32 reg = offset < 32 ? GPIO_ENABLE : GPIO_ENABLE1;
	u32 bit = BIT(offset % 32);

	return readl_relaxed(pctl->gpio_base + reg) & bit ?
		GPIO_LINE_DIRECTION_OUT : GPIO_LINE_DIRECTION_IN;
}

static int esp32s31_gpio_direction_input(struct gpio_chip *gc,
					 unsigned int offset)
{
	struct esp32s31_pinctrl *pctl = gpiochip_get_data(gc);
	u32 reg = offset < 32 ? GPIO_ENABLE_W1TC : GPIO_ENABLE1_W1TC;

	writel_relaxed(BIT(offset % 32), pctl->gpio_base + reg);
	esp32s31_update_bits(pctl, esp32s31_iomux_reg(pctl, offset),
			       IOMUX_FUN_IE, IOMUX_FUN_IE);
	return 0;
}

static void esp32s31_gpio_set(struct gpio_chip *gc, unsigned int offset,
			      int value)
{
	struct esp32s31_pinctrl *pctl = gpiochip_get_data(gc);
	u32 reg;

	if (offset < 32)
		reg = value ? GPIO_OUT_W1TS : GPIO_OUT_W1TC;
	else
		reg = value ? GPIO_OUT1_W1TS : GPIO_OUT1_W1TC;
	writel_relaxed(BIT(offset % 32), pctl->gpio_base + reg);
}

static int esp32s31_gpio_direction_output(struct gpio_chip *gc,
					  unsigned int offset, int value)
{
	struct esp32s31_pinctrl *pctl = gpiochip_get_data(gc);
	u32 reg = offset < 32 ? GPIO_ENABLE_W1TS : GPIO_ENABLE1_W1TS;

	esp32s31_gpio_set(gc, offset, value);
	writel_relaxed(BIT(offset % 32), pctl->gpio_base + reg);
	return 0;
}

static int esp32s31_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct esp32s31_pinctrl *pctl = gpiochip_get_data(gc);
	u32 reg = offset < 32 ? GPIO_IN : GPIO_IN1;

	return !!(readl_relaxed(pctl->gpio_base + reg) & BIT(offset % 32));
}

static int esp32s31_gpio_get_multiple(struct gpio_chip *gc,
				      unsigned long *mask,
				      unsigned long *bits)
{
	struct esp32s31_pinctrl *pctl = gpiochip_get_data(gc);
	u64 values = readl_relaxed(pctl->gpio_base + GPIO_IN);

	values |= (u64)readl_relaxed(pctl->gpio_base + GPIO_IN1) << 32;
	bitmap_from_u64(bits, values & esp32s31_bitmap_to_u64(mask));
	return 0;
}

static void esp32s31_gpio_set_multiple(struct gpio_chip *gc,
				       unsigned long *mask,
				       unsigned long *bits)
{
	struct esp32s31_pinctrl *pctl = gpiochip_get_data(gc);
	u64 m = esp32s31_bitmap_to_u64(mask);
	u64 b = esp32s31_bitmap_to_u64(bits);

	writel_relaxed((u32)(m & b), pctl->gpio_base + GPIO_OUT_W1TS);
	writel_relaxed((u32)(m & ~b), pctl->gpio_base + GPIO_OUT_W1TC);
	writel_relaxed((u32)((m & b) >> 32), pctl->gpio_base + GPIO_OUT1_W1TS);
	writel_relaxed((u32)((m & ~b) >> 32), pctl->gpio_base + GPIO_OUT1_W1TC);
}

static int esp32s31_gpio_set_config(struct gpio_chip *gc, unsigned int offset,
				    unsigned long config)
{
	struct esp32s31_pinctrl *pctl = gpiochip_get_data(gc);

	return esp32s31_pinconf_set(pctl->pctldev, offset, &config, 1);
}

static void esp32s31_gpio_irq_ack(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct esp32s31_pinctrl *pctl = gpiochip_get_data(gc);
	unsigned int pin = irqd_to_hwirq(d);
	u32 reg = pin < 32 ? GPIO_STATUS_W1TC : GPIO_STATUS1_W1TC;

	writel_relaxed(BIT(pin % 32), pctl->gpio_base + reg);
}

static void esp32s31_gpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct esp32s31_pinctrl *pctl = gpiochip_get_data(gc);
	unsigned int pin = irqd_to_hwirq(d);

	esp32s31_update_bits(pctl, esp32s31_pin_reg(pctl, pin),
			       GPIO_PIN_INT_ENA, 0);
	gpiochip_disable_irq(gc, pin);
}

static void esp32s31_gpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct esp32s31_pinctrl *pctl = gpiochip_get_data(gc);
	unsigned int pin = irqd_to_hwirq(d);

	gpiochip_enable_irq(gc, pin);
	esp32s31_update_bits(pctl, esp32s31_iomux_reg(pctl, pin),
			       IOMUX_FUN_IE, IOMUX_FUN_IE);
	esp32s31_update_bits(pctl, esp32s31_pin_reg(pctl, pin),
			       GPIO_PIN_INT_ENA, GPIO_PIN_INT0_ENA);
}

static int esp32s31_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct esp32s31_pinctrl *pctl = gpiochip_get_data(gc);
	unsigned int pin = irqd_to_hwirq(d);
	u32 hw_type;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		hw_type = ESP32S31_GPIO_INTR_POSEDGE;
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		hw_type = ESP32S31_GPIO_INTR_NEGEDGE;
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		hw_type = ESP32S31_GPIO_INTR_ANYEDGE;
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		hw_type = ESP32S31_GPIO_INTR_LOW_LEVEL;
		irq_set_handler_locked(d, handle_level_irq);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		hw_type = ESP32S31_GPIO_INTR_HIGH_LEVEL;
		irq_set_handler_locked(d, handle_level_irq);
		break;
	default:
		return -EINVAL;
	}

	pctl->irq_types[pin] = hw_type;
	esp32s31_update_bits(pctl, esp32s31_pin_reg(pctl, pin),
			       GPIO_PIN_INT_TYPE,
			       FIELD_PREP(GPIO_PIN_INT_TYPE, hw_type));
	esp32s31_gpio_irq_ack(d);
	return 0;
}

static int esp32s31_gpio_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct esp32s31_pinctrl *pctl = gpiochip_get_data(gc);
	unsigned int pin = irqd_to_hwirq(d);
	u8 type = pctl->irq_types[pin];

	if (on && type != ESP32S31_GPIO_INTR_LOW_LEVEL &&
	    type != ESP32S31_GPIO_INTR_HIGH_LEVEL)
		return -EINVAL;
	esp32s31_update_bits(pctl, esp32s31_pin_reg(pctl, pin),
			       GPIO_PIN_WAKEUP_ENABLE,
			       on ? GPIO_PIN_WAKEUP_ENABLE : 0);
	return 0;
}

static const struct irq_chip esp32s31_gpio_irq_chip = {
	.name = "ESP32-S31-GPIO",
	.irq_ack = esp32s31_gpio_irq_ack,
	.irq_mask = esp32s31_gpio_irq_mask,
	.irq_unmask = esp32s31_gpio_irq_unmask,
	.irq_set_type = esp32s31_gpio_irq_set_type,
	.irq_set_wake = esp32s31_gpio_irq_set_wake,
	.flags = IRQCHIP_IMMUTABLE,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static void esp32s31_gpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *gc = irq_desc_get_handler_data(desc);
	struct esp32s31_pinctrl *pctl = gpiochip_get_data(gc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u64 pending;
	unsigned int pin;

	chained_irq_enter(chip, desc);
	pending = readl_relaxed(pctl->gpio_base + GPIO_INT0);
	pending |= (u64)readl_relaxed(pctl->gpio_base + GPIO_INT0_1) << 32;
	pending &= ESP32S31_GPIO_VALID_MASK;

	for_each_set_bit(pin, (unsigned long *)&pending, ESP32S31_GPIO_NR)
		generic_handle_domain_irq(gc->irq.domain, pin);
	chained_irq_exit(chip, desc);
}

static void esp32s31_gpio_irq_init_valid_mask(struct gpio_chip *gc,
					      unsigned long *valid_mask,
					      unsigned int ngpios)
{
	bitmap_from_u64(valid_mask, ESP32S31_GPIO_VALID_MASK);
}

static int esp32s31_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp32s31_pinctrl *pctl;
	struct gpio_irq_chip *girq;
	int irq, ret;
	unsigned int i;

	pctl = devm_kzalloc(dev, sizeof(*pctl), GFP_KERNEL);
	if (!pctl)
		return -ENOMEM;
	pctl->dev = dev;
	pctl->gpio_base = devm_platform_ioremap_resource_byname(pdev, "gpio");
	if (IS_ERR(pctl->gpio_base))
		return PTR_ERR(pctl->gpio_base);
	pctl->iomux_base = devm_platform_ioremap_resource_byname(pdev, "iomux");
	if (IS_ERR(pctl->iomux_base))
		return PTR_ERR(pctl->iomux_base);
	pctl->cnnt_pad_base = devm_platform_ioremap_resource_byname(pdev,
							       "cnnt-pad-ctrl");
	if (IS_ERR(pctl->cnnt_pad_base))
		return PTR_ERR(pctl->cnnt_pad_base);
	raw_spin_lock_init(&pctl->lock);
	mutex_init(&pctl->groups_lock);
	esp32s31_update_bits(pctl, pctl->gpio_base + GPIO_CLOCK_GATE,
			       GPIO_CLOCK_GATE_EN, GPIO_CLOCK_GATE_EN);
	esp32s31_update_bits(pctl, pctl->cnnt_pad_base + CNNT_PAD_CLOCK_GATE,
			       CNNT_PAD_CLOCK_GATE_EN, CNNT_PAD_CLOCK_GATE_EN);

	pctl->pins = devm_kcalloc(dev, ESP32S31_GPIO_NR, sizeof(*pctl->pins),
				  GFP_KERNEL);
	if (!pctl->pins)
		return -ENOMEM;
	for (i = 0; i < ESP32S31_GPIO_NR; i++) {
		pctl->pins[i].number = i;
		pctl->pins[i].name = devm_kasprintf(dev, GFP_KERNEL, "gpio%u", i);
		if (!pctl->pins[i].name)
			return -ENOMEM;
	}

	pctl->pdesc.name = dev_name(dev);
	pctl->pdesc.pins = pctl->pins;
	pctl->pdesc.npins = ESP32S31_GPIO_NR;
	pctl->pdesc.pctlops = &esp32s31_pinctrl_ops;
	pctl->pdesc.pmxops = &esp32s31_pinmux_ops;
	pctl->pdesc.confops = &esp32s31_pinconf_ops;
	pctl->pdesc.owner = THIS_MODULE;
	pctl->pctldev = devm_pinctrl_register(dev, &pctl->pdesc, pctl);
	if (IS_ERR(pctl->pctldev))
		return dev_err_probe(dev, PTR_ERR(pctl->pctldev),
				     "failed to register pinctrl\n");

	pctl->gc.label = dev_name(dev);
	pctl->gc.parent = dev;
	pctl->gc.owner = THIS_MODULE;
	pctl->gc.base = -1;
	pctl->gc.ngpio = ESP32S31_GPIO_NR;
	pctl->gc.request = gpiochip_generic_request;
	pctl->gc.free = gpiochip_generic_free;
	pctl->gc.get_direction = esp32s31_gpio_get_direction;
	pctl->gc.direction_input = esp32s31_gpio_direction_input;
	pctl->gc.direction_output = esp32s31_gpio_direction_output;
	pctl->gc.get = esp32s31_gpio_get;
	pctl->gc.set = esp32s31_gpio_set;
	pctl->gc.get_multiple = esp32s31_gpio_get_multiple;
	pctl->gc.set_multiple = esp32s31_gpio_set_multiple;
	pctl->gc.set_config = esp32s31_gpio_set_config;
	pctl->gc.init_valid_mask = esp32s31_gpio_init_valid_mask;
	pctl->gc.can_sleep = false;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	girq = &pctl->gc.irq;
	gpio_irq_chip_set_chip(girq, &esp32s31_gpio_irq_chip);
	girq->parent_handler = esp32s31_gpio_irq_handler;
	girq->num_parents = 1;
	girq->parents = devm_kmalloc(dev, sizeof(*girq->parents), GFP_KERNEL);
	if (!girq->parents)
		return -ENOMEM;
	girq->parents[0] = irq;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_bad_irq;
	girq->init_valid_mask = esp32s31_gpio_irq_init_valid_mask;

	/* Disable all pin interrupts and discard boot-stage status. */
	for (i = 0; i < ESP32S31_GPIO_NR; i++)
		if (esp32s31_valid_pin(i))
			esp32s31_update_bits(pctl, esp32s31_pin_reg(pctl, i),
					       GPIO_PIN_INT_ENA, 0);
	writel_relaxed(~0U, pctl->gpio_base + GPIO_STATUS_W1TC);
	writel_relaxed(~0U, pctl->gpio_base + GPIO_STATUS1_W1TC);

	ret = devm_gpiochip_add_data(dev, &pctl->gc, pctl);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register gpiochip\n");

	ret = gpiochip_add_pin_range(&pctl->gc, dev_name(dev), 0, 0,
				     ESP32S31_GPIO_NR);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add GPIO pin range\n");

	platform_set_drvdata(pdev, pctl);
	dev_info(dev, "registered 60 GPIOs with pinmux, matrix and IRQ support\n");
	return 0;
}

static const struct of_device_id esp32s31_pinctrl_of_match[] = {
	{ .compatible = "espressif,esp32s31-pinctrl" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s31_pinctrl_of_match);

static struct platform_driver esp32s31_pinctrl_driver = {
	.probe = esp32s31_pinctrl_probe,
	.driver = {
		.name = "esp32s31-pinctrl",
		.of_match_table = esp32s31_pinctrl_of_match,
	},
};
module_platform_driver(esp32s31_pinctrl_driver);

MODULE_DESCRIPTION("Espressif ESP32-S31 pinctrl and GPIO driver");
MODULE_LICENSE("GPL");
