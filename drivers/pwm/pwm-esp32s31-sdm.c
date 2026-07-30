// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S31 GPIO sigma-delta modulator exposed as an 8-channel PWM chip.
 *
 * The PWM period selects the sigma-delta input clock prescaler and duty_cycle
 * selects the signed eight-bit pulse density.  An external low-pass filter is
 * required when the output is used as a DAC.
 */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>

#define SDM_MISC			0x04
#define SDM_CHANNEL(ch)			(0x08 + (ch) * 4)
#define SDM_DENSITY			GENMASK(7, 0)
#define SDM_PRESCALE			GENMASK(15, 8)
#define SDM_CLK_EN			BIT(0)

struct esp32s31_sdm {
	void __iomem *base;
	struct mutex lock;
	unsigned long rate;
	unsigned long enabled;
};

static inline struct esp32s31_sdm *to_esp32s31_sdm(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

static int esp32s31_sdm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			      const struct pwm_state *state)
{
	struct esp32s31_sdm *sdm = to_esp32s31_sdm(chip);
	u64 cycles;
	u32 prescale, val;
	s32 density;

	if (!state->period || state->duty_cycle > state->period ||
	    state->polarity != PWM_POLARITY_NORMAL)
		return -EINVAL;

	cycles = DIV64_U64_ROUND_CLOSEST((u64)sdm->rate * state->period,
					 NSEC_PER_SEC);
	if (!cycles || cycles > 256)
		return -ERANGE;
	prescale = cycles - 1;
	density = DIV64_U64_ROUND_CLOSEST(state->duty_cycle * 256,
					 state->period) - 128;
	density = clamp_t(s32, density, -128, 127);
	if (!state->enabled)
		density = -128;
	val = FIELD_PREP(SDM_PRESCALE, prescale) |
	      FIELD_PREP(SDM_DENSITY, (u8)density);

	mutex_lock(&sdm->lock);
	writel(val, sdm->base + SDM_CHANNEL(pwm->hwpwm));
	if (state->enabled)
		__set_bit(pwm->hwpwm, &sdm->enabled);
	else
		__clear_bit(pwm->hwpwm, &sdm->enabled);
	writel(sdm->enabled ? SDM_CLK_EN : 0, sdm->base + SDM_MISC);
	mutex_unlock(&sdm->lock);
	return 0;
}

static const struct pwm_ops esp32s31_sdm_ops = {
	.apply = esp32s31_sdm_apply,
};

static int esp32s31_sdm_probe(struct platform_device *pdev)
{
	struct esp32s31_sdm *sdm;
	struct pwm_chip *chip;
	u32 rate;
	int ret;

	chip = devm_pwmchip_alloc(&pdev->dev, 8, sizeof(*sdm));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	sdm = to_esp32s31_sdm(chip);
	mutex_init(&sdm->lock);
	sdm->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sdm->base))
		return PTR_ERR(sdm->base);
	if (of_property_read_u32(pdev->dev.of_node, "clock-frequency", &rate))
		sdm->rate = 80000000;
	else
		sdm->rate = rate;

	chip->ops = &esp32s31_sdm_ops;
	ret = pwmchip_add(chip);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, chip);
	return 0;
}

static void esp32s31_sdm_remove(struct platform_device *pdev)
{
	struct pwm_chip *chip = platform_get_drvdata(pdev);
	struct esp32s31_sdm *sdm = to_esp32s31_sdm(chip);

	writel(0, sdm->base + SDM_MISC);
	pwmchip_remove(chip);
}

static const struct of_device_id esp32s31_sdm_of_match[] = {
	{ .compatible = "espressif,esp32s31-sigma-delta" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s31_sdm_of_match);

static struct platform_driver esp32s31_sdm_driver = {
	.probe = esp32s31_sdm_probe,
	.remove = esp32s31_sdm_remove,
	.driver = {
		.name = "esp32s31-sigma-delta",
		.of_match_table = esp32s31_sdm_of_match,
	},
};
module_platform_driver(esp32s31_sdm_driver);

MODULE_DESCRIPTION("ESP32-S31 sigma-delta PWM driver");
MODULE_LICENSE("GPL");
