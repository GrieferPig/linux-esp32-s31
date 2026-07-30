// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S31 LEDC PWM controller.
 *
 * S31 has two identical low-speed LEDC groups.  Each group exposes eight
 * channels backed by four timers; channels 0/1, 2/3, 4/5 and 6/7 share a
 * timer.  Register fields and update ordering follow ESP-IDF ledc_ll.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>

#define LEDC_CH_STRIDE			0x14
#define LEDC_CH_CONF0(ch)		((ch) * LEDC_CH_STRIDE)
#define LEDC_CH_HPOINT(ch)		(0x04 + (ch) * LEDC_CH_STRIDE)
#define LEDC_CH_DUTY(ch)		(0x08 + (ch) * LEDC_CH_STRIDE)
#define LEDC_CH_CONF1(ch)		(0x0c + (ch) * LEDC_CH_STRIDE)
#define LEDC_TIMER_CONF(t)		(0xa0 + (t) * 8)
#define LEDC_CH_POWER			0x174
#define LEDC_TIMER_POWER		0x178

#define LEDC_CH_TIMER_SEL		GENMASK(1, 0)
#define LEDC_CH_SIG_OUT_EN		BIT(2)
#define LEDC_CH_IDLE_LEVEL		BIT(3)
#define LEDC_CH_PARA_UP			BIT(4)
#define LEDC_DUTY_START			BIT(31)

#define LEDC_TIMER_DUTY_RES		GENMASK(4, 0)
#define LEDC_TIMER_CLK_DIV		GENMASK(22, 5)
#define LEDC_TIMER_PAUSE		BIT(23)
#define LEDC_TIMER_RST			BIT(24)
#define LEDC_TIMER_PARA_UP		BIT(26)

#define LEDC_FRAC_BITS			8
#define LEDC_DIV_MIN			BIT(LEDC_FRAC_BITS)
#define LEDC_DIV_MAX			FIELD_MAX(LEDC_TIMER_CLK_DIV)

struct esp32s31_ledc {
	void __iomem *base;
	void __iomem *mem_lp;
	struct clk *clk;
	struct mutex lock;
	u64 timer_period[4];
	unsigned int timer_users[4];
};

static inline struct esp32s31_ledc *to_esp32s31_ledc(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

static int esp32s31_ledc_choose_timer(struct esp32s31_ledc *ledc, u64 period,
				      unsigned int *resolution, u32 *divider)
{
	u64 rate = clk_get_rate(ledc->clk);
	int res;

	for (res = 20; res >= 1; res--) {
		u64 div = DIV64_U64_ROUND_CLOSEST(rate * period *
						 BIT_ULL(LEDC_FRAC_BITS),
						 NSEC_PER_SEC * BIT_ULL(res));

		if (div >= LEDC_DIV_MIN && div <= LEDC_DIV_MAX) {
			*resolution = res;
			*divider = div;
			return 0;
		}
	}

	return -ERANGE;
}

static int esp32s31_ledc_request(struct pwm_chip *chip,
				 struct pwm_device *pwm)
{
	struct esp32s31_ledc *ledc = to_esp32s31_ledc(chip);

	mutex_lock(&ledc->lock);
	ledc->timer_users[pwm->hwpwm / 2]++;
	mutex_unlock(&ledc->lock);
	return 0;
}

static void esp32s31_ledc_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct esp32s31_ledc *ledc = to_esp32s31_ledc(chip);
	unsigned int timer = pwm->hwpwm / 2;

	mutex_lock(&ledc->lock);
	if (--ledc->timer_users[timer] == 0)
		ledc->timer_period[timer] = 0;
	mutex_unlock(&ledc->lock);
}

static int esp32s31_ledc_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			       const struct pwm_state *state)
{
	struct esp32s31_ledc *ledc = to_esp32s31_ledc(chip);
	unsigned int ch = pwm->hwpwm, timer = ch / 2, resolution;
	u32 divider, conf, duty;
	int ret;

	if (!state->period || state->duty_cycle > state->period ||
	    state->polarity != PWM_POLARITY_NORMAL)
		return -EINVAL;

	ret = esp32s31_ledc_choose_timer(ledc, state->period,
					 &resolution, &divider);
	if (ret)
		return ret;

	mutex_lock(&ledc->lock);
	if (ledc->timer_period[timer] &&
	    ledc->timer_period[timer] != state->period &&
	    ledc->timer_users[timer] > 1) {
		ret = -EBUSY;
		goto out;
	}

	ledc->timer_period[timer] = state->period;
	conf = FIELD_PREP(LEDC_TIMER_DUTY_RES, resolution) |
	       FIELD_PREP(LEDC_TIMER_CLK_DIV, divider) |
	       LEDC_TIMER_PARA_UP;
	writel(conf, ledc->base + LEDC_TIMER_CONF(timer));
	writel(BIT(timer), ledc->base + LEDC_TIMER_POWER);

	duty = DIV64_U64_ROUND_CLOSEST(state->duty_cycle * BIT_ULL(resolution),
				      state->period);
	duty = min_t(u32, duty, BIT(resolution));
	writel(0, ledc->base + LEDC_CH_HPOINT(ch));
	/* IDF stores the integer duty at bit 4 of the 25-bit field. */
	writel(duty << 4, ledc->base + LEDC_CH_DUTY(ch));
	writel(LEDC_DUTY_START, ledc->base + LEDC_CH_CONF1(ch));

	conf = FIELD_PREP(LEDC_CH_TIMER_SEL, timer) | LEDC_CH_PARA_UP;
	if (state->enabled)
		conf |= LEDC_CH_SIG_OUT_EN;
	writel(conf, ledc->base + LEDC_CH_CONF0(ch));
	writel(readl(ledc->base + LEDC_CH_POWER) |
	       (state->enabled ? BIT(ch) : 0),
	       ledc->base + LEDC_CH_POWER);
	if (!state->enabled)
		writel(readl(ledc->base + LEDC_CH_POWER) & ~BIT(ch),
		       ledc->base + LEDC_CH_POWER);
out:
	mutex_unlock(&ledc->lock);
	return ret;
}

static const struct pwm_ops esp32s31_ledc_ops = {
	.request = esp32s31_ledc_request,
	.free = esp32s31_ledc_free,
	.apply = esp32s31_ledc_apply,
};

static int esp32s31_ledc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp32s31_ledc *ledc;
	struct pwm_chip *chip;
	int ret;

	chip = devm_pwmchip_alloc(dev, 8, sizeof(*ledc));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	ledc = to_esp32s31_ledc(chip);
	mutex_init(&ledc->lock);

	ledc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ledc->base))
		return PTR_ERR(ledc->base);
	ledc->mem_lp = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(ledc->mem_lp))
		return PTR_ERR(ledc->mem_lp);
	ledc->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(ledc->clk))
		return dev_err_probe(dev, PTR_ERR(ledc->clk), "clock unavailable\n");

	/* IDF: force channel/gamma RAM on while Linux owns the controller. */
	writel((readl(ledc->mem_lp) & ~BIT(2)) | BIT(3), ledc->mem_lp);
	chip->ops = &esp32s31_ledc_ops;
	ret = pwmchip_add(chip);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, chip);
	return 0;
}

static void esp32s31_ledc_remove(struct platform_device *pdev)
{
	pwmchip_remove(platform_get_drvdata(pdev));
}

static const struct of_device_id esp32s31_ledc_of_match[] = {
	{ .compatible = "espressif,esp32s31-ledc" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s31_ledc_of_match);

static struct platform_driver esp32s31_ledc_driver = {
	.probe = esp32s31_ledc_probe,
	.remove = esp32s31_ledc_remove,
	.driver = {
		.name = "esp32s31-ledc",
		.of_match_table = esp32s31_ledc_of_match,
	},
};
module_platform_driver(esp32s31_ledc_driver);

MODULE_DESCRIPTION("ESP32-S31 LEDC PWM driver");
MODULE_LICENSE("GPL");
