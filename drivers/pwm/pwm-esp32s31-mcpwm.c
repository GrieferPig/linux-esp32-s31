// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S31 MCPWM basic waveform driver.
 *
 * Each of the four instances has three timer/operator pairs and two
 * generators per operator.  Capture, fault, sync and dead-time facilities
 * are intentionally left to the counter/capture interfaces; this driver
 * exposes all 24 basic PWM outputs through the PWM framework.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>

#define MCPWM_TIMER_CFG0(t)		((t) * 0x10)
#define MCPWM_TIMER_CFG1(t)		(0x04 + (t) * 0x10)
#define MCPWM_OPERATOR_TIMERSEL		0x34
#define MCPWM_OPERATOR_BASE(op)		(0x38 + (op) * 0x3c)
#define MCPWM_STAMP(op, cmp)		(MCPWM_OPERATOR_BASE(op) + 0x04 + \
					 (cmp) * 0x04)
#define MCPWM_GEN_CFG(op)		(MCPWM_OPERATOR_BASE(op) + 0x0c)
#define MCPWM_GENERATOR(op, gen)	(MCPWM_OPERATOR_BASE(op) + 0x14 + \
					 (gen) * 0x04)

#define MCPWM_TIMER_PRESCALE		GENMASK(7, 0)
#define MCPWM_TIMER_PERIOD		GENMASK(23, 8)
#define MCPWM_TIMER_START		GENMASK(2, 0)
#define MCPWM_TIMER_MODE		GENMASK(4, 3)
#define MCPWM_TIMER_START_FREE		2
#define MCPWM_TIMER_MODE_UP		1
#define MCPWM_STAMP_VALUE		GENMASK(15, 0)

#define MCPWM_ACTION_LOW		1
#define MCPWM_ACTION_HIGH		2

struct esp32s31_mcpwm {
	void __iomem *base;
	struct clk *clk;
	struct mutex lock;
	u64 timer_period[3];
	unsigned int timer_users[3];
};

static inline struct esp32s31_mcpwm *to_esp32s31_mcpwm(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

static int esp32s31_mcpwm_request(struct pwm_chip *chip,
				  struct pwm_device *pwm)
{
	struct esp32s31_mcpwm *pc = to_esp32s31_mcpwm(chip);

	mutex_lock(&pc->lock);
	pc->timer_users[pwm->hwpwm / 2]++;
	mutex_unlock(&pc->lock);
	return 0;
}

static void esp32s31_mcpwm_free(struct pwm_chip *chip,
				struct pwm_device *pwm)
{
	struct esp32s31_mcpwm *pc = to_esp32s31_mcpwm(chip);
	unsigned int timer = pwm->hwpwm / 2;

	mutex_lock(&pc->lock);
	if (--pc->timer_users[timer] == 0)
		pc->timer_period[timer] = 0;
	mutex_unlock(&pc->lock);
}

static int esp32s31_mcpwm_apply(struct pwm_chip *chip,
				struct pwm_device *pwm,
				const struct pwm_state *state)
{
	struct esp32s31_mcpwm *pc = to_esp32s31_mcpwm(chip);
	unsigned int ch = pwm->hwpwm, op = ch / 2, gen = ch & 1;
	u64 raw_ticks;
	u32 prescale, peak, compare, val, action;
	int ret = 0;

	if (!state->period || state->duty_cycle > state->period ||
	    state->polarity != PWM_POLARITY_NORMAL)
		return -EINVAL;

	raw_ticks = DIV64_U64_ROUND_CLOSEST((u64)clk_get_rate(pc->clk) *
					    state->period, NSEC_PER_SEC);
	prescale = DIV_ROUND_UP_ULL(raw_ticks, 65536);
	if (!prescale || prescale > 256)
		return -ERANGE;
	peak = DIV_ROUND_CLOSEST_ULL(raw_ticks, prescale);
	peak = clamp_t(u32, peak, 1, 65536);
	compare = DIV64_U64_ROUND_CLOSEST((u64)peak * state->duty_cycle,
					 state->period);

	mutex_lock(&pc->lock);
	if (pc->timer_period[op] && pc->timer_period[op] != state->period &&
	    pc->timer_users[op] > 1) {
		ret = -EBUSY;
		goto out;
	}
	pc->timer_period[op] = state->period;

	writel(FIELD_PREP(MCPWM_TIMER_PRESCALE, prescale - 1) |
	       FIELD_PREP(MCPWM_TIMER_PERIOD, peak - 1),
	       pc->base + MCPWM_TIMER_CFG0(op));
	val = readl(pc->base + MCPWM_OPERATOR_TIMERSEL);
	val &= ~(GENMASK(1, 0) << (op * 2));
	val |= op << (op * 2);
	writel(val, pc->base + MCPWM_OPERATOR_TIMERSEL);
	writel(FIELD_PREP(MCPWM_STAMP_VALUE, min(compare, peak - 1)),
	       pc->base + MCPWM_STAMP(op, gen));

	/*
	 * In up-count mode: set high at TEZ, set low at comparator A/B.
	 * The action encoding is IDF's KEEP/LOW/HIGH/TOGGLE = 0/1/2/3.
	 */
	if (!state->enabled || !compare)
		action = MCPWM_ACTION_LOW;
	else if (compare >= peak)
		action = MCPWM_ACTION_HIGH;
	else
		action = MCPWM_ACTION_HIGH |
			 (MCPWM_ACTION_LOW << (4 + gen * 2));
	writel(action, pc->base + MCPWM_GENERATOR(op, gen));
	writel(0, pc->base + MCPWM_GEN_CFG(op)); /* update immediately */
	writel(FIELD_PREP(MCPWM_TIMER_START,
			 state->enabled ? MCPWM_TIMER_START_FREE : 0) |
	       FIELD_PREP(MCPWM_TIMER_MODE,
			 state->enabled ? MCPWM_TIMER_MODE_UP : 0),
	       pc->base + MCPWM_TIMER_CFG1(op));
out:
	mutex_unlock(&pc->lock);
	return ret;
}

static const struct pwm_ops esp32s31_mcpwm_ops = {
	.request = esp32s31_mcpwm_request,
	.free = esp32s31_mcpwm_free,
	.apply = esp32s31_mcpwm_apply,
};

static int esp32s31_mcpwm_probe(struct platform_device *pdev)
{
	struct esp32s31_mcpwm *pc;
	struct pwm_chip *chip;
	int ret;

	chip = devm_pwmchip_alloc(&pdev->dev, 6, sizeof(*pc));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	pc = to_esp32s31_mcpwm(chip);
	mutex_init(&pc->lock);
	pc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pc->base))
		return PTR_ERR(pc->base);
	pc->clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(pc->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(pc->clk),
				     "clock unavailable\n");

	chip->ops = &esp32s31_mcpwm_ops;
	ret = pwmchip_add(chip);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, chip);
	return 0;
}

static void esp32s31_mcpwm_remove(struct platform_device *pdev)
{
	pwmchip_remove(platform_get_drvdata(pdev));
}

static const struct of_device_id esp32s31_mcpwm_of_match[] = {
	{ .compatible = "espressif,esp32s31-mcpwm" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s31_mcpwm_of_match);

static struct platform_driver esp32s31_mcpwm_driver = {
	.probe = esp32s31_mcpwm_probe,
	.remove = esp32s31_mcpwm_remove,
	.driver = {
		.name = "esp32s31-mcpwm",
		.of_match_table = esp32s31_mcpwm_of_match,
	},
};
module_platform_driver(esp32s31_mcpwm_driver);

MODULE_DESCRIPTION("ESP32-S31 MCPWM driver");
MODULE_LICENSE("GPL");
