// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S31 MCPWM basic waveform driver.
 *
 * Each of the four instances has three timer/operator pairs and two
 * generators per operator.  The PWM framework exposes all 24 basic outputs
 * and its capture ABI exposes the three capture inputs.  Device-tree policy
 * can additionally arm GPIO faults and timer synchronization.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/reset.h>

#define MCPWM_TIMER_CFG0(t)		((t) * 0x10)
#define MCPWM_TIMER_CFG1(t)		(0x04 + (t) * 0x10)
#define MCPWM_OPERATOR_TIMERSEL		0x34
#define MCPWM_OPERATOR_BASE(op)		(0x38 + (op) * 0x3c)
#define MCPWM_STAMP(op, cmp)		(MCPWM_OPERATOR_BASE(op) + 0x04 + \
					 (cmp) * 0x04)
#define MCPWM_GEN_CFG(op)		(MCPWM_OPERATOR_BASE(op) + 0x0c)
#define MCPWM_GENERATOR(op, gen)	(MCPWM_OPERATOR_BASE(op) + 0x14 + \
					 (gen) * 0x04)
#define MCPWM_TIMER_SYNC(t)		(0x08 + (t) * 0x10)
#define MCPWM_TIMER_SYNCI_CFG		0x30
#define MCPWM_FAULT_DETECT		0xe0
#define MCPWM_CAP_TIMER_CFG		0xe4
#define MCPWM_CAP_CH_CFG(ch)		(0xec + (ch) * 4)
#define MCPWM_CAP_CH_VALUE(ch)		(0xf8 + (ch) * 4)
#define MCPWM_CAP_STATUS		0x104
#define MCPWM_INT_RAW			0x110
#define MCPWM_INT_CLR			0x118
#define MCPWM_FH_CFG0(op)		(0x64 + (op) * 0x38)

#define MCPWM_TIMER_PRESCALE		GENMASK(7, 0)
#define MCPWM_TIMER_PERIOD		GENMASK(23, 8)
#define MCPWM_TIMER_START		GENMASK(2, 0)
#define MCPWM_TIMER_MODE		GENMASK(4, 3)
#define MCPWM_TIMER_START_FREE		2
#define MCPWM_TIMER_MODE_UP		1
#define MCPWM_STAMP_VALUE		GENMASK(15, 0)
#define MCPWM_TIMER_SYNC_ENABLE	BIT(0)
#define MCPWM_TIMER_SYNC_PHASE		GENMASK(19, 4)
#define MCPWM_CAP_TIMER_ENABLE		BIT(0)
#define MCPWM_CAP_ENABLE		BIT(0)
#define MCPWM_CAP_BOTH_EDGES		GENMASK(2, 1)
#define MCPWM_CAP_INT(ch)		BIT(27 + (ch))

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

static int esp32s31_mcpwm_capture(struct pwm_chip *chip,
				  struct pwm_device *pwm,
				  struct pwm_capture *result,
				  unsigned long timeout_ms)
{
	struct esp32s31_mcpwm *pc = to_esp32s31_mcpwm(chip);
	unsigned int cap = pwm->hwpwm % 3;
	ktime_t deadline = ktime_add_ms(ktime_get(), timeout_ms);
	u32 first_rise = 0, fall = 0, second_rise = 0, status;
	unsigned long rate = clk_get_rate(pc->clk);
	bool have_rise = false, have_fall = false, done = false;
	int ret = 0;

	if (!rate || !timeout_ms)
		return -EINVAL;

	mutex_lock(&pc->lock);
	writel(MCPWM_CAP_INT(cap), pc->base + MCPWM_INT_CLR);
	writel(MCPWM_CAP_ENABLE | MCPWM_CAP_BOTH_EDGES,
	       pc->base + MCPWM_CAP_CH_CFG(cap));
	writel(readl(pc->base + MCPWM_CAP_TIMER_CFG) |
	       MCPWM_CAP_TIMER_ENABLE, pc->base + MCPWM_CAP_TIMER_CFG);

	while (ktime_before(ktime_get(), deadline)) {
		u32 value;
		bool falling;

		status = readl(pc->base + MCPWM_INT_RAW);
		if (!(status & MCPWM_CAP_INT(cap))) {
			usleep_range(50, 100);
			continue;
		}
		value = readl(pc->base + MCPWM_CAP_CH_VALUE(cap));
		falling = readl(pc->base + MCPWM_CAP_STATUS) & BIT(cap);
		writel(MCPWM_CAP_INT(cap), pc->base + MCPWM_INT_CLR);

		if (!falling) {
			if (!have_rise) {
				first_rise = value;
				have_rise = true;
			} else if (have_fall) {
				second_rise = value;
				done = true;
				break;
			} else {
				first_rise = value;
			}
		} else if (have_rise) {
			fall = value;
			have_fall = true;
		}
	}
	writel(0, pc->base + MCPWM_CAP_CH_CFG(cap));
	writel(MCPWM_CAP_INT(cap), pc->base + MCPWM_INT_CLR);

	if (!done) {
		ret = -ETIMEDOUT;
		goto out;
	}
	result->period = mul_u64_u64_div_u64((u32)(second_rise - first_rise),
					       NSEC_PER_SEC, rate);
	result->duty_cycle = mul_u64_u64_div_u64((u32)(fall - first_rise),
						   NSEC_PER_SEC, rate);
out:
	mutex_unlock(&pc->lock);
	return ret;
}

static int esp32s31_mcpwm_init_fault_sync(struct platform_device *pdev,
					   struct esp32s31_mcpwm *pc)
{
	u32 fault_mask = 0, active_high = 0;
	u32 sync_sources[3] = { 0 }, sync_phases[3] = { 0 };
	u32 fault_cfg, sync_cfg = 0;
	unsigned int op, source;

	device_property_read_u32(&pdev->dev, "espressif,fault-mask",
				 &fault_mask);
	device_property_read_u32(&pdev->dev, "espressif,fault-active-high-mask",
				 &active_high);
	if ((fault_mask | active_high) & ~GENMASK(2, 0))
		return -EINVAL;
	fault_cfg = fault_mask | (active_high << 3);
	writel(fault_cfg, pc->base + MCPWM_FAULT_DETECT);
	if (fault_mask) {
		for (op = 0; op < 3; op++) {
			u32 cfg = BIT(12) | BIT(14) | BIT(20) | BIT(22);
			unsigned int fault;

			for (fault = 0; fault < 3; fault++)
				if (fault_mask & BIT(fault))
					cfg |= BIT(7 - fault);
			writel(cfg, pc->base + MCPWM_FH_CFG0(op));
		}
	}

	device_property_read_u32_array(&pdev->dev, "espressif,sync-inputs",
				       sync_sources, ARRAY_SIZE(sync_sources));
	device_property_read_u32_array(&pdev->dev, "espressif,sync-phases",
				       sync_phases, ARRAY_SIZE(sync_phases));
	for (op = 0; op < 3; op++) {
		source = sync_sources[op];
		if (source > 6 || sync_phases[op] > U16_MAX)
			return -EINVAL;
		sync_cfg |= source << (op * 3);
		if (source)
			writel(MCPWM_TIMER_SYNC_ENABLE |
			       FIELD_PREP(MCPWM_TIMER_SYNC_PHASE, sync_phases[op]),
			       pc->base + MCPWM_TIMER_SYNC(op));
	}
	writel(sync_cfg, pc->base + MCPWM_TIMER_SYNCI_CFG);
	return 0;
}

static const struct pwm_ops esp32s31_mcpwm_ops = {
	.request = esp32s31_mcpwm_request,
	.free = esp32s31_mcpwm_free,
	.capture = esp32s31_mcpwm_capture,
	.apply = esp32s31_mcpwm_apply,
};

static int esp32s31_mcpwm_probe(struct platform_device *pdev)
{
	struct esp32s31_mcpwm *pc;
	struct pwm_chip *chip;
	struct reset_control *rst;
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
	rst = devm_reset_control_get_optional_exclusive(&pdev->dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(&pdev->dev, PTR_ERR(rst),
				     "reset unavailable\n");
	ret = reset_control_reset(rst);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "reset failed\n");
	ret = esp32s31_mcpwm_init_fault_sync(pdev, pc);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "invalid fault/sync configuration\n");

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
