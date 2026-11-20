// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif LEDC PWM controller.
 *
 * S31 has two low-speed LEDC groups and P4 has one. Each group exposes eight
 * channels backed by four timers; channels 0/1, 2/3, 4/5 and 6/7 share a
 * timer.  Register fields and update ordering follow ESP-IDF ledc_ll.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/reset.h>

#define LEDC_CH_STRIDE			0x14
#define LEDC_CH_CONF0(ch)		((ch) * LEDC_CH_STRIDE)
#define LEDC_CH_HPOINT(ch)		(0x04 + (ch) * LEDC_CH_STRIDE)
#define LEDC_CH_DUTY(ch)		(0x08 + (ch) * LEDC_CH_STRIDE)
#define LEDC_CH_CONF1(ch)		(0x0c + (ch) * LEDC_CH_STRIDE)
#define LEDC_CH_DUTY_R(ch)		(0x10 + (ch) * LEDC_CH_STRIDE)
#define LEDC_TIMER_CONF(t)		(0xa0 + (t) * 8)
#define LEDC_INT_RAW			0xc0
#define LEDC_INT_CLR			0xcc
#define LEDC_GAMMA_CONF(ch)		(0x100 + (ch) * 4)
#define LEDC_GAMMA_RANGE(ch, range)	(0x400 + (ch) * 0x40 + (range) * 4)
/* S31-only power registers. On P4, 0x174 is LEDC_DATE. */
#define LEDC_CH_POWER			0x174
#define LEDC_TIMER_POWER		0x178

#define LEDC_CH_TIMER_SEL		GENMASK(1, 0)
#define LEDC_CH_SIG_OUT_EN		BIT(2)
#define LEDC_CH_IDLE_LEVEL		BIT(3)
#define LEDC_CH_PARA_UP			BIT(4)
#define LEDC_DUTY_START			BIT(31)
#define LEDC_DUTY_DONE(ch)		BIT(4 + (ch))
#define LEDC_GAMMA_ENTRY_NUM		GENMASK(4, 0)
#define LEDC_GAMMA_STEP			GENMASK(30, 21)
#define LEDC_GAMMA_SCALE		GENMASK(20, 11)
#define LEDC_GAMMA_CYCLE		GENMASK(10, 1)
#define LEDC_GAMMA_INCREASE		BIT(0)

#define LEDC_TIMER_DUTY_RES		GENMASK(4, 0)
#define LEDC_TIMER_CLK_DIV		GENMASK(22, 5)
#define LEDC_TIMER_PAUSE		BIT(23)
#define LEDC_TIMER_RST			BIT(24)
#define LEDC_TIMER_PARA_UP		BIT(26)

#define LEDC_MAX_RESOLUTION		20
#define LEDC_FRAC_BITS			8
#define LEDC_DIV_MIN			BIT(LEDC_FRAC_BITS)
#define LEDC_DIV_MAX			FIELD_MAX(LEDC_TIMER_CLK_DIV)

struct espressif_ledc {
	void __iomem *base;
	void __iomem *mem_lp;
	struct clk *clk;
	struct mutex lock;
	u64 timer_period[4];
	unsigned int timer_users[4];
	unsigned long enabled_channels;
	u64 logical_duty[8];
	u32 fade_time_ms;
	bool gamma_correction;
};

static inline struct espressif_ledc *to_espressif_ledc(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

static int espressif_ledc_choose_timer(struct espressif_ledc *ledc, u64 period,
				      unsigned int *resolution, u32 *divider)
{
	u64 scale = (u64)clk_get_rate(ledc->clk) << LEDC_FRAC_BITS;
	u64 max_ticks = (u64)NSEC_PER_SEC << LEDC_MAX_RESOLUTION;
	int res;

	/* Bound the period before evaluating the fixed-point divisor. */
	if (!scale ||
	    period > mul_u64_u64_div_u64(max_ticks, LEDC_DIV_MAX + 1, scale))
		return -ERANGE;

	for (res = LEDC_MAX_RESOLUTION; res >= 1; res--) {
		u64 ticks = (u64)NSEC_PER_SEC << res;
		u64 twice_div = mul_u64_u64_div_u64(scale * 2, period, ticks);
		u64 div = DIV_ROUND_UP_ULL(twice_div, 2);

		if (div >= LEDC_DIV_MIN && div <= LEDC_DIV_MAX) {
			*resolution = res;
			*divider = div;
			return 0;
		}
	}

	return -ERANGE;
}

static u32 espressif_ledc_duty_ticks(u64 duty_ns,
				  u64 period_ns, unsigned int resolution)
{
	u64 full = BIT_ULL(resolution);
	u64 logical = DIV64_U64_ROUND_CLOSEST(duty_ns * full, period_ns);

	logical = min(logical, full);
	/* The Linux PWM state always describes electrical on-time.  Gamma mode
	 * selects the multi-range fade engine; it must not square the requested
	 * steady-state duty cycle (25% would otherwise become 6.25%). */
	return logical;
}

static void esp32p4_ledc_update_channel(struct espressif_ledc *ledc,
					unsigned int ch)
{
	void __iomem *conf = ledc->base + LEDC_CH_CONF0(ch);

	/* TRM ch.60 LED PWM Controller: latch duty and fade via PARA_UP_CHn. */
	writel(readl(conf) | LEDC_CH_PARA_UP, conf);
}

static int espressif_ledc_fade(struct espressif_ledc *ledc, unsigned int ch,
			      unsigned int resolution, u64 period_ns,
			      u64 old_ns, u64 new_ns)
{
	unsigned int ranges = ledc->gamma_correction ? 16 : 1;
	u64 total_cycles = DIV64_U64_ROUND_CLOSEST((u64)ledc->fade_time_ms *
						     NSEC_PER_MSEC, period_ns);
	u32 start, target, delta, status;
	u64 cycles_per_range;
	unsigned int range;

	start = espressif_ledc_duty_ticks(old_ns, period_ns, resolution);
	target = espressif_ledc_duty_ticks(new_ns, period_ns, resolution);
	if (!total_cycles || start == target)
		return 0;
	ranges = min_t(u64, ranges, total_cycles);
	cycles_per_range = div64_u64(total_cycles, ranges);
	delta = abs((int)target - (int)start);

	writel(start << 4, ledc->base + LEDC_CH_DUTY(ch));
	for (range = 0; range < ranges; range++) {
		u32 first = DIV_ROUND_CLOSEST(delta * range, ranges);
		u32 last = DIV_ROUND_CLOSEST(delta * (range + 1), ranges);
		u32 span = last - first;
		u32 steps = clamp_t(u64, min_t(u64, span, cycles_per_range), 1, 1023);
		u32 scale = min(DIV_ROUND_CLOSEST(span, steps), 1023U);
		u32 cycle = min_t(u64, div64_u64(cycles_per_range, steps), 1023);
		u32 entry = FIELD_PREP(LEDC_GAMMA_STEP, steps) |
			    FIELD_PREP(LEDC_GAMMA_SCALE, scale) |
			    FIELD_PREP(LEDC_GAMMA_CYCLE, cycle);

		if (target > start)
			entry |= LEDC_GAMMA_INCREASE;
		writel(entry, ledc->base + LEDC_GAMMA_RANGE(ch, range));
	}
	writel(FIELD_PREP(LEDC_GAMMA_ENTRY_NUM, ranges),
	       ledc->base + LEDC_GAMMA_CONF(ch));
	writel(LEDC_DUTY_DONE(ch), ledc->base + LEDC_INT_CLR);
	writel(LEDC_DUTY_START, ledc->base + LEDC_CH_CONF1(ch));
	if (!ledc->mem_lp)
		esp32p4_ledc_update_channel(ledc, ch);
	return readl_poll_timeout(ledc->base + LEDC_INT_RAW, status,
				 status & LEDC_DUTY_DONE(ch), 100,
				 ledc->fade_time_ms * 1000 + 100000);
}

static int espressif_ledc_request(struct pwm_chip *chip,
				 struct pwm_device *pwm)
{
	struct espressif_ledc *ledc = to_espressif_ledc(chip);

	mutex_lock(&ledc->lock);
	ledc->timer_users[pwm->hwpwm / 2]++;
	mutex_unlock(&ledc->lock);
	return 0;
}

static void espressif_ledc_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct espressif_ledc *ledc = to_espressif_ledc(chip);
	unsigned int timer = pwm->hwpwm / 2;

	mutex_lock(&ledc->lock);
	if (--ledc->timer_users[timer] == 0)
		ledc->timer_period[timer] = 0;
	mutex_unlock(&ledc->lock);
}

static int espressif_ledc_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			       const struct pwm_state *state)
{
	struct espressif_ledc *ledc = to_espressif_ledc(chip);
	unsigned int ch = pwm->hwpwm, timer = ch / 2, resolution;
	void __iomem *timer_conf = ledc->base + LEDC_TIMER_CONF(timer);
	u32 divider, conf, duty;
	bool period_changed;
	int ret;

	if (!state->period || state->duty_cycle > state->period ||
	    state->polarity != PWM_POLARITY_NORMAL)
		return -EINVAL;

	ret = espressif_ledc_choose_timer(ledc, state->period,
					 &resolution, &divider);
	if (ret)
		return ret;

	mutex_lock(&ledc->lock);
	/*
	 * A P4 timer latches its configuration at overflow (TRM ch.60,
	 * TIMERx_PARA_UP). Keep a disabled channel's requested period in
	 * the PWM core until enable, so an unused slow period cannot hold
	 * up the first waveform.
	 */
	if (!ledc->mem_lp && !state->enabled &&
	    !test_bit(ch, &ledc->enabled_channels))
		goto out;
	if (ledc->timer_period[timer] &&
	    ledc->timer_period[timer] != state->period &&
	    ledc->timer_users[timer] > 1) {
		ret = -EBUSY;
		goto out;
	}

	period_changed = ledc->timer_period[timer] != state->period;
	if (state->enabled || ledc->mem_lp)
		ledc->timer_period[timer] = state->period;
	conf = FIELD_PREP(LEDC_TIMER_DUTY_RES, resolution) |
	       FIELD_PREP(LEDC_TIMER_CLK_DIV, divider);
	if (ledc->mem_lp) {
		writel(conf | LEDC_TIMER_PARA_UP, timer_conf);
		writel(BIT(timer), ledc->base + LEDC_TIMER_POWER);
	} else if (state->enabled && period_changed) {
		/*
		 * TRM ch.60 LED PWM Controller: TIMERx_PARA_UP normally waits
		 * for overflow. Reset the timer after changing its period
		 * as in ESP-IDF ledc_timer_config() and ledc_ll_timer_rst().
		 */
		writel(conf, timer_conf);
		writel(conf | LEDC_TIMER_PARA_UP, timer_conf);
		writel(readl(timer_conf) | LEDC_TIMER_RST, timer_conf);
		writel(readl(timer_conf) & ~LEDC_TIMER_RST, timer_conf);
	}

	duty = espressif_ledc_duty_ticks(state->duty_cycle,
				       state->period, resolution);
	writel(0, ledc->base + LEDC_CH_HPOINT(ch));

	conf = FIELD_PREP(LEDC_CH_TIMER_SEL, timer);
	if (state->enabled)
		conf |= LEDC_CH_SIG_OUT_EN;
	if (ledc->mem_lp)
		conf |= LEDC_CH_PARA_UP;
	writel(conf, ledc->base + LEDC_CH_CONF0(ch));
	if (ledc->mem_lp) {
		writel(readl(ledc->base + LEDC_CH_POWER) |
		       (state->enabled ? BIT(ch) : 0), ledc->base + LEDC_CH_POWER);
		if (!state->enabled)
			writel(readl(ledc->base + LEDC_CH_POWER) & ~BIT(ch),
			       ledc->base + LEDC_CH_POWER);
	}

	if (state->enabled && ledc->fade_time_ms &&
	    ledc->logical_duty[ch] != state->duty_cycle) {
		ret = espressif_ledc_fade(ledc, ch, resolution, state->period,
					ledc->logical_duty[ch], state->duty_cycle);
		if (ret)
			goto out;
	}
	/* End on the exact target even when a gamma segment rounded a step. */
	writel(0, ledc->base + LEDC_GAMMA_CONF(ch));
	writel(duty << 4, ledc->base + LEDC_CH_DUTY(ch));
	writel(LEDC_DUTY_START, ledc->base + LEDC_CH_CONF1(ch));
	if (!ledc->mem_lp)
		esp32p4_ledc_update_channel(ledc, ch);
	ledc->logical_duty[ch] = state->duty_cycle;
	if (state->enabled)
		set_bit(ch, &ledc->enabled_channels);
	else
		clear_bit(ch, &ledc->enabled_channels);
out:
	mutex_unlock(&ledc->lock);
	return ret;
}

static const struct pwm_ops espressif_ledc_ops = {
	.request = espressif_ledc_request,
	.free = espressif_ledc_free,
	.apply = espressif_ledc_apply,
};

static int espressif_ledc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct espressif_ledc *ledc;
	struct pwm_chip *chip;
	struct reset_control *rst;
	int ret;

	chip = devm_pwmchip_alloc(dev, 8, sizeof(*ledc));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	ledc = to_espressif_ledc(chip);
	mutex_init(&ledc->lock);

	ledc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ledc->base))
		return PTR_ERR(ledc->base);
	if (of_device_is_compatible(dev->of_node, "espressif,esp32s31-ledc")) {
		ledc->mem_lp = devm_platform_ioremap_resource(pdev, 1);
		if (IS_ERR(ledc->mem_lp))
			return PTR_ERR(ledc->mem_lp);
	}
	ledc->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(ledc->clk))
		return dev_err_probe(dev, PTR_ERR(ledc->clk), "clock unavailable\n");
	rst = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "reset unavailable\n");
	ret = reset_control_reset(rst);
	if (ret)
		return dev_err_probe(dev, ret, "reset failed\n");
	device_property_read_u32(dev, "espressif,fade-time-ms",
				 &ledc->fade_time_ms);
	ledc->gamma_correction = device_property_read_bool(dev,
						   "espressif,gamma-correction");

	/* S31 ledc_ll_mem_force_power_on() keeps channel/gamma RAM on. */
	if (ledc->mem_lp)
		writel((readl(ledc->mem_lp) & ~BIT(2)) | BIT(3), ledc->mem_lp);
	chip->ops = &espressif_ledc_ops;
	ret = pwmchip_add(chip);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, chip);
	return 0;
}

static void espressif_ledc_remove(struct platform_device *pdev)
{
	pwmchip_remove(platform_get_drvdata(pdev));
}

static const struct of_device_id espressif_ledc_of_match[] = {
	{ .compatible = "espressif,esp32s31-ledc" },
	{ .compatible = "espressif,esp32p4-ledc" },
	{ }
};
MODULE_DEVICE_TABLE(of, espressif_ledc_of_match);

static struct platform_driver espressif_ledc_driver = {
	.probe = espressif_ledc_probe,
	.remove = espressif_ledc_remove,
	.driver = {
		.name = "esp32s31-ledc",
		.of_match_table = espressif_ledc_of_match,
	},
};
module_platform_driver(espressif_ledc_driver);

MODULE_DESCRIPTION("Espressif ESP32-S31 and ESP32-P4 LEDC PWM driver");
MODULE_LICENSE("GPL");
