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
#include <linux/iopoll.h>
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
	u64 logical_duty[8];
	u32 fade_time_ms;
	bool gamma_correction;
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

static u32 esp32s31_ledc_map_duty(struct esp32s31_ledc *ledc, u64 duty_ns,
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

static int esp32s31_ledc_fade(struct esp32s31_ledc *ledc, unsigned int ch,
			      unsigned int resolution, u64 period_ns,
			      u64 old_ns, u64 new_ns)
{
	unsigned int ranges = ledc->gamma_correction ? 16 : 1;
	u64 total_cycles = DIV64_U64_ROUND_CLOSEST((u64)ledc->fade_time_ms *
						     NSEC_PER_MSEC, period_ns);
	u32 start, target, status;
	unsigned int range;

	start = esp32s31_ledc_map_duty(ledc, old_ns, period_ns, resolution);
	target = esp32s31_ledc_map_duty(ledc, new_ns, period_ns, resolution);
	if (!total_cycles || start == target)
		return 0;

	writel(start << 4, ledc->base + LEDC_CH_DUTY(ch));
	for (range = 0; range < ranges; range++) {
		u64 delta_ns = new_ns >= old_ns ? new_ns - old_ns :
						      old_ns - new_ns;
		u64 head_delta = div64_u64(delta_ns * range, ranges);
		u64 tail_delta = div64_u64(delta_ns * (range + 1), ranges);
		u64 logical_head = new_ns >= old_ns ? old_ns + head_delta :
						       old_ns - head_delta;
		u64 logical_tail = new_ns >= old_ns ? old_ns + tail_delta :
						       old_ns - tail_delta;
		u32 head = esp32s31_ledc_map_duty(ledc, logical_head,
						  period_ns, resolution);
		u32 tail = esp32s31_ledc_map_duty(ledc, logical_tail,
						  period_ns, resolution);
		u32 delta = abs((int)tail - (int)head);
		u32 steps = clamp_t(u32, delta, 1, 1023);
		u32 scale = delta ? clamp_t(u32,
			DIV_ROUND_CLOSEST(delta, steps), 1, 1023) : 0;
		u32 cycle = clamp_t(u64, div64_u64(total_cycles,
						   (u64)ranges * steps), 1, 1023);
		u32 entry = FIELD_PREP(LEDC_GAMMA_STEP, steps) |
			    FIELD_PREP(LEDC_GAMMA_SCALE, scale) |
			    FIELD_PREP(LEDC_GAMMA_CYCLE, cycle);

		if (tail >= head)
			entry |= LEDC_GAMMA_INCREASE;
		writel(entry, ledc->base + LEDC_GAMMA_RANGE(ch, range));
	}
	writel(FIELD_PREP(LEDC_GAMMA_ENTRY_NUM, ranges),
	       ledc->base + LEDC_GAMMA_CONF(ch));
	writel(LEDC_DUTY_DONE(ch), ledc->base + LEDC_INT_CLR);
	writel(LEDC_DUTY_START, ledc->base + LEDC_CH_CONF1(ch));
	return readl_poll_timeout(ledc->base + LEDC_INT_RAW, status,
				 status & LEDC_DUTY_DONE(ch), 100,
				 ledc->fade_time_ms * 1000 + 100000);
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

	duty = esp32s31_ledc_map_duty(ledc, state->duty_cycle,
				       state->period, resolution);
	writel(0, ledc->base + LEDC_CH_HPOINT(ch));

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

	if (state->enabled && ledc->fade_time_ms &&
	    ledc->logical_duty[ch] != state->duty_cycle) {
		ret = esp32s31_ledc_fade(ledc, ch, resolution, state->period,
					ledc->logical_duty[ch], state->duty_cycle);
		if (ret)
			goto out;
	}
	/* End on the exact target even when a gamma segment rounded a step. */
	writel(0, ledc->base + LEDC_GAMMA_CONF(ch));
	writel(duty << 4, ledc->base + LEDC_CH_DUTY(ch));
	writel(LEDC_DUTY_START, ledc->base + LEDC_CH_CONF1(ch));
	ledc->logical_duty[ch] = state->duty_cycle;
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
	struct reset_control *rst;
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
