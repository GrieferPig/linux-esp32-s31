// SPDX-License-Identifier: GPL-2.0-only
/* RTC target 0 belongs to HP while awake, and to the LP/SBI sleep handoff
 * during suspend. Keep its resource and lock in the LP controller rather
 * than mapping the same registers in a second platform driver.
 */

static bool esp32s31_lp_rtc_due(u64 now, u64 target)
{
	return ((now - target) & S31_RTC_TIMER_MASK) < BIT_ULL(47);
}

/* Caller holds rtc_lock. Calibration is fixed until the next cold boot. */
static u64 esp32s31_lp_rtc_elapsed_us(struct esp32s31_lp *lp, u64 now)
{
	return mul_u64_u32_shr((now - lp->rtc_base_ticks) & S31_RTC_TIMER_MASK,
			      lp->rtc_cal, S31_RTC_CAL_FRACT);
}

/* Caller holds rtc_lock (or is probing before RTC registration). Publish the
 * retained anchor atomically with respect to reset by writing magic last.
 */
static void esp32s31_lp_rtc_save_anchor(struct esp32s31_lp *lp)
{
	struct s31_lp_rtc_anchor anchor = {
		.magic = S31_LP_RTC_ANCHOR_MAGIC,
		.version = S31_LP_RTC_ANCHOR_VERSION,
		.size = sizeof(anchor),
		.seconds_lo = lower_32_bits(lp->rtc_base_time),
		.seconds_hi = upper_32_bits(lp->rtc_base_time),
		.ticks_lo = lower_32_bits(lp->rtc_base_ticks),
		.ticks_hi = upper_32_bits(lp->rtc_base_ticks),
		.calibration = lp->rtc_cal,
	};
	void __iomem *record = lp->sram + S31_LP_RTC_ANCHOR_OFFSET;

	anchor.crc = crc32_le(~0U, (const u8 *)&anchor,
			      offsetof(struct s31_lp_rtc_anchor, crc)) ^ ~0U;
	writel(0, record);
	memcpy_toio(record + sizeof(anchor.magic),
		    (const u8 *)&anchor + sizeof(anchor.magic),
		    sizeof(anchor) - sizeof(anchor.magic));
	wmb();
	writel(anchor.magic, record);
}

static void esp32s31_lp_rtc_restore_anchor(struct esp32s31_lp *lp)
{
	struct s31_lp_rtc_anchor anchor;
	void __iomem *record = lp->sram + S31_LP_RTC_ANCHOR_OFFSET;
	u64 now, ticks, delta, elapsed_us, seconds, fraction;
	u32 cause, crc, subsecond_us;

	/* Read the hardware reset cause, not ROM's optional STORE6 hint. Only
	 * known HP-only resets preserve both the counter and LP SRAM contract.
	 */
	cause = (readl(lp->reset_cause) >> 1) & 0x3f;
	memcpy_fromio(&anchor, record, sizeof(anchor));
	crc = crc32_le(~0U, (const u8 *)&anchor,
		       offsetof(struct s31_lp_rtc_anchor, crc)) ^ ~0U;
	if ((cause != 3 && cause != 5 && cause != 12) ||
	    anchor.magic != S31_LP_RTC_ANCHOR_MAGIC ||
	    anchor.version != S31_LP_RTC_ANCHOR_VERSION ||
	    anchor.size != sizeof(anchor) || anchor.reserved ||
	    anchor.ticks_hi > 0xffff || anchor.crc != crc ||
	    anchor.calibration <
		DIV_ROUND_UP_ULL(1000000ULL << S31_RTC_CAL_FRACT, 300000) ||
	    anchor.calibration >
		div_u64(1000000ULL << S31_RTC_CAL_FRACT, 20000))
		goto invalid;
	now = esp32s31_lp_rtc_count(lp);
	ticks = ((u64)anchor.ticks_hi << 32) | anchor.ticks_lo;
	delta = (now - ticks) & S31_RTC_TIMER_MASK;
	if (delta >= BIT_ULL(47))
		goto invalid;
	elapsed_us = mul_u64_u32_shr(delta, anchor.calibration, S31_RTC_CAL_FRACT);
	seconds = ((u64)anchor.seconds_hi << 32) | anchor.seconds_lo;
	if (seconds > RTC_TIMESTAMP_END_2099 ||
	    div_u64(elapsed_us, USEC_PER_SEC) > RTC_TIMESTAMP_END_2099 - seconds)
		goto invalid;
	lp->rtc_base_time = seconds + div_u64_rem(elapsed_us, USEC_PER_SEC,
						&subsecond_us);
	/* Re-anchor to the new calibration without losing the fractional second
	 * on each cold boot. Round the fractional age to one RTC tick.
	 */
	fraction = DIV_ROUND_CLOSEST_ULL((u64)subsecond_us << S31_RTC_CAL_FRACT, lp->rtc_cal);
	lp->rtc_base_ticks = (now - fraction) & S31_RTC_TIMER_MASK;
	lp->rtc_valid = true;
	esp32s31_lp_rtc_save_anchor(lp);
	return;
invalid:
	writel(0, record);
}

static void esp32s31_lp_rtc_disarm(struct esp32s31_lp *lp)
{
	writel(readl(lp->rtc_timer + S31_RTC_TIMER_INT_ENA_REG) &
	       ~S31_RTC_TIMER_SOC_WAKE, lp->rtc_timer + S31_RTC_TIMER_INT_ENA_REG);
	writel(readl(lp->rtc_timer + S31_RTC_TIMER_TARGET0_HIGH_REG) &
	       ~S31_RTC_TIMER_TARGET_EN,
	       lp->rtc_timer + S31_RTC_TIMER_TARGET0_HIGH_REG);
	writel(S31_RTC_TIMER_SOC_WAKE, lp->rtc_timer + S31_RTC_TIMER_INT_CLR_REG);
}

static void esp32s31_lp_rtc_arm(struct esp32s31_lp *lp)
{
	esp32s31_lp_rtc_disarm(lp);
	writel(lower_32_bits(lp->rtc_alarm_ticks),
	       lp->rtc_timer + S31_RTC_TIMER_TARGET0_LOW_REG);
	writel(upper_32_bits(lp->rtc_alarm_ticks) | S31_RTC_TIMER_TARGET_EN,
	       lp->rtc_timer + S31_RTC_TIMER_TARGET0_HIGH_REG);
	writel(readl(lp->rtc_timer + S31_RTC_TIMER_INT_ENA_REG) |
	       S31_RTC_TIMER_SOC_WAKE, lp->rtc_timer + S31_RTC_TIMER_INT_ENA_REG);
}

static void esp32s31_lp_rtc_notify(struct esp32s31_lp *lp)
{
	pm_wakeup_event(lp->dev, 0);
	rtc_update_irq(lp->rtc, 1, RTC_IRQF | RTC_AF);
}

static irqreturn_t esp32s31_lp_rtc_irq(int irq, void *data)
{
	struct esp32s31_lp *lp = data;
	unsigned long flags;
	bool fire;

	spin_lock_irqsave(&lp->rtc_lock, flags);
	if (!(readl(lp->rtc_timer + S31_RTC_TIMER_INT_RAW_REG) &
	      S31_RTC_TIMER_SOC_WAKE)) {
		spin_unlock_irqrestore(&lp->rtc_lock, flags);
		return IRQ_NONE;
	}
	esp32s31_lp_rtc_disarm(lp);
	fire = lp->rtc_alarm_enabled && !lp->rtc_alarm_handoff &&
	       esp32s31_lp_rtc_due(esp32s31_lp_rtc_count(lp), lp->rtc_alarm_ticks);
	if (fire) {
		lp->rtc_alarm_enabled = false;
		lp->rtc_alarm_pending = true;
	} else if (lp->rtc_alarm_enabled && !lp->rtc_alarm_handoff) {
		/* Ignore a stale status left by a previous sleep transaction. */
		esp32s31_lp_rtc_arm(lp);
	}
	spin_unlock_irqrestore(&lp->rtc_lock, flags);
	if (fire)
		esp32s31_lp_rtc_notify(lp);
	return IRQ_HANDLED;
}

static int esp32s31_lp_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct esp32s31_lp *lp = dev_get_drvdata(dev);
	unsigned long flags;
	time64_t seconds;

	spin_lock_irqsave(&lp->rtc_lock, flags);
	if (!lp->rtc_valid) {
		spin_unlock_irqrestore(&lp->rtc_lock, flags);
		return -EINVAL;
	}
	seconds = lp->rtc_base_time + div_u64(esp32s31_lp_rtc_elapsed_us(lp,
					 esp32s31_lp_rtc_count(lp)), USEC_PER_SEC);
	spin_unlock_irqrestore(&lp->rtc_lock, flags);
	rtc_time64_to_tm(seconds, tm);
	return 0;
}

/* Recompute an absolute target after setting wall time or an alarm. */
static int esp32s31_lp_rtc_target(struct esp32s31_lp *lp, u64 now)
{
	s64 remaining_us = (lp->rtc_alarm_time - lp->rtc_base_time) * USEC_PER_SEC -
			   esp32s31_lp_rtc_elapsed_us(lp, now);
	u64 ticks;

	if (remaining_us <= 0)
		return -ETIME;
	if (remaining_us > (u64)S31_RTC_ALARM_MAX_SECONDS * USEC_PER_SEC)
		return -ERANGE;
	ticks = DIV_ROUND_UP_ULL((u64)remaining_us << S31_RTC_CAL_FRACT,
				lp->rtc_cal);
	lp->rtc_alarm_ticks = (now + ticks) & S31_RTC_TIMER_MASK;
	return 0;
}

static int esp32s31_lp_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct esp32s31_lp *lp = dev_get_drvdata(dev);
	unsigned long flags;
	u64 now;
	bool fire = false;
	int ret;

	spin_lock_irqsave(&lp->rtc_lock, flags);
	if (lp->rtc_alarm_handoff) {
		spin_unlock_irqrestore(&lp->rtc_lock, flags);
		return -EBUSY;
	}
	now = esp32s31_lp_rtc_count(lp);
	lp->rtc_base_ticks = now;
	lp->rtc_base_time = rtc_tm_to_time64(tm);
	lp->rtc_valid = true;
	esp32s31_lp_rtc_save_anchor(lp);
	if (lp->rtc_alarm_enabled) {
		ret = esp32s31_lp_rtc_target(lp, now);
		if (!ret) {
			esp32s31_lp_rtc_arm(lp);
		} else {
			esp32s31_lp_rtc_disarm(lp);
			lp->rtc_alarm_enabled = false;
			fire = ret == -ETIME;
			lp->rtc_alarm_pending = fire;
		}
	}
	spin_unlock_irqrestore(&lp->rtc_lock, flags);
	if (fire)
		esp32s31_lp_rtc_notify(lp);
	return 0;
}

static int esp32s31_lp_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct esp32s31_lp *lp = dev_get_drvdata(dev);
	unsigned long flags;

	spin_lock_irqsave(&lp->rtc_lock, flags);
	rtc_time64_to_tm(lp->rtc_alarm_time, &alarm->time);
	alarm->enabled = lp->rtc_alarm_enabled;
	alarm->pending = lp->rtc_alarm_pending;
	spin_unlock_irqrestore(&lp->rtc_lock, flags);
	return 0;
}

static int esp32s31_lp_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct esp32s31_lp *lp = dev_get_drvdata(dev);
	unsigned long flags;
	time64_t previous;
	int ret = 0;

	spin_lock_irqsave(&lp->rtc_lock, flags);
	if (lp->rtc_alarm_handoff) {
		ret = -EBUSY;
		goto out;
	}
	if (!lp->rtc_valid) {
		ret = -EINVAL;
		goto out;
	}
	previous = lp->rtc_alarm_time;
	lp->rtc_alarm_time = rtc_tm_to_time64(&alarm->time);
	if (alarm->enabled) {
		ret = esp32s31_lp_rtc_target(lp, esp32s31_lp_rtc_count(lp));
		if (ret) {
			lp->rtc_alarm_time = previous;
			goto out;
		}
	}
	lp->rtc_alarm_enabled = alarm->enabled;
	lp->rtc_alarm_pending = false;
	if (alarm->enabled)
		esp32s31_lp_rtc_arm(lp);
	else
		esp32s31_lp_rtc_disarm(lp);
out:
	spin_unlock_irqrestore(&lp->rtc_lock, flags);
	return ret;
}

static int esp32s31_lp_rtc_alarm_enable(struct device *dev, unsigned int enabled)
{
	struct esp32s31_lp *lp = dev_get_drvdata(dev);
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&lp->rtc_lock, flags);
	if (lp->rtc_alarm_handoff) {
		ret = -EBUSY;
		goto out;
	}
	if (enabled) {
		if (!lp->rtc_valid) {
			ret = -EINVAL;
			goto out;
		}
		ret = esp32s31_lp_rtc_target(lp, esp32s31_lp_rtc_count(lp));
		if (ret)
			goto out;
		esp32s31_lp_rtc_arm(lp);
	} else {
		esp32s31_lp_rtc_disarm(lp);
	}
	lp->rtc_alarm_enabled = enabled;
	lp->rtc_alarm_pending = false;
out:
	spin_unlock_irqrestore(&lp->rtc_lock, flags);
	return ret;
}

static int esp32s31_lp_rtc_ioctl(struct device *dev, unsigned int cmd,
			       unsigned long arg)
{
	struct esp32s31_lp *lp = dev_get_drvdata(dev);

	if (cmd != RTC_VL_READ)
		return -ENOIOCTLCMD;
	return put_user(READ_ONCE(lp->rtc_valid) ? 0 : RTC_VL_DATA_INVALID,
			(unsigned int __user *)arg);
}

static const struct rtc_class_ops esp32s31_lp_rtc_ops = {
	.read_time = esp32s31_lp_rtc_read_time,
	.set_time = esp32s31_lp_rtc_set_time,
	.read_alarm = esp32s31_lp_rtc_read_alarm,
	.set_alarm = esp32s31_lp_rtc_set_alarm,
	.alarm_irq_enable = esp32s31_lp_rtc_alarm_enable,
	.ioctl = esp32s31_lp_rtc_ioctl,
};

static void esp32s31_lp_rtc_cleanup(void *data)
{
	struct esp32s31_lp *lp = data;

	esp32s31_lp_rtc_disarm(lp);
}

static int esp32s31_lp_rtc_register(struct platform_device *pdev,
				  struct esp32s31_lp *lp)
{
	int irq, ret;

	lp->rtc = devm_rtc_allocate_device(lp->dev);
	if (IS_ERR(lp->rtc))
		return PTR_ERR(lp->rtc);
	lp->rtc->ops = &esp32s31_lp_rtc_ops;
	lp->rtc->range_min = 0;
	lp->rtc->range_max = RTC_TIMESTAMP_END_2099;
	lp->rtc->alarm_offset_max = S31_RTC_ALARM_MAX_SECONDS;
	clear_bit(RTC_FEATURE_UPDATE_INTERRUPT, lp->rtc->features);
	esp32s31_lp_rtc_disarm(lp);
	irq = platform_get_irq_byname(pdev, "rtc-alarm");
	if (irq < 0)
		return irq;
	lp->rtc_irq = irq;
	ret = devm_request_irq(lp->dev, irq, esp32s31_lp_rtc_irq, 0,
			       "esp32s31-rtc", lp);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(lp->dev, esp32s31_lp_rtc_cleanup, lp);
	if (ret)
		return ret;
	esp32s31_lp_rtc_restore_anchor(lp);
	return devm_rtc_register_device(lp->rtc);
}

static int esp32s31_lp_rtc_handoff(struct esp32s31_lp *lp,
				 struct s31_lp_sleep_control *control)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&lp->rtc_lock, flags);
	if (lp->rtc_alarm_enabled && device_may_wakeup(lp->dev)) {
		if (esp32s31_lp_rtc_due(esp32s31_lp_rtc_count(lp), lp->rtc_alarm_ticks)) {
			ret = -EBUSY;
			goto out;
		}
		control->flags |= S31_LP_SLEEP_F_RTC_ABSOLUTE;
		control->deadline_lo = lower_32_bits(lp->rtc_alarm_ticks);
		control->deadline_hi = upper_32_bits(lp->rtc_alarm_ticks);
	}
	/* SBI uses target 0 even for an explicit relative timer. Restore any
	 * non-wakeup alarm on resume, too; it must not consume that timer IRQ. */
	lp->rtc_alarm_handoff = true;
	esp32s31_lp_rtc_disarm(lp);
out:
	spin_unlock_irqrestore(&lp->rtc_lock, flags);
	return ret;
}

static void esp32s31_lp_rtc_restore(struct esp32s31_lp *lp)
{
	unsigned long flags;
	bool fire = false;

	spin_lock_irqsave(&lp->rtc_lock, flags);
	if (!lp->rtc_alarm_handoff)
		goto out;
	lp->rtc_alarm_handoff = false;
	esp32s31_lp_rtc_disarm(lp);
	if (lp->rtc_alarm_enabled) {
		fire = esp32s31_lp_rtc_due(esp32s31_lp_rtc_count(lp), lp->rtc_alarm_ticks);
		if (fire) {
			lp->rtc_alarm_enabled = false;
			lp->rtc_alarm_pending = true;
		} else {
			esp32s31_lp_rtc_arm(lp);
		}
	}
out:
	spin_unlock_irqrestore(&lp->rtc_lock, flags);
	if (fire)
		esp32s31_lp_rtc_notify(lp);
}
