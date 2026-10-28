// SPDX-License-Identifier: GPL-2.0-only
/* SDK leases use the same PI locks and runtime references as native jobs.
 * A lease pins the provider from admission through its final clock disable
 * and unlock. Closing admission does not invalidate an existing owner's
 * release path. No radio-wide execution lock is involved.
 */

static unsigned int s31_crypto_domain_mask(unsigned int domain)
{
	switch (domain) {
	case S31_CRYPTO_KEY_MANAGER:
		/* Key Manager reset also interrupts an active ECC multiply. */
		return BIT(domain) | BIT(S31_CRYPTO_ECC);
	case S31_CRYPTO_BULK:
	case S31_CRYPTO_MPI:
	case S31_CRYPTO_ECC:
		return BIT(domain);
	case S31_CRYPTO_ECDSA:
		/* Even without SOC_ECDSA_USES_MPI, ECDSA reset propagates to
		 * SHA, MPI and ECC (see the S31 SDK's reset dependency clears).
		 * Serialize every affected engine across that reset.
		 */
		return BIT(domain) | BIT(S31_CRYPTO_BULK) | BIT(S31_CRYPTO_MPI) |
		       BIT(S31_CRYPTO_ECC) | BIT(S31_CRYPTO_KEY_MANAGER);
	case S31_CRYPTO_HMAC:
		/* HMAC needs DS released from reset. DS reset also affects MPI. */
		return BIT(domain) | BIT(S31_CRYPTO_BULK) | BIT(S31_CRYPTO_KEY_MANAGER) |
		       BIT(S31_CRYPTO_ECC) | BIT(S31_CRYPTO_DS) | BIT(S31_CRYPTO_MPI);
	case S31_CRYPTO_DS:
		return BIT(domain) | BIT(S31_CRYPTO_HMAC) | BIT(S31_CRYPTO_BULK) |
		       BIT(S31_CRYPTO_MPI) | BIT(S31_CRYPTO_KEY_MANAGER) | BIT(S31_CRYPTO_ECC);
	default:
		return 0;
	}
}

static unsigned int s31_crypto_clock_owner(unsigned int domain)
{
	return domain == S31_CRYPTO_AES || domain == S31_CRYPTO_SHA ?
		S31_CRYPTO_BULK : domain;
}

static u32 s31_crypto_failed_clocks(struct s31_crypto_dev *dev)
{
	unsigned int failed = atomic_read(&dev->failed_domains);
	u32 clocks = 0;

	if (failed & BIT(S31_CRYPTO_BULK))
		clocks |= S31_CRYPTO_AES_CLK | S31_CRYPTO_SHA_CLK;
	if (failed & BIT(S31_CRYPTO_MPI))
		clocks |= S31_CRYPTO_RSA_CLK;
	if (failed & BIT(S31_CRYPTO_ECC))
		clocks |= S31_CRYPTO_ECC_CLK;
	if (failed & BIT(S31_CRYPTO_ECDSA))
		clocks |= S31_CRYPTO_ECDSA_CLK;
	if (failed & BIT(S31_CRYPTO_HMAC))
		clocks |= S31_CRYPTO_HMAC_CLK;
	if (failed & BIT(S31_CRYPTO_DS))
		clocks |= S31_CRYPTO_DS_CLK;
	return clocks;
}

/* Caller owns every affected engine. A failed reset cannot establish that
 * private scratch is clean or the engine is stopped: retain reset and close
 * admission until removal/reprobe has run the hardware self-tests again.
 */
static void s31_crypto_fault(struct s31_crypto_dev *dev, unsigned int mask)
{
	unsigned long flags;
	unsigned int previous;
	u32 clocks, value;

	previous = atomic_fetch_or(mask, &dev->failed_domains);
	clocks = s31_crypto_failed_clocks(dev);
	raw_spin_lock_irqsave(&dev->clock_lock, flags);
	value = readl(dev->clkrst);
	writel((value | (clocks << 1)) & ~clocks, dev->clkrst);
	readl(dev->clkrst);
	if (mask & BIT(S31_CRYPTO_KEY_MANAGER)) {
		value = readl(dev->clkrst + 4);
		writel((value | BIT(1)) & ~(BIT(0) | BIT(2)), dev->clkrst + 4);
		readl(dev->clkrst + 4);
	}
	raw_spin_unlock_irqrestore(&dev->clock_lock, flags);
	if (mask & ~previous)
		dev_err(dev->dev, "hardware failure; crypto domains %#x unavailable until reprobe\n",
			mask);
}

/* A failed shared reset also makes scratch cleanup of this composite owner
 * unprovable. Quarantine its held engines, never engines owned by another task.
 */
static unsigned int s31_crypto_owned_mask(struct s31_crypto_dev *dev)
{
	unsigned int i, mask = 0;

	for (i = 0; i < S31_CRYPTO_DOMAINS; i++)
		if (READ_ONCE(dev->engine_owner[i]) == current)
			mask |= BIT(i);
	return mask;
}

static void s31_crypto_domain_unlock(struct s31_crypto_dev *dev, unsigned int mask)
{
	unsigned long flags;
	int i;

	for (i = S31_CRYPTO_DOMAINS - 1; i >= 0; i--) {
		if (!(mask & BIT(i)))
			continue;
		spin_lock_irqsave(&s31_crypto_admission, flags);
		if (--dev->engine_depth[i]) {
			spin_unlock_irqrestore(&s31_crypto_admission, flags);
			continue;
		}
		WRITE_ONCE(dev->engine_owner[i], NULL);
		spin_unlock_irqrestore(&s31_crypto_admission, flags);
		rt_mutex_unlock(&dev->engine_lock[i]);
	}
}

static long s31_crypto_sdk_acquire(unsigned int domain)
{
	struct s31_crypto_dev *dev;
	unsigned int mask = s31_crypto_domain_mask(domain), held = 0, i;
	unsigned long flags;
	int ret;

	if (!mask)
		return -EINVAL;
	spin_lock_irqsave(&s31_crypto_admission, flags);
	dev = s31_crypto_provider;
	if (!dev) {
		ret = -ENODEV;
		goto refuse;
	}
	for (i = 0; i < S31_CRYPTO_DOMAINS; i++)
		if (dev->engine_owner[i] == current)
			held |= BIT(i);
	if ((!s31_crypto || dev->suspending) && !held) {
		ret = -ESHUTDOWN;
		goto refuse;
	}
	/* Nested dependencies must already be held or follow the total order. */
	if (held && (mask & ~held) && __ffs(mask & ~held) < fls(held) - 1) {
		ret = -EDEADLK;
		goto refuse;
	}
	atomic_inc(&dev->users);
	spin_unlock_irqrestore(&s31_crypto_admission, flags);
	for (i = 0; i < S31_CRYPTO_DOMAINS; i++) {
		if (!(mask & BIT(i)))
			continue;
		if (!(held & BIT(i)))
			rt_mutex_lock(&dev->engine_lock[i]);
		spin_lock_irqsave(&s31_crypto_admission, flags);
		WRITE_ONCE(dev->engine_owner[i], current);
		dev->engine_depth[i]++;
		spin_unlock_irqrestore(&s31_crypto_admission, flags);
	}
	/* A previous owner may have failed while this task waited for its lock. */
	if (atomic_read(&dev->failed_domains) & mask) {
		s31_crypto_domain_unlock(dev, mask);
		s31_crypto_put(dev);
		return -EIO;
	}
	ret = pm_runtime_resume_and_get(dev->dev);
	if (ret < 0) {
		s31_crypto_domain_unlock(dev, mask);
		s31_crypto_put(dev);
		return ret;
	}
	spin_lock_irqsave(&s31_crypto_admission, flags);
	WRITE_ONCE(dev->domain_owner[domain], current);
	dev->domain_depth[domain]++;
	spin_unlock_irqrestore(&s31_crypto_admission, flags);
	return 0;
refuse:
	spin_unlock_irqrestore(&s31_crypto_admission, flags);
	return ret;
}

static long s31_crypto_sdk_release(struct s31_crypto_dev *dev, unsigned int domain)
{
	unsigned int mask = s31_crypto_domain_mask(domain), i, owner;
	unsigned long flags;

	if (!mask || READ_ONCE(dev->domain_owner[domain]) != current)
		return -EPERM;
	for (i = 0; i <= S31_CRYPTO_SHA; i++) {
		owner = s31_crypto_clock_owner(i);
		if (owner < S31_CRYPTO_DOMAINS && (mask & BIT(owner)) &&
		    dev->engine_depth[owner] == 1 && dev->sdk_clock_depth[i])
			return -EBUSY;
	}
	spin_lock_irqsave(&s31_crypto_admission, flags);
	if (!--dev->domain_depth[domain])
		WRITE_ONCE(dev->domain_owner[domain], NULL);
	spin_unlock_irqrestore(&s31_crypto_admission, flags);
	s31_crypto_domain_unlock(dev, mask);
	pm_runtime_mark_last_busy(dev->dev);
	pm_runtime_put_autosuspend(dev->dev);
	s31_crypto_put(dev);
	return 0;
}

static long s31_crypto_sdk_clock(struct s31_crypto_dev *dev, unsigned int domain,
				 bool enable)
{
	void __iomem *clock = dev->clkrst;
	unsigned long flags;
	u32 bit, reset, value;
	int ret = 0;

	switch (domain) {
	case S31_CRYPTO_AES: bit = S31_CRYPTO_AES_CLK; break;
	case S31_CRYPTO_SHA: bit = S31_CRYPTO_SHA_CLK; break;
	case S31_CRYPTO_MPI: bit = S31_CRYPTO_RSA_CLK; break;
	case S31_CRYPTO_ECC: bit = S31_CRYPTO_ECC_CLK; break;
	case S31_CRYPTO_HMAC: bit = S31_CRYPTO_HMAC_CLK; break;
	case S31_CRYPTO_DS: bit = S31_CRYPTO_DS_CLK; break;
	case S31_CRYPTO_ECDSA: bit = S31_CRYPTO_ECDSA_CLK; break;
	case S31_CRYPTO_KEY_MANAGER:
		/* Refuse before any reset/reference change. A KM reset must not
		 * invalidate an already active flash/PSRAM encryption key. Those
		 * key types are outside this provider's supported deployment API.
		 */
		if (enable && (readl(dev->key_manager + 0x30) & (BIT(2) | BIT(5))))
			return -EBUSY;
		clock += 4;
		bit = BIT(0) | BIT(2);
		break;
	default:
		return -EINVAL;
	}
	if (enable && (atomic_read(&dev->failed_domains) &
		       s31_crypto_domain_mask(s31_crypto_clock_owner(domain))))
		return -EIO;
	if (enable) {
		if (dev->sdk_clock_depth[domain]++)
			return 0;
	} else {
		if (!dev->sdk_clock_depth[domain])
			return -EINVAL;
		if (--dev->sdk_clock_depth[domain])
			return 0;
	}
	if (atomic_read(&dev->failed_domains) &
	    s31_crypto_domain_mask(s31_crypto_clock_owner(domain))) {
		s31_crypto_fault(dev, s31_crypto_domain_mask(s31_crypto_clock_owner(domain)));
		return -EIO;
	}
	reset = domain == S31_CRYPTO_KEY_MANAGER ? BIT(1) : bit << 1;
	raw_spin_lock_irqsave(&dev->clock_lock, flags);
	value = readl(clock) | bit;
	writel(value | reset, clock);
	readl(clock);
	writel(value & ~reset, clock);
	readl(clock);
	raw_spin_unlock_irqrestore(&dev->clock_lock, flags);
	/* Poll outside the shared register lock: independent engines keep running. */
	if (domain == S31_CRYPTO_ECDSA)
		ret = readl_poll_timeout(dev->base + 0x6000 + 0x20, value,
					!(value & 3), 20, 10000);
	else if (domain == S31_CRYPTO_KEY_MANAGER)
		ret = readl_poll_timeout(dev->key_manager + 0x28, value,
					!(value & 3), 20, 10000);
	else if (domain == S31_CRYPTO_MPI)
		ret = readl_poll_timeout(dev->base + S31_RSA_BASE + S31_RSA_QUERY_CLEAN,
					value, value & BIT(0), 20, 10000);
	if (ret) {
		/* A failed enable owns no clock reference. Checked SDK callers
		 * can unwind only earlier successful enables without guessing
		 * whether this failure happened before or after the increment.
		 */
		if (enable)
			dev->sdk_clock_depth[domain]--;
		s31_crypto_fault(dev, s31_crypto_owned_mask(dev));
		return ret;
	}
	if (!enable && !ret) {
		switch (domain) {
		case S31_CRYPTO_AES:
			s31_clear_words(dev->base + S31_AES_KEY, AES_MAX_KEY_SIZE);
			s31_clear_words(dev->base + S31_AES_TEXT_IN, AES_BLOCK_SIZE);
			break;
		case S31_CRYPTO_SHA:
			s31_clear_words(dev->base + S31_SHA_BASE + S31_SHA_H_MEM, 64);
			s31_clear_words(dev->base + S31_SHA_BASE + S31_SHA_M_MEM, 128);
			break;
		case S31_CRYPTO_MPI:
			s31_clear_words(dev->base + S31_RSA_BASE, 4 * S31_RSA_MAX_BYTES);
			break;
		case S31_CRYPTO_ECC:
			s31_clear_words(dev->base + S31_ECC_BASE + S31_ECC_K_MEM, 3 * S31_ECC_MAX_BYTES);
			break;
		case S31_CRYPTO_KEY_MANAGER:
			s31_clear_words(dev->key_manager + 0x100, 0xa0);
			readl(dev->key_manager + 0x19c);
			break;
		case S31_CRYPTO_ECDSA:
			/* Reset has stopped the operation; clear operand RAM before
			 * disabling its clock. The ECDSA composite lease covers ECC.
			 */
			s31_clear_words(dev->base + 0x6000 + 0x3e0, 5 * S31_ECC_MAX_BYTES);
			readl(dev->base + 0x6000 + 0x4cc);
			break;
		default:
			break;
		}
	}
	/* Native workers take a runtime reference before waiting for their PI
	 * lock. Keep their four clocks live until autosuspend, even if an SDK
	 * owner is handing the engine to an already-admitted native waiter.
	 */
	if (!enable && (domain == S31_CRYPTO_KEY_MANAGER ||
			(bit & S31_CRYPTO_EXTRA_CLOCKS))) {
		raw_spin_lock_irqsave(&dev->clock_lock, flags);
		writel(readl(clock) & ~bit, clock);
		readl(clock);
		raw_spin_unlock_irqrestore(&dev->clock_lock, flags);
	}
	return ret;
}

long esp32s31_crypto_radio_operation(unsigned int operation, unsigned int domain)
{
	struct s31_crypto_dev *dev;
	unsigned int owner, i;
	unsigned long flags;
	long ret = 0;

	if (in_interrupt() || irqs_disabled() || in_atomic())
		return -EWOULDBLOCK;
	if (operation == S31_CRYPTO_ACQUIRE)
		return s31_crypto_sdk_acquire(domain);
	spin_lock_irqsave(&s31_crypto_admission, flags);
	dev = s31_crypto_provider;
	if (operation == S31_CRYPTO_QUIESCENT) {
		if (dev)
			for (i = 0; i < S31_CRYPTO_DOMAINS; i++)
				if (READ_ONCE(dev->engine_owner[i]) == current)
					ret = -EBUSY;
		spin_unlock_irqrestore(&s31_crypto_admission, flags);
		return ret;
	}
	owner = s31_crypto_clock_owner(domain);
	if (!dev || owner >= S31_CRYPTO_DOMAINS ||
	    READ_ONCE(dev->engine_owner[owner]) != current) {
		spin_unlock_irqrestore(&s31_crypto_admission, flags);
		return -EPERM;
	}
	/* This task's existing lease pins dev after dropping admission. */
	spin_unlock_irqrestore(&s31_crypto_admission, flags);
	if (operation == S31_CRYPTO_RELEASE)
		return s31_crypto_sdk_release(dev, domain);
	if (operation == S31_CRYPTO_FAULT) {
		if (domain >= S31_CRYPTO_DOMAINS ||
		    READ_ONCE(dev->domain_owner[domain]) != current)
			return -EPERM;
		s31_crypto_fault(dev, s31_crypto_owned_mask(dev));
		return -EIO;
	}
	if (operation == S31_CRYPTO_CLOCK_ON || operation == S31_CRYPTO_CLOCK_OFF)
		return s31_crypto_sdk_clock(dev, domain, operation == S31_CRYPTO_CLOCK_ON);
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(esp32s31_crypto_radio_operation);
