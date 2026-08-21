// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S31 always-on system timer clocksource.
 *
 * The SYSTIMER is clocked from the XTAL through IDF's fixed 2.5 divider,
 * producing a 16 MHz counter that is independent of CPU frequency changes.
 */

#include <linux/clocksource.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <linux/clocksource/esp32s31-systimer.h>
#include <asm/fixmap.h>

#define S31_SYSTIMER_PHYS		0x20399000U

#define S31_SYSTIMER_UNIT1_OP		0x08
#define S31_SYSTIMER_UNIT1_VALUE_HI	0x48
#define S31_SYSTIMER_UNIT1_VALUE_LO	0x4c
#define S31_SYSTIMER_UPDATE		BIT(30)
#define S31_SYSTIMER_VALUE_VALID	BIT(29)
#define S31_SYSTIMER_VALUE_HI_MASK	GENMASK(19, 0)

#define S31_SYSTIMER_TARGET_HI(cpu)	(0x1c + (cpu) * 8)
#define S31_SYSTIMER_TARGET_LO(cpu)	(0x20 + (cpu) * 8)
#define S31_SYSTIMER_TARGET_CONF(cpu)	(0x34 + (cpu) * 4)
#define S31_SYSTIMER_COMP_LOAD(cpu)	(0x50 + (cpu) * 4)
#define S31_SYSTIMER_INT_ENA		0x64
#define S31_SYSTIMER_INT_ST		0x68
#define S31_SYSTIMER_INT_CLR		0x6c
#define S31_SYSTIMER_TARGET_EN(cpu)	BIT(24 - (cpu))
#define S31_SYSTIMER_COUNTER1_SEL	BIT(31)

static void __iomem *s31_systimer_base;
static u64 s31_systimer_last;
static DEFINE_RAW_SPINLOCK(s31_systimer_lock);

static void s31_systimer_ensure_mmio(void)
{
	u32 conf;

	if (likely(s31_systimer_base))
		return;

	/* timer_probe() and the RISC-V CPUHP timer callback both run before
	 * normal ioremap is available.  The fixed mapping is shared by the
	 * clocksource and the native per-CPU clockevents. */
	set_fixmap_io(FIX_S31_SYSTIMER, S31_SYSTIMER_PHYS);
	s31_systimer_base = (void __iomem *)fix_to_virt(FIX_S31_SYSTIMER);

	/* Counter 1 is Linux-owned, XTAL-derived and must not stall with either
	 * HP hart.  Keep the register clock and counter running. */
	conf = readl(s31_systimer_base);
	conf |= BIT(31) | BIT(29);
	conf &= ~(BIT(25) | BIT(26));
	writel(conf, s31_systimer_base);
}

static u64 notrace s31_systimer_read_counter(void)
{
	u32 status, hi, lo;
	unsigned int retries;

	writel(S31_SYSTIMER_UPDATE, s31_systimer_base +
	       S31_SYSTIMER_UNIT1_OP);
	for (retries = 0; retries < 100; retries++) {
		status = readl(s31_systimer_base + S31_SYSTIMER_UNIT1_OP);
		if (status & S31_SYSTIMER_VALUE_VALID)
			break;
		cpu_relax();
	}
	if (retries == 100)
		return s31_systimer_last;

	hi = readl(s31_systimer_base + S31_SYSTIMER_UNIT1_VALUE_HI) &
	     S31_SYSTIMER_VALUE_HI_MASK;
	lo = readl(s31_systimer_base + S31_SYSTIMER_UNIT1_VALUE_LO);
	return ((u64)hi << 32) | lo;
}

static u64 notrace s31_systimer_read(struct clocksource *cs)
{
	/* Snapshot counter 1, leaving the IDF counter 0 untouched. */
	s31_systimer_last = s31_systimer_read_counter();
	return s31_systimer_last;
}

void esp32s31_systimer_stop(void)
{
	unsigned long flags;
	unsigned int cpu = raw_smp_processor_id();
	u32 val;

	if (WARN_ON_ONCE(cpu > 1))
		return;

	s31_systimer_ensure_mmio();
	raw_spin_lock_irqsave(&s31_systimer_lock, flags);

	val = readl(s31_systimer_base);
	writel(val & ~S31_SYSTIMER_TARGET_EN(cpu), s31_systimer_base);
	val = readl(s31_systimer_base + S31_SYSTIMER_INT_ENA);
	writel(val & ~BIT(cpu), s31_systimer_base + S31_SYSTIMER_INT_ENA);
	writel(BIT(cpu), s31_systimer_base + S31_SYSTIMER_INT_CLR);

	raw_spin_unlock_irqrestore(&s31_systimer_lock, flags);
}

bool esp32s31_systimer_irq_pending(unsigned int cpu)
{
	if (WARN_ON_ONCE(cpu > 1))
		return false;

	s31_systimer_ensure_mmio();
	return readl(s31_systimer_base + S31_SYSTIMER_INT_ST) & BIT(cpu);
}

int esp32s31_systimer_set_next_event(unsigned long delta)
{
	unsigned long flags;
	unsigned int cpu = raw_smp_processor_id();
	u64 target;
	u32 val;

	if (WARN_ON_ONCE(cpu > 1))
		return -EINVAL;

	s31_systimer_ensure_mmio();
	raw_spin_lock_irqsave(&s31_systimer_lock, flags);

	/* Disable and acknowledge the previous one-shot before loading a target.
	 * Counter 1 and both comparators are shared, so serialize the register
	 * read/modify/write sequences across the two HP harts. */
	val = readl(s31_systimer_base);
	writel(val & ~S31_SYSTIMER_TARGET_EN(cpu), s31_systimer_base);
	val = readl(s31_systimer_base + S31_SYSTIMER_INT_ENA);
	writel(val & ~BIT(cpu), s31_systimer_base + S31_SYSTIMER_INT_ENA);
	writel(BIT(cpu), s31_systimer_base + S31_SYSTIMER_INT_CLR);

	target = s31_systimer_read_counter() + max_t(unsigned long, delta, 2);
	writel(upper_32_bits(target) & S31_SYSTIMER_VALUE_HI_MASK,
	       s31_systimer_base + S31_SYSTIMER_TARGET_HI(cpu));
	writel(lower_32_bits(target),
	       s31_systimer_base + S31_SYSTIMER_TARGET_LO(cpu));
	/* Target mode (not periodic), connected to Linux-owned counter 1. */
	writel(S31_SYSTIMER_COUNTER1_SEL,
	       s31_systimer_base + S31_SYSTIMER_TARGET_CONF(cpu));
	writel(1, s31_systimer_base + S31_SYSTIMER_COMP_LOAD(cpu));

	val = readl(s31_systimer_base + S31_SYSTIMER_INT_ENA);
	writel(val | BIT(cpu), s31_systimer_base + S31_SYSTIMER_INT_ENA);
	val = readl(s31_systimer_base);
	writel(val | S31_SYSTIMER_TARGET_EN(cpu), s31_systimer_base);

	raw_spin_unlock_irqrestore(&s31_systimer_lock, flags);
	return 0;
}

static struct clocksource s31_systimer_clocksource = {
	.name = "esp32s31_systimer",
	.rating = 450,
	.mask = CLOCKSOURCE_MASK(52),
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
	.read = s31_systimer_read,
	.vdso_clock_mode = VDSO_CLOCKMODE_NONE,
};

static int __init s31_systimer_init(struct device_node *np)
{
	int ret;

	s31_systimer_ensure_mmio();

	ret = clocksource_register_hz(&s31_systimer_clocksource,
				     ESP32S31_SYSTIMER_RATE);
	if (ret) {
		pr_err("failed to register clocksource: %d\n", ret);
		return ret;
	}

	pr_info("registered 16 MHz always-on clocksource\n");
	return 0;
}

TIMER_OF_DECLARE(esp32s31_systimer, "espressif,esp32s31-systimer",
		 s31_systimer_init);
