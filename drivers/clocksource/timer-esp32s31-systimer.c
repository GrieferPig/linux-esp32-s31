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
#include <linux/sched_clock.h>
#include <linux/hardirq.h>
#include <linux/seqlock.h>
#include <linux/spinlock.h>
#include <linux/syscore_ops.h>
#include <linux/timekeeping.h>
#include <linux/clocksource/esp32s31-systimer.h>
#include <asm/fixmap.h>
#include <asm/irqflags.h>

#define S31_SYSTIMER_PHYS		0x20399000U
#define S31_HP_CLKRST_PHYS		0x20587000U
#define S31_RTC_TIMER_PHYS		0x20800000U
#define S31_HP_CLKRST_SYSTIMER_CTRL0	0x120
#define S31_SYSTIMER_APB_CLK_EN		BIT(0)
#define S31_SYSTIMER_RST_EN		BIT(1)
#define S31_SYSTIMER_FORCE_NORST		BIT(2)
#define S31_SYSTIMER_CLK_SRC_SEL	BIT(3)
#define S31_SYSTIMER_FUNC_CLK_EN	BIT(4)

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
#define S31_SYSTIMER_ALL_TARGETS	GENMASK(2, 0)
#define S31_SYSTIMER_ALL_TARGET_EN	GENMASK(24, 22)
#define S31_SYSTIMER_TARGET_EN(cpu)	BIT(24 - (cpu))
#define S31_SYSTIMER_COUNTER1_SEL	BIT(31)

#define S31_RTC_TIMER_UPDATE		0x10
#define S31_RTC_TIMER_COUNTER0_LO	0x14
#define S31_RTC_TIMER_COUNTER0_HI	0x18
#define S31_RTC_TIMER_DATE		0x3fc
#define S31_RTC_TIMER_SNAPSHOT		BIT(27)
#define S31_RTC_TIMER_SYS_STALL		BIT(30)
#define S31_RTC_TIMER_SYS_RST		BIT(31)
#define S31_RTC_TIMER_CLK_EN		BIT(31)
#define S31_RTC_TIMER_HI_MASK		GENMASK(15, 0)
#define S31_RTC_SLOW_RATE		32768U

static void __iomem *s31_systimer_base;
static void __iomem *s31_rtc_timer_base;
static u64 s31_systimer_last;
/* Serialized writers publish two copies so a nested NMI never waits on its
 * interrupted snapshot owner or observes an RV32 half-written u64. */
static seqcount_latch_t s31_systimer_cache_seq =
 SEQCNT_LATCH_ZERO(s31_systimer_cache_seq);
static u64 s31_systimer_cache[2];

static u64 notrace s31_systimer_cached_counter(void)
{
 unsigned int seq;
 u64 value;
 do {
  seq=read_seqcount_latch(&s31_systimer_cache_seq);
  value=READ_ONCE(s31_systimer_cache[seq & 1]);
 } while (read_seqcount_latch_retry(&s31_systimer_cache_seq,seq));
 return value;
}

static void notrace s31_systimer_publish_counter(u64 value)
{
 raw_write_seqcount_latch(&s31_systimer_cache_seq);
 WRITE_ONCE(s31_systimer_cache[0],value);
 raw_write_seqcount_latch(&s31_systimer_cache_seq);
 WRITE_ONCE(s31_systimer_cache[1],value);
}
static DEFINE_RAW_SPINLOCK(s31_systimer_lock);

static void s31_rtc_timer_ensure_mmio(void)
{
	u32 val;

	if (likely(s31_rtc_timer_base))
		return;

	set_fixmap_io(FIX_S31_RTC_TIMER, S31_RTC_TIMER_PHYS);
	s31_rtc_timer_base = (void __iomem *)fix_to_virt(FIX_S31_RTC_TIMER);
	val = readl(s31_rtc_timer_base + S31_RTC_TIMER_DATE);
	writel(val | S31_RTC_TIMER_CLK_EN,
	       s31_rtc_timer_base + S31_RTC_TIMER_DATE);
	val = readl(s31_rtc_timer_base + S31_RTC_TIMER_UPDATE);
	val &= ~(S31_RTC_TIMER_SYS_STALL | S31_RTC_TIMER_SYS_RST);
	writel(val, s31_rtc_timer_base + S31_RTC_TIMER_UPDATE);
}

static u64 notrace s31_rtc_timer_read_counter(void)
{
	u32 val, hi, lo;

	s31_rtc_timer_ensure_mmio();
	val = readl(s31_rtc_timer_base + S31_RTC_TIMER_UPDATE);
	writel(val | S31_RTC_TIMER_SNAPSHOT,
	       s31_rtc_timer_base + S31_RTC_TIMER_UPDATE);
	lo = readl(s31_rtc_timer_base + S31_RTC_TIMER_COUNTER0_LO);
	hi = readl(s31_rtc_timer_base + S31_RTC_TIMER_COUNTER0_HI) &
	     S31_RTC_TIMER_HI_MASK;
	return ((u64)hi << 32) | lo;
}

void read_persistent_clock64(struct timespec64 *ts)
{
	u64 cycles = s31_rtc_timer_read_counter();
	u32 rem;

	ts->tv_sec = div_u64_rem(cycles, S31_RTC_SLOW_RATE, &rem);
	ts->tv_nsec = div_u64((u64)rem * NSEC_PER_SEC,
			      S31_RTC_SLOW_RATE);
}

static void s31_systimer_ensure_mmio(void)
{
	void __iomem *clkrst;
	u32 conf;

	if (likely(s31_systimer_base))
		return;

	/*
	 * The HP clock/reset gate powers up disabled and is not part of the
	 * SYSTIMER page. Do not depend on ROM, SPL, or an earlier IDF image to
	 * leave it enabled: establish a known XTAL-clocked peripheral state
	 * before the first SYSTIMER access.
	 */
	set_fixmap_io(FIX_S31_CLKRST, S31_HP_CLKRST_PHYS);
	clkrst = (void __iomem *)fix_to_virt(FIX_S31_CLKRST);
	conf = readl(clkrst + S31_HP_CLKRST_SYSTIMER_CTRL0);
	conf |= S31_SYSTIMER_APB_CLK_EN | S31_SYSTIMER_FUNC_CLK_EN;
	/* ESP-IDF maps clk_src_sel=0 to XTAL and 1 to RC_FAST. */
	conf &= ~(S31_SYSTIMER_RST_EN | S31_SYSTIMER_CLK_SRC_SEL);
	writel(conf,
	       clkrst + S31_HP_CLKRST_SYSTIMER_CTRL0);
	readl(clkrst + S31_HP_CLKRST_SYSTIMER_CTRL0);

	/* timer_probe() and the RISC-V CPUHP timer callback both run before
	 * normal ioremap is available.  The fixed mapping is shared by the
	 * clocksource and the native per-CPU clockevents. */
	set_fixmap_io(FIX_S31_SYSTIMER, S31_SYSTIMER_PHYS);
	s31_systimer_base = (void __iomem *)fix_to_virt(FIX_S31_SYSTIMER);

	/* Counter 1 is Linux-owned, XTAL-derived and must not stall with either
	 * HP hart.  Keep the register clock and counter running. */
	writel(0, s31_systimer_base + S31_SYSTIMER_INT_ENA);
	writel(S31_SYSTIMER_ALL_TARGETS,
	       s31_systimer_base + S31_SYSTIMER_INT_CLR);
	readl(s31_systimer_base + S31_SYSTIMER_INT_ST);
	conf = readl(s31_systimer_base);
	conf &= ~S31_SYSTIMER_ALL_TARGET_EN;
	conf |= BIT(31) | BIT(29);
	conf &= ~(BIT(25) | BIT(26));
	writel(conf, s31_systimer_base);
}

static void s31_systimer_syscore_resume(void)
{
	void __iomem *clkrst;
	u32 conf;

	if (!s31_systimer_base)
		return;

	/* HP sleep resets/gates this peripheral without invalidating Linux's
	 * fixed virtual mapping.  Re-establish its clock and Linux-owned counter
	 * before the timekeeping core or IRQ polling path accesses it. */
	clkrst = (void __iomem *)fix_to_virt(FIX_S31_CLKRST);
	conf = readl(clkrst + S31_HP_CLKRST_SYSTIMER_CTRL0);
	conf |= S31_SYSTIMER_APB_CLK_EN | S31_SYSTIMER_FUNC_CLK_EN;
	conf &= ~(S31_SYSTIMER_RST_EN | S31_SYSTIMER_CLK_SRC_SEL);
	writel(conf, clkrst + S31_HP_CLKRST_SYSTIMER_CTRL0);
	readl(clkrst + S31_HP_CLKRST_SYSTIMER_CTRL0);

	writel(0, s31_systimer_base + S31_SYSTIMER_INT_ENA);
	writel(S31_SYSTIMER_ALL_TARGETS,
	       s31_systimer_base + S31_SYSTIMER_INT_CLR);
	conf = readl(s31_systimer_base);
	conf &= ~S31_SYSTIMER_ALL_TARGET_EN;
	conf |= BIT(31) | BIT(29);
	conf &= ~(BIT(25) | BIT(26));
	writel(conf, s31_systimer_base);
}

static struct syscore_ops s31_systimer_syscore_ops = {
	.resume = s31_systimer_syscore_resume,
};

static u64 notrace s31_systimer_read_counter_locked(void)
{

	u32 status, hi, lo;
	unsigned int retries;

	/* This locked transaction touches only one timer's snapshot registers;
	 * it neither publishes DMA ownership nor observes DMA completion. The
	 * relaxed accessors retain same-peripheral UPDATE/VALID/HI/LO ordering
	 * without fencing unrelated normal-memory traffic on every clock read.
	 * Keep the shared snapshot lock and RV32/NMI latch publication below.
	 */
	/* VALUE_VALID is write-to-clear. The SDK's UPDATE bitfield RMW
	 * writes an observed VALID=1 back as well. Clear it explicitly so
	 * the poll cannot accept a previous snapshot while this update crosses
	 * the clock domain; do not rely on full MMIO barriers providing delay.
	 */
	writel_relaxed(S31_SYSTIMER_UPDATE | S31_SYSTIMER_VALUE_VALID,
		       s31_systimer_base +
		       S31_SYSTIMER_UNIT1_OP);
	for (retries = 0; retries < 100; retries++) {
		status = readl_relaxed(s31_systimer_base + S31_SYSTIMER_UNIT1_OP);
		if (status & S31_SYSTIMER_VALUE_VALID)
			break;
		cpu_relax();
	}
	if (retries == 100)
		return s31_systimer_last;

	hi = readl_relaxed(s31_systimer_base + S31_SYSTIMER_UNIT1_VALUE_HI) &
	     S31_SYSTIMER_VALUE_HI_MASK;
	lo = readl_relaxed(s31_systimer_base + S31_SYSTIMER_UNIT1_VALUE_LO);
	s31_systimer_last = ((u64)hi << 32) | lo;
	s31_systimer_publish_counter(s31_systimer_last);
	return s31_systimer_last;
}

static u64 notrace s31_systimer_read(struct clocksource *cs)
{
 unsigned long flags;
 u64 value;
 (void)cs;
 /* Counter 1 snapshot registers are shared by both HP harts and clockevent
  * programming. Serialize update/valid/HI/LO with the existing device lock.
  * An NMI uses the completed latch cache; it never re-enters that lock. */
 if (in_nmi())
  return s31_systimer_cached_counter();
 raw_spin_lock_irqsave(&s31_systimer_lock,flags);
 value=s31_systimer_read_counter_locked();
 raw_spin_unlock_irqrestore(&s31_systimer_lock,flags);
 return value;
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
	readl(s31_systimer_base);
	val = readl(s31_systimer_base + S31_SYSTIMER_INT_ENA);
	writel(val & ~BIT(cpu), s31_systimer_base + S31_SYSTIMER_INT_ENA);
	writel(BIT(cpu), s31_systimer_base + S31_SYSTIMER_INT_CLR);
	readl(s31_systimer_base + S31_SYSTIMER_INT_ST);

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
	readl(s31_systimer_base);
	val = readl(s31_systimer_base + S31_SYSTIMER_INT_ENA);
	writel(val & ~BIT(cpu), s31_systimer_base + S31_SYSTIMER_INT_ENA);
	writel(BIT(cpu), s31_systimer_base + S31_SYSTIMER_INT_CLR);
	readl(s31_systimer_base + S31_SYSTIMER_INT_ST);

	target = s31_systimer_read_counter_locked() + max_t(unsigned long, delta, 2);
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

/* sched_clock is called with runqueue/timer locks held, from instrumentation,
 * and through the noinstr entry path. Never wait for the snapshot owner: it
 * may be this CPU, or another CPU waiting for a lock held by our caller.
 *
 * Use only the underlying arch lock here. Normal raw-spinlock lockdep and
 * tracing helpers may themselves ask for sched_clock before/after taking
 * the physical lock. The failed try path and NMI use the completed latch
 * cache, published before every successful fresh read returns. No callback,
 * allocation, per-task state or instrumented IRQ state transition is needed.
 * Local IRQ exclusion prevents another maskable reader on this CPU; nested
 * NMI readers never modify the snapshot registers or take the lock.
 */
static u64 notrace s31_systimer_sched_clock(void)
{
	unsigned long flags;
	u64 value;

	if (in_nmi())
		return s31_systimer_cached_counter();
	flags = arch_local_irq_save();
	if (arch_spin_trylock(&s31_systimer_lock.raw_lock)) {
		value = s31_systimer_read_counter_locked();
		arch_spin_unlock(&s31_systimer_lock.raw_lock);
	} else {
		value = s31_systimer_cached_counter();
	}
	arch_local_irq_restore(flags);
	return value;
}

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

	/* Register the shared always-on counter for scheduling/accounting too.
	 * timer-riscv must not register the faster WFI-stalling CLINT source:
	 * generic sched_clock registration keeps the highest frequency. */
	sched_clock_register(s31_systimer_sched_clock, 52,
			     ESP32S31_SYSTIMER_RATE);
	pr_info("registered 16 MHz always-on clocksource\n");
	register_syscore_ops(&s31_systimer_syscore_ops);
	return 0;
}

TIMER_OF_DECLARE(esp32s31_systimer, "espressif,esp32s31-systimer",
		 s31_systimer_init);
