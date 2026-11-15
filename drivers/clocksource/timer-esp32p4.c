// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-P4 SYSTIMER: one shared 52-bit 16 MHz clocksource and one comparator
 * per HP hart. Reconstructed against ESP-IDF a602e67b hw_ver1 registers.
 * Snapshot and shared CONF/INT_ENA updates are serialized across both harts.
 */
#define pr_fmt(fmt) "esp32p4-timer: " fmt

#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/cpu.h>
#include <linux/cpuhotplug.h>
#include <linux/hardirq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>
#include <linux/seqlock.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <asm/smp.h>

#define P4_TIMER_RATE          16000000
#define P4_SOC_CLK_CTRL2        0x1c
#define P4_PERI_CLK_CTRL21      0x98
#define P4_APB_ENABLE          BIT(23)
#define P4_CLK_SOURCE_RC_FAST   BIT(29)
#define P4_CLK_ENABLE          BIT(30)

#define P4_CONF                0x00
#define P4_UNIT0_OP            0x04
#define P4_UPDATE              BIT(30)
#define P4_VALUE_VALID         BIT(29)
#define P4_TARGET_HI(n)        (0x1c + (n) * 8)
#define P4_TARGET_LO(n)        (0x20 + (n) * 8)
#define P4_TARGET_CONF(n)      (0x34 + (n) * 4)
#define P4_VALUE_HI            0x40
#define P4_VALUE_LO            0x44
#define P4_COMP_LOAD(n)        (0x50 + (n) * 4)
#define P4_INT_ENA             0x64
#define P4_INT_CLR             0x6c
#define P4_INT_ST              0x70
#define P4_TARGET_EN(n)        BIT(24 - (n))
#define P4_COUNTER_HI_MASK     GENMASK(19, 0)

struct p4_timer_event {
	struct clock_event_device ce;
	unsigned int hart;
	unsigned int irq;
};

static void __iomem *p4_timer;
static struct p4_timer_event p4_events[2];
static DEFINE_RAW_SPINLOCK(p4_timer_lock);
static u64 p4_last_counter;
/* A nested NMI cannot wait for the lock held by the code it interrupted. */
static seqcount_latch_t p4_counter_seq = SEQCNT_LATCH_ZERO(p4_counter_seq);
static u64 p4_counter_copy[2];

static u64 notrace p4_cached_counter(void)
{
	unsigned int seq;
	u64 value;

	do {
		seq = read_seqcount_latch(&p4_counter_seq);
		value = READ_ONCE(p4_counter_copy[seq & 1]);
	} while (read_seqcount_latch_retry(&p4_counter_seq, seq));
	return value;
}

static u64 notrace p4_read_counter_locked(void)
{
	u32 hi, lo;
	unsigned int tries;

	/* The vendor UPDATE bitfield RMW also writes observed VALID back. */
	writel(P4_UPDATE | P4_VALUE_VALID, p4_timer + P4_UNIT0_OP);
	for (tries = 0; tries < 1000; tries++) {
		if (readl(p4_timer + P4_UNIT0_OP) & P4_VALUE_VALID)
			break;
		cpu_relax();
	}
	if (unlikely(tries == 1000))
		return p4_last_counter;

	hi = readl(p4_timer + P4_VALUE_HI) & P4_COUNTER_HI_MASK;
	lo = readl(p4_timer + P4_VALUE_LO);
	p4_last_counter = ((u64)hi << 32) | lo;
	raw_write_seqcount_latch(&p4_counter_seq);
	WRITE_ONCE(p4_counter_copy[0], p4_last_counter);
	raw_write_seqcount_latch(&p4_counter_seq);
	WRITE_ONCE(p4_counter_copy[1], p4_last_counter);
	return p4_last_counter;
}

static u64 notrace p4_read_counter(void)
{
	unsigned long flags;
	u64 value;

	if (unlikely(in_nmi()))
		return p4_cached_counter();
	raw_spin_lock_irqsave(&p4_timer_lock, flags);
	value = p4_read_counter_locked();
	raw_spin_unlock_irqrestore(&p4_timer_lock, flags);
	return value;
}

static u64 p4_clocksource_read(struct clocksource *cs)
{
	return p4_read_counter();
}

static struct clocksource p4_clocksource = {
	.name = "esp32p4-systimer",
	.rating = 300,
	.read = p4_clocksource_read,
	.mask = CLOCKSOURCE_MASK(52),
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
};

static int p4_event_shutdown(struct clock_event_device *ce)
{
	struct p4_timer_event *event = container_of(ce, struct p4_timer_event, ce);
	unsigned int hart = event->hart;
	unsigned long flags;
	u32 val;

	raw_spin_lock_irqsave(&p4_timer_lock, flags);
	val = readl(p4_timer + P4_CONF);
	writel(val & ~P4_TARGET_EN(hart), p4_timer + P4_CONF);
	val = readl(p4_timer + P4_INT_ENA);
	writel(val & ~BIT(hart), p4_timer + P4_INT_ENA);
	writel(BIT(hart), p4_timer + P4_INT_CLR);
	raw_spin_unlock_irqrestore(&p4_timer_lock, flags);
	return 0;
}

static int p4_event_next(unsigned long delta, struct clock_event_device *ce)
{
	struct p4_timer_event *event = container_of(ce, struct p4_timer_event, ce);
	unsigned int hart = event->hart;
	unsigned long flags;
	u64 target;
	u32 val;

	raw_spin_lock_irqsave(&p4_timer_lock, flags);
	val = readl(p4_timer + P4_CONF);
	writel(val & ~P4_TARGET_EN(hart), p4_timer + P4_CONF);
	writel(BIT(hart), p4_timer + P4_INT_CLR);
	target = (p4_read_counter_locked() + delta) & CLOCKSOURCE_MASK(52);
	writel(target >> 32, p4_timer + P4_TARGET_HI(hart));
	writel((u32)target, p4_timer + P4_TARGET_LO(hart));
	/* Unit 0, one-shot mode. Latch the comparator after writing both words. */
	writel(0, p4_timer + P4_TARGET_CONF(hart));
	writel(1, p4_timer + P4_COMP_LOAD(hart));
	val = readl(p4_timer + P4_INT_ENA);
	writel(val | BIT(hart), p4_timer + P4_INT_ENA);
	val = readl(p4_timer + P4_CONF);
	writel(val | P4_TARGET_EN(hart), p4_timer + P4_CONF);
	raw_spin_unlock_irqrestore(&p4_timer_lock, flags);
	return 0;
}

static irqreturn_t p4_timer_interrupt(int irq, void *dev_id)
{
	struct p4_timer_event *event = dev_id;
	unsigned int hart = event->hart;
	unsigned long flags;
	u32 val;

	raw_spin_lock_irqsave(&p4_timer_lock, flags);
	if (!(readl(p4_timer + P4_INT_ST) & BIT(hart))) {
		raw_spin_unlock_irqrestore(&p4_timer_lock, flags);
		return IRQ_NONE;
	}
	val = readl(p4_timer + P4_CONF);
	writel(val & ~P4_TARGET_EN(hart), p4_timer + P4_CONF);
	writel(BIT(hart), p4_timer + P4_INT_CLR);
	raw_spin_unlock_irqrestore(&p4_timer_lock, flags);
	event->ce.event_handler(&event->ce);
	return IRQ_HANDLED;
}

static int p4_timer_starting_cpu(unsigned int cpu)
{
	unsigned int hart = cpuid_to_hartid_map(cpu);
	struct p4_timer_event *event;

	if (hart > 1)
		return -EINVAL;
	event = &p4_events[hart];
	event->ce.cpumask = cpumask_of(cpu);
	clockevents_config_and_register(&event->ce, P4_TIMER_RATE, 100, ULONG_MAX);
	enable_irq(event->irq);
	return 0;
}

static int __init p4_timer_init(struct device_node *node)
{
	void __iomem *clkrst;
	unsigned int cpu, hart;
	struct p4_timer_event *event;
	unsigned long flags;
	u32 val;
	int ret;

	p4_timer = of_iomap(node, 0);
	clkrst = of_iomap(node, 1);
	if (!p4_timer || !clkrst)
		return -ENOMEM;
	/*
	 * +0x1c BIT23 is the SYSTIMER bus gate on BOTH P4 revisions.
	 * +0x30 BIT23 is a PSRAM divider: never write it here.
	 */
	val = readl(clkrst + P4_SOC_CLK_CTRL2);
	writel(val | P4_APB_ENABLE, clkrst + P4_SOC_CLK_CTRL2);
	val = readl(clkrst + P4_PERI_CLK_CTRL21);
	writel((val | P4_CLK_ENABLE) & ~P4_CLK_SOURCE_RC_FAST,
	       clkrst + P4_PERI_CLK_CTRL21);
	readl(clkrst + P4_PERI_CLK_CTRL21);
	iounmap(clkrst);

	writel(0, p4_timer + P4_INT_ENA);
	writel(GENMASK(2, 0), p4_timer + P4_INT_CLR);
	val = readl(p4_timer + P4_CONF);
	val &= ~(GENMASK(24, 22) | BIT(28) | BIT(27));
	val |= BIT(31) | BIT(30); /* Register clock and unit0, no CPU stall. */
	writel(val, p4_timer + P4_CONF);
	for_each_possible_cpu(cpu) {
		hart = cpuid_to_hartid_map(cpu);
		if (hart > 1)
			return -EINVAL;
		event = &p4_events[hart];
		event->hart = hart;
		event->ce.name = "esp32p4-clockevent";
		event->ce.rating = 300;
		event->ce.features = CLOCK_EVT_FEAT_ONESHOT;
		event->ce.set_next_event = p4_event_next;
		event->ce.set_state_shutdown = p4_event_shutdown;
		event->ce.set_state_oneshot = p4_event_shutdown;
		event->ce.tick_resume = p4_event_shutdown;
		event->irq = irq_of_parse_and_map(node, hart);
		if (!event->irq)
			return -EINVAL;
		ret = irq_force_affinity(event->irq, cpumask_of(cpu));
		if (ret)
			return ret;
		ret = request_irq(event->irq, p4_timer_interrupt,
				  IRQF_TIMER | IRQF_NOBALANCING | IRQF_NO_AUTOEN,
				  event->ce.name, event);
		if (ret)
			return ret;
	}

	raw_spin_lock_irqsave(&p4_timer_lock, flags);
	p4_read_counter_locked();
	raw_spin_unlock_irqrestore(&p4_timer_lock, flags);
	ret = clocksource_register_hz(&p4_clocksource, P4_TIMER_RATE);
	if (ret)
		return ret;
	sched_clock_register(p4_read_counter, 52, P4_TIMER_RATE);
	ret = cpuhp_setup_state(CPUHP_AP_RISCV_TIMER_STARTING,
				"clockevents/esp32p4:starting",
				p4_timer_starting_cpu, NULL);
	if (ret)
		return ret;
	pr_info("16 MHz unit0 clocksource, two per-hart one-shot comparators\n");
	return 0;
}
TIMER_OF_DECLARE(esp32p4, "espressif,esp32p4-systimer", p4_timer_init);
