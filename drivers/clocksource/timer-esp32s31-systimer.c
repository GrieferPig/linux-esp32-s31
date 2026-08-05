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
#include <asm/fixmap.h>

#define S31_SYSTIMER_PHYS		0x20399000U

#define S31_SYSTIMER_UNIT1_OP		0x08
#define S31_SYSTIMER_UNIT1_VALUE_HI	0x48
#define S31_SYSTIMER_UNIT1_VALUE_LO	0x4c
#define S31_SYSTIMER_UPDATE		BIT(30)
#define S31_SYSTIMER_VALUE_VALID	BIT(29)
#define S31_SYSTIMER_VALUE_HI_MASK	GENMASK(19, 0)
#define S31_SYSTIMER_RATE		16000000U

static void __iomem *s31_systimer_base;
static u64 s31_systimer_last;

static u64 notrace s31_systimer_read(struct clocksource *cs)
{
	u32 status, hi, lo;
	unsigned int retries;

	/* Snapshot counter 1, leaving FreeRTOS's counter 0 untouched. */
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
	s31_systimer_last = ((u64)hi << 32) | lo;
	return s31_systimer_last;
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
	u32 conf;

	/* timer_probe() runs before normal ioremap is available. */
	set_fixmap_io(FIX_S31_SYSTIMER, S31_SYSTIMER_PHYS);
	s31_systimer_base = (void __iomem *)fix_to_virt(FIX_S31_SYSTIMER);

	/* Keep counter 1 running when Linux owns the clocksource. */
	conf = readl(s31_systimer_base);
	writel(conf | BIT(31) | BIT(29), s31_systimer_base);

	ret = clocksource_register_hz(&s31_systimer_clocksource,
				     S31_SYSTIMER_RATE);
	if (ret) {
		pr_err("failed to register clocksource: %d\n", ret);
		return ret;
	}

	pr_info("registered 16 MHz always-on clocksource\n");
	return 0;
}

TIMER_OF_DECLARE(esp32s31_systimer, "espressif,esp32s31-systimer",
		 s31_systimer_init);
