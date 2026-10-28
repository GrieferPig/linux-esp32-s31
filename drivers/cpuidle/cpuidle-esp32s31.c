// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 CLIC-aware idle through the platform OpenSBI extension. */

#include <linux/cpu.h>
#include <linux/cpuidle.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/sched/idle.h>
#include <linux/soc/espressif/esp32s31-pm.h>

void arch_cpu_idle_poll(void);

static __cpuidle int esp32s31_cpuidle_poll(struct cpuidle_device *dev,
					struct cpuidle_driver *drv, int index)
{
	u64 start = local_clock_noinstr();
	unsigned int loops = 0;
	u64 limit = cpuidle_poll_time(drv, dev);

	dev->poll_time_limit = false;
	raw_local_irq_enable();
	if (!current_set_polling_and_test()) {
		while (!need_resched()) {
			/* Retain the architectural CLIC sentinel recovery while
			 * polling, including for TLB-shootdown and device interrupts.
			 */
			arch_cpu_idle_poll();
			cpu_relax();
			if (loops++ < 200)
				continue;
			loops = 0;
			if (local_clock_noinstr() - start > limit) {
				dev->poll_time_limit = true;
				break;
			}
		}
	}
	raw_local_irq_disable();
	current_clr_polling();
	return index;
}

static __cpuidle int esp32s31_cpuidle_enter(struct cpuidle_device *dev,
					    struct cpuidle_driver *drv,
					    int index)
{
	/* Use the same wake-edge IRQ drain as the architectural idle path.
	 * A CLIC cross-privilege sentinel can leave a source pending after
	 * firmware WFI returns, even when enabling SIE would not deliver it.
	 */
	arch_cpu_idle();
	return index;
}

static struct cpuidle_driver esp32s31_cpuidle_driver = {
	.name = "esp32s31_cpuidle",
	.owner = THIS_MODULE,
	.states = {
		{
			.enter = esp32s31_cpuidle_poll,
			.flags = CPUIDLE_FLAG_POLLING,
			.power_usage = UINT_MAX,
			.name = "POLL",
			.desc = "IRQ-enabled CLIC-aware polling",
		},
		{
			.enter = esp32s31_cpuidle_enter,
			.power_usage = UINT_MAX,
			.name = "SMP-WFI",
			.desc = "S31 bounded per-hart OpenSBI WFI",
		},
	},
	.safe_state_index = 0,
	.state_count = 2,
};

static int __init esp32s31_cpuidle_init(void)
{
	int ret;

	ret = esp32s31_sbi_idle_activate();
	if (ret)
		return ret;
	/* A blocked cross-privilege interrupt may need the firmware guard.
	 * Include a conservative 1 ms allowance for the SBI transition and
	 * wake-edge recovery; do not advertise a measured hardware exit time.
	 * Without a supply meter this is also a conservative residency choice,
	 * not an experimentally determined energy break-even point.
	 */
	esp32s31_cpuidle_driver.states[1].exit_latency =
		esp32s31_sbi_idle_guard_us() + 1000;
	esp32s31_cpuidle_driver.states[1].target_residency =
		esp32s31_cpuidle_driver.states[1].exit_latency;

	ret = cpuidle_register(&esp32s31_cpuidle_driver, NULL);
	if (ret)
		esp32s31_sbi_idle_deactivate();
	return ret;
}
device_initcall(esp32s31_cpuidle_init);
