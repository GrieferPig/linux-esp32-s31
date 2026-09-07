// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 CLIC-aware idle through the platform OpenSBI extension. */

#include <linux/cpuidle.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/soc/espressif/esp32s31-pm.h>

static __cpuidle int esp32s31_cpuidle_enter(struct cpuidle_device *dev,
					    struct cpuidle_driver *drv,
					    int index)
{
	esp32s31_sbi_wfi();
	return index;
}

static struct cpuidle_driver esp32s31_cpuidle_driver = {
	.name = "esp32s31_cpuidle",
	.owner = THIS_MODULE,
	.states = {
		{
			.enter = esp32s31_cpuidle_enter,
			.exit_latency = 5,
			.target_residency = 50,
			.power_usage = UINT_MAX,
			.name = "SMP-WFI",
			.desc = "S31 bounded per-hart OpenSBI WFI",
		},
	},
	.safe_state_index = 0,
	.state_count = 1,
};

static int __init esp32s31_cpuidle_init(void)
{
	int ret;

	ret = esp32s31_sbi_idle_activate();
	if (ret)
		return ret;

	ret = cpuidle_register(&esp32s31_cpuidle_driver, NULL);
	if (ret)
		esp32s31_sbi_idle_deactivate();
	return ret;
}
device_initcall(esp32s31_cpuidle_init);
