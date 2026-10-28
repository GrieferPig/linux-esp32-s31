/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_SOC_ESPRESSIF_ESP32S31_GPIO_H
#define _LINUX_SOC_ESPRESSIF_ESP32S31_GPIO_H

#include <linux/kconfig.h>
#include <linux/types.h>

struct esp32s31_gpio_wake {
	u32 mask;
	/* Three hardware trigger bits per LP pad, pins 0..7. */
	u32 types;
	u32 pullup;
	u32 pulldown;
};

#if IS_ENABLED(CONFIG_PINCTRL_ESP32S31)
int esp32s31_gpio_get_lp_wake(struct esp32s31_gpio_wake *wake);
void esp32s31_gpio_replay_lp_wake(u32 mask);
#else
static inline int esp32s31_gpio_get_lp_wake(struct esp32s31_gpio_wake *wake)
{
	*wake = (struct esp32s31_gpio_wake){};
	return 0;
}
static inline void esp32s31_gpio_replay_lp_wake(u32 mask)
{
}
#endif

#endif
