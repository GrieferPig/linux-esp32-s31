/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_SOC_ESPRESSIF_ESP32S31_LP_H
#define _LINUX_SOC_ESPRESSIF_ESP32S31_LP_H

#include <linux/soc/espressif/esp32s31-lp-protocol.h>

int esp32s31_lp_system_suspend_prepare(void);
int esp32s31_lp_system_suspend_validate(void);
void esp32s31_lp_system_suspend_wake(void);
void esp32s31_lp_system_suspend_finish(void);

#endif
