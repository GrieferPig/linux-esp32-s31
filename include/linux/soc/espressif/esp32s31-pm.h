/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_SOC_ESPRESSIF_ESP32S31_PM_H
#define _LINUX_SOC_ESPRESSIF_ESP32S31_PM_H

#include <linux/types.h>

bool esp32s31_sbi_idle_enabled(void);
int esp32s31_sbi_idle_activate(void);
void esp32s31_sbi_idle_deactivate(void);
void esp32s31_sbi_wfi(void);

#endif
