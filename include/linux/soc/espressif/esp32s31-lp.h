/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_SOC_ESPRESSIF_ESP32S31_LP_H
#define _LINUX_SOC_ESPRESSIF_ESP32S31_LP_H

#include <linux/soc/espressif/esp32s31-lp-protocol.h>

int esp32s31_lp_sleep_prepare(struct s31_lp_sleep_control *control);
int esp32s31_lp_sleep_arm(struct s31_lp_sleep_control *control);
int esp32s31_lp_sleep_abort(struct s31_lp_sleep_control *control);
int esp32s31_lp_sleep_query(struct s31_lp_sleep_control *control);
int esp32s31_lp_sleep_reclaim(struct s31_lp_sleep_control *control);
bool esp32s31_lp_sleep_available(void);
int esp32s31_lp_system_suspend_prepare(void);
void esp32s31_lp_system_suspend_finish(void);

#endif
