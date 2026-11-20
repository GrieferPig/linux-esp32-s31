/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_SOC_ESPRESSIF_ESP32P4_ANALOG_H
#define __LINUX_SOC_ESPRESSIF_ESP32P4_ANALOG_H

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/kconfig.h>

struct device;
struct regmap;

#if IS_REACHABLE(CONFIG_MFD_ESP32P4_ANALOG)
struct regmap *esp32p4_analog_get(struct device *consumer);
#else
static inline struct regmap *esp32p4_analog_get(struct device *consumer)
{
	return ERR_PTR(-EOPNOTSUPP);
}
#endif

#endif
