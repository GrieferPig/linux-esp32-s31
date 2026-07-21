/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_SOC_ESPRESSIF_ESP32S31_CACHE_H
#define __LINUX_SOC_ESPRESSIF_ESP32S31_CACHE_H

#include <linux/types.h>

#ifdef CONFIG_ESP32S31_CACHE
void esp32s31_cache_sync_for_exec(phys_addr_t paddr, size_t size);
#else
static inline void esp32s31_cache_sync_for_exec(phys_addr_t paddr, size_t size)
{
}
#endif

#endif /* __LINUX_SOC_ESPRESSIF_ESP32S31_CACHE_H */
