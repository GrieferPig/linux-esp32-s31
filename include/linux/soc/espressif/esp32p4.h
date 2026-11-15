/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_SOC_ESPRESSIF_ESP32P4_H
#define _LINUX_SOC_ESPRESSIF_ESP32P4_H
#include <linux/types.h>

#define ESP32P4_RAM_START 0x48000000U
#define ESP32P4_RAM_END   0x49000000U
#define ESP32P4_FB_START  0x49000000U
#define ESP32P4_FB_END    0x491c2000U
#define ESP32P4_CACHE_CHUNK 0x4000U

/* Fixed retained-SRAM window of the audited IDF handoff. */
static inline bool esp32p4_retained_entry_valid(u32 entry)
{
	return entry >= 0x4ff00000U && entry < 0x4ff40000U && !(entry & 1);
}

extern u32 esp32p4_clic_rev1;
void esp32p4_cache_sync_for_exec(unsigned long start, unsigned long end);
void esp32p4_cache_sync_all_for_exec(void);
int esp32p4_display_cache_writeback(u32 start, u32 size);
#endif
