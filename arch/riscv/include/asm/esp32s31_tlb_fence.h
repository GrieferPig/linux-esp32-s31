/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ASM_ESP32S31_TLB_FENCE_H
#define _ASM_ESP32S31_TLB_FENCE_H
#include <linux/types.h>
struct cpumask;
#if defined(CONFIG_ESP32S31_CLIC) && defined(CONFIG_SMP)
void esp32s31_tlb_flush_poll(void);
bool esp32s31_tlb_flush_remote(const struct cpumask *mask);
#else
static inline void esp32s31_tlb_flush_poll(void) { }
static inline bool esp32s31_tlb_flush_remote(const struct cpumask *mask)
{
 return false;
}
#endif
#endif
