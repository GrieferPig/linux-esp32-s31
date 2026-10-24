/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ASM_RISCV_ESP32S31_RADIO_VIEW_H
#define _ASM_RISCV_ESP32S31_RADIO_VIEW_H
#include <linux/types.h>
#include <linux/errno.h>
struct task_struct;
#ifdef CONFIG_ESP32S31_RADIO_SMODE
/* Trusted finite callback on native Linux SP, already process-FP owned
 * and strictly core0. Permanent supervisor mappings; no mmap-lock loan.
 * No user return/exec/exit or FP-ownership change inside the body. */
int esp32s31_radio_mm_call(int (*body)(void *), void *opaque);
bool esp32s31_radio_mm_active(struct task_struct *);
void esp32s31_radio_mm_user_return_check(void);
#else
static inline int esp32s31_radio_mm_call(int (*body)(void *), void *opaque) { return -EOPNOTSUPP; }
static inline bool esp32s31_radio_mm_active(struct task_struct *t) { return false; }
static inline void esp32s31_radio_mm_user_return_check(void) { }
#endif
#endif
