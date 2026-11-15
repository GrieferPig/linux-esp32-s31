/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ASM_RISCV_ESP32P4_DEBUG_H
#define _ASM_RISCV_ESP32P4_DEBUG_H

#include <linux/types.h>

#ifdef CONFIG_SOC_ESP32P4
void esp32p4_backtrace_snapshot(bool after);
#else
static inline void esp32p4_backtrace_snapshot(bool after) { }
#endif

#endif
