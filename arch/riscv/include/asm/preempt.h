/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_RISCV_PREEMPT_H
#define _ASM_RISCV_PREEMPT_H

#include <asm-generic/preempt.h>

#if defined(CONFIG_ESP32S31_RADIO_SMODE) && defined(CONFIG_PREEMPTION)
/*
 * preempt_enable() may be reached by a BT critical-exit bridge while SP is
 * still in low SRAM.  The assembly wrapper pivots only anchored SRAM spans;
 * the generic scheduler and dynamic-preemption policy remain unchanged.
 */
extern asmlinkage void esp32s31_preempt_schedule(void);
extern asmlinkage void esp32s31_preempt_schedule_notrace(void);

#undef __preempt_schedule
#undef __preempt_schedule_notrace
#define __preempt_schedule() esp32s31_preempt_schedule()
#define __preempt_schedule_notrace() esp32s31_preempt_schedule_notrace()
#endif

#endif /* _ASM_RISCV_PREEMPT_H */
