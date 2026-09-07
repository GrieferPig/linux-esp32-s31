/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ASM_RISCV_ESP32S31_EXT_H
#define _ASM_RISCV_ESP32S31_EXT_H

#ifndef __ASSEMBLY__

#include <linux/types.h>

struct task_struct;
struct pt_regs;

bool esp32s31_current_uses_init_mm(void);
bool esp32s31_executable_address(unsigned long addr);
u64 esp32s31_task_runtime(struct task_struct *task);
void esp32s31_kthread_use_init_mm(void);
void esp32s31_kthread_unuse_init_mm(void);
int esp32s31_sched_setscheduler(struct task_struct *task, int policy,
			       int priority);

/* Must match OpenSBI's esp32s31_coproc.S and the signal UAPI layout. */
struct esp32s31_ext_state {
	u32 hwloop[6];
	u32 hwloop_state;
	u32 pie_state;
	u32 pie[54];
} __aligned(16);

#ifdef CONFIG_ESP32S31_COPROC_CONTEXT
void esp32s31_ext_switch(struct task_struct *prev, struct task_struct *next);
void esp32s31_ext_enter_kernel(struct pt_regs *regs);
void esp32s31_ext_exit_user(struct pt_regs *regs);
void esp32s31_ext_save(struct task_struct *task);
void esp32s31_ext_restore(struct task_struct *task);
void esp32s31_ext_reset(struct task_struct *task);
void esp32s31_ext_kernel_begin(void);
void esp32s31_ext_kernel_end(void);
#else
static inline void esp32s31_ext_switch(struct task_struct *prev,
				       struct task_struct *next) { }
static inline void esp32s31_ext_enter_kernel(struct pt_regs *regs) { }
static inline void esp32s31_ext_exit_user(struct pt_regs *regs) { }
static inline void esp32s31_ext_save(struct task_struct *task) { }
static inline void esp32s31_ext_restore(struct task_struct *task) { }
static inline void esp32s31_ext_reset(struct task_struct *task) { }
static inline void esp32s31_ext_kernel_begin(void) { }
static inline void esp32s31_ext_kernel_end(void) { }
#endif

#endif /* !__ASSEMBLY__ */

#endif
