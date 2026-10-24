/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ASM_RISCV_ESP32S31_EXT_H
#define _ASM_RISCV_ESP32S31_EXT_H

#ifndef __ASSEMBLY__

#include <linux/types.h>

struct task_struct;
struct pt_regs;

bool esp32s31_current_uses_init_mm(void);
bool esp32s31_executable_address(unsigned long addr);
/* Pinned task; returns Linux top-donor priority, or MAX_PRIO without a donor. */
int esp32s31_task_pi_priority(struct task_struct *task);
void esp32s31_kthread_use_init_mm(void);
void esp32s31_kthread_unuse_init_mm(void);
int esp32s31_sched_setscheduler(struct task_struct *task, int policy,
			       int priority);

/* Configure once after kthread_bind(0), before the creator's first wake.
 * This is not an active-task policy setter and admits no fork inheritance.
 * Caller owns the dormant task through this operation and stops it on error.
 */
int esp32s31_radio_set_rr_interval(struct task_struct *task, unsigned int ms);
/* Separate from the RR interval: also supports a private FIFO IRQ executor.
 * Creator owns an unpublished fixed-CPU0 kthread; role lasts until exit.
 * No live toggles or inherited privilege. Must run on a legal Linux stack. */
int esp32s31_radio_sched_admit(struct task_struct *task);

/* Must match OpenSBI's esp32s31_coproc.S and the signal UAPI layout. */
struct esp32s31_ext_state {
	u32 hwloop[6];
	u32 hwloop_state;
	u32 pie_state;
	u32 pie[54];
} __aligned(16);

#ifdef CONFIG_ESP32S31_COPROC_CONTEXT
void esp32s31_ext_cpu_init(void);
void esp32s31_ext_switch(struct task_struct *prev, struct task_struct *next);
/* Blob-FP switch hook, owned by the radio driver (set while its runtime
 * is up, cleared on shutdown).  Declared unconditionally so driver code
 * needs no CONFIG guard; defined with the S31 extension core. */
extern void (*s31_blob_fp_switch_hook)(struct task_struct *prev,
				       struct task_struct *next);
void esp32s31_ext_enter_kernel(struct pt_regs *regs);
void esp32s31_ext_exit_user(struct pt_regs *regs);
bool esp32s31_ext_first_use(struct pt_regs *regs);
void esp32s31_ext_save(struct task_struct *task);
void esp32s31_ext_restore(struct task_struct *task);
void esp32s31_ext_reset(struct task_struct *task);
void esp32s31_ext_kernel_begin(void);
void esp32s31_ext_kernel_end(void);
#else
static inline void esp32s31_ext_cpu_init(void) { }
static inline void esp32s31_ext_switch(struct task_struct *prev,
				       struct task_struct *next) { }
static inline void esp32s31_ext_enter_kernel(struct pt_regs *regs) { }
static inline void esp32s31_ext_exit_user(struct pt_regs *regs) { }
static inline bool esp32s31_ext_first_use(struct pt_regs *regs) { return false; }
static inline void esp32s31_ext_save(struct task_struct *task) { }
static inline void esp32s31_ext_restore(struct task_struct *task) { }
static inline void esp32s31_ext_reset(struct task_struct *task) { }
static inline void esp32s31_ext_kernel_begin(void) { }
static inline void esp32s31_ext_kernel_end(void) { }
#endif

#endif /* !__ASSEMBLY__ */

#endif
