/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ASM_RISCV_ESP32S31_EXT_H
#define _ASM_RISCV_ESP32S31_EXT_H

#ifndef __ASSEMBLY__

#include <linux/types.h>

struct task_struct;

/* Must match OpenSBI's esp32s31_coproc.S and the signal UAPI layout. */
struct esp32s31_ext_state {
	u32 hwloop[6];
	u32 reserved[2];
	u32 pie[54];
} __aligned(16);

#ifdef CONFIG_ESP32S31_COPROC_CONTEXT
void esp32s31_ext_switch(struct task_struct *prev, struct task_struct *next);
void esp32s31_ext_save(struct task_struct *task);
void esp32s31_ext_restore(struct task_struct *task);
void esp32s31_ext_reset(struct task_struct *task);
#else
static inline void esp32s31_ext_switch(struct task_struct *prev,
				       struct task_struct *next) { }
static inline void esp32s31_ext_save(struct task_struct *task) { }
static inline void esp32s31_ext_restore(struct task_struct *task) { }
static inline void esp32s31_ext_reset(struct task_struct *task) { }
#endif

#endif /* !__ASSEMBLY__ */

#endif
