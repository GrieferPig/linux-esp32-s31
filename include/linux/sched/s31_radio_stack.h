/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_SCHED_S31_RADIO_STACK_H
#define _LINUX_SCHED_S31_RADIO_STACK_H

#include <linux/types.h>

struct task_struct;
struct esp32s31_radio_stack;

#ifdef CONFIG_ESP32S31_RADIO_SMODE
/* The caller owns backing storage until destroy returns. A prepared lease
 * may be offered to one kthread creation only. Kernel RCU completion, rather
 * than a module callback, joins the last Linux task-stack reference. */
struct esp32s31_radio_stack *esp32s31_radio_stack_prepare(void *base, size_t size);
int esp32s31_radio_stack_destroy(struct esp32s31_radio_stack *stack);
bool esp32s31_radio_stack_released(struct esp32s31_radio_stack *stack);
unsigned long esp32s31_radio_stack_bytes(void);

/* Internal fork/stack hooks; no allocator or callback into a radio module. */
int esp32s31_radio_stack_install(struct task_struct *task);
bool esp32s31_radio_stack_release(struct task_struct *task);
bool esp32s31_radio_stack_owned(const struct task_struct *task);
#else
static inline bool esp32s31_radio_stack_owned(const struct task_struct *task)
{
	return false;
}
#endif
#endif
