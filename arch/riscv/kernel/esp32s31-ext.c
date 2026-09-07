// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 HWLoop and PIE task context management through OpenSBI. */

#include <linux/bug.h>
#include <linux/kallsyms.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/sched/cputime.h>
#include <linux/string.h>

#include <asm/esp32s31_ext.h>
#include <asm/page.h>
#include <asm/processor.h>
#include <asm/sbi.h>

#define S31_SBI_EXT_COPROC	0x09000002
#define S31_SBI_COPROC_SWITCH	0
#define S31_SBI_COPROC_SAVE	1
#define S31_SBI_COPROC_RESTORE	2
#define S31_SBI_COPROC_SUSPEND	3

static_assert(sizeof(struct esp32s31_ext_state) == 256);
static_assert(offsetof(struct esp32s31_ext_state, hwloop_state) == 0x18);
static_assert(offsetof(struct esp32s31_ext_state, pie_state) == 0x1c);
static_assert(offsetof(struct esp32s31_ext_state, pie) == 0x20);

bool esp32s31_current_uses_init_mm(void)
{
	return current->active_mm == &init_mm;
}
EXPORT_SYMBOL_GPL(esp32s31_current_uses_init_mm);

bool esp32s31_executable_address(unsigned long addr)
{
	return core_kernel_text(addr) || addr >= PAGE_OFFSET;
}
EXPORT_SYMBOL_GPL(esp32s31_executable_address);

u64 esp32s31_task_runtime(struct task_struct *task)
{
	return task_sched_runtime(task);
}
EXPORT_SYMBOL_GPL(esp32s31_task_runtime);

void esp32s31_kthread_use_init_mm(void)
{
	kthread_use_mm(&init_mm);
}
EXPORT_SYMBOL_GPL(esp32s31_kthread_use_init_mm);

void esp32s31_kthread_unuse_init_mm(void)
{
	kthread_unuse_mm(&init_mm);
}
EXPORT_SYMBOL_GPL(esp32s31_kthread_unuse_init_mm);

int esp32s31_sched_setscheduler(struct task_struct *task, int policy,
			       int priority)
{
	struct sched_param param = { .sched_priority = priority };

	return sched_setscheduler_nocheck(task, policy, &param);
}
EXPORT_SYMBOL_GPL(esp32s31_sched_setscheduler);

static unsigned long s31_ext_pa(struct task_struct *task)
{
	return __pa(&task->thread.esp32s31_ext);
}

static void s31_ext_check(struct sbiret ret)
{
	WARN_ONCE(ret.error, "ESP32-S31 coprocessor SBI failed: %ld\n",
		  ret.error);
}

static void esp32s31_ext_suspend_hw(struct task_struct *task, bool deactivate)
{
	struct sbiret ret;

	WARN_ON_ONCE(task != current);
	if (!task->thread.esp32s31_ext_active)
		return;

	ret = sbi_ecall(S31_SBI_EXT_COPROC, S31_SBI_COPROC_SUSPEND,
			s31_ext_pa(task), 0, 0, 0, 0, 0);
	s31_ext_check(ret);
	if (deactivate)
		task->thread.esp32s31_ext_active = false;
}

static void esp32s31_ext_restore_hw(struct task_struct *task)
{
	struct sbiret ret;

	ret = sbi_ecall(S31_SBI_EXT_COPROC, S31_SBI_COPROC_RESTORE,
			s31_ext_pa(task), 0, 0, 0, 0, 0);
	s31_ext_check(ret);
	task->thread.esp32s31_ext_active = true;
}

static void esp32s31_ext_switch_hw(struct task_struct *prev,
				   struct task_struct *next)
{
	struct sbiret ret;

	ret = sbi_ecall(S31_SBI_EXT_COPROC, S31_SBI_COPROC_SWITCH,
			s31_ext_pa(prev), s31_ext_pa(next), 0, 0, 0, 0);
	s31_ext_check(ret);
}

void esp32s31_ext_switch(struct task_struct *prev, struct task_struct *next)
{
	/*
	 * User contexts are suspended on trap entry and restored only at the
	 * final return-to-user boundary.  Kernel users such as the radio blob
	 * run ESP extensions in S-mode, so their state must survive involuntary
	 * preemption as part of the normal task switch.
	 */
	if (prev->thread.esp32s31_ext_active &&
	    next->thread.esp32s31_ext_active) {
		esp32s31_ext_switch_hw(prev, next);
		return;
	}
	if (prev->thread.esp32s31_ext_active)
		esp32s31_ext_suspend_hw(prev, false);
	if (next->thread.esp32s31_ext_active)
		esp32s31_ext_restore_hw(next);
}

void esp32s31_ext_save(struct task_struct *task)
{
	struct sbiret ret;

	/* Hardware state belongs to current; callers save current before copy. */
	WARN_ON_ONCE(task != current);
	if (!task->thread.esp32s31_ext_active)
		return;

	ret = sbi_ecall(S31_SBI_EXT_COPROC, S31_SBI_COPROC_SAVE,
			s31_ext_pa(task), 0, 0, 0, 0, 0);
	s31_ext_check(ret);
}

void esp32s31_ext_restore(struct task_struct *task)
{
	WARN_ON_ONCE(task != current);
	esp32s31_ext_restore_hw(task);
}

void esp32s31_ext_reset(struct task_struct *task)
{
	if (task == current)
		esp32s31_ext_suspend_hw(task, true);
	memset(&task->thread.esp32s31_ext, 0,
	       sizeof(task->thread.esp32s31_ext));
	task->thread.esp32s31_ext_active = false;
}

void esp32s31_ext_enter_kernel(struct pt_regs *regs)
{
	(void)regs;
	esp32s31_ext_suspend_hw(current, true);
}

void esp32s31_ext_exit_user(struct pt_regs *regs)
{
	(void)regs;
	esp32s31_ext_restore(current);
}

void esp32s31_ext_kernel_begin(void)
{
	esp32s31_ext_restore_hw(current);
}
EXPORT_SYMBOL_GPL(esp32s31_ext_kernel_begin);

void esp32s31_ext_kernel_end(void)
{
	esp32s31_ext_suspend_hw(current, true);
}
EXPORT_SYMBOL_GPL(esp32s31_ext_kernel_end);
