// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 HWLoop and PIE task context management through OpenSBI. */

#include <linux/jiffies.h>
#include <linux/bug.h>
#include <linux/err.h>
#include <linux/sched/prio.h>
#include <linux/kallsyms.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/sched/cputime.h>
#include <linux/sched/task_stack.h>
#include <linux/sched/s31_radio.h>
#include <linux/string.h>
#include <linux/percpu.h>
#include <linux/cpu.h>
#include <linux/uaccess.h>
#include <asm/ptrace.h>
#include <asm/timex.h>

#include <asm/esp32s31_ext.h>
#include <asm/page.h>
#include <asm/processor.h>
#include <asm/sbi.h>

#define S31_SBI_EXT_COPROC	0x09000002
#define S31_SBI_COPROC_SWITCH	0
#define S31_SBI_COPROC_SAVE	1
#define S31_SBI_COPROC_RESTORE	2
#define S31_SBI_COPROC_SUSPEND	3

void (*s31_blob_fp_switch_hook)(struct task_struct *prev,
				struct task_struct *next);
EXPORT_SYMBOL_GPL(s31_blob_fp_switch_hook);

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

int esp32s31_task_pi_priority(struct task_struct *task)
{
    int priority = MAX_PRIO;
#ifdef CONFIG_RT_MUTEXES
    struct task_struct *donor;
    unsigned long flags;

    raw_spin_lock_irqsave(&task->pi_lock, flags);
    /* pi_top_task is retained as a blocked donor until deboost clears it.
     * The caller pins task; do not follow the pointer after dropping pi_lock.
     */
    donor = task->pi_top_task;
    if (donor)
        priority = READ_ONCE(donor->prio);
    raw_spin_unlock_irqrestore(&task->pi_lock, flags);
#else
    (void)task;
#endif
    return priority;
}
EXPORT_SYMBOL_GPL(esp32s31_task_pi_priority);

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

int esp32s31_radio_set_rr_interval(struct task_struct *task, unsigned int ms)
{
#if defined(CONFIG_ESP32S31_COPROC_CONTEXT) && defined(CONFIG_FPU)
	unsigned int slice;

	if (in_interrupt() || in_atomic() || irqs_disabled())
		return -EWOULDBLOCK;
	if (!task || task == current || !ms || ms > 1000 ||
	    (task->flags & (PF_KTHREAD | PF_NO_SETAFFINITY)) !=
		(PF_KTHREAD | PF_NO_SETAFFINITY))
		return -EINVAL;
	if (!cpumask_equal(task->cpus_ptr, cpumask_of(0)))
		return -EXDEV;
	/* The creator has not published/woken this thread. Validate the dormant
	 * stop point; this check does not grant another caller ownership of it.
	 */
	if (!wait_task_inactive(task, TASK_UNINTERRUPTIBLE))
		return -EBUSY;
	if (READ_ONCE(task_thread_info(task)->s31_radio_rr_slice))
		return -EALREADY;
	slice = max_t(unsigned int, 1, msecs_to_jiffies(ms));
	WRITE_ONCE(task_thread_info(task)->s31_radio_rr_slice, slice);
	/* sched_fork starts every child with the generic slice. Install the
	 * admitted first slice here; the RT expiry path reloads the same value.
	 */
	task->rt.time_slice = slice;
	return 0;
#else
	return -EOPNOTSUPP;
#endif
}
EXPORT_SYMBOL_GPL(esp32s31_radio_set_rr_interval);

int esp32s31_radio_sched_admit(struct task_struct *task)
{
#ifdef CONFIG_ESP32S31_RADIO_SCHEDULER
	unsigned long witness;
	unsigned int role;

	if (in_interrupt() || in_atomic() || irqs_disabled())
		return -EWOULDBLOCK;
	if (!object_is_on_stack(&witness) || !task || task == current ||
	    (task->flags & (PF_KTHREAD | PF_NO_SETAFFINITY)) !=
		(PF_KTHREAD | PF_NO_SETAFFINITY))
		return -EINVAL;
	if (!cpumask_equal(task->cpus_ptr, cpumask_of(0)))
		return -EXDEV;
	/* Caller retains exclusive creator ownership. This stop-point check
	 * does not turn an arbitrary sleeping thread into a creator-owned one. */
	if (!wait_task_inactive(task, TASK_UNINTERRUPTIBLE))
		return -EBUSY;
	role = READ_ONCE(task_thread_info(task)->s31_radio_sched);
	if (role & S31_RADIO_SCHED_ADMITTED)
		return -EALREADY;
	if (role)
		return -EBUSY;
	WRITE_ONCE(task_thread_info(task)->s31_radio_sched,
		   S31_RADIO_SCHED_ADMITTED);
	return 0;
#else
	return -EOPNOTSUPP;
#endif
}
EXPORT_SYMBOL_GPL(esp32s31_radio_sched_admit);

static unsigned long s31_ext_pa(struct task_struct *task)
{
	return __pa(&task->thread.esp32s31_ext);
}

static void s31_ext_check(struct sbiret ret)
{
	WARN_ONCE(ret.error, "ESP32-S31 coprocessor SBI failed: %ld\n",
		  ret.error);
}

/* Firmware starts PIE enabled, including on non-retentive warmboot.
 * Discard that firmware-owned state once per CPU bring-up/resume. Keep the
 * suspended task's saved context untouched so user state can be restored.
 */
static DEFINE_PER_CPU_SHARED_ALIGNED(struct esp32s31_ext_state, s31_ext_boot_state);

void esp32s31_ext_cpu_init(void)
{
	unsigned long flags;
	struct sbiret ret;

	local_irq_save(flags);
	ret = sbi_ecall(S31_SBI_EXT_COPROC, S31_SBI_COPROC_SUSPEND,
			__pa(this_cpu_ptr(&s31_ext_boot_state)), 0, 0, 0, 0, 0);
	s31_ext_check(ret);
	local_irq_restore(flags);
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
	/* Preserve dirty state when switching the runtime control to lazy. */
	if (!ret.error && ((task->thread.esp32s31_ext.hwloop_state & 3) == 3 ||
			  (task->thread.esp32s31_ext.pie_state & 3) == 3))
		task->thread.esp32s31_ext_used = true;
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
	/* User extension contexts are suspended on trap entry and restored at
	 * the final return-to-user boundary. Actual non-radio kernel extension
	 * owners retain their normal switch handling. Radio execution does not
	 * use HWLoop/PIE and must never activate an extension context here.
	 */
	/* Radio fence invariant check.  Wi-Fi FP and foreign windows remain
	 * fenced; BT trap entry uses a task-owned Linux stack.  The hook must
	 * not dereference low SRAM after the scheduler has switched page tables.
	 * It is not an FP save/restore protocol for preemptible Wi-Fi blobs. */
	if (s31_blob_fp_switch_hook)
		s31_blob_fp_switch_hook(prev, next);
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
	task->thread.esp32s31_ext_used = false;
}

void esp32s31_ext_enter_kernel(struct pt_regs *regs)
{

	if (!current->thread.esp32s31_ext_active)
		return;

	esp32s31_ext_suspend_hw(current, true);

}

bool esp32s31_ext_first_use(struct pt_regs *regs)
{
	u16 insn;

	if (!true || !user_mode(regs) ||
	    current->thread.esp32s31_ext_used)
		return false;
	/* Vendor 32-bit extensions require 4-byte alignment in production
	 * (s31_ext_test.S, s31_pie_cases.inc with .p2align 2).  Board
	 * 20261008-a1: 4-aligned setupi at 0x10784 traps when HW OFF,
	 * first_use+1, returns 5 on CPU0; 2-misaligned setupi at 0x1e
	 * executes as silent NOP (returns 1, no trap, no first_use) when
	 * HW is OFF, but executes correctly once HW is pre-enabled
	 * (eager).  Lazy activation relies on the OFF-trap and therefore
	 * only supports 4-aligned vendor instructions.  Instructions may
	 * start on a two-byte boundary after RVC, so read the low half;
	 * a 32-bit vendor word always has low two bits 0b11, while
	 * compressed halves never match the vendor families below.
	 */
	if (get_user(insn, (u16 __user *)regs->epc))
		return false;
	/* Match the vendor families in the existing hart0 migration check.
	 * Retry once without advancing PC; ordinary SIGILL is unchanged.
	 */
	if ((insn & 0x7fU) != 0x2bU && (insn & 0x1bU) != 0x1bU)
		return false;
	current->thread.esp32s31_ext_used = true;

	return true;
}

void esp32s31_ext_exit_user(struct pt_regs *regs)
{

	if (true && !current->thread.esp32s31_ext_used) {

		return;
	}

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
