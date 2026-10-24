/* SPDX-License-Identifier: GPL-2.0-only */
/* Task-owned FP protocol. Radio cache-risk fence policy is retained. */
#include <linux/panic.h>
#include <linux/export.h>
#include <linux/errno.h>
#include <linux/sched/task_stack.h>
#include <linux/interrupt.h>
#include <linux/percpu.h>
#include <linux/preempt.h>
#include <linux/sched.h>
#include <linux/sched/cputime.h>
#include <linux/kernel_stat.h>
#include <linux/slab.h>
#include <linux/task_work.h>
#include <linux/cpumask.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <asm/switch_to.h>
#include <asm/esp32s31_radio_view.h>
#include <asm/csr.h>
#include <asm/ptrace.h>
#include <asm/page.h>
#include <asm/thread_info.h>
#include <asm/esp32s31_blob_fp.h>
#include <asm/esp32s31_radio_task.h>
#include <asm/esp32s31_ext.h>
#include "s31-blob-fp-protocol.h"

extern void s31_fp_bank_save_rv32(struct s31_fp_bank *);
extern void s31_fp_bank_restore_rv32(const struct s31_fp_bank *);
static DEFINE_PER_CPU(struct s31_fp_cpu, s31_fp_cpu_state);
/* These void interfaces cannot reject use and let the caller keep executing
 * FP or switching tasks. Invariant failures stop the kernel before clobber.
 */
static void require_transition(int error, const char *transition)
{
    if (error)
        panic("s31 blob FP: %s rejected (%d)", transition, error);
}
static void bank_save(void *unused, struct s31_fp_bank *bank)
{ s31_fp_bank_save_rv32(bank); }
static void bank_restore(void *unused, const struct s31_fp_bank *bank)
{ s31_fp_bank_restore_rv32(bank); }
static void bank_fs(void *unused, unsigned int enabled)
{
    if (enabled) csr_set(CSR_SSTATUS, SR_FS);
    else csr_clear(CSR_SSTATUS, SR_FS);
}
static const struct s31_fp_ops fp_ops = {bank_save, bank_restore, bank_fs, NULL};

/* Only before first wake: the caller owns a dormant PF_KTHREAD task.
 * A globally mapped kzalloc object must never be replaced with a SRAM cookie.
 */
int esp32s31_blob_fp_attach(struct task_struct *task)
{
    struct s31_fp_state *state;
    if (!task || task == current || !(task->flags & PF_KTHREAD) ||
        task_thread_info(task)->s31_blob_fp || task_thread_info(task)->s31_radio_context ||
        (task_pt_regs(task)->status & SR_FS) != SR_FS_OFF)
        return -EINVAL;
    state = kzalloc(sizeof(*state), GFP_KERNEL);
    if (!state) return -ENOMEM;
    state->valid = 1; /* Initial file and fcsr are all zero. */
    task_thread_info(task)->s31_blob_fp = state;
    return 0;
}
EXPORT_SYMBOL_GPL(esp32s31_blob_fp_attach);

/* The globally mapped wrapper is the canonical admission record. Driver
 * data may live in low SRAM; only the driver, while it owns init_mm, may
 * dereference it. Switch hooks use the FP member alone. */
struct s31_radio_domain_task;
struct s31_radio_task_context {
    struct s31_fp_state fp;
    const void *owner;
    void *driver_data;
    struct s31_radio_domain_task *domain_task;
};

#include "s31-radio-domain.inc"

int esp32s31_radio_task_attach(struct task_struct *task, const void *owner, void *data,
                               bool persistent_fp)
{
    struct s31_radio_task_context *context;

    if (!owner || !data || !task || task == current || !(task->flags & PF_KTHREAD) ||
        task_thread_info(task)->s31_blob_fp || task_thread_info(task)->s31_radio_context ||
        (task_pt_regs(task)->status & SR_FS) != SR_FS_OFF)
        return -EINVAL;
    context = kzalloc(sizeof(*context), GFP_KERNEL);
    if (!context) return -ENOMEM;
    context->fp.valid = 1;
    context->fp.radio = 1;
    context->fp.native = persistent_fp;
    context->owner = owner;
    context->driver_data = data;
    WRITE_ONCE(task_thread_info(task)->s31_radio_context, context);
    WRITE_ONCE(task_thread_info(task)->s31_blob_fp, &context->fp);
    return 0;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_task_attach);

void *esp32s31_radio_task_current_data(const void *owner)
{
    struct s31_radio_task_context *context;

    if (!owner || !(current->flags & PF_KTHREAD)) return NULL;
    context = READ_ONCE(current_thread_info()->s31_radio_context);
    if (!context) return NULL;
    return context->owner == owner ? context->driver_data : NULL;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_task_current_data);

/* Only the owning current task may detach, before returning from threadfn.
 * External detach after join is not supported: the task may already be freed.
 * A refusal leaves state intact; callers must abort cleanup, never ignore it.
 */
int esp32s31_blob_fp_detach(struct task_struct *task)
{
    struct s31_fp_state *state;
    struct s31_fp_cpu *cpu;
    struct s31_radio_task_context *context;
    unsigned long flags;
    int error = 0;
    if (task != current || in_interrupt())
        return -EINVAL;
    preempt_disable();
    local_irq_save(flags);
    state = current_thread_info()->s31_blob_fp;
    context = current_thread_info()->s31_radio_context;
    cpu = this_cpu_ptr(&s31_fp_cpu_state);
    if ((context && &context->fp != state) || (state && state->radio && !context))
        error = -EINVAL;
    if (!error && state && state->native)
        error = s31_fp_finish(cpu, state, &fp_ops);
    if (!error && state && (state->process || state->active || cpu->owner ||
                  cpu->borrower || cpu->kernel_borrow ||
                  cpu->foreign_active))
        error = -EBUSY;
    if (!error) {
        current_thread_info()->s31_blob_fp = NULL;
        if (context) {

            WRITE_ONCE(context->driver_data, NULL);
            if (context->domain_task)
                s31_radio_domain_fp_detached(context);
            else
                WRITE_ONCE(current_thread_info()->s31_radio_context, NULL);
        }
    }
    local_irq_restore(flags);
    preempt_enable();
    if (!error) {
        if (context) {
            if (!context->domain_task) kfree(context);
        }
        else
            kfree(state);
    }
    return error;
}
EXPORT_SYMBOL_GPL(esp32s31_blob_fp_detach);

/* Logical payload ownership survives a hardware checkpoint at pre_switch.
 * No low SRAM cookie, driver object or borrowed task stack is followed. The
 * scheduler owns the task/context lifetime. Otherwise query only current;
 * pinning a foreign task_struct alone cannot prevent owner-side detachment. */
bool esp32s31_blob_fp_radio_payload_active(struct task_struct *task)
{
    struct s31_fp_state *state = READ_ONCE(task_thread_info(task)->s31_blob_fp);
    return state && state->radio && state->native && state->active && state->payload;
}
EXPORT_SYMBOL_GPL(esp32s31_blob_fp_radio_payload_active);

void esp32s31_blob_fp_pre_switch(struct task_struct *prev)
{
    /* switch_mm has ALREADY happened. Only task/thread_info and global state
     * can be touched. No kthread_data(), stack/cookie/driver task dereference. */
    s31_radio_errno_pre_switch(prev);
    if ((prev->flags & PF_KTHREAD) && task_thread_info(prev)->s31_blob_fp &&
        (task_pt_regs(prev)->status & SR_FS) != SR_FS_OFF)
        panic("s31 blob FP: attached kthread acquired a user FP bank");
    require_transition(s31_fp_pre_switch(this_cpu_ptr(&s31_fp_cpu_state),
                                   task_thread_info(prev)->s31_blob_fp, &fp_ops), "pre-switch");
}
void esp32s31_blob_fp_post_switch(struct task_struct *next)
{
    /* current/tp is still prev; use explicit next. Do this after every generic
     * FPU/vector/ESP clobber and before __switch_to installs the next integer SP.
     */
    require_transition(s31_fp_post_switch(this_cpu_ptr(&s31_fp_cpu_state),
                                    task_thread_info(next)->s31_blob_fp, &fp_ops), "post-switch");
    s31_radio_errno_post_switch(next);
}
void esp32s31_blob_fp_enter(void)
{
    unsigned long flags;
    preempt_disable(); local_irq_save(flags);
    require_transition(s31_fp_enter(this_cpu_ptr(&s31_fp_cpu_state),
                              current_thread_info()->s31_blob_fp, &fp_ops), "enter");
    local_irq_restore(flags); preempt_enable();
}
EXPORT_SYMBOL_GPL(esp32s31_blob_fp_enter);
void esp32s31_blob_fp_pause(void)
{
    unsigned long flags;
    preempt_disable(); local_irq_save(flags);
    require_transition(s31_fp_pause(this_cpu_ptr(&s31_fp_cpu_state),
                              current_thread_info()->s31_blob_fp, &fp_ops), "pause");
    local_irq_restore(flags); preempt_enable();
}
EXPORT_SYMBOL_GPL(esp32s31_blob_fp_pause);
void esp32s31_blob_fp_kernel_begin(void)
{
    unsigned long flags;
    /* Called AFTER generic kernel_fpu_begin's preempt_disable, BEFORE its
     * generic fstate_save. Phase 1 requires process context and cache on.
     * Invalid context/ownership is rejected before generic FP clobber.
     */
    if (in_interrupt())
        panic("s31 blob FP: generic kernel FPU in interrupt context");
    local_irq_save(flags);
    if ((current->flags & PF_KTHREAD) && current_thread_info()->s31_blob_fp &&
        (task_pt_regs(current)->status & SR_FS) != SR_FS_OFF)
        panic("s31 blob FP: kernel borrow found a user FP bank");
    require_transition(s31_fp_kernel_begin(this_cpu_ptr(&s31_fp_cpu_state), &fp_ops), "kernel-begin");
    local_irq_restore(flags);
}
void esp32s31_blob_fp_kernel_end_check(void)
{
    unsigned long flags;
    /* Reject an unmatched/foreign/nested end BEFORE generic restore. */
    if (in_interrupt())
        panic("s31 blob FP: generic kernel FPU end in interrupt context");
    local_irq_save(flags);
    require_transition(s31_fp_kernel_end_check(this_cpu_ptr(&s31_fp_cpu_state)),
                       "kernel-end-admission");
    local_irq_restore(flags);
}
void esp32s31_blob_fp_kernel_end(void)
{
    unsigned long flags;
    /* AFTER generic kernel_fpu_end's fstate_restore, BEFORE preempt_enable. */
    local_irq_save(flags);
    require_transition(s31_fp_kernel_end(this_cpu_ptr(&s31_fp_cpu_state), &fp_ops), "kernel-end");
    local_irq_restore(flags);
}

/* These admission transitions are cache-on bridge operations. They must not
 * be called inside a cache-disabled closed payload span. A retained outer
 * foreign preemption fence/migration pin owns the whole installed window.
 */
void esp32s31_blob_fp_foreign_enter(void)
{
    unsigned long flags;
    /* A retained outer foreign preemption fence/migration pin owns the whole
     * installed window. On kernels with preemption counting the fence is a
     * real preempt_disable; on voluntary kernels preempt_disable() is a
     * barrier and preempt_count() is always zero in process context, so the
     * enforceable contract there is the migration pin (which owns the
     * per-CPU installation) plus never-in-IRQ. Preemption safety itself for
     * BT windows comes from the per-CPU FP owner/switch-hook protocol, not
     * from this count. */
#ifdef CONFIG_PREEMPT_COUNT
    if (!preempt_count() || in_interrupt())
        panic("s31 blob FP: foreign enter outside its process fence");
#else
    if (in_interrupt() || !current->migration_disabled)
        panic("s31 blob FP: foreign enter outside its process fence");
#endif
    preempt_disable(); local_irq_save(flags);
    require_transition(s31_fp_foreign_enter(this_cpu_ptr(&s31_fp_cpu_state)),
                       "foreign-enter");
    local_irq_restore(flags); preempt_enable();
}
EXPORT_SYMBOL_GPL(esp32s31_blob_fp_foreign_enter);
void esp32s31_blob_fp_foreign_leave(void)
{
    unsigned long flags;
    /* Same config-aware contract as foreign_enter (see above). */
#ifdef CONFIG_PREEMPT_COUNT
    if (!preempt_count() || in_interrupt())
        panic("s31 blob FP: foreign leave outside its process fence");
#else
    if (in_interrupt() || !current->migration_disabled)
        panic("s31 blob FP: foreign leave outside its process fence");
#endif
    preempt_disable(); local_irq_save(flags);
    require_transition(s31_fp_foreign_leave(this_cpu_ptr(&s31_fp_cpu_state)),
                       "foreign-leave");
    local_irq_restore(flags); preempt_enable();
}
EXPORT_SYMBOL_GPL(esp32s31_blob_fp_foreign_leave);

/* Per-syscall process ownership. This admits FP ownership only, not low
 * radio mappings or opaque entry. Caller must end on the same Linux stack
 * before returning to userspace. No init_mm borrowing or user-frame zeroing. */
struct s31_process_fp {
    struct s31_fp_state state;
    struct task_exit_work exit;
    unsigned long user_fs;
    bool retired;
};

static void s31_process_fp_exit(struct task_exit_work *work)
{
    struct s31_process_fp *process = container_of(work, struct s31_process_fp, exit);

    if (!READ_ONCE(process->retired))
        panic("s31 blob FP: process exited before user-bank retirement");
}

bool esp32s31_blob_fp_user_active(struct task_struct *task)
{
    struct s31_fp_state *state = READ_ONCE(task_thread_info(task)->s31_blob_fp);

    return state && READ_ONCE(state->process) && READ_ONCE(state->active);
}

void esp32s31_blob_fp_user_return_check(void)
{
    struct s31_fp_state *state = READ_ONCE(current_thread_info()->s31_blob_fp);

    if (state && READ_ONCE(state->process))
        panic("s31 blob FP: process returned before user-bank retirement");
}

int esp32s31_blob_fp_process_begin(void)
{
    struct s31_process_fp *process;
    struct s31_fp_cpu *cpu;
    struct pt_regs *regs = task_pt_regs(current);
    unsigned long flags;
    int ret = 0;

    if ((current->flags & PF_KTHREAD) || !current->mm || in_interrupt() ||
        in_atomic() || irqs_disabled())
        return -EWOULDBLOCK;
    if (raw_smp_processor_id() != 0 ||
        !cpumask_equal(&current->cpus_mask, cpumask_of(0)))
        return -EXDEV;
    if (READ_ONCE(current_thread_info()->s31_blob_fp))
        return -EBUSY;
    process = kzalloc(sizeof(*process), GFP_KERNEL);
    if (!process)
        return -ENOMEM;
    ret = task_exit_work_add(current, &process->exit, s31_process_fp_exit);
    if (ret) { kfree(process); return ret; }
    /* A migrate pin survives active scheduling and paused bridge sleeps.
     * No process-affinity restoration or low mapping is hidden in this API. */
    migrate_disable();
    preempt_disable(); local_irq_save(flags);
    cpu = this_cpu_ptr(&s31_fp_cpu_state);
    if (cpu->owner || cpu->borrower || cpu->kernel_borrow || cpu->foreign_active)
        ret = -EBUSY;
    if (!ret) {
        process->user_fs = regs->status & SR_FS;
        fstate_save(current, regs); /* Dirty user file -> its native bank. */
        process->state.valid = 1; /* Blob starts with zero file/fcsr. */
        process->state.process = 1;
        WRITE_ONCE(current_thread_info()->s31_blob_fp, &process->state);
    } else {
        WRITE_ONCE(process->retired, true);
    }
    local_irq_restore(flags); preempt_enable();
    if (ret) {
        if (task_exit_work_finish_current(&process->exit))
            panic("s31 blob FP: refused process monitor did not finish");
        migrate_enable();
        kfree(process);
    }
    return ret;
}
EXPORT_SYMBOL_GPL(esp32s31_blob_fp_process_begin);

int esp32s31_blob_fp_process_end(void)
{
    struct s31_fp_state *state;
    struct s31_process_fp *process;
    struct s31_fp_cpu *cpu;
    struct pt_regs *regs = task_pt_regs(current);
    unsigned long flags;
    int ret = 0;

    if ((current->flags & PF_KTHREAD) || in_interrupt() || in_atomic() || irqs_disabled())
        return -EWOULDBLOCK;
    preempt_disable(); local_irq_save(flags);
    state = current_thread_info()->s31_blob_fp;
    cpu = this_cpu_ptr(&s31_fp_cpu_state);
    if (!state || !state->process)
        ret = -EINVAL;
    else if (esp32s31_radio_mm_active(current) || state->active || cpu->owner || cpu->borrower ||
             cpu->kernel_borrow || cpu->foreign_active)
        ret = -EBUSY;
    if (ret) { local_irq_restore(flags); preempt_enable(); return ret; }
    process = container_of(state, struct s31_process_fp, state);
    fstate_restore(current, regs); /* Native user bank, never the blob bank. */
    regs->status = (regs->status & ~SR_FS) | process->user_fs;
    WRITE_ONCE(current_thread_info()->s31_blob_fp, NULL);
    WRITE_ONCE(process->retired, true);
    local_irq_restore(flags); preempt_enable();
    if (task_exit_work_finish_current(&process->exit))
        panic("s31 blob FP: retired process monitor did not finish");
    migrate_enable();
    kfree(process);
    return 0;
}
EXPORT_SYMBOL_GPL(esp32s31_blob_fp_process_end);
