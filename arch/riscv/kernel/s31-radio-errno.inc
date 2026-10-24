/* SPDX-License-Identifier: GPL-2.0-only */
/* Picolibc's relocated cell is an ABI register, banked at real task switches.
 * The loader owns its globally mapped backing until domain destruction. */
int esp32s31_radio_domain_bind_errno(struct esp32s31_radio_domain *domain, int *slot)
{
    unsigned long flags;
    int ret = 0;

    if (!domain || !slot || ((unsigned long)slot & (sizeof(*slot) - 1)) ||
        !is_linear_mapping((unsigned long)slot) || !virt_addr_valid(slot))
        return -EINVAL;
    if (in_interrupt()) return -EWOULDBLOCK;
    raw_spin_lock_irqsave(&domain->lock, flags);
    if (raw_smp_processor_id() != 0) ret = -EXDEV;
    else if (domain->state.closing) ret = -ESHUTDOWN;
    else if (domain->errno_slot || domain->state.task_admitted || domain->creating)
        ret = -EBUSY;
    else
        WRITE_ONCE(domain->errno_slot, slot);
    raw_spin_unlock_irqrestore(&domain->lock, flags);
    return ret;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_domain_bind_errno);

bool esp32s31_radio_task_errno_native(const void *owner)
{
    struct s31_radio_task_context *context;
    struct s31_radio_domain_task *task;

    if (!owner || !(current->flags & PF_KTHREAD)) return false;
    context = READ_ONCE(current_thread_info()->s31_radio_context);
    if (!context || context->owner != owner) return false;
    task = context->domain_task;
    return task && READ_ONCE(task->errno_slot) && !READ_ONCE(task->fp_detached);
}
EXPORT_SYMBOL_GPL(esp32s31_radio_task_errno_native);

static struct s31_radio_domain_task *s31_radio_errno_task(struct task_struct *thread)
{
    struct s31_radio_task_context *context = task_thread_info(thread)->s31_radio_context;
    struct s31_radio_domain_task *task;

    if (!context) return NULL;
    task = context->domain_task;
    if (!task || !task->errno_slot || task->fp_detached) return NULL;
    /* These hooks run after switch_mm: no low SRAM, module callback,
     * allocation, sleeping, or caller-owned memory is touched here. */
    if (raw_smp_processor_id() != 0)
        panic("s31 radio errno: task escaped CPU0");
    return task;
}

static void s31_radio_errno_pre_switch(struct task_struct *prev)
{
    struct s31_radio_domain_task *task = s31_radio_errno_task(prev);
    if (task) WRITE_ONCE(task->errno_value, READ_ONCE(*task->errno_slot));
}

static void s31_radio_errno_post_switch(struct task_struct *next)
{
    struct s31_radio_domain_task *task = s31_radio_errno_task(next);
    if (task) WRITE_ONCE(*task->errno_slot, READ_ONCE(task->errno_value));
}
