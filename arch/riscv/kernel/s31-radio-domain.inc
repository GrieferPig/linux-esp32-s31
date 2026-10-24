/* SPDX-License-Identifier: GPL-2.0-only */
/* Included by the architecture FP adapter: canonical task metadata and its
 * lifetime remain in globally mapped kernel memory after FP detachment. */
#include "s31-radio-domain-core.h"
static_assert(ESP32S31_RADIO_DOMAIN_FOREIGN == S31_DOMAIN_FOREIGN);
static_assert(ESP32S31_RADIO_DOMAIN_IRQ == S31_DOMAIN_IRQ);

struct esp32s31_radio_domain {
    raw_spinlock_t lock;
    struct mutex reap_lock;
    struct list_head tasks;
    struct s31_domain_state state;
    bool persistent_fp;
    unsigned int creating;
    int *errno_slot;
};

struct s31_radio_domain_task {
    struct s31_radio_task_context context;
    struct task_exit_work exit;
    struct list_head node;
    struct task_struct *thread;
    struct esp32s31_radio_domain *domain;
    bool fp_detached, retired, checked;
    int *errno_slot;
    int errno_value;
};

#include "s31-radio-errno.inc"

static void s31_radio_domain_exit(struct task_exit_work *work)
{
    struct s31_radio_domain_task *task = container_of(work, struct s31_radio_domain_task, exit);
    struct esp32s31_radio_domain *domain = task->domain;
    unsigned long flags;

    if (task->thread != current || !READ_ONCE(task->retired) ||
        !READ_ONCE(task->fp_detached) || READ_ONCE(task->checked) ||
        current_thread_info()->s31_radio_context != &task->context ||
        current_thread_info()->s31_blob_fp || task->context.driver_data)
        panic("s31 radio domain: task exited without complete retirement");
    WRITE_ONCE(current_thread_info()->s31_radio_context, NULL);
    raw_spin_lock_irqsave(&domain->lock, flags);
    task->checked = true;
    raw_spin_unlock_irqrestore(&domain->lock, flags);
    /* Generic exit-work publishes done AFTER this kernel callback returns.
     * No module callback, destructor or memory release runs under this lock. */
}

/* reap_lock prevents a join waiter from observing an object another reaper
 * frees. No vendor callback or task exit takes this lifecycle mutex. */
static void s31_radio_domain_reap_locked(struct esp32s31_radio_domain *domain)
{
    struct s31_radio_domain_task *task, *next;
    unsigned long flags;
    LIST_HEAD(done);

    raw_spin_lock_irqsave(&domain->lock, flags);
    list_for_each_entry_safe(task, next, &domain->tasks, node) {
        if (!completion_done(&task->exit.done)) continue;
        if (!task->checked || !task->retired || !task->fp_detached)
            panic("s31 radio domain: completed task lacks retirement");
        list_move(&task->node, &done);
        if (s31_domain_put(&domain->state, S31_DOMAIN_TASK))
            panic("s31 radio domain: task reference underflow");
    }
    raw_spin_unlock_irqrestore(&domain->lock, flags);
    list_for_each_entry_safe(task, next, &done, node) {
        list_del(&task->node);
        put_task_struct(task->thread);
        kfree(task);
    }
}

static void s31_radio_domain_reap(struct esp32s31_radio_domain *domain)
{
    mutex_lock(&domain->reap_lock);
    s31_radio_domain_reap_locked(domain);
    mutex_unlock(&domain->reap_lock);
}

struct esp32s31_radio_domain *esp32s31_radio_domain_create(bool persistent_fp)
{
    struct esp32s31_radio_domain *domain;
    if (in_interrupt() || in_atomic() || irqs_disabled()) return ERR_PTR(-EWOULDBLOCK);
    domain = kzalloc(sizeof(*domain), GFP_KERNEL);
    if (!domain) return ERR_PTR(-ENOMEM);
    raw_spin_lock_init(&domain->lock);
    mutex_init(&domain->reap_lock);
    INIT_LIST_HEAD(&domain->tasks);
    domain->persistent_fp = persistent_fp;
    return domain;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_domain_create);

void esp32s31_radio_domain_close(struct esp32s31_radio_domain *domain)
{
    unsigned long flags;
    raw_spin_lock_irqsave(&domain->lock, flags);
    domain->state.closing = 1;
    raw_spin_unlock_irqrestore(&domain->lock, flags);
}
EXPORT_SYMBOL_GPL(esp32s31_radio_domain_close);

int esp32s31_radio_domain_get(struct esp32s31_radio_domain *domain, unsigned int kind)
{
    unsigned long flags;
    int ret;
    /* Task references belong to the monitored attach/reap protocol. */
    if (!domain || kind == S31_DOMAIN_TASK) return -EINVAL;
    raw_spin_lock_irqsave(&domain->lock, flags);
    ret = s31_domain_get(&domain->state, kind);
    raw_spin_unlock_irqrestore(&domain->lock, flags);
    return ret;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_domain_get);

int esp32s31_radio_domain_put(struct esp32s31_radio_domain *domain, unsigned int kind)
{
    unsigned long flags;
    int ret;
    if (!domain || kind == S31_DOMAIN_TASK) return -EINVAL;
    raw_spin_lock_irqsave(&domain->lock, flags);
    ret = s31_domain_put(&domain->state, kind);
    raw_spin_unlock_irqrestore(&domain->lock, flags);
    return ret;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_domain_put);

int esp32s31_radio_domain_destroy(struct esp32s31_radio_domain *domain)
{
    unsigned long flags;
    int ret;
    if (!domain || in_interrupt() || in_atomic() || irqs_disabled()) return -EINVAL;
    mutex_lock(&domain->reap_lock);
    s31_radio_domain_reap_locked(domain);
    raw_spin_lock_irqsave(&domain->lock, flags);
    ret = s31_domain_destroyable(&domain->state);
    if (!ret && (!list_empty(&domain->tasks) || domain->creating)) ret = -EBUSY;
    raw_spin_unlock_irqrestore(&domain->lock, flags);
    mutex_unlock(&domain->reap_lock);
    if (ret) return ret; /* A refusal preserves the domain and live owners. */

    kfree(domain);
    return 0;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_domain_destroy);

int esp32s31_radio_domain_join_tasks(struct esp32s31_radio_domain *domain)
{
    struct s31_radio_domain_task *task;
    unsigned long flags;
    int ret = 0;
    if (!domain || in_interrupt() || in_atomic() || irqs_disabled()) return -EINVAL;
    mutex_lock(&domain->reap_lock);
    raw_spin_lock_irqsave(&domain->lock, flags);
    if (!domain->state.closing) ret = -EINVAL;
    else if (domain->creating) ret = -EBUSY;
    raw_spin_unlock_irqrestore(&domain->lock, flags);
    if (ret) goto out;
    /* The caller has quiesced creators and stopped live task bodies. Also
     * join self-exited tasks no longer present in the driver's live list. */
    while (!list_empty(&domain->tasks)) {
        task = list_first_entry(&domain->tasks, struct s31_radio_domain_task, node);
        wait_for_completion(&task->exit.done);
        s31_radio_domain_reap_locked(domain);
    }
out:
    mutex_unlock(&domain->reap_lock);
    return ret;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_domain_join_tasks);

static int s31_radio_domain_task_attach(struct task_struct *thread,
                                       struct esp32s31_radio_domain *domain, void *data,
                                       bool own_current, int initial_errno)
{
    struct s31_radio_domain_task *task;
    unsigned long flags;
    int ret;

    if (in_interrupt() || in_atomic() || irqs_disabled()) return -EWOULDBLOCK;
    if (!domain || !data || !thread || (!own_current && thread == current) ||
        !(thread->flags & PF_KTHREAD) || task_thread_info(thread)->s31_blob_fp ||
        task_thread_info(thread)->s31_radio_context ||
        (task_pt_regs(thread)->status & SR_FS) != SR_FS_OFF)
        return -EINVAL;
    /* The creator binds before admission. Runtime user affinity changes
     * cannot move a kthread_bind task; kernel clients retain that policy. */
    if (!cpumask_equal(thread->cpus_ptr, cpumask_of(0))) return -EXDEV;
    task = kzalloc(sizeof(*task), GFP_KERNEL);
    if (!task) return -ENOMEM;
    task->context.fp.valid = 1;
    task->context.fp.radio = 1;
    task->context.fp.native = domain->persistent_fp;
    task->context.owner = domain;
    task->context.driver_data = data;
    task->context.domain_task = task;
    task->domain = domain;
    task->thread = thread;
    task->errno_value = initial_errno;
    raw_spin_lock_irqsave(&domain->lock, flags);
    ret = own_current && !domain->errno_slot ? -EAGAIN :
        s31_domain_get(&domain->state, S31_DOMAIN_TASK);
    if (!ret) {
        domain->creating++;
        task->errno_slot = domain->errno_slot;
    }
    raw_spin_unlock_irqrestore(&domain->lock, flags);
    if (ret) { kfree(task); return ret; }
    s31_radio_domain_reap(domain);
    get_task_struct(thread);
    ret = task_exit_work_add(thread, &task->exit, s31_radio_domain_exit);
    if (ret) {
        put_task_struct(thread);
        raw_spin_lock_irqsave(&domain->lock, flags);
        if (s31_domain_put(&domain->state, S31_DOMAIN_TASK))
            panic("s31 radio domain: failed task lost reference");
        domain->creating--;
        raw_spin_unlock_irqrestore(&domain->lock, flags);
        kfree(task);
        return ret;
    }
    raw_spin_lock_irqsave(&domain->lock, flags);
    list_add(&task->node, &domain->tasks);
    domain->creating--;
    raw_spin_unlock_irqrestore(&domain->lock, flags);
    if (own_current) {
        /* Publish both switch-hook banks and the current errno cell as one
         * local transition. No sleeping or module callback under this guard. */
        preempt_disable();
        local_irq_save(flags);
        WRITE_ONCE(task_thread_info(thread)->s31_radio_context, &task->context);
        WRITE_ONCE(task_thread_info(thread)->s31_blob_fp, &task->context.fp);
        WRITE_ONCE(*task->errno_slot, task->errno_value);
        local_irq_restore(flags);
        preempt_enable();
    } else {
        WRITE_ONCE(task_thread_info(thread)->s31_radio_context, &task->context);
        WRITE_ONCE(task_thread_info(thread)->s31_blob_fp, &task->context.fp);
    }
    return 0;
}

int esp32s31_radio_domain_task_attach(struct task_struct *thread,
                                     struct esp32s31_radio_domain *domain, void *data)
{
    return s31_radio_domain_task_attach(thread, domain, data, false, 0);
}
EXPORT_SYMBOL_GPL(esp32s31_radio_domain_task_attach);

/* Bootstrap has ended on this owner's Linux stack, with no installed foreign
 * window or FP borrower. The driver retains the domain and data through exact
 * owner-side finalization, detach, retirement and generic exit completion. */
int esp32s31_radio_domain_current_attach(struct esp32s31_radio_domain *domain,
                                        void *data, int initial_errno)
{
    struct s31_fp_cpu *cpu;
    unsigned long flags;
    int ret = 0;

    if (in_interrupt() || in_atomic() || irqs_disabled()) return -EWOULDBLOCK;
    if (!object_is_on_stack(&flags) || !domain || !domain->persistent_fp || !data ||
        (current->flags & (PF_KTHREAD | PF_NO_SETAFFINITY)) !=
            (PF_KTHREAD | PF_NO_SETAFFINITY)) return -EINVAL;
    if (raw_smp_processor_id() != 0 ||
        !cpumask_equal(current->cpus_ptr, cpumask_of(0)) ||
        !current->mm || current->mm != current->active_mm ||
        !esp32s31_current_uses_init_mm()) return -EXDEV;
    preempt_disable();
    local_irq_save(flags);
    cpu = this_cpu_ptr(&s31_fp_cpu_state);
    if (cpu->owner || cpu->borrower || cpu->kernel_borrow || cpu->foreign_active)
        ret = -EBUSY;
    local_irq_restore(flags);
    preempt_enable();
    if (ret) return ret;
    /* This task has no bank yet. Synchronous allocation/exit registration
     * may sleep, but a balanced switch/IRQ/kernel borrow cannot create one.
     * The common path rejects any prior admission and a missing errno cell. */
    return s31_radio_domain_task_attach(current, domain, data, true, initial_errno);
}
EXPORT_SYMBOL_GPL(esp32s31_radio_domain_current_attach);

static void s31_radio_domain_fp_detached(struct s31_radio_task_context *context)
{
    struct s31_radio_domain_task *task = context->domain_task;
    if (!task || task->thread != current || task->fp_detached)
        panic("s31 radio domain: invalid FP detachment");
    WRITE_ONCE(task->fp_detached, true);
}

int esp32s31_radio_domain_task_retire(struct esp32s31_radio_domain *domain)
{
    struct s31_radio_task_context *context;
    struct s31_radio_domain_task *task;
    unsigned long flags;
    int ret = 0;
    if (in_interrupt() || in_atomic() || irqs_disabled()) return -EWOULDBLOCK;
    preempt_disable();
    local_irq_save(flags);
    context = current_thread_info()->s31_radio_context;
    task = context ? context->domain_task : NULL;
    if (!task || task->domain != domain || task->thread != current || task->retired)
        ret = -EINVAL;
    else if (!task->fp_detached || current_thread_info()->s31_blob_fp || context->driver_data)
        ret = -EBUSY;
    if (!ret) {
        WRITE_ONCE(task->retired, true);
        raw_spin_lock(&domain->lock);
        raw_spin_unlock(&domain->lock);
    }
    local_irq_restore(flags);
    preempt_enable();
    return ret;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_domain_task_retire);
