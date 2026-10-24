/* SPDX-License-Identifier: GPL-2.0 */
/* Native owner lifetime. All fallible storage preparation runs on Linux SP.
 * The generic task-exit callback completes AFTER module code returns. */
struct s31_foreign_owner {
 struct list_head node;
 struct task_struct *thread;
 struct task_exit_work exit;
 refcount_t refs;
 bool retiring;
 bool control_only, control_sealed;
 int errno_value;
 struct esp32s31_radio_domain *domain;
};
static LIST_HEAD(s31_foreign_owners);
static DEFINE_SPINLOCK(s31_foreign_owners_lock);
static DEFINE_MUTEX(s31_foreign_prepare_lock);
static bool s31_foreign_owners_closing;

static void s31_foreign_owner_put(struct s31_foreign_owner *owner)
{
 if (!owner || !refcount_dec_and_test(&owner->refs))
  return;
 if (WARN_ON_ONCE(!completion_done(&owner->exit.done)))
  panic("esp32s31-radio: freeing unfinished foreign owner");
 put_task_struct(owner->thread);
 s31_native_domain_put(owner->domain, ESP32S31_RADIO_DOMAIN_FOREIGN);
 kfree(owner);
}

static struct s31_foreign_owner *s31_foreign_owner_get(struct task_struct *thread)
{
 struct s31_foreign_owner *owner, *found = NULL;
 unsigned long flags;

 spin_lock_irqsave(&s31_foreign_owners_lock, flags);
 list_for_each_entry(owner, &s31_foreign_owners, node)
  if (owner->thread == thread) {
   refcount_inc(&owner->refs);
   found = owner;
   break;
  }
 spin_unlock_irqrestore(&s31_foreign_owners_lock, flags);
 return found;
}

static bool s31_foreign_owner_retiring(struct task_struct *thread)
{
 struct s31_foreign_owner *owner = s31_foreign_owner_get(thread);
 bool retiring = owner && READ_ONCE(owner->retiring);

 s31_foreign_owner_put(owner);
 return retiring;
}

/* prepare_lock serializes registry mutation. A lookup takes its own ref under
 * the list lock, so completion observers remain valid across removal. No
 * completed history or task pin is retained merely for a future unload. */
static void s31_foreign_owners_reap(void)
{
 struct s31_foreign_owner *owner, *next;
 unsigned long flags;
 LIST_HEAD(retired);

 list_for_each_entry_safe(owner, next, &s31_foreign_owners, node) {
  if (!completion_done(&owner->exit.done))
   continue;
  spin_lock_irqsave(&s31_foreign_owners_lock, flags);
  list_move(&owner->node, &retired);
  spin_unlock_irqrestore(&s31_foreign_owners_lock, flags);
 }
 list_for_each_entry_safe(owner, next, &retired, node) {
  list_del(&owner->node);
  s31_foreign_owner_put(owner); /* drop registry ref on ordinary Linux SP */
 }
}

static void s31_foreign_owner_exit(struct task_exit_work *work)
{
 struct s31_foreign_owner *owner = container_of(work, struct s31_foreign_owner, exit);

 if (owner->thread != current || !owner->control_only || in_interrupt() ||
     irqs_disabled() || preempt_count() || s31_linux_current_task() ||
     s31_linux_blob_held_by_current() || current->migration_disabled ||
     s31_blob_active())
  panic("esp32s31-radio: control retirement outside native continuation");
 WRITE_ONCE(owner->retiring, true);
 s31_native_mutex_assert_current_quiescent();

}

static int s31_foreign_owner_prepare_impl(void)
{
 struct s31_foreign_owner *owner;
 unsigned long flags;
 int ret = 0;

 /* This registry contains only the private Linux control executor. */
 if (s31_linux_current_task() || s31_current_service()) return 0;
 if (!(current->flags & PF_KTHREAD) || in_interrupt() || in_atomic() || irqs_disabled())
  return -EWOULDBLOCK;
 /* Native exit callbacks run on this same owner after the window ends.
  * A wide affinity could move retirement to core1; refuse before allocating
  * or publishing storage. Never change a shared pool worker's affinity. */
 if (raw_smp_processor_id() != 0 ||
     !cpumask_equal(current->cpus_ptr, cpumask_of(0)))
  return -EXDEV;
 owner = s31_foreign_owner_get(current);
 if (owner && !READ_ONCE(owner->retiring)) {
  ret = owner->control_only ? 0 : -EINVAL;
  s31_foreign_owner_put(owner);
  return ret;
 }
 s31_foreign_owner_put(owner);
 mutex_lock(&s31_foreign_prepare_lock);
 if (s31_foreign_owners_closing) { ret = -ESHUTDOWN; goto out; }
 s31_foreign_owners_reap();
 owner = s31_foreign_owner_get(current);
 if (owner) {
  if (READ_ONCE(owner->retiring)) ret = -EBUSY;
  else if (!owner->control_only) ret = -EINVAL;
  s31_foreign_owner_put(owner);
  goto out;
 }
 owner = kzalloc(sizeof(*owner), GFP_KERNEL);
 if (!owner) { ret = -ENOMEM; goto out; }
 owner->control_only = true;

 owner->thread = current;
 ret = s31_native_domain_get(ESP32S31_RADIO_DOMAIN_FOREIGN, &owner->domain);
 if (ret) {
  kfree(owner);
  goto out;
 }
 refcount_set(&owner->refs, 1); /* registry reference */
 get_task_struct(current);
 ret = task_exit_work_add(current, &owner->exit, s31_foreign_owner_exit);
 if (ret) {
  put_task_struct(current);
  s31_native_domain_put(owner->domain, ESP32S31_RADIO_DOMAIN_FOREIGN);
  kfree(owner);
  goto out;
 }
 spin_lock_irqsave(&s31_foreign_owners_lock, flags);
 list_add(&owner->node, &s31_foreign_owners);
 spin_unlock_irqrestore(&s31_foreign_owners_lock, flags);
out:
 mutex_unlock(&s31_foreign_prepare_lock);
 return ret;
}

int s31_foreign_owner_prepare_control(void)
{
 if ((current != s31_service_context.thread))
  return -EINVAL;
 return s31_foreign_owner_prepare_impl();
}

int s31_linux_control_bootstrap_enter(struct s31_blob_context *context)
{
 struct s31_foreign_owner *owner;
 unsigned long flags;
 int ret = 0;

 if (!context || (unsigned long)context < PAGE_OFFSET ||
     !s31_task_create_kernel_context() || current->migration_disabled ||
     READ_ONCE(s31_errno_slot) || !READ_ONCE(s31_native_domain) ||
     this_cpu_read(s31_foreign_installed))
  return -EWOULDBLOCK;
 owner = s31_foreign_owner_get(current);
 if (!owner || !owner->control_only || owner->control_sealed ||
     owner->retiring || current != s31_service_context.thread) {
  s31_foreign_owner_put(owner);
  return -EINVAL;
 }
 mutex_lock(&s31_task_start_lock);
 /* Keep ordinary Linux tasks from borrowing the CPU-local cold window;
  * timer/IPI handling remains enabled. This scope cannot sleep or create a
  * managed task. Publish under the factory lock before reopening it.
  */
 preempt_disable();
 spin_lock_irqsave(&s31_task_list_lock, flags);
 if (!s31_tasks_accepting || s31_task_list ||
     atomic_read(&s31_tasks_creating) || s31_linux_control_bootstrap_active())
  ret = -EBUSY;
 spin_unlock_irqrestore(&s31_task_list_lock, flags);
 spin_lock_irqsave(&s31_foreign_owners_lock, flags);
 if (!list_is_singular(&s31_foreign_owners)) ret = -EBUSY;
 spin_unlock_irqrestore(&s31_foreign_owners_lock, flags);
 if (!s31_radio_control_bootstrap_quiescent()) ret = -EBUSY;
 if (!ret) {
  WRITE_ONCE(s31_control_bootstrap_context, context);
  smp_store_release(&s31_control_bootstrap_thread, current);
 }
 mutex_unlock(&s31_task_start_lock);
 s31_foreign_owner_put(owner);
 if (ret) {
  preempt_enable();
  return ret;
 }
 s31_linux_blob_enter(context);
 return 0;
}

void s31_linux_control_bootstrap_leave(struct s31_blob_context *context)
{
 if (!s31_control_bootstrap_owns(context))
  panic("esp32s31-radio: exclusive bootstrap owner mismatch");
 s31_linux_blob_leave(context);
 if (this_cpu_read(s31_foreign_installed) || context->preempt_fenced ||
     context->errno_active || context->stack_switched ||
     current->migration_disabled)
  panic("esp32s31-radio: exclusive bootstrap leaked context");

 WRITE_ONCE(s31_control_bootstrap_context, NULL);
 smp_store_release(&s31_control_bootstrap_thread, NULL);
 preempt_enable();

}

int s31_foreign_owner_seal_control_current(void)
{
 struct s31_foreign_owner *owner;

 if (!s31_task_create_kernel_context() || s31_blob_active() ||
     s31_linux_blob_held_by_current() || current->migration_disabled)
  return -EWOULDBLOCK;
 owner = s31_foreign_owner_get(current);
 if (!owner || !owner->control_only || READ_ONCE(owner->retiring)) {
  s31_foreign_owner_put(owner);
  return -EINVAL;
 }
 WRITE_ONCE(owner->control_sealed, true);
 s31_foreign_owner_put(owner);

 return 0;
}

static void s31_control_owner_assert_vendor_entry(struct s31_blob_context *context)
{
 struct s31_foreign_owner *owner;
 bool sealed, control_only;

 /* Only the explicitly prepared private control thread can have this role.
  * Managed tasks and genuine foreign callers avoid a registry lookup here.
  */
 if (current != READ_ONCE(s31_service_context.thread)) return;
 owner = s31_foreign_owner_get(current);
 sealed = owner && READ_ONCE(owner->control_sealed);
 control_only = owner && owner->control_only;
 s31_foreign_owner_put(owner);
 if (sealed)
  panic("esp32s31-radio: sealed Linux control owner attempted vendor entry");
 if (control_only && !s31_control_bootstrap_owns(context))
  panic("esp32s31-radio: control owner entered outside exclusive bootstrap");
}

/* Dedicated executors retire on their own Linux stack while init_mm is
 * still borrowed. Complete the exact owner callback before stopping them. */
int s31_foreign_owner_finish_current(void)
{
 struct s31_foreign_owner *owner = s31_foreign_owner_get(current);
 int ret;

 if (!owner) return 0;
 ret = task_exit_work_finish_current(&owner->exit);
 s31_foreign_owner_put(owner);
 return ret;
}

void s31_foreign_owners_close(void)
{
 struct s31_foreign_owner *owner, *next;
 unsigned long flags;
 LIST_HEAD(retired);

 mutex_lock(&s31_foreign_prepare_lock);
 s31_foreign_owners_closing = true;
 list_for_each_entry(owner, &s31_foreign_owners, node)
  if (task_exit_work_request(owner->thread, &owner->exit))
   panic("esp32s31-radio: cannot request foreign owner teardown");
 list_for_each_entry(owner, &s31_foreign_owners, node)
  wait_for_completion(&owner->exit.done);
 spin_lock_irqsave(&s31_foreign_owners_lock, flags);
 list_splice_init(&s31_foreign_owners, &retired);
 spin_unlock_irqrestore(&s31_foreign_owners_lock, flags);
 list_for_each_entry_safe(owner, next, &retired, node) {
  list_del(&owner->node);
  s31_foreign_owner_put(owner);
 }
 s31_foreign_owners_closing = false;
 mutex_unlock(&s31_foreign_prepare_lock);
}
