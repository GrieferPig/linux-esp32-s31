/* SPDX-License-Identifier: GPL-2.0-only */
/* Canonical PI state is globally mapped. The list tracks allocations only;
 * ordinary claims/gives never take a global owner-registry lock or task pin.
 * Payload admissions retain the sync and cancellation flag through every
 * native wait, reservation handoff and same-owner unwind.
 */
struct s31_native_mutex {
	struct rt_mutex lock;
	atomic_t cancelled;
	atomic_t attempts;
	struct list_head allocations;
	/* Linux rejects self/cyclic rtmutex waits with EDEADLK. Preserve the
	 * caller's cancelable timeout semantics on this exceptional path only;
	 * it neither acquires nor donates through a second acquisition queue.
	 */
	wait_queue_head_t deadlock_wait;
	bool reserved;
	bool published;
	bool tx_credit;
};

static unsigned long s31_native_mutex_tx_credit_bind(void *opaque)
{
	struct s31_linux_sync *sync = opaque;

	if (!sync || !sync->native_mutex || !sync->native)
		return 0;
	WRITE_ONCE(sync->native->tx_credit, true);
	return 1;
}

static LIST_HEAD(s31_native_mutexes);
static DEFINE_RAW_SPINLOCK(s31_native_mutex_alloc_lock);
static unsigned long s31_native_mutex_created, s31_native_mutex_freed;
static unsigned long s31_native_atomic_calls, s31_native_atomic_created;

static struct s31_critical_state *s31_critical_state_current(void);

static struct task_struct *s31_native_mutex_owner(struct s31_native_mutex *mutex)
{
	return rt_mutex_owner(&mutex->lock.rtmutex);
}

static void s31_native_mutex_notify(struct s31_native_mutex *mutex)
{
	rt_mutex_wake_waiters(&mutex->lock);
	wake_up_all(&mutex->deadlock_wait);
}

static bool s31_native_mutex_release(struct s31_native_mutex *mutex)
{
	/* Ownership alone is not a reservation: a failed normal self-take still
	 * owns its older published span, which this unwind must not release.
	 */
	if (s31_native_mutex_owner(mutex) != current || !mutex->reserved ||
	    mutex->published)
		return false;
	mutex->reserved = false;
	if (atomic_read_acquire(&mutex->cancelled))
		s31_native_mutex_notify(mutex);
	{

		rt_mutex_unlock(&mutex->lock);
	}
	if (READ_ONCE(mutex->tx_credit))
		s31_radio_tx_credit_return(S31_TX_CREDIT_LOCK);
	return true;
}

/* Short metadata operations execute under the payload object's lock, except
 * END (the same admitted caller after its final unwind). No scheduler wait,
 * allocation, global owner lock, or SRAM TCB dereference happens here.
 */
static unsigned long s31_native_mutex_owner_op(struct s31_linux_sync *sync, u32 op)
{

	struct s31_native_mutex *mutex = sync->native;
	struct task_struct *owner = s31_native_mutex_owner(mutex);
	struct s31_linux_task *task;

	switch (op) {
	case S31_SYNC_NATIVE_BEGIN:
		if (atomic_read_acquire(&mutex->cancelled))
			return 0;
		atomic_inc(&mutex->attempts);
		return 1;
	case S31_SYNC_NATIVE_END:
		if (atomic_dec_return(&mutex->attempts) < 0)
			panic("esp32s31-radio: native mutex admission underflow");
		return 1;
	case S31_SYNC_NATIVE_BUSY:
		return atomic_read_acquire(&mutex->cancelled) ||
			atomic_read(&mutex->attempts) || mutex->published ||
			mutex->reserved || rt_mutex_base_is_locked(&mutex->lock.rtmutex);
	case S31_SYNC_NATIVE_AVAILABLE:
		return !atomic_read_acquire(&mutex->cancelled) &&
			!rt_mutex_base_is_locked(&mutex->lock.rtmutex);
	case S31_SYNC_OWNER_CLAIM:
		/* Managed body stops cancel publication. Foreign calls only poll;
		 * their registered same-owner TLS continuation remains allowed to
		 * acquire fresh locks after kthread_stop has requested retirement.
		 */
		task = s31_linux_current_task();
		if (owner != current || !mutex->reserved || mutex->published ||
		    atomic_read_acquire(&mutex->cancelled) ||
		    (task && !task->finalizing &&
		     (kthread_should_stop() || atomic_read(&task->stopping))))
			return 0;
		mutex->published = true;
		mutex->reserved = false;
		return 1;
	case S31_SYNC_OWNER_MATCH:
		return owner == current && mutex->published;
	case S31_SYNC_OWNER_ALIVE:
		return owner && mutex->published;
	case S31_SYNC_OWNER_DROP:
	case S31_SYNC_OWNER_ABANDON:
		if (owner != current || !mutex->published || mutex->reserved)
			return 0;
		if (op == S31_SYNC_OWNER_ABANDON)
			atomic_set_release(&mutex->cancelled, 1);
		mutex->published = false;
		mutex->reserved = true; /* Release is joined before its lease drops. */
		return 1;
	case S31_SYNC_OWNER_COOKIE:
		/* Native payload metadata already owns its guarded TCB association. */
		return 0;
	default:
		panic("esp32s31-radio: invalid native mutex identity operation %u", op);
	}
}

static bool s31_native_mutex_quiescent(struct s31_native_mutex *mutex)
{
	return !atomic_read(&mutex->attempts) && !mutex->reserved &&
		!mutex->published && !rt_mutex_base_is_locked(&mutex->lock.rtmutex) &&
		RB_EMPTY_ROOT(&mutex->lock.rtmutex.waiters.rb_root) &&
		!waitqueue_active(&mutex->deadlock_wait);
}

static void s31_native_mutex_destroy(struct s31_linux_sync *sync)
{
	struct s31_native_mutex *mutex = sync->native;
	unsigned long flags;

	if (!s31_native_mutex_quiescent(mutex))
		panic("esp32s31-radio: native mutex freed before quiescence");
	raw_spin_lock_irqsave(&s31_native_mutex_alloc_lock, flags);
	list_del(&mutex->allocations);
	s31_native_mutex_freed++;
	raw_spin_unlock_irqrestore(&s31_native_mutex_alloc_lock, flags);
	sync->native = NULL;
	sync->native_mutex = false;
	kfree(mutex);
}

/* Same-owner structured exit must prove this before PF_EXITING. No scan is
 * on the claim/give hot path. The allocation lock stabilizes globally mapped
 * objects, not vendor code, q->owner metadata, or another owner's lifetime.
 */
static void s31_native_mutex_assert_current_quiescent(void)
{
	struct s31_native_mutex *mutex;
	unsigned long flags;

	raw_spin_lock_irqsave(&s31_native_mutex_alloc_lock, flags);
	list_for_each_entry(mutex, &s31_native_mutexes, allocations)
		if (s31_native_mutex_owner(mutex) == current)
			panic("esp32s31-radio: thread exit retains native mutex ownership");
	raw_spin_unlock_irqrestore(&s31_native_mutex_alloc_lock, flags);
}

/* All producers/callbacks/tasks have joined. Retained IDF objects end with
 * their firmware heap; release their kernel sidecars before that heap resets.
 */
static void s31_native_mutex_retire_all(void)
{
	struct s31_native_mutex *mutex, *next;
	unsigned long retained = 0;

	list_for_each_entry_safe(mutex, next, &s31_native_mutexes, allocations) {
		if (!s31_native_mutex_quiescent(mutex))
			panic("esp32s31-radio: retained native mutex remains live");
		list_del(&mutex->allocations);
		kfree(mutex);
		s31_native_mutex_freed++;
		retained++;
	}
	if (s31_native_mutex_created != s31_native_mutex_freed)
		panic("esp32s31-radio: native mutex sidecar leak");

}

struct s31_native_object_args {
	struct s31_linux_sync *sync;
	struct s31_linux_task *task;
	u32 op, ticks;
	ktime_t deadline;
	long result;
	bool atomic_call;
};

static long s31_native_mutex_take(struct s31_native_mutex *mutex, u32 ticks,
				  ktime_t deadline)
{

	long result;
	bool acquired;

	if (atomic_read_acquire(&mutex->cancelled))
		return -EIDRM;
	/* Uncontended finalizer acquisitions remain legal after stop. The slow
	 * wait cancels immediately if stop is already pending, as before.
	 */
	acquired = rt_mutex_trylock(&mutex->lock);
	if (acquired)
		result = 0;
	else if (!ticks)
		result = -EBUSY;
	else {
		result = rt_mutex_lock_kthread_until_cancel(&mutex->lock, deadline,
							   &mutex->cancelled);
	}
	if (result == -EDEADLK) {
		/* Preserve a normal self/cycle wait's deadline and cancellation.
		 * The old published owner remains live; no new reservation exists.
		 */
		if (deadline == KTIME_MAX)
			result = wait_event_interruptible(mutex->deadlock_wait,
				atomic_read_acquire(&mutex->cancelled) || kthread_should_stop());
		else
			result = wait_event_interruptible_hrtimeout(mutex->deadlock_wait,
				atomic_read_acquire(&mutex->cancelled) || kthread_should_stop(),
				ktime_sub(deadline, ktime_get()));
		if (kthread_should_stop())
			result = -EINTR;
		else if (atomic_read_acquire(&mutex->cancelled))
			result = -EIDRM;
		else
			result = result ? result == -ETIME ? -ETIMEDOUT : result : -ETIMEDOUT;
	}
	if (!result) {
		if (mutex->published || mutex->reserved ||
		    s31_native_mutex_owner(mutex) != current)
			panic("esp32s31-radio: native acquisition has no new reservation");
		mutex->reserved = true;
	}
	return result;
}

static void s31_native_object_on_kernel_stack(void *opaque)
{

	struct s31_native_object_args *args = opaque;
	struct s31_native_mutex *mutex;
	unsigned long flags;

	if (args->atomic_call) {
		/* Keep the caller's portMUX, IRQ state, gate and FP bank intact.
		 * Only the physical SP changes; none of these operations may sleep. */
		if (!irqs_disabled() || !object_is_on_stack(&flags))
			panic("esp32s31-radio: atomic native operation escaped its fence");
		s31_native_atomic_calls++;
	} else if (!s31_task_create_kernel_context()) {
		panic("esp32s31-radio: native object operation outside Linux stack");
	}
	if (args->op == S31_SYNC_NATIVE_INIT) {
		mutex = kzalloc(sizeof(*mutex), args->atomic_call ? GFP_ATOMIC : GFP_KERNEL);
		args->result = mutex ? 1 : 0;
		if (mutex) {
			if (args->atomic_call)
				s31_native_atomic_created++;
			rt_mutex_init(&mutex->lock);
			atomic_set(&mutex->cancelled, 0);
			atomic_set(&mutex->attempts, 0);
			init_waitqueue_head(&mutex->deadlock_wait);
			raw_spin_lock_irqsave(&s31_native_mutex_alloc_lock, flags);
			list_add(&mutex->allocations, &s31_native_mutexes);
			s31_native_mutex_created++;
			raw_spin_unlock_irqrestore(&s31_native_mutex_alloc_lock, flags);
			args->sync->native = mutex;
			args->sync->native_mutex = true;
		}
	} else if (args->op == S31_SYNC_NATIVE_BASE) {
		struct task_struct *target = (void *)args->sync;

		/* A native target is already RR. Never enter the sleeping
		 * DEADLINE/cpuset transition from a retained portMUX section. */
		args->result = READ_ONCE(target->flags) & PF_EXITING ? -ESRCH :
			args->atomic_call && READ_ONCE(target->policy) != SCHED_RR ? -EINVAL :
			esp32s31_sched_setscheduler(target, SCHED_RR, args->ticks + 1);
	} else {
		mutex = args->sync->native;
		switch (args->op) {
		case S31_SYNC_NATIVE_TAKE:
			args->result = s31_native_mutex_take(mutex,
					args->atomic_call ? 0 : args->ticks,
					args->deadline);
			if (args->atomic_call && args->result == -EBUSY) {

				args->result = -EWOULDBLOCK;
			}
			/* Stop cannot longjmp out of a held portMUX. Publication
			 * still refuses a stopped body and unwinds any reservation;
			 * normal stop cleanup runs at the next legal boundary. */
			if (!args->atomic_call && args->task && !args->task->finalizing &&
			    (kthread_should_stop() || atomic_read(&args->task->stopping))) {
				s31_native_mutex_release(mutex);
				if (!args->task->finalizing) {
					s31_linux_task_exit_current();
					unreachable();
				}
				args->result = -EINTR;
			}
			break;
		case S31_SYNC_NATIVE_RELEASE:
			args->result = s31_native_mutex_release(mutex);
			break;
		case S31_SYNC_NATIVE_DELETE:
			atomic_set_release(&mutex->cancelled, 1);
			s31_native_mutex_notify(mutex);
			args->result = 1;
			break;
		case S31_SYNC_NATIVE_DESTROY:
			s31_native_mutex_destroy(args->sync);
			args->result = 1;
			break;
		default:
			panic("esp32s31-radio: invalid native object call %u", args->op);
		}
	}
}

static long s31_native_object_call(struct s31_linux_sync *sync, u32 op, u32 ticks)
{

	struct s31_linux_task *task = s31_linux_current_task();
	struct s31_native_object_args args = {
		.sync = sync, .task = task, .op = op, .ticks = ticks,
	};
	unsigned long flags;
	bool on_linux = object_is_on_stack(&args);
	long rejected = op == S31_SYNC_NATIVE_TAKE || op == S31_SYNC_NATIVE_BASE ? -EWOULDBLOCK : 0;

	if (in_interrupt() || (task && task->sync_lock_depth) || !on_linux)
		return rejected;
	/* An ordinary Linux cleanup/setup caller needs no vendor context.
	 * A foreign installed vendor window cannot acquire a blocking ABI. */
	if (!task && s31_blob_active())
		return rejected;
	/* A zero-time take cannot sleep, so it needs no clock read/deadline.
	 * Finite blocking takes still capture their deadline before trylock. */
	if (op == S31_SYNC_NATIVE_TAKE && ticks)
		args.deadline = s31_native_tick_deadline(ticks);
	/* Atomic object operations are part of the native contract, including
	 * ordinary IDF critical sections as well as portMUX. Preserve exclusion,
	 * use atomic allocation/trylock, and never suspend or pivot the caller. */
	if (in_atomic() || irqs_disabled()) {
		args.atomic_call = true;
		local_irq_save(flags);
		s31_native_object_on_kernel_stack(&args);
		local_irq_restore(flags);
		return args.result;
	}
	if (task) {
		if (!s31_task_native_linux_call(task, s31_native_object_on_kernel_stack,
			&args)) {
			return rejected;
  }
	} else {
		s31_native_object_on_kernel_stack(&args);
	}
	return args.result;
}

/* Coalesce existing metadata/primitive operations at the ABI boundary.
 * The payload still admits and pins its descriptor, and publishes the same
 * owner/call cleanup leases. These leaf transactions cannot cancel or exit
 * the current task: END and metadata unlock cannot sleep; RELEASE uses the
 * original guarded, nonblocking release entry (only TAKE has a stop exit).
 * Consequently the caller can select its settled cleanup before this call.
 * No raw metadata lock is held over the rtmutex release or its wakeups.
 */
static unsigned long s31_native_mutex_transaction(void *opaque, u32 op)
{
	struct s31_linux_sync *sync = opaque;

	if (!s31_native_sync_enabled(sync) || in_interrupt())
		return 0;
	if (op == S31_SYNC_NATIVE_META_ENTER) {
		s31_linux_sync_lock(sync);
		return S31_NATIVE_META_ENTERED |
			(s31_native_mutex_owner_op(sync, S31_SYNC_OWNER_MATCH) ?
			 S31_NATIVE_META_OWNER : 0);
	}
	if (op == S31_SYNC_NATIVE_TRY_BEGIN) {
		struct s31_native_mutex *mutex = sync->native;
		struct s31_linux_task *task;
		unsigned long on_stack;

		/* The durable payload call node already pins this object. Admit
		 * before trying its canonical Linux PI mutex; a miss keeps that
		 * same admission for the normal blocking/cancelable wait below.
		 * Bit 0 is admission, bit 1 a new unpublished reservation.
		 */
		if (!s31_native_mutex_owner_op(sync, S31_SYNC_NATIVE_BEGIN))
			return 0;
		task = s31_linux_current_task();
		if (!task || !task->blob_active || task->blob.preempt_fenced ||
		    task->critical.depth || task->critical.suspended ||
		    task->critical.mux_depth || task->sync_lock_depth != 1 ||
		    current->stack != task->payload_stack ||
		    !object_is_on_stack(&on_stack) || !irqs_disabled() ||
		    (!task->finalizing && (kthread_should_stop() ||
					 atomic_read(&task->stopping))))
			return 1;

		if (task_stack_end_corrupted(current))
			panic("esp32s31-radio: native try stack guard exhausted");
		/* rt_mutex_trylock is explicitly legal in an atomic thread
		 * region. Metadata masks IRQs, so there is no sleeping primitive
		 * or reentrant vendor handler and no Linux-call marker to enter.
		 * Linux still owns owner/waiter bits, PI and the contended path.
		 */
		{
			bool acquired;

			acquired = rt_mutex_trylock(&mutex->lock);
			if (!acquired)
				return 1;
		}
		if (mutex->published || mutex->reserved ||
		    s31_native_mutex_owner(mutex) != current)
			panic("esp32s31-radio: native try has no new reservation");
		mutex->reserved = true;
		return 3;
	}
	if (op == S31_SYNC_NATIVE_GIVE_LEAVE) {
		if (!s31_native_mutex_owner_op(sync, S31_SYNC_OWNER_DROP))
			panic("esp32s31-radio: native give lost published ownership");
		s31_linux_sync_unlock(sync);
		if (!s31_native_object_call(sync, S31_SYNC_NATIVE_RELEASE, 0))
			panic("esp32s31-radio: native give failed release");
		s31_native_mutex_owner_op(sync, S31_SYNC_NATIVE_END);
		return 1;
	}
	if (op == S31_SYNC_NATIVE_SETTLED_LEAVE) {
		s31_native_mutex_owner_op(sync, S31_SYNC_NATIVE_END);
		s31_linux_sync_unlock(sync);
		return 1;
	}
	panic("esp32s31-radio: invalid native mutex transaction %u", op);
}
