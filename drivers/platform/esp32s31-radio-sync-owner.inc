/* SPDX-License-Identifier: GPL-2.0 */
/* Included after the sync definition. All token mutations are serialized by
 * this native lock; callers also serialize their payload object state. A task
 * reference, rather than cookie bytes or a per-CPU identity, prevents ABA. */
static DEFINE_RAW_SPINLOCK(s31_sync_owner_lock);
static LIST_HEAD(s31_sync_owned);
static unsigned long s31_sync_owner_claims, s31_sync_owner_releases;
static unsigned long s31_sync_owner_abandons, s31_sync_owner_active;

enum {
	S31_SYNC_OWNER_CLAIM = 0,
	S31_SYNC_OWNER_MATCH = 1,
	S31_SYNC_OWNER_COOKIE = 2,
	S31_SYNC_OWNER_DROP = 3,
	S31_SYNC_OWNER_ABANDON = 4,
	S31_SYNC_OWNER_ALIVE = 5,
	S31_SYNC_OWNER_FORCE_DROP = 6,
	S31_SYNC_OWNER_FATAL = 7,
	S31_SYNC_MUX_ENTER = 8,
	S31_SYNC_MUX_EXIT = 9,
	S31_SYNC_CRITICAL_DEPTH = 10,
	S31_SYNC_OWNER_FOREIGN_TOKEN = 11,
	S31_SYNC_ERRNO_REGISTER = 16,
	S31_SYNC_NATIVE_MODE = 17,
	S31_SYNC_NATIVE_INIT = 18,
	S31_SYNC_NATIVE_BEGIN = 19,
	S31_SYNC_NATIVE_END = 20,
	S31_SYNC_NATIVE_BUSY = 21,
	S31_SYNC_NATIVE_AVAILABLE = 22,
	S31_SYNC_NATIVE_TAKE = 23,
	S31_SYNC_NATIVE_RELEASE = 24,
	S31_SYNC_NATIVE_DELETE = 25,
	S31_SYNC_NATIVE_DESTROY = 26,
	S31_SYNC_NATIVE_EFFECTIVE = 27,
	S31_SYNC_NATIVE_BASE = 28,
	S31_SYNC_NATIVE_BASE_QUERY = 29,
	S31_SYNC_STACK_RECLAIM = 36,
	S31_SYNC_TIMER_CHANGED = 39,
	S31_SYNC_TX_LOCK_BIND = 40,
	S31_SYNC_TX_FLIGHT_RETURN = 41,
	S31_SYNC_STACK_RAW_MINIMUM = 42,
	S31_SYNC_STACK_ALLOC = 43,
	S31_SYNC_STACK_FREE = 44,
	S31_SYNC_SOFTMAC_TX_COPY = 45,
	S31_SYNC_TX_REPAIR_CAP = 47,
	S31_SYNC_TX_POOL_BIND = 48,
	S31_SYNC_AUX_CONTEXT = 49,
	S31_SYNC_NATIVE_META_ENTER = 51,
	S31_SYNC_NATIVE_SETTLED_LEAVE = 52,
	S31_SYNC_NATIVE_GIVE_LEAVE = 53,
	S31_SYNC_NATIVE_TRANSACTION_ABI = 54,
	S31_SYNC_NATIVE_TRY_BEGIN = 55,
};
#define S31_NATIVE_META_ENTERED 1U
#define S31_NATIVE_META_OWNER 2U

static unsigned long s31_sync_owner_operation(struct s31_linux_sync *sync,
	u32 op, struct task_struct *thread, void *cookie)
{
	struct task_struct *release = NULL;
	unsigned long flags, result = 0;
	bool live;

	if (!sync)
		return 0;
	if (sync->native_mutex)
		return s31_native_mutex_owner_op(sync, op);
	raw_spin_lock_irqsave(&s31_sync_owner_lock, flags);
	live = sync->owner_thread && !sync->owner_abandoned &&
		!(READ_ONCE(sync->owner_thread->flags) & PF_EXITING);
	switch (op) {
	case S31_SYNC_OWNER_CLAIM:
		if (!sync->owner_thread && !sync->owner_abandoned &&
		    !(READ_ONCE(thread->flags) & PF_EXITING)) {
			get_task_struct(thread);
			sync->owner_thread = thread;
			sync->owner_cookie = cookie;
			list_add(&sync->owner_node, &s31_sync_owned);
			s31_sync_owner_claims++;
			s31_sync_owner_active++;
			result = 1;
		}
		break;
	case S31_SYNC_OWNER_MATCH:
		result = live && sync->owner_thread == thread &&
			(!sync->owner_cookie || sync->owner_cookie == cookie);
		break;
	case S31_SYNC_OWNER_COOKIE:
		result = live ? (unsigned long)sync->owner_cookie : 0;
		break;
	case S31_SYNC_OWNER_ALIVE:
		result = live;
		break;
	case S31_SYNC_OWNER_DROP:
	case S31_SYNC_OWNER_ABANDON:
		/* Only the owner may return or retire its lease. A dead owner can
		 * still run its own final continuation before Linux PF_EXITING. */
		if (sync->owner_thread != thread ||
		    (sync->owner_cookie && sync->owner_cookie != cookie))
			break;
		fallthrough;
	case S31_SYNC_OWNER_FORCE_DROP:
		if (sync->owner_thread) {
			release = sync->owner_thread;
			s31_sync_owner_releases++;
			s31_sync_owner_active--;
			if (op == S31_SYNC_OWNER_ABANDON) s31_sync_owner_abandons++;
			list_del_init(&sync->owner_node);
		}
		sync->owner_thread = NULL;
		sync->owner_cookie = NULL;
		if (op == S31_SYNC_OWNER_ABANDON)
			sync->owner_abandoned = true;
		result = 1;
		break;
	default:
		break;
	}
	raw_spin_unlock_irqrestore(&s31_sync_owner_lock, flags);
	if (release)
		put_task_struct(release);
	return result;
}

/* Called after every task, worker and IRQ is joined, before the private SRAM
 * pool is reclaimed. Retained IDF objects must not retain Linux task pins. */
void s31_linux_sync_owners_retire_all(void)
{
	struct s31_linux_sync *sync;
	struct task_struct *release;
	unsigned long flags;

	for (;;) {
		raw_spin_lock_irqsave(&s31_sync_owner_lock, flags);
		if (list_empty(&s31_sync_owned)) {
			raw_spin_unlock_irqrestore(&s31_sync_owner_lock, flags);
			break;
		}
		sync = list_first_entry(&s31_sync_owned, struct s31_linux_sync, owner_node);
		release = sync->owner_thread;
		s31_sync_owner_releases++;
		s31_sync_owner_active--;
		list_del_init(&sync->owner_node);
		sync->owner_thread = NULL;
		sync->owner_cookie = NULL;
		sync->owner_abandoned = true;
		raw_spin_unlock_irqrestore(&s31_sync_owner_lock, flags);
		put_task_struct(release);
	}
	s31_native_mutex_retire_all();
	/* Teardown has joined every caller; report retained task-pin accounting. */
	WARN_ON_ONCE(s31_sync_owner_active || s31_sync_owner_claims != s31_sync_owner_releases);

}
