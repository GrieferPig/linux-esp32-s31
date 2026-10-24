/* SPDX-License-Identifier: GPL-2.0 */
/* Native IRQ/preemption fence, with distinct legacy and portMUX domains.
 * The actual IDF mux owner/count layout is retained. No allocation or sleep. */
struct s31_idf_mux {
	u32 owner, count;
};
struct s31_mux_enter_args {
	void *mux;
	s32 timeout_cycles;
};
#define S31_MUX_FREE 0xb33fffffU
#define S31_MUX_CPU0 0xcdcdU
#define S31_MUX_CPU1 0xababU

static DEFINE_PER_CPU(struct s31_critical_state, s31_foreign_critical);
static DEFINE_PER_CPU(struct s31_critical_state, s31_irq_critical);

static bool s31_irq_critical_quiescent(void)
{
	return !this_cpu_read(s31_irq_critical.depth);
}

static struct s31_critical_state *s31_critical_state_current(void)
{
	struct s31_linux_task *task;
	if (in_interrupt())
		return this_cpu_ptr(&s31_irq_critical);
	task = s31_linux_current_task();
	return task ? &task->critical : this_cpu_ptr(&s31_foreign_critical);
}

/* The selected critical state is canonical. IRQ admission, sleep admission
 * and retirement read this same record; do not republish duplicate flags or
 * rediscover the task on each lock/unlock. Local IRQ/preemption exclusion
 * already covers every mutation below. */
/* Caller owns the temporary preemption/IRQ guard. Keep exactly one outer
 * guard regardless of recursion or mixing the two exclusion domains. Every
 * temporary preempt_enable below still leaves the retained outer reference;
 * only the final IRQ restore/outer enable can make this invocation schedulable. */
static void s31_critical_acquired(struct s31_critical_state *state,
	unsigned long flags)
{
	if (!state->depth) {
		state->thread = current;
		state->flags = flags;
	} else {
		preempt_enable();
	}
	state->depth++;
}

static void s31_critical_released(struct s31_critical_state *state)
{
	unsigned long flags;
	if (--state->depth) {
		preempt_enable(); /* temporary exit guard only */
		return;
	}
	flags = state->flags;
	state->flags = 0;
	state->thread = NULL;
	preempt_enable(); /* temporary exit guard */

	local_irq_restore(flags);
	preempt_enable(); /* outer acquisition guard */
	s31_radio_blob_run_pending_isrs();
}

static struct s31_critical_state *s31_critical_guard(unsigned long *flags)
{
	struct s31_critical_state *state;
	if (in_nmi())
		panic("esp32s31-radio: NMI critical entry unsupported");
	preempt_disable();
	local_irq_save(*flags);

	state = s31_critical_state_current();
	if (state->suspended || (state->depth && state->thread != current) ||
	    state->depth == U32_MAX)
		panic("esp32s31-radio: invalid critical invocation ownership");
	return state;
}

static u32 s31_mux_owner_cpu(void)
{
	u32 cpu = raw_smp_processor_id();
	if (cpu > 1)
		panic("esp32s31-radio: unsupported portMUX hart");
	return cpu ? S31_MUX_CPU1 : S31_MUX_CPU0;
}

static bool s31_mux_take(struct s31_idf_mux *mux,
	struct s31_critical_state *state, s32 timeout)
{
	u32 owner = s31_mux_owner_cpu(), observed, start;
	observed = READ_ONCE(mux->owner);
	if (observed == owner) {
		/* Recursion is legal only while this invocation has a live fence.
		 * A CPU-encoded owner alone is not a reusable Linux task identity. */
		if (!state->depth || !state->mux_depth ||
		    !READ_ONCE(mux->count) || READ_ONCE(mux->count) >= 0xff)
			panic("esp32s31-radio: unmatched/overflowed recursive portMUX");
		WRITE_ONCE(mux->count, READ_ONCE(mux->count) + 1);
		return true;
	}
	if (observed != S31_MUX_FREE && observed != S31_MUX_CPU0 &&
	    observed != S31_MUX_CPU1)
		panic("esp32s31-radio: corrupt portMUX owner");
	if (cmpxchg(&mux->owner, S31_MUX_FREE, owner) != S31_MUX_FREE) {
		if (!timeout)
			return false; /* exactly one attempt, no cycle read */
		start = s31_mux_cycle_count();
		do {
			if (cmpxchg(&mux->owner, S31_MUX_FREE, owner) == S31_MUX_FREE)
				goto acquired;
		} while (timeout == -1 ||
			 (u32)(s31_mux_cycle_count() - start) <= (u32)timeout);
		return false;
	}
acquired:
	if (READ_ONCE(mux->count))
		panic("esp32s31-radio: free portMUX has nonzero count");
	WRITE_ONCE(mux->count, 1);
	return true;
}

static unsigned long s31_critical_mux_operation(void *opaque, u32 op)
{
	struct s31_critical_state *state;
	struct s31_idf_mux *mux;
	struct s31_mux_enter_args *args;
	unsigned long flags;
	u32 count;
	if (op == S31_SYNC_CRITICAL_DEPTH) {
		if (in_nmi()) return U32_MAX;
		preempt_disable();
		local_irq_save(flags);

		state = s31_critical_state_current();
		count = state->depth;

		local_irq_restore(flags);
		preempt_enable();
		return count;
	}
	if (op == S31_SYNC_MUX_ENTER) {
		args = opaque;
		mux = args ? args->mux : NULL;
		if (!mux || (unsigned long)mux & 3)
			return 0;
		state = s31_critical_guard(&flags);

		if (!s31_mux_take(mux, state, args->timeout_cycles)) {

		local_irq_restore(flags);
			preempt_enable();
			return 0;
		}
		state->mux_depth++;
		s31_critical_acquired(state, flags);
		return 1;
	}
	if (op != S31_SYNC_MUX_EXIT)
		return 0;
	mux = opaque;
	state = s31_critical_guard(&flags);
	if (!mux || (unsigned long)mux & 3 || !state->depth ||
	    !state->mux_depth || READ_ONCE(mux->owner) != s31_mux_owner_cpu() ||
	    !READ_ONCE(mux->count))
		panic("esp32s31-radio: portMUX exit without matching owner");
	count = READ_ONCE(mux->count) - 1;
	WRITE_ONCE(mux->count, count);
	if (!count)
		smp_store_release(&mux->owner, S31_MUX_FREE);
	state->mux_depth--;
	s31_critical_released(state);
	return 1;
}

u32 s31_linux_critical_enter(void)
{

	struct s31_critical_state *state;
	unsigned long flags;
	state = s31_critical_guard(&flags);
	if (!state->legacy_depth)
		raw_spin_lock(&s31_critical_lock);
	state->legacy_depth++;
	s31_critical_acquired(state, flags);
	return state->flags;
}

void s31_linux_critical_exit(u32 unused_flags)
{

	struct s31_critical_state *state;
	unsigned long flags;
	(void)unused_flags; /* native invocation owns the outer IRQ state */
	state = s31_critical_guard(&flags);
	if (!state->depth || !state->legacy_depth)
		panic("esp32s31-radio: legacy critical exit without owner");
	if (!--state->legacy_depth)
		raw_spin_unlock(&s31_critical_lock);
	s31_critical_released(state);
}

void s31_linux_critical_suspend(void)
{
	struct s31_linux_task *task = s31_linux_current_task();
	struct s31_critical_state *state;
	unsigned long flags;
	if (in_nmi())
		panic("esp32s31-radio: NMI critical suspension unsupported");
	if (!task || in_interrupt()) {
		/* Foreign/IRQ callers have no managed legacy suspension protocol.
		 * Reject a held span before a bridge can release the blob gate. */
		preempt_disable();
		local_irq_save(flags);

		state = s31_critical_state_current();
		if (state->depth)
			panic("esp32s31-radio: foreign/IRQ critical sleep or escape");

		local_irq_restore(flags);
		preempt_enable();
		return;
	}
	state = &task->critical;
	if (!state->depth || state->suspended)
		return;
	if (state->thread != current || state->mux_depth ||
	    state->depth != state->legacy_depth)
		panic("esp32s31-radio: sleeping/escaping while portMUX is held");
	/* Transitional legacy-only suspension is preserved. Object portMUX
	 * ownership is never silently released by a blocking bridge. */
	raw_spin_unlock(&s31_critical_lock);
	state->suspended = true;

	local_irq_restore(state->flags);
	preempt_enable();
	s31_radio_blob_run_pending_isrs();
}

void s31_linux_critical_resume(void)
{
	struct s31_linux_task *task = s31_linux_current_task();
	struct s31_critical_state *state;
	unsigned long flags;
	if (!task || in_interrupt())
		return;
	state = &task->critical;
	if (!state->depth || !state->suspended)
		return;
	if (state->thread != current || state->mux_depth ||
	    state->depth != state->legacy_depth)
		panic("esp32s31-radio: invalid legacy critical resume");
	preempt_disable();
	local_irq_save(flags);

	raw_spin_lock(&s31_critical_lock);
	state->flags = flags;
	state->suspended = false;
}
