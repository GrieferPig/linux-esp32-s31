// SPDX-License-Identifier: GPL-2.0
/* Linux half of the Wi-Fi blob's FreeRTOS compatibility bridge. */

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/cpu.h>
#include <linux/fpu.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/mm_types.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/mutex.h>
#include <linux/soc/espressif/esp32s31-pmu.h>

#include <linux/rtmutex.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/sched/s31_radio.h>
#include <linux/sched/s31_radio_stack.h>
#include <linux/sched/cputime.h>
#include <linux/sched/task_stack.h>
#include <linux/slab.h>
#include <linux/refcount.h>
#include <linux/spinlock.h>
#include <linux/task_work.h>
#include <linux/thread_info.h>
#include <linux/wait.h>

#include <asm/asm-offsets.h>
#include <asm/csr.h>
#include <asm/irq_regs.h>
#include <asm/irq_stack.h>
#include <asm/ptrace.h>
#include <asm/esp32s31_ext.h>
#include <asm/esp32s31_blob_fp.h>
#include <asm/esp32s31_radio_task.h>

#include "esp32s31-radio-internal.h"
#include "esp32s31-radio-xip.h"

extern void sched_show_task(struct task_struct *p);

/*
 * Run a payload continuation on the task's registered SRAM Linux stack
 * (top == 0), or an explicit SRAM stack for alternate-stack callers.
 * Preserve the nonlocal deletion continuation and trap anchor in either
 * case. Native task bodies and waits do not pivot stacks. Closed ISR
 * bodies may still need the explicit SRAM-stack entry.
 *
 * save[16]: 0=ra 1=sp 2..13=s0..s11 14=previous Linux trap anchor
 * 15=entry tp.
 * Saving the complete integer callee-saved set is required for the vTaskDelete(self) escape path, which
 * intentionally bypasses the payload's normal ABI unwinding.  tp is saved
 * and restored as well: closed code legitimately uses tp for its own TLS
 * and need not restore the kernel value on every return.  A leaked blob tp
 * would otherwise poison all tp-derived kernel state after the span
 * (thread_info/preempt_count, current, in_interrupt) and misdirect the
 * anchor write below into blob memory.  JTAG proof on a BT-bring-up panic
 * (tp=0xc0e0c800, blob "$SYSTEM_CONF" area, foreign-enter fence tripped on
 * garbage): save/restore contains any clobber to its own span and is
 * transparent to nesting (an inner span restores whatever tp was live,
 * kernel or outer-blob).
 */
__attribute__((naked)) noinline static void s31_payload_run(unsigned long *save,
							     unsigned long top,
							     void (*entry)(void *),
							     void *arg,
							     bool trap_to_linux)
{
	__asm__ volatile(
		"mv t0, a0\n\t"
		"sw s0, 8(t0)\n\t"
		"mv s0, t0\n\t"
		"sw ra, 0(s0)\n\t"
		"sw sp, 4(s0)\n\t"
		"sw s1, 12(s0)\n\t"
		"sw s2, 16(s0)\n\t"
		"sw s3, 20(s0)\n\t"
		"sw s4, 24(s0)\n\t"
		"sw s5, 28(s0)\n\t"
		"sw s6, 32(s0)\n\t"
		"sw s7, 36(s0)\n\t"
		"sw s8, 40(s0)\n\t"
		"sw s9, 44(s0)\n\t"
		"sw s10, 48(s0)\n\t"
		"sw s11, 52(s0)\n\t"
		"mv t0, tp\n\t"
		"sw t0, 60(s0)\n\t"
		"lw t0, " __stringify(TASK_TI_S31_BLOB_LINUX_SP) "(tp)\n\t"
		"sw t0, 56(s0)\n\t"
		/* Only an outer Linux -> SRAM BT pivot changes the anchor.
		 * SRAM -> Linux blocking bridges retain their parent's anchor. */
		"beqz a4, 1f\n\t"
		"li t0, 0x2f000000\n\t"
		"bltu a1, t0, 1f\n\t"
		"li t0, 0x2f400000\n\t"
		"bgeu a1, t0, 1f\n\t"
		"li t0, 0xc0000000\n\t"
		"bltu sp, t0, 1f\n\t"
		"sw sp, " __stringify(TASK_TI_S31_BLOB_LINUX_SP) "(tp)\n\t"
		"1:\n\t"
		"mv t1, a2\n\t"
		/* A zero top invokes on the registered SRAM Linux stack. Keep
		 * only the nonlocal task-exit/TP continuation, with no SP pivot. */
		"beqz a1, 2f\n\t"
		"mv sp, a1\n\t"
		"2:\n\t"
		"mv a0, a3\n\t"
		"jalr t1\n\t"
		"mv t0, s0\n\t"
		"lw ra, 0(t0)\n\t"
		"lw sp, 4(t0)\n\t"
		"lw s0, 8(t0)\n\t"
		"lw s1, 12(t0)\n\t"
		"lw s2, 16(t0)\n\t"
		"lw s3, 20(t0)\n\t"
		"lw s4, 24(t0)\n\t"
		"lw s5, 28(t0)\n\t"
		"lw s6, 32(t0)\n\t"
		"lw s7, 36(t0)\n\t"
		"lw s8, 40(t0)\n\t"
		"lw s9, 44(t0)\n\t"
		"lw s10, 48(t0)\n\t"
		"lw s11, 52(t0)\n\t"
		/* Restore only after SP is back on the caller's stack. tp first:
		 * the anchor write below must use the entry tp, never a blob
		 * value leaked by the span just run. */
		"lw t1, 60(t0)\n\t"
		"mv tp, t1\n\t"
		"lw t1, 56(t0)\n\t"
		"sw t1, " __stringify(TASK_TI_S31_BLOB_LINUX_SP) "(tp)\n\t"
		"ret\n\t");
}

/* Longjmp back to the trampoline's kthread stack (vTaskDelete(self)). */
__attribute__((naked)) noinline static void s31_payload_sp_restore(unsigned long *save)
{
	__asm__ volatile(
		"mv t0, a0\n\t"
		"lw ra, 0(t0)\n\t"
		"lw s0, 8(t0)\n\t"
		"lw s1, 12(t0)\n\t"
		"lw s2, 16(t0)\n\t"
		"lw s3, 20(t0)\n\t"
		"lw s4, 24(t0)\n\t"
		"lw s5, 28(t0)\n\t"
		"lw s6, 32(t0)\n\t"
		"lw s7, 36(t0)\n\t"
		"lw s8, 40(t0)\n\t"
		"lw s9, 44(t0)\n\t"
		"lw s10, 48(t0)\n\t"
		"lw s11, 52(t0)\n\t"
		"lw sp, 4(t0)\n\t"
		/* The deleter runs inside payload with the blob tp live; the
		 * anchor write below must use the entry tp saved at span
		 * start (slot 60), or it corrupts blob memory. */
		"lw t1, 60(t0)\n\t"
		"mv tp, t1\n\t"
		"lw t1, 56(t0)\n\t"
		"sw t1, " __stringify(TASK_TI_S31_BLOB_LINUX_SP) "(tp)\n\t"
		"ret\n\t");
}

#define S31_LINUX_TASK_MAGIC 0x53333154U

/* Timer IRQs can interrupt a userspace mm. Low payload SRAM is mapped
 * only in init_mm, so every asynchronously dereferenced wait object must
 * be allocated in ordinary, globally mapped kernel memory. */

#include "esp32s31-radio-critical-types.h"

struct s31_linux_task {
	u32 magic;
	struct task_struct *thread;
	void (*entry)(void *);
	void *arg;
	void *cookie;
    void (*finalize)(void *);
    bool finalizing;
    bool payload_continuation_complete;
    bool finalizer_reported;
    bool runtime_wait_cancel_allowed;
    s32 finalizer_status;
	struct completion exited;
	struct completion started;
	struct completion creator_done;
	atomic_t stopping;
	bool blob_active;
	bool on_kernel_stack;
	bool exit_preempt_guard;
	u32 sync_lock_depth;
	unsigned long sync_irq_flags;
	struct s31_blob_context blob;
	int errno_value;
	/* Payload execution stack in internal SRAM.  The PHY changes the
	 * external-memory/cache state while the payload runs, so the DRAM
	 * kthread stack is unusable for payload code. */
	void *payload_stack;
	struct s31_stack_lease *stack_lease;
	u32 payload_stack_size;
	unsigned long sp_save[16];
	/* FP checkpoint/identity live in globally mapped thread_info state. */
	s32 requested_cpu;
	struct s31_linux_task *next;
	struct s31_critical_state critical;

};

/* Coherent native object/scheduler contract. */

static bool s31_native_mode_queried;
static bool s31_aux_context_queried;

struct s31_native_mutex;
struct s31_linux_sync;
static unsigned long s31_native_mutex_owner_op(struct s31_linux_sync *sync, u32 op);
static long s31_native_object_call(struct s31_linux_sync *sync, u32 op, u32 ticks);
static void s31_native_mutex_assert_current_quiescent(void);
static void s31_native_mutex_retire_all(void);
struct s31_foreign_owner;
static int s31_service_priority_setup(struct task_struct *thread);
static void s31_service_priority_retire_current(void);

struct s31_linux_sync {
	raw_spinlock_t lock;
	atomic_t sequence;
	union {
		wait_queue_head_t waitq;
		struct s31_native_mutex *native;
	};
	struct task_struct *owner_thread;
	void *owner_cookie;
	bool owner_abandoned;
	bool native_mutex;
	struct list_head owner_node;
};

static bool s31_native_sync_enabled(void *opaque)
{
	return opaque && ((struct s31_linux_sync *)opaque)->native_mutex;
}

#include "esp32s31-radio-sync-owner.inc"

static struct s31_blob_context *s31_blob_context_current(void);

static DEFINE_PER_CPU(struct s31_blob_context *, s31_foreign_installed);
/* Cold control-only initialization has no vendor producers. The exact Linux
 * owner and stack-local context define its non-sleeping exclusive scope.
 * No managed task stores extra bootstrap state in scarce SRAM.
 */
static struct task_struct *s31_control_bootstrap_thread;
static struct s31_blob_context *s31_control_bootstrap_context;

bool s31_linux_control_bootstrap_active(void)
{
 return smp_load_acquire(&s31_control_bootstrap_thread) != NULL;
}

static bool s31_control_bootstrap_owns(struct s31_blob_context *context)
{
 return context &&
        smp_load_acquire(&s31_control_bootstrap_thread) == current &&
        READ_ONCE(s31_control_bootstrap_context) == context;
}

struct s31_blob_fp_owner {
	/* Foreign-only identity; never keep a low SRAM task pointer here. */
	struct s31_blob_context *ctx;
};
static DEFINE_PER_CPU(struct s31_blob_fp_owner, s31_blob_fp_owner);
static noinline void s31_payload_fp_save_area(u32 *fp_saved, bool *fp_valid);
static noinline void s31_payload_fp_restore_area(u32 *fp_saved, bool fp_valid);
/* Foreign-context contract: compatibility tasks own their blob state in
 * their s31_linux_task (follows the task across migration and sleep).
 * Every other invocation (ordered-workqueue runner, command handling, RX
 * throttle, foreign sleep path) supplies its own
 * per-invocation descriptor (see esp32s31-radio-internal.h): the per-CPU
 * installed pointer above is the only shared foreign state, and it is NULL
 * across every suspend/sleep gap, so a second foreign thread entering on
 * the same CPU uses its own storage and can neither read nor clobber the
 * sleeper's saved bank. Explicit architecture admission identifies our
 * compatibility tasks; unrelated kthread_data is never interpreted here.
 */
static atomic_t s31_sync_stop_generation = ATOMIC_INIT(0);
/* Runtime cancellation is distinct from a target task's stop notification.
 * Entries live on globally mapped Linux stacks, never suspended SRAM stacks.
 * The registry lock pins each entry/thread through publication or removal.
 */
struct s31_runtime_waiter {
	struct list_head node;
	struct task_struct *thread;
};
static LIST_HEAD(s31_runtime_waiters);
static DEFINE_SPINLOCK(s31_runtime_waiters_lock);
static atomic_t s31_runtime_cancel_generation = ATOMIC_INIT(0);
static unsigned int s31_runtime_wait_active;

static struct s31_linux_task *s31_linux_current_task(void);
static DEFINE_RAW_SPINLOCK(s31_critical_lock);

/* Native scheduler and persistent context ownership are mandatory. */

static bool s31_stack_minimum_queried;

static bool s31_waitqueue_needs_wake(wait_queue_head_t *wq)
{
	/* The full barrier pairs with waiter publication. A waiter not yet
	 * listed must observe the changed condition before sleeping.
	 */
	return wq_has_sleeper(wq);
}

static struct s31_linux_task *s31_task_list;
static DEFINE_SPINLOCK(s31_task_list_lock);
/* Finalized-create publication and shutdown selection share this short mutex.
 * Never hold it across stop/join or payload callbacks. A listed finalized task
 * must enter thread_fn before kthread_stop can select it; otherwise Linux can
 * skip thread_fn entirely and leave the published payload cookie unretired. */
static DEFINE_MUTEX(s31_task_start_lock);

struct s31_stack_lease {
	struct list_head node;
	struct esp32s31_radio_stack *kernel;
	void *base;
	bool retired;
};
static LIST_HEAD(s31_stack_leases);
static DEFINE_SPINLOCK(s31_stack_leases_lock);
static DEFINE_MUTEX(s31_stack_reap_lock);

static void s31_stack_leases_reap(bool wait)
{
	struct s31_stack_lease *lease, *next;
	unsigned long flags;
	LIST_HEAD(done);

	mutex_lock(&s31_stack_reap_lock);
	spin_lock_irqsave(&s31_stack_leases_lock, flags);
	list_for_each_entry_safe(lease, next, &s31_stack_leases, node) {
		if (lease->retired &&
		    (wait || esp32s31_radio_stack_released(lease->kernel)))
			list_move_tail(&lease->node, &done);
	}
	spin_unlock_irqrestore(&s31_stack_leases_lock, flags);
	list_for_each_entry_safe(lease, next, &done, node) {
		if (esp32s31_radio_stack_destroy(lease->kernel))
			panic("s31 radio: native stack join refused");
		s31_radio_sram_stack_free(lease->base, THREAD_SIZE);
		list_del(&lease->node);
		kfree(lease);
	}
	mutex_unlock(&s31_stack_reap_lock);
}
/* A create reservation precedes kernel descriptor/native waiter allocation,
 * kthread creation and FP attachment. Payload TCB/contexts/stack were allocated
 * before this import; their runtime entry/heap lifetime is a separate contract.
 * Closing refuses new reservations and waits existing creators to publish/start
 * or fail, without the startup mutex. No external FP detach is needed. */
static bool s31_tasks_accepting;
static int *s31_errno_slot;
static struct esp32s31_radio_domain *s31_native_domain;
#include "esp32s31-radio-service-context-types.h"
static atomic_t s31_tasks_creating = ATOMIC_INIT(0);
static DECLARE_WAIT_QUEUE_HEAD(s31_tasks_create_waitq);

static bool s31_task_create_admit(void)
{
	bool admitted;

	mutex_lock(&s31_task_start_lock);
	admitted = s31_tasks_accepting && !s31_linux_control_bootstrap_active();
	if (admitted)
		atomic_inc(&s31_tasks_creating);
	mutex_unlock(&s31_task_start_lock);
	return admitted;
}

static void s31_task_create_complete(void)
{
	bool drained;

	mutex_lock(&s31_task_start_lock);
	if (WARN_ON_ONCE(!atomic_read(&s31_tasks_creating)))
		panic("esp32s31-radio: unmatched task create reservation");
	drained = atomic_dec_and_test(&s31_tasks_creating);
	mutex_unlock(&s31_task_start_lock);
	if (drained)
		wake_up_all(&s31_tasks_create_waitq);
}

/* Called after the payload bootstrap returns, before any vendor task starts.
 * ABI-1 alone does not distinguish older queue shims which ignore native PI.
 * Refuse that combination instead of claiming a contract never negotiated.
 */
int s31_linux_native_contract_ready(void)
{
	if ((!READ_ONCE(s31_native_mode_queried))) {
		pr_err("esp32s31-radio: payload lacks native mutex negotiation; update radio image\n");
		return -EPROTONOSUPPORT;
	}
    if (!READ_ONCE(s31_aux_context_queried)) {
        pr_err("esp32s31-radio: payload lacks explicit IRQ/bootstrap identity contract\n");
        return -EPROTONOSUPPORT;
    }
	if (!s31_stack_minimum_queried) {
		pr_err("esp32s31-radio: payload lacks headerless native SRAM stack negotiation\n");
		return -EPROTONOSUPPORT;
	}
	return 0;
}

int s31_linux_tasks_runtime_open(void)
{
	struct esp32s31_radio_domain *domain = NULL;

	/* Strict scheduling needs the complete persistent native contract and the
	 * owned role/placement facilities. Refuse a partial or unsupported
	 * selection instead of silently dropping scheduler admission. */
	{

		if (!IS_ENABLED(CONFIG_ESP32S31_RADIO_SCHEDULER))
			return -EOPNOTSUPP;
	}

	if ((((!IS_ENABLED(CONFIG_PREEMPTION)))))
        return -EINVAL;
    s31_stack_minimum_queried = false;

	WRITE_ONCE(s31_native_mode_queried, false);
    WRITE_ONCE(s31_aux_context_queried, false);

	{
		domain = esp32s31_radio_domain_create(true);
		if (IS_ERR(domain))
			return PTR_ERR(domain);
		/* Lease the placement domain on the fresh unpublished root before
		 * any owner/resource reference or publication. A later retirement
		 * consumes the lease exactly once in domain destroy. */

	}
	mutex_lock(&s31_task_start_lock);
	spin_lock(&s31_task_list_lock);
	if (s31_tasks_accepting || atomic_read(&s31_tasks_creating) || s31_task_list || s31_native_domain)
		panic("esp32s31-radio: task admission reopened before quiescence");
	/* No live owner survives runtime-open; the new payload registers its cell. */
	WRITE_ONCE(s31_errno_slot, NULL);
	WRITE_ONCE(s31_native_domain, domain);
	s31_tasks_accepting = true;
	spin_unlock(&s31_task_list_lock);
	mutex_unlock(&s31_task_start_lock);
	return 0;
}

static int s31_native_domain_get(unsigned int kind, struct esp32s31_radio_domain **lease)
{
	struct esp32s31_radio_domain *domain = READ_ONCE(s31_native_domain);
	int ret;

	if ((!domain)) {
		*lease = NULL;
		return -ESHUTDOWN;
	}
	ret = domain ? esp32s31_radio_domain_get(domain, kind) : 0;

	*lease = ret ? NULL : domain;
	return ret;
}

static void s31_native_domain_put(struct esp32s31_radio_domain *domain, unsigned int kind)
{
	if (domain && esp32s31_radio_domain_put(domain, kind))
		panic("esp32s31-radio: invalid domain resource release");
}

int s31_linux_domain_irq_get(struct esp32s31_radio_domain **lease)
{
	return s31_native_domain_get(ESP32S31_RADIO_DOMAIN_IRQ, lease);
}

void s31_linux_domain_irq_put(struct esp32s31_radio_domain *lease)
{
	s31_native_domain_put(lease, ESP32S31_RADIO_DOMAIN_IRQ);
}

void s31_linux_tasks_runtime_close(void)
{
	struct esp32s31_radio_domain *domain = READ_ONCE(s31_native_domain);
	unsigned long flags;
	bool waiters_live;

	if (s31_linux_control_bootstrap_active() ||
	    READ_ONCE(s31_control_bootstrap_context))
		panic("esp32s31-radio: runtime close retains bootstrap scope");

	/* Managed, service and foreign executors are joined by the caller. */
	spin_lock_irqsave(&s31_runtime_waiters_lock, flags);
	waiters_live = s31_runtime_wait_active || !list_empty(&s31_runtime_waiters);
	spin_unlock_irqrestore(&s31_runtime_waiters_lock, flags);
	if (waiters_live)
		panic("esp32s31-radio: runtime close retains registered waits");

	if (domain) {
		esp32s31_radio_domain_close(domain);
		/* Private service and IRQ executors have already retired. */
		if (esp32s31_radio_domain_join_tasks(domain))
			panic("esp32s31-radio: final kernel task exit join refused");
		/* Private payload reclamation must also join Linux stack readers
		 * and the last stack reference's RCU callback. */
		s31_stack_leases_reap(true);
		if (!list_empty(&s31_stack_leases))
			panic("s31 radio: runtime retains a live SRAM task stack");
		if (esp32s31_radio_domain_destroy(domain))
			panic("esp32s31-radio: runtime domain still has live owners");
		WRITE_ONCE(s31_native_domain, NULL);

	}
}

static void s31_linux_tasks_quiesce_creates(void)
{
	mutex_lock(&s31_task_start_lock);
	s31_tasks_accepting = false;
	mutex_unlock(&s31_task_start_lock);
	/* An admitted creator completes before blob_resume: it does not need the
	 * gate or a payload callback to release its reservation. */
	wait_event(s31_tasks_create_waitq, !atomic_read(&s31_tasks_creating));
	if (s31_native_domain)
		esp32s31_radio_domain_close(s31_native_domain);
}

/*
 * Radio payloads run only on dedicated PF_KTHREAD contexts.  They have no
 * userspace FP state to preserve, while s31_payload_fp_save_area() already
 * retains the ILP32F callee-saved state across every payload blocking point.
 * Avoid the generic kernel_fpu_begin()/end() pair here: it saves and restores
 * all 32 registers on every BTDM queue wake even though the surrounding
 * kthread never consumes that state. Native task scheduling preserves the
 * task bank. IDF forbids FP in ISRs: the direct hardirq path leaves FS off,
 * and a deferred ISR temporarily disables FS without copying registers.
 *
 * The pinned IDF radio objects contain no Xesploop or Xespv instructions, so
 * these kthreads deliberately do not opt into esp32s31_ext_kernel_begin().
 * User extension state is still suspended on kernel entry and managed by the
 * architecture code; paying two SBI transitions at every payload wait would
 * only save and restore an extension state that the radio never consumes.
 */

static void s31_blob_fpu_begin(void)
{
	struct s31_linux_task *task = s31_linux_current_task();
	struct s31_blob_context *context = s31_blob_context_current();
	struct s31_blob_fp_owner *owner;
	bool native = task || s31_current_service();

	if (!native && !context)
		panic("esp32s31-radio: foreign FP begin with no installed window");

	if (!(current->flags & PF_KTHREAD))
		panic("esp32s31-radio: blob FP entered outside a kthread");
	/* Registered task stacks and FP/errno banks are scheduler-owned from
	 * first publication through finalization, including cold startup. A
	 * readiness-dependent preemption fence makes their native waits illegal
	 * and formerly forced a suspend/pivot fallback. Actual cache-off and
	 * critical spans have their own bounded guards. Foreign/service windows
	 * still require their separate per-CPU ownership fence. */
	if (!task) {
		preempt_disable();
		context->preempt_fenced = true;
	}
	/* Cache-on bridge boundary, before installed state/closed payload entry.
	 * Foreign windows are marked even when the current payload is integer-only.
	 */
	if (!native)
		esp32s31_blob_fp_foreign_enter();
	if (native) {
		esp32s31_blob_fp_enter();
		return;
	}
	/* Foreign FP remains pinned/fenced and uses its existing SRAM area.
	 * Full-file FP ownership for fenced Wi-Fi windows.  The switch hook
	 * only checks this fence; it cannot checkpoint after generic FP restore.
	 * End/suspend checkpoint the current owner. A leaked or changed owner
	 * is fatal rather than being saved after an unknown clobber. Owner
	 * state stays per-CPU for architecture hooks; blob entry is core0-only. */
	owner = this_cpu_ptr(&s31_blob_fp_owner);
	if (owner->ctx)
		panic("esp32s31-radio: foreign FP owner was not released");
	owner->ctx = context;
	/* Never start with a previous task/user's physical file. Re-entry after
	 * a blocking bridge restores this invocation's complete checkpoint. */
	if (!context->fp_valid) {
		memset(context->fp_saved, 0, sizeof(context->fp_saved));
		context->fp_valid = true;
	}
	s31_payload_fp_restore_area(context->fp_saved, context->fp_valid);
}

static void s31_blob_fpu_end(void)
{
	struct s31_linux_task *task = s31_linux_current_task();
	struct s31_blob_context *context = s31_blob_context_current();
	struct s31_blob_fp_owner *owner;
	bool native = task || s31_current_service();
	if (!native && !context)
		panic("esp32s31-radio: foreign FP end with no installed window");
	if (native) {
		esp32s31_blob_fp_pause();
		return;
	}
	owner = this_cpu_ptr(&s31_blob_fp_owner);
	if (owner->ctx != context)
		panic("esp32s31-radio: foreign FP checkpoint owner mismatch");
	s31_payload_fp_save_area(context->fp_saved, &context->fp_valid);
	owner->ctx = NULL;
	csr_clear(CSR_SSTATUS, SR_FS);
}

static void s31_blob_unfence_context(struct s31_blob_context *context)
{
	/* Clear ownership before a pending reschedule can run another borrower.
	 * Callers have already restored context/active state and released the gate.
	 */
	if (context->preempt_fenced) {
		context->preempt_fenced = false;
		preempt_enable();
	}
}

/* Called after switch_mm(). SRAM is globally mapped; the scheduler-owned
 * FP bank remains authoritative across native task switches. Foreign
 * execution keeps its separate fencing and ownership checks.
 */
void s31_blob_fp_context_switch(struct task_struct *prev,
				struct task_struct *next)
{
	struct s31_blob_fp_owner *owner = this_cpu_ptr(&s31_blob_fp_owner);
	struct s31_blob_context *foreign =
		this_cpu_read(s31_foreign_installed);

	if (prev == READ_ONCE(s31_service_context.thread) &&
	    READ_ONCE(s31_service_context.active))
		panic("esp32s31-radio: cache-fenced service window scheduled");
	if (owner->ctx || (foreign && foreign->stack_switched))
		panic("esp32s31-radio: fenced foreign window scheduled");
}

/* Bounded polling retains the real native owner and payload stack. */

static unsigned long s31_ticks_to_jiffies(u32 ticks)
{
	u64 j;

	if (ticks == 0)
		return 0;
	if (ticks == 0xffffffffU)
		return MAX_SCHEDULE_TIMEOUT;
	j = DIV_ROUND_UP_ULL((u64)ticks * HZ, 100);
	return min_t(u64, j, MAX_SCHEDULE_TIMEOUT - 1);
}

/* One 100 Hz monotonic epoch for native tick reads and blocking deadlines.
 * Capture expiry before suspending/pivoting: Linux timer-wheel rounding and
 * bridge time must not restart a caller's remaining timeout. */
#define S31_NATIVE_TICK_NS 10000000ULL

u32 s31_linux_tick_count(void)
{
 return (u32)div_u64(ktime_get_ns(), S31_NATIVE_TICK_NS);
}

static ktime_t s31_native_tick_deadline(u32 ticks)
{
 u64 now;
 if (!ticks) return 0;
 if (ticks == U32_MAX) return KTIME_MAX;
 now = ktime_get_ns();
 return ns_to_ktime((div_u64(now, S31_NATIVE_TICK_NS) + ticks) * S31_NATIVE_TICK_NS);
}

static struct s31_linux_task *s31_linux_current_task(void)
{

	struct s31_linux_task *task;

	/* Only kernel-admitted radio tasks carry this identity. Never probe an
	 * arbitrary kthread's private data for a coincidentally matching magic. */
	task = esp32s31_radio_task_current_data(READ_ONCE(s31_native_domain));
	if (!task || s31_service_data(task)) {
		return NULL; /* Never interpret the service as a managed stop frame. */
	}
	if (task->magic != S31_LINUX_TASK_MAGIC || task->thread != current)
		panic("esp32s31-radio: admitted task identity corrupted");
	return task;
}

static noinline void s31_payload_fp_save_area(u32 *fp_saved, bool *fp_valid)
{

	/* RISC-V trap entry clears sstatus.FS.  A blocking bridge call may be
	 * reached from a native IDF ISR, so do not rely on the task-side FPU gate
	 * state surviving that trap.  Saves the full file (same bank order as
	 * the direct-ISR area) so a window interrupted at any instruction can
	 * be restored exactly. */
	csr_set(CSR_SSTATUS, SR_FS);
	asm volatile(
		"fsw ft0, 0(%0)\n\t"   "fsw ft1, 4(%0)\n\t"
		"fsw ft2, 8(%0)\n\t"   "fsw ft3, 12(%0)\n\t"
		"fsw ft4, 16(%0)\n\t"  "fsw ft5, 20(%0)\n\t"
		"fsw ft6, 24(%0)\n\t"  "fsw ft7, 28(%0)\n\t"
		"fsw fs0, 32(%0)\n\t"  "fsw fs1, 36(%0)\n\t"
		"fsw fa0, 40(%0)\n\t"  "fsw fa1, 44(%0)\n\t"
		"fsw fa2, 48(%0)\n\t"  "fsw fa3, 52(%0)\n\t"
		"fsw fa4, 56(%0)\n\t"  "fsw fa5, 60(%0)\n\t"
		"fsw fa6, 64(%0)\n\t"  "fsw fa7, 68(%0)\n\t"
		"fsw fs2, 72(%0)\n\t"  "fsw fs3, 76(%0)\n\t"
		"fsw fs4, 80(%0)\n\t"  "fsw fs5, 84(%0)\n\t"
		"fsw fs6, 88(%0)\n\t"  "fsw fs7, 92(%0)\n\t"
		"fsw fs8, 96(%0)\n\t"  "fsw fs9, 100(%0)\n\t"
		"fsw fs10, 104(%0)\n\t" "fsw fs11, 108(%0)\n\t"
		"fsw ft8, 112(%0)\n\t" "fsw ft9, 116(%0)\n\t"
		"fsw ft10, 120(%0)\n\t" "fsw ft11, 124(%0)\n\t"
		"frcsr t0\n\t" "sw t0, 128(%0)\n\t"
		: : "r" (fp_saved) : "t0", "memory");
	*fp_valid = true;
}

static noinline void s31_payload_fp_restore_area(u32 *fp_saved, bool fp_valid)
{

	if (!fp_valid)
		return;
	csr_set(CSR_SSTATUS, SR_FS);
	asm volatile(
		"flw ft0, 0(%0)\n\t"   "flw ft1, 4(%0)\n\t"
		"flw ft2, 8(%0)\n\t"   "flw ft3, 12(%0)\n\t"
		"flw ft4, 16(%0)\n\t"  "flw ft5, 20(%0)\n\t"
		"flw ft6, 24(%0)\n\t"  "flw ft7, 28(%0)\n\t"
		"flw fs0, 32(%0)\n\t"  "flw fs1, 36(%0)\n\t"
		"flw fa0, 40(%0)\n\t"  "flw fa1, 44(%0)\n\t"
		"flw fa2, 48(%0)\n\t"  "flw fa3, 52(%0)\n\t"
		"flw fa4, 56(%0)\n\t"  "flw fa5, 60(%0)\n\t"
		"flw fa6, 64(%0)\n\t"  "flw fa7, 68(%0)\n\t"
		"flw fs2, 72(%0)\n\t"  "flw fs3, 76(%0)\n\t"
		"flw fs4, 80(%0)\n\t"  "flw fs5, 84(%0)\n\t"
		"flw fs6, 88(%0)\n\t"  "flw fs7, 92(%0)\n\t"
		"flw fs8, 96(%0)\n\t"  "flw fs9, 100(%0)\n\t"
		"flw fs10, 104(%0)\n\t" "flw fs11, 108(%0)\n\t"
		"flw ft8, 112(%0)\n\t" "flw ft9, 116(%0)\n\t"
		"flw ft10, 120(%0)\n\t" "flw ft11, 124(%0)\n\t"
		"lw t0, 128(%0)\n\t" "fscsr t0\n\t"
		: : "r" (fp_saved) : "t0", "memory");
}

/* Registered native stacks, FP and errno banks already follow scheduling.
 * A cache-on Linux call therefore retains its persistent vendor context.
 * The narrow marker prevents a closed ISR reentering kernel primitive state;
 * it neither serializes other tasks nor disables Linux IRQs/preemption.
 * Held critical/IRQ/atomic sections are refused before any sleeping call. */
static bool s31_task_native_linux_call(struct s31_linux_task *task,
 void (*entry)(void *), void *arg)
{

 bool previous;

 if (!task || !task->blob_active || task->blob.preempt_fenced ||
     task->critical.depth || task->critical.suspended || task->critical.mux_depth || task->sync_lock_depth ||
     in_interrupt() || in_atomic() || irqs_disabled() ||
     current->stack != task->payload_stack || !object_is_on_stack(&previous)) {
  pr_err_ratelimited("esp32s31-radio: native call rejected task=%u active=%u fenced=%u critical=%u suspended=%u depth=%u mux=%u sync=%u preempt=%x irqoff=%u stack=%u\n",
   !!task, task && task->blob_active, task && task->blob.preempt_fenced,
   task && task->critical.depth, task && task->critical.suspended,
   task ? task->critical.depth : 0, task ? task->critical.mux_depth : 0,
   task ? task->sync_lock_depth : 0, preempt_count(), irqs_disabled(),
   task && current->stack == task->payload_stack);
  return false;
 }

 if (task_stack_end_corrupted(current))
  panic("esp32s31-radio: native Linux call stack guard exhausted");
 previous = task->on_kernel_stack;
 WRITE_ONCE(task->on_kernel_stack, true);
 barrier();

 entry(arg);

 barrier();
 WRITE_ONCE(task->on_kernel_stack, previous);
 return true;
}

/* Publish one per-invocation foreign window as the installed window of this
 * CPU. Suspend pops it (NULL) and hands the descriptor back to the sleeper;
 * resume reinstalls exactly that descriptor. Enter and resume fail-stop when
 * a window is already installed: installation always requires the blob gate,
 * so a second installation can only mean a broken suspend/resume pairing. */
static void s31_foreign_publish(struct s31_blob_context *ctx)
{
	if (WARN_ON_ONCE(this_cpu_read(s31_foreign_installed)))
		panic("esp32s31-radio: foreign window already installed");
	this_cpu_write(s31_foreign_installed, ctx);
}

static struct s31_blob_context *s31_blob_context_current(void)
{
	struct s31_linux_task *task = s31_linux_current_task();

	/* Foreign callers see the installed per-invocation window, or NULL
	 * when no foreign window is installed on this CPU (in particular
	 * across every suspend/sleep gap). Callers that dereference must
	 * either hold the installation or tolerate NULL explicitly. */
	if (task) return &task->blob;
	if (s31_current_service()) return &s31_service_context.blob;
	return this_cpu_read(s31_foreign_installed);
}

static bool s31_blob_active(void)
{
	struct s31_linux_task *task = s31_linux_current_task();
	struct s31_blob_context *foreign;

	if (task)
		return task->blob_active;
	if (s31_current_service()) return s31_service_context.active;
	foreign = this_cpu_read(s31_foreign_installed);
	return foreign && foreign->stack_switched;
}

static void s31_blob_set_active(bool active)
{
	struct s31_linux_task *task = s31_linux_current_task();

	if (task) {
		task->blob_active = active;
		return;
	}
	if (s31_current_service()) {
		s31_service_context.active = active;
		return;
	}
	if (active) {
		/* Activation follows installation: enter and resume publish
		 * before reaching here. */
		if (WARN_ON_ONCE(!this_cpu_read(s31_foreign_installed)))
			panic("esp32s31-radio: foreign activate with no window");
	} else {
		/* Leave and suspend pop the installation; the descriptor
		 * itself stays owned by its invocation until leave. */
		this_cpu_write(s31_foreign_installed, NULL);
	}
}

/* Native task banks follow Linux scheduling; foreign windows and nested IRQs
 * retain explicit installation/checkpoint boundaries. */
#include "esp32s31-radio-errno.inc"

static void s31_blob_install_context(struct s31_blob_context *context)
{
	/* Keep the existing compatibility name for the installed-window flag.
	 * Exception-stack selection belongs to entry.S/the trampoline anchor.
	 */
	s31_errno_enter(context);
	context->stack_switched = true;
}

static void s31_blob_restore_context(struct s31_blob_context *context)
{
	if (!context->stack_switched)
		return;
	s31_errno_leave(context);
	if (context->irq_disabled)
		arch_local_irq_restore(context->irq_flags);
	context->irq_disabled = false;
	context->stack_switched = false;
}

static void s31_linux_task_unregister(struct s31_linux_task *task)
{
	struct s31_linux_task **link;

	spin_lock(&s31_task_list_lock);
	for (link = &s31_task_list; *link; link = &(*link)->next) {
		if (*link == task) {
			*link = task->next;
			break;
		}
	}
	spin_unlock(&s31_task_list_lock);
}

static void s31_linux_task_cleanup(struct s31_linux_task *task)
{
	if (task->blob_active) {
		s31_blob_fpu_end();
		s31_blob_restore_context(&task->blob);
		task->blob_active = false;
		/* Abnormal exit still releases only resources actually owned. */

		s31_blob_unfence_context(&task->blob);
	}
	complete(&task->exited);
}

const char *s31_linux_blob_holder(void)
{
 return s31_blob_active() ? current->comm : NULL;
}

bool s31_linux_blob_held_by_current(void)
{
	/* Callable from hard IRQ context: current is the interrupted task.
	 * Persistent native ownership proves managed identity;
	 * a persistent managed/service task owns its own active blob window
	 * without the global gate, so the same proof is used everywhere. */
	struct s31_linux_task *task;

	if (s31_control_bootstrap_owns(this_cpu_read(s31_foreign_installed)))
		return true;

	task = s31_linux_current_task();
	if (task)
		return task->blob_active;
	return s31_current_service() && s31_service_context.active;
}

/* Physical internal SRAM from ESP32-S31 SOC_IRAM/DRAM_LOW/HIGH.
 * The larger 4MiB Sv32 identity window does not prove cache-safe storage. */
#define S31_INTERNAL_SRAM_START 0x2f000000UL
#define S31_INTERNAL_SRAM_END   0x2f080000UL

/* Closed callbacks use the current registered SRAM stack. There is no
 * high-stack-to-payload-stack pivot or alternate continuation to select. */
static unsigned long s31_closed_isr_sp(void)
{
	unsigned long sp;

	asm volatile("mv %0, sp" : "=r" (sp));
	return sp;
}

bool s31_linux_closed_isr_context_safe(void)
{
	struct s31_linux_task *task = s31_linux_current_task();
	unsigned long sp = s31_closed_isr_sp();
	unsigned long base, top;

	if (raw_smp_processor_id() != 0 || !s31_linux_blob_held_by_current())
		return false;
	if (!task)
		return sp >= S31_INTERNAL_SRAM_START && sp < S31_INTERNAL_SRAM_END;
	base = (unsigned long)task->payload_stack;
	top = base + task->payload_stack_size;
	if (base < S31_INTERNAL_SRAM_START || top > S31_INTERNAL_SRAM_END ||
	    top <= base || task->payload_stack_size < 2048)
		return false;
	if (sp > base + 2048 && sp <= top)
		return true;
	return false;
}

/* Contain a closed span's tp use when it runs without the payload_run
 * trampoline (direct handler() calls below). Closed code legitimately uses
 * tp for its own TLS; without a guard a leaked blob tp poisons subsequent
 * tp-derived kernel state (current, preempt_count, in_interrupt) until the
 * next context switch or trap. Transparent to nesting: whatever tp is live
 * (kernel or outer-blob) is restored. */
static inline unsigned long s31_tp_guard_save(void)
{
	unsigned long tp;

	__asm__ volatile("mv %0, tp" : "=r" (tp));
	return tp;
}

static inline void s31_tp_guard_restore(unsigned long tp)
{
	__asm__ volatile("mv tp, %0" :: "r" (tp));
}

bool s31_linux_closed_isr_call(void (*handler)(void *), void *arg)
{
	unsigned long fs, tp;
	int errno_before;

	if (!s31_linux_closed_isr_context_safe())
		return false;
	errno_before = s31_errno_irq_save();
	/* A deferred IDF ISR borrows a task with a live FP bank. Match IDF
	 * rtos_int_enter(): forbid FP, leave the register file untouched, then
	 * restore the interrupted task's exact FS state with IRQs still off. */
	fs = csr_read_clear(CSR_SSTATUS, SR_FS) & SR_FS;
	tp = s31_tp_guard_save();
	handler(arg);
	s31_tp_guard_restore(tp);
	if (fs)
		csr_set(CSR_SSTATUS, fs);
	s31_errno_irq_restore(errno_before);
	return true;
}

bool s31_linux_task_is_finalizing(void)
{
	struct s31_linux_task *task = s31_linux_current_task();

	return task && READ_ONCE(task->finalizing);
}

static bool s31_irq_critical_quiescent(void);

/* Every admitted hardirq runs on CPU0's leased Linux SRAM IRQ stack.
 * Linux trap entry already disables FS. IDF forbids FP in ISRs, so leave
 * the interrupted FP file/FCSR untouched. Preserve errno independently of
 * the interrupted owner; no init_mm borrow, FP-owner publication or gate.
 * Handler registration/retirement and source ack remain the IRQ layer's job. */
int s31_linux_blob_run_direct_isr(void (*handler)(void *), void *arg)
{
	struct s31_linux_task *task;

	unsigned long tp;
	unsigned int count, sync_depth;
	int errno_before;

	if (!handler || in_nmi() || !in_hardirq() || !irqs_disabled() ||
	    !esp32s31_radio_on_irq_stack())
		return S31_DIRECT_ISR_DEFER_CONTEXT;
	if (READ_ONCE(s31_rtos_isr_depth))
		return S31_DIRECT_ISR_DEFER_NESTED;
	task = s31_linux_current_task();
	if (task && ((task->critical.depth && !task->critical.suspended) ||
		     READ_ONCE(task->sync_lock_depth)))
		return S31_DIRECT_ISR_DEFER_UNSAFE;
	/* An active service/metadata span can hold vendor locks without managed
	 * depth publication. The same service thread outside a vendor window
	 * is an ordinary Linux owner and may take the independent direct ISR. */
	if (!task && ((s31_current_service() && s31_service_context.active) ||
		     this_cpu_read(s31_foreign_installed)))
		return S31_DIRECT_ISR_DEFER_OWNER;
	if (task && (!s31_blob_active() || task->finalizing))
		return S31_DIRECT_ISR_DEFER_UNSAFE;

	count = preempt_count();
	sync_depth = task ? task->sync_lock_depth : 0;
	errno_before = s31_errno_irq_save();
	tp = s31_tp_guard_save();
	s31_rtos_isr_depth++;

	handler(arg);
	s31_tp_guard_restore(tp);

	if (!irqs_disabled() || preempt_count() != count ||
	    s31_rtos_isr_depth != 1 ||
	    (task && task->sync_lock_depth != sync_depth) ||
	    !s31_irq_critical_quiescent())
		panic("esp32s31-radio: native ISR escaped its IRQ ownership");
	s31_rtos_isr_depth--;
	s31_errno_irq_restore(errno_before);
	return S31_DIRECT_ISR_HANDLED;
}

uint64_t s31_linux_time_ns(void)
{

	return ktime_get_mono_fast_ns();
}

int32_t s31_linux_in_interrupt(void)
{
	/* A payload ISR depth of zero is not proof of process context.
	 * in_interrupt excludes ordinary task preemption fences. */
	return in_interrupt() ? 1 : 0;
}

int32_t s31_linux_current_cpu(void)
{
	/* Hart identity is stable only with preemption fenced: inside a
	 * critical section (which fences for itself) or a fenced window.
	 * BT-only blob windows are preemptible since C3p1, so portMUX users
	 * must query this strictly within critical_enter/exit.  A raw value
	 * anywhere else may already belong to another hart. */
	return raw_smp_processor_id();
}

void s31_linux_vprintf(const char *fmt, void *arguments)
{
    va_list args = arguments;

    _Static_assert(sizeof(va_list) == sizeof(void *), "RV32 va_list import ABI");
    vprintk(fmt, args);
}

void s31_linux_task_dump_all(void)
{
	struct s31_linux_task *task;

	spin_lock(&s31_task_list_lock);
	for (task = s31_task_list; task; task = task->next)
		sched_show_task(task->thread);
	spin_unlock(&s31_task_list_lock);

}

/* The task body will never resume. Retire only this current task's shim
 * critical ownership; this is not native portMUX owner/count recovery. */
static void s31_task_retire_body_critical(struct s31_linux_task *task)
{
	if (task != s31_linux_current_task())
		panic("esp32s31-radio: critical retirement outside target");
	if (task->sync_lock_depth)
		panic("esp32s31-radio: task exited with a live sync lock");
	if (task->critical.depth && !task->critical.suspended) {
		if (!task->blob_active || !s31_linux_blob_held_by_current())
			panic("esp32s31-radio: live critical retirement outside owned blob");
		/* Matches bridge ordering: raw unlock, suspended publication, IRQ /
		 * preemption restore, then pending-ISR service while gate is held. */
		s31_linux_critical_suspend();
	}
	if (task->critical.depth && !task->critical.suspended)
		panic("esp32s31-radio: physical critical ownership survived retirement");
	/* No physical raw lock or owned IRQ/preempt reference remains. Clear
	 * driver metadata before any fresh callback span; payload resets its
	 * separate logical nesting only from its current finalizer below. */
	memset(&task->critical, 0, sizeof(task->critical));
}

/* A destructor's new locks must have balanced normally. Do not guess which
 * object mux/sync lock it intended to keep or free objects behind such a lock. */
static void s31_task_validate_finalizer_ownership(struct s31_linux_task *task)
{
	if (task != s31_linux_current_task() || !task->finalizing ||
	    !task->blob_active || !s31_linux_blob_held_by_current())
		panic("esp32s31-radio: finalizer ownership outside target blob");
	if (task->critical.depth || task->critical.suspended ||
	    task->critical.flags || task->critical.thread ||
        task->sync_lock_depth || task->sync_irq_flags)
		panic("esp32s31-radio: finalizer returned with critical/sync ownership");
}

static void s31_native_task_validate_retirement(struct s31_linux_task *task)
{
	/* Kernel exit monitoring cannot dereference the private SRAM descriptor.
	 * Validate all driver ownership before freeing it, then retire the
	 * kernel context only after the last SRAM access and init_mm release. */
	if (task->cookie || task->blob_active || task->finalizing ||
	    task->blob.errno_active || task->blob.errno_native || task->blob.errno_bank ||
	    task->critical.depth || task->critical.suspended || task->sync_lock_depth ||
	    task->critical.flags || task->critical.thread || task->sync_irq_flags ||
	    task->on_kernel_stack || task->exit_preempt_guard ||
	    READ_ONCE(current_thread_info()->s31_blob_fp) ||
	    s31_linux_blob_held_by_current() || in_interrupt() || irqs_disabled() || preempt_count())
		panic("esp32s31-radio: native task retirement incomplete");
}

static int s31_linux_task_main(void *arg)
{
	struct s31_linux_task *task = arg;
	struct s31_stack_lease *stack_lease = task->stack_lease;

	if (current->stack != task->payload_stack ||
	    task->payload_stack_size != THREAD_SIZE ||
	    task_stack_end_corrupted(current))
		panic("esp32s31-radio: invalid registered SRAM task stack");
	esp32s31_kthread_use_init_mm();
	complete(&task->started);
	wait_for_completion(&task->creator_done);

	s31_linux_blob_enter(NULL);
	/* A stop selected immediately after startup still retires the empty
	 * cookie on its target continuation without running user entry. */
	if (!kthread_should_stop() && !atomic_read(&task->stopping))
		s31_payload_run(task->sp_save, 0,
				task->entry, task->arg,
				true);
	/* The body has returned or nonlocally restored this trampoline; its
	 * low frames are abandoned before physical ownership/pending retirement.
	 * A stop inside a nested Linux bridge bypasses that bridge's flag reset.
	 * Only the verified restored main stack may retire its borrowed marker. */
	{
		unsigned long sp, base = (unsigned long)current->stack;
		asm volatile("mv %0, sp" : "=r"(sp));
		if (sp < base || sp >= base + THREAD_SIZE)
			panic("esp32s31-radio: body return outside native task stack");
		task->on_kernel_stack = false;
	}
	task->payload_continuation_complete = true;
	/* A self-delete returns here with one transition guard still held. */
	if (task->exit_preempt_guard) {
		task->exit_preempt_guard = false;
		preempt_enable();
	}
	s31_task_retire_body_critical(task);
    /* Remaining TLSP callbacks belong to this target's final continuation.
     * Stop/self-delete may already have left the blob; restore a fresh owned
     * FP context on the same registered SRAM Linux stack.
     * New payloads register this private function via mandatory finalized-
     * create import; the fixed XIP export table remains unchanged. */
    if (task->finalize) {
        /* End the task-body window before activating finalizer FP policy.
         * The globally mapped task bank stays attached until all callbacks
         * complete. Body critical/mux retirement is a separate prerequisite. */
        if (task->blob_active)
            s31_linux_blob_leave(NULL);
        task->finalizing = true;
        task->payload_continuation_complete = false;
        s31_linux_blob_enter(NULL);
        task->finalizer_reported = false;
        task->finalizer_status = -EINPROGRESS;
        s31_payload_run(task->sp_save, 0,
            task->finalize, task->cookie, !s31_radio_payload_uses_fp());
        task->payload_continuation_complete = true;
        if (task->exit_preempt_guard) {
            task->exit_preempt_guard = false;
            preempt_enable();
        }
        /* Never retire the cookie/FP bank after an abandoned callback.
         * Direct/nonlocal callback exits are not silently
         * converted into full cleanup; bridge stop cancellation returns normally. */
        if (!task->finalizer_reported || task->finalizer_status)
            panic("esp32s31-radio: task %s finalizer incomplete (%d, reported=%d)",
                  current->comm, task->finalizer_status, task->finalizer_reported);
        /* Completion is not a promise that callbacks released their locks.
         * Refuse all cookie/bank/stack retirement before dropping the gate. */
        s31_task_validate_finalizer_ownership(task);
        /* Checkpoint the owned bank before clearing finalizer fence policy. */
        if (task->blob_active)
            s31_linux_blob_leave(NULL);
        task->finalizing = false;
    }
	/* Reached on normal entry return or after vTaskDelete(self). */
	if (task->blob_active)
		s31_linux_blob_leave(NULL);
	/* Preserve D waiter cleanup while init_mm/SRAM allocations are alive. */
	s31_rtos_task_release(task->cookie);
	task->cookie = NULL;
	/* The payload allocator may enter task-owned critical sections. Keep
	 * canonical identity and init_mm until all payload/bridge cleanup ends.
	 * Only then retire the bank and unpublish identity, before descriptor
	 * free. An active payload/borrow refusal must stop cleanup. */
	s31_native_mutex_assert_current_quiescent();
	s31_linux_task_cleanup(task);

	if (task_stack_end_corrupted(current))
		panic("esp32s31-radio: native SRAM stack guard exhausted (%s)",
		      current->comm);

	/* This is still current's Linux stack. Its owner returns it only
	 * after the final Linux stack reference and RCU reader have joined. */
	if (esp32s31_blob_fp_detach(current))
		panic("esp32s31-radio: task %s FP detach refused", current->comm);

	s31_native_task_validate_retirement(task);
	s31_linux_task_unregister(task);
	task->magic = 0;
	s31_radio_sram_free(task);
	/* No low-SRAM descriptor, allocator or payload access after this point. */
	esp32s31_kthread_unuse_init_mm();
	if ((esp32s31_radio_domain_task_retire(s31_native_domain)))
		panic("esp32s31-radio: kernel task retirement refused");
	spin_lock(&s31_stack_leases_lock);
	stack_lease->retired = true;
	spin_unlock(&s31_stack_leases_lock);
	return 0;
}

static void *s31_task_create_admitted_on_linux_stack(void (*entry)(void *), const char *name,
				    u32 stack_size, void *stack_base,
				    void *arg, u32 priority, void *cookie,
				    s32 core_id, void (*finalize)(void *), void **ready_slot)
{
	struct s31_linux_task *task;
	struct sched_param param = { };
	bool realtime_task;
	int fp_error;
	struct s31_stack_lease *lease;

	if (!entry || !stack_base || stack_size != THREAD_SIZE || !cpu_online(0) ||
	    ((priority > 31)))
		return NULL;
	s31_stack_leases_reap(false);
	lease = kzalloc(sizeof(*lease), GFP_KERNEL);
	if (!lease)
		return NULL;
	lease->kernel = esp32s31_radio_stack_prepare(stack_base, stack_size);
	if (IS_ERR(lease->kernel)) {
		kfree(lease);
		return NULL;
	}
	lease->base = stack_base;
	task = s31_radio_sram_alloc(sizeof(*task));
	if (!task)
		goto failed_lease;
	memset(task, 0, sizeof(*task));
	task->magic = S31_LINUX_TASK_MAGIC;
	task->entry = entry;
	task->arg = arg;
	task->cookie = cookie;
    task->finalize = finalize;
	task->payload_stack = stack_base;
	task->stack_lease = lease;
	task->payload_stack_size = stack_size;
	if (stack_size < 8192)
		memset(stack_base,0xa5,stack_size);
	task->requested_cpu = 0;

	init_completion(&task->exited);
	init_completion(&task->started);
	init_completion(&task->creator_done);
	atomic_set(&task->stopping, 0);
	task->thread = kthread_create_on_s31_stack(s31_linux_task_main, task,
				      lease->kernel, "%s",
				      name && *name ? name : "s31-task");
	if (IS_ERR(task->thread)) {

		s31_radio_sram_free(task);
		goto failed_lease;
	}

	/* Before first wake, publish kernel-owned canonical identity and the
	 * task's fixed FP ownership policy. No name-based admission or inheritance. */
	kthread_bind(task->thread, 0);
	/* Owned managed factory: cold admit the dormant, creator-owned CPU0
	 * thread to the radio scheduler role before any publication or wake.
	 * A fresh bound dormant thread must always be admissible. */
	if ((esp32s31_radio_sched_admit(task->thread)))
		panic("esp32s31-radio: fresh managed task scheduler admission rejected");
	fp_error = esp32s31_radio_domain_task_attach(task->thread, s31_native_domain, task);
	if (fp_error) {
		/* ENOMEM never publishes state. Any invariant failure on a fresh
		 * dormant task makes its ownership/lifetime suspect: stop cleanup.
		 */
		if (fp_error != -ENOMEM && fp_error != -ESHUTDOWN)
			panic("esp32s31-radio: fresh task FP attach rejected (%d)", fp_error);
		kthread_stop(task->thread);

		s31_radio_sram_free(task);
		goto failed_lease;
	}
	spin_lock(&s31_stack_leases_lock);
	list_add_tail(&lease->node, &s31_stack_leases);
	spin_unlock(&s31_stack_leases_lock);
	/* Register independent high-memory monitor before first wake/publication. */

    /* A child may run before its creator resumes. Publish its canonical
     * Linux identity before wakeup. */
	mutex_lock(&s31_task_start_lock);
	if (ready_slot)
		WRITE_ONCE(*ready_slot, (void *)task);
	spin_lock(&s31_task_list_lock);
	task->next = s31_task_list;
	s31_task_list = task;
	spin_unlock(&s31_task_list_lock);
	/* Every managed task uses the radio RR role on CPU0. */
	{
		if (esp32s31_radio_set_rr_interval(task->thread, 10) ||
		    esp32s31_sched_setscheduler(task->thread, SCHED_RR, priority + 1))
			panic("esp32s31-radio: native task scheduling admission failed");
		realtime_task = true;
		param.sched_priority = priority + 1;
	}

    wake_up_process(task->thread);
	/* Join only the child's startup publication, not its vendor body. */
	wait_for_completion(&task->started);
	complete(&task->creator_done);
	mutex_unlock(&s31_task_start_lock);
	/* No dereference follows creator_done: child may finish immediately. */
	return task;

failed_lease:
	if (esp32s31_radio_stack_destroy(lease->kernel))
		panic("s31 radio: failed creation retained its stack");
	/* The payload creator still owns/frees backing storage on failure. */
	kfree(lease);
	return NULL;
}

/* A function name/compat flag is not proof of physical stack placement.
 * Foreign workers can enter SRAM without a compatibility task descriptor.
 * Refuse sleeping creation unless this actual SP belongs to current's Linux
 * stack and its process context is sleepable. Payload allocation happened
 * before the import; only kernel creation admission/allocation is guarded. */
static bool s31_task_create_kernel_context(void)
{
	unsigned long sp;

	asm volatile("mv %0, sp" : "=r" (sp));
	return !in_interrupt() && !irqs_disabled() && !in_atomic() &&
		object_is_on_stack((void *)sp);
}

static void *s31_task_create_on_linux_stack(void (*entry)(void *), const char *name,
				u32 stack_size, void *stack_base, void *arg,
				u32 priority, void *cookie, s32 core_id,
				void (*finalize)(void *), void **ready_slot)
{
	void *result;

	if (!entry || !stack_base || !s31_task_create_kernel_context() ||
	    !s31_task_create_admit())
		return NULL;
	result = s31_task_create_admitted_on_linux_stack(entry, name, stack_size,
		stack_base, arg, priority, cookie, core_id, finalize, ready_slot);
	s31_task_create_complete();
	return result;
}

struct s31_create_args {
 void (*entry)(void *);
 void (*finalize)(void *);
 void **ready_slot;
 const char *name;
 u32 stack_size;
 void *stack_base, *arg, *cookie, *result;
 u32 priority;
 s32 core_id;
};

static void s31_create_on_kernel_stack(void *opaque)
{
 struct s31_create_args *args = opaque;
 args->result = s31_task_create_on_linux_stack(args->entry, args->name,
  args->stack_size, args->stack_base, args->arg, args->priority,
  args->cookie, args->core_id, args->finalize, args->ready_slot);
}

/* Refuse atomic creation before any owner publication or allocation. */
static bool s31_task_create_sync_locked(void)
{
 struct s31_linux_task *task = s31_linux_current_task();

 return task && READ_ONCE(task->sync_lock_depth);
}

/* Cold task creation joins only already-retired stacks. No global runtime
 * lock, polling, or SRAM allocation is involved before this reclaim step. */
static void s31_stack_reclaim_on_kernel_stack(void *opaque)
{
 bool *result = opaque;

 if (s31_task_create_kernel_context()) {
  s31_stack_leases_reap(true);
  *result = true;
 }
}

static bool s31_stack_reclaim_before_create(void)
{
 struct s31_linux_task *task = s31_linux_current_task();
 bool result = false;

 if (!s31_task_create_kernel_context() || s31_task_create_sync_locked())
  return false;
 if (task)
  return s31_task_native_linux_call(task, s31_stack_reclaim_on_kernel_stack,
    &result) && result;
 s31_stack_reclaim_on_kernel_stack(&result);
 return result;
}

static void *s31_linux_task_create_impl(void (*entry)(void *), const char *name,
 u32 stack_size, void *stack_base, void *arg, u32 priority,
 void *cookie, s32 core_id, void (*finalize)(void *), void **ready_slot)
{
 struct s31_linux_task *caller;
 struct s31_create_args args = {
  .entry = entry, .name = name, .stack_size = stack_size,
  .stack_base = stack_base, .arg = arg, .priority = priority,
  .cookie = cookie, .core_id = core_id, .finalize = finalize, .ready_slot = ready_slot,
 };
 /* Do not suspend a live blob/critical/FP owner from an IRQ callback. */
 if (in_interrupt() || s31_task_create_sync_locked())
  return NULL;
 caller = s31_linux_current_task();
 if (!caller)
  return s31_task_create_on_linux_stack(entry, name, stack_size,
   stack_base, arg, priority, cookie, core_id, finalize, ready_slot);
 /* Creation can block on CPU1, using this same registered Linux stack.
  * Keep the persistent vendor context and bank across that native wait. */
 if (!s31_task_native_linux_call(caller, s31_create_on_kernel_stack, &args)) return NULL;
 return args.result;
}

void *s31_linux_task_create_finalized(void (*entry)(void *), const char *name,
 u32 stack_size, void *stack_base, void *arg, u32 priority,
 void *cookie, s32 core_id, void (*finalize)(void *), void **ready_slot)
{
 if (in_interrupt() || s31_task_create_sync_locked() ||
     !finalize || !ready_slot || READ_ONCE(*ready_slot))
  return NULL;
 return s31_linux_task_create_impl(entry, name, stack_size, stack_base, arg,
                                  priority, cookie, core_id, finalize, ready_slot);
}

void s31_linux_task_finalize_report(s32 status)
{
 struct s31_linux_task *task = s31_linux_current_task();

 if (!task || !task->finalizing)
  panic("esp32s31-radio: finalizer report outside target continuation");
 if (task->finalizer_reported)
  panic("esp32s31-radio: task %s duplicate finalizer report", current->comm);
 task->finalizer_status = status;
 task->finalizer_reported = true;
}

void *s31_linux_current_cookie(void)
{
	struct s31_linux_task *task = s31_linux_current_task();

	return task ? task->cookie : NULL;
}

void s31_linux_task_exit_current(void)
{
	struct s31_linux_task *task = s31_linux_current_task();
	if (!task)
		return;
	/* Release body-owned critical state before releasing the blob gate.
	 * A stopped kernel wait already suspended it; that path only retires
	 * released metadata and never unlocks another task's current raw lock. */
	s31_task_retire_body_critical(task);
	if (task->blob_active) {
		preempt_disable();
		task->exit_preempt_guard = true;
		s31_linux_blob_leave(NULL);
		task->blob_active = false;
	}
	/* Longjmp to the trampoline continuation on the kthread stack; it
	 * performs the unuse_mm/cleanup/unregister/free sequence. */
	s31_payload_sp_restore(task->sp_save);
	unreachable();
}

struct s31_stop_args {
	struct task_struct *target;
	struct s31_linux_task *owner;
};

static void s31_stop_on_kernel_stack(void *opaque)
{
	struct s31_stop_args *args = opaque;

	/* Admission precedes stop publication, so failed native admission cannot
	 * strand a target with a published stop but no joining owner. */
	struct s31_linux_task *task = args->owner;

	if (!task) panic("esp32s31-radio: stop target lost native owner");
	atomic_set(&task->stopping, 1);
	atomic_inc(&s31_sync_stop_generation);
	wake_up_process(args->target);
	kthread_stop(args->target);
}

void s31_linux_runtime_stop_publish(void)
{
	struct s31_runtime_waiter *waiter;
	unsigned long flags;

	/* Register-before-check pairs with this locked epoch publication. A
	 * waiter arriving after publication sees the epoch mismatch before sleep;
	 * an already sleeping waiter gets an explicit wake, even with no timer.
	 * Future waits may still be needed by terminal shutdown: this cancels
	 * admitted waits once, rather than globally forbidding subsequent waits.
	 */
	spin_lock_irqsave(&s31_runtime_waiters_lock, flags);
	atomic_inc(&s31_runtime_cancel_generation);
	atomic_inc(&s31_sync_stop_generation);

	list_for_each_entry(waiter, &s31_runtime_waiters, node) {
		wake_up_process(waiter->thread);

	}
	spin_unlock_irqrestore(&s31_runtime_waiters_lock, flags);
}

int32_t s31_linux_task_stop(void *opaque)
{
	struct s31_linux_task *task = opaque;
	struct s31_linux_task *caller = s31_linux_current_task();
	struct s31_stop_args args;

	if (!task || task->magic != S31_LINUX_TASK_MAGIC)
		return -EINVAL;
	if (task == caller) {
		s31_linux_task_exit_current();
		unreachable();
	}
	args.target = task->thread;
	args.owner = task;
	if (caller) {
		if (!s31_task_native_linux_call(caller, s31_stop_on_kernel_stack,
			&args)) return -EWOULDBLOCK;
	} else {
		if (!s31_task_create_kernel_context()) return -EWOULDBLOCK;
		s31_stop_on_kernel_stack(&args);
	}
	return 0;
}

int s31_linux_task_stop_thread(struct task_struct *thread)
{
 struct s31_linux_task *task;
 bool found = false;

 if (!thread || thread == current || !s31_task_create_kernel_context())
  return -EINVAL;
 /* Resolve only while the native record is published. Retain no private
  * descriptor past the lock: the target may free it during its finalizer.
  * The caller's Linux task reference remains valid through kthread_stop. */
 spin_lock(&s31_task_list_lock);
 for (task = s31_task_list; task; task = task->next)
  if (task->thread == thread) {
   atomic_set(&task->stopping, 1);
   found = true;
   break;
  }
 spin_unlock(&s31_task_list_lock);
 if (!found) return -ESRCH;
 atomic_inc(&s31_sync_stop_generation);
 wake_up_process(thread);
 kthread_stop(thread);
 return 0;
}

void s31_linux_tasks_stop_all(void)
{
	s31_linux_tasks_quiesce_creates();
	for (;;) {
		struct s31_linux_task *task;
		struct task_struct *thread;

		mutex_lock(&s31_task_start_lock);
		spin_lock(&s31_task_list_lock);
		task = s31_task_list;
		if (!task) {
			spin_unlock(&s31_task_list_lock);
			mutex_unlock(&s31_task_start_lock);
			break;
		}
		atomic_set(&task->stopping, 1);
		thread = task->thread;
		get_task_struct(thread);
		spin_unlock(&s31_task_list_lock);
		mutex_unlock(&s31_task_start_lock);

		atomic_inc(&s31_sync_stop_generation);
		wake_up_process(thread);
		kthread_stop(thread);
		put_task_struct(thread);
	}
	/* Kernel-owned records also cover the still-running private service.
	 * The final join follows its owner-side retirement and thread exit. */

}

struct s31_delay_args {
 ktime_t deadline;
 bool yielding;
};

/* The sleeper lives on the mapped Linux stack and is cancelled/destroyed
 * before resume or nonlocal stop. Unrelated task wakeups do not shorten a
 * vTaskDelay; only expiry or target stop ends this native wait. */
static void s31_native_delay_until(ktime_t deadline)
{
 struct hrtimer_sleeper sleeper;
 struct s31_linux_task *task = s31_linux_current_task();

 hrtimer_setup_sleeper_on_stack(&sleeper, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
 if (deadline != KTIME_MAX) {
  hrtimer_set_expires(&sleeper.timer, deadline);
  hrtimer_sleeper_start_expires(&sleeper, HRTIMER_MODE_ABS);
 }
 for (;;) {
  set_current_state(TASK_INTERRUPTIBLE);
  if (kthread_should_stop() || atomic_read(&task->stopping) ||
      (deadline != KTIME_MAX && !READ_ONCE(sleeper.task)))
   break;
  schedule();
 }
 __set_current_state(TASK_RUNNING);
 hrtimer_cancel(&sleeper.timer);
 destroy_hrtimer_on_stack(&sleeper.timer);
}
static void s31_delay_on_kernel_stack(void *opaque)
{
 struct s31_delay_args *args=opaque;
 if (args->yielding) {
  yield();
 } else {
  s31_native_delay_until(args->deadline);
 }
 /* Stop cancels a destructor's bridge wait normally; it must not longjmp
  * out of the final continuation. Direct nonlocal exits remain detected by
  * the mandatory completion report. Ordinary task-body stop is unchanged. */
 if (kthread_should_stop() && !s31_linux_current_task()->finalizing) {
  s31_linux_task_exit_current();
  unreachable();
 }
}

void s31_linux_task_delay(u32 ticks)
{
 struct s31_linux_task *task=s31_linux_current_task();
 struct s31_delay_args args={
  .yielding=!ticks,
  .deadline=s31_native_tick_deadline(ticks),
 };
 if (WARN_ONCE(in_interrupt(), "esp32s31-radio: task_delay in IRQ context\n"))
  return;
 if (!task || atomic_read(&task->stopping)) return;
 if (s31_task_native_linux_call(task, s31_delay_on_kernel_stack, &args)) return;
 panic("esp32s31-radio: task delay outside native execution context");
}

void s31_linux_task_yield(void)
{
 s31_linux_task_delay(0);
}

void s31_linux_task_set_priority(void *opaque, u32 priority, u32 boost)
{
	struct s31_linux_task *task = opaque;
	struct task_struct *thread;
	long result;

	if (!task || task->magic != S31_LINUX_TASK_MAGIC)
		return;
	if (boost || priority > 31)
		panic("esp32s31-radio: manual donation entered native scheduling");
	/* Pin before the Linux-stack wait; the descriptor may retire there. */
	thread = task->thread;
	get_task_struct(thread);
	result = s31_native_object_call((void *)thread, S31_SYNC_NATIVE_BASE, priority);
	put_task_struct(thread);
	if (result && result != -ESRCH)
		panic("esp32s31-radio: native base priority update failed (%ld)", result);
}

/* Service is the IDF timer role (22), below controller/PP tasks (23).
 * Temporary IRQ urgency stays separate. Backlog/ordering needs target proof.
 */
int s31_linux_radio_service_setup(struct task_struct *thread)
{
    int ret;

    if ((s31_service_context.admitted)) return -EBUSY;
    /* Owned private service/control executor: cold admit the dormant,
     * creator-owned CPU0 thread before it is first woken. */
    {
        ret = esp32s31_radio_sched_admit(thread);
        if (ret) return ret;
    }
    {
        ret = esp32s31_radio_set_rr_interval(thread, 10);
        if (ret) return ret;
        ret = esp32s31_sched_setscheduler(thread, SCHED_RR, 23);
        if (ret) return ret;
    }
    ret = s31_service_priority_setup(thread);
    if (ret) return ret;
    {
        memset(&s31_service_context, 0, sizeof(s31_service_context));
        s31_service_context.thread = thread;
    }
    return 0;
}

/* Optional threaded-IRQ executor admission. The executor is created and bound
 * to CPU0 before this call and is still dormant, so the cold role admission is
 * legal; a rejection is a hard failure the caller unwinds. */
int s31_linux_radio_irq_executor_admit(struct task_struct *thread)
{

    return esp32s31_radio_sched_admit(thread);
}

static unsigned long s31_native_effective_priority(void *opaque)
{
	struct s31_linux_task *task = opaque;
	int priority;

	if (!task || task->magic != S31_LINUX_TASK_MAGIC)
		return 0;
	priority = READ_ONCE(task->thread->prio);
	return clamp_t(int, MAX_RT_PRIO - 2 - priority, 0, 31);
}

static unsigned long s31_native_base_priority(void *opaque)
{
	struct s31_linux_task *task = opaque;

	if (!task || task->magic != S31_LINUX_TASK_MAGIC)
		return 0;
	return clamp_t(int, READ_ONCE(task->thread->rt_priority) - 1, 0, 31);
}

void *s31_linux_sync_create(void)
{
	struct s31_linux_sync *sync = s31_radio_sram_alloc(sizeof(*sync));

	if (sync)
		raw_spin_lock_init(&sync->lock);
	if (sync)
		atomic_set(&sync->sequence, 0);
	if (sync) {
		init_waitqueue_head(&sync->waitq);
		sync->owner_thread = NULL;
		sync->owner_cookie = NULL;
		sync->owner_abandoned = false;
		sync->native_mutex = false;
		INIT_LIST_HEAD(&sync->owner_node);
	}
	return sync;
}

static unsigned long s31_critical_mux_operation(void *opaque, u32 op);

#include "esp32s31-radio-foreign-owner.inc"
#include "esp32s31-radio-service-priority.inc"

static int *s31_errno_current_bank(void)
{
 struct s31_linux_task *task = s31_linux_current_task();
 struct s31_foreign_owner *owner;
 int *bank;
 if (task) return &task->errno_value;
 owner = s31_foreign_owner_get(current);
 if (!owner)
  panic("esp32s31-radio: errno has no live native owner");
 /* Registry and same-thread exit ownership retain the bank until leave. */
 bank = &owner->errno_value;
 s31_foreign_owner_put(owner);
 return bank;
}
#include "esp32s31-radio-service-context.inc"
static unsigned long s31_native_mutex_tx_credit_bind(void *opaque);
static unsigned long s31_native_mutex_transaction(void *opaque, u32 op);

unsigned long s31_linux_sync_owner(void *opaque, u32 op)
{

    /* These operations only manipulate a native identity reference and short
     * list metadata. No allocation, sleep, gate pivot or FP owner change. */
    if (op == S31_SYNC_OWNER_FATAL)
        panic("esp32s31-radio: invalid payload ownership or TLS allocation; refusing unsafe continuation");
    if (op >= S31_SYNC_MUX_ENTER && op <= S31_SYNC_CRITICAL_DEPTH)
        return s31_critical_mux_operation(opaque, op);
    if (op == S31_SYNC_NATIVE_AVAILABLE)
        return s31_native_sync_enabled(opaque) ?
            s31_native_mutex_owner_op(opaque, op) : 0;
    if (op == S31_SYNC_AUX_CONTEXT) {
        bool bootstrap = s31_control_bootstrap_owns(this_cpu_read(s31_foreign_installed));

        if (raw_smp_processor_id() != 0)
            return 0;
        if (bootstrap)
            WRITE_ONCE(s31_aux_context_queried, true);
        return bootstrap || (READ_ONCE(s31_rtos_isr_depth) &&
               (esp32s31_radio_on_irq_stack() || s31_linux_blob_held_by_current()));
    }
    if (op == S31_SYNC_TX_POOL_BIND)
        return in_interrupt() ? 0 : s31_radio_tx_pool_bind(opaque);
    if (op == S31_SYNC_TX_LOCK_BIND)
        return in_interrupt() ? 0 : s31_native_mutex_tx_credit_bind(opaque);
    if (op == S31_SYNC_TX_FLIGHT_RETURN) {
        s31_radio_tx_credit_return(S31_TX_CREDIT_FLIGHT);
        return 1;
    }
    /* SoftMAC startup requires the payload's PP repair notification contract. */
    if (op == S31_SYNC_TX_REPAIR_CAP)
        return IS_ENABLED(CONFIG_ESP32S31_WIFI_SOFTMAC);
    if (op == S31_SYNC_SOFTMAC_TX_COPY)
        return s31_radio_softmac_tx_copy(opaque);
    if (op == S31_SYNC_TIMER_CHANGED) {
        s31_radio_timer_changed();
        return 1;
    }
    if (in_interrupt())
        return 0;
    if (op == S31_SYNC_NATIVE_TRANSACTION_ABI)
        return 2;
    if ((op >= S31_SYNC_NATIVE_META_ENTER && op <= S31_SYNC_NATIVE_GIVE_LEAVE) ||
        op == S31_SYNC_NATIVE_TRY_BEGIN)
        return s31_native_mutex_transaction(opaque, op);
    /* Native ownership ignores the payload cookie: rtmutex/current is its
     * identity. Route frequent metadata operations before cold setup/probe
     * cases, without evaluating the unrelated generic-owner cookie getter.
     * Keep the same IRQ refusal and per-object admission as the slow route. */
    if (op <= S31_SYNC_OWNER_ALIVE && s31_native_sync_enabled(opaque))
        return s31_native_mutex_owner_op(opaque, op);
    if (op == S31_SYNC_NATIVE_BEGIN || op == S31_SYNC_NATIVE_END ||
        op == S31_SYNC_NATIVE_BUSY)
        return s31_native_sync_enabled(opaque) ?
            s31_native_mutex_owner_op(opaque, op) : 0;
    if (op == S31_SYNC_NATIVE_RELEASE)
        return s31_native_object_call(opaque, op, 0);
    /* Retired private operations are tombstones, never pointer admission.
     * Keep numeric values stable so older payloads fail before dereference. */
    if (op == S31_SYNC_OWNER_FOREIGN_TOKEN) return 0;

    if (op == S31_SYNC_STACK_RECLAIM)
        return s31_stack_reclaim_before_create();
     /* Query alone cannot negotiate an old allocator. */
    if (op == S31_SYNC_STACK_RAW_MINIMUM) {
        WRITE_ONCE(s31_stack_minimum_queried, true);
        return THREAD_SIZE;
    }
    if (op == S31_SYNC_STACK_ALLOC)
        return s31_stack_minimum_queried && !in_interrupt() ?
            (unsigned long)s31_radio_sram_stack_alloc((unsigned long)opaque) : 0;
    if (op == S31_SYNC_STACK_FREE) {
        if (!s31_stack_minimum_queried || in_interrupt())
            panic("s31 radio: native stack free outside negotiated task context");
        s31_radio_sram_stack_free(opaque, THREAD_SIZE);
        return 1;
    }
    if (op == S31_SYNC_NATIVE_MODE) {
        WRITE_ONCE(s31_native_mode_queried, true);
        return true;
    }
    if (op == S31_SYNC_NATIVE_EFFECTIVE)
        return s31_native_effective_priority(opaque);
    if (op == S31_SYNC_NATIVE_BASE_QUERY)
        return s31_native_base_priority(opaque);
    if (op == S31_SYNC_NATIVE_INIT || op == S31_SYNC_NATIVE_DELETE)
        return (s31_native_object_call(opaque, op, 0));
    if (op == S31_SYNC_ERRNO_REGISTER) {
        s31_errno_register(opaque, s31_blob_context_current());
        return 1;
    }
    return s31_sync_owner_operation(opaque, op, current,
                                   s31_linux_current_cookie());
}

void s31_linux_sync_destroy(void *opaque)
{
    struct s31_linux_sync *sync = opaque;

    if (!sync) return;
    if (sync->native_mutex) {
        if (!s31_native_object_call(sync, S31_SYNC_NATIVE_DESTROY, 0))
            panic("esp32s31-radio: native sidecar destruction refused");
    } else {
        s31_sync_owner_operation(opaque, S31_SYNC_OWNER_FORCE_DROP, current, NULL);
    }
    s31_radio_sram_free(opaque);
}

void s31_linux_sync_lock(void *opaque)
{

	struct s31_linux_sync *sync = opaque;
	struct s31_linux_task *task = s31_linux_current_task();
	unsigned long flags;

	if (!sync)
		return;
	/* These queue/event locks are never held across a blocking bridge call.
	 * Match the native FreeRTOS port by masking local interrupts across the
	 * short lock instead of walking Linux's generic disable/enable IRQ path on
	 * every BTDM queue operation.  The outermost depth owns the saved state.
	 * It also owns a preemption fence (C3): BT-only blob windows are
	 * preemptible, and the waiter/waker pairing assumes the holder cannot
	 * migrate mid-critical. */
	if (task) {
		if (!task->sync_lock_depth) {
			preempt_disable();
			local_irq_save(flags);

			task->sync_irq_flags = flags;
		}
		task->sync_lock_depth++;
	}
	raw_spin_lock(&sync->lock);
}

void s31_linux_sync_unlock(void *opaque)
{

	struct s31_linux_sync *sync = opaque;
	struct s31_linux_task *task = s31_linux_current_task();

	if (!sync)
		return;
	raw_spin_unlock(&sync->lock);
	if (task) {
		unsigned long flags = 0;

		WARN_ON_ONCE(!task->sync_lock_depth);
		if (task->sync_lock_depth) {
			task->sync_lock_depth--;
			if (!task->sync_lock_depth) {
				flags = task->sync_irq_flags;
				task->sync_irq_flags = 0;

				local_irq_restore(flags);
				preempt_enable();
			}
		}
		/* Nested object locks (queue -> task PI, queue -> registry)
		 * still protect live state. A callback may reenter those objects;
		 * service pending handlers only after the outermost release. */
		if (!task->sync_lock_depth)
			s31_radio_blob_run_pending_isrs();
	}
}

u32 s31_linux_sync_sequence(void *opaque)
{
	struct s31_linux_sync *sync = opaque;

	return sync && !sync->native_mutex ? atomic_read(&sync->sequence) : 0;
}

struct s31_wait_args {
 struct s31_linux_sync *sync;
 struct s31_linux_task *task;
 u32 sequence;
 int stop_generation, runtime_generation;
 long timeout, ret;
 ktime_t deadline;
 bool high_resolution, tick_deadline, runtime_cancellable;
 bool dispatcher, tx_repair;
 void (*before_wait)(void);
};

/* Linux stack only. Absolute mode prevents setup/bridge time from being
 * re-added to a native deadline. Queue publication and timer expiry both use
 * the standard prepare/check/schedule pairing; finish/cancel precede resume. */
static noinline long s31_wait_until_loop(struct s31_wait_args *args,
                                        struct hrtimer_sleeper *sleeper)
{
 wait_queue_entry_t entry;

 long result;

 init_wait_entry(&entry, 0);
 for (;;) {
  result = prepare_to_wait_event(&args->sync->waitq, &entry, TASK_INTERRUPTIBLE);
  if (atomic_read(&args->sync->sequence) != args->sequence ||
      atomic_read(&s31_sync_stop_generation) != args->stop_generation ||
      (args->runtime_cancellable &&
       atomic_read(&s31_runtime_cancel_generation) != args->runtime_generation) ||
      (args->tx_repair && s31_radio_softmac_tx_repair_cancelled()) ||
      kthread_should_stop()) {
   result = 1;
   break;
  }
  if (result) break;
  if (sleeper && !READ_ONCE(sleeper->task)) {
   result = 0;
   break;
  }
  schedule();
 }
 finish_wait(&args->sync->waitq, &entry);

 return result;
}

/* Keep timer storage out of indefinite waits, including compiler stack
 * initialization. Finite waits retain their original absolute deadline and
 * join the timer callback before the admitted waiter can retire. */
static noinline long s31_wait_until_finite(struct s31_wait_args *args)
{
 struct hrtimer_sleeper sleeper;

 long result;

 hrtimer_setup_sleeper_on_stack(&sleeper, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
 hrtimer_set_expires(&sleeper.timer, args->deadline);
 hrtimer_sleeper_start_expires(&sleeper, HRTIMER_MODE_ABS);

 result = s31_wait_until_loop(args, &sleeper);

 hrtimer_cancel(&sleeper.timer);
 destroy_hrtimer_on_stack(&sleeper.timer);

 return result;
}

static long s31_wait_until(struct s31_wait_args *args)
{
 if (args->deadline == KTIME_MAX) return s31_wait_until_loop(args, NULL);
 return s31_wait_until_finite(args);
}

static void s31_wait_on_kernel_stack(void *opaque)
{
 struct s31_wait_args *args=opaque;
 struct s31_linux_sync *sync=args->sync;

 struct s31_runtime_waiter runtime_waiter = { .thread = current };
 unsigned long runtime_flags;
 bool repair_admitted = false;
 long ret;
 /* Admission guarantees a registered task on its persistent SRAM Linux stack. */
 if (args->tx_repair)
  repair_admitted = s31_radio_softmac_tx_repair_begin();
 if (args->runtime_cancellable) {
  spin_lock_irqsave(&s31_runtime_waiters_lock, runtime_flags);
  list_add_tail(&runtime_waiter.node, &s31_runtime_waiters);
  s31_runtime_wait_active++;
  spin_unlock_irqrestore(&s31_runtime_waiters_lock, runtime_flags);
 }

 if (args->before_wait) {

  args->before_wait();

 }
 if (args->tx_repair && !repair_admitted) {
  ret = -EINTR;
 } else if (args->dispatcher && (!args->deadline ||
     atomic_read(&sync->sequence)!=args->sequence)) {
  /* A bounded TX batch or a new publication left runnable work. Yield through Linux instead
   * of installing a timer and sleeping while the radio has useful work. */
  cond_resched();
  ret=1;
 } else if (args->high_resolution) {
  ret=s31_wait_until(args);
 } else {

  ret=wait_event_interruptible_timeout(sync->waitq,
   atomic_read(&sync->sequence)!=args->sequence ||
   atomic_read(&s31_sync_stop_generation)!=args->stop_generation ||
   (args->runtime_cancellable &&
    atomic_read(&s31_runtime_cancel_generation)!=args->runtime_generation) ||
   (args->tx_repair && s31_radio_softmac_tx_repair_cancelled()) ||
   kthread_should_stop(),args->timeout);

 }

 /* All waitqueue/timer cleanup above must precede removal or any nonlocal
  * stop. Publication cannot follow a retired stack entry or miss an
  * indefinitely sleeping registered waiter. Cancellation wins while the
  * wait remains admitted, independently of ordinary per-task stop pokes.
  */
 if (args->runtime_cancellable) {
  spin_lock_irqsave(&s31_runtime_waiters_lock, runtime_flags);
  list_del(&runtime_waiter.node);
  s31_runtime_wait_active--;
  if (atomic_read(&s31_runtime_cancel_generation) != args->runtime_generation) {
   ret = -EINTR;

  }
  spin_unlock_irqrestore(&s31_runtime_waiters_lock, runtime_flags);
 }
 if (repair_admitted) {
  if (s31_radio_softmac_tx_repair_end()) ret = -EINTR;
 }
 if (args->task && args->task->finalizing &&
     (kthread_should_stop() || atomic_read(&args->task->stopping))) {
  /* finish_wait/hrtimer_cancel above precede the resumed cancellation. */
  ret = -EINTR;
 } else if (args->task && kthread_should_stop()) {
  s31_linux_task_exit_current();
  unreachable();
 }
 args->ret=ret;
 /* The dispatch waitq is globally mapped kernel memory. Only Linux-stack
  * continuations access it; the SRAM body carries its generation by value.
  * A wake during/after resume remains visible at the next wait check. */
 if (args->dispatcher) args->sequence=atomic_read(&sync->sequence);

}

/* Real managed dispatcher: firmware created its TCB and the ordinary task
 * trampoline owns cancellation/finalization. No foreign identity is invented. */
int s31_linux_dispatch_validate(void)
{
 return (0);
}

/* A caller must explicitly promise to handle cancellation as an error return.
 * Closed permanent service loops do not make that promise: BTDM dereferences
 * its indefinite event receive without a NULL check. Owner stop instead uses
 * the existing nonlocal body retirement after device quiescence. This flag
 * grants no permission to interrupt TLS/finalizer waits.
 */
int s31_linux_wait_cancel_allow(bool allow)
{
 struct s31_linux_task *task = s31_linux_current_task();

 if (!task || task->finalizing || !task->blob_active)
  return -EINVAL;
 WRITE_ONCE(task->runtime_wait_cancel_allowed, allow);
 return 0;
}

/* Unlike firmware queues, this object is never used by closed code. CPU1
 * producers and asynchronous Linux wakeups require a globally mapped waitq,
 * whose list links also stay globally mapped. Retire after all owners join. */
void *s31_linux_dispatch_sync_create(void)
{
 struct s31_linux_sync *sync=kzalloc(sizeof(*sync),GFP_KERNEL);
 if (!sync) return NULL;
 raw_spin_lock_init(&sync->lock);
 atomic_set(&sync->sequence,0);
 init_waitqueue_head(&sync->waitq);
 INIT_LIST_HEAD(&sync->owner_node);
 return sync;
}
void s31_linux_dispatch_sync_destroy(void *opaque)
{
 struct s31_linux_sync *sync=opaque;
 if (sync && WARN_ON_ONCE(waitqueue_active(&sync->waitq)))
  panic("esp32s31-radio: live dispatcher waiter at retirement");
 kfree(sync);
}

u32 s31_linux_dispatch_wait(void *opaque, u32 sequence, u64 deadline_ns,
                            void (*before_wait)(void))
{

 struct s31_linux_task *task=s31_linux_current_task();
 struct s31_wait_args args={
  .sync=opaque, .task=task, .sequence=sequence,
  .stop_generation=atomic_read(&s31_sync_stop_generation),
  .runtime_generation=atomic_read(&s31_runtime_cancel_generation),
  .runtime_cancellable=task && !task->finalizing &&
                       READ_ONCE(task->runtime_wait_cancel_allowed),
  .high_resolution=true, .dispatcher=true,
  /* Payload and Linux use the same monotonic epoch. Never restart a
   * relative delay after callbacks/completion work, or round it to50us. */
  .deadline=deadline_ns == U64_MAX ? KTIME_MAX : ns_to_ktime(deadline_ns),
  .before_wait=before_wait,
 };
 if (!task || !args.sync || !task->blob_active || task->sync_lock_depth ||
     in_interrupt() || task->critical.depth ||
     task->critical.mux_depth)
  panic("esp32s31-radio: invalid managed dispatcher wait");
 if (s31_task_native_linux_call(task, s31_wait_on_kernel_stack, &args)) return args.sequence;
 panic("esp32s31-radio: dispatcher wait outside native execution context");
}

static void s31_pmu_reclaim_on_kernel_stack(void *opaque)
{
 (void)opaque;
 if (!s31_task_create_kernel_context())
  panic("esp32s31-radio: PMU reclaim outside sleepable Linux context");
 s31_linux_pmu_reclaim_after_radio_init();
}

void s31_linux_pmu_reclaim_bridge(void)
{
 struct s31_linux_task *task = s31_linux_current_task();

 if (task) {
  if (!s31_task_native_linux_call(task, s31_pmu_reclaim_on_kernel_stack,
       NULL))
   panic("esp32s31-radio: PMU reclaim outside native execution context");
 } else {
  s31_pmu_reclaim_on_kernel_stack(NULL);
 }
}

bool s31_linux_native_rx_pressure_wait(void (*entry)(void *), void *arg)
{
 if (in_interrupt() || in_atomic() || irqs_disabled() ||
     !s31_linux_current_task()) return false;
 return s31_task_native_linux_call(s31_linux_current_task(), entry, arg);
}

#include "esp32s31-radio-native-object.inc"

static noinline int32_t s31_linux_queue_wait(struct s31_linux_sync *sync,
 struct s31_linux_task *task, u32 sequence, u32 ticks, bool tx_repair)
{
 struct s31_wait_args args={
  .sync=sync, .task=task, .sequence=sequence,
  .stop_generation=atomic_read(&s31_sync_stop_generation),
  .runtime_generation=atomic_read(&s31_runtime_cancel_generation),
  .runtime_cancellable=task && !task->finalizing &&
                       READ_ONCE(task->runtime_wait_cancel_allowed),
  .timeout=s31_ticks_to_jiffies(ticks),
  .tick_deadline=true,
  .tx_repair=tx_repair,
  .deadline=s31_native_tick_deadline(ticks),
 };
 /* Sleeping entries are process-context only.  In interrupt context there
  * is no task to suspend back onto a Linux stack; schedule() there would
  * panic.  Refuse loudly instead: legitimate blob sleep paths always run
  * in compatibility kthreads (direct-ISR work defers via DEFER_UNSAFE). */
 if (WARN_ONCE(in_interrupt(), "esp32s31-radio: sync_wait in IRQ context\n"))
  return -1;

 args.high_resolution=(args.tick_deadline) && ticks != U32_MAX;
 if (!args.sync || !args.timeout) return 0;
 if (!task && s31_foreign_owner_retiring(current)) return -1;
 /* A pre-existing target stop cancels without suspending the owned context.
  * Returning -1 lets queue/event/notify APIs unwind their ordinary waiters. */
 if (task && task->finalizing &&
     (kthread_should_stop() || atomic_read(&task->stopping)))
  return -1;

 if (s31_task_native_linux_call(task, s31_wait_on_kernel_stack, &args))
  return args.ret<0 ? -1 : args.ret ? 1 : 0;
 /* Native task context is mandatory. Do not pivot to a compatibility
  * continuation or admit a foreign sleeper when ownership checks fail. */
 return -1;
}

int32_t s31_linux_sync_wait(void *opaque, u32 sequence, u32 ticks, u32 reason)
{

 struct s31_linux_task *task=s31_linux_current_task();
 struct s31_linux_sync *sync=opaque;
 bool tx_repair=!!(reason & S31_SYNC_WAIT_TX_REPAIR);

 reason &= ~S31_SYNC_WAIT_TX_REPAIR;
 if (tx_repair && (reason != S31_BLOB_RELEASE_QUEUE_SEND || !task)) return -1;
 /* Do not construct or compiler-initialize generic queue wait state on the
  * native mutex path. Its own absolute deadline is still captured before
  * trylock, with PI, cancellation and object admission unchanged. */
 if (sync && sync->native_mutex) {
  long result;
  if (reason != S31_BLOB_RELEASE_SEMAPHORE_TAKE) return -1;
  result = s31_native_object_call(sync, S31_SYNC_NATIVE_TAKE, ticks);
  return result ? result == -ETIMEDOUT || result == -EBUSY ? 0 : -1 : 1;
 }
 return s31_linux_queue_wait(sync, task, sequence, ticks, tx_repair);
}

void s31_linux_sync_wake(void *opaque)
{

	struct s31_linux_sync *sync = opaque;

	if (!sync || sync->native_mutex)
		return;
	atomic_inc(&sync->sequence);
	/* A Wi-Fi RX interrupt usually makes exactly one queue consumable.  A
	 * global waitqueue woke every compatibility task for every frame and
	 * turned non-aggregated receive into a scheduler-bound 100 Hz path. */
	if (s31_waitqueue_needs_wake(&sync->waitq))
		wake_up_all(&sync->waitq);
}

static u32 s31_mux_cycle_count(void)
{
    return (u32)csr_read(CSR_CYCLE);
}
#include "esp32s31-radio-critical.inc"

void s31_linux_blob_enter(struct s31_blob_context *ctx)
{
 struct s31_linux_task *task = s31_linux_current_task();
 bool service = s31_current_service();
 struct s31_blob_context *context;

 s31_control_owner_assert_vendor_entry(ctx);
 if (raw_smp_processor_id() != 0)
  panic("esp32s31-radio: blob entry outside core0");
 if (task) {
  if (ctx && ctx != &task->blob)
   panic("esp32s31-radio: task enter with foreign window");
  context = &task->blob;
 } else if (service) {
  context = &s31_service_context.blob;
 } else {
  /* Only exclusive, non-sleeping pre-task metadata bootstrap can borrow
   * this CPU-local descriptor. General foreign vendor entry is removed. */
  if (!s31_control_bootstrap_owns(ctx))
   panic("esp32s31-radio: unadmitted foreign vendor entry");
  migrate_disable();
  s31_foreign_publish(ctx);
  context = this_cpu_read(s31_foreign_installed);
 }
 s31_blob_fpu_begin();
 s31_blob_set_active(true);
 s31_blob_install_context(context);

}

void s31_linux_blob_leave(struct s31_blob_context *ctx)
{
 struct s31_linux_task *task = s31_linux_current_task();
 bool service = s31_current_service();
 struct s31_blob_context *context;

 if (task) {
  if (ctx && ctx != &task->blob)
   panic("esp32s31-radio: task leave with foreign window");
  context = &task->blob;
 } else if (service) {
  context = &s31_service_context.blob;
 } else {
  if (!s31_control_bootstrap_owns(ctx) ||
      this_cpu_read(s31_foreign_installed) != ctx)
   panic("esp32s31-radio: bootstrap leave without own window");
  context = ctx;
 }
 s31_radio_blob_run_pending_isrs();
 s31_blob_fpu_end();
 s31_blob_restore_context(context);
 s31_blob_set_active(false);
 if (!task && !service) esp32s31_blob_fp_foreign_leave();
 s31_blob_unfence_context(context);
 if (!task && !service) migrate_enable();
}

struct s31_blob_context *s31_linux_blob_suspend(u32 reason)
{
 struct s31_linux_task *task = s31_linux_current_task();
 bool service = s31_current_service();
 struct s31_blob_context *context;

 (void)reason;
 if (!task && !service)
  panic("esp32s31-radio: non-native vendor context attempted to sleep");

 if (task && task_stack_end_corrupted(current))
  panic("esp32s31-radio: native SRAM stack guard overwritten (%s)", current->comm);
 context = task ? &task->blob : &s31_service_context.blob;
 s31_linux_critical_suspend();
 s31_radio_blob_run_pending_isrs();
 if (s31_blob_active()) {
  s31_blob_fpu_end();
  s31_blob_restore_context(context);
  s31_blob_set_active(false);
  s31_blob_unfence_context(context);
 }
 return context;
}

void s31_linux_blob_resume(struct s31_blob_context *ctx)
{
 struct s31_linux_task *task = s31_linux_current_task();
 bool service = s31_current_service();
 struct s31_blob_context *context;

 s31_control_owner_assert_vendor_entry(ctx);
 if (!task && !service)
  panic("esp32s31-radio: non-native vendor context attempted to resume");
 context = task ? &task->blob : &s31_service_context.blob;
 if (ctx && ctx != context)
  panic("esp32s31-radio: native resume lost its descriptor");
 if (!s31_blob_active()) {
  s31_blob_fpu_begin();
  s31_blob_set_active(true);
  s31_blob_install_context(context);

 }
 s31_linux_critical_resume();
}
