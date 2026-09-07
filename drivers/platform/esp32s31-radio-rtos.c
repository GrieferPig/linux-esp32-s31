// SPDX-License-Identifier: GPL-2.0
/* Linux half of the Wi-Fi blob's FreeRTOS compatibility bridge. */

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/cpu.h>
#include <linux/fpu.h>
#include <linux/interrupt.h>
#include <linux/mm_types.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/sched/cputime.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/thread_info.h>
#include <linux/wait.h>

#include <asm/csr.h>
#include <asm/esp32s31_ext.h>

#include "esp32s31-radio-internal.h"

extern void sched_show_task(struct task_struct *p);

/*
 * Switch the kthread's sp to the internal-SRAM payload stack, run the
 * payload entry, then restore the original kthread sp/ra.  The payload may
 * sleep on its own stack via the bridge (kernel sleep frames fit the 16-KiB
 * minimum); only the Linux trampoline below ever runs on the kthread stack.
 *
 * save[14]: 0=ra 1=sp 2..13=s0..s11.  Saving the complete integer
 * callee-saved set is required for the vTaskDelete(self) escape path, which
 * intentionally bypasses the payload's normal ABI unwinding.
 */
__attribute__((naked)) noinline static void s31_payload_run(unsigned long *save,
							     unsigned long top,
							     void (*entry)(void *),
							     void *arg)
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
		"mv t1, a2\n\t"
		"mv sp, a1\n\t"
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
		"ret\n\t");
}

#define S31_LINUX_TASK_MAGIC 0x53333154U
#define S31_RADIO_EXC_STACK_TOP 0x2f072360UL

/*
 * The S31 CLIC entry path takes its exception stack from
 * current_thread_info()->kernel_sp.  The blob's normal kthread stack is in
 * Linux-managed memory and cannot be relied on while the IDF code is changing
 * cache/MMU state.  The bootloader reserves this internal-SRAM stack for the
 * serialized radio execution window.
 */
struct s31_blob_context {
	unsigned long saved_kernel_sp;
	unsigned long irq_flags;
	u32 fp_saved[13]; /* fs0..fs11, fcsr for worker-side bridge waits */
	bool fp_valid;
	bool stack_switched;
	bool irq_disabled;
	u64 gate_acquired_ns;
	u64 gate_exec_start_ns;
	u32 gate_timing_generation;
	u32 tick_acquired;
	u32 last_wifi_event;
	bool last_wifi_event_valid;
};

static struct s31_blob_context *s31_blob_context_current(void);

struct s31_linux_task {
	u32 magic;
	struct task_struct *thread;
	void (*entry)(void *);
	void *arg;
	void *cookie;
	struct completion exited;
	atomic_t stopping;
	bool blob_active;
	bool critical_active;
	bool critical_suspended;
	u32 sync_lock_depth;
	unsigned long sync_irq_flags;
	unsigned long critical_flags;
	struct s31_blob_context blob;
	/* Payload execution stack in internal SRAM.  The PHY changes the
	 * external-memory/cache state while the payload runs, so the DRAM
	 * kthread stack is unusable for payload code. */
	void *payload_stack;
	u32 payload_stack_size;
	unsigned long sp_save[14];
	/* ILP32F callee-saved payload context.  These registers must be preserved
	 * explicitly across every FreeRTOS-style blocking point. */
	u32 fp_saved[13]; /* fs0..fs11, fcsr */
	bool fp_valid;
	u32 payload_stack_peak;
	s32 requested_cpu;
	struct s31_linux_task *next;
};

struct s31_linux_sync {
	raw_spinlock_t lock;
	atomic_t sequence;
	wait_queue_head_t waitq;
};

static DEFINE_PER_CPU(struct s31_blob_context, s31_foreign_blob_context);
static atomic_t s31_sync_stop_generation = ATOMIC_INIT(0);
static DEFINE_RAW_SPINLOCK(s31_critical_lock);
static DEFINE_MUTEX(s31_blob_mutex);
static struct s31_linux_task *s31_task_list;
static DEFINE_SPINLOCK(s31_task_list_lock);

/*
 * Radio payloads run only on dedicated PF_KTHREAD contexts.  They have no
 * userspace FP state to preserve, while s31_payload_fp_save_area() already
 * retains the ILP32F callee-saved state across every payload blocking point.
 * Avoid the generic kernel_fpu_begin()/end() pair here: it saves and restores
 * all 32 registers on every BTDM queue wake even though the surrounding
 * kthread never consumes that state.  Preemption remains disabled throughout
 * each payload execution window, and the direct hardirq path still saves the
 * complete interrupted FP register file before invoking a closed ISR.
 *
 * The pinned IDF radio objects contain no Xesploop or Xespv instructions, so
 * these kthreads deliberately do not opt into esp32s31_ext_kernel_begin().
 * User extension state is still suspended on kernel entry and managed by the
 * architecture code; paying two SBI transitions at every payload wait would
 * only save and restore an extension state that the radio never consumes.
 */
static void s31_blob_fpu_begin(void)
{
	WARN_ON_ONCE(!(current->flags & PF_KTHREAD));
	preempt_disable();
	if (s31_radio_payload_uses_fp())
		csr_set(CSR_SSTATUS, SR_FS);
}

static void s31_blob_fpu_end(void)
{
	if (s31_radio_payload_uses_fp())
		csr_clear(CSR_SSTATUS, SR_FS);
	preempt_enable();
}

/* Production keeps this disabled.  Retain one opt-in sample of each kind
 * without reserving more than 10 KiB of module BSS for dormant diagnostics. */
#define S31_GATE_TIMING_SLOTS 1
#define S31_GATE_LONGEST_SAMPLES 1

struct s31_gate_reason_timing {
	u64 wall_total_ns;
	u64 wall_max_ns;
	u64 exec_total_ns;
	u64 exec_max_ns;
	u64 offcpu_total_ns;
	u64 offcpu_max_ns;
	u32 count;
};

struct s31_gate_timing_slot {
	struct task_struct *thread;
	char name[TASK_COMM_LEN];
	u64 wait_total_ns;
	u64 wait_max_ns;
	u32 enters;
	struct s31_gate_reason_timing reason[S31_BLOB_RELEASE_COUNT];
};

struct s31_gate_long_sample {
	char name[TASK_COMM_LEN];
	u32 reason;
	u64 wall_ns;
	u64 exec_ns;
	u64 offcpu_ns;
	u32 last_wifi_event;
	bool last_wifi_event_valid;
};

static DEFINE_SPINLOCK(s31_gate_timing_lock);
static struct s31_gate_timing_slot s31_gate_timing[S31_GATE_TIMING_SLOTS];
static struct s31_gate_long_sample
	s31_gate_longest[S31_GATE_LONGEST_SAMPLES];
static u32 s31_gate_timing_generation;
static bool s31_gate_timing_enabled;

static const char * const s31_gate_reason_name[S31_BLOB_RELEASE_COUNT] = {
	[S31_BLOB_RELEASE_LEAVE] = "leave",
	[S31_BLOB_RELEASE_TASK_DELAY] = "task-delay",
	[S31_BLOB_RELEASE_TASK_YIELD] = "task-yield",
	[S31_BLOB_RELEASE_QUEUE_SEND] = "queue-send",
	[S31_BLOB_RELEASE_QUEUE_RECEIVE] = "queue-receive",
	[S31_BLOB_RELEASE_SEMAPHORE_TAKE] = "semaphore-take",
	[S31_BLOB_RELEASE_NOTIFY_TAKE] = "notify-take",
	[S31_BLOB_RELEASE_NOTIFY_WAIT] = "notify-wait",
	[S31_BLOB_RELEASE_EVENT_WAIT] = "event-wait",
	[S31_BLOB_RELEASE_TASK_SUSPEND] = "task-suspend",
};

struct s31_gate_release_info {
	bool valid;
	u32 reason;
	u64 wall_ns;
	u64 exec_ns;
	u32 tick_start;
};

static struct s31_gate_timing_slot *s31_gate_timing_slot_locked(void)
{
	struct s31_gate_timing_slot *free = NULL;
	int i;

	for (i = 0; i < S31_GATE_TIMING_SLOTS; i++) {
		if (s31_gate_timing[i].thread == current)
			return &s31_gate_timing[i];
		if (!s31_gate_timing[i].thread && !free)
			free = &s31_gate_timing[i];
	}
	if (!free)
		free = &s31_gate_timing[S31_GATE_TIMING_SLOTS - 1];
	if (!free->thread) {
		free->thread = current;
		strscpy(free->name, current->comm, sizeof(free->name));
	}
	return free;
}

static void s31_gate_timing_acquired(struct s31_blob_context *context,
				     u64 wait_start_ns, u64 acquired_ns)
{
	struct s31_gate_timing_slot *slot;
	unsigned long flags;
	u64 wait_ns = acquired_ns - wait_start_ns;

	/* The gate-hold histogram is diagnostic-only and proved the earlier
	 * bottleneck (ROM UART polling in esp_rom_printf).  Its bookkeeping
	 * (task_sched_runtime() + spin_lock_irqsave on every blob enter) is
	 * measurable overhead in the hot path, so keep it compiled out of the
	 * acquire/release fast path unless explicitly re-enabled. */
	if (!READ_ONCE(s31_gate_timing_enabled)) {
		context->gate_timing_generation = 0;
		context->gate_acquired_ns = 0;
		context->gate_exec_start_ns = 0;
		context->tick_acquired = 0;
		return;
	}
	context->tick_acquired = s31_linux_tick_count();
	spin_lock_irqsave(&s31_gate_timing_lock, flags);
	slot = s31_gate_timing_slot_locked();
	slot->wait_total_ns += wait_ns;
	slot->wait_max_ns = max(slot->wait_max_ns, wait_ns);
	slot->enters++;
	context->gate_timing_generation = s31_gate_timing_generation;
	context->gate_acquired_ns = acquired_ns;
	context->gate_exec_start_ns = esp32s31_task_runtime(current);
	context->last_wifi_event_valid = false;
	spin_unlock_irqrestore(&s31_gate_timing_lock, flags);
}

static void s31_gate_record_longest_locked(u32 reason, u64 wall_ns,
					   u64 exec_ns, u64 offcpu_ns)
{
	struct s31_gate_long_sample *sample;
	int i, pos = -1;

	for (i = 0; i < S31_GATE_LONGEST_SAMPLES; i++) {
		if (wall_ns > s31_gate_longest[i].wall_ns) {
			pos = i;
			break;
		}
	}
	if (pos < 0)
		return;
	for (i = S31_GATE_LONGEST_SAMPLES - 1; i > pos; i--)
		s31_gate_longest[i] = s31_gate_longest[i - 1];
	sample = &s31_gate_longest[pos];
	strscpy(sample->name, current->comm, sizeof(sample->name));
	sample->reason = reason;
	sample->wall_ns = wall_ns;
	sample->exec_ns = exec_ns;
	sample->offcpu_ns = offcpu_ns;
	sample->last_wifi_event = s31_blob_context_current()->last_wifi_event;
	sample->last_wifi_event_valid =
		s31_blob_context_current()->last_wifi_event_valid;
}

static void s31_gate_timing_released(struct s31_blob_context *context,
				     u32 reason,
				     struct s31_gate_release_info *info)
{
	struct s31_gate_timing_slot *slot;
	struct s31_gate_reason_timing *timing;
	unsigned long flags;
	u64 now, exec_now;
	u64 wall_ns = 0, exec_ns = 0, offcpu_ns = 0;

	if (info)
		info->valid = false;
	if (reason >= S31_BLOB_RELEASE_COUNT)
		reason = S31_BLOB_RELEASE_LEAVE;

	if (!READ_ONCE(s31_gate_timing_enabled) || !context->gate_acquired_ns) {
		context->gate_acquired_ns = 0;
		context->gate_exec_start_ns = 0;
		context->gate_timing_generation = 0;
		context->tick_acquired = 0;
		return;
	}
	now = ktime_get_mono_fast_ns();
	exec_now = esp32s31_task_runtime(current);

	spin_lock_irqsave(&s31_gate_timing_lock, flags);
	if (context->gate_timing_generation != s31_gate_timing_generation)
		goto out;
	wall_ns = now - context->gate_acquired_ns;
	exec_ns = exec_now - context->gate_exec_start_ns;
	offcpu_ns = wall_ns > exec_ns ? wall_ns - exec_ns : 0;
	slot = s31_gate_timing_slot_locked();
	timing = &slot->reason[reason];
	timing->wall_total_ns += wall_ns;
	timing->wall_max_ns = max(timing->wall_max_ns, wall_ns);
	timing->exec_total_ns += exec_ns;
	timing->exec_max_ns = max(timing->exec_max_ns, exec_ns);
	timing->offcpu_total_ns += offcpu_ns;
	timing->offcpu_max_ns = max(timing->offcpu_max_ns, offcpu_ns);
	timing->count++;
	s31_gate_record_longest_locked(reason, wall_ns, exec_ns, offcpu_ns);
	if (info) {
		info->valid = true;
		info->reason = reason;
		info->wall_ns = wall_ns;
		info->exec_ns = exec_ns;
		info->tick_start = context->tick_acquired;
	}
out:
	context->gate_acquired_ns = 0;
	context->gate_exec_start_ns = 0;
	context->gate_timing_generation = 0;
	context->tick_acquired = 0;
	spin_unlock_irqrestore(&s31_gate_timing_lock, flags);
}

void s31_linux_gate_timing_reset(void)
{
	unsigned long flags;

	spin_lock_irqsave(&s31_gate_timing_lock, flags);
	memset(s31_gate_timing, 0, sizeof(s31_gate_timing));
	memset(s31_gate_longest, 0, sizeof(s31_gate_longest));
	s31_gate_timing_generation++;
	if (!s31_gate_timing_generation)
		s31_gate_timing_generation++;
	/* Leave the histogram disabled: it was diagnostic for the UART-poll
	 * hold and its per-enter bookkeeping now costs hot-path cycles. */
	s31_gate_timing_enabled = false;
	spin_unlock_irqrestore(&s31_gate_timing_lock, flags);
}

void s31_linux_gate_timing_report(const char *stage)
{
	unsigned long flags;
	int i, reason;

	spin_lock_irqsave(&s31_gate_timing_lock, flags);
	s31_gate_timing_enabled = false;
	spin_unlock_irqrestore(&s31_gate_timing_lock, flags);

	for (i = 0; i < S31_GATE_TIMING_SLOTS; i++) {
		char name[TASK_COMM_LEN];
		u64 wait_total_ns, wait_max_ns;
		u32 enters;

		spin_lock_irqsave(&s31_gate_timing_lock, flags);
		strscpy(name, s31_gate_timing[i].name, sizeof(name));
		wait_total_ns = s31_gate_timing[i].wait_total_ns;
		wait_max_ns = s31_gate_timing[i].wait_max_ns;
		enters = s31_gate_timing[i].enters;
		spin_unlock_irqrestore(&s31_gate_timing_lock, flags);

		if (!name[0])
			continue;
		pr_info("esp32s31-radio: gate timing %s task=%s enter=%u wait_avg=%lluns wait_max=%lluns\n",
			stage, name, enters,
			enters ? div_u64(wait_total_ns, enters) : 0,
			wait_max_ns);
		for (reason = 0; reason < S31_BLOB_RELEASE_COUNT; reason++) {
			struct s31_gate_reason_timing timing;

			spin_lock_irqsave(&s31_gate_timing_lock, flags);
			timing = s31_gate_timing[i].reason[reason];
			spin_unlock_irqrestore(&s31_gate_timing_lock, flags);
			if (!timing.count)
				continue;
			pr_info("esp32s31-radio: gate hold %s task=%s reason=%s count=%u wall_avg=%lluns wall_max=%lluns exec_avg=%lluns exec_max=%lluns offcpu_avg=%lluns offcpu_max=%lluns\n",
				stage, name, s31_gate_reason_name[reason],
				timing.count,
				div_u64(timing.wall_total_ns, timing.count),
				timing.wall_max_ns,
				div_u64(timing.exec_total_ns, timing.count),
				timing.exec_max_ns,
				div_u64(timing.offcpu_total_ns, timing.count),
				timing.offcpu_max_ns);
		}
	}
	for (i = 0; i < S31_GATE_LONGEST_SAMPLES; i++) {
		struct s31_gate_long_sample sample;

		spin_lock_irqsave(&s31_gate_timing_lock, flags);
		sample = s31_gate_longest[i];
		spin_unlock_irqrestore(&s31_gate_timing_lock, flags);
		if (!sample.wall_ns)
			continue;
		pr_info("esp32s31-radio: gate longest %s rank=%d task=%s reason=%s wall=%lluns exec=%lluns offcpu=%lluns last_event=%s%u\n",
			stage, i + 1, sample.name,
			s31_gate_reason_name[sample.reason], sample.wall_ns,
			sample.exec_ns, sample.offcpu_ns,
			sample.last_wifi_event_valid ? "" : "none/",
			sample.last_wifi_event);
	}
}

void s31_linux_trace_wifi_event(u32 event)
{
	struct s31_blob_context *context = s31_blob_context_current();

	context->last_wifi_event = event;
	context->last_wifi_event_valid = true;
}

/* Set by the radio worker while it waits for the blob gate.  The worker is
 * the only thread that may touch the CLIC irqchip state, so while a task
 * holds the gate (and waits for an interrupt-driven payload event) the
 * worker can still install/enable the deferred IDF IRQ mappings.  Without
 * this, a task stuck inside e.g. chm_phy_change_channel() would never see
 * its interrupt: the CLIC source was never enabled because the worker could
 * not run its sync pass. */
void (*s31_blob_gate_wait_hook)(void);
static bool s31_gate_hook_busy;
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

static struct s31_linux_task *s31_linux_current_task(void)
{
	struct s31_linux_task *task = kthread_data(current);

	if (!task || task->magic != S31_LINUX_TASK_MAGIC)
		return NULL;
	return task;
}

static noinline void s31_payload_fp_save_area(u32 *fp_saved, bool *fp_valid)
{
	/* RISC-V trap entry clears sstatus.FS.  A blocking bridge call may be
	 * reached from a native IDF ISR, so do not rely on the task-side FPU gate
	 * state surviving that trap. */
	csr_set(CSR_SSTATUS, SR_FS);
	asm volatile(
		"fsw fs0, 0(%0)\n\t"  "fsw fs1, 4(%0)\n\t"
		"fsw fs2, 8(%0)\n\t"  "fsw fs3, 12(%0)\n\t"
		"fsw fs4, 16(%0)\n\t" "fsw fs5, 20(%0)\n\t"
		"fsw fs6, 24(%0)\n\t" "fsw fs7, 28(%0)\n\t"
		"fsw fs8, 32(%0)\n\t" "fsw fs9, 36(%0)\n\t"
		"fsw fs10, 40(%0)\n\t" "fsw fs11, 44(%0)\n\t"
		"frcsr t0\n\t" "sw t0, 48(%0)\n\t"
		: : "r" (fp_saved) : "t0", "memory");
	*fp_valid = true;
}

static noinline void s31_payload_fp_restore_area(u32 *fp_saved, bool fp_valid)
{
	if (!fp_valid)
		return;
	csr_set(CSR_SSTATUS, SR_FS);
	asm volatile(
		"flw fs0, 0(%0)\n\t"  "flw fs1, 4(%0)\n\t"
		"flw fs2, 8(%0)\n\t"  "flw fs3, 12(%0)\n\t"
		"flw fs4, 16(%0)\n\t" "flw fs5, 20(%0)\n\t"
		"flw fs6, 24(%0)\n\t" "flw fs7, 28(%0)\n\t"
		"flw fs8, 32(%0)\n\t" "flw fs9, 36(%0)\n\t"
		"flw fs10, 40(%0)\n\t" "flw fs11, 44(%0)\n\t"
		"lw t0, 48(%0)\n\t" "fscsr t0\n\t"
		: : "r" (fp_saved) : "t0", "memory");
}

static void s31_payload_stack_measure(struct s31_linux_task *task)
{
	unsigned long sp;
	unsigned long base;
	unsigned long top;
	u32 used;
	u32 old_peak;

	if (!task || !task->payload_stack || !task->payload_stack_size)
		return;
	asm volatile("mv %0, sp" : "=r" (sp));
	base = (unsigned long)task->payload_stack;
	top = base + task->payload_stack_size;
	if (sp < base || sp > top) {
		pr_err_ratelimited("esp32s31-radio: %s bridge sp=%08lx outside payload stack %08lx..%08lx\n",
			current->comm, sp, base, top);
		return;
	}
	used = top - sp;
	old_peak = task->payload_stack_peak;
	if (used <= old_peak)
		return;
	task->payload_stack_peak = used;
	if (used >= task->payload_stack_size - 2048 ||
	    used / 512 != old_peak / 512)
		pr_info("esp32s31-radio: %s payload stack peak=%u/%u sp=%08lx\n",
			current->comm, used, task->payload_stack_size, sp);
}

void s31_linux_call_on_stack(void *stack, u32 stack_size,
			     void (*entry)(void *), void *arg)
{
	unsigned long save[14];

	if (!stack || stack_size < 1024 || !entry)
		return;
	s31_payload_run(save, (unsigned long)stack + stack_size - 16,
			entry, arg);
}

static struct s31_blob_context *s31_blob_context_current(void)
{
	struct s31_linux_task *task = s31_linux_current_task();

	return task ? &task->blob : this_cpu_ptr(&s31_foreign_blob_context);
}

static bool s31_blob_active(void)
{
	struct s31_linux_task *task = s31_linux_current_task();

	return task ? task->blob_active :
		this_cpu_read(s31_foreign_blob_context.stack_switched);
}

static void s31_blob_set_active(bool active)
{
	struct s31_linux_task *task = s31_linux_current_task();

	if (task)
		task->blob_active = active;
	else if (!active)
		this_cpu_write(s31_foreign_blob_context.stack_switched, false);
}

static void s31_blob_install_context(struct s31_blob_context *context)
{
	unsigned long kernel_sp;

	if (context->stack_switched)
		return;
	kernel_sp = current_thread_info()->kernel_sp;
	context->saved_kernel_sp = kernel_sp;
	current_thread_info()->kernel_sp = S31_RADIO_EXC_STACK_TOP;
	context->stack_switched = true;
}

static void s31_blob_restore_context(struct s31_blob_context *context)
{
	if (!context->stack_switched)
		return;
	current_thread_info()->kernel_sp = context->saved_kernel_sp;
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
		mutex_unlock(&s31_blob_mutex);
		task->blob_active = false;
	}
	complete(&task->exited);
}

const char *s31_linux_blob_holder(void)
{
	struct task_struct *owner;

	/* Classic (non-RT) mutex: owner field with MUTEX_FLAG_* bits. */
	owner = (struct task_struct *)(atomic_long_read(&s31_blob_mutex.owner) & ~7UL);
	if (!owner)
		return NULL;
	return owner->comm;
}

static struct task_struct *s31_linux_blob_owner(void)
{
	/* Classic (non-RT) mutex: owner field with MUTEX_FLAG_* bits. */
	return (struct task_struct *)
		(atomic_long_read(&s31_blob_mutex.owner) & ~7UL);
}

bool s31_linux_blob_held_by_current(void)
{
	/* Callable from hard IRQ context: current is the interrupted task. */
	return s31_linux_blob_owner() == current;
}

/*
 * Linux clears sstatus.FS on every trap entry.  When Wi-Fi is enabled, a
 * native IDF ISR preserves the complete interrupted FP register file because
 * its payload objects contain FP instructions.  The pinned BTDM object region
 * is integer-only, so a Bluetooth-only runtime can avoid this save/restore.
 */
static noinline void s31_direct_isr_fp_save(u32 *fp)
{
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
		: : "r" (fp) : "t0", "memory");
}

static noinline void s31_direct_isr_fp_restore(u32 *fp)
{
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
		: : "r" (fp) : "t0", "memory");
	csr_clear(CSR_SSTATUS, SR_FS);
}

int s31_linux_blob_run_direct_isr(void (*handler)(void *), void *arg)
{
	struct s31_blob_context *context;
	struct task_struct *owner;
	struct s31_linux_task *task;
	bool preserve_fp;
	u32 fp[33] __aligned(16);

	/* User page tables do not contain the blob's low identity mapping.  Every
	 * compatibility task and the radio worker is a CPU0 kthread using init_mm;
	 * other contexts retain the deferred fallback. */
	if (!handler || !in_hardirq() || raw_smp_processor_id() != 0 ||
	    !(current->flags & PF_KTHREAD) ||
	    !esp32s31_current_uses_init_mm())
		return S31_DIRECT_ISR_DEFER_CONTEXT;
	task = s31_linux_current_task();
	if (task && ((task->critical_active && !task->critical_suspended) ||
		     READ_ONCE(task->sync_lock_depth)))
		return S31_DIRECT_ISR_DEFER_UNSAFE;
	if (READ_ONCE(s31_rtos_isr_depth))
		return S31_DIRECT_ISR_DEFER_NESTED;
	context = s31_blob_context_current();
	owner = s31_linux_blob_owner();

	/* Native nesting is safe only for a compatibility task, whose critical and
	 * sync-lock state is published above.  The radio worker has an installed
	 * foreign context too, but it can hold an SRAM sync lock without task-side
	 * lock-depth bookkeeping; nesting there would self-deadlock the closed ISR
	 * on that same lock. */
	if (owner == current) {
		if (!task || !s31_blob_active() || !context->stack_switched)
			return S31_DIRECT_ISR_DEFER_OWNER;
		preserve_fp = s31_radio_payload_uses_fp();
		if (preserve_fp)
			s31_direct_isr_fp_save(fp);
		s31_rtos_isr_depth++;
		handler(arg);
		s31_rtos_isr_depth--;
		if (preserve_fp)
			s31_direct_isr_fp_restore(fp);
		return S31_DIRECT_ISR_HANDLED;
	}
	if (owner)
		return S31_DIRECT_ISR_DEFER_OWNER;

	/*
	 * A free blob gate does not provide a native RTOS interrupt context.  In
	 * particular, the S31 CLIC entry path reuses a fixed exception stack and
	 * cannot safely tolerate the recursive trap window opened by running the
	 * closed ISR on an invented context.  Defer until a real owner or the
	 * serialized worker has installed the blob context.
	 */
	return S31_DIRECT_ISR_DEFER_CONTEXT;
}

uint64_t s31_linux_time_ns(void)
{
	return ktime_get_mono_fast_ns();
}

int32_t s31_linux_current_cpu(void)
{
	/* The blob gate disables preemption before payload code can query this,
	 * so the returned HP-hart identity remains stable for the duration of the
	 * IDF critical-section/portMUX operation. */
	return raw_smp_processor_id();
}

void s31_linux_printf(const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vprintk(fmt, args);
	va_end(args);
}

void s31_linux_task_dump_all(void)
{
	struct s31_linux_task *task;

	spin_lock(&s31_task_list_lock);
	for (task = s31_task_list; task; task = task->next)
		sched_show_task(task->thread);
	spin_unlock(&s31_task_list_lock);
	pr_info("esp32s31-radio: blob gate holder=%s\n",
		s31_linux_blob_holder() ?: "none");
}

static int s31_linux_task_main(void *arg)
{
	struct s31_linux_task *task = arg;

	esp32s31_kthread_use_init_mm();
	pr_info("esp32s31-radio: compatibility task %s entered\n", current->comm);
	s31_linux_blob_enter();
	s31_payload_run(task->sp_save,
			(unsigned long)task->payload_stack +
				task->payload_stack_size - 16,
			task->entry, task->arg);
	/* Reached on normal entry return or after vTaskDelete(self). */
	if (task->blob_active)
		s31_linux_blob_leave();
	pr_info("esp32s31-radio: compatibility task %s returned\n", current->comm);
	esp32s31_kthread_unuse_init_mm();
	s31_linux_task_cleanup(task);
	s31_linux_task_unregister(task);
	s31_rtos_free(task->payload_stack);
	s31_rtos_task_release(task->cookie);
	task->magic = 0;
	s31_radio_sram_free(task);
	return 0;
}

void *s31_linux_task_create(void (*entry)(void *), const char *name,
				    u32 stack_size, void *stack_base,
				    void *arg, u32 priority, void *cookie,
				    s32 core_id)
{
	struct s31_linux_task *task;
	struct sched_param param = { };
	bool realtime_task;

	if (!entry || !stack_base)
		return NULL;
	task = s31_radio_sram_alloc(sizeof(*task));
	if (!task)
		return NULL;
	memset(task, 0, sizeof(*task));
	task->magic = S31_LINUX_TASK_MAGIC;
	task->entry = entry;
	task->arg = arg;
	task->cookie = cookie;
	task->payload_stack = stack_base;
	task->payload_stack_size = stack_size;
	task->requested_cpu = core_id;
	init_completion(&task->exited);
	atomic_set(&task->stopping, 0);
	task->thread = kthread_create(s31_linux_task_main, task, "%s",
				      name && *name ? name : "s31-task");
	if (IS_ERR(task->thread)) {
		s31_radio_sram_free(task);
		return NULL;
	}
	spin_lock(&s31_task_list_lock);
	task->next = s31_task_list;
	s31_task_list = task;
	spin_unlock(&s31_task_list_lock);
	/* Keep IDF's requested affinity intact.  The blob execution gate remains
	 * the mutual-exclusion boundary, but releasing it while an IDF task blocks
	 * must allow the next native-designated task to run on the other HP core.
	 * This is required for dual-core IDF builds; NO_AFFINITY stays migratable. */
	if (core_id >= 0 && core_id < nr_cpu_ids && cpu_online(core_id))
		kthread_bind(task->thread, core_id);
	/* BTDM has hard controller deadlines on the native dedicated core.  In the
	 * Linux shared-core combo profile, SCHED_RR/80 preempts both the Wi-Fi task
	 * and the TCP-ACK worker whenever media IRQs are pending.  Keep RT only for
	 * BT-only mode; combo uses a high-priority CFS controller task so both
	 * clients receive service on the one compatibility hart. */
	realtime_task = name && !strcmp(name, "btdm") &&
		!s31_radio_payload_uses_fp();
	if (realtime_task) {
		param.sched_priority = 80;
		esp32s31_sched_setscheduler(task->thread, SCHED_RR,
					   param.sched_priority);
	} else {
		esp32s31_sched_setscheduler(task->thread, SCHED_NORMAL, 0);
		set_user_nice(task->thread,
			name && !strcmp(name, "btdm") ? -10 :
			name && !strcmp(name, "wifi") ? -10 : 5);
	}
	pr_info("esp32s31-radio: task %s FreeRTOS-prio=%u core=%d Linux=%s/%d\n",
		name && *name ? name : "s31-task", priority,
		core_id,
		realtime_task ? "RR" : "CFS",
		realtime_task ? param.sched_priority : task_nice(task->thread));
	wake_up_process(task->thread);
	return task;
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
	if (task->blob_active) {
		s31_linux_blob_leave();
		task->blob_active = false;
	}
	/* Longjmp to the trampoline continuation on the kthread stack; it
	 * performs the unuse_mm/cleanup/unregister/free sequence. */
	s31_payload_sp_restore(task->sp_save);
	unreachable();
}

int32_t s31_linux_task_stop(void *opaque)
{
	struct s31_linux_task *task = opaque;

	if (!task || task->magic != S31_LINUX_TASK_MAGIC)
		return -EINVAL;
	if (task == s31_linux_current_task()) {
		s31_linux_task_exit_current();
		unreachable();
	}
	atomic_set(&task->stopping, 1);
	atomic_inc(&s31_sync_stop_generation);
	wake_up_process(task->thread);
	kthread_stop(task->thread);
	return 0;
}

void s31_linux_tasks_stop_all(void)
{
	for (;;) {
		struct s31_linux_task *task;
		struct task_struct *thread;

		spin_lock(&s31_task_list_lock);
		task = s31_task_list;
		if (!task) {
			spin_unlock(&s31_task_list_lock);
			break;
		}
		atomic_set(&task->stopping, 1);
		thread = task->thread;
		get_task_struct(thread);
		spin_unlock(&s31_task_list_lock);

		atomic_inc(&s31_sync_stop_generation);
		wake_up_process(thread);
		kthread_stop(thread);
		put_task_struct(thread);
	}
}

void s31_linux_task_delay(u32 ticks)
{
	struct s31_linux_task *task = s31_linux_current_task();
	unsigned long timeout;

	if (!task || atomic_read(&task->stopping))
		return;
	if (!ticks) {
		cond_resched();
		return;
	}
	timeout = s31_ticks_to_jiffies(ticks);
	s31_linux_blob_suspend(S31_BLOB_RELEASE_TASK_DELAY);
	set_current_state(TASK_INTERRUPTIBLE);
	if (timeout == MAX_SCHEDULE_TIMEOUT)
		schedule();
	else
		schedule_timeout(timeout);
	__set_current_state(TASK_RUNNING);
	s31_linux_blob_resume();
}

void s31_linux_task_yield(void)
{
	struct s31_linux_task *task = s31_linux_current_task();

	if (!task || atomic_read(&task->stopping))
		return;
	/* FreeRTOS vTaskDelay(0) is an explicit scheduler yield. */
	s31_linux_blob_suspend(S31_BLOB_RELEASE_TASK_YIELD);
	yield();
	s31_linux_blob_resume();
}

void s31_linux_task_set_priority(void *opaque, u32 priority)
{
	struct s31_linux_task *task = opaque;
	struct sched_param param = { };
	bool realtime_task;

	if (!task || task->magic != S31_LINUX_TASK_MAGIC)
		return;
	/* Controller priority changes are normal during BTDM bring-up.  In the
	 * combo profile they must preserve the CFS policy selected at creation;
	 * promoting the task back to RR/80 here silently starves Wi-Fi despite the
	 * shared-core policy above. */
	realtime_task = !strcmp(task->thread->comm, "btdm") &&
		!s31_radio_payload_uses_fp();
	if (realtime_task) {
		param.sched_priority = 80;
		esp32s31_sched_setscheduler(task->thread, SCHED_RR,
					   param.sched_priority);
	} else {
		esp32s31_sched_setscheduler(task->thread, SCHED_NORMAL, 0);
		set_user_nice(task->thread,
			!strcmp(task->thread->comm, "btdm") ||
			!strcmp(task->thread->comm, "wifi") ? -10 : 5);
	}
	pr_info("esp32s31-radio: task %s priority update FreeRTOS=%u\n",
		task->thread->comm, priority);
}

void *s31_linux_sync_create(void)
{
	struct s31_linux_sync *sync = s31_radio_sram_alloc(sizeof(*sync));

	if (sync)
		raw_spin_lock_init(&sync->lock);
	if (sync)
		atomic_set(&sync->sequence, 0);
	if (sync)
		init_waitqueue_head(&sync->waitq);
	return sync;
}

void s31_linux_sync_destroy(void *opaque)
{
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
	 * every BTDM queue operation.  The outermost depth owns the saved state. */
	if (task) {
		if (!task->sync_lock_depth) {
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
			}
		}
		s31_radio_blob_run_pending_isrs();
	}
}

u32 s31_linux_sync_sequence(void *opaque)
{
	struct s31_linux_sync *sync = opaque;

	return sync ? atomic_read(&sync->sequence) : 0;
}

int32_t s31_linux_sync_wait(void *opaque, u32 sequence, u32 ticks, u32 reason)
{
	struct s31_linux_sync *sync = opaque;
	int stop_generation = atomic_read(&s31_sync_stop_generation);
	long timeout = s31_ticks_to_jiffies(ticks);
	long ret;

	if (!sync || !timeout)
		return 0;
	s31_linux_blob_suspend(reason);
	ret = wait_event_interruptible_timeout(sync->waitq,
		atomic_read(&sync->sequence) != sequence ||
		atomic_read(&s31_sync_stop_generation) != stop_generation ||
		kthread_should_stop(), timeout);
	/* vTaskDelete(other) is implemented with kthread_stop().  Waking the
	 * blocked FreeRTOS primitive is not sufficient: most blob tasks retry
	 * their queue/event wait after an error.  More importantly, the deleting
	 * task still owns the blob gate while it waits in kthread_stop(), so the
	 * target must escape before trying to reacquire that gate or the two tasks
	 * deadlock.  Let the normal task trampoline run compatibility clean-up. */
	if (kthread_should_stop()) {
		s31_linux_task_exit_current();
		unreachable();
	}
	s31_linux_blob_resume();
	if (ret < 0)
		return -1;
	return ret ? 1 : 0;
}

void s31_linux_sync_wake(void *opaque)
{
	struct s31_linux_sync *sync = opaque;

	if (!sync)
		return;
	atomic_inc(&sync->sequence);
	/* A Wi-Fi RX interrupt usually makes exactly one queue consumable.  A
	 * global waitqueue woke every compatibility task for every frame and
	 * turned non-aggregated receive into a scheduler-bound 100 Hz path. */
	wake_up_all(&sync->waitq);
}

u32 s31_linux_critical_enter(void)
{
	struct s31_linux_task *task = s31_linux_current_task();
	unsigned long flags;

	/* FreeRTOS portENTER_CRITICAL masks local interrupts before taking the
	 * controller lock.  The BT sleep sequencer starts its wake transaction
	 * inside this window and can miss the hardware acknowledgement when a
	 * Linux timer interrupt splits that short register sequence.  Preserve
	 * the native ordering here; blocking compatibility calls suspend this
	 * critical section below, so local IRQs are never held off while sleeping. */
	local_irq_save(flags);
	if (task) {
		task->critical_active = true;
		task->critical_flags = flags;
	}
	raw_spin_lock(&s31_critical_lock);
	return (u32)flags;
}

void s31_linux_critical_exit(u32 flags)
{
	struct s31_linux_task *task = s31_linux_current_task();
	unsigned long irq_flags = flags;

	raw_spin_unlock(&s31_critical_lock);
	/* Keep the exclusion published until the lock is actually free. */
	if (task) {
		task->critical_active = false;
		irq_flags = task->critical_flags;
		task->critical_flags = 0;
	}
	local_irq_restore(irq_flags);
	s31_radio_blob_run_pending_isrs();
}

void s31_linux_critical_suspend(void)
{
	struct s31_linux_task *task = s31_linux_current_task();

	if (!task || !task->critical_active || task->critical_suspended)
		return;
	raw_spin_unlock(&s31_critical_lock);
	/* The raw lock is free before this state permits direct nesting. */
	task->critical_suspended = true;
	local_irq_restore(task->critical_flags);
	s31_radio_blob_run_pending_isrs();
}

void s31_linux_critical_resume(void)
{
	struct s31_linux_task *task = s31_linux_current_task();
	unsigned long flags;

	if (!task || !task->critical_active || !task->critical_suspended)
		return;
	local_irq_save(flags);
	task->critical_flags = flags;
	raw_spin_lock(&s31_critical_lock);
	task->critical_suspended = false;
}

void s31_linux_blob_enter(void)
{
	struct s31_blob_context *context;
	bool timing = READ_ONCE(s31_gate_timing_enabled);
	u64 wait_start_ns = timing ? ktime_get_mono_fast_ns() : 0;
	u64 acquired_ns;

	/* A woken Wi-Fi task normally has a higher SCHED_FIFO priority than the
	 * radio worker.  It must sleep while the worker owns this gate; yielding
	 * in TASK_RUNNING state selects the same high-priority waiter forever and
	 * prevents the owner from releasing the mutex.  The worker may still need
	 * one CLIC sync pass before it blocks behind a payload task. */
	if (!mutex_trylock(&s31_blob_mutex)) {
		if (s31_blob_gate_wait_hook && !s31_gate_hook_busy) {
			s31_gate_hook_busy = true;
			s31_blob_gate_wait_hook();
			s31_gate_hook_busy = false;
		}
		mutex_lock(&s31_blob_mutex);
	}
	acquired_ns = timing ? ktime_get_mono_fast_ns() : 0;
	context = s31_blob_context_current();
	s31_gate_timing_acquired(context, wait_start_ns, acquired_ns);
	s31_blob_install_context(context);
	s31_blob_fpu_begin();
	s31_blob_set_active(true);
	s31_radio_timing_blob_enter();
}

void s31_linux_blob_leave(void)
{
	struct s31_blob_context *context = s31_blob_context_current();

	/* A hardirq which arrived at a conservatively rejected task-side point
	 * must not be left for a worker which is blocked behind this gate owner.
	 * Service it on the already installed payload stack before releasing the
	 * ownership domain. */
	s31_radio_blob_run_pending_isrs();
	s31_gate_timing_released(context, S31_BLOB_RELEASE_LEAVE, NULL);
	s31_blob_fpu_end();
	s31_blob_restore_context(context);
	s31_blob_set_active(false);
	mutex_unlock(&s31_blob_mutex);
}

void s31_linux_blob_suspend(u32 reason)
{
	struct s31_blob_context *context = s31_blob_context_current();
	struct s31_linux_task *task = s31_linux_current_task();
	struct s31_gate_release_info release;

	s31_payload_stack_measure(task);

	/* Drop the critical lock while the blob gate still prevents another
	 * payload task from entering it. Releasing the gate first lets a higher
	 * priority FIFO waiter preempt and spin forever on this raw lock. */
	s31_linux_critical_suspend();
	s31_radio_blob_run_pending_isrs();
	if (s31_blob_active()) {
		s31_gate_timing_released(context, reason, &release);
		if (s31_radio_payload_uses_fp()) {
			if (task)
				s31_payload_fp_save_area(task->fp_saved,
							 &task->fp_valid);
			else
				s31_payload_fp_save_area(context->fp_saved,
							 &context->fp_valid);
		}
		s31_blob_fpu_end();
		s31_blob_restore_context(context);
		s31_blob_set_active(false);
		mutex_unlock(&s31_blob_mutex);
		/* Long-gate PC sampling is kept for future diagnostics, but the
		 * automatic serial print is disabled: the holds were proven to be
		 * ROM UART polling in esp_rom_printf(), not a scheduler bug. */
	}
}

void s31_linux_blob_resume(void)
{
	struct s31_blob_context *context;
	struct s31_linux_task *task = s31_linux_current_task();
	bool timing = READ_ONCE(s31_gate_timing_enabled);
	u64 wait_start_ns;
	u64 acquired_ns;

	if (!s31_blob_active()) {
		wait_start_ns = timing ? ktime_get_mono_fast_ns() : 0;
		mutex_lock(&s31_blob_mutex);
		acquired_ns = timing ? ktime_get_mono_fast_ns() : 0;
		context = s31_blob_context_current();
		s31_gate_timing_acquired(context, wait_start_ns, acquired_ns);
		s31_blob_install_context(context);
		s31_blob_fpu_begin();
		if (s31_radio_payload_uses_fp()) {
			if (task)
				s31_payload_fp_restore_area(task->fp_saved,
							    task->fp_valid);
			else
				s31_payload_fp_restore_area(context->fp_saved,
							    context->fp_valid);
		}
		s31_blob_set_active(true);
		s31_radio_timing_blob_enter();
	}
	s31_linux_critical_resume();
}
