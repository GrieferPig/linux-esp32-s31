// SPDX-License-Identifier: GPL-2.0
/* Linux half of the Wi-Fi blob's FreeRTOS compatibility bridge. */

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/fpu.h>
#include <linux/interrupt.h>
#include <linux/mm_types.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/thread_info.h>
#include <linux/wait.h>

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
	u32 gate_timing_generation;
};

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
	unsigned long critical_flags;
	struct s31_blob_context blob;
	/* Payload execution stack in internal SRAM.  The PHY changes the
	 * external-memory/cache state while the payload runs, so the DRAM
	 * kthread stack is unusable for payload code. */
	void *payload_stack;
	u32 payload_stack_size;
	unsigned long sp_save[14];
	/* ILP32F callee-saved payload context.  kernel_fpu_end() restores the
	 * kthread's pre-blob kernel state, so these registers must be preserved
	 * explicitly across every FreeRTOS-style blocking point. */
	u32 fp_saved[13]; /* fs0..fs11, fcsr */
	bool fp_valid;
	u32 payload_stack_peak;
	struct s31_linux_task *next;
};

struct s31_linux_sync {
	raw_spinlock_t lock;
	atomic_t sequence;
};

static DEFINE_PER_CPU(struct s31_blob_context, s31_foreign_blob_context);
static DECLARE_WAIT_QUEUE_HEAD(s31_sync_waitq);
static atomic_t s31_sync_stop_generation = ATOMIC_INIT(0);
static DEFINE_RAW_SPINLOCK(s31_critical_lock);
static DEFINE_MUTEX(s31_blob_mutex);
static struct s31_linux_task *s31_task_list;
static DEFINE_SPINLOCK(s31_task_list_lock);

#define S31_GATE_TIMING_SLOTS 16

struct s31_gate_timing_slot {
	struct task_struct *thread;
	char name[TASK_COMM_LEN];
	u64 wait_total_ns;
	u64 wait_max_ns;
	u64 hold_total_ns;
	u64 hold_max_ns;
	u32 enters;
	u32 holds;
};

static DEFINE_SPINLOCK(s31_gate_timing_lock);
static struct s31_gate_timing_slot s31_gate_timing[S31_GATE_TIMING_SLOTS];
static u32 s31_gate_timing_generation;
static bool s31_gate_timing_enabled;

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

	spin_lock_irqsave(&s31_gate_timing_lock, flags);
	if (!s31_gate_timing_enabled) {
		context->gate_timing_generation = 0;
		context->gate_acquired_ns = 0;
		spin_unlock_irqrestore(&s31_gate_timing_lock, flags);
		return;
	}
	slot = s31_gate_timing_slot_locked();
	slot->wait_total_ns += wait_ns;
	slot->wait_max_ns = max(slot->wait_max_ns, wait_ns);
	slot->enters++;
	context->gate_timing_generation = s31_gate_timing_generation;
	context->gate_acquired_ns = acquired_ns;
	spin_unlock_irqrestore(&s31_gate_timing_lock, flags);
}

static void s31_gate_timing_released(struct s31_blob_context *context)
{
	struct s31_gate_timing_slot *slot;
	unsigned long flags;
	u64 now = ktime_get_mono_fast_ns();

	spin_lock_irqsave(&s31_gate_timing_lock, flags);
	if (!s31_gate_timing_enabled || !context->gate_acquired_ns ||
	    context->gate_timing_generation != s31_gate_timing_generation)
		goto out;
	slot = s31_gate_timing_slot_locked();
	slot->hold_total_ns += now - context->gate_acquired_ns;
	slot->hold_max_ns = max(slot->hold_max_ns,
				 now - context->gate_acquired_ns);
	slot->holds++;
out:
	context->gate_acquired_ns = 0;
	context->gate_timing_generation = 0;
	spin_unlock_irqrestore(&s31_gate_timing_lock, flags);
}

void s31_linux_gate_timing_reset(void)
{
	unsigned long flags;

	spin_lock_irqsave(&s31_gate_timing_lock, flags);
	memset(s31_gate_timing, 0, sizeof(s31_gate_timing));
	s31_gate_timing_generation++;
	if (!s31_gate_timing_generation)
		s31_gate_timing_generation++;
	s31_gate_timing_enabled = true;
	spin_unlock_irqrestore(&s31_gate_timing_lock, flags);
}

void s31_linux_gate_timing_report(const char *stage)
{
	unsigned long flags;
	int i;

	spin_lock_irqsave(&s31_gate_timing_lock, flags);
	s31_gate_timing_enabled = false;
	spin_unlock_irqrestore(&s31_gate_timing_lock, flags);

	for (i = 0; i < S31_GATE_TIMING_SLOTS; i++) {
		struct s31_gate_timing_slot slot;

		spin_lock_irqsave(&s31_gate_timing_lock, flags);
		slot = s31_gate_timing[i];
		spin_unlock_irqrestore(&s31_gate_timing_lock, flags);

		if (!slot.thread)
			continue;
		pr_info("esp32s31-radio: gate timing %s task=%s enter=%u wait_avg=%lluns wait_max=%lluns hold=%u hold_avg=%lluns hold_max=%lluns\n",
			stage, slot.name, slot.enters,
			slot.enters ? div_u64(slot.wait_total_ns,
						 slot.enters) : 0,
			slot.wait_max_ns, slot.holds,
			slot.holds ? div_u64(slot.hold_total_ns,
						 slot.holds) : 0,
			slot.hold_max_ns);
	}
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
		kernel_fpu_end();
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

	kthread_use_mm(&init_mm);
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
	kthread_unuse_mm(&init_mm);
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
				    void *arg, u32 priority, void *cookie)
{
	struct s31_linux_task *task;
	struct sched_param param = { };

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
	/* Preserve the FreeRTOS ordering: the init task is above operation tasks,
	 * and all measured Wi-Fi tasks are real-time Linux threads. */
	param.sched_priority = clamp_t(u32, 40 + priority, 1, 99);
	sched_setscheduler_nocheck(task->thread, SCHED_FIFO, &param);
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
	wake_up_all(&s31_sync_waitq);
	wake_up_process(task->thread);
	kthread_stop(task->thread);
	return 0;
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
	s31_linux_blob_suspend();
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
	/* FreeRTOS vTaskDelay(0) is an explicit scheduler yield.  cond_resched()
	 * is insufficient for a runnable SCHED_FIFO task and cannot let another
	 * payload enter while this task owns the serialized blob gate. */
	s31_linux_blob_suspend();
	yield();
	s31_linux_blob_resume();
}

void s31_linux_task_set_priority(void *opaque, u32 priority)
{
	struct s31_linux_task *task = opaque;
	struct sched_param param = { };

	if (!task || task->magic != S31_LINUX_TASK_MAGIC)
		return;
	param.sched_priority = clamp_t(u32, 40 + priority, 1, 99);
	sched_setscheduler_nocheck(task->thread, SCHED_FIFO, &param);
}

void *s31_linux_sync_create(void)
{
	struct s31_linux_sync *sync = s31_radio_sram_alloc(sizeof(*sync));

	if (sync)
		raw_spin_lock_init(&sync->lock);
	if (sync)
		atomic_set(&sync->sequence, 0);
	return sync;
}

void s31_linux_sync_destroy(void *opaque)
{
	s31_radio_sram_free(opaque);
}

void s31_linux_sync_lock(void *opaque)
{
	struct s31_linux_sync *sync = opaque;

	if (sync)
		raw_spin_lock(&sync->lock);
}

void s31_linux_sync_unlock(void *opaque)
{
	struct s31_linux_sync *sync = opaque;

	if (sync)
		raw_spin_unlock(&sync->lock);
}

u32 s31_linux_sync_sequence(void *opaque)
{
	struct s31_linux_sync *sync = opaque;

	return sync ? atomic_read(&sync->sequence) : 0;
}

int32_t s31_linux_sync_wait(void *opaque, u32 sequence, u32 ticks)
{
	struct s31_linux_sync *sync = opaque;
	int stop_generation = atomic_read(&s31_sync_stop_generation);
	long timeout = s31_ticks_to_jiffies(ticks);
	long ret;

	if (!sync || !timeout)
		return 0;
	s31_linux_blob_suspend();
	ret = wait_event_interruptible_timeout(s31_sync_waitq,
		atomic_read(&sync->sequence) != sequence ||
		atomic_read(&s31_sync_stop_generation) != stop_generation ||
		kthread_should_stop(), timeout);
	s31_linux_blob_resume();
	if (ret < 0 || kthread_should_stop())
		return -1;
	return ret ? 1 : 0;
}

void s31_linux_sync_wake(void *opaque)
{
	struct s31_linux_sync *sync = opaque;

	if (!sync)
		return;
	atomic_inc(&sync->sequence);
	wake_up_all(&s31_sync_waitq);
}

u32 s31_linux_critical_enter(void)
{
	struct s31_linux_task *task = s31_linux_current_task();
	unsigned long flags;

	raw_spin_lock_irqsave(&s31_critical_lock, flags);
	if (task) {
		task->critical_active = true;
		task->critical_flags = flags;
	}
	return (u32)flags;
}

void s31_linux_critical_exit(u32 flags)
{
	struct s31_linux_task *task = s31_linux_current_task();

	if (task)
		task->critical_active = false;
	raw_spin_unlock_irqrestore(&s31_critical_lock, (unsigned long)flags);
}

void s31_linux_critical_suspend(void)
{
	struct s31_linux_task *task = s31_linux_current_task();

	if (!task || !task->critical_active || task->critical_suspended)
		return;
	task->critical_suspended = true;
	raw_spin_unlock_irqrestore(&s31_critical_lock, task->critical_flags);
}

void s31_linux_critical_resume(void)
{
	struct s31_linux_task *task = s31_linux_current_task();
	unsigned long flags;

	if (!task || !task->critical_active || !task->critical_suspended)
		return;
	raw_spin_lock_irqsave(&s31_critical_lock, flags);
	task->critical_flags = flags;
	task->critical_suspended = false;
}

void s31_linux_blob_enter(void)
{
	struct s31_blob_context *context;
	u64 wait_start_ns = ktime_get_mono_fast_ns();
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
	acquired_ns = ktime_get_mono_fast_ns();
	context = s31_blob_context_current();
	s31_gate_timing_acquired(context, wait_start_ns, acquired_ns);
	s31_blob_install_context(context);
	kernel_fpu_begin();
	s31_blob_set_active(true);
	s31_radio_timing_blob_enter();
}

void s31_linux_blob_leave(void)
{
	struct s31_blob_context *context = s31_blob_context_current();

	s31_gate_timing_released(context);
	kernel_fpu_end();
	s31_blob_restore_context(context);
	s31_blob_set_active(false);
	mutex_unlock(&s31_blob_mutex);
}

void s31_linux_blob_suspend(void)
{
	struct s31_blob_context *context = s31_blob_context_current();
	struct s31_linux_task *task = s31_linux_current_task();

	s31_payload_stack_measure(task);

	/* Drop the critical lock while the blob gate still prevents another
	 * payload task from entering it. Releasing the gate first lets a higher
	 * priority FIFO waiter preempt and spin forever on this raw lock. */
	s31_linux_critical_suspend();
	if (s31_blob_active()) {
		s31_gate_timing_released(context);
		if (task)
			s31_payload_fp_save_area(task->fp_saved, &task->fp_valid);
		else
			s31_payload_fp_save_area(context->fp_saved,
					 &context->fp_valid);
		kernel_fpu_end();
		s31_blob_restore_context(context);
		s31_blob_set_active(false);
		mutex_unlock(&s31_blob_mutex);
	}
}

void s31_linux_blob_resume(void)
{
	struct s31_blob_context *context;
	struct s31_linux_task *task = s31_linux_current_task();
	u64 wait_start_ns;
	u64 acquired_ns;

	if (!s31_blob_active()) {
		wait_start_ns = ktime_get_mono_fast_ns();
		mutex_lock(&s31_blob_mutex);
		acquired_ns = ktime_get_mono_fast_ns();
		context = s31_blob_context_current();
		s31_gate_timing_acquired(context, wait_start_ns, acquired_ns);
		s31_blob_install_context(context);
		kernel_fpu_begin();
		if (task)
			s31_payload_fp_restore_area(task->fp_saved, task->fp_valid);
		else
			s31_payload_fp_restore_area(context->fp_saved,
					    context->fp_valid);
		s31_blob_set_active(true);
		s31_radio_timing_blob_enter();
	}
	s31_linux_critical_resume();
}
