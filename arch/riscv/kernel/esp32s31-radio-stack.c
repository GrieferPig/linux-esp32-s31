// SPDX-License-Identifier: GPL-2.0-only
/* Borrowed internal-SRAM kernel stacks with kernel-owned retirement. */
#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/interrupt.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/sched/s31_radio_stack.h>
#include <linux/slab.h>
#include <asm/thread_info.h>

enum s31_stack_state {
	S31_STACK_PREPARED,
	S31_STACK_INSTALLED,
	S31_STACK_RETIRED,
};

struct esp32s31_radio_stack {
	void *base;
	atomic_t state;
	struct rcu_head rcu;
	struct completion released;
};

static atomic_long_t s31_stack_reserved = ATOMIC_LONG_INIT(0);

struct esp32s31_radio_stack *esp32s31_radio_stack_prepare(void *base, size_t size)
{
	struct esp32s31_radio_stack *stack;
	unsigned long address = (unsigned long)base;

	if (in_interrupt() || in_atomic() || irqs_disabled())
		return ERR_PTR(-EWOULDBLOCK);
	/* Generic stack walkers and pt_regs placement use THREAD_SIZE. Keep
	 * the normal size/alignment contract instead of teaching each helper
	 * about an unregistered second stack or an arbitrary private size. */
	if (IS_ENABLED(CONFIG_VMAP_STACK) || THREAD_SIZE < PAGE_SIZE)
		return ERR_PTR(-EOPNOTSUPP);
	if (size != THREAD_SIZE || !IS_ALIGNED(address, THREAD_ALIGN) ||
	    address < 0x2f000000UL || address > 0x2f080000UL - THREAD_SIZE)
		return ERR_PTR(-EINVAL);
	stack = kzalloc(sizeof(*stack), GFP_KERNEL);
	if (!stack)
		return ERR_PTR(-ENOMEM);
	stack->base = base;
	atomic_set(&stack->state, S31_STACK_PREPARED);
	init_completion(&stack->released);
	return stack;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_stack_prepare);

bool esp32s31_radio_stack_owned(const struct task_struct *task)
{
	return READ_ONCE(task_thread_info(task)->s31_radio_stack) != NULL;
}

int esp32s31_radio_stack_install(struct task_struct *task)
{
	struct esp32s31_radio_stack *stack = task_thread_info(task)->s31_radio_stack;

	if (!stack || atomic_cmpxchg(&stack->state, S31_STACK_PREPARED,
				    S31_STACK_INSTALLED) != S31_STACK_PREPARED)
		return -EINVAL;
	memset(stack->base, 0, THREAD_SIZE);
	task->stack = stack->base;
	atomic_long_add(THREAD_SIZE, &s31_stack_reserved);
	return 0;
}

static void s31_radio_stack_released(struct rcu_head *rcu)
{
	struct esp32s31_radio_stack *stack = container_of(rcu,
					struct esp32s31_radio_stack, rcu);

	atomic_long_sub(THREAD_SIZE, &s31_stack_reserved);
	/* No backing-buffer access follows publication. The caller can free
	 * or return SRAM to its pool after destroy has joined this completion. */
	complete(&stack->released);
}

bool esp32s31_radio_stack_release(struct task_struct *task)
{
	struct esp32s31_radio_stack *stack = task_thread_info(task)->s31_radio_stack;

	if (!stack)
		return false;
	if (task == current || task->stack != stack->base ||
	    atomic_cmpxchg(&stack->state, S31_STACK_INSTALLED,
			   S31_STACK_RETIRED) != S31_STACK_INSTALLED)
		panic("s31 radio: invalid external kernel stack retirement");
	task_thread_info(task)->s31_radio_stack = NULL;
	task->stack = NULL;
	call_rcu(&stack->rcu, s31_radio_stack_released);
	return true;
}

int esp32s31_radio_stack_destroy(struct esp32s31_radio_stack *stack)
{
	if (!stack || IS_ERR(stack))
		return -EINVAL;
	if (in_interrupt() || in_atomic() || irqs_disabled() ||
	    current_thread_info()->s31_radio_stack == stack)
		return -EWOULDBLOCK;
	/* Creation is synchronous and destroy belongs to the exclusive lease
	 * owner. Once installation was attempted, join the kernel's last stack
	 * reference even if creation failed after allocating the task stack. */
	if (atomic_read(&stack->state) != S31_STACK_PREPARED)
		wait_for_completion(&stack->released);
	kfree(stack);
	return 0;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_stack_destroy);

bool esp32s31_radio_stack_released(struct esp32s31_radio_stack *stack)
{
	return completion_done(&stack->released);
}
EXPORT_SYMBOL_GPL(esp32s31_radio_stack_released);

unsigned long esp32s31_radio_stack_bytes(void)
{
	return atomic_long_read(&s31_stack_reserved);
}
EXPORT_SYMBOL_GPL(esp32s31_radio_stack_bytes);
