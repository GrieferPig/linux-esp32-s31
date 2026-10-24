// SPDX-License-Identifier: GPL-2.0-only
/* Native-process entry uses the permanent supervisor identity mappings.
 * This monitor owns only callback lifetime, never VMAs or an mmap lock. */
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/task_work.h>
#include <linux/sched/task_stack.h>
#include <linux/interrupt.h>
#include <linux/export.h>
#include <linux/panic.h>
#include <asm/pgtable.h>
#include <asm/esp32s31_blob_fp.h>
#include <asm/esp32s31_radio_view.h>
#include <asm/esp32s31_radio_map.h>

struct s31_radio_mm_view {
	struct task_exit_work exit;
	struct task_struct *thread;
	bool retired;
};

bool esp32s31_radio_mm_active(struct task_struct *task)
{
	return READ_ONCE(task_thread_info(task)->s31_radio_mm_view) != NULL;
}

void esp32s31_radio_mm_user_return_check(void)
{
	if (esp32s31_radio_mm_active(current))
		panic("s31 radio view: user return/exec before callback retirement");
}

static void s31_view_exit(struct task_exit_work *work)
{
	struct s31_radio_mm_view *view = container_of(work, struct s31_radio_mm_view, exit);

	if (view->thread != current || !READ_ONCE(view->retired))
		panic("s31 radio view: owner exited before callback retirement");
}

int esp32s31_radio_mm_call(int (*body)(void *), void *opaque)
{
	struct s31_radio_mm_view *view;
	struct mm_struct *mm = current->mm;
	unsigned long sp;
	unsigned int i;
	int ret;

	if (!body)
		return -EINVAL;
	if ((current->flags & PF_KTHREAD) || !mm || in_interrupt() ||
	    in_atomic() || irqs_disabled())
		return -EWOULDBLOCK;
	if (raw_smp_processor_id() != 0 ||
	    !cpumask_equal(&current->cpus_mask, cpumask_of(0)))
		return -EXDEV;
	if (!esp32s31_blob_fp_user_active(current) || current->active_mm != mm)
		return -EINVAL;
	if (esp32s31_radio_mm_active(current))
		return -EBUSY;
	asm volatile("mv %0, sp" : "=r"(sp));
	if (sp < (unsigned long)current->stack ||
	    sp >= (unsigned long)current->stack + THREAD_SIZE)
		return -EINVAL;
	/* Immutable leaves are copied at pgd allocation and excluded from all
	 * user mapping/growth and page-table retirement. No per-call remap. */
	for (i = 0; i < S31_RADIO_WINDOW_COUNT; i++) {
		pgd_t leaf = READ_ONCE(*pgd_offset(mm, s31_radio_window_base[i]));

		if (pgd_val(leaf) != pgd_val(READ_ONCE(*pgd_offset_k(s31_radio_window_base[i]))) ||
		    !(pgd_val(leaf) & (_PAGE_READ | _PAGE_WRITE | _PAGE_EXEC)) ||
		    (pgd_val(leaf) & _PAGE_USER))
			return -EINVAL;
	}
	view = kzalloc(sizeof(*view), GFP_KERNEL);
	if (!view)
		return -ENOMEM;
	view->thread = current;
	ret = task_exit_work_add(current, &view->exit, s31_view_exit);
	if (ret) {
		kfree(view);
		return ret;
	}
	WRITE_ONCE(current_thread_info()->s31_radio_mm_view, view);
	ret = body(opaque);
	if (current != view->thread || current->mm != mm || current->active_mm != mm ||
	    !esp32s31_blob_fp_user_active(current) || raw_smp_processor_id() != 0 ||
	    in_interrupt() || in_atomic() || irqs_disabled())
		panic("s31 radio view: callback changed native ownership");
	WRITE_ONCE(current_thread_info()->s31_radio_mm_view, NULL);
	WRITE_ONCE(view->retired, true);
	if (task_exit_work_finish_current(&view->exit))
		panic("s31 radio view: retired monitor did not finish");
	kfree(view);
	return ret;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_mm_call);
