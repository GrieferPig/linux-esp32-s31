/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_TASK_WORK_H
#define _LINUX_TASK_WORK_H

#include <linux/completion.h>
#include <linux/list.h>
#include <linux/sched.h>

typedef void (*task_work_func_t)(struct callback_head *);

static inline void
init_task_work(struct callback_head *twork, task_work_func_t func)
{
	twork->func = func;
}

enum task_work_notify_mode {
	TWA_NONE = 0,
	TWA_RESUME,
	TWA_SIGNAL,
	TWA_SIGNAL_NO_IPI,
	TWA_NMI_CURRENT,
};

static inline bool task_work_pending(struct task_struct *task)
{
	return READ_ONCE(task->task_works);
}

int task_work_add(struct task_struct *task, struct callback_head *twork,
			enum task_work_notify_mode mode);

struct callback_head *task_work_cancel_match(struct task_struct *task,
	bool (*match)(struct callback_head *, void *data), void *data);
struct callback_head *task_work_cancel_func(struct task_struct *, task_work_func_t);
bool task_work_cancel(struct task_struct *task, struct callback_head *cb);
void task_work_run(void);

/* Owner-thread callbacks retained until exit or explicitly requested at a
 * task_work safe point. The object must live until done; completion happens
 * in generic kernel code AFTER func returns, allowing module teardown to
 * reclaim both object and callback code safely. No implicit task reference.
 * External admission closes on exit. During owner delivery only that same
 * current task may add finite child guards, drained before PF_EXITING/MM
 * teardown. A live user owner is notified via TIF_NOTIFY_SIGNAL after pi_lock
 * is released; kthreads retain their explicit task_work safe points. */
struct task_exit_work {
 struct callback_head cb;
 void (*func)(struct task_exit_work *);
 struct completion done;
};
int task_exit_work_add(struct task_struct *task, struct task_exit_work *work,
                      void (*func)(struct task_exit_work *));
int task_exit_work_request(struct task_struct *task, struct task_exit_work *work);
/* Finish only this owner's work synchronously. The owner must quiesce its
 * invocation before calling; generic completion follows callback return. */
int task_exit_work_finish_current(struct task_exit_work *work);
void task_exit_work_run(void);

static inline void exit_task_work(struct task_struct *task)
{
	task_work_run();
}

#endif	/* _LINUX_TASK_WORK_H */
