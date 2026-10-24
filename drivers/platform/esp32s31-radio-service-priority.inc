/* SPDX-License-Identifier: GPL-2.0-only */
/* Cold single-executor ownership only. The control worker retains its
 * fixed admitted RR priority; no runtime urgency policy or vendor gate. */
static DEFINE_RAW_SPINLOCK(s31_service_priority_lock);
static struct task_struct *s31_service_priority_thread;

static int s31_service_priority_setup(struct task_struct *thread)
{
	unsigned long flags;
	int ret = 0;

	raw_spin_lock_irqsave(&s31_service_priority_lock, flags);
	if (s31_service_priority_thread)
		ret = -EBUSY;
	else {
		s31_service_priority_thread = thread;
	}
	raw_spin_unlock_irqrestore(&s31_service_priority_lock, flags);
	return ret;
}

static void s31_service_priority_retire_current(void)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&s31_service_priority_lock, flags);
	if (s31_service_priority_thread == current) {
		s31_service_priority_thread = NULL;
	}
	raw_spin_unlock_irqrestore(&s31_service_priority_lock, flags);
}
