/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_SCHED_S31_RADIO_H
#define _LINUX_SCHED_S31_RADIO_H
#include <linux/sched.h>
#include <linux/errno.h>

#define S31_RADIO_SCHED_ADMITTED 1U
#define S31_RADIO_SCHED_INHERITED 2U
#define S31_RADIO_SCHED_ROLE_MASK (S31_RADIO_SCHED_ADMITTED | S31_RADIO_SCHED_INHERITED)

/* One kernel-owned word supplies an atomic effective-role snapshot. Admission
 * is creator-owned and immutable through the task lifetime. PI changes only
 * its inherited bit under pi_lock + rq lock. No module/SRAM object is read. */
static inline bool s31_radio_sched_role(struct task_struct *p)
{
#ifdef CONFIG_ESP32S31_RADIO_SCHEDULER
	return READ_ONCE(task_thread_info(p)->s31_radio_sched) &
	       S31_RADIO_SCHED_ROLE_MASK;
#else
	return false;
#endif
}

static inline bool s31_radio_sched_pi_changed(struct task_struct *p,
					      struct task_struct *donor)
{
#ifdef CONFIG_ESP32S31_RADIO_SCHEDULER
	bool inherited = donor && s31_radio_sched_role(donor);
	unsigned int role = READ_ONCE(task_thread_info(p)->s31_radio_sched);
	return !!(role & S31_RADIO_SCHED_INHERITED) != inherited;
#else
	return false;
#endif
}

static inline bool s31_radio_sched_pi_update(struct task_struct *p,
					     struct task_struct *donor)
{
#ifdef CONFIG_ESP32S31_RADIO_SCHEDULER
	unsigned int role = READ_ONCE(task_thread_info(p)->s31_radio_sched);
	bool before = role & S31_RADIO_SCHED_ROLE_MASK;
	bool inherited = donor && s31_radio_sched_role(donor);
	unsigned int next = (role & ~S31_RADIO_SCHED_INHERITED) |
		(inherited ? S31_RADIO_SCHED_INHERITED : 0);
	WRITE_ONCE(task_thread_info(p)->s31_radio_sched, next);
	return before != s31_radio_sched_role(p);
#else
	return false;
#endif
}
#endif
