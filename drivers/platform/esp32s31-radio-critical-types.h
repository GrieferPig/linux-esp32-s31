/* SPDX-License-Identifier: GPL-2.0 */
/* Private native state, never an IDF-visible object layout. */
struct s31_critical_state {
	unsigned long flags;
	struct task_struct *thread;
	u32 depth, legacy_depth, mux_depth;
	bool suspended;
};
