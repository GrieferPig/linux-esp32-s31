/* SPDX-License-Identifier: GPL-2.0-only */
#include "s31-blob-fp-protocol.h"
#include <linux/errno.h>

int s31_fp_pre_switch(struct s31_fp_cpu *cpu, struct s31_fp_state *prev,
                      const struct s31_fp_ops *ops)
{
    if (cpu->foreign_active || cpu->kernel_borrow) return -EBUSY; /* kernel_fpu is preempt-fenced. */
    if (!cpu->owner) return 0;
    if (cpu->owner != prev || !prev || !prev->active) return -EINVAL;
    ops->save(ops->hw, &prev->bank); /* MUST precede generic next FP restore. */
    prev->valid = 1;
    cpu->owner = 0;
    ops->fs(ops->hw, 0);
    return 0;
}
int s31_fp_post_switch(struct s31_fp_cpu *cpu, struct s31_fp_state *next,
                       const struct s31_fp_ops *ops)
{
    if (cpu->foreign_active || cpu->kernel_borrow || cpu->owner) return -EBUSY;
    if (!next || !next->active) return 0; /* Keep generic user bank untouched. */
    if (!next->valid) return -EINVAL;
    ops->restore(ops->hw, &next->bank); /* AFTER generic/ESP/vector clobbers. */
    cpu->owner = next;
    /* Native bridge code keeps the physical bank but must not use FP.
     * A payload continuation has no sret to enable FS for it here. */
    ops->fs(ops->hw, !next->native || next->payload);
    return 0;
}
int s31_fp_enter(struct s31_fp_cpu *cpu, struct s31_fp_state *state,
                 const struct s31_fp_ops *ops)
{
    if (!state || !state->valid) return -EINVAL;
    if (state->native && state->active) {
        if (cpu->foreign_active || cpu->kernel_borrow ||
            cpu->owner != state || state->payload) return -EBUSY;
        state->payload = 1;
        ops->fs(ops->hw, 1);
        return 0; /* The same task still owns the physical bank. */
    }
    if (cpu->foreign_active || cpu->kernel_borrow || cpu->owner || state->active) return -EBUSY;
    ops->restore(ops->hw, &state->bank);
    state->active = 1;
    if (state->native) state->payload = 1;
    cpu->owner = state;
    ops->fs(ops->hw, 1);
    return 0;
}
int s31_fp_pause(struct s31_fp_cpu *cpu, struct s31_fp_state *state,
                 const struct s31_fp_ops *ops)
{
    if (!state || !state->active) return -EINVAL;
    if (cpu->foreign_active || cpu->kernel_borrow || cpu->owner != state) return -EBUSY;
    if (state->native) {
        if (!state->payload) return -EINVAL;
        state->payload = 0;
        ops->fs(ops->hw, 0);
        return 0; /* Only an actual task switch/borrow needs a checkpoint. */
    }
    ops->save(ops->hw, &state->bank);
    state->valid = 1;
    state->active = 0; /* Scheduler must not restore a paused blob into bridge. */
    cpu->owner = 0;
    ops->fs(ops->hw, 0);
    return 0;
}
/* Final owner-side retirement, after the last payload/finalizer and all
 * bridge cleanup. Never checkpoint a bank that is about to be freed. */
int s31_fp_finish(struct s31_fp_cpu *cpu, struct s31_fp_state *state,
                  const struct s31_fp_ops *ops)
{
    if (!state || !state->native || state->process || state->payload)
        return -EINVAL;
    if (cpu->foreign_active || cpu->kernel_borrow || cpu->borrower ||
        (state->active ? cpu->owner != state : cpu->owner != 0))
        return -EBUSY;
    cpu->owner = 0;
    state->active = 0;
    ops->fs(ops->hw, 0);
    return 0;
}
int s31_fp_kernel_begin(struct s31_fp_cpu *cpu, const struct s31_fp_ops *ops)
{
    if (cpu->foreign_active || cpu->kernel_borrow) return -EBUSY; /* Generic kernel_fpu is not nested. */
    if (cpu->owner && (!cpu->owner->active || !cpu->owner->valid))
        return -EINVAL;
    cpu->kernel_borrow = 1;
    cpu->borrower = cpu->owner;
    if (cpu->owner) {
        ops->save(ops->hw, &cpu->borrowed_bank);
        cpu->owner = 0;
        ops->fs(ops->hw, 0);
    }
    return 0;
}
int s31_fp_kernel_end_check(struct s31_fp_cpu *cpu)
{
    if (cpu->foreign_active || !cpu->kernel_borrow || cpu->owner ||
        (cpu->borrower && (!cpu->borrower->active || !cpu->borrower->valid)))
        return -EINVAL;
    return 0;
}
int s31_fp_kernel_end(struct s31_fp_cpu *cpu, const struct s31_fp_ops *ops)
{
    int error = s31_fp_kernel_end_check(cpu);
    if (error) return error;
    if (cpu->borrower) {
        ops->restore(ops->hw, &cpu->borrowed_bank);
        cpu->owner = cpu->borrower;
        ops->fs(ops->hw, !cpu->borrower->native || cpu->borrower->payload);
    }
    cpu->borrower = 0;
    cpu->kernel_borrow = 0;
    return 0;
}

int s31_fp_foreign_enter(struct s31_fp_cpu *cpu)
{
    if (cpu->foreign_active || cpu->owner || cpu->kernel_borrow)
        return -EBUSY;
    cpu->foreign_active = 1;
    return 0;
}
int s31_fp_foreign_leave(struct s31_fp_cpu *cpu)
{
    if (!cpu->foreign_active || cpu->owner || cpu->kernel_borrow)
        return -EINVAL;
    cpu->foreign_active = 0;
    return 0;
}
