/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef S31_FP_PROTOCOL_H
#define S31_FP_PROTOCOL_H
#include <linux/types.h>
typedef u32 s31_fp_u32;

struct s31_fp_bank { s31_fp_u32 f[32]; s31_fp_u32 fcsr; };
struct s31_fp_state {
    struct s31_fp_bank bank;
    unsigned int valid;
    unsigned int active; /* Logical blob ownership survives involuntary switch. */
    unsigned int process; /* Owning syscall also has a separate user bank. */
    unsigned int radio; /* Explicit radio admission; wrapper owns driver data. */
    unsigned int native; /* Keep hardware ownership across Linux bridge calls. */
    unsigned int payload; /* Native continuation may execute FP instructions. */
};
struct s31_fp_cpu {
    struct s31_fp_state *owner; /* Hardware bank is this task's blob bank. */
    struct s31_fp_state *borrower;
    struct s31_fp_bank borrowed_bank;
    unsigned int kernel_borrow;
    unsigned int foreign_active; /* Legacy window: generic borrowing forbidden. */
};
struct s31_fp_ops {
    void (*save)(void *hw, struct s31_fp_bank *bank);
    void (*restore)(void *hw, const struct s31_fp_bank *bank);
    void (*fs)(void *hw, unsigned int enabled);
    void *hw;
};

/* Caller must exclude migration, preemption and local IRQ nesting during each
 * transition. State, cpu and ops are globally mapped/cache-accessible memory.
 * No function may infer ownership from a low SRAM cookie or task payload SP.
 * Save/restore callbacks touch all f0-f31 + fcsr and temporarily enable FS.
 */
int s31_fp_pre_switch(struct s31_fp_cpu *, struct s31_fp_state *, const struct s31_fp_ops *);
int s31_fp_post_switch(struct s31_fp_cpu *, struct s31_fp_state *, const struct s31_fp_ops *);
int s31_fp_enter(struct s31_fp_cpu *, struct s31_fp_state *, const struct s31_fp_ops *);
int s31_fp_pause(struct s31_fp_cpu *, struct s31_fp_state *, const struct s31_fp_ops *);
int s31_fp_finish(struct s31_fp_cpu *, struct s31_fp_state *, const struct s31_fp_ops *);
int s31_fp_kernel_begin(struct s31_fp_cpu *, const struct s31_fp_ops *);
int s31_fp_kernel_end_check(struct s31_fp_cpu *);
int s31_fp_kernel_end(struct s31_fp_cpu *, const struct s31_fp_ops *);
int s31_fp_foreign_enter(struct s31_fp_cpu *);
int s31_fp_foreign_leave(struct s31_fp_cpu *);
#endif
