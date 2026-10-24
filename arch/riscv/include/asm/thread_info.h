/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2009 Chen Liqin <liqin.chen@sunplusct.com>
 * Copyright (C) 2012 Regents of the University of California
 * Copyright (C) 2017 SiFive
 */

#ifndef _ASM_RISCV_THREAD_INFO_H
#define _ASM_RISCV_THREAD_INFO_H

#include <asm/page.h>
#include <linux/const.h>
#include <linux/sizes.h>

/* thread information allocation */
#ifdef CONFIG_KASAN
#define KASAN_STACK_ORDER	1
#else
#define KASAN_STACK_ORDER	0
#endif
#define THREAD_SIZE_ORDER	(CONFIG_THREAD_SIZE_ORDER + KASAN_STACK_ORDER)
#define THREAD_SIZE		(PAGE_SIZE << THREAD_SIZE_ORDER)

/*
 * By aligning VMAP'd stacks to 2 * THREAD_SIZE, we can detect overflow by
 * checking sp & (1 << THREAD_SHIFT), which we can do cheaply in the entry
 * assembly.
 */
#ifdef CONFIG_VMAP_STACK
#define THREAD_ALIGN            (2 * THREAD_SIZE)
#else
#define THREAD_ALIGN            THREAD_SIZE
#endif

#define THREAD_SHIFT            (PAGE_SHIFT + THREAD_SIZE_ORDER)
#define OVERFLOW_STACK_SIZE     SZ_4K

#define IRQ_STACK_SIZE		THREAD_SIZE

#ifndef __ASSEMBLER__

#include <asm/processor.h>
#include <asm/csr.h>

/*
 * low level task data that entry.S needs immediate access to
 * - this struct should fit entirely inside of one cache line
 * - if the members of this struct changes, the assembly constants
 *   in asm-offsets.c must be updated accordingly
 * - thread_info is included in task_struct at an offset of 0.  This means that
 *   tp points to both thread_info and task_struct.
 */
struct s31_fp_state;
struct s31_radio_task_context;
struct s31_radio_mm_view;
struct esp32s31_radio_stack;

struct thread_info {
	unsigned long		flags;		/* low level flags */
	int                     preempt_count;  /* 0=>preemptible, <0=>BUG */
	/*
	 * These stack pointers are overwritten on every system call or
	 * exception.  SP is also saved to the stack it can be recovered when
	 * overwritten.
	 */
	long			kernel_sp;	/* Kernel stack pointer */
	long			user_sp;	/* User stack pointer */
	int			cpu;
	unsigned long		syscall_work;	/* SYSCALL_WORK_ flags */
#if defined(CONFIG_ESP32S31_COPROC_CONTEXT) && defined(CONFIG_FPU)
	/* Task-owned checkpoint in a globally mapped kernel allocation. */
	struct s31_fp_state	*s31_blob_fp;
	/* Canonical radio identity/lifetime survives FP bank detachment. */
	struct s31_radio_task_context *s31_radio_context;
	/* Explicit dormant-kthread admission; zero preserves generic RR. */
	unsigned int s31_radio_rr_slice;
#endif
#ifdef CONFIG_ESP32S31_RADIO_SMODE
	/* Task-owned, globally mapped stack anchor for BT SRAM trap entry.
	 * Zero outside an anchored Linux -> SRAM trampoline. */
	unsigned long		s31_blob_linux_sp;
	unsigned long		s31_blob_trap_t0;
	/* Original interrupted TP; sscratch always retains canonical Linux TP. */
	unsigned long s31_blob_trap_tp;
	/* Globally mapped, owner-local supervisor view; never inherited. */
	struct s31_radio_mm_view *s31_radio_mm_view;
	/* Borrowed SRAM registered as the actual Linux task stack. */
	struct esp32s31_radio_stack *s31_radio_stack;
	/* Lowest live thread-stack frame while executing on the IRQ stack. */
	unsigned long s31_irq_stack_sp;
#endif
#ifdef CONFIG_SHADOW_CALL_STACK
	void			*scs_base;
	void			*scs_sp;
#endif
#ifdef CONFIG_64BIT
	/*
	 * Used in handle_exception() to save a0, a1 and a2 before knowing if we
	 * can access the kernel stack.
	 */
	unsigned long		a0, a1, a2;
#endif
#ifdef CONFIG_ESP32S31_RADIO_SCHEDULER
	/* Kernel-owned scheduling roles follow every existing trap/stack field. */
	unsigned int s31_radio_sched;
#endif
};

#ifdef CONFIG_SHADOW_CALL_STACK
#define INIT_SCS							\
	.scs_base	= init_shadow_call_stack,			\
	.scs_sp		= init_shadow_call_stack,
#else
#define INIT_SCS
#endif

/*
 * macros/functions for gaining access to the thread information structure
 *
 * preempt_count needs to be 1 initially, until the scheduler is functional.
 */
#define INIT_THREAD_INFO(tsk)			\
{						\
	.flags		= 0,			\
	.preempt_count	= INIT_PREEMPT_COUNT,	\
	INIT_SCS				\
}

void arch_release_task_struct(struct task_struct *tsk);
int arch_dup_task_struct(struct task_struct *dst, struct task_struct *src);

#endif /* !__ASSEMBLER__ */

/*
 * thread information flags
 * - these are process state flags that various assembly files may need to
 *   access
 * - pending work-to-be-done flags are in lowest half-word
 * - other flags in upper half-word(s)
 */

/*
 * Tell the generic TIF infrastructure which bits riscv supports
 */
#define HAVE_TIF_NEED_RESCHED_LAZY
#define HAVE_TIF_RESTORE_SIGMASK

#include <asm-generic/thread_info_tif.h>

#define TIF_32BIT			16	/* compat-mode 32bit process */
#define TIF_RISCV_V_DEFER_RESTORE	17	/* restore Vector before returing to user */

#define _TIF_RISCV_V_DEFER_RESTORE	BIT(TIF_RISCV_V_DEFER_RESTORE)

#endif /* _ASM_RISCV_THREAD_INFO_H */
