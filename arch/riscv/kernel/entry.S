/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2012 Regents of the University of California
 * Copyright (C) 2017 SiFive
 */

#include <linux/export.h>
#include <linux/init.h>
#include <linux/linkage.h>

#include <asm/alternative-macros.h>
#include <asm/asm.h>
#include <asm/csr.h>
#include <asm/scs.h>
#include <asm/unistd.h>
#include <asm/page.h>
#include <asm/thread_info.h>
#include <asm/asm-offsets.h>
#include <asm/errata_list.h>
#include <linux/sizes.h>

	.section .irqentry.text, "ax"

/*
 * ESP32-S31 CLIC quirk: scause has CLIC extension bits in [30:12]
 * (SPP at [28], SPIL at [27:16]). Strip them so the cause matches
 * the standard RISC-V exception codes that excp_vect_table and the
 * C handlers expect. Preserves the Interrupt flag (bit XLEN-1) and
 * the low 12 bits. Gated by RISCV_ESP_CLIC_TRAP_QUIRK (selected by
 * ESP_CLIC); a no-op macro on every other platform.
 */
.macro mask_clic_scause reg, scratch
#ifdef CONFIG_RISCV_ESP_CLIC_TRAP_QUIRK
	li \scratch, CAUSE_IRQ_FLAG | 0xFFF
	and \reg, \reg, \scratch
#endif
.endm

.macro esp32s31_clic_switch_level_reset
#if defined(CONFIG_SOC_ESP32S31) && !defined(CONFIG_RISCV_M_MODE)
	/* Linux can switch from an IRQ/exception continuation before its sret.
	 * SIE alone does not clear CLIC SIL: the new kernel continuation must
	 * not inherit that source's level (or the exception's 0xff sentinel).
	 * Recover on the new task's Linux stack, still IRQ-disabled. Preserve
	 * all trap CSRs and a0/a1; only ABI caller-saved t0..t4 are scratch.
	 * No ecall, pending-source poll, acknowledgement or IRQ enable.
	 */
	csrr t0, CSR_STATUS
	csrci CSR_STATUS, SR_IE
	csrr t1, CSR_EPC
	csrr t2, CSR_CAUSE
	li t3, CAUSE_IRQ_FLAG
	csrw CSR_CAUSE, t3
	ori t3, t0, SR_SPP
	li t4, ~(SR_IE | SR_PIE)
	and t3, t3, t4
	csrw CSR_STATUS, t3
	lla t3, 991f
	csrw CSR_EPC, t3
	sret
991:
	csrw CSR_CAUSE, t2
	csrw CSR_EPC, t1
	csrw CSR_STATUS, t0
#endif
.endm

.macro new_vmalloc_check
	REG_S 	a0, TASK_TI_A0(tp)
	csrr 	a0, CSR_CAUSE
	/* Exclude IRQs */
	blt  	a0, zero, .Lnew_vmalloc_restore_context_a0

	REG_S 	a1, TASK_TI_A1(tp)
	/* Only check new_vmalloc if we are in page/protection fault */
	li   	a1, EXC_LOAD_PAGE_FAULT
	beq  	a0, a1, .Lnew_vmalloc_kernel_address
	li   	a1, EXC_STORE_PAGE_FAULT
	beq  	a0, a1, .Lnew_vmalloc_kernel_address
	li   	a1, EXC_INST_PAGE_FAULT
	bne  	a0, a1, .Lnew_vmalloc_restore_context_a1

.Lnew_vmalloc_kernel_address:
	/* Is it a kernel address? */
	csrr 	a0, CSR_TVAL
	bge 	a0, zero, .Lnew_vmalloc_restore_context_a1

	/* Check if a new vmalloc mapping appeared that could explain the trap */
	REG_S	a2, TASK_TI_A2(tp)
	/*
	 * Computes:
	 * a0 = &new_vmalloc[BIT_WORD(cpu)]
	 * a1 = BIT_MASK(cpu)
	 */
	lw	a2, TASK_TI_CPU(tp)
	/*
	 * Compute the new_vmalloc element position:
	 * (cpu / 64) * 8 = (cpu >> 6) << 3
	 */
	srli	a1, a2, 6
	slli	a1, a1, 3
	la	a0, new_vmalloc
	add	a0, a0, a1
	/*
	 * Compute the bit position in the new_vmalloc element:
	 * bit_pos = cpu % 64 = cpu - (cpu / 64) * 64 = cpu - (cpu >> 6) << 6
	 * 	   = cpu - ((cpu >> 6) << 3) << 3
	 */
	slli	a1, a1, 3
	sub	a1, a2, a1
	/* Compute the "get mask": 1 << bit_pos */
	li	a2, 1
	sll	a1, a2, a1

	/* Check the value of new_vmalloc for this cpu */
	REG_L	a2, 0(a0)
	and	a2, a2, a1
	beq	a2, zero, .Lnew_vmalloc_restore_context

	/* Atomically reset the current cpu bit in new_vmalloc */
	amoxor.d	a0, a1, (a0)

	/* Only emit a sfence.vma if the uarch caches invalid entries */
	ALTERNATIVE("sfence.vma", "nop", 0, RISCV_ISA_EXT_SVVPTC, 1)

	REG_L	a0, TASK_TI_A0(tp)
	REG_L	a1, TASK_TI_A1(tp)
	REG_L	a2, TASK_TI_A2(tp)
	csrw	CSR_SCRATCH, x0
	sret

.Lnew_vmalloc_restore_context:
	REG_L 	a2, TASK_TI_A2(tp)
.Lnew_vmalloc_restore_context_a1:
	REG_L 	a1, TASK_TI_A1(tp)
.Lnew_vmalloc_restore_context_a0:
	REG_L	a0, TASK_TI_A0(tp)
.endm

#ifdef CONFIG_SOC_ESP32S31
	/*
	 * ESP32-S31 runs with stvec forced to CLIC mode, whose BASE field is
	 * 64-byte aligned.  Keep the common Linux exception entry on that
	 * boundary so synchronous traps do not land before handle_exception.
	 */
	.balign 64
#endif
SYM_CODE_START(handle_exception)
	/*
	 * Preserve the interrupted thread pointer and recover Linux identity.
	 * The generic path uses zero scratch for kernel origin; the S31 native
	 * path below uses a permanent canonical pointer and the saved privilege.
	 */
#ifdef CONFIG_ESP32S31_RADIO_SMODE
	/* Native closed code may use tp as scratch/TLS. sscratch is always
	 * canonical, on both harts and in both U/S contexts. Classify origin
	 * from SPP, never from the interrupted tp or a zero scratch sentinel.
	 * Only t0 is used before the complete integer frame has been saved. */
	csrrw tp, CSR_SCRATCH, tp
	REG_S t0, TASK_TI_S31_BLOB_TRAP_T0(tp)
	csrr t0, CSR_SCRATCH
	REG_S t0, TASK_TI_S31_BLOB_TRAP_TP(tp)
	csrw CSR_SCRATCH, tp
	csrr t0, CSR_STATUS
	andi t0, t0, SR_SPP
	bnez t0, .Ls31_kernel_tp_ready
	REG_L t0, TASK_TI_S31_BLOB_TRAP_T0(tp)
	j .Lsave_context
.Ls31_kernel_tp_ready:
#else
	csrrw tp, CSR_SCRATCH, tp
	bnez tp, .Lsave_context

.Lrestore_kernel_tpsp:
	csrr tp, CSR_SCRATCH
#endif

#ifdef CONFIG_64BIT
	/*
	 * The RISC-V kernel does not eagerly emit a sfence.vma after each
	 * new vmalloc mapping, which may result in exceptions:
	 * - if the uarch caches invalid entries, the new mapping would not be
	 *   observed by the page table walker and an invalidation is needed.
	 * - if the uarch does not cache invalid entries, a reordered access
	 *   could "miss" the new mapping and traps: in that case, we only need
	 *   to retry the access, no sfence.vma is required.
	 */
	new_vmalloc_check
#endif

#ifdef CONFIG_ESP32S31_RADIO_SMODE
	/* Explicit alternate-stack entries can publish a dormant Linux SP.
	 * Native radio tasks already run on their registered SRAM Linux stack
	 * and leave this anchor zero. SRAM survives ordinary switch_mm().
	 * Keep PT_SP as the interrupted SP when an alternate entry pivots the
	 * trap frame; nested traps already on a Linux stack use their live SP.
	 */
	REG_S sp, TASK_TI_USER_SP(tp)
	li t0, 0x2f000000
	bltu sp, t0, .Ls31_kernel_sp_ready
	li t0, 0x2f400000
	bgeu sp, t0, .Ls31_kernel_sp_ready
	/* Startup, foreign and critical spans retain SRAM trap frames.
	 * Only a span permitted to schedule may touch its Linux anchor.
	 */
	REG_L t0, TASK_TI_PREEMPT_COUNT(tp)
	bnez t0, .Ls31_kernel_sp_ready
	REG_L t0, TASK_TI_S31_BLOB_LINUX_SP(tp)
	beqz t0, .Ls31_kernel_sp_ready
	mv sp, t0
.Ls31_kernel_sp_ready:
#endif
	REG_S sp, TASK_TI_KERNEL_SP(tp)

#ifdef CONFIG_VMAP_STACK
	addi sp, sp, -(PT_SIZE_ON_STACK)
	srli sp, sp, THREAD_SHIFT
	andi sp, sp, 0x1
	bnez sp, handle_kernel_stack_overflow
	REG_L sp, TASK_TI_KERNEL_SP(tp)
#endif

#ifdef CONFIG_ESP32S31_RADIO_SMODE
	REG_L t0, TASK_TI_S31_BLOB_TRAP_T0(tp)
	j .Ls31_save_kernel_context
#endif
.Lsave_context:
	REG_S sp, TASK_TI_USER_SP(tp)
.Ls31_save_kernel_context:
	REG_L sp, TASK_TI_KERNEL_SP(tp)
	addi sp, sp, -(PT_SIZE_ON_STACK)
	REG_S x1,  PT_RA(sp)
	REG_S x3,  PT_GP(sp)
	REG_S x5,  PT_T0(sp)
	save_from_x6_to_x31

	/*
	 * Disable user-mode memory access as it should only be set in the
	 * actual user copy routines.
	 *
	 * Disable the FPU/Vector to detect illegal usage of floating point
	 * or vector in kernel space.
	 */
	li t0, SR_SUM | SR_FS_VS

	REG_L s0, TASK_TI_USER_SP(tp)
	csrrc s1, CSR_STATUS, t0
	csrr s2, CSR_EPC
	csrr s3, CSR_TVAL
	csrr s4, CSR_CAUSE
#ifdef CONFIG_RISCV_ESP_CLIC_TRAP_QUIRK
	REG_S s4, PT_CAUSE_RAW(sp)	/* raw scause (SPP/SPIL) for sret level restore */
#endif
	mask_clic_scause s4, t0	/* ESP32-S31: strip CLIC SPP/SPIL bits */
#ifdef CONFIG_ESP32S31_RADIO_SMODE
	REG_L s5, TASK_TI_S31_BLOB_TRAP_TP(tp)
#else
	csrr s5, CSR_SCRATCH
#endif
	REG_S s0, PT_SP(sp)
	REG_S s1, PT_STATUS(sp)
	REG_S s2, PT_EPC(sp)
	REG_S s3, PT_BADADDR(sp)
	REG_S s4, PT_CAUSE(sp)
	REG_S s5, PT_TP(sp)

	/*
	 * Generic recursive entry uses the zero-kernel sentinel. S31 already
	 * restored its canonical pointer before touching the exception stack.
	 */
#ifndef CONFIG_ESP32S31_RADIO_SMODE
	csrw CSR_SCRATCH, x0
#endif

	/* Load the global pointer */
	load_global_pointer

	/* Load the kernel shadow call stack pointer if coming from userspace */
	scs_load_current_if_task_changed s5

#ifdef CONFIG_SOC_ESP32S31
	andi t0, s1, SR_SPP
	bnez t0, .Lesp32s31_skip_ext_enter
	move a0, sp
	call esp32s31_ext_enter_kernel
.Lesp32s31_skip_ext_enter:
#endif

#ifdef CONFIG_RISCV_ISA_V_PREEMPTIVE
	move a0, sp
	call riscv_v_context_nesting_start
#endif
	move a0, sp /* pt_regs */

	/*
	 * MSB of cause differentiates between
	 * interrupts and exceptions
	 */
	bge s4, zero, 1f

	/* Handle interrupts */
#ifdef CONFIG_SOC_ESP32S31
	/* Linux sees only the architectural interrupt bit and CLIC ID. */
	li t0, 0xfff
	and s4, s4, t0
	li t0, CAUSE_IRQ_FLAG
	or s4, s4, t0
	REG_S s4, PT_CAUSE(sp)
#endif
	call do_irq
	j ret_from_exception
1:
	/* Handle other exceptions */
#ifdef CONFIG_SOC_ESP32S31
	/* ESP32-S31 Hack: Mask CLIC implementation bits for exceptions */
	li t0, 0xfff
	and s4, s4, t0
	REG_S s4, PT_CAUSE(sp)   /* Store masked cause so handlers see correct code */
#endif
	slli t0, s4, RISCV_LGPTR
	la t1, excp_vect_table
	la t2, excp_vect_table_end
	add t0, t1, t0
	/* Check if exception code lies within bounds */
	bgeu t0, t2, 3f
	REG_L t1, 0(t0)
2:	jalr t1
	j ret_from_exception
3:

	la t1, do_trap_unknown
	j 2b
SYM_CODE_END(handle_exception)
ASM_NOKPROBE(handle_exception)

/*
 * The ret_from_exception must be called with interrupt disabled. Here is the
 * caller list:
 *  - handle_exception
 *  - ret_from_fork
 */
SYM_CODE_START_NOALIGN(ret_from_exception)
	REG_L s0, PT_STATUS(sp)
#ifdef CONFIG_RISCV_M_MODE
	/* the MPP value is too large to be used as an immediate arg for addi */
	li t0, SR_MPP
	and s0, s0, t0
#else
	andi s0, s0, SR_SPP
#endif
	bnez s0, 1f

#ifdef CONFIG_KSTACK_ERASE
	call	stackleak_erase_on_task_stack
#endif

	/* Save unwound kernel stack pointer in thread_info */
	addi s0, sp, PT_SIZE_ON_STACK
	REG_S s0, TASK_TI_KERNEL_SP(tp)

	/* Save the kernel shadow call stack pointer */
	scs_save_current

	/*
	 * Save TP into the scratch register , so we can find the kernel data
	 * structures again.
	 */
#ifdef CONFIG_SOC_ESP32S31
	move a0, sp
	call esp32s31_ext_exit_user
#endif
#ifndef CONFIG_ESP32S31_RADIO_SMODE
	csrw CSR_SCRATCH, tp
#endif
1:
#ifdef CONFIG_ESP32S31_RADIO_SMODE
	/* Install canonical identity before restoring interrupted vendor/user TP. */
	csrw CSR_SCRATCH, tp
#endif
#ifdef CONFIG_RISCV_ISA_V_PREEMPTIVE
	move a0, sp
	call riscv_v_context_nesting_end
#endif
	REG_L a0, PT_STATUS(sp)
	/*
	 * The current load reservation is effectively part of the processor's
	 * state, in the sense that load reservations cannot be shared between
	 * different hart contexts.  We can't actually save and restore a load
	 * reservation, so instead here we clear any existing reservation --
	 * it's always legal for implementations to clear load reservations at
	 * any point (as long as the forward progress guarantee is kept, but
	 * we'll ignore that here).
	 *
	 * Dangling load reservations can be the result of taking a trap in the
	 * middle of an LR/SC sequence, but can also be the result of a taken
	 * forward branch around an SC -- which is how we implement CAS.  As a
	 * result we need to clear reservations between the last CAS and the
	 * jump back to the new context.  While it is unlikely the store
	 * completes, implementations are allowed to expand reservations to be
	 * arbitrarily large.
	 */
	REG_L  a2, PT_EPC(sp)
	REG_SC x0, a2, PT_EPC(sp)
	csrw CSR_STATUS, a0
	csrw CSR_EPC, a2

#ifdef CONFIG_SOC_ESP32S31
	/* Match Espressif downstream: every exit looks interrupt-type to sret,
	 * with SPIL cleared so the supervisor interrupt level returns to idle. */
	REG_L a1, PT_CAUSE_RAW(sp)
	li t0, 0x00ff0000
	not t0, t0
	and a1, a1, t0
	li t0, CAUSE_IRQ_FLAG
	or a1, a1, t0
	csrw CSR_CAUSE, a1
#endif

	REG_L x1,  PT_RA(sp)
	REG_L x3,  PT_GP(sp)
	REG_L x4,  PT_TP(sp)
	REG_L x5,  PT_T0(sp)
	restore_from_x6_to_x31

	REG_L x2,  PT_SP(sp)

#ifdef CONFIG_RISCV_M_MODE
	mret
#else
	sret
#endif
SYM_INNER_LABEL(ret_from_exception_end, SYM_L_GLOBAL)
SYM_CODE_END(ret_from_exception)
ASM_NOKPROBE(ret_from_exception)

#ifdef CONFIG_VMAP_STACK
SYM_CODE_START_LOCAL(handle_kernel_stack_overflow)
	/* Preserve x31 in scratch while selecting the overflow stack. */
	csrrw x31, CSR_SCRATCH, x31
	asm_per_cpu sp, overflow_stack, x31
	li x31, OVERFLOW_STACK_SIZE
	add sp, sp, x31
	/* zero out x31 again and restore x31 */
#ifdef CONFIG_ESP32S31_RADIO_SMODE
	csrrw x31, CSR_SCRATCH, tp
#else
	xor x31, x31, x31
	csrrw x31, CSR_SCRATCH, x31
#endif

	addi sp, sp, -(PT_SIZE_ON_STACK)

	//save context to overflow stack
	REG_S x1,  PT_RA(sp)
	REG_S x3,  PT_GP(sp)
	REG_S x5,  PT_T0(sp)
	save_from_x6_to_x31

	REG_L s0, TASK_TI_KERNEL_SP(tp)
	csrr s1, CSR_STATUS
	csrr s2, CSR_EPC
	csrr s3, CSR_TVAL
	csrr s4, CSR_CAUSE
	mask_clic_scause s4, t0	/* ESP32-S31: strip CLIC SPP/SPIL bits */
#ifdef CONFIG_ESP32S31_RADIO_SMODE
	REG_L s5, TASK_TI_S31_BLOB_TRAP_TP(tp)
#else
	csrr s5, CSR_SCRATCH
#endif
	REG_S s0, PT_SP(sp)
	REG_S s1, PT_STATUS(sp)
	REG_S s2, PT_EPC(sp)
	REG_S s3, PT_BADADDR(sp)
	REG_S s4, PT_CAUSE(sp)
	REG_S s5, PT_TP(sp)
	move a0, sp
	tail handle_bad_stack
SYM_CODE_END(handle_kernel_stack_overflow)
ASM_NOKPROBE(handle_kernel_stack_overflow)
#endif

SYM_CODE_START(ret_from_fork_kernel_asm)
	call schedule_tail
	move a0, s1 /* fn_arg */
	move a1, s0 /* fn */
	move a2, sp /* pt_regs */
	call ret_from_fork_kernel
	j ret_from_exception
SYM_CODE_END(ret_from_fork_kernel_asm)

SYM_CODE_START(ret_from_fork_user_asm)
	call schedule_tail
	move a0, sp /* pt_regs */
	call ret_from_fork_user
	j ret_from_exception
SYM_CODE_END(ret_from_fork_user_asm)

#ifdef CONFIG_IRQ_STACKS
/*
 * void call_on_irq_stack(struct pt_regs *regs,
 * 		          void (*func)(struct pt_regs *));
 *
 * Calls func(regs) using the per-CPU IRQ stack.
 */
SYM_FUNC_START(call_on_irq_stack)
	/* Create a frame record to save ra and s0 (fp) */
	addi	sp, sp, -STACKFRAME_SIZE_ON_STACK
	REG_S	ra, STACKFRAME_RA(sp)
	REG_S	s0, STACKFRAME_FP(sp)
	addi	s0, sp, STACKFRAME_SIZE_ON_STACK

	/* Switch to the per-CPU shadow call stack */
	scs_save_current
	scs_load_irq_stack t0

#ifdef CONFIG_ESP32S31_RADIO_SMODE
	/* Include do_irq() and this live bridge frame in the parked boundary.
	 * A closed ISR borrowing SRAM must not overwrite either frame. */
	REG_S sp, TASK_TI_S31_IRQ_STACK_SP(tp)
#endif
	/* Switch to the per-CPU IRQ stack and call the handler */
	load_per_cpu t0, irq_stack_ptr, t1
	li	t1, IRQ_STACK_SIZE
	add	sp, t0, t1
	jalr	a1

	/* Switch back to the thread shadow call stack */
	scs_load_current

#ifdef CONFIG_ESP32S31_RADIO_SMODE
	REG_S zero, TASK_TI_S31_IRQ_STACK_SP(tp)
#endif
	/* Switch back to the thread stack and restore ra and s0 */
	addi	sp, s0, -STACKFRAME_SIZE_ON_STACK
	REG_L	ra, STACKFRAME_RA(sp)
	REG_L	s0, STACKFRAME_FP(sp)
	addi	sp, sp, STACKFRAME_SIZE_ON_STACK

	ret
SYM_FUNC_END(call_on_irq_stack)
#endif /* CONFIG_IRQ_STACKS */

/*
 * Integer register context switch
 * The callee-saved registers must be saved and restored.
 *
 *   a0: previous task_struct (must be preserved across the switch)
 *   a1: next task_struct
 *
 * The value of a0 and a1 must be preserved by this function, as that's how
 * arguments are passed to schedule_tail.
 */
SYM_FUNC_START(__switch_to)
	/* Save context into prev->thread */
	li    a4,  TASK_THREAD_RA
	add   a3, a0, a4
	add   a4, a1, a4
	REG_S ra,  TASK_THREAD_RA_RA(a3)
	REG_S sp,  TASK_THREAD_SP_RA(a3)
	REG_S s0,  TASK_THREAD_S0_RA(a3)
	REG_S s1,  TASK_THREAD_S1_RA(a3)
	REG_S s2,  TASK_THREAD_S2_RA(a3)
	REG_S s3,  TASK_THREAD_S3_RA(a3)
	REG_S s4,  TASK_THREAD_S4_RA(a3)
	REG_S s5,  TASK_THREAD_S5_RA(a3)
	REG_S s6,  TASK_THREAD_S6_RA(a3)
	REG_S s7,  TASK_THREAD_S7_RA(a3)
	REG_S s8,  TASK_THREAD_S8_RA(a3)
	REG_S s9,  TASK_THREAD_S9_RA(a3)
	REG_S s10, TASK_THREAD_S10_RA(a3)
	REG_S s11, TASK_THREAD_S11_RA(a3)

	/* save the user space access flag */
	csrr  s0, CSR_STATUS
	REG_S s0, TASK_THREAD_SUM_RA(a3)

	/* Save the kernel shadow call stack pointer */
	scs_save_current
	/* Restore context from next->thread */
	REG_L s0,  TASK_THREAD_SUM_RA(a4)
	li    s1,  SR_SUM
	and   s0,  s0, s1
	csrs  CSR_STATUS, s0
	REG_L ra,  TASK_THREAD_RA_RA(a4)
	REG_L sp,  TASK_THREAD_SP_RA(a4)
	REG_L s0,  TASK_THREAD_S0_RA(a4)
	REG_L s1,  TASK_THREAD_S1_RA(a4)
	REG_L s2,  TASK_THREAD_S2_RA(a4)
	REG_L s3,  TASK_THREAD_S3_RA(a4)
	REG_L s4,  TASK_THREAD_S4_RA(a4)
	REG_L s5,  TASK_THREAD_S5_RA(a4)
	REG_L s6,  TASK_THREAD_S6_RA(a4)
	REG_L s7,  TASK_THREAD_S7_RA(a4)
	REG_L s8,  TASK_THREAD_S8_RA(a4)
	REG_L s9,  TASK_THREAD_S9_RA(a4)
	REG_L s10, TASK_THREAD_S10_RA(a4)
	REG_L s11, TASK_THREAD_S11_RA(a4)
	/* The offset of thread_info in task_struct is zero. */
	move tp, a1
#ifdef CONFIG_ESP32S31_RADIO_SMODE
	csrw CSR_SCRATCH, tp
#endif
	/* Switch to the next shadow call stack */
	scs_load_current
	esp32s31_clic_switch_level_reset
	ret
SYM_FUNC_END(__switch_to)

#if defined(CONFIG_SOC_ESP32S31) && !defined(CONFIG_RISCV_M_MODE)
/* Complete the outer CLIC level after its Linux flow handler has returned,
 * before generic IRQ exit may run interruptible softirqs or reschedule.
 * Reuse the task-switch sequence: preserve trap CSRs, retain IRQ exclusion,
 * and never enter M-mode or change the interrupted exception frame.
 */
SYM_FUNC_START(esp32s31_clic_irq_level_complete)
	esp32s31_clic_switch_level_reset
	ret
SYM_FUNC_END(esp32s31_clic_irq_level_complete)
#endif

#ifndef CONFIG_MMU
#define do_page_fault do_trap_unknown
#endif

	.section ".rodata"
	.align LGREG
	/* Exception vector table */
SYM_DATA_START_LOCAL(excp_vect_table)
	RISCV_PTR do_trap_insn_misaligned
	ALT_INSN_FAULT(RISCV_PTR do_trap_insn_fault)
	RISCV_PTR do_trap_insn_illegal
	RISCV_PTR do_trap_break
	RISCV_PTR do_trap_load_misaligned
	RISCV_PTR do_trap_load_fault
	RISCV_PTR do_trap_store_misaligned
	RISCV_PTR do_trap_store_fault
	RISCV_PTR do_trap_ecall_u /* system call */
	RISCV_PTR do_trap_ecall_s
	RISCV_PTR do_trap_unknown
	RISCV_PTR do_trap_ecall_m
	/* instruction page fault */
	ALT_PAGE_FAULT(RISCV_PTR do_page_fault)
	RISCV_PTR do_page_fault   /* load page fault */
	RISCV_PTR do_trap_unknown
	RISCV_PTR do_page_fault   /* store page fault */
SYM_DATA_END_LABEL(excp_vect_table, SYM_L_LOCAL, excp_vect_table_end)

#ifndef CONFIG_MMU
SYM_DATA_START(__user_rt_sigreturn)
	li a7, __NR_rt_sigreturn
	ecall
SYM_DATA_END(__user_rt_sigreturn)
#endif

#if defined(CONFIG_ESP32S31_RADIO_SMODE) && defined(CONFIG_PREEMPTION)
/*
 * Explicit alternate-stack callers can reach generic preempt-enable.
 * Use their published Linux anchor so scheduling and stack inspection see
 * the registered task stack. Native radio tasks use their registered SRAM
 * stack directly and leave the alternate-stack anchor zero.
 *
 * No C prologue, instrumentation, allocation or locking precedes the pivot.
 * s0 retains the alternate frame across schedule(). Leave tp/gp and the
 * published anchor unchanged; nested traps use the live Linux SP.
 */
	.pushsection .text, "ax"

.macro s31_preempt_stack_bridge name, target
SYM_FUNC_START(\name)
	li t0, 0x2f000000
	bltu sp, t0, .Ls31_preempt_plain\@
	li t0, 0x2f400000
	bgeu sp, t0, .Ls31_preempt_plain\@
	REG_L t0, TASK_TI_S31_BLOB_LINUX_SP(tp)
	beqz t0, .Ls31_preempt_plain\@

	/* Both SRAM and published Linux SP obey the 16-byte ABI alignment. */
	addi sp, sp, -16
	REG_S ra, 0(sp)
	REG_S s0, SZREG(sp)
	mv s0, sp
	addi sp, t0, -16
	call \target
	mv sp, s0
	REG_L ra, 0(sp)
	REG_L s0, SZREG(sp)
	addi sp, sp, 16
	ret

.Ls31_preempt_plain\@:
	tail \target
SYM_FUNC_END(\name)
ASM_NOKPROBE(\name)
.endm

/*
 * Preserve the exact target selected by asm-generic/preempt.h.  In a dynamic
 * key build the target retains the existing boot/runtime preemption policy.
 */
#if defined(CONFIG_PREEMPT_DYNAMIC) && defined(CONFIG_HAVE_PREEMPT_DYNAMIC_KEY)
s31_preempt_stack_bridge esp32s31_preempt_schedule, dynamic_preempt_schedule
s31_preempt_stack_bridge esp32s31_preempt_schedule_notrace, dynamic_preempt_schedule_notrace
#else
s31_preempt_stack_bridge esp32s31_preempt_schedule, preempt_schedule
s31_preempt_stack_bridge esp32s31_preempt_schedule_notrace, preempt_schedule_notrace
#endif

EXPORT_SYMBOL(esp32s31_preempt_schedule)
EXPORT_SYMBOL_GPL(esp32s31_preempt_schedule_notrace)
	.popsection
#endif /* CONFIG_ESP32S31_RADIO_SMODE && CONFIG_PREEMPTION */
