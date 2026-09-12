/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ASM_RISCV_SBI_ECALL_H
#define _ASM_RISCV_SBI_ECALL_H

#ifdef CONFIG_SOC_ESP32S31
/* An exception-type mret can leave CLIC SIL at its 0xff sentinel. Recover
 * through an IRQ-disabled S-mode sret, including the direct noinstr WFI call.
 * OpenSBI preserves t0..t2 across ecall. Preserve all caller trap CSRs and
 * a0/a1 results; declare every scratch register to the compiler.
 * Temporary scause sets INT with SPIL=0. Temporary sstatus sets SPP and
 * clears SIE/SPIE (mask ~0x22), so sret stays in S-mode with IRQs disabled.
 */
#define RISCV_SBI_ECALL_ASM \
	"csrr t0, sstatus\n" \
	"csrci sstatus, 2\n" \
	"csrr t1, sepc\n" \
	"csrr t2, scause\n" \
	"ecall\n" \
	"li t3, 0x80000000\n" \
	"csrw scause, t3\n" \
	"ori t3, t0, 0x100\n" \
	"li t4, -35\n" \
	"and t3, t3, t4\n" \
	"csrw sstatus, t3\n" \
	"lla t3, 1f\n" \
	"csrw sepc, t3\n" \
	"sret\n" \
	"1: csrw scause, t2\n" \
	"csrw sepc, t1\n" \
	"csrw sstatus, t0\n"
#define RISCV_SBI_ECALL_CLOBBERS "memory", "t0", "t1", "t2", "t3", "t4"
#else
#define RISCV_SBI_ECALL_ASM "ecall"
#define RISCV_SBI_ECALL_CLOBBERS "memory"
#endif

#endif /* _ASM_RISCV_SBI_ECALL_H */
