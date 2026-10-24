/* SPDX-License-Identifier: GPL-2.0-only */
/* RV32 little-endian, kernel-to-kernel copy with Internet checksum.
 * Every source byte is loaded once. Word loads and word/halfword stores
 * are naturally aligned; the prefix/tail never over-read either buffer.
 * No FP, vector, PIE, HWLoop, CSR or privileged operation is used.
 */
#include <linux/linkage.h>
#include <linux/export.h>
#include <asm/asm.h>

#ifdef CONFIG_64BIT
#error "ESP32-S31 checksum-copy requires RV32"
#endif

/* a0=source, a1=destination, a3=word sum, a4=end-around carry count.
 * t0=data, t1=store/carry scratch. The byte-store variant also uses t3.
 */
.macro S31_CSUM_COPY_WORD offset, kind
	lw t0, \offset(a0)
	.ifc \kind,word
	sw t0, \offset(a1)
	.else
	.ifc \kind,half
	sh t0, \offset(a1)
	srli t1, t0, 16
	sh t1, (\offset+2)(a1)
	.else
	sb t0, \offset(a1)
	srli t1, t0, 8
	sb t1, (\offset+1)(a1)
	srli t1, t0, 16
	sb t1, (\offset+2)(a1)
	srli t1, t0, 24
	sb t1, (\offset+3)(a1)
	.endif
	.endif
	add a3, a3, t0
	sltu t1, a3, t0
	add a4, a4, t1
.endm

/* Select destination alignment once, outside the unrolled word loop. */
.macro S31_CSUM_COPY_BODY name, kind
.Ls31_csum_\name:
	sltiu t0, a2, 32
	bnez t0, .Ls31_csum_\name\()_short
	andi t2, a2, -32
	add t2, t2, a0
.Ls31_csum_\name\()_loop:
	.irp offset,0,4,8,12,16,20,24,28
	S31_CSUM_COPY_WORD \offset, \kind
	.endr
	addi a0, a0, 32
	addi a1, a1, 32
	bltu a0, t2, .Ls31_csum_\name\()_loop
	andi a2, a2, 31
.Ls31_csum_\name\()_short:
	sltiu t0, a2, 4
	bnez t0, .Ls31_csum_tail
	S31_CSUM_COPY_WORD 0, \kind
	addi a0, a0, 4
	addi a1, a1, 4
	addi a2, a2, -4
	j .Ls31_csum_\name\()_short
.endm

#ifdef CONFIG_XIP_KERNEL
.section .s31_rx_ram.text,"ax",@progbits
#else
.text
#endif
SYM_FUNC_START(__s31_csum_copy)
	blez a2, .Ls31_csum_zero
	li a3, 0
	li a4, 0
	li a5, 0 /* checksum of zero to three leading bytes */
	li a6, 0 /* alternating byte shift */
	li t4, 0 /* prefix parity */
	andi t0, a0, 3
	beqz t0, .Ls31_csum_aligned
.Ls31_csum_prefix:
	lbu t0, 0(a0)
	sb t0, 0(a1)
	sll t1, t0, a6
	add a5, a5, t1
	xori a6, a6, 8
	xori t4, t4, 1
	addi a0, a0, 1
	addi a1, a1, 1
	addi a2, a2, -1
	beqz a2, .Ls31_csum_fold
	andi t0, a0, 3
	bnez t0, .Ls31_csum_prefix
.Ls31_csum_aligned:
	andi t0, a1, 3
	beqz t0, .Ls31_csum_word
	li t1, 2
	beq t0, t1, .Ls31_csum_half
	j .Ls31_csum_byte

	S31_CSUM_COPY_BODY word, word
	S31_CSUM_COPY_BODY half, half
	S31_CSUM_COPY_BODY byte, byte

.Ls31_csum_tail:
	beqz a2, .Ls31_csum_fold
	li t5, 0
.Ls31_csum_tail_loop:
	lbu t0, 0(a0)
	sb t0, 0(a1)
	sll t0, t0, t5
	add a3, a3, t0
	sltu t1, a3, t0
	add a4, a4, t1
	xori t5, t5, 8
	addi a0, a0, 1
	addi a1, a1, 1
	addi a2, a2, -1
	bnez a2, .Ls31_csum_tail_loop
.Ls31_csum_fold:
	/* Add the word carries, then fold to an uncomplemented 16-bit sum.
	 * The maximum positive int length cannot overflow the carry count. */
	add a3, a3, a4
	sltu t0, a3, a4
	add a3, a3, t0
	li t2, 0xffff
	srli t0, a3, 16
	and a3, a3, t2
	add a3, a3, t0
	srli t0, a3, 16
	and a3, a3, t2
	add a3, a3, t0
	beqz t4, .Ls31_csum_join_prefix
	/* The aligned body began at an odd checksum offset. */
	srli t0, a3, 8
	slli a3, a3, 8
	or a3, a3, t0
	and a3, a3, t2
.Ls31_csum_join_prefix:
	add a3, a3, a5
	srli t0, a3, 16
	and a3, a3, t2
	add a3, a3, t0
	srli t0, a3, 16
	and a3, a3, t2
	add a0, a3, t0
	ret
.Ls31_csum_zero:
	li a0, 0
	ret
SYM_FUNC_END(__s31_csum_copy)
EXPORT_SYMBOL(__s31_csum_copy)
