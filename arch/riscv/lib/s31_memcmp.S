/* SPDX-License-Identifier: GPL-2.0-only */
#include <linux/linkage.h>
#include <asm/asm.h>

/* Compare aligned words without reading outside either supplied range.
 * Unequal words use byte order to preserve the ordinary memcmp result.
 * No FPU, vector, or optional bit-manipulation state is touched.
 */
.pushsection .s31_rx_ram.text,"ax",@progbits
SYM_FUNC_START(memcmp)
	beqz a2, .Lequal
	xor t0, a0, a1
	andi t0, t0, 3
	bnez t0, .Lbytes
.Lalign:
	andi t0, a0, 3
	beqz t0, .Lwords
	lbu t0, 0(a0)
	lbu t1, 0(a1)
	sub a3, t0, t1
	bnez a3, .Ldifferent
	addi a0, a0, 1
	addi a1, a1, 1
	addi a2, a2, -1
	bnez a2, .Lalign
	j .Lequal
.Lwords:
	li t2, 16
	bltu a2, t2, .Lword_tail
	lw t0, 0(a0)
	lw t1, 0(a1)
	bne t0, t1, .Lbytes
	lw t0, 4(a0)
	lw t1, 4(a1)
	bne t0, t1, .Lword_4
	lw t0, 8(a0)
	lw t1, 8(a1)
	bne t0, t1, .Lword_8
	lw t0, 12(a0)
	lw t1, 12(a1)
	bne t0, t1, .Lword_12
	addi a0, a0, 16
	addi a1, a1, 16
	addi a2, a2, -16
	j .Lwords
.Lword_12:
	addi a0, a0, 4
	addi a1, a1, 4
	addi a2, a2, -4
.Lword_8:
	addi a0, a0, 4
	addi a1, a1, 4
	addi a2, a2, -4
.Lword_4:
	addi a0, a0, 4
	addi a1, a1, 4
	addi a2, a2, -4
	j .Lbytes
.Lword_tail:
	li t2, 4
	bltu a2, t2, .Lbytes
	lw t0, 0(a0)
	lw t1, 0(a1)
	bne t0, t1, .Lbytes
	addi a0, a0, 4
	addi a1, a1, 4
	addi a2, a2, -4
	j .Lword_tail
.Lbytes:
	beqz a2, .Lequal
	lbu t0, 0(a0)
	lbu t1, 0(a1)
	sub a3, t0, t1
	bnez a3, .Ldifferent
	addi a0, a0, 1
	addi a1, a1, 1
	addi a2, a2, -1
	j .Lbytes
.Ldifferent:
	mv a0, a3
	ret
.Lequal:
	li a0, 0
	ret
SYM_FUNC_END(memcmp)
.popsection
