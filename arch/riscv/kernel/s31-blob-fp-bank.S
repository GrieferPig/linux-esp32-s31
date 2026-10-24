/* SPDX-License-Identifier: GPL-2.0-only */
/* RV32F full register bank. This implementation requires RV32F.
 * a0 points at globally mapped/cache-accessible 132-byte bank.
 * t0/t1 are caller-saved. Only FS bits of sstatus are changed.
 */
.text
.balign 4
.global s31_fp_bank_save_rv32
.type s31_fp_bank_save_rv32,@function
s31_fp_bank_save_rv32:
  li t1, 0x6000
  csrrs t0, sstatus, t1
  and t0, t0, t1
  fsw f0, 0(a0)
  fsw f1, 4(a0)
  fsw f2, 8(a0)
  fsw f3, 12(a0)
  fsw f4, 16(a0)
  fsw f5, 20(a0)
  fsw f6, 24(a0)
  fsw f7, 28(a0)
  fsw f8, 32(a0)
  fsw f9, 36(a0)
  fsw f10, 40(a0)
  fsw f11, 44(a0)
  fsw f12, 48(a0)
  fsw f13, 52(a0)
  fsw f14, 56(a0)
  fsw f15, 60(a0)
  fsw f16, 64(a0)
  fsw f17, 68(a0)
  fsw f18, 72(a0)
  fsw f19, 76(a0)
  fsw f20, 80(a0)
  fsw f21, 84(a0)
  fsw f22, 88(a0)
  fsw f23, 92(a0)
  fsw f24, 96(a0)
  fsw f25, 100(a0)
  fsw f26, 104(a0)
  fsw f27, 108(a0)
  fsw f28, 112(a0)
  fsw f29, 116(a0)
  fsw f30, 120(a0)
  fsw f31, 124(a0)
  frcsr t1
  sw t1, 128(a0)
  li t1, 0x6000
  csrc sstatus, t1
  csrs sstatus, t0
  ret
.size s31_fp_bank_save_rv32,.-s31_fp_bank_save_rv32
.balign 4
.global s31_fp_bank_restore_rv32
.type s31_fp_bank_restore_rv32,@function
s31_fp_bank_restore_rv32:
  li t1, 0x6000
  csrrs t0, sstatus, t1
  and t0, t0, t1
  flw f0, 0(a0)
  flw f1, 4(a0)
  flw f2, 8(a0)
  flw f3, 12(a0)
  flw f4, 16(a0)
  flw f5, 20(a0)
  flw f6, 24(a0)
  flw f7, 28(a0)
  flw f8, 32(a0)
  flw f9, 36(a0)
  flw f10, 40(a0)
  flw f11, 44(a0)
  flw f12, 48(a0)
  flw f13, 52(a0)
  flw f14, 56(a0)
  flw f15, 60(a0)
  flw f16, 64(a0)
  flw f17, 68(a0)
  flw f18, 72(a0)
  flw f19, 76(a0)
  flw f20, 80(a0)
  flw f21, 84(a0)
  flw f22, 88(a0)
  flw f23, 92(a0)
  flw f24, 96(a0)
  flw f25, 100(a0)
  flw f26, 104(a0)
  flw f27, 108(a0)
  flw f28, 112(a0)
  flw f29, 116(a0)
  flw f30, 120(a0)
  flw f31, 124(a0)
  lw t1, 128(a0)
  fscsr t1
  li t1, 0x6000
  csrc sstatus, t1
  csrs sstatus, t0
  ret
.size s31_fp_bank_restore_rv32,.-s31_fp_bank_restore_rv32
