/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ASM_RISCV_ESP32S31_CACHE_H
#define _ASM_RISCV_ESP32S31_CACHE_H

#ifdef __ASSEMBLY__
/*
 * S31's Sv32 page-table walker does not snoop the shared write-back D-cache.
 * Publish cached PSRAM before installing a page-table root stored there.  The
 * cache operation is issued twice to match the ESP-IDF ROM workaround.
 *
 * Clobbers t0-t2 and is valid only while physical peripheral addresses are
 * directly accessible.
 */
	.macro ESP32S31_DCACHE_WRITEBACK_LINUX_PSRAM
		li t0, 0x2c000000
		li t1, 0x10		/* CACHE_MAP_L1_DCACHE */
		sw t1, 0xa0(t0)
		li t1, CONFIG_PHYS_RAM_BASE
		sw t1, 0xa4(t0)
		li t1, 0x01000000
		sw t1, 0xa8(t0)
		fence ow, ow
		li t1, 0x4		/* CACHE_WRITEBACK_ENA */
		sw t1, 0x9c(t0)
.Lcache_wb_wait1\@:
		lw t2, 0x9c(t0)
		andi t2, t2, 0x10
		beqz t2, .Lcache_wb_wait1\@
		sw t1, 0x9c(t0)
.Lcache_wb_wait2\@:
		lw t2, 0x9c(t0)
		andi t2, t2, 0x10
		beqz t2, .Lcache_wb_wait2\@
		fence iorw, iorw
	.endm
#endif

#endif /* _ASM_RISCV_ESP32S31_CACHE_H */
