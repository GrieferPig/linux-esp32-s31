/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2012 Regents of the University of California
 * Copyright (C) 2017 SiFive
 */

#define RO_EXCEPTION_TABLE_ALIGN	4
#define RUNTIME_DISCARD_EXIT

#ifdef CONFIG_XIP_KERNEL
#include "vmlinux-xip.lds.S"
#else

#include <asm/pgtable.h>
#ifdef CONFIG_ESP32P4_NOMMU_XIP
#define LOAD_OFFSET CONFIG_ESP32P4_XIP_PHYS_ADDR
#define RO_AFTER_INIT_DATA
#else
#define LOAD_OFFSET KERNEL_LINK_ADDR
#endif

#include <asm/vmlinux.lds.h>
#include <asm/page.h>
#include <asm/cache.h>
#include <asm/thread_info.h>
#include <asm/set_memory.h>
#include "image-vars.h"

#include <linux/sizes.h>
OUTPUT_ARCH(riscv)
ENTRY(_start)

jiffies = jiffies_64;

SECTIONS
{
	/* Beginning of code and text segment */
	. = LOAD_OFFSET;
	_start = .;
	HEAD_TEXT_SECTION
	. = ALIGN(PAGE_SIZE);

	.text : {
		_text = .;
		_stext = .;
		TEXT_TEXT
		SCHED_TEXT
		LOCK_TEXT
		KPROBES_TEXT
		ENTRY_TEXT
		IRQENTRY_TEXT
		SOFTIRQENTRY_TEXT
		_etext = .;
	}

	. = ALIGN(SECTION_ALIGN);
#ifndef CONFIG_ESP32P4_NOMMU_XIP
	__init_begin = .;
#endif
	__init_text_begin = .;
	.init.text : AT(ADDR(.init.text) - LOAD_OFFSET) ALIGN(SECTION_ALIGN) { \
		_sinittext = .;						\
		INIT_TEXT						\
		_einittext = .;						\
	}

	. = ALIGN(8);
	__soc_early_init_table : {
		__soc_early_init_table_start = .;
		KEEP(*(__soc_early_init_table))
		__soc_early_init_table_end = .;
	}
	__soc_builtin_dtb_table : {
		__soc_builtin_dtb_table_start = .;
		KEEP(*(__soc_builtin_dtb_table))
		__soc_builtin_dtb_table_end = .;
	}
	/* we have to discard exit text and such at runtime, not link time */
	__exittext_begin = .;
	.exit.text :
	{
		EXIT_TEXT
	}
	__exittext_end = .;

	__init_text_end = .;
	. = ALIGN(SECTION_ALIGN);
#ifdef CONFIG_EFI
	. = ALIGN(PECOFF_SECTION_ALIGNMENT);
	__pecoff_text_end = .;
#endif
#ifdef CONFIG_ESP32P4_NOMMU_XIP
	/* P4 TRM ch.9 System and Memory - separate flash and PSRAM windows. */
	RO_DATA(SECTION_ALIGN)
	.srodata : { *(.srodata*) }
	__esp32p4_data_load = ALIGN(., THREAD_ALIGN);
	. = CONFIG_PHYS_RAM_BASE;
	__init_begin = .;
#undef LOAD_OFFSET
#define LOAD_OFFSET (CONFIG_ESP32P4_XIP_PHYS_ADDR + __init_begin - __esp32p4_data_load)
#endif

	/* Start of init data section */
	__init_data_begin = .;
	INIT_DATA_SECTION(16)

	.init.pi : {
		KEEP(*(.init.pi*))
	}

	.init.bss : {
		KEEP(*(.init.bss*))	/* from the EFI stub */
	}
	.exit.data :
	{
		EXIT_DATA
	}

	RUNTIME_CONST_VARIABLES

	PERCPU_SECTION(L1_CACHE_BYTES)

	.rel.dyn : {
		*(.rel.dyn*)
	}

	.rela.dyn : ALIGN(8) {
		__rela_dyn_start = .;
		*(.rela .rela*)
		__rela_dyn_end = .;
	}

	__init_data_end = .;

	. = ALIGN(8);
	.alternative : {
		__alt_start = .;
		KEEP(*(.alternative))
		__alt_end = .;
	}
	__init_end = .;

	/* Start of data section */
	_sdata = .;
#ifndef CONFIG_ESP32P4_NOMMU_XIP
	RO_DATA(SECTION_ALIGN)
	.srodata : {
		*(.srodata*)
	}
#endif

	. = ALIGN(SECTION_ALIGN);
	_data = .;

	RW_DATA(L1_CACHE_BYTES, PAGE_SIZE, THREAD_ALIGN)
	.sdata : {
		__global_pointer$ = . + 0x800;
		*(.sdata*)
	}

	.got : { *(.got*) }

#ifdef CONFIG_RELOCATABLE
	.data.rel : { *(.data.rel*) }
	.plt : { *(.plt) }
	.dynamic : { *(.dynamic) }
	.dynsym : { *(.dynsym) }
	.dynstr : { *(.dynstr) }
	.hash : { *(.hash) }
	.gnu.hash : { *(.gnu.hash) }
#endif

#ifdef CONFIG_EFI
	.pecoff_edata_padding : { BYTE(0); . = ALIGN(PECOFF_FILE_ALIGNMENT); }
	__pecoff_data_raw_size = ABSOLUTE(. - __pecoff_text_end);
	__pecoff_data_raw_end = ABSOLUTE(.);
#endif

	/* End of data section */
#ifdef CONFIG_ESP32P4_NOMMU_XIP
	.data.ro_after_init : AT(ADDR(.data.ro_after_init) - LOAD_OFFSET) {
		__start_ro_after_init = .;
		*(.data..ro_after_init)
		. = ALIGN(4);
		__end_ro_after_init = .;
	}
#endif
	_edata = .;
#ifdef CONFIG_ESP32P4_NOMMU_XIP
	__esp32p4_xip_size = ABSOLUTE(__esp32p4_data_load - _start + _edata - __init_begin);
	ASSERT((__esp32p4_data_load & 3) == 0 && (_edata & 3) == 0,
	       "P4 initialized data must be word-aligned")
#endif

	BSS_SECTION(PAGE_SIZE, PAGE_SIZE, 0)

#ifdef CONFIG_EFI
	. = ALIGN(PECOFF_SECTION_ALIGNMENT);
	__pecoff_data_virt_size = ABSOLUTE(. - __pecoff_text_end);
	__pecoff_data_virt_end = ABSOLUTE(.);
#endif
	_end = .;

	STABS_DEBUG
	DWARF_DEBUG
	ELF_DETAILS
	.riscv.attributes 0 : { *(.riscv.attributes) }

	DISCARDS
}
#endif /* CONFIG_XIP_KERNEL */
