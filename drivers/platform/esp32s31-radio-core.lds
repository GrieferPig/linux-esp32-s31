/* SPDX-License-Identifier: GPL-2.0-only */
/* Preserve the versioned Linux client API while pruning unused IDF sections. */
SECTIONS
{
	/* The composite link uses --gc-sections.  Keep every parameter record and
	 * its description, including parameters declared outside the module entry
	 * translation unit. */
	__param : { KEEP(*(__param)) }
	.modinfo : { KEEP(*(.modinfo)) }
	.init.text : { KEEP(*(.init.text)) KEEP(*(.init.text.*)) }
	.exit.text : { KEEP(*(.exit.text)) KEEP(*(.exit.text.*)) }
	.init.data : { KEEP(*(.init.data)) KEEP(*(.init.data.*)) }
	.exit.data : { KEEP(*(.exit.data)) KEEP(*(.exit.data.*)) }
	.export_symbol : { KEEP(*(.export_symbol)) }
	.rodata.s31_fw_imports : {
		KEEP(*(.rodata.s31_fw_imports))
		KEEP(*(.rodata.s31_fw_import_names))
	}
}
