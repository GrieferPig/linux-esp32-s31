// SPDX-License-Identifier: GPL-2.0-only
/* Fixed flash-XIP and writable SRAM mapping for the S31 radio. */
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#ifdef CONFIG_ESP32S31_RADIO_XIP
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioremap.h>
#include <linux/string.h>
#include <asm/tlbflush.h>
#include "esp32s31-radio-xip.h"

u8 esp32s31_radio_xip_ram[S31_XIP_RAM_SIZE] __aligned(PAGE_SIZE);
EXPORT_SYMBOL_GPL(esp32s31_radio_xip_ram);
static const void *esp32s31_radio_xip_mapping;

const void *esp32s31_radio_xip_get(void);
const void *esp32s31_radio_xip_get(void)
{
	return esp32s31_radio_xip_mapping;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_xip_get);

static int __init esp32s31_radio_xip_init(void)
{
	struct vm_struct *area;
	pgd_t *pgdp;

	BUILD_BUG_ON(S31_XIP_MAP_BASE < VMALLOC_START ||
		     S31_XIP_MAP_BASE + S31_XIP_MAP_SIZE >= VMALLOC_END ||
		     S31_XIP_MAP_SIZE != PGDIR_SIZE);
	area = __get_vm_area_caller(S31_XIP_MAP_SIZE, VM_IOREMAP,
		S31_XIP_MAP_BASE, S31_XIP_MAP_BASE + S31_XIP_MAP_SIZE + PAGE_SIZE,
		__builtin_return_address(0));
	if (!area) {
		pr_err("esp32s31-radio: cannot reserve XIP virtual window\n");
		return -ENOMEM;
	}
	if ((unsigned long)area->addr != S31_XIP_MAP_BASE) {
		pr_err("esp32s31-radio: XIP window allocated at %px\n", area->addr);
		free_vm_area(area);
		return -EBUSY;
	}
	area->phys_addr = S31_XIP_MAP_PHYS;
	pgdp = pgd_offset_k(S31_XIP_MAP_BASE);
	/* pgd_none() is a folded-level stub on Sv32; test the real PMD. */
	if (!pmd_none(*(pmd_t *)pgdp)) {
		pr_err("esp32s31-radio: XIP PGD already populated: %lx\n", pgd_val(*pgdp));
		free_vm_area(area);
		return -EBUSY;
	}
	/* Use the same aligned Sv32 leaf format as the working kernel XIP map.
	 * The image is at offset 0x210000 within this reserved flash window.
	 */
	set_pmd((pmd_t *)pgdp, __pmd(pgd_val(pfn_pgd(
		S31_XIP_MAP_PHYS >> PAGE_SHIFT, PAGE_KERNEL_READ_EXEC))));
	flush_tlb_kernel_range(S31_XIP_MAP_BASE,
		S31_XIP_MAP_BASE + S31_XIP_MAP_SIZE);
	esp32s31_radio_xip_mapping = (const void *)S31_XIP_BASE;
	pr_info("esp32s31-radio: reserved XIP %px, RAM %px/%u bytes\n",
		esp32s31_radio_xip_mapping, esp32s31_radio_xip_ram, S31_XIP_RAM_SIZE);
	return 0;
}
postcore_initcall(esp32s31_radio_xip_init);
#endif
