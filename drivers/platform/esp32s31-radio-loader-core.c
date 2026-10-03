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
#include <asm/set_memory.h>
#include "esp32s31-radio-xip.h"

u8 esp32s31_radio_xip_ram[S31_XIP_RAM_SIZE] __aligned(PAGE_SIZE);
EXPORT_SYMBOL_GPL(esp32s31_radio_xip_ram);
static const void *esp32s31_radio_xip_mapping;
static void *esp32s31_radio_wifi_iram_mapping;

const void *esp32s31_radio_xip_get(void);
const void *esp32s31_radio_xip_get(void)
{
	return esp32s31_radio_xip_mapping;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_xip_get);

void *esp32s31_radio_wifi_iram_get(void);
void *esp32s31_radio_wifi_iram_get(void)
{
	return esp32s31_radio_wifi_iram_mapping;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_wifi_iram_get);

static int __init esp32s31_radio_xip_init(void)
{
	struct vm_struct *area;
	pgd_t *pgdp;
	int ret;

	BUILD_BUG_ON(S31_XIP_MAP_BASE < VMALLOC_START ||
		     S31_XIP_MAP_BASE + S31_XIP_MAP_SIZE >= VMALLOC_END ||
		     S31_XIP_MAP_SIZE != PGDIR_SIZE ||
		     (S31_XIP_MAP_BASE & (PGDIR_SIZE - 1)) ||
		     (S31_XIP_MAP_PHYS & (PGDIR_SIZE - 1)) ||
		     S31_XIP_BASE < S31_XIP_MAP_BASE ||
		     S31_XIP_BASE + S31_XIP_SLOT_SIZE >
					S31_XIP_MAP_BASE + S31_XIP_MAP_SIZE ||
		     S31_XIP_PHYS - S31_XIP_MAP_PHYS !=
					S31_XIP_BASE - S31_XIP_MAP_BASE);
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
	 * The compact image starts at raw flash offset 0x6e000 within this
	 * reserved window. SPL maps raw flash offset zero at 0x40000000.
	 */
	set_pmd((pmd_t *)pgdp, __pmd(pgd_val(pfn_pgd(
		S31_XIP_MAP_PHYS >> PAGE_SHIFT, PAGE_KERNEL_READ_EXEC))));
	flush_tlb_kernel_range(S31_XIP_MAP_BASE,
		S31_XIP_MAP_BASE + S31_XIP_MAP_SIZE);
	esp32s31_radio_xip_mapping = (const void *)S31_XIP_BASE;
	/* The Wi-Fi-specific IDF IRAM sections execute from the unused upper
	 * part of the private SRAM heap.  A high alias keeps PC-relative calls
	 * near radio XIP, and unlike the init_mm-only physical mapping it remains
	 * reachable when an interrupt arrives in a user process page table.
	 * Leave a gap for the XIP vmalloc area's guard page.  VM_IOREMAP
	 * aligns this 72 KiB request to 128 KiB, so the alias must be aligned
	 * to that boundary as well. */
	BUILD_BUG_ON(S31_XIP_WIFI_IRAM_BASE <
		     S31_XIP_MAP_BASE + S31_XIP_MAP_SIZE + PAGE_SIZE ||
		     (S31_XIP_WIFI_IRAM_BASE & 0x1ffffU) ||
		     S31_XIP_WIFI_IRAM_BASE + S31_XIP_WIFI_IRAM_MAP_SIZE +
						 PAGE_SIZE >= VMALLOC_END ||
		     S31_XIP_WIFI_IRAM_MAP_SIZE != PAGE_ALIGN(
						 S31_XIP_WIFI_IRAM_CAPACITY));
	area = __get_vm_area_caller(S31_XIP_WIFI_IRAM_MAP_SIZE, VM_IOREMAP,
		S31_XIP_WIFI_IRAM_BASE,
		S31_XIP_WIFI_IRAM_BASE + S31_XIP_WIFI_IRAM_MAP_SIZE + PAGE_SIZE,
		__builtin_return_address(0));
	if (!area || (unsigned long)area->addr != S31_XIP_WIFI_IRAM_BASE) {
		pr_err("esp32s31-radio: cannot reserve Wi-Fi IRAM alias\n");
		if (area)
			free_vm_area(area);
		return -ENOMEM;
	}
	area->phys_addr = S31_XIP_WIFI_IRAM_PHYS;
	ret = ioremap_page_range(S31_XIP_WIFI_IRAM_BASE,
		S31_XIP_WIFI_IRAM_BASE + S31_XIP_WIFI_IRAM_MAP_SIZE,
		S31_XIP_WIFI_IRAM_PHYS, PAGE_KERNEL);
	if (!ret)
		ret = set_memory_x(S31_XIP_WIFI_IRAM_BASE,
				   S31_XIP_WIFI_IRAM_MAP_SIZE >> PAGE_SHIFT);
	if (ret) {
		pr_err("esp32s31-radio: Wi-Fi IRAM mapping failed: %d\n", ret);
		vunmap_range(S31_XIP_WIFI_IRAM_BASE,
			     S31_XIP_WIFI_IRAM_BASE + S31_XIP_WIFI_IRAM_MAP_SIZE);
		free_vm_area(area);
		return ret;
	}
	flush_tlb_kernel_range(S31_XIP_WIFI_IRAM_BASE,
		S31_XIP_WIFI_IRAM_BASE + S31_XIP_WIFI_IRAM_MAP_SIZE);
	esp32s31_radio_wifi_iram_mapping = area->addr;
	pr_info("esp32s31-radio: reserved XIP %px, RAM %px/%u bytes\n",
		esp32s31_radio_xip_mapping, esp32s31_radio_xip_ram, S31_XIP_RAM_SIZE);
	pr_info("esp32s31-radio: Wi-Fi IRAM %px -> %#x (%u bytes)\n",
		esp32s31_radio_wifi_iram_mapping, S31_XIP_WIFI_IRAM_PHYS,
		S31_XIP_WIFI_IRAM_CAPACITY);
	return 0;
}
postcore_initcall(esp32s31_radio_xip_init);
#endif
