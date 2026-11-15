// SPDX-License-Identifier: GPL-2.0-only
/*
 * Independent reconstruction against the audited retained IDF callback ABI.
 * Does not implement DMA invalidation: only executable sync and display WB.
 */
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/libfdt.h>
#include <linux/printk.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/soc/espressif/esp32p4.h>
#include <asm/soc.h>

typedef int (*p4_cache_fn)(u32 start, u32 size);
static p4_cache_fn exec_sync;
static p4_cache_fn display_writeback;
static DEFINE_RAW_SPINLOCK(p4_cache_engine_lock);
/* The retained target is v1.3; IRQ init verifies chosen revision before enable. */
u32 esp32p4_clic_rev1 = 1;

static u32 __init p4_fdt_u32(const void *fdt, int node, const char *name)
{
	const fdt32_t *p;
	int size;

	p = fdt_getprop(fdt, node, name, &size);
	return p && size == sizeof(*p) ? fdt32_to_cpu(*p) : 0;
}

static void __init p4_cache_early_init(const void *fdt)
{
	int chosen = fdt_path_offset(fdt, "/chosen");
	int fb, child;
	int matches = 0;
	int status_len;
	const char *status;
	u32 entry = p4_fdt_u32(fdt, chosen, "espressif,cache-sync-entry");
	u32 wb = 0;
	u32 rev = p4_fdt_u32(fdt, chosen, "espressif,chip-revision");

	fb = -1;
	fdt_for_each_subnode(child, fdt, chosen) {
		if (fdt_node_check_compatible(fdt, child,
				"espressif,esp32p4-idf-framebuffer"))
			continue;
		status = fdt_getprop(fdt, child, "status", &status_len);
		if (status && (status_len < 5 || strcmp(status, "okay")))
			continue;
		fb = child;
		matches++;
	}
	if (matches == 1)
		wb = p4_fdt_u32(fdt, fb, "espressif,cache-writeback-entry");
	if (esp32p4_retained_entry_valid(entry))
		exec_sync = (p4_cache_fn)(unsigned long)entry;
	if (esp32p4_retained_entry_valid(wb))
		display_writeback = (p4_cache_fn)(unsigned long)wb;
	/* Default .data value is already v1 before the first possible trap.
	 * Only this retained revision103 handoff is reconstructed. Never switch
	 * to an unverified CSR bank before a later IRQ probe rejects the DT. */
	if (rev != 103)
		panic("P4 requires revision103 retained IDF handoff");
	esp32p4_clic_rev1 = 1;
}
SOC_EARLY_INIT_DECLARE(esp32p4_cache, "espressif,esp32p4", p4_cache_early_init);

int esp32p4_display_cache_writeback(u32 start, u32 size)
{
	unsigned long flags;
	int ret;

	if (!size)
		return 0;
	if (!display_writeback)
		return -ENODEV;
	if (start < ESP32P4_FB_START || start >= ESP32P4_FB_END ||
	    size > ESP32P4_FB_END - start || size > ESP32P4_CACHE_CHUNK ||
	    ((start | size) & 63))
		return -EINVAL;
	raw_spin_lock_irqsave(&p4_cache_engine_lock, flags);
	ret = display_writeback(start, size);
	raw_spin_unlock_irqrestore(&p4_cache_engine_lock, flags);
	return ret ? -EIO : 0;
}
EXPORT_SYMBOL_GPL(esp32p4_display_cache_writeback);

void esp32p4_cache_sync_for_exec(unsigned long start, unsigned long end)
{
	unsigned long flags;
	int ret;

	if (end <= start)
		return;
	if (start < ESP32P4_RAM_START || end > ESP32P4_RAM_END)
		panic("P4 executable cache range invalid: %lx..%lx", start, end);
	if (!exec_sync)
		panic("P4 retained executable cache callback missing");
	raw_spin_lock_irqsave(&p4_cache_engine_lock, flags);
	ret = exec_sync(start, end - start);
	raw_spin_unlock_irqrestore(&p4_cache_engine_lock, flags);
	if (ret)
		panic("P4 executable cache sync failed: %lx..%lx", start, end);
	asm volatile("fence.i" ::: "memory");
}
EXPORT_SYMBOL_GPL(esp32p4_cache_sync_for_exec);

void esp32p4_cache_sync_all_for_exec(void)
{
	esp32p4_cache_sync_for_exec(ESP32P4_RAM_START, ESP32P4_RAM_END);
}
