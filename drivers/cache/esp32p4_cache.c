// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-P4 cache maintenance for DMA, executable data, and display.
 */
#include <linux/cacheflush.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/libfdt.h>
#include <linux/printk.h>
#include <linux/spinlock.h>
#include <linux/sizes.h>
#include <linux/string.h>
#include <linux/soc/espressif/esp32p4.h>
#include <asm/dma-noncoherent.h>
#include <asm/soc.h>

typedef int (*p4_cache_fn)(u32 start, u32 size);
static p4_cache_fn exec_sync;
static p4_cache_fn display_writeback;
static p4_cache_fn camera_invalidate;
static DEFINE_RAW_SPINLOCK(p4_cache_engine_lock);
static u32 p4_exec_ram_end __ro_after_init = ESP32P4_RAM_END;
/* The retained target is v1.3; IRQ init verifies chosen revision before enable. */
u32 esp32p4_clic_rev1 = 1;

/*
 * TRM ch.9 System and Memory - external RAM and DMA bus paths.
 * The public TRM does not describe the cache-sync register bank.
 * See ESP-IDF cache_ll_writeback_addr() and cache_ll_invalidate_addr()
 * for the rev1.x CACHE_SYNC_CTRL/MAP/ADDR/SIZE programming sequence.
 */
#define P4_CACHE_SYNC_BASE	0x3ff10098u
#define P4_CACHE_SYNC_CTRL	0x00
#define P4_CACHE_SYNC_MAP	0x04
#define P4_CACHE_SYNC_ADDR	0x08
#define P4_CACHE_SYNC_SIZE	0x0c
#define P4_CACHE_SYNC_OPS	GENMASK(3, 0)
#define P4_CACHE_SYNC_DONE	BIT(4)
#define P4_CACHE_SYNC_WB	BIT(2)
#define P4_CACHE_SYNC_INV	BIT(0)
#define P4_CACHE_L1D		BIT(4)
#define P4_CACHE_L2		BIT(5)
#define P4_DMA_CACHE_ALIGN	128

static int p4_cache_dma_sync(u32 start, u32 size, u32 map, u32 op)
{
	void __iomem *regs = (void __iomem *)P4_CACHE_SYNC_BASE;
	u32 status;
	int ret;

	ret = readl_poll_timeout_atomic(regs + P4_CACHE_SYNC_CTRL, status,
				       !(status & P4_CACHE_SYNC_OPS), 0, 1000);
	if (ret)
		return ret;
	writel(start, regs + P4_CACHE_SYNC_ADDR);
	writel(size, regs + P4_CACHE_SYNC_SIZE);
	writel(map, regs + P4_CACHE_SYNC_MAP);
	writel(op, regs + P4_CACHE_SYNC_CTRL);
	return readl_poll_timeout_atomic(regs + P4_CACHE_SYNC_CTRL, status,
					 (status & (P4_CACHE_SYNC_OPS |
						    P4_CACHE_SYNC_DONE)) ==
					 P4_CACHE_SYNC_DONE, 0, 1000);
}

static int p4_cache_range_op(phys_addr_t paddr, size_t size, u32 op)
{
	unsigned long flags;
	u32 start, end, len;
	int ret = 0;

	if (!size)
		return 0;
	if (paddr < ESP32P4_RAM_START || paddr >= p4_exec_ram_end ||
	    size > p4_exec_ram_end - paddr)
		return -EINVAL;

	start = ALIGN_DOWN(paddr, P4_DMA_CACHE_ALIGN);
	end = ALIGN(paddr + size, P4_DMA_CACHE_ALIGN);
	raw_spin_lock_irqsave(&p4_cache_engine_lock, flags);
	while (start < end) {
		len = min_t(u32, end - start, SZ_64K - (start & (SZ_64K - 1)));
		ret = p4_cache_dma_sync(start, len, P4_CACHE_L1D, op);
		if (!ret)
			ret = p4_cache_dma_sync(start, len, P4_CACHE_L2, op);
		if (ret)
			break;
		start += len;
	}
	raw_spin_unlock_irqrestore(&p4_cache_engine_lock, flags);
	return ret;
}

static void p4_cache_dma_op(phys_addr_t paddr, size_t size, u32 op)
{
	int ret = p4_cache_range_op(paddr, size, op);

	if (ret)
		panic("P4 DMA cache sync failed: %pa, %zu bytes, op %#x", &paddr,
		      size, op);
}

static void p4_cache_dma_writeback(phys_addr_t paddr, size_t size)
{
	p4_cache_dma_op(paddr, size, P4_CACHE_SYNC_WB);
}

static void p4_cache_dma_invalidate(phys_addr_t paddr, size_t size)
{
	p4_cache_dma_op(paddr, size, P4_CACHE_SYNC_INV);
}

static void p4_cache_dma_writeback_invalidate(phys_addr_t paddr, size_t size)
{
	p4_cache_dma_writeback(paddr, size);
	p4_cache_dma_invalidate(paddr, size);
}

static const struct riscv_nonstd_cache_ops p4_dma_cache_ops __initconst = {
	.wback = p4_cache_dma_writeback,
	.inv = p4_cache_dma_invalidate,
	.wback_inv = p4_cache_dma_writeback_invalidate,
};

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
	int mem = fdt_path_offset(fdt, "/memory@48000000");
	int reg_len;
	const fdt32_t *reg = fdt_getprop(fdt, mem, "reg", &reg_len);
	u32 capacity;
	int fb, child;
	int matches = 0;
	int status_len;
	const char *status;
	u32 entry = p4_fdt_u32(fdt, chosen, "espressif,cache-sync-entry");
	u32 cam_inv = p4_fdt_u32(fdt, chosen,
				  "espressif,camera-invalidate-entry");
	u32 wb = 0;
	u32 rev = p4_fdt_u32(fdt, chosen, "espressif,chip-revision");

	/* The retained callback receives physical addresses under NOMMU.
	 * Trust one bounded DT RAM bank, not a fixed legacy 16 MiB ceiling. */
	if (!reg || reg_len != 2 * sizeof(*reg) ||
	    fdt_address_cells(fdt, 0) != 1 || fdt_size_cells(fdt, 0) != 1 ||
	    fdt32_to_cpu(reg[0]) != ESP32P4_RAM_START)
		panic("P4 invalid executable PSRAM bank");
	capacity = fdt32_to_cpu(reg[1]);
	if (capacity != SZ_16M && capacity != SZ_32M && capacity != SZ_64M)
		panic("P4 unsupported executable PSRAM capacity: %x", capacity);
	p4_exec_ram_end = ESP32P4_RAM_START + capacity;

	if (!IS_ENABLED(CONFIG_ESP32P4_NOMMU_XIP)) {
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
		if (esp32p4_retained_entry_valid(cam_inv))
			camera_invalidate = (p4_cache_fn)(unsigned long)cam_inv;
	}
	/* Default .data value is already v1 before the first possible trap.
	 * Only this retained revision103 handoff is reconstructed. Never switch
	 * to an unverified CSR bank before a later IRQ probe rejects the DT. */
	if (rev != 103)
		panic("P4 cache setup requires chip revision 1.3");
	esp32p4_clic_rev1 = 1;
	dma_cache_alignment = P4_DMA_CACHE_ALIGN;
	riscv_noncoherent_register_cache_ops(&p4_dma_cache_ops);
	riscv_noncoherent_supported();
}
SOC_EARLY_INIT_DECLARE(esp32p4_cache, "espressif,esp32p4", p4_cache_early_init);

int esp32p4_display_cache_writeback(u32 start, u32 size)
{
	unsigned long flags;
	int ret;

	if (!size)
		return 0;
	if (start < ESP32P4_FB_START || start >= ESP32P4_FB_END ||
	    size > ESP32P4_FB_END - start || size > ESP32P4_CACHE_CHUNK ||
	    ((start | size) & 63))
		return -EINVAL;
	if (IS_ENABLED(CONFIG_ESP32P4_NOMMU_XIP))
		return p4_cache_range_op(start, size, P4_CACHE_SYNC_WB);
	if (!display_writeback)
		return -ENODEV;
	raw_spin_lock_irqsave(&p4_cache_engine_lock, flags);
	ret = display_writeback(start, size);
	raw_spin_unlock_irqrestore(&p4_cache_engine_lock, flags);
	return ret ? -EIO : 0;
}
EXPORT_SYMBOL_GPL(esp32p4_display_cache_writeback);

int esp32p4_camera_cache_invalidate(u32 start, u32 size)
{
	unsigned long flags;
	int ret;

	if (!size)
		return 0;
	if (IS_ENABLED(CONFIG_ESP32P4_NOMMU_XIP))
		return p4_cache_range_op(start, size, P4_CACHE_SYNC_INV);
	if (start < ESP32P4_RAM_START || start >= ESP32P4_RAM_END ||
	    size > ESP32P4_RAM_END - start)
		return -EINVAL;
	if (!camera_invalidate)
		return -ENODEV;
	raw_spin_lock_irqsave(&p4_cache_engine_lock, flags);
	ret = camera_invalidate(start, size);
	raw_spin_unlock_irqrestore(&p4_cache_engine_lock, flags);
	return ret ? -EIO : 0;
}
EXPORT_SYMBOL_GPL(esp32p4_camera_cache_invalidate);

void esp32p4_cache_sync_for_exec(unsigned long start, unsigned long end)
{
	unsigned long flags;
	int ret;

	if (end <= start)
		return;
	if (start < ESP32P4_RAM_START || end > p4_exec_ram_end)
		panic("P4 executable cache range invalid: %lx..%lx", start, end);
	if (IS_ENABLED(CONFIG_ESP32P4_NOMMU_XIP)) {
		ret = p4_cache_range_op(start, end - start, P4_CACHE_SYNC_WB);
	} else {
		if (!exec_sync)
			panic("P4 retained executable cache callback missing");
		raw_spin_lock_irqsave(&p4_cache_engine_lock, flags);
		ret = exec_sync(start, end - start);
		raw_spin_unlock_irqrestore(&p4_cache_engine_lock, flags);
	}
	if (ret)
		panic("P4 executable cache sync failed: %lx..%lx", start, end);
	asm volatile("fence.i" ::: "memory");
}
EXPORT_SYMBOL_GPL(esp32p4_cache_sync_for_exec);

void esp32p4_cache_sync_all_for_exec(void)
{
	esp32p4_cache_sync_for_exec(ESP32P4_RAM_START, p4_exec_ram_end);
}
