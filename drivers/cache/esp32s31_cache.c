// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S31 external-memory cache maintenance
 *
 * The register programming follows ESP-IDF's ESP32-S31 cache LL and ROM
 * writeback workaround. In particular, writeback-class operations must be
 * issued twice on this SoC to avoid losing a synchronization request.
 */

#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/spinlock.h>

#include <asm/cacheflush.h>
#include <asm/csr.h>
#include <asm/dma-noncoherent.h>
#include <asm/fixmap.h>

#include <linux/soc/espressif/esp32s31-cache.h>

#define ESP32S31_CACHE_SYNC_CTRL		0x09c
#define ESP32S31_CACHE_SYNC_MAP		0x0a0
#define ESP32S31_CACHE_SYNC_ADDR		0x0a4
#define ESP32S31_CACHE_SYNC_SIZE		0x0a8

#define ESP32S31_CACHE_INVALIDATE	BIT(0)
#define ESP32S31_CACHE_WRITEBACK		BIT(2)
#define ESP32S31_CACHE_WB_INVALIDATE	BIT(3)
#define ESP32S31_CACHE_SYNC_DONE		BIT(4)

#define ESP32S31_CACHE_MAP_ICACHE0	BIT(0)
#define ESP32S31_CACHE_MAP_ICACHE1	BIT(1)
#define ESP32S31_CACHE_MAP_DCACHE	BIT(4)

#define ESP32S31_CACHE_LINE_SIZE		64U
#define ESP32S31_CACHE_SYNC_SIZE_MAX	GENMASK(27, 0)

static void __iomem *esp32s31_cache_base;
static u32 esp32s31_icache_map = ESP32S31_CACHE_MAP_ICACHE0;
static DEFINE_RAW_SPINLOCK(esp32s31_cache_lock);

static bool esp32s31_cache_ensure_base(void)
{
	if (likely(esp32s31_cache_base))
		return true;

	/*
	 * setup_vm() calls the page-table helpers with SATP disabled, before the
	 * bootstrap fixmap is usable.  Once SATP is enabled, the fixmap PTE built
	 * by setup_vm() is already visible because head.S wrote back all early
	 * tables.  This breaks the ioremap/page-table-writeback dependency cycle
	 * before the cache driver's early_initcall can run.
	 */
	if (!csr_read(CSR_SATP))
		return false;

	esp32s31_cache_base =
		(void __iomem *)__fix_to_virt(FIX_S31_CACHE);
	return true;
}

static void esp32s31_cache_wait_done(void)
{
	while (!(readl_relaxed(esp32s31_cache_base +
				ESP32S31_CACHE_SYNC_CTRL) &
		 ESP32S31_CACHE_SYNC_DONE))
		cpu_relax();
}

static void esp32s31_cache_issue(u32 map, u32 addr, u32 size, u32 op)
{
	writel_relaxed(map, esp32s31_cache_base + ESP32S31_CACHE_SYNC_MAP);
	writel_relaxed(addr, esp32s31_cache_base + ESP32S31_CACHE_SYNC_ADDR);
	writel_relaxed(size, esp32s31_cache_base + ESP32S31_CACHE_SYNC_SIZE);
	wmb();

	writel_relaxed(op, esp32s31_cache_base + ESP32S31_CACHE_SYNC_CTRL);
	esp32s31_cache_wait_done();

	/* ESP-IDF's S31 workaround repeats writeback-class sync requests. */
	if (op & (ESP32S31_CACHE_WRITEBACK |
		  ESP32S31_CACHE_WB_INVALIDATE)) {
		writel_relaxed(op, esp32s31_cache_base +
			       ESP32S31_CACHE_SYNC_CTRL);
		esp32s31_cache_wait_done();
	}

	mb();
}

static bool esp32s31_cache_align_range(phys_addr_t paddr, size_t size,
				       u32 *addr, u32 *len)
{
	u64 start, end;

	if (!size || paddr > U32_MAX)
		return false;

	start = (u64)paddr & ~(ESP32S31_CACHE_LINE_SIZE - 1);
	end = ALIGN((u64)paddr + size, ESP32S31_CACHE_LINE_SIZE);
	if (end <= start || end > (u64)U32_MAX + 1 ||
	    end - start > ESP32S31_CACHE_SYNC_SIZE_MAX)
		return false;

	*addr = start;
	*len = end - start;
	return true;
}

static void esp32s31_cache_range(phys_addr_t paddr, size_t size, u32 op)
{
	unsigned long flags;
	u32 addr, len;

	if (unlikely(!esp32s31_cache_ensure_base()) ||
	    !esp32s31_cache_align_range(paddr, size, &addr, &len))
		return;

	raw_spin_lock_irqsave(&esp32s31_cache_lock, flags);
	esp32s31_cache_issue(ESP32S31_CACHE_MAP_DCACHE, addr, len, op);
	raw_spin_unlock_irqrestore(&esp32s31_cache_lock, flags);
}

static void esp32s31_cache_dma_writeback(phys_addr_t paddr, size_t size)
{
	esp32s31_cache_range(paddr, size, ESP32S31_CACHE_WRITEBACK);
}

void esp32s31_cache_writeback(phys_addr_t paddr, size_t size)
{
	esp32s31_cache_range(paddr, size, ESP32S31_CACHE_WRITEBACK);
}
EXPORT_SYMBOL_GPL(esp32s31_cache_writeback);

void esp32s31_cache_invalidate(phys_addr_t paddr, size_t size)
{
	unsigned long flags;
	u32 addr, len;
	u32 icache_map;

	if (unlikely(!esp32s31_cache_ensure_base()) ||
	    !esp32s31_cache_align_range(paddr, size, &addr, &len))
		return;

	/* External Flash aliases are served by I-cache; PSRAM data uses D-cache.
	 * Both harts have a private I-cache: invalidate both on SMP so freshly
	 * loaded userspace code is visible no matter which hart executes it. */
	icache_map = IS_ENABLED(CONFIG_SMP) ?
		(ESP32S31_CACHE_MAP_ICACHE0 | ESP32S31_CACHE_MAP_ICACHE1) :
		esp32s31_icache_map;
	raw_spin_lock_irqsave(&esp32s31_cache_lock, flags);
	esp32s31_cache_issue(ESP32S31_CACHE_MAP_DCACHE | icache_map,
			     addr, len, ESP32S31_CACHE_INVALIDATE);
	raw_spin_unlock_irqrestore(&esp32s31_cache_lock, flags);
}
EXPORT_SYMBOL_GPL(esp32s31_cache_invalidate);

static void esp32s31_cache_dma_invalidate(phys_addr_t paddr, size_t size)
{
	esp32s31_cache_range(paddr, size, ESP32S31_CACHE_INVALIDATE);
}

static void esp32s31_cache_dma_wback_invalidate(phys_addr_t paddr, size_t size)
{
	esp32s31_cache_range(paddr, size, ESP32S31_CACHE_WB_INVALIDATE);
}

void esp32s31_cache_sync_for_exec(phys_addr_t paddr, size_t size)
{
	unsigned long flags;
	u32 addr, len;
	u32 icache_map;

	if (unlikely(!esp32s31_cache_ensure_base()) ||
	    !esp32s31_cache_align_range(paddr, size, &addr, &len))
		return;

	/* Both harts have a private I-cache; invalidate both on SMP so freshly
	 * mapped executable pages are coherent on either hart.  The D-cache is
	 * shared, so one writeback covers both. */
	icache_map = IS_ENABLED(CONFIG_SMP) ?
		(ESP32S31_CACHE_MAP_ICACHE0 | ESP32S31_CACHE_MAP_ICACHE1) :
		esp32s31_icache_map;
	raw_spin_lock_irqsave(&esp32s31_cache_lock, flags);
	esp32s31_cache_issue(ESP32S31_CACHE_MAP_DCACHE, addr, len,
			     ESP32S31_CACHE_WRITEBACK);
	esp32s31_cache_issue(icache_map, addr, len,
			     ESP32S31_CACHE_INVALIDATE);
	raw_spin_unlock_irqrestore(&esp32s31_cache_lock, flags);
}

static const struct riscv_nonstd_cache_ops esp32s31_cache_ops __initconst = {
	.wback = esp32s31_cache_dma_writeback,
	.inv = esp32s31_cache_dma_invalidate,
	.wback_inv = esp32s31_cache_dma_wback_invalidate,
};

static const struct of_device_id esp32s31_cache_ids[] __initconst = {
	{ .compatible = "espressif,esp32s31-cache" },
	{ }
};

static int __init esp32s31_cache_init(void)
{
	struct device_node *np;
	u32 icache_id = 0;

	np = of_find_matching_node(NULL, esp32s31_cache_ids);
	if (!np)
		return -ENODEV;

	esp32s31_cache_base = of_iomap(np, 0);
	if (!esp32s31_cache_base) {
		of_node_put(np);
		return -ENOMEM;
	}

	of_property_read_u32(np, "espressif,icache-id", &icache_id);
	if (icache_id > 1) {
		pr_warn("ESP32-S31 cache: invalid I-cache ID %u, using 0\n",
			icache_id);
		icache_id = 0;
	}
	esp32s31_icache_map = icache_id ? ESP32S31_CACHE_MAP_ICACHE1 :
					 ESP32S31_CACHE_MAP_ICACHE0;
	of_node_put(np);

	riscv_noncoherent_register_cache_ops(&esp32s31_cache_ops);
	pr_info("ESP32-S31 cache: 64-byte lines, I-cache %u, writeback workaround enabled\n",
		icache_id);
	return 0;
}
early_initcall(esp32s31_cache_init);
