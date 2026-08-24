// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S31 external-memory cache maintenance.
 *
 * Follow the official port's privilege boundary: S-mode never programs the
 * cache controller directly.  A vendor SBI ecall enters OpenSBI, which calls
 * the chip ROM Cache_* helpers in M-mode.  The local raw lock is the sole SMP
 * addition and serializes the shared cache engine across both harts.
 */

#define pr_fmt(fmt) "esp32s31-cache: " fmt

#include <linux/cacheflush.h>
#include <linux/init.h>
#include <linux/printk.h>
#include <linux/spinlock.h>

#include <asm/dma-noncoherent.h>
#include <asm/sbi.h>

#include <linux/soc/espressif/esp32s31-cache.h>

#define S31_SBI_CACHE_WBACK		0
#define S31_SBI_CACHE_INVAL		1
#define S31_SBI_CACHE_WBACK_INVAL	2
#define S31_SBI_ICACHE_SYNC		3
#define S31_SBI_ICACHE_SYNC_RANGE	4

static DEFINE_RAW_SPINLOCK(esp32s31_cache_lock);

static unsigned long esp32s31_cache_extid(void)
{
	static unsigned long extid;
	unsigned long id = READ_ONCE(extid);

	if (unlikely(!id)) {
		id = SBI_EXT_VENDOR_START +
			(sbi_get_mvendorid() &
			 (SBI_EXT_VENDOR_END - SBI_EXT_VENDOR_START));
		WRITE_ONCE(extid, id);
	}
	return id;
}

static struct sbiret esp32s31_cache_op(unsigned long func, phys_addr_t paddr,
				       size_t size)
{
	unsigned long flags;
	struct sbiret ret;

	raw_spin_lock_irqsave(&esp32s31_cache_lock, flags);
	ret = sbi_ecall(esp32s31_cache_extid(), func, (unsigned long)paddr,
			(unsigned long)size, 0, 0, 0, 0);
	raw_spin_unlock_irqrestore(&esp32s31_cache_lock, flags);
	return ret;
}

static void esp32s31_cache_dma_writeback(phys_addr_t paddr, size_t size)
{
	esp32s31_cache_op(S31_SBI_CACHE_WBACK, paddr, size);
}

static void esp32s31_cache_dma_invalidate(phys_addr_t paddr, size_t size)
{
	esp32s31_cache_op(S31_SBI_CACHE_INVAL, paddr, size);
}

static void esp32s31_cache_dma_wback_invalidate(phys_addr_t paddr,
					 size_t size)
{
	esp32s31_cache_op(S31_SBI_CACHE_WBACK_INVAL, paddr, size);
}

void esp32s31_cache_writeback(phys_addr_t paddr, size_t size)
{
	esp32s31_cache_op(S31_SBI_CACHE_WBACK, paddr, size);
}
EXPORT_SYMBOL_GPL(esp32s31_cache_writeback);

void esp32s31_cache_invalidate(phys_addr_t paddr, size_t size)
{
	/* Flash is read through the shared external D-cache by MTD and may also
	 * contain executable mappings.  Drop the data copy first, then both hart
	 * I-caches, so the operation that just completed is immediately visible
	 * without requiring a reboot.
	 */
	esp32s31_cache_op(S31_SBI_CACHE_INVAL, paddr, size);
	esp32s31_cache_op(S31_SBI_ICACHE_SYNC_RANGE, paddr, size);
}
EXPORT_SYMBOL_GPL(esp32s31_cache_invalidate);

void esp32s31_cache_sync_for_exec(phys_addr_t paddr, size_t size)
{
	esp32s31_cache_op(S31_SBI_ICACHE_SYNC_RANGE, paddr, size);
}

void esp32s31_cache_sync_all_for_exec(void)
{
	esp32s31_cache_op(S31_SBI_ICACHE_SYNC, 0, 0);
}

static const struct riscv_nonstd_cache_ops esp32s31_cache_ops __initconst = {
	.wback = esp32s31_cache_dma_writeback,
	.inv = esp32s31_cache_dma_invalidate,
	.wback_inv = esp32s31_cache_dma_wback_invalidate,
};

static int __init esp32s31_cache_init(void)
{
	struct sbiret ret;

	/* A non-zero mapped PSRAM range proves that the M-mode ROM call works;
	 * a zero-length probe would hide a bad OpenSBI/cache ABI until first DMA. */
	ret = esp32s31_cache_op(S31_SBI_CACHE_WBACK, 0x50000000UL, 64);
	if (ret.error) {
		pr_err("cache SBI ecall unavailable (err %ld); DMA will be incoherent\n",
		       ret.error);
		return -ENODEV;
	}

	riscv_noncoherent_register_cache_ops(&esp32s31_cache_ops);
	riscv_noncoherent_supported();
	pr_info("DMA and I-cache ops registered via vendor SBI ecall\n");
	return 0;
}
arch_initcall(esp32s31_cache_init);
