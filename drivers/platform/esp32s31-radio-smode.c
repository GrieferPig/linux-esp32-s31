// SPDX-License-Identifier: GPL-2.0
/* ESP32-S31 S-mode Wi-Fi/Bluetooth runtime. */

#include <linux/init.h>
#include <linux/bitmap.h>
#include <linux/completion.h>
#include <linux/cpu.h>
#include <linux/soc/espressif/esp32s31-pm.h>
#include <linux/if_ether.h>
#include <linux/ieee80211.h>
#include <linux/genalloc.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irqdomain.h>
#include <linux/irqdesc.h>
#include <linux/jiffies.h>
#include <linux/kallsyms.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/math64.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/percpu.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/thread_info.h>
#include <linux/timekeeping.h>
#include <linux/unaligned.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/esp32s31-radio.h>
#include <linux/irqchip/irq-esp32s31-clic.h>
#include <net/checksum.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci.h>
#include <asm/csr.h>
#include <asm/esp32s31_ext.h>
#include <asm/fpu.h>
#include <asm/irq_regs.h>
#include <asm/irq_stack.h>
#include <asm/sbi.h>

#include "esp32s31-radio-internal.h"
#include "esp32s31-radio-hci-frame.h"
#include "esp32s31-radio-xip.h"

u64 s31_linux_wall_time_seconds(void)
{
	return ktime_get_real_seconds();
}

#ifdef CONFIG_SMP
extern void esp32s31_irq_poll(void);
#endif

#define S31_ROM_BASE		0x2f800000UL
#define S31_ROM_CPU_FREQ	0x2f800040UL

typedef unsigned int (*s31_rom_cpu_freq_t)(void);

static bool s31_radio_disabled;

static unsigned int s31_radio_bt_hci_rx_slots = 8;
module_param_named(bt_hci_rx_slots, s31_radio_bt_hci_rx_slots, uint, 0444);
MODULE_PARM_DESC(bt_hci_rx_slots, "BT-only HCI RX ring slots: 8,16,32 (default 8)");
static unsigned int s31_radio_bt_hci_tx_slots = 4;
module_param_named(bt_hci_tx_slots, s31_radio_bt_hci_tx_slots, uint, 0444);
MODULE_PARM_DESC(bt_hci_tx_slots, "BT-only HCI TX ring slots: 4,8,16 (default 4)");

#ifndef MODULE
static int __init s31_radio_disable_setup(char *value)
{
	if (value && !strcmp(value, "off"))
		s31_radio_disabled = true;

	return 0;
}
early_param("esp32s31_radio", s31_radio_disable_setup);
#endif

bool esp32s31_radio_is_disabled(void)
{
	return s31_radio_disabled;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_is_disabled);

#ifdef CONFIG_ESP32S31_RADIO_BLOBS
/* The loader carves out 0x2f030000..0x2f072380 exclusively for Linux radio.
 * The linked blob data/BSS occupies the beginning. Keep the final 0xb80 bytes
 * for the synchronous-trap stack; descriptors begin at 0x2f072380. */
#define S31_RADIO_HEAP_END	0x2f071800UL
#define S31_RADIO_HEAP_BASE	0x2f030000UL
/* Wi-Fi-only SoftMAC can leave a 70 KiB aligned tail for a separate owner.
 * Keep the full heap for Bluetooth and other radio profiles. */
#define S31_RADIO_HEAP_WIFI_END	0x2f060000UL
#define S31_RADIO_EXC_STACK_TOP	0x2f072360UL
/* Idle SRAM above the DMA reservations (ROM download-mode buffers tail +
 * PRO-CPU startup stack), reclaimed as a second blob heap. */
#define S31_RADIO_HEAP2_BASE	0x2f078c00UL
#define S31_RADIO_HEAP2_END	0x2f07cfb0UL
#define S31_RADIO_HEAP2_SIZE	(S31_RADIO_HEAP2_END - S31_RADIO_HEAP2_BASE)
/* Low SRAM may contain OpenSBI runtime stacks and heap. Reclaim only the
 * tail above the running firmware's reported exclusive end. */
#define S31_RADIO_HEAP_LOW_BASE	0x2f018000UL
#define S31_RADIO_HEAP_LOW_END	0x2f030000UL

extern char __s31_radio_data_start[];
extern char __s31_radio_data_end[];
extern char __s31_radio_data_load[];
extern char __s31_radio_bss_start[];
extern char __s31_radio_bss_end[];
extern char __s31_radio_static_end[];

static struct gen_pool *s31_radio_heap_pool;
/* Resource generations close publish-before-sleep races. The CPU0 submitter
 * publishes one interest; producers notify only actual resource returns. */
static atomic_t s31_tx_credit_sequence[S31_TX_CREDIT_COUNT];
static atomic_t s31_tx_credit_wait = ATOMIC_INIT(0);

/* Pinned ROM esf_buf_recycle increments class1's recycle count under the
 * Wi-Fi mutex, before its final unlock. Storage is the fixed payload arena,
 * not an allocation or a ROM function-table replacement. */
static const u32 *s31_tx_pool_returns;
static u32 s31_tx_pool_wait_generation;

unsigned long s31_radio_tx_pool_bind(void *counter)
{
 if (counter && !s31_radio_fw_errno_slot_valid((const int *)counter))
  return 0;
 smp_store_release(&s31_tx_pool_returns, counter);
 return 1;
}

static u32 s31_radio_tx_pool_generation(void)
{

 const u32 *counter = smp_load_acquire(&s31_tx_pool_returns);

 return counter ? READ_ONCE(*counter) : 0;
}

static bool s31_radio_tx_pool_changed(void)
{
 return s31_radio_tx_pool_generation() != READ_ONCE(s31_tx_pool_wait_generation);
}
static struct task_struct *s31_tx_attempt_owner;

/* Published only during the exceptional queue-capacity wait. The waiter
 * clears this under tx_lock before task/queue cleanup or owner retirement. */
static struct task_struct *s31_tx_repair_waiter;

#define S31_RADIO_HEAP_ALLOC_ORDER 4

static void *s31_radio_heap_linux;
static void *s31_radio_heap_low_linux;
static unsigned long s31_radio_heap_low_base;
static size_t s31_radio_heap_low_size;
static void *s31_radio_heap2_linux;
static unsigned long s31_radio_heap_base;
static unsigned long s31_radio_heap_end;
static unsigned long s31_radio_heap_tail_base;
static size_t s31_radio_heap_size;

static void s31_radio_heap_credit_return(void)
{
 /* A genpool extent or reclaimable cached block is reusable now. Do not
  * turn this submitter's failed-allocation cleanup into a retry spin. */
 if (in_interrupt() || current != READ_ONCE(s31_tx_attempt_owner))
  s31_radio_tx_credit_return(S31_TX_CREDIT_BUFFER);
}

static void s31_radio_heap_free(unsigned long base, size_t size,
                                unsigned long caller)
{
 gen_pool_free(s31_radio_heap_pool, base, size);
 s31_radio_heap_credit_return();
}

static void *s31_radio_heap_alloc_algo(size_t size, genpool_algo_t algo, void *data)
{
 unsigned long base = gen_pool_alloc_algo(s31_radio_heap_pool, size,
   algo ? algo : s31_radio_heap_pool->algo,
   algo ? data : s31_radio_heap_pool->data);
 return (void *)base;
}

/* Function 6 returns the complete running OpenSBI footprint, including
 * hart stacks, scratch and its heap; ELF section bounds exclude these. An
 * older firmware without this query must keep all low SRAM reserved. */
static unsigned long s31_radio_low_heap_start(void)
{
	struct sbiret bounds = sbi_ecall(0x09000003, 6, 0, 0, 0, 0, 0, 0);
	if (bounds.error || bounds.value < 0x2f010000UL ||
	    bounds.value > S31_RADIO_HEAP_LOW_END) {
		pr_warn("esp32s31-radio: OpenSBI SRAM bounds unavailable; low heap disabled\n");
		return S31_RADIO_HEAP_LOW_END;
	}

	return ALIGN(max(bounds.value, S31_RADIO_HEAP_LOW_BASE), 64);
}

static void *s31_radio_sram_linux_alias(const void *ptr)
{
	unsigned long p = (unsigned long)ptr;

	if (p >= s31_radio_heap_low_base && p < S31_RADIO_HEAP_LOW_END)
		return s31_radio_heap_low_linux + (p - s31_radio_heap_low_base);
	if (p >= S31_RADIO_HEAP2_BASE && p < S31_RADIO_HEAP2_END)
		return s31_radio_heap2_linux + (p - S31_RADIO_HEAP2_BASE);
	return s31_radio_heap_linux + (p - s31_radio_heap_base);
}

struct s31_idf_alloc_header {
	u32 magic;
	size_t size;
	size_t total;
	void *base;
};

/* The bitmap allocator aligns the payload rather than overallocating padding.
 * Cache admission and accounting must use that same reserved prefix. */
#define S31_IDF_ALLOC_PREFIX ALIGN(sizeof(struct s31_idf_alloc_header), \
				  1UL << S31_RADIO_HEAP_ALLOC_ORDER)

#define S31_IDF_ALLOC_MAGIC	0x5333414c

/* Live-allocation histogram for the internal SRAM heap.  Bucketed by request
 * size so a periodic report can show exactly which buffer classes consume the
 * bounded pool (RX esf_buf ~1848, TX ~1600, task stacks, queues, ...). */
/* Bounded cache for the measured ROM dynamic RX allocation class.
 * Cached blocks retain their gen_pool reservation and count as heap used.
 * IRQ-safe list operations never call the allocator while holding the lock.
 * General allocation pressure drains the cache before a final retry. */
#define S31_IDF_RX_CACHE_SIZE 1848U
#define S31_IDF_RX_CACHE_MAX 64U
static const unsigned int s31_idf_rx_cache_limit = 48;
static const bool s31_idf_rx_cache = true;

static DEFINE_RAW_SPINLOCK(s31_idf_rx_cache_lock);
static void *s31_idf_rx_cache_blocks[S31_IDF_RX_CACHE_MAX];
static unsigned int s31_idf_rx_cache_count;

static void *s31_idf_rx_cache_pop(void)
{
 unsigned long flags;
 void *ptr = NULL;
 raw_spin_lock_irqsave(&s31_idf_rx_cache_lock, flags);
 if (s31_idf_rx_cache_count) {
  ptr = s31_idf_rx_cache_blocks[--s31_idf_rx_cache_count];
  s31_idf_rx_cache_blocks[s31_idf_rx_cache_count] = NULL;
 }
 raw_spin_unlock_irqrestore(&s31_idf_rx_cache_lock, flags);
 return ptr;
}
static bool s31_idf_rx_cache_push(void *ptr)
{
 unsigned long flags;
 bool cached = false;
 raw_spin_lock_irqsave(&s31_idf_rx_cache_lock, flags);
 if (s31_idf_rx_cache_count < min(READ_ONCE(s31_idf_rx_cache_limit), S31_IDF_RX_CACHE_MAX)) {
  s31_idf_rx_cache_blocks[s31_idf_rx_cache_count++] = ptr;
  cached = true;
 }
 raw_spin_unlock_irqrestore(&s31_idf_rx_cache_lock, flags);
 return cached;
}
static void s31_idf_rx_cache_flush(void)
{
 void *ptr;
 while ((ptr = s31_idf_rx_cache_pop())) {
  struct s31_idf_alloc_header *h = ptr - sizeof(*h);
  s31_radio_heap_free((unsigned long)h->base, h->total, _RET_IP_);

 }
}

/* Cached RX extents remain reclaimable by every heap consumer, including
 * raw aligned native stacks. Retry only after returning actual extents;
 * no timer, polling loop or alternate allocation domain is involved. */
static void *s31_radio_heap_alloc_reclaim(size_t size, genpool_algo_t algo,
                                        void *data)
{

 void *base;

 base = s31_radio_heap_alloc_algo(size, algo, data);

 if (!base && READ_ONCE(s31_idf_rx_cache_count)) {
  s31_idf_rx_cache_flush();
  base = s31_radio_heap_alloc_algo(size, algo, data);
 }

 return base;
}

struct s31_heap_alignment {
	size_t alignment;
	size_t prefix;
};

static unsigned long s31_heap_first_fit_payload(unsigned long *map,
		unsigned long bits, unsigned long start, unsigned int count,
		void *opaque, struct gen_pool *pool, unsigned long chunk_base)
{
	const struct s31_heap_alignment *data = opaque;
	unsigned int order = pool->min_alloc_order;
	unsigned long mask = (data->alignment >> order) - 1;
	unsigned long offset = ((chunk_base + data->prefix) &
				(data->alignment - 1)) >> order;

	return bitmap_find_next_zero_area_off(map, bits, start, count, mask, offset);
}

/* Default unaligned radio allocations need a suitable extent, not a full
 * smallest-hole census. Keep genpool's atomic bitmap reservation/return and
 * the existing payload-aligned policy for stronger alignment requests. */
static unsigned long s31_heap_first_fit(unsigned long *map, unsigned long bits,
        unsigned long start, unsigned int count, void *data,
        struct gen_pool *pool, unsigned long chunk_base)
{
 return bitmap_find_next_zero_area(map, bits, start, count, 0);
}

static void *s31_idf_alloc(size_t size, size_t alignment, bool zero, u32 caps)
{

	struct s31_idf_alloc_header *header;
	void *base, *ptr;
	size_t total, unit = 1UL << s31_radio_heap_pool->min_alloc_order;
	struct s31_heap_alignment data = {
		.prefix = S31_IDF_ALLOC_PREFIX,
	};

	(void)caps;

	if (alignment < sizeof(void *))
		alignment = sizeof(void *);
	if (!is_power_of_2(alignment) || size > SIZE_MAX - data.prefix)
		return NULL;
	data.alignment = max(alignment, unit);
	/* Align the payload through the bitmap search, reserving only header
	 * and data bytes. An 8 KiB stack must not reserve another 8 KiB merely
	 * as alignment padding; skipped extents remain available to the pool. */
	total = size + data.prefix;
 if (s31_idf_rx_cache && size == S31_IDF_RX_CACHE_SIZE &&
     alignment == sizeof(void *) && !caps) {
  ptr = s31_idf_rx_cache_pop();
  if (ptr) {
   header = ptr - sizeof(*header);
   base = header->base;

   goto allocated;
  }

 }
	base = s31_radio_heap_alloc_reclaim(total,
		data.alignment > unit ? s31_heap_first_fit_payload : NULL, &data);
	if (!base)
		return NULL;

	ptr = (u8 *)base + data.prefix;
	header = ptr - sizeof(*header);
allocated:
	header->magic = S31_IDF_ALLOC_MAGIC;
	header->size = size;
	header->total = total;
	header->base = base;
	if (zero)
		memset(ptr, 0, size);

	return ptr;
}

void *__wrap_heap_caps_malloc(size_t size, u32 caps)
{
 return s31_idf_alloc(size, sizeof(void *), false, caps);
}

void __wrap_heap_caps_free(void *ptr)
{

	struct s31_idf_alloc_header *header;

	if (!ptr)
		return;
	header = ptr - sizeof(*header);
	if (WARN_ON_ONCE(header->magic != S31_IDF_ALLOC_MAGIC))
		return;
	header->magic = 0;

 /* Reuse only the exact current layout. Stronger-aligned allocations also
  * satisfy a default alignment request: every block is internal SRAM and
  * the original base/extent stay reserved until cache flush. */
 if (s31_idf_rx_cache && header->size == S31_IDF_RX_CACHE_SIZE &&
     header->total == header->size + S31_IDF_ALLOC_PREFIX &&
     ptr == (u8 *)header->base + S31_IDF_ALLOC_PREFIX &&
     s31_idf_rx_cache_push(ptr)) {
  /* The extent stays reserved, but all allocation classes can reclaim it.
   * Publish that resource return after dropping the cache lock, otherwise
   * a BUFFER waiter can sleep despite newly reclaimable memory. */
  s31_radio_heap_credit_return();
  return;
 }
	s31_radio_heap_free((unsigned long)header->base, header->total, _RET_IP_);
}

void *__wrap_heap_caps_calloc(size_t n, size_t size, u32 caps)
{
	if (n && size > SIZE_MAX / n)
		return NULL;
	return s31_idf_alloc(n * size, sizeof(void *), true, caps);
}

void *__wrap_heap_caps_realloc(void *ptr, size_t size, u32 caps)
{

	struct s31_idf_alloc_header *header;
	void *new_ptr;

	if (!ptr)
		return __wrap_heap_caps_malloc(size, caps);
	if (!size) {
		__wrap_heap_caps_free(ptr);
		return NULL;
	}
	header = ptr - sizeof(*header);
	if (header->magic != S31_IDF_ALLOC_MAGIC)
		return NULL;
	new_ptr = __wrap_heap_caps_malloc(size, caps);
	if (new_ptr) {
		memcpy(new_ptr, ptr, min(size, header->size));
		__wrap_heap_caps_free(ptr);
	}
	return new_ptr;
}

void *__wrap_heap_caps_aligned_alloc(size_t alignment, size_t size, u32 caps)
{
	return s31_idf_alloc(size, alignment, false, caps);
}

void *__wrap_heap_caps_aligned_calloc(size_t alignment, size_t n,
				       size_t size, u32 caps)
{
	if (n && size > SIZE_MAX / n)
		return NULL;
	return s31_idf_alloc(n * size, alignment, true, caps);
}

void *__wrap_heap_caps_malloc_default(size_t size)
{
	return __wrap_heap_caps_malloc(size, 0);
}

void *__wrap_heap_caps_realloc_default(void *ptr, size_t size)
{
	return __wrap_heap_caps_realloc(ptr, size, 0);
}

void *__wrap_heap_caps_malloc_prefer(size_t size, size_t count, ...)
{
	return __wrap_heap_caps_malloc(size, 0);
}

size_t __wrap_heap_caps_get_free_size(u32 caps)
{
	(void)caps;
	return gen_pool_avail(s31_radio_heap_pool);
}

static atomic_t s31_wifi_rx_direct_pending = ATOMIC_INIT(0);

/* Every allocation that can be touched while the PHY/controller changes the
 * external-memory/cache state must live in the internal-SRAM radio heap.
 * The Linux bridge objects (task bookkeeping, queue/event sync state) are
 * allocated here for the same reason as the payload's own stacks and TCBs. */
#define S31_SRAM_ALLOC_MAGIC 0x53334d31U

struct s31_sram_alloc_header {
	u32 magic;
	size_t size;
};

void *s31_radio_sram_alloc(size_t size)
{
	struct s31_sram_alloc_header *header;

	/* Match s31_idf_alloc: a wrapped size would under-allocate the pool
	 * block and overflow it on use. All current callers pass bounded
	 * sizes; refuse absurd ones instead of trusting that. */
	if (size > SIZE_MAX - sizeof(*header))
		return NULL;
	header = s31_radio_heap_alloc_reclaim(size + sizeof(*header), NULL, NULL);
	if (!header)
		return NULL;
	header->magic = S31_SRAM_ALLOC_MAGIC;
	header->size = size;
	return header + 1;
}

void s31_radio_sram_free(void *ptr)
{
	struct s31_sram_alloc_header *header;

	if (!ptr)
		return;
	header = ptr - sizeof(*header);
	if (WARN_ON_ONCE(header->magic != S31_SRAM_ALLOC_MAGIC))
		return;
	header->magic = 0;
	s31_radio_heap_free((unsigned long)header,
		      header->size + sizeof(*header), _RET_IP_);
}

/* Stack metadata lives in the kernel's lease object, not in a header that
 * consumes the preceding aligned extent. Exact pages can be adjacent and
 * can use a free extent of precisely THREAD_SIZE bytes. */
void *s31_radio_sram_stack_alloc(u32 size)
{
 struct s31_heap_alignment alignment = { .prefix = 0, .alignment = THREAD_SIZE };
 void *base;

 if (size != THREAD_SIZE)
  return NULL;
 base = s31_radio_heap_alloc_reclaim(size, s31_heap_first_fit_payload, &alignment);
 return base;
}

void s31_radio_sram_stack_free(void *base, u32 size)
{
 if (!base)
  return;
 if (size != THREAD_SIZE || !IS_ALIGNED((unsigned long)base, THREAD_SIZE) ||
     !gen_pool_has_addr(s31_radio_heap_pool, (unsigned long)base, size))
  panic("s31 radio: invalid native stack extent");
 s31_radio_heap_free((unsigned long)base, size, _RET_IP_);
}

static void s31_radio_heap_reset_chunk(struct gen_pool *pool,
				       struct gen_pool_chunk *chunk, void *data)
{
	unsigned long bytes = chunk->end_addr - chunk->start_addr + 1;
	unsigned long bits = bytes >> pool->min_alloc_order;

	bitmap_zero(chunk->bits, bits);
	atomic_long_set(&chunk->avail, bytes);
}

static void s31_radio_heap_destroy(void)
{
	size_t leaked;

	if (!s31_radio_heap_pool)
		return;

 s31_idf_rx_cache_flush();
	s31_linux_sync_owners_retire_all();
	leaked = gen_pool_size(s31_radio_heap_pool) -
		 gen_pool_avail(s31_radio_heap_pool);
	if (leaked) {
		/* ESP-IDF keeps process-lifetime queue, timer and controller objects
		 * after its public deinit calls.  At this point every compatibility
		 * task, worker and IRQ has been stopped, and this pool owns a dedicated
		 * SRAM carveout, so reset the allocation bitmap as the hardware-domain
		 * equivalent of releasing the complete firmware address space. */
		pr_warn("esp32s31-radio: reclaiming %zu bytes retained by the stopped payload\n",
			leaked);
		gen_pool_for_each_chunk(s31_radio_heap_pool,
					s31_radio_heap_reset_chunk, NULL);
	}
	gen_pool_destroy(s31_radio_heap_pool);
	s31_radio_heap_pool = NULL;

}

void _interrupt_handler(void)
{
}

/* Keep the IDF module-clock contract intact.  esp_phy_enable() enables the
 * common PHY/calibration clocks, while the Wi-Fi adapter separately enables
 * WIFI_MAC/APB/BB and their PLL dependencies through these hooks.  Leaving
 * them as no-ops makes the ROM PHY wait forever for WDEVTXQ_BLOCK to idle. */
extern void s31_radio_wifi_clock_enable(void);
extern void s31_radio_wifi_clock_disable(void);

void wifi_module_enable(void)
{
	s31_radio_wifi_clock_enable();
}

void wifi_module_disable(void)
{
	s31_radio_wifi_clock_disable();
}

int esp_deep_sleep_register_phy_hook(void *callback)
{
	(void)callback;
	return 0;
}

void touch_hal_prepare_deep_sleep(void)
{
}

/* Define the modem peripheral register base addresses that the ESP-IDF
 * modem clock HAL expects.  The peripherals.ld linker script provides
 * these for the blob's relocatable link. */
asm(".globl MODEM_SYSCON\n"
    ".set MODEM_SYSCON, 0x20109C00\n"
    ".globl MODEM_LPCON\n"
    ".set MODEM_LPCON, 0x2010F000\n");

/* Temperature sensor HAL stubs — the blob's RF calibration reads chip
 * temperature for compensation but the HAL functions are undefined in
 * the blob (they live in the soc_temperature component).  Return safe
 * defaults so the calibration path doesn't crash on garbage pointers. */
int temperature_sensor_hal_init(void *hal, void *cfg) { (void)hal; (void)cfg; return 0; }
float temperature_sensor_hal_get_degree(void *hal, int *degree)
{
	(void)hal;
	if (degree && (unsigned long)degree >= PAGE_SIZE)
		*degree = 25;
	return 25.0f;
}
void temperature_sensor_hal_i2c_saradc_reg_backup(void) { }
void temperature_sensor_hal_i2c_saradc_reg_restore(void) { }

const unsigned long _mtvt_table[48] = {
	[0 ... 47] = (unsigned long)_interrupt_handler,
};

void *intr_handler_get(int int_no)
{
	return NULL;
}

u32 __wrap_esp_log_timestamp(void)
{
	/* Linux deliberately starts jiffies near UINT_MAX to expose wrap bugs.
	 * IDF logs expect milliseconds since boot, not raw Linux jiffies. */
	return div_u64(ktime_get_mono_fast_ns(), NSEC_PER_MSEC);
}

/*
 * IDF's interrupt allocator normally accepts ISR code only from its IRAM and
 * mask-ROM address windows.  The radio objects are built into Linux .text in
 * this configuration, which is resident executable memory as well.  Keep the
 * allocator's IRAM-safety check, but teach it about the new owner/address
 * space through the link-time wrapper.
 */
bool __wrap_esp_intr_ptr_in_isr_region(void *ptr)
{
	unsigned long addr = (unsigned long)ptr;

	/* ESP32-S31 executes the XIP kernel from its permanent 0xc0000000
	 * mapping; core_kernel_text() does not include that architecture-specific
	 * alias.  The radio payload's IRAM-attributed handlers live there. */
	return esp32s31_executable_address(addr);
}

struct s31_shared_irq_source {
	int source, virq;
	unsigned int members;
	u8 level;
	bool allocated, retiring, hw_enabled, iram;
};

struct s31_idf_irq_registration {
	int source;
	unsigned int logical_intr;
	int flags;
	/* Firmware level and last successfully applied policy byte. IDF's
	 * allocation flags are a mask of acceptable levels, not a number. */
	u8 prio_level, applied_prio_ctl;
	u32 enable_generation;
	bool priority_applied;
	int priority_error;
	void (*handler)(void *);
	void *arg;
	int virq;
	struct esp32s31_radio_domain *domain;
	bool install_pending;
	bool free_pending;
	bool enabled;
	bool hw_enabled;
	/* Shared actions do not own independent hardware disable depths. */
	struct s31_shared_irq_source *shared_source;
	bool shared_enabled;
	/* Set once an absent handler has been masked and joined, so an
	 * already-quiescent slot is not re-joined on every pass. Cleared when a
	 * handler is (re)installed. */
	bool absent_synced;
	bool allocated;

	atomic_t callback_pending;

};

#define S31_RADIO_IRQ_SLOTS	7
static struct s31_idf_irq_registration s31_idf_irqs[S31_RADIO_IRQ_SLOTS];
static struct s31_shared_irq_source s31_shared_irqs[S31_RADIO_IRQ_SLOTS];
/* Resource metadata only, never a vendor execution gate. Mutators and
 * final slot reclamation are atomic against admitted CPU0 task preemption.
 * Linux IRQ joins/allocation and callbacks always run outside this lock. */
static DEFINE_RAW_SPINLOCK(s31_irq_registry_lock);

static atomic_t s31_radio_pending_isrs = ATOMIC_INIT(0);
static atomic_t s31_radio_immediate_pending = ATOMIC_INIT(0);

/* Dedicated CLIC dispatch is mandatory for every owned radio source. */
static struct esp_clic_priority_lease *s31_irq_priority_lease;

static void s31_radio_irq_priority_restore(void)
{
	if (!s31_irq_priority_lease)
		return;
	if (esp_clic_priority_release(s31_irq_priority_lease))
		panic("esp32s31-radio: lost IRQ priority lease");
	s31_irq_priority_lease = NULL;
}

static const uint s31_irq_background_level = 1;
/* The pinned IDF accepts levels by BIT(level), choosing the lowest
 * available priority when several are permitted. Our dedicated mapping has
 * no vector-sharing pressure, so use the lowest requested level, or1 for
 * the IDF default low/medium mask. Keep one background band below it.
 * Refuse unrepresentable levels rather than silently wrapping/collapsing
 * relative radio priorities into the three implemented CLIC bits. */
static unsigned int s31_irq_level_from_flags(unsigned int flags)
{
 unsigned int levels=flags & GENMASK(7,1);
 return levels ? __ffs(levels) : 1;
}
static int s31_irq_level_control(unsigned int level, u8 *ctl)
{
 if (!level || level>7 || (level+s31_irq_background_level>7))
  return -EINVAL;
 *ctl=ESP32S31_CLIC_PRIO(level+s31_irq_background_level);
 return 0;
}
static void s31_radio_apply_irq_priority(struct s31_idf_irq_registration *r)
{
 u8 ctl;
 int ret;
 if (!r->virq) return;
 ret=s31_irq_level_control(READ_ONCE(r->prio_level),&ctl);
 if (!ret && r->priority_applied && r->applied_prio_ctl==ctl) return;
 /* Background IRQ affinity must not decide a radio owner's placement.
  * Both exclusive and shared actions have acquired explicit CPU0 ownership
  * through required direct registration before priority is applied. */
 if (!ret && !r->priority_applied) {
  struct irq_data *data=irq_get_irq_data(r->virq);
  ret=irq_set_affinity(r->virq,cpumask_of(0));
  if (!ret && (!data || !cpumask_equal(
       irq_data_get_effective_affinity_mask(data),cpumask_of(0))))
   ret=-EAGAIN;
 }
 if (!ret) ret=esp_clic_irq_set_priority(r->virq,ctl);
 WRITE_ONCE(r->priority_error,ret);
 if (ret) {
  pr_warn_ratelimited("esp32s31-radio: source%d IRQ priority refused: %d\n",r->source,ret);
  return;
 }
 WRITE_ONCE(r->applied_prio_ctl,ctl);
 WRITE_ONCE(r->priority_applied,true);
}

enum s31_radio_command_type {
	S31_RADIO_COMMAND_WIFI_GET_MAC,
	S31_RADIO_COMMAND_WIFI_SCAN,
	S31_RADIO_COMMAND_WIFI_CONNECT,
	S31_RADIO_COMMAND_WIFI_DISCONNECT,
	S31_RADIO_COMMAND_WIFI_CONTROL,
	S31_RADIO_COMMAND_BT_ENABLE,
	S31_RADIO_COMMAND_BT_DISABLE,
	S31_RADIO_COMMAND_SHUTDOWN,
};

/* H4 type plus Linux HCI_MAX_FRAME_SIZE (1028-byte ACL header/payload). */
#define S31_HCI_FRAME_SIZE	ESP32S31_RADIO_HCI_FRAME_MAX
#define S31_HCI_RX_SLOTS_DEFAULT	8
#define S31_HCI_TX_SLOTS_DEFAULT	4
static unsigned int s31_hci_rx_slots = S31_HCI_RX_SLOTS_DEFAULT;
static unsigned int s31_hci_tx_slots = S31_HCI_TX_SLOTS_DEFAULT;

struct s31_hci_frame {
	u16 length;
	u8 data[S31_HCI_FRAME_SIZE];
};

static struct s31_hci_frame *s31_hci_rx;
static struct s31_hci_frame *s31_hci_tx;
static unsigned int s31_hci_rx_head;
static unsigned int s31_hci_rx_tail;
static unsigned int s31_hci_tx_head;
static unsigned int s31_hci_tx_tail;
/* Written/consumed only by the single radio executor, outside init. */
static bool s31_hci_tx_room_ready;
static DEFINE_SPINLOCK(s31_hci_lock);
/* Protected by s31_hci_lock; reopening requires fresh runtime rings. */
static bool s31_hci_rx_accepting;
static const struct esp32s31_radio_hci_ops *s31_hci_ops;
static void *s31_hci_context;

/* First-full values survive printk ring wrap. Only the failure path writes;
 * no payload pointers or bytes outlive the controller callback. */

/* Frames actually handed to the controller by the HCI TX drain. */

/* Ring writers and health snapshots already use s31_hci_lock. Update each
 * telemetry pair there rather than taking two generic atomic64 locks on RV32.
 * These counters are independent of controller credits and admission. */

static const struct esp32s31_radio_wifi_ops *s31_wifi_ops;
static void *s31_wifi_context;
static DEFINE_MUTEX(s31_wifi_registration_lock);
static u16 s31_wifi_scan_count;
static int s31_wifi_scan_status;
static bool s31_wifi_scan_inflight;
static bool s31_wifi_scan_ready;

#define S31_WIFI_FRAME_SIZE	ESP32S31_RADIO_WIFI_FRAME_MAX
/* Copy out of the closed driver's RX buffer while its callback owns it.  The
 * closed driver does not permit the esf_buf lifetime to extend past callback
 * return, so staging data must live in internal SRAM and be returned at once.
 * In combo mode, 32 4144-byte buffers plus TX/HCI rings exceed the 192 KiB
 * first heap before the non-contiguous SRAM pools are attached. */
#define S31_WIFI_RX_SLOTS	8
/* Only a bounded metadata/source lease batch leaves mac80211. SoftMAC
 * payload bytes stay in skb storage until the final vendor buffer is owned. */
#define S31_WIFI_TX_SLOTS	32

struct s31_wifi_frame {
	u16 length;
	u8 interface;
	bool cancel_requested;
	struct sk_buff *skb; /* Independent source lease, released after attempt. */
	union {
		struct s31_softmac_tx_view view;
		u8 data[sizeof(struct s31_softmac_tx_view)];
	};
};

struct s31_wifi_rx_desc {
	u16 length;
};

enum s31_tx_timing_kind {
	S31_TX_OTHER,
	S31_TX_IPV4,
	S31_TX_TCP_ACK,
	S31_TX_DHCP,
	S31_TX_ARP,
	S31_TX_IPV6,
	S31_TX_KIND_COUNT,
};

#define S31_TX_TIMING_SLOTS 1
#define S31_RADIO_WORKER_NICE (-10)

static const char * const s31_tx_kind_name[S31_TX_KIND_COUNT] = {
	[S31_TX_OTHER] = "other",
	[S31_TX_IPV4] = "ipv4",
	[S31_TX_TCP_ACK] = "tcp-ack",
	[S31_TX_DHCP] = "dhcp",
	[S31_TX_ARP] = "arp",
	[S31_TX_IPV6] = "ipv6",
};

static u8 s31_radio_tx_kind(const u8 *frame, u16 length)
{
	u16 ethertype;
	u8 ihl;
	u8 tcp_hlen;
	u8 tcp_flags;
	u16 ip_length;
	u16 sport, dport;

	if (!frame || length < ETH_HLEN)
		return S31_TX_OTHER;
	ethertype = ((u16)frame[12] << 8) | frame[13];
	if (ethertype == ETH_P_ARP)
		return S31_TX_ARP;
	if (ethertype == ETH_P_IPV6)
		return S31_TX_IPV6;
	if (ethertype != ETH_P_IP || length < ETH_HLEN + 20)
		return S31_TX_OTHER;
	ihl = (frame[ETH_HLEN] & 0x0f) * 4;
	if (ihl < 20 || length < ETH_HLEN + ihl)
		return S31_TX_IPV4;
	if (frame[ETH_HLEN + 9] == 6 &&
	    length >= ETH_HLEN + ihl + 20) {
		ip_length = ((u16)frame[ETH_HLEN + 2] << 8) |
			frame[ETH_HLEN + 3];
		tcp_hlen = (frame[ETH_HLEN + ihl + 12] >> 4) * 4;
		tcp_flags = frame[ETH_HLEN + ihl + 13];
		if (tcp_hlen >= 20 && ip_length == ihl + tcp_hlen &&
		    (tcp_flags & 0x10) && !(tcp_flags & 0x07))
			return S31_TX_TCP_ACK;
		return S31_TX_IPV4;
	}
	if (length < ETH_HLEN + ihl + 8 ||
	    frame[ETH_HLEN + 9] != 17)
		return S31_TX_IPV4;
	sport = ((u16)frame[ETH_HLEN + ihl] << 8) |
		frame[ETH_HLEN + ihl + 1];
	dport = ((u16)frame[ETH_HLEN + ihl + 2] << 8) |
		frame[ETH_HLEN + ihl + 3];
	if ((sport == 67 && dport == 68) || (sport == 68 && dport == 67))
		return S31_TX_DHCP;
	return S31_TX_IPV4;
}

/* Private CPU0 Linux control worker coalesces finite command/completion work.
 * The managed vendor task owns all timer and data-plane service deadlines.
 * s31_radio_runner identifies a control pass with borrowed init_mm. */
static struct kthread_worker *s31_radio_worker;
static struct kthread_work s31_radio_work;
/* The producer lock protects the private Linux worker and dispatcher waitq.
 * STOPPING refuses new business but permits one terminal SHUTDOWN wake.
 * OFF is published only after task, work and IRQ producers are joined. */
static enum { S31_Q_OFF, S31_Q_RUNNING, S31_Q_STOPPING } s31_radio_q_state = S31_Q_OFF;
static DEFINE_SPINLOCK(s31_radio_q_lock);
static struct task_struct *s31_radio_runner;

#include "esp32s31-radio-service.inc"
static DECLARE_WAIT_QUEUE_HEAD(s31_radio_bt_disable_waitq);
/* All feature combinations share the managed vendor dispatcher. */
static void s31_radio_wake(void);
static void s31_radio_queue(void);

static bool s31_radio_dispatch_irq_notify(bool deferred);
static void __s31_radio_queue_force(void);
static void s31_radio_control_kick(void);

#define S31_DEFER_REASON_COUNT (S31_DIRECT_ISR_DEFER_NESTED + 1)

static struct s31_wifi_rx_desc *s31_wifi_rx;
static u8 *s31_wifi_rx_data;
static struct s31_wifi_frame *s31_wifi_tx;
static unsigned int s31_wifi_rx_head;
static unsigned int s31_wifi_rx_tail;
static unsigned int s31_wifi_tx_head;
static unsigned int s31_wifi_tx_tail;
/* Source-space interest is published under the existing ring lock by a
 * refused producer. Only the CPU0 consumer publishes/consumes the ready hint.
 * No frontend lock/callback is taken while holding the source-ring lock. */
static bool s31_wifi_tx_space_waiting;
static bool s31_wifi_tx_space_ready;
static unsigned int s31_wifi_tx_wait_generation;

static DEFINE_SPINLOCK(s31_wifi_tx_lock);
static DECLARE_WAIT_QUEUE_HEAD(s31_wifi_rx_space_waitq);
static struct esp32s31_radio_wifi_connect_params *s31_wifi_connect_params;
static struct s31_wifi_scan_params *s31_wifi_scan_params;
static u16 *s31_wifi_disconnect_reason;
static struct s31_wifi_control *s31_wifi_control_buffer;
static DEFINE_MUTEX(s31_wifi_control_mutex);
static DECLARE_COMPLETION(s31_wifi_control_done);
static int s31_wifi_control_result;
static bool s31_wifi_control_poisoned;

static void *s31_wifi_control_sync;
static struct task_struct *s31_wifi_control_thread;
static DECLARE_COMPLETION(s31_wifi_control_ready);
static bool s31_wifi_control_started;
static atomic_t s31_wifi_control_pending = ATOMIC_INIT(0);

static u8 s31_wifi_event_bssid[6];
static u8 s31_wifi_event_channel;
static u16 s31_wifi_event_reason;

static int s31_wifi_connect_status;
static bool s31_wifi_connected_ready;
static bool s31_wifi_disconnected_ready;

struct s31_radio_command {
	struct list_head node;
	struct completion done;
	enum s31_radio_command_type type;
	int result;
	u8 vendor_mac[ETH_ALEN];
	union {
		u8 *mac;
		const struct esp32s31_radio_wifi_connect_params *connect;
		const struct s31_wifi_scan_params *scan;
		u16 disconnect_reason;

	};
};

static LIST_HEAD(s31_radio_commands);
static DEFINE_SPINLOCK(s31_radio_command_lock);
/* One CPU0 Linux IRQ-stack lease, independent of every radio task stack.
 * The raw allocation is retained until all actions and the pointer lease join. */
static void *s31_native_irq_allocation;
static void *s31_native_irq_stack;
static bool s31_native_irq_installed;
/* A dedicated managed vendor dispatcher; the Linux worker stays independent. */

/* Caller-owned command storage lasts through its uninterruptible completion.
 * In-flight storage cannot be released by cancellation until the dispatcher
 * has joined. All queue publication uses this leaf lock; no vendor body or
 * complete() executes with the lock held. */
static DEFINE_SPINLOCK(s31_dispatch_command_lock);
static LIST_HEAD(s31_dispatch_commands);
static LIST_HEAD(s31_dispatch_completions);
static struct s31_radio_command *s31_dispatch_inflight;

static void *s31_dispatch_sync;
static bool s31_dispatch_started;

#define S31_RADIO_SERVICE_STACK_SIZE 8192U
/* Next RTOS tick edge for the serialized runner's tick accounting. */
/* Mask-ROM identity verified and radio-init task created once per runtime
 * on the first pass, where the runner has borrowed init_mm (the low mapping
 * exists only there).  Creation runs on a pool kworker exactly like the
 * retired dedicated thread did; never in module-load process context. */
static bool s31_radio_hw_checked;
/* Set when first-pass init fails terminally (ROM or radio-init task):
 * the runner parks permanently, matching the retired thread which exited
 * on the same failures.  Reset in runtime_init for reload. */
static bool s31_radio_init_dead;
static u32 s31_radio_features;
static atomic_t s31_radio_state = ATOMIC_INIT(ESP32S31_RADIO_OFFLINE);

void s31_radio_managed_task_failed(void)
{
 /* Fatal-exit monitor cannot sleep, service radio or acquire the gate. */
 atomic_set(&s31_radio_state, ESP32S31_RADIO_FAILED);
}

static int s31_wifi_init_result = -EINPROGRESS;
static int s31_bt_init_result = -EINPROGRESS;
static int s31_bt_enable_result = -EINPROGRESS;
static bool s31_bt_enable_inflight;
static DEFINE_SPINLOCK(s31_bt_enable_lock);
static DECLARE_WAIT_QUEUE_HEAD(s31_radio_bt_enable_waitq);
static int s31_bt_disable_result = -EINPROGRESS;
static int s31_shutdown_result = -EINPROGRESS;
static DECLARE_COMPLETION(s31_shutdown_done);

#define S31_IDF_INTR_DISABLED	BIT(11) /* ESP_INTR_FLAG_INTRDISABLED */
#define S31_IDF_INTR_SHARED	BIT(8)
#define S31_IDF_INTR_EDGE	BIT(9)
#define S31_IDF_INTR_IRAM	BIT(10)

static void *s31_radio_init_args(void)
{
	u32 args = READ_ONCE(s31_radio_features);

	return (void *)(uintptr_t)args;
}

bool s31_radio_is_bt_only(void)
{
	return READ_ONCE(s31_radio_features) == S31_RADIO_FEATURE_BLUETOOTH;
}

/* Startup/cache calibration remains fenced. A span records its own fence,
 * so readiness changing during a live span never drops an unowned guard. */
bool s31_radio_managed_preempt_ready(void)
{
 return atomic_read(&s31_radio_state) == ESP32S31_RADIO_READY &&
        (!(READ_ONCE(s31_radio_features) & S31_RADIO_FEATURE_WIFI) ||
         READ_ONCE(s31_wifi_init_result) == 0) &&
        (!(READ_ONCE(s31_radio_features) & S31_RADIO_FEATURE_BLUETOOTH) ||
         READ_ONCE(s31_bt_enable_result) == 0);
}

/* The lifecycle probe exercises VHCI pools as well as task ownership. */

bool s31_radio_control_bootstrap_quiescent(void)
{
 int i;

 if ((atomic_read(&s31_radio_state) != ESP32S31_RADIO_STARTING) ||
     current != READ_ONCE(s31_radio_runner) ||
     READ_ONCE(s31_radio_hw_checked) || READ_ONCE(s31_dispatch_started) ||
     atomic_read(&s31_radio_pending_isrs))
  return false;
 for (i=0; i<S31_RADIO_IRQ_SLOTS; i++)
  if (READ_ONCE(s31_idf_irqs[i].allocated) ||
      READ_ONCE(s31_idf_irqs[i].handler) ||
      READ_ONCE(s31_idf_irqs[i].virq))
   return false;
 return true;
}

static bool s31_radio_bt_batching_active(void)
{
	return s31_radio_features == S31_RADIO_FEATURE_BLUETOOTH &&
	       READ_ONCE(s31_bt_enable_result) == 0;
}

bool s31_radio_payload_uses_fp(void)
{
	/* The pinned BTDM object region is integer-only.  Wi-Fi payload objects
	 * still contain floating-point instructions and require full interrupted
	 * FP state preservation around a nested native ISR. */
	return READ_ONCE(s31_radio_features) & S31_RADIO_FEATURE_WIFI;
}

void s31_radio_report_wifi_init(int result)
{
	WRITE_ONCE(s31_wifi_init_result, result);
	s31_radio_control_kick();
}

void s31_radio_report_bt_init(int result)
{
	WRITE_ONCE(s31_bt_init_result, result);
	s31_radio_control_kick();
}

void s31_radio_report_bt_enable(int result)
{
	unsigned long flags;

	spin_lock_irqsave(&s31_bt_enable_lock, flags);
	WRITE_ONCE(s31_bt_enable_result, result);
	s31_bt_enable_inflight = false;
	spin_unlock_irqrestore(&s31_bt_enable_lock, flags);
	wake_up_all(&s31_radio_bt_enable_waitq);
}

void s31_radio_report_bt_disable(int result)
{
	WRITE_ONCE(s31_bt_disable_result, result);
	if (!result)
		WRITE_ONCE(s31_bt_enable_result, -EHOSTDOWN);
	s31_radio_wake();
	wake_up(&s31_radio_bt_disable_waitq);
}

void s31_radio_report_shutdown(int result)
{
	WRITE_ONCE(s31_shutdown_result, result);
	complete(&s31_shutdown_done);
}

int __wrap_esp_intr_alloc(int source, int flags, void (*handler)(void *),
			  void *arg, void **ret_handle)
{
	struct s31_idf_irq_registration *registration = NULL;
	struct s31_shared_irq_source *shared = NULL;
	int i;
	unsigned int level=s31_irq_level_from_flags(flags);
	u8 ctl;
	unsigned long irqflags;

	if (s31_linux_control_bootstrap_active())
		return -EAGAIN;

	if (!handler || source < 0 || source > 177 ||
	    ((flags & S31_IDF_INTR_SHARED) && (flags & S31_IDF_INTR_EDGE)) ||
	    s31_irq_level_control(level,&ctl))
		return -EINVAL;
	raw_spin_lock_irqsave(&s31_irq_registry_lock,irqflags);
	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++) {
		if (s31_shared_irqs[i].allocated && s31_shared_irqs[i].source == source &&
		    s31_shared_irqs[i].retiring) {
			raw_spin_unlock_irqrestore(&s31_irq_registry_lock,irqflags);
			return -EBUSY;
		}
	}
	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++) {
		struct s31_idf_irq_registration *r = &s31_idf_irqs[i];

		if (!r->allocated) {
			if (!registration) registration = r;
			continue;
		}
		if (r->source != source) continue;
		/* A non-shared source has one owner, including while retiring.
		 * Never discover a duplicate only after creating its Linux mapping. */
		if (!(flags & S31_IDF_INTR_SHARED) || !r->shared_source) {
			raw_spin_unlock_irqrestore(&s31_irq_registry_lock,irqflags);
			return -EBUSY;
		}
		shared = r->shared_source;
	}
	if (shared) {
		unsigned int levels = flags & GENMASK(7, 1);

		if (!levels) levels = BIT(1); /* SDK shared default. */
		if (shared->retiring || !(levels & BIT(shared->level)) ||
		    shared->iram != !!(flags & S31_IDF_INTR_IRAM)) {
			raw_spin_unlock_irqrestore(&s31_irq_registry_lock,irqflags);
			return -EINVAL;
		}
		level = shared->level;
	}
	if (!registration) {
		raw_spin_unlock_irqrestore(&s31_irq_registry_lock,irqflags);
		return -ENOMEM;
	}
	if ((flags & S31_IDF_INTR_SHARED) && !shared) {
		for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++)
			if (!s31_shared_irqs[i].allocated) {
				shared = &s31_shared_irqs[i];
				break;
			}
		if (!shared) {
			raw_spin_unlock_irqrestore(&s31_irq_registry_lock,irqflags);
			return -ENOMEM;
		}
		shared->allocated = true;
		shared->source = source;
		shared->level = level;
		shared->iram = !!(flags & S31_IDF_INTR_IRAM);
	}
	if (shared) shared->members++;
	registration->shared_source = shared;
	registration->source = source;
	registration->logical_intr = UINT_MAX;
	registration->flags = flags;
	registration->prio_level = level;
	registration->priority_applied = false;
	registration->priority_error = 0;
	registration->handler = handler;
	registration->arg = arg;
	registration->install_pending = true;
	registration->enabled = !(flags & S31_IDF_INTR_DISABLED);
	registration->allocated = true;

	atomic_set(&registration->callback_pending, 0);

	if (ret_handle)
		*ret_handle = registration;
	raw_spin_unlock_irqrestore(&s31_irq_registry_lock,irqflags);
	s31_radio_control_kick();

	return 0;
}

static struct s31_idf_irq_registration *s31_radio_irq_handle(void *handle)
{
	int i;

	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++)
		if (handle == &s31_idf_irqs[i] && s31_idf_irqs[i].allocated &&
            !s31_idf_irqs[i].free_pending)
			return &s31_idf_irqs[i];
	return NULL;
}

static int s31_radio_irq_set_enabled(void *handle, bool enabled, bool freeing,
                                     bool conditional, u32 *generation)
{
 struct s31_idf_irq_registration *registration;
 unsigned long flags;
 raw_spin_lock_irqsave(&s31_irq_registry_lock,flags);
 registration=s31_radio_irq_handle(handle);
 if (!registration || (conditional && registration->enable_generation!=*generation)) {
  raw_spin_unlock_irqrestore(&s31_irq_registry_lock,flags);
  return conditional ? -EAGAIN : -EINVAL;
 }
 WRITE_ONCE(registration->enabled,enabled);
 registration->enable_generation++;
 if (generation) *generation=registration->enable_generation;
 if (enabled && atomic_read(&registration->callback_pending))
  atomic_set(&s31_radio_pending_isrs,1);
 /* Always let the control owner reclaim a slot. An uninstalled mapping
  * may already be in creation while this task preempts that owner. */
 if (freeing) WRITE_ONCE(registration->free_pending,true);
 raw_spin_unlock_irqrestore(&s31_irq_registry_lock,flags);
 s31_radio_wake();
 s31_radio_control_kick();
 return 0;
}
int __wrap_esp_intr_enable(void *handle)
{
 return s31_radio_irq_set_enabled(handle,true,false,false,NULL);
}
int __wrap_esp_intr_disable(void *handle)
{
 return s31_radio_irq_set_enabled(handle,false,false,false,NULL);
}
int __wrap_esp_intr_free(void *handle)
{
 return s31_radio_irq_set_enabled(handle,false,true,false,NULL);
}

/*
 * The Wi-Fi OS adapter normally routes its two MAC sources to local CLIC1
 * and then calls mask-ROM helpers which program the M-mode CLIC window at
 * 0x10801000.  Linux runs the blob in S-mode, so translate that legacy
 * logical interrupt into ordinary external CLIC slots owned by Linux.  The
 * actual matrix routing and CLIC programming is deferred until the worker is
 * outside the blob gate, just like esp_intr_alloc().
 */
void s31_radio_wifi_intr_configure(u32 source, u32 logical_intr, u32 priority)
{
	struct s31_idf_irq_registration *registration = NULL;
	int i;

	u8 ctl;
	unsigned long irqflags;
	bool enabled;
	unsigned int level=priority ?: 1;

	if (s31_irq_level_control(level,&ctl)) {
		pr_err("esp32s31-radio: Wi-Fi source%u IRQ level%u cannot fit policy band\n",source,level);
		return;
	}
	raw_spin_lock_irqsave(&s31_irq_registry_lock,irqflags);
	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++) {
		if (s31_idf_irqs[i].allocated &&
		    s31_idf_irqs[i].source == source) {
			registration = &s31_idf_irqs[i];
			break;
		}
		if (!registration && !s31_idf_irqs[i].allocated)
			registration = &s31_idf_irqs[i];
	}
	if (!registration || registration->free_pending || registration->shared_source) {
		raw_spin_unlock_irqrestore(&s31_irq_registry_lock,irqflags);
		pr_err("esp32s31-radio: no CLIC slot for Wi-Fi source %u\n",
		       source);
		return;
	}
	if (!registration->allocated) {
		memset(registration, 0, sizeof(*registration));
		registration->source = source;
		registration->allocated = true;

		atomic_set(&registration->callback_pending, 0);

	}
	/* The closed Wi-Fi driver may reserve a source before its OS adapter
	 * supplies the legacy local interrupt number.  Merge both halves of the
	 * registration instead of leaving logical_intr at UINT_MAX. */
	registration->logical_intr = logical_intr;
	WRITE_ONCE(registration->prio_level,level);
	enabled=registration->enabled;
	raw_spin_unlock_irqrestore(&s31_irq_registry_lock,irqflags);
	/* A reused mapping must apply a changed request even without a new ISR. */
	s31_radio_control_kick();

}

void s31_radio_wifi_intr_set_isr(u32 logical_intr, void (*handler)(void *),
				 void *arg)
{
	int i;
	unsigned long irqflags;

	raw_spin_lock_irqsave(&s31_irq_registry_lock,irqflags);
	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++) {
		struct s31_idf_irq_registration *registration = &s31_idf_irqs[i];

		if (!registration->allocated || registration->free_pending ||
		    registration->logical_intr != logical_intr)
			continue;
		registration->handler = handler;
		registration->arg = arg;
		registration->install_pending = handler != NULL && !registration->virq;
	}
	raw_spin_unlock_irqrestore(&s31_irq_registry_lock,irqflags);
	s31_radio_control_kick();
}

void s31_radio_wifi_intr_mask(u32 mask, bool enable)
{
	int i;
	unsigned long irqflags;

	raw_spin_lock_irqsave(&s31_irq_registry_lock,irqflags);
	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++) {
		struct s31_idf_irq_registration *registration = &s31_idf_irqs[i];

		if (!registration->allocated || registration->free_pending || registration->logical_intr >= 32 ||
		    !(mask & BIT(registration->logical_intr)))
			continue;
		WRITE_ONCE(registration->enabled, enable);
		registration->enable_generation++;
		if (enable && atomic_read(&registration->callback_pending))
			atomic_set(&s31_radio_pending_isrs,1);

	}
	raw_spin_unlock_irqrestore(&s31_irq_registry_lock,irqflags);
	s31_radio_wake();
	s31_radio_control_kick();
}

/* One explicit Linux disable depth per shared physical source. Callback
 * claims and reconciliation are CPU0 IRQ-fenced; no registry lock surrounds
 * a vendor handler or a Linux IRQ operation. */
static void s31_radio_shared_irq_mask(struct s31_shared_irq_source *source)
{
	if (source->virq && READ_ONCE(source->hw_enabled)) {
		WRITE_ONCE(source->hw_enabled, false);
		disable_irq_nosync(source->virq);
	}
}

static void s31_radio_shared_irq_reconcile(struct s31_shared_irq_source *source)
{
	unsigned long flags;
	bool active = false, pending = false, enable;
	int i;

	preempt_disable();
	local_irq_save(flags);
	if (!source->virq) goto out;
	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++) {
		struct s31_idf_irq_registration *r = &s31_idf_irqs[i];

		if (READ_ONCE(r->shared_source) != source || !READ_ONCE(r->allocated) ||
		    READ_ONCE(r->virq) != source->virq || !READ_ONCE(r->domain) ||
		    READ_ONCE(r->install_pending) || READ_ONCE(r->free_pending) ||
		    !READ_ONCE(r->handler) || !READ_ONCE(r->enabled))
			continue;
		active = true;
		pending |= !!atomic_read(&r->callback_pending);
	}
	enable = active && !pending && !READ_ONCE(source->retiring);
	if (enable && !READ_ONCE(source->hw_enabled)) {
		WRITE_ONCE(source->hw_enabled, true);
		enable_irq(source->virq);
	} else if (!enable) {
		s31_radio_shared_irq_mask(source);
	}
out:
	local_irq_restore(flags);
	preempt_enable();
}

static void s31_radio_irq_rearm(struct s31_idf_irq_registration *r)
{
	int virq = READ_ONCE(r->virq);

	if (r->shared_source) {
		s31_radio_shared_irq_reconcile(r->shared_source);
	} else if (virq && READ_ONCE(r->enabled) && !READ_ONCE(r->hw_enabled)) {
		WRITE_ONCE(r->hw_enabled, true);
		enable_irq(virq);

	}
}

/*
 * A hardirq can arrive while the current blob owner has published an unsafe
 * task-side point.  The hardirq then leaves the device IRQ explicitly disabled
 * and records one pending closed callback.  Do not wait for the radio worker:
 * it cannot enter the blob while this task owns the gate.  Instead, drain the
 * callback as soon as the owner reaches a safe boundary, on the already
 * installed payload context and stack.
 *
 * callback_pending is the ownership handoff between this path and the worker.
 * Local IRQ masking prevents the same level source from being re-entered while
 * its closed handler acknowledges the hardware.  Calls made from inside a
 * closed ISR are rejected to avoid recursive ISR execution through an unlock.
 */
bool s31_radio_blob_run_pending_isrs(void)
{
	unsigned long flags;
	bool handled = false;
	int i;

	if (!atomic_read(&s31_radio_pending_isrs) ||
	    !s31_linux_blob_held_by_current() ||
	    false ||
	    !s31_linux_closed_isr_context_safe() ||
	    READ_ONCE(s31_rtos_isr_depth))
		return false;

	/* The callback wrapper disables FS, matching IDF ISR semantics. */
	preempt_disable();
	local_irq_save(flags);

	atomic_set(&s31_radio_pending_isrs, 0);
	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++) {
		struct s31_idf_irq_registration *registration =
			&s31_idf_irqs[i];

		if (!READ_ONCE(registration->handler) || !READ_ONCE(registration->enabled) ||
            READ_ONCE(registration->free_pending) ||
		    !atomic_xchg(&registration->callback_pending, 0))
			continue;

		s31_rtos_isr_depth++;
		if (!s31_linux_closed_isr_call(registration->handler, registration->arg)) {
			s31_rtos_isr_depth--;
			/* A deeper dispatcher prologue can consume the last safe
			 * SRAM margin after the first check. Retain the IRQ-disable
			 * handoff and retry on a later safe boundary; never panic or
			 * enable a callback whose claimed handler did not run. */
			atomic_set(&registration->callback_pending, 1);
			atomic_set(&s31_radio_pending_isrs, 1);
			break;
		}
		s31_rtos_isr_depth--;

		handled = true;

		/* Balance the explicit disable depth installed by the fallback
		 * hardirq only after the closed ISR has acknowledged the device.  If
		 * the ISR logically disabled its source, retain that disable depth. */
		s31_radio_irq_rearm(registration);
	}

	local_irq_restore(flags);
	preempt_enable();
	return handled;
}

/* One non-sleeping callback on the worker's installed SRAM continuation.
 * Match owner-drain containment and rearm only after acknowledgement. */
static bool s31_radio_worker_irq_service(struct s31_idf_irq_registration *r)
{
 unsigned long flags;

 bool handled;

 /* CPU0 control retirement can preempt this dispatcher between a pending
  * claim and the handler load. Fence before either: no zeroed/reused
  * registration can slip into the claimed callback span. */
 preempt_disable();
 local_irq_save(flags);
 if (!READ_ONCE(r->handler) || !READ_ONCE(r->enabled) || READ_ONCE(r->free_pending) ||
     !atomic_xchg(&r->callback_pending,0)) {
  local_irq_restore(flags);
  preempt_enable();
  return false;
 }

 s31_rtos_isr_depth++;
 handled=s31_linux_closed_isr_call(r->handler,r->arg);
 s31_rtos_isr_depth--;
 if (!handled) {
  atomic_set(&r->callback_pending,1);
  atomic_set(&s31_radio_pending_isrs,1);

 } else {

  s31_radio_irq_rearm(r);
 }

 local_irq_restore(flags);
 preempt_enable();
 return handled;
}

/*
 * A level IRQ is already masked by handle_level_irq() while this action runs.
 * The independent SRAM IRQ context admits any legal interrupted CPU0
 * context and runs the closed ISR now. It acknowledges the MAC/DMA source;
 * generic IRQ exit unmasks the CLIC slot. Otherwise an explicit disable
 * depth holds the source until its qualified deferred consumer runs it.
 */
static irqreturn_t s31_radio_hardirq(int irq, void *data)
{
	struct s31_idf_irq_registration *registration = data;

	int direct_result;

 if (!READ_ONCE(registration->handler) || !READ_ONCE(registration->enabled) ||
     READ_ONCE(registration->free_pending)) {
  if (!registration->shared_source && READ_ONCE(registration->hw_enabled)) {
   disable_irq_nosync(irq);
   WRITE_ONCE(registration->hw_enabled, false);
  }
  return IRQ_HANDLED;
 }

	direct_result = s31_linux_blob_run_direct_isr(registration->handler,
						      registration->arg);

	if (direct_result == S31_DIRECT_ISR_HANDLED) {

		atomic_set(&registration->callback_pending, 0);
		/* The closed ISR may have logically masked its source.  In that case
		 * retain an explicit disable depth across handle_level_irq()'s exit;
		 * otherwise its hardware acknowledgement is enough and generic IRQ
		 * completion immediately unmasks the source. */
		if (!registration->shared_source && !READ_ONCE(registration->enabled) &&
		    READ_ONCE(registration->hw_enabled)) {
			disable_irq_nosync(irq);
			WRITE_ONCE(registration->hw_enabled, false);
		}
	} else {
		if (registration->shared_source)
			s31_radio_shared_irq_mask(registration->shared_source);
		else {
			disable_irq_nosync(irq);
			WRITE_ONCE(registration->hw_enabled, false);
		}

		atomic_set(&registration->callback_pending, 1);
		atomic_set(&s31_radio_pending_isrs, 1);
	}

 /* Successful handlers already published their actual RX/control edges.
  * The managed dispatcher owns timer service independently. A deferred
  * handler needs that dispatcher, not an extra Linux control-worker hop. */
 if (s31_radio_dispatch_irq_notify(direct_result != S31_DIRECT_ISR_HANDLED))
  return IRQ_HANDLED;
	if (!s31_radio_bt_batching_active() ||
	    direct_result != S31_DIRECT_ISR_HANDLED)
		s31_radio_queue();
	return IRQ_HANDLED;
}

/* Linux mapping ownership is source-wide. Never discard a mapping while a
 * different action remains attached (including a failed duplicate request). */
static bool s31_radio_irq_mapping_busy(unsigned int virq)
{
	struct irq_data *data = irq_get_irq_data(virq);
	struct irq_desc *desc;
	unsigned long flags;
	bool busy;

	if (!data) return false;
	desc = irq_data_to_desc(data);
	raw_spin_lock_irqsave(&desc->lock, flags);
	busy = !!desc->action;
	raw_spin_unlock_irqrestore(&desc->lock, flags);
	return busy;
}

static void s31_radio_irq_dispose_unused(unsigned int virq)
{
	if (!s31_radio_irq_mapping_busy(virq)) irq_dispose_mapping(virq);
}

static void s31_radio_irq_install_failed(struct s31_idf_irq_registration *r,
				       int error, u32 generation)
{
	unsigned long flags;

	/* Sleepable installation may race newer enable/disable intent. Stop
	 * retrying this failed generation, but preserve a newer explicit retry. */
	raw_spin_lock_irqsave(&s31_irq_registry_lock, flags);
	r->priority_error = error;
	r->priority_applied = false;
	if (r->enable_generation == generation)
		r->enabled = false;
	r->install_pending = !r->free_pending && r->enabled && r->handler;
	r->shared_enabled = false;
	raw_spin_unlock_irqrestore(&s31_irq_registry_lock, flags);
}

static irqreturn_t s31_radio_shared_hardirq(int irq, void *opaque)
{
	struct s31_shared_irq_source *source = opaque;
	int i;

	/* One exclusive Linux action fans out this runtime's logical handlers.
	 * The physical source uses dedicated CLIC dispatch like every radio IRQ. */
	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++) {
		struct s31_idf_irq_registration *r = &s31_idf_irqs[i];

		if (READ_ONCE(r->shared_source) == source && READ_ONCE(r->allocated) &&
		    READ_ONCE(r->virq) == irq && READ_ONCE(r->domain) &&
		    !READ_ONCE(r->install_pending) && !READ_ONCE(r->free_pending) &&
		    READ_ONCE(r->enabled) && READ_ONCE(r->handler))
			s31_radio_hardirq(irq, r);
	}
	s31_radio_shared_irq_reconcile(source);
	return IRQ_HANDLED;
}

static void s31_radio_sync_shared_irq(struct s31_idf_irq_registration *r)
{
	struct s31_shared_irq_source *source = r->shared_source;
	struct device_node *node;
	struct irq_fwspec spec = {};
	unsigned long flags;
	bool last;
	u32 generation = READ_ONCE(r->enable_generation);
	int virq, ret;

	if (READ_ONCE(r->free_pending)) {
		/* Admission is already closed. All callback paths are CPU0-fenced;
		 * join the Linux source action before ending this member's lifetime. */
		if (source->virq) synchronize_irq(source->virq);
		atomic_set(&r->callback_pending, 0);
		s31_linux_domain_irq_put(r->domain);
		raw_spin_lock_irqsave(&s31_irq_registry_lock, flags);
		last = --source->members == 0;
		if (last) source->retiring = true;
		memset(r, 0, sizeof(*r));
		raw_spin_unlock_irqrestore(&s31_irq_registry_lock, flags);
		if (last) {
			if (source->virq) {
				s31_radio_shared_irq_mask(source);
				esp_clic_direct_unregister(source->virq, source);
				free_irq(source->virq, source);
				s31_radio_irq_dispose_unused(source->virq);
			}
			raw_spin_lock_irqsave(&s31_irq_registry_lock, flags);
			memset(source, 0, sizeof(*source));
			raw_spin_unlock_irqrestore(&s31_irq_registry_lock, flags);
		} else {
			s31_radio_shared_irq_reconcile(source);
		}
		return;
	}

	if (!r->virq && READ_ONCE(r->handler) &&
	    (READ_ONCE(r->install_pending) || READ_ONCE(r->enabled))) {
		ret = s31_linux_domain_irq_get(&r->domain);
		if (ret) {
			s31_radio_irq_install_failed(r, ret, generation);
			return;
		}
		if (!source->virq) {
			node = of_find_compatible_node(NULL, NULL, "espressif,esp32s31-intmtx");
			if (!node) { ret = -ENODEV; goto failed; }
			spec.fwnode = of_node_to_fwnode(node);
			spec.param_count = 2;
			spec.param[0] = source->source;
			spec.param[1] = IRQ_TYPE_LEVEL_HIGH;
			virq = irq_create_fwspec_mapping(&spec);
			of_node_put(node);
			if (!virq) { ret = -ENXIO; goto failed; }
			/* Joining an unrelated Linux action is outside this source
			 * owner's scope. In particular, never dispose its mapping. */
			if (s31_radio_irq_mapping_busy(virq)) { ret = -EBUSY; goto failed; }
			source->virq = virq;
			r->virq = virq;
			r->priority_applied = false;
			/* Acquire the disabled action before changing hardware/flags;
			 * genirq may refuse a racing unrelated owner after our check. */
			r->install_pending = false;
			source->hw_enabled = false;
			ret = request_irq(virq, s31_radio_shared_hardirq,
					  IRQF_NO_AUTOEN | IRQF_NOBALANCING,
					  "esp32s31-radio-shared", source);
			if (ret) goto discard;
			ret = esp_clic_direct_register(virq, source);
			if (ret) {
				free_irq(virq, source);
				goto discard;
			}
			irq_set_status_flags(virq, IRQ_DISABLE_UNLAZY);
			s31_radio_apply_irq_priority(r);
			if (r->priority_error) {
				ret = r->priority_error;
				esp_clic_direct_unregister(virq, source);
				free_irq(virq, source);
				goto discard;
			}

		} else {
			/* The existing source action/mapping survives member admission. */
			r->virq = source->virq;
			s31_radio_apply_irq_priority(r);
			if (r->priority_error) {
				ret = r->priority_error;
				goto failed;
			}
			r->install_pending = false;
		}
	}
	r->shared_enabled = READ_ONCE(r->enabled) && !!READ_ONCE(r->handler);
	s31_radio_shared_irq_reconcile(source);
	return;
discard:
	source->hw_enabled = false;
	source->virq = 0;
	s31_radio_irq_dispose_unused(virq);
failed:
	r->virq = 0;
	s31_linux_domain_irq_put(r->domain);
	r->domain = NULL;
	/* Do not spin an RT control worker after a terminal installation error.
	 * A later explicit enable can retry; free still retires this owner. */
	s31_radio_irq_install_failed(r, ret, generation);
	s31_radio_shared_irq_reconcile(source);
	pr_err("esp32s31-radio: shared source%d installation failed: %d\n", source->source, ret);
}

static void s31_radio_sync_one_irq(struct s31_idf_irq_registration *registration)
{
	struct device_node *intmtx_node;
	struct irq_data *irqd;
	irq_hw_number_t clic_slot = 0;
	struct irq_fwspec fwspec = { };
	u32 generation = READ_ONCE(registration->enable_generation);
	int virq, ret;

	if (registration->shared_source) {
		s31_radio_sync_shared_irq(registration);
		return;
	}
	if (registration->free_pending) {
  if (registration->virq) {
   if (registration->hw_enabled) {
    disable_irq(registration->virq);
    registration->hw_enabled = false;
   }
   esp_clic_direct_unregister(registration->virq, registration);
   free_irq(registration->virq, registration);
   s31_radio_irq_dispose_unused(registration->virq);
  }
  s31_linux_domain_irq_put(registration->domain);
  {
   unsigned long flags;
   raw_spin_lock_irqsave(&s31_irq_registry_lock,flags);
   memset(registration,0,sizeof(*registration));
   raw_spin_unlock_irqrestore(&s31_irq_registry_lock,flags);
  }
		return;
	}

	/* Wi-Fi may remove its ISR without freeing the reusable mapping. Join
	 * the disabled action and cancel its deferred callback before treating
	 * the source as idle. An absent handler cannot consume pending work.
	 * Join only on the transition to absent: once the source is masked and
	 * joined it is quiescent, so re-joining it on every pass is wasted.
	 * callback_pending is still cleared every pass so a late publication
	 * cannot wedge the pass predicate. */
	if (!READ_ONCE(registration->handler)) {
		bool just_disabled = false;

		if (registration->virq) {
			if (READ_ONCE(registration->hw_enabled)) {
				WRITE_ONCE(registration->hw_enabled, false);
				disable_irq_nosync(registration->virq);
				just_disabled = true;
			}
			if (!registration->absent_synced || just_disabled)
				synchronize_irq(registration->virq);
		}
		registration->absent_synced = true;
		atomic_set(&registration->callback_pending, 0);
		return;
	}
	registration->absent_synced = false;

	if (registration->install_pending && !registration->virq) {
		/* Follow the official irqchip hierarchy: the radio owns an ETS
		 * peripheral source, not a CLIC slot.  INTMTX selects a free CLIC
		 * slot and pins the resulting device IRQ to hart 0. */
		intmtx_node = of_find_compatible_node(NULL, NULL,
						 "espressif,esp32s31-intmtx");
		if (!intmtx_node) {
			pr_err_ratelimited("esp32s31-radio: cannot find S-mode INTMTX domain\n");
			s31_radio_irq_install_failed(registration, -ENODEV, generation);
			return;
		}
		fwspec.fwnode = of_node_to_fwnode(intmtx_node);
		fwspec.param_count = 2;
		fwspec.param[0] = registration->source;
		fwspec.param[1] = IRQ_TYPE_LEVEL_HIGH;
		virq = irq_create_fwspec_mapping(&fwspec);
		of_node_put(intmtx_node);
		if (!virq) {
			pr_err_ratelimited("esp32s31-radio: cannot map IDF source %d\n",
					   registration->source);
			s31_radio_irq_install_failed(registration, -ENXIO, generation);
			return;
		}
		if (s31_radio_irq_mapping_busy(virq)) {
			s31_radio_irq_install_failed(registration, -EBUSY, generation);
			return;
		}
		/* Publish the mapping before request_irq(); a live level source
		 * may invoke the hard IRQ handler as soon as it is enabled. */
		ret = s31_linux_domain_irq_get(&registration->domain);
		if (ret) {
			s31_radio_irq_dispose_unused(virq);
			s31_radio_irq_install_failed(registration, ret, generation);
			return;
		}
		registration->virq = virq;
		/* A retry may receive a different CLIC slot than the failed map. */
		registration->priority_applied=false;
		registration->hw_enabled = false;
		/* No source routing/priority/status mutation until genirq confirms
		 * this action's ownership. NO_AUTOEN keeps it masked during bind. */
		ret = request_irq(virq, s31_radio_hardirq,
				  IRQF_NO_AUTOEN | IRQF_NOBALANCING,
				  "esp32s31-radio", registration);
		if (ret) {
			pr_err_ratelimited("esp32s31-radio: request IRQ%d for source %d failed: %d\n",
					   virq, registration->source, ret);
			registration->virq = 0;
			s31_radio_irq_dispose_unused(virq);
			s31_linux_domain_irq_put(registration->domain);
			registration->domain = NULL;
			s31_radio_irq_install_failed(registration, ret, generation);
			return;
		}
		ret = esp_clic_direct_register(virq, registration);
		if (ret) {
			/* Refuse this source rather than silently using generic dispatch. */
			free_irq(virq, registration);
			registration->virq = 0;
			s31_radio_irq_dispose_unused(virq);
			s31_linux_domain_irq_put(registration->domain);
			registration->domain = NULL;
			s31_radio_irq_install_failed(registration, ret, generation);
			pr_err("esp32s31-radio: required direct IRQ%d failed: %d\n", virq, ret);
			return;
		}
		/* IDF disables must mask immediately, rather than lazily after the
		 * next assertion. Only our successfully acquired action is changed. */
		irq_set_status_flags(virq, IRQ_DISABLE_UNLAZY);
		s31_radio_apply_irq_priority(registration);
		if (registration->priority_error) {
			ret = registration->priority_error;
			esp_clic_direct_unregister(virq, registration);
			free_irq(virq, registration);
			registration->virq = 0;
			s31_radio_irq_dispose_unused(virq);
			s31_linux_domain_irq_put(registration->domain);
			registration->domain = NULL;
			s31_radio_irq_install_failed(registration, ret, generation);
			return;
		}
		registration->install_pending = false;
		irqd = irq_get_irq_data(virq);
		if (irqd && irqd->parent_data)
			clic_slot = irqd->parent_data->hwirq;

	}

	if (!registration->virq)
		return;
	/* Apply on reused mappings as well as first installation. */
	s31_radio_apply_irq_priority(registration);
	if (registration->priority_error) {
		s31_radio_irq_install_failed(registration,
			READ_ONCE(registration->priority_error), generation);
		if (registration->hw_enabled) {
			registration->hw_enabled = false;
			disable_irq_nosync(registration->virq);
		}
		return;
	}
	if (READ_ONCE(registration->enabled) && !registration->hw_enabled &&
	    !atomic_read(&registration->callback_pending)) {
		registration->hw_enabled = true;
		/* Publish the intended state before unmasking.  A level source may
		 * re-enter immediately from enable_irq(); its hardirq transition back
		 * to false must win instead of being overwritten on return. */
		enable_irq(registration->virq);
	} else if (!READ_ONCE(registration->enabled) &&
		   registration->hw_enabled) {
		registration->hw_enabled = false;
		disable_irq_nosync(registration->virq);
	}
}

static void s31_radio_sync_irq_registrations(void)
{
	int i;

	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++)
		s31_radio_sync_one_irq(&s31_idf_irqs[i]);
}

static bool s31_radio_irq_callback_pending(void)
{
	return atomic_read(&s31_radio_pending_isrs) != 0;
}

bool s31_radio_wifi_present(void)
{
	return !!(s31_radio_features & S31_RADIO_FEATURE_WIFI);
}

static void s31_radio_control_kick(void)
{
	struct kthread_worker *worker;
	unsigned long flags;

	/* Same publication lock as the periodic service path. Keep it through
	 * enqueue so a late IRQ cannot retain a worker past its destruction.
	 * STOPPING still admits the terminal command until state is OFFLINE.
	 */
	spin_lock_irqsave(&s31_radio_q_lock, flags);
	worker = s31_radio_worker;
	if (worker && s31_radio_q_state != S31_Q_OFF &&
	    atomic_read(&s31_radio_state) != ESP32S31_RADIO_OFFLINE) {
		kthread_queue_work(worker, &s31_radio_work);
	}
	spin_unlock_irqrestore(&s31_radio_q_lock, flags);
}

static void s31_radio_submit_command(struct s31_radio_command *command)
{
	unsigned long flags;

	/* Serialize admission AND list publication with STOPPING/OFFLINE.
	 * Lock order is q_lock -> command_lock; consumers detach under only
	 * command_lock and execute/complete after unlocking. */
	spin_lock_irqsave(&s31_radio_q_lock, flags);
	if (atomic_read(&s31_radio_state) == ESP32S31_RADIO_OFFLINE ||
	    (s31_radio_q_state != S31_Q_RUNNING &&
	     !(s31_radio_q_state == S31_Q_STOPPING &&
	       command->type == S31_RADIO_COMMAND_SHUTDOWN))) {
		spin_unlock_irqrestore(&s31_radio_q_lock, flags);
		command->result = -ESHUTDOWN;
		complete(&command->done);
		return;
	}
	spin_lock(&s31_radio_command_lock);
	list_add_tail(&command->node, &s31_radio_commands);
	spin_unlock(&s31_radio_command_lock);
	spin_unlock_irqrestore(&s31_radio_q_lock, flags);

	s31_radio_control_kick();
	if (command->type == S31_RADIO_COMMAND_SHUTDOWN)
		__s31_radio_queue_force();
	else
		s31_radio_wake();
}

static bool s31_radio_command_work_pending(void)
{
	bool pending;

	spin_lock(&s31_radio_command_lock);
	pending = !list_empty(&s31_radio_commands);
	spin_unlock(&s31_radio_command_lock);
	return pending;
}

/* Queue an immediate serialized radio pass.  Safe from any context
 * (including hard IRQ): workqueue queueing never sleeps, and duplicate
 * queues collapse into the already-pending pass.  Once STOPPING, new
 * business is refused; only the SHUTDOWN force path may still wake the
 * worker for its terminal pass. */
static void __s31_radio_queue_force(void)
{
	unsigned long flags;
	struct kthread_worker *worker;

	spin_lock_irqsave(&s31_radio_q_lock, flags);
	worker = s31_radio_worker;
	if ((worker) &&
	    s31_radio_q_state != S31_Q_OFF &&
	    !READ_ONCE(s31_radio_init_dead)) {
		kthread_queue_work(worker, &s31_radio_work);
	}
	spin_unlock_irqrestore(&s31_radio_q_lock, flags);
}

static void s31_radio_queue(void)
{
	unsigned long flags;
	struct kthread_worker *worker;

	/* Queueing is IRQ-safe and nonblocking. Keep publication locked through
	 * the native enqueue so teardown cannot free a pointer sampled by a
	 * producer just before it was unpublished. No per-wake pin is needed.
	 * The worker drops its internal lock before entering any callback.
	 */
	spin_lock_irqsave(&s31_radio_q_lock, flags);
	worker = s31_radio_worker;
	if ((!worker) ||
	    s31_radio_q_state != S31_Q_RUNNING ||
	    READ_ONCE(s31_radio_init_dead))
		goto out;
	if (atomic_xchg(&s31_radio_immediate_pending, 1))
		goto out;
	kthread_queue_work(worker, &s31_radio_work);
out:
	spin_unlock_irqrestore(&s31_radio_q_lock, flags);
}

static void s31_radio_wake(void)
{
	{
		s31_radio_queue();
	}
}

/* Packet TX does not require Linux receive/control delivery. Keep the
 * lifetime lock through the wake: STOPPING closes producers before join/free.
 * Cold initialization uses its separate control worker. Before dispatcher
 * publication or after retirement this notification has no packet consumer.
 * Never use it for HCI RX or commands requiring Linux consumer completions. */
static void s31_radio_tx_wake(void)
{
 unsigned long flags;
 spin_lock_irqsave(&s31_radio_q_lock,flags);
 if ((s31_radio_q_state==S31_Q_RUNNING) &&
     s31_dispatch_started && s31_dispatch_sync &&
     !READ_ONCE(s31_radio_init_dead)) {
  s31_linux_sync_wake(s31_dispatch_sync);

 }
 spin_unlock_irqrestore(&s31_radio_q_lock,flags);
}

/* First usable credit consumes the wait interest. Further returns update
 * the generation but do not issue duplicate scheduler notifications. */
void s31_radio_tx_credit_return(enum s31_tx_credit_resource resource)
{
 if ((unsigned int)resource >= S31_TX_CREDIT_COUNT)
  return;
 /* Only the exact Wi-Fi mutex is bound to LOCK. An unlock alone is not
  * buffer credit: check the ROM's actual recycle marker only while BUFFER
  * interest exists. Acquire pairs with publication of its baseline. */
 if (resource == S31_TX_CREDIT_LOCK &&
     atomic_read_acquire(&s31_tx_credit_wait) == S31_TX_CREDIT_BUFFER + 1 &&
     s31_radio_tx_pool_changed()) {

  s31_radio_tx_credit_return(S31_TX_CREDIT_BUFFER);
 }
 atomic_inc(&s31_tx_credit_sequence[resource]);
 if (atomic_cmpxchg(&s31_tx_credit_wait, resource + 1, 0) == resource + 1) {

  s31_radio_tx_wake();
 }
}

/* Timer mutation is a dispatcher event, not packet/control work. Publish
 * after the timer mux is released; sequence wake closes the sleep race. */
void s31_radio_timer_changed(void)
{
 unsigned long flags;
 bool direct = false;

 spin_lock_irqsave(&s31_radio_q_lock, flags);
 if (s31_radio_q_state == S31_Q_RUNNING && s31_dispatch_started &&
     s31_dispatch_sync && !READ_ONCE(s31_radio_init_dead)) {
  s31_linux_sync_wake(s31_dispatch_sync);
  direct = true;
 }
 spin_unlock_irqrestore(&s31_radio_q_lock, flags);
 if (!direct)
  s31_radio_queue();
}

static bool s31_radio_dispatch_irq_notify(bool deferred)
{
 unsigned long flags;
 bool managed=false;
 spin_lock_irqsave(&s31_radio_q_lock,flags);
 if ((s31_radio_q_state==S31_Q_RUNNING) &&
     s31_dispatch_started && s31_dispatch_sync &&
     !READ_ONCE(s31_radio_init_dead)) {
  if (deferred) {
   s31_linux_sync_wake(s31_dispatch_sync);

  }
  managed=true;
 }
 spin_unlock_irqrestore(&s31_radio_q_lock,flags);
 return managed;
}

static bool s31_radio_hci_tx_pending(void)
{
	return READ_ONCE(s31_hci_tx_head) != READ_ONCE(s31_hci_tx_tail);
}

/* Set when the last TX drain stopped for lack of controller credit (not
 * for lack of frames).  While set, an unbusy ring alone must not requeue
 * empty gate passes: progress waits for the credit event
 * (s31_radio_vhci_send_available or a Number-of-Completed-Packets wake),
 * which clears this and queues the retry pass.  Single ordered passes
 * re-evaluate it freshly, so a clear racing a new stall always converges
 * on the next queued pass. */
static bool s31_hci_tx_blocked_on_credit;

void s31_radio_vhci_send_available(void)
{
	/* The controller reports available credits even when the Linux HCI TX ring
	 * is already empty.  Waking the serialized worker for those notifications
	 * adds a full blob-gate pass to every active A2DP scheduling round. */
	if (s31_radio_hci_tx_pending()) {
		WRITE_ONCE(s31_hci_tx_blocked_on_credit, false);
		s31_radio_tx_wake();
	}
}

int s31_radio_vhci_receive(u8 *frame, u16 length)
{
	const struct esp32s31_radio_hci_ops *ops;
	void *context;
	bool was_empty, native_rx;
	unsigned long flags;
	unsigned int next, tail;

 if (!frame || !length || length > S31_HCI_FRAME_SIZE)
  return -EINVAL;
 spin_lock_irqsave(&s31_hci_lock, flags);
	if (!s31_hci_rx_accepting) {
		/* The host transport is detached and its controller generation is
		 * being destroyed. These events cannot return credits to a live
		 * host; resume reports Hardware Error before accepting new commands. */

		spin_unlock_irqrestore(&s31_hci_lock, flags);
		return 0;
	}
	tail = smp_load_acquire(&s31_hci_rx_tail);
	was_empty = s31_hci_rx_head == tail;
	next = (s31_hci_rx_head + 1) & (s31_hci_rx_slots - 1);
	if (next == tail) {

		spin_unlock_irqrestore(&s31_hci_lock, flags);
		pr_warn_ratelimited("esp32s31-radio: HCI RX ring full\n");
		return -ENOSPC;
	}
	s31_hci_rx[s31_hci_rx_head].length = length;
	memcpy(s31_hci_rx[s31_hci_rx_head].data, frame, length);
	smp_store_release(&s31_hci_rx_head, next);
	ops = s31_hci_ops;
	context = s31_hci_context;
	native_rx = ops && ops->receive;
	spin_unlock_irqrestore(&s31_hci_lock, flags);

	if (was_empty && ops && ops->rx_ready)
		ops->rx_ready(context);
	/* Native HCI has no independent userspace reader. Queue its existing
	 * Linux consumer on the first non-event packet rather than waiting for
	 * a controller event or timer. Events use the latency-sensitive path
	 * below; direct-HCI ACL keeps its independent rx_ready reader wake. */
	if (was_empty && native_rx && frame[0] != HCI_EVENT_PKT)
		s31_radio_wake();
	/* Direct-HCI credit-only events already woke the reader above. */
 if (frame[0] == HCI_EVENT_PKT &&
     (native_rx || length < 3 || frame[1] != 0x13 || s31_radio_hci_tx_pending())) {
  WRITE_ONCE(s31_hci_tx_blocked_on_credit, false);
  s31_radio_wake();
 }

	return 0;
}

int s31_radio_wifi_receive(u8 *frame, u16 length)
{
	return s31_radio_wifi_receive_zerocopy(frame, NULL, length);
}

int s31_radio_wifi_receive_zerocopy(u8 *frame, void *eb, u16 length)
{
	const struct esp32s31_radio_wifi_ops *ops;
	void *context;
	int ret;

	if (!frame || length < ETH_HLEN || length > S31_WIFI_FRAME_SIZE)
		return -EINVAL;

	rcu_read_lock();
	ops = smp_load_acquire(&s31_wifi_ops);
	context = READ_ONCE(s31_wifi_context);
	if (ops && ops->rx_copy) {
		atomic_inc(&s31_wifi_rx_direct_pending);
		ret = ops->rx_copy(context, frame, length);
		if (ret) {
			atomic_dec(&s31_wifi_rx_direct_pending);

			rcu_read_unlock();
			return ret;
		}

		if (ops->rx_ready)
			ops->rx_ready(context);
		rcu_read_unlock();
		return 0;
	}
	rcu_read_unlock();

	return -ENODEV;
}

static unsigned int s31_radio_wifi_rx_occupancy(void)
{
	const struct esp32s31_radio_wifi_ops *ops = READ_ONCE(s31_wifi_ops);
	unsigned int head = smp_load_acquire(&s31_wifi_rx_head);
	unsigned int tail = READ_ONCE(s31_wifi_rx_tail);

	if (ops && ops->rx_copy)
		return atomic_read(&s31_wifi_rx_direct_pending);
	return head >= tail ? head - tail : S31_WIFI_RX_SLOTS - tail + head;
}

static void s31_radio_wifi_rx_pressure_wait(void *unused)
{
 (void)unused;
 /* The consumer returns an explicit resource edge; task stop/queue close
  * cancels this native wait. No timeout or vendor-context transition. */
 wait_event(s31_wifi_rx_space_waitq,
   s31_radio_wifi_rx_occupancy() <= 24 || kthread_should_stop() ||
   READ_ONCE(s31_radio_q_state) != S31_Q_RUNNING);
}

void s31_radio_wifi_rx_throttle(void)
{
 if (s31_radio_wifi_rx_occupancy() < 48) return;
 /* IRQ/atomic callbacks cannot block. NAPI drains after that callback;
  * an ordinary native task can instead wait on the consumer's real edge. */
 s31_linux_native_rx_pressure_wait(s31_radio_wifi_rx_pressure_wait, NULL);
}

void s31_radio_wifi_connected(const u8 *bssid, u8 channel, int status)
{
	if (bssid)
		memcpy(s31_wifi_event_bssid, bssid, sizeof(s31_wifi_event_bssid));
	s31_wifi_event_channel = channel;
	s31_wifi_connect_status = status;
	s31_wifi_connected_ready = true;
	s31_radio_wake();
	/* Workerless mode has no pass runner to drain this: kick the
	 * control work so delivery (and the SoftMAC bridge) runs. */
	s31_radio_control_kick();
}

void s31_radio_wifi_disconnected(u16 reason)
{
	s31_wifi_event_reason = reason;
	s31_wifi_disconnected_ready = true;
	s31_radio_wake();
	s31_radio_control_kick();
}

/* Called by the single native radio-service task. */
static void s31_radio_hci_process_tx(void)
{
#if IS_ENABLED(CONFIG_BT_ESP32S31)
	bool freed = false, credit_stalled = false;
	unsigned int budget = s31_hci_tx_slots;

	/* CPU1 producers may refill concurrently; bound one service pass. */
	while (budget-- && s31_hci_tx_tail != smp_load_acquire(&s31_hci_tx_head)) {
		struct s31_hci_frame *frame = &s31_hci_tx[s31_hci_tx_tail];
		int coex_result,send_result;

		/* Bluedroid uses 0xfc82 to pass A2DP streaming/paused bits to
		 * libcoexist.  ESP32-S31's controller wrapper leaves that optional
		 * VSC disabled, so route the exact IDF command to the same IDF status
		 * API.  Older version-1 payloads without this optional export still
		 * pass the command through to their controller unchanged. */
		if (frame->length == 7 && frame->data[0] == HCI_COMMAND_PKT &&
		    frame->data[1] == 0x82 && frame->data[2] == 0xfc &&
		    frame->data[3] == 3 && frame->data[4] == 2) {
			u8 complete[] = {
				HCI_EVENT_PKT, HCI_EV_CMD_COMPLETE, 4, 1,
				0x82, 0xfc, 0,
			};

			coex_result = s31_radio_coex_status(frame->data[4],
							 frame->data[5],
							 frame->data[6]);
			if (coex_result != -EOPNOTSUPP) {
				complete[6] = coex_result ? 0x12 : 0;
				s31_radio_vhci_receive(complete, sizeof(complete));
				s31_hci_tx_tail = (s31_hci_tx_tail + 1) &
						 (s31_hci_tx_slots - 1);

				freed = true;
				continue;
			}
		}

		{

			send_result=s31_radio_vhci_try_send(frame->data, frame->length);

		}
		if (send_result) {
			credit_stalled = true;

			break;
		}

		s31_hci_tx_tail = (s31_hci_tx_tail + 1) & (s31_hci_tx_slots - 1);

		freed = true;
	}
	/* Fresh verdict for the credit gate: an unbusy ring alone retries only
	 * while frames still flow.  A stall for lack of credit parks until the
	 * credit event (send_available/NOCP) clears and requeues. */
	WRITE_ONCE(s31_hci_tx_blocked_on_credit, credit_stalled &&
		   s31_hci_tx_tail != smp_load_acquire(&s31_hci_tx_head));
	/* The direct-HCI endpoint uses the same readiness edge for RX data and
	 * newly available TX space.  Its callback only wakes the shared poll wait
	 * queue, so this remains harmless for the kernel HCI frontend, whose
	 * receive callback does not provide rx_ready(). */
	if (freed)
		s31_hci_tx_room_ready = true;
	if (freed && s31_hci_ops && s31_hci_ops->rx_ready)
		s31_hci_ops->rx_ready(s31_hci_context);
#endif
}

/* This runs after the gate has restored Linux IRQ and FPU state. */

static void s31_radio_hci_deliver_rx(void)
{
	unsigned int budget = 8;

	for (;;) {
		const struct esp32s31_radio_hci_ops *ops;
		struct s31_hci_frame *frame;
		void *context;
		unsigned long flags;

		/* Continuous A2DP can keep the direct-HCI ring nonempty indefinitely.
		 * Deliver a bounded batch, then return to the blob for Wi-Fi RX/TX and
		 * coexist callbacks.  The worker wait predicate below immediately starts
		 * another round while frames remain, so this is fairness, not throttling. */
		if (!budget--)
			break;

		spin_lock_irqsave(&s31_hci_lock, flags);
		if (s31_hci_rx_tail == s31_hci_rx_head) {
			spin_unlock_irqrestore(&s31_hci_lock, flags);
			break;
		}
		ops = s31_hci_ops;
		context = s31_hci_context;
		frame = &((struct s31_hci_frame *)
			  s31_radio_sram_linux_alias(s31_hci_rx))[s31_hci_rx_tail];
		if (!ops || !ops->receive) {
			spin_unlock_irqrestore(&s31_hci_lock, flags);
			break;
		}
		spin_unlock_irqrestore(&s31_hci_lock, flags);

		ops->receive(context, frame->data, frame->length);
		/* Publish the completed copy before the producer may reuse this slot. */
		smp_store_release(&s31_hci_rx_tail,
				  (s31_hci_rx_tail + 1) & (s31_hci_rx_slots - 1));
	}
}

void s31_radio_wifi_scan_complete(const struct esp32s31_radio_wifi_ap *aps,
				  u16 count, int status)
{
	s31_wifi_scan_count = count;
	s31_wifi_scan_status = status;
	s31_wifi_scan_ready = true;
	s31_radio_wake();
}

static void s31_radio_wifi_deliver_scan(void)
{
	if (!s31_wifi_scan_ready || !s31_wifi_ops)
		return;
	s31_wifi_scan_ready = false;
	s31_wifi_scan_inflight = false;
	/* The payload has restored its optional probe IEs and stopped using this
	 * cold request before publishing final completion. No idle SRAM reserve. */
	s31_radio_sram_free(s31_wifi_scan_params);
	s31_wifi_scan_params = NULL;
	s31_wifi_ops->scan_complete(s31_wifi_context, s31_wifi_scan_status,
				    NULL, s31_wifi_scan_count);
}

/* Bound each native pass so TX batching cannot monopolize the owner. */

/* Ring-space edges are published inside the blob pass. Native skb retries
 * run only here, on Linux SP after FP/IRQ restoration, with no extra worker. */
static void s31_radio_hci_notify_tx_ready(void)
{
	const struct esp32s31_radio_hci_ops *ops;
	void *context;
	unsigned long flags;
	if (!s31_hci_tx_room_ready)
		return;
	s31_hci_tx_room_ready = false;
	spin_lock_irqsave(&s31_hci_lock, flags);
	ops = s31_hci_ops;
	context = s31_hci_context;
	spin_unlock_irqrestore(&s31_hci_lock, flags);
	/* Runtime shutdown joins this executor before frontend/device release. */
	if (ops && ops->tx_ready)
		ops->tx_ready(context);
}

static void s31_radio_wifi_tx_wait_publish(unsigned int resource, unsigned int generation)
{
 s31_wifi_tx_wait_generation = generation;
 /* Order CPU1 buffer-return observations against interest publication. */
 atomic_xchg(&s31_tx_credit_wait, resource + 1);
}

static void s31_radio_wifi_tx_wait_recheck(unsigned int resource, unsigned int generation)
{
 if (atomic_read(&s31_tx_credit_sequence[resource]) != generation ||
     (resource == S31_TX_CREDIT_BUFFER && s31_radio_tx_pool_changed()))
  atomic_set(&s31_tx_credit_wait, 0);
}

/* Retry only on the selected resource, ROM recycle, or source cancellation. */
static bool s31_radio_wifi_tx_retry_ready(void)
{
 int wait = atomic_read(&s31_tx_credit_wait);

 if (READ_ONCE(s31_wifi_tx_head) != READ_ONCE(s31_wifi_tx_tail) &&
     READ_ONCE(s31_wifi_tx[s31_wifi_tx_tail].cancel_requested))
  return true;
 if (!wait)
  return true;
 if (atomic_read(&s31_tx_credit_sequence[wait - 1]) != s31_wifi_tx_wait_generation)
  return true;
 return wait == S31_TX_CREDIT_BUFFER + 1 && s31_radio_tx_pool_changed();
}

/* A payload may resolve only the currently claimed source lease. No skb
 * pointer is trusted merely because it came back across the private ABI. */
static DECLARE_WAIT_QUEUE_HEAD(s31_wifi_tx_source_wait);

static bool s31_radio_softmac_source_pending(u32 epoch)
{
 unsigned long flags;
 unsigned int i;
 bool pending = false;
 spin_lock_irqsave(&s31_wifi_tx_lock, flags);
 if (s31_wifi_tx)
  for (i = s31_wifi_tx_tail; i != s31_wifi_tx_head; i = (i + 1) % S31_WIFI_TX_SLOTS)
   if (s31_wifi_tx[i].skb && s31_wifi_tx[i].view.header.epoch == epoch) {
    pending = true;
    break;
   }
 spin_unlock_irqrestore(&s31_wifi_tx_lock, flags);
 return pending;
}

/* Nonblocking request. A true return means retain the original skb until
 * status or a later check proves its source lease was consumed. A claimed
 * attempt may already have passed its cancellation boundary. Cookie0 is a
 * cold whole-epoch drain, never a normal packet identifier. */
bool esp32s31_radio_softmac_tx_cancel(u32 cookie, u32 epoch)
{
 unsigned long flags;
 unsigned int i;
 bool pending = false;
 spin_lock_irqsave(&s31_wifi_tx_lock, flags);
 if (s31_wifi_tx)
  for (i = s31_wifi_tx_tail; i != s31_wifi_tx_head; i = (i + 1) % S31_WIFI_TX_SLOTS) {
   struct s31_wifi_frame *slot = &s31_wifi_tx[i];
   if (!slot->skb || slot->view.header.epoch != epoch ||
       (cookie && slot->view.header.cookie != cookie))
    continue;
   WRITE_ONCE(slot->cancel_requested, true);
   pending = true;
  }
 /* Waking the dispatcher sync object does not wake a task parked on the
  * PP queue's different waitq. Target its published task directly; the
  * wait predicate rechecks cancellation after prepare_to_wait. tx_lock
  * keeps publication/cleanup and this task pointer's lifetime ordered. */
 if (s31_tx_repair_waiter && s31_wifi_tx &&
     READ_ONCE(s31_wifi_tx[s31_wifi_tx_tail].cancel_requested))
  wake_up_process(s31_tx_repair_waiter);
 spin_unlock_irqrestore(&s31_wifi_tx_lock, flags);
 if (pending)
  s31_radio_tx_wake();
 return pending;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_softmac_tx_cancel);

bool s31_radio_softmac_tx_repair_cancelled(void)
{
 return current != READ_ONCE(s31_tx_attempt_owner) || !s31_wifi_tx ||
        READ_ONCE(s31_wifi_tx[s31_wifi_tx_tail].cancel_requested);
}

bool s31_radio_softmac_tx_repair_begin(void)
{
 unsigned long flags;
 bool ready = false;
 spin_lock_irqsave(&s31_wifi_tx_lock, flags);
 if (current == READ_ONCE(s31_tx_attempt_owner) && s31_wifi_tx &&
     s31_wifi_tx_tail != s31_wifi_tx_head &&
     s31_wifi_tx[s31_wifi_tx_tail].interface == S31_WIFI_IF_SOFTMAC_TX_VIEW) {

  if (!s31_radio_softmac_tx_repair_cancelled()) {
   WARN_ON_ONCE(s31_tx_repair_waiter);
   s31_tx_repair_waiter = current;
   ready = true;
  }
 }
 spin_unlock_irqrestore(&s31_wifi_tx_lock, flags);
 return ready;
}

bool s31_radio_softmac_tx_repair_end(void)
{
 unsigned long flags;
 bool cancelled;
 spin_lock_irqsave(&s31_wifi_tx_lock, flags);
 cancelled = s31_radio_softmac_tx_repair_cancelled();
 WARN_ON_ONCE(s31_tx_repair_waiter != current);
 s31_tx_repair_waiter = NULL;

 spin_unlock_irqrestore(&s31_wifi_tx_lock, flags);
 return cancelled;
}

/* Submitters must already be closed/joined. No frontend or submit lock may
 * be held while the persistent CPU0 owner acknowledges these source leases. */
void esp32s31_radio_softmac_tx_quiesce(u32 epoch)
{
 might_sleep();
 if (esp32s31_radio_softmac_tx_cancel(0, epoch))
  wait_event(s31_wifi_tx_source_wait, !s31_radio_softmac_source_pending(epoch));
}
EXPORT_SYMBOL_GPL(esp32s31_radio_softmac_tx_quiesce);

/* netif_tx_disable joins enqueue admission before this sleepable cold drain.
 * An attempt already inside the vendor copy must return before acknowledgement. */

/* CPU0 is the sole final-copy owner. These are calculation counters, not
 * delivered-packet counters; an attempt can still fail before PP publication. */

/* 802.11 skb data is commonly +2 aligned, so the generic skb copy falls back
 * to byte-wise stores into the radio TX SRAM. Merge aligned source words the
 * same way the RX copy does, after fixing destination alignment. The source
 * and destination must not overlap and every load stays inside the span. */
static void s31_tx_sram_copy(u8 *d, const u8 *s, size_t length)
{
	while (length && ((unsigned long)d & 3)) {
		*d++ = *s++;
		length--;
	}
	while (length >= 4) {
		u32 v = get_unaligned((const u32 *)s);

		*(u32 *)d = v;
		d += 4;
		s += 4;
		length -= 4;
	}
	while (length--)
		*d++ = *s++;
}

/* Same unfolded one's-complement sum as csum_partial() for the linear span.
 * csum_partial is not exported to modules; this keeps the fast path local. */
static u32 s31_csum_linear(const u8 *p, unsigned int length)
{
	u32 sum = 0;

	while (length >= 4) {
		u32 v = get_unaligned_le32(p);

		sum += (v & 0xffff) + (v >> 16);
		p += 4;
		length -= 4;
	}
	if (length >= 2) {
		sum += get_unaligned_le16(p);
		p += 2;
		length -= 2;
	}
	if (length)
		sum += *p;
	return sum;
}

static int s31_tx_copy_checksum_part(const struct sk_buff *skb, u8 *to,
					unsigned int length)
{
	int start, field;
	__sum16 checksum;
	bool linear = !skb_is_nonlinear(skb) && skb->len == length;

	if (skb->ip_summed != CHECKSUM_PARTIAL) {
		if (linear) {
			s31_tx_sram_copy(to, skb->data, length);
			return 0;
		}

		return skb_copy_bits(skb, 0, to, length);
	}
	/* Internet CHECKSUM_PARTIAL only. The netdev does not advertise SCTP CRC,
	 * so the core completes CRC32c before calling the frontend. */
#if IS_ENABLED(CONFIG_IP_SCTP)
	if (skb->csum_not_inet)
		return -EOPNOTSUPP;
#endif
	if (!skb_frags_readable(skb))
		return -EOPNOTSUPP;
	start = skb_checksum_start_offset(skb);
	if (start < 0 || start > length || start > skb_headlen(skb) ||
	    length - start < sizeof(checksum) ||
	    skb->csum_offset > length - start - sizeof(checksum) ||
	    skb_headlen(skb) - start < sizeof(checksum) ||
	    skb->csum_offset > skb_headlen(skb) - start - sizeof(checksum))
		return -EINVAL;
	field = start + skb->csum_offset;
	if (linear) {
		s31_tx_sram_copy(to, skb->data, length);
		checksum = csum_fold((__force __wsum)
				     s31_csum_linear(skb->data + start,
						     length - start));
		put_unaligned(checksum ? checksum : CSUM_MANGLED_0,
			      (__sum16 *)(to + field));

		return 0;
	} else {
		/* A-MSDU parents keep the first MSDU in the linear head and the
		 * second in page fragments. Copy/checksum the unaligned head with
		 * aligned stores, then let the skb helper walk only the fragments. */
		unsigned int head = skb_headlen(skb);
		__wsum sum;

		if (head >= start && head <= length &&
		    !((head - start) & 1)) {
			s31_tx_sram_copy(to, skb->data, head);
			sum = (__force __wsum)s31_csum_linear(skb->data + start,
							     head - start);
			if (length > head)
				sum += skb_copy_and_csum_bits(skb, head, to + head,
							      length - head);
			checksum = csum_fold(sum);
			put_unaligned(checksum ? checksum : CSUM_MANGLED_0,
				      (__sum16 *)(to + field));

			return 0;
		}
	}
	/* Preserve the source skb and its pseudo-header seed, including cloned
	 * and nonlinear data. The standard helper copies/checksums into SRAM;
	 * this is completed before PP can see the new descriptor. */
 { int copy_error; {  copy_error=skb_copy_bits(skb, 0, to, start); } if(copy_error)return -EFAULT; }
 {
	checksum = csum_fold(skb_copy_and_csum_bits(skb, start, to + start,
						 length - start));
 }
	put_unaligned(checksum ? checksum : CSUM_MANGLED_0,
		      (__sum16 *)(to + field));

	return 0;
}

/* The native vendor allocator owns this private DMA reservation. It has not
 * published the descriptor to HMAC yet. Resolve only the retained source of
 * this task's current attempt, and never retain either pointer on return. */

static int s31_softmac_copy_checksums(const struct sk_buff *skb, u8 *to,
				     unsigned int length)
{
	const struct ieee80211_hdr *hdr = (const void *)skb->data;
	const struct sk_buff *child = skb_shinfo(skb)->frag_list;
	unsigned int first, qos_offset;
	int ret;

	/* Ordinary frag_list is one packet. mac80211's A-MSDU frag_list instead
	 * joins independently checksummed MSDUs, including leading padding in
	 * the second skb. The frontend admits at most two, with no nested list.
	 * Never checksum across the following subframe or its padding. */
	if (!child || !ieee80211_is_data_qos(hdr->frame_control))
		return s31_tx_copy_checksum_part(skb, to, length);
	qos_offset = ieee80211_has_a4(hdr->frame_control) ? 30 : 24;
	if (skb_headlen(skb) < qos_offset + 2)
		return -EINVAL;
	if (!(skb->data[qos_offset] & IEEE80211_QOS_CTL_A_MSDU_PRESENT))
		return s31_tx_copy_checksum_part(skb, to, length);
	first = skb_pagelen(skb);
	if (first > length || child->next || skb_has_frag_list(child) ||
	    child->len != length - first)
		return -EINVAL;
	ret = s31_tx_copy_checksum_part(skb, to, first);
	if (ret)
		return ret;
	return s31_tx_copy_checksum_part(child, to + first, child->len);
}

unsigned long s31_radio_softmac_tx_copy(void *opaque)
{

 int ret;
 const struct s31_softmac_tx_copy *request = opaque;
 struct s31_wifi_frame *slot;
 unsigned long destination;

 if (!request)
  return IS_ENABLED(CONFIG_ESP32S31_WIFI_SOFTMAC) ? S31_SOFTMAC_TX_BATCH_ABI : 0;
 if (in_interrupt() || current != READ_ONCE(s31_tx_attempt_owner) ||
     raw_smp_processor_id() != 0 || !s31_wifi_tx)
  return 0;
 slot = &s31_wifi_tx[s31_wifi_tx_tail];
 destination = request->destination;
 if (!slot->skb || slot->interface != S31_WIFI_IF_SOFTMAC_TX_VIEW ||
     request->token != (u32)(unsigned long)slot->skb ||
     request->length != slot->skb->len ||
     destination < 0x2f000000UL || destination >= 0x2f080000UL ||
     request->length > 0x2f080000UL - destination)
  return 0;

 ret = s31_softmac_copy_checksums(slot->skb, (void *)destination, request->length);

 if (ret)
  return 0;

 return 1;
}

/* Joined producers and dispatcher only: release unpublished/cancelled source
 * leases before destroying the SRAM metadata ring. */
static void s31_radio_wifi_tx_release(void)
{
 unsigned int i;
 if (!s31_wifi_tx)
  return;
 for (i = 0; i < S31_WIFI_TX_SLOTS; i++) {
  struct sk_buff *skb = s31_wifi_tx[i].skb;
  s31_wifi_tx[i].skb = NULL;
  if (skb) {

   consume_skb(skb);
  }
 }
}

static void s31_radio_wifi_tx_batch_close(bool *open)
{
 if (!*open)
  return;
 if (s31_radio_wifi_try_send_interface(S31_WIFI_IF_TX_BATCH_END, NULL, 0))
  panic("esp32s31-radio: Wi-Fi TX resource scope lost its native owner");
 *open = false;
}

static void s31_radio_wifi_process_tx(void)
{

 unsigned int budget = S31_WIFI_TX_BATCH_MAX;
 bool consumed = false, batch_open = false;

 if (!s31_radio_wifi_tx_retry_ready())
  return;
 atomic_set(&s31_tx_credit_wait, 0);
 while (budget-- && s31_wifi_tx_tail != smp_load_acquire(&s31_wifi_tx_head)) {
  struct s31_wifi_frame *frame = &s31_wifi_tx[s31_wifi_tx_tail];
  unsigned int generation[S31_TX_CREDIT_COUNT], resource, i;
  u32 pool_generation;

  int ret;

  if (READ_ONCE(frame->cancel_requested)) {
   struct s31_softmac_tx_status status = { .cookie = frame->view.header.cookie };

   if (frame->interface == S31_WIFI_IF_SOFTMAC_TX_VIEW)
    s31_radio_wifi_receive_aux(S31_WIFI_IF_TX_STATUS, (const u8 *)&status,
                               sizeof(status), 0, 0);
   goto consume_source;
  }

  for (i = 0; i < S31_TX_CREDIT_COUNT; i++)
   generation[i] = atomic_read(&s31_tx_credit_sequence[i]);
  pool_generation = s31_radio_tx_pool_generation();
  WRITE_ONCE(s31_tx_attempt_owner, current);

  ret = 0;
  if ((frame->interface == S31_WIFI_IF_STA || frame->interface == S31_WIFI_IF_AP ||
       frame->interface == S31_WIFI_IF_SOFTMAC_TX_VIEW) &&
      !batch_open) {
   ret = s31_radio_wifi_try_send_interface(S31_WIFI_IF_TX_BATCH_BEGIN,
                                          NULL, 0);
   if (!ret) {
    batch_open = true;

   }
  } else if (frame->interface != S31_WIFI_IF_STA && frame->interface != S31_WIFI_IF_AP &&
             frame->interface != S31_WIFI_IF_SOFTMAC_TX_VIEW) {
   s31_radio_wifi_tx_batch_close(&batch_open);
  }
  if (!ret) {

   ret = s31_radio_wifi_try_send_interface(frame->interface,
          frame->interface == S31_WIFI_IF_SOFTMAC_TX_VIEW ? frame->data : frame->skb->data,
          frame->length);
  }

  WRITE_ONCE(s31_tx_attempt_owner, NULL);
  if (ret == S31_WIFI_TX_BATCH_CLOSED) {
   /* PP owns this accepted frame. Its notification wait already ran with
    * the resource mutex released; the next frame starts a fresh scope. */
   batch_open = false;

   ret = 0;
  }
  if (ret == S31_WIFI_TX_RETRY_LOCK || ret == S31_WIFI_TX_RETRY_FLIGHT ||
      ret == -EAGAIN || ret == 257) {
   s31_radio_wifi_tx_batch_close(&batch_open);
   resource = ret == S31_WIFI_TX_RETRY_LOCK ? S31_TX_CREDIT_LOCK :
              ret == S31_WIFI_TX_RETRY_FLIGHT ? S31_TX_CREDIT_FLIGHT : S31_TX_CREDIT_BUFFER;

   /* Sample before the allocation attempt, publish before interest, then
    * recheck: returns before publication need no separate wake edge. */
   WRITE_ONCE(s31_tx_pool_wait_generation, pool_generation);
   s31_radio_wifi_tx_wait_publish(resource, generation[resource]);
   s31_radio_wifi_tx_wait_recheck(resource, generation[resource]);
   if (consumed)
    wake_up_all(&s31_wifi_tx_source_wait);
   return;
  }
  if (ret) {

   pr_warn_ratelimited("esp32s31-radio: dropping Wi-Fi TX frame: %d\n", ret);
  }
consume_source:
  /* The final descriptor, never this source skb, passes to PP. Keep an
   * independent skb reference even if NAPI status/timeout ran concurrently. */
  {

   struct sk_buff *skb;
   unsigned long flags;
   /* Cold cancellation scans never dereference an unretained skb. Once
    * this publication removes the source, payload reads have finished. */
   skb = READ_ONCE(frame->skb);
   if (skb)
    consume_skb(skb);
   /* Publish drain acknowledgement after the independent reference has
    * actually been released. Cold scanners inspect only metadata/token,
    * never the skb; the slot cannot be reused before tail publication. */
   spin_lock_irqsave(&s31_wifi_tx_lock, flags);
   frame->skb = NULL;
   smp_store_release(&s31_wifi_tx_tail,
                    (s31_wifi_tx_tail + 1) % S31_WIFI_TX_SLOTS);
   if (s31_wifi_tx_space_waiting) {
    s31_wifi_tx_space_waiting = false;
    s31_wifi_tx_space_ready = true;
   }
   spin_unlock_irqrestore(&s31_wifi_tx_lock, flags);
   consumed = true;
  }
 }
 s31_radio_wifi_tx_batch_close(&batch_open);
 if (consumed)
  wake_up_all(&s31_wifi_tx_source_wait);
}

static bool s31_radio_wifi_tx_pending(void)
{
 return READ_ONCE(s31_wifi_tx_head) != READ_ONCE(s31_wifi_tx_tail) &&
        s31_radio_wifi_tx_retry_ready();
}

bool esp32s31_radio_wifi_rx_pending(void)
{
	if (atomic_read(&s31_wifi_rx_direct_pending))
		return true;
	return smp_load_acquire(&s31_wifi_rx_head) !=
	       READ_ONCE(s31_wifi_rx_tail);
}
EXPORT_SYMBOL_GPL(esp32s31_radio_wifi_rx_pending);

void esp32s31_radio_wifi_rx_complete(void)
{
	atomic_dec_if_positive(&s31_wifi_rx_direct_pending);
	wake_up(&s31_wifi_rx_space_waitq);
}
EXPORT_SYMBOL_GPL(esp32s31_radio_wifi_rx_complete);

static void s31_radio_wifi_deliver_control_events(void)
{
	const struct esp32s31_radio_wifi_ops *ops = s31_wifi_ops;

	if (!ops)
		return;
	if (s31_wifi_connected_ready) {
		s31_wifi_connected_ready = false;
		if (ops->connected)
			ops->connected(s31_wifi_context, s31_wifi_connect_status,
				       s31_wifi_event_bssid, s31_wifi_event_channel);
	}
	if (s31_wifi_disconnected_ready) {
		s31_wifi_disconnected_ready = false;
		if (ops->disconnected)
			ops->disconnected(s31_wifi_context, s31_wifi_event_reason);
	}
}

int esp32s31_radio_wifi_rx_dequeue(u8 *frame, size_t capacity)
{
	struct s31_wifi_rx_desc *desc;
	u8 *data;
	unsigned int tail;
	u16 length;

	tail = READ_ONCE(s31_wifi_rx_tail);
	if (tail == smp_load_acquire(&s31_wifi_rx_head))
		return -ENODATA;

	desc = &((struct s31_wifi_rx_desc *)
		s31_radio_sram_linux_alias(s31_wifi_rx))[tail];
	length = READ_ONCE(desc->length);
	if (unlikely(length < ETH_HLEN || length > S31_WIFI_FRAME_SIZE)) {

		length = 0;
	} else if (frame) {
		if (capacity < length)
			return -EMSGSIZE;
		data = s31_radio_sram_linux_alias(s31_wifi_rx_data) +
		       tail * S31_WIFI_FRAME_SIZE;
		memcpy(frame, data, length);
	}

	/* Publish consumption only after the copy.  The closed callback is the
	 * sole producer and NAPI is the sole consumer, so no shared lock is held
	 * while copying a full Ethernet frame. */
	smp_store_release(&s31_wifi_rx_tail,
			  (tail + 1) % S31_WIFI_RX_SLOTS);

	wake_up(&s31_wifi_rx_space_waitq);
	return length;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_wifi_rx_dequeue);

/* Ordinary Linux stack only. A completed request may release the caller's
 * stack immediately, so no field is touched after complete(). */
static void s31_dispatch_complete_commands(void)
{
 LIST_HEAD(done);
 struct s31_radio_command *command, *next;
 unsigned long flags;

 spin_lock_irqsave(&s31_dispatch_command_lock, flags);
 list_splice_init(&s31_dispatch_completions, &done);
 spin_unlock_irqrestore(&s31_dispatch_command_lock, flags);
 list_for_each_entry_safe(command, next, &done, node) {
  list_del_init(&command->node);
  if (!command->result)
   memcpy(command->mac, command->vendor_mac, ETH_ALEN);
  complete(&command->done);
 }
}

static void s31_dispatch_queue_command(struct s31_radio_command *command)
{
 unsigned long flags;
 spin_lock_irqsave(&s31_dispatch_command_lock, flags);
 list_add_tail(&command->node, &s31_dispatch_commands);

 spin_unlock_irqrestore(&s31_dispatch_command_lock, flags);
 /* The control pass requests/wakes the dispatcher after command processing;
  * first-use creation is likewise on that independent Linux stack. */
}

/* This body runs only in the real firmware-created managed dispatcher, on
 * its SRAM stack, with an installed TCB/FP/errno context. */
static void s31_dispatch_run_commands(void)
{
 struct s31_radio_command *command;
 unsigned long flags;
 unsigned int budget = 8;
 u8 mac[ETH_ALEN];

 while (budget--) {
  spin_lock_irqsave(&s31_dispatch_command_lock, flags);
  if (list_empty(&s31_dispatch_commands)) {
   spin_unlock_irqrestore(&s31_dispatch_command_lock, flags);
   break;
  }
  command = list_first_entry(&s31_dispatch_commands, struct s31_radio_command, node);
  list_del_init(&command->node);
  if (s31_dispatch_inflight)
   panic("esp32s31-radio: recursive managed command execution");
  s31_dispatch_inflight = command;
  spin_unlock_irqrestore(&s31_dispatch_command_lock, flags);
  if (!s31_linux_current_cookie())
   panic("esp32s31-radio: vendor command without real managed TCB");
  command->result = s31_radio_wifi_read_mac(mac);
  if (!command->result) memcpy(command->vendor_mac, mac, ETH_ALEN);

  spin_lock_irqsave(&s31_dispatch_command_lock, flags);
  s31_dispatch_inflight = NULL;
  list_add_tail(&command->node, &s31_dispatch_completions);
  spin_unlock_irqrestore(&s31_dispatch_command_lock, flags);
  /* Let Linux complete independently even if the next vendor timer/IRQ
   * callback blocks before this task next reaches its wait bridge. */
  s31_radio_control_kick();
 }
}

/* Pending requests can be cancelled without touching an executing body.
 * Only a joined dispatcher can relinquish a nonlocally abandoned in-flight
 * request. The caller's stack remains alive until this final completion. */
static void s31_dispatch_cancel_commands(bool joined)
{
 struct s31_radio_command *command, *next;
 unsigned long flags;
 LIST_HEAD(cancelled);

 spin_lock_irqsave(&s31_dispatch_command_lock, flags);
 list_splice_init(&s31_dispatch_commands, &cancelled);
 if (joined && s31_dispatch_inflight) {
  list_add_tail(&s31_dispatch_inflight->node, &cancelled);
  s31_dispatch_inflight = NULL;
 }
 list_for_each_entry_safe(command, next, &cancelled, node) {
  command->result = -ESHUTDOWN;

 }
 list_splice_tail_init(&cancelled, &s31_dispatch_completions);
 spin_unlock_irqrestore(&s31_dispatch_command_lock, flags);
 s31_dispatch_complete_commands();
}

/* A separate native owner is required: control may wait for PP while the
 * dispatcher must continue IRQ/timer/TX service. One serialized SRAM request
 * is retained through timeout; no per-request stack or TLS owner is created. */
static void s31_wifi_control_owner_entry(void *unused)
{
 get_task_struct(current);
 WRITE_ONCE(s31_wifi_control_thread, current);

 complete(&s31_wifi_control_ready);
 for (;;) {
  u32 sequence = s31_linux_sync_sequence(s31_wifi_control_sync);
  if (atomic_xchg(&s31_wifi_control_pending, 0)) {
   WRITE_ONCE(s31_wifi_control_result, -EINPROGRESS);
   s31_radio_wifi_control_task(s31_wifi_control_buffer);

   /* The payload has returned and cannot still read the shared request.
    * Completion lets its caller wipe/reuse it immediately. */
   complete(&s31_wifi_control_done);
  } else {
   s31_linux_sync_wait(s31_wifi_control_sync, sequence, U32_MAX,
                       S31_BLOB_RELEASE_QUEUE_RECEIVE);
  }
 }
}

static int s31_wifi_control_publish(void)
{
 unsigned long flags;
 int result = 0;

 spin_lock_irqsave(&s31_radio_q_lock, flags);
 if (s31_radio_q_state != S31_Q_RUNNING ||
     atomic_read(&s31_radio_state) != ESP32S31_RADIO_READY ||
     !s31_wifi_control_started) {
  result = -ESHUTDOWN;
 } else {
  /* The caller mutex permits only one pending/in-flight SRAM request. */
  atomic_set_release(&s31_wifi_control_pending, 1);
  s31_linux_sync_wake(s31_wifi_control_sync);
 }
 spin_unlock_irqrestore(&s31_radio_q_lock, flags);
 return result;
}

/* First use only, on the CPU0 Linux control worker. Subsequent calls publish
 * directly to this owner, bypassing the intermediate Linux work item. */
static int s31_wifi_control_owner_start(void)
{
 if (!s31_wifi_control_started) {
  s31_wifi_control_sync = s31_linux_dispatch_sync_create();
  if (!s31_wifi_control_sync) return -ENOMEM;
  reinit_completion(&s31_wifi_control_ready);
  /* SCO can keep the controller's RR class runnable continuously. Control
   * must share that class with PP/BT rather than depend on lower-priority
   * slack: even a capability reply otherwise misses its caller's deadline.
   * Synchronous PP waits still block normally and retain native PI. */
  if (xTaskCreatePinnedToCore(s31_wifi_control_owner_entry, "wifi-control",
                            8192, NULL, 23, NULL, 0) != 1) {
   s31_linux_dispatch_sync_destroy(s31_wifi_control_sync);
   s31_wifi_control_sync = NULL;
   return -ENOMEM;
  }
  wait_for_completion(&s31_wifi_control_ready);

  smp_store_release(&s31_wifi_control_started, true);

 }
 return s31_wifi_control_publish();
}

/* Queue admission is closed, but PP/IRQs and the dispatcher are still alive.
 * Join any timed-out operation before stopping the idle owner and its TLS
 * finalizers. Never cancel an unknown closed RPC and free its stack/request
 * while PP could still reference it. A stuck operation keeps ownership. */
static void s31_wifi_control_owner_stop(void)
{
 mutex_lock(&s31_wifi_control_mutex);
 if (s31_wifi_control_started) {
  struct task_struct *thread = s31_wifi_control_thread;
  int ret;

  if (s31_wifi_control_poisoned)
   wait_for_completion(&s31_wifi_control_done);
  ret = s31_linux_task_stop_thread(thread);
  if (ret) panic("esp32s31-radio: control owner join failed (%d)", ret);
  WRITE_ONCE(s31_wifi_control_started, false);
  WRITE_ONCE(s31_wifi_control_thread, NULL);
  put_task_struct(thread);

 }
 mutex_unlock(&s31_wifi_control_mutex);
}

/* Run only from the radio worker, on its normal kernel stack. */
static void s31_radio_process_commands(void)
{
	LIST_HEAD(commands);
	struct s31_radio_command *command, *next;

	spin_lock(&s31_radio_command_lock);
	list_splice_init(&s31_radio_commands, &commands);
	spin_unlock(&s31_radio_command_lock);

	list_for_each_entry_safe(command, next, &commands, node) {
		list_del_init(&command->node);

		switch (command->type) {

		case S31_RADIO_COMMAND_WIFI_GET_MAC:
			{
				s31_dispatch_queue_command(command);
				continue; /* Managed execution, then Linux completion. */
			}
			/* Short non-sleeping entry: still needs its own window
			 * so a sleeper's popped descriptor is never reused. */

		case S31_RADIO_COMMAND_WIFI_SCAN:
			if (s31_wifi_scan_inflight) {
				command->result = -EBUSY;
				break;
			}
			s31_wifi_scan_params = s31_radio_sram_alloc(sizeof(*command->scan));
			if (!s31_wifi_scan_params) {
				command->result = -ENOMEM;
				break;
			}
			memcpy(s31_radio_sram_linux_alias(s31_wifi_scan_params),
			       command->scan, sizeof(*command->scan));
			/* Publish before starting a task that may complete immediately. */
			s31_wifi_scan_inflight = true;
			if (xTaskCreatePinnedToCore(s31_radio_wifi_scan_task,
						    "wifi-scan", 4096, s31_wifi_scan_params,
						    20, NULL, 0) != 1) {
				s31_wifi_scan_inflight = false;
				s31_radio_sram_free(s31_wifi_scan_params);
				s31_wifi_scan_params = NULL;
				command->result = -ENOMEM;
				break;
			}
			command->result = 0;
			break;
		case S31_RADIO_COMMAND_WIFI_CONNECT:
			memcpy(s31_radio_sram_linux_alias(s31_wifi_connect_params),
			       command->connect,
			       sizeof(*s31_wifi_connect_params));
			if (xTaskCreatePinnedToCore(s31_radio_wifi_connect_task,
						    "wifi-connect", 4096,
						    s31_wifi_connect_params, 20,
						    NULL, 0) != 1)
				command->result = -ENOMEM;
			else
				command->result = 0;
			break;
		case S31_RADIO_COMMAND_WIFI_DISCONNECT:
			*(u16 *)s31_radio_sram_linux_alias(s31_wifi_disconnect_reason) =
				command->disconnect_reason;
			if (xTaskCreatePinnedToCore(s31_radio_wifi_disconnect_task,
						    "wifi-disconnect", 2048,
						    s31_wifi_disconnect_reason, 20,
						    NULL, 0) != 1)
				command->result = -ENOMEM;
			else
				command->result = 0;
			break;
		case S31_RADIO_COMMAND_WIFI_CONTROL:
			command->result = s31_wifi_control_owner_start();
			break;
		case S31_RADIO_COMMAND_BT_ENABLE:
#if IS_ENABLED(CONFIG_BT_ESP32S31)
		{
			unsigned long flags;
			bool start;

			/* Reserve one controller transition before publishing its
			 * task. A second command joins the existing operation; no
			 * vendor body or task allocation runs under this short lock. */
			spin_lock_irqsave(&s31_bt_enable_lock, flags);
			start = !s31_bt_enable_inflight && s31_bt_enable_result != 0;
			if (start) {
				s31_bt_enable_inflight = true;
				WRITE_ONCE(s31_bt_enable_result, -EINPROGRESS);
			}
			spin_unlock_irqrestore(&s31_bt_enable_lock, flags);
			command->result = 0;
			if (start && xTaskCreatePinnedToCore(s31_radio_bt_enable_task,
						    "bt-enable", 8192, NULL, 22,
						    NULL, 0) != 1) {
				command->result = -ENOMEM;
				s31_radio_report_bt_enable(-ENOMEM);
			}
		}
#else
			command->result = -EOPNOTSUPP;
#endif
			break;
		case S31_RADIO_COMMAND_BT_DISABLE:
#if IS_ENABLED(CONFIG_BT_ESP32S31)
			if (xTaskCreatePinnedToCore(s31_radio_bt_disable_task,
						    "bt-disable", 4096, NULL, 22,
						    NULL, 0) != 1)
				command->result = -ENOMEM;
			else
				command->result = 0;
#else
			command->result = -EOPNOTSUPP;
#endif
			break;
		case S31_RADIO_COMMAND_SHUTDOWN:
			if (xTaskCreatePinnedToCore(s31_radio_shutdown_task,
						    "radio-shutdown", 8192,
						    (void *)(uintptr_t)s31_radio_features,
						    22, NULL, 0) != 1)
				command->result = -ENOMEM;
			else
				command->result = 0;
			break;
		default:
			command->result = -EOPNOTSUPP;
			break;
		}
		complete(&command->done);
	}
}

/* Optional IDF tables are empty in the built-in radio payload. */
asm(".globl _esp_err_msg_tbl_start\n"
    ".globl _esp_err_msg_tbl_end\n"
    ".set _esp_err_msg_tbl_start, 0\n"
    ".set _esp_err_msg_tbl_end, 0\n");
#endif

int esp32s31_radio_hci_register(const struct esp32s31_radio_hci_ops *ops,
				void *context)
{
	unsigned long flags;
	int ret = 0;

	if (!ops || (!ops->receive && !ops->rx_ready))
		return -EINVAL;
	if (atomic_read(&s31_radio_state) != ESP32S31_RADIO_READY)
		return -EAGAIN;
	spin_lock_irqsave(&s31_hci_lock, flags);
	if (s31_hci_ops)
		ret = -EBUSY;
	else {
		s31_hci_context = context;
		smp_store_release(&s31_hci_ops, ops);
	}
	spin_unlock_irqrestore(&s31_hci_lock, flags);
	if (!ret)
		s31_radio_wake();
	return ret;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_hci_register);

void esp32s31_radio_hci_unregister(const struct esp32s31_radio_hci_ops *ops,
				   void *context)
{
	unsigned long flags;

	spin_lock_irqsave(&s31_hci_lock, flags);
	if (s31_hci_ops == ops && s31_hci_context == context) {
		s31_hci_ops = NULL;
		s31_hci_context = NULL;
	}
	spin_unlock_irqrestore(&s31_hci_lock, flags);
}
EXPORT_SYMBOL_GPL(esp32s31_radio_hci_unregister);

static unsigned long s31_tx_batch_calls, s31_tx_batch_frames, s31_tx_batch_partial;
/* Frames actually handed to the controller by the HCI TX drain, and the
 * native-owner pass count. Together they distinguish "owner never polled"
 * from "owner polled but the controller produced no RX". */

/* Private entry for a validated, stable kernel frame. Dynamic controller
 * readiness and per-frame ring capacity still apply to every batch member. */
static int s31_hci_enqueue_validated(u8 packet_type, const u8 *data, size_t length,
				   bool wake)
{
	unsigned long flags;
	unsigned int next;
	struct s31_hci_frame *frame;

	if (atomic_read(&s31_radio_state) != ESP32S31_RADIO_READY)
		return -ENODEV;
	spin_lock_irqsave(&s31_hci_lock, flags);
	next = (s31_hci_tx_head + 1) & (s31_hci_tx_slots - 1);
	if (next == s31_hci_tx_tail) {
		spin_unlock_irqrestore(&s31_hci_lock, flags);
		return -EAGAIN;
	}
	frame = &((struct s31_hci_frame *)
		  s31_radio_sram_linux_alias(s31_hci_tx))[s31_hci_tx_head];
	frame->data[0] = packet_type;
	memcpy(frame->data + 1, data, length);
	frame->length = length + 1;
	smp_store_release(&s31_hci_tx_head, next);
	if (packet_type == HCI_ACLDATA_PKT) {

	}
	spin_unlock_irqrestore(&s31_hci_lock, flags);

	if (wake)
		s31_radio_tx_wake();
	return 0;
}
int esp32s31_radio_hci_send(u8 packet_type, const u8 *data, size_t length)
{
	if (!data || length > S31_HCI_FRAME_SIZE - 1)
		return -EMSGSIZE;
	if (!s31_hci_frame_valid(packet_type, data, length))
		return -EINVAL;
	return s31_hci_enqueue_validated(packet_type, data, length, true);
}
EXPORT_SYMBOL_GPL(esp32s31_radio_hci_send);

/* Validate the complete batch before any side effect. Return a byte count
 * ending at an accepted record boundary; caller retains the unsent suffix.
 * Individual ring operations retain their original IRQ-off duration. */
int esp32s31_radio_hci_send_batch(const u8 *data, size_t length)
{
	size_t pos = 1, accepted = 1;
	unsigned int frames = 0;
	int ret = -EINVAL;

	if (length < 5 || data[0] != 0xff)
		return -EINVAL;
	while (pos < length) {
		unsigned int n;
		if (length - pos < 2 || ++frames > 8)
			return -EINVAL;
		n = data[pos] | (data[pos + 1] << 8);
		pos += 2;
		if (n < 2 || n > S31_HCI_FRAME_SIZE || n > length - pos)
			return -EINVAL;
		if (!s31_hci_frame_valid(data[pos], data + pos + 1, n - 1))
			return -EINVAL;
		pos += n;
	}
	s31_tx_batch_calls++;
	pos = 1;
	while (pos < length) {
		unsigned int n = data[pos] | (data[pos + 1] << 8);
		ret = s31_hci_enqueue_validated(data[pos + 2], data + pos + 3, n - 1, false);
		if (ret)
			break;
		pos += 2 + n;
		accepted = pos;
		s31_tx_batch_frames++;
	}
	if (accepted > 1) {
		if (accepted < length)
			s31_tx_batch_partial++;
		s31_radio_tx_wake();
		return accepted;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_hci_send_batch);

bool esp32s31_radio_hci_tx_has_space(void)
{
	unsigned int head = smp_load_acquire(&s31_hci_tx_head);
	unsigned int tail = READ_ONCE(s31_hci_tx_tail);

	return ((head + 1) & (s31_hci_tx_slots - 1)) != tail;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_hci_tx_has_space);

bool esp32s31_radio_hci_rx_pending(void)
{
	return smp_load_acquire(&s31_hci_rx_head) != READ_ONCE(s31_hci_rx_tail);
}
EXPORT_SYMBOL_GPL(esp32s31_radio_hci_rx_pending);

int esp32s31_radio_hci_peek(const u8 **data, size_t *length)
{
	struct s31_hci_frame *frame;
	unsigned int tail;
	u16 frame_length;

	if (!data || !length)
		return -EINVAL;
	tail = READ_ONCE(s31_hci_rx_tail);
	if (tail == smp_load_acquire(&s31_hci_rx_head))
		return -ENODATA;
	frame = &((struct s31_hci_frame *)
		  s31_radio_sram_linux_alias(s31_hci_rx))[tail];
	frame_length = READ_ONCE(frame->length);
	if (!frame_length || frame_length > S31_HCI_FRAME_SIZE)
		return -EIO;
	*data = frame->data;
	*length = frame_length;
	return 0;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_hci_peek);

void esp32s31_radio_hci_consume(void)
{
	unsigned long flags;

	spin_lock_irqsave(&s31_hci_lock, flags);
	if (s31_hci_rx_tail != smp_load_acquire(&s31_hci_rx_head))
		smp_store_release(&s31_hci_rx_tail,
				  (s31_hci_rx_tail + 1) & (s31_hci_rx_slots - 1));
	spin_unlock_irqrestore(&s31_hci_lock, flags);
}
EXPORT_SYMBOL_GPL(esp32s31_radio_hci_consume);

int esp32s31_radio_hci_dequeue(u8 *data, size_t capacity)
{
	struct s31_hci_frame *frame;
	unsigned long flags;
	u16 length;

	if (!data)
		return -EINVAL;
	spin_lock_irqsave(&s31_hci_lock, flags);
	if (s31_hci_rx_tail == smp_load_acquire(&s31_hci_rx_head)) {
		spin_unlock_irqrestore(&s31_hci_lock, flags);
		return -ENODATA;
	}
	frame = &((struct s31_hci_frame *)
		  s31_radio_sram_linux_alias(s31_hci_rx))[s31_hci_rx_tail];
	length = READ_ONCE(frame->length);
	if (length > capacity) {
		spin_unlock_irqrestore(&s31_hci_lock, flags);
		return -EMSGSIZE;
	}
	memcpy(data, frame->data, length);
	s31_hci_rx_tail = (s31_hci_rx_tail + 1) & (s31_hci_rx_slots - 1);
	spin_unlock_irqrestore(&s31_hci_lock, flags);
	return length;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_hci_dequeue);

void s31_radio_hci_rx_quiesce(void)
{
	unsigned long flags;

	spin_lock_irqsave(&s31_hci_lock, flags);
	s31_hci_rx_accepting = false;

	s31_hci_rx_tail = s31_hci_rx_head;
	spin_unlock_irqrestore(&s31_hci_lock, flags);
}

void esp32s31_radio_hci_purge(void)
{
	unsigned long flags;

	spin_lock_irqsave(&s31_hci_lock, flags);
	s31_hci_rx_tail = s31_hci_rx_head;
	spin_unlock_irqrestore(&s31_hci_lock, flags);
}
EXPORT_SYMBOL_GPL(esp32s31_radio_hci_purge);

int esp32s31_radio_wifi_register(const struct esp32s31_radio_wifi_ops *ops,
				 void *context)
{
	if (!ops || !ops->scan_complete || !ops->rx_ready)
		return -EINVAL;
	if (atomic_read(&s31_radio_state) != ESP32S31_RADIO_READY)
		return -EAGAIN;
	mutex_lock(&s31_wifi_registration_lock);
	if (READ_ONCE(s31_wifi_ops)) {
		mutex_unlock(&s31_wifi_registration_lock);
		return -EBUSY;
	}
	WRITE_ONCE(s31_wifi_context, context);
	smp_store_release(&s31_wifi_ops, ops);
	mutex_unlock(&s31_wifi_registration_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_wifi_register);

void esp32s31_radio_wifi_unregister(const struct esp32s31_radio_wifi_ops *ops,
				    void *context)
{
	mutex_lock(&s31_wifi_registration_lock);
	if (READ_ONCE(s31_wifi_ops) == ops &&
	    READ_ONCE(s31_wifi_context) == context) {
		smp_store_release(&s31_wifi_ops, NULL);
		/* IRQ/vendor RX and station callbacks retain the context under RCU.
		 * Scan/connection delivery runs only on the Linux pass and may sleep. */
		synchronize_rcu();
		kthread_flush_work(&s31_radio_work);
		WRITE_ONCE(s31_wifi_context, NULL);
	}
	mutex_unlock(&s31_wifi_registration_lock);
}
EXPORT_SYMBOL_GPL(esp32s31_radio_wifi_unregister);

int esp32s31_radio_wifi_get_mac(u8 mac[6])
{
	struct s31_radio_command command;

	if (!mac)
		return -EINVAL;
	if (!s31_wifi_ops)
		return -ENODEV;
	INIT_LIST_HEAD(&command.node);
	init_completion(&command.done);
	command.type = S31_RADIO_COMMAND_WIFI_GET_MAC;
	command.result = -EINPROGRESS;
	command.mac = mac;
	s31_radio_submit_command(&command);
	wait_for_completion(&command.done);
	return command.result;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_wifi_get_mac);

int esp32s31_radio_wifi_scan(const struct s31_wifi_scan_params *params)
{
	struct s31_radio_command command;
	unsigned int i;

	if (!params || params->n_ssids > S31_WIFI_SCAN_MAX_SSIDS ||
	    params->ie_length > S31_WIFI_SCAN_MAX_IE ||
	    !params->channels || (params->channels & ~0x3ffeU) ||
	    (params->passive_channels & ~params->channels) || params->dwell_ms > 1500)
		return -EINVAL;
	for (i = 0; i < params->n_ssids; i++)
		if (params->ssid_length[i] > 32 ||
		    params->ssid[i][params->ssid_length[i]] ||
		    memchr(params->ssid[i], 0, params->ssid_length[i]))
			return -EINVAL;
	if (!s31_wifi_ops)
		return -ENODEV;
	INIT_LIST_HEAD(&command.node);
	init_completion(&command.done);
	command.type = S31_RADIO_COMMAND_WIFI_SCAN;
	command.result = -EINPROGRESS;
	command.scan = params;
	s31_radio_submit_command(&command);
	wait_for_completion(&command.done);
	return command.result;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_wifi_scan);

int esp32s31_radio_wifi_connect(
		const struct esp32s31_radio_wifi_connect_params *params)
{
	struct s31_radio_command command;

	if (!params || !params->ssid_length || params->ssid_length > 32)
		return -EINVAL;
	if (!s31_wifi_ops)
		return -ENODEV;
	INIT_LIST_HEAD(&command.node);
	init_completion(&command.done);
	command.type = S31_RADIO_COMMAND_WIFI_CONNECT;
	command.result = -EINPROGRESS;
	command.connect = params;
	s31_radio_submit_command(&command);
	wait_for_completion(&command.done);
	return command.result;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_wifi_connect);

int esp32s31_radio_wifi_disconnect(u16 reason)
{
	struct s31_radio_command command;

	if (!s31_wifi_ops)
		return -ENODEV;
	INIT_LIST_HEAD(&command.node);
	init_completion(&command.done);
	command.type = S31_RADIO_COMMAND_WIFI_DISCONNECT;
	command.result = -EINPROGRESS;
	command.disconnect_reason = reason;
	s31_radio_submit_command(&command);
	wait_for_completion(&command.done);
	return command.result;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_wifi_disconnect);

int esp32s31_radio_bt_enable(void)
{
#if !IS_ENABLED(CONFIG_BT_ESP32S31)
	return -EOPNOTSUPP;
#else
	struct s31_radio_command command;

	if (atomic_read(&s31_radio_state) != ESP32S31_RADIO_READY)
		return -EAGAIN;
	/* The BTDM controller enable path is one-shot: a second enable from
	 * bluetoothd/hci0 open would run esp_bt_controller_enable() again and
	 * block in the blob.  If the payload already reported success, tell the
	 * HCI open path that the controller is already enabled.
	 */
	if (READ_ONCE(s31_bt_enable_result) == 0)
		return 0;
	INIT_LIST_HEAD(&command.node);
	init_completion(&command.done);
	command.type = S31_RADIO_COMMAND_BT_ENABLE;
	command.result = -EINPROGRESS;
	s31_radio_submit_command(&command);
	wait_for_completion(&command.done);
	if (command.result)
		return command.result;
	/* Queue completion means task admission, not controller readiness.
	 * Resume must not expose a reset event/HCI writer until this completes. */
	if (!wait_event_timeout(s31_radio_bt_enable_waitq,
		READ_ONCE(s31_bt_enable_result) != -EINPROGRESS ||
		READ_ONCE(s31_radio_q_state) != S31_Q_RUNNING, 5 * HZ))
		return -ETIMEDOUT;
	if (READ_ONCE(s31_radio_q_state) != S31_Q_RUNNING)
		return -EHOSTDOWN;
	return READ_ONCE(s31_bt_enable_result);
#endif
}
EXPORT_SYMBOL_GPL(esp32s31_radio_bt_enable);

int esp32s31_radio_bt_disable(void)
{
#if !IS_ENABLED(CONFIG_BT_ESP32S31)
	return -EOPNOTSUPP;
#else
	struct s31_radio_command command;

	if (atomic_read(&s31_radio_state) != ESP32S31_RADIO_READY)
		return -EAGAIN;
	if (READ_ONCE(s31_bt_enable_result) != 0)
		return 0;
	WRITE_ONCE(s31_bt_disable_result, -EINPROGRESS);
	INIT_LIST_HEAD(&command.node);
	init_completion(&command.done);
	command.type = S31_RADIO_COMMAND_BT_DISABLE;
	command.result = -EINPROGRESS;
	s31_radio_submit_command(&command);
	wait_for_completion(&command.done);
	if (command.result)
		return command.result;
	if (!wait_event_timeout(s31_radio_bt_disable_waitq,
				READ_ONCE(s31_bt_disable_result) != -EINPROGRESS,
				2 * HZ))
		return -ETIMEDOUT;
	return READ_ONCE(s31_bt_disable_result);
#endif
}
EXPORT_SYMBOL_GPL(esp32s31_radio_bt_disable);

static int s31_radio_wifi_enqueue(u8 interface, const u8 *frame, size_t length,
                                 const struct sk_buff *skb)
{
	unsigned long flags;
	unsigned int next;
	u8 kind;

	if (atomic_read(&s31_radio_state) != ESP32S31_RADIO_READY || !s31_wifi_tx)
		return -ENODEV;
	if ((interface > S31_WIFI_IF_AP && interface != S31_WIFI_IF_SOFTMAC_TX_VIEW) ||
	    !frame || length < (interface == S31_WIFI_IF_SOFTMAC_TX_VIEW ? sizeof(struct s31_softmac_tx_view) : ETH_HLEN) ||
	    length > S31_WIFI_FRAME_SIZE)
		return -EINVAL;
	if (!skb)
		return -EOPNOTSUPP;
	kind = interface == S31_WIFI_IF_SOFTMAC_TX_VIEW ? 0 :
		s31_radio_tx_kind(frame, min_t(size_t, length, skb_headlen(skb)));
	spin_lock_irqsave(&s31_wifi_tx_lock, flags);
	next = (s31_wifi_tx_head + 1) % S31_WIFI_TX_SLOTS;
	if (next == s31_wifi_tx_tail) {
		s31_wifi_tx_space_waiting = true;
		spin_unlock_irqrestore(&s31_wifi_tx_lock, flags);
		return -ENOSPC;
	}
	{
		struct s31_wifi_frame *slot = &((struct s31_wifi_frame *)
			s31_radio_sram_linux_alias(s31_wifi_tx))[s31_wifi_tx_head];

		slot->length = length;
		slot->interface = interface;
		slot->cancel_requested = false;

		if (interface == S31_WIFI_IF_SOFTMAC_TX_VIEW) {
			struct s31_softmac_tx_view *view = &slot->view;
			int ret;

			memset(view, 0, sizeof(*view));
			memcpy(&view->header, frame, sizeof(view->header));
			view->length = skb->len;
			ret = skb_copy_bits(skb, 0, view->prefix,
				min_t(unsigned int, skb->len, sizeof(view->prefix)));
			if (ret) {
				spin_unlock_irqrestore(&s31_wifi_tx_lock, flags);
				return ret;
			}

			slot->skb = skb_get((struct sk_buff *)skb);
			view->token = (u32)(unsigned long)slot->skb;
		} else {
			/* Retain source ownership until the synchronous vendor copy
			 * returns. Never copy payload into this metadata ring. */
			slot->skb = skb_get((struct sk_buff *)skb);
		}
	}
	smp_store_release(&s31_wifi_tx_head, next);
	spin_unlock_irqrestore(&s31_wifi_tx_lock, flags);
	if (interface != S31_WIFI_IF_SOFTMAC_TX_VIEW)
		s31_radio_tx_wake();
	/* Under PREEMPT_VOLUNTARY a same-hart CFS task can otherwise run until
	 * the next tick even though the ACK just queued the TX pass.  Ask
	 * for one reschedule at the next safe kernel/softirq exit.  Never do this
	 * while current owns the blob gate: the runner needs that task to resume
	 * and release the gate before esp_wifi_internal_tx() can run. */
	if (kind == S31_TX_TCP_ACK && raw_smp_processor_id() == 0 &&
	    current != READ_ONCE(s31_radio_runner) &&
	    !s31_linux_blob_held_by_current() &&
	    !test_tsk_need_resched(current)) {
		set_tsk_need_resched(current);
	}
	return 0;
}

int esp32s31_radio_wifi_send_interface(u8 interface, const u8 *frame, size_t length)
{
	return s31_radio_wifi_enqueue(interface, frame, length, NULL);
}
EXPORT_SYMBOL_GPL(esp32s31_radio_wifi_send_interface);

int esp32s31_radio_softmac_send_skb(const struct s31_softmac_tx_header *header,
                                  const struct sk_buff *skb)
{
	if (!header || !skb || skb->len < 24 || skb->len > S31_SOFTMAC_FRAME_MAX)
		return -EINVAL;
	return s31_radio_wifi_enqueue(S31_WIFI_IF_SOFTMAC_TX_VIEW, (const u8 *)header,
				     sizeof(struct s31_softmac_tx_view), skb);
}
EXPORT_SYMBOL_GPL(esp32s31_radio_softmac_send_skb);

/* One publication notification after the frontend's bounded metadata batch. */
void esp32s31_radio_softmac_tx_kick(void)
{
 s31_radio_tx_wake();
}
EXPORT_SYMBOL_GPL(esp32s31_radio_softmac_tx_kick);

int esp32s31_radio_wifi_send(const u8 *frame, size_t length)
{
	return esp32s31_radio_wifi_send_interface(S31_WIFI_IF_STA, frame, length);
}
EXPORT_SYMBOL_GPL(esp32s31_radio_wifi_send);

void s31_radio_wifi_control_complete(int result)
{
	WRITE_ONCE(s31_wifi_control_result, result ? -EIO : 0);
 /* The persistent owner completes only after the payload has returned. */
}

void s31_radio_wifi_mgmt(const u8 *bssid, u8 subtype, const u8 *body,
			u16 length, u8 channel, s8 signal)
{
	const struct esp32s31_radio_wifi_ops *ops;

	/* Unregistration closes publication and joins this RCU borrow before
	 * cfg80211/frontend storage can be freed. No vendor pointer escapes. */
	rcu_read_lock();
	ops = smp_load_acquire(&s31_wifi_ops);
	if (ops && ops->mgmt)
		ops->mgmt(READ_ONCE(s31_wifi_context), bssid, subtype, body,
			  length, channel, signal);
	rcu_read_unlock();
}

void s31_radio_wifi_receive_aux(u8 interface, const u8 *frame, u16 length,
			      u8 channel, s8 signal)
{

	const struct esp32s31_radio_wifi_ops *ops;

	rcu_read_lock();
	ops = smp_load_acquire(&s31_wifi_ops);
	if (ops && ops->receive_aux)
		ops->receive_aux(READ_ONCE(s31_wifi_context), interface, frame,
				 length, channel, signal);
	rcu_read_unlock();
}

void s31_radio_wifi_ap_station(const u8 *mac, bool joined)
{
	const struct esp32s31_radio_wifi_ops *ops;

	rcu_read_lock();
	ops = smp_load_acquire(&s31_wifi_ops);
	if (ops && ops->ap_station)
		ops->ap_station(READ_ONCE(s31_wifi_context), mac, joined);
	rcu_read_unlock();
}

int esp32s31_radio_wifi_control(const struct s31_wifi_control *control)
{
	struct s31_radio_command command = { .type = S31_RADIO_COMMAND_WIFI_CONTROL };
	int ret;

	mutex_lock(&s31_wifi_control_mutex);
	if (atomic_read(&s31_radio_state) != ESP32S31_RADIO_READY ||
	    !s31_wifi_control_buffer || s31_wifi_control_poisoned) {
		ret = -ENODEV;
		goto out;
	}
	memcpy(s31_radio_sram_linux_alias(s31_wifi_control_buffer), control, sizeof(*control));
	reinit_completion(&s31_wifi_control_done);
	if (smp_load_acquire(&s31_wifi_control_started)) {
		ret = s31_wifi_control_publish();

	} else {
		INIT_LIST_HEAD(&command.node);
		init_completion(&command.done);
		s31_radio_submit_command(&command);
		wait_for_completion(&command.done);
		ret = command.result;
	}
	if (!ret && !wait_for_completion_timeout(&s31_wifi_control_done, 5 * HZ)) {
		/* The task may still reference the SRAM request.  Never overwrite
		 * it after timeout; teardown owns its eventual reclamation. */
		s31_wifi_control_poisoned = true;

		ret = -ETIMEDOUT;
	} else if (!ret) {
		ret = READ_ONCE(s31_wifi_control_result);
	}
	if (!s31_wifi_control_poisoned)
		memzero_explicit(s31_radio_sram_linux_alias(s31_wifi_control_buffer),
				 sizeof(*control));
out:
	mutex_unlock(&s31_wifi_control_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_wifi_control);

bool esp32s31_radio_wifi_tx_has_space(void)
{
	unsigned long flags;
	bool has_space;

	spin_lock_irqsave(&s31_wifi_tx_lock, flags);
	has_space = (s31_wifi_tx_head + 1) % S31_WIFI_TX_SLOTS !=
		    s31_wifi_tx_tail;
	if (!has_space)
		s31_wifi_tx_space_waiting = true;
	spin_unlock_irqrestore(&s31_wifi_tx_lock, flags);
	return has_space;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_wifi_tx_has_space);

#ifdef CONFIG_ESP32S31_RADIO_BLOBS
struct s31_radio_blob_pass_args {
	int tick_events;

	u64 timer_deadline_ns;

};

static void s31_radio_run_blob_pass(void *opaque)
{

	struct s31_radio_blob_pass_args *args = opaque;
	int tick_events = args->tick_events;
	bool irq_pass = s31_radio_irq_callback_pending();

	int i;

	/* A device IRQ can publish callback work after the first pending read.
	 * Refresh its publication before claiming work in the native task.
	 * Acknowledge pending hardware before lower-priority software timers/TX.
	 * Clear the aggregate hint before the bounded per-slot claims so a new
	 * IRQ or a refused callback can republish work for the next pass. */
	irq_pass |= s31_radio_irq_callback_pending();
	if (irq_pass)
		atomic_set(&s31_radio_pending_isrs, 0);
	for (i = 0; irq_pass && i < S31_RADIO_IRQ_SLOTS; i++)
		s31_radio_worker_irq_service(&s31_idf_irqs[i]);

	/* Linux jiffies is the task tick, and the local esp_timer shim compares
	 * absolute monotonic deadlines and advances missed periodic deadlines in a
	 * single scan.  Repeating that scan for every elapsed 10 ms slice cannot
	 * dispatch additional callbacks; it only walks the same timer list again. */
	/* esp_timer owns absolute monotonic deadlines. Scan once per managed
	 * dispatch pass; repeated scans never advance the public RTOS tick. */
	if (tick_events > 0 ||
	    (s31_radio_features & (S31_RADIO_FEATURE_WIFI |
				    S31_RADIO_FEATURE_BLUETOOTH)) ==
	    (S31_RADIO_FEATURE_WIFI | S31_RADIO_FEATURE_BLUETOOTH)) {

		{  s31_rtos_tick(); }

	}

	s31_radio_hci_process_tx();

	s31_radio_wifi_process_tx();

	/* TX and callbacks may consume the old hint or start an earlier timer. */
	args->timer_deadline_ns = s31_linux_timer_next_deadline_ns();

}

/* Native Linux completions before the event wait. The managed task retains
 * its own registered SRAM Linux stack throughout. */
static void s31_dispatch_linux_complete(void)
{
 const struct esp32s31_radio_wifi_ops *ops;

 s31_dispatch_complete_commands();

 /* Normal CLIC entry/exit and task switching complete the hardware level.
  * Packet completion must not scan the timer/IPI/device pending registers
  * or synthesize an extra IRQ dispatch on every radio service pass. */

 s31_radio_hci_notify_tx_ready();
 /* Notify only a producer which actually ran out of source slots. A
  * periodic callback while the ring merely has room both contends on the
  * frontend lock and hides the enqueue/requeue race. The frontend rechecks
  * source space after requeue, closing an edge delivered before that skb
  * became visible. A later full rejection arms the next consumer edge. */
 if (s31_wifi_tx_space_ready) {
  s31_wifi_tx_space_ready = false;
  rcu_read_lock();
  ops=READ_ONCE(s31_wifi_ops);
  if (ops && ops->tx_wakeup)
   ops->tx_wakeup(READ_ONCE(s31_wifi_context));
  rcu_read_unlock();
 }
}

/* Called by the real firmware task wrapper on its separately owned SRAM
 * stack. The ordinary managed trampoline installs/releases its blob window. */
static void s31_managed_dispatch_entry(void *unused)
{
 u32 sequence=0;
 while (atomic_read(&s31_radio_state)!=ESP32S31_RADIO_OFFLINE) {
  struct s31_radio_blob_pass_args args={
   .tick_events=1, .timer_deadline_ns=U64_MAX,
  };

  s31_dispatch_run_commands();
  s31_radio_run_blob_pass(&args);

  /* Exhausted batches are runnable work, not a reason to sleep10ms.
   * Resource/credit stalls retain their timer/event-driven retry policy. */
  if (s31_radio_wifi_tx_pending() ||
      (s31_radio_hci_tx_pending() && !READ_ONCE(s31_hci_tx_blocked_on_credit)))
   args.timer_deadline_ns=0;
  sequence=s31_linux_dispatch_wait(s31_dispatch_sync,sequence,
                         args.timer_deadline_ns,
                         s31_dispatch_linux_complete);
 }
}

static int s31_dispatch_request(void)
{
 if (!s31_dispatch_started) {
  if (READ_ONCE(s31_radio_q_state)!=S31_Q_RUNNING)
   return 0;
  s31_dispatch_sync=s31_linux_dispatch_sync_create();
  if (!s31_dispatch_sync) return -ENOMEM;
  /* Creation is on the Linux control stack and publishes a real firmware
   * TCB/managed stop frame. The child never borrows the worker SRAM stack. */
  /* TX source leases also carry TCP ACKs generated on CPU1. Running below
   * the vendor Wi-Fi task lets a sustained receive burst postpone those
   * ACKs even while the source is ready. Share its native priority23 and
   * existing10ms RR quantum: neither runnable owner outranks the other.
   * Keep bounded batches and resource-event waits; do not elevate this
   * dispatcher above Wi-Fi or replace its native synchronization.
   */
  if (xTaskCreatePinnedToCore(s31_managed_dispatch_entry,"radio-service",
                            S31_RADIO_SERVICE_STACK_SIZE,NULL,23,NULL,0)!=1) {
   s31_linux_dispatch_sync_destroy(s31_dispatch_sync);
   s31_dispatch_sync=NULL;
   return -ENOMEM;
  }
  /* Publish the fully initialized waitq under the producer lifetime lock. */
  {
   unsigned long flags;
   spin_lock_irqsave(&s31_radio_q_lock,flags);
   WRITE_ONCE(s31_dispatch_started,true);
   spin_unlock_irqrestore(&s31_radio_q_lock,flags);
  }
 }
 if (atomic_read(&s31_radio_state)!=ESP32S31_RADIO_OFFLINE)
  s31_linux_sync_wake(s31_dispatch_sync);
 return 0;
}

static int s31_radio_run_linux_pass(void)
{

	int ret;

	/* Linux-only producers and completions never enter a vendor window. */
	s31_dispatch_complete_commands();
	s31_radio_hci_deliver_rx();
	s31_radio_wifi_deliver_scan();
	s31_radio_wifi_deliver_control_events();
	s31_radio_process_commands();
	ret = s31_dispatch_request();

	return ret;
}

static void s31_radio_update_state(void)
{
 int wifi = READ_ONCE(s31_wifi_init_result);
 if (wifi != -EINPROGRESS && wifi) {
  atomic_set(&s31_radio_state, ESP32S31_RADIO_FAILED);
  return;
 }
 if (!wifi)
  atomic_cmpxchg(&s31_radio_state, ESP32S31_RADIO_STARTING, ESP32S31_RADIO_READY);
}

static void s31_radio_fail_commands(void)
{
	LIST_HEAD(commands);
	struct s31_radio_command *command, *next;

	s31_dispatch_cancel_commands(false);
	spin_lock(&s31_radio_command_lock);
	list_splice_init(&s31_radio_commands, &commands);
	spin_unlock(&s31_radio_command_lock);
	list_for_each_entry_safe(command, next, &commands, node) {
		list_del_init(&command->node);
		command->result = -ENODEV;
		complete(&command->done);
	}
}

#endif

/* Compatibility bootstrap touches the relocated libc cell and installs
 * native hooks. Run it once in an owned core0 window, before creating any
 * opaque radio task. Its audited body is cache-on metadata preparation, so
 * Linux SP is valid; actual radio/PHY entries keep their SRAM stacks. */
static int s31_radio_initialize_rtos(void)
{
#ifdef CONFIG_ESP32S31_RADIO_BLOBS
 struct s31_blob_context ctx = { 0 };
 int ret = (s31_foreign_owner_prepare_control());
 if (ret) return ret;
 {
  ret = s31_linux_control_bootstrap_enter(&ctx);
  if (ret) return ret;
 }
 s31_rtos_init();
 {
  s31_linux_control_bootstrap_leave(&ctx);
  ret = s31_foreign_owner_seal_control_current();
  if (ret) return ret;
 }
#else
 s31_rtos_init();
#endif
 return s31_linux_native_contract_ready();
}

/* Called only at a guarded native-owner boundary. Hardware/deferred IRQ
 * publication and SRAM TX publication already wake this owner. Inspect the
 * timer deadline here because native callbacks can start/restart a timer
 * while the PP queue remains busy. Do not cache a stale deadline. */

#ifdef CONFIG_ESP32S31_RADIO_BLOBS
/* Immediate-work predicate: the retired thread's wait_event conditions
 * minus the tick, scoped to work this pass can actually advance.  An
 * unread RX ring alone requeues only while an in-pass consumer exists
 * (native hci0 .receive with budget); direct-HCI frames are drained by
 * the userspace reader already woken via rx_ready/poll, so its backlog
 * must not spin the runner.  Likewise a credit-stalled TX ring parks
 * until the credit event requeues. */
static bool s31_radio_hci_rx_deliverable(void)
{
	const struct esp32s31_radio_hci_ops *ops;
	bool pending;
	unsigned long flags;

	spin_lock_irqsave(&s31_hci_lock, flags);
	pending = READ_ONCE(s31_hci_rx_tail) != smp_load_acquire(&s31_hci_rx_head);
	ops = s31_hci_ops;
	spin_unlock_irqrestore(&s31_hci_lock, flags);
	return pending && ops && ops->receive;
}

/* A bounded Linux control pass; vendor execution belongs to the managed
 * dispatcher. Producers explicitly queue control/completion work. */
static void s31_radio_workfn(struct kthread_work *work)
{
	int ret;

	atomic_xchg(&s31_radio_immediate_pending, 0);
	WRITE_ONCE(s31_radio_runner, current);
	/* The private worker retains init_mm across all passes, making the
	 * low identity mappings (payload SRAM, ROM) addressable without a
	 * per-pass mapping transition.
	 */

 if (!READ_ONCE(s31_native_irq_installed)) {
  ret = esp32s31_radio_irq_stack_install(s31_native_irq_stack, IRQ_STACK_SIZE);
  if (ret) {
   pr_err("esp32s31-radio: native IRQ-stack install failed: %d\n", ret);
   goto hw_failed;
  }
  WRITE_ONCE(s31_native_irq_installed, true);

 }
	if (!READ_ONCE(s31_radio_hw_checked)) {
		void __iomem *rom;
		u32 identity_word, ioremap_word;
		unsigned int cpu_mhz;
		ret = s31_radio_initialize_rtos();
		if (ret) {
			pr_err("esp32s31-radio: core0 compatibility bootstrap failed: %d\n", ret);
			goto hw_failed;
		}
		rom = ioremap(S31_ROM_BASE, PAGE_SIZE);

		if (!rom) {
			pr_err("esp32s31-radio: cannot map mask ROM for comparison\n");
			goto hw_failed;
		}
		identity_word = READ_ONCE(*(u32 *)S31_ROM_BASE);
		ioremap_word = readl(rom);
		iounmap(rom);
		if (identity_word != ioremap_word) {
			pr_err("esp32s31-radio: identity ROM mismatch %08x != %08x\n",
			       identity_word, ioremap_word);
			goto hw_failed;
		}
		/* This ROM routine is side-effect free and proves execute
		 * permission. */
		cpu_mhz = ((s31_rom_cpu_freq_t)S31_ROM_CPU_FREQ)();

		if (xTaskCreatePinnedToCore(s31_radio_stack_task, "radio-init",
					    8192,
					    s31_radio_init_args(),
					    24, NULL, 0) != 1) {
			pr_err("esp32s31-radio: cannot create IDF init task\n");
			goto hw_failed;
		}

		WRITE_ONCE(s31_radio_hw_checked, true);
	}
#ifdef CONFIG_SMP
	/* A direct S-mode CLIC source can remain pending after an SBI cache
	 * ecall because of S31's cross-privilege 0xff sentinel.  Service
	 * pending native sources at this bounded scheduling point; their
	 * direct hardware paths remain the fast path. */
	esp32s31_irq_poll();
#endif
	s31_radio_sync_irq_registrations();
	s31_radio_update_state();
	ret = s31_radio_run_linux_pass();
	if (ret) {
		pr_err("esp32s31-radio: serialized blob pass failed: %d\n", ret);
		atomic_set(&s31_radio_state, ESP32S31_RADIO_FAILED);
		s31_radio_fail_commands();
	} else {
		/* No fixed tick sleep anymore; the scheduler tick alone supplies
		 * fair preemption.  Do not force a context switch after every
		 * radio IRQ: without RX aggregation that limits receive progress
		 * to roughly one Ethernet frame per scheduling round.
		 * cond_resched() still yields to a runnable compatibility task
		 * immediately and to ordinary Linux work whenever the slice
		 * expires. */
		cond_resched();
		/* Requeue: immediately while event work is pending (the old
		 * wait_event predicate), else at the next esp_timer deadline.
		 * The compatibility timer bridge uses the same absolute
		 * monotonic deadlines as IDF, so this avoids both the old fixed
		 * polling jitter and unnecessary idle wakeups.  While STOPPING
		 * the worker runs at most this final pass: no self-requeue
		 * and no timer rearm, so shutdown's double timer cancel
		 * provably quiesces the queue.
		 */
		if (READ_ONCE(s31_radio_q_state) != S31_Q_RUNNING) {
			/* Terminal pass; fall through to balanced exit. */
		} else if ((READ_ONCE(s31_dispatch_started))) {
			/* Only Linux work belongs here. The managed task owns all
			 * vendor deadlines and rings; polling those on this higher
			 * priority worker would starve the task meant to drain them.
			 * External publications still queue this worker normally. */
			if (s31_radio_command_work_pending() ||
			    s31_radio_hci_rx_deliverable())
				s31_radio_queue();
		}
	}
	goto out;
hw_failed:
	atomic_set(&s31_radio_state, ESP32S31_RADIO_FAILED);
	s31_radio_fail_commands();
	WRITE_ONCE(s31_radio_init_dead, true);
out:
	WRITE_ONCE(s31_radio_runner, NULL);
}
#endif

int s31_radio_runtime_init(bool enable_wifi, bool enable_bt)
{
	int ret;

	if (esp32s31_radio_is_disabled()) {

		return 0;
	}

	if (s31_irq_priority_lease)
		return -EBUSY;

	if (s31_irq_background_level > 6)
		return -EINVAL;
#ifdef CONFIG_ESP32S31_RADIO_BLOBS

#endif

	s31_hci_rx_slots = S31_HCI_RX_SLOTS_DEFAULT;
	s31_hci_tx_slots = S31_HCI_TX_SLOTS_DEFAULT;
	if (enable_bt && !enable_wifi) {
		unsigned int rx = s31_radio_bt_hci_rx_slots;
		unsigned int tx = s31_radio_bt_hci_tx_slots;

		if ((rx != 8 && rx != 16 && rx != 32) ||
		    (tx != 4 && tx != 8 && tx != 16))
			return -EINVAL;
		s31_hci_rx_slots = rx;
		s31_hci_tx_slots = tx;
	}

	/* Reload within one module lifetime must not reuse indices or pending
	 * events referencing the previous payload heap.  Frontends are detached. */
	s31_hci_rx_head = s31_hci_rx_tail = 0;
	s31_hci_tx_head = s31_hci_tx_tail = 0;
	s31_hci_tx_room_ready = false;
	s31_wifi_rx_head = s31_wifi_rx_tail = 0;
	s31_wifi_tx_head = s31_wifi_tx_tail = 0;
 s31_wifi_tx_space_waiting = s31_wifi_tx_space_ready = false;
 atomic_set(&s31_tx_credit_wait, 0);
 WRITE_ONCE(s31_tx_attempt_owner, NULL);
 WRITE_ONCE(s31_tx_pool_returns, NULL);
 WRITE_ONCE(s31_tx_pool_wait_generation, 0);
 for (ret = 0; ret < S31_TX_CREDIT_COUNT; ret++) {
  atomic_set(&s31_tx_credit_sequence[ret], 0);

 }

	s31_wifi_scan_inflight = s31_wifi_scan_ready = false;
	s31_wifi_connected_ready = s31_wifi_disconnected_ready = false;
	if (WARN_ON(s31_wifi_control_started || s31_wifi_control_thread ||
	            s31_wifi_control_sync)) return -EBUSY;
	s31_wifi_control_poisoned = false;

	atomic_set(&s31_wifi_control_pending, 0);

	atomic_set(&s31_wifi_rx_direct_pending, 0);
	WRITE_ONCE(s31_radio_hw_checked, false);
	WRITE_ONCE(s31_radio_init_dead, false);
	/* A warm runtime must wait for its own init reports, not retained
	 * success from the retired heap/controller generation. */
	WRITE_ONCE(s31_wifi_init_result, -EINPROGRESS);
	WRITE_ONCE(s31_bt_init_result, -EINPROGRESS);
	/* All producers and callers of the old generation are quiescent. */
	s31_bt_enable_inflight = false;
	WRITE_ONCE(s31_bt_enable_result, -EINPROGRESS);
	WRITE_ONCE(s31_bt_disable_result, -EINPROGRESS);
	atomic_set(&s31_radio_state, ESP32S31_RADIO_STARTING);
	if ((((s31_linux_dispatch_validate()))))
		return -EINVAL;
	if (WARN_ON(s31_dispatch_inflight || !list_empty(&s31_dispatch_commands) ||
                    !list_empty(&s31_dispatch_completions)))
		return -EBUSY;
	WRITE_ONCE(s31_dispatch_started,false);
	s31_dispatch_sync=NULL;

	s31_radio_features = (enable_wifi ? S31_RADIO_FEATURE_WIFI : 0) |
		(enable_bt ? S31_RADIO_FEATURE_BLUETOOTH : 0) |
		(enable_wifi && !enable_bt ? S31_RADIO_FEATURE_RX_LEAN16 : 0);

	atomic_set(&s31_radio_immediate_pending, 0);
	WRITE_ONCE(s31_hci_tx_blocked_on_credit, false);
	WRITE_ONCE(s31_wifi_init_result, enable_wifi ? -EINPROGRESS : 0);
	WRITE_ONCE(s31_bt_init_result, enable_bt ? -EINPROGRESS : 0);

#ifdef CONFIG_ESP32S31_RADIO_BLOBS
	#ifdef MODULE
	/* A loadable radio bundle cannot use the vmlinux XIP linker script.  Its
	 * compatibility objects are relocated by the module loader, while the
	 * controller heap still uses loader-reserved, cache-independent SRAM. */
	s31_radio_heap_base = S31_RADIO_HEAP_BASE;
	#else
	/* These sections have an XIP load image but execute/read from HP SRAM.
	 * Linux's generic data copier does not cover the custom low VMA. */
	memcpy(__s31_radio_data_start, __s31_radio_data_load,
	       __s31_radio_data_end - __s31_radio_data_start);
	memset(__s31_radio_bss_start, 0,
	       __s31_radio_bss_end - __s31_radio_bss_start);
	s31_radio_heap_base = ALIGN((unsigned long)__s31_radio_static_end, 64);
	#endif
	/* The image always executes its Wi-Fi hot code in this SRAM tail. */
	if (!s31_radio_fw_wifi_iram_size())
		return -ENOEXEC;
	s31_radio_heap_end = S31_RADIO_HEAP_WIFI_END;
	s31_radio_heap_tail_base = ALIGN(S31_RADIO_HEAP_WIFI_END +
					s31_radio_fw_wifi_iram_size(), 64);
	if (s31_radio_heap_tail_base > S31_RADIO_HEAP_END)
		return -ENOMEM;
	if (s31_radio_heap_base >= s31_radio_heap_end)
		return -ENOMEM;
	s31_radio_heap_size = s31_radio_heap_end - s31_radio_heap_base;
	s31_radio_heap_linux = memremap(s31_radio_heap_base,
					S31_RADIO_HEAP_END - s31_radio_heap_base,
					MEMREMAP_WB);
	if (!s31_radio_heap_linux)
		return -ENOMEM;
	s31_radio_heap_low_base = s31_radio_low_heap_start();
	s31_radio_heap_low_size = S31_RADIO_HEAP_LOW_END - s31_radio_heap_low_base;
	s31_radio_heap_low_linux = s31_radio_heap_low_size ?
		memremap(s31_radio_heap_low_base, s31_radio_heap_low_size, MEMREMAP_WB) : NULL;
	if (s31_radio_heap_low_size && !s31_radio_heap_low_linux)
		pr_warn("esp32s31-radio: cannot map low heap chunk\n");
	s31_radio_heap2_linux = memremap(S31_RADIO_HEAP2_BASE,
					 S31_RADIO_HEAP2_SIZE, MEMREMAP_WB);
	if (!s31_radio_heap2_linux)
		pr_warn("esp32s31-radio: cannot map heap2 chunk\n");

	s31_radio_heap_pool = gen_pool_create(S31_RADIO_HEAP_ALLOC_ORDER, -1);
	if (!s31_radio_heap_pool) {
		ret = -ENOMEM;
		goto unmap_heaps;
	}
	/* Wi-Fi esf_bufs (~1848 bytes), BT controller objects and small RTOS
	 * bridge allocations have very different lifetimes.  First-fit leaves
	 * the three non-contiguous SRAM chunks peppered with holes and can reject
	 * an RX buffer while tens of KiB remain free.  Best-fit preserves the
	 * larger extents needed by the RX pool during sustained traffic. */
	gen_pool_set_algo(s31_radio_heap_pool,
                  s31_heap_first_fit, NULL);
	ret = gen_pool_add(s31_radio_heap_pool, s31_radio_heap_base,
			   s31_radio_heap_size, -1);
	if (ret) {
		gen_pool_destroy(s31_radio_heap_pool);
		s31_radio_heap_pool = NULL;
		goto unmap_heaps;
	}
	#ifndef MODULE

	#else

	#endif

	/* These rings are accessed from callbacks executing inside the blob gate.
	 * Keep both sides in the same internal-SRAM ownership domain as the blob. */
	if (enable_bt) {
		s31_hci_rx = s31_radio_sram_alloc(sizeof(*s31_hci_rx) *
						 s31_hci_rx_slots);
		s31_hci_tx = s31_radio_sram_alloc(sizeof(*s31_hci_tx) *
						 s31_hci_tx_slots);
	}
	if (enable_wifi) {
		/* RX is copied directly into frontend-owned skbs before callback
		 * return. No unused Ethernet payload ring is reserved in SRAM. */
		s31_wifi_tx = s31_radio_sram_alloc(sizeof(*s31_wifi_tx) *
						  S31_WIFI_TX_SLOTS);
		if (s31_wifi_tx)
			memset(s31_wifi_tx, 0, sizeof(*s31_wifi_tx) * S31_WIFI_TX_SLOTS);
		s31_wifi_connect_params =
			s31_radio_sram_alloc(sizeof(*s31_wifi_connect_params));
		s31_wifi_disconnect_reason =
			s31_radio_sram_alloc(sizeof(*s31_wifi_disconnect_reason));
		s31_wifi_control_buffer = s31_radio_sram_alloc(sizeof(*s31_wifi_control_buffer));
	}
	if ((enable_bt && (!s31_hci_rx || !s31_hci_tx)) ||
	    (enable_wifi && (!s31_wifi_tx || !s31_wifi_connect_params ||
			     !s31_wifi_disconnect_reason || !s31_wifi_control_buffer))) {
		pr_err("esp32s31-radio: cannot allocate blob buffers in SRAM\n");
		ret = -ENOMEM;
		goto free_rings;
	}

	/* No controller producer exists until runtime initialization below. */

	s31_hci_rx_accepting = enable_bt;
	/*
	 * Reclaim the parked factory app's idle upper-heap chunk as a second
	 * blob-heap chunk.  It must be added only now: the Linux rings above
	 * were just carved from the main chunk, so s31_radio_sram_linux_alias()
	 * will never be handed a pointer that lives in this non-contiguous
	 * chunk.  The blob's own heap_caps allocations touch it directly
	 * through the init_mm identity mapping instead.
	 */
	ret = gen_pool_add(s31_radio_heap_pool, S31_RADIO_HEAP2_BASE,
			   S31_RADIO_HEAP2_SIZE, -1);
	if (ret) {
		pr_warn("esp32s31-radio: cannot add SRAM heap2 %#lx..%#lx: %d\n",
			S31_RADIO_HEAP2_BASE, S31_RADIO_HEAP2_END, ret);
	} else {

	}
	/* Reuse only bytes after the executable image, never the code itself. */
	if (s31_radio_heap_tail_base < S31_RADIO_HEAP_END) {
		ret = gen_pool_add(s31_radio_heap_pool, s31_radio_heap_tail_base,
				   S31_RADIO_HEAP_END - s31_radio_heap_tail_base, -1);
		if (ret)
			goto free_rings;
	}
	/* Preserve the running OpenSBI heap, including dynamically allocated
	 * SSE state. The tail is added after allocating the Linux rings. */
	if (s31_radio_heap_low_linux) {
		ret = gen_pool_add(s31_radio_heap_pool, s31_radio_heap_low_base,
				   s31_radio_heap_low_size, -1);
		if (ret)
			pr_warn("esp32s31-radio: cannot add SRAM heap-low: %d\n", ret);

	}
#endif

#ifdef CONFIG_ESP32S31_RADIO_BLOBS
	/* Acquire only after validation and mappings, before any source/task
	 * publication. Every later failure goes through free_rings. */
	{
		s31_irq_priority_lease = esp_clic_priority_acquire(
			ESP32S31_CLIC_PRIO(s31_irq_background_level));
		if (IS_ERR(s31_irq_priority_lease)) {
			ret = PTR_ERR(s31_irq_priority_lease);
			s31_irq_priority_lease = NULL;
			goto free_rings;
		}

	}
#endif

#ifdef CONFIG_ESP32S31_RADIO_BLOBS
	/* Private Linux executor: policy and lifetime belong to this runtime.
	 * The one-shot hrtimer remains the only deadline source. The first pass
	 * performs the identity check with its persistent init_mm borrow.
	 */
	atomic_set(&s31_radio_state, ESP32S31_RADIO_STARTING);

	/* TIMG1 hardirq tick removed: use Linux jiffies as the FreeRTOS/esp_timer
	 * time base.  Blocking already uses Linux sleep/wait; only xTaskGetTickCount
	 * and esp_timer_get_time need a monotonic 10 ms tick, which jiffies gives.
	 */

 s31_native_irq_allocation = s31_radio_sram_stack_alloc(IRQ_STACK_SIZE);
 if (!s31_native_irq_allocation) {
  ret = -ENOMEM;
  goto free_rings;
 }
 s31_native_irq_stack = s31_native_irq_allocation;
 s31_native_irq_installed = false;
	s31_radio_worker = s31_radio_service_create(0);
	if (IS_ERR(s31_radio_worker)) {
		ret = PTR_ERR(s31_radio_worker);
		s31_radio_worker = NULL;
		goto free_rings;
	}

	kthread_init_work(&s31_radio_work, s31_radio_workfn);

	/* The radio-init task is created by the first serialized pass, which
	 * runs on the private Linux worker with its lifetime init_mm borrow. Creating
	 * it here in
	 * module-load process context would expose payload task APIs
	 * (kthread_data and friends) to a non-kthread caller.  Init only
	 * queues the first pass and returns; wait_ready() observes STARTING
	 * until the pass reports READY/FAILED, unchanged from before. */
	ret = s31_linux_tasks_runtime_open();
	if (ret) {
		s31_radio_sram_stack_free(s31_native_irq_allocation, IRQ_STACK_SIZE);
		s31_native_irq_allocation = NULL;
		s31_native_irq_stack = NULL;
		s31_radio_service_discard();
		goto free_rings;
	}

	s31_blob_fp_switch_hook = s31_blob_fp_context_switch;
	spin_lock_irq(&s31_radio_q_lock);
	s31_radio_q_state = S31_Q_RUNNING;
	kthread_queue_work(s31_radio_worker, &s31_radio_work);
	spin_unlock_irq(&s31_radio_q_lock);
	return 0;
#else
	return 0;
#endif

#ifdef CONFIG_ESP32S31_RADIO_BLOBS
free_rings:
 s31_radio_sram_stack_free(s31_native_irq_allocation, IRQ_STACK_SIZE);
 s31_native_irq_allocation = NULL;
 s31_native_irq_stack = NULL;
	s31_radio_irq_priority_restore();
	s31_radio_sram_free(s31_wifi_connect_params);
	s31_radio_sram_free(s31_wifi_scan_params);
	s31_wifi_scan_params = NULL;
	s31_radio_sram_free(s31_wifi_disconnect_reason);
	s31_radio_sram_free(s31_wifi_control_buffer);
	s31_wifi_connect_params = NULL;
	s31_wifi_disconnect_reason = NULL;
	s31_wifi_control_buffer = NULL;
	s31_radio_sram_free(s31_wifi_rx_data);
	s31_radio_wifi_tx_release();
	s31_radio_sram_free(s31_wifi_tx);
	s31_radio_sram_free(s31_wifi_rx);
	s31_radio_sram_free(s31_hci_tx);
	s31_radio_sram_free(s31_hci_rx);
	s31_wifi_tx = NULL;
	s31_wifi_rx = NULL;
	s31_wifi_rx_data = NULL;
	s31_hci_tx = NULL;
	s31_hci_rx = NULL;
 s31_idf_rx_cache_flush();
	gen_pool_destroy(s31_radio_heap_pool);
	s31_radio_heap_pool = NULL;
unmap_heaps:

	if (s31_radio_heap2_linux) {
		memunmap(s31_radio_heap2_linux);
		s31_radio_heap2_linux = NULL;
	}
	if (s31_radio_heap_low_linux) {
		memunmap(s31_radio_heap_low_linux);
		s31_radio_heap_low_linux = NULL;
	}
	memunmap(s31_radio_heap_linux);
	s31_radio_heap_linux = NULL;
	return ret;
#else
free_rings:
	return ret;
#endif
}

int s31_radio_runtime_wait_ready(void)
{
	unsigned long timeout = jiffies + 10 * HZ;

	do {
		int state = atomic_read(&s31_radio_state);
		int bt = READ_ONCE(s31_bt_init_result);

		if (state == ESP32S31_RADIO_FAILED ||
		    (bt != -EINPROGRESS && bt))
			return -EIO;
		if (state == ESP32S31_RADIO_READY && !bt)
			return 0;
		msleep(20);
	} while (time_before(jiffies, timeout));
	return -ETIMEDOUT;
}

int s31_radio_runtime_shutdown(void)
{
#ifdef CONFIG_ESP32S31_RADIO_BLOBS
	struct s31_radio_command shutdown;
	struct kthread_worker *worker;
	int shutdown_ret = 0;
	int i;
	unsigned long flags;
	cpumask_var_t saved_mask;
	int pin_ret;
	bool resume_admission = false;

	/* Blob state is strictly core0 bound (see s31_linux_blob_enter()).
	 * This runs in the caller's process context (rmmod, suspend), which
	 * the scheduler may place on CPU1. Pin to CPU0 for the whole
	 * shutdown and restore the caller's affinity afterwards, so no
	 * teardown subpath can enter blob code off core0. */
	if (!alloc_cpumask_var(&saved_mask, GFP_KERNEL))
		return -ENOMEM;
	cpumask_copy(saved_mask, current->cpus_ptr);
	/* Stop periodic/immediate self-requeue before migrating this ordinary
	 * caller onto CPU0. An always-runnable RT service can otherwise starve
	 * the caller at set_cpus_allowed_ptr(), before it closes admission.
	 * Keep terminal SHUTDOWN admission and all payload owners alive. */
	spin_lock_irqsave(&s31_radio_q_lock, flags);
	if (s31_radio_q_state == S31_Q_RUNNING) {
		s31_radio_q_state = S31_Q_STOPPING;
		resume_admission = true;
	}
	spin_unlock_irqrestore(&s31_radio_q_lock, flags);
	wake_up_all(&s31_radio_bt_enable_waitq);
	/* Publish runtime wait cancellation before submitting the terminal
	 * SHUTDOWN command, so admitted blocking waits unwind instead of
	 * stalling the teardown that needs their owner to make progress. */

	pin_ret = set_cpus_allowed_ptr(current, cpumask_of(0));
	if (pin_ret) {
		pr_err("esp32s31-radio: shutdown cannot pin to core0: %d\n",
		       pin_ret);
		shutdown_ret = pin_ret;
		goto out_restore;
	}
	resume_admission = false; /* Teardown now owns the closed queue. */
	/* Also covers teardown without a registered frontend. */
	s31_radio_hci_rx_quiesce();
	/* TLS destructors may own Wi-Fi thread semaphores. Join their native
	 * owner before vendor stop/deinit, while its PP/IRQ dependencies live. */
	s31_wifi_control_owner_stop();

	/* Block new business first, while keeping the executor, IRQ paths
	 * and compatibility tasks alive for the payload shutdown below.
	 * From here queue()/timer/submit refuse everything except the
	 * terminal SHUTDOWN, and the worker performs at most one final
	 * pass with no requeue or timer rearm.  No lock is held across
	 * the waits below, so callbacks never block on us.
	 */

	if (((READ_ONCE(s31_radio_worker))) &&
	    atomic_read(&s31_radio_state) == ESP32S31_RADIO_READY) {
		reinit_completion(&s31_shutdown_done);
		WRITE_ONCE(s31_shutdown_result, -EINPROGRESS);
		INIT_LIST_HEAD(&shutdown.node);
		init_completion(&shutdown.done);
		shutdown.type = S31_RADIO_COMMAND_SHUTDOWN;
		shutdown.result = -EINPROGRESS;
		s31_radio_submit_command(&shutdown);
		wait_for_completion(&shutdown.done);
		if (shutdown.result)
			shutdown_ret = shutdown.result;
		else if (!wait_for_completion_timeout(&s31_shutdown_done, 5 * HZ))
			shutdown_ret = -ETIMEDOUT;
		else
			shutdown_ret = READ_ONCE(s31_shutdown_result);
		if (shutdown_ret)
			pr_warn("esp32s31-radio: payload shutdown returned %d\n",
				shutdown_ret);
	}
	/* Close the terminal-command exception before the last list drain. */
	spin_lock_irqsave(&s31_radio_q_lock, flags);
	atomic_set(&s31_radio_state, ESP32S31_RADIO_OFFLINE);
	spin_unlock_irqrestore(&s31_radio_q_lock, flags);
	if (s31_dispatch_sync) s31_linux_sync_wake(s31_dispatch_sync);
	s31_radio_fail_commands();

	/* Task teardown still needs the loaded payload and SRAM allocator. */
	s31_linux_tasks_stop_all();
	/* Unpublish before joining the control work so late IRQ producers see
	 * STOPPING/NULL and cannot retain the executor beyond its lifetime. */
	spin_lock_irqsave(&s31_radio_q_lock, flags);
	worker = READ_ONCE(s31_radio_worker);
	WRITE_ONCE(s31_radio_worker, NULL);
	spin_unlock_irqrestore(&s31_radio_q_lock, flags);
	if (worker) {
		{
			kthread_cancel_work_sync(&s31_radio_work);
		}
		kthread_destroy_worker(worker);
	}
	spin_lock_irqsave(&s31_radio_q_lock, flags);
	s31_radio_q_state = S31_Q_OFF;
	spin_unlock_irqrestore(&s31_radio_q_lock, flags);
	WRITE_ONCE(s31_radio_runner, NULL);
	s31_dispatch_cancel_commands(true);

	s31_foreign_owners_close();
	s31_blob_fp_switch_hook = NULL;

	/* Close every action before retiring any source, so a shared sibling
	 * cannot re-enable hardware during the terminal drain. */
	raw_spin_lock_irqsave(&s31_irq_registry_lock, flags);
	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++) {
		if (!s31_idf_irqs[i].allocated) continue;
		s31_idf_irqs[i].enabled = false;
		s31_idf_irqs[i].free_pending = true;
	}
	raw_spin_unlock_irqrestore(&s31_irq_registry_lock, flags);
	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++)
		if (s31_idf_irqs[i].allocated)
			s31_radio_sync_one_irq(&s31_idf_irqs[i]);

	/* Keep the native task reference until every IRQ wake has finished. */

	atomic_set(&s31_radio_pending_isrs, 0);
	/* Managed exit work, foreign finalizers and all generic IRQ actions have
	 * completed. The kernel domain refuses destruction if any lease remains. */
	s31_linux_tasks_runtime_close();

 /* All hardware actions were joined by free_irq above; no callback can still
  * hold this stack or obtain it after runtime shutdown. */
 if (READ_ONCE(s31_native_irq_installed)) {
  esp32s31_radio_irq_stack_restore(s31_native_irq_stack);
  WRITE_ONCE(s31_native_irq_installed, false);

 }
 s31_radio_sram_stack_free(s31_native_irq_allocation, IRQ_STACK_SIZE);
 s31_native_irq_allocation = NULL;
 s31_native_irq_stack = NULL;
	/* Managed tasks and the Linux worker are joined; no dispatcher wait or
	 * producer can retain this sync object past the heap teardown. */
	if (s31_dispatch_sync) s31_linux_dispatch_sync_destroy(s31_dispatch_sync);
	s31_dispatch_sync=NULL;
	WRITE_ONCE(s31_dispatch_started,false);
	if (s31_wifi_control_sync)
		s31_linux_dispatch_sync_destroy(s31_wifi_control_sync);
	s31_wifi_control_sync = NULL;
	s31_radio_sram_free(s31_wifi_disconnect_reason);
	s31_radio_sram_free(s31_wifi_control_buffer);
	s31_wifi_control_buffer = NULL;
	s31_radio_sram_free(s31_wifi_connect_params);
	s31_radio_sram_free(s31_wifi_scan_params);
	s31_wifi_scan_params = NULL;
	s31_radio_sram_free(s31_wifi_rx_data);
	s31_radio_wifi_tx_release();
	s31_radio_sram_free(s31_wifi_tx);
	s31_radio_sram_free(s31_wifi_rx);
	s31_radio_sram_free(s31_hci_tx);
	s31_radio_sram_free(s31_hci_rx);
	s31_wifi_disconnect_reason = NULL;
	s31_wifi_connect_params = NULL;
	s31_wifi_rx_data = NULL;
	s31_wifi_tx = NULL;
	s31_wifi_rx = NULL;
	s31_hci_tx = NULL;
	s31_hci_rx = NULL;
	s31_radio_heap_destroy();
	if (s31_radio_heap2_linux) {
		memunmap(s31_radio_heap2_linux);
		s31_radio_heap2_linux = NULL;
	}
	if (s31_radio_heap_low_linux) {
		memunmap(s31_radio_heap_low_linux);
		s31_radio_heap_low_linux = NULL;
	}
	if (s31_radio_heap_linux) {
		memunmap(s31_radio_heap_linux);
		s31_radio_heap_linux = NULL;
	}
	/* All radio actions and executors have been joined; no owned source
	 * can change its CLIC priority after the lease is returned. */
	s31_radio_irq_priority_restore();
out_restore:
	/* Restore the caller's scheduler placement before returning to it. */
	set_cpus_allowed_ptr(current, saved_mask);
	free_cpumask_var(saved_mask);
	if (resume_admission) {
		spin_lock_irqsave(&s31_radio_q_lock, flags);
		if (s31_radio_q_state == S31_Q_STOPPING)
			s31_radio_q_state = S31_Q_RUNNING;
		spin_unlock_irqrestore(&s31_radio_q_lock, flags);
		/* A pass may have observed STOPPING and skipped its timer rearm. */
		__s31_radio_queue_force();
	}
	return shutdown_ret;
#else
	return -EOPNOTSUPP;
#endif
}
