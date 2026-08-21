// SPDX-License-Identifier: GPL-2.0
/* ESP32-S31 S-mode Wi-Fi/Bluetooth runtime. */

#include <linux/init.h>
#include <linux/completion.h>
#include <linux/if_ether.h>
#include <linux/genalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irqdomain.h>
#include <linux/jiffies.h>
#include <linux/kallsyms.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/math64.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/thread_info.h>
#include <linux/timekeeping.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/esp32s31-radio.h>
#include <asm/csr.h>
#include <asm/fpu.h>
#include <asm/irq_regs.h>
#include <asm/ptrace.h>

#include "esp32s31-radio-internal.h"

#define S31_ROM_BASE		0x2f800000UL
#define S31_ROM_CPU_FREQ	0x2f800040UL

typedef unsigned int (*s31_rom_cpu_freq_t)(void);

static bool s31_radio_disabled;

static int __init s31_radio_disable_setup(char *value)
{
	if (value && !strcmp(value, "off"))
		s31_radio_disabled = true;

	return 0;
}
early_param("esp32s31_radio", s31_radio_disable_setup);

bool esp32s31_radio_is_disabled(void)
{
	return s31_radio_disabled;
}

#ifdef CONFIG_ESP32S31_RADIO_BLOBS
/* The loader carves out 0x2f030000..0x2f072380 exclusively for Linux radio.
 * The linked blob data/BSS occupies the beginning. Keep the final 0xb80 bytes
 * for the synchronous-trap stack; descriptors begin at 0x2f072380. */
#define S31_RADIO_HEAP_END	0x2f071800UL
#define S31_RADIO_EXC_STACK_TOP	0x2f072360UL
/* Idle SRAM above the DMA reservations (ROM download-mode buffers tail +
 * PRO-CPU startup stack), reclaimed as a second blob heap. */
#define S31_RADIO_HEAP2_BASE	0x2f078c00UL
#define S31_RADIO_HEAP2_END	0x2f07cfb0UL
#define S31_RADIO_HEAP2_SIZE	(S31_RADIO_HEAP2_END - S31_RADIO_HEAP2_BASE)
/* Parked factory-app FreeRTOS heap, reserved and cleared by the loader. */
#define S31_RADIO_HEAP_LOW_BASE	0x2f018000UL
#define S31_RADIO_HEAP_LOW_END	0x2f030000UL
#define S31_RADIO_HEAP_LOW_SIZE	(S31_RADIO_HEAP_LOW_END - S31_RADIO_HEAP_LOW_BASE)

extern char __s31_radio_data_start[];
extern char __s31_radio_data_end[];
extern char __s31_radio_data_load[];
extern char __s31_radio_bss_start[];
extern char __s31_radio_bss_end[];
extern char __s31_radio_static_end[];

static struct gen_pool *s31_radio_heap_pool;
static size_t s31_radio_heap_peak;
static void *s31_radio_heap_linux;
static void *s31_radio_heap_low_linux;
static void *s31_radio_heap2_linux;
static unsigned long s31_radio_heap_base;
static size_t s31_radio_heap_size;

static void *s31_radio_sram_linux_alias(const void *ptr)
{
	unsigned long p = (unsigned long)ptr;

	if (p >= S31_RADIO_HEAP_LOW_BASE && p < S31_RADIO_HEAP_LOW_END)
		return s31_radio_heap_low_linux + (p - S31_RADIO_HEAP_LOW_BASE);
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

#define S31_IDF_ALLOC_MAGIC	0x5333414c

/* Live-allocation histogram for the internal SRAM heap.  Bucketed by request
 * size so a periodic report can show exactly which buffer classes consume the
 * bounded pool (RX esf_buf ~1848, TX ~1600, task stacks, queues, ...). */
struct s31_heap_hist_bucket {
	u32 live;
	u32 peak;
	size_t bytes;
};
#define S31_HEAP_HIST_BUCKETS 14
static struct s31_heap_hist_bucket s31_heap_hist[S31_HEAP_HIST_BUCKETS];
static atomic_t s31_idf_alloc_failures = ATOMIC_INIT(0);
static const size_t s31_heap_hist_max[S31_HEAP_HIST_BUCKETS] = {
	64, 128, 256, 512, 1024, 1536, 1600, 1848, 2048, 4096,
	8192, 16384, 65536, SIZE_MAX
};

static int s31_heap_hist_index(size_t size)
{
	int i;

	for (i = 0; i < S31_HEAP_HIST_BUCKETS - 1; i++)
		if (size <= s31_heap_hist_max[i])
			return i;
	return S31_HEAP_HIST_BUCKETS - 1;
}

static void *s31_idf_alloc(size_t size, size_t alignment, bool zero, u32 caps)
{
	struct s31_idf_alloc_header *header;
	void *base, *ptr;
	size_t total;

	(void)caps;

	if (alignment < sizeof(void *))
		alignment = sizeof(void *);
	if (!is_power_of_2(alignment) || size > SIZE_MAX - alignment - sizeof(*header))
		return NULL;
	total = size + alignment - 1 + sizeof(*header);
	base = (void *)gen_pool_alloc(s31_radio_heap_pool, total);
	if (!base)
		return NULL;
	s31_radio_heap_peak = max(s31_radio_heap_peak,
		gen_pool_size(s31_radio_heap_pool) -
			gen_pool_avail(s31_radio_heap_pool));
	ptr = PTR_ALIGN((u8 *)base + sizeof(*header), alignment);
	header = ptr - sizeof(*header);
	header->magic = S31_IDF_ALLOC_MAGIC;
	header->size = size;
	header->total = total;
	header->base = base;
	if (zero)
		memset(ptr, 0, size);
	{
		int b = s31_heap_hist_index(size);

		s31_heap_hist[b].live++;
		s31_heap_hist[b].peak = max(s31_heap_hist[b].peak,
					    s31_heap_hist[b].live);
		s31_heap_hist[b].bytes += size;
	}
	return ptr;
}

void *__wrap_heap_caps_malloc(size_t size, u32 caps)
{
	void *ptr = s31_idf_alloc(size, sizeof(void *), false, caps);

	if (!ptr) {
		/* An RX-buffer shortfall during an AMPDU burst emits one of
		 * these per frame.  printk flushes the UART0 DMA console
		 * synchronously from inside the blob gate, so an unbounded
		 * burst stalls the Wi-Fi task behind 115200-baud output and
		 * hangs the download.  Rate-limit it and keep a running
		 * fail counter instead of a per-frame console write. */
		atomic_inc(&s31_idf_alloc_failures);
		pr_err_ratelimited("esp32s31-radio: heap malloc size=%zu caps=%#x failed\n",
				   size, caps);
		if (!(atomic_read(&s31_idf_alloc_failures) & 0xff))
			s31_radio_heap_report("malloc-fail");
	}
	return ptr;
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
	{
		int b = s31_heap_hist_index(header->size);

		if (s31_heap_hist[b].live)
			s31_heap_hist[b].live--;
		s31_heap_hist[b].bytes -= min(s31_heap_hist[b].bytes,
					      header->size);
	}
	gen_pool_free(s31_radio_heap_pool, (unsigned long)header->base,
		      header->total);
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

/* Wi-Fi RX/TX drop counters, read by the heap report for stall diagnostics. */
static atomic_t s31_wifi_rx_dropped = ATOMIC_INIT(0);
static atomic_t s31_wifi_tx_dropped = ATOMIC_INIT(0);
static atomic_t s31_wifi_rx_delivered = ATOMIC_INIT(0);
static atomic_t s31_wifi_rx_freed = ATOMIC_INIT(0);
static atomic_t s31_wifi_tx_done = ATOMIC_INIT(0);
static atomic_t s31_wifi_rx_cb = ATOMIC_INIT(0);

void s31_radio_heap_report(const char *stage)
{
	size_t free = gen_pool_avail(s31_radio_heap_pool);
	size_t total = gen_pool_size(s31_radio_heap_pool);
	int i;

	pr_info("esp32s31-radio: SRAM heap %s used=%zu peak=%zu free=%zu total=%zu allocfail=%d rx_drop=%d tx_drop=%d rx_del=%d rx_freed=%d tx_done=%d rx_cb=%d\n",
		stage, total - free, s31_radio_heap_peak, free, total,
		atomic_read(&s31_idf_alloc_failures),
		atomic_read(&s31_wifi_rx_dropped),
		atomic_read(&s31_wifi_tx_dropped),
		atomic_read(&s31_wifi_rx_delivered),
		atomic_read(&s31_wifi_rx_freed),
		atomic_read(&s31_wifi_tx_done),
		atomic_read(&s31_wifi_rx_cb));
	if (stage[0] == 'p') {	/* periodic */
		for (i = 0; i < S31_HEAP_HIST_BUCKETS; i++) {
			if (!s31_heap_hist[i].live && !s31_heap_hist[i].peak)
				continue;
			pr_info("esp32s31-radio:   heap[%d] <=%zu live=%u peak=%u bytes=%zu\n",
				i, s31_heap_hist_max[i], s31_heap_hist[i].live,
				s31_heap_hist[i].peak, s31_heap_hist[i].bytes);
		}
	}
}

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

	header = (struct s31_sram_alloc_header *)gen_pool_alloc(s31_radio_heap_pool,
								size + sizeof(*header));
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
	gen_pool_free(s31_radio_heap_pool, (unsigned long)header,
		      header->size + sizeof(*header));
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
	return core_kernel_text(addr) || addr >= PAGE_OFFSET;
}

struct s31_idf_irq_registration {
	int source;
	unsigned int logical_intr;
	unsigned int hwirq;
	int flags;
	void (*handler)(void *);
	void *arg;
	int virq;
	bool install_pending;
	bool free_pending;
	bool enabled;
	bool hw_enabled;
	bool allocated;
	atomic_t hardirq_count;
	atomic_t callback_pending;
	atomic_t callback_count;
	atomic_t mask_count;
	u64 pending_since_ns;
	u64 latency_total_ns;
	u64 latency_max_ns;
	u64 gate_total_ns;
	u64 gate_max_ns;
	u64 handler_total_ns;
	u64 handler_max_ns;
};

#define S31_RADIO_IRQ_SLOTS	7
static const unsigned int s31_radio_hwirqs[S31_RADIO_IRQ_SLOTS] = {
	47, 45, 44, 43, 42, 41, 40,
};
static struct s31_idf_irq_registration s31_idf_irqs[S31_RADIO_IRQ_SLOTS];
static DECLARE_WAIT_QUEUE_HEAD(s31_radio_waitq);

enum s31_radio_command_type {
	S31_RADIO_COMMAND_HEALTH,
	S31_RADIO_COMMAND_WIFI_GET_MAC,
	S31_RADIO_COMMAND_WIFI_SCAN,
	S31_RADIO_COMMAND_WIFI_CONNECT,
	S31_RADIO_COMMAND_WIFI_DISCONNECT,
	S31_RADIO_COMMAND_BT_ENABLE,
	S31_RADIO_COMMAND_TASK_CREATE,
};

/* H4 type plus Linux HCI_MAX_FRAME_SIZE (1028-byte ACL header/payload). */
#define S31_HCI_FRAME_SIZE	1029
#define S31_HCI_RX_SLOTS	8
#define S31_HCI_TX_SLOTS	4

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
static DEFINE_SPINLOCK(s31_hci_lock);
static const struct esp32s31_radio_hci_ops *s31_hci_ops;
static void *s31_hci_context;
static atomic_t s31_hci_rx_dropped = ATOMIC_INIT(0);
static atomic_t s31_hci_tx_dropped = ATOMIC_INIT(0);
static const struct esp32s31_radio_wifi_ops *s31_wifi_ops;
static void *s31_wifi_context;
static struct esp32s31_radio_wifi_ap
	s31_wifi_scan_aps[ESP32S31_RADIO_WIFI_MAX_APS];
static u16 s31_wifi_scan_count;
static int s31_wifi_scan_status;
static bool s31_wifi_scan_inflight;
static bool s31_wifi_scan_ready;

#define S31_WIFI_FRAME_SIZE	1600
/* The IDF driver already owns its packet buffering.  These are only short
 * staging queues between serialized blob callbacks and the Linux net stack.
 * RX is now a zero-copy descriptor ring (12 B/slot), so it can be made deep
 * enough to absorb a full rx_ba_win=64 AMPDU burst without the old 25.6 KiB
 * data-ring cost. */
#define S31_WIFI_RX_SLOTS	128
/* 48 slots is needed for BT+WiFi on a busy 2.4G channel: the Griefer ACK
 * burst filled the 32-slot ring and dropped TCP ACKs.  The blob heap still
 * has ~95 KiB free with dynamic RX capped at 32. */
#define S31_WIFI_TX_SLOTS	48

struct s31_wifi_frame {
	u16 length;
	u64 enqueue_ns;
	u8 data[S31_WIFI_FRAME_SIZE];
};

/* Zero-copy RX descriptor: the blob hands over its esf_buf instead of copying
 * the frame into the ring.  `data` is the esf_buf payload pointer (HP-SRAM
 * physical), `eb` is the esf_buf handle to recycle after the net stack has
 * consumed the frame. */
struct s31_wifi_rx_desc {
	u16 length;
	void *data;
	void *eb;
};

/* esf_buf handles awaiting recycle.  Written by the worker after the net
 * stack consumes a frame, drained inside the blob pass where the gate is
 * held (single-threaded with the MAC RX allocator).
 *
 * Must be larger than S31_WIFI_RX_SLOTS: the worker can deliver a full
 * zero-copy RX ring (128 ebs) before the next blob pass drains the recycle
 * queue.  A full recycle ring used to silently advance the head and leak the
 * queued esf_bufs; that leak exhausted the BT+WiFi heap during downloads.
 */
#define S31_WIFI_FREE_SLOTS	160
static void *s31_wifi_free_ring[S31_WIFI_FREE_SLOTS];
static unsigned int s31_wifi_free_head;
static unsigned int s31_wifi_free_tail;

struct s31_radio_timing_stats {
	u64 irq_to_worker_total_ns;
	u64 irq_to_worker_max_ns;
	u64 worker_gate_total_ns;
	u64 worker_gate_max_ns;
	u64 isr_total_ns;
	u64 isr_max_ns;
	u64 isr_to_task_total_ns;
	u64 isr_to_task_max_ns;
	u64 tx_enqueue_total_ns;
	u64 tx_enqueue_max_ns;
	u64 tx_call_total_ns;
	u64 tx_call_max_ns;
	u64 tx_done_total_ns;
	u64 tx_done_max_ns;
	u64 last_isr_end_ns;
	u32 irq_samples;
	u32 gate_samples;
	u32 isr_samples;
	u32 task_samples;
	u32 tx_samples;
	u32 tx_done_samples;
};

enum s31_tx_timing_kind {
	S31_TX_OTHER,
	S31_TX_IPV4,
	S31_TX_DHCP,
	S31_TX_ARP,
	S31_TX_IPV6,
	S31_TX_KIND_COUNT,
};

#define S31_TX_TIMING_SLOTS 32

struct s31_tx_timing_sample {
	u64 enqueue_ns;
	u64 start_ns;
	u64 return_ns;
	u64 done_ns;
	u32 sequence;
	u16 length;
	u8 kind;
	int result;
	bool returned;
	bool done;
	bool status;
};

static struct s31_tx_timing_sample s31_tx_samples[S31_TX_TIMING_SLOTS];
static u32 s31_tx_sequence;
static u32 s31_tx_head;
static u32 s31_tx_tail;
static u32 s31_tx_done_cursor;
static DEFINE_SPINLOCK(s31_tx_timing_lock);

static const char * const s31_tx_kind_name[S31_TX_KIND_COUNT] = {
	[S31_TX_OTHER] = "other",
	[S31_TX_IPV4] = "ipv4",
	[S31_TX_DHCP] = "dhcp",
	[S31_TX_ARP] = "arp",
	[S31_TX_IPV6] = "ipv6",
};

static u8 s31_radio_tx_kind(const u8 *frame, u16 length)
{
	u16 ethertype;
	u8 ihl;
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
	if (ihl < 20 || length < ETH_HLEN + ihl + 8 ||
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

static struct task_struct *s31_radio_worker;
static struct s31_radio_timing_stats s31_timing;

void s31_radio_timing_reset(void)
{
	unsigned long flags;

	memset(&s31_timing, 0, sizeof(s31_timing));
	spin_lock_irqsave(&s31_tx_timing_lock, flags);
	memset(s31_tx_samples, 0, sizeof(s31_tx_samples));
	s31_tx_head = 0;
	s31_tx_tail = 0;
	s31_tx_done_cursor = 0;
	spin_unlock_irqrestore(&s31_tx_timing_lock, flags);
}

static void s31_timing_add(u64 delta, u64 *total, u64 *maximum)
{
	*total += delta;
	*maximum = max(*maximum, delta);
}

void s31_radio_timing_blob_enter(void)
{
	u64 now, then;

	if (current == READ_ONCE(s31_radio_worker))
		return;
	then = READ_ONCE(s31_timing.last_isr_end_ns);
	if (!then)
		return;
	now = ktime_get_mono_fast_ns();
	if (now >= then) {
		s31_timing_add(now - then, &s31_timing.isr_to_task_total_ns,
			       &s31_timing.isr_to_task_max_ns);
		s31_timing.task_samples++;
	}
	WRITE_ONCE(s31_timing.last_isr_end_ns, 0);
}

u32 s31_radio_timing_tx_begin(u64 enqueue_ns, u64 start_ns,
			      const u8 *frame, u16 length)
{
	struct s31_tx_timing_sample *sample;
	unsigned long flags;
	u32 sequence;

	if (start_ns >= enqueue_ns)
		s31_timing_add(start_ns - enqueue_ns,
			       &s31_timing.tx_enqueue_total_ns,
			       &s31_timing.tx_enqueue_max_ns);
	s31_timing.tx_samples++;
	spin_lock_irqsave(&s31_tx_timing_lock, flags);
	sequence = ++s31_tx_sequence;
	if (!sequence)
		sequence = ++s31_tx_sequence;
	sample = &s31_tx_samples[s31_tx_head % S31_TX_TIMING_SLOTS];
	memset(sample, 0, sizeof(*sample));
	sample->enqueue_ns = enqueue_ns;
	sample->start_ns = start_ns;
	sample->sequence = sequence;
	sample->length = length;
	sample->kind = s31_radio_tx_kind(frame, length);
	s31_tx_head++;
	if (s31_tx_head - s31_tx_tail > S31_TX_TIMING_SLOTS)
		s31_tx_tail = s31_tx_head - S31_TX_TIMING_SLOTS;
	if (s31_tx_done_cursor < s31_tx_tail)
		s31_tx_done_cursor = s31_tx_tail;
	spin_unlock_irqrestore(&s31_tx_timing_lock, flags);
	return sequence;
}

void s31_radio_timing_tx_return(u32 sequence, u64 end_ns, int result)
{
	struct s31_tx_timing_sample *sample;
	unsigned long flags;
	u32 i;

	spin_lock_irqsave(&s31_tx_timing_lock, flags);
	for (i = s31_tx_done_cursor; i < s31_tx_head; i++) {
		sample = &s31_tx_samples[i % S31_TX_TIMING_SLOTS];
		if (sample->sequence != sequence)
			continue;
		sample->return_ns = end_ns;
		sample->result = result;
		sample->returned = true;
		if (end_ns >= sample->start_ns)
			s31_timing_add(end_ns - sample->start_ns,
				       &s31_timing.tx_call_total_ns,
				       &s31_timing.tx_call_max_ns);
		break;
	}
	spin_unlock_irqrestore(&s31_tx_timing_lock, flags);
}

void s31_radio_timing_tx_done(bool status, const u8 *frame, u16 length)
{
	struct s31_tx_timing_sample *sample = NULL;
	unsigned long flags;
	u64 now = ktime_get_mono_fast_ns();
	u32 i;

	atomic_inc(&s31_wifi_tx_done);

	spin_lock_irqsave(&s31_tx_timing_lock, flags);
	for (i = s31_tx_tail; i < s31_tx_head; i++) {
		sample = &s31_tx_samples[i % S31_TX_TIMING_SLOTS];
		if (!sample->done)
			break;
		sample = NULL;
	}
	if (sample) {
		sample->done_ns = now;
		sample->done = true;
		sample->status = status;
		if (sample->return_ns && now >= sample->return_ns) {
			s31_timing_add(now - sample->return_ns,
				       &s31_timing.tx_done_total_ns,
				       &s31_timing.tx_done_max_ns);
			s31_timing.tx_done_samples++;
		}
		s31_tx_done_cursor = i + 1;
	}
	spin_unlock_irqrestore(&s31_tx_timing_lock, flags);

	(void)frame;
	(void)length;
}

static void s31_radio_timing_report(const char *stage)
{
	unsigned long flags;
	u32 i;
#define S31_AVG(total, count) ((count) ? div_u64((total), (count)) : 0)
	pr_info("esp32s31-radio: timing %s irq->worker n=%u avg=%lluns max=%lluns; gate n=%u avg=%lluns max=%lluns; isr n=%u avg=%lluns max=%lluns\n",
		stage, s31_timing.irq_samples,
		S31_AVG(s31_timing.irq_to_worker_total_ns, s31_timing.irq_samples),
		s31_timing.irq_to_worker_max_ns, s31_timing.gate_samples,
		S31_AVG(s31_timing.worker_gate_total_ns, s31_timing.gate_samples),
		s31_timing.worker_gate_max_ns, s31_timing.isr_samples,
		S31_AVG(s31_timing.isr_total_ns, s31_timing.isr_samples),
		s31_timing.isr_max_ns);
	pr_info("esp32s31-radio: timing %s isr->task n=%u avg=%lluns max=%lluns; tx enqueue->call n=%u avg=%lluns max=%lluns; call avg=%lluns max=%lluns; done n=%u avg=%lluns max=%lluns\n",
		stage, s31_timing.task_samples,
		S31_AVG(s31_timing.isr_to_task_total_ns, s31_timing.task_samples),
		s31_timing.isr_to_task_max_ns, s31_timing.tx_samples,
		S31_AVG(s31_timing.tx_enqueue_total_ns, s31_timing.tx_samples),
		s31_timing.tx_enqueue_max_ns,
		S31_AVG(s31_timing.tx_call_total_ns, s31_timing.tx_samples),
		s31_timing.tx_call_max_ns, s31_timing.tx_done_samples,
		S31_AVG(s31_timing.tx_done_total_ns, s31_timing.tx_done_samples),
		s31_timing.tx_done_max_ns);
	spin_lock_irqsave(&s31_tx_timing_lock, flags);
	i = s31_tx_tail;
	spin_unlock_irqrestore(&s31_tx_timing_lock, flags);
	for (; ; i++) {
		struct s31_tx_timing_sample sample;
		u32 head;

		spin_lock_irqsave(&s31_tx_timing_lock, flags);
		head = s31_tx_head;
		if (i < head)
			sample = s31_tx_samples[i % S31_TX_TIMING_SLOTS];
		spin_unlock_irqrestore(&s31_tx_timing_lock, flags);
		if (i >= head)
			break;

		if (sample.kind != S31_TX_DHCP && sample.kind != S31_TX_ARP)
			continue;
		pr_info("esp32s31-radio: tx timing %s seq=%u kind=%s len=%u enqueue->worker=%lluns call=%lluns return->done=%lluns result=%d done=%u status=%u\n",
			stage, sample.sequence, s31_tx_kind_name[sample.kind],
			sample.length, sample.start_ns - sample.enqueue_ns,
			sample.returned ? sample.return_ns - sample.start_ns : 0,
			sample.done && sample.return_ns ?
				sample.done_ns - sample.return_ns : 0,
			sample.result, sample.done, sample.status);
	}
#undef S31_AVG
}

static struct s31_wifi_rx_desc *s31_wifi_rx;
static struct s31_wifi_frame *s31_wifi_tx;
static unsigned int s31_wifi_rx_head;
static unsigned int s31_wifi_rx_tail;
static unsigned int s31_wifi_tx_head;
static unsigned int s31_wifi_tx_tail;
static unsigned long s31_wifi_tx_retry_at;
static DEFINE_SPINLOCK(s31_wifi_frame_lock);
static struct esp32s31_radio_wifi_connect_params *s31_wifi_connect_params;
static u16 *s31_wifi_disconnect_reason;
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
	union {
		struct esp32s31_radio_health *health;
		u8 *mac;
		const struct esp32s31_radio_wifi_connect_params *connect;
		u16 disconnect_reason;
		struct {
			void (*entry)(void *);
			const char *name;
			u32 stack_size;
			void *stack_base;
			void *arg;
			u32 priority;
			void *cookie;
			void *linux_task;
		} task_create;
	};
};

static LIST_HEAD(s31_radio_commands);
static DEFINE_SPINLOCK(s31_radio_command_lock);
static void *s31_radio_worker_stack;
#define S31_RADIO_WORKER_STACK_SIZE 8192U
static atomic_t s31_radio_state = ATOMIC_INIT(ESP32S31_RADIO_OFFLINE);
static atomic_t s31_radio_worker_passes = ATOMIC_INIT(0);
static atomic_t s31_radio_commands_completed = ATOMIC_INIT(0);
static int s31_wifi_init_result = -EINPROGRESS;
static int s31_bt_init_result = -EINPROGRESS;
static int s31_bt_enable_result = -EINPROGRESS;
static void s31_radio_health_workfn(struct work_struct *work);
static DECLARE_WORK(s31_radio_health_work, s31_radio_health_workfn);

#define S31_IDF_INTR_DISABLED	BIT(5)
static atomic_t s31_tick_pending = ATOMIC_INIT(0);

/* Gate-held PC samples were previously collected from the 100 Hz TIMG1
 * hardirq.  That interrupt source has been removed; the ring is retained
 * so the long-gate diagnostic path still compiles and prints empty state. */
#define S31_PC_SAMPLE_RING 128
struct s31_pc_sample {
	u32 tick;
	u32 epc;
	u32 ra;
	u32 pid;
};
static struct s31_pc_sample s31_pc_sample_ring[S31_PC_SAMPLE_RING];
static unsigned int s31_pc_sample_head;

void s31_radio_report_wifi_init(int result)
{
	WRITE_ONCE(s31_wifi_init_result, result);
}

void s31_radio_report_bt_init(int result)
{
	WRITE_ONCE(s31_bt_init_result, result);
}

void s31_radio_report_bt_enable(int result)
{
	WRITE_ONCE(s31_bt_enable_result, result);
}

u32 s31_linux_tick_count(void)
{
	/* Linux jiffies is the radio world time base now.  The 100 Hz TIMG1
	 * hardirq was removed so a blob-gate busy-wait cannot starve the tick
	 * source: jiffies advances on the Linux scheduler tick independently.
	 * jiffies starts at INITIAL_JIFFIES (near UINT_MAX on 32-bit); use the
	 * elapsed count so the returned 10 ms tick matches milliseconds since
	 * boot and wraps like the FreeRTOS TickType_t counter.
	 */
	return (u32)(jiffies_to_msecs((unsigned long)(jiffies - INITIAL_JIFFIES)) / 10U);
}

int __wrap_esp_intr_alloc(int source, int flags, void (*handler)(void *),
			  void *arg, void **ret_handle)
{
	struct s31_idf_irq_registration *registration = NULL;
	int i;

	if (!handler)
		return -EINVAL;
	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++) {
		if (!s31_idf_irqs[i].allocated) {
			registration = &s31_idf_irqs[i];
			break;
		}
	}
	if (!registration)
		return -ENOMEM;
	registration->source = source;
	registration->logical_intr = UINT_MAX;
	registration->hwirq = s31_radio_hwirqs[i];
	registration->flags = flags;
	registration->handler = handler;
	registration->arg = arg;
	registration->install_pending = true;
	registration->enabled = !(flags & S31_IDF_INTR_DISABLED);
	registration->allocated = true;
	atomic_set(&registration->hardirq_count, 0);
	atomic_set(&registration->callback_pending, 0);
	atomic_set(&registration->callback_count, 0);
	atomic_set(&registration->mask_count, 0);
	if (ret_handle)
		*ret_handle = registration;
	pr_info("esp32s31-radio: deferred IDF irq source=%d CLIC%d flags=%#x handler=%pS\n",
		source, registration->hwirq, flags, handler);
	return 0;
}

static struct s31_idf_irq_registration *s31_radio_irq_handle(void *handle)
{
	int i;

	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++)
		if (handle == &s31_idf_irqs[i] && s31_idf_irqs[i].allocated)
			return &s31_idf_irqs[i];
	return NULL;
}

int __wrap_esp_intr_enable(void *handle)
{
	struct s31_idf_irq_registration *registration =
		s31_radio_irq_handle(handle);

	if (!registration)
		return -EINVAL;
	WRITE_ONCE(registration->enabled, true);
	wake_up(&s31_radio_waitq);
	return 0;
}

int __wrap_esp_intr_disable(void *handle)
{
	struct s31_idf_irq_registration *registration =
		s31_radio_irq_handle(handle);

	if (!registration)
		return -EINVAL;
	WRITE_ONCE(registration->enabled, false);
	wake_up(&s31_radio_waitq);
	return 0;
}

int __wrap_esp_intr_free(void *handle)
{
	struct s31_idf_irq_registration *registration =
		s31_radio_irq_handle(handle);

	if (!registration)
		return -EINVAL;
	WRITE_ONCE(registration->enabled, false);
	if (registration->virq)
		registration->free_pending = true;
	else
		memset(registration, 0, sizeof(*registration));
	wake_up(&s31_radio_waitq);
	return 0;
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

	(void)priority;
	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++) {
		if (s31_idf_irqs[i].allocated &&
		    s31_idf_irqs[i].source == source) {
			registration = &s31_idf_irqs[i];
			break;
		}
		if (!registration && !s31_idf_irqs[i].allocated)
			registration = &s31_idf_irqs[i];
	}
	if (!registration) {
		pr_err("esp32s31-radio: no CLIC slot for Wi-Fi source %u\n",
		       source);
		return;
	}
	if (!registration->allocated) {
		memset(registration, 0, sizeof(*registration));
		registration->source = source;
		registration->hwirq = s31_radio_hwirqs[registration - s31_idf_irqs];
		registration->allocated = true;
			atomic_set(&registration->hardirq_count, 0);
		atomic_set(&registration->callback_pending, 0);
		atomic_set(&registration->callback_count, 0);
		atomic_set(&registration->mask_count, 0);
	}
	/* The closed Wi-Fi driver may reserve a source before its OS adapter
	 * supplies the legacy local interrupt number.  Merge both halves of the
	 * registration instead of leaving logical_intr at UINT_MAX. */
	registration->logical_intr = logical_intr;
	if (source == 120)
		pr_info("esp32s31-radio: Wi-Fi IRQ configure source=%u logical=%u CLIC%u enabled=%u\n",
			source, logical_intr, registration->hwirq,
			registration->enabled);
}

void s31_radio_wifi_intr_set_isr(u32 logical_intr, void (*handler)(void *),
				 void *arg)
{
	int i;

	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++) {
		struct s31_idf_irq_registration *registration = &s31_idf_irqs[i];

		if (!registration->allocated ||
		    registration->logical_intr != logical_intr)
			continue;
		registration->handler = handler;
		registration->arg = arg;
		registration->install_pending = handler != NULL;
	}
}

void s31_radio_wifi_intr_mask(u32 mask, bool enable)
{
	int i;

	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++) {
		struct s31_idf_irq_registration *registration = &s31_idf_irqs[i];

		if (!registration->allocated || registration->logical_intr >= 32 ||
		    !(mask & BIT(registration->logical_intr)))
			continue;
		WRITE_ONCE(registration->enabled, enable);
		atomic_inc(&registration->mask_count);
	}
	wake_up(&s31_radio_waitq);
}

/*
 * Radio payload ISRs must execute inside the blob gate to avoid data races
 * with compatibility tasks.  Hardirq context only acknowledges the CLIC
 * source and enqueues work; the serialized worker dispatches callbacks after
 * acquiring the gate, switching to the exception stack, and saving FPU state.
 */
static irqreturn_t s31_radio_hardirq(int irq, void *data)
{
	struct s31_idf_irq_registration *registration = data;
	struct task_struct *worker;
	int hard_count;

	/* The blob ISR is XIP code and may use the compatibility RTOS and FP.
	 * Keep all of that out of hardirq context.  Holding the CLIC slot masked
	 * also avoids the S31's unsupported cross-privilege interrupt nesting. */
	disable_irq_nosync(irq);
	WRITE_ONCE(registration->hw_enabled, false);
	WRITE_ONCE(registration->pending_since_ns, ktime_get_mono_fast_ns());
	hard_count = atomic_inc_return(&registration->hardirq_count);
	(void)hard_count;
	atomic_set(&registration->callback_pending, 1);
	/* Wake the radio worker directly instead of going through the wait
	 * queue: wake_up_process() on a SCHED_FIFO thread is a single
	 * scheduler call compared to the spinlock + waiter iteration that
	 * the waitq path carries.  The worker checks callback_pending after
	 * each blob pass so no additional condition variable is needed. */
	worker = READ_ONCE(s31_radio_worker);
	if (worker)
		wake_up_process(worker);
	return IRQ_HANDLED;
}

static void s31_radio_sync_one_irq(struct s31_idf_irq_registration *registration)
{
	struct device_node *clic_node;
	struct irq_fwspec fwspec = { };
	int virq, ret;

	if (!registration->handler)
		return;
	if (registration->free_pending && registration->virq) {
		free_irq(registration->virq, registration);
		irq_dispose_mapping(registration->virq);
		memset(registration, 0, sizeof(*registration));
		return;
	}

	if (registration->install_pending && !registration->virq) {
		clic_node = of_find_compatible_node(NULL, NULL,
						"espressif,esp32s31-clic");
		if (!clic_node) {
			pr_err("esp32s31-radio: cannot find S-mode CLIC domain\n");
			return;
		}
		fwspec.fwnode = of_node_to_fwnode(clic_node);
		fwspec.param_count = 3;
		fwspec.param[0] = registration->hwirq;
		fwspec.param[1] = registration->source;
		fwspec.param[2] = IRQ_TYPE_LEVEL_HIGH;
		virq = irq_create_fwspec_mapping(&fwspec);
		of_node_put(clic_node);
		if (!virq) {
			pr_err("esp32s31-radio: cannot map IDF source %d\n",
			       registration->source);
			return;
		}
		/* Publish the mapping before request_irq(); a live level source
		 * may invoke the hard IRQ handler as soon as it is enabled. */
		registration->virq = virq;
		registration->hw_enabled = true;
		ret = request_irq(virq, s31_radio_hardirq, 0,
				  "esp32s31-radio", registration);
		if (ret) {
			pr_err("esp32s31-radio: request IRQ%d failed: %d\n",
			       virq, ret);
			registration->virq = 0;
			registration->hw_enabled = false;
			irq_dispose_mapping(virq);
			return;
		}
		registration->install_pending = false;
		pr_info("esp32s31-radio: IDF source %d routed to CLIC%d/IRQ%d\n",
			registration->source, registration->hwirq, virq);
	}

	if (!registration->virq)
		return;
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

/* Called from s31_linux_blob_enter() while this worker waits for the blob
 * gate.  A payload task can be stuck waiting for an IDF interrupt whose CLIC
 * mapping only this worker can install; syncing here breaks that cycle. */
static void s31_radio_gate_wait_sync(void)
{
	if (current != READ_ONCE(s31_radio_worker))
		return;
	s31_radio_sync_irq_registrations();
}

static bool s31_radio_irq_work_pending(void)
{
	int i;

	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++) {
		struct s31_idf_irq_registration *registration =
			&s31_idf_irqs[i];

		if (atomic_read(&registration->callback_pending) ||
		    registration->install_pending || registration->free_pending ||
		    (registration->virq &&
		     registration->enabled != registration->hw_enabled))
			return true;
	}
	return false;
}

static bool s31_radio_irq_callback_pending(void)
{
	int i;

	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++)
		if (atomic_read(&s31_idf_irqs[i].callback_pending))
			return true;
	return false;
}

/* Print the most recent gate-held PC samples with symbol resolution.  The
 * tick IRQ samples the interrupted PC whenever current holds the blob gate,
 * so this is a cheap view of where the payload/worker spends long holds. */
static void s31_pc_sample_dump(void)
{
	u32 head = READ_ONCE(s31_pc_sample_head);
	u32 printed, i, start;

	if (!head)
		return;
	printed = head > 32 ? 32 : head;
	start = head - printed;
	pr_info("esp32s31-radio: gate-held PC samples (last %u):\n", printed);
	for (i = 0; i < printed; i++) {
		u32 idx = (start + i) % S31_PC_SAMPLE_RING;
		struct s31_pc_sample *sample = &s31_pc_sample_ring[idx];

		pr_info("esp32s31-radio:   tick=%u pid=%u epc=%pS ra=%pS\n",
			sample->tick, sample->pid,
			(void *)(unsigned long)sample->epc,
			(void *)(unsigned long)sample->ra);
	}
}

void s31_radio_diag_long_gate_release(u32 reason, u64 wall_ns,
				      u64 exec_ns, u32 tick_start)
{
	static int print_count;
	struct task_struct *worker = READ_ONCE(s31_radio_worker);
	u32 tick_end = s31_linux_tick_count();
	u32 tick_delta = tick_end - tick_start;
	u32 head = READ_ONCE(s31_pc_sample_head);
	u32 printed, i;

	if (print_count >= 8)
		return;
	print_count++;

	pr_info("esp32s31-radio: long gate release #%d reason=%u wall=%lluns exec=%lluns ticks=%u..%u delta=%u tick_pending=%d irq_work=%d worker_state=%ld gate_owner=%s pc_head=%u\n",
		print_count, reason, wall_ns, exec_ns, tick_start, tick_end,
		tick_delta, atomic_read(&s31_tick_pending),
		s31_radio_irq_work_pending(),
		worker ? (long)READ_ONCE(worker->__state) : -1L,
		s31_linux_blob_holder() ?: "none", head);

	if (!head)
		return;
	if (head > 64)
		printed = 64;
	else
		printed = head;
	for (i = 0; i < printed; i++) {
		u32 idx = (head - printed + i) % S31_PC_SAMPLE_RING;
		struct s31_pc_sample *sample = &s31_pc_sample_ring[idx];

		pr_info("esp32s31-radio: pc[%02u] tick=%u pid=%u epc=%px ra=%px\n",
			i, sample->tick, sample->pid,
			(void *)(unsigned long)sample->epc,
			(void *)(unsigned long)sample->ra);
	}
}

static bool s31_radio_command_work_pending(void)
{
	bool pending;

	spin_lock(&s31_radio_command_lock);
	pending = !list_empty(&s31_radio_commands);
	spin_unlock(&s31_radio_command_lock);
	return pending;
}

static bool s31_radio_hci_tx_pending(void)
{
	return READ_ONCE(s31_hci_tx_head) != READ_ONCE(s31_hci_tx_tail);
}

void s31_radio_vhci_send_available(void)
{
	wake_up(&s31_radio_waitq);
}

int s31_radio_vhci_receive(u8 *frame, u16 length)
{
	unsigned long flags;
	unsigned int next;

	if (!frame || !length || length > S31_HCI_FRAME_SIZE)
		return -EINVAL;
	spin_lock_irqsave(&s31_hci_lock, flags);
	next = (s31_hci_rx_head + 1) % S31_HCI_RX_SLOTS;
	if (next == s31_hci_rx_tail) {
		atomic_inc(&s31_hci_rx_dropped);
		spin_unlock_irqrestore(&s31_hci_lock, flags);
		return -ENOSPC;
	}
	s31_hci_rx[s31_hci_rx_head].length = length;
	memcpy(s31_hci_rx[s31_hci_rx_head].data, frame, length);
	smp_store_release(&s31_hci_rx_head, next);
	spin_unlock_irqrestore(&s31_hci_lock, flags);
	return 0;
}

int s31_radio_wifi_receive(u8 *frame, u16 length)
{
	return s31_radio_wifi_receive_zerocopy(frame, NULL, length);
}

int s31_radio_wifi_receive_zerocopy(u8 *frame, void *eb, u16 length)
{
	unsigned long flags;
	unsigned int next;
	static unsigned int rx_count;

	if (!frame || length < ETH_HLEN || length > S31_WIFI_FRAME_SIZE)
		return -EINVAL;
	atomic_inc(&s31_wifi_rx_cb);
	rx_count++;
	if (rx_count <= 4)
		pr_info("esp32s31-radio: wifi_receive #%u len=%u\n",
			rx_count, length);
	spin_lock_irqsave(&s31_wifi_frame_lock, flags);
	next = (s31_wifi_rx_head + 1) % S31_WIFI_RX_SLOTS;
	if (next == s31_wifi_rx_tail) {
		atomic_inc(&s31_wifi_rx_dropped);
		spin_unlock_irqrestore(&s31_wifi_frame_lock, flags);
		return -ENOSPC;
	}
	s31_wifi_rx[s31_wifi_rx_head].length = length;
	s31_wifi_rx[s31_wifi_rx_head].data = frame;
	s31_wifi_rx[s31_wifi_rx_head].eb = eb;
	smp_store_release(&s31_wifi_rx_head, next);
	spin_unlock_irqrestore(&s31_wifi_frame_lock, flags);
	wake_up(&s31_radio_waitq);
	return 0;
}

void s31_radio_wifi_connected(const u8 *bssid, u8 channel, int status)
{
	if (bssid)
		memcpy(s31_wifi_event_bssid, bssid, sizeof(s31_wifi_event_bssid));
	s31_wifi_event_channel = channel;
	s31_wifi_connect_status = status;
	s31_wifi_connected_ready = true;
	wake_up(&s31_radio_waitq);
}

void s31_radio_wifi_disconnected(u16 reason)
{
	s31_wifi_event_reason = reason;
	s31_wifi_disconnected_ready = true;
	wake_up(&s31_radio_waitq);
}

/* Called only inside the serialized blob execution gate. */
static void s31_radio_hci_process_tx(void)
{
#ifdef CONFIG_BT_ESP32S31
	while (s31_hci_tx_tail != smp_load_acquire(&s31_hci_tx_head)) {
		struct s31_hci_frame *frame = &s31_hci_tx[s31_hci_tx_tail];

		if (s31_radio_vhci_try_send(frame->data, frame->length))
			break;
		s31_hci_tx_tail = (s31_hci_tx_tail + 1) % S31_HCI_TX_SLOTS;
	}
#endif
}

/* This runs after the gate has restored Linux IRQ and FPU state. */
static void s31_radio_hci_deliver_rx(void)
{
	for (;;) {
		const struct esp32s31_radio_hci_ops *ops;
		struct s31_hci_frame *frame;
		void *context;
		unsigned long flags;

		spin_lock_irqsave(&s31_hci_lock, flags);
		if (s31_hci_rx_tail == s31_hci_rx_head) {
			spin_unlock_irqrestore(&s31_hci_lock, flags);
			break;
		}
		ops = s31_hci_ops;
		context = s31_hci_context;
		frame = &((struct s31_hci_frame *)
			  s31_radio_sram_linux_alias(s31_hci_rx))[s31_hci_rx_tail];
		if (!ops) {
			spin_unlock_irqrestore(&s31_hci_lock, flags);
			break;
		}
		spin_unlock_irqrestore(&s31_hci_lock, flags);

		ops->receive(context, frame->data, frame->length);
		s31_hci_rx_tail = (s31_hci_rx_tail + 1) % S31_HCI_RX_SLOTS;
	}
}

void s31_radio_wifi_scan_complete(const struct esp32s31_radio_wifi_ap *aps,
				  u16 count, int status)
{
	if (count > ESP32S31_RADIO_WIFI_MAX_APS)
		count = ESP32S31_RADIO_WIFI_MAX_APS;
	if (aps && count)
		memcpy(s31_wifi_scan_aps, aps, sizeof(*aps) * count);
	s31_wifi_scan_count = count;
	s31_wifi_scan_status = status;
	s31_wifi_scan_ready = true;
	wake_up(&s31_radio_waitq);
}

/* Defined in the radio payload (blob glue): recycles an esf_buf. */
void s31_radio_wifi_free_rx_buffer(void *eb);

/* Drain the RX esf_buf recycle queue.  Runs inside the blob pass with the
 * gate held, so it is serialized with the MAC RX allocator and the Wi-Fi
 * task. */
static void s31_radio_wifi_free_pending(void)
{
	while (s31_wifi_free_tail != READ_ONCE(s31_wifi_free_head)) {
		void *eb = s31_wifi_free_ring[s31_wifi_free_tail];

		s31_wifi_free_ring[s31_wifi_free_tail] = NULL;
		s31_wifi_free_tail = (s31_wifi_free_tail + 1) % S31_WIFI_FREE_SLOTS;
		if (eb) {
			s31_radio_wifi_free_rx_buffer(eb);
			atomic_inc(&s31_wifi_rx_freed);
		}
	}
}

static void s31_radio_wifi_deliver_scan(void)
{
	if (!s31_wifi_scan_ready || !s31_wifi_ops)
		return;
	s31_wifi_scan_ready = false;
	s31_wifi_scan_inflight = false;
	s31_wifi_ops->scan_complete(s31_wifi_context, s31_wifi_scan_status,
				    s31_wifi_scan_aps, s31_wifi_scan_count);
}

static void s31_radio_wifi_process_tx(void)
{
	while (s31_wifi_tx_tail != smp_load_acquire(&s31_wifi_tx_head)) {
		struct s31_wifi_frame *frame = &s31_wifi_tx[s31_wifi_tx_tail];
		int ret;

		{
			u64 start_ns = ktime_get_mono_fast_ns();
			u32 sequence = s31_radio_timing_tx_begin(
				frame->enqueue_ns, start_ns, frame->data,
				frame->length);

			ret = s31_radio_wifi_try_send(frame->data, frame->length);
			s31_radio_timing_tx_return(sequence,
						   ktime_get_mono_fast_ns(), ret);
		}
		if (ret == -EAGAIN || ret == 257) { /* 257 == ESP_ERR_NO_MEM */
			s31_wifi_tx_retry_at = jiffies + 1;
			break;
		}
		if (ret) {
			atomic_inc(&s31_wifi_tx_dropped);
			pr_warn_ratelimited("esp32s31-radio: dropping Wi-Fi TX frame: %d\n",
					    ret);
		}
		s31_wifi_tx_tail = (s31_wifi_tx_tail + 1) % S31_WIFI_TX_SLOTS;
	}
}

static bool s31_radio_wifi_tx_pending(void)
{
	return READ_ONCE(s31_wifi_tx_head) != READ_ONCE(s31_wifi_tx_tail) &&
	       time_after_eq(jiffies, READ_ONCE(s31_wifi_tx_retry_at));
}

static bool s31_radio_wifi_rx_pending(void)
{
	return READ_ONCE(s31_wifi_rx_head) != READ_ONCE(s31_wifi_rx_tail);
}

static void s31_radio_wifi_deliver_events(void)
{
	const struct esp32s31_radio_wifi_ops *ops = s31_wifi_ops;
	static unsigned int rx_frames_delivered;

	if (!ops)
		return;
	if (s31_wifi_connected_ready) {
		s31_wifi_connected_ready = false;
		ops->connected(s31_wifi_context, s31_wifi_connect_status,
			       s31_wifi_event_bssid, s31_wifi_event_channel);
	}
	if (s31_wifi_disconnected_ready) {
		s31_wifi_disconnected_ready = false;
		s31_radio_timing_report("disconnect");
		s31_linux_gate_timing_report("disconnect");
		ops->disconnected(s31_wifi_context, s31_wifi_event_reason);
	}
	while (s31_wifi_rx_tail != smp_load_acquire(&s31_wifi_rx_head)) {
		struct s31_wifi_rx_desc *desc = &((struct s31_wifi_rx_desc *)
			s31_radio_sram_linux_alias(s31_wifi_rx))[s31_wifi_rx_tail];

		rx_frames_delivered++;
		if (rx_frames_delivered <= 8)
			pr_info("esp32s31-radio: RX frame #%u len=%u "
				"delivered to cfg80211\n",
				rx_frames_delivered, desc->length);
		ops->receive(s31_wifi_context,
			     s31_radio_sram_linux_alias(desc->data),
			     desc->length);
		atomic_inc(&s31_wifi_rx_delivered);
		s31_wifi_rx_tail = (s31_wifi_rx_tail + 1) % S31_WIFI_RX_SLOTS;
		/* Queue the esf_buf for recycling; the blob pass drains it. */
		if (desc->eb) {
			unsigned int next =
				(s31_wifi_free_head + 1) % S31_WIFI_FREE_SLOTS;

			if (next != s31_wifi_free_tail) {
				s31_wifi_free_ring[s31_wifi_free_head] = desc->eb;
				s31_wifi_free_head = next;
			} else {
				/* With S31_WIFI_FREE_SLOTS > S31_WIFI_RX_SLOTS
				 * this is unreachable.  Do not advance the head:
				 * that would make the full ring look empty and
				 * leak every queued esf_buf.
				 */
				pr_warn_ratelimited("esp32s31-radio: RX free ring full\n");
			}
		}
	}
}

static void s31_radio_fill_health(struct esp32s31_radio_health *health)
{
	size_t free = gen_pool_avail(s31_radio_heap_pool);
	size_t total = gen_pool_size(s31_radio_heap_pool);

	health->state = atomic_read(&s31_radio_state);
	health->wifi_init_result = READ_ONCE(s31_wifi_init_result);
	health->bt_init_result = READ_ONCE(s31_bt_init_result);
	health->bt_enable_result = READ_ONCE(s31_bt_enable_result);
	health->tick_irqs = s31_linux_tick_count();
	health->worker_passes = atomic_read(&s31_radio_worker_passes);
	health->commands_completed = atomic_read(&s31_radio_commands_completed);
	health->heap_used = total - free;
	health->heap_peak = s31_radio_heap_peak;
	health->heap_total = total;
	health->wifi_rx_dropped = atomic_read(&s31_wifi_rx_dropped);
	health->wifi_tx_dropped = atomic_read(&s31_wifi_tx_dropped);
}

/* Run only from the radio worker, on its normal kernel stack.
 * Individual cases acquire the blob gate if the payload entry needs it.
 */
static void s31_radio_process_commands(void)
{
	LIST_HEAD(commands);
	struct s31_radio_command *command, *next;

	spin_lock(&s31_radio_command_lock);
	list_splice_init(&s31_radio_commands, &commands);
	spin_unlock(&s31_radio_command_lock);

	list_for_each_entry_safe(command, next, &commands, node) {
		list_del_init(&command->node);
		atomic_inc(&s31_radio_commands_completed);
		switch (command->type) {
		case S31_RADIO_COMMAND_HEALTH:
			s31_radio_fill_health(command->health);
			command->result = 0;
			break;
		case S31_RADIO_COMMAND_WIFI_GET_MAC:
			s31_linux_blob_enter();
			command->result = s31_radio_wifi_read_mac(command->mac);
			s31_linux_blob_leave();
			break;
		case S31_RADIO_COMMAND_WIFI_SCAN:
			if (s31_wifi_scan_inflight) {
				command->result = -EBUSY;
				break;
			}
			if (xTaskCreatePinnedToCore(s31_radio_wifi_scan_task,
						    "wifi-scan", 4096, NULL,
						    20, NULL, 0) != 1) {
				command->result = -ENOMEM;
				break;
			}
			s31_wifi_scan_inflight = true;
			command->result = 0;
			break;
		case S31_RADIO_COMMAND_WIFI_CONNECT:
			s31_radio_timing_reset();
			s31_linux_gate_timing_reset();
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
		case S31_RADIO_COMMAND_BT_ENABLE:
#ifdef CONFIG_BT_ESP32S31
			if (xTaskCreatePinnedToCore(s31_radio_bt_enable_task,
						    "bt-enable", 8192, NULL, 22,
						    NULL, 0) != 1)
				command->result = -ENOMEM;
			else
				command->result = 0;
#else
			command->result = -EOPNOTSUPP;
#endif
			break;
		case S31_RADIO_COMMAND_TASK_CREATE:
			command->task_create.linux_task = s31_linux_task_create(
				command->task_create.entry,
				command->task_create.name,
				command->task_create.stack_size,
				command->task_create.stack_base,
				command->task_create.arg,
				command->task_create.priority,
				command->task_create.cookie);
			command->result = command->task_create.linux_task ? 0 : -ENOMEM;
			break;
		default:
			command->result = -EOPNOTSUPP;
			break;
		}
		complete(&command->done);
	}
}

/*
 * s31_linux_task_create() calls this bridge when the caller is a compat
 * task executing on its HP-SRAM payload stack.  kthread_create() puts its
 * on-stack completion on that payload stack, and the freshly forked kthread
 * can complete it from the secondary hart before kthread_bind() runs.  Queue
 * the request to the radio worker instead; the worker creates the kthread on
 * its normal kernel stack, binds it to hart0, and then completes this
 * normal-memory command object.
 */
void *s31_radio_task_create_deferred(void (*entry)(void *), const char *name,
				     u32 stack_size, void *stack_base,
				     void *arg, u32 priority, void *cookie)
{
	struct s31_radio_command *command;
	void *linux_task;

	command = kzalloc(sizeof(*command), GFP_KERNEL);
	if (!command)
		return NULL;
	INIT_LIST_HEAD(&command->node);
	init_completion(&command->done);
	command->type = S31_RADIO_COMMAND_TASK_CREATE;
	command->task_create.entry = entry;
	command->task_create.name = name;
	command->task_create.stack_size = stack_size;
	command->task_create.stack_base = stack_base;
	command->task_create.arg = arg;
	command->task_create.priority = priority;
	command->task_create.cookie = cookie;

	spin_lock(&s31_radio_command_lock);
	list_add_tail(&command->node, &s31_radio_commands);
	spin_unlock(&s31_radio_command_lock);
	wake_up(&s31_radio_waitq);

	wait_for_completion(&command->done);
	linux_task = command->task_create.linux_task;
	kfree(command);
	return linux_task;
}

/* Optional IDF tables are empty in the built-in radio payload. */
asm(".globl _esp_err_msg_tbl_start\n"
    ".globl _esp_err_msg_tbl_end\n"
    ".set _esp_err_msg_tbl_start, 0\n"
    ".set _esp_err_msg_tbl_end, 0\n");
#endif

int esp32s31_radio_get_health(struct esp32s31_radio_health *health)
{
#ifdef CONFIG_ESP32S31_RADIO_BLOBS
	struct s31_radio_command command;

	if (!health)
		return -EINVAL;
	if (current == READ_ONCE(s31_radio_worker))
		return -EDEADLK;
	if (atomic_read(&s31_radio_state) != ESP32S31_RADIO_READY)
		return -EAGAIN;

	INIT_LIST_HEAD(&command.node);
	init_completion(&command.done);
	command.type = S31_RADIO_COMMAND_HEALTH;
	command.result = -EINPROGRESS;
	command.health = health;
	spin_lock(&s31_radio_command_lock);
	list_add_tail(&command.node, &s31_radio_commands);
	spin_unlock(&s31_radio_command_lock);
	wake_up(&s31_radio_waitq);
	wait_for_completion(&command.done);
	return command.result;
#else
	return -EOPNOTSUPP;
#endif
}
EXPORT_SYMBOL_GPL(esp32s31_radio_get_health);

int esp32s31_radio_hci_register(const struct esp32s31_radio_hci_ops *ops,
				void *context)
{
	unsigned long flags;
	int ret = 0;

	if (!ops || !ops->receive)
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
		wake_up(&s31_radio_waitq);
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

int esp32s31_radio_hci_send(u8 packet_type, const u8 *data, size_t length)
{
	unsigned long flags;
	unsigned int next;
	struct s31_hci_frame *frame;

	if (!data || length + 1 > S31_HCI_FRAME_SIZE)
		return -EMSGSIZE;
	if (atomic_read(&s31_radio_state) != ESP32S31_RADIO_READY)
		return -ENODEV;
	spin_lock_irqsave(&s31_hci_lock, flags);
	next = (s31_hci_tx_head + 1) % S31_HCI_TX_SLOTS;
	if (next == s31_hci_tx_tail) {
		atomic_inc(&s31_hci_tx_dropped);
		spin_unlock_irqrestore(&s31_hci_lock, flags);
		return -ENOSPC;
	}
	frame = &((struct s31_hci_frame *)
		  s31_radio_sram_linux_alias(s31_hci_tx))[s31_hci_tx_head];
	frame->data[0] = packet_type;
	memcpy(frame->data + 1, data, length);
	frame->length = length + 1;
	smp_store_release(&s31_hci_tx_head, next);
	spin_unlock_irqrestore(&s31_hci_lock, flags);
	wake_up(&s31_radio_waitq);
	return 0;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_hci_send);

int esp32s31_radio_wifi_register(const struct esp32s31_radio_wifi_ops *ops,
				 void *context)
{
	if (!ops || !ops->scan_complete || !ops->connected ||
	    !ops->disconnected || !ops->receive)
		return -EINVAL;
	if (atomic_read(&s31_radio_state) != ESP32S31_RADIO_READY)
		return -EAGAIN;
	if (cmpxchg(&s31_wifi_ops, NULL, ops))
		return -EBUSY;
	s31_wifi_context = context;
	return 0;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_wifi_register);

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
	spin_lock(&s31_radio_command_lock);
	list_add_tail(&command.node, &s31_radio_commands);
	spin_unlock(&s31_radio_command_lock);
	wake_up(&s31_radio_waitq);
	wait_for_completion(&command.done);
	return command.result;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_wifi_get_mac);

int esp32s31_radio_wifi_scan(void)
{
	struct s31_radio_command command;

	if (!s31_wifi_ops)
		return -ENODEV;
	INIT_LIST_HEAD(&command.node);
	init_completion(&command.done);
	command.type = S31_RADIO_COMMAND_WIFI_SCAN;
	command.result = -EINPROGRESS;
	spin_lock(&s31_radio_command_lock);
	list_add_tail(&command.node, &s31_radio_commands);
	spin_unlock(&s31_radio_command_lock);
	wake_up(&s31_radio_waitq);
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
	spin_lock(&s31_radio_command_lock);
	list_add_tail(&command.node, &s31_radio_commands);
	spin_unlock(&s31_radio_command_lock);
	wake_up(&s31_radio_waitq);
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
	spin_lock(&s31_radio_command_lock);
	list_add_tail(&command.node, &s31_radio_commands);
	spin_unlock(&s31_radio_command_lock);
	wake_up(&s31_radio_waitq);
	wait_for_completion(&command.done);
	return command.result;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_wifi_disconnect);

int esp32s31_radio_bt_enable(void)
{
#ifndef CONFIG_BT_ESP32S31
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
	spin_lock(&s31_radio_command_lock);
	list_add_tail(&command.node, &s31_radio_commands);
	spin_unlock(&s31_radio_command_lock);
	wake_up(&s31_radio_waitq);
	wait_for_completion(&command.done);
	return command.result;
#endif
}
EXPORT_SYMBOL_GPL(esp32s31_radio_bt_enable);

int esp32s31_radio_wifi_send(const u8 *frame, size_t length)
{
	unsigned long flags;
	unsigned int next;

	if (!frame || length < ETH_HLEN || length > S31_WIFI_FRAME_SIZE)
		return -EINVAL;
	spin_lock_irqsave(&s31_wifi_frame_lock, flags);
	next = (s31_wifi_tx_head + 1) % S31_WIFI_TX_SLOTS;
	if (next == s31_wifi_tx_tail) {
		atomic_inc(&s31_wifi_tx_dropped);
		pr_warn_ratelimited("esp32s31-radio: TX ring full, dropping frame len=%zu\n",
				    length);
		spin_unlock_irqrestore(&s31_wifi_frame_lock, flags);
		return -ENOSPC;
	}
	{
		struct s31_wifi_frame *slot = &((struct s31_wifi_frame *)
			s31_radio_sram_linux_alias(s31_wifi_tx))[s31_wifi_tx_head];

		slot->length = length;
		slot->enqueue_ns = ktime_get_mono_fast_ns();
		memcpy(slot->data, frame, length);
	}
	smp_store_release(&s31_wifi_tx_head, next);
	spin_unlock_irqrestore(&s31_wifi_frame_lock, flags);
	wake_up(&s31_radio_waitq);
	return 0;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_wifi_send);

#ifdef CONFIG_ESP32S31_RADIO_BLOBS
struct s31_radio_blob_pass_args {
	int tick_events;
	u64 worker_start_ns;
};

static void s31_radio_run_blob_pass(void *opaque)
{
	struct s31_radio_blob_pass_args *args = opaque;
	struct sched_param rt_param = { .sched_priority = 90 };
	struct sched_param cfs_param = { };
	int tick_events = args->tick_events;
	bool irq_pass = s31_radio_irq_callback_pending();
	u64 gate_end_ns;
	int i;

	/* Match the native ordering only for interrupt service.  A permanent FIFO
	 * worker can stay runnable on periodic/tick work and starve Linux; a CFS
	 * worker can miss BLE LLL deadlines under a pinned CoreMark. */
	if (irq_pass)
		sched_setscheduler_nocheck(current, SCHED_FIFO, &rt_param);
	args->worker_start_ns = ktime_get_mono_fast_ns();
	s31_linux_blob_enter();
	gate_end_ns = ktime_get_mono_fast_ns();
	if (gate_end_ns >= args->worker_start_ns) {
		s31_timing_add(gate_end_ns - args->worker_start_ns,
			       &s31_timing.worker_gate_total_ns,
			       &s31_timing.worker_gate_max_ns);
		s31_timing.gate_samples++;
	}
	while (tick_events-- > 0)
		s31_rtos_tick();
	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++) {
		struct s31_idf_irq_registration *registration = &s31_idf_irqs[i];
		u64 worker_ns, entered_ns, handler_end_ns;

		if (!registration->handler ||
		    !atomic_xchg(&registration->callback_pending, 0))
			continue;
		worker_ns = args->worker_start_ns;
		if (worker_ns >= registration->pending_since_ns) {
			s31_timing_add(worker_ns - registration->pending_since_ns,
				       &s31_timing.irq_to_worker_total_ns,
				       &s31_timing.irq_to_worker_max_ns);
			s31_timing.irq_samples++;
		}
		entered_ns = ktime_get_mono_fast_ns();
		s31_rtos_isr_depth++;
		registration->handler(registration->arg);
		s31_rtos_isr_depth--;
		handler_end_ns = ktime_get_mono_fast_ns();
		s31_timing_add(handler_end_ns - entered_ns,
			       &s31_timing.isr_total_ns, &s31_timing.isr_max_ns);
		s31_timing.isr_samples++;
		WRITE_ONCE(s31_timing.last_isr_end_ns, handler_end_ns);
		atomic_inc(&registration->callback_count);
	}
	s31_radio_hci_process_tx();
	s31_radio_wifi_process_tx();
	s31_radio_wifi_free_pending();
	atomic_inc(&s31_radio_worker_passes);
	s31_linux_blob_leave();
	if (irq_pass) {
		sched_setscheduler_nocheck(current, SCHED_NORMAL, &cfs_param);
		set_user_nice(current, 0);
	}
	/*
	 * The compatibility tasks are blocked on the blob mutex; Wi-Fi and BTDM
	 * use RR priority while sys_evt remains CFS.  After releasing the gate,
	 * yield the
	 * CPU so they can enter the gate, process their event queues, and
	 * push received frames into the Linux ring buffers while the worker
	 * is still running its post-gate delivery step.
	 */
	cond_resched();
}

static int s31_radio_run_linux_pass(void)
{
	struct s31_radio_blob_pass_args args = {
		.tick_events = atomic_xchg(&s31_tick_pending, 0),
	};

	/*
	 * Drain the RX rings before acquiring the blob gate.  These only touch
	 * Linux network-stack state (skb allocation, netif_rx) and must run in
	 * ordinary Linux context.  Doing this first means a received frame is
	 * handed to the net stack immediately after the RX callback wakes the
	 * worker, instead of waiting for the worker to win the blob gate behind
	 * a Wi-Fi task that can hold it for an entire AMPDU batch.
	 */
	s31_radio_hci_deliver_rx();
	s31_radio_wifi_deliver_scan();
	s31_radio_wifi_deliver_events();

	/*
	 * Command processing creates kthreads through kthread_create().  It
	 * must run on the worker's normal kernel stack and, for task-create
	 * requests, before the worker blocks on the blob gate: the compat task
	 * that queued the request may itself be the gate holder.
	 */
	s31_radio_process_commands();

	if (s31_radio_worker_stack)
		s31_linux_call_on_stack(s31_radio_worker_stack,
					S31_RADIO_WORKER_STACK_SIZE,
					s31_radio_run_blob_pass, &args);
	else
		s31_radio_run_blob_pass(&args);

	/*
	 * The blob pass may have drained the TX ring.  If the net queue was
	 * stopped because ndo_start_xmit saw the ring full, wake it now that
	 * space is available again.  netif_wake_queue() is a no-op when the
	 * queue is already running, so this is cheap in the common case.
	 */
	if (s31_wifi_ops && s31_wifi_ops->tx_wakeup &&
	    (s31_wifi_tx_head + 1) % S31_WIFI_TX_SLOTS != s31_wifi_tx_tail)
		s31_wifi_ops->tx_wakeup(s31_wifi_context);

	return 0;
}

static void s31_radio_update_state(void)
{
	int wifi = READ_ONCE(s31_wifi_init_result);

	if (wifi != -EINPROGRESS && wifi) {
		atomic_set(&s31_radio_state, ESP32S31_RADIO_FAILED);
		return;
	}
	if (!wifi &&
	    atomic_cmpxchg(&s31_radio_state, ESP32S31_RADIO_STARTING,
			   ESP32S31_RADIO_READY) == ESP32S31_RADIO_STARTING)
		schedule_work(&s31_radio_health_work);
}

static void s31_radio_fail_commands(void)
{
	LIST_HEAD(commands);
	struct s31_radio_command *command, *next;

	spin_lock(&s31_radio_command_lock);
	list_splice_init(&s31_radio_commands, &commands);
	spin_unlock(&s31_radio_command_lock);
	list_for_each_entry_safe(command, next, &commands, node) {
		list_del_init(&command->node);
		command->result = -ENODEV;
		complete(&command->done);
	}
}

static void s31_radio_health_workfn(struct work_struct *work)
{
	struct esp32s31_radio_health health;
	int ret;

	(void)work;
	ret = esp32s31_radio_get_health(&health);
	if (ret) {
		pr_err("esp32s31-radio: serialized health request failed: %d\n", ret);
		return;
	}
	pr_info("esp32s31-radio: serialized core ready passes=%u commands=%u "
		"ticks=%u heap=%u/%u peak=%u wifi_dropped=%u/%u\n",
		health.worker_passes, health.commands_completed, health.tick_irqs,
		health.heap_used, health.heap_total, health.heap_peak,
		health.wifi_rx_dropped, health.wifi_tx_dropped);
}
#endif

static int s31_radio_runtime_thread(void *unused)
{
	struct sched_param param = { };
	void __iomem *rom;
	unsigned long next_tick;
	/* 10 ms RTOS tick (CONFIG_HZ=100) for esp_timer/xTaskDelay. */
	const unsigned long tick_period = max_t(unsigned long, 1,
						 msecs_to_jiffies(10));
	bool hw_tick;
	u32 identity_word, ioremap_word;
	unsigned int cpu_mhz;
	int ret;

	pr_info("esp32s31-radio: worker entered\n");
	/*
	 * The radio worker uses ordinary SCHED_NORMAL nice 0
	 * so it gets the lion's share of CPU time but does NOT starve init
	 * or other SCHED_NORMAL processes.  With PREEMPT_NONE, SCHED_FIFO
	 * would monopolise the CPU because schedule() always picks FIFO over
	 * NORMAL.  The blob gate + cond_resched() in sync_unlock ensures
	 * periodic yield points.
	 */
	sched_setscheduler_nocheck(current, SCHED_NORMAL, &param);
	set_user_nice(current, 0);
	/* Low identity mappings intentionally exist only in init_mm. */
	kthread_use_mm(&init_mm);
	pr_info("esp32s31-radio: init_mm active\n");

	rom = ioremap(S31_ROM_BASE, PAGE_SIZE);
	if (!rom) {
		pr_err("esp32s31-radio: cannot map mask ROM for comparison\n");
		goto out;
	}

	identity_word = READ_ONCE(*(u32 *)S31_ROM_BASE);
	ioremap_word = readl(rom);
	if (identity_word != ioremap_word) {
		pr_err("esp32s31-radio: identity ROM mismatch %08x != %08x\n",
		       identity_word, ioremap_word);
		iounmap(rom);
		goto out;
	}
	iounmap(rom);

	/* This ROM routine is side-effect free and proves execute permission. */
	cpu_mhz = ((s31_rom_cpu_freq_t)S31_ROM_CPU_FREQ)();
	pr_info("esp32s31-radio: S-mode identity map ready, rom=%08x cpu=%u MHz\n",
		identity_word, cpu_mhz);
#ifdef CONFIG_ESP32S31_RADIO_BLOBS
	atomic_set(&s31_radio_state, ESP32S31_RADIO_STARTING);
	pr_info("esp32s31-radio: ILP32F Wi-Fi/BT payload linked\n");
	/* TIMG1 hardirq tick removed: use Linux jiffies as the FreeRTOS/esp_timer
	 * time base.  Blocking already uses Linux sleep/wait; only xTaskGetTickCount
	 * and esp_timer_get_time need a monotonic 10 ms tick, which jiffies gives.
	 */
	hw_tick = false;
	pr_info("esp32s31-radio: using Linux jiffies tick\n");
	pr_info("esp32s31-radio: initializing compatibility RTOS\n");
	s31_rtos_init();
	if (xTaskCreatePinnedToCore(s31_radio_stack_task, "radio-init", 8192,
				    NULL, 24, NULL, 0) != 1) {
		pr_err("esp32s31-radio: cannot create IDF init task\n");
		goto failed;
	}
	pr_info("esp32s31-radio: compatibility RTOS initialized\n");
	pr_info("esp32s31-radio: Linux kthread task started\n");

	next_tick = jiffies + tick_period;
	for (;;) {
		s31_radio_sync_irq_registrations();
		s31_radio_update_state();
		/*
		 * Periodic heap/PC reports are intentionally disabled: the
		 * printk flood slowed the serial console and disturbed
		 * dual-core CoreMark measurements.  s31_radio_heap_report()
		 * remains available for diagnostic stages.
		 */
		if (!s31_radio_worker_stack &&
		    atomic_read(&s31_radio_state) == ESP32S31_RADIO_READY) {
			s31_radio_worker_stack =
				s31_radio_sram_alloc(S31_RADIO_WORKER_STACK_SIZE);
			if (!s31_radio_worker_stack) {
				pr_err("esp32s31-radio: cannot allocate internal worker stack\n");
				goto failed;
			}
			pr_info("esp32s31-radio: worker blob stack %px..%px in HP SRAM\n",
				s31_radio_worker_stack,
				(u8 *)s31_radio_worker_stack +
					S31_RADIO_WORKER_STACK_SIZE);
		}
		if (kthread_should_stop())
			break;
		if (!hw_tick && time_after_eq(jiffies, next_tick)) {
			unsigned long elapsed = jiffies - next_tick;
			unsigned long ticks = elapsed / tick_period + 1;

			atomic_add(ticks, &s31_tick_pending);
			next_tick += ticks * tick_period;
		}
		ret = s31_radio_run_linux_pass();
		if (ret) {
			pr_err("esp32s31-radio: serialized blob pass failed: %d\n", ret);
			goto failed;
		}
		/*
		 * Unconditionally reschedule after every blob pass.  CFS
		 * wake-up bonus resets the compat task's vruntime on every
		 * sleep/wake cycle, making it lower than init's vruntime.
		 * schedule() forces the scheduler to compare vruntimes and
		 * pick init when the radio tasks have used their fair share.
		 */
		schedule();
		wait_event_interruptible_timeout(s31_radio_waitq,
			kthread_should_stop() ||
			atomic_read(&s31_tick_pending) ||
			s31_radio_irq_work_pending() ||
			s31_radio_command_work_pending() ||
				s31_radio_hci_tx_pending() ||
				s31_radio_wifi_tx_pending() ||
				s31_radio_wifi_rx_pending(),
			time_before(jiffies, next_tick) ?
				next_tick - jiffies : 1);
		/* next_tick is advanced in the fallback branch above. */
	}
	pr_info("esp32s31-radio: scheduler worker stopped\n");
	goto out;

failed:
	atomic_set(&s31_radio_state, ESP32S31_RADIO_FAILED);
	s31_radio_fail_commands();
#endif
out:
	#ifdef CONFIG_ESP32S31_RADIO_BLOBS
	WRITE_ONCE(s31_radio_worker, NULL);
	#endif
	kthread_unuse_mm(&init_mm);
	return 0;
}

/* Debug aid: if the blob gate stays held by the same task for more than a
 * few seconds the payload is stuck in a busy-wait (the holder cannot be
 * preempted by the scheduler on this PREEMPT_NONE kernel).  Dump every
 * compatibility task's kernel stack so the exact spin location is visible. */
static int s31_radio_watchdog_thread(void *unused)
{
	const char *last_holder = NULL;
	unsigned long last_hold = 0;
	unsigned long last_dump = 0;
	struct sched_param param = { };

	(void)unused;
	/* The watchdog dumps s31_linux_task objects that live in HP SRAM
	 * physical addresses.  Use init_mm so those low identity mappings are
	 * active on the watchdog thread as well; without it, task_dump_all()
	 * faults on 0x2f01xxxx on hart0.
	 */
	kthread_use_mm(&init_mm);
	sched_setscheduler_nocheck(current, SCHED_NORMAL, &param);
	set_user_nice(current, 0);
	while (!kthread_should_stop()) {
		const char *holder = s31_linux_blob_holder();

		if (holder && holder != last_holder) {
			last_holder = holder;
			last_hold = jiffies;
			last_dump = 0;
		} else if (holder && time_after(jiffies, last_hold + 6 * HZ) &&
			   time_after(jiffies, last_dump + 6 * HZ)) {
			pr_info("esp32s31-radio: watchdog: gate held by \"%s\" "
				"for >6 s, dumping tasks\n", holder);
			s31_linux_task_dump_all();
			last_dump = jiffies;
		} else if (!holder) {
			last_holder = NULL;
		}
		ssleep(2);
	}
	kthread_unuse_mm(&init_mm);
	return 0;
}

static int __init s31_radio_runtime_init(void)
{
	struct task_struct *task;
	int ret;

	if (esp32s31_radio_is_disabled()) {
		pr_info("esp32s31-radio: disabled by kernel command line\n");
		return 0;
	}

	pr_info("esp32s31-radio: late init entered\n");

#ifdef CONFIG_ESP32S31_RADIO_BLOBS
	/* These sections have an XIP load image but execute/read from HP SRAM.
	 * Linux's generic data copier does not cover the custom low VMA. */
	memcpy(__s31_radio_data_start, __s31_radio_data_load,
	       __s31_radio_data_end - __s31_radio_data_start);
	memset(__s31_radio_bss_start, 0,
	       __s31_radio_bss_end - __s31_radio_bss_start);
	s31_radio_heap_base = ALIGN((unsigned long)__s31_radio_static_end, 64);
	if (s31_radio_heap_base >= S31_RADIO_HEAP_END)
		return -ENOMEM;
	s31_radio_heap_size = S31_RADIO_HEAP_END - s31_radio_heap_base;
	s31_radio_heap_linux = memremap(s31_radio_heap_base,
					s31_radio_heap_size, MEMREMAP_WB);
	if (!s31_radio_heap_linux)
		return -ENOMEM;
	s31_radio_heap_low_linux = memremap(S31_RADIO_HEAP_LOW_BASE,
					    S31_RADIO_HEAP_LOW_SIZE, MEMREMAP_WB);
	if (!s31_radio_heap_low_linux)
		pr_warn("esp32s31-radio: cannot map low heap chunk\n");
	s31_radio_heap2_linux = memremap(S31_RADIO_HEAP2_BASE,
					 S31_RADIO_HEAP2_SIZE, MEMREMAP_WB);
	if (!s31_radio_heap2_linux)
		pr_warn("esp32s31-radio: cannot map heap2 chunk\n");
	s31_radio_heap_pool = gen_pool_create(4, -1);
	if (!s31_radio_heap_pool) {
		memunmap(s31_radio_heap_linux);
		return -ENOMEM;
	}
	ret = gen_pool_add(s31_radio_heap_pool, s31_radio_heap_base,
			   s31_radio_heap_size, -1);
	if (ret) {
		gen_pool_destroy(s31_radio_heap_pool);
		memunmap(s31_radio_heap_linux);
		return ret;
	}
	pr_info("esp32s31-radio: internal static data %#lx..%#lx, bss %#lx..%#lx\n",
		(unsigned long)__s31_radio_data_start,
		(unsigned long)__s31_radio_data_end,
		(unsigned long)__s31_radio_bss_start,
		(unsigned long)__s31_radio_bss_end);
	pr_info("esp32s31-radio: private SRAM heap %#lx..%#lx (%zu bytes)\n",
		s31_radio_heap_base, S31_RADIO_HEAP_END, s31_radio_heap_size);

	/* These rings are accessed from callbacks executing inside the blob gate.
	 * Keep both sides in the same internal-SRAM ownership domain as the blob. */
	s31_hci_rx = s31_radio_sram_alloc(sizeof(*s31_hci_rx) * S31_HCI_RX_SLOTS);
	s31_hci_tx = s31_radio_sram_alloc(sizeof(*s31_hci_tx) * S31_HCI_TX_SLOTS);
	s31_wifi_rx = s31_radio_sram_alloc(sizeof(*s31_wifi_rx) * S31_WIFI_RX_SLOTS);
	s31_wifi_tx = s31_radio_sram_alloc(sizeof(*s31_wifi_tx) * S31_WIFI_TX_SLOTS);
	s31_wifi_connect_params = s31_radio_sram_alloc(sizeof(*s31_wifi_connect_params));
	s31_wifi_disconnect_reason = s31_radio_sram_alloc(sizeof(*s31_wifi_disconnect_reason));
	if (!s31_hci_rx || !s31_hci_tx || !s31_wifi_rx || !s31_wifi_tx ||
	    !s31_wifi_connect_params || !s31_wifi_disconnect_reason) {
		pr_err("esp32s31-radio: cannot allocate blob buffers in SRAM\n");
		ret = -ENOMEM;
		goto free_rings;
	}
	pr_info("esp32s31-radio: SRAM rings hci=%p/%p wifi=%p/%p\n",
		s31_hci_rx, s31_hci_tx, s31_wifi_rx, s31_wifi_tx);
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
		pr_warn("esp32s31-radio: cannot add SRAM heap2 %#x..%#x: %d\n",
			S31_RADIO_HEAP2_BASE, S31_RADIO_HEAP2_END, ret);
	} else {
		pr_info("esp32s31-radio: added SRAM heap2 %#x..%#x (%zu bytes)\n",
			S31_RADIO_HEAP2_BASE, S31_RADIO_HEAP2_END,
			(size_t)S31_RADIO_HEAP2_SIZE);
	}
	/*
	 * Reclaim the parked factory app's FreeRTOS heap as a third (low)
	 * chunk.  The loader now reserves and zeroes it with a D-cache
	 * writeback-invalidate, so unlike the earlier low-chunk experiment the
	 * physical SRAM is clean before the MAC DMA touches any esf_buf that
	 * lands here.  Still added after the rings so the Linux alias is never
	 * asked to translate a pointer into this chunk.
	 */
	ret = gen_pool_add(s31_radio_heap_pool, S31_RADIO_HEAP_LOW_BASE,
			   S31_RADIO_HEAP_LOW_SIZE, -1);
	if (ret) {
		pr_warn("esp32s31-radio: cannot add SRAM heap-low %#x..%#x: %d\n",
			S31_RADIO_HEAP_LOW_BASE, S31_RADIO_HEAP_LOW_END, ret);
	} else {
		pr_info("esp32s31-radio: added SRAM heap-low %#x..%#x (%zu bytes)\n",
			S31_RADIO_HEAP_LOW_BASE, S31_RADIO_HEAP_LOW_END,
			(size_t)S31_RADIO_HEAP_LOW_SIZE);
	}
	task = kthread_create(s31_radio_watchdog_thread, NULL,
			      "s31-radio-watchdog");
	if (!IS_ERR(task)) {
		kthread_bind(task, 0);
		wake_up_process(task);
	} else {
		pr_warn("esp32s31-radio: cannot create watchdog: %ld\n",
			PTR_ERR(task));
	}
#endif

	task = kthread_create(s31_radio_runtime_thread, NULL, "s31-radio");
	if (IS_ERR(task)) {
		ret = PTR_ERR(task);
		goto free_rings;
	}
	#ifdef CONFIG_ESP32S31_RADIO_BLOBS
	WRITE_ONCE(s31_radio_worker, task);
	#endif
	s31_blob_gate_wait_hook = s31_radio_gate_wait_sync;
	kthread_bind(task, 0);
	wake_up_process(task);
	return 0;

#ifdef CONFIG_ESP32S31_RADIO_BLOBS
free_rings:
	s31_radio_sram_free(s31_wifi_tx);
	s31_radio_sram_free(s31_wifi_rx);
	s31_radio_sram_free(s31_hci_tx);
	s31_radio_sram_free(s31_hci_rx);
	s31_wifi_tx = NULL;
	s31_wifi_rx = NULL;
	s31_hci_tx = NULL;
	s31_hci_rx = NULL;
	gen_pool_destroy(s31_radio_heap_pool);
	memunmap(s31_radio_heap_linux);
	return ret;
#else
free_rings:
	return ret;
#endif
}
late_initcall(s31_radio_runtime_init);
