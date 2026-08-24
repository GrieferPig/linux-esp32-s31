// SPDX-License-Identifier: GPL-2.0
/* ESP32-S31 S-mode Wi-Fi/Bluetooth runtime. */

#include <linux/init.h>
#include <linux/completion.h>
#include <linux/cpu.h>
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

#ifdef CONFIG_SMP
extern void esp32s31_irq_poll(void);
#endif

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
		/* This can run inside the closed driver's RX/critical path.  Even a
		 * rate-limited printk has a burst allowance and can serialize that path
		 * behind the console.  Record the failure and report it only from an
		 * explicitly requested, process-context heap report. */
		atomic_inc(&s31_idf_alloc_failures);
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
static atomic_t s31_wifi_rx_direct_pending = ATOMIC_INIT(0);

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
	atomic_t direct_count;
	atomic_t owner_drained_count;
	atomic_t deferred_count;
	atomic_t defer_context_count;
	atomic_t defer_owner_count;
	atomic_t defer_unsafe_count;
	atomic_t defer_nested_count;
	atomic_t mask_count;
	u64 direct_total_ns;
	u64 direct_max_ns;
	u64 pending_since_ns;
	u64 latency_total_ns;
	u64 latency_max_ns;
	u64 gate_total_ns;
	u64 gate_max_ns;
	u64 handler_total_ns;
	u64 handler_max_ns;
};

#define S31_RADIO_IRQ_SLOTS	7
static struct s31_idf_irq_registration s31_idf_irqs[S31_RADIO_IRQ_SLOTS];
static DECLARE_WAIT_QUEUE_HEAD(s31_radio_waitq);

enum s31_radio_command_type {
	S31_RADIO_COMMAND_HEALTH,
	S31_RADIO_COMMAND_WIFI_GET_MAC,
	S31_RADIO_COMMAND_WIFI_SCAN,
	S31_RADIO_COMMAND_WIFI_CONNECT,
	S31_RADIO_COMMAND_WIFI_DISCONNECT,
	S31_RADIO_COMMAND_BT_ENABLE,
	S31_RADIO_COMMAND_BT_DISABLE,
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

#define S31_WIFI_FRAME_SIZE	ESP32S31_RADIO_WIFI_FRAME_MAX
/* Copy out of the closed driver's RX buffer while its callback owns it.  The
 * closed driver does not permit the esf_buf lifetime to extend past callback
 * return, so staging data must live in internal SRAM and be returned at once. */
#define S31_WIFI_RX_SLOTS	32
/* The netdev now stops its queue before this staging ring fills and the radio
 * worker wakes it after consuming entries.  A small ring therefore provides
 * bounded buffering without stealing the SRAM needed by dynamic RX esf_bufs. */
#define S31_WIFI_TX_SLOTS	16

struct s31_wifi_frame {
	u16 length;
	u64 enqueue_ns;
	u8 data[S31_WIFI_FRAME_SIZE];
};

struct s31_wifi_rx_desc {
	u16 length;
};

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
	u64 tcp_ack_enqueue_total_ns;
	u64 tcp_ack_enqueue_max_ns;
	u64 last_isr_end_ns;
	u32 irq_samples;
	u32 gate_samples;
	u32 isr_samples;
	u32 task_samples;
	u32 tx_samples;
	u32 tx_done_samples;
	u32 tcp_ack_samples;
	u32 tx_no_mem;
	u32 tx_done_wakes;
	u32 tcp_ack_resched_kicks;
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

#define S31_TX_TIMING_SLOTS 32
#define S31_RADIO_WORKER_NICE (-10)

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

static struct task_struct *s31_radio_worker;
static struct s31_radio_timing_stats s31_timing;
static void s31_radio_wifi_tx_capacity_available(void);

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
	u8 kind = s31_radio_tx_kind(frame, length);
	u32 sequence;

	if (start_ns >= enqueue_ns) {
		s31_timing_add(start_ns - enqueue_ns,
			       &s31_timing.tx_enqueue_total_ns,
			       &s31_timing.tx_enqueue_max_ns);
		if (kind == S31_TX_TCP_ACK)
			s31_timing_add(start_ns - enqueue_ns,
				       &s31_timing.tcp_ack_enqueue_total_ns,
				       &s31_timing.tcp_ack_enqueue_max_ns);
	}
	s31_timing.tx_samples++;
	if (kind == S31_TX_TCP_ACK)
		s31_timing.tcp_ack_samples++;
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
	sample->kind = kind;
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

	/* A prior esp_wifi_internal_tx() may be waiting for an IDF TX-pool
	 * descriptor.  Completion is the exact capacity signal: retry now rather
	 * than leaving a latency-sensitive TCP ACK parked until the next 10 ms
	 * Linux jiffy.  The callback may run under the blob gate, so only publish
	 * the deadline and wake the already pinned radio worker here. */
	s31_radio_wifi_tx_capacity_available();

	(void)frame;
	(void)length;
}

static void s31_radio_timing_report(const char *stage)
{
	unsigned long flags;
	u32 i, direct = 0, owner_drained = 0, deferred = 0;
	u32 defer_context = 0, defer_owner = 0;
	u32 defer_unsafe = 0, defer_nested = 0;
	u64 direct_total_ns = 0, direct_max_ns = 0;
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
	pr_info("esp32s31-radio: timing %s TCP ACK enqueue->call n=%u avg=%lluns max=%lluns; no_mem=%u done_wakes=%u resched_kicks=%u\n",
		stage, s31_timing.tcp_ack_samples,
		S31_AVG(s31_timing.tcp_ack_enqueue_total_ns,
			s31_timing.tcp_ack_samples),
		s31_timing.tcp_ack_enqueue_max_ns, s31_timing.tx_no_mem,
		s31_timing.tx_done_wakes, s31_timing.tcp_ack_resched_kicks);
	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++) {
		direct += atomic_read(&s31_idf_irqs[i].direct_count);
		owner_drained +=
			atomic_read(&s31_idf_irqs[i].owner_drained_count);
		deferred += atomic_read(&s31_idf_irqs[i].deferred_count);
		defer_context += atomic_read(&s31_idf_irqs[i].defer_context_count);
		defer_owner += atomic_read(&s31_idf_irqs[i].defer_owner_count);
		defer_unsafe += atomic_read(&s31_idf_irqs[i].defer_unsafe_count);
		defer_nested += atomic_read(&s31_idf_irqs[i].defer_nested_count);
		direct_total_ns += READ_ONCE(s31_idf_irqs[i].direct_total_ns);
		direct_max_ns = max(direct_max_ns,
				    READ_ONCE(s31_idf_irqs[i].direct_max_ns));
	}
	pr_info("esp32s31-radio: timing %s ISR dispatch direct=%u avg=%lluns max=%lluns owner-drained=%u deferred=%u context=%u owner=%u unsafe=%u nested=%u\n",
		stage, direct, S31_AVG(direct_total_ns, direct),
		direct_max_ns, owner_drained, deferred, defer_context, defer_owner,
		defer_unsafe, defer_nested);
	pr_info("esp32s31-radio: timing %s RX bridge callbacks=%d delivered=%d staging_dropped=%d\n",
		stage, atomic_read(&s31_wifi_rx_cb),
		atomic_read(&s31_wifi_rx_delivered),
		atomic_read(&s31_wifi_rx_dropped));
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
static u8 *s31_wifi_rx_data;
static struct s31_wifi_frame *s31_wifi_tx;
static unsigned int s31_wifi_rx_head;
static unsigned int s31_wifi_rx_tail;
static unsigned int s31_wifi_tx_head;
static unsigned int s31_wifi_tx_tail;
static unsigned long s31_wifi_tx_retry_at;
static DEFINE_SPINLOCK(s31_wifi_rx_lock);
static DEFINE_SPINLOCK(s31_wifi_tx_lock);
static DECLARE_WAIT_QUEUE_HEAD(s31_wifi_rx_space_waitq);
static struct esp32s31_radio_wifi_connect_params *s31_wifi_connect_params;
static u16 *s31_wifi_disconnect_reason;
static u8 s31_wifi_event_bssid[6];
static u8 s31_wifi_event_channel;
static u16 s31_wifi_event_reason;

static void s31_radio_wifi_tx_capacity_available(void)
{
	if (READ_ONCE(s31_wifi_tx_head) == READ_ONCE(s31_wifi_tx_tail) ||
	    !time_before(jiffies, READ_ONCE(s31_wifi_tx_retry_at)))
		return;
	WRITE_ONCE(s31_wifi_tx_retry_at, jiffies);
	s31_timing.tx_done_wakes++;
	wake_up(&s31_radio_waitq);
}
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
static int s31_bt_disable_result = -EINPROGRESS;
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

void s31_radio_report_bt_disable(int result)
{
	WRITE_ONCE(s31_bt_disable_result, result);
	if (!result)
		WRITE_ONCE(s31_bt_enable_result, -EHOSTDOWN);
	wake_up(&s31_radio_waitq);
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
	registration->flags = flags;
	registration->handler = handler;
	registration->arg = arg;
	registration->install_pending = true;
	registration->enabled = !(flags & S31_IDF_INTR_DISABLED);
	registration->allocated = true;
	atomic_set(&registration->hardirq_count, 0);
	atomic_set(&registration->callback_pending, 0);
	atomic_set(&registration->callback_count, 0);
	atomic_set(&registration->direct_count, 0);
	atomic_set(&registration->owner_drained_count, 0);
	atomic_set(&registration->deferred_count, 0);
	atomic_set(&registration->defer_context_count, 0);
	atomic_set(&registration->defer_owner_count, 0);
	atomic_set(&registration->defer_unsafe_count, 0);
	atomic_set(&registration->defer_nested_count, 0);
	atomic_set(&registration->mask_count, 0);
	registration->direct_total_ns = 0;
	registration->direct_max_ns = 0;
	if (ret_handle)
		*ret_handle = registration;
	pr_info("esp32s31-radio: deferred IDF irq source=%d flags=%#x handler=%pS\n",
		source, flags, handler);
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
		registration->allocated = true;
		atomic_set(&registration->hardirq_count, 0);
		atomic_set(&registration->callback_pending, 0);
		atomic_set(&registration->callback_count, 0);
		atomic_set(&registration->direct_count, 0);
		atomic_set(&registration->owner_drained_count, 0);
		atomic_set(&registration->deferred_count, 0);
		atomic_set(&registration->defer_context_count, 0);
		atomic_set(&registration->defer_owner_count, 0);
		atomic_set(&registration->defer_unsafe_count, 0);
		atomic_set(&registration->defer_nested_count, 0);
		atomic_set(&registration->mask_count, 0);
		registration->direct_total_ns = 0;
		registration->direct_max_ns = 0;
	}
	/* The closed Wi-Fi driver may reserve a source before its OS adapter
	 * supplies the legacy local interrupt number.  Merge both halves of the
	 * registration instead of leaving logical_intr at UINT_MAX. */
	registration->logical_intr = logical_intr;
	if (source == 120)
		pr_info("esp32s31-radio: Wi-Fi IRQ configure source=%u logical=%u enabled=%u\n",
			source, logical_intr,
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
 * Native IDF critical sections prevent the Wi-Fi interrupt from arriving at
 * an unsafe lock-held point.  Linux must preserve that property without
 * globally clearing SIE (which would also suppress the scheduler tick, UART,
 * timer and IPI).  Temporarily add a disable depth only to installed radio
 * IRQs; the caller stores the returned token until its raw lock is released.
 *
 * All payload tasks and radio device IRQs are pinned to hart0 and the blob
 * gate serializes task-side callers, so a small bit token is sufficient.
 */
u32 s31_radio_blob_irqs_mask(void)
{
	u32 mask = 0;
	int i;

	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++) {
		struct s31_idf_irq_registration *registration = &s31_idf_irqs[i];
		int virq = READ_ONCE(registration->virq);

		if (!virq || !READ_ONCE(registration->hw_enabled))
			continue;
		disable_irq_nosync(virq);
		mask |= BIT(i);
	}
	return mask;
}

void s31_radio_blob_irqs_restore(u32 mask)
{
	unsigned long flags;
	int i;

	/* Balance every temporary disable before returning.  Keep local hard
	 * IRQs off across the balance/re-mask pair so a source which the closed
	 * driver logically disabled cannot enter in the middle. */
	local_irq_save(flags);
	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++) {
		struct s31_idf_irq_registration *registration = &s31_idf_irqs[i];
		int virq;

		if (!(mask & BIT(i)))
			continue;
		virq = READ_ONCE(registration->virq);
		if (!virq)
			continue;
		enable_irq(virq);
		if (!READ_ONCE(registration->enabled) &&
		    READ_ONCE(registration->hw_enabled)) {
			WRITE_ONCE(registration->hw_enabled, false);
			disable_irq_nosync(virq);
		}
	}
	local_irq_restore(flags);
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

	if (!s31_linux_blob_held_by_current() ||
	    READ_ONCE(s31_rtos_isr_depth))
		return false;

	local_irq_save(flags);
	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++) {
		struct s31_idf_irq_registration *registration =
			&s31_idf_irqs[i];
		int virq;

		if (!READ_ONCE(registration->handler) ||
		    !atomic_xchg(&registration->callback_pending, 0))
			continue;

		s31_rtos_isr_depth++;
		registration->handler(registration->arg);
		s31_rtos_isr_depth--;
		atomic_inc(&registration->callback_count);
		atomic_inc(&registration->owner_drained_count);
		handled = true;

		/* Balance the explicit disable depth installed by the fallback
		 * hardirq only after the closed ISR has acknowledged the device.  If
		 * the ISR logically disabled its source, retain that disable depth. */
		virq = READ_ONCE(registration->virq);
		if (virq && READ_ONCE(registration->enabled) &&
		    !READ_ONCE(registration->hw_enabled)) {
			WRITE_ONCE(registration->hw_enabled, true);
			enable_irq(virq);
		}
	}
	local_irq_restore(flags);

	return handled;
}

/*
 * A level IRQ is already masked by handle_level_irq() while this action runs.
 * If the interrupted hart0 blob task is at a published safe point, run the
 * closed ISR now: it acknowledges the MAC/DMA source and the generic IRQ flow
 * unmasks the CLIC slot as this action returns.  Otherwise add an explicit
 * disable depth and leave the source masked until the serialized worker has
 * run the closed ISR.
 */
static irqreturn_t s31_radio_hardirq(int irq, void *data)
{
	struct s31_idf_irq_registration *registration = data;
	struct task_struct *worker;
	int direct_result;
	u64 entered_ns, handler_end_ns;

	entered_ns = ktime_get_mono_fast_ns();
	WRITE_ONCE(registration->pending_since_ns, entered_ns);
	atomic_inc(&registration->hardirq_count);
	direct_result = s31_linux_blob_run_direct_isr(registration->handler,
						      registration->arg);
	if (direct_result == S31_DIRECT_ISR_HANDLED) {
		handler_end_ns = ktime_get_mono_fast_ns();
		registration->direct_total_ns += handler_end_ns - entered_ns;
		registration->direct_max_ns = max(registration->direct_max_ns,
						  handler_end_ns - entered_ns);
		atomic_inc(&registration->direct_count);
		atomic_inc(&registration->callback_count);
		atomic_set(&registration->callback_pending, 0);
		/* The closed ISR may have logically masked its source.  In that case
		 * retain an explicit disable depth across handle_level_irq()'s exit;
		 * otherwise its hardware acknowledgement is enough and generic IRQ
		 * completion immediately unmasks the source. */
		if (!READ_ONCE(registration->enabled) &&
		    READ_ONCE(registration->hw_enabled)) {
			disable_irq_nosync(irq);
			WRITE_ONCE(registration->hw_enabled, false);
		}
	} else {
		disable_irq_nosync(irq);
		WRITE_ONCE(registration->hw_enabled, false);
		atomic_inc(&registration->deferred_count);
		switch (direct_result) {
		case S31_DIRECT_ISR_DEFER_CONTEXT:
			atomic_inc(&registration->defer_context_count);
			break;
		case S31_DIRECT_ISR_DEFER_OWNER:
			atomic_inc(&registration->defer_owner_count);
			break;
		case S31_DIRECT_ISR_DEFER_UNSAFE:
			atomic_inc(&registration->defer_unsafe_count);
			break;
		case S31_DIRECT_ISR_DEFER_NESTED:
			atomic_inc(&registration->defer_nested_count);
			break;
		default:
			break;
		}
		atomic_set(&registration->callback_pending, 1);
	}
	/*
	 * The closed ISR acknowledges the hardware directly, but it can also make
	 * compatibility tasks runnable or queue blob-internal follow-up work.  Wake
	 * the serialized worker after both direct and deferred callbacks so that
	 * this software tail cannot remain asleep behind an already-cleared IRQ.
	 */
	worker = READ_ONCE(s31_radio_worker);
	if (worker)
		wake_up_process(worker);
	return IRQ_HANDLED;
}

static void s31_radio_sync_one_irq(struct s31_idf_irq_registration *registration)
{
	struct device_node *intmtx_node;
	struct irq_data *irqd;
	irq_hw_number_t clic_slot = 0;
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
		/* Follow the official irqchip hierarchy: the radio owns an ETS
		 * peripheral source, not a CLIC slot.  INTMTX selects a free CLIC
		 * slot and pins the resulting device IRQ to hart 0. */
		intmtx_node = of_find_compatible_node(NULL, NULL,
						 "espressif,esp32s31-intmtx");
		if (!intmtx_node) {
			pr_err_ratelimited("esp32s31-radio: cannot find S-mode INTMTX domain\n");
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
			return;
		}
		/* Publish the mapping before request_irq(); a live level source
		 * may invoke the hard IRQ handler as soon as it is enabled. */
		registration->virq = virq;
		registration->hw_enabled = true;
		ret = request_irq(virq, s31_radio_hardirq, 0,
				  "esp32s31-radio", registration);
		if (ret) {
			pr_err_ratelimited("esp32s31-radio: request IRQ%d for source %d failed: %d\n",
					   virq, registration->source, ret);
			registration->virq = 0;
			registration->hw_enabled = false;
			irq_dispose_mapping(virq);
			return;
		}
		registration->install_pending = false;
		irqd = irq_get_irq_data(virq);
		if (irqd && irqd->parent_data)
			clic_slot = irqd->parent_data->hwirq;
		pr_info("esp32s31-radio: IDF source %d routed by INTMTX to CLIC%lu/IRQ%d\n",
			registration->source, clic_slot, virq);
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
	const struct esp32s31_radio_wifi_ops *ops;
	void *context;
	unsigned long flags;
	unsigned int next;
	int ret;

	if (!frame || length < ETH_HLEN || length > S31_WIFI_FRAME_SIZE)
		return -EINVAL;
	atomic_inc(&s31_wifi_rx_cb);
	if (eb)
		atomic_inc(&s31_wifi_rx_freed);
	ops = READ_ONCE(s31_wifi_ops);
	context = READ_ONCE(s31_wifi_context);
	if (ops && ops->rx_copy) {
		atomic_inc(&s31_wifi_rx_direct_pending);
		ret = ops->rx_copy(context, frame, length);
		if (ret) {
			atomic_dec(&s31_wifi_rx_direct_pending);
			atomic_inc(&s31_wifi_rx_dropped);
			return ret;
		}
		atomic_inc(&s31_wifi_rx_delivered);
		if (ops->rx_ready)
			ops->rx_ready(context);
		return 0;
	}
	/* Keep RX independent from the TX ACK ring, but retain an AMO-backed lock:
	 * the closed callback can run from either a compat task or the direct ISR,
	 * and S31 ordinary cache accesses are not coherent with all AMO users. */
	spin_lock_irqsave(&s31_wifi_rx_lock, flags);
	next = (s31_wifi_rx_head + 1) % S31_WIFI_RX_SLOTS;
	if (next == smp_load_acquire(&s31_wifi_rx_tail)) {
		atomic_inc(&s31_wifi_rx_dropped);
		spin_unlock_irqrestore(&s31_wifi_rx_lock, flags);
		return -ENOSPC;
	}
	s31_wifi_rx[s31_wifi_rx_head].length = length;
	memcpy(s31_wifi_rx_data + s31_wifi_rx_head * S31_WIFI_FRAME_SIZE,
	       frame, length);
	smp_store_release(&s31_wifi_rx_head, next);
	spin_unlock_irqrestore(&s31_wifi_rx_lock, flags);

	/* The closed driver still owns |frame| and may free/reuse its esf_buf as
	 * soon as this callback returns.  The complete frame is now in the
	 * Linux-owned SRAM ring, so hand only a readiness edge to the netdev. */
	if (ops && ops->rx_ready)
		ops->rx_ready(context);
	return 0;
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

void s31_radio_wifi_rx_throttle(void)
{
	/* The callback has already copied the frame and returned its esf_buf.
	 * Re-create the scheduling opportunity that native FreeRTOS has between
	 * callbacks when the Linux consumer falls one BA window behind. */
		if (s31_radio_wifi_rx_occupancy() < 48)
		return;
	/* A native hard ISR cannot block either.  RX readiness schedules NAPI,
	 * which drains the staging ring immediately after the nested callback
	 * returns. */
	if (in_hardirq())
		return;
	s31_linux_blob_suspend(S31_BLOB_RELEASE_QUEUE_SEND);
	wait_event_timeout(s31_wifi_rx_space_waitq,
			    s31_radio_wifi_rx_occupancy() <= 24,
			msecs_to_jiffies(20));
	s31_linux_blob_resume();
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
	unsigned int budget = 4;

	while (budget-- &&
	       s31_wifi_tx_tail != smp_load_acquire(&s31_wifi_tx_head)) {
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
			s31_timing.tx_no_mem++;
			s31_wifi_tx_retry_at = jiffies + 1;
			return;
		}
		if (ret) {
			atomic_inc(&s31_wifi_tx_dropped);
			pr_warn_ratelimited("esp32s31-radio: dropping Wi-Fi TX frame: %d\n",
					    ret);
		}
		/* esp_wifi_internal_tx() copies the Ethernet frame into its dynamic
		 * TX pool.  Its return code, not the later global tx_done callback,
		 * defines the lifetime of this staging slot. */
		s31_wifi_tx_tail = (s31_wifi_tx_tail + 1) % S31_WIFI_TX_SLOTS;
	}

	/* The four-frame budget bounds one blob-gate hold.  Budget exhaustion is
	 * not backpressure: run another pass immediately and let cond_resched()
	 * provide Linux fairness between passes.  Only a real IDF NO_MEM result
	 * above waits one jiffy before retrying. */
	if (s31_wifi_tx_tail != smp_load_acquire(&s31_wifi_tx_head))
		s31_wifi_tx_retry_at = jiffies;
}

static bool s31_radio_wifi_tx_pending(void)
{
	return READ_ONCE(s31_wifi_tx_head) != READ_ONCE(s31_wifi_tx_tail) &&
	       time_after_eq(jiffies, READ_ONCE(s31_wifi_tx_retry_at));
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
		ops->connected(s31_wifi_context, s31_wifi_connect_status,
			       s31_wifi_event_bssid, s31_wifi_event_channel);
	}
	if (s31_wifi_disconnected_ready) {
		s31_wifi_disconnected_ready = false;
		s31_radio_timing_report("disconnect");
		s31_linux_gate_timing_report("disconnect");
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
		atomic_inc(&s31_wifi_rx_dropped);
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
	atomic_inc(&s31_wifi_rx_delivered);
	wake_up(&s31_wifi_rx_space_waitq);
	return length;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_wifi_rx_dequeue);

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
		case S31_RADIO_COMMAND_BT_DISABLE:
#ifdef CONFIG_BT_ESP32S31
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
	    !ops->disconnected || !ops->rx_ready)
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

int esp32s31_radio_bt_disable(void)
{
#ifndef CONFIG_BT_ESP32S31
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
	spin_lock(&s31_radio_command_lock);
	list_add_tail(&command.node, &s31_radio_commands);
	spin_unlock(&s31_radio_command_lock);
	wake_up(&s31_radio_waitq);
	wait_for_completion(&command.done);
	if (command.result)
		return command.result;
	if (!wait_event_timeout(s31_radio_waitq,
				READ_ONCE(s31_bt_disable_result) != -EINPROGRESS,
				2 * HZ))
		return -ETIMEDOUT;
	return READ_ONCE(s31_bt_disable_result);
#endif
}
EXPORT_SYMBOL_GPL(esp32s31_radio_bt_disable);

int esp32s31_radio_wifi_send(const u8 *frame, size_t length)
{
	unsigned long flags;
	unsigned int next;
	u8 kind;

	if (!frame || length < ETH_HLEN || length > S31_WIFI_FRAME_SIZE)
		return -EINVAL;
	kind = s31_radio_tx_kind(frame, length);
	spin_lock_irqsave(&s31_wifi_tx_lock, flags);
	next = (s31_wifi_tx_head + 1) % S31_WIFI_TX_SLOTS;
	if (next == s31_wifi_tx_tail) {
		spin_unlock_irqrestore(&s31_wifi_tx_lock, flags);
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
	spin_unlock_irqrestore(&s31_wifi_tx_lock, flags);
	wake_up(&s31_radio_waitq);
	/* Under PREEMPT_VOLUNTARY a same-hart CFS task can otherwise run until
	 * the next 10 ms tick even though the ACK just woke the TX worker.  Ask
	 * for one reschedule at the next safe kernel/softirq exit.  Never do this
	 * while current owns the blob gate: the worker needs that task to resume
	 * and release the gate before esp_wifi_internal_tx() can run. */
	if (kind == S31_TX_TCP_ACK && raw_smp_processor_id() == 0 &&
	    current != READ_ONCE(s31_radio_worker) &&
	    !s31_linux_blob_held_by_current() &&
	    !test_tsk_need_resched(current)) {
		set_tsk_need_resched(current);
		s31_timing.tcp_ack_resched_kicks++;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_wifi_send);

bool esp32s31_radio_wifi_tx_has_space(void)
{
	unsigned long flags;
	bool has_space;

	spin_lock_irqsave(&s31_wifi_tx_lock, flags);
	has_space = (s31_wifi_tx_head + 1) % S31_WIFI_TX_SLOTS !=
		    s31_wifi_tx_tail;
	spin_unlock_irqrestore(&s31_wifi_tx_lock, flags);
	return has_space;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_wifi_tx_has_space);

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
	atomic_inc(&s31_radio_worker_passes);
	s31_linux_blob_leave();
	if (irq_pass) {
		sched_setscheduler_nocheck(current, SCHED_NORMAL, &cfs_param);
		set_user_nice(current, S31_RADIO_WORKER_NICE);
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
	s31_radio_wifi_deliver_control_events();

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
	/* The worker services TCP ACK TX and every IRQ that cannot use the owner-only
	 * direct path.  Keep it in ordinary CFS so the closed Wi-Fi task can still
	 * run, with enough weight to service a freshly queued ACK promptly without
	 * starving that task under sustained receive load. */
	sched_setscheduler_nocheck(current, SCHED_NORMAL, &param);
	set_user_nice(current, S31_RADIO_WORKER_NICE);
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
#ifdef CONFIG_SMP
		/* A direct S-mode CLIC source can remain pending after an SBI cache
		 * ecall because of S31's cross-privilege 0xff sentinel.  The radio
		 * worker keeps hart 0 runnable, so the idle poll fallback never gets
		 * a chance to drain it.  Service pending native sources at this bounded
		 * scheduling point; their direct hardware paths remain the fast path. */
		esp32s31_irq_poll();
#endif
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
		/* The worker is SCHED_NORMAL, so the scheduler tick supplies fair
		 * preemption.  Do not force a context switch after every radio IRQ:
		 * without RX aggregation that limits receive progress to roughly one
		 * Ethernet frame per scheduling round.  cond_resched() still yields to
		 * a runnable compatibility task immediately and to ordinary Linux work
		 * whenever the CFS slice expires. */
		cond_resched();
		wait_event_interruptible_timeout(s31_radio_waitq,
			kthread_should_stop() ||
			atomic_read(&s31_tick_pending) ||
			s31_radio_irq_work_pending() ||
			s31_radio_command_work_pending() ||
				s31_radio_hci_tx_pending() ||
			s31_radio_wifi_tx_pending(),
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
	/* Wi-Fi esf_bufs (~1848 bytes), BT controller objects and small RTOS
	 * bridge allocations have very different lifetimes.  First-fit leaves
	 * the three non-contiguous SRAM chunks peppered with holes and can reject
	 * an RX buffer while tens of KiB remain free.  Best-fit preserves the
	 * larger extents needed by the RX pool during sustained traffic. */
	gen_pool_set_algo(s31_radio_heap_pool, gen_pool_best_fit, NULL);
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
	s31_wifi_rx_data = s31_radio_sram_alloc(S31_WIFI_RX_SLOTS *
						 S31_WIFI_FRAME_SIZE);
	s31_wifi_tx = s31_radio_sram_alloc(sizeof(*s31_wifi_tx) * S31_WIFI_TX_SLOTS);
	s31_wifi_connect_params = s31_radio_sram_alloc(sizeof(*s31_wifi_connect_params));
	s31_wifi_disconnect_reason = s31_radio_sram_alloc(sizeof(*s31_wifi_disconnect_reason));
	if (!s31_hci_rx || !s31_hci_tx || !s31_wifi_rx || !s31_wifi_rx_data ||
	    !s31_wifi_tx ||
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
	s31_radio_sram_free(s31_wifi_rx_data);
	s31_radio_sram_free(s31_wifi_tx);
	s31_radio_sram_free(s31_wifi_rx);
	s31_radio_sram_free(s31_hci_tx);
	s31_radio_sram_free(s31_hci_rx);
	s31_wifi_tx = NULL;
	s31_wifi_rx = NULL;
	s31_wifi_rx_data = NULL;
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
