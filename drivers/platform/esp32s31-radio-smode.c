// SPDX-License-Identifier: GPL-2.0
/* ESP32-S31 S-mode Wi-Fi/Bluetooth runtime. */

#include <linux/init.h>
#include <linux/genalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irqdomain.h>
#include <linux/jiffies.h>
#include <linux/kallsyms.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <asm/fpu.h>

#define S31_ROM_BASE		0x2f800000UL
#define S31_ROM_CPU_FREQ	0x2f800040UL

typedef unsigned int (*s31_rom_cpu_freq_t)(void);

#ifdef CONFIG_ESP32S31_RADIO_BLOBS
extern void s31_radio_stack_task(void *arg);
extern void s31_radio_bt_enable_task(void *arg);
extern void s31_rtos_init(void);
extern void s31_rtos_schedule(void);
extern void s31_rtos_tick(void);
extern u32 s31_rtos_isr_depth;
extern int xTaskCreatePinnedToCore(void (*task)(void *), const char *name,
				   u32 stack_size, void *arg, u32 priority,
				   void *task_handle, int core_id);

#define S31_RADIO_IRAM_BASE	0x2f030000UL
/* The loader carves out 0x2f030000..0x2f072380 exclusively for Linux radio.
 * Keep the final 0xb80 bytes for the synchronous-trap stack; the AXI
 * descriptor reservation starts at 0x2f072380 and is never part of this pool. */
#define S31_RADIO_IRAM_END	0x2f071800UL
#define S31_RADIO_IRAM_SIZE	(S31_RADIO_IRAM_END - S31_RADIO_IRAM_BASE)
#define S31_RADIO_EXC_STACK_TOP	0x2f072360UL

static struct gen_pool *s31_radio_iram_pool;
static size_t s31_radio_heap_peak;

void *__wrap_heap_caps_malloc(size_t size, u32 caps);
void __wrap_heap_caps_free(void *ptr);
void *__wrap_heap_caps_calloc(size_t n, size_t size, u32 caps);
void *__wrap_heap_caps_realloc(void *ptr, size_t size, u32 caps);
void *__wrap_heap_caps_aligned_alloc(size_t alignment, size_t size, u32 caps);
void *__wrap_heap_caps_aligned_calloc(size_t alignment, size_t n,
				       size_t size, u32 caps);
void *__wrap_heap_caps_malloc_default(size_t size);
void *__wrap_heap_caps_realloc_default(void *ptr, size_t size);
void *__wrap_heap_caps_malloc_prefer(size_t size, size_t count, ...);
size_t __wrap_heap_caps_get_free_size(u32 caps);
void s31_radio_heap_report(const char *stage);
void _interrupt_handler(void);
void *intr_handler_get(int int_no);
u32 __wrap_esp_log_timestamp(void);
bool __wrap_esp_intr_ptr_in_isr_region(void *ptr);
int __wrap_esp_intr_alloc(int source, int flags, void (*handler)(void *),
			  void *arg, void **ret_handle);
int __wrap_esp_intr_enable(void *handle);
int __wrap_esp_intr_disable(void *handle);
int __wrap_esp_intr_free(void *handle);

struct s31_idf_alloc_header {
	u32 magic;
	size_t size;
	size_t total;
	void *base;
};

#define S31_IDF_ALLOC_MAGIC	0x5333414c

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
	base = (void *)gen_pool_alloc(s31_radio_iram_pool, total);
	if (!base)
		return NULL;
	s31_radio_heap_peak = max(s31_radio_heap_peak,
		S31_RADIO_IRAM_SIZE - gen_pool_avail(s31_radio_iram_pool));
	ptr = PTR_ALIGN((u8 *)base + sizeof(*header), alignment);
	header = ptr - sizeof(*header);
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
	void *ptr = s31_idf_alloc(size, sizeof(void *), false, caps);

	if (!ptr)
		pr_err("esp32s31-radio: heap malloc size=%zu caps=%#x failed\n",
		       size, caps);
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
	gen_pool_free(s31_radio_iram_pool, (unsigned long)header->base,
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
	return gen_pool_avail(s31_radio_iram_pool);
}

void s31_radio_heap_report(const char *stage)
{
	size_t free = gen_pool_avail(s31_radio_iram_pool);

	pr_info("esp32s31-radio: SRAM heap %s used=%zu peak=%zu free=%zu total=%lu\n",
		stage, (size_t)(S31_RADIO_IRAM_SIZE - free), s31_radio_heap_peak,
		free, S31_RADIO_IRAM_SIZE);
}

void _interrupt_handler(void)
{
}

const unsigned long _mtvt_table[48] = {
	[0 ... 47] = (unsigned long)_interrupt_handler,
};

void *intr_handler_get(int int_no)
{
	return NULL;
}

u32 __wrap_esp_log_timestamp(void)
{
	return jiffies_to_msecs(jiffies);
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
	unsigned int hwirq;
	int flags;
	void (*handler)(void *);
	void *arg;
	int virq;
	bool install_pending;
	bool free_pending;
	bool enabled;
	bool hw_enabled;
	atomic_t callback_pending;
	atomic_t hardirq_count;
	atomic_t callback_count;
};

#define S31_RADIO_IRQ_SLOTS	4
static const unsigned int s31_radio_hwirqs[S31_RADIO_IRQ_SLOTS] = {
	47, 45, 44, 43,
};
static struct s31_idf_irq_registration s31_idf_irqs[S31_RADIO_IRQ_SLOTS];
static DECLARE_WAIT_QUEUE_HEAD(s31_radio_waitq);

#define S31_IDF_INTR_DISABLED	BIT(5)
#define S31_TICK_CLIC_HWIRQ	46
#define S31_TIMG1_T1_SOURCE	29
#define S31_TIMG1_BASE		0x20581000UL
#define S31_HP_CLKRST_BASE	0x20587000UL
#define S31_HP_CLKRST_TIMG1	0x11c
#define S31_TIMG1_APB_CLK_EN	BIT(0)
#define S31_TIMG1_T1_SRC_MASK	GENMASK(7, 6)
#define S31_TIMG1_T1_CLK_EN	BIT(8)
#define S31_TIMG_T1_CONFIG	0x24
#define S31_TIMG_T1_ALARM_LO	0x34
#define S31_TIMG_T1_ALARM_HI	0x38
#define S31_TIMG_T1_LOAD_LO	0x3c
#define S31_TIMG_T1_LOAD_HI	0x40
#define S31_TIMG_T1_LOAD		0x44
#define S31_TIMG_INT_ENA		0x70
#define S31_TIMG_INT_CLR		0x7c
#define S31_TIMG_T1_INT		BIT(1)
#define S31_TIMG_T1_ALARM_EN	BIT(10)
#define S31_TIMG_T1_DIV_RST	BIT(12)
#define S31_TIMG_T1_DIVIDER(x)	((x) << 13)
#define S31_TIMG_T1_AUTORELOAD	BIT(29)
#define S31_TIMG_T1_INCREASE	BIT(30)
#define S31_TIMG_T1_ENABLE	BIT(31)

static void __iomem *s31_timg1;
static int s31_tick_virq;
static atomic_t s31_tick_pending = ATOMIC_INIT(0);
static atomic_t s31_tick_irq_count = ATOMIC_INIT(0);
static bool s31_bt_enable_started;

static int s31_radio_map_clic_irq(unsigned int hwirq, unsigned int source)
{
	struct device_node *clic_node;
	struct irq_fwspec fwspec = { };
	int virq;

	clic_node = of_find_compatible_node(NULL, NULL,
					"espressif,esp32s31-clic");
	if (!clic_node)
		return 0;
	fwspec.fwnode = of_node_to_fwnode(clic_node);
	fwspec.param_count = 3;
	fwspec.param[0] = hwirq;
	fwspec.param[1] = source;
	fwspec.param[2] = IRQ_TYPE_LEVEL_HIGH;
	virq = irq_create_fwspec_mapping(&fwspec);
	of_node_put(clic_node);
	return virq;
}

static irqreturn_t s31_tick_hardirq(int irq, void *data)
{
	u32 config;

	writel(S31_TIMG_T1_INT, s31_timg1 + S31_TIMG_INT_CLR);
	/* ALARM_EN is self-clearing even in auto-reload mode. */
	config = readl(s31_timg1 + S31_TIMG_T1_CONFIG);
	writel(config | S31_TIMG_T1_ALARM_EN,
	       s31_timg1 + S31_TIMG_T1_CONFIG);
	atomic_inc(&s31_tick_irq_count);
	atomic_inc(&s31_tick_pending);
	wake_up(&s31_radio_waitq);
	return IRQ_HANDLED;
}

static int s31_radio_tick_init(void)
{
	void __iomem *clkrst;
	u32 config;
	u32 clk_config;
	int ret;

	s31_timg1 = ioremap(S31_TIMG1_BASE, 0x100);
	if (!s31_timg1)
		return -ENOMEM;
	s31_tick_virq = s31_radio_map_clic_irq(S31_TICK_CLIC_HWIRQ,
						 S31_TIMG1_T1_SOURCE);
	if (!s31_tick_virq) {
		ret = -EINVAL;
		goto err_unmap;
	}
	ret = request_irq(s31_tick_virq, s31_tick_hardirq, 0,
			  "esp32s31-radio-tick", s31_timg1);
	if (ret)
		goto err_mapping;

	clkrst = ioremap(S31_HP_CLKRST_BASE, 0x200);
	if (!clkrst) {
		ret = -ENOMEM;
		goto err_irq;
	}
	clk_config = readl(clkrst + S31_HP_CLKRST_TIMG1);
	clk_config &= ~S31_TIMG1_T1_SRC_MASK; /* XTAL, 40 MHz */
	clk_config |= S31_TIMG1_APB_CLK_EN | S31_TIMG1_T1_CLK_EN;
	writel(clk_config, clkrst + S31_HP_CLKRST_TIMG1);
	iounmap(clkrst);

	/* TIMG1 is already clocked by its watchdog device.  Divide the 40-MHz
	 * APB clock to 1 kHz and alarm every 10 counts for a 100-Hz RTOS tick. */
	writel(0, s31_timg1 + S31_TIMG_T1_CONFIG);
	writel(readl(s31_timg1 + S31_TIMG_INT_ENA) | S31_TIMG_T1_INT,
	       s31_timg1 + S31_TIMG_INT_ENA);
	writel(S31_TIMG_T1_INT, s31_timg1 + S31_TIMG_INT_CLR);
	writel(0, s31_timg1 + S31_TIMG_T1_LOAD_LO);
	writel(0, s31_timg1 + S31_TIMG_T1_LOAD_HI);
	writel(1, s31_timg1 + S31_TIMG_T1_LOAD);
	writel(10, s31_timg1 + S31_TIMG_T1_ALARM_LO);
	writel(0, s31_timg1 + S31_TIMG_T1_ALARM_HI);
	config = S31_TIMG_T1_DIVIDER(40000) | S31_TIMG_T1_DIV_RST |
		 S31_TIMG_T1_AUTORELOAD | S31_TIMG_T1_INCREASE |
		 S31_TIMG_T1_ALARM_EN | S31_TIMG_T1_ENABLE;
	writel(config, s31_timg1 + S31_TIMG_T1_CONFIG);
	pr_info("esp32s31-radio: TIMG1/T1 tick routed to CLIC%d/IRQ%d at 100 Hz\n",
		S31_TICK_CLIC_HWIRQ, s31_tick_virq);
	return 0;

err_irq:
	free_irq(s31_tick_virq, s31_timg1);
err_mapping:
	irq_dispose_mapping(s31_tick_virq);
	s31_tick_virq = 0;
err_unmap:
	iounmap(s31_timg1);
	s31_timg1 = NULL;
	return ret;
}

int __wrap_esp_intr_alloc(int source, int flags, void (*handler)(void *),
			  void *arg, void **ret_handle)
{
	struct s31_idf_irq_registration *registration = NULL;
	int i;

	if (!handler)
		return -EINVAL;
	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++) {
		if (!s31_idf_irqs[i].handler) {
			registration = &s31_idf_irqs[i];
			break;
		}
	}
	if (!registration)
		return -ENOMEM;
	registration->source = source;
	registration->hwirq = s31_radio_hwirqs[i];
	registration->flags = flags;
	registration->handler = handler;
	registration->arg = arg;
	registration->install_pending = true;
	registration->enabled = !(flags & S31_IDF_INTR_DISABLED);
	atomic_set(&registration->callback_pending, 0);
	atomic_set(&registration->hardirq_count, 0);
	atomic_set(&registration->callback_count, 0);
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
		if (handle == &s31_idf_irqs[i] && s31_idf_irqs[i].handler)
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

static irqreturn_t s31_radio_hardirq(int irq, void *data)
{
	struct s31_idf_irq_registration *registration = data;

	/* The blob ISR is XIP code and may use the compatibility RTOS and FP.
	 * Keep all of that out of hardirq context.  Holding the CLIC slot masked
	 * also avoids the S31's unsupported cross-privilege interrupt nesting. */
	disable_irq_nosync(irq);
	WRITE_ONCE(registration->hw_enabled, false);
	atomic_inc(&registration->hardirq_count);
	atomic_set(&registration->callback_pending, 1);
	wake_up(&s31_radio_waitq);
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
		ret = request_irq(virq, s31_radio_hardirq, 0, "esp32s31-radio",
				  registration);
		if (ret) {
			pr_err("esp32s31-radio: request IRQ%d failed: %d\n",
			       virq, ret);
			irq_dispose_mapping(virq);
			return;
		}
		registration->virq = virq;
		registration->install_pending = false;
		registration->hw_enabled = true;
		pr_info("esp32s31-radio: IDF source %d routed to CLIC%d/IRQ%d\n",
			registration->source, registration->hwirq, virq);
	}

	if (!registration->virq)
		return;
	if (READ_ONCE(registration->enabled) && !registration->hw_enabled &&
	    !atomic_read(&registration->callback_pending)) {
		enable_irq(registration->virq);
		registration->hw_enabled = true;
	} else if (!READ_ONCE(registration->enabled) &&
		   registration->hw_enabled) {
		disable_irq_nosync(registration->virq);
		registration->hw_enabled = false;
	}
}

static void s31_radio_sync_irq_registrations(void)
{
	int i;

	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++)
		s31_radio_sync_one_irq(&s31_idf_irqs[i]);
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

static struct s31_idf_irq_registration *s31_radio_source_irq(int source)
{
	int i;

	for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++)
		if (s31_idf_irqs[i].handler && s31_idf_irqs[i].source == source)
			return &s31_idf_irqs[i];
	return NULL;
}

/* Optional IDF tables are empty in the built-in radio payload. */
asm(".globl _esp_err_msg_tbl_start\n"
    ".globl _esp_err_msg_tbl_end\n"
    ".set _esp_err_msg_tbl_start, 0\n"
    ".set _esp_err_msg_tbl_end, 0\n");
#endif

static int s31_radio_runtime_thread(void *unused)
{
	void __iomem *rom;
	unsigned long irq_flags;
	unsigned long kernel_sp;
	unsigned long next_tick;
	const unsigned long tick_period = max_t(unsigned long, 1,
							msecs_to_jiffies(10));
	bool hw_tick;
	u32 identity_word, ioremap_word;
	unsigned int cpu_mhz;
	int i;

	pr_info("esp32s31-radio: worker entered\n");
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
	pr_info("esp32s31-radio: ILP32F Wi-Fi/BT payload linked\n");
	pr_info("esp32s31-radio: initializing compatibility RTOS\n");
	s31_rtos_init();
	pr_info("esp32s31-radio: compatibility RTOS initialized\n");
	if (xTaskCreatePinnedToCore(s31_radio_stack_task, "radio-init", 8192,
				    NULL, 24, NULL, 0) != 1) {
		pr_err("esp32s31-radio: cannot create IDF init task\n");
		goto out;
	}
	hw_tick = s31_radio_tick_init() == 0;
	if (!hw_tick)
		pr_warn("esp32s31-radio: TIMG1/T1 unavailable, using jiffies tick\n");
	/*
	 * The cooperative IDF compatibility scheduler changes to private task
	 * stacks. kernel_fpu_begin() both owns the FP registers and prevents a
	 * Linux context switch until all ready IDF tasks have blocked.
	 */
	pr_info("esp32s31-radio: entering first scheduler pass\n");
	/*
	 * IDF PHY bring-up can transiently reconfigure the shared external-memory
	 * path.  A trap taken with thread_info.kernel_sp in PSRAM would then fault
	 * recursively before Linux can save sepc/scause.  Keep the trap frame in
	 * reserved HP SRAM for the whole blob call window.  Local interrupts stay
	 * off until the Linux-owned CLIC dispatch path is installed; synchronous
	 * faults still enter normally and are now diagnosable.
	 */
	kernel_sp = current->thread_info.kernel_sp;
	next_tick = jiffies + tick_period;
	for (;;) {
		int tick_events = atomic_xchg(&s31_tick_pending, 0);

		while (tick_events-- > 0)
			s31_rtos_tick();

		current->thread_info.kernel_sp = S31_RADIO_EXC_STACK_TOP;
		local_irq_save(irq_flags);
		kernel_fpu_begin();
		for (i = 0; i < S31_RADIO_IRQ_SLOTS; i++) {
			struct s31_idf_irq_registration *registration =
				&s31_idf_irqs[i];

			if (!registration->handler ||
			    !atomic_xchg(&registration->callback_pending, 0))
				continue;
			s31_rtos_isr_depth++;
			registration->handler(registration->arg);
			s31_rtos_isr_depth--;
			if (atomic_inc_return(&registration->callback_count) <= 4)
				pr_info("esp32s31-radio: source %d worker IRQ %d/%d\n",
					registration->source,
					atomic_read(&registration->callback_count),
					atomic_read(&registration->hardirq_count));
		}
		s31_rtos_schedule();
		kernel_fpu_end();
		local_irq_restore(irq_flags);
		current->thread_info.kernel_sp = kernel_sp;

		s31_radio_sync_irq_registrations();
		if (kthread_should_stop())
			break;
		if (s31_radio_source_irq(127) &&
		    s31_radio_source_irq(127)->virq && !s31_bt_enable_started) {
			if (xTaskCreatePinnedToCore(s31_radio_bt_enable_task,
						    "bt-enable", 4096, NULL, 24,
						    NULL, 0) != 1) {
				pr_err("esp32s31-radio: cannot create BT enable task\n");
				break;
			}
			s31_bt_enable_started = true;
			pr_info("esp32s31-radio: starting BT after IRQ route is live\n");
			continue;
		}
		wait_event_interruptible_timeout(s31_radio_waitq,
			kthread_should_stop() ||
			atomic_read(&s31_tick_pending) ||
			s31_radio_irq_work_pending(),
			time_before(jiffies, next_tick) ?
				next_tick - jiffies : 1);
		while (!hw_tick && time_after_eq(jiffies, next_tick)) {
			s31_rtos_tick();
			next_tick += tick_period;
		}
		if (hw_tick)
			next_tick = jiffies + tick_period;
	}
	pr_info("esp32s31-radio: scheduler worker stopped\n");
#endif
out:
	kthread_unuse_mm(&init_mm);
	return 0;
}

static int __init s31_radio_runtime_init(void)
{
	struct task_struct *task;
	int ret;

	pr_info("esp32s31-radio: late init entered\n");

#ifdef CONFIG_ESP32S31_RADIO_BLOBS
	s31_radio_iram_pool = gen_pool_create(4, -1);
	if (!s31_radio_iram_pool)
		return -ENOMEM;
	ret = gen_pool_add(s31_radio_iram_pool, S31_RADIO_IRAM_BASE,
			   S31_RADIO_IRAM_SIZE, -1);
	if (ret) {
		gen_pool_destroy(s31_radio_iram_pool);
		return ret;
	}
	pr_info("esp32s31-radio: private SRAM heap %#lx..%#lx (%lu bytes)\n",
		S31_RADIO_IRAM_BASE, S31_RADIO_IRAM_END,
		S31_RADIO_IRAM_SIZE);
#endif

	task = kthread_run(s31_radio_runtime_thread, NULL, "s31-radio");
	if (IS_ERR(task)) {
	#ifdef CONFIG_ESP32S31_RADIO_BLOBS
		gen_pool_destroy(s31_radio_iram_pool);
	#endif
		return PTR_ERR(task);
	}
	return 0;
}
late_initcall(s31_radio_runtime_init);
