// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S31 SMP-local interrupts.
 *
 * Keep these fixed per-hart routes out of the generic CLIC and INTMTX
 * domains: normal device IRQs remain dynamically allocated and pinned to
 * hart 0, while each hart owns one doorbell slot and one direct SYSTIMER
 * slot.  Timer and IPI never share a CLIC entry.
 */

#include <linux/types.h>
#include <linux/completion.h>
#include <linux/kthread.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/sched/task.h>
#include <linux/cpu.h>
#include <linux/init.h>
#include <linux/clocksource/esp32s31-systimer.h>
#include <linux/cpuhotplug.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqflags.h>
#include <linux/hardirq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/percpu.h>
#include <linux/soc/espressif/esp32s31-pm.h>
#include <linux/smp.h>
#include <linux/string.h>

#include <asm/irq_regs.h>
#include <asm/irq_stack.h>
#include <asm/stacktrace.h>
#include <asm/ptrace.h>
#include <asm/sbi.h>
#include <asm/smp.h>
#include <asm/tlbflush.h>
#include <asm/esp32s31_tlb_fence.h>

#include "irq-esp32s31-internal.h"

#define ESP32S31_IPI_DOORBELL_BASE	0x20586010U
#define ESP32S31_IPI_DOORBELL_SIZE	0x8U
#define ESP32S31_IPI_TO_CPU0_OFF		0x0U
#define ESP32S31_IPI_TO_CPU1_OFF		0x4U

#define ESP32S31_IPI_TO_CPU0_SOURCE	65U
#define ESP32S31_IPI_TO_CPU1_SOURCE	66U
#define ESP32S31_SYSTIMER_CPU0_SOURCE	33U
#define ESP32S31_SYSTIMER_CPU1_SOURCE	34U

#define ESP32S31_SBI_EXT_CLIC		0x09000003UL
#define ESP32S31_SBI_CLIC_MINTSTATUS	0
#define ESP32S31_SBI_CLIC_MINTTHRESH	1
#define ESP32S31_SBI_CLIC_MIP		2
#define ESP32S31_SBI_CLIC_MIE		3
#define ESP32S31_SBI_CLIC_WFI		4

static void __iomem *esp32s31_doorbells;
/* Keep real WFI command-line gated so older OpenSBI images fail safe. */
static bool esp32s31_idle_requested;
static bool esp32s31_idle_proxy_ready;

static int __init esp32s31_idle_setup(char *str)
{
	esp32s31_idle_requested = !strcmp(str, "wfi");
	return 0;
}
early_param("esp32s31_idle", esp32s31_idle_setup);

bool noinstr esp32s31_sbi_idle_enabled(void)
{
	return esp32s31_idle_proxy_ready;
}

int esp32s31_sbi_idle_activate(void)
{
	long probe;

	if (!esp32s31_idle_requested)
		return -ENODEV;
	/* OpenSBI owns one timer-group guard comparator per hart, so it can enter
	 * WFI without using the shared CLINT alias or relying on delegated
	 * interrupts to cross an M-mode WFI boundary.
	 */
	probe = sbi_probe_extension(ESP32S31_SBI_EXT_CLIC);
	if (probe <= 0)
		return -ENODEV;

	/* Runtime timers are initialized before the cpuidle device initcall. */
	esp32s31_idle_proxy_ready = true;
	cpu_idle_poll_ctrl(false);
	pr_info("esp32s31-smp: concurrent per-hart WFI enabled (M-mode guard)\n");
	return 0;
}

void esp32s31_sbi_idle_deactivate(void)
{
	esp32s31_idle_proxy_ready = false;
	cpu_idle_poll_ctrl(true);
}

static unsigned long esp32s31_clic_sbi_read(unsigned long funcid)
{
	struct sbiret ret;

	ret = sbi_ecall(ESP32S31_SBI_EXT_CLIC, funcid,
			0, 0, 0, 0, 0, 0);
	return ret.error ? ~0UL : ret.value;
}

static void esp32s31_clic_log_state(unsigned int cpu)
{
	pr_info("esp32s31-smp: hart%u CLIC mintstatus=%#lx mintthresh=%#lx mip=%#lx mie=%#lx\n",
		cpu,
		esp32s31_clic_sbi_read(ESP32S31_SBI_CLIC_MINTSTATUS),
		esp32s31_clic_sbi_read(ESP32S31_SBI_CLIC_MINTTHRESH),
		esp32s31_clic_sbi_read(ESP32S31_SBI_CLIC_MIP),
		esp32s31_clic_sbi_read(ESP32S31_SBI_CLIC_MIE));
}

void noinstr esp32s31_sbi_wfi(void)
{
	register unsigned long a0 asm("a0") = 0;
	register unsigned long a1 asm("a1") = 0;
	register unsigned long a2 asm("a2") = 0;
	register unsigned long a3 asm("a3") = 0;
	register unsigned long a4 asm("a4") = 0;
	register unsigned long a5 asm("a5") = 0;
	register unsigned long a6 asm("a6") = ESP32S31_SBI_CLIC_WFI;
	register unsigned long a7 asm("a7") = ESP32S31_SBI_EXT_CLIC;

	/* Keep the idle core's IRQ-disabled entry atomic with respect to wake
	 * events. Enabling IRQs here can consume an IPI/timer before the SBI
	 * boundary masks SIE again, then enter WFI after that wake was handled.
	 * The private M-level guard wakes with SIE clear; cpuidle enables IRQs
	 * after return. Preserve the existing SBI trap-state recovery sequence.
	 */
	raw_local_irq_disable();
	asm volatile(RISCV_SBI_ECALL_ASM
		     : "+r"(a0), "+r"(a1)
		     : "r"(a2), "r"(a3), "r"(a4), "r"(a5),
		       "r"(a6), "r"(a7)
		     : RISCV_SBI_ECALL_CLOBBERS);
	raw_local_irq_disable();
}

static unsigned int esp32s31_doorbell_offset(unsigned int cpu)
{
	return cpu ? ESP32S31_IPI_TO_CPU1_OFF : ESP32S31_IPI_TO_CPU0_OFF;
}

static unsigned int esp32s31_ipi_source(unsigned int cpu)
{
	return cpu ? ESP32S31_IPI_TO_CPU1_SOURCE :
		     ESP32S31_IPI_TO_CPU0_SOURCE;
}

static unsigned int esp32s31_timer_source(unsigned int cpu)
{
	return cpu ? ESP32S31_SYSTIMER_CPU1_SOURCE :
		     ESP32S31_SYSTIMER_CPU0_SOURCE;
}

static void esp32s31_ipi_send(unsigned int cpu)
{
	if (WARN_ON_ONCE(!esp32s31_doorbells || cpu >= nr_cpu_ids))
		return;

	/* Publish the ipi_mux reason before asserting the level doorbell. */
	wmb();
	writel(1, esp32s31_doorbells + esp32s31_doorbell_offset(cpu));
}

#include "irq-esp32s31-tlb-fence.inc"

static void esp32s31_ipi_chained(struct irq_desc *desc)
{

	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int cpu = raw_smp_processor_id();
	unsigned int off = esp32s31_doorbell_offset(cpu);

	chained_irq_enter(chip, desc);
	if (readl(esp32s31_doorbells + off)) {
		do {

			writel(0, esp32s31_doorbells + off);

			esp32s31_tlb_flush_poll();

			ipi_mux_process();

			/* Pair the doorbell clear with a sender that races while its
			 * mux bit is already set and therefore does not ring again. */
			smp_mb();
		} while (readl(esp32s31_doorbells + off) || ipi_mux_pending());
	}
	chained_irq_exit(chip, desc);
}

static void esp32s31_timer_chained(struct irq_desc *desc)
{

	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int cpu = raw_smp_processor_id();

	chained_irq_enter(chip, desc);
	if (esp32s31_systimer_irq_pending(cpu))
		esp32s31_riscv_timer_interrupt();
	chained_irq_exit(chip, desc);
}

/*
 * Preserve the official CLIC/INTMTX interrupt domains and handlers, but give
 * SMP an escape hatch for S31's cross-privilege 0xff sentinel.  Both harts use
 * the generic IRQ-enabled idle polling loop; when hardware has left a source
 * pending but refuses CLIC entry, drain that same source through its normal
 * Linux handler without manufacturing an xret boundary.
 */
struct esp32s31_irq_poll_pass {
	struct pt_regs regs;
	unsigned int cpu, off;
	u32 device_pending;
	bool ipi_pending, timer_pending;
};

static void esp32s31_irq_poll_dispatch(struct pt_regs *regs)
{

	struct esp32s31_irq_poll_pass *pass = container_of(regs,
		struct esp32s31_irq_poll_pass, regs);

	irq_enter();

	if (pass->ipi_pending) {
		do {
			writel(0, esp32s31_doorbells + pass->off);
			esp32s31_tlb_flush_poll();
			ipi_mux_process();
			smp_mb();
		} while (readl(esp32s31_doorbells + pass->off) || ipi_mux_pending());
	}
	if (pass->timer_pending)
		esp32s31_riscv_timer_interrupt();
	if (pass->device_pending)
		esp_clic_handle_pending(pass->device_pending);

	irq_exit();

}

void esp32s31_irq_poll(void)
{
	struct esp32s31_irq_poll_pass pass = { };
	struct pt_regs *old_regs;
	unsigned long flags;

	pass.cpu = raw_smp_processor_id();
	if (unlikely(!esp32s31_doorbells || pass.cpu > 1))
		return;
	pass.off = esp32s31_doorbell_offset(pass.cpu);
	local_irq_save(flags);
	pass.ipi_pending = readl(esp32s31_doorbells + pass.off);
	pass.timer_pending = esp32s31_systimer_irq_pending(pass.cpu);
	pass.device_pending = esp_clic_pending_mask(pass.cpu);
	if (!pass.ipi_pending && !pass.timer_pending && !pass.device_pending)
		goto out;
	pass.regs.status = SR_SPP;
	old_regs = set_irq_regs(&pass.regs);
#ifdef CONFIG_IRQ_STACKS
	if (on_thread_stack())
		call_on_irq_stack(&pass.regs, esp32s31_irq_poll_dispatch);
	else
#endif
		esp32s31_irq_poll_dispatch(&pass.regs);
	set_irq_regs(old_regs);
out:
	local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(esp32s31_irq_poll);

static int esp32s31_local_irq_starting(unsigned int cpu)
{
	if (cpu > 1)
		return -EINVAL;

	writel(0, esp32s31_doorbells + esp32s31_doorbell_offset(cpu));
	esp_intmtx_route_local(cpu, esp32s31_ipi_source(cpu),
				ESP32S31_CLIC_IPI_SLOT);
	esp_clic_configure_local(cpu, ESP32S31_CLIC_IPI_SLOT, true);
	esp32s31_tlb_cpu_ready(cpu, true);
	esp32s31_clic_log_state(cpu);
	return 0;
}

static int esp32s31_local_irq_dying(unsigned int cpu)
{
	esp32s31_tlb_cpu_ready(cpu, false);
	esp_clic_configure_local(cpu, ESP32S31_CLIC_IPI_SLOT, false);
	esp_intmtx_unroute_local(cpu, esp32s31_ipi_source(cpu));
	writel(0, esp32s31_doorbells + esp32s31_doorbell_offset(cpu));
	return 0;
}

int esp32s31_systimer_irq_starting(unsigned int cpu)
{
	if (cpu > 1)
		return -EINVAL;

	/*
	 * riscv_timer_starting_cpu() cleared stale state before registering the
	 * clockevent.  Do not stop it here: registration may already have armed
	 * the first event needed to start the kernel tick.
	 */
	esp_intmtx_route_local(cpu, esp32s31_timer_source(cpu),
				ESP32S31_CLIC_TIMER_SLOT);
	esp_clic_configure_local(cpu, ESP32S31_CLIC_TIMER_SLOT, true);
	return 0;
}

int esp32s31_systimer_irq_dying(unsigned int cpu)
{
	esp_clic_configure_local(cpu, ESP32S31_CLIC_TIMER_SLOT, false);
	esp_intmtx_unroute_local(cpu, esp32s31_timer_source(cpu));
	return 0;
}

int __init esp32s31_smp_irq_init(void)
{
	int ret, virq;

	esp32s31_doorbells = ioremap(ESP32S31_IPI_DOORBELL_BASE,
				      ESP32S31_IPI_DOORBELL_SIZE);
	if (!esp32s31_doorbells)
		return -ENOMEM;
	esp32s31_tlb_fence_init();
	ret = esp_clic_install_local(ESP32S31_CLIC_IPI_SLOT,
				     esp32s31_ipi_chained, NULL);
	if (ret)
		return ret;
	ret = esp_clic_install_local(ESP32S31_CLIC_TIMER_SLOT,
				     esp32s31_timer_chained, NULL);
	if (ret)
		return ret;

	virq = ipi_mux_create(BITS_PER_BYTE, esp32s31_ipi_send);
	if (virq <= 0)
		return virq < 0 ? virq : -ENOMEM;
	riscv_ipi_set_virq_range(virq, BITS_PER_BYTE);

	ret = cpuhp_setup_state_nocalls(CPUHP_AP_IRQ_RISCV_SBI_IPI_STARTING,
					 "irqchip/esp32s31:starting",
					 esp32s31_local_irq_starting,
					 esp32s31_local_irq_dying);
	if (ret)
		return ret;

	ret = esp32s31_local_irq_starting(0);
	if (ret)
		return ret;

	pr_info("esp32s31-smp: hart-local IPI slot %u, direct timer slot %u\n",
		ESP32S31_CLIC_IPI_SLOT, ESP32S31_CLIC_TIMER_SLOT);
	return 0;
}
