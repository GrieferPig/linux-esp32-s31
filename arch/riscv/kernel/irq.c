// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2012 Regents of the University of California
 * Copyright (C) 2017 SiFive
 * Copyright (C) 2018 Christoph Hellwig
 */

#include <linux/interrupt.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/scs.h>
#include <linux/seq_file.h>
#include <asm/sbi.h>
#include <asm/smp.h>
#include <asm/softirq_stack.h>
#include <asm/stacktrace.h>

static struct fwnode_handle *(*__get_intc_node)(void);

void riscv_set_intc_hwnode_fn(struct fwnode_handle *(*fn)(void))
{
	__get_intc_node = fn;
}

struct fwnode_handle *riscv_get_intc_hwnode(void)
{
	if (__get_intc_node)
		return __get_intc_node();

	return NULL;
}
EXPORT_SYMBOL_GPL(riscv_get_intc_hwnode);

/**
 * riscv_get_hart_index() - get hart index for interrupt delivery
 * @fwnode: interrupt controller node
 * @logical_index: index within the "interrupts-extended" property
 * @hart_index: filled with the hart index to use
 *
 * RISC-V uses term "hart index" for its interrupt controllers, for the
 * purpose of the interrupt routing to destination harts.
 * It may be arbitrary numbers assigned to each destination hart in context
 * of the particular interrupt domain.
 *
 * These numbers encoded in the optional property "riscv,hart-indexes"
 * that should contain hart index for each interrupt destination in the same
 * order as in the "interrupts-extended" property. If this property
 * not exist, it assumed equal to the logical index, i.e. index within the
 * "interrupts-extended" property.
 *
 * Return: error code
 */
int riscv_get_hart_index(struct fwnode_handle *fwnode, u32 logical_index,
			 u32 *hart_index)
{
	static const char *prop_hart_index = "riscv,hart-indexes";
	struct device_node *np = to_of_node(fwnode);

	if (!np || !of_property_present(np, prop_hart_index)) {
		*hart_index = logical_index;
		return 0;
	}

	return of_property_read_u32_index(np, prop_hart_index,
					  logical_index, hart_index);
}

#ifdef CONFIG_IRQ_STACKS
#include <asm/irq_stack.h>

DECLARE_PER_CPU(ulong *, irq_shadow_call_stack_ptr);

#ifdef CONFIG_SHADOW_CALL_STACK
DEFINE_PER_CPU(ulong *, irq_shadow_call_stack_ptr);
#endif

static void init_irq_scs(void)
{
	int cpu;

	if (!scs_is_enabled())
		return;

	for_each_possible_cpu(cpu)
		per_cpu(irq_shadow_call_stack_ptr, cpu) =
			scs_alloc(cpu_to_node(cpu));
}

DEFINE_PER_CPU(ulong *, irq_stack_ptr);

#ifdef CONFIG_VMAP_STACK
static void init_irq_stacks(void)
{
	int cpu;
	ulong *p;

	for_each_possible_cpu(cpu) {
		p = arch_alloc_vmap_stack(IRQ_STACK_SIZE, cpu_to_node(cpu));
		per_cpu(irq_stack_ptr, cpu) = p;
	}
}
#else
/* irq stack only needs to be 16 byte aligned - not IRQ_STACK_SIZE aligned. */
DEFINE_PER_CPU_ALIGNED(ulong [IRQ_STACK_SIZE/sizeof(ulong)], irq_stack);

static void init_irq_stacks(void)
{
	int cpu;

	for_each_possible_cpu(cpu)
		per_cpu(irq_stack_ptr, cpu) = per_cpu(irq_stack, cpu);
}
#endif /* CONFIG_VMAP_STACK */

#ifdef CONFIG_ESP32S31_RADIO_SMODE
static ulong *s31_radio_irq_stack;
static ulong *s31_previous_irq_stack;
#define S31_IRQ_STACK_MAGIC 0x53314951UL

int esp32s31_radio_irq_stack_install(void *base, unsigned long size)
{
	unsigned long start = (unsigned long)base, flags;
	int ret = 0;

	if (in_interrupt() || in_atomic() || irqs_disabled() ||
	    size != IRQ_STACK_SIZE || (start & 15) ||
	    start < 0x2f000000UL || start > 0x2f080000UL - size)
		return -EINVAL;
	preempt_disable();
	local_irq_save(flags);
	if (smp_processor_id() != 0)
		ret = -EXDEV;
	else if (READ_ONCE(s31_radio_irq_stack))
		ret = -EBUSY;
	else {
		*(ulong *)base = S31_IRQ_STACK_MAGIC;
		s31_previous_irq_stack = this_cpu_read(irq_stack_ptr);
		WRITE_ONCE(s31_radio_irq_stack, base);
		this_cpu_write(irq_stack_ptr, base);
	}
	local_irq_restore(flags);
	preempt_enable();
	return ret;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_irq_stack_install);

void esp32s31_radio_irq_stack_restore(void *base)
{
	unsigned long flags;

	if (in_interrupt() || in_atomic() || irqs_disabled())
		panic("s31 IRQ stack: restore outside schedulable task");
	preempt_disable();
	local_irq_save(flags);
	if (smp_processor_id() != 0 || base != READ_ONCE(s31_radio_irq_stack) ||
	    this_cpu_read(irq_stack_ptr) != base ||
	    *(ulong *)base != S31_IRQ_STACK_MAGIC)
		panic("s31 IRQ stack: lease or stack guard corrupted");
	/* A CPU0 task cannot run until its preceding IRQ frame has returned.
	 * Local IRQ exclusion then makes the pointer replacement indivisible. */
	this_cpu_write(irq_stack_ptr, s31_previous_irq_stack);
	WRITE_ONCE(s31_radio_irq_stack, NULL);
	s31_previous_irq_stack = NULL;
	local_irq_restore(flags);
	preempt_enable();
}
EXPORT_SYMBOL_GPL(esp32s31_radio_irq_stack_restore);

bool esp32s31_radio_on_irq_stack(void)
{
	unsigned long sp, base = (unsigned long)READ_ONCE(s31_radio_irq_stack);

	asm volatile("mv %0, sp" : "=r"(sp));
	return raw_smp_processor_id() == 0 && base &&
	       sp > base && sp < base + IRQ_STACK_SIZE;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_on_irq_stack);
#endif

#ifdef CONFIG_SOFTIRQ_ON_OWN_STACK
static void ___do_softirq(struct pt_regs *regs)
{
	__do_softirq();
}

void do_softirq_own_stack(void)
{
	if (on_thread_stack())
		call_on_irq_stack(NULL, ___do_softirq);
	else
		__do_softirq();
}
#endif /* CONFIG_SOFTIRQ_ON_OWN_STACK */

#else
static void init_irq_scs(void) {}
static void init_irq_stacks(void) {}
#endif /* CONFIG_IRQ_STACKS */

int arch_show_interrupts(struct seq_file *p, int prec)
{
	show_ipi_stats(p, prec);
	return 0;
}

void __init init_IRQ(void)
{
	init_irq_scs();
	init_irq_stacks();
	irqchip_init();
	if (!handle_arch_irq)
		panic("No interrupt controller found.");
	sbi_ipi_init();
}
