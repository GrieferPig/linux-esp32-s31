// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-P4 M-mode CLIC and peripheral interrupt matrix.
 *
 * Reconstructed from retained v4 machine-code/DT evidence and Espressif
 * ESP-IDF a602e67b register definitions. This is not recovered v4 source.
 * The CLIC windows are hart-relative; matrix windows are absolute hart IDs.
 * No shared source, CPU hotplug, or interrupt priority nesting is supported.
 */
#define pr_fmt(fmt) "esp32p4-irq: " fmt

#include <linux/bitops.h>
#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/irq-esp-clic-p4.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <linux/soc/espressif/esp32p4.h>
#include <asm/csr.h>
#include <asm/irq.h>
#include <asm/ptrace.h>
#include <asm/smp.h>

#define P4_SOURCES             128
#define P4_SLOTS               48
#define P4_FIRST_SLOT          16
#define P4_IPI_SLOT            47
#define P4_IPI_SOURCE(hart)     (79 + (hart))
#define P4_IPI_OFFSET(hart)     (0x10 + (hart) * 4)
#define P4_MATRIX_STRIDE        0x800
#define P4_CLIC_PEER            0x10000
#define P4_CLIC_CTRL(slot)      (0x1000 + 4 * (slot))
#define P4_CLIC_ATTR_M_LEVEL    0xc0
#define P4_IPI_COUNT           8

struct p4_irq_route {
	unsigned int slot;
	unsigned int hart;
	unsigned int type;
	bool enabled;
};

static void __iomem *p4_clic, *p4_matrix, *p4_ipi;
static struct irq_domain *p4_domain;
static struct p4_irq_route p4_routes[P4_SOURCES];
/* Zero means unmapped; values are source + 1. */
static unsigned int p4_slot_source[2][P4_SLOTS];
static unsigned long p4_slots;
static DEFINE_RAW_SPINLOCK(p4_irq_lock);

/* Caller is pinned, by IRQ descriptor lock, irq-off entry, or p4_irq_lock. */
static void __iomem *p4_slot_reg(unsigned int hart, unsigned int slot)
{
	unsigned long here = csr_read(CSR_MHARTID);

	return p4_clic + (here == hart ? 0 : P4_CLIC_PEER) +
	       P4_CLIC_CTRL(slot);
}

static void p4_route(unsigned int hart, unsigned int source, unsigned int slot)
{
	writel(slot, p4_matrix + hart * P4_MATRIX_STRIDE + source * 4);
}

static void p4_configure_slot(struct p4_irq_route *r)
{
	void __iomem *reg = p4_slot_reg(r->hart, r->slot);
	u8 attr = P4_CLIC_ATTR_M_LEVEL;

	if (r->type == IRQ_TYPE_EDGE_RISING)
		attr |= BIT(1);
	writeb(0, reg + 1);
	writeb(0, reg);
	writeb(attr, reg + 2);
	writeb(0xff, reg + 3);
	writeb(r->enabled, reg + 1);
}

static void p4_irq_mask(struct irq_data *d)
{
	struct p4_irq_route *r = irq_data_get_irq_chip_data(d);

	r->enabled = false;
	writeb(0, p4_slot_reg(r->hart, r->slot) + 1);
}

static void p4_irq_unmask(struct irq_data *d)
{
	struct p4_irq_route *r = irq_data_get_irq_chip_data(d);

	r->enabled = true;
	writeb(1, p4_slot_reg(r->hart, r->slot) + 1);
}

static void p4_irq_ack(struct irq_data *d)
{
	struct p4_irq_route *r = irq_data_get_irq_chip_data(d);

	/* Level pending follows its peripheral. Only edge IP is SW-cleared. */
	if (r->type == IRQ_TYPE_EDGE_RISING)
		writeb(0, p4_slot_reg(r->hart, r->slot));
}

static int p4_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct p4_irq_route *r = irq_data_get_irq_chip_data(d);

	type &= IRQ_TYPE_SENSE_MASK;
	if (type != IRQ_TYPE_LEVEL_HIGH && type != IRQ_TYPE_EDGE_RISING)
		return -EINVAL;
	r->type = type;
	p4_configure_slot(r);
	irq_set_handler_locked(d, type == IRQ_TYPE_LEVEL_HIGH ?
			       handle_level_irq : handle_edge_irq);
	return 0;
}

#ifdef CONFIG_SMP
static int p4_irq_set_affinity(struct irq_data *d, const struct cpumask *mask,
			       bool force)
{
	struct p4_irq_route *r = irq_data_get_irq_chip_data(d);
	unsigned int cpu, hart;
	unsigned long flags;

	cpu = force ? cpumask_first_and(mask, cpu_possible_mask) :
		      cpumask_any_and(mask, cpu_online_mask);
	if (cpu >= nr_cpu_ids)
		return -EINVAL;
	hart = cpuid_to_hartid_map(cpu);
	if (hart > 1)
		return -EINVAL;

	raw_spin_lock_irqsave(&p4_irq_lock, flags);
	writeb(0, p4_slot_reg(r->hart, r->slot) + 1);
	p4_route(r->hart, d->hwirq, 0);
	WRITE_ONCE(p4_slot_source[r->hart][r->slot], 0);
	r->hart = hart;
	WRITE_ONCE(p4_slot_source[hart][r->slot], d->hwirq + 1);
	p4_configure_slot(r);
	p4_route(hart, d->hwirq, r->slot);
	raw_spin_unlock_irqrestore(&p4_irq_lock, flags);
	irq_data_update_effective_affinity(d, cpumask_of(cpu));
	return IRQ_SET_MASK_OK_DONE;
}
#endif

static struct irq_chip p4_irq_chip = {
	.name = "P4-CLIC",
	.irq_mask = p4_irq_mask,
	.irq_unmask = p4_irq_unmask,
	.irq_ack = p4_irq_ack,
	.irq_set_type = p4_irq_set_type,
#ifdef CONFIG_SMP
	.irq_set_affinity = p4_irq_set_affinity,
#endif
};

static int p4_domain_map(struct irq_domain *d, unsigned int virq,
			 irq_hw_number_t source)
{
	struct p4_irq_route *r;
	unsigned long flags;
	unsigned int bit;

	if (source >= P4_SOURCES ||
	    source == P4_IPI_SOURCE(0) || source == P4_IPI_SOURCE(1))
		return -EINVAL;

	raw_spin_lock_irqsave(&p4_irq_lock, flags);
	bit = find_first_zero_bit(&p4_slots, P4_IPI_SLOT - P4_FIRST_SLOT);
	if (bit >= P4_IPI_SLOT - P4_FIRST_SLOT) {
		raw_spin_unlock_irqrestore(&p4_irq_lock, flags);
		return -ENOSPC;
	}
	__set_bit(bit, &p4_slots);
	r = &p4_routes[source];
	r->slot = P4_FIRST_SLOT + bit;
	r->hart = cpuid_to_hartid_map(0);
	r->type = IRQ_TYPE_LEVEL_HIGH;
	r->enabled = false;
	WRITE_ONCE(p4_slot_source[r->hart][r->slot], source + 1);
	p4_configure_slot(r);
	p4_route(r->hart, source, r->slot);
	raw_spin_unlock_irqrestore(&p4_irq_lock, flags);
	irq_domain_set_info(d, virq, source, &p4_irq_chip, r,
			    handle_level_irq, NULL, NULL);
	irq_data_update_effective_affinity(irq_get_irq_data(virq), cpumask_of(0));
	return 0;
}

static void p4_domain_unmap(struct irq_domain *d, unsigned int virq)
{
	struct irq_data *data = irq_get_irq_data(virq);
	struct p4_irq_route *r = irq_data_get_irq_chip_data(data);
	unsigned long flags;

	raw_spin_lock_irqsave(&p4_irq_lock, flags);
	writeb(0, p4_slot_reg(r->hart, r->slot) + 1);
	p4_route(r->hart, data->hwirq, 0);
	WRITE_ONCE(p4_slot_source[r->hart][r->slot], 0);
	__clear_bit(r->slot - P4_FIRST_SLOT, &p4_slots);
	r->slot = 0;
	r->enabled = false;
	raw_spin_unlock_irqrestore(&p4_irq_lock, flags);
	irq_set_chip_and_handler(virq, NULL, NULL);
	irq_set_chip_data(virq, NULL);
}

static const struct irq_domain_ops p4_domain_ops = {
	.map = p4_domain_map,
	.unmap = p4_domain_unmap,
	.xlate = irq_domain_xlate_twocell,
};

#ifdef CONFIG_SMP
static void p4_ipi_send(unsigned int cpu)
{
	unsigned int hart = cpuid_to_hartid_map(cpu);

	if (WARN_ON_ONCE(hart > 1))
		return;
	/* Publish the mux reason and payload before the hardware doorbell. */
	wmb();
	writel(1, p4_ipi + P4_IPI_OFFSET(hart));
}
#endif

static void p4_handle_irq(struct pt_regs *regs)
{
	unsigned int slot = regs->cause & 0xfff;
	unsigned int hart = csr_read(CSR_MHARTID);
	unsigned int source;

	if (unlikely(hart > 1 || slot >= P4_SLOTS))
		goto bad_irq;
#ifdef CONFIG_SMP
	if (slot == P4_IPI_SLOT) {
		/*
		 * Clear before processing. Recheck both the level and mux bits:
		 * a sender can set a reason while its old reason is still set.
		 */
		do {
			writel(0, p4_ipi + P4_IPI_OFFSET(hart));
			ipi_mux_process();
			/* Pair the clear/process transaction with a racing sender. */
			smp_mb();
		} while (readl(p4_ipi + P4_IPI_OFFSET(hart)) ||
			 ipi_mux_pending());
		return;
	}
#endif
	source = READ_ONCE(p4_slot_source[hart][slot]);
	if (source && !generic_handle_domain_irq(p4_domain, source - 1))
		return;
bad_irq:
	pr_warn_ratelimited("unexpected hart %u CLIC slot %u\n", hart, slot);
}

int esp32p4_irq_cpu_starting(unsigned int cpu)
{
	unsigned int hart = cpuid_to_hartid_map(cpu);
	unsigned int slot, source;
	void __iomem *reg;

	if (!p4_clic || hart > 1 || hart != csr_read(CSR_MHARTID))
		return -EINVAL;

	/* Only the executing hart's view. Never reset the peer's live slots. */
	writel(esp32p4_clic_rev1 ? 6 : 3, p4_clic);
	if (esp32p4_clic_rev1)
		writel(0x1f000000, p4_clic + 8);
	else
		csr_write(0x347, 0x1f);
	for (slot = 0; slot < P4_SLOTS; slot++) {
		reg = p4_slot_reg(hart, slot);
		writeb(0, reg + 1);
		writeb(0, reg);
		writeb(P4_CLIC_ATTR_M_LEVEL, reg + 2);
		writeb(0xff, reg + 3);
	}
	for (source = 0; source < P4_SOURCES; source++)
		if (p4_routes[source].slot && p4_routes[source].hart == hart)
			p4_configure_slot(&p4_routes[source]);
#ifdef CONFIG_SMP
	writel(0, p4_ipi + P4_IPI_OFFSET(hart));
	p4_route(hart, P4_IPI_SOURCE(hart), P4_IPI_SLOT);
	writeb(1, p4_slot_reg(hart, P4_IPI_SLOT) + 1);
#endif
	/* Source controls, not mie bit positions, select CLIC interrupts. */
	return 0;
}

static struct fwnode_handle *p4_intc_hwnode(void)
{
	return p4_domain->fwnode;
}

static int __init p4_irq_init(struct device_node *node,
			     struct device_node *parent)
{
	struct device_node *chosen;
	unsigned int hart, source;
	u32 revision;
	int ret;

	chosen = of_find_node_by_path("/chosen");
	ret = of_property_read_u32(chosen, "espressif,chip-revision", &revision);
	of_node_put(chosen);
	if (ret || revision != 103) {
		pr_err("requires an explicit, verified revision 1.3 firmware handoff\n");
		return -ENODEV;
	}
	esp32p4_clic_rev1 = 1;
	p4_clic = of_iomap(node, 0);
	p4_matrix = of_iomap(node, 1);
	p4_ipi = of_iomap(node, 2);
	if (!p4_clic || !p4_matrix || !p4_ipi)
		return -ENOMEM;
	/* Discard all firmware peripheral routes, including SYSTIMER/IPC. */
	for (hart = 0; hart < 2; hart++)
		for (source = 0; source < P4_SOURCES; source++)
			p4_route(hart, source, 0);
	p4_domain = irq_domain_add_linear(node, P4_SOURCES,
					 &p4_domain_ops, NULL);
	if (!p4_domain)
		return -ENOMEM;
	ret = set_handle_irq(p4_handle_irq);
	if (ret)
		return ret;
	irq_set_default_domain(p4_domain);
	riscv_set_intc_hwnode_fn(p4_intc_hwnode);
	ret = esp32p4_irq_cpu_starting(smp_processor_id());
	if (ret)
		return ret;
#ifdef CONFIG_SMP
	ret = ipi_mux_create(P4_IPI_COUNT, p4_ipi_send);
	if (ret <= 0)
		return ret ?: -EINVAL;
	riscv_ipi_set_virq_range(ret, P4_IPI_COUNT);
#endif
	pr_info("revision 1.3, hart-relative CLIC, two hardware IPI doorbells\n");
	return 0;
}
IRQCHIP_DECLARE(esp32p4, "espressif,esp32p4-interrupt-controller", p4_irq_init);
