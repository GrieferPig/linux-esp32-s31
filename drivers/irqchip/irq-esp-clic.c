// SPDX-License-Identifier: GPL-2.0
/*
 * Espressif ESP32-S31 CLIC irqchip — S-mode view of the chip's CLIC.
 *
 * The S31 has a Core-Local Interrupt Controller plus an INTMTX
 * peripheral routing matrix. OpenSBI delegates the external CLIC
 * sources (IDs 16-47) to S-mode at boot (clicintattr.mode = 01),
 * then this driver drives enable/mask/priority/trigger via the CLIC
 * Supervisor bank mapped at 0x10A00000.
 *
 * Why not hierarchical with riscv-intc?
 *   riscv-intc's domain_map tags every IRQ as percpu_devid and uses
 *   handle_percpu_devid_irq, which is right for the timer/IPI and
 *   wrong for a peripheral (request_irq() WARNs and returns -EINVAL
 *   on percpu_devid IRQs). PLIC sidesteps this by hanging itself off
 *   intc as a chained handler on the S-external IRQ; CLIC has no
 *   single "claim" pin, each source is its own scause exccode, so we
 *   install one chained handler per CLIC slot we actually use.
 *
 * INTMTX (peripheral source -> CLIC slot routing) is a hierarchical
 * child of this domain. Peripherals reference INTMTX in DT;
 * INTMTX.alloc forwards to clic.alloc with the chosen slot.
 *
 * ESP32-S31 extends the parent riscv-intc domain to cover external slots
 * 16-47 even on RV32; their enables live in this MMIO layer, not sie/sieh.
 */

#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/smp.h>

#include "irq-esp32s31-internal.h"

/* Supervisor bank per-source byte layout (mirror of M-bank in OpenSBI):
 *   +0 clicintip   pending bit
 *   +1 clicintie   interrupt enable
 *   +2 clicintattr [7:6] mode, [2:1] trigger, [0] SHV
 *   +3 clicintctl  priority/level
 */
#define CLIC_CTRL_OFF		0x1000U

#define CLIC_INTIP_OFF(s)	(CLIC_CTRL_OFF + (s) * 4U + 0U)
#define CLIC_INTIE_OFF(s)	(CLIC_CTRL_OFF + (s) * 4U + 1U)
#define CLIC_INTATTR_OFF(s)	(CLIC_CTRL_OFF + (s) * 4U + 2U)
#define CLIC_INTCTL_OFF(s)	(CLIC_CTRL_OFF + (s) * 4U + 3U)

#define CLIC_ATTR_TRIG_MASK	0x6U
#define CLIC_ATTR_TRIG_LEVEL	0x0U
#define CLIC_ATTR_MODE_MASK	0xC0U
#define CLIC_ATTR_MODE_S	0x40U
#define CLIC_ATTR_SHV		0x1U

#define CLIC_CTL_MAX_PRIO	0xE0U

#define CLIC_EXT_FIRST		ESP32S31_CLIC_EXT_FIRST
#define CLIC_EXT_LAST		ESP32S31_CLIC_EXT_LAST
#define CLIC_EXT_COUNT		(CLIC_EXT_LAST - CLIC_EXT_FIRST + 1U)

/* The S-mode view is hart-relative; +64 KiB selects the other hart. */
#define CLIC_OTHER_HART_OFF	0x10000U

struct esp_clic {
	void __iomem		*base;
	struct irq_domain	*domain;
	struct irq_domain	*parent_domain;
	unsigned int		intc_virqs[CLIC_EXT_COUNT];
};

static struct esp_clic *clic;

static inline unsigned int slot_to_idx(unsigned int slot)
{
	return slot - CLIC_EXT_FIRST;
}

static inline void __iomem *esp_clic_hart_reg(struct esp_clic *priv,
					       unsigned int cpu,
					       unsigned int off)
{
	void __iomem *base = priv->base;

	if (cpu != raw_smp_processor_id())
		base += CLIC_OTHER_HART_OFF;
	return base + off;
}

u32 esp_clic_pending_mask(unsigned int cpu)
{
	u32 pending = 0;
	unsigned int slot;

	/* All normal device IRQs are deliberately pinned to hart 0. */
	if (unlikely(!clic || cpu != 0))
		return 0;

	for (slot = CLIC_EXT_FIRST; slot <= CLIC_EXT_LAST; slot++) {
		if (slot == ESP32S31_CLIC_IPI_SLOT ||
		    slot == ESP32S31_CLIC_TIMER_SLOT)
			continue;
		/* Avoid two MMIO reads for every unallocated CLIC slot. */
		if (!clic->intc_virqs[slot_to_idx(slot)])
			continue;
		if ((readb_relaxed(esp_clic_hart_reg(clic, cpu,
						 CLIC_INTIP_OFF(slot))) & 1) &&
		    (readb_relaxed(esp_clic_hart_reg(clic, cpu,
						 CLIC_INTIE_OFF(slot))) & 1))
			pending |= BIT(slot - CLIC_EXT_FIRST);
	}
	return pending;
}

void esp_clic_handle_pending(u32 pending)
{
	unsigned int bit;

	if (unlikely(!clic))
		return;
	for_each_set_bit(bit, (unsigned long *)&pending, CLIC_EXT_COUNT)
		generic_handle_domain_irq(clic->domain, CLIC_EXT_FIRST + bit);
}

static void esp_clic_mask(struct irq_data *d)
{
	writeb_relaxed(0, esp_clic_hart_reg(clic, 0,
						CLIC_INTIE_OFF(d->hwirq)));
}

static void esp_clic_unmask(struct irq_data *d)
{
	writeb_relaxed(1, esp_clic_hart_reg(clic, 0,
						CLIC_INTIE_OFF(d->hwirq)));
}

static int esp_clic_set_type(struct irq_data *d, unsigned int type)
{
	u8 attr;

	if (type != IRQ_TYPE_LEVEL_HIGH)
		return -EINVAL;

	attr = readb_relaxed(esp_clic_hart_reg(clic, 0,
						  CLIC_INTATTR_OFF(d->hwirq)));
	/* S31's S-mode window does not reliably return the MODE bits. */
	attr &= ~(CLIC_ATTR_MODE_MASK | CLIC_ATTR_TRIG_MASK | CLIC_ATTR_SHV);
	attr |= CLIC_ATTR_MODE_S | CLIC_ATTR_TRIG_LEVEL;
	writeb_relaxed(attr, esp_clic_hart_reg(clic, 0,
						   CLIC_INTATTR_OFF(d->hwirq)));
	irq_set_handler_locked(d, handle_level_irq);
	return 0;
}

static struct irq_chip esp_clic_chip = {
	.name		= "esp-clic",
	.irq_mask	= esp_clic_mask,
	.irq_unmask	= esp_clic_unmask,
	.irq_set_type	= esp_clic_set_type,
	.flags		= IRQCHIP_SKIP_SET_WAKE,
};

/*
 * Chained handler — runs in interrupt context when scause lands on a
 * CLIC slot we own. The intc-layer descriptor's hwirq is the slot ID,
 * which is also our CLIC domain's hwirq, so a straight forward.
 */
static void esp_clic_chained_handler(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int slot = irq_desc_get_irq_data(desc)->hwirq;

	chained_irq_enter(chip, desc);
	generic_handle_domain_irq(clic->domain, slot);
	chained_irq_exit(chip, desc);
}

static int esp_clic_install_chained(struct esp_clic *priv, unsigned int slot)
{
	unsigned int idx = slot_to_idx(slot);
	unsigned int intc_virq;

	if (priv->intc_virqs[idx])
		return 0;	/* already installed (shared slot) */

	intc_virq = irq_create_mapping(priv->parent_domain, slot);
	if (!intc_virq) {
		pr_err("esp-clic: failed to map intc IRQ for slot %u\n", slot);
		return -ENXIO;
	}

	irq_set_chained_handler_and_data(intc_virq, esp_clic_chained_handler,
					 priv);
	priv->intc_virqs[idx] = intc_virq;
	return 0;
}

int esp_clic_install_local(unsigned int slot,
			   void (*handler)(struct irq_desc *), void *data)
{
	unsigned int idx, intc_virq;

	if (!clic || slot < CLIC_EXT_FIRST || slot > CLIC_EXT_LAST)
		return -EINVAL;

	idx = slot_to_idx(slot);
	if (clic->intc_virqs[idx])
		return -EBUSY;

	intc_virq = irq_create_mapping(clic->parent_domain, slot);
	if (!intc_virq)
		return -ENXIO;

	irq_set_chained_handler_and_data(intc_virq, handler, data);
	clic->intc_virqs[idx] = intc_virq;
	return 0;
}

void esp_clic_configure_local(unsigned int cpu, unsigned int slot, bool enable)
{
	if (WARN_ON_ONCE(!clic || cpu >= nr_cpu_ids ||
			 slot < CLIC_EXT_FIRST || slot > CLIC_EXT_LAST))
		return;

	writeb_relaxed(0, esp_clic_hart_reg(clic, cpu, CLIC_INTIE_OFF(slot)));
	writeb_relaxed(0, esp_clic_hart_reg(clic, cpu, CLIC_INTIP_OFF(slot)));
	/* MODE is not reliably readable through S31's S-mode CLIC window. */
	writeb_relaxed(CLIC_ATTR_MODE_S | CLIC_ATTR_TRIG_LEVEL,
		       esp_clic_hart_reg(clic, cpu, CLIC_INTATTR_OFF(slot)));
	writeb_relaxed(CLIC_CTL_MAX_PRIO,
		       esp_clic_hart_reg(clic, cpu, CLIC_INTCTL_OFF(slot)));
	if (enable)
		writeb_relaxed(1, esp_clic_hart_reg(clic, cpu,
							CLIC_INTIE_OFF(slot)));
}

static int esp_clic_domain_alloc(struct irq_domain *domain, unsigned int virq,
				 unsigned int nr_irqs, void *arg)
{
	struct irq_fwspec *fwspec = arg;
	struct esp_clic *priv = domain->host_data;
	irq_hw_number_t hwirq;
	unsigned int type;
	int i, ret;

	ret = irq_domain_translate_twocell(domain, fwspec, &hwirq, &type);
	if (ret)
		return ret;

	if (hwirq < CLIC_EXT_FIRST || hwirq + nr_irqs - 1 > CLIC_EXT_LAST)
		return -EINVAL;

	for (i = 0; i < nr_irqs; i++) {
		irq_hw_number_t src = hwirq + i;

		/* clear any pending bit latched by ROM/OpenSBI before this slot
		 * gets a handler, so the first unmask doesn't take a spurious
		 * interrupt.
		 */
		writeb_relaxed(0, esp_clic_hart_reg(priv, 0,
						  CLIC_INTIP_OFF(src)));
		writeb_relaxed(CLIC_CTL_MAX_PRIO,
				       esp_clic_hart_reg(priv, 0,
						 CLIC_INTCTL_OFF(src)));

		irq_domain_set_info(domain, virq + i, src, &esp_clic_chip,
				    priv, handle_level_irq, NULL, NULL);

		ret = esp_clic_install_chained(priv, src);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct irq_domain_ops esp_clic_domain_ops = {
	.translate	= irq_domain_translate_twocell,
	.alloc		= esp_clic_domain_alloc,
	.free		= irq_domain_free_irqs_common,
};

static int __init esp_clic_init(struct device_node *node,
				struct device_node *parent)
{
	struct irq_domain *parent_domain, *domain;
	struct esp_clic *priv;

	if (clic) {
		pr_err("esp-clic: already initialised\n");
		return -EEXIST;
	}

	parent_domain = irq_find_host(parent);
	if (!parent_domain) {
		pr_err("esp-clic: %pOF: parent intc domain missing\n", node);
		return -ENXIO;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = of_iomap(node, 0);
	if (!priv->base) {
		pr_err("esp-clic: %pOF: failed to map regs\n", node);
		kfree(priv);
		return -ENXIO;
	}
	priv->parent_domain = parent_domain;

	domain = irq_domain_create_linear(of_fwnode_handle(node),
					  CLIC_EXT_LAST + 1,
					  &esp_clic_domain_ops, priv);
	if (!domain) {
		iounmap(priv->base);
		kfree(priv);
		return -ENOMEM;
	}

	priv->domain = domain;
	clic = priv;

	pr_info("esp-clic: %u external slots (IDs %u-%u), chained on %pOF\n",
		CLIC_EXT_COUNT, CLIC_EXT_FIRST, CLIC_EXT_LAST, parent);
	return 0;
}

IRQCHIP_DECLARE(esp32s31_clic, "espressif,esp32s31-clic", esp_clic_init);
