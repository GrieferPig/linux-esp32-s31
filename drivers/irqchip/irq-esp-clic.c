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

#include <linux/export.h>
#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/irqdesc.h>
#include <linux/mutex.h>
#include <linux/rcupdate.h>
#include <linux/moduleparam.h>
#include <linux/err.h>
#include <linux/irqchip/irq-esp32s31-clic.h>
#include <linux/spinlock.h>
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
#include <linux/syscore_ops.h>

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
static inline void __iomem *esp_clic_hart_reg(struct esp_clic *priv,
					       unsigned int cpu,
					       unsigned int off);
static u8 clic_saved_ie[2][CLIC_EXT_LAST + 1];
/* Per-source priority/level byte, so suspend/resume restores the actual
 * configuration instead of forcing every source back to one constant. */
static u8 clic_saved_ctl[2][CLIC_EXT_LAST + 1];

/* One mandatory radio policy owner at a time. The raw lock pins the hart-relative
 * MMIO view and serializes priority updates with acquisition/retirement.
 * Only CPU0 peripheral bands are demoted; CPU1 retains the normal source
 * priority. The suspend snapshot is independent: sleep preserves active policy,
 * while release restores the pre-policy values after radio IRQs are joined.
 */
struct esp_clic_priority_lease {
	u8 saved[2][CLIC_EXT_LAST + 1];
	u8 background;
};
static DEFINE_RAW_SPINLOCK(clic_priority_lock);
static struct esp_clic_priority_lease *clic_priority_owner;
static unsigned long clic_priority_verify_failures, clic_priority_restores;
module_param_named(priority_verify_failures,clic_priority_verify_failures,ulong,0400);
module_param_named(priority_restores,clic_priority_restores,ulong,0400);

bool esp_clic_radio_isolation_active(void)
{
	/* Readers test presence only; they never dereference a retiring lease. */
	return READ_ONCE(clic_priority_owner) != NULL;
}


static bool esp_clic_priority_peripheral(unsigned int slot)
{
	return slot != ESP32S31_CLIC_IPI_SLOT &&
	       slot != ESP32S31_CLIC_TIMER_SLOT;
}

struct esp_clic_priority_lease *esp_clic_priority_acquire(unsigned int ctl)
{
	struct esp_clic_priority_lease *lease;
	unsigned long flags;
	unsigned int cpu, slot;
	int ret = 0;

	if (ctl & ~CLIC_CTL_MAX_PRIO)
		return ERR_PTR(-EINVAL);
	lease = kzalloc(sizeof(*lease), GFP_KERNEL);
	if (!lease)
		return ERR_PTR(-ENOMEM);
	lease->background = ctl;
	/* Serialize publication against CPU1 teardown. A single-core radio
	 * start cannot silently move ordinary peripheral work onto CPU0. */
	cpus_read_lock();
	if (!cpu_online(1)) {
		cpus_read_unlock();
		kfree(lease);
		return ERR_PTR(-EHOSTDOWN);
	}
	raw_spin_lock_irqsave(&clic_priority_lock, flags);
	if (!clic)
		ret = -ENODEV;
	else if (clic_priority_owner)
		ret = -EBUSY;
	else {
		for (cpu = 0; cpu < 2; cpu++)
			for (slot = CLIC_EXT_FIRST; slot <= CLIC_EXT_LAST; slot++) {
				if (!esp_clic_priority_peripheral(slot))
					continue;
				lease->saved[cpu][slot] = readb_relaxed(
					esp_clic_hart_reg(clic, cpu, CLIC_INTCTL_OFF(slot)));
				/* Radio priority separation is local to CPU0. CPU1
				 * runs the peripherals and has no radio action to protect.
				 */
				if (cpu != 0)
					continue;
				writeb_relaxed(ctl, esp_clic_hart_reg(clic, cpu,
								CLIC_INTCTL_OFF(slot)));
				if ((readb_relaxed(esp_clic_hart_reg(clic,cpu,CLIC_INTCTL_OFF(slot))) &
				     CLIC_CTL_MAX_PRIO)!=ctl) {
					clic_priority_verify_failures++;
					ret=-EIO;
				}
			}
		if (ret) {
			/* Failed cold admission restores only the saved policy state;
			 * no lease or live radio source has been published yet. */
			for (cpu=0;cpu<2;cpu++)
				for (slot=CLIC_EXT_FIRST;slot<=CLIC_EXT_LAST;slot++)
					if (esp_clic_priority_peripheral(slot))
						writeb_relaxed(lease->saved[cpu][slot],
						 esp_clic_hart_reg(clic,cpu,CLIC_INTCTL_OFF(slot)));
		} else clic_priority_owner = lease;
	}
	raw_spin_unlock_irqrestore(&clic_priority_lock, flags);
	if (!ret) {
		/* Publication closes affinity races before the cold descriptor walk.
		 * IRQ mappings/actions and vendor tasks are not published by the
		 * radio owner until this admission returns successfully. */
		ret = esp_intmtx_radio_isolation_validate();
		if (ret) {
			esp_clic_priority_release(lease);
			cpus_read_unlock();
			return ERR_PTR(ret);
		}
	}
	cpus_read_unlock();
	if (ret) {
		kfree(lease);
		return ERR_PTR(ret);
	}
	return lease;
}
EXPORT_SYMBOL_GPL(esp_clic_priority_acquire);

int esp_clic_priority_release(struct esp_clic_priority_lease *lease)
{
	unsigned long flags;
	unsigned int cpu, slot;

	raw_spin_lock_irqsave(&clic_priority_lock, flags);
	if (!lease || lease != clic_priority_owner) {
		raw_spin_unlock_irqrestore(&clic_priority_lock, flags);
		return -EINVAL;
	}
	for (cpu = 0; cpu < 2; cpu++)
		for (slot = CLIC_EXT_FIRST; slot <= CLIC_EXT_LAST; slot++) {
			if (!esp_clic_priority_peripheral(slot))
				continue;
			writeb_relaxed(lease->saved[cpu][slot],
				esp_clic_hart_reg(clic, cpu, CLIC_INTCTL_OFF(slot)));
			if ((readb_relaxed(esp_clic_hart_reg(clic,cpu,CLIC_INTCTL_OFF(slot))) &
			     CLIC_CTL_MAX_PRIO) != (lease->saved[cpu][slot] & CLIC_CTL_MAX_PRIO))
				clic_priority_verify_failures++;
		}
	clic_priority_restores++;
	clic_priority_owner = NULL;
	raw_spin_unlock_irqrestore(&clic_priority_lock, flags);
	kfree(lease);
	return 0;
}
EXPORT_SYMBOL_GPL(esp_clic_priority_release);

/* Domain allocation may happen after the policy snapshot. Retain the
 * allocation's ordinary default as its release value, and keep the new
 * background source in the active band until a radio owner raises it.
 */
static void esp_clic_priority_default(unsigned int slot)
{
	unsigned long flags;
	u8 ctl = CLIC_CTL_MAX_PRIO;

	raw_spin_lock_irqsave(&clic_priority_lock, flags);
	if (clic_priority_owner && esp_clic_priority_peripheral(slot)) {
		clic_priority_owner->saved[0][slot] = ctl;
		ctl = clic_priority_owner->background;
	}
	writeb_relaxed(ctl, esp_clic_hart_reg(clic, 0, CLIC_INTCTL_OFF(slot)));
	raw_spin_unlock_irqrestore(&clic_priority_lock, flags);
}

static int esp_clic_syscore_suspend(void)
{
	unsigned int cpu, slot;

	if (!clic)
		return 0;
	for (cpu = 0; cpu < 2; cpu++)
		for (slot = CLIC_EXT_FIRST; slot <= CLIC_EXT_LAST; slot++) {
			clic_saved_ie[cpu][slot] =
				readb_relaxed(esp_clic_hart_reg(clic, cpu,
						      CLIC_INTIE_OFF(slot)));
			clic_saved_ctl[cpu][slot] =
				readb_relaxed(esp_clic_hart_reg(clic, cpu,
						      CLIC_INTCTL_OFF(slot)));
		}
	return 0;
}

static void esp_clic_syscore_resume(void)
{
	unsigned int cpu, slot;

	if (!clic)
		return;
	for (cpu = 0; cpu < 2; cpu++) {
		for (slot = CLIC_EXT_FIRST; slot <= CLIC_EXT_LAST; slot++) {
			writeb_relaxed(0, esp_clic_hart_reg(clic, cpu,
						       CLIC_INTIE_OFF(slot)));
			writeb_relaxed(0, esp_clic_hart_reg(clic, cpu,
						       CLIC_INTIP_OFF(slot)));
			writeb_relaxed(CLIC_ATTR_MODE_S | CLIC_ATTR_TRIG_LEVEL,
				esp_clic_hart_reg(clic, cpu,
						  CLIC_INTATTR_OFF(slot)));
			writeb_relaxed(clic_saved_ctl[cpu][slot],
				esp_clic_hart_reg(clic, cpu,
						  CLIC_INTCTL_OFF(slot)));
			writeb_relaxed(clic_saved_ie[cpu][slot] & 1,
				esp_clic_hart_reg(clic, cpu,
						  CLIC_INTIE_OFF(slot)));
		}
	}
}

/* Explicit dedicated-source registrations only. The descriptor's normal
 * Linux flow handler still masks/acks, accounts and runs the action. No
 * module callback is hidden from free_irq()/synchronize_irq(). RCU protects
 * the cached descriptor between lookup and entering that flow handler.
 */
static struct irq_desc __rcu *clic_direct_desc[CLIC_EXT_COUNT];
static void *clic_direct_owner[CLIC_EXT_COUNT];
static DEFINE_MUTEX(clic_direct_lock);
static unsigned long clic_direct_hits;
module_param_named(direct_hits, clic_direct_hits, ulong, 0400);

bool esp_clic_handle_direct(unsigned long slot)
{
	struct irq_desc *desc;
	bool handled = false;

	if (slot < CLIC_EXT_FIRST || slot > CLIC_EXT_LAST ||
	    raw_smp_processor_id() != 0 || !in_hardirq())
		return false;
	rcu_read_lock();
	desc = rcu_dereference(clic_direct_desc[slot - CLIC_EXT_FIRST]);
	if (desc) {
		/* CPU0-only dispatch, with IRQs already disabled by normal entry
		 * or the recovery poll. Local timer/IPI slots are never published. */
		WRITE_ONCE(clic_direct_hits, READ_ONCE(clic_direct_hits) + 1);
		handle_irq_desc(desc);
		handled = true;
	}
	rcu_read_unlock();
	return handled;
}

static struct syscore_ops esp_clic_syscore_ops = {
	.suspend = esp_clic_syscore_suspend,
	.resume = esp_clic_syscore_resume,
};

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

	/* Scan the selected hart; affinity migration masks the old bank. */
	if (unlikely(!clic || cpu > 1))
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
		if (!esp_clic_handle_direct(CLIC_EXT_FIRST + bit))
			generic_handle_domain_irq(clic->domain, CLIC_EXT_FIRST + bit);
}

static unsigned int esp_clic_target_cpu(struct irq_data *d)
{
	return cpumask_first(irq_data_get_effective_affinity_mask(d));
}

static void esp_clic_mask(struct irq_data *d)
{
	writeb_relaxed(0, esp_clic_hart_reg(clic, esp_clic_target_cpu(d),
						CLIC_INTIE_OFF(d->hwirq)));
}

static void esp_clic_unmask(struct irq_data *d)
{
	writeb_relaxed(1, esp_clic_hart_reg(clic, esp_clic_target_cpu(d),
						CLIC_INTIE_OFF(d->hwirq)));
}

/* The parent irq descriptor lock serializes migration against its flow
 * handler. The matrix caller removes the old route between prepare/finish.
 * Level sources reassert on the destination; pending bits are not replayed.
 */
int esp_clic_move_prepare(struct irq_data *d, unsigned int old_cpu, unsigned int new_cpu)
{
	unsigned int slot;
	unsigned long flags;
	bool enabled;
	u8 ctl;

	if (!clic || !d || d->domain != clic->domain || old_cpu > 1 || new_cpu > 1)
		return -EINVAL;
	slot = d->hwirq;
	if (slot < CLIC_EXT_FIRST || slot > CLIC_EXT_LAST ||
	    !esp_clic_priority_peripheral(slot))
		return -EINVAL;
	if (new_cpu && rcu_access_pointer(clic_direct_desc[slot - CLIC_EXT_FIRST]))
		return -EBUSY;
	raw_spin_lock_irqsave(&clic_priority_lock, flags);
	enabled = readb_relaxed(esp_clic_hart_reg(clic, old_cpu, CLIC_INTIE_OFF(slot))) & 1;
	ctl = readb_relaxed(esp_clic_hart_reg(clic, old_cpu, CLIC_INTCTL_OFF(slot)));
	if (clic_priority_owner) {
		if (new_cpu == 1) {
			/* Restore the source's normal band when leaving CPU0,
			 * rather than carrying its radio-only demotion to CPU1.
			 * Release must retain that source priority on its live bank.
			 */
			ctl = clic_priority_owner->saved[0][slot];
			clic_priority_owner->saved[1][slot] = ctl;
		} else {
			/* Frozen PM may temporarily park CPU1 peripherals on CPU0.
			 * Retain their normal band for the return route and release.
			 */
			clic_priority_owner->saved[0][slot] = ctl;
			ctl = clic_priority_owner->background;
		}
	}
	writeb_relaxed(0, esp_clic_hart_reg(clic, old_cpu, CLIC_INTIE_OFF(slot)));
	writeb_relaxed(0, esp_clic_hart_reg(clic, new_cpu, CLIC_INTIE_OFF(slot)));
	/* Clear the masked destination before INTMTX installs its route. A
	 * reused hart bank may retain a cause from ROM or an earlier owner.
	 * Clearing after routing could discard a fresh device assertion. */
	writeb_relaxed(0, esp_clic_hart_reg(clic, new_cpu, CLIC_INTIP_OFF(slot)));
	writeb_relaxed(CLIC_ATTR_MODE_S | CLIC_ATTR_TRIG_LEVEL,
		       esp_clic_hart_reg(clic, new_cpu, CLIC_INTATTR_OFF(slot)));
	writeb_relaxed(ctl, esp_clic_hart_reg(clic, new_cpu, CLIC_INTCTL_OFF(slot)));
	raw_spin_unlock_irqrestore(&clic_priority_lock, flags);
	return enabled;
}

void esp_clic_move_finish(struct irq_data *d, unsigned int old_cpu,
			 unsigned int new_cpu, bool enabled)
{
	unsigned int slot = d->hwirq;

	/* Old route is gone. Clear a stale latched cause before that slot may
	 * be reused; never clear destination pending after enabling its route. */
	writeb_relaxed(0, esp_clic_hart_reg(clic, old_cpu, CLIC_INTIP_OFF(slot)));
	if (enabled)
		writeb_relaxed(1, esp_clic_hart_reg(clic, new_cpu, CLIC_INTIE_OFF(slot)));
}

static int esp_clic_set_type(struct irq_data *d, unsigned int type)
{
	u8 attr;

	if (type != IRQ_TYPE_LEVEL_HIGH)
		return -EINVAL;

	attr = readb_relaxed(esp_clic_hart_reg(clic, esp_clic_target_cpu(d),
						  CLIC_INTATTR_OFF(d->hwirq)));
	/* S31's S-mode window does not reliably return the MODE bits. */
	attr &= ~(CLIC_ATTR_MODE_MASK | CLIC_ATTR_TRIG_MASK | CLIC_ATTR_SHV);
	attr |= CLIC_ATTR_MODE_S | CLIC_ATTR_TRIG_LEVEL;
	writeb_relaxed(attr, esp_clic_hart_reg(clic, esp_clic_target_cpu(d),
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
	readb_relaxed(esp_clic_hart_reg(clic, cpu, CLIC_INTIE_OFF(slot)));
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
		esp_clic_priority_default(src);

		irq_domain_set_info(domain, virq + i, src, &esp_clic_chip,
				    priv, handle_level_irq, NULL, NULL);

		ret = esp_clic_install_chained(priv, src);
		if (ret)
			return ret;
	}

	return 0;
}

/* Resolve the CLIC external slot behind a virq, following the INTMTX parent
 * link. A source that is not an external CLIC slot is refused. */
static int esp_clic_slot_for_virq(unsigned int virq, unsigned int *slot)
{
	struct irq_data *d = irq_get_irq_data(virq);
	irq_hw_number_t hw;

	if (!clic)
		return -EINVAL;
	while (d && d->domain != clic->domain)
		d = d->parent_data;
	if (!d)
		return -EINVAL;
	hw = d->hwirq;
	if (hw < CLIC_EXT_FIRST || hw > CLIC_EXT_LAST ||
	    !esp_clic_priority_peripheral(hw))
		return -EINVAL;
	*slot = hw;
	return 0;
}

/* Caller owns this mapping/action for the complete registration. Shared
 * actions, special flow handlers and local sources retain generic dispatch.
 * Unregister BEFORE free_irq/dispose; synchronize_rcu joins a dispatch that
 * sampled the descriptor but had not yet entered its IRQ flow accounting.
 */
int esp_clic_direct_register(unsigned int virq, void *owner)
{
	struct irq_desc *desc = irq_to_desc(virq);
	unsigned int slot, idx;
	unsigned long flags;
	int ret;

	if (!owner || !desc)
		return -EINVAL;
	ret = esp_clic_slot_for_virq(virq, &slot);
	if (ret)
		return ret;
	idx = slot - CLIC_EXT_FIRST;
	mutex_lock(&clic_direct_lock);
	raw_spin_lock_irqsave(&desc->lock, flags);
	if (rcu_access_pointer(clic_direct_desc[idx]))
		ret = -EBUSY;
	else if (desc->handle_irq != handle_level_irq || !desc->action ||
		 desc->action->next || desc->action->dev_id != owner ||
		 (desc->action->flags & IRQF_SHARED) ||
		 !(desc->action->flags & IRQF_NO_AUTOEN) ||
		 !irqd_irq_disabled(&desc->irq_data))
		ret = -EINVAL;
	else {
		ret = esp_intmtx_radio_bind(&desc->irq_data, owner);
		if (!ret) {
			clic_direct_owner[idx] = owner;
			rcu_assign_pointer(clic_direct_desc[idx], desc);
		}
	}
	raw_spin_unlock_irqrestore(&desc->lock, flags);
	mutex_unlock(&clic_direct_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(esp_clic_direct_register);

int esp_clic_direct_unregister(unsigned int virq, void *owner)
{
	struct irq_desc *desc;
	unsigned int slot, idx;
	int ret = esp_clic_slot_for_virq(virq, &slot);

	if (ret)
		return ret;
	idx = slot - CLIC_EXT_FIRST;
	mutex_lock(&clic_direct_lock);
	desc = rcu_dereference_protected(clic_direct_desc[idx],
					 lockdep_is_held(&clic_direct_lock));
	if (!desc || clic_direct_owner[idx] != owner || irq_desc_get_irq(desc) != virq) {
		mutex_unlock(&clic_direct_lock);
		return -ENOENT;
	}
	RCU_INIT_POINTER(clic_direct_desc[idx], NULL);
	/* Keep the writer lock through the grace period, so a reused slot
	 * cannot be published before every reader of the old mapping exits. */
	synchronize_rcu();
	clic_direct_owner[idx] = NULL;
	mutex_unlock(&clic_direct_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(esp_clic_direct_unregister);

/* Mapping lifetime belongs to the caller. Lock order matches the matrix
 * affinity path: descriptor, then policy; both operations pin the MMIO view. */
int esp_clic_irq_get_state(unsigned int virq,struct esp_clic_irq_state *state)
{
 struct irq_desc *desc=irq_to_desc(virq);
 unsigned int slot,cpu;
 unsigned long flags;
 if (!state || !desc || esp_clic_slot_for_virq(virq,&slot)) return -EINVAL;
 raw_spin_lock_irqsave(&desc->lock,flags);
 cpu=esp_clic_target_cpu(&desc->irq_data);
 if (cpu>1) {
  raw_spin_unlock_irqrestore(&desc->lock,flags);
  return -EINVAL;
 }
 raw_spin_lock(&clic_priority_lock);
 state->slot=slot; state->cpu=cpu;
 state->priority=readb_relaxed(esp_clic_hart_reg(clic,cpu,CLIC_INTCTL_OFF(slot))) & CLIC_CTL_MAX_PRIO;
 state->enabled=readb_relaxed(esp_clic_hart_reg(clic,cpu,CLIC_INTIE_OFF(slot))) & 1;
 state->pending=readb_relaxed(esp_clic_hart_reg(clic,cpu,CLIC_INTIP_OFF(slot))) & 1;
 state->direct=!!rcu_access_pointer(clic_direct_desc[slot-CLIC_EXT_FIRST]);
 raw_spin_unlock(&clic_priority_lock);
 raw_spin_unlock_irqrestore(&desc->lock,flags);
 return 0;
}
EXPORT_SYMBOL_GPL(esp_clic_irq_get_state);

int esp_clic_irq_set_priority(unsigned int virq, unsigned int ctl)
{
 struct irq_desc *desc=irq_to_desc(virq);
 unsigned int slot,cpu;
 unsigned long flags;
 int ret=0;
 if (!desc || (ctl & ~CLIC_CTL_MAX_PRIO) || esp_clic_slot_for_virq(virq,&slot))
  return -EINVAL;
 raw_spin_lock_irqsave(&desc->lock,flags);
 cpu=esp_clic_target_cpu(&desc->irq_data);
 if (cpu>1) { ret=-EINVAL; goto out; }
 raw_spin_lock(&clic_priority_lock);
 writeb_relaxed(ctl,esp_clic_hart_reg(clic,cpu,CLIC_INTCTL_OFF(slot)));
 if ((readb_relaxed(esp_clic_hart_reg(clic,cpu,CLIC_INTCTL_OFF(slot))) & CLIC_CTL_MAX_PRIO)!=ctl) {
  clic_priority_verify_failures++;
  ret=-EIO;
 }
 raw_spin_unlock(&clic_priority_lock);
out:
 raw_spin_unlock_irqrestore(&desc->lock,flags);
 return ret;
}
EXPORT_SYMBOL_GPL(esp_clic_irq_set_priority);


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
	register_syscore_ops(&esp_clic_syscore_ops);

	pr_info("esp-clic: %u external slots (IDs %u-%u), chained on %pOF\n",
		CLIC_EXT_COUNT, CLIC_EXT_FIRST, CLIC_EXT_LAST, parent);
	return 0;
}

IRQCHIP_DECLARE(esp32s31_clic, "espressif,esp32s31-clic", esp_clic_init);
