// SPDX-License-Identifier: GPL-2.0
/*
 * Espressif ESP32-S31 INTMTX — peripheral-source -> CLIC-slot routing.
 *
 * The chip's peripherals each drive a fixed "ETS source" line into the
 * INTMTX. The matrix is a flat 32-bit register per source: writing the
 * 6-bit CLIC slot number maps that source onto that slot; writing 0
 * disables routing. The CLIC then delivers interrupts on the chosen
 * slot to S-mode as a regular scause exccode, where irq-esp-clic
 * picks them up.
 *
 * This driver hangs underneath irq-esp-clic in the irq-domain
 * hierarchy. DT peripherals reference INTMTX with their ETS source
 * number + trigger type; alloc picks a free CLIC slot from a bitmap
 * (slots 16-31 — limited by riscv-intc's 32-hwirq cap on RV32),
 * programs the matrix, and forwards the request to the CLIC layer
 * with the slot number as parent hwirq. mask/unmask/set_type pass through
 * to the parent (CLIC) chip; INTMTX itself has no per-source gate.
 *
 * Device routes start on core0 and can migrate to an online hart through
 * the IRQ affinity callback. Timer/IPI slots remain owned by the SMP layer.
 */

#include <linux/bitmap.h>
#include <linux/cpu.h>
#include <linux/cpuhotplug.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/irqdesc.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/spinlock.h>
#include <linux/syscore_ops.h>

#include "irq-esp32s31-internal.h"

/* Per-source 32-bit register, low 6 bits = target CLIC slot. */
#define INTMTX_SRC_REG(src)	((src) * 4U)
#define INTMTX_SLOT_MASK	0x3FU
#define INTMTX_PASS_LEVEL_SHIFT	8U
#define INTMTX_PASS_LEVEL_MASK	(0x3U << INTMTX_PASS_LEVEL_SHIFT)
#define INTMTX_PASS_LEVEL_S	(0x1U << INTMTX_PASS_LEVEL_SHIFT)

/* External CLIC slots available to Linux; 40/41 are reserved below. */
#define CLIC_SLOT_FIRST		ESP32S31_CLIC_EXT_FIRST
#define CLIC_SLOT_LAST		ESP32S31_CLIC_EXT_LAST
#define CLIC_SLOT_COUNT		(CLIC_SLOT_LAST - CLIC_SLOT_FIRST + 1U)

#define INTMTX_CORE_STRIDE	0x800U

/* ESP-IDF's esp32s31 interrupt table contains sources 0..177.  Each hart's
 * matrix bank has an 0x800-byte window, so all 178 32-bit route registers fit
 * before the next hart bank.  In particular BLE also uses source 133
 * (MODEM_BT_MAC_INT1), which must not be clipped to the older 128-source
 * assumption. */
#define INTMTX_MAX_SRC		177U

struct esp_intmtx_radio_claim {
	void *owner;
	unsigned long status;
	cpumask_t affinity;
};

struct esp_intmtx {
	void __iomem		*base;
	raw_spinlock_t		lock;
	DECLARE_BITMAP(slots_in_use, CLIC_SLOT_COUNT);
	u8			src_to_slot[INTMTX_MAX_SRC + 1];
	/* Explicit direct-radio admission, protected by each IRQ descriptor.
	 * The action owns this cookie; last-action resource release clears it. */
	struct esp_intmtx_radio_claim claims[CLIC_SLOT_COUNT];
};

static struct esp_intmtx *intmtx;
static u32 intmtx_saved[2][INTMTX_MAX_SRC + 1];

static int esp_intmtx_syscore_suspend(void)
{
	unsigned int cpu, src;

	if (!intmtx)
		return 0;
	for (cpu = 0; cpu < 2; cpu++)
		for (src = 0; src <= INTMTX_MAX_SRC; src++)
			intmtx_saved[cpu][src] = readl_relaxed(intmtx->base +
				cpu * INTMTX_CORE_STRIDE + INTMTX_SRC_REG(src));
	return 0;
}

static void esp_intmtx_syscore_resume(void)
{
	unsigned int cpu, src;

	if (!intmtx)
		return;
	for (cpu = 0; cpu < 2; cpu++)
		for (src = 0; src <= INTMTX_MAX_SRC; src++)
			writel_relaxed(intmtx_saved[cpu][src], intmtx->base +
				cpu * INTMTX_CORE_STRIDE + INTMTX_SRC_REG(src));
}

static struct syscore_ops esp_intmtx_syscore_ops = {
	.suspend = esp_intmtx_syscore_suspend,
	.resume = esp_intmtx_syscore_resume,
};

static int esp_intmtx_alloc_slot(struct esp_intmtx *priv)
{
	int bit;

	bit = find_first_zero_bit(priv->slots_in_use, CLIC_SLOT_COUNT);
	if (bit >= CLIC_SLOT_COUNT)
		return -ENOSPC;

	set_bit(bit, priv->slots_in_use);
	return CLIC_SLOT_FIRST + bit;
}

static void esp_intmtx_free_slot(struct esp_intmtx *priv, unsigned int slot)
{
	if (slot < CLIC_SLOT_FIRST || slot > CLIC_SLOT_LAST)
		return;
	clear_bit(slot - CLIC_SLOT_FIRST, priv->slots_in_use);
}

static struct esp_intmtx_radio_claim *esp_intmtx_claim(struct irq_data *data)
{
	struct esp_intmtx *priv = irq_data_get_irq_chip_data(data);
	struct irq_data *parent = data->parent_data;

	if (!priv || !parent || parent->hwirq < CLIC_SLOT_FIRST ||
	    parent->hwirq > CLIC_SLOT_LAST)
		return NULL;
	return &priv->claims[parent->hwirq - CLIC_SLOT_FIRST];
}

static bool esp_intmtx_cpu0_owner(struct irq_data *data)
{
	struct esp_intmtx_radio_claim *claim = esp_intmtx_claim(data);

	/* IRQ_NO_BALANCING is a generic device property, not radio ownership.
	 * LP IPC alone has a fixed source exception before its action exists. */
	return data->hwirq == 139 || (claim && claim->owner);
}

static bool esp_intmtx_pm_migration(void)
{
	/* CPU hotplug retains the last tasks_frozen value after resume.
	 * Only the actual suspend window permits quiesced CPU0 migration. */
	return READ_ONCE(cpuhp_tasks_frozen) && pm_suspend_in_progress();
}

static int esp_intmtx_set_affinity(struct irq_data *d,
				    const struct cpumask *mask, bool force)
{
	struct esp_intmtx *priv = irq_data_get_irq_chip_data(d);
	unsigned int cpu, old_cpu, slot, src = d->hwirq;
	unsigned long flags;
	int enabled;

	/* Linux adds the boot CPU to irqaffinity=, even when it names only
	 * CPU1. Prefer CPU1 within a multi-CPU mask so ordinary peripherals
	 * actually leave CPU0. Explicit radio/IPC CPU0 requests still win.
	 * Hotplug migration must only choose a hart that remains online. */
	cpu = cpumask_test_cpu(1, mask) && cpu_online(1) ? 1 :
		cpumask_any_and(mask, cpu_online_mask);
	if (cpu > 1 || src > INTMTX_MAX_SRC)
		return -EINVAL;
	/* Explicit source admission marks CPU0 radio owners. Ordinary
	 * peripherals may not consume the radio admission budget on CPU0.
	 * Suspend's frozen-CPU migration follows device/IRQ quiescence and is
	 * restored by the CPU1 online callback before normal device resume. */
	if (!cpu && !esp_intmtx_cpu0_owner(d) &&
	    esp_clic_radio_isolation_active() && !esp_intmtx_pm_migration())
		/* EBUSY is converted into a deferred affinity request by genirq. */
		return -EINVAL;
	old_cpu = cpumask_first(irq_data_get_effective_affinity_mask(d));
	if (old_cpu == cpu)
		return IRQ_SET_MASK_OK_DONE;
	raw_spin_lock_irqsave(&priv->lock, flags);
	slot = priv->src_to_slot[src];
	if (!slot) {
		raw_spin_unlock_irqrestore(&priv->lock, flags);
		return -EINVAL;
	}
	enabled = esp_clic_move_prepare(d->parent_data, old_cpu, cpu);
	if (enabled < 0) {
		raw_spin_unlock_irqrestore(&priv->lock, flags);
		return enabled;
	}
	writel_relaxed(0, priv->base + old_cpu * INTMTX_CORE_STRIDE + INTMTX_SRC_REG(src));
	/* CPU0 preserves its qualified delegation path. CPU1 device delivery
	 * uses the S-mode route, just like its existing local timer and IPI. */
	writel_relaxed(slot | (cpu ? INTMTX_PASS_LEVEL_S : 0),
		       priv->base + cpu * INTMTX_CORE_STRIDE + INTMTX_SRC_REG(src));
	irq_data_update_effective_affinity(d, cpumask_of(cpu));
	esp_clic_move_finish(d->parent_data, old_cpu, cpu, enabled);
	raw_spin_unlock_irqrestore(&priv->lock, flags);
	return IRQ_SET_MASK_OK_DONE;
}

static int esp_intmtx_request_resources(struct irq_data *data)
{
	struct esp_intmtx_radio_claim *claim = esp_intmtx_claim(data);
	struct irq_desc *desc = irq_data_to_desc(data);
	unsigned long flags;
	int ret = 0;

	if (!claim)
		return -ENODEV;
	/* irq_startup ignores irq_setup_affinity errors. Refuse a conflicting
	 * new action here, before it can be unmasked on its allocation CPU. */
	raw_spin_lock_irqsave(&desc->lock, flags);
	claim->status = desc->status_use_accessors &
		(IRQ_NO_BALANCING | IRQ_DISABLE_UNLAZY);
	cpumask_copy(&claim->affinity, irq_data_get_affinity_mask(data));
	if (esp_clic_radio_isolation_active() && !esp_intmtx_pm_migration() &&
	    !esp_intmtx_cpu0_owner(data)) {
		if (!cpu_online(1) ||
		    !cpumask_test_cpu(1, irq_data_get_affinity_mask(data)))
			ret = -EBUSY;
		else {
			/* No action is installed yet. Program our controller now;
			 * genirq's ordinary setter may defer an inactive IRQ move. */
			ret = esp_intmtx_set_affinity(data,
					irq_data_get_affinity_mask(data), false);
			if (ret >= 0)
				ret = 0;
		}
		if (!ret && !cpumask_equal(
			irq_data_get_effective_affinity_mask(data), cpumask_of(1)))
			ret = -EIO;
	}
	raw_spin_unlock_irqrestore(&desc->lock, flags);
	return ret;
}

static void esp_intmtx_release_resources(struct irq_data *data)
{
	struct esp_intmtx_radio_claim *claim = esp_intmtx_claim(data);
	struct irq_desc *desc = irq_data_to_desc(data);
	unsigned long flags;
	unsigned long saved;
	bool radio;
	int ret;

	if (WARN_ON_ONCE(!claim))
		return;

	/* Last action is removed and joined; request_mutex prevents reuse until
	 * the generic IRQ core finishes releasing this source's resources. */
	raw_spin_lock_irqsave(&desc->lock, flags);
	radio = !!claim->owner;
	saved = claim->status;
	claim->owner = NULL;
	if (radio) {
		ret = irq_set_affinity_locked(data, &claim->affinity, false);
		WARN_ON_ONCE(ret);
	}
	raw_spin_unlock_irqrestore(&desc->lock, flags);
	if (radio)
		irq_modify_status(data->irq, IRQ_NO_BALANCING | IRQ_DISABLE_UNLAZY,
				  saved);
}

static struct irq_chip esp_intmtx_chip = {
	.name		= "esp-intmtx",
	.irq_mask	= irq_chip_mask_parent,
	.irq_unmask	= irq_chip_unmask_parent,
	.irq_set_type	= irq_chip_set_type_parent,
	.irq_set_affinity = esp_intmtx_set_affinity,
	.irq_request_resources = esp_intmtx_request_resources,
	.irq_release_resources = esp_intmtx_release_resources,
	.flags		= IRQCHIP_SKIP_SET_WAKE | IRQCHIP_AFFINITY_PRE_STARTUP,
};

/* Caller holds desc->lock and has verified its exclusive, disabled action
 * and owner cookie. No unowned mapping or generic IRQ flag grants CPU0. */
int esp_intmtx_radio_bind(struct irq_data *data, void *owner)
{
	struct esp_intmtx_radio_claim *claim = esp_intmtx_claim(data);
	int ret;

	if (!owner || !claim || irq_data_get_irq_chip(data) != &esp_intmtx_chip ||
	    data->hwirq > INTMTX_MAX_SRC || claim->owner)
		return -EINVAL;
	claim->owner = owner;
	ret = irq_set_affinity_locked(data, cpumask_of(0), false);
	if (ret || !cpumask_equal(irq_data_get_effective_affinity_mask(data),
				 cpumask_of(0))) {
		/* The exclusive action still owns its failed admission attempt.
		 * free_irq's resource release must restore its pre-request flags
		 * and affinity even when direct registration never publishes. */
		return ret ?: -EIO;
	}
	return 0;
}

/* Cold radio admission only, never an execution/packet admission scan.
 * The caller holds cpus_read_lock and has published the policy, preventing
 * CPU1 removal and new conflicting affinity requests during this walk. */
int esp_intmtx_radio_isolation_validate(void)
{
	struct irq_desc *desc;
	unsigned int irq;
	int ret = 0;

	if (!intmtx)
		return -ENODEV;
	irq_lock_sparse();
	for_each_irq_desc(irq, desc) {
		struct irq_data *data = irq_desc_get_irq_data(desc);
		unsigned long flags;

		/* Join an installation that passed resource admission just before
		 * policy publication, but has not yet published its action. */
		if (irq_data_get_irq_chip(data) != &esp_intmtx_chip)
			continue;
		mutex_lock(&desc->request_mutex);
		raw_spin_lock_irqsave(&desc->lock, flags);
		if (irq_data_get_irq_chip(data) == &esp_intmtx_chip &&
		    desc->action && !esp_intmtx_cpu0_owner(data)) {
			if (!cpumask_test_cpu(1, irq_data_get_affinity_mask(data)))
				ret = -EBUSY;
			else if (!cpumask_equal(
				irq_data_get_effective_affinity_mask(data), cpumask_of(1))) {
				if (irqd_affinity_is_managed(data))
					ret = -EBUSY;
				else
					ret = irq_set_affinity_locked(data,
						irq_data_get_affinity_mask(data), false);
				if (!ret && !cpumask_equal(
					irq_data_get_effective_affinity_mask(data), cpumask_of(1)))
					ret = -EIO;
			}
		}
		raw_spin_unlock_irqrestore(&desc->lock, flags);
		mutex_unlock(&desc->request_mutex);
		if (ret) {
			pr_err("esp-intmtx: radio admission needs IRQ%u on CPU1: %d\n",
			       irq, ret);
			break;
		}
	}
	irq_unlock_sparse();
	return ret;
}

/* Generic online restoration covers managed IRQs only. Reapply CPU1 to
 * this controller's active unmanaged peripherals after CPU1 comes back.
 * Radio/LP IPC are unbalanceable CPU0 owners; explicit CPU0 affinity wins. */
static int esp_intmtx_cpu_online(unsigned int cpu)
{
	struct irq_desc *desc;
	unsigned int irq, moved = 0;
	int ret = 0;

	if (cpu != 1)
		return 0;
	/* Same lifetime/lock order as IRQ core online restoration: sparse IRQ
	 * mutex, descriptor, chip policy. Disposal cannot invalidate this walk. */
	irq_lock_sparse();
	for_each_irq_desc(irq, desc) {
		struct irq_data *data = irq_desc_get_irq_data(desc);
		unsigned long flags;

		raw_spin_lock_irqsave(&desc->lock, flags);
		if (irq_data_get_irq_chip(data) == &esp_intmtx_chip &&
		    desc->action && !esp_intmtx_cpu0_owner(data) &&
		    !irqd_affinity_is_managed(data) &&
		    cpumask_test_cpu(cpu, irq_data_get_affinity_mask(data)) &&
		    !cpumask_equal(irq_data_get_effective_affinity_mask(data),
				   cpumask_of(cpu))) {
			ret = irq_set_affinity_locked(data,
					irq_data_get_affinity_mask(data), false);
			if (!ret && !cpumask_equal(
				    irq_data_get_effective_affinity_mask(data),
				    cpumask_of(cpu)))
				ret = -EIO;
			if (!ret)
				moved++;
		}
		raw_spin_unlock_irqrestore(&desc->lock, flags);
		if (ret) {
			pr_err("esp-intmtx: CPU1 IRQ%u affinity restore failed: %d\n",
			       irq, ret);
			break;
		}
	}
	irq_unlock_sparse();
	if (!ret)
		pr_info("esp-intmtx: CPU1 online restored %u peripheral IRQs\n", moved);
	return ret;
}

static int esp_intmtx_cpu_down(unsigned int cpu)
{
	/* AP_ONLINE teardown can refuse before IRQ affinity migration or CPU
	 * death. Ordinary hotplug cannot break an active radio isolation lease.
	 * Frozen system suspend has already quiesced the devices and IRQs. */
	if (cpu == 1 && esp_clic_radio_isolation_active() && !esp_intmtx_pm_migration())
		return -EBUSY;
	return 0;
}

static int __init esp_intmtx_hotplug_init(void)
{
	int ret;

	if (!intmtx)
		return 0;
	ret = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN, "irqchip/esp-intmtx:online",
			       esp_intmtx_cpu_online, esp_intmtx_cpu_down);
	if (ret < 0)
		panic("esp-intmtx: required peripheral hotplug policy failed: %d", ret);
	return 0;
}
subsys_initcall(esp_intmtx_hotplug_init);

void esp_intmtx_route_local(unsigned int cpu, unsigned int source,
			    unsigned int slot)
{
	void __iomem *reg;
	u32 val;

	if (WARN_ON_ONCE(!intmtx || cpu > 1 || source > INTMTX_MAX_SRC ||
			 slot > INTMTX_SLOT_MASK))
		return;

	/*
	 * Unlike the official hart0 device path, the SMP-local timer and IPI
	 * routes enter Linux directly from the per-hart S-mode CLIC bank.  S31
	 * therefore requires PASS_LEVEL_S on these routes.
	 */
	reg = intmtx->base + cpu * INTMTX_CORE_STRIDE +
	      INTMTX_SRC_REG(source);
	val = readl_relaxed(reg);
	val &= ~(INTMTX_SLOT_MASK | INTMTX_PASS_LEVEL_MASK);
	val |= (slot & INTMTX_SLOT_MASK) | INTMTX_PASS_LEVEL_S;
	writel_relaxed(val, reg);
}

void esp_intmtx_unroute_local(unsigned int cpu, unsigned int source)
{
	void __iomem *reg;
	u32 val;

	if (WARN_ON_ONCE(!intmtx || cpu > 1 || source > INTMTX_MAX_SRC))
		return;

	reg = intmtx->base + cpu * INTMTX_CORE_STRIDE +
	      INTMTX_SRC_REG(source);
	val = readl_relaxed(reg);
	val &= ~(INTMTX_SLOT_MASK | INTMTX_PASS_LEVEL_MASK);
	writel_relaxed(val, reg);
}

static int esp_intmtx_domain_alloc(struct irq_domain *domain,
				   unsigned int virq, unsigned int nr_irqs,
				   void *arg)
{
	struct irq_fwspec *fwspec = arg;
	struct irq_fwspec parent_fwspec;
	struct esp_intmtx *priv = domain->host_data;
	irq_hw_number_t src;
	unsigned int type;
	unsigned long flags;
	int slot, ret;

	if (nr_irqs != 1)
		return -EINVAL;

	ret = irq_domain_translate_twocell(domain, fwspec, &src, &type);
	if (ret)
		return ret;

	if (src > INTMTX_MAX_SRC)
		return -EINVAL;

	raw_spin_lock_irqsave(&priv->lock, flags);
	if (priv->src_to_slot[src]) {
		ret = -EBUSY;
		goto out;
	}

	slot = esp_intmtx_alloc_slot(priv);
	if (slot < 0) {
		ret = slot;
		goto out;
	}

	writel_relaxed((u32)slot & INTMTX_SLOT_MASK,
		       priv->base + INTMTX_SRC_REG(src));
	priv->src_to_slot[src] = (u8)slot;
	raw_spin_unlock_irqrestore(&priv->lock, flags);

	irq_domain_set_info(domain, virq, src, &esp_intmtx_chip,
			    priv, handle_level_irq, NULL, NULL);

	parent_fwspec.fwnode = domain->parent->fwnode;
	parent_fwspec.param_count = 2;
	parent_fwspec.param[0] = slot;
	parent_fwspec.param[1] = type;

	ret = irq_domain_alloc_irqs_parent(domain, virq, 1, &parent_fwspec);
	if (ret) {
		raw_spin_lock_irqsave(&priv->lock, flags);
		writel_relaxed(0, priv->base + INTMTX_SRC_REG(src));
		priv->src_to_slot[src] = 0;
		esp_intmtx_free_slot(priv, slot);
		raw_spin_unlock_irqrestore(&priv->lock, flags);
		return ret;
	}

	irq_data_update_effective_affinity(irq_get_irq_data(virq), cpumask_of(0));
	return 0;

out:
	raw_spin_unlock_irqrestore(&priv->lock, flags);
	return ret;
}

static void esp_intmtx_domain_free(struct irq_domain *domain,
				   unsigned int virq, unsigned int nr_irqs)
{
	struct esp_intmtx *priv = domain->host_data;
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	unsigned long flags;
	unsigned int src, slot;

	if (!d)
		return;

	src = d->hwirq;
	if (src > INTMTX_MAX_SRC)
		return;

	raw_spin_lock_irqsave(&priv->lock, flags);
	slot = priv->src_to_slot[src];
	writel_relaxed(0, priv->base + INTMTX_SRC_REG(src));
	writel_relaxed(0, priv->base + INTMTX_CORE_STRIDE + INTMTX_SRC_REG(src));
	priv->src_to_slot[src] = 0;
	esp_intmtx_free_slot(priv, slot);
	raw_spin_unlock_irqrestore(&priv->lock, flags);

	irq_domain_free_irqs_common(domain, virq, nr_irqs);
}

static const struct irq_domain_ops esp_intmtx_domain_ops = {
	.translate	= irq_domain_translate_twocell,
	.alloc		= esp_intmtx_domain_alloc,
	.free		= esp_intmtx_domain_free,
};

static int __init esp_intmtx_init(struct device_node *node,
				  struct device_node *parent)
{
	struct irq_domain *parent_domain, *domain;
	struct esp_intmtx *priv;
	unsigned int src;

	if (intmtx) {
		pr_err("esp-intmtx: already initialised\n");
		return -EEXIST;
	}

	parent_domain = irq_find_host(parent);
	if (!parent_domain) {
		pr_err("esp-intmtx: %pOF: parent CLIC domain missing\n", node);
		return -ENXIO;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = of_iomap(node, 0);
	if (!priv->base) {
		pr_err("esp-intmtx: %pOF: failed to map regs\n", node);
		kfree(priv);
		return -ENXIO;
	}
	raw_spin_lock_init(&priv->lock);
	set_bit(ESP32S31_CLIC_IPI_SLOT - CLIC_SLOT_FIRST,
		priv->slots_in_use);
	set_bit(ESP32S31_CLIC_TIMER_SLOT - CLIC_SLOT_FIRST,
		priv->slots_in_use);

	/* Clear every source's slot routing so we start from a known
	 * state — anything OpenSBI or the chip ROM left in place is
	 * unmapped here.
	 */
	for (src = 0; src <= INTMTX_MAX_SRC; src++)
		writel_relaxed(0, priv->base + INTMTX_SRC_REG(src));

	intmtx = priv;
	register_syscore_ops(&esp_intmtx_syscore_ops);

	domain = irq_domain_create_hierarchy(parent_domain, 0,
					     INTMTX_MAX_SRC + 1,
					     of_fwnode_handle(node),
					     &esp_intmtx_domain_ops, priv);
	if (!domain) {
		iounmap(priv->base);
		kfree(priv);
		intmtx = NULL;
		return -ENOMEM;
	}

	if (IS_ENABLED(CONFIG_SMP)) {
		int ret = esp32s31_smp_irq_init();

		if (ret) {
			pr_err("esp-intmtx: failed to initialize SMP local IRQs: %d\n",
			       ret);
			return ret;
		}
	}

	pr_info("esp-intmtx: %u sources, parent %pOF; %u device slots, CPU1 preferred\n",
		 INTMTX_MAX_SRC + 1, parent, CLIC_SLOT_COUNT - 2);
	return 0;
}

IRQCHIP_DECLARE(esp32s31_intmtx, "espressif,esp32s31-intmtx", esp_intmtx_init);
