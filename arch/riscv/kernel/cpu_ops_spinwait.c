// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 Western Digital Corporation or its affiliates.
 */

#include <linux/errno.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/string.h>
#include <linux/sched/task_stack.h>
#include <asm/cpu_ops.h>
#include <asm/pgtable.h>
#include <asm/sbi.h>
#include <asm/smp.h>

#include "head.h"

const struct cpu_operations cpu_ops_spinwait;
void *__cpu_spinwait_stack_pointer[NR_CPUS] __section(".data");
void *__cpu_spinwait_task_pointer[NR_CPUS] __section(".data");

static void cpu_update_secondary_bootdata(unsigned int cpuid,
				   struct task_struct *tidle)
{
	unsigned long hartid = cpuid_to_hartid_map(cpuid);

	/*
	 * The hartid must be less than NR_CPUS to avoid out-of-bound access
	 * errors for __cpu_spinwait_stack/task_pointer. That is not always possible
	 * for platforms with discontiguous hartid numbering scheme. That's why
	 * spinwait booting is not the recommended approach for any platforms
	 * booting Linux in S-mode and can be disabled in the future.
	 */
	if (hartid == INVALID_HARTID || hartid >= (unsigned long) NR_CPUS)
		return;

	/* Make sure tidle is updated */
	smp_mb();
	WRITE_ONCE(__cpu_spinwait_stack_pointer[hartid], task_pt_regs(tidle));
	WRITE_ONCE(__cpu_spinwait_task_pointer[hartid], tidle);
}

#ifdef CONFIG_SOC_ESP32P4
/*
 * Matches the retained IDF loader, not a generic spin-table protocol.
 * CPU1 is parked with interrupts off in internal SRAM; it only accepts the
 * fixed kernel Image entry and original external DTB. The Image head itself
 * selects the secondary path before any boot-CPU initialization.
 */
struct p4_secondary_mailbox {
	u32 magic;
	u32 ready;
	u32 release;
	u32 status;
	u32 entry;
	u32 dtb;
	u32 revision;
};

/* cpu_start is kept in cpu_operations, but P4 permits only initial boot.
 * Keep this noinline reference annotated: _dtb_early_pa is init-only and all
 * P4 secondary starts complete before free_initmem(). No CPU hotplug path
 * may call this after the boot-time DT storage has been released. */
static int __ref p4_prepare_secondary(unsigned int cpu,
				struct p4_secondary_mailbox __iomem **result)
{
	struct device_node *chosen;
	struct p4_secondary_mailbox __iomem *mailbox;
	u32 prop[2];
	int ret;

	BUILD_BUG_ON(IS_ENABLED(CONFIG_HOTPLUG_CPU));
	if (cpuid_to_hartid_map(cpu) != 1)
		return -EINVAL;
	chosen = of_find_node_by_path("/chosen");
	ret = of_property_read_u32_array(chosen,
				"espressif,cpu1-boot-mailbox", prop, 2);
	of_node_put(chosen);
	if (ret) {
		pr_err("P4 SMP: cold-ROM CPU1 start is not reconstructed; firmware mailbox required\n");
		return -EOPNOTSUPP;
	}
	if (prop[1] != sizeof(*mailbox) || (prop[0] & 63) ||
	    prop[0] < 0x4ff00000 || prop[0] > 0x4ff40000 - sizeof(*mailbox))
		return -EINVAL;

	mailbox = ioremap(prop[0], sizeof(*mailbox));
	if (!mailbox)
		return -ENOMEM;
	if (readl(&mailbox->magic) != 0x534d5034 ||
	    readl(&mailbox->ready) != 1 || readl(&mailbox->release) != 0 ||
	    readl(&mailbox->status) != 0 ||
	    readl(&mailbox->entry) != 0x48000000 ||
	    readl(&mailbox->dtb) != 0x48f00000 ||
	    dtb_early_pa != 0x48f00000 ||
	    readl(&mailbox->revision) != 103) {
		pr_err("P4 SMP: invalid firmware mailbox ready/release/status/entry/DT/revision\n");
		iounmap(mailbox);
		return -EINVAL;
	}
	*result = mailbox;
	return 0;
}
#endif

static int spinwait_cpu_start(unsigned int cpuid, struct task_struct *tidle)
{
#ifdef CONFIG_SOC_ESP32P4
	struct p4_secondary_mailbox __iomem *mailbox;
	int ret = p4_prepare_secondary(cpuid, &mailbox);

	if (ret)
		return ret;
#endif
	/*
	 * In this protocol, all cpus boot on their own accord.  _start
	 * selects the first cpu to boot the kernel and causes the remainder
	 * of the cpus to spin in a loop waiting for their stack pointer to be
	 * setup by that main cpu.  Writing to bootdata
	 * (i.e __cpu_spinwait_stack_pointer) signals to the spinning cpus that they
	 * can continue the boot process.
	 */
	cpu_update_secondary_bootdata(cpuid, tidle);
#ifdef CONFIG_SOC_ESP32P4
	/* Both HP harts share L1D. Publish both bootdata words before release.
	 * Do not change entry: the firmware explicitly rejects non-Image PCs. */
	wmb();
	writel(1, &mailbox->release);
	readl(&mailbox->release);
	iounmap(mailbox);
#endif

	return 0;
}

const struct cpu_operations cpu_ops_spinwait = {
	.cpu_start	= spinwait_cpu_start,
};
