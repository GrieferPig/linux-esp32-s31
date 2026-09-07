// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 Western Digital Corporation or its affiliates.
 * Copyright (c) 2022 Ventana Micro Systems Inc.
 */

#define pr_fmt(fmt) "suspend: " fmt

#include <linux/ftrace.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/sizes.h>
#include <linux/soc/espressif/esp32s31-cache.h>
#include <linux/soc/espressif/esp32s31-lp.h>
#include <linux/suspend.h>
#include <asm/csr.h>
#include <asm/pgtable.h>
#include <asm/sbi.h>
#include <asm/suspend.h>

void suspend_save_csrs(struct suspend_context *context)
{
	if (riscv_has_extension_unlikely(RISCV_ISA_EXT_XLINUXENVCFG))
		context->envcfg = csr_read(CSR_ENVCFG);
	context->tvec = csr_read(CSR_TVEC);
	context->ie = csr_read(CSR_IE);

	/*
	 * No need to save/restore IP CSR (i.e. MIP or SIP) because:
	 *
	 * 1. For no-MMU (M-mode) kernel, the bits in MIP are set by
	 *    external devices (such as interrupt controller, timer, etc).
	 * 2. For MMU (S-mode) kernel, the bits in SIP are set by
	 *    M-mode firmware and external devices (such as interrupt
	 *    controller, etc).
	 */

#ifdef CONFIG_MMU
	if (riscv_has_extension_unlikely(RISCV_ISA_EXT_SSTC)) {
		context->stimecmp = csr_read(CSR_STIMECMP);
#if __riscv_xlen < 64
		context->stimecmph = csr_read(CSR_STIMECMPH);
#endif
	}

	context->satp = csr_read(CSR_SATP);
#endif
}

#ifdef CONFIG_SOC_ESP32S31
static unsigned long esp32s31_suspend_satp(void)
{
	pgd_t *pgd = current->active_mm ? current->active_mm->pgd :
					       swapper_pg_dir;

#ifdef CONFIG_XIP_KERNEL
	extern char _sdata[];

	return PFN_DOWN(CONFIG_PHYS_RAM_BASE + (uintptr_t)pgd -
			(uintptr_t)_sdata) | (unsigned long)satp_mode;
#else
	return virt_to_pfn(pgd) | (unsigned long)satp_mode;
#endif
}
#endif

void suspend_restore_csrs(struct suspend_context *context)
{
	csr_write(CSR_SCRATCH, 0);
	if (riscv_has_extension_unlikely(RISCV_ISA_EXT_XLINUXENVCFG))
		csr_write(CSR_ENVCFG, context->envcfg);
	csr_write(CSR_TVEC, context->tvec);
#ifdef CONFIG_SOC_ESP32S31
	/*
	 * S31 CLIC interrupt enables live in per-source MMIO slots.  SIE can
	 * be read for generic context capture but a direct S-mode write is not
	 * implemented by the CLIC execution mode on current silicon.
	 */
#else
	csr_write(CSR_IE, context->ie);
#endif

#ifdef CONFIG_MMU
	if (riscv_has_extension_unlikely(RISCV_ISA_EXT_SSTC)) {
		csr_write(CSR_STIMECMP, context->stimecmp);
#if __riscv_xlen < 64
		csr_write(CSR_STIMECMPH, context->stimecmph);
#endif
	}
	csr_write(CSR_SATP, context->satp);
#endif
}

int cpu_suspend(unsigned long arg,
		int (*finish)(unsigned long arg,
			      unsigned long entry,
			      unsigned long context))
{
	int rc = 0;
	struct suspend_context context = { 0 };

	/* Finisher should be non-NULL */
	if (!finish)
		return -EINVAL;

	/* Save additional CSRs*/
	suspend_save_csrs(&context);
#ifdef CONFIG_SOC_ESP32S31
	if (!context.satp)
		WRITE_ONCE(context.satp, esp32s31_suspend_satp());
#endif

	/*
	 * Function graph tracer state gets incosistent when the kernel
	 * calls functions that never return (aka finishers) hence disable
	 * graph tracing during their execution.
	 */
	pause_graph_tracing();

	/* Save context on stack */
	if (__cpu_suspend_enter(&context)) {
		/* Call the finisher */
		rc = finish(arg, __pa_symbol(__cpu_resume_enter),
			    (ulong)&context);

		/*
		 * Should never reach here, unless the suspend finisher
		 * fails. Successful cpu_suspend() should return from
		 * __cpu_resume_entry()
		 */
		if (!rc)
			rc = -EOPNOTSUPP;
	}

	/* Enable function graph tracer */
	unpause_graph_tracing();

	/* Restore additional CSRs */
	suspend_restore_csrs(&context);

	return rc;
}

#ifdef CONFIG_RISCV_SBI
static int sbi_system_suspend(unsigned long sleep_type,
			      unsigned long resume_addr,
			      unsigned long opaque)
{
	struct sbiret ret;

#ifdef CONFIG_SOC_ESP32S31
	/*
	 * OpenSBI's non-retentive warmboot reinitializes the HP cache before
	 * Linux can consume its stack-resident suspend context.  Publish all
	 * cached PSRAM, including page tables, before crossing that boundary.
	 */
	esp32s31_cache_writeback(CONFIG_PHYS_RAM_BASE, SZ_16M);
#endif
	ret = sbi_ecall(SBI_EXT_SUSP, SBI_EXT_SUSP_SYSTEM_SUSPEND,
			sleep_type, resume_addr, opaque, 0, 0, 0);
	if (ret.error)
		return sbi_err_map_linux_errno(ret.error);

	return ret.value;
}

static int sbi_system_suspend_enter(suspend_state_t state)
{
	return cpu_suspend(SBI_SUSP_SLEEP_TYPE_SUSPEND_TO_RAM, sbi_system_suspend);
}

#ifdef CONFIG_SOC_ESP32S31
static int sbi_system_suspend_prepare(void)
{
	return esp32s31_lp_system_suspend_prepare();
}

static void sbi_system_suspend_finish(void)
{
	esp32s31_lp_system_suspend_finish();
}
#endif

static const struct platform_suspend_ops sbi_system_suspend_ops = {
	.valid = suspend_valid_only_mem,
	#ifdef CONFIG_SOC_ESP32S31
	.prepare = sbi_system_suspend_prepare,
	.finish = sbi_system_suspend_finish,
	#endif
	.enter = sbi_system_suspend_enter,
};

static int __init sbi_system_suspend_init(void)
{
	if (sbi_spec_version >= sbi_mk_version(2, 0) &&
	    sbi_probe_extension(SBI_EXT_SUSP) > 0) {
		pr_info("SBI SUSP extension detected\n");
		if (IS_ENABLED(CONFIG_SUSPEND))
			suspend_set_ops(&sbi_system_suspend_ops);
	}

	return 0;
}

arch_initcall(sbi_system_suspend_init);

static int sbi_suspend_finisher(unsigned long suspend_type,
				unsigned long resume_addr,
				unsigned long opaque)
{
	struct sbiret ret;

	ret = sbi_ecall(SBI_EXT_HSM, SBI_EXT_HSM_HART_SUSPEND,
			suspend_type, resume_addr, opaque, 0, 0, 0);

	return (ret.error) ? sbi_err_map_linux_errno(ret.error) : 0;
}

int riscv_sbi_hart_suspend(u32 state)
{
	if (state & SBI_HSM_SUSP_NON_RET_BIT)
		return cpu_suspend(state, sbi_suspend_finisher);
	else
		return sbi_suspend_finisher(state, 0, 0);
}

bool riscv_sbi_suspend_state_is_valid(u32 state)
{
	if (state > SBI_HSM_SUSPEND_RET_DEFAULT &&
	    state < SBI_HSM_SUSPEND_RET_PLATFORM)
		return false;

	if (state > SBI_HSM_SUSPEND_NON_RET_DEFAULT &&
	    state < SBI_HSM_SUSPEND_NON_RET_PLATFORM)
		return false;

	return true;
}

bool riscv_sbi_hsm_is_supported(void)
{
	/*
	 * The SBI HSM suspend function is only available when:
	 * 1) SBI version is 0.3 or higher
	 * 2) SBI HSM extension is available
	 */
	if (sbi_spec_version < sbi_mk_version(0, 3) ||
	    !sbi_probe_extension(SBI_EXT_HSM)) {
		pr_info("HSM suspend not available\n");
		return false;
	}

	return true;
}
#endif /* CONFIG_RISCV_SBI */
