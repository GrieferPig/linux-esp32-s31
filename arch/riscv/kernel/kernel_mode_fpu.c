// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiFive
 */

#include <linux/export.h>
#include <linux/preempt.h>

#include <asm/csr.h>
#include <asm/fpu.h>
#include <asm/processor.h>
#include <asm/switch_to.h>
#include <asm/esp32s31_blob_fp.h>

void kernel_fpu_begin(void)
{
	preempt_disable();
	esp32s31_blob_fp_kernel_begin();
	fstate_save(current, task_pt_regs(current));
	csr_set(CSR_SSTATUS, SR_FS);
}
EXPORT_SYMBOL_GPL(kernel_fpu_begin);

void kernel_fpu_end(void)
{
	esp32s31_blob_fp_kernel_end_check();
	csr_clear(CSR_SSTATUS, SR_FS);
	fstate_restore(current, task_pt_regs(current));
	esp32s31_blob_fp_kernel_end();
	preempt_enable();
}
EXPORT_SYMBOL_GPL(kernel_fpu_end);
