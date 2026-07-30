// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 HWLoop and PIE task context management through OpenSBI. */

#include <linux/bug.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/string.h>

#include <asm/esp32s31_ext.h>
#include <asm/page.h>
#include <asm/processor.h>
#include <asm/sbi.h>

#define S31_SBI_EXT_COPROC	0x09000002
#define S31_SBI_COPROC_SWITCH	0
#define S31_SBI_COPROC_SAVE	1
#define S31_SBI_COPROC_RESTORE	2

static_assert(sizeof(struct esp32s31_ext_state) == 256);

static unsigned long s31_ext_pa(struct task_struct *task)
{
	return __pa(&task->thread.esp32s31_ext);
}

static void s31_ext_check(struct sbiret ret)
{
	WARN_ONCE(ret.error, "ESP32-S31 coprocessor SBI failed: %ld\n",
		  ret.error);
}

void esp32s31_ext_switch(struct task_struct *prev, struct task_struct *next)
{
	struct sbiret ret;

	ret = sbi_ecall(S31_SBI_EXT_COPROC, S31_SBI_COPROC_SWITCH,
			s31_ext_pa(prev), s31_ext_pa(next), 0, 0, 0, 0);
	s31_ext_check(ret);
}

void esp32s31_ext_save(struct task_struct *task)
{
	struct sbiret ret;

	/* Hardware state belongs to current; callers save current before copy. */
	WARN_ON_ONCE(task != current);
	ret = sbi_ecall(S31_SBI_EXT_COPROC, S31_SBI_COPROC_SAVE,
			s31_ext_pa(task), 0, 0, 0, 0, 0);
	s31_ext_check(ret);
}

void esp32s31_ext_restore(struct task_struct *task)
{
	struct sbiret ret;

	WARN_ON_ONCE(task != current);
	ret = sbi_ecall(S31_SBI_EXT_COPROC, S31_SBI_COPROC_RESTORE,
			s31_ext_pa(task), 0, 0, 0, 0, 0);
	s31_ext_check(ret);
}

void esp32s31_ext_reset(struct task_struct *task)
{
	memset(&task->thread.esp32s31_ext, 0,
	       sizeof(task->thread.esp32s31_ext));
	if (task == current)
		esp32s31_ext_restore(task);
}
