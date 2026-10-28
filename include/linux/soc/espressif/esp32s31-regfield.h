/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_ESP32S31_REGFIELD_H
#define _LINUX_ESP32S31_REGFIELD_H
#include <linux/io.h>
/* Register-layout bitfields are used only in local values. All hardware
 * accesses use readl/writel, never C bitfield MMIO read-modify-write.
 */
#define s31_reg_set(hw, reg, field, value)       \
	do {                                     \
		typeof((hw)->reg) __v;           \
		__v.val = readl(&(hw)->reg.val); \
		__v.field = (value);             \
		writel(__v.val, &(hw)->reg.val); \
	} while (0)
#endif
