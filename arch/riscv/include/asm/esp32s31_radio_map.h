/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ASM_RISCV_ESP32S31_RADIO_MAP_H
#define _ASM_RISCV_ESP32S31_RADIO_MAP_H

#include <linux/types.h>

/* Sv32 supervisor leaves shared with init_mm. They are never user VMAs. */
#define S31_RADIO_WINDOW_SIZE 0x00400000UL
static const unsigned long s31_radio_window_base[] = {
	0x20000000UL, 0x20400000UL, 0x20800000UL,
	0x2c000000UL, 0x2f000000UL, 0x2f800000UL,
};
#define S31_RADIO_WINDOW_COUNT 6

/* Return a conflicting window, without computing a possibly wrapping end. */
static inline bool s31_radio_reserved_range(unsigned long addr,
		unsigned long len, unsigned long *low, unsigned long *high)
{
	unsigned int i;

	if (!len)
		return false;
	for (i = 0; i < S31_RADIO_WINDOW_COUNT; i++) {
		unsigned long base = s31_radio_window_base[i];

		if (addr < base + S31_RADIO_WINDOW_SIZE &&
		    (addr >= base || len > base - addr)) {
			if (low)
				*low = base;
			if (high)
				*high = base + S31_RADIO_WINDOW_SIZE;
			return true;
		}
	}
	return false;
}
#endif
