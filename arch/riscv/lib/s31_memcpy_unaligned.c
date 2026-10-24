/* SPDX-License-Identifier: GPL-2.0-only */
#include <linux/compiler.h>
#include <linux/types.h>

/* S31 runtime RAM-to-RAM copy, never an MMIO accessor. Linux's
 * RV32 memcpy uses bytes when the alignments differ (e.g. skb data at +2).
 * Merge aligned source words instead. Every load stays within the requested
 * span: copy four initial bytes before rounding the source down, and reserve
 * eight bytes for each merged word. No unaligned loads or over-read padding.
 * Entered only for differing alignments and length >= 32. Source/destination
 * must not overlap, as for memcpy. Early __memcpy/__pi_* remain unchanged. */
void *__s31_memcpy_unaligned(void *destination, const void *source, size_t length);

void *notrace __section(".s31_rx_ram.text")
__s31_memcpy_unaligned(void *destination,
					    const void *source, size_t length)
{
	u8 *d = destination;
	const u8 *s = source;
	const u32 *words;
	u32 low, high;
	unsigned int shift;

	while (length && ((unsigned long)d & 3)) {
		*d++ = *s++;
		length--;
	}
	if (length >= 12) {
		unsigned int i;

		/* Keep the first rounded-down source load inside the input span. */
		for (i = 0; i < 4; i++)
			*d++ = *s++;
		length -= 4;
		shift = ((unsigned long)s & 3) * 8;
		words = (const u32 *)((unsigned long)s & ~3UL);
		low = *words++;
		while (length >= 20) {
			u32 a = words[0], b = words[1];
			u32 c = words[2], e = words[3];

			((u32 *)d)[0] = (low >> shift) | (a << (32 - shift));
			((u32 *)d)[1] = (a >> shift) | (b << (32 - shift));
			((u32 *)d)[2] = (b >> shift) | (c << (32 - shift));
			((u32 *)d)[3] = (c >> shift) | (e << (32 - shift));
			low = e;
			words += 4;
			s += 16;
			d += 16;
			length -= 16;
		}
		while (length >= 8) {
			high = *words++;
			*(u32 *)d = (low >> shift) | (high << (32 - shift));
			low = high;
			s += 4;
			d += 4;
			length -= 4;
		}
	}
	while (length--)
		*d++ = *s++;
	return destination;
}
