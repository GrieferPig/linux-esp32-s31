/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ASM_RISCV_SPINLOCK_H
#define _ASM_RISCV_SPINLOCK_H

#ifdef CONFIG_SOC_ESP32S31

#include <linux/atomic.h>
#include <asm-generic/spinlock_types.h>

/*
 * S31's AMO path and ordinary cached loads/stores are not coherent for the
 * lock word.  In particular, the generic ticket lock releases with a 16-bit
 * cached store and waits with cached loads, while its ticket increment uses a
 * 32-bit AMO.  Under SMP the backing word can already be unlocked while both
 * harts keep spinning on a stale cached ticket value.
 *
 * Keep every access to an S31 spinlock on the same 32-bit AMO path.  This is a
 * simple test-and-set lock because the official S31 port is single-hart; SMP
 * is the local extension that needs this hardware-specific serialization.
 */
static __always_inline u32 esp32s31_spin_acquire(arch_spinlock_t *lock)
{
	u32 old;

	__asm__ __volatile__("amoswap.w.aq %0, %2, %1"
		: "=r" (old), "+A" (lock->val.counter)
		: "r" (1)
		: "memory");
	return old;
}

static __always_inline u32 esp32s31_spin_read(arch_spinlock_t *lock)
{
	u32 value;

	__asm__ __volatile__("amoadd.w.aq %0, zero, %1"
		: "=r" (value), "+A" (lock->val.counter)
		:
		: "memory");
	return value;
}

static __always_inline void arch_spin_lock_init(arch_spinlock_t *lock)
{
	u32 old;

	/* Dynamic locks can be allocated from a recycled cache line whose
	 * backing word still contains old slab data.  Initialize through AMO so
	 * the first acquire observes zero on the same path it will later use. */
	__asm__ __volatile__("amoswap.w.rl %0, zero, %1"
		: "=r" (old), "+A" (lock->val.counter)
		:
		: "memory");
}
#define arch_spin_lock_init arch_spin_lock_init

static __always_inline void arch_spin_lock(arch_spinlock_t *lock)
{
	/* Never dispatch Linux IPIs while waiting for a raw lock. The generic
	 * CALL_FUNC queue also carries scheduler TTWU callbacks, which can acquire
	 * a runqueue lock and recurse into this path. Cross-hart fences use the
	 * SBI RFENCE extension instead.
	 */
	while (esp32s31_spin_acquire(lock))
		__asm__ __volatile__("nop");
}

static __always_inline bool arch_spin_trylock(arch_spinlock_t *lock)
{
	return !esp32s31_spin_acquire(lock);
}

static __always_inline void arch_spin_unlock(arch_spinlock_t *lock)
{
	u32 old;

	__asm__ __volatile__("amoswap.w.rl %0, zero, %1"
		: "=r" (old), "+A" (lock->val.counter)
		:
		: "memory");
}

static __always_inline int arch_spin_value_unlocked(arch_spinlock_t lock)
{
	return !lock.val.counter;
}

static __always_inline int arch_spin_is_locked(arch_spinlock_t *lock)
{
	return !!esp32s31_spin_read(lock);
}

static __always_inline int arch_spin_is_contended(arch_spinlock_t *lock)
{
	return arch_spin_is_locked(lock);
}

#include <asm/qrwlock.h>

#else

#include <asm-generic/spinlock.h>

#endif /* CONFIG_SOC_ESP32S31 */

#endif /* _ASM_RISCV_SPINLOCK_H */
