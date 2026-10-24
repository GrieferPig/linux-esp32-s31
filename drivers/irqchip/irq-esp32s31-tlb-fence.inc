/* SPDX-License-Identifier: GPL-2.0-only */
/* Only SFENCE.VMA may run from a raw-lock/CSD wait. No CALL_FUNC, TTWU,
 * scheduler, cache/SBI, device IRQ or payload callback is dispatched here. */
#ifdef CONFIG_SMP
static DEFINE_RAW_SPINLOCK(esp32s31_tlb_fence_lock);
static atomic_t esp32s31_tlb_fence_ready = ATOMIC_INIT(0);
static atomic_t esp32s31_tlb_fence_slots[2] = { ATOMIC_INIT(0), ATOMIC_INIT(0) };

/* Use RMW observations throughout: cached loads must not be mixed with the
 * S31 AMO path for handshake words. 1=request, 2=claimed, 0=fence finished. */
static int esp32s31_tlb_word_read(atomic_t *word)
{
 return atomic_fetch_add(0, word);
}

void esp32s31_tlb_flush_poll(void)
{

 unsigned long flags;
 unsigned int cpu;
 atomic_t *slot;

 if (unlikely(!READ_ONCE(esp32s31_doorbells) || in_nmi())) return;
 raw_local_irq_save(flags);
 cpu = raw_smp_processor_id();
 if (cpu < 2) {
  slot = &esp32s31_tlb_fence_slots[cpu];
  /* IRQ masking gives this hart one consumer; NMI never enters this leaf.
   * The serialized sender cannot modify its request until acknowledgement.
   * Claim with AMO, not a cached read or an LR/SC observation. */
  if (esp32s31_tlb_word_read(slot) == 1) {
   if (WARN_ON_ONCE(atomic_xchg(slot, 2) != 1))
    panic("esp32s31: concurrent TLB mailbox consumer");
   /* Full local invalidation also covers the no-ASID S31 configuration.
    * This leaf does not touch the shared cache controller or call SBI. */
   asm volatile("sfence.vma zero, zero" ::: "memory");
   atomic_xchg(slot, 0);
  }
 }
 raw_local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(esp32s31_tlb_flush_poll);

static void esp32s31_tlb_fence_init(void)
{
 raw_spin_lock_init(&esp32s31_tlb_fence_lock);
 atomic_xchg(&esp32s31_tlb_fence_ready, 0);
 atomic_xchg(&esp32s31_tlb_fence_slots[0], 0);
 atomic_xchg(&esp32s31_tlb_fence_slots[1], 0);
}

static void esp32s31_tlb_cpu_ready(unsigned int cpu, bool ready)
{
 unsigned long flags;

 if (cpu > 1) return;
 raw_spin_lock_irqsave(&esp32s31_tlb_fence_lock, flags);
 /* A new hart has no surviving translations from its prior lifetime. A
  * dying hart serves requests while waiting for this lock, then becomes
  * unavailable atomically with respect to new senders. */
 asm volatile("sfence.vma zero, zero" ::: "memory");
 atomic_xchg(&esp32s31_tlb_fence_slots[cpu], 0);
 if (ready) atomic_or(1U << cpu, &esp32s31_tlb_fence_ready);
 else atomic_andnot(1U << cpu, &esp32s31_tlb_fence_ready);
 raw_spin_unlock_irqrestore(&esp32s31_tlb_fence_lock, flags);
}

bool esp32s31_tlb_flush_remote(const struct cpumask *mask)
{
 unsigned long flags;
 unsigned int cpu, self, targets = 0;
 int ready;

 if (unlikely(!READ_ONCE(esp32s31_doorbells))) return false;
 /* TLB shootdowns are not an NMI API. Re-entering a sender would strand
  * its own publication lock; do not report a flush that did not occur. */
 if (WARN_ON_ONCE(in_nmi())) panic("esp32s31: TLB shootdown from NMI");
 raw_spin_lock_irqsave(&esp32s31_tlb_fence_lock, flags);
 self = raw_smp_processor_id();
 ready = esp32s31_tlb_word_read(&esp32s31_tlb_fence_ready);
 for (cpu = 0; cpu < 2; cpu++) {
  if (!(ready & (1U << cpu)) || !cpu_online(cpu) ||
      (mask && !cpumask_test_cpu(cpu, mask))) continue;
  if (cpu == self) {
   asm volatile("sfence.vma zero, zero" ::: "memory");
   continue;
  }
  if (WARN_ON_ONCE(atomic_xchg(&esp32s31_tlb_fence_slots[cpu], 1)))
   panic("esp32s31: overwriting an unfinished TLB shootdown");
  targets |= 1U << cpu;
  wmb();
  writel(1, esp32s31_doorbells + esp32s31_doorbell_offset(cpu));
 }
 for (cpu = 0; cpu < 2; cpu++) {
  if (!(targets & (1U << cpu))) continue;
  while (esp32s31_tlb_word_read(&esp32s31_tlb_fence_slots[cpu])) {
   /* Two opposite senders can meet: the lock waiter serves this request,
    * and a sender can serve a request that preceded its own publication. */
   esp32s31_tlb_flush_poll();
   cpu_relax();
  }
 }
 raw_spin_unlock_irqrestore(&esp32s31_tlb_fence_lock, flags);
 return true;
}
#else
static void esp32s31_tlb_fence_init(void) { }
static void esp32s31_tlb_cpu_ready(unsigned int cpu, bool ready) { }
#endif
