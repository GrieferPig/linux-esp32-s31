/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_IRQCHIP_IRQ_ESP32S31_CLIC_H
#define _LINUX_IRQCHIP_IRQ_ESP32S31_CLIC_H
#include <linux/types.h>

/*
 * ESP32-S31 CLIC external-source priority selection.
 *
 * The priority/level byte lives in clicintctl[7:5]; the maximum (priority 7)
 * is 0xE0. Callers pass the byte masked with ESP32S31_CLIC_PRIO_MAX.
 */
#define ESP32S31_CLIC_PRIO_MAX		0xE0U
#define ESP32S31_CLIC_PRIO(level)	(((level) & 0x7U) << 5)

/* Caller owns the mapping through these calls. Descriptor and policy locks
 * serialize the selected hart bank against routing/priority changes. */
struct esp_clic_irq_state {
 u32 slot, cpu, priority, enabled, pending, direct;
};
int esp_clic_irq_get_state(unsigned int virq, struct esp_clic_irq_state *state);
/* Set and read back priority on the source's effective target hart. */
int esp_clic_irq_set_priority(unsigned int virq, unsigned int ctl);
/* Exclusive CPU0 background-priority policy. Timer/IPI slots are excluded.
 * Acquire in sleepable context; release after the owner's sources are joined.
 * CPU1 retains normal source priorities, including across affinity/PM moves.
 * Runtime CPU0 allocations remain in-band and recover their default on
 * release. A syscore sleep snapshot never replaces the pre-policy snapshot.
 */
struct esp_clic_priority_lease;
struct esp_clic_priority_lease *esp_clic_priority_acquire(unsigned int ctl);
int esp_clic_priority_release(struct esp_clic_priority_lease *lease);

/* Dedicated IRQs retain normal Linux flow/disable/free semantics. Register
 * after an exclusive IRQF_NO_AUTOEN request while disabled. Registration
 * admits that action's owner to CPU0; generic NO_BALANCING is insufficient.
 * Unregister and join before free_irq or mapping disposal. Last-action
 * resource release retires the CPU0 admission before another request.
 * Both management calls require a sleepable Linux-stack caller.
 */
int esp_clic_direct_register(unsigned int virq, void *owner);
int esp_clic_direct_unregister(unsigned int virq, void *owner);
bool esp_clic_handle_direct(unsigned long slot);

#endif /* _LINUX_IRQCHIP_IRQ_ESP32S31_CLIC_H */
