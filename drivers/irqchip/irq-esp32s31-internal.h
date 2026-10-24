/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _IRQ_ESP32S31_INTERNAL_H
#define _IRQ_ESP32S31_INTERNAL_H

#include <linux/types.h>

struct irq_desc;
struct irq_data;

#define ESP32S31_CLIC_EXT_FIRST	16U
#define ESP32S31_CLIC_EXT_LAST	47U
#define ESP32S31_CLIC_IPI_SLOT	40U
#define ESP32S31_CLIC_TIMER_SLOT	41U

int esp_clic_move_prepare(struct irq_data *d, unsigned int old_cpu, unsigned int new_cpu);
void esp_clic_move_finish(struct irq_data *d, unsigned int old_cpu,
			 unsigned int new_cpu, bool enabled);
int esp_clic_install_local(unsigned int slot,
			   void (*handler)(struct irq_desc *), void *data);
void esp_clic_configure_local(unsigned int cpu, unsigned int slot,
			      bool enable);
void esp_intmtx_route_local(unsigned int cpu, unsigned int source,
			    unsigned int slot);
void esp_intmtx_unroute_local(unsigned int cpu, unsigned int source);
bool esp_clic_radio_isolation_active(void);
int esp_intmtx_radio_isolation_validate(void);
int esp_intmtx_radio_bind(struct irq_data *data, void *owner);

int __init esp32s31_smp_irq_init(void);
void esp32s31_irq_poll(void);
u32 esp_clic_pending_mask(unsigned int cpu);
void esp_clic_handle_pending(u32 pending);

#endif
