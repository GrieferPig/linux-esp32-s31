/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _IRQ_ESP32S31_INTERNAL_H
#define _IRQ_ESP32S31_INTERNAL_H

#include <linux/types.h>

struct irq_desc;

#define ESP32S31_CLIC_EXT_FIRST	16U
#define ESP32S31_CLIC_EXT_LAST	47U
#define ESP32S31_CLIC_IPI_SLOT	40U
#define ESP32S31_CLIC_TIMER_SLOT	41U

int esp_clic_install_local(unsigned int slot,
			   void (*handler)(struct irq_desc *), void *data);
void esp_clic_configure_local(unsigned int cpu, unsigned int slot,
			      bool enable);

void esp_intmtx_route_local(unsigned int cpu, unsigned int source,
			    unsigned int slot);
void esp_intmtx_unroute_local(unsigned int cpu, unsigned int source);

int __init esp32s31_smp_irq_init(void);
void esp32s31_irq_poll(void);
void esp32s31_ipi_poll(void);
u32 esp_clic_pending_mask(unsigned int cpu);
void esp_clic_handle_pending(u32 pending);

#endif
