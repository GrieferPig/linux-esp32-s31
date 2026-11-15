/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_IRQCHIP_ESP32P4_H
#define _LINUX_IRQCHIP_ESP32P4_H
#include <linux/kconfig.h>
#ifdef CONFIG_ESP32P4_IRQCHIP
int esp32p4_irq_cpu_starting(unsigned int cpu);
#else
static inline int esp32p4_irq_cpu_starting(unsigned int cpu) { return 0; }
#endif
#endif
