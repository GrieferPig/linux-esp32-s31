/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_CLOCKSOURCE_ESP32S31_SYSTIMER_H
#define _LINUX_CLOCKSOURCE_ESP32S31_SYSTIMER_H

#define ESP32S31_SYSTIMER_RATE 16000000U

int esp32s31_systimer_set_next_event(unsigned long delta);
void esp32s31_systimer_stop(void);
bool esp32s31_systimer_irq_pending(unsigned int cpu);
void esp32s31_riscv_timer_interrupt(void);
int esp32s31_systimer_irq_starting(unsigned int cpu);
int esp32s31_systimer_irq_dying(unsigned int cpu);

#endif
