/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ESP32S31_RADIO_INTERNAL_H
#define _ESP32S31_RADIO_INTERNAL_H

#include <linux/types.h>

/* Private link boundary between the Linux core and localized IDF payload. */
extern void s31_radio_stack_task(void *arg);
extern void s31_radio_bt_enable_task(void *arg);
extern void s31_rtos_init(void);
extern void s31_rtos_schedule(void);
extern void s31_rtos_tick(void);
extern u32 s31_rtos_isr_depth;
extern int xTaskCreatePinnedToCore(void (*task)(void *), const char *name,
				   u32 stack_size, void *arg, u32 priority,
				   void *task_handle, int core_id);

void s31_radio_report_wifi_init(int result);
void s31_radio_report_bt_init(int result);
void s31_radio_report_bt_enable(int result);
void s31_radio_heap_report(const char *stage);

void *__wrap_heap_caps_malloc(size_t size, u32 caps);
void __wrap_heap_caps_free(void *ptr);
void *__wrap_heap_caps_calloc(size_t n, size_t size, u32 caps);
void *__wrap_heap_caps_realloc(void *ptr, size_t size, u32 caps);
void *__wrap_heap_caps_aligned_alloc(size_t alignment, size_t size, u32 caps);
void *__wrap_heap_caps_aligned_calloc(size_t alignment, size_t n,
				       size_t size, u32 caps);
void *__wrap_heap_caps_malloc_default(size_t size);
void *__wrap_heap_caps_realloc_default(void *ptr, size_t size);
void *__wrap_heap_caps_malloc_prefer(size_t size, size_t count, ...);
size_t __wrap_heap_caps_get_free_size(u32 caps);

void _interrupt_handler(void);
void *intr_handler_get(int int_no);
u32 __wrap_esp_log_timestamp(void);
bool __wrap_esp_intr_ptr_in_isr_region(void *ptr);
int __wrap_esp_intr_alloc(int source, int flags, void (*handler)(void *),
			  void *arg, void **ret_handle);
int __wrap_esp_intr_enable(void *handle);
int __wrap_esp_intr_disable(void *handle);
int __wrap_esp_intr_free(void *handle);

#endif /* _ESP32S31_RADIO_INTERNAL_H */
