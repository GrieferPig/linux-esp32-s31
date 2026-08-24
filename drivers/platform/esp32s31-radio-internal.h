/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ESP32S31_RADIO_INTERNAL_H
#define _ESP32S31_RADIO_INTERNAL_H

#include <linux/types.h>

struct esp32s31_radio_wifi_ap;

/* Keep these values synchronized with radio_firmware/s31_rtos/s31_rtos.h. */
enum s31_blob_release_reason {
	S31_BLOB_RELEASE_LEAVE = 0,
	S31_BLOB_RELEASE_TASK_DELAY,
	S31_BLOB_RELEASE_TASK_YIELD,
	S31_BLOB_RELEASE_QUEUE_SEND,
	S31_BLOB_RELEASE_QUEUE_RECEIVE,
	S31_BLOB_RELEASE_SEMAPHORE_TAKE,
	S31_BLOB_RELEASE_NOTIFY_TAKE,
	S31_BLOB_RELEASE_NOTIFY_WAIT,
	S31_BLOB_RELEASE_EVENT_WAIT,
	S31_BLOB_RELEASE_TASK_SUSPEND,
	S31_BLOB_RELEASE_COUNT,
};

/* Private link boundary between the Linux core and localized IDF payload. */
extern void s31_radio_stack_task(void *arg);
extern void s31_radio_bt_enable_task(void *arg);
extern void s31_radio_bt_disable_task(void *arg);
extern int s31_radio_vhci_try_send(u8 *frame, u16 length);
extern void s31_radio_wifi_scan_task(void *arg);
extern void s31_radio_wifi_connect_task(void *arg);
extern void s31_radio_wifi_disconnect_task(void *arg);
extern int s31_radio_wifi_read_mac(u8 *mac);
extern int s31_radio_wifi_try_send(u8 *frame, u16 length);
extern void s31_radio_wifi_guard_pp_state(void);
extern void s31_rtos_init(void);
extern void s31_rtos_tick(void);
extern void s31_rtos_hard_tick(void);
extern void s31_rtos_free(void *ptr);
extern void s31_rtos_task_release(void *cookie);
extern u32 s31_rtos_isr_depth;
void *s31_linux_task_create(void (*entry)(void *), const char *name,
				    u32 stack_size, void *stack_base,
				    void *arg, u32 priority, void *cookie);
void *s31_radio_task_create_deferred(void (*entry)(void *), const char *name,
				     u32 stack_size, void *stack_base,
				     void *arg, u32 priority, void *cookie);
void s31_linux_task_exit_current(void);
int s31_linux_task_stop(void *task);
void *s31_linux_current_cookie(void);
void s31_linux_task_delay(u32 ticks);
u32 s31_linux_tick_count(void);
uint64_t s31_linux_time_ns(void);
void s31_linux_printf(const char *fmt, ...);
void *s31_linux_sync_create(void);
void s31_linux_sync_destroy(void *sync);
void s31_linux_sync_lock(void *sync);
void s31_linux_sync_unlock(void *sync);
u32 s31_linux_sync_sequence(void *sync);
int s31_linux_sync_wait(void *sync, u32 sequence, u32 timeout, u32 reason);
void s31_linux_sync_wake(void *sync);
u32 s31_linux_critical_enter(void);
void s31_linux_critical_exit(u32 flags);
void s31_linux_critical_suspend(void);
void s31_linux_critical_resume(void);
void s31_linux_blob_enter(void);
void s31_linux_blob_leave(void);
void s31_linux_blob_suspend(u32 reason);
void s31_linux_blob_resume(void);
void s31_linux_trace_wifi_event(u32 event);
void s31_linux_gate_timing_reset(void);
void s31_linux_gate_timing_report(const char *stage);
void s31_linux_call_on_stack(void *stack, u32 stack_size,
			     void (*entry)(void *), void *arg);
const char *s31_linux_blob_holder(void);
bool s31_linux_blob_held_by_current(void);
enum s31_direct_isr_result {
	S31_DIRECT_ISR_HANDLED = 1,
	S31_DIRECT_ISR_DEFER_CONTEXT,
	S31_DIRECT_ISR_DEFER_OWNER,
	S31_DIRECT_ISR_DEFER_UNSAFE,
	S31_DIRECT_ISR_DEFER_NESTED,
};
int s31_linux_blob_run_direct_isr(void (*handler)(void *), void *arg);
bool s31_radio_blob_run_pending_isrs(void);
u32 s31_radio_blob_irqs_mask(void);
void s31_radio_blob_irqs_restore(u32 mask);
void s31_linux_task_dump_all(void);
void s31_radio_diag_long_gate_release(u32 reason, u64 wall_ns, u64 exec_ns,
				     u32 tick_start);
extern void (*s31_blob_gate_wait_hook)(void);
extern int xTaskCreatePinnedToCore(void (*task)(void *), const char *name,
				   u32 stack_size, void *arg, u32 priority,
				   void *task_handle, int core_id);

void s31_radio_report_wifi_init(int result);
void s31_radio_report_bt_init(int result);
void s31_radio_report_bt_enable(int result);
void s31_radio_report_bt_disable(int result);
void *s31_radio_sram_alloc(size_t size);
void s31_radio_sram_free(void *ptr);
void s31_radio_vhci_send_available(void);
int s31_radio_vhci_receive(u8 *frame, u16 length);
void s31_radio_wifi_intr_configure(u32 source, u32 logical_intr, u32 priority);
void s31_radio_wifi_intr_set_isr(u32 logical_intr, void (*handler)(void *),
				 void *arg);
void s31_radio_wifi_intr_mask(u32 mask, bool enable);
void s31_radio_wifi_scan_complete(const struct esp32s31_radio_wifi_ap *aps,
				  u16 count, int status);
void s31_radio_wifi_connected(const u8 *bssid, u8 channel, int status);
void s31_radio_wifi_disconnected(u16 reason);
int s31_radio_wifi_receive(u8 *frame, u16 length);
int s31_radio_wifi_receive_zerocopy(u8 *frame, void *eb, u16 length);
void s31_radio_timing_blob_enter(void);
void s31_radio_timing_reset(void);
u32 s31_radio_timing_tx_begin(u64 enqueue_ns, u64 start_ns,
			      const u8 *frame, u16 length);
void s31_radio_timing_tx_return(u32 sequence, u64 end_ns, int result);
void s31_radio_timing_tx_done(bool status, const u8 *frame, u16 length);
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
