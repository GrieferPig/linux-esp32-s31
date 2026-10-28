/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ESP32S31_RADIO_INTERNAL_H
#define _ESP32S31_RADIO_INTERNAL_H

#include <linux/types.h>

/* Integer-only IDF import services implemented by the kernel adapter. */
void s31_linux_task_yield(void);
void s31_linux_task_set_priority(void *task, u32 priority, u32 boost);
void s31_radio_report_shutdown(int result);
void s31_radio_wifi_rx_throttle(void);
void wifi_module_enable(void);
void wifi_module_disable(void);
int esp_deep_sleep_register_phy_hook(void *callback);
void touch_hal_prepare_deep_sleep(void);
int temperature_sensor_hal_init(void *hal, void *cfg);
float temperature_sensor_hal_get_degree(void *hal, int *degree);
void temperature_sensor_hal_i2c_saradc_reg_backup(void);
void temperature_sensor_hal_i2c_saradc_reg_restore(void);

/* Exact native stack extents; physical free follows the kernel lease join. */
void *s31_radio_sram_stack_alloc(u32 size);
void s31_radio_sram_stack_free(void *base, u32 size);

enum s31_tx_credit_resource {
	S31_TX_CREDIT_LOCK,
	S31_TX_CREDIT_FLIGHT,
	S31_TX_CREDIT_BUFFER,
	S31_TX_CREDIT_COUNT,
};
void s31_radio_tx_credit_return(enum s31_tx_credit_resource resource);
unsigned long s31_radio_tx_pool_bind(void *counter);
unsigned long s31_radio_softmac_tx_copy(void *opaque);

/* Persistent native task/service FP and errno ownership. The only borrowed
 * descriptor is the exclusive pre-task metadata bootstrap. Installed state
 * does not select or pivot stacks: Linux owns native task and IRQ stacks. */
struct s31_blob_context {
	unsigned long irq_flags;
	/* Full interrupted FP file (f0-f31, fcsr) for Wi-Fi payload objects.
	 * Layout matches the direct-ISR bank below: ft0-ft7, fs0-fs1,
	 * fa0-fa7, fs2-fs11, ft8-ft11, fcsr. Callee-saved-only snapshots
	 * cannot survive preemption at an arbitrary instruction. */
	u32 fp_saved[33];
	bool fp_valid;
	bool stack_switched;
	bool irq_disabled;
	bool preempt_fenced;
	int *errno_bank;
	bool errno_active;
	bool errno_native;
};

struct esp32s31_radio_wifi_ap;
struct device;
struct task_struct;
struct esp32s31_radio_domain;
struct kthread_work;

bool s31_radio_wifi_present(void);
void s31_linux_runtime_stop_publish(void);
int s31_linux_wait_cancel_allow(bool allow);

#define S31_RADIO_FEATURE_WIFI		(1U << 0)
#define S31_RADIO_FEATURE_BLUETOOTH	(1U << 1)

int s31_radio_runtime_init(bool enable_wifi, bool enable_bt);
int s31_radio_runtime_shutdown(void);
void s31_radio_managed_task_failed(void);
int s31_radio_runtime_wait_ready(void);
int s31_radio_fw_reset(void);
size_t s31_radio_fw_wifi_iram_size(void);
bool s31_radio_fw_errno_slot_valid(const int *slot);
int s31_radio_wifi_frontend_suspend(void);
int s31_radio_wifi_frontend_resume(void);
int s31_radio_btdm_frontend_suspend(void);
int s31_radio_btdm_frontend_resume(void);
int s31_radio_fw_load(struct device *device);
void s31_radio_fw_unload(void);
u32 *s31_radio_fw_isr_depth(void);
void s31_radio_wifi_clock_enable(void);
void s31_radio_wifi_clock_disable(void);
int s31_radio_wifi_frontend_init(struct device *parent);
void s31_radio_wifi_frontend_exit(void);
int s31_radio_btdm_frontend_init(struct device *parent, bool direct_hci);
void s31_radio_btdm_frontend_exit(void);

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
#define S31_SYNC_WAIT_TX_REPAIR (1U << 31)
bool s31_radio_softmac_tx_repair_begin(void);
bool s31_radio_softmac_tx_repair_cancelled(void);
bool s31_radio_softmac_tx_repair_end(void);

/* Private link boundary between the Linux core and localized IDF payload. */
extern void s31_radio_stack_task(void *arg);
extern void s31_radio_bt_enable_task(void *arg);
extern void s31_radio_bt_disable_task(void *arg);
extern void s31_radio_shutdown_task(void *arg);
extern int s31_radio_vhci_try_send(u8 *frame, u16 length);
extern int s31_radio_coex_status(u8 type, u8 op, u8 status);
extern void s31_radio_wifi_scan_task(void *arg);
extern void s31_radio_wifi_connect_task(void *arg);
extern void s31_radio_wifi_disconnect_task(void *arg);
extern void s31_radio_wifi_control_task(void *arg);
extern int s31_radio_wifi_try_send_interface(u8 interface, u8 *frame, u16 length);
void s31_radio_wifi_control_complete(int result);
void s31_radio_wifi_receive_aux(u8 interface, const u8 *frame, u16 length,
			      u8 channel, s8 signal);
extern int s31_radio_wifi_read_mac(u8 *mac);
extern int s31_radio_wifi_try_send(u8 *frame, u16 length);
extern void s31_rtos_init(void);
extern void s31_rtos_tick(void);
extern u64 s31_linux_timer_next_deadline_ns(void);
extern void s31_rtos_free(void *ptr);
extern void s31_rtos_task_release(void *cookie);
int s31_linux_native_contract_ready(void);
int s31_linux_dispatch_validate(void);
bool s31_radio_managed_preempt_ready(void);
void *s31_linux_dispatch_sync_create(void);
void s31_linux_dispatch_sync_destroy(void *sync);
u32 s31_linux_dispatch_wait(void *sync, u32 sequence, u64 deadline_ns,
                             void (*before_wait)(void));
int s31_linux_tasks_runtime_open(void);
void s31_linux_tasks_runtime_close(void);
int s31_linux_domain_irq_get(struct esp32s31_radio_domain **lease);
void s31_linux_domain_irq_put(struct esp32s31_radio_domain *lease);
void s31_linux_tasks_stop_all(void);
#define s31_rtos_isr_depth (*s31_radio_fw_isr_depth())

void *s31_linux_task_create_finalized(void (*entry)(void *), const char *name,
    u32 stack_size, void *stack_base, void *arg, u32 priority,
    void *cookie, s32 core_id, void (*finalize)(void *), void **ready_slot);
void s31_linux_task_finalize_report(s32 status);

void s31_linux_task_exit_current(void);
int s31_linux_task_stop(void *task);
/* Caller owns a task_struct reference; ordinary sleepable Linux context. */
int s31_linux_task_stop_thread(struct task_struct *thread);
void *s31_linux_current_cookie(void);
void s31_linux_task_delay(u32 ticks);
u32 s31_linux_tick_count(void);
uint64_t s31_linux_time_ns(void);
/* Actual Linux hard/soft IRQ context; creation admission only. */
int32_t s31_linux_in_interrupt(void);
int32_t s31_linux_current_cpu(void);
void s31_linux_vprintf(const char *fmt, void *arguments);
u64 s31_linux_wall_time_seconds(void);
void *s31_linux_sync_create(void);
unsigned long s31_linux_sync_owner(void *sync, u32 op);
int s31_linux_radio_service_setup(struct task_struct *thread);
/* Optional threaded-IRQ executor: cold-admit its dormant creator-owned CPU0
 * thread to the radio scheduler role when strict scheduling is selected.
 * A no-op returning 0 when the selector is off. */
int s31_linux_radio_irq_executor_admit(struct task_struct *thread);
struct s31_blob_context *s31_linux_radio_service_context(void);
int s31_linux_radio_service_admit_current(void);
int s31_linux_radio_service_retire_current(void);
void s31_linux_sync_owners_retire_all(void);
void s31_linux_sync_destroy(void *sync);
/* Per-thread foreign TLS teardown (see s31_rtos.h): the payload registers
 * its releaser once at init; the kernel invokes it on known foreign-thread
 * exits with the exiting thread's token. The unsigned long token couples
 * with the payload's uintptr_t; both are one machine word. */
int s31_foreign_owner_prepare_control(void);
int s31_foreign_owner_seal_control_current(void);
bool s31_linux_control_bootstrap_active(void);
bool s31_radio_control_bootstrap_quiescent(void);
int s31_linux_control_bootstrap_enter(struct s31_blob_context *context);
void s31_linux_control_bootstrap_leave(struct s31_blob_context *context);
void s31_foreign_owners_close(void);
int s31_foreign_owner_finish_current(void);
void s31_linux_sync_lock(void *sync);
void s31_linux_sync_unlock(void *sync);
u32 s31_linux_sync_sequence(void *sync);
int s31_linux_sync_wait(void *sync, u32 sequence, u32 timeout, u32 reason);
void s31_linux_sync_wake(void *sync);
u32 s31_linux_critical_enter(void);
void s31_linux_critical_exit(u32 flags);
void s31_linux_critical_suspend(void);
void s31_linux_critical_resume(void);
/* Blob enter/leave take the caller's window: NULL selects the current
 * compatibility task's window (fail-stop when there is no current task);
 * foreign callers pass their own per-invocation storage (fail-stop NULL).
 * Suspend returns the installed window (popped, so a later entrant on the
 * same CPU cannot reuse it); resume explicitly reinstalls the descriptor
 * captured at suspend (NULL again selects the current task's window). */
void s31_linux_blob_enter(struct s31_blob_context *ctx);
void s31_linux_blob_leave(struct s31_blob_context *ctx);
struct s31_blob_context *s31_linux_blob_suspend(u32 reason);
void s31_linux_blob_resume(struct s31_blob_context *ctx);

const char *s31_linux_blob_holder(void);
bool s31_linux_blob_held_by_current(void);
enum s31_direct_isr_result {
	S31_DIRECT_ISR_HANDLED = 1,
	S31_DIRECT_ISR_DEFER_CONTEXT,
	S31_DIRECT_ISR_DEFER_OWNER,
	S31_DIRECT_ISR_DEFER_UNSAFE,
	S31_DIRECT_ISR_DEFER_NESTED,
};

bool s31_linux_closed_isr_context_safe(void);
bool s31_linux_closed_isr_call(void (*handler)(void *), void *arg);
bool s31_linux_task_is_finalizing(void);
int s31_linux_blob_run_direct_isr(void (*handler)(void *), void *arg);
bool s31_radio_payload_uses_fp(void);
bool s31_radio_is_bt_only(void);
void s31_radio_timer_changed(void);
bool s31_radio_blob_run_pending_isrs(void);
void s31_linux_task_dump_all(void);
void s31_blob_fp_context_switch(struct task_struct *prev,
				struct task_struct *next);
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
void s31_radio_wifi_mgmt(const u8 *bssid, u8 subtype, const u8 *body,
			u16 length, u8 channel, s8 signal);
void s31_radio_wifi_connected(const u8 *bssid, u8 channel, int status);
void s31_radio_wifi_disconnected(u16 reason);
int s31_radio_wifi_receive(u8 *frame, u16 length);
int s31_radio_wifi_receive_zerocopy(u8 *frame, void *eb, u16 length);

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

void s31_radio_wifi_ap_station(const u8 *mac, bool joined);


void s31_linux_pmu_reclaim_bridge(void);
long s31_linux_crypto_operation(unsigned int operation, unsigned int domain);

/* Native pressure wait on an already registered Linux radio stack. */
bool s31_linux_native_rx_pressure_wait(void (*entry)(void *), void *arg);

#endif /* _ESP32S31_RADIO_INTERNAL_H */
