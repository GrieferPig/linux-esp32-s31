/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ASM_RISCV_ESP32S31_RADIO_TASK_H
#define _ASM_RISCV_ESP32S31_RADIO_TASK_H
#include <linux/types.h>
#include <linux/stddef.h>
#include <linux/errno.h>
#include <linux/err.h>
struct task_struct;
struct esp32s31_radio_domain;
#define ESP32S31_RADIO_DOMAIN_FOREIGN 1U
#define ESP32S31_RADIO_DOMAIN_IRQ 2U
#if defined(CONFIG_ESP32S31_COPROC_CONTEXT) && defined(CONFIG_FPU)
/* The caller owns the root lifetime and joins all producers before destroy.
 * Close is monotonic and rejects new resource/task admissions. Destroy refuses
 * live references; it does not wait while a callback needs the caller's locks.
 * A task reference lasts through generic exit-work completion, not FP detach.
 * No module reference is held that would prevent module_exit from joining. */
struct esp32s31_radio_domain *esp32s31_radio_domain_create(bool persistent_fp);
void esp32s31_radio_domain_close(struct esp32s31_radio_domain *domain);
int esp32s31_radio_domain_destroy(struct esp32s31_radio_domain *domain);
int esp32s31_radio_domain_join_tasks(struct esp32s31_radio_domain *domain);
int esp32s31_radio_domain_get(struct esp32s31_radio_domain *domain, unsigned int kind);
int esp32s31_radio_domain_put(struct esp32s31_radio_domain *domain, unsigned int kind);
int esp32s31_radio_domain_task_attach(struct task_struct *task,
                                    struct esp32s31_radio_domain *domain, void *data);
/* Cold owner-side admission after a foreign bootstrap has fully left. Only
 * a nonmovable CPU0 kthread on its Linux stack, borrowing init_mm, with no
 * active FP/foreign borrower may join. Errno must already be bound; the caller
 * supplies this owner's saved value. Retire exactly as a dormant admission. */
int esp32s31_radio_domain_current_attach(struct esp32s31_radio_domain *domain,
                                        void *data, int initial_errno);
int esp32s31_radio_domain_task_retire(struct esp32s31_radio_domain *domain);
/* Bind before the first task admission. The caller retains the writable
 * linear kernel allocation through domain destruction; per-task banks start
 * at zero. IRQ and foreign-window nesting keep their explicit save protocol. */
int esp32s31_radio_domain_bind_errno(struct esp32s31_radio_domain *domain, int *slot);
bool esp32s31_radio_task_errno_native(const void *owner);
/* Admit a dormant, kernel-created task before its first wake. The driver
 * owns data until owner-side detach; scheduler hooks never dereference it.
 * owner is a non-NULL identity token, kept alive through detach by the
 * caller's existing lifetime/join contract. This token does not itself take
 * a module reference. persistent_fp is fixed for this task's lifetime.
 * No inherited admission; querying another owner's task returns NULL. */
int esp32s31_radio_task_attach(struct task_struct *task, const void *owner, void *data,
                              bool persistent_fp);
void *esp32s31_radio_task_current_data(const void *owner);
#else
static inline struct esp32s31_radio_domain *esp32s31_radio_domain_create(bool persistent_fp)
{ return ERR_PTR(-EOPNOTSUPP); }
static inline void esp32s31_radio_domain_close(struct esp32s31_radio_domain *domain) { }
static inline int esp32s31_radio_domain_destroy(struct esp32s31_radio_domain *domain)
{ return -EOPNOTSUPP; }
static inline int esp32s31_radio_domain_join_tasks(struct esp32s31_radio_domain *domain)
{ return -EOPNOTSUPP; }
static inline int esp32s31_radio_domain_get(struct esp32s31_radio_domain *domain, unsigned int kind)
{ return -EOPNOTSUPP; }
static inline int esp32s31_radio_domain_put(struct esp32s31_radio_domain *domain, unsigned int kind)
{ return -EOPNOTSUPP; }
static inline int esp32s31_radio_domain_task_attach(struct task_struct *task,
                                                struct esp32s31_radio_domain *domain, void *data)
{ return -EOPNOTSUPP; }
static inline int esp32s31_radio_domain_current_attach(struct esp32s31_radio_domain *domain,
                                                      void *data, int initial_errno)
{ return -EOPNOTSUPP; }
static inline int esp32s31_radio_domain_task_retire(struct esp32s31_radio_domain *domain)
{ return -EOPNOTSUPP; }
static inline int esp32s31_radio_domain_bind_errno(struct esp32s31_radio_domain *domain, int *slot)
{ return -EOPNOTSUPP; }
static inline bool esp32s31_radio_task_errno_native(const void *owner) { return false; }
static inline int esp32s31_radio_task_attach(struct task_struct *task, const void *owner, void *data,
                                           bool persistent_fp)
{ return -EOPNOTSUPP; }
static inline void *esp32s31_radio_task_current_data(const void *owner) { return NULL; }
#endif
/* Owner-side detach uses esp32s31_blob_fp_detach(), after the final payload
 * and bridge cleanup, before freeing data or dropping its SRAM mapping. */
#endif
