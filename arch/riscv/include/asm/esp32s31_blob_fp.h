/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ASM_RISCV_ESP32S31_BLOB_FP_H
#define _ASM_RISCV_ESP32S31_BLOB_FP_H
#include <linux/types.h>
struct task_struct;
#if defined(CONFIG_ESP32S31_COPROC_CONTEXT) && defined(CONFIG_FPU)
int esp32s31_blob_fp_process_begin(void);
int esp32s31_blob_fp_process_end(void);
bool esp32s31_blob_fp_user_active(struct task_struct *);
/* Globally mapped metadata only; safe at a scheduler switch after switch_mm. */
bool esp32s31_blob_fp_radio_payload_active(struct task_struct *);
void esp32s31_blob_fp_user_return_check(void);
int esp32s31_blob_fp_attach(struct task_struct *);
int esp32s31_blob_fp_detach(struct task_struct *);
void esp32s31_blob_fp_pre_switch(struct task_struct *);
void esp32s31_blob_fp_post_switch(struct task_struct *);
void esp32s31_blob_fp_enter(void);
void esp32s31_blob_fp_pause(void);
void esp32s31_blob_fp_kernel_begin(void);
void esp32s31_blob_fp_kernel_end_check(void);
void esp32s31_blob_fp_kernel_end(void);
void esp32s31_blob_fp_foreign_enter(void);
void esp32s31_blob_fp_foreign_leave(void);
#else
static inline int esp32s31_blob_fp_process_begin(void) { return -95; }
static inline int esp32s31_blob_fp_process_end(void) { return -95; }
static inline bool esp32s31_blob_fp_user_active(struct task_struct *t) { return false; }
static inline bool esp32s31_blob_fp_radio_payload_active(struct task_struct *t) { return false; }
static inline void esp32s31_blob_fp_user_return_check(void) { }
static inline int esp32s31_blob_fp_attach(struct task_struct *t) { return 0; }
static inline int esp32s31_blob_fp_detach(struct task_struct *t) { return 0; }
static inline void esp32s31_blob_fp_pre_switch(struct task_struct *t) { }
static inline void esp32s31_blob_fp_post_switch(struct task_struct *t) { }
static inline void esp32s31_blob_fp_enter(void) { }
static inline void esp32s31_blob_fp_pause(void) { }
static inline void esp32s31_blob_fp_kernel_begin(void) { }
static inline void esp32s31_blob_fp_kernel_end_check(void) { }
static inline void esp32s31_blob_fp_kernel_end(void) { }
static inline void esp32s31_blob_fp_foreign_enter(void) { }
static inline void esp32s31_blob_fp_foreign_leave(void) { }
#endif
#endif
