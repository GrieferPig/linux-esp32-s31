/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Kernel-build include shim for the verbatim ESP-IDF register struct headers
 * under include/video/esp32p4-dsi/.  Those headers are kept byte-identical to
 * ESP-IDF and include <stdint.h>; the kernel is compiled with -nostdinc, and
 * this directory is on the kbuild include path.
 */
#ifndef _ESP32P4_DSI_STDINT_H
#define _ESP32P4_DSI_STDINT_H

#include <linux/types.h>

typedef u8 uint8_t;
typedef u16 uint16_t;
typedef u32 uint32_t;
typedef s8 int8_t;
typedef s16 int16_t;
typedef s32 int32_t;

#endif /* _ESP32P4_DSI_STDINT_H */
