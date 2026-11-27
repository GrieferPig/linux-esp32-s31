/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_ESP32S31_CORDIC_H
#define _UAPI_LINUX_ESP32S31_CORDIC_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define ESP32S31_CORDIC_COS	0
#define ESP32S31_CORDIC_SIN	1
#define ESP32S31_CORDIC_ATAN2	2
#define ESP32S31_CORDIC_COSH	5
#define ESP32S31_CORDIC_SINH	6
#define ESP32S31_CORDIC_SQRT	9

/*
 * Signed Q1.31 inputs and result. SIN/COS take x as the angle divided
 * by pi. ATAN2 takes x and y and returns atan2(y, x) divided by pi.
 * SINH/COSH take half the argument and return half the result, with x
 * in [-0.559, 0.559]. SQRT takes x in [0.1069, 1). Set y to zero for
 * every function except ATAN2.
 */
struct esp32s31_cordic_op {
	__u32 function;
	__s32 x;
	__s32 y;
	__s32 result;
};

#define ESP32S31_CORDIC_MAX_BATCH	128

/* ops points to an array of count operations, updated in place on success. */
struct esp32s31_cordic_batch {
	__aligned_u64 ops;
	__u32 count;
	__u32 reserved;
};

#define ESP32S31_CORDIC_IOC_CALC	_IOWR(0xd2, 0, struct esp32s31_cordic_op)
#define ESP32S31_CORDIC_IOC_BATCH	_IOW(0xd2, 1, struct esp32s31_cordic_batch)

#endif
