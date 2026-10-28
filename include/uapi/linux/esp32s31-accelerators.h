/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_ESP32S31_ACCELERATORS_H
#define _UAPI_ESP32S31_ACCELERATORS_H
#include <linux/ioctl.h>
#include <linux/types.h>

#define S31_ACCEL_ABI_VERSION 1
#define S31_CORDIC_MAX_SAMPLES 4096
#define S31_CORDIC_MAX_ITERATIONS 6
/* Function values follow hardware: cos, sin, phase, modulus, atan, cosh,
 * sinh, atanh, log, sqrt. Q15 arguments occupy the low signed 16 bits.
 * Q31 angles are fractions of pi for circular functions; hyperbolic inputs
 * and scaled functions follow the hardware fixed-point convention.
 */
struct s31_cordic_sample {
	__s32 arg1, arg2;
	__s32 result1, result2;
};
struct s31_cordic_job {
	__u32 version;
	__u32 function;
	__u32 format; /* 0: Q15, 1: Q31 */
	__u32 iterations; /* 1..S31_CORDIC_MAX_ITERATIONS */
	__u32 scale;
	__u32 count;
	__aligned_u64 samples; /* in/out array of s31_cordic_sample */
	__u32 completed; /* prefix completed, including on an operation error */
	__u32 reserved[3];
};
#define S31_CORDIC_RUN _IOWR('Q', 0x80, struct s31_cordic_job)

#define S31_ETM_AUTO_CHANNEL 0xffffffffU
/* Routes are owned by the opening file and released at close. Configure
 * endpoints through their owning drivers before enabling a route. Raw
 * endpoint routing requires CAP_SYS_RAWIO because tasks can affect other
 * peripherals. No MMIO or physical memory is exposed to userspace.
 */
struct s31_etm_route {
	__u32 version;
	__u32 channel; /* AUTO on creation; returned channel on success */
	__u32 event;
	__u32 task;
	__u32 enabled;
	__u32 event_seen; /* sticky hardware status, not an event count */
	__u32 task_seen;
	__u32 reserved;
};
#define S31_ETM_ROUTE_SET _IOWR('Q', 0x81, struct s31_etm_route)
#define S31_ETM_ROUTE_GET _IOWR('Q', 0x82, struct s31_etm_route)
#define S31_ETM_ROUTE_DEL _IOW('Q', 0x83, struct s31_etm_route)

/* Program format: ESP-IDF bsasm v1 binary (12-byte header, up to eight
 * 36-byte instructions, up to 2048 LUT bytes). Only memory loopback is
 * exposed; the driver owns the borrowed peripheral route and DMA buffers.
 */
#define S31_BS_MAX_BYTES (256 * 1024)
struct s31_bitscrambler_job {
	__u32 version;
	__u32 program_bytes;
	__aligned_u64 program;
	__aligned_u64 input;
	__aligned_u64 output;
	__u32 input_bytes;
	__u32 output_bytes; /* output capacity */
	__u32 written;
	__u32 reserved;
};
#define S31_BITSCRAMBLER_RUN _IOWR('Q', 0x84, struct s31_bitscrambler_job)
#endif
