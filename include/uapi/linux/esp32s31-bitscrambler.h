/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_ESP32S31_BITSCRAMBLER_H
#define _UAPI_LINUX_ESP32S31_BITSCRAMBLER_H

#include <linux/ioctl.h>
#include <linux/types.h>

/*
 * Program is an ESP-IDF bsasm V1 binary including its optional LUT.
 * src_len and dst_len are in bytes, with a maximum of 4096 per buffer.
 * On completion dst_len is the actual number of bytes produced.
 */
struct esp32s31_bitscrambler_run {
	__aligned_u64 program;
	__aligned_u64 src;
	__aligned_u64 dst;
	__u32 program_len;
	__u32 src_len;
	__u32 dst_len;
	__u32 reserved;
};

#define ESP32S31_BITSCRAMBLER_IOC_RUN _IOWR('B', 0x73, struct esp32s31_bitscrambler_run)

#endif
