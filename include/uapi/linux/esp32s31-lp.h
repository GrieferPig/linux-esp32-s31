/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_ESP32S31_LP_H
#define _UAPI_LINUX_ESP32S31_LP_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define ESP32S31_LP_GDMA_MAX_BYTES 128

/*
 * Copy data through LP SRAM using LP AHB-GDMA. Input and output occupy
 * data[0..length-1]. reserved must be zero.
 */
struct esp32s31_lp_gdma_copy {
	__u32 length;
	__u32 reserved;
	__u8 data[ESP32S31_LP_GDMA_MAX_BYTES];
};

/* INFO returns the maximum copy length without starting the DMA controller. */
#define ESP32S31_LP_IOC_GDMA_INFO _IOR('L', 0x51, __u32)
#define ESP32S31_LP_IOC_GDMA_COPY _IOWR('L', 0x52, struct esp32s31_lp_gdma_copy)

#endif
