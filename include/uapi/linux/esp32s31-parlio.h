/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_ESP32S31_PARLIO_H
#define _UAPI_LINUX_ESP32S31_PARLIO_H

#include <linux/ioctl.h>
#include <linux/types.h>

/*
 * One parallel-I/O frame. A zero buffer address disables that direction.
 * A zero data_width selects the maximum width declared for the board.
 * Both buffers have length bytes, with low data lanes in low bits.
 * length is the frame size in bytes.
 */
struct esp32s31_parlio_frame {
	__aligned_u64 tx_buf;
	__aligned_u64 rx_buf;
	__u32 length;
	__u32 data_width;
};

#define ESP32S31_PARLIO_IOC_FRAME _IOW('P', 0x71, struct esp32s31_parlio_frame)

#endif
