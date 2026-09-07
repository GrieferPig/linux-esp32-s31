/* SPDX-License-Identifier: (GPL-2.0-only OR MIT) */
#ifndef _ESP32S31_LP_PROTOCOL_H
#define _ESP32S31_LP_PROTOCOL_H

#ifdef __KERNEL__
#include <linux/types.h>
typedef __u32 s31_lp_u32;
#define S31_LP_PACKED __packed
#else
#include <stdint.h>
typedef uint32_t s31_lp_u32;
#define S31_LP_PACKED __attribute__((packed))
#endif

#define S31_LP_ABI_VERSION		2U
#define S31_LP_MESSAGE_MASK		0xffff0000U
#define S31_LP_SEQUENCE_MASK		0x0000ffffU

#define S31_LP_MSG_READY		0x53310001U
#define S31_LP_MSG_WAKE			0x53310002U
#define S31_LP_CMD_PING			0x00010000U
#define S31_LP_RSP_PONG			0x80010000U
#define S31_LP_CMD_STATUS		0x00020000U
#define S31_LP_RSP_STATUS		0x80020000U
#define S31_LP_CMD_SLEEP_PREPARE	0x00030000U
#define S31_LP_RSP_SLEEP_PREPARED	0x80030000U
#define S31_LP_CMD_SLEEP_ARM		0x00040000U
#define S31_LP_RSP_SLEEP_ARMED		0x80040000U
#define S31_LP_CMD_SLEEP_ABORT		0x00050000U
#define S31_LP_RSP_SLEEP_ABORTED	0x80050000U
#define S31_LP_CMD_SLEEP_QUERY		0x00060000U
#define S31_LP_RSP_SLEEP_STATUS		0x80060000U
#define S31_LP_CMD_SLEEP_RECLAIM	0x00070000U
#define S31_LP_RSP_SLEEP_RECLAIMED	0x80070000U
#define S31_LP_RSP_ERROR		0xffff0000U

/* Last KiB of the 32-KiB LP SRAM is outside the LP firmware link region. */
#define S31_LP_SLEEP_CONTROL_ADDR	0x2e007c00U
#define S31_LP_SLEEP_CONTROL_OFFSET	0x00007c00U
#define S31_LP_SLEEP_CONTROL_MAGIC	0x5331504dU /* "S1PM" */

#define S31_LP_CAP_HANDSHAKE		(1U << 0)
#define S31_LP_CAP_TIMER_WAKE		(1U << 1)
#define S31_LP_CAP_GPIO_WAKE		(1U << 2)
#define S31_LP_CAP_WAKE_LOG		(1U << 3)
#define S31_LP_CAP_RETENTION_DESC	(1U << 4)

#define S31_LP_WAKE_TIMER		(1U << 0)
#define S31_LP_WAKE_GPIO		(1U << 1)
#define S31_LP_WAKE_LP_UART		(1U << 2)

#define S31_LP_SLEEP_F_S2IDLE		(1U << 0)
#define S31_LP_SLEEP_F_STANDBY		(1U << 1)
#define S31_LP_SLEEP_F_MEM		(1U << 2)
#define S31_LP_SLEEP_F_DEEP_REBOOT	(1U << 3)
#define S31_LP_SLEEP_F_GPIO_PULL_UP	(1U << 4)
#define S31_LP_SLEEP_F_GPIO_PULL_DOWN	(1U << 5)
#define S31_LP_SLEEP_F_DRY_RUN		(1U << 31)

enum s31_lp_sleep_state {
	S31_LP_SLEEP_BOOT = 0,
	S31_LP_SLEEP_READY,
	S31_LP_SLEEP_PREPARED,
	S31_LP_SLEEP_ARMED,
	S31_LP_SLEEP_HP_ASLEEP,
	S31_LP_SLEEP_WAKING,
	S31_LP_SLEEP_RESUMED,
	S31_LP_SLEEP_ABORTED,
	S31_LP_SLEEP_REJECTED,
};

enum s31_lp_sleep_result {
	S31_LP_SLEEP_OK = 0,
	S31_LP_SLEEP_ERR_ABI,
	S31_LP_SLEEP_ERR_CRC,
	S31_LP_SLEEP_ERR_SEQUENCE,
	S31_LP_SLEEP_ERR_STATE,
	S31_LP_SLEEP_ERR_WAKE_MASK,
	S31_LP_SLEEP_ERR_UNSUPPORTED,
	S31_LP_SLEEP_ERR_DEADLINE,
};

/*
 * Request CRC covers every byte before request_crc.  Response CRC covers
 * every byte before response_crc, including the request and result fields.
 */
struct s31_lp_sleep_control {
	s31_lp_u32 magic;
	s31_lp_u32 version;
	s31_lp_u32 size;
	s31_lp_u32 sequence;
	s31_lp_u32 flags;
	s31_lp_u32 wake_mask;
	/* Relative one-shot timer duration in microseconds. */
	s31_lp_u32 deadline_lo;
	s31_lp_u32 deadline_hi;
	s31_lp_u32 gpio_mask_lo;
	s31_lp_u32 gpio_mask_hi;
	s31_lp_u32 gpio_level_lo;
	s31_lp_u32 gpio_level_hi;
	s31_lp_u32 retention_mask;
	s31_lp_u32 domain_mask;
	s31_lp_u32 clock_mask;
	s31_lp_u32 resume_vector;
	s31_lp_u32 resume_arg;
	s31_lp_u32 request_crc;
	s31_lp_u32 capabilities;
	s31_lp_u32 state;
	s31_lp_u32 result;
	s31_lp_u32 wake_reason;
	s31_lp_u32 wake_raw;
	s31_lp_u32 sleep_ticks_lo;
	s31_lp_u32 sleep_ticks_hi;
	s31_lp_u32 wake_ticks_lo;
	s31_lp_u32 wake_ticks_hi;
	s31_lp_u32 response_crc;
} S31_LP_PACKED;

#undef S31_LP_PACKED

#endif
