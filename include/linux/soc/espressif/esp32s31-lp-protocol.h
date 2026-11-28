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

#define S31_LP_ABI_VERSION		1U
#define S31_LP_MESSAGE_MASK		0xffff0000U
#define S31_LP_SEQUENCE_MASK		0x0000ffffU

#define S31_LP_MSG_READY		0x53310001U
#define S31_LP_MSG_WAKE			0x53310002U
#define S31_LP_MSG_UART_EVENT		0x53310003U
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
#define S31_LP_CMD_PERIPHERAL_RPC	0x00080000U
#define S31_LP_RSP_PERIPHERAL_RPC	0x80080000U
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

#define S31_LP_CAP_PERIPHERAL_RPC	(1U << 5)

/* TRM ch.5.3.2.3 LP SRAM: use the ELF-excluded tail after S1PM. */
#define S31_LP_RPC_OFFSET		0x00007d00U
#define S31_LP_RPC_ADDR			0x2e007d00U
#define S31_LP_RPC_BYTES		0x300U
#define S31_LP_RPC_MAGIC		0x53315250U /* "S1RP" */
#define S31_LP_RPC_VERSION		1U
#define S31_LP_RPC_SLOTS		4U
#define S31_LP_RPC_DATA_BYTES		128U

enum s31_lp_rpc_operation {
	S31_LP_RPC_CAPABILITIES = 1,
	S31_LP_RPC_I2C_XFER,
	S31_LP_RPC_SPI,
	S31_LP_RPC_UART,
};

enum s31_lp_rpc_result {
	S31_LP_RPC_OK,
	S31_LP_RPC_INVALID,
	S31_LP_RPC_UNSUPPORTED,
	S31_LP_RPC_BUSY,
	S31_LP_RPC_NACK,
	S31_LP_RPC_TIMEOUT,
	S31_LP_RPC_IO,
	S31_LP_RPC_ARBITRATION,
};

#define S31_LP_SERVICE_I2C		(1U << 0)
#define S31_LP_SERVICE_SPI		(1U << 1)
#define S31_LP_SERVICE_UART		(1U << 2)
#define S31_LP_GPIO_COUNT		8U
#define S31_LP_I2C_FIFO_BYTES		16U
#define S31_LP_I2C_STANDARD_HZ		100000U
#define S31_LP_I2C_FAST_HZ		400000U
#define S31_LP_I2C_PULLUP		(1U << 0)

/*
 * I2C args: [0] 7-bit address, [1] SDA | SCL << 8, [2] SCL Hz,
 * [3] flags. input_len is the write length and output_len is the read
 * length. Nonzero lengths in both fields request a repeated START.
 * Read data replaces write data in the slot.
 */

/* SPI args: pin bytes in SCLK/MOSI/MISO/CS order, speed in Hz, mode, action. */
#define S31_LP_SPI_CHUNK_BYTES		60U
#define S31_LP_SPI_MIN_HZ		50000U
#define S31_LP_SPI_MAX_HZ		10000000U
#define S31_LP_SPI_CPHA			(1U << 0)
#define S31_LP_SPI_CPOL			(1U << 1)
#define S31_LP_SPI_CS_HIGH		(1U << 2)
#define S31_LP_SPI_LSB_FIRST		(1U << 3)

enum s31_lp_spi_action {
	S31_LP_SPI_CLOCK,
	S31_LP_SPI_SELECT,
	S31_LP_SPI_RELEASE,
};

#define S31_LP_UART_MIN_BAUD		50U
#define S31_LP_UART_MAX_BAUD		115200U
#define S31_LP_UART_TX_BYTES		16U
#define S31_LP_UART_BITS_MASK		3U /* 0 to 3 represent 5 to 8 data bits */
#define S31_LP_UART_STOP_2		(1U << 2)
#define S31_LP_UART_PARITY		(1U << 3)
#define S31_LP_UART_ODD			(1U << 4)
#define S31_LP_UART_OVERRUN		(1U << 0)
#define S31_LP_UART_BAD_FRAME		(1U << 1)
#define S31_LP_UART_BAD_PARITY		(1U << 2)
#define S31_LP_UART_BREAK		(1U << 3)

/*
 * CONFIG args: action, TX | RX << 8, baud, format. Returns actual baud in
 * args[2]. STOP disables the port. Both require zero input and output.
 * EXCHANGE args: action. Writes up to input_len and returns the accepted
 * count in args[1]. args[2] reports whether TX is idle, and args[3] carries
 * RX errors. Read data replaces write data, up to the requested output_len.
 */
enum s31_lp_uart_action {
	S31_LP_UART_CONFIG,
	S31_LP_UART_STOP,
	S31_LP_UART_EXCHANGE,
};

/*
 * HP publishes a request by advancing producer before ringing the mailbox.
 * LP releases the slot by advancing consumer after writing the result.
 * Unconsumed slots remain owned by LP across HP timeout and signal paths.
 * Counters wrap modulo 2^32.
 */
struct s31_lp_rpc_slot {
	s31_lp_u32 generation;
	s31_lp_u32 sequence;
	s31_lp_u32 operation;
	s31_lp_u32 input_len;
	s31_lp_u32 output_len;
	s31_lp_u32 result;
	s31_lp_u32 args[4];
	unsigned char data[S31_LP_RPC_DATA_BYTES];
};

struct s31_lp_rpc_ring {
	s31_lp_u32 magic;
	s31_lp_u32 version;
	s31_lp_u32 producer;
	s31_lp_u32 consumer;
	struct s31_lp_rpc_slot slots[S31_LP_RPC_SLOTS];
};

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
