// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif ESP32-S31 high-performance I2C master controller.
 *
 * The register layout and command encoding follow ESP-IDF's esp32s31
 * i2c_ll implementation.  Linux owns only HP I2C0/I2C1; LP I2C ownership is
 * deliberately left for the future LP-core/remoteproc design.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#define S31_I2C_SCL_LOW			0x00
#define S31_I2C_CTR			0x04
#define S31_I2C_SR			0x08
#define S31_I2C_TIMEOUT			0x0c
#define S31_I2C_SLAVE_ADDR		0x10
#define S31_I2C_FIFO_CONF		0x18
#define S31_I2C_DATA			0x1c
#define S31_I2C_INT_CLR			0x24
#define S31_I2C_INT_ENA			0x28
#define S31_I2C_INT_STATUS		0x2c
#define S31_I2C_SDA_HOLD		0x30
#define S31_I2C_SDA_SAMPLE		0x34
#define S31_I2C_SCL_HIGH		0x38
#define S31_I2C_START_HOLD		0x40
#define S31_I2C_RSTART_SETUP		0x44
#define S31_I2C_STOP_HOLD		0x48
#define S31_I2C_STOP_SETUP		0x4c
#define S31_I2C_FILTER_CFG		0x50
#define S31_I2C_COMMAND(n)		(0x58 + (n) * 4)
#define S31_I2C_SCL_SP_CONF		0x80
#define S31_I2C_SCL_STRETCH_CONF	0x84
#define S31_I2C_DATE			0xf8

#define S31_I2C_CTR_MASTER		BIT(4)
#define S31_I2C_CTR_TRANS_START		BIT(5)
#define S31_I2C_CTR_CLK_EN		BIT(8)
#define S31_I2C_CTR_ARB_EN		BIT(9)
#define S31_I2C_CTR_FSM_RST		BIT(10)
#define S31_I2C_CTR_CONF_UPDATE		BIT(11)
#define S31_I2C_CTR_SLV_TX_AUTO_START	BIT(12)

#define S31_I2C_SR_BUS_BUSY		BIT(4)
#define S31_I2C_SR_SLAVE_RW		BIT(1)
#define S31_I2C_SR_RXFIFO_CNT		GENMASK(13, 8)

#define S31_I2C_FIFO_RX_RST		BIT(12)
#define S31_I2C_FIFO_TX_RST		BIT(13)
#define S31_I2C_FIFO_PTR_EN		BIT(14)
#define S31_I2C_FIFO_RX_WM		GENMASK(4, 0)
#define S31_I2C_FIFO_TX_WM		GENMASK(9, 5)

#define S31_I2C_INT_END_DETECT		BIT(3)
#define S31_I2C_INT_RXFIFO_WM		BIT(0)
#define S31_I2C_INT_TXFIFO_WM		BIT(1)
#define S31_I2C_INT_RXFIFO_OVF		BIT(2)
#define S31_I2C_INT_ARB_LOST		BIT(5)
#define S31_I2C_INT_TRANS_COMPLETE	BIT(7)
#define S31_I2C_INT_TIMEOUT		BIT(8)
#define S31_I2C_INT_NACK			BIT(10)
#define S31_I2C_INT_TXFIFO_OVF		BIT(11)
#define S31_I2C_INT_RXFIFO_UDF		BIT(12)
#define S31_I2C_INT_SCL_STATE_TIMEOUT	BIT(13)
#define S31_I2C_INT_MAIN_STATE_TIMEOUT	BIT(14)
#define S31_I2C_INT_SLAVE_STRETCH	BIT(16)
#define S31_I2C_INT_ERRORS		(S31_I2C_INT_ARB_LOST | \
					 S31_I2C_INT_TIMEOUT | \
					 S31_I2C_INT_NACK | \
					 S31_I2C_INT_SCL_STATE_TIMEOUT | \
					 S31_I2C_INT_MAIN_STATE_TIMEOUT)
#define S31_I2C_INT_XFER			(S31_I2C_INT_TRANS_COMPLETE | \
					 S31_I2C_INT_ERRORS)
#define S31_I2C_INT_ALL			GENMASK(18, 0)
#define S31_I2C_INT_SLAVE		(S31_I2C_INT_RXFIFO_WM | \
					 S31_I2C_INT_TXFIFO_WM | \
					 S31_I2C_INT_RXFIFO_OVF | \
					 S31_I2C_INT_TRANS_COMPLETE | \
					 S31_I2C_INT_TXFIFO_OVF | \
					 S31_I2C_INT_RXFIFO_UDF | \
					 S31_I2C_INT_SLAVE_STRETCH)

#define S31_I2C_FILTER_SCL		GENMASK(3, 0)
#define S31_I2C_FILTER_SDA		GENMASK(7, 4)
#define S31_I2C_FILTER_SCL_EN		BIT(8)
#define S31_I2C_FILTER_SDA_EN		BIT(9)

#define S31_I2C_TIMEOUT_VALUE		GENMASK(4, 0)
#define S31_I2C_TIMEOUT_EN		BIT(5)

#define S31_I2C_SCL_HIGH_PERIOD		GENMASK(8, 0)
#define S31_I2C_SCL_WAIT_HIGH		GENMASK(15, 9)

#define S31_I2C_BUS_CLEAR_EN		BIT(0)
#define S31_I2C_BUS_CLEAR_PULSES		GENMASK(5, 1)
#define S31_I2C_SLAVE_ADDR_VALUE	GENMASK(14, 0)
#define S31_I2C_SLAVE_ADDR_10BIT	BIT(31)
#define S31_I2C_SLAVE_STRETCH_EN	BIT(10)
#define S31_I2C_SLAVE_STRETCH_CLR	BIT(11)

#define S31_I2C_CMD_LEN			GENMASK(7, 0)
#define S31_I2C_CMD_ACK_CHECK		BIT(8)
#define S31_I2C_CMD_ACK_EXPECT		BIT(9)
#define S31_I2C_CMD_ACK_VALUE		BIT(10)
#define S31_I2C_CMD_OPCODE		GENMASK(13, 11)

#define S31_I2C_CMD_WRITE		1
#define S31_I2C_CMD_STOP			2
#define S31_I2C_CMD_READ			3
#define S31_I2C_CMD_END			4
#define S31_I2C_CMD_RESTART		6

#define S31_I2C_FIFO_LEN			32
#define S31_I2C_COMMANDS			8
#define S31_I2C_XFER_TIMEOUT_MS		500

struct esp32s31_i2c {
	void __iomem *base;
	struct clk *clk;
	struct completion complete;
	struct i2c_adapter adapter;
	struct i2c_bus_recovery_info recovery;
	int irq;
	int result;
	u32 frequency;
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	struct i2c_client *slave;
	bool slave_active;
#endif
};

static inline u32 s31_i2c_read(struct esp32s31_i2c *i2c, u32 reg)
{
	return readl(i2c->base + reg);
}

static inline void s31_i2c_write(struct esp32s31_i2c *i2c, u32 reg, u32 value)
{
	writel(value, i2c->base + reg);
}

static u32 s31_i2c_command(u32 opcode, u32 length, bool ack_check,
			   bool ack_value)
{
	return FIELD_PREP(S31_I2C_CMD_OPCODE, opcode) |
	       FIELD_PREP(S31_I2C_CMD_LEN, length) |
	       (ack_check ? S31_I2C_CMD_ACK_CHECK : 0) |
	       (ack_value ? S31_I2C_CMD_ACK_VALUE : 0);
}

static int s31_i2c_add_command(u32 *commands, unsigned int *count,
			       u32 opcode, u32 length, bool ack_check,
			       bool ack_value)
{
	if (*count >= S31_I2C_COMMANDS)
		return -EOPNOTSUPP;

	commands[(*count)++] = s31_i2c_command(opcode, length, ack_check,
					       ack_value);
	return 0;
}

static int s31_i2c_put_byte(u8 *fifo, unsigned int *count, u8 value)
{
	if (*count >= S31_I2C_FIFO_LEN)
		return -EOPNOTSUPP;
	fifo[(*count)++] = value;
	return 0;
}

static int s31_i2c_build_commands(struct i2c_msg *msgs, int num,
				   u32 *commands, unsigned int *command_count,
				   u8 *tx_fifo, unsigned int *tx_count,
				   unsigned int *rx_count)
{
	int i, ret;

	*command_count = 0;
	*tx_count = 0;
	*rx_count = 0;

	for (i = 0; i < num; i++) {
		struct i2c_msg *msg = &msgs[i];
		bool read = msg->flags & I2C_M_RD;
		bool ten = msg->flags & I2C_M_TEN;
		bool ten_read_after_write = i && read && ten &&
			!(msgs[i - 1].flags & I2C_M_RD) &&
			(msgs[i - 1].flags & I2C_M_TEN) &&
			msgs[i - 1].addr == msg->addr;
		unsigned int write_start;
		u8 header;
		int j;

		if (msg->flags & ~(I2C_M_RD | I2C_M_TEN | I2C_M_DMA_SAFE))
			return -EOPNOTSUPP;

		/*
		 * A standalone 10-bit read first selects A9..A0 using the
		 * write header, then repeats START with the read header.  A
		 * combined write/read to the same target already selected the
		 * low address byte in the first message.
		 */
		if (read && ten && !ten_read_after_write) {
			ret = s31_i2c_add_command(commands, command_count,
						  S31_I2C_CMD_RESTART, 0,
						  false, false);
			if (ret)
				return ret;
			write_start = *tx_count;
			header = 0xf0 | ((msg->addr >> 7) & 0x06);
			ret = s31_i2c_put_byte(tx_fifo, tx_count, header);
			if (ret)
				return ret;
			ret = s31_i2c_put_byte(tx_fifo, tx_count, msg->addr);
			if (ret)
				return ret;
			ret = s31_i2c_add_command(commands, command_count,
						  S31_I2C_CMD_WRITE,
						  *tx_count - write_start,
						  true, false);
			if (ret)
				return ret;
		}

		ret = s31_i2c_add_command(commands, command_count,
					  S31_I2C_CMD_RESTART, 0, false, false);
		if (ret)
			return ret;

		write_start = *tx_count;
		if (ten) {
			header = 0xf0 | ((msg->addr >> 7) & 0x06) |
				 (read ? 1 : 0);
			ret = s31_i2c_put_byte(tx_fifo, tx_count, header);
			if (ret)
				return ret;
			if (!read) {
				ret = s31_i2c_put_byte(tx_fifo, tx_count,
						       msg->addr);
				if (ret)
					return ret;
			}
		} else {
			ret = s31_i2c_put_byte(tx_fifo, tx_count,
					       (msg->addr << 1) | read);
			if (ret)
				return ret;
		}

		if (!read)
			for (j = 0; j < msg->len; j++) {
				ret = s31_i2c_put_byte(tx_fifo, tx_count,
						       msg->buf[j]);
				if (ret)
					return ret;
			}

		ret = s31_i2c_add_command(commands, command_count,
					  S31_I2C_CMD_WRITE,
					  *tx_count - write_start, true, false);
		if (ret)
			return ret;

		if (read) {
			if (msg->len > 1) {
				ret = s31_i2c_add_command(commands, command_count,
							  S31_I2C_CMD_READ,
							  msg->len - 1,
							  false, false);
				if (ret)
					return ret;
			}
			if (msg->len) {
				ret = s31_i2c_add_command(commands, command_count,
							  S31_I2C_CMD_READ, 1,
							  false, true);
				if (ret)
					return ret;
			}
			*rx_count += msg->len;
			if (*rx_count > S31_I2C_FIFO_LEN)
				return -EOPNOTSUPP;
		}
	}

	return s31_i2c_add_command(commands, command_count,
				   S31_I2C_CMD_STOP, 0, false, false);
}

static void s31_i2c_reset_fifos(struct esp32s31_i2c *i2c)
{
	u32 value = s31_i2c_read(i2c, S31_I2C_FIFO_CONF);

	value &= ~BIT(10);
	value |= S31_I2C_FIFO_PTR_EN |
		 S31_I2C_FIFO_RX_RST | S31_I2C_FIFO_TX_RST;
	s31_i2c_write(i2c, S31_I2C_FIFO_CONF, value);
	value &= ~(S31_I2C_FIFO_RX_RST | S31_I2C_FIFO_TX_RST);
	s31_i2c_write(i2c, S31_I2C_FIFO_CONF, value);
}

static void s31_i2c_reset_fsm(struct esp32s31_i2c *i2c)
{
	u32 ctr = s31_i2c_read(i2c, S31_I2C_CTR);

	s31_i2c_write(i2c, S31_I2C_CTR, ctr | S31_I2C_CTR_FSM_RST);
	s31_i2c_write(i2c, S31_I2C_CTR, ctr | S31_I2C_CTR_CONF_UPDATE);
}

static int s31_i2c_recover_bus(struct i2c_adapter *adapter)
{
	struct esp32s31_i2c *i2c = i2c_get_adapdata(adapter);
	u32 value;
	int ret;

	s31_i2c_write(i2c, S31_I2C_INT_ENA, 0);
	s31_i2c_write(i2c, S31_I2C_INT_CLR, S31_I2C_INT_ALL);
	s31_i2c_reset_fsm(i2c);

	value = FIELD_PREP(S31_I2C_BUS_CLEAR_PULSES, 9) |
		S31_I2C_BUS_CLEAR_EN;
	s31_i2c_write(i2c, S31_I2C_SCL_SP_CONF, value);
	s31_i2c_write(i2c, S31_I2C_CTR,
		      s31_i2c_read(i2c, S31_I2C_CTR) |
		      S31_I2C_CTR_CONF_UPDATE);
	ret = readl_poll_timeout(i2c->base + S31_I2C_SCL_SP_CONF, value,
				 !(value & S31_I2C_BUS_CLEAR_EN), 10, 20000);
	s31_i2c_reset_fsm(i2c);

	return ret;
}

#if IS_ENABLED(CONFIG_I2C_SLAVE)
static void s31_i2c_slave_tx_byte(struct esp32s31_i2c *i2c)
{
	u8 value = 0xff;

	i2c_slave_event(i2c->slave,
			i2c->slave_active ? I2C_SLAVE_READ_PROCESSED :
			I2C_SLAVE_READ_REQUESTED, &value);
	i2c->slave_active = true;
	s31_i2c_write(i2c, S31_I2C_DATA, value);
}

static irqreturn_t s31_i2c_slave_irq(struct esp32s31_i2c *i2c, u32 status)
{
	u32 sr = s31_i2c_read(i2c, S31_I2C_SR);
	bool read = sr & S31_I2C_SR_SLAVE_RW;
	u32 count;
	u8 value = 0;

	if (!i2c->slave)
		return IRQ_NONE;

	if (!read) {
		count = FIELD_GET(S31_I2C_SR_RXFIFO_CNT, sr);
		if (count && !i2c->slave_active) {
			i2c_slave_event(i2c->slave, I2C_SLAVE_WRITE_REQUESTED,
					&value);
			i2c->slave_active = true;
		}
		while (count--) {
			value = s31_i2c_read(i2c, S31_I2C_DATA);
			i2c_slave_event(i2c->slave, I2C_SLAVE_WRITE_RECEIVED,
					&value);
		}
	} else if (status & (S31_I2C_INT_TXFIFO_WM |
			     S31_I2C_INT_SLAVE_STRETCH)) {
		s31_i2c_slave_tx_byte(i2c);
	}

	if (status & S31_I2C_INT_TRANS_COMPLETE) {
		if (i2c->slave_active)
			i2c_slave_event(i2c->slave, I2C_SLAVE_STOP, &value);
		i2c->slave_active = false;
	}

	s31_i2c_write(i2c, S31_I2C_INT_CLR, status);
	if (status & S31_I2C_INT_SLAVE_STRETCH)
		s31_i2c_write(i2c, S31_I2C_SCL_STRETCH_CONF,
			      S31_I2C_SLAVE_STRETCH_EN |
			      S31_I2C_SLAVE_STRETCH_CLR);
	return IRQ_HANDLED;
}
#endif

static irqreturn_t s31_i2c_irq(int irq, void *data)
{
	struct esp32s31_i2c *i2c = data;
	u32 status = s31_i2c_read(i2c, S31_I2C_INT_STATUS);

	if (!status)
		return IRQ_NONE;

#if IS_ENABLED(CONFIG_I2C_SLAVE)
	if (i2c->slave)
		return s31_i2c_slave_irq(i2c, status);
#endif

	s31_i2c_write(i2c, S31_I2C_INT_ENA, 0);
	s31_i2c_write(i2c, S31_I2C_INT_CLR, status);

	if (status & S31_I2C_INT_ARB_LOST)
		i2c->result = -EAGAIN;
	else if (status & S31_I2C_INT_NACK)
		i2c->result = -ENXIO;
	else if (status & (S31_I2C_INT_TIMEOUT |
			   S31_I2C_INT_SCL_STATE_TIMEOUT |
			   S31_I2C_INT_MAIN_STATE_TIMEOUT))
		i2c->result = -ETIMEDOUT;
	else if (status & (S31_I2C_INT_TRANS_COMPLETE | S31_I2C_INT_END_DETECT))
		i2c->result = 0;
	else
		return IRQ_NONE;

	complete(&i2c->complete);
	return IRQ_HANDLED;
}

/* END pauses the command engine with the bus owned; unlike STOP it does
 * not terminate the peripheral transaction.  Each batch fits the FIFOs,
 * so IRQ latency cannot cause an RX overflow or TX underrun. */
static int s31_i2c_batch(struct esp32s31_i2c *i2c, const u32 *commands,
		       unsigned int ncommands, const u8 *tx, unsigned int ntx,
		       u8 *rx, unsigned int nrx)
{
	unsigned long completed;
	unsigned int i;

	s31_i2c_write(i2c, S31_I2C_INT_ENA, 0);
	synchronize_irq(i2c->irq);
	s31_i2c_write(i2c, S31_I2C_INT_CLR, S31_I2C_INT_ALL);
	reinit_completion(&i2c->complete);
	i2c->result = -ETIMEDOUT;
	s31_i2c_reset_fifos(i2c);
	for (i = 0; i < S31_I2C_COMMANDS; i++)
		s31_i2c_write(i2c, S31_I2C_COMMAND(i),
			      i < ncommands ? commands[i] : 0);
	for (i = 0; i < ntx; i++)
		s31_i2c_write(i2c, S31_I2C_DATA, tx[i]);
	s31_i2c_write(i2c, S31_I2C_INT_ENA,
		      S31_I2C_INT_XFER | S31_I2C_INT_END_DETECT);
	s31_i2c_write(i2c, S31_I2C_CTR,
		      s31_i2c_read(i2c, S31_I2C_CTR) | S31_I2C_CTR_CONF_UPDATE);
	s31_i2c_write(i2c, S31_I2C_CTR,
		      s31_i2c_read(i2c, S31_I2C_CTR) | S31_I2C_CTR_TRANS_START);
	completed = wait_for_completion_timeout(&i2c->complete,
				msecs_to_jiffies(S31_I2C_XFER_TIMEOUT_MS));
	s31_i2c_write(i2c, S31_I2C_INT_ENA, 0);
	synchronize_irq(i2c->irq);
	if (!completed)
		return -ETIMEDOUT;
	if (i2c->result)
		return i2c->result;
	if (FIELD_GET(S31_I2C_SR_RXFIFO_CNT,
		      s31_i2c_read(i2c, S31_I2C_SR)) != nrx)
		return -EIO;
	for (i = 0; i < nrx; i++)
		rx[i] = s31_i2c_read(i2c, S31_I2C_DATA);
	return 0;
}

static int s31_i2c_long_xfer(struct esp32s31_i2c *i2c,
			   struct i2c_msg *msgs, int num)
{
	u32 commands[S31_I2C_COMMANDS];
	u8 address[2];
	unsigned int n, len, offset;
	int i, ret;

	for (i = 0; i < num; i++) {
		struct i2c_msg *msg = &msgs[i];
		bool read = msg->flags & I2C_M_RD;
		bool ten = msg->flags & I2C_M_TEN;

		/* Address separately from payload.  A 10-bit read first selects
		 * both address bytes in write mode, then repeats the read header. */
		address[0] = ten ? 0xf0 | ((msg->addr >> 7) & 6) :
				 (msg->addr << 1) | read;
		address[1] = msg->addr;
		commands[0] = s31_i2c_command(S31_I2C_CMD_RESTART, 0, false, false);
		commands[1] = s31_i2c_command(S31_I2C_CMD_WRITE,
					    ten ? 2 : 1, true, false);
		commands[2] = s31_i2c_command(S31_I2C_CMD_END, 0, false, false);
		ret = s31_i2c_batch(i2c, commands, 3, address, ten ? 2 : 1, NULL, 0);
		if (ret)
			return ret;
		if (ten && read) {
			address[0] |= 1;
			commands[1] = s31_i2c_command(S31_I2C_CMD_WRITE, 1, true, false);
			ret = s31_i2c_batch(i2c, commands, 3, address, 1, NULL, 0);
			if (ret)
				return ret;
		}

		offset = 0;
		do {
			bool final;

			/* Leave an RX FIFO slot free until the ACK/NACK handshake
			 * and END/STOP finish; only then may the CPU drain it. */
			len = min_t(unsigned int, msg->len - offset,
				    S31_I2C_FIFO_LEN - (read ? 1 : 0));
			final = offset + len == msg->len;
			n = 0;
			if (read && len) {
				unsigned int ack_len = len - (final ? 1 : 0);

				if (ack_len)
					commands[n++] = s31_i2c_command(S31_I2C_CMD_READ,
							ack_len, false, false);
				if (final)
					commands[n++] = s31_i2c_command(S31_I2C_CMD_READ,
							1, false, true);
			} else if (len) {
				commands[n++] = s31_i2c_command(S31_I2C_CMD_WRITE,
							len, true, false);
			}
			commands[n++] = s31_i2c_command(final && i == num - 1 ?
				S31_I2C_CMD_STOP : S31_I2C_CMD_END, 0, false, false);
			ret = s31_i2c_batch(i2c, commands, n,
				read || !len ? NULL : msg->buf + offset, read ? 0 : len,
				read && len ? msg->buf + offset : NULL, read ? len : 0);
			if (ret)
				return ret;
			offset += len;
		} while (offset < msg->len);
	}
	return num;
}

static int s31_i2c_xfer(struct i2c_adapter *adapter,
			 struct i2c_msg *msgs, int num)
{
	struct esp32s31_i2c *i2c = i2c_get_adapdata(adapter);
	u32 commands[S31_I2C_COMMANDS] = {};
	u8 tx_fifo[S31_I2C_FIFO_LEN];
	unsigned int command_count, tx_count, rx_count;
	unsigned long completed;
	unsigned int i, rx_pos = 0;
	int ret;

#if IS_ENABLED(CONFIG_I2C_SLAVE)
	if (i2c->slave)
		return -EBUSY;
#endif

	ret = s31_i2c_build_commands(msgs, num, commands, &command_count,
				      tx_fifo, &tx_count, &rx_count);
	if (ret && ret != -EOPNOTSUPP)
		return ret;
	if (ret == -EOPNOTSUPP) {
		/* Reject an unsupported message before placing any START on wire. */
		for (i = 0; i < num; i++)
			if (msgs[i].flags & ~(I2C_M_RD | I2C_M_TEN | I2C_M_DMA_SAFE))
				return -EOPNOTSUPP;
	}

	if (s31_i2c_read(i2c, S31_I2C_SR) & S31_I2C_SR_BUS_BUSY) {
		int recovery = s31_i2c_recover_bus(adapter);

		if (recovery)
			return recovery;
	}
	if (ret == -EOPNOTSUPP) {
		ret = s31_i2c_long_xfer(i2c, msgs, num);
		if (ret < 0) {
			s31_i2c_reset_fsm(i2c);
			if (ret != -EAGAIN)
				s31_i2c_recover_bus(adapter);
		}
		return ret;
	}

	reinit_completion(&i2c->complete);
	i2c->result = -ETIMEDOUT;
	s31_i2c_write(i2c, S31_I2C_INT_ENA, 0);
	s31_i2c_write(i2c, S31_I2C_INT_CLR, S31_I2C_INT_ALL);
	s31_i2c_reset_fifos(i2c);

	for (i = 0; i < S31_I2C_COMMANDS; i++)
		s31_i2c_write(i2c, S31_I2C_COMMAND(i),
			      i < command_count ? commands[i] : 0);
	for (i = 0; i < tx_count; i++)
		s31_i2c_write(i2c, S31_I2C_DATA, tx_fifo[i]);

	s31_i2c_write(i2c, S31_I2C_INT_ENA, S31_I2C_INT_XFER);
	s31_i2c_write(i2c, S31_I2C_CTR,
		      s31_i2c_read(i2c, S31_I2C_CTR) |
		      S31_I2C_CTR_CONF_UPDATE);
	s31_i2c_write(i2c, S31_I2C_CTR,
		      s31_i2c_read(i2c, S31_I2C_CTR) |
		      S31_I2C_CTR_TRANS_START);

	completed = wait_for_completion_timeout(&i2c->complete,
					msecs_to_jiffies(S31_I2C_XFER_TIMEOUT_MS));
	s31_i2c_write(i2c, S31_I2C_INT_ENA, 0);
	if (!completed)
		i2c->result = -ETIMEDOUT;
	ret = i2c->result;
	if (ret) {
		s31_i2c_reset_fsm(i2c);
		if (ret == -ETIMEDOUT)
			s31_i2c_recover_bus(adapter);
		return ret;
	}

	if (FIELD_GET(S31_I2C_SR_RXFIFO_CNT,
		      s31_i2c_read(i2c, S31_I2C_SR)) < rx_count) {
		s31_i2c_reset_fsm(i2c);
		return -EIO;
	}

	for (i = 0; i < num; i++)
		if (msgs[i].flags & I2C_M_RD) {
			unsigned int j;

			for (j = 0; j < msgs[i].len; j++)
				msgs[i].buf[j] =
					s31_i2c_read(i2c, S31_I2C_DATA);
			rx_pos += msgs[i].len;
		}

	if (rx_pos != rx_count)
		return -EIO;

	return num;
}

static u32 s31_i2c_functionality(struct i2c_adapter *adapter)
{
	u32 functionality = I2C_FUNC_I2C | I2C_FUNC_10BIT_ADDR |
			    I2C_FUNC_SMBUS_EMUL;

#if IS_ENABLED(CONFIG_I2C_SLAVE)
	functionality |= I2C_FUNC_SLAVE;
#endif
	return functionality;
}

#if IS_ENABLED(CONFIG_I2C_SLAVE)
static int s31_i2c_reg_slave(struct i2c_client *slave)
{
	struct esp32s31_i2c *i2c = i2c_get_adapdata(slave->adapter);
	u32 address, ctr, fifo;

	if (i2c->slave)
		return -EBUSY;
	if (slave->flags & I2C_CLIENT_PEC)
		return -EOPNOTSUPP;

	if (slave->flags & I2C_CLIENT_TEN)
		address = S31_I2C_SLAVE_ADDR_10BIT |
			  (((slave->addr & 0xff) << 7) |
			   (((slave->addr >> 8) & 0x3) | 0x78));
	else
		address = slave->addr;
	address &= S31_I2C_SLAVE_ADDR_VALUE |
		   S31_I2C_SLAVE_ADDR_10BIT;

	s31_i2c_write(i2c, S31_I2C_INT_ENA, 0);
	s31_i2c_write(i2c, S31_I2C_INT_CLR, S31_I2C_INT_ALL);
	s31_i2c_reset_fifos(i2c);
	fifo = s31_i2c_read(i2c, S31_I2C_FIFO_CONF);
	fifo &= ~(S31_I2C_FIFO_RX_WM | S31_I2C_FIFO_TX_WM);
	fifo |= FIELD_PREP(S31_I2C_FIFO_RX_WM, 0) |
		FIELD_PREP(S31_I2C_FIFO_TX_WM, 0);
	s31_i2c_write(i2c, S31_I2C_FIFO_CONF, fifo);
	s31_i2c_write(i2c, S31_I2C_SLAVE_ADDR, address);
	s31_i2c_write(i2c, S31_I2C_SCL_STRETCH_CONF,
		      FIELD_PREP(GENMASK(9, 0), 0x3ff) |
		      S31_I2C_SLAVE_STRETCH_EN);
	ctr = s31_i2c_read(i2c, S31_I2C_CTR);
	ctr &= ~(S31_I2C_CTR_MASTER | S31_I2C_CTR_TRANS_START);
	ctr |= S31_I2C_CTR_CLK_EN | S31_I2C_CTR_SLV_TX_AUTO_START |
	       S31_I2C_CTR_CONF_UPDATE;
	s31_i2c_write(i2c, S31_I2C_CTR, ctr);
	i2c->slave = slave;
	i2c->slave_active = false;
	s31_i2c_write(i2c, S31_I2C_INT_ENA, S31_I2C_INT_SLAVE);
	return 0;
}

static int s31_i2c_unreg_slave(struct i2c_client *slave)
{
	struct esp32s31_i2c *i2c = i2c_get_adapdata(slave->adapter);
	u32 ctr;

	if (i2c->slave != slave)
		return -EINVAL;
	s31_i2c_write(i2c, S31_I2C_INT_ENA, 0);
	s31_i2c_write(i2c, S31_I2C_INT_CLR, S31_I2C_INT_ALL);
	i2c->slave = NULL;
	i2c->slave_active = false;
	ctr = s31_i2c_read(i2c, S31_I2C_CTR);
	ctr &= ~S31_I2C_CTR_SLV_TX_AUTO_START;
	ctr |= S31_I2C_CTR_MASTER | S31_I2C_CTR_ARB_EN |
	       S31_I2C_CTR_CONF_UPDATE;
	s31_i2c_write(i2c, S31_I2C_CTR, ctr);
	return 0;
}
#endif

static const struct i2c_algorithm s31_i2c_algorithm = {
	.xfer = s31_i2c_xfer,
	.functionality = s31_i2c_functionality,
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	.reg_slave = s31_i2c_reg_slave,
	.unreg_slave = s31_i2c_unreg_slave,
#endif
};

static const struct i2c_adapter_quirks s31_i2c_quirks = {
	.flags = I2C_AQ_NO_ZERO_LEN_READ,
};

static int s31_i2c_init_hardware(struct esp32s31_i2c *i2c)
{
	unsigned long source_rate = clk_get_rate(i2c->clk);
	u32 half_cycle, wait_high, high, timeout_exp;
	u32 ctr;

	if (!source_rate || i2c->frequency < 50000 ||
	    i2c->frequency > 1000000)
		return -EINVAL;

	half_cycle = DIV_ROUND_UP(source_rate, i2c->frequency * 2);
	if (half_cycle < 4 || half_cycle > 512)
		return -EINVAL;
	wait_high = min_t(u32, half_cycle / 2 - 2, 127);
	high = half_cycle - wait_high;
	/* END batches hold SCL while Linux services the FIFO. Allow bounded
	 * scheduler latency and target clock stretching (at least 100 ms). */
	timeout_exp = min_t(u32, fls(max_t(u32, source_rate / 10, 1) - 1),
			    31);

	ctr = S31_I2C_CTR_MASTER | S31_I2C_CTR_CLK_EN |
	      S31_I2C_CTR_ARB_EN;
	s31_i2c_write(i2c, S31_I2C_CTR, ctr);
	s31_i2c_write(i2c, S31_I2C_SCL_LOW, half_cycle - 1);
	s31_i2c_write(i2c, S31_I2C_SCL_HIGH,
		      FIELD_PREP(S31_I2C_SCL_HIGH_PERIOD, high) |
		      FIELD_PREP(S31_I2C_SCL_WAIT_HIGH, wait_high));
	s31_i2c_write(i2c, S31_I2C_SDA_HOLD, half_cycle / 4 - 1);
	s31_i2c_write(i2c, S31_I2C_SDA_SAMPLE, half_cycle / 2 - 1);
	s31_i2c_write(i2c, S31_I2C_RSTART_SETUP, half_cycle - 1);
	s31_i2c_write(i2c, S31_I2C_STOP_SETUP, half_cycle - 1);
	s31_i2c_write(i2c, S31_I2C_START_HOLD, half_cycle - 1);
	s31_i2c_write(i2c, S31_I2C_STOP_HOLD, half_cycle - 1);
	s31_i2c_write(i2c, S31_I2C_TIMEOUT,
		      FIELD_PREP(S31_I2C_TIMEOUT_VALUE, timeout_exp) |
		      S31_I2C_TIMEOUT_EN);
	s31_i2c_write(i2c, S31_I2C_FILTER_CFG,
		      FIELD_PREP(S31_I2C_FILTER_SCL, 2) |
		      FIELD_PREP(S31_I2C_FILTER_SDA, 2) |
		      S31_I2C_FILTER_SCL_EN | S31_I2C_FILTER_SDA_EN);
	s31_i2c_reset_fifos(i2c);
	s31_i2c_write(i2c, S31_I2C_INT_ENA, 0);
	s31_i2c_write(i2c, S31_I2C_INT_CLR, S31_I2C_INT_ALL);
	s31_i2c_write(i2c, S31_I2C_CTR, ctr | S31_I2C_CTR_CONF_UPDATE);

	return 0;
}

static void s31_i2c_disable_clock(void *data)
{
	clk_disable_unprepare(data);
}

static int s31_i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp32s31_i2c *i2c;
	struct reset_control *rst;
	int irq, ret;

	i2c = devm_kzalloc(dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;
	i2c->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(i2c->base))
		return PTR_ERR(i2c->base);
	i2c->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(i2c->clk))
		return dev_err_probe(dev, PTR_ERR(i2c->clk),
				     "failed to get controller clock\n");
	ret = clk_prepare_enable(i2c->clk);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable clock\n");
	ret = devm_add_action_or_reset(dev, s31_i2c_disable_clock, i2c->clk);
	if (ret)
		return ret;
	rst = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "reset unavailable\n");
	ret = reset_control_reset(rst);
	if (ret)
		return dev_err_probe(dev, ret, "reset failed\n");

	i2c->frequency = 100000;
	device_property_read_u32(dev, "clock-frequency", &i2c->frequency);
	ret = s31_i2c_init_hardware(i2c);
	if (ret)
		return dev_err_probe(dev, ret, "invalid bus timing\n");

	init_completion(&i2c->complete);
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	i2c->irq = irq;
	ret = devm_request_irq(dev, irq, s31_i2c_irq, 0, dev_name(dev), i2c);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	i2c->recovery.recover_bus = s31_i2c_recover_bus;
	i2c->adapter.owner = THIS_MODULE;
	i2c->adapter.algo = &s31_i2c_algorithm;
	i2c->adapter.quirks = &s31_i2c_quirks;
	i2c->adapter.bus_recovery_info = &i2c->recovery;
	i2c->adapter.dev.parent = dev;
	i2c->adapter.dev.of_node = dev->of_node;
	i2c->adapter.timeout = msecs_to_jiffies(S31_I2C_XFER_TIMEOUT_MS);
	snprintf(i2c->adapter.name, sizeof(i2c->adapter.name),
		 "ESP32-S31 I2C");
	i2c_set_adapdata(&i2c->adapter, i2c);
	platform_set_drvdata(pdev, i2c);

	ret = devm_i2c_add_adapter(dev, &i2c->adapter);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add adapter\n");

	dev_info(dev, "%u Hz master, version %#x\n", i2c->frequency,
		 s31_i2c_read(i2c, S31_I2C_DATE));
	return 0;
}

static const struct of_device_id s31_i2c_of_match[] = {
	{ .compatible = "espressif,esp32s31-i2c" },
	{ }
};
MODULE_DEVICE_TABLE(of, s31_i2c_of_match);

static struct platform_driver s31_i2c_driver = {
	.probe = s31_i2c_probe,
	.driver = {
		.name = "esp32s31-i2c",
		.of_match_table = s31_i2c_of_match,
	},
};
module_platform_driver(s31_i2c_driver);

MODULE_DESCRIPTION("Espressif ESP32-S31 I2C master controller");
MODULE_LICENSE("GPL");
