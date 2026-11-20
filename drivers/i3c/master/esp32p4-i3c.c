// SPDX-License-Identifier: GPL-2.0-only
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/i3c/master.h>
#include <linux/interrupt.h>
#include <linux/math64.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

/* TRM ch.50 I3C Controller — master control, timing and FIFO registers. */
#define P4_I3C_CTRL		0x00
#define  CTRL_START		BIT(2)
#define  CTRL_REG_CLK		BIT(3)
#define P4_I3C_CMD		0x08
#define P4_I3C_RESP		0x0c
#define P4_I3C_RX		0x10
#define P4_I3C_TX		0x14
#define P4_I3C_IBI_REJECT	0x2c
#define P4_I3C_INT_CLR		0x30
#define P4_I3C_INT_RAW		0x34
#define P4_I3C_INT_ST		0x38
#define P4_I3C_INT_ENA		0x3c
#define  INT_ERROR		BIT(6)
#define  INT_COMPLETE		BIT(7)
#define P4_I3C_RESET		0x44
#define  RESET_CORE		BIT(0)
#define  RESET_QUEUES		GENMASK(4, 1)
#define P4_I3C_BUF_LEVEL		0x48
#define  RESP_COUNT		GENMASK(11, 8)
#define P4_I3C_DATA_LEVEL	0x4c
#define  RX_COUNT		GENMASK(21, 16)
#define P4_I3C_PRESENT		0x50
#define  PRESENT_LINES_HIGH	GENMASK(1, 0)
#define  PRESENT_BUSY		BIT(2)
#define P4_I3C_OD_TIMING	0x60
#define P4_I3C_PP_TIMING	0x64
#define P4_I3C_FM_TIMING	0x68
#define  TIMING_LOW		GENMASK(15, 0)
#define  TIMING_HIGH		GENMASK(31, 16)
#define P4_I3C_START_HOLD	0x7c
#define P4_I3C_RSTART_SETUP	0x80
#define P4_I3C_STOP_HOLD	0x84
#define P4_I3C_STOP_SETUP	0x88
#define P4_I3C_BUS_FREE		0x90
#define P4_I3C_DAT		0xc0
#define  DAT_LEGACY		BIT(31)
#define  DAT_PARITY		BIT(23)
#define  DAT_DYNAMIC		GENMASK(22, 16)

#define CMD_IMMEDIATE		1
#define CMD_ADDRESS_ASSIGN	2
#define CMD_TID			GENMASK(6, 3)
#define CMD_CCC_CODE		GENMASK(14, 7)
#define CMD_CCC_PRESENT		BIT(15)
#define CMD_IMMEDIATE_BYTES	GENMASK(25, 23)
#define CMD_READ		BIT(28)
#define CMD_RESP		BIT(29)
#define CMD_STOP		BIT(30)
#define CMD_REGULAR_BYTES	GENMASK(31, 16)
#define CMD_DAA_COUNT		GENMASK(29, 26)
#define CMD_DAA_RESP		BIT(30)
#define CMD_DAA_STOP		BIT(31)
#define RESP_ERROR		GENMASK(31, 28)
#define RESP_TID		GENMASK(27, 24)
#define RESP_LENGTH		GENMASK(15, 0)
#define RESP_BCAST_NACK		4
#define RESP_ADDR_NACK		5
#define RESP_DATA_NACK		9

#define P4_I3C_FIFO_BYTES	128
#define P4_I3C_DEVICE_COUNT	12
#define P4_I3C_OD_HZ		100000
#define P4_I3C_IRQ_MARGIN_MS	10

struct p4_i3c {
	struct i3c_master_controller master;
	void __iomem *base;
	struct clk *clk;
	struct mutex lock;
	struct completion done;
	unsigned int irq;
	u32 od_rate;
	u32 i2c_rate;
	u32 status;
};

struct p4_i3c_command {
	u32 low;
	u32 high;
	const u8 *tx;
	u8 *rx;
	u16 len;
};

static struct p4_i3c *to_p4_i3c(struct i3c_master_controller *master)
{
	return container_of(master, struct p4_i3c, master);
}

static irqreturn_t p4_i3c_irq(int irq, void *data)
{
	struct p4_i3c *p4 = data;
	u32 status = readl(p4->base + P4_I3C_INT_ST) & (INT_COMPLETE | INT_ERROR);

	if (!status)
		return IRQ_NONE;

	writel(0, p4->base + P4_I3C_INT_ENA);
	p4->status = status;
	complete(&p4->done);
	return IRQ_HANDLED;
}

static void p4_i3c_reset_queues(struct p4_i3c *p4)
{
	writel(RESET_QUEUES, p4->base + P4_I3C_RESET);
	writel(0, p4->base + P4_I3C_RESET);
}

static void p4_i3c_abort(struct p4_i3c *p4)
{
	/*
	 * TRM ch.50 — CORE_SOFT_RST resets bus tracking without clearing
	 * timing or DAT. A broadcast address NACK can leave BUS_BUSY set
	 * with both lines released even after TRANSFER_COMPLETE.
	 */
	writel(RESET_CORE, p4->base + P4_I3C_RESET);
	p4_i3c_reset_queues(p4);
	writel(INT_COMPLETE | INT_ERROR, p4->base + P4_I3C_INT_CLR);
}

static int p4_i3c_execute(struct p4_i3c *p4, struct p4_i3c_command *cmds,
			 unsigned int ncmds, u32 dat, u32 *response)
{
	void __iomem *base = p4->base;
	const struct p4_i3c_command *last = &cmds[ncmds - 1];
	u32 ctrl, raw, state, length;
	unsigned int i, j, bytes = 0;
	unsigned long timeout;
	u32 rate;
	int ret;

	lockdep_assert_held(&p4->lock);
	state = readl(base + P4_I3C_PRESENT);
	if ((state & (PRESENT_BUSY | PRESENT_LINES_HIGH)) != PRESENT_LINES_HIGH)
		return -EBUSY;

	p4_i3c_reset_queues(p4);
	writel(INT_COMPLETE | INT_ERROR, base + P4_I3C_INT_CLR);
	writel(dat, base + P4_I3C_DAT);
	for (i = 0; i < ncmds; i++) {
		const struct p4_i3c_command *cmd = &cmds[i];

		bytes += cmd->len;
		writel(cmd->low | FIELD_PREP(CMD_TID, i), base + P4_I3C_CMD);
		writel(cmd->high, base + P4_I3C_CMD);
		if (!cmd->tx)
			continue;
		for (j = 0; j < cmd->len; j += sizeof(__le32)) {
			__le32 word = 0;

			memcpy(&word, cmd->tx + j,
			       min_t(size_t, cmd->len - j, sizeof(word)));
			writel(le32_to_cpu(word), base + P4_I3C_TX);
		}
	}

	/*
	 * Budget the wire time at the open-drain rate, including address
	 * and framing. ENTDAA also shifts up to eight identification bytes.
	 */
	rate = dat & DAT_LEGACY ? p4->i2c_rate : p4->od_rate;
	timeout = usecs_to_jiffies(DIV_ROUND_UP_ULL(
				(u64)(bytes + 12 * ncmds) * 9 * USEC_PER_SEC, rate));
	timeout += msecs_to_jiffies(P4_I3C_IRQ_MARGIN_MS);

	reinit_completion(&p4->done);
	writel(INT_COMPLETE | INT_ERROR, base + P4_I3C_INT_ENA);
	/* Keep the hardware defaults, including I3C_MST_REG_SCL_OE_FORCE_EN. */
	ctrl = readl(base + P4_I3C_CTRL);
	writel(ctrl | CTRL_START, base + P4_I3C_CTRL);
	writel(ctrl, base + P4_I3C_CTRL);
	if (!wait_for_completion_timeout(&p4->done, timeout)) {
		writel(0, base + P4_I3C_INT_ENA);
		synchronize_irq(p4->irq);
		p4_i3c_abort(p4);
		return -ETIMEDOUT;
	}

	if (FIELD_GET(RESP_COUNT, readl(base + P4_I3C_BUF_LEVEL)) != 1) {
		p4_i3c_abort(p4);
		return -EIO;
	}
	*response = readl(base + P4_I3C_RESP);
	length = FIELD_GET(RESP_LENGTH, *response);
	if (p4->status & INT_ERROR) {
		/* The error IRQ can precede the STOP and completion interrupt. */
		ret = readl_poll_timeout(base + P4_I3C_INT_RAW, raw,
					raw & INT_COMPLETE, 0, 1000);
		if (ret) {
			p4_i3c_abort(p4);
			return ret;
		}
		state = readl(base + P4_I3C_PRESENT);
		writel(INT_COMPLETE | INT_ERROR, base + P4_I3C_INT_CLR);
		if ((state & (PRESENT_BUSY | PRESENT_LINES_HIGH)) ==
		    (PRESENT_BUSY | PRESENT_LINES_HIGH))
			p4_i3c_abort(p4);
		return -EIO;
	}

	writel(INT_COMPLETE, base + P4_I3C_INT_CLR);
	if (FIELD_GET(RESP_TID, *response) != ncmds - 1 ||
	    FIELD_GET(RESP_ERROR, *response))
		return -EIO;

	if (!last->rx)
		return 0;

	if (length > last->len ||
	    DIV_ROUND_UP(length, sizeof(__le32)) !=
	    FIELD_GET(RX_COUNT, readl(base + P4_I3C_DATA_LEVEL)))
		return -EIO;

	for (i = 0; i < length; i += sizeof(__le32)) {
		__le32 data = cpu_to_le32(readl(base + P4_I3C_RX));

		memcpy(last->rx + i, &data, min_t(size_t, length - i, sizeof(data)));
	}
	return 0;
}

static void p4_i3c_regular(struct p4_i3c_command *cmd, u16 len, bool read,
			   const u8 *tx, u8 *rx)
{
	cmd->len = len;
	cmd->low = read ? CMD_READ : 0;
	cmd->high = FIELD_PREP(CMD_REGULAR_BYTES, len);
	cmd->tx = tx;
	cmd->rx = rx;
}

static u32 p4_i3c_dynamic(u8 addr)
{
	return FIELD_PREP(DAT_DYNAMIC, addr) |
	       ((hweight8(addr) & 1) ? 0 : DAT_PARITY);
}

static int p4_i3c_i2c_xfers(struct i2c_dev_desc *dev, struct i2c_msg *xfers,
			   int nxfers)
{
	struct p4_i3c *p4 = to_p4_i3c(i2c_dev_get_master(dev));
	struct p4_i3c_command cmds[2];
	u32 response = 0;
	int i, ret;

	if (nxfers < 1 || nxfers > ARRAY_SIZE(cmds) ||
	    (nxfers == 2 && ((xfers[0].flags & I2C_M_RD) ||
			    !(xfers[1].flags & I2C_M_RD))))
		return -EOPNOTSUPP;
	for (i = 0; i < nxfers; i++) {
		u16 flags = xfers[i].flags;
		bool read = flags & I2C_M_RD;

		if (flags & ~(I2C_M_RD | I2C_M_DMA_SAFE) ||
		    xfers[i].len > P4_I3C_FIFO_BYTES)
			return -EOPNOTSUPP;
		p4_i3c_regular(&cmds[i], xfers[i].len, read,
			       read ? NULL : xfers[i].buf, read ? xfers[i].buf : NULL);
	}
	cmds[nxfers - 1].low |= CMD_RESP | CMD_STOP;
	mutex_lock(&p4->lock);
	ret = p4_i3c_execute(p4, cmds, nxfers, DAT_LEGACY | dev->addr, &response);
	mutex_unlock(&p4->lock);
	if (ret == -EIO && FIELD_GET(RESP_ERROR, response) == RESP_ADDR_NACK)
		return -ENXIO;
	if (ret == -EIO && FIELD_GET(RESP_ERROR, response) == RESP_DATA_NACK)
		return -EREMOTEIO;
	if (!ret && FIELD_GET(RESP_LENGTH, response) !=
	    ((xfers[nxfers - 1].flags & I2C_M_RD) ? xfers[nxfers - 1].len : 0))
		return -EIO;
	return ret;
}

static int p4_i3c_priv_xfers(struct i3c_dev_desc *dev, struct i3c_priv_xfer *xfers,
			    int nxfers)
{
	struct p4_i3c *p4 = to_p4_i3c(i3c_dev_get_master(dev));
	struct p4_i3c_command cmds[2];
	u32 response = 0;
	int i, ret;

	if (nxfers < 1 || nxfers > ARRAY_SIZE(cmds) ||
	    (nxfers == 2 && (xfers[0].rnw || !xfers[1].rnw)))
		return -EOPNOTSUPP;
	for (i = 0; i < nxfers; i++) {
		struct i3c_priv_xfer *msg = &xfers[i];

		if (msg->len > P4_I3C_FIFO_BYTES)
			return -EOPNOTSUPP;
		p4_i3c_regular(&cmds[i], msg->len, msg->rnw,
			       msg->rnw ? NULL : msg->data.out,
			       msg->rnw ? msg->data.in : NULL);
	}
	cmds[nxfers - 1].low |= CMD_RESP | CMD_STOP;
	mutex_lock(&p4->lock);
	ret = p4_i3c_execute(p4, cmds, nxfers, p4_i3c_dynamic(dev->info.dyn_addr),
			    &response);
	mutex_unlock(&p4->lock);
	i = FIELD_GET(RESP_TID, response);
	if (ret == -EIO && i < nxfers &&
	    FIELD_GET(RESP_ERROR, response) == RESP_ADDR_NACK)
		xfers[i].err = I3C_ERROR_M2;
	if (ret)
		return ret;

	for (i = 0; i < nxfers; i++)
		xfers[i].actual_len = xfers[i].rnw ? FIELD_GET(RESP_LENGTH, response) :
					     xfers[i].len;
	return 0;
}

static u32 p4_i3c_daa_command(u8 ccc)
{
	return CMD_ADDRESS_ASSIGN | FIELD_PREP(CMD_CCC_CODE, ccc) |
	       FIELD_PREP(CMD_DAA_COUNT, 1) | CMD_DAA_RESP | CMD_DAA_STOP;
}

static int p4_i3c_send_ccc(struct i3c_master_controller *master,
			   struct i3c_ccc_cmd *ccc)
{
	struct p4_i3c *p4 = to_p4_i3c(master);
	struct p4_i3c_command cmd = { };
	struct i3c_ccc_cmd_dest *dest = ccc->dests;
	u32 dat = 0, response = 0;
	bool direct = ccc->id & I3C_CCC_DIRECT;
	unsigned int i;
	int ret;

	if (ccc->ndests != 1 || !dest || dest->payload.len > P4_I3C_FIFO_BYTES ||
	    (ccc->rnw && !direct))
		return -EOPNOTSUPP;
	if (ccc->id == I3C_CCC_SETDASA) {
		const u8 *address = dest->payload.data;

		if (dest->payload.len != 1)
			return -EINVAL;
		dat = dest->addr | FIELD_PREP(DAT_DYNAMIC, *address >> 1);
		cmd.low = p4_i3c_daa_command(ccc->id);
	} else {
		if (direct)
			dat = p4_i3c_dynamic(dest->addr);
		if (!ccc->rnw && dest->payload.len <= sizeof(cmd.high)) {
			const u8 *data = dest->payload.data;

			cmd.low = CMD_IMMEDIATE |
				  FIELD_PREP(CMD_IMMEDIATE_BYTES, dest->payload.len);
			for (i = 0; i < dest->payload.len; i++)
				cmd.high |= (u32)data[i] << (8 * i);
		} else {
			p4_i3c_regular(&cmd, dest->payload.len, ccc->rnw,
				       ccc->rnw ? NULL : dest->payload.data,
				       ccc->rnw ? dest->payload.data : NULL);
		}
		cmd.low |= CMD_CCC_PRESENT | FIELD_PREP(CMD_CCC_CODE, ccc->id) |
			   CMD_RESP | CMD_STOP;
	}

	mutex_lock(&p4->lock);
	ret = p4_i3c_execute(p4, &cmd, 1, dat, &response);
	mutex_unlock(&p4->lock);
	if (ret == -EIO &&
	    (FIELD_GET(RESP_ERROR, response) == RESP_BCAST_NACK ||
	     FIELD_GET(RESP_ERROR, response) == RESP_ADDR_NACK))
		ccc->err = I3C_ERROR_M2;
	if (!ret && ccc->rnw)
		dest->payload.len = FIELD_GET(RESP_LENGTH, response);
	return ret;
}

static int p4_i3c_daa(struct i3c_master_controller *master)
{
	struct p4_i3c *p4 = to_p4_i3c(master);
	struct p4_i3c_command cmd = { .low = p4_i3c_daa_command(I3C_CCC_ENTDAA) };
	unsigned int i;
	int addr, ret;

	for (i = 0; i < P4_I3C_DEVICE_COUNT; i++) {
		u32 response = 0;

		addr = i3c_master_get_free_addr(master, 0);
		if (addr < 0)
			return addr;
		mutex_lock(&p4->lock);
		ret = p4_i3c_execute(p4, &cmd, 1, p4_i3c_dynamic(addr), &response);
		mutex_unlock(&p4->lock);
		if (ret == -EIO &&
		    (FIELD_GET(RESP_ERROR, response) == RESP_BCAST_NACK ||
		     (FIELD_GET(RESP_ERROR, response) == RESP_ADDR_NACK &&
		      FIELD_GET(RESP_LENGTH, response) == 1)))
			return 0;
		if (ret)
			return ret;
		if (FIELD_GET(RESP_LENGTH, response))
			return -EIO;

		ret = i3c_master_add_i3c_dev_locked(master, addr);
		if (ret)
			return ret;
	}
	return 0;
}

static int p4_i3c_bus_init(struct i3c_master_controller *master)
{
	struct p4_i3c *p4 = to_p4_i3c(master);
	struct i3c_device_info info = { };
	unsigned long rate = clk_get_rate(p4->clk);
	u32 od, pp, fm, fm_low, fm_high, i2c_rate, timing;
	int addr;

	if (!rate || !master->bus.scl_rate.i3c ||
	    master->bus.scl_rate.i2c > 400000)
		return -EINVAL;
	od = DIV_ROUND_UP(rate, 2 * P4_I3C_OD_HZ);
	pp = max_t(u32, 2, DIV_ROUND_UP(rate, 2 * master->bus.scl_rate.i3c));
	i2c_rate = master->bus.scl_rate.i2c ?: P4_I3C_OD_HZ;
	fm = DIV_ROUND_UP(rate, i2c_rate);
	/* Fast mode requires at least 1.3 us low across the supported rates. */
	fm_low = max_t(u32, DIV_ROUND_UP(fm, 2),
		       DIV_ROUND_UP_ULL((u64)rate * 1300, NSEC_PER_SEC));
	fm_high = fm - fm_low;
	timing = DIV_ROUND_UP(rate, 250000);
	if (od > U16_MAX || !fm_high || fm_low > U16_MAX ||
	    fm_high > U16_MAX || pp > 256 || timing > 511)
		return -ERANGE;
	p4->i2c_rate = rate / fm;
	p4->od_rate = rate / (2 * od);

	writel(FIELD_PREP(TIMING_LOW, od - 1) | FIELD_PREP(TIMING_HIGH, od - 1),
	       p4->base + P4_I3C_OD_TIMING);
	writel(FIELD_PREP(TIMING_LOW, pp - 1) | FIELD_PREP(TIMING_HIGH, pp - 1),
	       p4->base + P4_I3C_PP_TIMING);
	writel(FIELD_PREP(TIMING_LOW, fm_low - 1) | FIELD_PREP(TIMING_HIGH, fm_high - 1),
	       p4->base + P4_I3C_FM_TIMING);
	writel(timing, p4->base + P4_I3C_START_HOLD);
	writel(timing, p4->base + P4_I3C_RSTART_SETUP);
	writel(timing, p4->base + P4_I3C_STOP_HOLD);
	writel(timing, p4->base + P4_I3C_STOP_SETUP);
	writel(od, p4->base + P4_I3C_BUS_FREE);
	writel(GENMASK(11, 0), p4->base + P4_I3C_IBI_REJECT);

	addr = i3c_master_get_free_addr(master, 0);
	if (addr < 0)
		return addr;
	info.dyn_addr = addr;
	return i3c_master_set_info(master, &info);
}

static const struct i3c_master_controller_ops p4_i3c_ops = {
	.bus_init = p4_i3c_bus_init,
	.do_daa = p4_i3c_daa,
	.send_ccc_cmd = p4_i3c_send_ccc,
	.priv_xfers = p4_i3c_priv_xfers,
	.i2c_xfers = p4_i3c_i2c_xfers,
};

static int p4_i3c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *reset;
	struct p4_i3c *p4;
	int irq, ret;

	p4 = devm_kzalloc(dev, sizeof(*p4), GFP_KERNEL);
	if (!p4)
		return -ENOMEM;

	p4->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(p4->base))
		return PTR_ERR(p4->base);

	p4->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(p4->clk))
		return dev_err_probe(dev, PTR_ERR(p4->clk), "cannot enable clock\n");

	reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset), "cannot get reset\n");

	/*
	 * The platform core selects pinctrl before probe. Reset the bus
	 * monitor after that selection, which can appear as a START.
	 */
	ret = reset_control_reset(reset);
	if (ret)
		return dev_err_probe(dev, ret, "cannot reset controller\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	p4->irq = irq;
	init_completion(&p4->done);
	mutex_init(&p4->lock);
	writel(0, p4->base + P4_I3C_INT_ENA);
	writel(readl(p4->base + P4_I3C_CTRL) | CTRL_REG_CLK, p4->base + P4_I3C_CTRL);
	ret = devm_request_irq(dev, irq, p4_i3c_irq, 0, dev_name(dev), p4);
	if (ret)
		return dev_err_probe(dev, ret, "cannot request IRQ\n");

	platform_set_drvdata(pdev, p4);
	return i3c_master_register(&p4->master, dev, &p4_i3c_ops, false);
}

static void p4_i3c_remove(struct platform_device *pdev)
{
	struct p4_i3c *p4 = platform_get_drvdata(pdev);

	i3c_master_unregister(&p4->master);
}

static const struct of_device_id p4_i3c_of_match[] = {
	{ .compatible = "espressif,esp32p4-i3c" },
	{ }
};
MODULE_DEVICE_TABLE(of, p4_i3c_of_match);

static struct platform_driver p4_i3c_driver = {
	.probe = p4_i3c_probe,
	.remove = p4_i3c_remove,
	.driver = {
		.name = "esp32p4-i3c",
		.of_match_table = p4_i3c_of_match,
	},
};
module_platform_driver(p4_i3c_driver);

MODULE_DESCRIPTION("Espressif ESP32-P4 I3C controller");
MODULE_LICENSE("GPL");
