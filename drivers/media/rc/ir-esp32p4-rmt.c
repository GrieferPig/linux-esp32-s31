// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-P4 RMT pulse transmitter and receiver using two disjoint RAM banks. */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/spinlock.h>
#include <media/rc-core.h>

/* TRM ch.62 RMT: CH0/CH4 FIFO registers and configuration. */
#define RMT_TX_FIFO		0x00
#define RMT_RX_FIFO		0x10
#define RMT_TX_CONF		0x20
#define RMT_RX_CONF0		0x30
#define RMT_RX_CONF1		0x34
#define RMT_RX_STATUS		0x60
#define RMT_INT_ST		0x74
#define RMT_INT_ENA		0x78
#define RMT_INT_CLR		0x7c
#define RMT_TX_CARRIER		0x80
#define RMT_SYS_CONF		0xc0
#define RMT_REF_CNT_RST		0xc8

#define RMT_TX_START		BIT(0)
#define RMT_TX_RD_RST		BIT(1)
#define RMT_TX_APB_RST		BIT(2)
#define RMT_TX_IDLE_ENABLE	BIT(6)
#define RMT_TX_STOP		BIT(7)
#define RMT_TX_DIV		GENMASK(15, 8)
#define RMT_TX_BLOCKS		GENMASK(19, 16)
#define RMT_TX_CARRIER_EFF	BIT(20)
#define RMT_TX_CARRIER_EN	BIT(21)
#define RMT_TX_CARRIER_HIGH	BIT(22)
#define RMT_TX_UPDATE		BIT(24)

#define RMT_RX_DIV		GENMASK(7, 0)
#define RMT_RX_IDLE_THRES	GENMASK(22, 8)
#define RMT_RX_BLOCKS		GENMASK(27, 24)
#define RMT_RX_EN		BIT(0)
#define RMT_RX_WR_RST		BIT(1)
#define RMT_RX_APB_RST		BIT(2)
#define RMT_RX_OWNER		BIT(3)
#define RMT_RX_UPDATE		BIT(15)
#define RMT_RX_WRITE_ADDR	GENMASK(9, 0)

#define RMT_TX_END		BIT(0)
#define RMT_TX_ERR		BIT(4)
#define RMT_RX_END		BIT(16)
#define RMT_RX_ERR		BIT(20)
#define RMT_TX_IRQS		(RMT_TX_END | RMT_TX_ERR)
#define RMT_RX_IRQS		(RMT_RX_END | RMT_RX_ERR)

#define RMT_RAM_CLOCK		BIT(1)
#define RMT_RAM_ON		BIT(3)
#define RMT_REG_CLOCK		BIT(31)

#define RMT_RAM_BLOCK_WORDS	48
#define RMT_BANK_BLOCKS		4
#define RMT_BANK_WORDS		(RMT_RAM_BLOCK_WORDS * RMT_BANK_BLOCKS)
#define RMT_TICKS_PER_SEC	1000000
#define RMT_DURATION		GENMASK(14, 0)
#define RMT_LEVEL		BIT(15)
#define RMT_DEFAULT_TIMEOUT	20000

struct esp32p4_rmt {
	void __iomem *base;
	struct rc_dev *rc;
	struct completion tx_done;
	struct mutex tx_lock;
	spinlock_t lock;
	unsigned long clk_rate;
	unsigned int clock_div;
	u32 carrier;
	u32 duty_cycle;
	u32 rx_timeout;
	bool rx_enabled;
	bool rx_active_low;
	int tx_result;
	int irq;
};

static void esp32p4_rmt_rx_stop(struct esp32p4_rmt *rmt)
{
	/* Transfer memory ownership from the receiver to the APB bus. */
	writel(0, rmt->base + RMT_RX_CONF1);
	writel(RMT_RX_UPDATE, rmt->base + RMT_RX_CONF1);
}

static void esp32p4_rmt_rx_arm(struct esp32p4_rmt *rmt)
{
	u32 conf = FIELD_PREP(RMT_RX_DIV, rmt->clock_div & 0xff) |
		   FIELD_PREP(RMT_RX_IDLE_THRES, rmt->rx_timeout) |
		   FIELD_PREP(RMT_RX_BLOCKS, RMT_BANK_BLOCKS);

	writel(RMT_RX_WR_RST | RMT_RX_APB_RST, rmt->base + RMT_RX_CONF1);
	writel(0, rmt->base + RMT_RX_CONF1);
	writel(conf, rmt->base + RMT_RX_CONF0);
	writel(BIT(4), rmt->base + RMT_REF_CNT_RST);
	writel(RMT_RX_OWNER | RMT_RX_EN, rmt->base + RMT_RX_CONF1);
	writel(RMT_RX_OWNER | RMT_RX_EN | RMT_RX_UPDATE, rmt->base + RMT_RX_CONF1);
}

static void esp32p4_rmt_rx_frame(struct esp32p4_rmt *rmt, bool overflow)
{
	unsigned int words, address, i;
	u32 word = 0;

	/*
	 * MEM_WADDR_EX is relative to all RMT RAM. As in IDF
	 * rmt_ll_rx_get_memory_writer_offset(), subtract the channel base.
	 */
	address = FIELD_GET(RMT_RX_WRITE_ADDR, readl(rmt->base + RMT_RX_STATUS));
	if (overflow || address < RMT_BANK_WORDS || address > 2 * RMT_BANK_WORDS) {
		ir_raw_event_overflow(rmt->rc);
		return;
	}

	words = address - RMT_BANK_WORDS;
	for (i = 0; i < 2 * words; i++) {
		struct ir_raw_event event = {};

		if (!(i & 1))
			word = readl(rmt->base + RMT_RX_FIFO);
		else
			word >>= 16;

		event.duration = FIELD_GET(RMT_DURATION, word);
		if (!event.duration)
			break;
		event.pulse = !!(word & RMT_LEVEL) ^ rmt->rx_active_low;
		ir_raw_event_store_with_filter(rmt->rc, &event);
	}
	/*
	 * RMT RX_END records the idle interval in hardware. Commit it as a
	 * space so rc-core flushes the last pulse before emitting a timeout.
	 * Setting idle directly would mark the last pulse itself as a timeout.
	 */
	if (!rmt->rc->idle) {
		struct ir_raw_event idle = { .duration = rmt->rx_timeout };

		ir_raw_event_store_with_filter(rmt->rc, &idle);
	}
}

static irqreturn_t esp32p4_rmt_irq(int irq, void *data)
{
	struct esp32p4_rmt *rmt = data;
	bool deliver = false;
	u32 status;

	spin_lock(&rmt->lock);
	status = readl(rmt->base + RMT_INT_ST) & (RMT_TX_IRQS | RMT_RX_IRQS);
	if (!status) {
		spin_unlock(&rmt->lock);
		return IRQ_NONE;
	}

	writel(status, rmt->base + RMT_INT_CLR);
	if (status & RMT_TX_IRQS) {
		rmt->tx_result = status & RMT_TX_ERR ? -EIO : 0;
		complete(&rmt->tx_done);
	}
	if ((status & RMT_RX_IRQS) && rmt->rx_enabled) {
		esp32p4_rmt_rx_stop(rmt);
		esp32p4_rmt_rx_frame(rmt, status & RMT_RX_ERR);
		esp32p4_rmt_rx_arm(rmt);
		deliver = true;
	}
	spin_unlock(&rmt->lock);

	if (deliver)
		ir_raw_event_handle(rmt->rc);
	return IRQ_HANDLED;
}

static int esp32p4_rmt_open(struct rc_dev *rc)
{
	struct esp32p4_rmt *rmt = rc->priv;
	unsigned long flags;

	spin_lock_irqsave(&rmt->lock, flags);
	writel(RMT_RX_IRQS, rmt->base + RMT_INT_CLR);
	rmt->rx_enabled = true;
	esp32p4_rmt_rx_arm(rmt);
	writel(RMT_TX_IRQS | RMT_RX_IRQS, rmt->base + RMT_INT_ENA);
	spin_unlock_irqrestore(&rmt->lock, flags);
	return 0;
}

static void esp32p4_rmt_close(struct rc_dev *rc)
{
	struct esp32p4_rmt *rmt = rc->priv;
	unsigned long flags;

	spin_lock_irqsave(&rmt->lock, flags);
	writel(RMT_TX_IRQS, rmt->base + RMT_INT_ENA);
	rmt->rx_enabled = false;
	esp32p4_rmt_rx_stop(rmt);
	writel(RMT_RX_IRQS, rmt->base + RMT_INT_CLR);
	spin_unlock_irqrestore(&rmt->lock, flags);
	synchronize_irq(rmt->irq);
}

static int esp32p4_rmt_set_carrier(struct rc_dev *rc, u32 carrier)
{
	struct esp32p4_rmt *rmt = rc->priv;
	u32 cycles;

	if (carrier) {
		cycles = DIV_ROUND_CLOSEST(rmt->clk_rate, carrier);
		if (cycles < 2 || cycles > U16_MAX)
			return -ERANGE;
	}
	mutex_lock(&rmt->tx_lock);
	rmt->carrier = carrier;
	mutex_unlock(&rmt->tx_lock);
	return 0;
}

static int esp32p4_rmt_set_duty(struct rc_dev *rc, u32 duty)
{
	struct esp32p4_rmt *rmt = rc->priv;

	if (!duty || duty >= 100)
		return -EINVAL;
	mutex_lock(&rmt->tx_lock);
	rmt->duty_cycle = duty;
	mutex_unlock(&rmt->tx_lock);
	return 0;
}

static int esp32p4_rmt_set_timeout(struct rc_dev *rc, unsigned int timeout)
{
	struct esp32p4_rmt *rmt = rc->priv;
	unsigned long flags;

	if (timeout < rc->min_timeout || timeout > rc->max_timeout)
		return -ERANGE;
	spin_lock_irqsave(&rmt->lock, flags);
	rmt->rx_timeout = timeout;
	rc->timeout = timeout;
	if (rmt->rx_enabled) {
		esp32p4_rmt_rx_stop(rmt);
		writel(RMT_RX_IRQS, rmt->base + RMT_INT_CLR);
		esp32p4_rmt_rx_arm(rmt);
	}
	spin_unlock_irqrestore(&rmt->lock, flags);
	return 0;
}

static int esp32p4_rmt_tx(struct rc_dev *rc, unsigned int *durations, unsigned int n)
{
	struct esp32p4_rmt *rmt = rc->priv;
	u32 words[RMT_BANK_WORDS] = {};
	unsigned int i, entries = 0, elapsed = 0;
	unsigned long flags;
	u32 conf;
	int ret;

	for (i = 0; i < n; i++) {
		unsigned int left = durations[i];

		if (!left)
			return -EINVAL;
		while (left) {
			u32 part = min_t(u32, left, FIELD_MAX(RMT_DURATION));
			u32 symbol = part | ((i & 1) ? 0 : RMT_LEVEL);

			/* The final zero half-word is the hardware end marker. */
			if (entries == 2 * RMT_BANK_WORDS - 1)
				return -EMSGSIZE;
			words[entries / 2] |= symbol << (16 * (entries & 1));
			elapsed += part;
			left -= part;
			entries++;
		}
	}
	if (!entries)
		return -EINVAL;

	mutex_lock(&rmt->tx_lock);
	conf = FIELD_PREP(RMT_TX_DIV, rmt->clock_div & 0xff) |
	       FIELD_PREP(RMT_TX_BLOCKS, RMT_BANK_BLOCKS) | RMT_TX_IDLE_ENABLE;
	if (rmt->carrier) {
		u32 cycles = DIV_ROUND_CLOSEST(rmt->clk_rate, rmt->carrier);
		u32 high = clamp_t(u32, DIV_ROUND_CLOSEST(cycles * rmt->duty_cycle, 100),
				   1, cycles - 1);

		writel(((high - 1) << 16) | (cycles - high - 1),
		       rmt->base + RMT_TX_CARRIER);
		conf |= RMT_TX_CARRIER_EN | RMT_TX_CARRIER_EFF | RMT_TX_CARRIER_HIGH;
	}

	spin_lock_irqsave(&rmt->lock, flags);
	reinit_completion(&rmt->tx_done);
	writel(RMT_TX_IRQS, rmt->base + RMT_INT_CLR);
	writel(conf | RMT_TX_RD_RST | RMT_TX_APB_RST, rmt->base + RMT_TX_CONF);
	writel(conf, rmt->base + RMT_TX_CONF);
	writel(conf | RMT_TX_UPDATE, rmt->base + RMT_TX_CONF);
	writel(BIT(0), rmt->base + RMT_REF_CNT_RST);
	for (i = 0; i <= entries / 2; i++)
		writel(words[i], rmt->base + RMT_TX_FIFO);
	writel(conf | RMT_TX_START, rmt->base + RMT_TX_CONF);
	spin_unlock_irqrestore(&rmt->lock, flags);

	if (!wait_for_completion_timeout(&rmt->tx_done,
					 usecs_to_jiffies(elapsed) + msecs_to_jiffies(50))) {
		spin_lock_irqsave(&rmt->lock, flags);
		writel(conf | RMT_TX_STOP, rmt->base + RMT_TX_CONF);
		writel(RMT_TX_IRQS, rmt->base + RMT_INT_CLR);
		spin_unlock_irqrestore(&rmt->lock, flags);
		ret = -ETIMEDOUT;
	} else {
		ret = rmt->tx_result;
	}
	mutex_unlock(&rmt->tx_lock);
	return ret ? ret : n;
}

static int esp32p4_rmt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp32p4_rmt *rmt;
	struct reset_control *reset;
	struct rc_dev *rc;
	struct clk *clk;
	int ret;

	rmt = devm_kzalloc(dev, sizeof(*rmt), GFP_KERNEL);
	if (!rmt)
		return -ENOMEM;
	rmt->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(rmt->base))
		return PTR_ERR(rmt->base);

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "RMT clock unavailable\n");
	rmt->clk_rate = clk_get_rate(clk);
	rmt->clock_div = rmt->clk_rate / RMT_TICKS_PER_SEC;
	if (rmt->clk_rate % RMT_TICKS_PER_SEC || !rmt->clock_div ||
	    rmt->clock_div > 256)
		return -ERANGE;

	reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset), "RMT reset unavailable\n");
	ret = reset_control_reset(reset);
	if (ret)
		return ret;

	spin_lock_init(&rmt->lock);
	mutex_init(&rmt->tx_lock);
	init_completion(&rmt->tx_done);
	rmt->carrier = 38000;
	rmt->duty_cycle = 33;
	rmt->rx_timeout = RMT_DEFAULT_TIMEOUT;
	rmt->rx_active_low = device_property_read_bool(dev, "espressif,rx-active-low");

	rc = devm_rc_allocate_device(dev, RC_DRIVER_IR_RAW);
	if (!rc)
		return -ENOMEM;
	rmt->rc = rc;
	rc->priv = rmt;
	rc->device_name = "ESP32-P4 RMT";
	rc->driver_name = "esp32p4-rmt";
	rc->map_name = RC_MAP_EMPTY;
	rc->allowed_protocols = RC_PROTO_BIT_ALL_IR_DECODER;
	rc->rx_resolution = 1;
	rc->timeout = RMT_DEFAULT_TIMEOUT;
	rc->min_timeout = 1000;
	rc->max_timeout = FIELD_MAX(RMT_DURATION);
	rc->open = esp32p4_rmt_open;
	rc->close = esp32p4_rmt_close;
	rc->s_timeout = esp32p4_rmt_set_timeout;
	rc->tx_ir = esp32p4_rmt_tx;
	rc->s_tx_carrier = esp32p4_rmt_set_carrier;
	rc->s_tx_duty_cycle = esp32p4_rmt_set_duty;

	/* RMT_SYS_CONF.APB_FIFO_MASK is clear for FIFO access. */
	writel(RMT_REG_CLOCK | RMT_RAM_ON | RMT_RAM_CLOCK, rmt->base + RMT_SYS_CONF);
	writel(0, rmt->base + RMT_INT_ENA);
	writel(RMT_TX_IRQS | RMT_RX_IRQS, rmt->base + RMT_INT_CLR);
	rmt->irq = platform_get_irq(pdev, 0);
	if (rmt->irq < 0)
		return rmt->irq;
	ret = devm_request_irq(dev, rmt->irq, esp32p4_rmt_irq, 0, dev_name(dev), rmt);
	if (ret)
		return ret;

	writel(RMT_TX_IRQS, rmt->base + RMT_INT_ENA);
	return devm_rc_register_device(dev, rc);
}

static const struct of_device_id esp32p4_rmt_of_match[] = {
	{ .compatible = "espressif,esp32p4-rmt" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32p4_rmt_of_match);

static struct platform_driver esp32p4_rmt_driver = {
	.probe = esp32p4_rmt_probe,
	.driver = {
		.name = "esp32p4-rmt",
		.of_match_table = esp32p4_rmt_of_match,
	},
};
module_platform_driver(esp32p4_rmt_driver);

MODULE_DESCRIPTION("ESP32-P4 RMT pulse transmitter and receiver");
MODULE_LICENSE("GPL");
