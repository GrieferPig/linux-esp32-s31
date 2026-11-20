// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-P4 TWAI: SocketCAN over the SJA1000 frame and IRQ machinery. */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include "sja1000.h"

/* TRM ch.58 TWAI: BUS_TIMING_0.BAUD_PRESC and SYNC_JUMP_WIDTH. */
#define P4_TWAI_BRP		GENMASK(13, 0)
#define P4_TWAI_SJW		GENMASK(15, 14)
#define P4_TWAI_TSEG1		GENMASK(3, 0)
#define P4_TWAI_TSEG2		GENMASK(6, 4)
#define P4_TWAI_TRIPLE_SAMPLE	BIT(7)
#define P4_TWAI_IRQ_TIMESTAMP	BIT(4)

static u8 p4_twai_read(const struct sja1000_priv *priv, int reg)
{
	return readl(priv->reg_base + reg * sizeof(u32));
}

static void p4_twai_write(const struct sja1000_priv *priv, int reg, u8 value)
{
	/* The SJA1000 output control address is reserved on P4. */
	if (reg == SJA1000_OCR)
		return;
	if (reg == SJA1000_IER)
		value &= ~P4_TWAI_IRQ_TIMESTAMP;
	/* TRM ch.58 MODE.SELF_TEST_MODE lets SRR complete without an ACK. */
	if (reg == SJA1000_MOD && !(value & MOD_RM) &&
	    (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK))
		value |= MOD_STM;
	writel(value, priv->reg_base + reg * sizeof(u32));
}

static const struct can_bittiming_const p4_twai_timing = {
	.name = "esp32p4-twai",
	.tseg1_min = 1,
	.tseg1_max = 16,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 16384,
	.brp_inc = 1,
};

static int p4_twai_set_bittiming(struct net_device *netdev)
{
	struct sja1000_priv *priv = netdev_priv(netdev);
	const struct can_bittiming *bt = &priv->can.bittiming;
	u32 btr0, btr1;

	btr0 = FIELD_PREP(P4_TWAI_BRP, bt->brp - 1) |
	       FIELD_PREP(P4_TWAI_SJW, bt->sjw - 1);
	btr1 = FIELD_PREP(P4_TWAI_TSEG1, bt->prop_seg + bt->phase_seg1 - 1) |
	       FIELD_PREP(P4_TWAI_TSEG2, bt->phase_seg2 - 1);
	if (priv->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES)
		btr1 |= P4_TWAI_TRIPLE_SAMPLE;
	writel(btr0, priv->reg_base + SJA1000_BTR0 * sizeof(u32));
	writel(btr1, priv->reg_base + SJA1000_BTR1 * sizeof(u32));
	return 0;
}

static int p4_twai_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *reset;
	struct net_device *netdev;
	struct sja1000_priv *priv;
	void __iomem *regs;
	unsigned long rate;
	struct clk *clk;
	int irq, ret;

	regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(regs))
		return PTR_ERR(regs);
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "TWAI clock unavailable\n");
	rate = clk_get_rate(clk);
	if (rate < 2 || rate % 2)
		return dev_err_probe(dev, -EINVAL, "invalid TWAI clock rate %lu\n", rate);

	reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset), "TWAI reset unavailable\n");
	ret = reset_control_reset(reset);
	if (ret)
		return dev_err_probe(dev, ret, "TWAI reset failed\n");

	netdev = alloc_sja1000dev(0);
	if (!netdev)
		return -ENOMEM;

	priv = netdev_priv(netdev);
	priv->reg_base = regs;
	priv->read_reg = p4_twai_read;
	priv->write_reg = p4_twai_write;
	priv->flags = SJA1000_QUIRK_NO_CDR_REG;
	/* TRM ch.58 Bit Timing: time quantum is 2 * (BRP + 1) core cycles. */
	priv->can.clock.freq = rate / 2;
	priv->can.bittiming_const = &p4_twai_timing;
	priv->can.do_set_bittiming = p4_twai_set_bittiming;

	netdev->irq = irq;
	SET_NETDEV_DEV(netdev, dev);
	ret = register_sja1000dev(netdev);
	if (ret) {
		free_sja1000dev(netdev);
		return ret;
	}
	platform_set_drvdata(pdev, netdev);
	return 0;
}

static void p4_twai_remove(struct platform_device *pdev)
{
	struct net_device *netdev = platform_get_drvdata(pdev);

	unregister_sja1000dev(netdev);
	free_sja1000dev(netdev);
}

static const struct of_device_id p4_twai_match[] = {
	{ .compatible = "espressif,esp32p4-twai" },
	{ }
};
MODULE_DEVICE_TABLE(of, p4_twai_match);

static struct platform_driver p4_twai_driver = {
	.probe = p4_twai_probe,
	.remove = p4_twai_remove,
	.driver = {
		.name = "esp32p4-twai",
		.of_match_table = p4_twai_match,
	},
};
module_platform_driver(p4_twai_driver);

MODULE_DESCRIPTION("Espressif ESP32-P4 TWAI controller");
MODULE_LICENSE("GPL");
