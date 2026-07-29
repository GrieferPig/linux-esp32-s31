// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S31 peripheral clock/reset controller.
 *
 * This owns every gate, reset and connectivity-domain clock needed by the
 * Linux UART, SDMMC and GMAC drivers.  PLL programming is kept fixed because
 * the CPU, flash XIP and PSRAM need those PLLs before Linux can execute.
 */

#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include <dt-bindings/clock/esp32s31-clock.h>

#define ESP32S31_XTAL_RATE		40000000UL
#define ESP32S31_SYS_RATE		80000000UL
#define ESP32S31_MPLL_RATE		500000000UL
#define ESP32S31_SDMMC_CIU_RATE		10000000UL

#define HP_SDIO_HOST_CTRL0		0xc0
#define HP_SDIO_HOST_FUNC_CTRL0		0xc4
#define HP_EMAC_CTRL0			0xc8
#define HP_UART0_CTRL0			0x88
#define HP_UART_CTRL_STRIDE		0x04
#define HP_TIMERGRP1_CTRL0		0x11c

#define HP_SDMMC_SYS_CLK_EN		BIT(0)
#define HP_SDIO_LS_CLK_SRC_SEL		BIT(2)
#define HP_SDIO_LS_CLK_EN		BIT(3)
#define HP_SDIO_LS_CLK_DIV_NUM		GENMASK(11, 4)

#define HP_SDIO_EDGE_UPDATE		BIT(0)
#define HP_SDIO_EDGE_L			GENMASK(4, 1)
#define HP_SDIO_EDGE_H			GENMASK(8, 5)
#define HP_SDIO_EDGE_N			GENMASK(12, 9)
#define HP_SDIO_SLF_CLK_EDGE_SEL	GENMASK(15, 13)
#define HP_SDIO_DRV_CLK_EDGE_SEL	GENMASK(18, 16)
#define HP_SDIO_SAM_CLK_EDGE_SEL	GENMASK(21, 19)
#define HP_SDIO_SLF_CLK_EN		BIT(22)
#define HP_SDIO_DRV_CLK_EN		BIT(23)
#define HP_SDIO_SAM_CLK_EN		BIT(24)

#define HP_EMAC_SYS_CLK_EN		BIT(0)

#define HP_UART_SYS_CLK_EN		BIT(0)
#define HP_UART_APB_CLK_EN		BIT(1)
#define HP_UART_CORE_RST_EN		BIT(2)
#define HP_UART_APB_RST_EN		BIT(3)
#define HP_UART_FORCE_NORST		BIT(4)
#define HP_UART_CLK_SRC_SEL		GENMASK(6, 5)
#define HP_UART_CLK_EN			BIT(7)
#define HP_UART_SCLK_DIV_NUM		GENMASK(15, 8)
#define HP_UART_SCLK_DIV_NUMERATOR	GENMASK(23, 16)
#define HP_UART_SCLK_DIV_DENOMINATOR	GENMASK(31, 24)

#define HP_TIMERGRP_APB_CLK_EN		BIT(0)
#define HP_TIMERGRP_RST_EN		BIT(1)
#define HP_TIMERGRP_FORCE_NORST		BIT(2)
#define HP_TIMERGRP_WDT_SRC_SEL		GENMASK(10, 9)
#define HP_TIMERGRP_WDT_CLK_EN		BIT(11)

#define HP_SYS_UART_MEM_LP_CTRL0		0x28c
#define HP_SYS_UART_MEM_LP_STRIDE	0x04
#define HP_SYS_UART_MEM_LP_EN		BIT(2)
#define HP_SYS_UART_MEM_FORCE_CTRL	BIT(3)

#define CNNT_CLK_EN			0x00
#define CNNT_SYS_SDMMC_MEM_LP_CTRL	0x10
#define CNNT_SYS_GMAC_MEM_LP_CTRL	0x1c
#define CNNT_SYS_HP_SDMMC_CTRL		0x38
#define CNNT_SYS_HP_EMAC_CTRL		0x3c
#define CNNT_SYS_HP_EMAC_REF_CTRL	0x40
#define CNNT_SYS_HP_EMAC_RMII_PAD_CTRL	0x44
#define CNNT_SYS_HP_EMAC_RMII_CTRL	0x48
#define CNNT_SYS_HP_EMAC_RX_CTRL		0x4c
#define CNNT_SYS_HP_EMAC_TX_CTRL		0x50
#define CNNT_SYS_HP_EMAC_PTP_CTRL	0x54
#define CNNT_SYS_GMAC_CTRL0		0x60

#define CNNT_SYS_CLK_EN			BIT(31)
#define CNNT_MEM_LP_EN			BIT(2)
#define CNNT_MEM_LP_FORCE_CTRL		BIT(3)

#define CNNT_SDMMC_AHB2AXI_POST_WRITE_EN BIT(29)
#define CNNT_SDMMC_RST_EN		BIT(30)
#define CNNT_SDMMC_FORCE_NORST		BIT(31)

#define CNNT_EMAC_USELESS_CLK_EN	BIT(0)
#define CNNT_EMAC_RST_EN		BIT(1)
#define CNNT_EMAC_FORCE_NORST		BIT(2)

#define CNNT_EMAC_REF_CLK_SEL		GENMASK(1, 0)
#define CNNT_EMAC_REF_CLK_EN		BIT(2)
#define CNNT_EMAC_REF_CLK_DIV		GENMASK(15, 8)

#define CNNT_EMAC_PTP_REF_CLK_EN	BIT(0)
#define CNNT_EMAC_PTP_REF_CLK_SEL	BIT(1)

#define CNNT_EMAC_RMII_PAD_CLK_EN	BIT(1)
#define CNNT_EMAC_RMII_CLK_EN		BIT(1)
#define CNNT_EMAC_RMII_PAD_OUT_CLK_EN	BIT(2)
#define CNNT_EMAC_RX_PAD_CLK_EN		BIT(0)
#define CNNT_EMAC_RX_CLK_SEL		BIT(2)
#define CNNT_EMAC_RX_180_CLK_EN		BIT(3)
#define CNNT_EMAC_TX_180_CLK_EN		BIT(3)

#define CNNT_PHY_INTF_SEL		GENMASK(4, 2)
#define CNNT_PHY_INTF_RGMII		1
#define CNNT_GMAC_MEM_CLK_FORCE_ON	BIT(5)
#define CNNT_GMAC_APB_POSTW_EN		BIT(11)

struct esp32s31_clk_priv {
	void __iomem *hp;
	void __iomem *hp_sys;
	void __iomem *cnnt;
	spinlock_t lock;
	struct clk_hw_onecell_data *onecell;
	bool sdmmc_initialized;
	bool emac_initialized;
	bool uart_initialized[3];
	bool wdt1_initialized;
};

enum esp32s31_clk_kind {
	ESP32S31_CLK_KIND_FIXED,
	ESP32S31_CLK_KIND_SDMMC_BIU,
	ESP32S31_CLK_KIND_SDMMC_CIU,
	ESP32S31_CLK_KIND_EMAC_BUS,
	ESP32S31_CLK_KIND_EMAC_PTP,
	ESP32S31_CLK_KIND_EMAC_TXC,
	ESP32S31_CLK_KIND_UART0,
	ESP32S31_CLK_KIND_UART1,
	ESP32S31_CLK_KIND_UART2,
	ESP32S31_CLK_KIND_WDT1,
};

struct esp32s31_clk {
	struct clk_hw hw;
	struct esp32s31_clk_priv *priv;
	enum esp32s31_clk_kind kind;
	unsigned long fixed_rate;
};

#define to_esp32s31_clk(_hw) container_of(_hw, struct esp32s31_clk, hw)

static void esp32s31_rmw(void __iomem *base, u32 off, u32 clr, u32 set)
{
	u32 val = readl(base + off);

	val &= ~clr;
	val |= set;
	writel(val, base + off);
}

static void esp32s31_sdmmc_prepare(struct esp32s31_clk_priv *priv)
{
	u32 val;

	esp32s31_rmw(priv->cnnt, CNNT_CLK_EN, 0, CNNT_SYS_CLK_EN);
	esp32s31_rmw(priv->cnnt, CNNT_SYS_SDMMC_MEM_LP_CTRL,
		     CNNT_MEM_LP_EN, CNNT_MEM_LP_FORCE_CTRL);

	esp32s31_rmw(priv->hp, HP_SDIO_HOST_CTRL0,
		     HP_SDIO_LS_CLK_SRC_SEL, HP_SDMMC_SYS_CLK_EN |
		     HP_SDIO_LS_CLK_EN | FIELD_PREP(HP_SDIO_LS_CLK_DIV_NUM, 7));

	val = FIELD_PREP(HP_SDIO_EDGE_H, 3) |
	      FIELD_PREP(HP_SDIO_EDGE_N, 7) |
	      FIELD_PREP(HP_SDIO_EDGE_L, 7) |
	      FIELD_PREP(HP_SDIO_DRV_CLK_EDGE_SEL, 1) |
	      HP_SDIO_SLF_CLK_EN | HP_SDIO_DRV_CLK_EN | HP_SDIO_SAM_CLK_EN;
	writel(val | HP_SDIO_EDGE_UPDATE, priv->hp + HP_SDIO_HOST_FUNC_CTRL0);
	writel(val, priv->hp + HP_SDIO_HOST_FUNC_CTRL0);

	esp32s31_rmw(priv->cnnt, CNNT_SYS_HP_SDMMC_CTRL, 0,
		     CNNT_SDMMC_AHB2AXI_POST_WRITE_EN | CNNT_SDMMC_FORCE_NORST);
	esp32s31_rmw(priv->cnnt, CNNT_SYS_HP_SDMMC_CTRL, 0, CNNT_SDMMC_RST_EN);
	esp32s31_rmw(priv->cnnt, CNNT_SYS_HP_SDMMC_CTRL, CNNT_SDMMC_RST_EN, 0);
}

static void esp32s31_emac_prepare(struct esp32s31_clk_priv *priv)
{
	esp32s31_rmw(priv->cnnt, CNNT_CLK_EN, 0, CNNT_SYS_CLK_EN);
	esp32s31_rmw(priv->cnnt, CNNT_SYS_GMAC_MEM_LP_CTRL,
		     CNNT_MEM_LP_EN, CNNT_MEM_LP_FORCE_CTRL);
	esp32s31_rmw(priv->hp, HP_EMAC_CTRL0, 0, HP_EMAC_SYS_CLK_EN);
	esp32s31_rmw(priv->cnnt, CNNT_SYS_GMAC_CTRL0,
		     CNNT_PHY_INTF_SEL,
		     FIELD_PREP(CNNT_PHY_INTF_SEL, CNNT_PHY_INTF_RGMII) |
		     CNNT_GMAC_MEM_CLK_FORCE_ON | CNNT_GMAC_APB_POSTW_EN);
	esp32s31_rmw(priv->cnnt, CNNT_SYS_HP_EMAC_CTRL, 0,
		     CNNT_EMAC_USELESS_CLK_EN | CNNT_EMAC_FORCE_NORST);
	esp32s31_rmw(priv->cnnt, CNNT_SYS_HP_EMAC_CTRL, 0, CNNT_EMAC_RST_EN);
	esp32s31_rmw(priv->cnnt, CNNT_SYS_HP_EMAC_CTRL, CNNT_EMAC_RST_EN, 0);

	/* IDF's native RGMII clock path: MPLL/4 TXC and pad-sourced RXC. */
	esp32s31_rmw(priv->cnnt, CNNT_SYS_HP_EMAC_REF_CTRL,
		     CNNT_EMAC_REF_CLK_SEL | CNNT_EMAC_REF_CLK_DIV,
		     CNNT_EMAC_REF_CLK_EN |
		     FIELD_PREP(CNNT_EMAC_REF_CLK_DIV, 3));
	esp32s31_rmw(priv->cnnt, CNNT_SYS_HP_EMAC_RMII_PAD_CTRL,
		     CNNT_EMAC_RMII_PAD_CLK_EN, 0);
	esp32s31_rmw(priv->cnnt, CNNT_SYS_HP_EMAC_RMII_CTRL,
		     CNNT_EMAC_RMII_CLK_EN,
		     CNNT_EMAC_RMII_PAD_OUT_CLK_EN);
	esp32s31_rmw(priv->cnnt, CNNT_SYS_HP_EMAC_RX_CTRL, 0,
		     CNNT_EMAC_RX_PAD_CLK_EN | CNNT_EMAC_RX_CLK_SEL |
		     CNNT_EMAC_RX_180_CLK_EN);
	esp32s31_rmw(priv->cnnt, CNNT_SYS_HP_EMAC_TX_CTRL, 0,
		     CNNT_EMAC_TX_180_CLK_EN);
	esp32s31_rmw(priv->cnnt, CNNT_SYS_HP_EMAC_PTP_CTRL,
		     CNNT_EMAC_PTP_REF_CLK_SEL, CNNT_EMAC_PTP_REF_CLK_EN);
}

static void esp32s31_uart_prepare(struct esp32s31_clk_priv *priv,
				  unsigned int port)
{
	u32 reg = HP_UART0_CTRL0 + port * HP_UART_CTRL_STRIDE;
	u32 mem_reg = HP_SYS_UART_MEM_LP_CTRL0 +
		      port * HP_SYS_UART_MEM_LP_STRIDE;
	u32 mask = HP_UART_SYS_CLK_EN | HP_UART_APB_CLK_EN |
		   HP_UART_CORE_RST_EN | HP_UART_APB_RST_EN |
		   HP_UART_FORCE_NORST | HP_UART_CLK_SRC_SEL |
		   HP_UART_CLK_EN | HP_UART_SCLK_DIV_NUM |
		   HP_UART_SCLK_DIV_NUMERATOR |
		   HP_UART_SCLK_DIV_DENOMINATOR;
	u32 val = HP_UART_SYS_CLK_EN | HP_UART_APB_CLK_EN |
		  HP_UART_FORCE_NORST | HP_UART_CLK_EN;

	/* XTAL source, integer divide by one, then reset the APB register bank. */
	esp32s31_rmw(priv->hp, reg, mask, val);
	esp32s31_rmw(priv->hp, reg, 0, HP_UART_APB_RST_EN);
	esp32s31_rmw(priv->hp, reg, HP_UART_APB_RST_EN, 0);

	/* UART1/2 FIFO SRAM powers up disabled; force it on while in use. */
	esp32s31_rmw(priv->hp_sys, mem_reg,
		     HP_SYS_UART_MEM_LP_EN | HP_SYS_UART_MEM_FORCE_CTRL,
		     HP_SYS_UART_MEM_FORCE_CTRL);
}

static void esp32s31_wdt1_prepare(struct esp32s31_clk_priv *priv)
{
	/* IDF default watchdog source is XTAL (source selector 0). */
	esp32s31_rmw(priv->hp, HP_TIMERGRP1_CTRL0,
		     HP_TIMERGRP_RST_EN | HP_TIMERGRP_WDT_SRC_SEL,
		     HP_TIMERGRP_APB_CLK_EN | HP_TIMERGRP_FORCE_NORST |
		     HP_TIMERGRP_WDT_CLK_EN);
}

static int esp32s31_clk_prepare(struct clk_hw *hw)
{
	struct esp32s31_clk *clk = to_esp32s31_clk(hw);
	struct esp32s31_clk_priv *priv = clk->priv;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	switch (clk->kind) {
	case ESP32S31_CLK_KIND_SDMMC_BIU:
	case ESP32S31_CLK_KIND_SDMMC_CIU:
		if (!priv->sdmmc_initialized) {
			esp32s31_sdmmc_prepare(priv);
			priv->sdmmc_initialized = true;
		}
		break;
	case ESP32S31_CLK_KIND_EMAC_BUS:
		if (!priv->emac_initialized) {
			esp32s31_emac_prepare(priv);
			priv->emac_initialized = true;
		}
		break;
	case ESP32S31_CLK_KIND_EMAC_PTP:
		if (!priv->emac_initialized) {
			esp32s31_emac_prepare(priv);
			priv->emac_initialized = true;
		}
		esp32s31_rmw(priv->cnnt, CNNT_SYS_HP_EMAC_PTP_CTRL,
			     CNNT_EMAC_PTP_REF_CLK_SEL, CNNT_EMAC_PTP_REF_CLK_EN);
		break;
	case ESP32S31_CLK_KIND_EMAC_TXC:
		if (!priv->emac_initialized) {
			esp32s31_emac_prepare(priv);
			priv->emac_initialized = true;
		}
		esp32s31_rmw(priv->cnnt, CNNT_SYS_HP_EMAC_REF_CTRL,
			     CNNT_EMAC_REF_CLK_SEL, CNNT_EMAC_REF_CLK_EN);
		break;
	case ESP32S31_CLK_KIND_UART0:
	case ESP32S31_CLK_KIND_UART1:
	case ESP32S31_CLK_KIND_UART2: {
		unsigned int port = clk->kind - ESP32S31_CLK_KIND_UART0;

		if (!priv->uart_initialized[port]) {
			esp32s31_uart_prepare(priv, port);
			priv->uart_initialized[port] = true;
		}
		break;
	}
	case ESP32S31_CLK_KIND_WDT1:
		if (!priv->wdt1_initialized) {
			esp32s31_wdt1_prepare(priv);
			priv->wdt1_initialized = true;
		}
		break;
	case ESP32S31_CLK_KIND_FIXED:
		break;
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static void esp32s31_clk_unprepare(struct clk_hw *hw)
{
	/*
	 * Leave S31 peripheral gates and SRAM power forced on for now.  The
	 * current port has no genpd/runtime-PM sequencing.
	 */
}

static unsigned long esp32s31_clk_recalc_rate(struct clk_hw *hw,
					      unsigned long parent_rate)
{
	struct esp32s31_clk *clk = to_esp32s31_clk(hw);
	u32 div;

	if (clk->kind != ESP32S31_CLK_KIND_EMAC_TXC)
		return clk->fixed_rate;

	div = FIELD_GET(CNNT_EMAC_REF_CLK_DIV,
			readl(clk->priv->cnnt + CNNT_SYS_HP_EMAC_REF_CTRL));

	return ESP32S31_MPLL_RATE / (div + 1);
}

static long esp32s31_txc_round_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long *parent_rate)
{
	if (rate >= 125000000)
		return 125000000;
	if (rate >= 25000000)
		return 25000000;

	return 2500000;
}

static int esp32s31_txc_set_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long parent_rate)
{
	struct esp32s31_clk *clk = to_esp32s31_clk(hw);
	struct esp32s31_clk_priv *priv = clk->priv;
	unsigned long flags;
	u32 div;

	switch (rate) {
	case 125000000:
		div = 3;
		break;
	case 25000000:
		div = 19;
		break;
	case 2500000:
		div = 199;
		break;
	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&priv->lock, flags);
	esp32s31_rmw(priv->cnnt, CNNT_SYS_HP_EMAC_REF_CTRL,
		     CNNT_EMAC_REF_CLK_SEL | CNNT_EMAC_REF_CLK_DIV,
		     CNNT_EMAC_REF_CLK_EN | FIELD_PREP(CNNT_EMAC_REF_CLK_DIV, div));
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static const struct clk_ops esp32s31_fixed_ops = {
	.recalc_rate = esp32s31_clk_recalc_rate,
};

static const struct clk_ops esp32s31_gate_ops = {
	.prepare = esp32s31_clk_prepare,
	.unprepare = esp32s31_clk_unprepare,
	.recalc_rate = esp32s31_clk_recalc_rate,
};

static const struct clk_ops esp32s31_txc_ops = {
	.prepare = esp32s31_clk_prepare,
	.unprepare = esp32s31_clk_unprepare,
	.recalc_rate = esp32s31_clk_recalc_rate,
	.round_rate = esp32s31_txc_round_rate,
	.set_rate = esp32s31_txc_set_rate,
};

static int esp32s31_register_clk(struct device *dev,
				 struct esp32s31_clk_priv *priv,
				 unsigned int id, const char *name,
				 enum esp32s31_clk_kind kind,
				 unsigned long rate)
{
	struct clk_init_data init = {};
	struct esp32s31_clk *clk;
	const struct clk_ops *ops;
	int ret;

	clk = devm_kzalloc(dev, sizeof(*clk), GFP_KERNEL);
	if (!clk)
		return -ENOMEM;

	switch (kind) {
	case ESP32S31_CLK_KIND_FIXED:
		ops = &esp32s31_fixed_ops;
		break;
	case ESP32S31_CLK_KIND_EMAC_TXC:
		ops = &esp32s31_txc_ops;
		break;
	default:
		ops = &esp32s31_gate_ops;
		break;
	}

	init.name = name;
	init.ops = ops;

	clk->hw.init = &init;
	clk->priv = priv;
	clk->kind = kind;
	clk->fixed_rate = rate;

	ret = devm_clk_hw_register(dev, &clk->hw);
	if (ret)
		return ret;

	priv->onecell->hws[id] = &clk->hw;

	return 0;
}

static int esp32s31_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp32s31_clk_priv *priv;
	struct clk_hw_onecell_data *onecell;
	struct resource *res;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	onecell = devm_kzalloc(dev, struct_size(onecell, hws, ESP32S31_CLK_NR),
			       GFP_KERNEL);
	if (!onecell)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "hp-sys-clkrst");
	if (!res)
		return -EINVAL;
	priv->hp = devm_ioremap(dev, res->start, resource_size(res));
	if (!priv->hp)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "hp-system");
	if (!res)
		return -EINVAL;
	priv->hp_sys = devm_ioremap(dev, res->start, resource_size(res));
	if (!priv->hp_sys)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cnnt-sys");
	if (!res)
		return -EINVAL;
	priv->cnnt = devm_ioremap(dev, res->start, resource_size(res));
	if (!priv->cnnt)
		return -ENOMEM;

	spin_lock_init(&priv->lock);
	priv->onecell = onecell;
	onecell->num = ESP32S31_CLK_NR;

	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_XTAL, "xtal",
				    ESP32S31_CLK_KIND_FIXED, ESP32S31_XTAL_RATE);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_SYS, "sys",
				    ESP32S31_CLK_KIND_FIXED, ESP32S31_SYS_RATE);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_MPLL, "mpll",
				    ESP32S31_CLK_KIND_FIXED, ESP32S31_MPLL_RATE);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_SDMMC_BIU,
				    "sdmmc-biu", ESP32S31_CLK_KIND_SDMMC_BIU,
				    ESP32S31_SYS_RATE);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_SDMMC_CIU,
				    "sdmmc-ciu", ESP32S31_CLK_KIND_SDMMC_CIU,
				    ESP32S31_SDMMC_CIU_RATE);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_EMAC_STMMACETH,
				    "emac-stmmaceth", ESP32S31_CLK_KIND_EMAC_BUS,
				    ESP32S31_SYS_RATE);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_EMAC_PCLK,
				    "emac-pclk", ESP32S31_CLK_KIND_EMAC_BUS,
				    ESP32S31_SYS_RATE);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_EMAC_PTP_REF,
				    "emac-ptp-ref", ESP32S31_CLK_KIND_EMAC_PTP,
				    ESP32S31_XTAL_RATE);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_EMAC_RGMII_TXC,
				    "emac-rgmii-txc", ESP32S31_CLK_KIND_EMAC_TXC,
				    125000000);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_UART0,
				    "uart0", ESP32S31_CLK_KIND_UART0,
				    ESP32S31_XTAL_RATE);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_UART1,
				    "uart1", ESP32S31_CLK_KIND_UART1,
				    ESP32S31_XTAL_RATE);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_UART2,
				    "uart2", ESP32S31_CLK_KIND_UART2,
				    ESP32S31_XTAL_RATE);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_WDT1,
				    "wdt1", ESP32S31_CLK_KIND_WDT1,
				    ESP32S31_XTAL_RATE);
	if (ret)
		return ret;

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, onecell);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, priv);

	return 0;
}

static const struct of_device_id esp32s31_clk_of_match[] = {
	{ .compatible = "espressif,esp32s31-clock" },
	{ }
};

static struct platform_driver esp32s31_clk_driver = {
	.probe = esp32s31_clk_probe,
	.driver = {
		.name = "esp32s31-clock",
		.of_match_table = esp32s31_clk_of_match,
	},
};
builtin_platform_driver(esp32s31_clk_driver);

MODULE_DESCRIPTION("ESP32-S31 clock/reset controller");
MODULE_LICENSE("GPL");
