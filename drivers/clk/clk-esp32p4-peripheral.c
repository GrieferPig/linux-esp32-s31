// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-P4 hw_ver1 peripheral clock/reset subset, using only the XTAL. */
#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/rational.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <dt-bindings/clock/esp32p4-peripheral-clock.h>

#define SOC_CLK1		0x18
#define SOC_CLK2		0x1c
#define SOC_CLK3		0x20
#define EMAC_RMII_CLK_CTRL	0x30
#define SDIO_CLK_SOURCE		0x34
#define SDIO_CLK_EDGES		0x38
#define I2C_CLK10		0x40
#define I2C_I2S_CLK11		0x44
#define I2S_CLK12		0x48
#define I2S_CLK13		0x4c
#define I2S_CLK14		0x50
#define GPSPI_CLK116		0x80
#define TIMER_CLK20		0x94
#define TIMER_CLK21		0x98
#define LEDC_CLK22		0x9c
#define IOMUX_CLK26		0xac
#define HP_RST0			0xc0
#define HP_RST1			0xc4
#define HP_RST2			0xc8

/* TRM ch.12 Reset and Clock: SDMMC_SYS_CLK_EN and SDIO_LS_CLK_*.
 * Use PLL_F160M with a divide-by-eight waveform (sdmmc_ll_set_clock_div()).
 */
#define AHB_PDMA_SYS_EN		BIT(3)
#define HP_AHB_PDMA_RESET	BIT(1)
#define SDMMC_SYS_EN		BIT(14)
#define USB_OTG20_SYS_EN	BIT(16)
#define SDIO_HS_MODE		BIT(22)
#define SDIO_LS_SRC		BIT(23)
#define SDIO_LS_EN		BIT(24)
#define SDIO_EDGE_UPDATE	BIT(8)
#define SDIO_EDGE_L		GENMASK(12, 9)
#define SDIO_EDGE_H		GENMASK(16, 13)
#define SDIO_EDGE_N		GENMASK(20, 17)
#define SDIO_SAMPLE_EDGE	GENMASK(26, 25)
#define SDIO_DRIVE_EDGE		GENMASK(24, 23)
#define SDIO_SELF_EDGE		GENMASK(22, 21)
#define SDIO_EDGE_ENABLE	GENMASK(29, 27)
#define SDIO_CLOCK_DIV		8
#define LP_SDMMC_RESET		BIT(28)

/* TRM ch.12 Reset and Clock and ch.57 EMAC - external RMII reference. */
#define EMAC_SYS_EN		BIT(13)
#define LP_EMAC_RESET		BIT(30)
#define LP_EMAC_PAD_TX		BIT(13)
#define LP_EMAC_PAD_RX		BIT(14)
#define LP_EMAC_PAD_TXRX	BIT(15)
#define RMII_REF_OUT		BIT(24)
#define RMII_CLK_SRC		GENMASK(26, 25)
#define RMII_CLK_EN		BIT(27)
#define RMII_RX_CLK_SRC		BIT(28)
#define RMII_RX_CLK_EN		BIT(29)
#define RMII_RX_CLK_DIV		GENMASK(7, 0)
#define RMII_TX_CLK_SRC		BIT(8)
#define RMII_TX_CLK_EN		BIT(9)
#define RMII_TX_CLK_DIV		GENMASK(17, 10)
#define EMAC_PTP_CLK_SRC	BIT(18)
#define EMAC_PTP_CLK_EN		BIT(19)
#define EMAC_PHY_INTF		GENMASK(4, 2)
#define EMAC_PHY_RMII		4

/* TRM ch.61 Motor Control PWM - MCPWMn clock source, gate and reset. */
#define MCPWM0_APB_EN		BIT(27)
#define MCPWM1_APB_EN		BIT(28)
#define MCPWM0_SRC		GENMASK(1, 0)
#define MCPWM0_CLK_EN		BIT(2)
#define MCPWM1_SRC		GENMASK(12, 11)
#define MCPWM1_CLK_EN		BIT(13)
#define MCPWM0_RESET		BIT(24)
#define MCPWM1_RESET		BIT(25)

/* TRM ch.62 RMT and ch.12 Reset and Clock - RMT_SYS_CLK_EN, RMT_CLK_SRC_SEL. */
#define RMT_SYS_EN		BIT(0)
#define RMT_CLK_SRC		GENMASK(4, 3)
#define RMT_FUNC_EN		BIT(5)
#define RMT_DIV_FIELDS		GENMASK(29, 6)
#define RMT_RESET		BIT(23)

/* TRM ch.60 LED PWM Controller - LEDC clock source, gate and reset. */
#define LEDC_APB_EN		BIT(0)
#define LEDC_SRC		GENMASK(1, 0)
#define LEDC_CLK_EN		BIT(2)
#define LEDC_RESET		BIT(29)

/* TRM ch.12 Reset and Clock - I2S0 RX/TX_CLK_SRC_SEL, DIV_X/Y/Z/N. */
#define I2S0_APB_EN		BIT(14)
#define I2S0_APB_RESET		BIT(4)
#define I2S_RX_EN		BIT(24)
#define I2S_RX_SRC		GENMASK(26, 25)
#define I2S_RX_N		GENMASK(7, 0)
#define I2S_RX_X		GENMASK(16, 8)
#define I2S_RX_Y		GENMASK(25, 17)
#define I2S_RX_Z		GENMASK(8, 0)
#define I2S_RX_YN1		BIT(9)
#define I2S_TX_EN		BIT(10)
#define I2S_TX_SRC		GENMASK(12, 11)
#define I2S_TX_N		GENMASK(20, 13)
#define I2S_TX_X		GENMASK(29, 21)
#define I2S_TX_Y		GENMASK(8, 0)
#define I2S_TX_Z		GENMASK(17, 9)
#define I2S_TX_YN1		BIT(18)
#define I2S_MCLK_TX		BIT(19)
#define I2S_DIV_MAX		511

/* TRM ch.19 Watchdog Timers - TIMERGRP<n>_WDT_SRC_SEL and WDT_CLK_EN. */
#define TIMG0_APB_EN		BIT(21)
#define TIMG1_APB_EN		BIT(22)
#define MWDT0_SRC		GENMASK(29, 28)
#define MWDT0_EN		BIT(30)
#define MWDT1_SRC		GENMASK(27, 26)
#define MWDT1_EN		BIT(28)
/* TRM ch.12 - PCNT_APB_CLK_EN in SOC_CLK_CTRL2. */
#define PCNT_APB_EN		BIT(30)
#define PCNT_APB_RESET		BIT(30)

#define I2C1_APB_EN		BIT(13)
#define I2C1_XTAL_MASK		BIT(26)
#define I2C1_FUNC_EN		BIT(27)
#define I2C1_PREDIV		GENMASK(23, 0)
#define I2C1_RST		BIT(21)

#define GPSPI2_SYS_EN		BIT(0)
#define GPSPI2_APB_EN		BIT(19)
#define GPSPI2_SRC		GENMASK(2, 0)
#define GPSPI2_HS_EN		BIT(3)
#define GPSPI2_HS_DIV		GENMASK(11, 4)
#define GPSPI2_MST_DIV		GENMASK(19, 12)
#define GPSPI2_MST_EN		BIT(20)
#define GPSPI2_RST		BIT(7)

#define IOMUX_APB_EN		BIT(3)
#define IOMUX_FUNC_EN		BIT(9)

struct p4_cc {
	void __iomem *base;
	void __iomem *lp_reset;
	void __iomem *emac_ref;
	void __iomem *emac_ctrl;
	spinlock_t lock;
	struct reset_controller_dev reset;
};

struct p4_clk {
	struct clk_hw hw;
	struct p4_cc *cc;
	unsigned int id;
};

static void p4_update(struct p4_cc *cc, u32 reg, u32 mask, u32 val)
{
	writel((readl(cc->base + reg) & ~mask) | val, cc->base + reg);
}

static void p4_update_external(void __iomem *reg, u32 mask, u32 val)
{
	writel((readl(reg) & ~mask) | val, reg);
}

static void p4_lp_pulse_reset(struct p4_cc *cc, u32 mask)
{
	p4_update_external(cc->lp_reset, mask, mask);
	readl(cc->lp_reset);
	p4_update_external(cc->lp_reset, mask, 0);
}

static int p4_clk_enable(struct clk_hw *hw)
{
	struct p4_clk *clk = container_of(hw, struct p4_clk, hw);
	struct p4_cc *cc = clk->cc;
	unsigned long flags;

	spin_lock_irqsave(&cc->lock, flags);
	switch (clk->id) {
	case ESP32P4_CLK_I2C1:
		p4_update(cc, SOC_CLK2, I2C1_APB_EN, I2C1_APB_EN);
		p4_update(cc, I2C_CLK10, I2C1_FUNC_EN, 0);
		p4_update(cc, I2C_I2S_CLK11, I2C1_PREDIV, 0);
		p4_update(cc, I2C_CLK10, I2C1_XTAL_MASK | I2C1_FUNC_EN,
			  I2C1_FUNC_EN);
		break;
	case ESP32P4_CLK_GPSPI2:
		p4_update(cc, SOC_CLK1, GPSPI2_SYS_EN, GPSPI2_SYS_EN);
		p4_update(cc, SOC_CLK2, GPSPI2_APB_EN, GPSPI2_APB_EN);
		p4_update(cc, GPSPI_CLK116, GPSPI2_HS_EN | GPSPI2_MST_EN, 0);
		/* XTAL / 1 (HS) / 2 (master). Leave SPI3's fields alone. */
		p4_update(cc, GPSPI_CLK116, GPSPI2_SRC | GPSPI2_HS_DIV |
			  GPSPI2_MST_DIV | GPSPI2_HS_EN | GPSPI2_MST_EN,
			  FIELD_PREP(GPSPI2_MST_DIV, 1) | GPSPI2_HS_EN |
			  GPSPI2_MST_EN);
		break;
	case ESP32P4_CLK_EMAC:
		p4_update(cc, SOC_CLK1, EMAC_SYS_EN, EMAC_SYS_EN);
		p4_lp_pulse_reset(cc, LP_EMAC_RESET);
		break;
	case ESP32P4_CLK_EMAC_RMII:
		p4_update_external(cc->emac_ctrl, EMAC_PHY_INTF,
				   FIELD_PREP(EMAC_PHY_INTF, EMAC_PHY_RMII));
		p4_update(cc, EMAC_RMII_CLK_CTRL, RMII_REF_OUT | RMII_CLK_SRC |
			  RMII_RX_CLK_SRC | RMII_CLK_EN | RMII_RX_CLK_EN,
			  RMII_CLK_EN | RMII_RX_CLK_EN);
		p4_update(cc, SDIO_CLK_SOURCE, RMII_TX_CLK_SRC | RMII_TX_CLK_EN,
			  RMII_TX_CLK_EN);
		p4_update_external(cc->emac_ref, LP_EMAC_PAD_TX | LP_EMAC_PAD_RX |
				   LP_EMAC_PAD_TXRX, LP_EMAC_PAD_TXRX);
		break;
	case ESP32P4_CLK_EMAC_PTP:
		/* TRM ch.12 - EMAC_PTP_REF_CLK_SRC_SEL selects XTAL. */
		p4_update(cc, SDIO_CLK_SOURCE, EMAC_PTP_CLK_SRC | EMAC_PTP_CLK_EN,
			  EMAC_PTP_CLK_EN);
		break;
	case ESP32P4_CLK_AHB_GDMA:
		/* TRM ch.12 Reset and Clock - AHB_PDMA_SYS_CLK_EN. */
		p4_update(cc, SOC_CLK1, AHB_PDMA_SYS_EN, AHB_PDMA_SYS_EN);
		break;
	case ESP32P4_CLK_MCPWM0:
		p4_update(cc, SOC_CLK2, MCPWM0_APB_EN, MCPWM0_APB_EN);
		p4_update(cc, TIMER_CLK20, MCPWM0_SRC | MCPWM0_CLK_EN, MCPWM0_CLK_EN);
		break;
	case ESP32P4_CLK_MCPWM1:
		p4_update(cc, SOC_CLK2, MCPWM1_APB_EN, MCPWM1_APB_EN);
		p4_update(cc, TIMER_CLK20, MCPWM1_SRC | MCPWM1_CLK_EN, MCPWM1_CLK_EN);
		break;
	case ESP32P4_CLK_RMT:
		p4_update(cc, SOC_CLK2, RMT_SYS_EN, RMT_SYS_EN);
		p4_update(cc, LEDC_CLK22, RMT_CLK_SRC | RMT_DIV_FIELDS | RMT_FUNC_EN,
			  RMT_FUNC_EN);
		break;
	case ESP32P4_CLK_LEDC:
		p4_update(cc, SOC_CLK3, LEDC_APB_EN, LEDC_APB_EN);
		p4_update(cc, LEDC_CLK22, LEDC_SRC | LEDC_CLK_EN, LEDC_CLK_EN);
		break;
	case ESP32P4_CLK_PCNT:
		p4_update(cc, SOC_CLK2, PCNT_APB_EN, PCNT_APB_EN);
		break;
	case ESP32P4_CLK_MWDT0:
		p4_update(cc, SOC_CLK2, TIMG0_APB_EN, TIMG0_APB_EN);
		p4_update(cc, TIMER_CLK20, MWDT0_SRC | MWDT0_EN, MWDT0_EN);
		break;
	case ESP32P4_CLK_MWDT1:
		p4_update(cc, SOC_CLK2, TIMG1_APB_EN, TIMG1_APB_EN);
		p4_update(cc, TIMER_CLK21, MWDT1_SRC | MWDT1_EN, MWDT1_EN);
		break;
	case ESP32P4_CLK_I2S0:
		p4_update(cc, SOC_CLK2, I2S0_APB_EN, I2S0_APB_EN);
		/* Source encoding zero selects XTAL for both clock domains. */
		p4_update(cc, I2C_I2S_CLK11, I2S_RX_EN | I2S_RX_SRC, I2S_RX_EN);
		p4_update(cc, I2S_CLK13, I2S_TX_EN | I2S_TX_SRC, I2S_TX_EN);
		p4_update(cc, I2S_CLK14, I2S_MCLK_TX, I2S_MCLK_TX);
		break;
	case ESP32P4_CLK_USBHS:
		/* TRM ch.12 Reset and Clock - USB_OTG20_SYS_CLK_EN. */
		p4_update(cc, SOC_CLK1, USB_OTG20_SYS_EN, USB_OTG20_SYS_EN);
		break;
	case ESP32P4_CLK_SDMMC:
		p4_update(cc, SOC_CLK1, SDMMC_SYS_EN, SDMMC_SYS_EN);
		/* LP_CLKRST_HP_SDMMC_EMAC_RST_CTRL, TRM ch.12. */
		p4_lp_pulse_reset(cc, LP_SDMMC_RESET);
		p4_update(cc, SDIO_CLK_SOURCE, SDIO_HS_MODE | SDIO_LS_SRC |
			  SDIO_LS_EN, SDIO_LS_EN);
		p4_update(cc, SDIO_CLK_EDGES, SDIO_EDGE_L | SDIO_EDGE_H | SDIO_EDGE_N |
			  SDIO_EDGE_ENABLE | SDIO_SAMPLE_EDGE | SDIO_DRIVE_EDGE |
			  SDIO_SELF_EDGE,
			  FIELD_PREP(SDIO_EDGE_L, SDIO_CLOCK_DIV - 1) |
			  FIELD_PREP(SDIO_EDGE_H, SDIO_CLOCK_DIV / 2 - 1) |
			  FIELD_PREP(SDIO_EDGE_N, SDIO_CLOCK_DIV - 1) |
			  FIELD_PREP(SDIO_DRIVE_EDGE, 1) | SDIO_EDGE_ENABLE);
		p4_update(cc, SDIO_CLK_EDGES, SDIO_EDGE_UPDATE, SDIO_EDGE_UPDATE);
		p4_update(cc, SDIO_CLK_EDGES, SDIO_EDGE_UPDATE, 0);
		break;
	case ESP32P4_CLK_IOMUX:
		/* Other pads are firmware-owned. Keep their existing source/divider. */
		p4_update(cc, SOC_CLK3, IOMUX_APB_EN, IOMUX_APB_EN);
		p4_update(cc, IOMUX_CLK26, IOMUX_FUNC_EN, IOMUX_FUNC_EN);
		break;
	}
	spin_unlock_irqrestore(&cc->lock, flags);

	return 0;
}

static void p4_clk_disable(struct clk_hw *hw)
{
	struct p4_clk *clk = container_of(hw, struct p4_clk, hw);
	struct p4_cc *cc = clk->cc;
	unsigned long flags;

	spin_lock_irqsave(&cc->lock, flags);
	switch (clk->id) {
	case ESP32P4_CLK_I2C1:
		p4_update(cc, I2C_CLK10, I2C1_FUNC_EN, 0);
		p4_update(cc, SOC_CLK2, I2C1_APB_EN, 0);
		break;
	case ESP32P4_CLK_EMAC:
		p4_update(cc, SOC_CLK1, EMAC_SYS_EN, 0);
		break;
	case ESP32P4_CLK_EMAC_RMII:
		p4_update(cc, SDIO_CLK_SOURCE, RMII_TX_CLK_EN, 0);
		p4_update(cc, EMAC_RMII_CLK_CTRL, RMII_CLK_EN | RMII_RX_CLK_EN, 0);
		p4_update_external(cc->emac_ref, LP_EMAC_PAD_TXRX, 0);
		break;
	case ESP32P4_CLK_EMAC_PTP:
		p4_update(cc, SDIO_CLK_SOURCE, EMAC_PTP_CLK_EN, 0);
		break;
	case ESP32P4_CLK_AHB_GDMA:
		p4_update(cc, SOC_CLK1, AHB_PDMA_SYS_EN, 0);
		break;
	case ESP32P4_CLK_MCPWM0:
		p4_update(cc, TIMER_CLK20, MCPWM0_CLK_EN, 0);
		p4_update(cc, SOC_CLK2, MCPWM0_APB_EN, 0);
		break;
	case ESP32P4_CLK_MCPWM1:
		p4_update(cc, TIMER_CLK20, MCPWM1_CLK_EN, 0);
		p4_update(cc, SOC_CLK2, MCPWM1_APB_EN, 0);
		break;
	case ESP32P4_CLK_RMT:
		p4_update(cc, LEDC_CLK22, RMT_FUNC_EN, 0);
		p4_update(cc, SOC_CLK2, RMT_SYS_EN, 0);
		break;
	case ESP32P4_CLK_LEDC:
		p4_update(cc, LEDC_CLK22, LEDC_CLK_EN, 0);
		p4_update(cc, SOC_CLK3, LEDC_APB_EN, 0);
		break;
	case ESP32P4_CLK_PCNT:
		p4_update(cc, SOC_CLK2, PCNT_APB_EN, 0);
		break;
	case ESP32P4_CLK_MWDT0:
		p4_update(cc, TIMER_CLK20, MWDT0_EN, 0);
		break;
	case ESP32P4_CLK_MWDT1:
		p4_update(cc, TIMER_CLK21, MWDT1_EN, 0);
		break;
	case ESP32P4_CLK_I2S0:
		p4_update(cc, I2S_CLK13, I2S_TX_EN, 0);
		p4_update(cc, I2C_I2S_CLK11, I2S_RX_EN, 0);
		p4_update(cc, SOC_CLK2, I2S0_APB_EN, 0);
		break;
	case ESP32P4_CLK_USBHS:
		p4_update(cc, SOC_CLK1, USB_OTG20_SYS_EN, 0);
		break;
	case ESP32P4_CLK_SDMMC:
		p4_update(cc, SDIO_CLK_EDGES, SDIO_EDGE_ENABLE, 0);
		p4_update(cc, SDIO_CLK_SOURCE, SDIO_LS_EN, 0);
		p4_update(cc, SOC_CLK1, SDMMC_SYS_EN, 0);
		break;
	case ESP32P4_CLK_GPSPI2:
		p4_update(cc, GPSPI_CLK116, GPSPI2_HS_EN | GPSPI2_MST_EN, 0);
		p4_update(cc, SOC_CLK2, GPSPI2_APB_EN, 0);
		p4_update(cc, SOC_CLK1, GPSPI2_SYS_EN, 0);
		break;
	}
	spin_unlock_irqrestore(&cc->lock, flags);
}

static unsigned long p4_clk_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct p4_clk *clk = container_of(hw, struct p4_clk, hw);

	if (clk->id == ESP32P4_CLK_SDMMC)
		return parent_rate / SDIO_CLOCK_DIV;
	return clk->id == ESP32P4_CLK_GPSPI2 ? parent_rate / 2 : parent_rate;
}

static const struct clk_ops p4_clk_ops = {
	.enable = p4_clk_enable,
	.disable = p4_clk_disable,
	.recalc_rate = p4_clk_rate,
};

static unsigned long p4_rmii_rate(struct clk_hw *hw, unsigned long parent)
{
	struct p4_clk *clk = container_of(hw, struct p4_clk, hw);
	u32 div = FIELD_GET(RMII_RX_CLK_DIV, readl(clk->cc->base + SDIO_CLK_SOURCE));

	return parent / (div + 1);
}

static int p4_rmii_determine_rate(struct clk_hw *hw, struct clk_rate_request *req)
{
	unsigned long parent = req->best_parent_rate;

	req->rate = req->rate <= parent / 20 ? parent / 20 : parent / 2;
	return 0;
}

static int p4_rmii_set_rate(struct clk_hw *hw, unsigned long rate,
			   unsigned long parent)
{
	struct p4_clk *clk = container_of(hw, struct p4_clk, hw);
	struct p4_cc *cc = clk->cc;
	unsigned long flags;
	u32 div;

	if (!rate || (rate != parent / 2 && rate != parent / 20))
		return -EINVAL;

	/* Reference clock remains 50 MHz. RX/TX require 25 or 2.5 MHz. */
	div = parent / rate - 1;
	spin_lock_irqsave(&cc->lock, flags);
	p4_update(cc, SDIO_CLK_SOURCE, RMII_RX_CLK_DIV | RMII_TX_CLK_DIV,
		  FIELD_PREP(RMII_RX_CLK_DIV, div) |
		  FIELD_PREP(RMII_TX_CLK_DIV, div));
	spin_unlock_irqrestore(&cc->lock, flags);
	return 0;
}

static const struct clk_ops p4_rmii_clk_ops = {
	.enable = p4_clk_enable,
	.disable = p4_clk_disable,
	.recalc_rate = p4_rmii_rate,
	.determine_rate = p4_rmii_determine_rate,
	.set_rate = p4_rmii_set_rate,
};

static unsigned long p4_i2s_rate(struct clk_hw *hw, unsigned long parent)
{
	struct p4_clk *clk = container_of(hw, struct p4_clk, hw);
	struct p4_cc *cc = clk->cc;
	u32 high = readl(cc->base + I2S_CLK13);
	u32 low = readl(cc->base + I2S_CLK14);
	u32 integer = FIELD_GET(I2S_TX_N, high) ?: 256;
	u32 z = FIELD_GET(I2S_TX_Z, low);
	u32 denom, num;

	integer = max(integer, 2U);
	if (!z)
		return parent / integer;

	denom = (FIELD_GET(I2S_TX_X, high) + 1) * z + FIELD_GET(I2S_TX_Y, low);
	num = low & I2S_TX_YN1 ? denom - z : z;
	return DIV_ROUND_CLOSEST_ULL((u64)parent * denom, integer * denom + num);
}

static int p4_i2s_determine_rate(struct clk_hw *hw, struct clk_rate_request *req)
{
	if (req->rate < DIV_ROUND_UP(req->best_parent_rate, 256) ||
	    req->rate > req->best_parent_rate / 2)
		return -ERANGE;
	return 0;
}

static int p4_i2s_set_rate(struct clk_hw *hw, unsigned long rate,
			  unsigned long parent)
{
	struct p4_clk *clk = container_of(hw, struct p4_clk, hw);
	struct p4_cc *cc = clk->cc;
	unsigned long flags, num, denom;
	u32 integer, x = 0, y = 0, z = 0;
	bool yn1 = false;

	if (!rate)
		return -EINVAL;
	integer = parent / rate;
	if (integer < 2 || integer > 256)
		return -ERANGE;

	num = parent % rate;
	denom = rate;
	if (num) {
		rational_best_approximation(num, denom, I2S_DIV_MAX, I2S_DIV_MAX,
					    &num, &denom);
		if (num == denom)
			integer++;
		else if (num) {
			yn1 = num > denom / 2;
			z = yn1 ? denom - num : num;
			x = denom / z - 1;
			y = denom % z;
		}
	}
	if (integer > 256)
		return -ERANGE;

	spin_lock_irqsave(&cc->lock, flags);
	p4_update(cc, I2S_CLK12, I2S_RX_N | I2S_RX_X | I2S_RX_Y,
		  FIELD_PREP(I2S_RX_N, integer & 0xff) |
		  FIELD_PREP(I2S_RX_X, x) | FIELD_PREP(I2S_RX_Y, y));
	p4_update(cc, I2S_CLK13, I2S_RX_Z | I2S_RX_YN1 | I2S_TX_N | I2S_TX_X,
		  FIELD_PREP(I2S_RX_Z, z) | (yn1 ? I2S_RX_YN1 : 0) |
		  FIELD_PREP(I2S_TX_N, integer & 0xff) | FIELD_PREP(I2S_TX_X, x));
	p4_update(cc, I2S_CLK14, I2S_TX_Y | I2S_TX_Z | I2S_TX_YN1,
		  FIELD_PREP(I2S_TX_Y, y) | FIELD_PREP(I2S_TX_Z, z) |
		  (yn1 ? I2S_TX_YN1 : 0));
	spin_unlock_irqrestore(&cc->lock, flags);
	return 0;
}

static const struct clk_ops p4_i2s_clk_ops = {
	.enable = p4_clk_enable,
	.disable = p4_clk_disable,
	.recalc_rate = p4_i2s_rate,
	.determine_rate = p4_i2s_determine_rate,
	.set_rate = p4_i2s_set_rate,
};

static int p4_reset(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct p4_cc *cc = container_of(rcdev, struct p4_cc, reset);
	unsigned long flags;
	u32 reg, bit;

	switch (id) {
	case ESP32P4_RST_I2C1:
		reg = HP_RST1;
		bit = I2C1_RST;
		break;
	case ESP32P4_RST_GPSPI2:
		reg = HP_RST2;
		bit = GPSPI2_RST;
		break;
	case ESP32P4_RST_AHB_GDMA:
		reg = HP_RST0;
		bit = HP_AHB_PDMA_RESET;
		break;
	case ESP32P4_RST_MCPWM0:
		reg = HP_RST1;
		bit = MCPWM0_RESET;
		break;
	case ESP32P4_RST_MCPWM1:
		reg = HP_RST1;
		bit = MCPWM1_RESET;
		break;
	case ESP32P4_RST_RMT:
		reg = HP_RST1;
		bit = RMT_RESET;
		break;
	case ESP32P4_RST_LEDC:
		reg = HP_RST1;
		bit = LEDC_RESET;
		break;
	case ESP32P4_RST_PCNT:
		reg = HP_RST1;
		bit = PCNT_APB_RESET;
		break;
	case ESP32P4_RST_I2S0:
		reg = HP_RST2;
		bit = I2S0_APB_RESET;
		break;
	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&cc->lock, flags);
	p4_update(cc, reg, bit, bit);
	readl(cc->base + reg);
	udelay(1);
	p4_update(cc, reg, bit, 0);
	readl(cc->base + reg);
	spin_unlock_irqrestore(&cc->lock, flags);

	return 0;
}

static const struct reset_control_ops p4_reset_ops = {
	.reset = p4_reset,
};

static int p4_peripheral_probe(struct platform_device *pdev)
{
	static const char * const names[] = {
		"p4-i2c1", "p4-gpspi2", "p4-iomux", "p4-sdmmc", "p4-usbhs",
		"p4-emac", "p4-emac-rmii", "p4-emac-ptp", "p4-ahb-gdma",
		"p4-i2s0",
		"p4-mwdt0",
		"p4-mwdt1",
		"p4-pcnt",
		"p4-ledc",
		"p4-mcpwm0",
		"p4-mcpwm1",
		"p4-rmt",
	};
	struct device *dev = &pdev->dev;
	struct clk_hw_onecell_data *data;
	struct p4_cc *cc;
	int i, ret;

	cc = devm_kzalloc(dev, sizeof(*cc), GFP_KERNEL);
	if (!cc)
		return -ENOMEM;
	cc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cc->base))
		return PTR_ERR(cc->base);
	if (platform_get_resource(pdev, IORESOURCE_MEM, 1)) {
		cc->lp_reset = devm_platform_ioremap_resource(pdev, 1);
		if (IS_ERR(cc->lp_reset))
			return PTR_ERR(cc->lp_reset);
	}
	if (platform_get_resource(pdev, IORESOURCE_MEM, 2)) {
		cc->emac_ref = devm_platform_ioremap_resource(pdev, 2);
		if (IS_ERR(cc->emac_ref))
			return PTR_ERR(cc->emac_ref);
		cc->emac_ctrl = devm_platform_ioremap_resource(pdev, 3);
		if (IS_ERR(cc->emac_ctrl))
			return PTR_ERR(cc->emac_ctrl);
	}
	spin_lock_init(&cc->lock);

	data = devm_kzalloc(dev, struct_size(data, hws, ESP32P4_CLK_NR),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->num = ESP32P4_CLK_NR;
	for (i = 0; i < data->num; i++) {
		const struct clk_parent_data parent = {
			.index = i == ESP32P4_CLK_EMAC_RMII ? 2 :
				 i == ESP32P4_CLK_SDMMC ? 1 : 0,
		};
		const struct clk_init_data init = {
			.name = names[i],
			.ops = i == ESP32P4_CLK_EMAC_RMII ? &p4_rmii_clk_ops :
			       i == ESP32P4_CLK_I2S0 ? &p4_i2s_clk_ops : &p4_clk_ops,
			.parent_data = &parent,
			.num_parents = (i == ESP32P4_CLK_USBHS ||
					i == ESP32P4_CLK_EMAC ||
					i == ESP32P4_CLK_AHB_GDMA ||
					i == ESP32P4_CLK_PCNT) ? 0 : 1,
		};
		struct p4_clk *clk;

		if (((i == ESP32P4_CLK_SDMMC || i == ESP32P4_CLK_EMAC) &&
		     !cc->lp_reset) || (i == ESP32P4_CLK_EMAC_RMII && !cc->emac_ref)) {
			data->hws[i] = ERR_PTR(-ENOENT);
			continue;
		}
		clk = devm_kzalloc(dev, sizeof(*clk), GFP_KERNEL);
		if (!clk)
			return -ENOMEM;
		clk->cc = cc;
		clk->id = i;
		clk->hw.init = &init;
		ret = devm_clk_hw_register(dev, &clk->hw);
		if (ret)
			return ret;
		data->hws[i] = &clk->hw;
	}

	cc->reset.owner = THIS_MODULE;
	cc->reset.ops = &p4_reset_ops;
	cc->reset.of_node = dev->of_node;
	cc->reset.nr_resets = ESP32P4_RST_NR;
	ret = devm_reset_controller_register(dev, &cc->reset);
	if (ret)
		return ret;
	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, data);
}

static const struct of_device_id p4_peripheral_of_match[] = {
	{ .compatible = "espressif,esp32p4-rev1-peripheral-clock" },
	{ }
};
MODULE_DEVICE_TABLE(of, p4_peripheral_of_match);

static struct platform_driver p4_peripheral_driver = {
	.probe = p4_peripheral_probe,
	.driver = {
		.name = "esp32p4-peripheral-clock",
		.of_match_table = p4_peripheral_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(p4_peripheral_driver);
