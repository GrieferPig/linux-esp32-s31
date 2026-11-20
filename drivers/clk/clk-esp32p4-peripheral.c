// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-P4 hw_ver1 peripheral clock/reset subset, using only the XTAL. */
#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/mfd/syscon.h>
#include <linux/rational.h>
#include <linux/regmap.h>
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
#define I2S_CLK15		0x54
#define I2S_CLK16		0x58
#define I2S_CLK17		0x5c
#define I2S_CLK18		0x60
#define I2S_CLK19		0x64
#define TWAI_CLK115		0x7c
#define GPSPI_CLK116		0x80
#define GPSPI_CLK117		0x84
#define TIMER_CLK20		0x94
#define TIMER_CLK21		0x98
#define LEDC_CLK22		0x9c
#define CRYPTO_CLK25		0xa8
#define IOMUX_CLK26		0xac
#define HP_RST0			0xc0
#define HP_RST1			0xc4
#define HP_RST2			0xc8

/* TRM ch.12 Reset and Clock: SDMMC_SYS_CLK_EN and SDIO_LS_CLK_*.
 * Use PLL_F160M with a divide-by-eight waveform (sdmmc_ll_set_clock_div()).
 */
#define AHB_PDMA_SYS_EN		BIT(3)
#define HP_AHB_PDMA_RESET	BIT(1)
#define AXI_PDMA_SYS_EN		BIT(4)
#define HP_AXI_PDMA_RESET	BIT(2)
/* TRM ch.12 Reset and Clock — DMA2D_SYS_CLK_EN and RST_EN_DMA2D. */
#define DMA2D_SYS_EN		BIT(6)
#define HP_DMA2D_RESET		BIT(31)
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

/* TRM ch.12 - TWAI<n>_APB_CLK_EN, CLK_SRC_SEL, CLK_EN and RST_EN. */
#define TWAI_APB_EN(n)		BIT(24 + (n))
#define TWAI_CLK_SRC(n)		BIT(24 + 2 * (n))
#define TWAI_CLK_EN(n)		BIT(25 + 2 * (n))
#define TWAI_RESET(n)		BIT(26 + (n))

/* TRM ch.61 Motor Control PWM - MCPWMn clock source, gate and reset. */
#define MCPWM0_APB_EN		BIT(27)
#define MCPWM1_APB_EN		BIT(28)
#define MCPWM0_SRC		GENMASK(1, 0)
#define MCPWM0_CLK_EN		BIT(2)
#define MCPWM1_SRC		GENMASK(12, 11)
#define MCPWM1_CLK_EN		BIT(13)
#define MCPWM0_RESET		BIT(24)
#define MCPWM1_RESET		BIT(25)

/*
 * TRM ch.12 Reset and Clock - PERI_CLK_CTRL25 and HP_RST_EN2.
 * TRM ch.28 AES and ch.32 SHA require the shared engines out of reset.
 */
#define AES_SHA_CLK_SRC		GENMASK(13, 12)
#define AES_SHA_CLK_ENABLE	(BIT(14) | BIT(20))
#define AES_SHA_RESET		(BIT(14) | GENMASK(20, 16) | BIT(23))
/* TRM ch.31 RSA / ch.29 ECC - clear the dependent reset lines as well. */
#define RSA_CLK_ENABLE		BIT(18)
#define RSA_RESET		(BIT(14) | BIT(17) | BIT(20) | BIT(21))
#define ECC_CLK_ENABLE		BIT(16)
#define ECC_RESET		(BIT(14) | BIT(20) | BIT(22))

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

/* TRM ch.12 Reset and Clock — I2S<n> APB gates and PERI_CLK_CTRL11–19. */
#define I2S_APB_EN(port)	BIT(14 + (port))
#define I2S_APB_RESET(port)	BIT(4 + (port))
#define I2S_PAD_MCLK_EN(port)	BIT(12 - (port))
#define I2S_DIV_MAX		511

struct p4_i2s_field {
	u8 reg;
	u8 shift;
};

struct p4_i2s_div {
	struct p4_i2s_field gate;
	struct p4_i2s_field n;
	struct p4_i2s_field x;
	struct p4_i2s_field y;
	struct p4_i2s_field z;
	struct p4_i2s_field yn1;
};

struct p4_i2s_layout {
	struct p4_i2s_div rx;
	struct p4_i2s_div tx;
	struct p4_i2s_field mclk;
	unsigned int port;
};

static const struct p4_i2s_layout p4_i2s_layouts[] = {
	[0] = {
		.rx = { .gate = { I2C_I2S_CLK11, 24 }, .n = { I2S_CLK12, 0 },
			.x = { I2S_CLK12, 8 }, .y = { I2S_CLK12, 17 },
			.z = { I2S_CLK13, 0 }, .yn1 = { I2S_CLK13, 9 } },
		.tx = { .gate = { I2S_CLK13, 10 }, .n = { I2S_CLK13, 13 },
			.x = { I2S_CLK13, 21 }, .y = { I2S_CLK14, 0 },
			.z = { I2S_CLK14, 9 }, .yn1 = { I2S_CLK14, 18 } },
		.mclk = { I2S_CLK14, 19 },
		.port = 0,
	},
	[1] = {
		.rx = { .gate = { I2S_CLK14, 20 }, .n = { I2S_CLK14, 23 },
			.x = { I2S_CLK15, 0 }, .y = { I2S_CLK15, 9 },
			.z = { I2S_CLK15, 18 }, .yn1 = { I2S_CLK15, 27 } },
		.tx = { .gate = { I2S_CLK15, 28 }, .n = { I2S_CLK16, 0 },
			.x = { I2S_CLK16, 8 }, .y = { I2S_CLK16, 17 },
			.z = { I2S_CLK17, 0 }, .yn1 = { I2S_CLK17, 9 } },
		.mclk = { I2S_CLK17, 10 },
		.port = 1,
	},
	[2] = {
		.rx = { .gate = { I2S_CLK17, 11 }, .n = { I2S_CLK17, 14 },
			.x = { I2S_CLK17, 22 }, .y = { I2S_CLK18, 0 },
			.z = { I2S_CLK18, 9 }, .yn1 = { I2S_CLK18, 18 } },
		.tx = { .gate = { I2S_CLK18, 19 }, .n = { I2S_CLK18, 22 },
			.x = { I2S_CLK19, 0 }, .y = { I2S_CLK19, 9 },
			.z = { I2S_CLK19, 18 }, .yn1 = { I2S_CLK19, 27 } },
		.mclk = { I2S_CLK19, 28 },
		.port = 2,
	},
};

static const struct p4_i2s_layout *p4_i2s_layout(unsigned int id)
{
	switch (id) {
	case ESP32P4_CLK_I2S0:
		return &p4_i2s_layouts[0];
	case ESP32P4_CLK_I2S1:
		return &p4_i2s_layouts[1];
	case ESP32P4_CLK_I2S2:
		return &p4_i2s_layouts[2];
	default:
		return NULL;
	}
}

/* TRM ch.12 Reset and Clock - TIMERGRP<n>_T<m>_SRC_SEL and CLK_EN. */
#define GPTIMER0_SRC		(GENMASK(23, 22) | GENMASK(26, 25))
#define GPTIMER0_ENABLE		(BIT(24) | BIT(27))
#define GPTIMER1_SRC		(GENMASK(21, 20) | GENMASK(24, 23))
#define GPTIMER1_ENABLE		(BIT(22) | BIT(25))

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

/* TRM ch.12 Reset and Clock — I2C0/I2C1 APB gates and PERI_CLK_CTRL10. */
#define I2C0_APB_EN		BIT(12)
#define I2C0_CLK_SRC		BIT(0)
#define I2C0_FUNC_EN		BIT(1)
#define I2C0_PREDIV		GENMASK(25, 2)
#define I2C0_RST		BIT(22)
#define I2C1_APB_EN		BIT(13)
#define I2C1_XTAL_MASK		BIT(26)
#define I2C1_FUNC_EN		BIT(27)
#define I2C1_PREDIV		GENMASK(23, 0)
#define I2C1_RST		BIT(21)

/* TRM ch.12 Reset and Clock — LPPERI_LP_I2C_CLK_SEL and CK_EN_LP_I2C. */
#define LP_PERI_CLOCK_EN	0x00
#define LP_PERI_CLOCK_SEL	0x04
#define LP_PERI_RESET_EN	0x08
#define LP_I2C_XTAL_D2_SEL	BIT(28)
#define LP_I2C_CLK_SEL_MASK	GENMASK(29, 28)
#define LP_I2C_CLOCK_EN		BIT(28)
#define LP_I2C_RESET_EN		BIT(27)
/* TRM ch.12 — LP_FAST_CLK_SEL, CK_EN_LP_SPI and RST_EN_LP_SPI. */
#define LP_FAST_CLK_SEL		GENMASK(3, 2)
#define LP_FAST_CLK_XTAL	1
#define LP_SPI_CLOCK_EN		BIT(22)
#define LP_SPI_RESET_EN		BIT(23)

/* TRM ch.12 — GPSPI2/3 gates and PERI_CLK_CTRL116/117 source/divider fields. */
#define GPSPI2_SYS_EN		BIT(0)
#define GPSPI2_APB_EN		BIT(19)
#define GPSPI2_SRC		GENMASK(2, 0)
#define GPSPI2_HS_EN		BIT(3)
#define GPSPI2_HS_DIV		GENMASK(11, 4)
#define GPSPI2_MST_DIV		GENMASK(19, 12)
#define GPSPI2_MST_EN		BIT(20)
#define GPSPI2_RST		BIT(7)
#define GPSPI3_SYS_EN		BIT(1)
#define GPSPI3_APB_EN		BIT(20)
#define GPSPI3_SRC		GENMASK(23, 21)
#define GPSPI3_HS_EN		BIT(24)
#define GPSPI3_HS_DIV		GENMASK(7, 0)
#define GPSPI3_MST_DIV		GENMASK(15, 8)
#define GPSPI3_MST_EN		BIT(16)
#define GPSPI3_RST		BIT(8)

#define IOMUX_APB_EN		BIT(3)
#define IOMUX_FUNC_EN		BIT(9)

struct p4_cc {
	void __iomem *base;
	void __iomem *lp_reset;
	void __iomem *lp_hp_clk;
	void __iomem *lp_fast_clksel;
	void __iomem *emac_ctrl;
	struct regmap *lpperi;
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

static void p4_i2s_write(struct p4_cc *cc, struct p4_i2s_field field,
			 u32 mask, u32 val)
{
	p4_update(cc, field.reg, mask << field.shift, val << field.shift);
}

static u32 p4_i2s_read(struct p4_cc *cc, struct p4_i2s_field field, u32 mask)
{
	return (readl(cc->base + field.reg) >> field.shift) & mask;
}

static void p4_i2s_gate(struct p4_cc *cc, const struct p4_i2s_layout *layout,
			bool enable)
{
	u32 gate = I2S_APB_EN(layout->port);
	u32 pad = I2S_PAD_MCLK_EN(layout->port);

	if (enable)
		p4_update(cc, SOC_CLK2, gate, gate);

	/* The two bits above each enable select the source. Zero is XTAL. */
	p4_i2s_write(cc, layout->rx.gate, enable ? 7 : 1, enable);
	p4_i2s_write(cc, layout->tx.gate, enable ? 7 : 1, enable);

	/* TRM ch.12 — I2S<n>_MST_CLK_SEL and LP_HP_CLK_CTRL pad gate. */
	if (enable)
		p4_i2s_write(cc, layout->mclk, 1, 1);
	if (cc->lp_hp_clk)
		p4_update_external(cc->lp_hp_clk, pad, enable ? pad : 0);
	if (!enable)
		p4_update(cc, SOC_CLK2, gate, 0);
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
	int ret;

	if (clk->id == ESP32P4_CLK_LP_SPI) {
		if (FIELD_GET(LP_FAST_CLK_SEL, readl(cc->lp_fast_clksel)) !=
		    LP_FAST_CLK_XTAL)
			return -EINVAL;
		return regmap_set_bits(cc->lpperi, LP_PERI_CLOCK_EN, LP_SPI_CLOCK_EN);
	}

	if (clk->id == ESP32P4_CLK_LP_I2C) {
		ret = regmap_update_bits(cc->lpperi, LP_PERI_CLOCK_SEL,
					 LP_I2C_CLK_SEL_MASK, LP_I2C_XTAL_D2_SEL);
		if (ret)
			return ret;
		return regmap_set_bits(cc->lpperi, LP_PERI_CLOCK_EN, LP_I2C_CLOCK_EN);
	}
	spin_lock_irqsave(&cc->lock, flags);
	switch (clk->id) {
	case ESP32P4_CLK_I2C0:
		p4_update(cc, SOC_CLK2, I2C0_APB_EN, I2C0_APB_EN);
		p4_update(cc, I2C_CLK10, I2C0_FUNC_EN, 0);
		p4_update(cc, I2C_CLK10, I2C0_CLK_SRC | I2C0_PREDIV |
			  I2C0_FUNC_EN, I2C0_FUNC_EN);
		break;
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
	case ESP32P4_CLK_GPSPI3:
		p4_update(cc, SOC_CLK1, GPSPI3_SYS_EN, GPSPI3_SYS_EN);
		p4_update(cc, SOC_CLK2, GPSPI3_APB_EN, GPSPI3_APB_EN);
		p4_update(cc, GPSPI_CLK116, GPSPI3_HS_EN, 0);
		p4_update(cc, GPSPI_CLK117, GPSPI3_MST_EN, 0);
		p4_update(cc, GPSPI_CLK116, GPSPI3_SRC, 0);
		p4_update(cc, GPSPI_CLK117, GPSPI3_HS_DIV | GPSPI3_MST_DIV,
			  FIELD_PREP(GPSPI3_MST_DIV, 1));
		p4_update(cc, GPSPI_CLK116, GPSPI3_HS_EN, GPSPI3_HS_EN);
		p4_update(cc, GPSPI_CLK117, GPSPI3_MST_EN, GPSPI3_MST_EN);
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
		p4_update_external(cc->lp_hp_clk, LP_EMAC_PAD_TX | LP_EMAC_PAD_RX |
				   LP_EMAC_PAD_TXRX, LP_EMAC_PAD_TXRX);
		break;
	case ESP32P4_CLK_EMAC_PTP:
		/* TRM ch.12 - EMAC_PTP_REF_CLK_SRC_SEL selects XTAL. */
		p4_update(cc, SDIO_CLK_SOURCE, EMAC_PTP_CLK_SRC | EMAC_PTP_CLK_EN,
			  EMAC_PTP_CLK_EN);
		break;
	case ESP32P4_CLK_DMA2D:
		p4_update(cc, SOC_CLK1, DMA2D_SYS_EN, DMA2D_SYS_EN);
		break;
	case ESP32P4_CLK_AXI_GDMA:
		/* TRM ch.12 Reset and Clock — AXI_PDMA_SYS_CLK_EN. */
		p4_update(cc, SOC_CLK1, AXI_PDMA_SYS_EN, AXI_PDMA_SYS_EN);
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
	case ESP32P4_CLK_AES_SHA:
		p4_update(cc, CRYPTO_CLK25, AES_SHA_CLK_SRC | AES_SHA_CLK_ENABLE,
			  AES_SHA_CLK_ENABLE);
		p4_update(cc, HP_RST2, AES_SHA_RESET, 0);
		break;
	case ESP32P4_CLK_RSA:
		p4_update(cc, CRYPTO_CLK25, RSA_CLK_ENABLE, RSA_CLK_ENABLE);
		p4_update(cc, HP_RST2, RSA_RESET, 0);
		break;
	case ESP32P4_CLK_ECC:
		p4_update(cc, CRYPTO_CLK25, ECC_CLK_ENABLE, ECC_CLK_ENABLE);
		p4_update(cc, HP_RST2, ECC_RESET, 0);
		break;
	case ESP32P4_CLK_GPTIMER0:
		p4_update(cc, SOC_CLK2, TIMG0_APB_EN, TIMG0_APB_EN);
		p4_update(cc, TIMER_CLK20, GPTIMER0_SRC | GPTIMER0_ENABLE,
			  GPTIMER0_ENABLE);
		break;
	case ESP32P4_CLK_GPTIMER1:
		p4_update(cc, SOC_CLK2, TIMG1_APB_EN, TIMG1_APB_EN);
		p4_update(cc, TIMER_CLK21, GPTIMER1_SRC | GPTIMER1_ENABLE,
			  GPTIMER1_ENABLE);
		break;
	case ESP32P4_CLK_TWAI0 ... ESP32P4_CLK_TWAI2: {
		unsigned int n = clk->id - ESP32P4_CLK_TWAI0;

		p4_update(cc, SOC_CLK2, TWAI_APB_EN(n), TWAI_APB_EN(n));
		p4_update(cc, TWAI_CLK115, TWAI_CLK_SRC(n) | TWAI_CLK_EN(n),
			  TWAI_CLK_EN(n));
		break;
	}
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
	case ESP32P4_CLK_I2S1:
	case ESP32P4_CLK_I2S2:
		p4_i2s_gate(cc, p4_i2s_layout(clk->id), true);
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
	int ret;

	if (clk->id == ESP32P4_CLK_LP_SPI) {
		ret = regmap_clear_bits(cc->lpperi, LP_PERI_CLOCK_EN, LP_SPI_CLOCK_EN);
		if (ret)
			dev_err(clk->cc->reset.dev, "cannot gate LP SPI clock: %d\n", ret);
		return;
	}

	if (clk->id == ESP32P4_CLK_LP_I2C) {
		ret = regmap_clear_bits(cc->lpperi, LP_PERI_CLOCK_EN, LP_I2C_CLOCK_EN);
		if (ret)
			pr_err("ESP32-P4: failed to disable LP I2C clock: %d\n", ret);
		return;
	}
	spin_lock_irqsave(&cc->lock, flags);
	switch (clk->id) {
	case ESP32P4_CLK_I2C0:
		p4_update(cc, I2C_CLK10, I2C0_FUNC_EN, 0);
		p4_update(cc, SOC_CLK2, I2C0_APB_EN, 0);
		break;
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
		p4_update_external(cc->lp_hp_clk, LP_EMAC_PAD_TXRX, 0);
		break;
	case ESP32P4_CLK_EMAC_PTP:
		p4_update(cc, SDIO_CLK_SOURCE, EMAC_PTP_CLK_EN, 0);
		break;
	case ESP32P4_CLK_DMA2D:
		p4_update(cc, SOC_CLK1, DMA2D_SYS_EN, 0);
		break;
	case ESP32P4_CLK_AXI_GDMA:
		p4_update(cc, SOC_CLK1, AXI_PDMA_SYS_EN, 0);
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
	case ESP32P4_CLK_AES_SHA:
		p4_update(cc, CRYPTO_CLK25, AES_SHA_CLK_ENABLE, 0);
		break;
	case ESP32P4_CLK_RSA:
		p4_update(cc, CRYPTO_CLK25, RSA_CLK_ENABLE, 0);
		break;
	case ESP32P4_CLK_ECC:
		p4_update(cc, CRYPTO_CLK25, ECC_CLK_ENABLE, 0);
		break;
	case ESP32P4_CLK_GPTIMER0:
		/* Leave the shared APB clock available to the watchdog. */
		p4_update(cc, TIMER_CLK20, GPTIMER0_ENABLE, 0);
		break;
	case ESP32P4_CLK_GPTIMER1:
		p4_update(cc, TIMER_CLK21, GPTIMER1_ENABLE, 0);
		break;
	case ESP32P4_CLK_TWAI0 ... ESP32P4_CLK_TWAI2: {
		unsigned int n = clk->id - ESP32P4_CLK_TWAI0;

		p4_update(cc, TWAI_CLK115, TWAI_CLK_EN(n), 0);
		p4_update(cc, SOC_CLK2, TWAI_APB_EN(n), 0);
		break;
	}
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
	case ESP32P4_CLK_I2S1:
	case ESP32P4_CLK_I2S2:
		p4_i2s_gate(cc, p4_i2s_layout(clk->id), false);
		break;
	case ESP32P4_CLK_USBHS:
		p4_update(cc, SOC_CLK1, USB_OTG20_SYS_EN, 0);
		break;
	case ESP32P4_CLK_SDMMC:
		p4_update(cc, SDIO_CLK_EDGES, SDIO_EDGE_ENABLE, 0);
		p4_update(cc, SDIO_CLK_SOURCE, SDIO_LS_EN, 0);
		p4_update(cc, SOC_CLK1, SDMMC_SYS_EN, 0);
		break;
	case ESP32P4_CLK_GPSPI3:
		p4_update(cc, GPSPI_CLK117, GPSPI3_MST_EN, 0);
		p4_update(cc, GPSPI_CLK116, GPSPI3_HS_EN, 0);
		p4_update(cc, SOC_CLK2, GPSPI3_APB_EN, 0);
		p4_update(cc, SOC_CLK1, GPSPI3_SYS_EN, 0);
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
	if (clk->id == ESP32P4_CLK_GPSPI2 || clk->id == ESP32P4_CLK_GPSPI3 ||
	    clk->id == ESP32P4_CLK_LP_I2C)
		return parent_rate / 2;
	return parent_rate;
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
	const struct p4_i2s_div *tx = &p4_i2s_layout(clk->id)->tx;
	u32 integer = p4_i2s_read(cc, tx->n, 0xff) ?: 256;
	u32 z = p4_i2s_read(cc, tx->z, I2S_DIV_MAX);
	u32 denom, num;

	integer = max(integer, 2U);
	if (!z)
		return parent / integer;

	denom = (p4_i2s_read(cc, tx->x, I2S_DIV_MAX) + 1) * z +
		p4_i2s_read(cc, tx->y, I2S_DIV_MAX);
	num = p4_i2s_read(cc, tx->yn1, 1) ? denom - z : z;
	return DIV_ROUND_CLOSEST_ULL((u64)parent * denom, integer * denom + num);
}

static int p4_i2s_determine_rate(struct clk_hw *hw, struct clk_rate_request *req)
{
	if (req->rate < DIV_ROUND_UP(req->best_parent_rate, 256) ||
	    req->rate > req->best_parent_rate / 2)
		return -ERANGE;
	return 0;
}

static void p4_i2s_write_div(struct p4_cc *cc, const struct p4_i2s_div *div,
			    u32 integer, u32 x, u32 y, u32 z, bool yn1)
{
	p4_i2s_write(cc, div->n, 0xff, integer & 0xff);
	p4_i2s_write(cc, div->x, I2S_DIV_MAX, x);
	p4_i2s_write(cc, div->y, I2S_DIV_MAX, y);
	p4_i2s_write(cc, div->z, I2S_DIV_MAX, z);
	p4_i2s_write(cc, div->yn1, 1, yn1);
}

static int p4_i2s_set_rate(struct clk_hw *hw, unsigned long rate,
			  unsigned long parent)
{
	struct p4_clk *clk = container_of(hw, struct p4_clk, hw);
	struct p4_cc *cc = clk->cc;
	const struct p4_i2s_layout *layout = p4_i2s_layout(clk->id);
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
	p4_i2s_write_div(cc, &layout->rx, integer, x, y, z, yn1);
	p4_i2s_write_div(cc, &layout->tx, integer, x, y, z, yn1);
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
	int ret;

	if (id == ESP32P4_RST_LP_SPI) {
		if (!cc->lpperi)
			return -ENODEV;
		ret = regmap_set_bits(cc->lpperi, LP_PERI_RESET_EN, LP_SPI_RESET_EN);
		if (ret)
			return ret;
		return regmap_clear_bits(cc->lpperi, LP_PERI_RESET_EN, LP_SPI_RESET_EN);
	}

	switch (id) {
	case ESP32P4_RST_LP_I2C:
		if (!cc->lpperi)
			return -ENOENT;
		ret = regmap_set_bits(cc->lpperi, LP_PERI_RESET_EN, LP_I2C_RESET_EN);
		if (ret)
			return ret;
		return regmap_clear_bits(cc->lpperi, LP_PERI_RESET_EN, LP_I2C_RESET_EN);
	case ESP32P4_RST_I2C0:
		reg = HP_RST1;
		bit = I2C0_RST;
		break;
	case ESP32P4_RST_I2C1:
		reg = HP_RST1;
		bit = I2C1_RST;
		break;
	case ESP32P4_RST_GPSPI3:
		reg = HP_RST2;
		bit = GPSPI3_RST;
		break;
	case ESP32P4_RST_GPSPI2:
		reg = HP_RST2;
		bit = GPSPI2_RST;
		break;
	case ESP32P4_RST_DMA2D:
		reg = HP_RST0;
		bit = HP_DMA2D_RESET;
		break;
	case ESP32P4_RST_AXI_GDMA:
		reg = HP_RST0;
		bit = HP_AXI_PDMA_RESET;
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
	case ESP32P4_RST_TWAI0 ... ESP32P4_RST_TWAI2:
		reg = HP_RST1;
		bit = TWAI_RESET(id - ESP32P4_RST_TWAI0);
		break;
	case ESP32P4_RST_I2S0:
		reg = HP_RST2;
		bit = I2S_APB_RESET(0);
		break;
	case ESP32P4_RST_I2S1 ... ESP32P4_RST_I2S2:
		reg = HP_RST2;
		bit = I2S_APB_RESET(1 + id - ESP32P4_RST_I2S1);
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
		"p4-aes-sha",
		"p4-twai0", "p4-twai1", "p4-twai2",
		"p4-rsa", "p4-ecc",
		"p4-gptimer0", "p4-gptimer1",
		"p4-axi-gdma",
		"p4-i2c0",
		"p4-lp-i2c",
		"p4-dma2d",
		"p4-i2s1", "p4-i2s2",
		"p4-gpspi3",
		"p4-lp-spi",
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
		cc->lp_hp_clk = devm_platform_ioremap_resource(pdev, 2);
		if (IS_ERR(cc->lp_hp_clk))
			return PTR_ERR(cc->lp_hp_clk);
		cc->emac_ctrl = devm_platform_ioremap_resource(pdev, 3);
		if (IS_ERR(cc->emac_ctrl))
			return PTR_ERR(cc->emac_ctrl);
	}
	if (platform_get_resource(pdev, IORESOURCE_MEM, 4)) {
		cc->lp_fast_clksel = devm_platform_ioremap_resource(pdev, 4);
		if (IS_ERR(cc->lp_fast_clksel))
			return PTR_ERR(cc->lp_fast_clksel);
	}
	cc->lpperi = syscon_regmap_lookup_by_phandle_optional(dev->of_node,
							     "espressif,lpperi");
	if (IS_ERR(cc->lpperi))
		return PTR_ERR(cc->lpperi);
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
			       p4_i2s_layout(i) ? &p4_i2s_clk_ops : &p4_clk_ops,
			.parent_data = &parent,
			.num_parents = (i == ESP32P4_CLK_USBHS ||
					i == ESP32P4_CLK_EMAC ||
					i == ESP32P4_CLK_AHB_GDMA ||
					i == ESP32P4_CLK_AXI_GDMA ||
					i == ESP32P4_CLK_DMA2D ||
					i == ESP32P4_CLK_PCNT) ? 0 : 1,
		};
		struct p4_clk *clk;

		if (((i == ESP32P4_CLK_SDMMC || i == ESP32P4_CLK_EMAC) &&
		     !cc->lp_reset) || (i == ESP32P4_CLK_EMAC_RMII && !cc->lp_hp_clk) ||
		    (i == ESP32P4_CLK_LP_I2C && !cc->lpperi) ||
		    (i == ESP32P4_CLK_LP_SPI && (!cc->lpperi || !cc->lp_fast_clksel))) {
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
