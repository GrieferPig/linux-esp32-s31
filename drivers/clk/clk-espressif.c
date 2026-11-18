// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S31 peripheral clock/reset controller.
 *
 * This owns every gate, reset and connectivity-domain clock needed by the
 * Linux UART, SDMMC and GMAC drivers.  PLL programming is kept fixed because
 * the CPU, flash XIP and PSRAM need those PLLs before Linux can execute.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rational.h>
#include <linux/reboot.h>
#include <linux/reset-controller.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>

#include <dt-bindings/clock/esp32s31-clock.h>
#include <dt-bindings/reset/esp32s31-reset.h>

#define ESP32S31_XTAL_RATE		40000000UL
#define ESP32S31_RC_FAST_RATE		17500000UL
#define ESP32S31_RC_SLOW_RATE		136000UL
#define ESP32S31_XTAL32K_RATE		32768UL
#define ESP32S31_BBPLL_RATE		480000000UL
#define ESP32S31_GPSPI_RATE		80000000UL
#define ESP32S31_SDMMC_CIU_RATE		10000000UL

#define HP_SOC_CLK_SEL			0x00
#define HP_CPU_FREQ_CTRL0		0x04
#define HP_MEM_FREQ_CTRL0		0x08
#define HP_SYS_FREQ_CTRL0		0x0c
#define HP_APB_FREQ_CTRL0		0x10
#define HP_ROOT_CLK_CTRL0		0x14
#define HP_REF_500M_CTRL0		0x178
#define HP_REF_240M_CTRL0		0x17c
#define HP_REF_160M_CTRL0		0x180
#define HP_REF_120M_CTRL0		0x184
#define HP_REF_80M_CTRL0		0x188
#define HP_REF_60M_CTRL0		0x18c
#define HP_REF_20M_CTRL0		0x190
#define HP_REF_50M_CTRL0		0x194
#define HP_REF_25M_CTRL0		0x198

#define HP_SOC_CLK_SEL_MASK		GENMASK(1, 0)
#define HP_CLK_DIV_NUM			GENMASK(7, 0)
#define HP_CLK_DIV_NUMERATOR		GENMASK(10, 8)
#define HP_CLK_DIV_DENOMINATOR		GENMASK(13, 11)
#define HP_SOC_CLK_UPDATE		BIT(0)

#define LP_HP_CLK_CTRL			0x48
#define LP_CPLL_DIV			0x4c
#define LP_APLL_DIV			0x50
#define LP_MPLL_DIV			0x54
#define LP_APLL_SDM			0x64

#define LP_XTAL_FREQ_MHZ		GENMASK(6, 0)
#define LP_CPLL_REF_DIV			GENMASK(3, 0)
#define LP_CPLL_FB_DIV			GENMASK(11, 4)
#define LP_APLL_OUT_DIV			GENMASK(7, 3)
#define LP_MPLL_FB_DIV			GENMASK(7, 3)
#define LP_APLL_SDM_MASK		GENMASK(21, 0)

#define HP_REF_CLK_DIV_NUM		GENMASK(7, 0)

#define HP_SDIO_HOST_CTRL0		0xc0
#define HP_SDIO_HOST_FUNC_CTRL0		0xc4
#define HP_EMAC_CTRL0			0xc8
#define HP_UART0_CTRL0			0x88
#define HP_UART_CTRL_STRIDE		0x04
#define HP_GPSPI2_CTRL0			0x6c
#define HP_GPSPI_CTRL_STRIDE		0x04
#define HP_I2S0_APB_CTRL0		0xe4
#define HP_I2S_CTRL_STRIDE		0x14
#define HP_I2S0_RX_CTRL0		0xe8
#define HP_I2S0_RX_DIV_CTRL0		0xec
#define HP_I2S0_TX_CTRL0		0xf0
#define HP_I2S0_TX_DIV_CTRL0		0xf4
#define HP_TWAI0_CTRL0			0x10c
#define HP_TWAI_CTRL_STRIDE		0x04
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

#define HP_GPSPI_SYS_CLK_EN		BIT(0)
#define HP_GPSPI_APB_CLK_EN		BIT(1)
#define HP_GPSPI_RST_EN			BIT(2)
#define HP_GPSPI_FORCE_NORST		BIT(3)
#define HP_GPSPI_CLK_SRC_SEL		GENMASK(5, 4)
#define HP_GPSPI_HS_CLK_EN		BIT(6)
#define HP_GPSPI_HS_CLK_DIV_NUM		GENMASK(14, 7)
#define HP_GPSPI_MST_CLK_DIV_NUM	GENMASK(22, 15)
#define HP_GPSPI_MST_CLK_EN		BIT(23)

#define HP_I2S_APB_CLK_EN		BIT(0)
#define HP_I2S_APB_RST_EN		BIT(1)
#define HP_I2S_FORCE_NORST		BIT(2)
#define HP_I2S_CORE_CLK_EN		BIT(0)
#define HP_I2S_CORE_CLK_SRC_SEL		GENMASK(2, 1)
#define HP_I2S_CORE_DIV_N		GENMASK(10, 3)
#define HP_I2S_MST_CLK_SEL		BIT(11)
#define HP_I2S_DIV_X			GENMASK(8, 0)
#define HP_I2S_DIV_Y			GENMASK(17, 9)
#define HP_I2S_DIV_Z			GENMASK(26, 18)
#define HP_I2S_DIV_YN1			BIT(27)

#define HP_ALIVE_PAD_I2S0_CTRL		0x08
#define HP_ALIVE_PAD_I2S_STRIDE		0x04
#define HP_ALIVE_PAD_I2S_MCLK_EN	BIT(31)

#define HP_TWAI_APB_CLK_EN		BIT(0)
#define HP_TWAI_CLK_SRC_SEL		GENMASK(2, 1)
#define HP_TWAI_CLK_EN			BIT(3)
#define HP_TWAI_RST_EN			BIT(4)
#define HP_TWAI_FORCE_NORST		BIT(5)

#define HP_TIMERGRP_APB_CLK_EN		BIT(0)
#define HP_TIMERGRP_RST_EN		BIT(1)
#define HP_TIMERGRP_FORCE_NORST		BIT(2)
#define HP_TIMERGRP_WDT_SRC_SEL		GENMASK(10, 9)
#define HP_TIMERGRP_WDT_CLK_EN		BIT(11)

#define HP_SYS_UART_MEM_LP_CTRL0		0x28c
#define HP_SYS_UART_MEM_LP_STRIDE	0x04
#define HP_SYS_UART3_MEM_LP_CTRL		0x1f0
#define HP_SYS_UART_MEM_LP_EN		BIT(2)
#define HP_SYS_UART_MEM_FORCE_CTRL	BIT(3)
#define HP_SYS_TWAI0_MEM_LP_CTRL		0x228
#define HP_SYS_TWAI1_MEM_LP_CTRL		0x2a4
#define HP_SYS_TWAI_MEM_LP_EN		BIT(2)
#define HP_SYS_TWAI_MEM_FORCE_CTRL	BIT(3)

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
	void __iomem *hp_alive;
	void __iomem *lp_aon;
	spinlock_t lock;
	struct clk_hw_onecell_data *onecell;
	struct reset_controller_dev rcdev;
	bool sdmmc_initialized;
	bool emac_initialized;
	bool uart_initialized[4];
	bool twai_initialized[2];
	bool gpspi_initialized[2];
	bool i2s_initialized[2];
	unsigned int sdmmc_users;
	unsigned int emac_users;
};

enum esp32s31_clk_kind {
	ESP32S31_CLK_KIND_FIXED,
	ESP32S31_CLK_KIND_XTAL,
	ESP32S31_CLK_KIND_CPLL,
	ESP32S31_CLK_KIND_MPLL,
	ESP32S31_CLK_KIND_APLL,
	ESP32S31_CLK_KIND_CPU,
	ESP32S31_CLK_KIND_MEM,
	ESP32S31_CLK_KIND_SYS,
	ESP32S31_CLK_KIND_APB,
	ESP32S31_CLK_KIND_PLL_DIV,
	ESP32S31_CLK_KIND_SDMMC_BIU,
	ESP32S31_CLK_KIND_SDMMC_CIU,
	ESP32S31_CLK_KIND_EMAC_BUS,
	ESP32S31_CLK_KIND_EMAC_PTP,
	ESP32S31_CLK_KIND_EMAC_TXC,
	ESP32S31_CLK_KIND_UART0,
	ESP32S31_CLK_KIND_UART1,
	ESP32S31_CLK_KIND_UART2,
	ESP32S31_CLK_KIND_UART3,
	ESP32S31_CLK_KIND_GPSPI2,
	ESP32S31_CLK_KIND_GPSPI3,
	ESP32S31_CLK_KIND_I2S0,
	ESP32S31_CLK_KIND_I2S1,
	ESP32S31_CLK_KIND_TWAI0,
	ESP32S31_CLK_KIND_TWAI1,
	ESP32S31_CLK_KIND_WDT1,
	ESP32S31_CLK_KIND_GENERIC,
};

struct esp32s31_clk {
	struct clk_hw hw;
	struct esp32s31_clk_priv *priv;
	enum esp32s31_clk_kind kind;
	unsigned long fixed_rate;
	u16 reg;
	u32 enable_mask;
	u32 reset_mask;
	u32 force_norst_mask;
	bool initialized;
};

enum esp32s31_reg_bank {
	ESP32S31_BANK_HP,
	ESP32S31_BANK_CNNT,
};

struct esp32s31_reset_desc {
	u16 reg;
	u32 reset_mask;
	u32 force_norst_mask;
	u8 bank;
};

#define to_esp32s31_clk(_hw) container_of(_hw, struct esp32s31_clk, hw)

static void esp32s31_rmw(void __iomem *base, u32 off, u32 clr, u32 set)
{
	u32 val = readl(base + off);

	val &= ~clr;
	val |= set;
	writel(val, base + off);
}

static const struct esp32s31_reset_desc esp32s31_resets[ESP32S31_RST_NR] = {
	[ESP32S31_RST_SDMMC] = { CNNT_SYS_HP_SDMMC_CTRL,
		CNNT_SDMMC_RST_EN, CNNT_SDMMC_FORCE_NORST, ESP32S31_BANK_CNNT },
	[ESP32S31_RST_EMAC] = { CNNT_SYS_HP_EMAC_CTRL,
		CNNT_EMAC_RST_EN, CNNT_EMAC_FORCE_NORST, ESP32S31_BANK_CNNT },
	[ESP32S31_RST_UART0] = { HP_UART0_CTRL0,
		HP_UART_CORE_RST_EN | HP_UART_APB_RST_EN, HP_UART_FORCE_NORST,
		ESP32S31_BANK_HP },
	[ESP32S31_RST_UART1] = { HP_UART0_CTRL0 + HP_UART_CTRL_STRIDE,
		HP_UART_CORE_RST_EN | HP_UART_APB_RST_EN, HP_UART_FORCE_NORST,
		ESP32S31_BANK_HP },
	[ESP32S31_RST_UART2] = { HP_UART0_CTRL0 + 2 * HP_UART_CTRL_STRIDE,
		HP_UART_CORE_RST_EN | HP_UART_APB_RST_EN, HP_UART_FORCE_NORST,
		ESP32S31_BANK_HP },
	[ESP32S31_RST_UART3] = { HP_UART0_CTRL0 + 3 * HP_UART_CTRL_STRIDE,
		HP_UART_CORE_RST_EN | HP_UART_APB_RST_EN, HP_UART_FORCE_NORST,
		ESP32S31_BANK_HP },
	[ESP32S31_RST_WDT0] = { 0x114, BIT(1), BIT(2), ESP32S31_BANK_HP },
	[ESP32S31_RST_WDT1] = { HP_TIMERGRP1_CTRL0, HP_TIMERGRP_RST_EN,
		HP_TIMERGRP_FORCE_NORST, ESP32S31_BANK_HP },
	[ESP32S31_RST_SYSTIMER] = { 0x120, BIT(1), BIT(2), ESP32S31_BANK_HP },
	[ESP32S31_RST_AHB_GDMA] = { 0x7c, BIT(1), BIT(2), ESP32S31_BANK_HP },
	[ESP32S31_RST_UHCI] = { 0x84, BIT(2), BIT(3), ESP32S31_BANK_HP },
	[ESP32S31_RST_LEDC0] = { 0x148, BIT(1), BIT(2), ESP32S31_BANK_HP },
	[ESP32S31_RST_LEDC1] = { 0x148, BIT(7), BIT(8), ESP32S31_BANK_HP },
	[ESP32S31_RST_MCPWM0] = { 0x124, BIT(1), BIT(2), ESP32S31_BANK_HP },
	[ESP32S31_RST_MCPWM1] = { 0x128, BIT(1), BIT(2), ESP32S31_BANK_HP },
	[ESP32S31_RST_MCPWM2] = { 0x12c, BIT(1), BIT(2), ESP32S31_BANK_HP },
	[ESP32S31_RST_MCPWM3] = { 0x130, BIT(1), BIT(2), ESP32S31_BANK_HP },
	[ESP32S31_RST_PCNT0] = { 0x138, BIT(1), BIT(2), ESP32S31_BANK_HP },
	[ESP32S31_RST_PCNT1] = { 0x138, BIT(4), BIT(5), ESP32S31_BANK_HP },
	[ESP32S31_RST_I2C0] = { 0xdc, BIT(1), BIT(2), ESP32S31_BANK_HP },
	[ESP32S31_RST_I2C1] = { 0xe0, BIT(1), BIT(2), ESP32S31_BANK_HP },
	[ESP32S31_RST_TWAI0] = { HP_TWAI0_CTRL0, HP_TWAI_RST_EN,
		HP_TWAI_FORCE_NORST, ESP32S31_BANK_HP },
	[ESP32S31_RST_TWAI1] = { HP_TWAI0_CTRL0 + HP_TWAI_CTRL_STRIDE,
		HP_TWAI_RST_EN, HP_TWAI_FORCE_NORST, ESP32S31_BANK_HP },
	[ESP32S31_RST_GPSPI2] = { HP_GPSPI2_CTRL0, HP_GPSPI_RST_EN,
		HP_GPSPI_FORCE_NORST, ESP32S31_BANK_HP },
	[ESP32S31_RST_GPSPI3] = { HP_GPSPI2_CTRL0 + HP_GPSPI_CTRL_STRIDE,
		HP_GPSPI_RST_EN, HP_GPSPI_FORCE_NORST, ESP32S31_BANK_HP },
	[ESP32S31_RST_I2S0] = { HP_I2S0_APB_CTRL0, HP_I2S_APB_RST_EN,
		HP_I2S_FORCE_NORST, ESP32S31_BANK_HP },
	[ESP32S31_RST_I2S1] = { HP_I2S0_APB_CTRL0 + HP_I2S_CTRL_STRIDE,
		HP_I2S_APB_RST_EN, HP_I2S_FORCE_NORST, ESP32S31_BANK_HP },
};

static void __iomem *esp32s31_reset_base(struct esp32s31_clk_priv *priv,
					 const struct esp32s31_reset_desc *desc)
{
	return desc->bank == ESP32S31_BANK_CNNT ? priv->cnnt : priv->hp;
}

static int esp32s31_reset_assert(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	struct esp32s31_clk_priv *priv =
		container_of(rcdev, struct esp32s31_clk_priv, rcdev);
	const struct esp32s31_reset_desc *desc;
	unsigned long flags;

	if (id >= ARRAY_SIZE(esp32s31_resets))
		return -EINVAL;
	desc = &esp32s31_resets[id];

	spin_lock_irqsave(&priv->lock, flags);
	esp32s31_rmw(esp32s31_reset_base(priv, desc), desc->reg,
		      desc->force_norst_mask, desc->reset_mask);
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int esp32s31_reset_deassert(struct reset_controller_dev *rcdev,
				  unsigned long id)
{
	struct esp32s31_clk_priv *priv =
		container_of(rcdev, struct esp32s31_clk_priv, rcdev);
	const struct esp32s31_reset_desc *desc;
	unsigned long flags;

	if (id >= ARRAY_SIZE(esp32s31_resets))
		return -EINVAL;
	desc = &esp32s31_resets[id];

	spin_lock_irqsave(&priv->lock, flags);
	esp32s31_rmw(esp32s31_reset_base(priv, desc), desc->reg,
		      desc->reset_mask, desc->force_norst_mask);
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int esp32s31_reset(struct reset_controller_dev *rcdev,
			 unsigned long id)
{
	struct esp32s31_clk_priv *priv =
		container_of(rcdev, struct esp32s31_clk_priv, rcdev);
	const struct esp32s31_reset_desc *desc;
	void __iomem *base;
	unsigned long flags;

	if (id >= ARRAY_SIZE(esp32s31_resets))
		return -EINVAL;
	desc = &esp32s31_resets[id];
	base = esp32s31_reset_base(priv, desc);

	/* Match the IDF peripheral reset pulse: assert, then release. */
	spin_lock_irqsave(&priv->lock, flags);
	esp32s31_rmw(base, desc->reg, desc->force_norst_mask,
		      desc->reset_mask);
	esp32s31_rmw(base, desc->reg, desc->reset_mask,
		      desc->force_norst_mask);
	readl(base + desc->reg);
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int esp32s31_reset_status(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	struct esp32s31_clk_priv *priv =
		container_of(rcdev, struct esp32s31_clk_priv, rcdev);
	const struct esp32s31_reset_desc *desc;

	if (id >= ARRAY_SIZE(esp32s31_resets))
		return -EINVAL;
	desc = &esp32s31_resets[id];

	return !!(readl(esp32s31_reset_base(priv, desc) + desc->reg) &
		  desc->reset_mask);
}

static const struct reset_control_ops esp32s31_reset_ops = {
	.reset = esp32s31_reset,
	.assert = esp32s31_reset_assert,
	.deassert = esp32s31_reset_deassert,
	.status = esp32s31_reset_status,
};

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
	u32 mem_reg = port == 3 ? HP_SYS_UART3_MEM_LP_CTRL :
		      HP_SYS_UART_MEM_LP_CTRL0 +
		      port * HP_SYS_UART_MEM_LP_STRIDE;
	u32 mask = HP_UART_SYS_CLK_EN | HP_UART_APB_CLK_EN |
		   HP_UART_CORE_RST_EN | HP_UART_APB_RST_EN |
		   HP_UART_FORCE_NORST | HP_UART_CLK_SRC_SEL |
		   HP_UART_CLK_EN | HP_UART_SCLK_DIV_NUM |
		   HP_UART_SCLK_DIV_NUMERATOR |
		   HP_UART_SCLK_DIV_DENOMINATOR;
	u32 val = HP_UART_SYS_CLK_EN | HP_UART_APB_CLK_EN |
		  HP_UART_FORCE_NORST | HP_UART_CLK_EN;

	/* XTAL source, integer divide by one. Reset is consumer-controlled. */
	esp32s31_rmw(priv->hp, reg, mask, val);

	/* UART1/2/3 FIFO SRAM powers up disabled; UART3 has a non-linear
	 * power-control register. Force the selected block on while in use.
	 */
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

static void esp32s31_gpspi_prepare(struct esp32s31_clk_priv *priv,
				   unsigned int port)
{
	u32 reg = HP_GPSPI2_CTRL0 + port * HP_GPSPI_CTRL_STRIDE;
	u32 mask = HP_GPSPI_SYS_CLK_EN | HP_GPSPI_APB_CLK_EN |
		   HP_GPSPI_RST_EN | HP_GPSPI_FORCE_NORST |
		   HP_GPSPI_CLK_SRC_SEL | HP_GPSPI_HS_CLK_EN |
		   HP_GPSPI_HS_CLK_DIV_NUM | HP_GPSPI_MST_CLK_DIV_NUM |
		   HP_GPSPI_MST_CLK_EN;
	u32 val = HP_GPSPI_SYS_CLK_EN | HP_GPSPI_APB_CLK_EN |
		  HP_GPSPI_FORCE_NORST |
		  FIELD_PREP(HP_GPSPI_CLK_SRC_SEL, 2) |
		  HP_GPSPI_HS_CLK_EN |
		  FIELD_PREP(HP_GPSPI_HS_CLK_DIV_NUM, 3 - 1) |
		  FIELD_PREP(HP_GPSPI_MST_CLK_DIV_NUM, 2 - 1) |
		  HP_GPSPI_MST_CLK_EN;

	/*
	 * Match the IDF default GPSPI clock path: BBPLL / 3 / 2 = 80 MHz.
	 * Besides restoring the documented peripheral clock range, this gives
	 * common rates such as 20 MHz an even main divider and therefore a 50%
	 * SCLK duty cycle.  The former 40 MHz XTAL path generated 13.33 MHz with
	 * an asymmetric divide-by-three waveform for requests above 13.33 MHz.
	 */
	esp32s31_rmw(priv->hp, reg, mask, val);
}

static void esp32s31_i2s_prepare(struct esp32s31_clk_priv *priv,
				 unsigned int port)
{
	u32 reg = HP_I2S0_APB_CTRL0 + port * HP_I2S_CTRL_STRIDE;
	u32 rx_reg = HP_I2S0_RX_CTRL0 + port * HP_I2S_CTRL_STRIDE;
	u32 tx_reg = HP_I2S0_TX_CTRL0 + port * HP_I2S_CTRL_STRIDE;
	u32 rx_div_reg = HP_I2S0_RX_DIV_CTRL0 + port * HP_I2S_CTRL_STRIDE;
	u32 tx_div_reg = HP_I2S0_TX_DIV_CTRL0 + port * HP_I2S_CTRL_STRIDE;

	esp32s31_rmw(priv->hp, reg, HP_I2S_APB_RST_EN,
		     HP_I2S_APB_CLK_EN | HP_I2S_FORCE_NORST);
	esp32s31_rmw(priv->hp_alive,
		     HP_ALIVE_PAD_I2S0_CTRL + port * HP_ALIVE_PAD_I2S_STRIDE,
		     0, HP_ALIVE_PAD_I2S_MCLK_EN);
	/*
	 * Follow the S31 IDF divider programming workaround exactly: switch
	 * to a small integer divider, touch the fractional divider, clear it,
	 * and only then install the target divider.  Programming DIV_N from
	 * reset straight to the target can leave the I2S clock domain stopped;
	 * in that state TX/RX UPDATE never self-clears.
	 */
	esp32s31_rmw(priv->hp, rx_reg,
		     HP_I2S_CORE_CLK_SRC_SEL | HP_I2S_CORE_DIV_N |
		     HP_I2S_MST_CLK_SEL,
		     HP_I2S_CORE_CLK_EN | FIELD_PREP(HP_I2S_CORE_DIV_N, 2) |
		     HP_I2S_MST_CLK_SEL);
	esp32s31_rmw(priv->hp, tx_reg,
		     HP_I2S_CORE_CLK_SRC_SEL | HP_I2S_CORE_DIV_N,
		     HP_I2S_CORE_CLK_EN | FIELD_PREP(HP_I2S_CORE_DIV_N, 2));
	writel(FIELD_PREP(HP_I2S_DIV_Y, 1), priv->hp + rx_div_reg);
	writel(FIELD_PREP(HP_I2S_DIV_Y, 1), priv->hp + tx_div_reg);
	writel(0, priv->hp + rx_div_reg);
	writel(0, priv->hp + tx_div_reg);

	/*
	 * 40 MHz / (3 + 49 / 192) = 12.288 MHz, the standard 256 * Fs
	 * master clock for 48 kHz audio.  The S31 fractional representation
	 * for 49/192 is X=2, Y=45, Z=49, YN1=0.
	 */
	writel(FIELD_PREP(HP_I2S_DIV_X, 2) |
	       FIELD_PREP(HP_I2S_DIV_Y, 45) |
	       FIELD_PREP(HP_I2S_DIV_Z, 49), priv->hp + rx_div_reg);
	writel(FIELD_PREP(HP_I2S_DIV_X, 2) |
	       FIELD_PREP(HP_I2S_DIV_Y, 45) |
	       FIELD_PREP(HP_I2S_DIV_Z, 49), priv->hp + tx_div_reg);
	esp32s31_rmw(priv->hp, rx_reg, HP_I2S_CORE_DIV_N,
		     FIELD_PREP(HP_I2S_CORE_DIV_N, 3));
	esp32s31_rmw(priv->hp, tx_reg, HP_I2S_CORE_DIV_N,
		     FIELD_PREP(HP_I2S_CORE_DIV_N, 3));
}

static void esp32s31_twai_prepare(struct esp32s31_clk_priv *priv,
				  unsigned int port)
{
	u32 reg = HP_TWAI0_CTRL0 + port * HP_TWAI_CTRL_STRIDE;
	u32 mem_reg = port ? HP_SYS_TWAI1_MEM_LP_CTRL :
			     HP_SYS_TWAI0_MEM_LP_CTRL;
	u32 mask = HP_TWAI_APB_CLK_EN | HP_TWAI_CLK_SRC_SEL |
		   HP_TWAI_CLK_EN | HP_TWAI_RST_EN | HP_TWAI_FORCE_NORST;
	u32 val = HP_TWAI_APB_CLK_EN |
		  FIELD_PREP(HP_TWAI_CLK_SRC_SEL, 2) |
		  HP_TWAI_CLK_EN | HP_TWAI_FORCE_NORST;

	/* Match IDF: PLL_F80M source and SRAM forced out of low-power mode. */
	esp32s31_rmw(priv->hp_sys, mem_reg,
		     HP_SYS_TWAI_MEM_LP_EN | HP_SYS_TWAI_MEM_FORCE_CTRL,
		     HP_SYS_TWAI_MEM_FORCE_CTRL);
	esp32s31_rmw(priv->hp, reg, mask, val);
	esp32s31_rmw(priv->hp, reg, 0, HP_TWAI_RST_EN);
	esp32s31_rmw(priv->hp, reg, HP_TWAI_RST_EN, 0);
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
		priv->sdmmc_users++;
		if (!priv->sdmmc_initialized) {
			esp32s31_sdmmc_prepare(priv);
			priv->sdmmc_initialized = true;
		}
		break;
	case ESP32S31_CLK_KIND_EMAC_BUS:
		priv->emac_users++;
		if (!priv->emac_initialized) {
			esp32s31_emac_prepare(priv);
			priv->emac_initialized = true;
		}
		break;
	case ESP32S31_CLK_KIND_EMAC_PTP:
		priv->emac_users++;
		if (!priv->emac_initialized) {
			esp32s31_emac_prepare(priv);
			priv->emac_initialized = true;
		}
		esp32s31_rmw(priv->cnnt, CNNT_SYS_HP_EMAC_PTP_CTRL,
			     CNNT_EMAC_PTP_REF_CLK_SEL, CNNT_EMAC_PTP_REF_CLK_EN);
		break;
	case ESP32S31_CLK_KIND_EMAC_TXC:
		priv->emac_users++;
		if (!priv->emac_initialized) {
			esp32s31_emac_prepare(priv);
			priv->emac_initialized = true;
		}
		esp32s31_rmw(priv->cnnt, CNNT_SYS_HP_EMAC_REF_CTRL,
			     CNNT_EMAC_REF_CLK_SEL, CNNT_EMAC_REF_CLK_EN);
		break;
	case ESP32S31_CLK_KIND_UART0:
	case ESP32S31_CLK_KIND_UART1:
	case ESP32S31_CLK_KIND_UART2:
	case ESP32S31_CLK_KIND_UART3: {
		unsigned int port = clk->kind - ESP32S31_CLK_KIND_UART0;

		if (!priv->uart_initialized[port]) {
			esp32s31_uart_prepare(priv, port);
			priv->uart_initialized[port] = true;
		}
		break;
	}
	case ESP32S31_CLK_KIND_GPSPI2:
	case ESP32S31_CLK_KIND_GPSPI3: {
		unsigned int port = clk->kind - ESP32S31_CLK_KIND_GPSPI2;

		if (!priv->gpspi_initialized[port]) {
			esp32s31_gpspi_prepare(priv, port);
			priv->gpspi_initialized[port] = true;
		}
		break;
	}
	case ESP32S31_CLK_KIND_I2S0:
	case ESP32S31_CLK_KIND_I2S1: {
		unsigned int port = clk->kind - ESP32S31_CLK_KIND_I2S0;

		if (!priv->i2s_initialized[port]) {
			esp32s31_i2s_prepare(priv, port);
			priv->i2s_initialized[port] = true;
		}
		break;
	}
	case ESP32S31_CLK_KIND_TWAI0:
	case ESP32S31_CLK_KIND_TWAI1: {
		unsigned int port = clk->kind - ESP32S31_CLK_KIND_TWAI0;

		if (!priv->twai_initialized[port]) {
			esp32s31_twai_prepare(priv, port);
			priv->twai_initialized[port] = true;
		}
		break;
	}
	case ESP32S31_CLK_KIND_WDT1:
		if (!clk->initialized) {
			esp32s31_wdt1_prepare(priv);
			clk->initialized = true;
		}
		break;
	case ESP32S31_CLK_KIND_GENERIC:
		if (!clk->initialized) {
			esp32s31_rmw(priv->hp, clk->reg, clk->reset_mask,
				     clk->enable_mask | clk->force_norst_mask);
			clk->initialized = true;
		}
		break;
	case ESP32S31_CLK_KIND_FIXED:
	case ESP32S31_CLK_KIND_XTAL:
	case ESP32S31_CLK_KIND_CPLL:
	case ESP32S31_CLK_KIND_MPLL:
	case ESP32S31_CLK_KIND_APLL:
	case ESP32S31_CLK_KIND_CPU:
	case ESP32S31_CLK_KIND_MEM:
	case ESP32S31_CLK_KIND_SYS:
	case ESP32S31_CLK_KIND_APB:
	case ESP32S31_CLK_KIND_PLL_DIV:
		break;
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static void esp32s31_clk_unprepare(struct clk_hw *hw)
{
	struct esp32s31_clk *clk = to_esp32s31_clk(hw);
	struct esp32s31_clk_priv *priv = clk->priv;
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&priv->lock, flags);
	switch (clk->kind) {
	case ESP32S31_CLK_KIND_SDMMC_BIU:
	case ESP32S31_CLK_KIND_SDMMC_CIU:
		/*
		 * The boot loader may leave the shared gate enabled without a
		 * matching CCF prepare count.  In that case late unused-clock
		 * cleanup reaches us with zero software users and must still be
		 * allowed to converge the hardware to the CCF state.
		 */
		if (priv->sdmmc_users && --priv->sdmmc_users)
			break;
		esp32s31_rmw(priv->hp, HP_SDIO_HOST_CTRL0,
			      HP_SDMMC_SYS_CLK_EN | HP_SDIO_LS_CLK_EN, 0);
		esp32s31_rmw(priv->hp, HP_SDIO_HOST_FUNC_CTRL0,
			      HP_SDIO_SLF_CLK_EN | HP_SDIO_DRV_CLK_EN |
			      HP_SDIO_SAM_CLK_EN, 0);
		priv->sdmmc_initialized = false;
		break;
	case ESP32S31_CLK_KIND_EMAC_BUS:
	case ESP32S31_CLK_KIND_EMAC_PTP:
	case ESP32S31_CLK_KIND_EMAC_TXC:
		/* See the SDMMC shared-gate boot-state case above. */
		if (priv->emac_users && --priv->emac_users)
			break;
		esp32s31_rmw(priv->hp, HP_EMAC_CTRL0, HP_EMAC_SYS_CLK_EN, 0);
		esp32s31_rmw(priv->cnnt, CNNT_SYS_HP_EMAC_REF_CTRL,
			      CNNT_EMAC_REF_CLK_EN, 0);
		esp32s31_rmw(priv->cnnt, CNNT_SYS_HP_EMAC_PTP_CTRL,
			      CNNT_EMAC_PTP_REF_CLK_EN, 0);
		esp32s31_rmw(priv->cnnt, CNNT_SYS_HP_EMAC_RX_CTRL,
			      CNNT_EMAC_RX_PAD_CLK_EN | CNNT_EMAC_RX_180_CLK_EN, 0);
		esp32s31_rmw(priv->cnnt, CNNT_SYS_HP_EMAC_TX_CTRL,
			      CNNT_EMAC_TX_180_CLK_EN, 0);
		priv->emac_initialized = false;
		break;
	case ESP32S31_CLK_KIND_UART0:
	case ESP32S31_CLK_KIND_UART1:
	case ESP32S31_CLK_KIND_UART2:
	case ESP32S31_CLK_KIND_UART3:
		reg = clk->kind - ESP32S31_CLK_KIND_UART0;
		esp32s31_rmw(priv->hp, HP_UART0_CTRL0 + reg * HP_UART_CTRL_STRIDE,
			      HP_UART_SYS_CLK_EN | HP_UART_APB_CLK_EN |
			      HP_UART_CLK_EN, 0);
		priv->uart_initialized[reg] = false;
		break;
	case ESP32S31_CLK_KIND_GPSPI2:
	case ESP32S31_CLK_KIND_GPSPI3:
		reg = clk->kind - ESP32S31_CLK_KIND_GPSPI2;
		esp32s31_rmw(priv->hp, HP_GPSPI2_CTRL0 + reg * HP_GPSPI_CTRL_STRIDE,
			      HP_GPSPI_SYS_CLK_EN | HP_GPSPI_APB_CLK_EN |
			      HP_GPSPI_HS_CLK_EN | HP_GPSPI_MST_CLK_EN, 0);
		priv->gpspi_initialized[reg] = false;
		break;
	case ESP32S31_CLK_KIND_I2S0:
	case ESP32S31_CLK_KIND_I2S1:
		reg = clk->kind - ESP32S31_CLK_KIND_I2S0;
		esp32s31_rmw(priv->hp, HP_I2S0_APB_CTRL0 + reg * HP_I2S_CTRL_STRIDE,
			      HP_I2S_APB_CLK_EN, 0);
		esp32s31_rmw(priv->hp, HP_I2S0_RX_CTRL0 + reg * HP_I2S_CTRL_STRIDE,
			      HP_I2S_CORE_CLK_EN, 0);
		esp32s31_rmw(priv->hp, HP_I2S0_TX_CTRL0 + reg * HP_I2S_CTRL_STRIDE,
			      HP_I2S_CORE_CLK_EN, 0);
		esp32s31_rmw(priv->hp_alive,
			      HP_ALIVE_PAD_I2S0_CTRL + reg * HP_ALIVE_PAD_I2S_STRIDE,
			      HP_ALIVE_PAD_I2S_MCLK_EN, 0);
		priv->i2s_initialized[reg] = false;
		break;
	case ESP32S31_CLK_KIND_TWAI0:
	case ESP32S31_CLK_KIND_TWAI1:
		reg = clk->kind - ESP32S31_CLK_KIND_TWAI0;
		esp32s31_rmw(priv->hp, HP_TWAI0_CTRL0 + reg * HP_TWAI_CTRL_STRIDE,
			      HP_TWAI_APB_CLK_EN | HP_TWAI_CLK_EN, 0);
		priv->twai_initialized[reg] = false;
		break;
	case ESP32S31_CLK_KIND_WDT1:
		esp32s31_rmw(priv->hp, HP_TIMERGRP1_CTRL0,
			      HP_TIMERGRP_APB_CLK_EN | HP_TIMERGRP_WDT_CLK_EN, 0);
		clk->initialized = false;
		break;
	case ESP32S31_CLK_KIND_GENERIC:
		esp32s31_rmw(priv->hp, clk->reg, clk->enable_mask, 0);
		clk->initialized = false;
		break;
	default:
		break;
	}
	spin_unlock_irqrestore(&priv->lock, flags);
}

static int esp32s31_clk_is_prepared(struct clk_hw *hw)
{
	struct esp32s31_clk *clk = to_esp32s31_clk(hw);
	struct esp32s31_clk_priv *priv = clk->priv;
	u32 reg;

	switch (clk->kind) {
	case ESP32S31_CLK_KIND_SDMMC_BIU:
		return !!(readl(priv->hp + HP_SDIO_HOST_CTRL0) &
			  HP_SDMMC_SYS_CLK_EN);
	case ESP32S31_CLK_KIND_SDMMC_CIU:
		return !!(readl(priv->hp + HP_SDIO_HOST_FUNC_CTRL0) &
			  HP_SDIO_SLF_CLK_EN);
	case ESP32S31_CLK_KIND_EMAC_BUS:
		return !!(readl(priv->hp + HP_EMAC_CTRL0) & HP_EMAC_SYS_CLK_EN);
	case ESP32S31_CLK_KIND_EMAC_PTP:
		return !!(readl(priv->cnnt + CNNT_SYS_HP_EMAC_PTP_CTRL) &
			  CNNT_EMAC_PTP_REF_CLK_EN);
	case ESP32S31_CLK_KIND_EMAC_TXC:
		return !!(readl(priv->cnnt + CNNT_SYS_HP_EMAC_REF_CTRL) &
			  CNNT_EMAC_REF_CLK_EN);
	case ESP32S31_CLK_KIND_UART0:
	case ESP32S31_CLK_KIND_UART1:
	case ESP32S31_CLK_KIND_UART2:
	case ESP32S31_CLK_KIND_UART3:
		reg = clk->kind - ESP32S31_CLK_KIND_UART0;
		return !!(readl(priv->hp + HP_UART0_CTRL0 +
				reg * HP_UART_CTRL_STRIDE) & HP_UART_CLK_EN);
	case ESP32S31_CLK_KIND_GPSPI2:
	case ESP32S31_CLK_KIND_GPSPI3:
		reg = clk->kind - ESP32S31_CLK_KIND_GPSPI2;
		return !!(readl(priv->hp + HP_GPSPI2_CTRL0 +
				reg * HP_GPSPI_CTRL_STRIDE) & HP_GPSPI_MST_CLK_EN);
	case ESP32S31_CLK_KIND_I2S0:
	case ESP32S31_CLK_KIND_I2S1:
		reg = clk->kind - ESP32S31_CLK_KIND_I2S0;
		return !!(readl(priv->hp + HP_I2S0_APB_CTRL0 +
				reg * HP_I2S_CTRL_STRIDE) & HP_I2S_APB_CLK_EN);
	case ESP32S31_CLK_KIND_TWAI0:
	case ESP32S31_CLK_KIND_TWAI1:
		reg = clk->kind - ESP32S31_CLK_KIND_TWAI0;
		return !!(readl(priv->hp + HP_TWAI0_CTRL0 +
				reg * HP_TWAI_CTRL_STRIDE) & HP_TWAI_CLK_EN);
	case ESP32S31_CLK_KIND_WDT1:
		return !!(readl(priv->hp + HP_TIMERGRP1_CTRL0) &
			  HP_TIMERGRP_WDT_CLK_EN);
	case ESP32S31_CLK_KIND_GENERIC:
		reg = readl(priv->hp + clk->reg);
		return (reg & clk->enable_mask) == clk->enable_mask;
	default:
		return 1;
	}
}

static unsigned long esp32s31_clk_recalc_rate(struct clk_hw *hw,
					      unsigned long parent_rate)
{
	struct esp32s31_clk *clk = to_esp32s31_clk(hw);
	struct esp32s31_clk_priv *priv = clk->priv;
	u32 div, denominator, numerator, reg, xtal_mhz;
	u64 rate;

	switch (clk->kind) {
	case ESP32S31_CLK_KIND_XTAL:
		xtal_mhz = FIELD_GET(LP_XTAL_FREQ_MHZ,
				     readl(priv->lp_aon + LP_HP_CLK_CTRL));
		return xtal_mhz ? xtal_mhz * 1000000UL : ESP32S31_XTAL_RATE;
	case ESP32S31_CLK_KIND_CPLL:
		reg = readl(priv->lp_aon + LP_CPLL_DIV);
		div = FIELD_GET(LP_CPLL_REF_DIV, reg);
		if (!div)
			return 0;
		return parent_rate * FIELD_GET(LP_CPLL_FB_DIV, reg) / div;
	case ESP32S31_CLK_KIND_MPLL:
		reg = readl(priv->lp_aon + LP_MPLL_DIV);
		return parent_rate * (FIELD_GET(LP_MPLL_FB_DIV, reg) + 1) / 2;
	case ESP32S31_CLK_KIND_APLL:
		reg = readl(priv->lp_aon + LP_APLL_SDM) & LP_APLL_SDM_MASK;
		numerator = ((4 + ((reg >> 16) & 0x3f)) << 16) |
			    (reg & 0xffff);
		denominator = (FIELD_GET(LP_APLL_OUT_DIV,
				readl(priv->lp_aon + LP_APLL_DIV)) + 2) << 17;
		return div64_u64((u64)parent_rate * numerator, denominator);
	case ESP32S31_CLK_KIND_CPU:
		reg = readl(priv->hp + HP_CPU_FREQ_CTRL0);
		div = FIELD_GET(HP_CLK_DIV_NUM, reg) + 1;
		numerator = FIELD_GET(HP_CLK_DIV_NUMERATOR, reg);
		denominator = FIELD_GET(HP_CLK_DIV_DENOMINATOR, reg);
		if (!numerator || !denominator)
			return parent_rate / div;
		rate = (u64)parent_rate * denominator;
		return div64_u64(rate, div * denominator + numerator);
	case ESP32S31_CLK_KIND_MEM:
		return parent_rate /
		       ((readl(priv->hp + HP_MEM_FREQ_CTRL0) & BIT(0)) + 1);
	case ESP32S31_CLK_KIND_SYS:
		reg = readl(priv->hp + HP_SYS_FREQ_CTRL0);
		break;
	case ESP32S31_CLK_KIND_APB:
		reg = readl(priv->hp + HP_APB_FREQ_CTRL0);
		break;
	case ESP32S31_CLK_KIND_PLL_DIV:
		div = FIELD_GET(HP_REF_CLK_DIV_NUM,
				readl(priv->hp + clk->reg)) + 1;
		return parent_rate / div;
	case ESP32S31_CLK_KIND_EMAC_TXC:
		div = FIELD_GET(CNNT_EMAC_REF_CLK_DIV,
				readl(priv->cnnt + CNNT_SYS_HP_EMAC_REF_CTRL));
		return parent_rate / (div + 1);
	default:
		return clk->fixed_rate;
	}

	div = FIELD_GET(HP_CLK_DIV_NUM, reg) + 1;
	numerator = FIELD_GET(HP_CLK_DIV_NUMERATOR, reg);
	denominator = FIELD_GET(HP_CLK_DIV_DENOMINATOR, reg);
	if (!numerator || !denominator)
		return parent_rate / div;
	rate = (u64)parent_rate * denominator;

	return div64_u64(rate, div * denominator + numerator);
}

static u8 esp32s31_cpu_get_parent(struct clk_hw *hw)
{
	struct esp32s31_clk *clk = to_esp32s31_clk(hw);

	return FIELD_GET(HP_SOC_CLK_SEL_MASK,
			 readl(clk->priv->hp + HP_SOC_CLK_SEL));
}

struct esp32s31_cpu_rate {
	unsigned long rate;
	u8 source;
	u8 cpu_div;
	u8 mem_div;
	u8 sys_div;
	u8 apb_div;
};

static const struct esp32s31_cpu_rate esp32s31_cpu_rates[] = {
	{  40000000UL, 0, 1, 1, 1, 1 },
	{  80000000UL, 1, 4, 1, 1, 2 },
	{ 160000000UL, 1, 2, 1, 2, 2 },
	{ 240000000UL, 3, 1, 2, 3, 2 },
	{ 320000000UL, 1, 1, 2, 3, 2 },
};

static const struct esp32s31_cpu_rate *esp32s31_cpu_find_rate(unsigned long rate)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(esp32s31_cpu_rates); i++)
		if (esp32s31_cpu_rates[i].rate == rate)
			return &esp32s31_cpu_rates[i];

	return NULL;
}

static long esp32s31_cpu_round_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long *parent_rate)
{
	const struct esp32s31_cpu_rate *best = &esp32s31_cpu_rates[0];
	unsigned long best_delta = abs_diff(rate, best->rate);
	unsigned int i;

	for (i = 1; i < ARRAY_SIZE(esp32s31_cpu_rates); i++) {
		unsigned long delta = abs_diff(rate, esp32s31_cpu_rates[i].rate);

		if (delta < best_delta) {
			best = &esp32s31_cpu_rates[i];
			best_delta = delta;
		}
	}

	return best->rate;
}

static int esp32s31_cpu_set_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long parent_rate)
{
	struct esp32s31_clk *clk = to_esp32s31_clk(hw);
	struct esp32s31_clk_priv *priv = clk->priv;
	const struct esp32s31_cpu_rate *config;
	unsigned long flags;
	u32 value;
	int timeout;

	config = esp32s31_cpu_find_rate(rate);
	if (!config)
		return -EINVAL;

	/*
	 * All divider and mux changes are latched together by SOC_CLK_UPDATE.
	 * This is the same atomic bus update sequence used by ESP-IDF and keeps
	 * both harts on one coherent clock configuration throughout a change.
	 */
	spin_lock_irqsave(&priv->lock, flags);
	value = FIELD_PREP(HP_CLK_DIV_NUM, config->cpu_div - 1);
	writel(value, priv->hp + HP_CPU_FREQ_CTRL0);
	writel(config->mem_div - 1, priv->hp + HP_MEM_FREQ_CTRL0);
	writel(config->sys_div - 1, priv->hp + HP_SYS_FREQ_CTRL0);
	writel(config->apb_div - 1, priv->hp + HP_APB_FREQ_CTRL0);
	esp32s31_rmw(priv->hp, HP_SOC_CLK_SEL, HP_SOC_CLK_SEL_MASK,
		     FIELD_PREP(HP_SOC_CLK_SEL_MASK, config->source));
	writel(HP_SOC_CLK_UPDATE, priv->hp + HP_ROOT_CLK_CTRL0);
	for (timeout = 10000; timeout > 0; timeout--) {
		if (!(readl(priv->hp + HP_ROOT_CLK_CTRL0) & HP_SOC_CLK_UPDATE))
			break;
		cpu_relax();
	}
	spin_unlock_irqrestore(&priv->lock, flags);

	if (!timeout)
		return -ETIMEDOUT;
	if (clk_hw_get_rate(hw) != rate)
		return -EIO;

	return 0;
}

static u8 esp32s31_pll_div_get_parent(struct clk_hw *hw)
{
	struct esp32s31_clk *clk = to_esp32s31_clk(hw);

	if (clk->reg != HP_REF_50M_CTRL0)
		return 0;

	return readl(clk->priv->hp + HP_REF_500M_CTRL0) & BIT(0) ? 1 : 0;
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

static long esp32s31_i2s_round_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long *parent_rate)
{
	if (rate < 1000000 || rate > ESP32S31_XTAL_RATE)
		return -EINVAL;
	return rate;
}

static int esp32s31_i2s_set_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long parent_rate)
{
	struct esp32s31_clk *clk = to_esp32s31_clk(hw);
	struct esp32s31_clk_priv *priv = clk->priv;
	unsigned int port = clk->kind - ESP32S31_CLK_KIND_I2S0;
	u32 rx_reg = HP_I2S0_RX_CTRL0 + port * HP_I2S_CTRL_STRIDE;
	u32 tx_reg = HP_I2S0_TX_CTRL0 + port * HP_I2S_CTRL_STRIDE;
	u32 rx_div_reg = HP_I2S0_RX_DIV_CTRL0 + port * HP_I2S_CTRL_STRIDE;
	u32 tx_div_reg = HP_I2S0_TX_DIV_CTRL0 + port * HP_I2S_CTRL_STRIDE;
	unsigned long flags;
	unsigned long numerator, denominator;
	u32 integer, x = 0, y = 0, z = 0, yn1 = 0, fractional = 0;
	u64 actual_denominator;

	if (rate < 1000000 || rate > ESP32S31_XTAL_RATE)
		return -EINVAL;
	integer = ESP32S31_XTAL_RATE / rate;
	if (!integer || integer > FIELD_MAX(HP_I2S_CORE_DIV_N))
		return -ERANGE;
	numerator = ESP32S31_XTAL_RATE - (unsigned long)integer * rate;
	denominator = rate;
	if (numerator) {
		rational_best_approximation(numerator, denominator, 511, 511,
					    &numerator, &denominator);
		yn1 = numerator * 2 > denominator;
		z = yn1 ? denominator - numerator : numerator;
		if (!z)
			return -ERANGE;
		x = denominator / z - 1;
		y = denominator % z;
		if (x > 511 || y > 511 || z > 511)
			return -ERANGE;
		fractional = FIELD_PREP(HP_I2S_DIV_X, x) |
			     FIELD_PREP(HP_I2S_DIV_Y, y) |
			     FIELD_PREP(HP_I2S_DIV_Z, z) |
			     (yn1 ? HP_I2S_DIV_YN1 : 0);
	}

	spin_lock_irqsave(&priv->lock, flags);
	/* Preserve the hardware workaround used during prepare. */
	esp32s31_rmw(priv->hp, rx_reg, HP_I2S_CORE_DIV_N,
		     FIELD_PREP(HP_I2S_CORE_DIV_N, 2));
	esp32s31_rmw(priv->hp, tx_reg, HP_I2S_CORE_DIV_N,
		     FIELD_PREP(HP_I2S_CORE_DIV_N, 2));
	writel(FIELD_PREP(HP_I2S_DIV_Y, 1), priv->hp + rx_div_reg);
	writel(FIELD_PREP(HP_I2S_DIV_Y, 1), priv->hp + tx_div_reg);
	writel(0, priv->hp + rx_div_reg);
	writel(0, priv->hp + tx_div_reg);
	writel(fractional, priv->hp + rx_div_reg);
	writel(fractional, priv->hp + tx_div_reg);
	esp32s31_rmw(priv->hp, rx_reg, HP_I2S_CORE_DIV_N,
		     FIELD_PREP(HP_I2S_CORE_DIV_N, integer));
	esp32s31_rmw(priv->hp, tx_reg, HP_I2S_CORE_DIV_N,
		     FIELD_PREP(HP_I2S_CORE_DIV_N, integer));
	actual_denominator = (u64)integer * denominator + numerator;
	clk->fixed_rate = div64_u64((u64)ESP32S31_XTAL_RATE * denominator,
				    actual_denominator);
	spin_unlock_irqrestore(&priv->lock, flags);
	return 0;
}

static const struct clk_ops esp32s31_fixed_ops = {
	.recalc_rate = esp32s31_clk_recalc_rate,
};

static const struct clk_ops esp32s31_cpu_ops = {
	.get_parent = esp32s31_cpu_get_parent,
	.recalc_rate = esp32s31_clk_recalc_rate,
	.round_rate = esp32s31_cpu_round_rate,
	.set_rate = esp32s31_cpu_set_rate,
};

static const struct clk_ops esp32s31_pll_div_ops = {
	.get_parent = esp32s31_pll_div_get_parent,
	.recalc_rate = esp32s31_clk_recalc_rate,
};

static const struct clk_ops esp32s31_gate_ops = {
	.prepare = esp32s31_clk_prepare,
	.unprepare = esp32s31_clk_unprepare,
	.is_prepared = esp32s31_clk_is_prepared,
	.recalc_rate = esp32s31_clk_recalc_rate,
};

static const struct clk_ops esp32s31_txc_ops = {
	.prepare = esp32s31_clk_prepare,
	.unprepare = esp32s31_clk_unprepare,
	.is_prepared = esp32s31_clk_is_prepared,
	.recalc_rate = esp32s31_clk_recalc_rate,
	.round_rate = esp32s31_txc_round_rate,
	.set_rate = esp32s31_txc_set_rate,
};

static const struct clk_ops esp32s31_i2s_ops = {
	.prepare = esp32s31_clk_prepare,
	.unprepare = esp32s31_clk_unprepare,
	.is_prepared = esp32s31_clk_is_prepared,
	.recalc_rate = esp32s31_clk_recalc_rate,
	.round_rate = esp32s31_i2s_round_rate,
	.set_rate = esp32s31_i2s_set_rate,
};

static bool esp32s31_clk_is_critical(unsigned int id)
{
	switch (id) {
	case ESP32S31_CLK_XTAL:
	case ESP32S31_CLK_SYS:
	case ESP32S31_CLK_MPLL:
	case ESP32S31_CLK_UART0:
	case ESP32S31_CLK_SYSTIMER:
	case ESP32S31_CLK_CPLL:
	case ESP32S31_CLK_BBPLL:
	case ESP32S31_CLK_CPU:
	case ESP32S31_CLK_MEM:
	case ESP32S31_CLK_APB:
		return true;
	default:
		return false;
	}
}

static int __esp32s31_register_clk(struct device *dev,
				   struct esp32s31_clk_priv *priv,
				   unsigned int id, const char *name,
				   enum esp32s31_clk_kind kind,
				   unsigned long rate,
				   const char * const *parent_names,
				   unsigned int num_parents)
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
	case ESP32S31_CLK_KIND_XTAL:
	case ESP32S31_CLK_KIND_CPLL:
	case ESP32S31_CLK_KIND_MPLL:
	case ESP32S31_CLK_KIND_APLL:
	case ESP32S31_CLK_KIND_MEM:
	case ESP32S31_CLK_KIND_SYS:
	case ESP32S31_CLK_KIND_APB:
		ops = &esp32s31_fixed_ops;
		break;
	case ESP32S31_CLK_KIND_PLL_DIV:
		ops = &esp32s31_pll_div_ops;
		break;
	case ESP32S31_CLK_KIND_CPU:
		ops = &esp32s31_cpu_ops;
		break;
	case ESP32S31_CLK_KIND_EMAC_TXC:
		ops = &esp32s31_txc_ops;
		break;
	case ESP32S31_CLK_KIND_I2S0:
	case ESP32S31_CLK_KIND_I2S1:
		ops = &esp32s31_i2s_ops;
		break;
	default:
		ops = &esp32s31_gate_ops;
		break;
	}

	init.name = name;
	init.ops = ops;
	init.parent_names = parent_names;
	init.num_parents = num_parents;
	if (esp32s31_clk_is_critical(id))
		init.flags |= CLK_IS_CRITICAL;
	if (kind == ESP32S31_CLK_KIND_CPU || kind == ESP32S31_CLK_KIND_MEM ||
	    kind == ESP32S31_CLK_KIND_SYS || kind == ESP32S31_CLK_KIND_APB)
		init.flags |= CLK_GET_RATE_NOCACHE;

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

static int esp32s31_register_clk(struct device *dev,
				 struct esp32s31_clk_priv *priv,
				 unsigned int id, const char *name,
				 enum esp32s31_clk_kind kind,
				 unsigned long rate)
{
	return __esp32s31_register_clk(dev, priv, id, name, kind, rate,
				       NULL, 0);
}

static int esp32s31_register_parented_clk(struct device *dev,
					  struct esp32s31_clk_priv *priv,
					  unsigned int id, const char *name,
					  enum esp32s31_clk_kind kind,
					  unsigned long rate,
					  const char * const *parent_names,
					  unsigned int num_parents)
{
	return __esp32s31_register_clk(dev, priv, id, name, kind, rate,
				       parent_names, num_parents);
}

static int esp32s31_register_pll_div(struct device *dev,
				     struct esp32s31_clk_priv *priv,
				     unsigned int id, const char *name, u16 reg,
				     const char * const *parent_names,
				     unsigned int num_parents)
{
	struct esp32s31_clk *clk;
	int ret;

	ret = esp32s31_register_parented_clk(dev, priv, id, name,
					 ESP32S31_CLK_KIND_PLL_DIV, 0,
					 parent_names, num_parents);
	if (ret)
		return ret;

	clk = to_esp32s31_clk(priv->onecell->hws[id]);
	clk->reg = reg;

	return 0;
}

static int esp32s31_register_gate(struct device *dev,
				  struct esp32s31_clk_priv *priv,
				  unsigned int id, const char *name,
				  unsigned long rate, u16 reg,
				  u32 enable_mask, u32 reset_mask,
				  u32 force_norst_mask)
{
	struct esp32s31_clk *clk;
	int ret;

	ret = esp32s31_register_clk(dev, priv, id, name,
				    ESP32S31_CLK_KIND_GENERIC, rate);
	if (ret)
		return ret;

	clk = to_esp32s31_clk(priv->onecell->hws[id]);
	clk->reg = reg;
	clk->enable_mask = enable_mask;
	clk->reset_mask = reset_mask;
	clk->force_norst_mask = force_norst_mask;

	return 0;
}

static const char * const esp32s31_xtal_parent[] = { "xtal" };
static const char * const esp32s31_bbpll_parent[] = { "bbpll" };
static const char * const esp32s31_mpll_parent[] = { "mpll" };
static const char * const esp32s31_ref50_parents[] = { "cpll", "mpll" };
static const char * const esp32s31_cpu_parents[] = {
	"xtal", "cpll", "rc-fast", "pll-f240",
};
static const char * const esp32s31_cpu_parent[] = { "cpu" };
static const char * const esp32s31_sys_parent[] = { "sys" };

static ssize_t clocks_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct esp32s31_clk_priv *priv = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned int id;

	for (id = 0; id < priv->onecell->num; id++) {
		struct clk_hw *hw = priv->onecell->hws[id];

		if (!hw)
			continue;
		len += sysfs_emit_at(buf, len,
			"%u %s state=%s critical=%u rate=%lu\n", id,
			clk_hw_get_name(hw),
			clk_hw_is_prepared(hw) ? "on" : "off",
			esp32s31_clk_is_critical(id), clk_hw_get_rate(hw));
		if (len >= PAGE_SIZE)
			break;
	}

	return len;
}
static DEVICE_ATTR_RO(clocks);

static struct attribute *esp32s31_clk_attrs[] = {
	&dev_attr_clocks.attr,
	NULL,
};

static const struct attribute_group esp32s31_clk_attr_group = {
	.attrs = esp32s31_clk_attrs,
};

static int esp32s31_clk_poweroff_prepare(struct sys_off_data *data);

static int esp32s31_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp32s31_clk_priv *priv;
	struct clk_hw_onecell_data *onecell;
	struct resource *res;
	unsigned long apb_rate, sys_rate;
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

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "hp-alive-sys");
	if (!res)
		return -EINVAL;
	priv->hp_alive = devm_ioremap(dev, res->start, resource_size(res));
	if (!priv->hp_alive)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "lp-aon-clkrst");
	if (!res)
		return -EINVAL;
	priv->lp_aon = devm_ioremap(dev, res->start, resource_size(res));
	if (!priv->lp_aon)
		return -ENOMEM;

	spin_lock_init(&priv->lock);
	priv->onecell = onecell;
	onecell->num = ESP32S31_CLK_NR;

	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_XTAL, "xtal",
				    ESP32S31_CLK_KIND_XTAL, 0);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_RC_FAST, "rc-fast",
				    ESP32S31_CLK_KIND_FIXED, ESP32S31_RC_FAST_RATE);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_RC_SLOW, "rc-slow",
				    ESP32S31_CLK_KIND_FIXED, ESP32S31_RC_SLOW_RATE);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_XTAL32K, "xtal32k",
				    ESP32S31_CLK_KIND_FIXED, ESP32S31_XTAL32K_RATE);
	if (ret)
		return ret;
	ret = esp32s31_register_parented_clk(dev, priv, ESP32S31_CLK_CPLL,
					     "cpll", ESP32S31_CLK_KIND_CPLL, 0,
					     esp32s31_xtal_parent, 1);
	if (ret)
		return ret;
	ret = esp32s31_register_parented_clk(dev, priv, ESP32S31_CLK_BBPLL,
					     "bbpll", ESP32S31_CLK_KIND_FIXED,
					     ESP32S31_BBPLL_RATE,
					     esp32s31_xtal_parent, 1);
	if (ret)
		return ret;
	ret = esp32s31_register_parented_clk(dev, priv, ESP32S31_CLK_MPLL,
					     "mpll", ESP32S31_CLK_KIND_MPLL, 0,
					     esp32s31_xtal_parent, 1);
	if (ret)
		return ret;
	ret = esp32s31_register_parented_clk(dev, priv, ESP32S31_CLK_APLL,
					     "apll", ESP32S31_CLK_KIND_APLL, 0,
					     esp32s31_xtal_parent, 1);
	if (ret)
		return ret;
	ret = esp32s31_register_pll_div(dev, priv, ESP32S31_CLK_PLL_F20,
					"pll-f20", HP_REF_20M_CTRL0,
					esp32s31_bbpll_parent, 1);
	if (ret)
		return ret;
	ret = esp32s31_register_pll_div(dev, priv, ESP32S31_CLK_PLL_F25,
					"pll-f25", HP_REF_25M_CTRL0,
					esp32s31_mpll_parent, 1);
	if (ret)
		return ret;
	ret = esp32s31_register_pll_div(dev, priv, ESP32S31_CLK_PLL_F50,
					"pll-f50", HP_REF_50M_CTRL0,
					esp32s31_ref50_parents, 2);
	if (ret)
		return ret;
	ret = esp32s31_register_pll_div(dev, priv, ESP32S31_CLK_PLL_F80,
					"pll-f80", HP_REF_80M_CTRL0,
					esp32s31_bbpll_parent, 1);
	if (ret)
		return ret;
	ret = esp32s31_register_pll_div(dev, priv, ESP32S31_CLK_PLL_F160,
					"pll-f160", HP_REF_160M_CTRL0,
					esp32s31_bbpll_parent, 1);
	if (ret)
		return ret;
	ret = esp32s31_register_pll_div(dev, priv, ESP32S31_CLK_PLL_F240,
					"pll-f240", HP_REF_240M_CTRL0,
					esp32s31_bbpll_parent, 1);
	if (ret)
		return ret;
	ret = esp32s31_register_pll_div(dev, priv, ESP32S31_CLK_PLL_F120,
					"pll-f120", HP_REF_120M_CTRL0,
					esp32s31_bbpll_parent, 1);
	if (ret)
		return ret;
	ret = esp32s31_register_pll_div(dev, priv, ESP32S31_CLK_PLL_F60,
					"pll-f60", HP_REF_60M_CTRL0,
					esp32s31_bbpll_parent, 1);
	if (ret)
		return ret;
	ret = esp32s31_register_parented_clk(dev, priv, ESP32S31_CLK_XTAL_D2,
					     "xtal-d2", ESP32S31_CLK_KIND_FIXED,
					     ESP32S31_XTAL_RATE / 2,
					     esp32s31_xtal_parent, 1);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_RTC_FAST,
				    "rtc-fast", ESP32S31_CLK_KIND_FIXED,
				    ESP32S31_RC_FAST_RATE);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_RTC_SLOW,
				    "rtc-slow", ESP32S31_CLK_KIND_FIXED,
				    ESP32S31_RC_SLOW_RATE);
	if (ret)
		return ret;
	ret = esp32s31_register_parented_clk(dev, priv, ESP32S31_CLK_CPU,
					     "cpu", ESP32S31_CLK_KIND_CPU, 0,
					     esp32s31_cpu_parents,
					     ARRAY_SIZE(esp32s31_cpu_parents));
	if (ret)
		return ret;
	ret = esp32s31_register_parented_clk(dev, priv, ESP32S31_CLK_MEM,
					     "mem", ESP32S31_CLK_KIND_MEM, 0,
					     esp32s31_cpu_parent, 1);
	if (ret)
		return ret;
	ret = esp32s31_register_parented_clk(dev, priv, ESP32S31_CLK_SYS,
					     "sys", ESP32S31_CLK_KIND_SYS, 0,
					     esp32s31_cpu_parent, 1);
	if (ret)
		return ret;
	ret = esp32s31_register_parented_clk(dev, priv, ESP32S31_CLK_APB,
					     "apb", ESP32S31_CLK_KIND_APB, 0,
					     esp32s31_sys_parent, 1);
	if (ret)
		return ret;

	sys_rate = clk_hw_get_rate(onecell->hws[ESP32S31_CLK_SYS]);
	apb_rate = clk_hw_get_rate(onecell->hws[ESP32S31_CLK_APB]);
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_SDMMC_BIU,
				    "sdmmc-biu", ESP32S31_CLK_KIND_SDMMC_BIU,
				    sys_rate);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_SDMMC_CIU,
				    "sdmmc-ciu", ESP32S31_CLK_KIND_SDMMC_CIU,
				    ESP32S31_SDMMC_CIU_RATE);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_EMAC_STMMACETH,
				    "emac-stmmaceth", ESP32S31_CLK_KIND_EMAC_BUS,
				    sys_rate);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_EMAC_PCLK,
				    "emac-pclk", ESP32S31_CLK_KIND_EMAC_BUS,
				    sys_rate);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_EMAC_PTP_REF,
				    "emac-ptp-ref", ESP32S31_CLK_KIND_EMAC_PTP,
				    ESP32S31_XTAL_RATE);
	if (ret)
		return ret;
	ret = esp32s31_register_parented_clk(dev, priv,
					     ESP32S31_CLK_EMAC_RGMII_TXC,
					     "emac-rgmii-txc",
					     ESP32S31_CLK_KIND_EMAC_TXC,
					     125000000,
					     esp32s31_mpll_parent, 1);
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
	ret = esp32s31_register_gate(dev, priv, ESP32S31_CLK_WDT0, "wdt0",
				     ESP32S31_XTAL_RATE, 0x114,
				     BIT(0) | BIT(11), BIT(1), BIT(2));
	if (ret)
		return ret;
	ret = esp32s31_register_gate(dev, priv, ESP32S31_CLK_SYSTIMER,
				     "systimer", 16000000UL, 0x120,
				     BIT(0) | BIT(4), BIT(1), BIT(2));
	if (ret)
		return ret;
	ret = esp32s31_register_gate(dev, priv, ESP32S31_CLK_AHB_GDMA,
				     "ahb-gdma", sys_rate, 0x7c,
				     BIT(0), BIT(1), BIT(2));
	if (ret)
		return ret;
	ret = esp32s31_register_gate(dev, priv, ESP32S31_CLK_UHCI,
				     "uhci", sys_rate, 0x84,
				     BIT(0) | BIT(1), BIT(2), BIT(3));
	if (ret)
		return ret;
	ret = esp32s31_register_gate(dev, priv, ESP32S31_CLK_LEDC0, "ledc0",
				     ESP32S31_XTAL_RATE, 0x148,
				     BIT(0) | BIT(5), BIT(1), BIT(2));
	if (ret)
		return ret;
	ret = esp32s31_register_gate(dev, priv, ESP32S31_CLK_LEDC1, "ledc1",
				     ESP32S31_XTAL_RATE, 0x148,
				     BIT(6) | BIT(11), BIT(7), BIT(8));
	if (ret)
		return ret;
	ret = esp32s31_register_gate(dev, priv, ESP32S31_CLK_MCPWM0, "mcpwm0",
				     sys_rate, 0x124,
				     BIT(0) | BIT(5), BIT(1), BIT(2));
	if (ret)
		return ret;
	ret = esp32s31_register_gate(dev, priv, ESP32S31_CLK_MCPWM1, "mcpwm1",
				     sys_rate, 0x128,
				     BIT(0) | BIT(5), BIT(1), BIT(2));
	if (ret)
		return ret;
	ret = esp32s31_register_gate(dev, priv, ESP32S31_CLK_MCPWM2, "mcpwm2",
				     sys_rate, 0x12c,
				     BIT(0) | BIT(5), BIT(1), BIT(2));
	if (ret)
		return ret;
	ret = esp32s31_register_gate(dev, priv, ESP32S31_CLK_MCPWM3, "mcpwm3",
				     sys_rate, 0x130,
				     BIT(0) | BIT(5), BIT(1), BIT(2));
	if (ret)
		return ret;
	ret = esp32s31_register_gate(dev, priv, ESP32S31_CLK_PCNT0, "pcnt0",
				     apb_rate, 0x138,
				     BIT(0), BIT(1), BIT(2));
	if (ret)
		return ret;
	ret = esp32s31_register_gate(dev, priv, ESP32S31_CLK_PCNT1, "pcnt1",
				     apb_rate, 0x138,
				     BIT(3), BIT(4), BIT(5));
	if (ret)
		return ret;
	ret = esp32s31_register_gate(dev, priv, ESP32S31_CLK_I2C0, "i2c0",
				     ESP32S31_XTAL_RATE, 0xdc,
				     BIT(0) | BIT(4), BIT(1), BIT(2));
	if (ret)
		return ret;
	ret = esp32s31_register_gate(dev, priv, ESP32S31_CLK_I2C1, "i2c1",
				     ESP32S31_XTAL_RATE, 0xe0,
				     BIT(0) | BIT(4), BIT(1), BIT(2));
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_TWAI0, "twai0",
				    ESP32S31_CLK_KIND_TWAI0,
				    80000000UL);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_TWAI1, "twai1",
				    ESP32S31_CLK_KIND_TWAI1,
				    80000000UL);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_UART3,
				    "uart3", ESP32S31_CLK_KIND_UART3,
				    ESP32S31_XTAL_RATE);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_GPSPI2,
				    "gpspi2", ESP32S31_CLK_KIND_GPSPI2,
				    ESP32S31_GPSPI_RATE);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_GPSPI3,
				    "gpspi3", ESP32S31_CLK_KIND_GPSPI3,
				    ESP32S31_GPSPI_RATE);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_I2S0,
				    "i2s0", ESP32S31_CLK_KIND_I2S0,
				    12288000UL);
	if (ret)
		return ret;
	ret = esp32s31_register_clk(dev, priv, ESP32S31_CLK_I2S1,
				    "i2s1", ESP32S31_CLK_KIND_I2S1,
				    12288000UL);
	if (ret)
		return ret;

	priv->rcdev.owner = THIS_MODULE;
	priv->rcdev.nr_resets = ESP32S31_RST_NR;
	priv->rcdev.ops = &esp32s31_reset_ops;
	priv->rcdev.of_node = dev->of_node;
	priv->rcdev.dev = dev;
	ret = devm_reset_controller_register(dev, &priv->rcdev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register reset controller\n");

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, onecell);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, priv);
	ret = devm_register_sys_off_handler(dev, SYS_OFF_MODE_POWER_OFF_PREPARE,
					 SYS_OFF_PRIO_DEFAULT,
					 esp32s31_clk_poweroff_prepare, priv);
	if (ret)
		return dev_err_probe(dev, ret, "poweroff clock handoff unavailable\n");
	ret = devm_device_add_group(dev, &esp32s31_clk_attr_group);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to add clock state attributes\n");
	dev_info(dev, "boot clocks: cpu=%lu mem=%lu sys=%lu apb=%lu Hz\n",
		 clk_hw_get_rate(onecell->hws[ESP32S31_CLK_CPU]),
		 clk_hw_get_rate(onecell->hws[ESP32S31_CLK_MEM]), sys_rate, apb_rate);

	return 0;
}

/* Power-off must also work without loading the optional LP remoteproc.
 * This phase still has both harts online for SOC_CLK_UPDATE. */
static int esp32s31_clk_poweroff_prepare(struct sys_off_data *data)
{
	struct esp32s31_clk_priv *priv = data->cb_data;
	struct clk *cpu;
	int ret;

	cpu = clk_hw_get_clk(priv->onecell->hws[ESP32S31_CLK_CPU], NULL);
	if (IS_ERR(cpu))
		return NOTIFY_BAD;
	ret = clk_set_rate(cpu, ESP32S31_XTAL_RATE);
	clk_put(cpu);
	if (ret)
		pr_err("S31 poweroff: XTAL handoff failed: %d\n", ret);
	return ret ? NOTIFY_BAD : NOTIFY_DONE;
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
