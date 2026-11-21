// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-P4 MIPI DSI display bring-up for a simple fbdev driver.
 *
 * This file is a Linux port of the ESP-IDF MIPI DSI host, DSI bridge and
 * DW-GDMA bring-up.  Source provenance (ESP-IDF commit
 * a602e67b0bf9ee0806dc4e1df7afc9affedf5c33):
 *
 *  - components/esp_hal_lcd/mipi_dsi_hal.c
 *      mipi_dsi_hal_init(), mipi_dsi_hal_configure_phy_pll(),
 *      mipi_dsi_hal_host_dpi_set_horizontal/vertical_timing(),
 *      mipi_dsi_hal_host_dpi_calculate_divider(), PHY test-port helpers,
 *      generic write/read packet helpers.
 *  - components/esp_hal_lcd/esp32p4/include/hal/mipi_dsi_host_ll.h,
 *    mipi_dsi_brg_ll.h, mipi_dsi_phy_ll.h
 *  - components/esp_hal_lcd/esp32p4/include/hal/mipi_dsi_ll.h
 *  - components/esp_lcd/dsi/esp_lcd_mipi_dsi_bus.c
 *      esp_lcd_new_dsi_bus(): bus clock, register reset, PHY clock sources,
 *      mipi_dsi_hal_init(), PLL lock/lane-stop waits, host packet config.
 *  - components/esp_lcd/dsi/esp_lcd_panel_dpi.c
 *      esp_lcd_new_panel_dpi()/dpi_panel_init() and
 *      mipi_dsi_dma_trans_done_cb() re-arm logic.
 *  - components/esp_lcd/dsi/esp_lcd_panel_io_dbi.c
 *      DCS short/long write and DCS read through the host generic interface.
 *  - components/esp_driver_dma/src/dw_gdma.c and
 *    components/esp_hal_dma/esp32p4/include/hal/dw_gdma_ll.h
 *      VDMA channel setup and link-list item programming.
 *  - components/esp_hal_pmu/esp32p4/include/hal/ldo_ll.h and
 *    components/esp_hw_support/ldo/esp_ldo_regulator.c
 *      PHY LDO (channel 3, 2500 mV).
 *  - firmware/p4-hardware-loader/main/display_handoff.c (same board tree)
 *      Board-specific sequence for the Raspberry Pi Touch Display 2:
 *      MCU reset, page select, identity probe, init table, post commands.
 *
 * The MIPI DSI register struct headers are copied verbatim from ESP-IDF and
 * live in include/video/esp32p4-dsi/.
 *
 * Hardware behaviour is annotated against the ESP32-P4 TRM (v1.0):
 *  - Chapter 44 "MIPI DSI": host, DPI video mode, PHY test interface and the
 *    DSI bridge.
 *  - Chapter 7 "VDMA Controller (VDMA)": DMAC channels, linked-list items,
 *    block markers and flow control (base 0x5008_1000 in Table 9.3-2).
 *  - Chapter 12 "Reset and Clock": HP_SYS_CLKRST peripheral clocks/resets.
 *  - Chapter 16 "Low-Power Management": PMU and the four EXT_LDO regulators.
 *  - Chapter 10 "eFuse Controller": BLOCK2 LDO calibration fields.
 *
 * The IDF-derived portions are licensed Apache-2.0 OR MIT.
 */

#include <linux/backlight.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/gcd.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include "esp32p4-dsi.h"
#include <video/esp32p4-dsi/dw_gdma_struct.h>
#include <video/esp32p4-dsi/mipi_dsi_bridge_struct.h>
#include <video/esp32p4-dsi/mipi_dsi_host_struct.h>

#include "esp32p4-dsi-panel.h"

#include <linux/soc/espressif/esp32p4.h>

#define P4_DSI_FB_COMPAT	"espressif,esp32p4-idf-framebuffer"

/* CACHE_LL_L2MEM_NON_CACHE_ADDR() for internal SRAM on this SoC. */
#define DSI_SRAM_NC_OFFSET	0x40000000U

/* Register bases from ESP-IDF soc/reg_base.h (see task description). */
#define DSI_HOST_BASE			0x500a0000
#define DSI_BRIDGE_OFFSET		0x800
#define DSI_BRIDGE_MEM_BASE		0x50105000
#define DW_GDMA_BASE			0x50081000
#define HP_SYS_CLKRST_BASE		0x500e6000
#define PMU_BASE			0x50115000
#define EFUSE_BASE			0x5012d000

/* HP_SYS_CLKRST register offsets (soc/esp32p4/register/hw_ver3). */
#define HP_SYS_SOC_CLK_CTRL0		0x14
#define HP_SYS_SOC_CLK_CTRL1		0x18
#define HP_SYS_PERI_CLK_CTRL02		0x38
#define HP_SYS_PERI_CLK_CTRL03		0x3c
#define HP_SYS_HP_RST_EN0		0xc0

#define HP_SYS_GDMA_CPU_CLK_EN		BIT(13)
#define HP_SYS_GDMA_SYS_CLK_EN		BIT(5)
#define HP_SYS_DSI_SYS_CLK_EN		BIT(12)
#define HP_SYS_DSI_BRG_RST_EN		BIT(26)
#define HP_SYS_GDMA_RST_EN		BIT(21)

/* PMU external LDO register layout (pmu_struct.h). */
#define PMU_EXT_LDO_OFFSET		0x1b8
#define PMU_EXT_LDO_STRIDE		8
#define PMU_LDO_XPD			BIT(8)
#define PMU_LDO_FORCE_TIEH_SEL		BIT(7)
#define PMU_LDO_TIEH_SEL_MASK		GENMASK(11, 9)
#define PMU_LDO_TIEH			BIT(14)
#define PMU_LDO_ANA_MUL_MASK		GENMASK(25, 23)
#define PMU_LDO_ANA_MUL_SHIFT		23
#define PMU_LDO_ANA_VDET		BIT(26)
#define PMU_LDO_ANA_CUR_LIM		BIT(27)
#define PMU_LDO_ANA_DREF_MASK		GENMASK(31, 28)
#define PMU_LDO_ANA_DREF_SHIFT		28

/* eFuse calibration registers (efuse_struct.h, block 1). */
#define EFUSE_RD_MAC_SYS2		0x4c
#define EFUSE_RD_MAC_SYS3		0x50
#define EFUSE_RD_MAC_SYS4		0x54
#define EFUSE_BLK_VERSION_MAJOR		GENMASK(12, 11)

/* Board DSI/DPI timing, from display_handoff.c and the panel datasheet. */
#define DSI_NUM_DATA_LANES		2
/* ILI9881C datasheet rates two data lanes at up to 850 Mbps/lane.
 * RGB565 at 80 MHz requires 640 Mbps/lane during active video.
 */
#define DSI_LANE_BIT_RATE_MBPS		800
#define DSI_PHY_REF_MHZ			20
#define DSI_DPI_CLK_SRC_MHZ		240	/* MIPI_DSI_DPI_CLK_SRC_DEFAULT */
#define DSI_DPI_CLK_MHZ			80
#define DSI_DPI_CLK_SRC_SEL		1	/* PLL_F240M */
#define DSI_DPI_CLK_DIV			3	/* 240 MHz / 3 = 80 MHz */
#define DSI_DPI_TIMEOUT_CLK_MHZ		10
#define DSI_DPI_ESCAPE_CLK_MHZ		18

#define DSI_HSA				2
#define DSI_HBP				105
#define DSI_HFP				110
#define DSI_VSA				2
#define DSI_VBP				100
#define DSI_VFP				100

#define DSI_MCU_I2C_ADDR		0x45
#define DSI_MCU_BRIGHTNESS		0x90

/*
 * DW_GDMA: handshaking peripheral and channel used for scanout.
 * TODO(verify): the DT does not describe a DMA channel; channel 0 is assumed
 * to be free (the retained firmware used the first allocated channel).
 */
#define DSI_GDMA_CHANNEL		0
#define DSI_GDMA_HW_HANDSHAKE_DSI	0
#define DSI_GDMA_MASTER_PORT_MEMORY	1
#define DSI_GDMA_MASTER_PORT_DSI	0
#define DSI_DMA_EVENT_FRAME_DONE	BIT(1)
#define DSI_DMA_EVENT_ERRORS		GENMASK(13, 5)
#define DSI_GDMA_LLI_ALIGN		64
#define DSI_DMA_MULTBLK_RELOAD		1
#define DSI_DMA_MULTBLK_LINKED		3

/* The DW_GDMA interrupt source on ESP32-P4 (soc/interrupts.h). */
#define DSI_GDMA_HWIRQ			24

/* Bridge start-up heuristic: wait until this many bytes are buffered. */
#define DSI_BRIDGE_FIFO_WORDS		1024
#define DSI_BRIDGE_BURST_WORDS		256
#define DSI_BRIDGE_PREFILL_WORDS		(DSI_BRIDGE_FIFO_WORDS - DSI_BRIDGE_BURST_WORDS)
#define DSI_BRIDGE_PREFILL_US		2000

#define DSI_DCS_SHORT_WRITE_0		0x05
#define DSI_DCS_SHORT_WRITE_1		0x15
#define DSI_DCS_LONG_WRITE		0x39
#define DSI_DCS_READ_0			0x06
#define DSI_SET_MAXIMUM_RETURN_PKT	0x37

#define DSI_FIFO_TIMEOUT_US		100000

/* MIPI DSI LL flow-control / video constants (mipi_dsi_*_ll.h). */
#define DSI_LL_FLOW_CONTROLLER_DMA	0
/* TRM ch.44 MIPI DSI - DSI_HOST_VID_MODE_TYPE. */
#define DSI_LL_VIDEO_BURST_SYNC_PULSES	2
#define DSI_LL_CLOCK_LANE_STATE_HS	1
#define DSI_LL_TRANS_SPEED_HS		0
#define DSI_LL_TRANS_SPEED_LP		1

#define DSI_COLOR_CODE_16BIT_CONFIG1	0

struct esp32p4_dsi_lli {
	dmac_chn_sar0_reg_t sar_lo;
	dmac_chn_sar1_reg_t sar_hi;
	dmac_chn_dar0_reg_t dar_lo;
	dmac_chn_dar1_reg_t dar_hi;
	dmac_chn_block_ts0_reg_t block_ts_lo;
	u32 reserved_14;
	dmac_chn_llp0_reg_t llp_lo;
	dmac_chn_llp1_reg_t llp_hi;
	dmac_chn_ctl0_reg_t ctrl_lo;
	dmac_chn_ctl1_reg_t ctrl_hi;
	dmac_chn_sstat0_reg_t sstat;
	dmac_chn_dstat0_reg_t dstat;
	dmac_chn_status0_reg_t status_lo;
	dmac_chn_status1_reg_t status_hi;
	u32 reserved_38;
	u32 reserved_3c;
} __aligned(DSI_GDMA_LLI_ALIGN);

struct soc_mipi_dsi_phy_pll_range {
	u16 start_mbps;
	u16 end_mbps;
	u8 hs_freq_sel;
};

/*
 * PHY PLL frequency ranges, copied from
 * components/esp_hal_lcd/esp32p4/mipi_dsi_periph.c.
 */
static const struct soc_mipi_dsi_phy_pll_range dsi_phy_pll_ranges[] = {
	{ 80, 89, 0x00 },	{ 90, 99, 0x10 },	{ 100, 109, 0x20 },
	{ 110, 129, 0x01 },	{ 130, 139, 0x11 },	{ 140, 149, 0x21 },
	{ 150, 169, 0x02 },	{ 170, 179, 0x12 },	{ 180, 199, 0x22 },
	{ 200, 219, 0x03 },	{ 220, 239, 0x13 },	{ 240, 249, 0x23 },
	{ 250, 269, 0x04 },	{ 270, 299, 0x14 },	{ 300, 329, 0x05 },
	{ 330, 359, 0x15 },	{ 360, 399, 0x25 },	{ 400, 449, 0x06 },
	{ 450, 499, 0x16 },	{ 500, 549, 0x07 },	{ 550, 599, 0x17 },
	{ 600, 649, 0x08 },	{ 650, 699, 0x18 },	{ 700, 749, 0x09 },
	{ 750, 799, 0x19 },	{ 800, 849, 0x29 },	{ 850, 899, 0x39 },
	{ 900, 949, 0x0a },	{ 950, 999, 0x1a },	{ 1000, 1049, 0x2a },
	{ 1050, 1099, 0x3a },	{ 1100, 1149, 0x0b },	{ 1150, 1199, 0x1b },
	{ 1200, 1249, 0x2b },	{ 1250, 1299, 0x3b },	{ 1300, 1349, 0x0c },
	{ 1350, 1399, 0x1c },	{ 1400, 1449, 0x2c },	{ 1450, 1500, 0x3c },
};

struct esp32p4_dsi {
	struct device *dev;
	phys_addr_t fb_phys;
	u32 width;
	u32 height;

	void __iomem *host_base;
	dsi_host_dev_t *host;
	dsi_brg_dev_t *bridge;
	void __iomem *clkrst;
	void __iomem *gdma_base;
	dw_gdma_dev_t *gdma;
	void __iomem *pmu;
	void __iomem *efuse;
	struct i2c_client *mcu;
	struct gpio_desc *panel_reset;
	struct backlight_device *backlight;

	struct esp32p4_dsi_lli *lli;
	dma_addr_t lli_dma;

	unsigned int irq;
	u32 dpi_src_mhz;
	u32 dpi_div;
	u32 dpi_real_uhz;

	spinlock_t lock;	/* protects started/fatal and the DMA channel */
	bool started;
	bool fatal;
	bool ldo_on;
	bool hw_ready;
	bool lli_in_sram;
	u32 block_size;
};

static inline u32 dsi_round_div(u64 num, u64 den)
{
	/* rv32 has no 64-bit divide instruction; use the kernel helper. */
	return (u32)div64_u64(num + den / 2, den);
}

static inline u32 dsi_field_prep(u32 val, u32 mask)
{
	return (val << __ffs(mask)) & mask;
}

static void dsi_update_bits(void __iomem *addr, u32 mask, u32 val)
{
	u32 tmp = readl(addr);

	tmp = (tmp & ~mask) | (val & mask);
	writel(tmp, addr);
}

/* ------------------------------------------------------------------------- */
/* PHY test-port access (mipi_dsi_hal_phy_write_register)		     */
/* ------------------------------------------------------------------------- */

static void dsi_phy_write_clock(struct esp32p4_dsi *dsi, u32 clock_level,
				bool clear)
{
	dsi->host->phy_tst_ctrl0.val = clock_level << 1 | clear;
}

static void dsi_phy_write_reg_addr(struct esp32p4_dsi *dsi, u8 reg_addr)
{
	/* Bit 16 is the "write" command bit of the test interface. */
	dsi->host->phy_tst_ctrl1.val = BIT(16) | reg_addr;
}

static void dsi_phy_write_reg_val(struct esp32p4_dsi *dsi, u8 reg_val)
{
	dsi->host->phy_tst_ctrl1.val = reg_val;
}

static void dsi_phy_write_reg(struct esp32p4_dsi *dsi, u8 reg, u8 val)
{
	dsi_phy_write_clock(dsi, 0, false);
	dsi_phy_write_reg_addr(dsi, reg);
	dsi_phy_write_clock(dsi, 1, false);
	dsi_phy_write_clock(dsi, 0, false);
	dsi_phy_write_reg_val(dsi, val);
	dsi_phy_write_clock(dsi, 1, false);
	dsi_phy_write_clock(dsi, 0, false);
}

/* ------------------------------------------------------------------------- */
/* Integer PHY PLL search						     */
/* ------------------------------------------------------------------------- */

/*
 * Find M/N such that f_vco = ref * M / N is as close as possible to the
 * requested lane bit rate.  M must be even and 5 MHz <= ref / N <= 40 MHz.
 * This is the float algorithm of mipi_dsi_hal_configure_phy_pll() rewritten
 * with integers only: delta = |rate - ref * M / N| is compared as
 * |rate * N - ref * M| / N, i.e. without floating point.
 */
static int dsi_pll_find_mn(u32 ref_mhz, u32 rate_mbps, u16 *out_m, u8 *out_n)
{
	u32 min_n = max_t(u32, 1, ref_mhz / 40);
	u32 max_n = ref_mhz / 5;
	u32 best_m = 0, best_n = 0, best_delta_milli = U32_MAX;
	u32 n;

	for (n = min_n; n <= max_n; n++) {
		u32 m = rate_mbps * n / ref_mhz;
		u64 diff, delta_milli;

		/*
		 * Only even feedback ratios are representable: the M-1 field
		 * dropped its low bit in hardware.
		 */
		if (m & 1)
			continue;

		diff = abs_diff((u64)rate_mbps * n, (u64)ref_mhz * m);
		delta_milli = div_u64(diff * 1000, n);
		if (delta_milli < best_delta_milli) {
			best_delta_milli = delta_milli;
			best_m = m;
			best_n = n;
			if (delta_milli < 10)
				break;
		}
	}

	if (!best_m || !best_n)
		return -EINVAL;

	*out_m = best_m;
	*out_n = best_n;
	return 0;
}

static u8 dsi_pll_hs_freq_sel(u32 rate_mbps)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(dsi_phy_pll_ranges); i++) {
		if (rate_mbps >= dsi_phy_pll_ranges[i].start_mbps &&
		    rate_mbps <= dsi_phy_pll_ranges[i].end_mbps)
			return dsi_phy_pll_ranges[i].hs_freq_sel;
	}

	return 0;
}

static int dsi_configure_phy_pll(struct esp32p4_dsi *dsi, u32 ref_mhz,
				 u32 rate_mbps)
{
	u16 pll_m;
	u8 pll_n, hs_freq_sel;
	u32 real;
	int ret;

	ret = dsi_pll_find_mn(ref_mhz, rate_mbps, &pll_m, &pll_n);
	if (ret)
		return ret;

	hs_freq_sel = dsi_pll_hs_freq_sel(rate_mbps);

	dsi_phy_write_reg(dsi, 0x44, hs_freq_sel << 1);
	/* Use the N/M factors configured in registers 0x17 and 0x18. */
	dsi_phy_write_reg(dsi, 0x19, 0x30);
	dsi_phy_write_reg(dsi, 0x17, pll_n - 1);
	dsi_phy_write_reg(dsi, 0x18, (pll_m - 1) & 0x1f);
	dsi_phy_write_reg(dsi, 0x18, 0x80 | (((pll_m - 1) >> 5) & 0x0f));

	real = dsi_round_div((u64)ref_mhz * pll_m, pll_n);
	dev_info(dsi->dev,
		 "phy pll: ref=%u MHz M=%u N=%u hs_freq_sel=0x%02x rate=%u Mbps (ref*M/N=%u)\n",
		 ref_mhz, pll_m, pll_n, hs_freq_sel, rate_mbps, real);

	return 0;
}

/* ------------------------------------------------------------------------- */
/* LDO (esp_ldo channel 3, unit 2, 2500 mV)				     */
/* ------------------------------------------------------------------------- */

static const u8 dsi_ldo_unit_to_pmu_index[4] = { 0, 3, 1, 4 };

static void dsi_ldo_voltage_to_dref_mul(struct esp32p4_dsi *dsi, int unit,
					int voltage_mv, u8 *dref, u8 *mul,
					bool *use_rail_voltage)
{
	/*
	 * TODO(verify): calibration is only applied when eFuse block version is
	 * >= 1; confirm the eFuse block version and values of this board.
	 */
	u32 sys2 = readl(dsi->efuse + EFUSE_RD_MAC_SYS2);
	u32 sys3 = readl(dsi->efuse + EFUSE_RD_MAC_SYS3);
	u32 sys4 = readl(dsi->efuse + EFUSE_RD_MAC_SYS4);
	u32 blk_version = FIELD_GET(EFUSE_BLK_VERSION_MAJOR, sys2);
	int k_1000 = 1000, vos_1000 = 0, c_1000 = 1000;
	u8 efuse_k = 0, efuse_vos = 0, efuse_c = 0;
	int best_diff = 400000000;
	u8 best_dref = 0, best_mul = 0;
	u8 dref_val, mul_val;

	if (blk_version >= 1) {
		if (unit == 2) {
			efuse_k = FIELD_GET(GENMASK(13, 6), sys3);
			efuse_vos = FIELD_GET(GENMASK(19, 14), sys3);
			efuse_c = FIELD_GET(GENMASK(25, 20), sys3);
		}
		if (unit == 3) {
			efuse_k = (FIELD_GET(GENMASK(1, 0), sys4) << 6) |
				  FIELD_GET(GENMASK(31, 26), sys3);
			efuse_vos = FIELD_GET(GENMASK(7, 2), sys4);
			efuse_c = FIELD_GET(GENMASK(13, 8), sys4);
		}
		if (efuse_k)
			k_1000 = efuse_k & 0x80 ? -1 * (efuse_k & 0x7f) + 975 :
						  efuse_k + 975;
		if (efuse_vos)
			vos_1000 = efuse_vos & 0x20 ?
				   -1 * (efuse_vos & 0x1f) - 3 :
				   efuse_vos - 3;
		if (efuse_c)
			c_1000 = efuse_c & 0x20 ?
				 -1 * (efuse_c & 0x1f) + 990 :
				 efuse_c + 990;
	}

	for (dref_val = 0; dref_val < 16; dref_val++) {
		int vref_20 = dref_val < 9 ? 10 + dref_val :
					     20 + (dref_val - 9) * 2;

		for (mul_val = 0; mul_val < 8; mul_val++) {
			int vout = (vref_20 * k_1000 + 20 * vos_1000) *
				   (4000 + mul_val * c_1000);
			int diff = voltage_mv * 80000 - vout;

			diff = abs(diff);
			if (diff < best_diff) {
				best_diff = diff;
				best_dref = dref_val;
				best_mul = mul_val;
			}
		}
	}

	/* eFuse-calibrated 1.8 V settings for VO1/VO2, as in ldo_ll.h. */
	if (blk_version >= 1 && voltage_mv == 1800) {
		u8 cal_dref = FIELD_GET(GENMASK(27, 24), sys2);
		u8 cal_mul = FIELD_GET(GENMASK(2, 0), sys3);

		if (unit == 0 && cal_dref && cal_mul) {
			best_dref = cal_dref;
			best_mul = cal_mul;
		}
		cal_dref = FIELD_GET(GENMASK(31, 28), sys2);
		cal_mul = FIELD_GET(GENMASK(5, 3), sys3);
		if (unit == 1 && cal_dref && cal_mul) {
			best_dref = cal_dref;
			best_mul = cal_mul;
		}
	}

	*dref = best_dref;
	*mul = best_mul;
	*use_rail_voltage = voltage_mv == 3300;
}

static int dsi_ldo_enable(struct esp32p4_dsi *dsi, int voltage_mv)
{
	/* esp_ldo channel 3 maps to LDO unit 2, PMU register index 1. */
	int unit = 2;
	void __iomem *base;
	u8 dref, mul;
	bool rail;

	base = dsi->pmu + PMU_EXT_LDO_OFFSET +
	       dsi_ldo_unit_to_pmu_index[unit] * PMU_EXT_LDO_STRIDE;

	dsi_ldo_voltage_to_dref_mul(dsi, unit, voltage_mv, &dref, &mul, &rail);

	/* Limit inrush current while the rail comes up. */
	dsi_update_bits(base + 4, PMU_LDO_ANA_CUR_LIM, PMU_LDO_ANA_CUR_LIM);
	dsi_update_bits(base + 4, PMU_LDO_ANA_DREF_MASK,
			dsi_field_prep(dref, PMU_LDO_ANA_DREF_MASK));
	dsi_update_bits(base + 4, PMU_LDO_ANA_MUL_MASK,
			dsi_field_prep(mul, PMU_LDO_ANA_MUL_MASK));
	dsi_update_bits(base, PMU_LDO_TIEH, rail ? PMU_LDO_TIEH : 0);
	/* Software owned, tieh_sel = 0 (Vref * mul). */
	dsi_update_bits(base, PMU_LDO_FORCE_TIEH_SEL | PMU_LDO_TIEH_SEL_MASK,
			PMU_LDO_FORCE_TIEH_SEL);
	dsi_update_bits(base + 4, PMU_LDO_ANA_VDET, PMU_LDO_ANA_VDET);
	dsi_update_bits(base, PMU_LDO_XPD, PMU_LDO_XPD);
	dsi_update_bits(base + 4, PMU_LDO_ANA_CUR_LIM, 0);

	dsi->ldo_on = true;
	dev_info(dsi->dev, "phy ldo: 2500 mV (dref=%u mul=%u)\n", dref, mul);
	return 0;
}

static void dsi_ldo_disable(struct esp32p4_dsi *dsi)
{
	int unit = 2;
	void __iomem *base;

	if (!dsi->ldo_on)
		return;

	base = dsi->pmu + PMU_EXT_LDO_OFFSET +
	       dsi_ldo_unit_to_pmu_index[unit] * PMU_EXT_LDO_STRIDE;
	dsi_update_bits(base, PMU_LDO_XPD, 0);
	dsi->ldo_on = false;
}

/* ------------------------------------------------------------------------- */
/* Clock and reset control						     */
/* ------------------------------------------------------------------------- */

static void dsi_reset_pulse(void __iomem *reg, u32 bit)
{
	dsi_update_bits(reg, bit, bit);
	dsi_update_bits(reg, bit, 0);
}

static void dsi_enable_bus_clocks(struct esp32p4_dsi *dsi)
{
	void __iomem *rst = dsi->clkrst + HP_SYS_HP_RST_EN0;

	/* APB clock for the DSI host/bridge and both GDMA clocks. */
	dsi_update_bits(dsi->clkrst + HP_SYS_SOC_CLK_CTRL1,
			HP_SYS_DSI_SYS_CLK_EN | HP_SYS_GDMA_SYS_CLK_EN,
			HP_SYS_DSI_SYS_CLK_EN | HP_SYS_GDMA_SYS_CLK_EN);
	dsi_update_bits(dsi->clkrst + HP_SYS_SOC_CLK_CTRL0,
			HP_SYS_GDMA_CPU_CLK_EN, HP_SYS_GDMA_CPU_CLK_EN);

	dsi_reset_pulse(rst, HP_SYS_DSI_BRG_RST_EN);
	dsi_reset_pulse(rst, HP_SYS_GDMA_RST_EN);
}

static void dsi_enable_phy_clocks(struct esp32p4_dsi *dsi)
{
	/*
	 * PHY configuration clock and PHY PLL reference: PLL_F20M (clk_src_sel
	 * = 0), divider 1.  That is what esp_lcd_new_dsi_bus() selects on this
	 * silicon, so the PLL factors are computed against 20 MHz.
	 * TODO(verify): pre-v3.0 silicon uses PERI_CLK_CTRL02 bit 31:30 for the
	 * PLL reference source and a different divider layout.
	 */
	/* PHY PLL reference source: PLL_F20M (clk_src_sel == 0). */
	dsi_update_bits(dsi->clkrst + HP_SYS_PERI_CLK_CTRL02, GENMASK(31, 30), 0);
	/* PHY configuration clock (bit 0) and PLL reference clock (bit 1). */
	dsi_update_bits(dsi->clkrst + HP_SYS_PERI_CLK_CTRL03,
			BIT(0) | BIT(1), BIT(0) | BIT(1));
}

static void dsi_enable_dpi_clock(struct esp32p4_dsi *dsi)
{
	dsi_update_bits(dsi->clkrst + HP_SYS_PERI_CLK_CTRL03,
			GENMASK(15, 5),
			dsi_field_prep(DSI_DPI_CLK_SRC_SEL, GENMASK(6, 5)) |
			BIT(7) |
			dsi_field_prep(dsi->dpi_div - 1, GENMASK(15, 8)));
}

static void dsi_disable_dpi_clock(struct esp32p4_dsi *dsi)
{
	dsi_update_bits(dsi->clkrst + HP_SYS_PERI_CLK_CTRL03, BIT(7), 0);
}

/* ------------------------------------------------------------------------- */
/* MIPI DSI host initialisation					     */
/* ------------------------------------------------------------------------- */

static void dsi_host_phy_config(struct esp32p4_dsi *dsi)
{
	dsi_host_dev_t *host = dsi->host;

	/*
	 * TRM ch.44 MIPI DSI - DSI_HOST_PHY_IF_CFG.
	 * Program N_LANES and PHY_STOP_WAIT_TIME with one 32-bit access.
	 * A byte write to PHY_STOP_WAIT_TIME clears N_LANES on rev1.3.
	 */
	host->pwr_up.val = 1;
	writel((0x3f << 8) | (DSI_NUM_DATA_LANES - 1), &host->phy_if_cfg.val);
	host->phy_rstz.phy_shutdownz = 1;
	host->phy_rstz.phy_rstz = 0;
	host->phy_rstz.phy_rstz = 1;
	host->phy_rstz.phy_enableclk = 1;
	host->phy_rstz.phy_forcepll = 1;
	/* Reset the DSI bridge as mipi_dsi_brg_ll_reset() does. */
	dsi->bridge->en.dsi_brig_rst = 1;
	dsi->bridge->en.dsi_brig_rst = 0;
}

static int dsi_wait_pll_locked(struct esp32p4_dsi *dsi)
{
	int i;

	for (i = 0; i < 100; i++) {
		if (dsi->host->phy_status.phy_lock)
			return 0;
		msleep(1);
	}

	return -ETIMEDOUT;
}

static int dsi_wait_lanes_stopped(struct esp32p4_dsi *dsi)
{
	/* Clock lane plus both data lanes (mipi_dsi_phy_ll_are_lanes_stopped). */
	u32 mask = BIT(2) | BIT(4) | BIT(7);
	int i;

	for (i = 0; i < 100; i++) {
		if ((dsi->host->phy_status.val & mask) == mask)
			return 0;
		msleep(1);
	}

	return -ETIMEDOUT;
}

static void dsi_host_dbi_modes(struct esp32p4_dsi *dsi)
{
	dsi_host_dev_t *host = dsi->host;

	/* No tear-effect support, command acknowledge on. */
	host->cmd_mode_cfg.tear_fx_en = 0;
	host->cmd_mode_cfg.ack_rqst_en = 1;
	/* All generic and DCS transactions use low-power mode. */
	host->cmd_mode_cfg.gen_sw_0p_tx = DSI_LL_TRANS_SPEED_LP;
	host->cmd_mode_cfg.gen_sw_1p_tx = DSI_LL_TRANS_SPEED_LP;
	host->cmd_mode_cfg.gen_sw_2p_tx = DSI_LL_TRANS_SPEED_LP;
	host->cmd_mode_cfg.gen_lw_tx = DSI_LL_TRANS_SPEED_LP;
	host->cmd_mode_cfg.gen_sr_0p_tx = DSI_LL_TRANS_SPEED_LP;
	host->cmd_mode_cfg.gen_sr_1p_tx = DSI_LL_TRANS_SPEED_LP;
	host->cmd_mode_cfg.gen_sr_2p_tx = DSI_LL_TRANS_SPEED_LP;
	host->cmd_mode_cfg.dcs_sw_0p_tx = DSI_LL_TRANS_SPEED_LP;
	host->cmd_mode_cfg.dcs_sw_1p_tx = DSI_LL_TRANS_SPEED_LP;
	host->cmd_mode_cfg.dcs_lw_tx = DSI_LL_TRANS_SPEED_LP;
	host->cmd_mode_cfg.dcs_sr_0p_tx = DSI_LL_TRANS_SPEED_LP;
	host->cmd_mode_cfg.max_rd_pkt_size = DSI_LL_TRANS_SPEED_LP;
}

static int dsi_host_init(struct esp32p4_dsi *dsi)
{
	dsi_host_dev_t *host = dsi->host;
	u32 to_div, esc_div;
	int ret;

	dsi_host_phy_config(dsi);

	ret = dsi_configure_phy_pll(dsi, DSI_PHY_REF_MHZ,
				    DSI_LANE_BIT_RATE_MBPS);
	if (ret)
		return ret;

	ret = dsi_wait_pll_locked(dsi);
	if (ret) {
		dev_err(dsi->dev, "PHY PLL did not lock\n");
		return ret;
	}
	ret = dsi_wait_lanes_stopped(dsi);
	if (ret) {
		dev_err(dsi->dev, "DSI lanes did not reach stop state\n");
		return ret;
	}

	/* Command mode until dpi_start(). */
	host->mode_cfg.cmd_video_mode = 1;
	/* Match mipi_dsi_host_ll_set_clock_lane_state(...AUTO) in IDF. */
	host->lpclk_ctrl.auto_clklane_ctrl = 1;
	host->lpclk_ctrl.phy_txrequestclkhs = 1;
	/* mipi_dsi_phy_ll_set_switch_time(50, 104, 46, 128). */
	host->phy_tmr_cfg.phy_hs2lp_time = 50;
	host->phy_tmr_cfg.phy_lp2hs_time = 104;
	host->phy_tmr_lpclk_cfg.phy_clkhs2lp_time = 46;
	host->phy_tmr_lpclk_cfg.phy_clklp2hs_time = 128;

	/* CRC/ECC reception, EoTp transmission in HS mode. */
	host->pckhdl_cfg.crc_rx_en = 1;
	host->pckhdl_cfg.ecc_rx_en = 1;
	host->pckhdl_cfg.eotp_tx_en = 1;
	host->pckhdl_cfg.eotp_tx_lp_en = 0;

	to_div = dsi_round_div((u64)DSI_LANE_BIT_RATE_MBPS, 8 *
			       DSI_DPI_TIMEOUT_CLK_MHZ);
	esc_div = dsi_round_div((u64)DSI_LANE_BIT_RATE_MBPS, 8 *
				DSI_DPI_ESCAPE_CLK_MHZ);
	/* TRM ch.44 MIPI DSI - DSI_HOST_CLKMGR_CFG, likewise word access. */
	writel((to_div << 8) | esc_div, &host->clkmgr_cfg.val);

	/* Disable all timeouts, as ESP-IDF does. */
	host->to_cnt_cfg.hstx_to_cnt = 0;
	host->to_cnt_cfg.lprx_to_cnt = 0;
	host->hs_rd_timeout_cnt.hs_rd_to_cnt = 0;
	host->lp_rd_timeout_cnt.lp_rd_to_cnt = 0;
	host->hs_wr_timeout_cnt.hs_wr_to_cnt = 0;
	host->lp_wr_timeout_cnt.lp_wr_to_cnt = 0;
	host->bta_timeout_cnt.bta_to_cnt = 0;

	host->phy_tmr_rd_cfg.max_rd_time = 6000;

	dsi_host_dbi_modes(dsi);

	dev_info(dsi->dev, "DSI host up: %u lanes @ %u Mbps\n",
		 DSI_NUM_DATA_LANES, DSI_LANE_BIT_RATE_MBPS);
	return 0;
}

/* ------------------------------------------------------------------------- */
/* DCS command helpers (esp_lcd_panel_io_dbi.c / mipi_dsi_hal.c)	     */
/* ------------------------------------------------------------------------- */

static int dsi_wait_cmd_fifo(struct esp32p4_dsi *dsi)
{
	u32 i;

	for (i = 0; i < DSI_FIFO_TIMEOUT_US; i++) {
		if (!dsi->host->cmd_pkt_status.gen_cmd_full)
			return 0;
		udelay(1);
	}

	return -ETIMEDOUT;
}

static int dsi_wait_write_fifo(struct esp32p4_dsi *dsi)
{
	u32 i;

	for (i = 0; i < DSI_FIFO_TIMEOUT_US; i++) {
		if (!dsi->host->cmd_pkt_status.gen_pld_w_full)
			return 0;
		udelay(1);
	}

	return -ETIMEDOUT;
}

static void dsi_gen_set_header(struct esp32p4_dsi *dsi, u8 vc, u8 dt,
			       u8 msb, u8 lsb)
{
	dsi->host->gen_hdr.val = (msb << 16) | (lsb << 8) | (vc << 6) | dt;
}

static int dsi_gen_write_payload(struct esp32p4_dsi *dsi, u32 word)
{
	int ret = dsi_wait_write_fifo(dsi);

	if (ret)
		return ret;
	dsi->host->gen_pld_data.val = word;
	return 0;
}

static int dsi_gen_short_packet(struct esp32p4_dsi *dsi, u8 dt, u16 data)
{
	int ret = dsi_wait_cmd_fifo(dsi);

	if (ret)
		return ret;
	dsi_gen_set_header(dsi, 0, dt, data >> 8, data & 0xff);
	return 0;
}

/*
 * Generic DCS write, exactly the payload/DT selection of
 * mipi_dsi_hal_host_gen_write_dcs_command() with lcd_cmd_bits = 8.
 */
static int dsi_dcs_write(struct esp32p4_dsi *dsi, u8 cmd, const u8 *param,
			 size_t param_size)
{
	size_t merged = min_t(size_t, 4 - 1, param_size);
	size_t payload_size = 1 + param_size;
	u32 word = cmd;
	size_t remain, i;
	u8 dt, msb, lsb;
	int ret;

	for (i = 0; i < merged; i++)
		word |= (u32)param[i] << (8 * (i + 1));

	if (payload_size > 2) {
		ret = dsi_gen_write_payload(dsi, word);
		if (ret)
			return ret;
		param += merged;
		remain = param_size - merged;
		while (remain >= 4) {
			u32 tmp;

			memcpy(&tmp, param, sizeof(tmp));
			ret = dsi_gen_write_payload(dsi, tmp);
			if (ret)
				return ret;
			param += 4;
			remain -= 4;
		}
		if (remain) {
			u32 tmp = 0;

			memcpy(&tmp, param, remain);
			ret = dsi_gen_write_payload(dsi, tmp);
			if (ret)
				return ret;
		}
		dt = DSI_DCS_LONG_WRITE;
		msb = (payload_size >> 8) & 0xff;
		lsb = payload_size & 0xff;
	} else if (payload_size == 2) {
		dt = DSI_DCS_SHORT_WRITE_1;
		msb = (word >> 8) & 0xff;
		lsb = word & 0xff;
	} else {
		dt = DSI_DCS_SHORT_WRITE_0;
		msb = (word >> 8) & 0xff;
		lsb = word & 0xff;
	}

	ret = dsi_wait_cmd_fifo(dsi);
	if (ret)
		return ret;
	dsi_gen_set_header(dsi, 0, dt, msb, lsb);
	return 0;
}

/*
 * DCS read using the host generic interface.  IDF waits forever on a missing
 * panel; keep the same protocol but bound every wait.
 */
static int dsi_dcs_read(struct esp32p4_dsi *dsi, u8 cmd, u8 *buf, u16 len)
{
	u32 data, i, counter = 0;
	int ret;

	ret = dsi_gen_short_packet(dsi, DSI_SET_MAXIMUM_RETURN_PKT, len);
	if (ret)
		return ret;

	/* Make sure command mode is on and BTA responses are enabled. */
	dsi->host->mode_cfg.cmd_video_mode = 1;
	dsi->host->pckhdl_cfg.bta_en = 1;
	dsi->host->gen_vcid.gen_vcid_rx = 0;

	ret = dsi_gen_short_packet(dsi, DSI_DCS_READ_0, cmd);
	if (ret)
		return ret;

	for (i = 0; i < DSI_FIFO_TIMEOUT_US; i++) {
		if (!dsi->host->cmd_pkt_status.gen_rd_cmd_busy &&
		    !dsi->host->cmd_pkt_status.gen_pld_r_empty)
			break;
		udelay(1);
	}
	if (i == DSI_FIFO_TIMEOUT_US)
		return -ETIMEDOUT;

	while (!dsi->host->cmd_pkt_status.gen_pld_r_empty) {
		data = dsi->host->gen_pld_data.val;
		for (i = 0; i < 4; i++) {
			if (counter < len) {
				buf[counter] =
					(data >> (8 * i)) & 0xff;
				counter++;
			}
		}
		if (counter >= len)
			break;
		if (++i > 64)
			return -EIO;
	}

	if (counter < len)
		return -EIO;

	/* Drain any extra words so the next command starts clean. */
	for (i = 0; i < 64; i++) {
		if (dsi->host->cmd_pkt_status.gen_pld_r_empty)
			break;
		(void)dsi->host->gen_pld_data.val;
	}

	return 0;
}

/* ------------------------------------------------------------------------- */
/* Panel MCU (I2C 0x45) and panel init table				     */
/* ------------------------------------------------------------------------- */

static int dsi_mcu_write(struct esp32p4_dsi *dsi, u8 reg, u8 val)
{
	u8 buf[2] = { reg, val };
	struct i2c_msg msg = {
		.addr = dsi->mcu->addr,
		.flags = dsi->mcu->flags,
		.len = sizeof(buf),
		.buf = buf,
	};
	int ret = i2c_transfer(dsi->mcu->adapter, &msg, 1);

	return ret == 1 ? 0 : (ret < 0 ? ret : -EIO);
}

static int dsi_mcu_read(struct esp32p4_dsi *dsi, u8 reg, u8 *val)
{
	u8 buf;
	struct i2c_msg msgs[2] = {
		{
			.addr = dsi->mcu->addr,
			.flags = dsi->mcu->flags,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = dsi->mcu->addr,
			.flags = dsi->mcu->flags | I2C_M_RD,
			.len = 1,
			.buf = &buf,
		},
	};
	int ret = i2c_transfer(dsi->mcu->adapter, msgs, 2);

	if (ret != 2)
		return ret < 0 ? ret : -EIO;
	*val = buf;
	return 0;
}

static int dsi_panel_page(struct esp32p4_dsi *dsi, u8 page)
{
	const u8 params[3] = { 0x98, 0x81, page };

	return dsi_dcs_write(dsi, 0xff, params, sizeof(params));
}

static int dsi_panel_mcu_reset(struct esp32p4_dsi *dsi)
{
	u8 id = 0;
	int ret, retries;

	for (retries = 0; retries < 3; retries++) {
		ret = dsi_mcu_read(dsi, 1, &id);
		if (!ret) {
			dev_info(dsi->dev, "MCU answered on attempt %d\n", retries + 1);
			break;
		}
		msleep(100);
	}
	if (ret)
		return ret;

	dev_info(dsi->dev, "touch display MCU id=0x%02x\n", id);
	/* Reported for the log only: the DCS identity read is authoritative. */

	ret = dsi_mcu_write(dsi, 3, 0);	/* dark while resetting */
	if (ret)
		return ret;
	ret = dsi_mcu_write(dsi, 2, 0);	/* LCD and CTP held in reset */
	if (ret)
		return ret;
	msleep(20);
	ret = dsi_mcu_write(dsi, 2, 3);	/* release */
	if (ret)
		return ret;
	/* The panel needs ~120 ms after reset release before it decodes DCS. */
	msleep(120);
	return 0;
}

static int dsi_panel_dcs_init(struct esp32p4_dsi *dsi)
{
	const u8 lane_mode = 0x03, madctl = 0x00, rgb565 = 0x55;
	u8 identity[3], inversion, source_timing;
	size_t i;
	int ret, try;

	/* Retry the identity read: the panel may still be waking up. */
	for (try = 0; try < 10; try++) {
		msleep(20);
		ret = dsi_panel_page(dsi, 1);
		if (!ret) {
			for (i = 0; i < ARRAY_SIZE(identity); i++) {
				ret = dsi_dcs_read(dsi, i, &identity[i], 1);
				if (ret)
					break;
			}
		}
		if (!ret)
			break;
	}
	if (ret)
		return ret;

	dev_info(dsi->dev, "panel id=%02x/%02x/%02x\n", identity[0],
		 identity[1], identity[2]);
	if (identity[0] != 0x98 || identity[1] != 0x81 ||
	    identity[2] != 0x1c)
		return -EIO;

	/*
	 * The panel is still on command page 1. Match the verified IDF
	 * esp_lcd_ili9881c panel_ili9881c_init() sequence and program the
	 * panel's two-lane pad mode before waking it.
	 */
	ret = dsi_dcs_write(dsi, 0xb7, &lane_mode, 1);
	if (ret)
		return ret;
	ret = dsi_panel_page(dsi, 0);
	if (ret)
		return ret;
	/*
	 * ILI9881C datasheet section 5.3.11: Sleep Out reloads the
	 * supplier defaults. Program the vendor table after the reload.
	 */
	ret = dsi_dcs_write(dsi, 0x11, NULL, 0);
	if (ret)
		return ret;
	msleep(120);
	ret = dsi_dcs_write(dsi, 0x36, &madctl, 1);
	if (ret)
		return ret;
	ret = dsi_dcs_write(dsi, 0x3a, &rgb565, 1);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(rpi_5inch_init); i++) {
		u8 cmd = rpi_5inch_init[i][0];
		u8 val = rpi_5inch_init[i][1];

		if (cmd == 0xff)
			ret = dsi_panel_page(dsi, val);
		else
			ret = dsi_dcs_write(dsi, cmd, &val, 1);
		if (ret)
			return ret;
	}

	/* The vendor table ends on page 1. Verify the optical timing profile. */
	ret = dsi_dcs_read(dsi, 0x31, &inversion, sizeof(inversion));
	if (ret)
		return ret;
	ret = dsi_dcs_read(dsi, 0x60, &source_timing, sizeof(source_timing));
	if (ret)
		return ret;
	if (inversion != 0x02 || source_timing != 0x20) {
		dev_err(dsi->dev, "panel settings did not latch (inversion %#x, source timing %#x)\n",
			inversion, source_timing);
		return -EIO;
	}
	dev_info(dsi->dev, "panel readback: inversion %#x, source timing %#x\n",
		 inversion, source_timing);

	ret = dsi_panel_page(dsi, 0);
	if (ret)
		return ret;
	/* Panel supplier sequence: let the vendor registers settle. */
	msleep(200);
	return 0;
}

/* ------------------------------------------------------------------------- */
/* DPI timing and bridge configuration				     */
/* ------------------------------------------------------------------------- */

static void dsi_set_horizontal_timing(struct esp32p4_dsi *dsi, u32 hsw,
				      u32 hbp, u32 act, u32 hfp)
{
	u32 dpi_bits_mhz = DSI_DPI_CLK_MHZ * 8;
	u32 dpi_line_unit = dpi_bits_mhz /
			   gcd(dpi_bits_mhz, DSI_LANE_BIT_RATE_MBPS);
	u32 htotal = roundup(hsw + hbp + act + hfp, dpi_line_unit);
	u32 host_hsw = dsi_round_div((u64)hsw * DSI_LANE_BIT_RATE_MBPS,
				     dpi_bits_mhz);
	u32 host_hbp = dsi_round_div((u64)hbp * DSI_LANE_BIT_RATE_MBPS,
				     dpi_bits_mhz);

	/*
	 * TRM ch.44 MIPI DSI - DSI_BRG_HTOTAL and DSI_HOST_VID_HLINE_TIME.
	 * Use whole clock cycles in both domains with identical line periods.
	 * Round the front porch up to a whole clock ratio so no phase
	 * error accumulates between the bridge and host.
	 */
	dsi->host->vid_hsa_time.vid_hsa_time = host_hsw;
	dsi->host->vid_hbp_time.vid_hbp_time = host_hbp;
	dsi->host->vid_hline_time.vid_hline_time =
		div_u64((u64)htotal * DSI_LANE_BIT_RATE_MBPS, dpi_bits_mhz);

	dsi->bridge->dpi_h_cfg0.hdisp = act;
	dsi->bridge->dpi_h_cfg0.htotal = htotal;
	dsi->bridge->dpi_h_cfg1.hsync = hsw;
	dsi->bridge->dpi_h_cfg1.hbank = hbp;
}

static void dsi_set_vertical_timing(struct esp32p4_dsi *dsi, u32 vsa,
				    u32 vbp, u32 act, u32 vfp)
{
	dsi->host->vid_vsa_lines.vsa_lines = vsa;
	dsi->host->vid_vbp_lines.vbp_lines = vbp;
	dsi->host->vid_vactive_lines.v_active_lines = act;
	dsi->host->vid_vfp_lines.vfp_lines = vfp;

	dsi->bridge->dpi_v_cfg0.vdisp = act;
	dsi->bridge->dpi_v_cfg0.vtotal = vsa + vbp + act + vfp;
	dsi->bridge->dpi_v_cfg1.vsync = vsa;
	dsi->bridge->dpi_v_cfg1.vbank = vbp;
}

static int dsi_dpi_init(struct esp32p4_dsi *dsi)
{
	u32 pixel_bits = dsi->width * dsi->height * 16;
	dsi_host_dev_t *host = dsi->host;
	dsi_brg_dev_t *brg = dsi->bridge;

	dsi->dpi_src_mhz = DSI_DPI_CLK_SRC_MHZ;
	dsi->dpi_div = DSI_DPI_CLK_DIV;
	dsi->dpi_real_uhz = div_u64((u64)dsi->dpi_src_mhz * 1000000,
				    dsi->dpi_div);
	dsi_enable_dpi_clock(dsi);

	/* mipi_dsi_host_ll_dpi_* configuration. */
	host->dpi_vcid.dpi_vcid = 0;
	host->dpi_color_coding.dpi_color_coding = DSI_COLOR_CODE_16BIT_CONFIG1;
	host->dpi_cfg_pol.hsync_active_low = 0;
	host->dpi_cfg_pol.vsync_active_low = 0;
	host->dpi_cfg_pol.dataen_active_low = 0;
	host->dpi_cfg_pol.shutd_active_low = 0;
	host->dpi_cfg_pol.colorm_active_low = 0;

	host->vid_mode_cfg.lp_hbp_en = 1;
	host->vid_mode_cfg.lp_hfp_en = 1;
	host->vid_mode_cfg.lp_vsa_en = 1;
	host->vid_mode_cfg.lp_vbp_en = 1;
	host->vid_mode_cfg.lp_vfp_en = 1;
	host->vid_mode_cfg.lp_vact_en = 1;
	host->vid_mode_cfg.lp_cmd_en = 1;
	/* The IDF reference uses frame BTA with the clock lane in AUTO. */
	host->vid_mode_cfg.frame_bta_ack_en = 1;
	host->vid_mode_cfg.vid_mode_type = DSI_LL_VIDEO_BURST_SYNC_PULSES;
	host->vid_pkt_size.vid_pkt_size = dsi->width;
	host->vid_num_chunks.vid_num_chunks = 0;
	host->vid_null_size.vid_null_size = 0;

	dsi_set_horizontal_timing(dsi, DSI_HSA, DSI_HBP, dsi->width, DSI_HFP);
	dsi_set_vertical_timing(dsi, DSI_VSA, DSI_VBP, dsi->height, DSI_VFP);

	/* Bridge. */
	brg->raw_num_cfg.raw_num_total = (pixel_bits + 63) / 64;
	brg->raw_num_cfg.unalign_64bit_en = pixel_bits % 64 ? 1 : 0;
	brg->raw_num_cfg.raw_num_total_set = 1;
	/*
	 * TRM ch.44 MIPI DSI - FIFO_UNDERRUN_DISCARD_VCNT suppresses
	 * underrun events before the selected scanline. Detect them from
	 * the first line, including when starting the next VDMA block.
	 */
	brg->dpi_misc_config.fifo_underrun_discard_vcnt = 0;
	/*
	 * Input and output are both RGB565 (chip revision >= 3.0 fields).
	 * TODO(verify): on pre-3.0 silicon pixel_type.raw_type is also used for
	 * the output format; the v3.0 dpi_type field is written here.
	 */
	brg->pixel_type.raw_type = 2;
	brg->pixel_type.data_in_type = 0;
	brg->pixel_type.dpi_type = 2;
	brg->pixel_type.dpi_config = 0;
	brg->dma_flow_ctrl.dsi_dma_flow_controller =
		DSI_LL_FLOW_CONTROLLER_DMA;
	brg->dma_flow_ctrl.dma_flow_multiblk_num = 1;
	brg->dma_frame_interval.dma_multiblk_en = 0;
	brg->dma_req_cfg.dma_burst_len = DSI_BRIDGE_BURST_WORDS;
	brg->raw_buf_almost_empty_thrd.dsi_raw_buf_almost_empty_thrd =
		DSI_BRIDGE_PREFILL_WORDS;
	brg->en.dsi_en = 1;
	brg->dpi_misc_config.dpi_en = 0;
	brg->int_ena.val = 0;
	brg->int_clr.val = UINT_MAX;
	brg->dpi_config_update.dpi_config_update = 1;

	dev_info(dsi->dev, "DPI on, %ux%u RGB565, pixel_bits=%u, dpi=%u MHz\n",
		 dsi->width, dsi->height, pixel_bits, DSI_DPI_CLK_MHZ);
	dev_info(dsi->dev, "dpi clk: src=%u MHz div=%u real=%u Hz; htotal=%u vtotal=%u\n",
		 dsi->dpi_src_mhz, dsi->dpi_div, dsi->dpi_real_uhz,
		 dsi->bridge->dpi_h_cfg0.htotal,
		 dsi->bridge->dpi_v_cfg0.vtotal);
	return 0;
}

/* ------------------------------------------------------------------------- */
/* VDMA channel and link list						     */
/* ------------------------------------------------------------------------- */

static int dsi_dma_reset(struct esp32p4_dsi *dsi)
{
	u32 i;

	dsi->gdma->reset0.dmac_rst = 1;
	for (i = 0; i < 1000; i++) {
		if (!dsi->gdma->reset0.dmac_rst)
			break;
		udelay(1);
	}
	if (i == 1000)
		return -ETIMEDOUT;

	dsi->gdma->cfg0.dmac_en = 1;
	dsi->gdma->cfg0.int_en = 1;
	return 0;
}

static void dsi_dma_channel_config(struct esp32p4_dsi *dsi)
{
	dmac_channel_reg_t *ch =
		(dmac_channel_reg_t *)&dsi->gdma->ch[DSI_GDMA_CHANNEL];

	/* Memory -> peripheral, DMA as flow controller. */
	ch->cfg1.tt_fc = 1;
	if (IS_ENABLED(CONFIG_ESP32P4_NOMMU_XIP)) {
		ch->cfg0.src_multblk_type = DSI_DMA_MULTBLK_RELOAD;
		ch->cfg0.dst_multblk_type = DSI_DMA_MULTBLK_RELOAD;
	} else {
		ch->cfg0.src_multblk_type = DSI_DMA_MULTBLK_LINKED;
		ch->cfg0.dst_multblk_type = DSI_DMA_MULTBLK_LINKED;
	}
	ch->cfg1.hs_sel_src = 0;	/* hardware handshake */
	ch->cfg1.hs_sel_dst = 0;
	ch->cfg1.src_per = DSI_GDMA_HW_HANDSHAKE_DSI;
	ch->cfg1.dst_per = DSI_GDMA_HW_HANDSHAKE_DSI;
	/* TRM ch.7 VDMA - DMAC_CHn_CFG1.CH_PRIOR. Scanout has a fixed
	 * deadline and shares the VDMA read/write arbiters with CSI.
	 */
	ch->cfg1.ch_prior = 3;
	ch->cfg1.src_osr_lmt = 5 - 1;
	ch->cfg1.dst_osr_lmt = 2 - 1;
	ch->sstatar0.sstatar0 = 0;
	ch->dstatar0.dstatar0 = 0;

	/* No interrupt until dpi_start(). */
	ch->int_st_ena0.val = 0;
	ch->int_sig_ena0.val = 0;
	ch->int_clr0.val = UINT_MAX;
}

/* The descriptor is written through the SRAM non-cache alias. */
static struct esp32p4_dsi_lli *dsi_lli_view(struct esp32p4_dsi *dsi)
{
	if (!dsi->lli_in_sram)
		return dsi->lli;

	return (struct esp32p4_dsi_lli *)((uintptr_t)dsi->lli +
					  DSI_SRAM_NC_OFFSET);
}

static void dsi_dma_program(struct esp32p4_dsi *dsi)
{
	dmac_chn_ctl0_reg_t ctl0 = {
		.sms = DSI_GDMA_MASTER_PORT_MEMORY,
		.dms = DSI_GDMA_MASTER_PORT_DSI,
		.dinc = 1,		/* Fixed DSI FIFO address */
		.src_tr_width = 3,	/* 64 bits */
		.dst_tr_width = 3,
		.src_msize = 8,		/* 512 items */
		.dst_msize = 7,		/* 256 items */
	};
	/* TRM ch.7 VDMA - DMAC_CHn_CTL1.ARLEN and AWLEN. */
	dmac_chn_ctl1_reg_t ctl1 = {
		.arlen_en = 1,
		.arlen = 16 - 1,
		.awlen_en = 1,
		.awlen = 16 - 1,
	};
	struct esp32p4_dsi_lli *lli;

	if (IS_ENABLED(CONFIG_ESP32P4_NOMMU_XIP)) {
		dmac_channel_reg_t *ch =
			(dmac_channel_reg_t *)&dsi->gdma->ch[DSI_GDMA_CHANNEL];

		/*
		 * TRM ch.7 VDMA - Auto Reloading, DMAC_CHn_CFG0 and IOC_BLKTFR.
		 * Reload the same RGB565 source and fixed DSI FIFO address at
		 * every block boundary. Leave block completion disabled so the
		 * next frame can start without the CPU servicing an interrupt.
		 */
		ch->sar0.val = lower_32_bits(dsi->fb_phys);
		ch->dar0.val = DSI_BRIDGE_MEM_BASE;
		ch->block_ts0.val = dsi->block_size - 1;
		ch->ctl0.val = ctl0.val;
		ch->ctl1.val = ctl1.val;
		ch->llp0.val = 0;
		ch->llp1.val = 0;
		return;
	}

	lli = dsi_lli_view(dsi);
	memset(lli, 0, sizeof(*lli));
	lli->sar_lo.sar0 = lower_32_bits(dsi->fb_phys);
	lli->dar_lo.dar0 = DSI_BRIDGE_MEM_BASE;
	lli->block_ts_lo.block_ts = dsi->block_size - 1;
	lli->ctrl_lo.val = ctl0.val;
	lli->ctrl_hi.val = ctl1.val;
	lli->ctrl_hi.shadowreg_or_lli_last = 1;
	lli->ctrl_hi.shadowreg_or_lli_valid = 1;
	lli->llp_lo.lms = DSI_GDMA_MASTER_PORT_MEMORY;
	dma_wmb();
}

static int dsi_dma_init(struct esp32p4_dsi *dsi)
{
	u64 frame_bytes = (u64)dsi->width * dsi->height * 2;
	u32 block_size;
	int ret;

	if (frame_bytes % 8) {
		dev_err(dsi->dev, "framebuffer size is not 64-bit aligned\n");
		return -EINVAL;
	}
	block_size = frame_bytes / 8;
	if (!block_size || block_size > GENMASK(21, 0)) {
		dev_err(dsi->dev, "invalid VDMA block size %u\n", block_size);
		return -EINVAL;
	}

	ret = dsi_dma_reset(dsi);
	if (ret) {
		dev_err(dsi->dev, "DW-GDMA reset timed out\n");
		return ret;
	}

	dsi->block_size = block_size;
	dsi_dma_channel_config(dsi);

	dev_info(dsi->dev, "VDMA channel %d, %llu bytes, %u x 64-bit (%s)\n",
		 DSI_GDMA_CHANNEL, frame_bytes, block_size,
		 IS_ENABLED(CONFIG_ESP32P4_NOMMU_XIP) ? "address reload" :
							"linked list");
	return 0;
}

/* ------------------------------------------------------------------------- */
/* Stream start / stop / re-arm						     */
/* ------------------------------------------------------------------------- */

static void dsi_channel_disable(struct esp32p4_dsi *dsi)
{
	dsi->gdma->chen0.val = 0x100 << DSI_GDMA_CHANNEL;
}

static void dsi_channel_enable(struct esp32p4_dsi *dsi)
{
	dsi->gdma->chen0.val = 0x101 << DSI_GDMA_CHANNEL;
}

/*
 * Re-arm the legacy single LLI. VDMA clears its valid bit on descriptor
 * writeback, so it must be set before every enable.
 * See mipi_dsi_dma_trans_done_cb() in esp_lcd_panel_dpi.c.
 */
static void dsi_lli_rearm(struct esp32p4_dsi *dsi)
{
	struct esp32p4_dsi_lli *lli = dsi_lli_view(dsi);

	lli->ctrl_hi.shadowreg_or_lli_last = 1;
	lli->ctrl_hi.shadowreg_or_lli_valid = 1;
	dma_wmb();
	if (!dsi->lli_in_sram)
		esp32p4_display_cache_writeback((u32)dsi->lli_dma,
						(u32)sizeof(*lli));

	dsi->gdma->ch[DSI_GDMA_CHANNEL].llp0.lms = DSI_GDMA_MASTER_PORT_MEMORY;
	dsi->gdma->ch[DSI_GDMA_CHANNEL].llp0.loc0 =
		lower_32_bits(dsi->lli_dma) >> 6;
	dsi->gdma->ch[DSI_GDMA_CHANNEL].llp1.val = 0;

	dsi_channel_enable(dsi);
}

static int dsi_stream_on(struct esp32p4_dsi *dsi)
{
	u32 events = DSI_DMA_EVENT_ERRORS;
	u32 depth;
	int ret;

	if (!IS_ENABLED(CONFIG_ESP32P4_NOMMU_XIP))
		events |= DSI_DMA_EVENT_FRAME_DONE;

	/* Drop any bridge event left by command-mode initialization. */
	dsi->bridge->int_clr.val = UINT_MAX;

	/* Blank the buffer while the channel is idle so no stale pixels appear. */
	if (dsi->fb_phys) {
		size_t bytes = (size_t)dsi->width * dsi->height * 2;

		memset((void *)(uintptr_t)dsi->fb_phys, 0, bytes);
		esp32p4_display_cache_writeback((u32)dsi->fb_phys, (u32)bytes);
	}

	/* Clear and arm the events before the channel can complete a block. */
	dsi->gdma->ch[DSI_GDMA_CHANNEL].int_clr0.val = UINT_MAX;
	dsi->gdma->ch[DSI_GDMA_CHANNEL].int_st_ena0.val = events;
	dsi->gdma->ch[DSI_GDMA_CHANNEL].int_sig_ena0.val = events;
	dsi_dma_program(dsi);
	if (IS_ENABLED(CONFIG_ESP32P4_NOMMU_XIP))
		dsi_channel_enable(dsi);
	else
		dsi_lli_rearm(dsi);

	/*
	 * TRM ch.44 MIPI DSI - DSI_BRG_FIFO_FLOW_STATUS and DSI_BRG_DPI_EN.
	 * Prime the raw FIFO while DPI is idle so the initial active line
	 * does not compete with the first VDMA descriptor fetch.
	 */
	ret = readl_poll_timeout_atomic(&dsi->bridge->fifo_flow_status.val,
					depth, (depth & GENMASK(13, 0)) >=
					DSI_BRIDGE_PREFILL_WORDS,
					1, DSI_BRIDGE_PREFILL_US);
	if (ret) {
		dsi_channel_disable(dsi);
		dsi->gdma->ch[DSI_GDMA_CHANNEL].int_sig_ena0.val = 0;
		dsi->gdma->ch[DSI_GDMA_CHANNEL].int_clr0.val = UINT_MAX;
		dev_err(dsi->dev, "VDMA failed to prefill DSI FIFO (depth %u)\n", depth);
		return ret;
	}

	dsi->host->mode_cfg.cmd_video_mode = 0;
	dsi->bridge->dpi_misc_config.dpi_en = 1;
	dsi->bridge->dpi_config_update.dpi_config_update = 1;

	return 0;
}

int esp32p4_dsi_start(struct esp32p4_dsi *dsi)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&dsi->lock, flags);
	if (!dsi->hw_ready) {
		ret = -ENODEV;
		goto out;
	}
	if (dsi->started || dsi->fatal) {
		ret = dsi->fatal ? -EIO : 0;
		goto out;
	}

	ret = dsi_stream_on(dsi);
	if (!ret)
		dsi->started = true;
out:
	spin_unlock_irqrestore(&dsi->lock, flags);
	return ret;
}

void esp32p4_dsi_stop(struct esp32p4_dsi *dsi)
{
	unsigned long flags;

	if (!dsi->hw_ready)
		return;

	spin_lock_irqsave(&dsi->lock, flags);
	dsi->gdma->ch[DSI_GDMA_CHANNEL].int_st_ena0.val = 0;
	dsi->gdma->ch[DSI_GDMA_CHANNEL].int_sig_ena0.val = 0;
	dsi->bridge->dpi_misc_config.dpi_en = 0;
	dsi->bridge->dpi_config_update.dpi_config_update = 1;
	dsi_channel_disable(dsi);
	dsi->host->mode_cfg.cmd_video_mode = 1;
	dsi->gdma->ch[DSI_GDMA_CHANNEL].int_clr0.val = UINT_MAX;
	if (dsi->lli) {
		dsi_lli_view(dsi)->ctrl_hi.shadowreg_or_lli_valid = 0;
		dma_wmb();
	}
	dsi->started = false;
	spin_unlock_irqrestore(&dsi->lock, flags);
}

static int dsi_handle_irq_locked(struct esp32p4_dsi *dsi)
{
	dmac_channel_reg_t *ch =
		(dmac_channel_reg_t *)&dsi->gdma->ch[DSI_GDMA_CHANNEL];
	u32 status = ch->int_st0.val;
	int ret = 0;

	if (!dsi->started)
		return 0;

	if (!status)
		return 0;

	ch->int_clr0.val = status;

	if (status & DSI_DMA_EVENT_ERRORS) {
		dev_err_ratelimited(dsi->dev, "VDMA error, status=0x%08x\n",
				    status);
		ch->int_st_ena0.val = 0;
		ch->int_sig_ena0.val = 0;
		dsi->bridge->dpi_misc_config.dpi_en = 0;
		dsi->bridge->dpi_config_update.dpi_config_update = 1;
		dsi_channel_disable(dsi);
		dsi->fatal = true;
		return -EIO;
	}

	if (IS_ENABLED(CONFIG_ESP32P4_NOMMU_XIP) ||
	    !(status & DSI_DMA_EVENT_FRAME_DONE))
		return 0;

	/* Restore the legacy descriptor after VDMA writes it back. */
	dsi_lli_rearm(dsi);
	ret = 1;

	return ret;
}

/*
 * Bring up the panel controller and the backlight once the DSI stream runs.
 * Deferred controllers answer only after the link is active.
 */
int esp32p4_dsi_late_init(struct esp32p4_dsi *dsi)
{
	int ret;

	if (!dsi)
		return -EINVAL;
	/* Match the IDF panel enable order: video first, then DCS DISPON. */
	ret = dsi_dcs_write(dsi, 0x29, NULL, 0);
	if (ret)
		return ret;

	if (dsi->backlight)
		return backlight_enable(dsi->backlight);
	msleep(200);

	dev_info(dsi->dev, "dsi st: host0=0x%08x host1=0x%08x phy=0x%08x brg=0x%08x fifo=0x%08x en=0x%08x\n",
		 dsi->host->int_st0.val, dsi->host->int_st1.val,
		 dsi->host->phy_status.val, dsi->bridge->int_st.val,
		 dsi->bridge->fifo_flow_status.val, dsi->bridge->en.val);
	dev_info(dsi->dev, "phy cfg: pwr=0x%08x rstz=0x%08x lpclk=0x%08x mode=0x%08x vmode=0x%08x\n",
		 dsi->host->pwr_up.val, dsi->host->phy_rstz.val,
		 dsi->host->lpclk_ctrl.val, dsi->host->mode_cfg.val,
		 dsi->host->vid_mode_cfg.val);

	ret = dsi_mcu_write(dsi, 3, DSI_MCU_BRIGHTNESS);
	dev_info(dsi->dev, "backlight brightness write %d\n", ret);
	return ret;
}

int esp32p4_dsi_rearm(struct esp32p4_dsi *dsi)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&dsi->lock, flags);
	ret = dsi_handle_irq_locked(dsi);
	spin_unlock_irqrestore(&dsi->lock, flags);
	return ret;
}

int esp32p4_dsi_irq(struct esp32p4_dsi *dsi)
{
	return esp32p4_dsi_rearm(dsi);
}

unsigned int esp32p4_dsi_irq_number(struct esp32p4_dsi *dsi)
{
	return dsi->irq;
}

/* ------------------------------------------------------------------------- */
/* Probe / create / destroy						     */
/* ------------------------------------------------------------------------- */

/*
 * DW-GDMA source 24 handles scanout DMA errors and, for the legacy linked
 * list, completion. The DSI bridge interrupt is a separate source.
 */
static int dsi_map_irq(struct esp32p4_dsi *dsi)
{
	struct device_node *np = dsi->dev->of_node;
	struct device_node *parent;
	struct irq_domain *domain;
	unsigned int irq;

	if (!np)
		return -EINVAL;

	parent = of_irq_find_parent(np);
	domain = parent ? irq_find_host(parent) : NULL;
	of_node_put(parent);
	if (domain) {
		irq = irq_create_mapping(domain, DSI_GDMA_HWIRQ);
		if (irq) {
			dsi->irq = irq;
			return 0;
		}
	}

	/* Fall back to the interrupt described by the firmware. */
	irq = platform_get_irq(to_platform_device(dsi->dev), 0);
	if ((int)irq < 0)
		return irq;

	dsi->irq = irq;
	dev_warn(dsi->dev,
		 "using DT IRQ %u; expected DW-GDMA source %d\n",
		 irq, DSI_GDMA_HWIRQ);
	return 0;
}

struct esp32p4_dsi *esp32p4_dsi_create(struct device *dev,
				       const struct esp32p4_dsi_config *cfg)
{
	struct esp32p4_dsi *dsi;
	struct device_node *np;
	u64 frame_bytes;
	int ret;

	if (!dev || !cfg || !cfg->fb_phys || !cfg->width || !cfg->height)
		return ERR_PTR(-EINVAL);

	frame_bytes = (u64)cfg->width * cfg->height * 2;
	if (frame_bytes > SIZE_MAX)
		return ERR_PTR(-EINVAL);

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return ERR_PTR(-ENOMEM);

	dsi->dev = dev;
	/*
	 * TODO(verify): cache maintenance for the RGB565 framebuffer is the fb
	 * driver's responsibility; the VDMA reads it through the memory port and
	 * this driver never flushes or invalidates it.
	 */
	dsi->fb_phys = cfg->fb_phys;
	dsi->width = cfg->width;
	dsi->height = cfg->height;
	spin_lock_init(&dsi->lock);

	dsi->host_base = devm_ioremap(dev, DSI_HOST_BASE, 0x1000);
	if (!dsi->host_base)
		return ERR_PTR(-ENOMEM);
	dsi->host = (dsi_host_dev_t *)dsi->host_base;
	dsi->bridge = (dsi_brg_dev_t *)(dsi->host_base + DSI_BRIDGE_OFFSET);

	dsi->clkrst = devm_ioremap(dev, HP_SYS_CLKRST_BASE, 0x1000);
	dsi->gdma_base = devm_ioremap(dev, DW_GDMA_BASE,
				      sizeof(dw_gdma_dev_t));
	dsi->pmu = devm_ioremap(dev, PMU_BASE, 0x400);
	dsi->efuse = devm_ioremap(dev, EFUSE_BASE, 0x100);
	if (!dsi->clkrst || !dsi->gdma_base || !dsi->pmu || !dsi->efuse)
		return ERR_PTR(-ENOMEM);
	dsi->gdma = (dw_gdma_dev_t *)dsi->gdma_base;

	dsi->panel_reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(dsi->panel_reset))
		return ERR_CAST(dsi->panel_reset);
	if (dsi->panel_reset) {
		dsi->backlight = devm_of_find_backlight(dev);
		if (IS_ERR(dsi->backlight))
			return ERR_CAST(dsi->backlight);
		if (!dsi->backlight)
			return ERR_PTR(-EINVAL);
	} else {
		np = of_find_compatible_node(NULL, NULL,
					     "raspberrypi,touch-display-2-backlight");
		if (!np)
			return ERR_PTR(-ENODEV);
		dsi->mcu = of_find_i2c_device_by_node(np);
		of_node_put(np);
		if (!dsi->mcu)
			return ERR_PTR(-EPROBE_DEFER);
		if (dsi->mcu->addr != DSI_MCU_I2C_ADDR) {
			ret = -ENODEV;
			goto err_mcu;
		}
	}

	if (!IS_ENABLED(CONFIG_ESP32P4_NOMMU_XIP)) {
		struct device_node *fb_node;
		u32 lli_addr = 0;

		ret = dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
		if (ret)
			goto err_mcu;

		fb_node = of_get_compatible_child(of_chosen, P4_DSI_FB_COMPAT);
		if (fb_node) {
			of_property_read_u32(fb_node, "espressif,dsi-lli",
					     &lli_addr);
			of_node_put(fb_node);
		}
		if (lli_addr) {
			dsi->lli = (void *)(uintptr_t)lli_addr;
			dsi->lli_dma = lli_addr;
			dsi->lli_in_sram = true;
		} else {
			dsi->lli = dma_alloc_coherent(dev, sizeof(*dsi->lli),
						      &dsi->lli_dma, GFP_KERNEL);
			if (!dsi->lli) {
				ret = -ENOMEM;
				goto err_mcu;
			}
		}
	}

	ret = dsi_map_irq(dsi);
	if (ret) {
		dev_err(dev, "failed to resolve scanout IRQ: %d\n", ret);
		goto err_lli;
	}

	dev_dbg(dev, "DSI host @%p bridge @%p gdma @%p irq=%u\n",
		dsi->host, dsi->bridge, dsi->gdma, dsi->irq);
	return dsi;

err_lli:
	if (dsi->lli && !dsi->lli_in_sram)
		dma_free_coherent(dev, sizeof(*dsi->lli), dsi->lli,
				  dsi->lli_dma);
	dsi->lli = NULL;
err_mcu:
	if (dsi->mcu)
		put_device(&dsi->mcu->dev);
	dsi->mcu = NULL;
	return ERR_PTR(ret);
}

int esp32p4_dsi_probe(struct esp32p4_dsi *dsi)
{
	int i;
	int ret;

	/*
	 * Wake the panel controller while the PHY is still quiet: once the PLL is
	 * locked and the clock lane runs in HS it stops acknowledging on I2C.
	 */
	ret = dsi_ldo_enable(dsi, 2500);
	if (ret) {
		dev_err(dsi->dev, "PHY LDO enable failed: %d\n", ret);
		return ret;
	}

	dsi_enable_bus_clocks(dsi);
	dsi_enable_phy_clocks(dsi);

	ret = dsi_host_init(dsi);
	if (ret)
		return ret;
	dsi->hw_ready = true;

	if (dsi->panel_reset) {
		/*
		 * The GPIO is asserted in create(). Use the reference board's
		 * 100 ms LCD reset pulse. The touch supply owns CTP reset.
		 */
		msleep(100);
		ret = gpiod_set_value_cansleep(dsi->panel_reset, 0);
		if (ret)
			return ret;
		/* Touch Display 2 requires 120 ms after LCD reset before DCS. */
		msleep(120);
	} else {
		/*
		 * The panel controller answers only once the link is up, and its reset
		 * sequence holds the panel in reset, so it must run before the panel is
		 * initialised. Retry while the controller boots.
		 */
		for (i = 0; i < 20; i++) {
			ret = dsi_panel_mcu_reset(dsi);
			if (!ret)
				break;
			msleep(50);
		}
		if (ret) {
			dev_warn(dsi->dev,
				 "panel controller did not answer (%d); continuing on DSI only\n",
				 ret);
			ret = 0;
		}
	}

	ret = dsi_panel_dcs_init(dsi);
	if (ret) {
		dev_err(dsi->dev, "panel DCS init failed: %d\n", ret);
		return ret;
	}

	ret = dsi_dpi_init(dsi);
	if (ret)
		return ret;

	ret = dsi_dma_init(dsi);
	if (ret)
		return ret;

	/* Retained-loader DTBs provide only the raw MCU brightness register. */
	if (dsi->mcu) {
		ret = dsi_mcu_write(dsi, 3, DSI_MCU_BRIGHTNESS);
		if (ret)
			dev_warn(dsi->dev, "backlight write failed: %d\n", ret);
	}

	dev_info(dsi->dev, "ESP32-P4 DSI panel ready (%ux%u)\n",
		 dsi->width, dsi->height);
	return 0;
}

/*
 * ILI9881C datasheet, sections 5.3.3 and 5.3.4. Sample the sink after
 * stopping DPI, when DBI can use bus turnaround without disrupting video.
 * RDNUMED clears on read. Bit 7 reports counter overflow.
 */
void esp32p4_dsi_report_link(struct esp32p4_dsi *dsi)
{
	u8 errors, power;
	int ret;

	ret = dsi_dcs_read(dsi, 0x05, &errors, sizeof(errors));
	if (ret) {
		dev_dbg(dsi->dev, "panel DSI error counter unavailable: %d\n", ret);
		return;
	}
	ret = dsi_dcs_read(dsi, 0x0a, &power, sizeof(power));
	if (ret) {
		dev_info(dsi->dev, "panel DSI error count %#02x, power mode unavailable: %d\n",
			 errors, ret);
		return;
	}
	dev_info(dsi->dev, "panel DSI error count %#02x, power mode %#02x\n",
		 errors, power);
}

void esp32p4_dsi_destroy(struct esp32p4_dsi *dsi)
{
	if (!dsi)
		return;

	if (dsi->backlight)
		backlight_disable(dsi->backlight);

	if (dsi->hw_ready) {
		esp32p4_dsi_stop(dsi);
		dsi->bridge->en.dsi_en = 0;
		dsi_disable_dpi_clock(dsi);
	}
	dsi_ldo_disable(dsi);

	if (dsi->lli && !dsi->lli_in_sram)
		dma_free_coherent(dsi->dev, sizeof(*dsi->lli), dsi->lli,
				  dsi->lli_dma);
	if (dsi->mcu)
		put_device(&dsi->mcu->dev);
}
