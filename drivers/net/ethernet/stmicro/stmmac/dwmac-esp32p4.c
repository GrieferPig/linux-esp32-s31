// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "stmmac.h"
#include "stmmac_platform.h"

struct esp32p4_dwmac {
	struct device *dev;
	struct clk *rmii;
};

static void esp32p4_dwmac_set_speed(void *data, int speed, unsigned int mode)
{
	struct esp32p4_dwmac *dwmac = data;
	unsigned long rate;
	int ret;

	/* TRM ch.57 EMAC - RX/TX clocks divide the external 50 MHz RMII clock. */
	switch (speed) {
	case SPEED_10:
		rate = 2500000;
		break;
	case SPEED_100:
		rate = 25000000;
		break;
	default:
		dev_err(dwmac->dev, "invalid RMII speed %d\n", speed);
		return;
	}

	ret = clk_set_rate(dwmac->rmii, rate);
	if (ret)
		dev_err(dwmac->dev, "failed to set RMII rate %lu: %d\n", rate, ret);
}

static int esp32p4_dwmac_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct plat_stmmacenet_data *plat;
	struct stmmac_resources res;
	struct esp32p4_dwmac *dwmac;
	int ret;

	ret = stmmac_get_platform_resources(pdev, &res);
	if (ret)
		return ret;

	dwmac = devm_kzalloc(dev, sizeof(*dwmac), GFP_KERNEL);
	if (!dwmac)
		return -ENOMEM;
	dwmac->dev = dev;
	dwmac->rmii = devm_clk_get_enabled(dev, "rmii");
	if (IS_ERR(dwmac->rmii))
		return dev_err_probe(dev, PTR_ERR(dwmac->rmii), "RMII clock unavailable\n");
	ret = clk_set_rate(dwmac->rmii, 25000000);
	if (ret)
		return dev_err_probe(dev, ret, "could not set initial RMII rate\n");

	plat = devm_stmmac_probe_config_dt(pdev, res.mac);
	if (IS_ERR(plat))
		return PTR_ERR(plat);

	plat->has_gmac = 1;
	/* TRM ch.57 EMAC - the 256-byte FIFOs cannot store a full frame.
	 * Use cut-through thresholds as in IDF emac_hal_init_dma_default().
	 */
	plat->force_thresh_dma_mode = 1;
	/*
	 * TRM ch.57 EMAC - AHB TX DMA must refill the 256-byte FIFO before
	 * it drains. The RX FIFO is also only 256 bytes. PSRAM bursts at
	 * 100 Mb/s caused TX underruns and RX errors. Stage both directions
	 * in coherent low SRAM, as in IDF emac_esp_dma_{transmit,receive}_frame().
	 */
	/* Keep the two descriptor rings within one SRAM page each. */
	plat->tx_coherent_slots = 8;
	plat->rx_coherent_slots = 8;
	plat->maxmtu = ETH_DATA_LEN;
	plat->bsp_priv = dwmac;
	plat->fix_mac_speed = esp32p4_dwmac_set_speed;

	return devm_stmmac_pltfr_probe(pdev, plat, &res);
}

static const struct of_device_id esp32p4_dwmac_match[] = {
	{ .compatible = "espressif,esp32p4-dwmac" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32p4_dwmac_match);

static struct platform_driver esp32p4_dwmac_driver = {
	.probe = esp32p4_dwmac_probe,
	.driver = {
		.name = "dwmac-esp32p4",
		.of_match_table = esp32p4_dwmac_match,
		.pm = &stmmac_pltfr_pm_ops,
	},
};
module_platform_driver(esp32p4_dwmac_driver);

MODULE_DESCRIPTION("Espressif ESP32-P4 RMII Ethernet controller");
MODULE_LICENSE("GPL");
