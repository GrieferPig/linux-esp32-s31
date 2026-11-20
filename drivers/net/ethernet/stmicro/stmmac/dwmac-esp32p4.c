// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>

#include "stmmac.h"
#include "stmmac_platform.h"

struct esp32p4_dwmac {
	struct device *dev;
	struct clk *rmii;
	struct notifier_block netdev_notifier;
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

static int esp32p4_dwmac_netdev_event(struct notifier_block *notifier,
				      unsigned long event, void *ptr)
{
	struct esp32p4_dwmac *dwmac = container_of(notifier, struct esp32p4_dwmac,
						  netdev_notifier);
	struct net_device *netdev = netdev_notifier_info_to_dev(ptr);
	struct stmmac_priv *priv;

	if (event != NETDEV_UP || netdev->dev.parent != dwmac->dev)
		return NOTIFY_DONE;

	/*
	 * TRM v1.3 ch.50 EMAC: TDES0.IC signals TX completion. The
	 * default 25-frame interval can fill the eight-slot SRAM ring
	 * before an interrupt, leaving reclamation to the TX timer.
	 * Apply the P4 threshold after stmmac initializes coalescing.
	 */
	priv = netdev_priv(netdev);
	WRITE_ONCE(priv->tx_coal_frames[0], 4);
	return NOTIFY_DONE;
}

static void esp32p4_dwmac_unregister_notifier(void *data)
{
	struct esp32p4_dwmac *dwmac = data;

	unregister_netdevice_notifier(&dwmac->netdev_notifier);
}

static void esp32p4_dwmac_release_dma(void *data)
{
	of_reserved_mem_device_release(data);
}

static void esp32p4_dwmac_tx_copy(void *dst, const void *src, unsigned int len)
{
	const u16 *in = src;
	u32 *out = dst;

	/*
	 * TRM ch.57 EMAC - TDES2 points at the TX buffer in coherent SRAM.
	 * Leave that address word-aligned. For the usual skb alignment,
	 * assemble each word from two aligned halfword loads.
	 */
	static_assert(STMMAC_COHERENT_TX_SLOT_SIZE % sizeof(u32) == 0);
	if (((unsigned long)src & (sizeof(u32) - 1)) == sizeof(u16)) {
		for (; len >= sizeof(u32); len -= sizeof(u32)) {
			*out++ = in[0] | ((u32)in[1] << 16);
			in += 2;
		}
	}
	memcpy(out, in, len);
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
	plat->tx_coherent_copy = esp32p4_dwmac_tx_copy;
	plat->rx_coherent_slots = 8;
	plat->maxmtu = ETH_DATA_LEN;
	plat->bsp_priv = dwmac;
	plat->fix_mac_speed = esp32p4_dwmac_set_speed;

	if (of_property_present(dev->of_node, "memory-region")) {
		ret = of_reserved_mem_device_init(dev);
		if (ret)
			return dev_err_probe(dev, ret, "could not attach EMAC DMA pool\n");

		ret = devm_add_action_or_reset(dev, esp32p4_dwmac_release_dma, dev);
		if (ret)
			return ret;
	}

	dwmac->netdev_notifier.notifier_call = esp32p4_dwmac_netdev_event;
	ret = register_netdevice_notifier(&dwmac->netdev_notifier);
	if (ret)
		return dev_err_probe(dev, ret, "could not register netdev notifier\n");

	ret = devm_add_action_or_reset(dev, esp32p4_dwmac_unregister_notifier,
				       dwmac);
	if (ret)
		return ret;

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
