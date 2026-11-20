// SPDX-License-Identifier: GPL-2.0-only
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

/* TRM ch.12 — LP_CLKRST_HP_USB_CLKRST_CTRL0/1, OTG11 clock and reset. */
#define P4_USBFS_CLK48_EN	BIT(4)
#define P4_USBFS_RESET		BIT(3)

/* TRM ch.22 System Registers — LP_SYSTEM_USB_CTRL PHY selection. */
#define P4_USB_PHY_SW_SEL	BIT(0)
#define P4_USB_PHY_SWAP		BIT(1)

/* TRM ch.55 USB Full-Speed OTG — USB_WRAP_OTG_CONF internal PHY and pad. */
#define P4_USB_WRAP_EXTERNAL	BIT(2)
#define P4_USB_WRAP_PAD_EN	BIT(18)
#define P4_USB_WRAP_CLK_OVERRIDE (BIT(19) | BIT(20))

struct esp32p4_usbfs {
	struct clk *bus_clk;
	struct regmap *reset;
	void __iomem *wrap;
	void __iomem *clock;
	void __iomem *route;
	u32 route_before;
	u32 phy_index;
};

static void esp32p4_usbfs_update(void __iomem *reg, u32 mask, u32 value)
{
	writel((readl(reg) & ~mask) | value, reg);
}

static int esp32p4_usbfs_reset(struct esp32p4_usbfs *usb)
{
	int ret;

	ret = clk_prepare_enable(usb->bus_clk);
	if (ret)
		return ret;
	esp32p4_usbfs_update(usb->clock, P4_USBFS_CLK48_EN, P4_USBFS_CLK48_EN);
	ret = regmap_set_bits(usb->reset, 0, P4_USBFS_RESET);
	if (!ret)
		ret = regmap_clear_bits(usb->reset, 0, P4_USBFS_RESET);
	esp32p4_usbfs_update(usb->clock, P4_USBFS_CLK48_EN, 0);
	clk_disable_unprepare(usb->bus_clk);

	return ret;
}

static int esp32p4_usbfs_init(struct phy *phy)
{
	struct esp32p4_usbfs *usb = phy_get_drvdata(phy);
	u32 route_mask = P4_USB_PHY_SW_SEL | P4_USB_PHY_SWAP;
	u32 swap = usb->phy_index == 0 ? P4_USB_PHY_SWAP : 0;
	int ret;

	ret = clk_prepare_enable(usb->bus_clk);
	if (ret)
		return ret;

	esp32p4_usbfs_update(usb->clock, P4_USBFS_CLK48_EN, P4_USBFS_CLK48_EN);

	/* Keep the controller disconnected until its internal PHY is routed. */
	esp32p4_usbfs_update(usb->wrap, P4_USB_WRAP_EXTERNAL | P4_USB_WRAP_PAD_EN |
			    P4_USB_WRAP_CLK_OVERRIDE, 0);
	usb->route_before = readl(usb->route) & route_mask;
	esp32p4_usbfs_update(usb->route, route_mask, P4_USB_PHY_SW_SEL | swap);
	esp32p4_usbfs_update(usb->wrap, P4_USB_WRAP_PAD_EN, P4_USB_WRAP_PAD_EN);
	return 0;
}

static int esp32p4_usbfs_exit(struct phy *phy)
{
	struct esp32p4_usbfs *usb = phy_get_drvdata(phy);

	esp32p4_usbfs_update(usb->wrap, P4_USB_WRAP_PAD_EN, 0);
	esp32p4_usbfs_update(usb->route, P4_USB_PHY_SW_SEL | P4_USB_PHY_SWAP,
			    usb->route_before);
	esp32p4_usbfs_update(usb->clock, P4_USBFS_CLK48_EN, 0);
	clk_disable_unprepare(usb->bus_clk);
	return 0;
}

static const struct phy_ops esp32p4_usbfs_ops = {
	.init = esp32p4_usbfs_init,
	.exit = esp32p4_usbfs_exit,
	.owner = THIS_MODULE,
};

static int esp32p4_usbfs_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *usj;
	struct esp32p4_usbfs *usb;
	struct phy *phy;
	bool usj_enabled;
	int ret;

	usb = devm_kzalloc(dev, sizeof(*usb), GFP_KERNEL);
	if (!usb)
		return -ENOMEM;

	ret = of_property_read_u32(dev->of_node, "espressif,phy-index", &usb->phy_index);
	if (ret || usb->phy_index > 1)
		return dev_err_probe(dev, -EINVAL, "invalid USB FS PHY index\n");

	usj = of_parse_phandle(dev->of_node, "espressif,shared-serial-jtag", 0);
	usj_enabled = usj && of_device_is_available(usj);
	of_node_put(usj);
	if (usb->phy_index == 0 && usj_enabled)
		return dev_err_probe(dev, -EBUSY, "PHY 0 is assigned to USB Serial/JTAG\n");

	usb->wrap = devm_platform_ioremap_resource_byname(pdev, "wrap");
	if (IS_ERR(usb->wrap))
		return PTR_ERR(usb->wrap);
	usb->clock = devm_platform_ioremap_resource_byname(pdev, "phy-clock");
	if (IS_ERR(usb->clock))
		return PTR_ERR(usb->clock);
	usb->route = devm_platform_ioremap_resource_byname(pdev, "phy-route");
	if (IS_ERR(usb->route))
		return PTR_ERR(usb->route);

	usb->bus_clk = devm_clk_get(dev, NULL);
	if (IS_ERR(usb->bus_clk))
		return dev_err_probe(dev, PTR_ERR(usb->bus_clk), "cannot get bus clock\n");

	usb->reset = syscon_regmap_lookup_by_phandle(dev->of_node, "espressif,otg-clkrst");
	if (IS_ERR(usb->reset))
		return dev_err_probe(dev, PTR_ERR(usb->reset), "cannot get OTG reset register\n");

	/*
	 * DWC2 gates the PHY before any gadget is bound. Only reset at
	 * probe so opening the PHY again preserves GUSBCFG force-device.
	 */
	ret = esp32p4_usbfs_reset(usb);
	if (ret)
		return dev_err_probe(dev, ret, "cannot reset OTG FS controller\n");

	phy = devm_phy_create(dev, NULL, &esp32p4_usbfs_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);
	phy_set_drvdata(phy, usb);

	return PTR_ERR_OR_ZERO(devm_of_phy_provider_register(dev, of_phy_simple_xlate));
}

static const struct of_device_id esp32p4_usbfs_of_match[] = {
	{ .compatible = "espressif,esp32p4-rev1-usb-fs-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32p4_usbfs_of_match);

static struct platform_driver esp32p4_usbfs_driver = {
	.probe = esp32p4_usbfs_probe,
	.driver = {
		.name = "esp32p4-usbfs-phy",
		.of_match_table = esp32p4_usbfs_of_match,
	},
};
module_platform_driver(esp32p4_usbfs_driver);

MODULE_DESCRIPTION("ESP32-P4 USB full-speed internal PHY selection");
MODULE_LICENSE("GPL");
