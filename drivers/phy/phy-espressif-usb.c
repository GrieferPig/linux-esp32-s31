// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif integrated USB2 UTMI PHY
 *
 * The clock, reset, low-speed and pull-down sequence follows ESP-IDF's
 * usb_utmi_hal/usb_utmi_ll implementations for ESP32-S31 and ESP32-P4.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#define ESP32S31_USB_CLK_APB_EN		BIT(0)
#define ESP32S31_USB_CLK_SYS_EN		BIT(1)

#define ESP32S31_USB_UTMIFS_CLK_EN	BIT(23)
#define ESP32S31_USB_PHYREF_CLK_EN	BIT(27)
#define ESP32S31_USB_PHY_RST_EN		BIT(29)
#define ESP32S31_USB_AHB_RST_EN		BIT(30)
#define ESP32S31_USB_APB_RST_EN		BIT(31)

#define ESP32S31_USB_DM_PULLDOWN		BIT(2)
#define ESP32S31_USB_DP_PULLDOWN		BIT(3)

#define ESP32S31_USB_PHY_PLL_FORCE_EN	BIT(0)
#define ESP32S31_USB_PHY_SUSPEND_FORCE_EN BIT(2)
#define ESP32S31_USB_PHY_OTG_SUSPENDM	BIT(7)

#define ESP32S31_UTMI_FC06		0x18
#define ESP32S31_UTMI_LS_PAR_EN		BIT(0)
#define ESP32S31_UTMI_LS_KPALV_EN	BIT(3)

/* TRM ch.12 Reset and Clock - LP_CLKRST_HP_USB_CLKRST_CTRL1.
 * TRM ch.54 USB 2.0 High-Speed OTG - UTMI PHY and disconnect detection.
 */
#define ESP32P4_USB_PHYREF_CLK_EN	BIT(30)
#define ESP32P4_USB_PHY_RST_EN		BIT(1)
#define ESP32P4_USB_CORE_RST_EN		BIT(2)
#define ESP32P4_USB_OTG_SUSPENDM		BIT(21)

struct espressif_usb_phy_variant {
	u32 refclk;
	u32 phy_reset;
	u32 core_reset;
	u32 overrides;
	u32 suspendm;
	u32 pulldowns;
	bool external_bus_clock;
};

struct espressif_usb_phy {
	const struct espressif_usb_phy_variant *variant;
	struct clk *bus_clock;
	void __iomem *utmi;
	void __iomem *clkrst;
	void __iomem *otg_ctrl;
	void __iomem *usb_ctrl;
	void __iomem *phy_ctrl;
	struct phy *phy;
	enum phy_mode mode;
	bool powered;
};

static void espressif_usb_phy_pm_disable(void *data)
{
	pm_runtime_disable(data);
}

static void espressif_usb_update(void __iomem *reg, u32 clear, u32 set)
{
	u32 value = readl(reg);

	value &= ~clear;
	value |= set;
	writel(value, reg);
}

static void espressif_usb_set_pulldowns(struct espressif_usb_phy *priv)
{
	u32 pulldowns = priv->variant->pulldowns;

	if (pulldowns)
		espressif_usb_update(priv->usb_ctrl, pulldowns,
				     priv->mode == PHY_MODE_USB_HOST ? pulldowns : 0);
}

static int espressif_usb_phy_init(struct phy *phy)
{
	struct espressif_usb_phy *priv = phy_get_drvdata(phy);
	const struct espressif_usb_phy_variant *variant = priv->variant;
	u32 resets = variant->phy_reset | variant->core_reset;
	int ret;

	if (priv->bus_clock) {
		ret = clk_prepare_enable(priv->bus_clock);
		if (ret)
			return ret;
	} else {
		espressif_usb_update(priv->clkrst, 0, ESP32S31_USB_CLK_APB_EN |
				     ESP32S31_USB_CLK_SYS_EN);
	}
	espressif_usb_update(priv->otg_ctrl, 0, variant->refclk);

	/* S31 routes suspend and PLL control to DWC2. */
	if (variant->overrides)
		espressif_usb_update(priv->phy_ctrl, variant->overrides, 0);

	/* Release the PHY before the controller-side reset lines. */
	espressif_usb_update(priv->otg_ctrl, 0, resets);
	espressif_usb_update(priv->otg_ctrl, variant->phy_reset, 0);
	espressif_usb_update(priv->otg_ctrl, variant->core_reset, 0);

	espressif_usb_update(priv->phy_ctrl, 0, variant->suspendm);
	espressif_usb_update(priv->utmi + ESP32S31_UTMI_FC06, 0,
			     ESP32S31_UTMI_LS_PAR_EN | ESP32S31_UTMI_LS_KPALV_EN);
	espressif_usb_set_pulldowns(priv);
	priv->powered = true;

	return 0;
}

static int espressif_usb_phy_exit(struct phy *phy)
{
	struct espressif_usb_phy *priv = phy_get_drvdata(phy);
	const struct espressif_usb_phy_variant *variant = priv->variant;

	if (variant->pulldowns)
		espressif_usb_update(priv->usb_ctrl, variant->pulldowns, 0);
	espressif_usb_update(priv->otg_ctrl, variant->refclk, 0);
	if (priv->bus_clock)
		clk_disable_unprepare(priv->bus_clock);
	else
		espressif_usb_update(priv->clkrst, ESP32S31_USB_CLK_APB_EN |
				     ESP32S31_USB_CLK_SYS_EN, 0);
	priv->powered = false;

	return 0;
}

static int espressif_usb_phy_set_mode(struct phy *phy, enum phy_mode mode,
				     int submode)
{
	struct espressif_usb_phy *priv = phy_get_drvdata(phy);

	(void)submode;

	if (mode != PHY_MODE_USB_HOST && mode != PHY_MODE_USB_DEVICE &&
	    mode != PHY_MODE_USB_OTG)
		return -EINVAL;

	priv->mode = mode;
	if (priv->powered)
		espressif_usb_set_pulldowns(priv);

	return 0;
}

static const struct phy_ops espressif_usb_phy_ops = {
	.init = espressif_usb_phy_init,
	.exit = espressif_usb_phy_exit,
	.set_mode = espressif_usb_phy_set_mode,
	.owner = THIS_MODULE,
};

static int espressif_usb_phy_probe(struct platform_device *pdev)
{
	struct phy_provider *provider;
	struct espressif_usb_phy *priv;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->variant = device_get_match_data(&pdev->dev);

	priv->utmi = devm_platform_ioremap_resource_byname(pdev, "utmi");
	if (IS_ERR(priv->utmi))
		return PTR_ERR(priv->utmi);

	if (priv->variant->external_bus_clock) {
		priv->bus_clock = devm_clk_get(&pdev->dev, NULL);
		if (IS_ERR(priv->bus_clock))
			return PTR_ERR(priv->bus_clock);
	} else {
		priv->clkrst = devm_platform_ioremap_resource_byname(pdev, "clkrst");
		if (IS_ERR(priv->clkrst))
			return PTR_ERR(priv->clkrst);
	}

	priv->otg_ctrl = devm_platform_ioremap_resource_byname(pdev,
							 "otg-ctrl");
	if (IS_ERR(priv->otg_ctrl))
		return PTR_ERR(priv->otg_ctrl);

	if (priv->variant->pulldowns) {
		priv->usb_ctrl = devm_platform_ioremap_resource_byname(pdev, "usb-ctrl");
		if (IS_ERR(priv->usb_ctrl))
			return PTR_ERR(priv->usb_ctrl);
	}

	priv->phy_ctrl = devm_platform_ioremap_resource_byname(pdev,
							 "phy-ctrl");
	if (IS_ERR(priv->phy_ctrl))
		return PTR_ERR(priv->phy_ctrl);

	priv->mode = PHY_MODE_USB_HOST;

	/*
	 * Enable runtime PM before creating the generic PHY.  The PHY core then
	 * mirrors runtime PM into its child device, so a PHY user resumes this
	 * provider (and HPCNNT) before any register access.
	 */
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	ret = devm_add_action_or_reset(&pdev->dev,
				       espressif_usb_phy_pm_disable,
				       &pdev->dev);
	if (ret)
		return ret;

	priv->phy = devm_phy_create(&pdev->dev, NULL,
				    &espressif_usb_phy_ops);
	if (IS_ERR(priv->phy))
		return PTR_ERR(priv->phy);

	phy_set_drvdata(priv->phy, priv);
	platform_set_drvdata(pdev, priv);

	provider = devm_of_phy_provider_register(&pdev->dev,
						 of_phy_simple_xlate);
	if (IS_ERR(provider))
		return PTR_ERR(provider);

	/* No PHY user exists yet, so release the provider's initial active vote. */
	pm_runtime_idle(&pdev->dev);
	dev_info(&pdev->dev, "integrated 16-bit UTMI+ PHY registered\n");
	return 0;
}

static const struct espressif_usb_phy_variant esp32s31_phy = {
	.refclk = ESP32S31_USB_UTMIFS_CLK_EN | ESP32S31_USB_PHYREF_CLK_EN,
	.phy_reset = ESP32S31_USB_PHY_RST_EN,
	.core_reset = ESP32S31_USB_AHB_RST_EN | ESP32S31_USB_APB_RST_EN,
	.overrides = ESP32S31_USB_PHY_PLL_FORCE_EN | ESP32S31_USB_PHY_SUSPEND_FORCE_EN,
	.suspendm = ESP32S31_USB_PHY_OTG_SUSPENDM,
	.pulldowns = ESP32S31_USB_DM_PULLDOWN | ESP32S31_USB_DP_PULLDOWN,
};

/* P4 rev1 routes D+/D- pulldowns through the USB OTG controller. */
static const struct espressif_usb_phy_variant esp32p4_rev1_phy = {
	.refclk = ESP32P4_USB_PHYREF_CLK_EN,
	.phy_reset = ESP32P4_USB_PHY_RST_EN,
	.core_reset = ESP32P4_USB_CORE_RST_EN,
	.suspendm = ESP32P4_USB_OTG_SUSPENDM,
	.external_bus_clock = true,
};

static const struct of_device_id espressif_usb_phy_of_match[] = {
	{ .compatible = "espressif,esp32s31-usb-phy", .data = &esp32s31_phy },
	{ .compatible = "espressif,esp32p4-rev1-usb-phy", .data = &esp32p4_rev1_phy },
	{ }
};
MODULE_DEVICE_TABLE(of, espressif_usb_phy_of_match);

static struct platform_driver espressif_usb_phy_driver = {
	.probe = espressif_usb_phy_probe,
	.driver = {
		.name = "esp32s31-usb-phy",
		.of_match_table = espressif_usb_phy_of_match,
	},
};
module_platform_driver(espressif_usb_phy_driver);

MODULE_DESCRIPTION("Espressif USB2 UTMI PHY driver");
MODULE_LICENSE("GPL");
