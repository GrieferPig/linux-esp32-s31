// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif ESP32-S31 integrated USB2 UTMI PHY
 *
 * The clock, reset, low-speed and pull-down sequence follows ESP-IDF's
 * usb_utmi_hal/usb_utmi_ll implementation for ESP32-S31.
 */

#include <linux/bitops.h>
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

struct esp32s31_usb_phy {
	void __iomem *utmi;
	void __iomem *clkrst;
	void __iomem *otg_ctrl;
	void __iomem *usb_ctrl;
	void __iomem *phy_ctrl;
	struct phy *phy;
	enum phy_mode mode;
	bool powered;
};

static void esp32s31_usb_phy_pm_disable(void *data)
{
	pm_runtime_disable(data);
}

static void esp32s31_usb_update(void __iomem *reg, u32 clear, u32 set)
{
	u32 value = readl(reg);

	value &= ~clear;
	value |= set;
	writel(value, reg);
}

static void esp32s31_usb_set_pulldowns(struct esp32s31_usb_phy *priv)
{
	u32 pulldowns = ESP32S31_USB_DM_PULLDOWN |
			ESP32S31_USB_DP_PULLDOWN;

	esp32s31_usb_update(priv->usb_ctrl, pulldowns,
			     priv->mode == PHY_MODE_USB_HOST ? pulldowns : 0);
}

static int esp32s31_usb_phy_init(struct phy *phy)
{
	struct esp32s31_usb_phy *priv = phy_get_drvdata(phy);
	u32 resets = ESP32S31_USB_PHY_RST_EN |
		     ESP32S31_USB_AHB_RST_EN |
		     ESP32S31_USB_APB_RST_EN;

	/* Enable the DWC2 APB/system clocks and the UTMI/PHY reference clocks. */
	esp32s31_usb_update(priv->clkrst, 0,
			     ESP32S31_USB_CLK_APB_EN |
			     ESP32S31_USB_CLK_SYS_EN);
	esp32s31_usb_update(priv->otg_ctrl, 0,
			     ESP32S31_USB_UTMIFS_CLK_EN |
			     ESP32S31_USB_PHYREF_CLK_EN);

	/* Route suspend and PLL control to DWC2 instead of software overrides. */
	esp32s31_usb_update(priv->phy_ctrl,
			     ESP32S31_USB_PHY_PLL_FORCE_EN |
			     ESP32S31_USB_PHY_SUSPEND_FORCE_EN, 0);

	/* IDF releases the PHY before the controller AHB and APB resets. */
	esp32s31_usb_update(priv->otg_ctrl, 0, resets);
	esp32s31_usb_update(priv->otg_ctrl, ESP32S31_USB_PHY_RST_EN, 0);
	esp32s31_usb_update(priv->otg_ctrl,
			     ESP32S31_USB_AHB_RST_EN |
			     ESP32S31_USB_APB_RST_EN, 0);

	/* Keep the precise disconnect path active and use parallel LS mode. */
	esp32s31_usb_update(priv->phy_ctrl, 0,
			     ESP32S31_USB_PHY_OTG_SUSPENDM);
	esp32s31_usb_update(priv->utmi + ESP32S31_UTMI_FC06, 0,
			     ESP32S31_UTMI_LS_PAR_EN |
			     ESP32S31_UTMI_LS_KPALV_EN);

	esp32s31_usb_set_pulldowns(priv);
	priv->powered = true;

	return 0;
}

static int esp32s31_usb_phy_exit(struct phy *phy)
{
	struct esp32s31_usb_phy *priv = phy_get_drvdata(phy);
	u32 pulldowns = ESP32S31_USB_DM_PULLDOWN |
			ESP32S31_USB_DP_PULLDOWN;

	esp32s31_usb_update(priv->usb_ctrl, pulldowns, 0);
	esp32s31_usb_update(priv->otg_ctrl,
			     ESP32S31_USB_UTMIFS_CLK_EN |
			     ESP32S31_USB_PHYREF_CLK_EN, 0);
	esp32s31_usb_update(priv->clkrst,
			     ESP32S31_USB_CLK_APB_EN |
			     ESP32S31_USB_CLK_SYS_EN, 0);
	priv->powered = false;

	return 0;
}

static int esp32s31_usb_phy_set_mode(struct phy *phy, enum phy_mode mode,
				     int submode)
{
	struct esp32s31_usb_phy *priv = phy_get_drvdata(phy);

	(void)submode;

	if (mode != PHY_MODE_USB_HOST && mode != PHY_MODE_USB_DEVICE &&
	    mode != PHY_MODE_USB_OTG)
		return -EINVAL;

	priv->mode = mode;
	if (priv->powered)
		esp32s31_usb_set_pulldowns(priv);

	return 0;
}

static const struct phy_ops esp32s31_usb_phy_ops = {
	.init = esp32s31_usb_phy_init,
	.exit = esp32s31_usb_phy_exit,
	.set_mode = esp32s31_usb_phy_set_mode,
	.owner = THIS_MODULE,
};

static int esp32s31_usb_phy_probe(struct platform_device *pdev)
{
	struct phy_provider *provider;
	struct esp32s31_usb_phy *priv;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->utmi = devm_platform_ioremap_resource_byname(pdev, "utmi");
	if (IS_ERR(priv->utmi))
		return PTR_ERR(priv->utmi);

	priv->clkrst = devm_platform_ioremap_resource_byname(pdev, "clkrst");
	if (IS_ERR(priv->clkrst))
		return PTR_ERR(priv->clkrst);

	priv->otg_ctrl = devm_platform_ioremap_resource_byname(pdev,
							 "otg-ctrl");
	if (IS_ERR(priv->otg_ctrl))
		return PTR_ERR(priv->otg_ctrl);

	priv->usb_ctrl = devm_platform_ioremap_resource_byname(pdev,
							 "usb-ctrl");
	if (IS_ERR(priv->usb_ctrl))
		return PTR_ERR(priv->usb_ctrl);

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
				       esp32s31_usb_phy_pm_disable,
				       &pdev->dev);
	if (ret)
		return ret;

	priv->phy = devm_phy_create(&pdev->dev, NULL,
				    &esp32s31_usb_phy_ops);
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

static const struct of_device_id esp32s31_usb_phy_of_match[] = {
	{ .compatible = "espressif,esp32s31-usb-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s31_usb_phy_of_match);

static struct platform_driver esp32s31_usb_phy_driver = {
	.probe = esp32s31_usb_phy_probe,
	.driver = {
		.name = "esp32s31-usb-phy",
		.of_match_table = esp32s31_usb_phy_of_match,
	},
};
module_platform_driver(esp32s31_usb_phy_driver);

MODULE_DESCRIPTION("Espressif ESP32-S31 USB2 UTMI PHY driver");
MODULE_LICENSE("GPL");
