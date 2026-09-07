// SPDX-License-Identifier: GPL-2.0-or-later
/*******************************************************************************
 *
 * CTU CAN FD IP Core
 *
 * Copyright (C) 2015-2018 Ondrej Ille <ondrej.ille@gmail.com> FEE CTU
 * Copyright (C) 2018-2021 Ondrej Ille <ondrej.ille@gmail.com> self-funded
 * Copyright (C) 2018-2019 Martin Jerabek <martin.jerabek01@gmail.com> FEE CTU
 * Copyright (C) 2018-2022 Pavel Pisa <pisa@cmp.felk.cvut.cz> FEE CTU/self-funded
 *
 * Project advisors:
 *     Jiri Novak <jnovak@fel.cvut.cz>
 *     Pavel Pisa <pisa@cmp.felk.cvut.cz>
 *
 * Department of Measurement         (http://meas.fel.cvut.cz/)
 * Faculty of Electrical Engineering (http://www.fel.cvut.cz)
 * Czech Technical University        (http://www.cvut.cz/)
 ******************************************************************************/

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#include "ctucanfd.h"
#include "ctucanfd_kregs.h"

#define DRV_NAME	"ctucanfd"

static void ctucan_platform_set_drvdata(struct device *dev,
					struct net_device *ndev)
{
	struct platform_device *pdev = container_of(dev, struct platform_device,
						    dev);

	platform_set_drvdata(pdev, ndev);
}

static int ctucan_esp32s31_configure_filters(struct device *dev,
					     struct net_device *ndev)
{
	static const char * const properties[] = {
		"espressif,acceptance-filter-a",
		"espressif,acceptance-filter-b",
		"espressif,acceptance-filter-c",
	};
	static const enum ctu_can_fd_can_registers mask_regs[] = {
		CTUCANFD_FILTER_A_MASK, CTUCANFD_FILTER_B_MASK,
		CTUCANFD_FILTER_C_MASK,
	};
	static const enum ctu_can_fd_can_registers value_regs[] = {
		CTUCANFD_FILTER_A_VAL, CTUCANFD_FILTER_B_VAL,
		CTUCANFD_FILTER_C_VAL,
	};
	struct ctucan_priv *priv = netdev_priv(ndev);
	u32 values[3], control = 0;
	bool configured = false;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(properties); i++) {
		if (!device_property_present(dev, properties[i]))
			continue;
		if (device_property_read_u32_array(dev, properties[i], values, 3))
			return dev_err_probe(dev, -EINVAL,
					     "%s needs mask, value and frame flags\n",
					     properties[i]);
		if (values[0] & ~GENMASK(28, 0) ||
		    values[1] & ~GENMASK(28, 0) || values[2] & ~GENMASK(3, 0))
			return -ERANGE;
		priv->write_reg(priv, mask_regs[i], values[0]);
		priv->write_reg(priv, value_regs[i], values[1]);
		control |= (values[2] & 0xf) << (i * 4);
		control |= BIT(16 + i);
		configured = true;
	}
	if (device_property_present(dev, "espressif,acceptance-filter-range")) {
		if (device_property_read_u32_array(dev,
						   "espressif,acceptance-filter-range",
						   values, 3))
			return -EINVAL;
		if (values[0] & ~GENMASK(28, 0) ||
		    values[1] & ~GENMASK(28, 0) || values[2] & ~GENMASK(3, 0))
			return -ERANGE;
		priv->write_reg(priv, CTUCANFD_FILTER_RAN_LOW, values[0]);
		priv->write_reg(priv, CTUCANFD_FILTER_RAN_HIGH, values[1]);
		control |= (values[2] & 0xf) << 12;
		control |= BIT(19);
		configured = true;
	}
	if (configured)
		priv->write_reg(priv, CTUCANFD_FILTER_CONTROL, control);
	return 0;
}

/**
 * ctucan_platform_probe - Platform registration call
 * @pdev:	Handle to the platform device structure
 *
 * This function does all the memory allocation and registration for the CAN
 * device.
 *
 * Return: 0 on success and failure value on error
 */
static int ctucan_platform_probe(struct platform_device *pdev)
{
	struct device	*dev = &pdev->dev;
	void __iomem *addr;
	struct clk *clk;
	struct reset_control *rst;
	int ret;
	unsigned int ntxbufs;
	int irq;

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		goto err;
	}
	rst = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(rst)) {
		ret = PTR_ERR(rst);
		goto err;
	}
	ret = reset_control_reset(rst);
	if (ret)
		goto err;

	/* Get the virtual base address for the device */
	addr = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(addr)) {
		ret = PTR_ERR(addr);
		goto err;
	}
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = irq;
		goto err;
	}

	/* Number of tx bufs might be change in HW for future. If so,
	 * it will be passed as property via device tree
	 */
	ntxbufs = 4;
	ret = ctucan_probe_common(dev, addr, irq, ntxbufs, 0,
				  1, ctucan_platform_set_drvdata);

	if (ret < 0)
		platform_set_drvdata(pdev, NULL);
	else if (device_is_compatible(dev, "espressif,esp32s31-twaifd")) {
		struct net_device *ndev = platform_get_drvdata(pdev);

		ret = ctucan_esp32s31_configure_filters(dev, ndev);
		if (ret) {
			struct ctucan_priv *priv = netdev_priv(ndev);

			unregister_candev(ndev);
			pm_runtime_disable(dev);
			netif_napi_del(&priv->napi);
			free_candev(ndev);
			platform_set_drvdata(pdev, NULL);
		}
	}

err:
	return ret;
}

/**
 * ctucan_platform_remove - Unregister the device after releasing the resources
 * @pdev:	Handle to the platform device structure
 *
 * This function frees all the resources allocated to the device.
 * Return: 0 always
 */
static void ctucan_platform_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct ctucan_priv *priv = netdev_priv(ndev);

	netdev_dbg(ndev, "ctucan_remove");

	unregister_candev(ndev);
	pm_runtime_disable(&pdev->dev);
	netif_napi_del(&priv->napi);
	free_candev(ndev);
}

static SIMPLE_DEV_PM_OPS(ctucan_platform_pm_ops, ctucan_suspend, ctucan_resume);

/* Match table for OF platform binding */
static const struct of_device_id ctucan_of_match[] = {
	{ .compatible = "espressif,esp32s31-twaifd", },
	{ .compatible = "ctu,ctucanfd-2", },
	{ .compatible = "ctu,ctucanfd", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, ctucan_of_match);

static struct platform_driver ctucanfd_driver = {
	.probe	= ctucan_platform_probe,
	.remove = ctucan_platform_remove,
	.driver	= {
		.name = DRV_NAME,
		.pm = &ctucan_platform_pm_ops,
		.of_match_table	= ctucan_of_match,
	},
};

module_platform_driver(ctucanfd_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Martin Jerabek");
MODULE_DESCRIPTION("CTU CAN FD for platform");
