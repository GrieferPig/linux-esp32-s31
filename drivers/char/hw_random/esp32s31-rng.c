// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S31 true random number generator
 *
 * Keep the enable sequence and sampling cadence aligned with ESP-IDF's
 * esp32s31 rng_ll implementation.  The hardware has no data-ready flag;
 * IDF limits reads so the entropy source is not drained faster than it is
 * replenished.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/hw_random.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define ESP32S31_TRNG_CONF		0x00
#define ESP32S31_TRNG_DATA		0x48
#define ESP32S31_TRNG_DATE		0xfc

#define ESP32S31_TRNG_NOISE_CRC_EN	BIT(30)
#define ESP32S31_TRNG_SAMPLE_ENABLE	BIT(31)
#define ESP32S31_TRNG_CLK_EN		BIT(28)

#define ESP32S31_RNG_BUS_CLK_EN		BIT(30)
#define ESP32S31_RNG_BUS_RST_EN		BIT(31)

struct esp32s31_rng {
	void __iomem *base;
	void __iomem *clkrst;
	struct hwrng rng;
};

static void esp32s31_rng_enable(struct esp32s31_rng *priv)
{
	u32 val;

	val = readl(priv->clkrst);
	writel(val | ESP32S31_RNG_BUS_CLK_EN | ESP32S31_RNG_BUS_RST_EN,
	       priv->clkrst);
	writel((val | ESP32S31_RNG_BUS_CLK_EN) & ~ESP32S31_RNG_BUS_RST_EN,
	       priv->clkrst);

	val = readl(priv->base + ESP32S31_TRNG_DATE);
	writel(val | ESP32S31_TRNG_CLK_EN,
	       priv->base + ESP32S31_TRNG_DATE);

	val = readl(priv->base + ESP32S31_TRNG_CONF);
	writel(val | ESP32S31_TRNG_SAMPLE_ENABLE |
	       ESP32S31_TRNG_NOISE_CRC_EN,
	       priv->base + ESP32S31_TRNG_CONF);
}

static void esp32s31_rng_disable(void *data)
{
	struct esp32s31_rng *priv = data;
	u32 val;

	val = readl(priv->base + ESP32S31_TRNG_CONF);
	writel(val & ~(ESP32S31_TRNG_SAMPLE_ENABLE |
		       ESP32S31_TRNG_NOISE_CRC_EN),
	       priv->base + ESP32S31_TRNG_CONF);

	val = readl(priv->base + ESP32S31_TRNG_DATE);
	writel(val & ~ESP32S31_TRNG_CLK_EN,
	       priv->base + ESP32S31_TRNG_DATE);

	val = readl(priv->clkrst);
	writel(val & ~ESP32S31_RNG_BUS_CLK_EN, priv->clkrst);
}

static int esp32s31_rng_read(struct hwrng *rng, void *data, size_t max,
			     bool wait)
{
	struct esp32s31_rng *priv =
		container_of(rng, struct esp32s31_rng, rng);
	u8 *buf = data;
	size_t done = 0;

	/*
	 * IDF waits at least 16 80 MHz APB cycles for each output byte.
	 * ndelay(200) expresses the same lower bound without depending on the
	 * current CPU clock.
	 */
	while (done < max) {
		u32 word = readl(priv->base + ESP32S31_TRNG_DATA);
		unsigned int i;

		for (i = 0; i < sizeof(word) && done < max; i++) {
			buf[done++] = word >> (i * 8);
			ndelay(200);
			word ^= readl(priv->base + ESP32S31_TRNG_DATA);
		}
	}

	return done;
}

static int esp32s31_rng_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp32s31_rng *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = devm_platform_ioremap_resource_byname(pdev, "trng");
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->clkrst = devm_platform_ioremap_resource_byname(pdev, "clkrst");
	if (IS_ERR(priv->clkrst))
		return PTR_ERR(priv->clkrst);

	esp32s31_rng_enable(priv);
	ret = devm_add_action_or_reset(dev, esp32s31_rng_disable, priv);
	if (ret)
		return ret;

	priv->rng.name = "esp32s31-trng";
	priv->rng.read = esp32s31_rng_read;
	priv->rng.quality = 900;

	return devm_hwrng_register(dev, &priv->rng);
}

static const struct of_device_id esp32s31_rng_of_match[] = {
	{ .compatible = "espressif,esp32s31-trng" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s31_rng_of_match);

static struct platform_driver esp32s31_rng_driver = {
	.probe = esp32s31_rng_probe,
	.driver = {
		.name = "esp32s31-rng",
		.of_match_table = esp32s31_rng_of_match,
	},
};
module_platform_driver(esp32s31_rng_driver);

MODULE_DESCRIPTION("Espressif ESP32-S31 true random number generator");
MODULE_LICENSE("GPL");
