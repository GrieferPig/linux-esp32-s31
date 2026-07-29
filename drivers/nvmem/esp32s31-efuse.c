// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S31 eFuse read-shadow NVMEM provider
 *
 * The S31 hardware presents all ten eFuse blocks as one contiguous range of
 * little-endian 32-bit read-shadow registers. ESP-IDF describes MAC_FACTORY
 * in reverse byte order (BLK1 bits 40..0), so mac-base cells need a six-byte
 * reversal before the standard consumer-specific address offset is applied.
 */

#include <linux/etherdevice.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/of.h>
#include <linux/platform_device.h>

struct esp32s31_efuse {
	void __iomem *base;
};

static int esp32s31_efuse_read(void *context, unsigned int offset,
			       void *data, size_t bytes)
{
	struct esp32s31_efuse *efuse = context;
	u8 *buf = data;
	size_t i;

	/* S31 peripheral registers must be accessed as aligned 32-bit words. */
	for (i = 0; i < bytes; i++) {
		unsigned int byte = offset + i;
		u32 word = readl(efuse->base + (byte & ~3));

		buf[i] = word >> (8 * (byte & 3));
	}

	return 0;
}

static int esp32s31_efuse_mac_base_pp(void *priv, const char *id, int index,
				      unsigned int offset, void *data,
				      size_t bytes)
{
	u8 *mac = data;
	int i;

	if (bytes != ETH_ALEN || index < 0)
		return -EINVAL;

	for (i = 0; i < ETH_ALEN / 2; i++)
		swap(mac[i], mac[ETH_ALEN - 1 - i]);

	if (!is_valid_ether_addr(mac))
		return -EINVAL;

	eth_addr_add(mac, index);
	return 0;
}

static void esp32s31_efuse_fixup_cell(struct nvmem_device *nvmem,
				      struct nvmem_cell_info *cell)
{
	if (of_device_is_compatible(cell->np, "mac-base"))
		cell->read_post_process = esp32s31_efuse_mac_base_pp;
}

static int esp32s31_efuse_probe(struct platform_device *pdev)
{
	struct nvmem_config config = { };
	struct esp32s31_efuse *efuse;
	struct resource *res;

	efuse = devm_kzalloc(&pdev->dev, sizeof(*efuse), GFP_KERNEL);
	if (!efuse)
		return -ENOMEM;

	efuse->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(efuse->base))
		return PTR_ERR(efuse->base);

	config.dev = &pdev->dev;
	config.name = "esp32s31-efuse";
	config.id = NVMEM_DEVID_NONE;
	config.type = NVMEM_TYPE_OTP;
	config.read_only = true;
	config.word_size = 1;
	config.stride = 1;
	config.size = resource_size(res);
	config.reg_read = esp32s31_efuse_read;
	config.fixup_dt_cell_info = esp32s31_efuse_fixup_cell;
	config.priv = efuse;

	return PTR_ERR_OR_ZERO(devm_nvmem_register(&pdev->dev, &config));
}

static const struct of_device_id esp32s31_efuse_of_match[] = {
	{ .compatible = "espressif,esp32s31-efuse" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s31_efuse_of_match);

static struct platform_driver esp32s31_efuse_driver = {
	.probe = esp32s31_efuse_probe,
	.driver = {
		.name = "esp32s31-efuse",
		.of_match_table = esp32s31_efuse_of_match,
	},
};
module_platform_driver(esp32s31_efuse_driver);

MODULE_DESCRIPTION("Espressif ESP32-S31 eFuse NVMEM driver");
MODULE_LICENSE("GPL");
