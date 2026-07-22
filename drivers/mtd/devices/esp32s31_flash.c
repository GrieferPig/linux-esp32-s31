// SPDX-License-Identifier: GPL-2.0-only
/*
 * Read-only MTD access to the ESP32-S31 bootloader-provided Flash MMU alias.
 *
 * The S31 MSPI controller is owned by the ROM/bootloader while Linux is
 * executing from Flash XIP. The bootloader maps the whole chip through a
 * non-overlapping Flash-MMU alias and this driver presents that aperture as
 * an MTD NOR device. Write and erase support must not be added until a Linux
 * MSPI controller driver can safely suspend Flash XIP.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define ESP_PARTITION_TABLE_OFFSET	0x8000
#define ESP_PARTITION_TABLE_SIZE	0xc00
#define ESP_PARTITION_ENTRY_SIZE	32
#define ESP_PARTITION_MAGIC		0x50aa
#define ESP_PARTITION_MAGIC_MD5		0xebeb
#define ESP_PARTITION_MAGIC_END		0xffff

struct esp_partition_entry {
	__le16 magic;
	u8 type;
	u8 subtype;
	__le32 offset;
	__le32 size;
	char label[16];
	__le32 flags;
} __packed;

struct esp32s31_flash {
	void __iomem *base;
	struct mtd_info mtd;
	struct mtd_partition *parts;
	unsigned int nr_parts;
};

static int esp32s31_flash_read(struct mtd_info *mtd, loff_t from,
				       size_t len, size_t *retlen, u_char *buf)
{
	struct esp32s31_flash *flash = container_of(mtd, struct esp32s31_flash, mtd);

	if (from < 0 || from >= mtd->size || len > mtd->size - from)
		return -EINVAL;

	memcpy_fromio(buf, flash->base + from, len);
	*retlen = len;
	return 0;
}

static int esp32s31_flash_parse_partitions(struct device *dev,
					    struct esp32s31_flash *flash)
{
	struct mtd_partition *parts;
	char (*part_names)[sizeof_field(struct esp_partition_entry, label) + 1];
	const unsigned int max_parts = ESP_PARTITION_TABLE_SIZE /
		ESP_PARTITION_ENTRY_SIZE;
	unsigned int i, nr_parts = 0;

	if (ESP_PARTITION_TABLE_OFFSET + ESP_PARTITION_TABLE_SIZE > flash->mtd.size)
		return dev_err_probe(dev, -EINVAL, "Flash is too small for partition table\n");

	parts = devm_kcalloc(dev, max_parts, sizeof(*parts), GFP_KERNEL);
	if (!parts)
		return -ENOMEM;
	part_names = devm_kcalloc(dev, max_parts, sizeof(*part_names), GFP_KERNEL);
	if (!part_names)
		return -ENOMEM;

	for (i = 0; i < max_parts; i++) {
		struct esp_partition_entry entry;
		u16 magic;
		u32 offset, size;

		memcpy_fromio(&entry, flash->base + ESP_PARTITION_TABLE_OFFSET +
			      i * ESP_PARTITION_ENTRY_SIZE, sizeof(entry));
		magic = le16_to_cpu(entry.magic);

		if (magic == ESP_PARTITION_MAGIC_END)
			break;
		if (magic == ESP_PARTITION_MAGIC_MD5)
			continue;
		if (magic != ESP_PARTITION_MAGIC)
			return dev_err_probe(dev, -EINVAL,
					     "invalid partition table entry %u (magic %#x)\n",
					     i, magic);

		offset = le32_to_cpu(entry.offset);
		size = le32_to_cpu(entry.size);
		if (!size || offset >= flash->mtd.size || size > flash->mtd.size - offset)
			return dev_err_probe(dev, -EINVAL,
					     "partition %u is outside Flash\n", i);

		memcpy(part_names[nr_parts], entry.label, sizeof(entry.label));
		if (!part_names[nr_parts][0])
			scnprintf(part_names[nr_parts], sizeof(*part_names),
				  "part-%02x-%02x", entry.type, entry.subtype);

		parts[nr_parts].name = part_names[nr_parts];
		parts[nr_parts].offset = offset;
		parts[nr_parts].size = size;
		parts[nr_parts].mask_flags = MTD_WRITEABLE | MTD_BIT_WRITEABLE;
		nr_parts++;
	}

	if (!nr_parts)
		return dev_err_probe(dev, -EINVAL, "no ESP-IDF partitions found\n");

	flash->parts = parts;
	flash->nr_parts = nr_parts;
	dev_info(dev, "parsed %u ESP-IDF partition table entries\n", nr_parts);
	return 0;
}

static int esp32s31_flash_probe(struct platform_device *pdev)
{
	struct esp32s31_flash *flash;
	struct resource *res;
	int ret;

	flash = devm_kzalloc(&pdev->dev, sizeof(*flash), GFP_KERNEL);
	if (!flash)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	flash->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(flash->base))
		return PTR_ERR(flash->base);

	flash->mtd.type = MTD_NORFLASH;
	/*
	 * The bootloader-owned XIP aperture can only be read while Linux is
	 * running.  Mark it as non-erasable too, otherwise mtdcore expects an
	 * erase callback even for this explicitly read-only device.
	 */
	flash->mtd.flags = (MTD_CAP_NORFLASH &
			    ~(MTD_WRITEABLE | MTD_BIT_WRITEABLE)) | MTD_NO_ERASE;
	flash->mtd.size = resource_size(res);
	flash->mtd.erasesize = SZ_4K;
	flash->mtd.writesize = 1;
	flash->mtd.writebufsize = 1;
	flash->mtd._read = esp32s31_flash_read;
	flash->mtd.owner = THIS_MODULE;
	flash->mtd.dev.parent = &pdev->dev;
	flash->mtd.name = dev_name(&pdev->dev);
	mtd_set_of_node(&flash->mtd, pdev->dev.of_node);

	ret = esp32s31_flash_parse_partitions(&pdev->dev, flash);
	if (ret)
		return ret;

	ret = mtd_device_register(&flash->mtd, flash->parts, flash->nr_parts);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to register MTD device\n");

	platform_set_drvdata(pdev, flash);
	dev_info(&pdev->dev, "registered %llu KiB read-only Flash MTD window\n",
		 (unsigned long long)(flash->mtd.size / SZ_1K));
	return 0;
}

static void esp32s31_flash_remove(struct platform_device *pdev)
{
	struct esp32s31_flash *flash = platform_get_drvdata(pdev);

	mtd_device_unregister(&flash->mtd);
}

static const struct of_device_id esp32s31_flash_of_match[] = {
	{ .compatible = "espressif,esp32s31-flash-mtd" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s31_flash_of_match);

static struct platform_driver esp32s31_flash_driver = {
	.probe = esp32s31_flash_probe,
	.remove = esp32s31_flash_remove,
	.driver = {
		.name = "esp32s31-flash-mtd",
		.of_match_table = esp32s31_flash_of_match,
	},
};
module_platform_driver(esp32s31_flash_driver);

MODULE_DESCRIPTION("ESP32-S31 bootloader-mapped Flash MTD driver");
MODULE_LICENSE("GPL");
