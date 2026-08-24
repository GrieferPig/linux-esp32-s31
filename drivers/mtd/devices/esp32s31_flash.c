// SPDX-License-Identifier: GPL-2.0-only
/*
 * MTD access to the ESP32-S31 bootloader-provided Flash MMU window.
 *
 * Linux exposes this XIP window read-only.  The ROM write/erase helpers below
 * are retained for bring-up, but are deliberately not registered as MTD
 * operations: SPI1 auto-suspend alone does not coordinate a dual-hart Linux
 * XIP workload with ROM flash commands.
 */

#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <asm/sbi.h>

#include <linux/soc/espressif/esp32s31-cache.h>

#define ESP32S31_FLASH_WRITE_SIZE	32
#define ESP32S31_SBI_EXT_FLASH		0x09000000
#define ESP32S31_SBI_FLASH_WRITE	0
#define ESP32S31_SBI_FLASH_ERASE	1
#define ESP32S31_FLASH_XIP_BASE		0x40000000
#define ESP32S31_FLASH_XIP_SIZE		0x00f00000
#define ESP32S31_FLASH_RAW_OFFSET	0x00100000

struct esp32s31_flash {
	void __iomem *base;
	phys_addr_t phys_base;
	struct mtd_info mtd;
	u32 raw_offset;
	struct mutex lock;
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

static int esp32s31_flash_rom_result(const char *operation, struct sbiret ret)
{
	int result = ret.error ? ret.value ?: 1 : ret.value;

	if (!result)
		return 0;
	pr_err("esp32s31-flash: ROM %s failed: SBI error %ld, result %#x\n",
	       operation, ret.error, result);
	return result == 2 ? -ETIMEDOUT : -EIO;
}

static int esp32s31_flash_program(u32 address, const u32 *buffer, u32 length)
{
	struct sbiret ret = sbi_ecall(ESP32S31_SBI_EXT_FLASH,
		ESP32S31_SBI_FLASH_WRITE, address, virt_to_phys((void *)buffer), length,
		0, 0, 0);

	return esp32s31_flash_rom_result("write", ret);
}

static int esp32s31_flash_erase_rom(u32 address, u32 length)
{
	struct sbiret ret = sbi_ecall(ESP32S31_SBI_EXT_FLASH,
		ESP32S31_SBI_FLASH_ERASE, address, 0, length, 0, 0, 0);

	return esp32s31_flash_rom_result("erase", ret);
}

static int esp32s31_flash_write(struct mtd_info *mtd, loff_t to, size_t len,
				 size_t *retlen, const u_char *buf)
{
	struct esp32s31_flash *flash = container_of(mtd, struct esp32s31_flash, mtd);
	u8 *write_buf;
	int ret = 0;

	if (to < 0 || to >= mtd->size || len > mtd->size - to)
		return -EINVAL;

	write_buf = kmalloc(ESP32S31_FLASH_WRITE_SIZE + 3, GFP_KERNEL);
	if (!write_buf)
		return -ENOMEM;

	mutex_lock(&flash->lock);
	while (len) {
		u32 aligned_to = round_down((u32)to, 4);
		u32 head = (u32)to - aligned_to;
		u32 bytes = min_t(size_t, len, ESP32S31_FLASH_WRITE_SIZE - head);
		u32 write_len = round_up(head + bytes, 4);

		memcpy_fromio(write_buf, flash->base + aligned_to, write_len);
		memcpy(write_buf + head, buf, bytes);
		ret = esp32s31_flash_program(flash->raw_offset + aligned_to,
					     (const u32 *)write_buf,
					     write_len);
		if (ret)
			break;
		/* phys_base is the CPU-visible identity base, not a flash offset. */
		esp32s31_cache_invalidate(flash->phys_base + aligned_to,
					 write_len);
		to += bytes;
		buf += bytes;
		len -= bytes;
		*retlen += bytes;
	}
	mutex_unlock(&flash->lock);
	kfree(write_buf);
	return ret;
}

static int esp32s31_flash_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct esp32s31_flash *flash = container_of(mtd, struct esp32s31_flash, mtd);
	u64 offset = instr->addr, len = instr->len;
	int ret = 0;

	if (!len || offset & (mtd->erasesize - 1) || len & (mtd->erasesize - 1))
		return -EINVAL;

	mutex_lock(&flash->lock);
	while (len) {
		ret = esp32s31_flash_erase_rom(flash->raw_offset + offset,
					       mtd->erasesize);
		if (ret) {
			instr->fail_addr = offset;
			break;
		}
		esp32s31_cache_invalidate(flash->phys_base + offset,
					 mtd->erasesize);
		offset += mtd->erasesize;
		len -= mtd->erasesize;
		cond_resched();
	}
	mutex_unlock(&flash->lock);
	return ret;
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
	if (!res)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "missing Flash MMU resource\n");
	if (res->start != ESP32S31_FLASH_XIP_BASE ||
	    resource_size(res) != ESP32S31_FLASH_XIP_SIZE)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "Flash MMU resource must be 0x%08x..0x%08x\n",
				     ESP32S31_FLASH_XIP_BASE,
				     ESP32S31_FLASH_XIP_BASE + ESP32S31_FLASH_XIP_SIZE);
	flash->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(flash->base))
		return PTR_ERR(flash->base);
	flash->phys_base = res->start;
	flash->raw_offset = ESP32S31_FLASH_RAW_OFFSET;

	flash->mtd.type = MTD_NORFLASH;
	/* The kernel and OpenSBI execute in place from this same NOR.  Keep the
	 * runtime MTD read-only, matching the official port's mtd-rom device:
	 * a ROM program/erase operation can make either hart fault on its next
	 * XIP instruction fetch even when flash auto-suspend is configured. */
	flash->mtd.flags = MTD_CAP_ROM;
	flash->mtd.size = resource_size(res);
	/* JFFS2 requires an 8 KiB minimum erase sector on this NOR. */
	flash->mtd.erasesize = SZ_8K;
	flash->mtd.writesize = 1;
	flash->mtd.writebufsize = 1;
	flash->mtd._read = esp32s31_flash_read;
	flash->mtd._write = esp32s31_flash_write;
	flash->mtd._erase = esp32s31_flash_erase;
	flash->mtd.owner = THIS_MODULE;
	flash->mtd.dev.parent = &pdev->dev;
	flash->mtd.name = dev_name(&pdev->dev);
	mutex_init(&flash->lock);
	mtd_set_of_node(&flash->mtd, pdev->dev.of_node);

	ret = mtd_device_parse_register(&flash->mtd, NULL, NULL, NULL, 0);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to register MTD device\n");

	platform_set_drvdata(pdev, flash);
	dev_info(&pdev->dev, "registered %llu KiB read-only XIP Flash MTD window\n",
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
