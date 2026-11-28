// SPDX-License-Identifier: GPL-2.0-only

#include <linux/esp32s31-radio.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <uapi/linux/esp32s31-ds.h>

static long s31_ds_ioctl(struct file *file, unsigned int command, unsigned long arg)
{
	struct esp32s31_ds_sign __user *out = (void __user *)arg;
	struct s31_radio_ds_payload *payload;
	struct esp32s31_ds_sign *request;
	u32 words, bytes;
	int ret = -EINVAL;

	if (command != ESP32S31_DS_IOC_SIGN)
		return -ENOTTY;

	request = memdup_user(out, offsetof(struct esp32s31_ds_sign, signature));
	if (IS_ERR(request))
		return PTR_ERR(request);

	words = le32_to_cpu(request->key.rsa_length);
	if (request->reserved || request->slot >= S31_RADIO_DS_SLOTS)
		goto free_request;
	if (words != 31 && words != 63 && words != 95 && words != 127)
		goto free_request;
	bytes = (words + 1) * 4;

	payload = kzalloc(sizeof(*payload), GFP_KERNEL);
	if (!payload) {
		ret = -ENOMEM;
		goto free_request;
	}
	static_assert(sizeof(payload->key) == sizeof(request->key));
	memcpy(payload->key, &request->key, sizeof(payload->key));
	memcpy(payload->message, request->message, bytes);

	/* The IDF owner coordinates HMAC, RSA, SHA and AES for DS. */
	ret = esp32s31_radio_ds_sign(payload, request->slot, bytes);
	if (!ret && copy_to_user(out->signature, payload->signature,
				 sizeof(payload->signature)))
		ret = -EFAULT;
	kfree_sensitive(payload);
free_request:
	kfree_sensitive(request);
	return ret;
}

static const struct file_operations s31_ds_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = s31_ds_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

static int s31_ds_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *radio_node __free(device_node) =
		of_parse_phandle(dev->of_node, "espressif,radio", 0);
	struct s31_wifi_control cap = { .operation = S31_RADIO_DS_CAP };
	struct platform_device *radio;
	struct device_link *link;
	struct miscdevice *misc;
	int ret;

	if (!radio_node)
		return -EINVAL;
	radio = of_find_device_by_node(radio_node);
	if (!radio)
		return -EPROBE_DEFER;
	link = device_link_add(dev, &radio->dev, DL_FLAG_AUTOREMOVE_CONSUMER);
	put_device(&radio->dev);
	if (!link)
		return -EINVAL;

	ret = esp32s31_radio_security_control(&cap);
	if (ret)
		return dev_err_probe(dev, ret == -ENODEV ? -EPROBE_DEFER : ret,
				     "S31 DS service unavailable\n");
	if (cap.field != S31_RADIO_DS_ABI)
		return -EPROTONOSUPPORT;

	misc = devm_kzalloc(dev, sizeof(*misc), GFP_KERNEL);
	if (!misc)
		return -ENOMEM;
	misc->minor = MISC_DYNAMIC_MINOR;
	misc->name = "esp32s31-ds";
	misc->fops = &s31_ds_fops;
	misc->parent = dev;
	misc->mode = 0600;
	platform_set_drvdata(pdev, misc);
	return misc_register(misc);
}

static void s31_ds_remove(struct platform_device *pdev)
{
	misc_deregister(platform_get_drvdata(pdev));
}

static const struct of_device_id s31_ds_of_match[] = {
	{ .compatible = "espressif,esp32s31-ds" },
	{ }
};
MODULE_DEVICE_TABLE(of, s31_ds_of_match);

static struct platform_driver s31_ds_driver = {
	.probe = s31_ds_probe,
	.remove = s31_ds_remove,
	.driver = {
		.name = "esp32s31-ds",
		.of_match_table = s31_ds_of_match,
	},
};
module_platform_driver(s31_ds_driver);

MODULE_DESCRIPTION("ESP32-S31 protected RSA digital signature peripheral");
MODULE_LICENSE("GPL");
