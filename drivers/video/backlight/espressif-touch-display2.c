// SPDX-License-Identifier: GPL-2.0-only
/* Brightness register of the MCU already initialized by the retained loader. */
#include <linux/backlight.h>
#include <linux/i2c.h>
#include <linux/module.h>

static int p4td2_update(struct backlight_device *bl)
{
	struct i2c_client *client = bl_get_data(bl);
	u8 data[] = { 3, backlight_get_brightness(bl) };
	int ret = i2c_master_send(client, data, sizeof(data));

	return ret < 0 ? ret : ret == sizeof(data) ? 0 : -EIO;
}
static const struct backlight_ops p4td2_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = p4td2_update,
};
static int p4td2_probe(struct i2c_client *client)
{
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.max_brightness = 255,
		.brightness = 255,
	};
	struct backlight_device *bl;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;
	bl = devm_backlight_device_register(&client->dev, "rpi-touch-display-2",
					   &client->dev, client, &p4td2_ops, &props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);
	ret = backlight_update_status(bl);
	if (ret)
		return ret;
	dev_info(&client->dev, "retained panel backlight at full brightness\n");
	return 0;
}
static const struct of_device_id p4td2_match[] = {
	{ .compatible = "raspberrypi,touch-display-2-backlight" },
	{ }
};
MODULE_DEVICE_TABLE(of, p4td2_match);
static struct i2c_driver p4td2_driver = {
	.driver = {
		.name = "p4-touch-display-2-backlight",
		.of_match_table = p4td2_match,
	},
	.probe = p4td2_probe,
};
module_i2c_driver(p4td2_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Retained Raspberry Pi Touch Display 2 MCU brightness");
