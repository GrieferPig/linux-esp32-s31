// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/esp32s31-etm.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/uaccess.h>

/*
 * S31 TRM ch.5 maps SOC_ETM and GPIO_EXT. The ETM register chapter is
 * pending. Offsets and IDs follow IDF soc_etm_reg.h, soc_etm_source.h
 * and gpio_etm_ll_gpio_set_task_channel().
 */
#define ETM_CHANNELS		50
#define ETM_GPIO_SLOTS		8
#define ETM_ENABLE(ch)		((ch) < 32 ? 0x04 : 0x10)
#define ETM_DISABLE(ch)		((ch) < 32 ? 0x08 : 0x14)
#define ETM_EVENT(ch)		(0x18 + 8 * (ch))
#define ETM_TASK(ch)		(0x1c + 8 * (ch))
#define GPIO_TASK_REG(pin)	(4 * ((pin) / 5))
#define GPIO_TASK_SHIFT(pin)	(6 * ((pin) % 5))
#define GPIO_TASK_ENABLE	BIT(5)
#define GPIO_TASK_FIELD		(GENMASK(2, 0) | GPIO_TASK_ENABLE)

struct s31_etm {
	void __iomem *base;
	void __iomem *gpio_tasks;
	struct gpio_desc *outputs[ETM_GPIO_SLOTS];
	unsigned int pins[ETM_GPIO_SLOTS];
	unsigned int num_outputs;
	const void *owners[ETM_CHANNELS];
	struct mutex lock;
	struct miscdevice misc;
	struct kref refs;
	bool removed;
};

static int s31_etm_bind(struct s31_etm *etm, const struct esp32s31_etm_route *route,
			const void *owner)
{
	unsigned int ch = route->channel;

	if (etm->removed)
		return -ENODEV;
	if (!owner || ch >= ETM_CHANNELS ||
	    route->event < ESP32S31_ETM_TG0_T0_COMPARE ||
	    route->event > ESP32S31_ETM_TG1_T1_COMPARE ||
	    !route->task || route->task > ESP32S31_ETM_GPIO_TOGGLE(7) ||
	    (route->task - 1) % ETM_GPIO_SLOTS >= etm->num_outputs)
		return -EINVAL;
	if (etm->owners[ch])
		return -EBUSY;

	/* IDF etm_ll_channel_set_event/task() precede enabling the channel. */
	writel(route->event, etm->base + ETM_EVENT(ch));
	writel(route->task, etm->base + ETM_TASK(ch));
	writel(BIT(ch % 32), etm->base + ETM_ENABLE(ch));
	etm->owners[ch] = owner;
	return 0;
}

static int s31_etm_unbind(struct s31_etm *etm, unsigned int ch, const void *owner)
{
	if (etm->removed)
		return -ENODEV;
	if (!owner || ch >= ETM_CHANNELS)
		return -EINVAL;
	if (etm->owners[ch] != owner)
		return -ENOENT;

	writel(BIT(ch % 32), etm->base + ETM_DISABLE(ch));
	writel(0, etm->base + ETM_EVENT(ch));
	writel(0, etm->base + ETM_TASK(ch));
	etm->owners[ch] = NULL;
	return 0;
}

static struct platform_driver s31_etm_driver;

int esp32s31_etm_bind(struct device *dev, const struct esp32s31_etm_route *route,
		      const void *owner)
{
	struct s31_etm *etm;
	int ret = -ENODEV;

	if (!dev || !route)
		return -EINVAL;
	device_lock(dev);
	if (dev->driver == &s31_etm_driver.driver) {
		etm = dev_get_drvdata(dev);
		mutex_lock(&etm->lock);
		ret = s31_etm_bind(etm, route, owner);
		mutex_unlock(&etm->lock);
	}
	device_unlock(dev);
	return ret;
}
EXPORT_SYMBOL_GPL(esp32s31_etm_bind);

int esp32s31_etm_unbind(struct device *dev, unsigned int ch, const void *owner)
{
	struct s31_etm *etm;
	int ret = -ENODEV;

	if (!dev)
		return -EINVAL;
	device_lock(dev);
	if (dev->driver == &s31_etm_driver.driver) {
		etm = dev_get_drvdata(dev);
		mutex_lock(&etm->lock);
		ret = s31_etm_unbind(etm, ch, owner);
		mutex_unlock(&etm->lock);
	}
	device_unlock(dev);
	return ret;
}
EXPORT_SYMBOL_GPL(esp32s31_etm_unbind);

static void s31_etm_free(struct kref *refs)
{
	kfree(container_of(refs, struct s31_etm, refs));
}

static void s31_etm_put(void *data)
{
	struct s31_etm *etm = data;

	kref_put(&etm->refs, s31_etm_free);
}

static int s31_etm_open(struct inode *inode, struct file *file)
{
	struct s31_etm *etm = container_of(file->private_data, struct s31_etm, misc);

	kref_get(&etm->refs);
	file->private_data = etm;
	return nonseekable_open(inode, file);
}

static int s31_etm_release(struct inode *inode, struct file *file)
{
	struct s31_etm *etm = file->private_data;
	unsigned int ch;

	mutex_lock(&etm->lock);
	if (!etm->removed)
		for (ch = 0; ch < ETM_CHANNELS; ch++)
			if (etm->owners[ch] == file)
				s31_etm_unbind(etm, ch, file);
	mutex_unlock(&etm->lock);
	s31_etm_put(etm);
	return 0;
}

static long s31_etm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct s31_etm *etm = file->private_data;
	void __user *user = (void __user *)arg;
	struct esp32s31_etm_route route;
	struct esp32s31_etm_level level;
	u32 ch;
	int ret;

	switch (cmd) {
	case ESP32S31_ETM_IOC_BIND:
		if (copy_from_user(&route, user, sizeof(route)))
			return -EFAULT;
		mutex_lock(&etm->lock);
		ret = s31_etm_bind(etm, &route, file);
		mutex_unlock(&etm->lock);
		return ret;
	case ESP32S31_ETM_IOC_UNBIND:
		if (get_user(ch, (u32 __user *)user))
			return -EFAULT;
		mutex_lock(&etm->lock);
		ret = s31_etm_unbind(etm, ch, file);
		mutex_unlock(&etm->lock);
		return ret;
	case ESP32S31_ETM_IOC_LEVEL:
		if (copy_from_user(&level, user, sizeof(level)))
			return -EFAULT;
		mutex_lock(&etm->lock);
		if (etm->removed)
			ret = -ENODEV;
		else if (level.slot >= etm->num_outputs)
			ret = -EINVAL;
		else
			ret = gpiod_get_raw_value_cansleep(etm->outputs[level.slot]);
		mutex_unlock(&etm->lock);
		if (ret < 0)
			return ret;
		level.value = ret;
		return copy_to_user(user, &level, sizeof(level)) ? -EFAULT : 0;
	default:
		return -ENOTTY;
	}
}

static const struct file_operations s31_etm_fops = {
	.owner = THIS_MODULE,
	.open = s31_etm_open,
	.release = s31_etm_release,
	.unlocked_ioctl = s31_etm_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

static void s31_etm_set_gpio(struct s31_etm *etm, unsigned int pin, u32 config)
{
	void __iomem *reg = etm->gpio_tasks + GPIO_TASK_REG(pin);
	unsigned int shift = GPIO_TASK_SHIFT(pin);

	writel((readl(reg) & ~(GPIO_TASK_FIELD << shift)) | (config << shift), reg);
}

static void s31_etm_disable(void *data)
{
	struct s31_etm *etm = data;
	unsigned int n;

	writel(U32_MAX, etm->base + ETM_DISABLE(0));
	writel(GENMASK(ETM_CHANNELS - 33, 0), etm->base + ETM_DISABLE(32));
	for (n = 0; n < etm->num_outputs; n++)
		s31_etm_set_gpio(etm, etm->pins[n], 0);
}

static int s31_etm_get_gpio(struct device *dev, struct s31_etm *etm, unsigned int n)
{
	struct of_phandle_args args;
	struct gpio_desc *gpio;
	unsigned int pin;
	bool valid;
	int ret;

	ret = of_parse_phandle_with_args(dev->of_node, "task-gpios", "#gpio-cells",
					 n, &args);
	if (ret)
		return ret;
	valid = args.args_count == 2 && args.args[0] < 64 &&
		of_device_is_compatible(args.np, "espressif,esp32s31-pinctrl");
	pin = args.args[0];
	of_node_put(args.np);
	if (!valid)
		return -EINVAL;

	gpio = devm_gpiod_get_index(dev, "task", n, GPIOD_OUT_LOW);
	if (IS_ERR(gpio))
		return PTR_ERR(gpio);
	ret = gpiod_set_config(gpio, PIN_CONF_PACKED(PIN_CONFIG_INPUT_ENABLE, 1));
	if (ret)
		return ret;

	etm->pins[n] = pin;
	etm->outputs[n] = gpio;
	etm->num_outputs++;
	s31_etm_set_gpio(etm, pin, GPIO_TASK_ENABLE | n);
	return 0;
}

static int s31_etm_static_routes(struct device *dev, struct s31_etm *etm)
{
	struct esp32s31_etm_route route;
	u32 words[3 * ETM_CHANNELS];
	int entries, i, ret;

	if (!of_property_present(dev->of_node, "espressif,routes"))
		return 0;
	entries = of_property_count_u32_elems(dev->of_node, "espressif,routes");
	if (entries <= 0 || entries % 3 || entries > 3 * ETM_CHANNELS)
		return -EINVAL;

	ret = of_property_read_u32_array(dev->of_node, "espressif,routes", words, entries);
	if (ret)
		return ret;
	for (i = 0; i < entries; i += 3) {
		route.channel = words[i];
		route.event = words[i + 1];
		route.task = words[i + 2];
		ret = s31_etm_bind(etm, &route, etm);
		if (ret)
			return ret;
	}
	return 0;
}

static int s31_etm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *rst;
	struct s31_etm *etm;
	struct clk *clk;
	int n, ret;

	etm = kzalloc(sizeof(*etm), GFP_KERNEL);
	if (!etm)
		return -ENOMEM;
	kref_init(&etm->refs);
	ret = devm_add_action_or_reset(dev, s31_etm_put, etm);
	if (ret)
		return ret;
	mutex_init(&etm->lock);

	etm->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(etm->base))
		return PTR_ERR(etm->base);
	etm->gpio_tasks = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(etm->gpio_tasks))
		return PTR_ERR(etm->gpio_tasks);
	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "clock unavailable\n");
	rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "reset unavailable\n");
	ret = reset_control_reset(rst);
	if (ret)
		return ret;

	n = gpiod_count(dev, "task");
	if (n <= 0 || n > ETM_GPIO_SLOTS)
		return dev_err_probe(dev, n < 0 ? n : -EINVAL, "invalid task GPIOs\n");
	while (etm->num_outputs < n) {
		ret = s31_etm_get_gpio(dev, etm, etm->num_outputs);
		if (ret) {
			s31_etm_disable(etm);
			return dev_err_probe(dev, ret, "task GPIO unavailable\n");
		}
	}
	ret = devm_add_action_or_reset(dev, s31_etm_disable, etm);
	if (ret)
		return ret;
	ret = s31_etm_static_routes(dev, etm);
	if (ret)
		return dev_err_probe(dev, ret, "invalid static ETM route\n");

	etm->misc.minor = MISC_DYNAMIC_MINOR;
	etm->misc.name = "esp32s31-etm";
	etm->misc.fops = &s31_etm_fops;
	etm->misc.parent = dev;
	etm->misc.mode = 0600;
	platform_set_drvdata(pdev, etm);
	return misc_register(&etm->misc);
}

static void s31_etm_remove(struct platform_device *pdev)
{
	struct s31_etm *etm = platform_get_drvdata(pdev);

	misc_deregister(&etm->misc);
	mutex_lock(&etm->lock);
	etm->removed = true;
	mutex_unlock(&etm->lock);
}

static const struct of_device_id s31_etm_of_match[] = {
	{ .compatible = "espressif,esp32s31-etm" },
	{ }
};
MODULE_DEVICE_TABLE(of, s31_etm_of_match);

static struct platform_driver s31_etm_driver = {
	.probe = s31_etm_probe,
	.remove = s31_etm_remove,
	.driver = {
		.name = "esp32s31-etm",
		.of_match_table = s31_etm_of_match,
	},
};
module_platform_driver(s31_etm_driver);

MODULE_DESCRIPTION("Espressif ESP32-S31 Event Task Matrix");
MODULE_LICENSE("GPL");
