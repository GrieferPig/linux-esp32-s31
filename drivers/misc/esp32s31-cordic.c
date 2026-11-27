// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 CORDIC accelerator in register mode. */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/compat.h>
#include <linux/fs.h>
#include <linux/iopoll.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <uapi/linux/esp32s31-cordic.h>

/*
 * S31 TRM ch.5 System and Memory lists the CORDIC aperture. The CORDIC
 * chapter is pending in v0.2. Register mode follows ESP-IDF cordic_reg.h
 * and cordic_calculate_polling().
 */
#define S31_CORDIC_CFG			0x00
#define S31_CORDIC_ARG1			0x04
#define S31_CORDIC_ARG2			0x08
#define S31_CORDIC_RES1			0x0c
#define S31_CORDIC_RES2			0x10

#define S31_CORDIC_FUNC			GENMASK(3, 0)
#define S31_CORDIC_PREC			GENMASK(7, 4)
#define S31_CORDIC_SCALE		GENMASK(11, 8)
#define S31_CORDIC_TWO_RESULTS		BIT(19)
#define S31_CORDIC_TWO_ARGS		BIT(20)
#define S31_CORDIC_Q31			(BIT(21) | BIT(22))
#define S31_CORDIC_START		BIT(29)
#define S31_CORDIC_READY		BIT(31)

#define S31_CORDIC_PREC_12		2
#define S31_CORDIC_PREC_24		5
#define S31_CORDIC_HYP_MAX		1200443359	/* floor(0.559 * 2^31) */
#define S31_CORDIC_SQRT_MIN		229566002	/* ceil(0.1069 * 2^31) */

struct esp32s31_cordic {
	struct miscdevice misc;
	struct kref ref;
	struct mutex lock;
	void __iomem *base;
	struct clk *clk;
	struct reset_control *reset;
};

static void esp32s31_cordic_free(struct kref *ref)
{
	kfree(container_of(ref, struct esp32s31_cordic, ref));
}

static int esp32s31_cordic_config(const struct esp32s31_cordic_op *op, u32 *cfg)
{
	u32 mode = S31_CORDIC_Q31 | S31_CORDIC_TWO_RESULTS |
		   FIELD_PREP(S31_CORDIC_PREC, S31_CORDIC_PREC_24);

	switch (op->function) {
	case ESP32S31_CORDIC_COS:
	case ESP32S31_CORDIC_SIN:
		break;
	case ESP32S31_CORDIC_ATAN2:
		if (!op->x && !op->y)
			return -EDOM;
		mode |= S31_CORDIC_TWO_ARGS;
		break;
	case ESP32S31_CORDIC_COSH:
	case ESP32S31_CORDIC_SINH:
		if (op->x < -S31_CORDIC_HYP_MAX || op->x > S31_CORDIC_HYP_MAX)
			return -ERANGE;
		mode |= FIELD_PREP(S31_CORDIC_SCALE, 1);
		break;
	case ESP32S31_CORDIC_SQRT:
		if (op->x < S31_CORDIC_SQRT_MIN)
			return -ERANGE;
		mode &= ~S31_CORDIC_PREC;
		mode |= FIELD_PREP(S31_CORDIC_PREC, S31_CORDIC_PREC_12);
		break;
	default:
		return -EINVAL;
	}
	if (!(mode & S31_CORDIC_TWO_ARGS) && op->y)
		return -EINVAL;
	*cfg = mode | FIELD_PREP(S31_CORDIC_FUNC, op->function);
	return 0;
}

static int esp32s31_cordic_run(struct esp32s31_cordic *cordic,
			       struct esp32s31_cordic_op *ops, u32 count)
{
	u32 cfg[ESP32S31_CORDIC_MAX_BATCH];
	u32 state, i;
	int ret;

	for (i = 0; i < count; i++) {
		ret = esp32s31_cordic_config(&ops[i], &cfg[i]);
		if (ret)
			return ret;
	}

	ret = mutex_lock_interruptible(&cordic->lock);
	if (ret)
		return ret;
	if (!cordic->base) {
		ret = -ENODEV;
		goto unlock;
	}
	ret = clk_prepare_enable(cordic->clk);
	if (ret)
		goto unlock;

	ret = reset_control_reset(cordic->reset);
	if (ret)
		goto disable;
	for (i = 0; i < count; i++) {
		writel(cfg[i], cordic->base + S31_CORDIC_CFG);
		writel(ops[i].x, cordic->base + S31_CORDIC_ARG1);
		if (cfg[i] & S31_CORDIC_TWO_ARGS)
			writel(ops[i].y, cordic->base + S31_CORDIC_ARG2);
		writel(cfg[i] | S31_CORDIC_START, cordic->base + S31_CORDIC_CFG);
		ret = readl_poll_timeout(cordic->base + S31_CORDIC_CFG, state,
					 state & S31_CORDIC_READY, 0, 1000);
		if (ret)
			break;
		ops[i].result = readl(cordic->base + S31_CORDIC_RES1);
		/* Consume both words of the Q31 result pair. */
		readl(cordic->base + S31_CORDIC_RES2);
	}
disable:
	clk_disable_unprepare(cordic->clk);
unlock:
	mutex_unlock(&cordic->lock);
	return ret;
}

static long esp32s31_cordic_ioctl(struct file *file, unsigned int cmd,
				  unsigned long arg)
{
	struct esp32s31_cordic *cordic = file->private_data;
	void __user *user_ops = (void __user *)arg;
	struct esp32s31_cordic_batch batch;
	struct esp32s31_cordic_op *ops;
	u32 count = 1;
	size_t size;
	int ret;

	switch (cmd) {
	case ESP32S31_CORDIC_IOC_CALC:
		break;
	case ESP32S31_CORDIC_IOC_BATCH:
		if (copy_from_user(&batch, user_ops, sizeof(batch)))
			return -EFAULT;
		if (!batch.count || batch.count > ESP32S31_CORDIC_MAX_BATCH ||
		    batch.reserved)
			return -EINVAL;
		user_ops = u64_to_user_ptr(batch.ops);
		count = batch.count;
		break;
	default:
		return -ENOTTY;
	}

	size = count * sizeof(*ops);
	ops = memdup_user(user_ops, size);
	if (IS_ERR(ops))
		return PTR_ERR(ops);
	ret = esp32s31_cordic_run(cordic, ops, count);
	if (!ret && copy_to_user(user_ops, ops, size))
		ret = -EFAULT;
	kfree(ops);

	return ret;
}

static int esp32s31_cordic_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct esp32s31_cordic *cordic =
		container_of(misc, struct esp32s31_cordic, misc);

	kref_get(&cordic->ref);
	file->private_data = cordic;
	return nonseekable_open(inode, file);
}

static int esp32s31_cordic_release(struct inode *inode, struct file *file)
{
	struct esp32s31_cordic *cordic = file->private_data;

	kref_put(&cordic->ref, esp32s31_cordic_free);
	return 0;
}

static const struct file_operations esp32s31_cordic_fops = {
	.owner = THIS_MODULE,
	.open = esp32s31_cordic_open,
	.release = esp32s31_cordic_release,
	.unlocked_ioctl = esp32s31_cordic_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

static void esp32s31_cordic_put(void *data)
{
	struct esp32s31_cordic *cordic = data;

	kref_put(&cordic->ref, esp32s31_cordic_free);
}

static int esp32s31_cordic_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp32s31_cordic *cordic;
	int ret;

	cordic = kzalloc(sizeof(*cordic), GFP_KERNEL);
	if (!cordic)
		return -ENOMEM;
	kref_init(&cordic->ref);
	mutex_init(&cordic->lock);
	ret = devm_add_action_or_reset(dev, esp32s31_cordic_put, cordic);
	if (ret)
		return ret;

	cordic->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cordic->base))
		return PTR_ERR(cordic->base);
	cordic->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(cordic->clk))
		return dev_err_probe(dev, PTR_ERR(cordic->clk), "failed to get clock\n");
	cordic->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(cordic->reset))
		return dev_err_probe(dev, PTR_ERR(cordic->reset), "failed to get reset\n");

	cordic->misc.minor = MISC_DYNAMIC_MINOR;
	cordic->misc.name = "esp32s31-cordic";
	cordic->misc.fops = &esp32s31_cordic_fops;
	cordic->misc.parent = dev;
	platform_set_drvdata(pdev, cordic);

	return misc_register(&cordic->misc);
}

static void esp32s31_cordic_remove(struct platform_device *pdev)
{
	struct esp32s31_cordic *cordic = platform_get_drvdata(pdev);

	misc_deregister(&cordic->misc);
	mutex_lock(&cordic->lock);
	cordic->base = NULL;
	mutex_unlock(&cordic->lock);
}

static const struct of_device_id esp32s31_cordic_match[] = {
	{ .compatible = "espressif,esp32s31-cordic" },
	{}
};
MODULE_DEVICE_TABLE(of, esp32s31_cordic_match);

static struct platform_driver esp32s31_cordic_driver = {
	.probe = esp32s31_cordic_probe,
	.remove = esp32s31_cordic_remove,
	.driver = {
		.name = "esp32s31-cordic",
		.of_match_table = esp32s31_cordic_match,
	},
};
module_platform_driver(esp32s31_cordic_driver);

MODULE_DESCRIPTION("ESP32-S31 CORDIC accelerator");
MODULE_LICENSE("GPL");
