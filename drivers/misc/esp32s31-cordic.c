// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 CORDIC register-mode accelerator. Register protocol follows
 * ESP-IDF esp_hal_cordic/esp32s31; batches amortize system-call overhead.
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/compat.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/sched/signal.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <uapi/linux/esp32s31-accelerators.h>

struct s31_cordic {
	struct miscdevice misc;
	struct device *dev;
	void __iomem *regs;
	struct clk *clk;
	struct reset_control *reset;
	struct mutex lock;
	struct kref refs;
	bool removed;
};
static void s31_cordic_free(struct kref *ref)
{
	kfree(container_of(ref, struct s31_cordic, refs));
}
static int s31_cordic_open(struct inode *inode, struct file *file)
{
	struct s31_cordic *c =
		container_of(file->private_data, struct s31_cordic, misc);
	kref_get(&c->refs);
	file->private_data = c;
	return nonseekable_open(inode, file);
}
static int s31_cordic_close(struct inode *inode, struct file *file)
{
	struct s31_cordic *c = file->private_data;

	kref_put(&c->refs, s31_cordic_free);
	return 0;
}
static long s31_cordic_ioctl(struct file *file, unsigned int cmd,
			     unsigned long arg)
{
	static const u8 min_scale[] = { 0, 0, 0, 0, 0, 1, 1, 1, 1, 0 };
	static const u8 max_scale[] = { 0, 0, 15, 15, 15, 1, 1, 1, 1, 4 };
	struct s31_cordic *c = file->private_data;
	struct s31_cordic_job job;
	struct s31_cordic_sample *samples;
	void __user *up = (void __user *)arg;
	u32 config, status, i;
	bool two;
	int ret;

	if (cmd != S31_CORDIC_RUN)
		return -ENOTTY;
	if (copy_from_user(&job, up, sizeof(job)))
		return -EFAULT;
	if (job.version != S31_ACCEL_ABI_VERSION ||
	    job.function >= ARRAY_SIZE(min_scale) || job.format > 1 ||
	    !job.iterations || job.iterations > S31_CORDIC_MAX_ITERATIONS ||
	    job.scale < min_scale[job.function] ||
	    job.scale > max_scale[job.function] || !job.count ||
	    job.count > S31_CORDIC_MAX_SAMPLES ||
	    memchr_inv(job.reserved, 0, sizeof(job.reserved)))
		return -EINVAL;
	if (job.samples > ULONG_MAX)
		return -EFAULT;
	samples = memdup_array_user(u64_to_user_ptr(job.samples), job.count,
				    sizeof(*samples));
	if (IS_ERR(samples))
		return PTR_ERR(samples);
	job.completed = 0;
	ret = mutex_lock_interruptible(&c->lock);
	if (ret)
		goto out;
	if (c->removed) {
		ret = -ENODEV;
		goto unlock;
	}
	ret = pm_runtime_resume_and_get(c->dev);
	if (ret < 0)
		goto unlock;
	two = job.function == 2 || job.function == 3;
	config = job.function | ((job.iterations - 1) << 4) | (job.scale << 8) |
		 BIT(19) | (two ? BIT(20) : 0) |
		 (job.format ? BIT(21) | BIT(22) : 0);
	writel(0,
	       c->regs + 0x28); /* bounded polling, as in IDF register mode */
	writel(0x1f, c->regs + 0x34);
	for (i = 0; i < job.count; i++) {
		u32 a = samples[i].arg1;

		if (!job.format)
			a = (u16)a |
			    (two ? (u32)(u16)samples[i].arg2 << 16 : 0);
		writel(a, c->regs + 4);
		if (two && job.format)
			writel(samples[i].arg2, c->regs + 8);
		writel(config | BIT(29), c->regs);
		ret = readl_poll_timeout_atomic(c->regs, status,
						status & BIT(31), 0, 1000);
		if (ret) {
			reset_control_reset(c->reset);
			break;
		}
		a = readl(c->regs + 12);
		samples[i].result1 = job.format ? (s32)a : (s16)a;
		samples[i].result2 = job.format ? (s32)readl(c->regs + 16) :
						  (s16)(a >> 16);
		job.completed++;
		if (!(i & 63)) {
			if (signal_pending(current)) {
				ret = -EINTR;
				break;
			}
			cond_resched();
		}
	}
	pm_runtime_mark_last_busy(c->dev);
	pm_runtime_put_autosuspend(c->dev);
unlock:
	mutex_unlock(&c->lock);
	if (copy_to_user(u64_to_user_ptr(job.samples), samples,
			 job.completed * sizeof(*samples)) ||
	    copy_to_user(up, &job, sizeof(job)))
		ret = -EFAULT;
out:
	kfree(samples);
	return ret;
}
static const struct file_operations s31_cordic_fops = {
	.owner = THIS_MODULE,
	.open = s31_cordic_open,
	.release = s31_cordic_close,
	.unlocked_ioctl = s31_cordic_ioctl,
	.compat_ioctl = compat_ptr_ioctl,

};
static int s31_cordic_runtime_resume(struct device *dev)
{
	struct s31_cordic *c = dev_get_drvdata(dev);
	int ret = clk_prepare_enable(c->clk);

	if (ret)
		return ret;
	ret = reset_control_reset(c->reset);
	if (ret)
		clk_disable_unprepare(c->clk);
	return ret;
}
static int s31_cordic_runtime_suspend(struct device *dev)
{
	struct s31_cordic *c = dev_get_drvdata(dev);

	clk_disable_unprepare(c->clk);
	return 0;
}
static const struct dev_pm_ops s31_cordic_pm = { SET_RUNTIME_PM_OPS(
	s31_cordic_runtime_suspend, s31_cordic_runtime_resume,
	NULL) SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				      pm_runtime_force_resume) };
static int s31_cordic_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s31_cordic *c = kzalloc(sizeof(*c), GFP_KERNEL);
	int ret;

	if (!c)
		return -ENOMEM;
	c->dev = dev;
	mutex_init(&c->lock);
	kref_init(&c->refs);
	c->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(c->regs)) {
		ret = PTR_ERR(c->regs);
		goto free;
	}
	c->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(c->clk)) {
		ret = PTR_ERR(c->clk);
		goto free;
	}
	c->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(c->reset)) {
		ret = PTR_ERR(c->reset);
		goto free;
	}
	platform_set_drvdata(pdev, c);
	pm_runtime_set_autosuspend_delay(dev, 100);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);
	c->misc = (struct miscdevice){ .minor = MISC_DYNAMIC_MINOR,
				       .name = "s31-cordic",
				       .fops = &s31_cordic_fops,
				       .parent = dev,
				       .mode = 0600 };
	ret = misc_register(&c->misc);
	if (!ret)
		return 0;
	pm_runtime_disable(dev);
free:
	kref_put(&c->refs, s31_cordic_free);
	return dev_err_probe(dev, ret, "CORDIC registration failed\n");
}
static void s31_cordic_remove(struct platform_device *pdev)
{
	struct s31_cordic *c = platform_get_drvdata(pdev);

	misc_deregister(&c->misc);
	mutex_lock(&c->lock);
	c->removed = true;
	pm_runtime_disable(c->dev);
	if (!pm_runtime_status_suspended(c->dev))
		s31_cordic_runtime_suspend(c->dev);
	mutex_unlock(&c->lock);
	kref_put(&c->refs, s31_cordic_free);
}
static const struct of_device_id s31_cordic_match[] = {
	{ .compatible = "espressif,esp32s31-cordic" },
	{}
};
MODULE_DEVICE_TABLE(of, s31_cordic_match);
static struct platform_driver s31_cordic_driver = {
	.probe = s31_cordic_probe,
	.remove = s31_cordic_remove,
	.driver = { .name = "esp32s31-cordic",
		    .of_match_table = s31_cordic_match,
		    .pm = &s31_cordic_pm },
};
module_platform_driver(s31_cordic_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ESP32-S31 batched fixed-point CORDIC accelerator");
