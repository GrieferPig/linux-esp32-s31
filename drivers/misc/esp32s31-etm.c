// SPDX-License-Identifier: GPL-2.0-only
/* File-owned ESP32-S31 event/task routes. Endpoints remain owned by their
 * peripheral drivers. This privileged API changes routing, not pin muxes,
 * clocksource registers, DMA descriptors or peripheral configuration.
 */
#include <linux/bitmap.h>
#include <linux/capability.h>
#include <linux/clk.h>
#include <linux/compat.h>
#include <linux/io.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <uapi/linux/esp32s31-accelerators.h>

#define S31_ETM_CHANNELS 50
#define S31_ETM_EVENT_MAX 398
#define S31_ETM_TASK_MAX 357
#define S31_ETM_ROUTE_BASE 0x18
#define S31_ETM_EVENT_STATUS 0x1a8
#define S31_ETM_TASK_STATUS 0x210
struct s31_etm {
	struct miscdevice misc;
	struct device *dev;
	void __iomem *regs;
	struct clk *clk;
	struct reset_control *reset;
	struct mutex lock;
	struct kref refs;
	DECLARE_BITMAP(allocated, S31_ETM_CHANNELS);
	struct s31_etm_route routes[S31_ETM_CHANNELS];
	unsigned int open_count;
	bool removed;
};
struct s31_etm_file {
	struct s31_etm *etm;
	DECLARE_BITMAP(owned, S31_ETM_CHANNELS);
};
static void s31_etm_free(struct kref *ref)
{
	kfree(container_of(ref, struct s31_etm, refs));
}
static void s31_etm_enable(struct s31_etm *e, u32 channel, bool enabled)
{
	writel(BIT(channel % 32),
	       e->regs + (channel < 32 ? 0 : 12) + (enabled ? 4 : 8));
}
static int s31_etm_open(struct inode *inode, struct file *file)
{
	struct s31_etm *e =
		container_of(file->private_data, struct s31_etm, misc);
	struct s31_etm_file *ctx;
	int ret;

	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	mutex_lock(&e->lock);
	ret = e->removed ? -ENODEV : pm_runtime_resume_and_get(e->dev);
	if (ret < 0) {
		mutex_unlock(&e->lock);
		kfree(ctx);
		return ret;
	}
	e->open_count++;
	kref_get(&e->refs);
	ctx->etm = e;
	file->private_data = ctx;
	mutex_unlock(&e->lock);
	return nonseekable_open(inode, file);
}
static int s31_etm_close(struct inode *inode, struct file *file)
{
	struct s31_etm_file *ctx = file->private_data;
	struct s31_etm *e = ctx->etm;
	unsigned long ch;

	mutex_lock(&e->lock);
	for_each_set_bit(ch, ctx->owned, S31_ETM_CHANNELS) {
		if (!e->removed) {
			s31_etm_enable(e, ch, false);
			writel(0, e->regs + S31_ETM_ROUTE_BASE + ch * 8);
			writel(0, e->regs + S31_ETM_ROUTE_BASE + ch * 8 + 4);
		}
		clear_bit(ch, e->allocated);
		memset(&e->routes[ch], 0, sizeof(e->routes[ch]));
	}
	if (!e->removed) {
		e->open_count--;
		pm_runtime_put_sync(e->dev);
	}
	mutex_unlock(&e->lock);
	kref_put(&e->refs, s31_etm_free);
	kfree(ctx);
	return 0;
}
static long s31_etm_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct s31_etm_file *ctx = file->private_data;
	struct s31_etm *e = ctx->etm;
	struct s31_etm_route r, old;
	void __user *up = (void __user *)arg;
	u32 ch;
	int ret = 0;

	if (cmd != S31_ETM_ROUTE_SET && cmd != S31_ETM_ROUTE_GET &&
	    cmd != S31_ETM_ROUTE_DEL)
		return -ENOTTY;
	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;
	if (copy_from_user(&r, up, sizeof(r)))
		return -EFAULT;
	if (r.version != S31_ACCEL_ABI_VERSION || r.reserved)
		return -EINVAL;
	if (mutex_lock_interruptible(&e->lock))
		return -ERESTARTSYS;
	if (e->removed) {
		ret = -ENODEV;
		goto out;
	}
	ch = r.channel;
	if (cmd == S31_ETM_ROUTE_SET && ch == S31_ETM_AUTO_CHANNEL) {
		ch = find_first_zero_bit(e->allocated, S31_ETM_CHANNELS);
		if (ch == S31_ETM_CHANNELS) {
			ret = -ENOSPC;
			goto out;
		}
	} else if (ch >= S31_ETM_CHANNELS || !test_bit(ch, ctx->owned)) {
		ret = -ENOENT;
		goto out;
	}
	if (cmd == S31_ETM_ROUTE_DEL) {
		s31_etm_enable(e, ch, false);
		writel(0, e->regs + S31_ETM_ROUTE_BASE + ch * 8);
		writel(0, e->regs + S31_ETM_ROUTE_BASE + ch * 8 + 4);
		clear_bit(ch, ctx->owned);
		clear_bit(ch, e->allocated);
		memset(&e->routes[ch], 0, sizeof(r));
		goto out;
	}
	if (cmd == S31_ETM_ROUTE_SET) {
		if (!r.event || r.event > S31_ETM_EVENT_MAX || !r.task ||
		    r.task > S31_ETM_TASK_MAX || r.enabled > 1) {
			ret = -EINVAL;
			goto out;
		}
		old = e->routes[ch];
		s31_etm_enable(e, ch, false);
		writel(r.event, e->regs + S31_ETM_ROUTE_BASE + ch * 8);
		writel(r.task, e->regs + S31_ETM_ROUTE_BASE + ch * 8 + 4);
		r.channel = ch;
		r.event_seen = 0;
		r.task_seen = 0;
		e->routes[ch] = r;
		s31_etm_enable(e, ch, r.enabled);
		if (copy_to_user(up, &r, sizeof(r))) {
			/* Restore ownership and hardware if userspace cannot receive the handle. */
			s31_etm_enable(e, ch, false);
			e->routes[ch] = old;
			writel(old.event,
			       e->regs + S31_ETM_ROUTE_BASE + ch * 8);
			writel(old.task,
			       e->regs + S31_ETM_ROUTE_BASE + ch * 8 + 4);
			s31_etm_enable(e, ch, old.enabled);
			ret = -EFAULT;
			goto out;
		}
		set_bit(ch, e->allocated);
		set_bit(ch, ctx->owned);
	} else {
		r = e->routes[ch];
		r.event_seen = !!(readl(e->regs + S31_ETM_EVENT_STATUS +
					((r.event - 1) / 32) * 8) &
				  BIT((r.event - 1) % 32));
		r.task_seen = !!(readl(e->regs + S31_ETM_TASK_STATUS +
				       ((r.task - 1) / 32) * 8) &
				 BIT((r.task - 1) % 32));
		if (copy_to_user(up, &r, sizeof(r)))
			ret = -EFAULT;
	}
out:
	mutex_unlock(&e->lock);
	return ret;
}
static const struct file_operations s31_etm_fops = {
	.owner = THIS_MODULE,
	.open = s31_etm_open,
	.release = s31_etm_close,
	.unlocked_ioctl = s31_etm_ioctl,
	.compat_ioctl = compat_ptr_ioctl,

};
static int s31_etm_resume(struct device *dev)
{
	struct s31_etm *e = dev_get_drvdata(dev);
	unsigned long ch;
	int ret = clk_prepare_enable(e->clk);

	if (ret)
		return ret;
	ret = reset_control_reset(e->reset);
	if (ret) {
		clk_disable_unprepare(e->clk);
		return ret;
	}
	writel(~0U, e->regs + 8);
	writel(GENMASK(17, 0), e->regs + 20);
	for_each_set_bit(ch, e->allocated, S31_ETM_CHANNELS) {
		writel(e->routes[ch].event,
		       e->regs + S31_ETM_ROUTE_BASE + ch * 8);
		writel(e->routes[ch].task,
		       e->regs + S31_ETM_ROUTE_BASE + ch * 8 + 4);
		s31_etm_enable(e, ch, e->routes[ch].enabled);
	}
	return 0;
}
static int s31_etm_suspend(struct device *dev)
{
	struct s31_etm *e = dev_get_drvdata(dev);

	writel(~0U, e->regs + 8);
	writel(GENMASK(17, 0), e->regs + 20);
	clk_disable_unprepare(e->clk);
	return 0;
}
static const struct dev_pm_ops s31_etm_pm = {
	SET_RUNTIME_PM_OPS(s31_etm_suspend, s31_etm_resume, NULL)
		SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
					pm_runtime_force_resume)
};
static int s31_etm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s31_etm *e = kzalloc(sizeof(*e), GFP_KERNEL);
	int ret;

	if (!e)
		return -ENOMEM;
	e->dev = dev;
	kref_init(&e->refs);
	mutex_init(&e->lock);
	e->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(e->regs)) {
		ret = PTR_ERR(e->regs);
		goto free;
	}
	e->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(e->clk)) {
		ret = PTR_ERR(e->clk);
		goto free;
	}
	e->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(e->reset)) {
		ret = PTR_ERR(e->reset);
		goto free;
	}
	platform_set_drvdata(pdev, e);
	pm_runtime_enable(dev);
	e->misc = (struct miscdevice){ .minor = MISC_DYNAMIC_MINOR,
				       .name = "s31-etm",
				       .fops = &s31_etm_fops,
				       .parent = dev,
				       .mode = 0600 };
	ret = misc_register(&e->misc);
	if (!ret)
		return 0;
	pm_runtime_disable(dev);
free:
	kref_put(&e->refs, s31_etm_free);
	return dev_err_probe(dev, ret, "ETM registration failed\n");
}
static void s31_etm_remove(struct platform_device *pdev)
{
	struct s31_etm *e = platform_get_drvdata(pdev);

	misc_deregister(&e->misc);
	mutex_lock(&e->lock);
	e->removed = true;
	/* Retired file handles must not retain runtime-PM usage references on
	 * this platform device when a fresh driver instance binds to it.
	 */
	while (e->open_count) {
		pm_runtime_put_noidle(e->dev);
		e->open_count--;
	}
	pm_runtime_disable(e->dev);
	if (!pm_runtime_status_suspended(e->dev))
		s31_etm_suspend(e->dev);
	mutex_unlock(&e->lock);
	kref_put(&e->refs, s31_etm_free);
}
static const struct of_device_id s31_etm_match[] = {
	{ .compatible = "espressif,esp32s31-etm" },
	{}
};
MODULE_DEVICE_TABLE(of, s31_etm_match);
static struct platform_driver s31_etm_driver = {
	.probe = s31_etm_probe,
	.remove = s31_etm_remove,
	.driver = { .name = "esp32s31-etm",
		    .of_match_table = s31_etm_match,
		    .pm = &s31_etm_pm },
};
module_platform_driver(s31_etm_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ESP32-S31 file-owned event/task routes");
