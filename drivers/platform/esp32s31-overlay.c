// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 transactional multi-overlay loader with resource arbitration. */

#include <linux/bitmap.h>
#include <linux/fs.h>
#include <linux/gpio/driver.h>
#include <linux/ioctl.h>
#include <linux/irqdomain.h>
#include <linux/libfdt.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/uaccess.h>

#define S31_OVERLAY_MAX_SIZE	SZ_128K
#define S31_OVERLAY_MAX_ACTIVE	32
#define S31_OVERLAY_NAME_LEN	32
#define S31_OVERLAY_RESOURCE_LEN	256
#define S31_MATRIX_INPUTS	256
#define S31_GPIO_NR		62

#define S31_PINMUX_TYPE_SHIFT	28
#define S31_PINMUX_TYPE_MASK	0x3
#define S31_PINMUX_MATRIX_IN	2
#define S31_MATRIX_CONST_ONE	0x80
#define S31_MATRIX_CONST_ZERO	0xc0

#define S31_OVERLAY_IOC_MAGIC	'O'
#define S31_OVERLAY_IOC_REMOVE_ALL _IO(S31_OVERLAY_IOC_MAGIC, 0)
#define S31_OVERLAY_IOC_GET_LAST_ID _IOR(S31_OVERLAY_IOC_MAGIC, 1, int)
#define S31_OVERLAY_IOC_REMOVE_NAME _IOW(S31_OVERLAY_IOC_MAGIC, 2, \
					 struct s31_overlay_name)
#define S31_OVERLAY_IOC_LIST _IOR(S31_OVERLAY_IOC_MAGIC, 3, \
				  struct s31_overlay_list)

struct s31_overlay_name {
	char name[S31_OVERLAY_NAME_LEN];
};

struct s31_overlay_item {
	s32 id;
	char name[S31_OVERLAY_NAME_LEN];
	u64 gpios;
};

struct s31_overlay_list {
	u32 count;
	struct s31_overlay_item items[S31_OVERLAY_MAX_ACTIVE];
};

struct s31_overlay {
	struct list_head node;
	void *blob;
	size_t blob_len;
	int id;
	char name[S31_OVERLAY_NAME_LEN];
	u64 gpios;
	DECLARE_BITMAP(matrix_inputs, S31_MATRIX_INPUTS);
	char resources[S31_OVERLAY_RESOURCE_LEN];
};

static DEFINE_MUTEX(s31_overlay_lock);
static LIST_HEAD(s31_overlays);
static int s31_overlay_last_id;
static bool s31_overlay_removing;

/*
 * platform_get_irq() mappings normally live for the life of a static DT
 * node.  Runtime overlays repeatedly create and destroy platform devices,
 * however, and the generic platform teardown does not dispose those
 * mappings.  On S31 that eventually exhausts the finite INTMTX-to-CLIC slot
 * pool.  Reclaim only devices synchronously removed by this manager, after
 * their drivers and managed resources have gone away.
 */
static int s31_overlay_platform_notify(struct notifier_block *nb,
				       unsigned long action, void *data)
{
	struct platform_device *pdev;
	struct device_node *np;
	int count, i, irq;

	if (action != BUS_NOTIFY_REMOVED_DEVICE ||
	    !READ_ONCE(s31_overlay_removing))
		return NOTIFY_DONE;

	pdev = to_platform_device(data);
	np = pdev->dev.of_node;
	if (!np)
		return NOTIFY_DONE;

	/*
	 * of_device_alloc() only puts address ranges in pdev->resource[];
	 * DT interrupts are mapped lazily by platform_get_irq().  Look them
	 * up through the node while it is still alive.  Existing mappings are
	 * returned unchanged.  If a driver never requested an IRQ, of_irq_get()
	 * may create it here; disposing it immediately is still correct and
	 * keeps the overlay lifecycle balanced.
	 */
	count = of_irq_count(np);
	for (i = 0; i < count; i++) {
		irq = of_irq_get(np, i);
		if (irq > 0)
			irq_dispose_mapping(irq);
	}

	return NOTIFY_OK;
}

static struct notifier_block s31_overlay_platform_nb = {
	.notifier_call = s31_overlay_platform_notify,
};

static int s31_of_overlay_remove(int *id)
{
	int ret;

	WRITE_ONCE(s31_overlay_removing, true);
	ret = of_overlay_remove(id);
	WRITE_ONCE(s31_overlay_removing, false);
	return ret;
}

static bool s31_valid_name(const char *name)
{
	size_t i, len = strnlen(name, S31_OVERLAY_NAME_LEN);

	if (!len || len == S31_OVERLAY_NAME_LEN)
		return false;
	for (i = 0; i < len; i++)
		if (!(name[i] == '-' || name[i] == '_' ||
		      (name[i] >= '0' && name[i] <= '9') ||
		      (name[i] >= 'a' && name[i] <= 'z')))
			return false;
	return true;
}

static bool s31_valid_gpio(u32 pin)
{
	/* GPIO26..32 carry the live XIP flash bus and cannot be reassigned. */
	return pin < S31_GPIO_NR && (pin < 26 || pin > 32) &&
	       pin != 33 && pin != 34 && pin != 41;
}

static int s31_overlay_parse(struct s31_overlay *overlay)
{
	const fdt32_t *cells;
	const char *name, *resources;
	int depth = 0, len, node = -1;

	if (fdt_check_header(overlay->blob))
		return -EINVAL;
	name = fdt_getprop(overlay->blob, 0, "espressif,overlay-name", &len);
	if (!name || len <= 0 || strnlen(name, len) == len ||
	    strscpy(overlay->name, name, sizeof(overlay->name)) < 0 ||
	    !s31_valid_name(overlay->name))
		return -EINVAL;

	resources = fdt_getprop(overlay->blob, 0,
				"espressif,resource-claims", &len);
	if (resources) {
		if (len <= 0 || len > sizeof(overlay->resources))
			return -E2BIG;
		memcpy(overlay->resources, resources, len);
		if (overlay->resources[len - 1])
			return -EINVAL;
	}
	cells = fdt_getprop(overlay->blob, 0, "espressif,gpio-claims", &len);
	if (cells) {
		int i;

		if (len <= 0 || len % sizeof(*cells))
			return -EINVAL;
		for (i = 0; i < len / sizeof(*cells); i++) {
			u32 pin = fdt32_to_cpu(cells[i]);

			if (!s31_valid_gpio(pin))
				return -EINVAL;
			overlay->gpios |= BIT_ULL(pin);
		}
	}

	while ((node = fdt_next_node(overlay->blob, node, &depth)) >= 0) {
		int i, count;

		cells = fdt_getprop(overlay->blob, node, "pinmux", &len);
		if (!cells)
			continue;
		if (len <= 0 || len % sizeof(*cells))
			return -EINVAL;
		count = len / sizeof(*cells);
		for (i = 0; i < count; i++) {
			u32 cell = fdt32_to_cpu(cells[i]);
			u32 type = (cell >> S31_PINMUX_TYPE_SHIFT) &
				   S31_PINMUX_TYPE_MASK;
			u32 pin = cell & 0xff;
			u32 signal = (cell >> 8) & 0x1ff;

			if (s31_valid_gpio(pin))
				overlay->gpios |= BIT_ULL(pin);
			else if (!(type == S31_PINMUX_MATRIX_IN &&
				   (pin == S31_MATRIX_CONST_ONE ||
				    pin == S31_MATRIX_CONST_ZERO)))
				return -EINVAL;
			if (type == S31_PINMUX_MATRIX_IN) {
				if (signal >= S31_MATRIX_INPUTS)
					return -EINVAL;
				__set_bit(signal, overlay->matrix_inputs);
			}
		}
	}
	return 0;
}

static bool s31_stringlist_intersects(const char *a, const char *b)
{
	const char *pa, *pb;

	for (pa = a; *pa; pa += strlen(pa) + 1)
		for (pb = b; *pb; pb += strlen(pb) + 1)
			if (!strcmp(pa, pb))
				return true;
	return false;
}

static struct s31_overlay *s31_overlay_find(const char *name)
{
	struct s31_overlay *overlay;

	list_for_each_entry(overlay, &s31_overlays, node)
		if (!strcmp(overlay->name, name))
			return overlay;
	return NULL;
}

static int s31_overlay_check_conflicts(struct s31_overlay *new,
				       struct s31_overlay *replaced)
{
	struct s31_overlay *active;
	struct gpio_device *gdev;
	struct device_node *np;
	struct gpio_chip *gc;
	unsigned int pin;

	/* UART0 is the live console and is not supplied by an overlay. */
	if (new->gpios & (BIT_ULL(58) | BIT_ULL(59))) {
		pr_warn("s31-overlay: %s conflicts with console GPIO mask %#llx\n",
			new->name, new->gpios & (BIT_ULL(58) | BIT_ULL(59)));
		return -EBUSY;
	}

	/* Also reject lines currently requested through the GPIO character API. */
	np = of_find_compatible_node(NULL, NULL, "espressif,esp32s31-pinctrl");
	if (np) {
		gdev = gpio_device_find_by_fwnode(of_fwnode_handle(np));
		of_node_put(np);
		if (!IS_ERR_OR_NULL(gdev)) {
			gc = gpio_device_get_chip(gdev);
			for (pin = 0; pin < S31_GPIO_NR; pin++) {
				char *label;

				if (!(new->gpios & BIT_ULL(pin)))
					continue;
				label = gpiochip_dup_line_label(gc, pin);
				if (IS_ERR_OR_NULL(label))
					continue;
				pr_warn("s31-overlay: %s conflicts with GPIO%u owner %s\n",
					new->name, pin, label);
				kfree(label);
				gpio_device_put(gdev);
				return -EBUSY;
			}
			gpio_device_put(gdev);
		}
	}

	list_for_each_entry(active, &s31_overlays, node) {
		if (active == replaced)
			continue;
		if (new->gpios & active->gpios) {
			pr_warn("s31-overlay: %s conflicts with %s on GPIO mask %#llx\n",
				new->name, active->name,
				new->gpios & active->gpios);
			return -EBUSY;
		}
		if (bitmap_intersects(new->matrix_inputs, active->matrix_inputs,
				      S31_MATRIX_INPUTS)) {
			pr_warn("s31-overlay: %s conflicts with %s on a matrix input\n",
				new->name, active->name);
			return -EBUSY;
		}
		if (s31_stringlist_intersects(new->resources, active->resources)) {
			pr_warn("s31-overlay: %s conflicts with %s on an exclusive resource\n",
				new->name, active->name);
			return -EBUSY;
		}
	}
	return 0;
}

static int s31_overlay_remove_locked(struct s31_overlay *overlay)
{
	int ret = s31_of_overlay_remove(&overlay->id);

	if (ret)
		return ret;
	list_del(&overlay->node);
	kfree(overlay->blob);
	kfree(overlay);
	return 0;
}

static int s31_overlay_remove_all_locked(void)
{
	struct s31_overlay *overlay, *tmp;
	int ret;

	list_for_each_entry_safe_reverse(overlay, tmp, &s31_overlays, node) {
		ret = s31_overlay_remove_locked(overlay);
		if (ret)
			return ret;
	}
	s31_overlay_last_id = 0;
	return 0;
}

static ssize_t s31_overlay_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	struct s31_overlay *new, *old;
	int rollback, ret;

	if (*ppos || count < sizeof(struct fdt_header) ||
	    count > S31_OVERLAY_MAX_SIZE)
		return -EINVAL;
	new = kzalloc(sizeof(*new), GFP_KERNEL);
	if (!new)
		return -ENOMEM;
	new->blob = memdup_user(buf, count);
	if (IS_ERR(new->blob)) {
		ret = PTR_ERR(new->blob);
		kfree(new);
		return ret;
	}
	new->blob_len = count;
	ret = s31_overlay_parse(new);
	if (ret)
		goto out_free;

	mutex_lock(&s31_overlay_lock);
	old = s31_overlay_find(new->name);
	ret = s31_overlay_check_conflicts(new, old);
	if (ret)
		goto out_unlock;

	if (old) {
		ret = s31_of_overlay_remove(&old->id);
		if (ret)
			goto out_unlock;
		list_del(&old->node);
	}

	ret = of_overlay_fdt_apply(new->blob, count, &new->id, NULL);
	if (!ret) {
		list_add_tail(&new->node, &s31_overlays);
		s31_overlay_last_id = new->id;
		if (old) {
			kfree(old->blob);
			kfree(old);
		}
		mutex_unlock(&s31_overlay_lock);
		*ppos += count;
		return count;
	}

	if (new->id)
		s31_of_overlay_remove(&new->id);
	if (old) {
		rollback = of_overlay_fdt_apply(old->blob, old->blob_len,
						&old->id, NULL);
		if (rollback) {
			pr_err("s31-overlay: rollback of %s failed: %d\n",
			       old->name, rollback);
			kfree(old->blob);
			kfree(old);
		} else {
			list_add_tail(&old->node, &s31_overlays);
		}
	}
out_unlock:
	mutex_unlock(&s31_overlay_lock);
out_free:
	kfree(new->blob);
	kfree(new);
	return ret;
}

static long s31_overlay_ioctl(struct file *file, unsigned int cmd,
			      unsigned long arg)
{
	struct s31_overlay_name requested;
	struct s31_overlay_list *list;
	struct s31_overlay *overlay;
	int ret = 0;

	switch (cmd) {
	case S31_OVERLAY_IOC_REMOVE_ALL:
		mutex_lock(&s31_overlay_lock);
		ret = s31_overlay_remove_all_locked();
		mutex_unlock(&s31_overlay_lock);
		return ret;
	case S31_OVERLAY_IOC_GET_LAST_ID:
		return copy_to_user((int __user *)arg, &s31_overlay_last_id,
				    sizeof(s31_overlay_last_id)) ? -EFAULT : 0;
	case S31_OVERLAY_IOC_REMOVE_NAME:
		if (copy_from_user(&requested, (void __user *)arg,
				   sizeof(requested)))
			return -EFAULT;
		requested.name[sizeof(requested.name) - 1] = '\0';
		if (!s31_valid_name(requested.name))
			return -EINVAL;
		mutex_lock(&s31_overlay_lock);
		overlay = s31_overlay_find(requested.name);
		ret = overlay ? s31_overlay_remove_locked(overlay) : -ENOENT;
		mutex_unlock(&s31_overlay_lock);
		return ret;
	case S31_OVERLAY_IOC_LIST:
		list = kzalloc(sizeof(*list), GFP_KERNEL);
		if (!list)
			return -ENOMEM;
		mutex_lock(&s31_overlay_lock);
		list_for_each_entry(overlay, &s31_overlays, node) {
			struct s31_overlay_item *item;

			if (list->count == S31_OVERLAY_MAX_ACTIVE)
				break;
			item = &list->items[list->count++];
			item->id = overlay->id;
			item->gpios = overlay->gpios;
			strscpy(item->name, overlay->name, sizeof(item->name));
		}
		mutex_unlock(&s31_overlay_lock);
		ret = copy_to_user((void __user *)arg, list, sizeof(*list)) ?
			-EFAULT : 0;
		kfree(list);
		return ret;
	default:
		return -ENOTTY;
	}
}

static const struct file_operations s31_overlay_fops = {
	.owner = THIS_MODULE,
	.write = s31_overlay_write,
	.unlocked_ioctl = s31_overlay_ioctl,
	.llseek = noop_llseek,
};

static struct miscdevice s31_overlay_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "s31-overlay",
	.fops = &s31_overlay_fops,
	.mode = 0600,
};

static int __init s31_overlay_init(void)
{
	int ret;

	ret = bus_register_notifier(&platform_bus_type,
				    &s31_overlay_platform_nb);
	if (ret)
		return ret;
	ret = misc_register(&s31_overlay_miscdev);
	if (ret)
		bus_unregister_notifier(&platform_bus_type,
					&s31_overlay_platform_nb);
	return ret;
}
module_init(s31_overlay_init);

static void __exit s31_overlay_exit(void)
{
	misc_deregister(&s31_overlay_miscdev);
	bus_unregister_notifier(&platform_bus_type,
				&s31_overlay_platform_nb);
}
module_exit(s31_overlay_exit);

MODULE_DESCRIPTION("ESP32-S31 multi-overlay loader and resource arbiter");
MODULE_LICENSE("GPL");
