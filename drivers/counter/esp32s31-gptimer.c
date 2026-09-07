// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 timer-group general-purpose timers. */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/counter.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define GPTIMER_STRIDE		0x24
#define GPTIMER_CONFIG(n)	((n) * GPTIMER_STRIDE + 0x00)
#define GPTIMER_LO(n)		((n) * GPTIMER_STRIDE + 0x04)
#define GPTIMER_HI(n)		((n) * GPTIMER_STRIDE + 0x08)
#define GPTIMER_UPDATE(n)	((n) * GPTIMER_STRIDE + 0x0c)
#define GPTIMER_LOAD_LO(n)	((n) * GPTIMER_STRIDE + 0x18)
#define GPTIMER_LOAD_HI(n)	((n) * GPTIMER_STRIDE + 0x1c)
#define GPTIMER_LOAD(n)		((n) * GPTIMER_STRIDE + 0x20)

#define GPTIMER_ALARM_EN	BIT(10)
#define GPTIMER_DIV_RST		BIT(12)
#define GPTIMER_DIVIDER		GENMASK(28, 13)
#define GPTIMER_AUTORELOAD	BIT(29)
#define GPTIMER_INCREASE	BIT(30)
#define GPTIMER_ENABLE		BIT(31)
#define GPTIMER_UPDATE_LATCH	BIT(31)
#define GPTIMER_HI_MASK		GENMASK(21, 0)

struct esp32s31_gptimer {
	void __iomem *base;
	struct clk *clk;
	struct mutex lock;
	u32 reserved_timer_mask;
	struct counter_count counts[2];
	struct counter_comp count_ext[2][2];
};

static const enum counter_function esp32s31_gptimer_functions[] = {
	COUNTER_FUNCTION_INCREASE,
	COUNTER_FUNCTION_DECREASE,
};

static int esp32s31_gptimer_count_read(struct counter_device *counter,
				       struct counter_count *count, u64 *value)
{
	struct esp32s31_gptimer *timer = counter_priv(counter);
	u32 update;
	int ret;

	writel(GPTIMER_UPDATE_LATCH, timer->base + GPTIMER_UPDATE(count->id));
	ret = readl_poll_timeout(timer->base + GPTIMER_UPDATE(count->id),
				 update, !(update & GPTIMER_UPDATE_LATCH), 0, 1000);
	if (ret)
		return ret;
	*value = ((u64)(readl(timer->base + GPTIMER_HI(count->id)) &
			GPTIMER_HI_MASK) << 32) |
		 readl(timer->base + GPTIMER_LO(count->id));
	return 0;
}

static int esp32s31_gptimer_count_write(struct counter_device *counter,
					struct counter_count *count, u64 value)
{
	struct esp32s31_gptimer *timer = counter_priv(counter);

	if (value >= BIT_ULL(54))
		return -ERANGE;
	mutex_lock(&timer->lock);
	writel(lower_32_bits(value), timer->base + GPTIMER_LOAD_LO(count->id));
	writel(upper_32_bits(value) & GPTIMER_HI_MASK,
	       timer->base + GPTIMER_LOAD_HI(count->id));
	writel(1, timer->base + GPTIMER_LOAD(count->id));
	mutex_unlock(&timer->lock);
	return 0;
}

static int esp32s31_gptimer_function_read(struct counter_device *counter,
					  struct counter_count *count,
					  enum counter_function *function)
{
	struct esp32s31_gptimer *timer = counter_priv(counter);

	*function = readl(timer->base + GPTIMER_CONFIG(count->id)) &
		GPTIMER_INCREASE ? COUNTER_FUNCTION_INCREASE :
		COUNTER_FUNCTION_DECREASE;
	return 0;
}

static int esp32s31_gptimer_function_write(struct counter_device *counter,
					   struct counter_count *count,
					   enum counter_function function)
{
	struct esp32s31_gptimer *timer = counter_priv(counter);
	u32 config;

	mutex_lock(&timer->lock);
	config = readl(timer->base + GPTIMER_CONFIG(count->id));
	if (function == COUNTER_FUNCTION_INCREASE)
		config |= GPTIMER_INCREASE;
	else if (function == COUNTER_FUNCTION_DECREASE)
		config &= ~GPTIMER_INCREASE;
	else {
		mutex_unlock(&timer->lock);
		return -EINVAL;
	}
	writel(config, timer->base + GPTIMER_CONFIG(count->id));
	mutex_unlock(&timer->lock);
	return 0;
}

static int esp32s31_gptimer_enable_read(struct counter_device *counter,
					struct counter_count *count, u8 *enable)
{
	struct esp32s31_gptimer *timer = counter_priv(counter);

	*enable = !!(readl(timer->base + GPTIMER_CONFIG(count->id)) &
		     GPTIMER_ENABLE);
	return 0;
}

static int esp32s31_gptimer_enable_write(struct counter_device *counter,
					 struct counter_count *count, u8 enable)
{
	struct esp32s31_gptimer *timer = counter_priv(counter);
	u32 config;

	mutex_lock(&timer->lock);
	config = readl(timer->base + GPTIMER_CONFIG(count->id));
	config &= ~(GPTIMER_ENABLE | GPTIMER_ALARM_EN | GPTIMER_AUTORELOAD);
	if (enable)
		config |= GPTIMER_ENABLE;
	writel(config, timer->base + GPTIMER_CONFIG(count->id));
	mutex_unlock(&timer->lock);
	return 0;
}

static int esp32s31_gptimer_prescaler_read(struct counter_device *counter,
					   struct counter_count *count,
					   u64 *prescaler)
{
	struct esp32s31_gptimer *timer = counter_priv(counter);
	u32 value = FIELD_GET(GPTIMER_DIVIDER,
			      readl(timer->base + GPTIMER_CONFIG(count->id)));

	*prescaler = value ?: 65536;
	return 0;
}

static int esp32s31_gptimer_prescaler_write(struct counter_device *counter,
					    struct counter_count *count,
					    u64 prescaler)
{
	struct esp32s31_gptimer *timer = counter_priv(counter);
	u32 config;

	if (prescaler < 2 || prescaler > 65536)
		return -ERANGE;
	mutex_lock(&timer->lock);
	config = readl(timer->base + GPTIMER_CONFIG(count->id));
	config &= ~GPTIMER_DIVIDER;
	config |= FIELD_PREP(GPTIMER_DIVIDER,
			     prescaler == 65536 ? 0 : prescaler) |
		  GPTIMER_DIV_RST;
	writel(config, timer->base + GPTIMER_CONFIG(count->id));
	mutex_unlock(&timer->lock);
	return 0;
}

static const struct counter_ops esp32s31_gptimer_ops = {
	.count_read = esp32s31_gptimer_count_read,
	.count_write = esp32s31_gptimer_count_write,
	.function_read = esp32s31_gptimer_function_read,
	.function_write = esp32s31_gptimer_function_write,
};

static int esp32s31_gptimer_probe(struct platform_device *pdev)
{
	struct counter_device *counter;
	struct esp32s31_gptimer *timer;
	struct counter_count *count;
	u32 prescaler;
	unsigned int i, exposed = 0;

	counter = devm_counter_alloc(&pdev->dev, sizeof(*timer));
	if (!counter)
		return -ENOMEM;
	timer = counter_priv(counter);
	mutex_init(&timer->lock);
	timer->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(timer->base))
		return PTR_ERR(timer->base);
	timer->clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(timer->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(timer->clk),
				     "clock unavailable\n");
	of_property_read_u32(pdev->dev.of_node,
			     "espressif,reserved-timer-mask",
			     &timer->reserved_timer_mask);
	if (timer->reserved_timer_mask & ~GENMASK(1, 0))
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "invalid reserved timer mask %#x\n",
				     timer->reserved_timer_mask);
	prescaler = clamp_t(u32, DIV_ROUND_CLOSEST(clk_get_rate(timer->clk),
						     1000000), 2, 65536);

	for (i = 0; i < ARRAY_SIZE(timer->counts); i++) {
		if (timer->reserved_timer_mask & BIT(i))
			continue;

		count = &timer->counts[exposed];
		count->id = i;
		count->name = devm_kasprintf(&pdev->dev, GFP_KERNEL,
					      "timer%u", i);
		if (!count->name)
			return -ENOMEM;
		count->functions_list = esp32s31_gptimer_functions;
		count->num_functions =
			ARRAY_SIZE(esp32s31_gptimer_functions);
		timer->count_ext[exposed][0] = (struct counter_comp)
			COUNTER_COMP_ENABLE(esp32s31_gptimer_enable_read,
					    esp32s31_gptimer_enable_write);
		timer->count_ext[exposed][1] = (struct counter_comp)
			COUNTER_COMP_COUNT_U64("prescaler",
				esp32s31_gptimer_prescaler_read,
				esp32s31_gptimer_prescaler_write);
		count->ext = timer->count_ext[exposed];
		count->num_ext = ARRAY_SIZE(timer->count_ext[exposed]);

		/* Safe default: stopped, count up, 1 MHz from the 80 MHz clock. */
		writel(GPTIMER_INCREASE |
		       FIELD_PREP(GPTIMER_DIVIDER,
				  prescaler == 65536 ? 0 : prescaler) |
		       GPTIMER_DIV_RST, timer->base + GPTIMER_CONFIG(i));
		exposed++;
	}
	if (!exposed)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "all timers are firmware-reserved\n");

	counter->name = dev_name(&pdev->dev);
	counter->parent = &pdev->dev;
	counter->ops = &esp32s31_gptimer_ops;
	counter->counts = timer->counts;
	counter->num_counts = exposed;
	return devm_counter_add(&pdev->dev, counter);
}

static const struct of_device_id esp32s31_gptimer_of_match[] = {
	{ .compatible = "espressif,esp32s31-gptimer" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s31_gptimer_of_match);

static struct platform_driver esp32s31_gptimer_driver = {
	.probe = esp32s31_gptimer_probe,
	.driver = {
		.name = "esp32s31-gptimer",
		.of_match_table = esp32s31_gptimer_of_match,
	},
};
module_platform_driver(esp32s31_gptimer_driver);

MODULE_DESCRIPTION("ESP32-S31 timer-group general-purpose timers");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("COUNTER");
