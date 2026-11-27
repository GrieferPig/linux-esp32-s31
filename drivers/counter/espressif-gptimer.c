// SPDX-License-Identifier: GPL-2.0-only
/* Espressif timer-group general-purpose timers. */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/counter.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/platform_device.h>

/* ESP32-P4 TRM ch.18 TIMG - TnCONFIG, TnUPDATE, TnLOAD and 54-bit count. */
#define GPTIMER_STRIDE		0x24
#define GPTIMER_CONFIG(n)	((n) * GPTIMER_STRIDE + 0x00)
#define GPTIMER_LO(n)		((n) * GPTIMER_STRIDE + 0x04)
#define GPTIMER_HI(n)		((n) * GPTIMER_STRIDE + 0x08)
#define GPTIMER_UPDATE(n)	((n) * GPTIMER_STRIDE + 0x0c)
#define GPTIMER_ALARM_LO(n)	((n) * GPTIMER_STRIDE + 0x10)
#define GPTIMER_ALARM_HI(n)	((n) * GPTIMER_STRIDE + 0x14)
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

/* TRM ch.18 Timer Group — INT_ENA_TIMERS, INT_ST_TIMERS, INT_CLR_TIMERS. */
#define GPTIMER_INT_ENA		0x00
#define GPTIMER_INT_ST		0x08
#define GPTIMER_INT_CLR		0x0c
/* S31 TRM ch.7 Timer Group: TIMG_REGCLK.TIMG_ETM_EN. */
#define S31_GPTIMER_ETM_EN	BIT(28)

struct esp_gptimer_alarm {
	struct counter_device *counter;
	unsigned int index;
	int irq;
};

struct esp32s31_gptimer {
	void __iomem *base;
	void __iomem *interrupts;
	void __iomem *etm_enable;
	struct clk *clk;
	spinlock_t lock;
	u32 alarm_mask;
	u32 armed_mask;
	u32 etm_alarm_mask;
	struct esp_gptimer_alarm alarms[2];
	u32 reserved_timer_mask;
	struct counter_count counts[2];
	struct counter_comp count_ext[2][5];
};

static const enum counter_function esp32s31_gptimer_functions[] = {
	COUNTER_FUNCTION_INCREASE,
	COUNTER_FUNCTION_DECREASE,
};

static int esp32s31_gptimer_count_read(struct counter_device *counter,
				       struct counter_count *count, u64 *value)
{
	struct esp32s31_gptimer *timer = counter_priv(counter);
	unsigned long flags;
	u32 update;
	int ret;

	spin_lock_irqsave(&timer->lock, flags);
	writel(GPTIMER_UPDATE_LATCH, timer->base + GPTIMER_UPDATE(count->id));
	ret = readl_poll_timeout_atomic(timer->base + GPTIMER_UPDATE(count->id),
				 update, !(update & GPTIMER_UPDATE_LATCH), 1, 1000);
	if (!ret)
		*value = ((u64)(readl(timer->base + GPTIMER_HI(count->id)) &
				GPTIMER_HI_MASK) << 32) |
			 readl(timer->base + GPTIMER_LO(count->id));
	spin_unlock_irqrestore(&timer->lock, flags);
	return ret;
}

static int esp32s31_gptimer_count_write(struct counter_device *counter,
					struct counter_count *count, u64 value)
{
	struct esp32s31_gptimer *timer = counter_priv(counter);
	unsigned long flags;

	if (value >= BIT_ULL(54))
		return -ERANGE;
	spin_lock_irqsave(&timer->lock, flags);
	writel(lower_32_bits(value), timer->base + GPTIMER_LOAD_LO(count->id));
	writel(upper_32_bits(value) & GPTIMER_HI_MASK,
	       timer->base + GPTIMER_LOAD_HI(count->id));
	writel(1, timer->base + GPTIMER_LOAD(count->id));
	spin_unlock_irqrestore(&timer->lock, flags);
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
	unsigned long flags;
	u32 config;

	spin_lock_irqsave(&timer->lock, flags);
	config = readl(timer->base + GPTIMER_CONFIG(count->id));
	if (function == COUNTER_FUNCTION_INCREASE)
		config |= GPTIMER_INCREASE;
	else if (function == COUNTER_FUNCTION_DECREASE)
		config &= ~GPTIMER_INCREASE;
	else {
		spin_unlock_irqrestore(&timer->lock, flags);
		return -EINVAL;
	}
	writel(config, timer->base + GPTIMER_CONFIG(count->id));
	spin_unlock_irqrestore(&timer->lock, flags);
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
	unsigned long flags;
	u32 config;

	spin_lock_irqsave(&timer->lock, flags);
	config = readl(timer->base + GPTIMER_CONFIG(count->id));
	if (!((timer->alarm_mask | timer->etm_alarm_mask) & BIT(count->id)))
		config &= ~(GPTIMER_ALARM_EN | GPTIMER_AUTORELOAD);
	if (enable) {
		config |= GPTIMER_ENABLE;
	} else {
		config &= ~(GPTIMER_ENABLE | GPTIMER_ALARM_EN);
		timer->armed_mask &= ~BIT(count->id);
	}
	writel(config, timer->base + GPTIMER_CONFIG(count->id));
	if (!enable && (timer->alarm_mask & BIT(count->id)))
		writel(BIT(count->id), timer->interrupts + GPTIMER_INT_CLR);
	spin_unlock_irqrestore(&timer->lock, flags);
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
	unsigned long flags;
	u32 config;

	if (prescaler < 2 || prescaler > 65536)
		return -ERANGE;
	spin_lock_irqsave(&timer->lock, flags);
	config = readl(timer->base + GPTIMER_CONFIG(count->id));
	config &= ~GPTIMER_DIVIDER;
	config |= FIELD_PREP(GPTIMER_DIVIDER,
			     prescaler == 65536 ? 0 : prescaler) |
		  GPTIMER_DIV_RST;
	writel(config, timer->base + GPTIMER_CONFIG(count->id));
	spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}

static int esp_gptimer_alarm_read(struct counter_device *counter,
				  struct counter_count *count, u64 *value)
{
	struct esp32s31_gptimer *timer = counter_priv(counter);
	unsigned long flags;
	u32 lo, hi;

	spin_lock_irqsave(&timer->lock, flags);
	lo = readl(timer->base + GPTIMER_ALARM_LO(count->id));
	hi = readl(timer->base + GPTIMER_ALARM_HI(count->id));
	spin_unlock_irqrestore(&timer->lock, flags);
	*value = ((u64)(hi & GPTIMER_HI_MASK) << 32) | lo;
	return 0;
}

static int esp_gptimer_alarm_write(struct counter_device *counter,
				   struct counter_count *count, u64 value)
{
	struct esp32s31_gptimer *timer = counter_priv(counter);
	unsigned long flags;

	if (value >= BIT_ULL(54))
		return -ERANGE;
	spin_lock_irqsave(&timer->lock, flags);
	if ((timer->armed_mask & BIT(count->id)) ||
	    ((timer->etm_alarm_mask & BIT(count->id)) &&
	     (readl(timer->base + GPTIMER_CONFIG(count->id)) & GPTIMER_ALARM_EN))) {
		spin_unlock_irqrestore(&timer->lock, flags);
		return -EBUSY;
	}
	writel(lower_32_bits(value), timer->base + GPTIMER_ALARM_LO(count->id));
	writel(upper_32_bits(value), timer->base + GPTIMER_ALARM_HI(count->id));
	spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}

static int esp_gptimer_alarm_enable_read(struct counter_device *counter,
					 struct counter_count *count,
					 u8 *enable)
{
	struct esp32s31_gptimer *timer = counter_priv(counter);

	if (timer->etm_alarm_mask & BIT(count->id))
		*enable = !!(readl(timer->base + GPTIMER_CONFIG(count->id)) & GPTIMER_ALARM_EN);
	else
		*enable = !!(READ_ONCE(timer->armed_mask) & BIT(count->id));
	return 0;
}

static int esp_gptimer_alarm_enable_write(struct counter_device *counter,
					  struct counter_count *count,
					  u8 enable)
{
	struct esp32s31_gptimer *timer = counter_priv(counter);
	void __iomem *reg = timer->base + GPTIMER_CONFIG(count->id);
	unsigned long flags;
	u32 config;

	spin_lock_irqsave(&timer->lock, flags);
	config = readl(reg);
	if (enable) {
		if (timer->alarm_mask & BIT(count->id))
			timer->armed_mask |= BIT(count->id);
		config |= GPTIMER_ALARM_EN;
	} else {
		timer->armed_mask &= ~BIT(count->id);
		config &= ~GPTIMER_ALARM_EN;
	}
	writel(config, reg);
	if (!enable && timer->interrupts)
		writel(BIT(count->id), timer->interrupts + GPTIMER_INT_CLR);
	spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}

static int esp_gptimer_autoreload_read(struct counter_device *counter,
				       struct counter_count *count, u8 *enable)
{
	struct esp32s31_gptimer *timer = counter_priv(counter);

	*enable = !!(readl(timer->base + GPTIMER_CONFIG(count->id)) & GPTIMER_AUTORELOAD);
	return 0;
}

static int esp_gptimer_autoreload_write(struct counter_device *counter,
					struct counter_count *count, u8 enable)
{
	struct esp32s31_gptimer *timer = counter_priv(counter);
	void __iomem *reg = timer->base + GPTIMER_CONFIG(count->id);
	unsigned long flags;
	u32 config;

	spin_lock_irqsave(&timer->lock, flags);
	config = readl(reg);
	if (enable)
		config |= GPTIMER_AUTORELOAD;
	else
		config &= ~GPTIMER_AUTORELOAD;
	writel(config, reg);
	spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}

static int esp_gptimer_watch_validate(struct counter_device *counter,
				      const struct counter_watch *watch)
{
	struct esp32s31_gptimer *timer = counter_priv(counter);

	if (timer->etm_alarm_mask)
		return -EOPNOTSUPP;
	if (!timer->alarm_mask)
		return 0;
	if (watch->event != COUNTER_EVENT_THRESHOLD ||
	    watch->channel >= ARRAY_SIZE(timer->alarms) ||
	    !(timer->alarm_mask & BIT(watch->channel)))
		return -EINVAL;
	return 0;
}

static irqreturn_t esp_gptimer_alarm_irq(int irq, void *data)
{
	struct esp_gptimer_alarm *alarm = data;
	struct esp32s31_gptimer *timer = counter_priv(alarm->counter);
	void __iomem *reg = timer->base + GPTIMER_CONFIG(alarm->index);
	unsigned long flags;
	u32 bit = BIT(alarm->index);
	u32 config;

	spin_lock_irqsave(&timer->lock, flags);
	if (!(readl(timer->interrupts + GPTIMER_INT_ST) & bit)) {
		spin_unlock_irqrestore(&timer->lock, flags);
		return IRQ_NONE;
	}

	writel(bit, timer->interrupts + GPTIMER_INT_CLR);
	if (!(timer->armed_mask & bit)) {
		spin_unlock_irqrestore(&timer->lock, flags);
		return IRQ_HANDLED;
	}
	config = readl(reg);
	/* TRM 18.4.3: hardware clears ALARM_EN on match, even with reload. */
	if (config & GPTIMER_AUTORELOAD)
		writel(config | GPTIMER_ALARM_EN, reg);
	else
		timer->armed_mask &= ~bit;
	spin_unlock_irqrestore(&timer->lock, flags);

	counter_push_event(alarm->counter, COUNTER_EVENT_THRESHOLD, alarm->index);
	return IRQ_HANDLED;
}

static void esp_gptimer_disable_alarms(void *data)
{
	struct esp32s31_gptimer *timer = data;
	unsigned long flags;
	unsigned int i;
	u32 mask;

	spin_lock_irqsave(&timer->lock, flags);
	mask = readl(timer->interrupts + GPTIMER_INT_ENA);
	writel(mask & ~timer->alarm_mask, timer->interrupts + GPTIMER_INT_ENA);
	writel(timer->alarm_mask, timer->interrupts + GPTIMER_INT_CLR);
	for (i = 0; i < ARRAY_SIZE(timer->alarms); i++) {
		void __iomem *reg = timer->base + GPTIMER_CONFIG(i);

		if (timer->alarm_mask & BIT(i))
			writel(readl(reg) & ~(GPTIMER_ENABLE | GPTIMER_ALARM_EN), reg);
	}
	spin_unlock_irqrestore(&timer->lock, flags);
	for (i = 0; i < ARRAY_SIZE(timer->alarms); i++)
		if (timer->alarm_mask & BIT(i))
			synchronize_irq(timer->alarms[i].irq);
}

static int esp_gptimer_register_alarm(struct platform_device *pdev,
				      struct counter_device *counter,
				      unsigned int index)
{
	struct esp32s31_gptimer *timer = counter_priv(counter);
	struct esp_gptimer_alarm *alarm = &timer->alarms[index];
	const char *name = index ? "timer1" : "timer0";
	int ret;

	alarm->irq = platform_get_irq_byname(pdev, name);
	if (alarm->irq < 0)
		return alarm->irq;
	alarm->counter = counter;
	alarm->index = index;
	ret = devm_request_irq(&pdev->dev, alarm->irq, esp_gptimer_alarm_irq,
			       0, name, alarm);
	if (ret)
		return ret;
	timer->alarm_mask |= BIT(index);
	return 0;
}

static const struct counter_ops esp32s31_gptimer_ops = {
	.count_read = esp32s31_gptimer_count_read,
	.count_write = esp32s31_gptimer_count_write,
	.function_read = esp32s31_gptimer_function_read,
	.function_write = esp32s31_gptimer_function_write,
	.watch_validate = esp_gptimer_watch_validate,
};

static void s31_gptimer_disable_etm(void *arg)
{
	struct esp32s31_gptimer *timer = arg;
	u32 cfg;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(timer->counts); i++) {
		if (timer->etm_alarm_mask & BIT(i)) {
			cfg = readl(timer->base + GPTIMER_CONFIG(i));
			writel(cfg & ~(GPTIMER_ENABLE | GPTIMER_ALARM_EN),
			       timer->base + GPTIMER_CONFIG(i));
		}
	}
	cfg = readl(timer->etm_enable);
	writel(cfg & ~S31_GPTIMER_ETM_EN, timer->etm_enable);
}

static int esp32s31_gptimer_probe(struct platform_device *pdev)
{
	struct counter_device *counter;
	struct esp32s31_gptimer *timer;
	struct counter_count *count;
	u32 prescaler;
	unsigned int i, exposed = 0;
	int ret;

	counter = devm_counter_alloc(&pdev->dev, sizeof(*timer));
	if (!counter)
		return -ENOMEM;
	timer = counter_priv(counter);
	spin_lock_init(&timer->lock);
	timer->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(timer->base))
		return PTR_ERR(timer->base);
	timer->clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(timer->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(timer->clk),
				     "clock unavailable\n");
	if (of_device_is_compatible(pdev->dev.of_node, "espressif,esp32p4-gptimer") &&
	    platform_get_resource(pdev, IORESOURCE_MEM, 1)) {
		timer->interrupts = devm_platform_ioremap_resource(pdev, 1);
		if (IS_ERR(timer->interrupts))
			return PTR_ERR(timer->interrupts);
	}
	of_property_read_u32(pdev->dev.of_node,
			     "espressif,reserved-timer-mask",
			     &timer->reserved_timer_mask);
	if (timer->reserved_timer_mask & ~GENMASK(1, 0))
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "invalid reserved timer mask %#x\n",
				     timer->reserved_timer_mask);
	if (of_device_is_compatible(pdev->dev.of_node, "espressif,esp32s31-gptimer") &&
	    of_property_present(pdev->dev.of_node, "espressif,etm-alarm-mask")) {
		ret = of_property_read_u32(pdev->dev.of_node, "espressif,etm-alarm-mask",
					   &timer->etm_alarm_mask);
		if (ret)
			return ret;
		if (!timer->etm_alarm_mask ||
		    timer->etm_alarm_mask & (~GENMASK(1, 0) | timer->reserved_timer_mask))
			return -EINVAL;
		timer->etm_enable =
			devm_platform_ioremap_resource_byname(pdev, "etm-enable");
		if (IS_ERR(timer->etm_enable))
			return PTR_ERR(timer->etm_enable);
		writel(readl(timer->etm_enable) | S31_GPTIMER_ETM_EN, timer->etm_enable);
		ret = devm_add_action_or_reset(&pdev->dev, s31_gptimer_disable_etm, timer);
		if (ret)
			return ret;
	}
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
		count->num_ext = 2;
		if (timer->interrupts) {
			writel(readl(timer->interrupts + GPTIMER_INT_ENA) & ~BIT(i),
			       timer->interrupts + GPTIMER_INT_ENA);
			writel(BIT(i), timer->interrupts + GPTIMER_INT_CLR);
			ret = esp_gptimer_register_alarm(pdev, counter, i);
			if (ret)
				return ret;

		}
		if (timer->interrupts || (timer->etm_alarm_mask & BIT(i))) {
			timer->count_ext[exposed][2] = (struct counter_comp)
				COUNTER_COMP_COUNT_U64("alarm_value", esp_gptimer_alarm_read,
						       esp_gptimer_alarm_write);
			timer->count_ext[exposed][3] = (struct counter_comp)
				COUNTER_COMP_COUNT_BOOL("alarm_enable",
							esp_gptimer_alarm_enable_read,
							esp_gptimer_alarm_enable_write);
			timer->count_ext[exposed][4] = (struct counter_comp)
				COUNTER_COMP_COUNT_BOOL("autoreload", esp_gptimer_autoreload_read,
							esp_gptimer_autoreload_write);
			count->num_ext = ARRAY_SIZE(timer->count_ext[exposed]);
		}

		/* Default to a stopped up-counter with a nominal 1 MHz tick. */
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
	ret = devm_counter_add(&pdev->dev, counter);
	if (ret || !timer->interrupts)
		return ret;
	ret = devm_add_action_or_reset(&pdev->dev, esp_gptimer_disable_alarms,
				       timer);
	if (ret)
		return ret;
	writel(readl(timer->interrupts + GPTIMER_INT_ENA) | timer->alarm_mask,
	       timer->interrupts + GPTIMER_INT_ENA);
	return 0;
}

static const struct of_device_id esp32s31_gptimer_of_match[] = {
	{ .compatible = "espressif,esp32s31-gptimer" },
	{ .compatible = "espressif,esp32p4-gptimer" },
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

MODULE_DESCRIPTION("Espressif timer-group general-purpose timers");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("COUNTER");
