// SPDX-License-Identifier: GPL-2.0-only

#include <linux/completion.h>
#include <linux/esp32s31-radio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>
#include <net/mac802154.h>

#define S31_WPAN_MIN_CHANNEL	11
#define S31_WPAN_MAX_CHANNEL	26
#define S31_WPAN_RX_SLOTS	8
#define S31_WPAN_WAIT_MS	100

struct s31_wpan_frame {
	u8 data[S31_WPAN_MAX_PSDU - S31_WPAN_FCS_BYTES];
	u8 length;
	u8 lqi;
};

struct s31_wpan {
	struct ieee802154_hw *hw;
	struct mutex config_lock;
	spinlock_t event_lock;
	struct work_struct tx_work;
	struct work_struct rx_work;
	struct completion tx_ready;
	struct completion ed_ready;
	struct sk_buff *tx_pending;
	struct s31_wpan_frame rx[S31_WPAN_RX_SLOTS];
	unsigned int rx_head;
	unsigned int rx_tail;
	struct ieee802154_hw_addr_filt filter;
	u8 channel;
	u8 tx_status;
	u32 tx_token;
	u32 ed_token;
	s8 ed_value;
	bool promiscuous;
	bool started;
	bool suspended;
};

static void s31_wpan_event(void *context, u8 type, const u8 *data, u16 length,
			    u8 status, s8 signal)
{
	struct s31_wpan *wpan = context;
	unsigned long flags;
	unsigned int next;

	switch (type) {
	case S31_RADIO_IF_WPAN_RX:
		if (!length || length > sizeof(wpan->rx[0].data))
			return;
		spin_lock_irqsave(&wpan->event_lock, flags);
		next = (wpan->rx_head + 1) % S31_WPAN_RX_SLOTS;
		if (wpan->started && next != wpan->rx_tail) {
			struct s31_wpan_frame *frame = &wpan->rx[wpan->rx_head];

			memcpy(frame->data, data, length);
			frame->length = length;
			frame->lqi = status;
			wpan->rx_head = next;
			schedule_work(&wpan->rx_work);
		}
		spin_unlock_irqrestore(&wpan->event_lock, flags);
		break;
	case S31_RADIO_IF_WPAN_TX:
		if (length == sizeof(u32) && get_unaligned_le32(data) == READ_ONCE(wpan->tx_token)) {
			WRITE_ONCE(wpan->tx_status, status);
			complete(&wpan->tx_ready);
		}
		break;
	case S31_RADIO_IF_WPAN_ED:
		if (length == sizeof(u32) && get_unaligned_le32(data) == READ_ONCE(wpan->ed_token)) {
			WRITE_ONCE(wpan->ed_value, signal);
			complete(&wpan->ed_ready);
		}
		break;
	}
}

static const struct esp32s31_radio_wpan_ops s31_wpan_events = {
	.event = s31_wpan_event,
};

static void s31_wpan_rx_work(struct work_struct *work)
{
	struct s31_wpan *wpan = container_of(work, struct s31_wpan, rx_work);
	unsigned long flags;

	for (;;) {
		struct s31_wpan_frame frame;
		struct sk_buff *skb;

		spin_lock_irqsave(&wpan->event_lock, flags);
		if (!wpan->started || wpan->rx_head == wpan->rx_tail) {
			spin_unlock_irqrestore(&wpan->event_lock, flags);
			break;
		}
		frame = wpan->rx[wpan->rx_tail];
		wpan->rx_tail = (wpan->rx_tail + 1) % S31_WPAN_RX_SLOTS;
		spin_unlock_irqrestore(&wpan->event_lock, flags);

		skb = alloc_skb(frame.length, GFP_KERNEL);
		if (skb) {
			skb_put_data(skb, frame.data, frame.length);
			ieee802154_rx_irqsafe(wpan->hw, skb, frame.lqi);
		}
	}
}

static void s31_wpan_report_tx(struct s31_wpan *wpan, struct sk_buff *skb,
			       int ret, u8 status)
{
	if (ret) {
		ieee802154_xmit_hw_error(wpan->hw, skb);
		return;
	}

	switch (status) {
	case S31_WPAN_TX_OK:
		ieee802154_xmit_complete(wpan->hw, skb, false);
		break;
	case S31_WPAN_TX_CHANNEL_BUSY:
		ieee802154_xmit_error(wpan->hw, skb, IEEE802154_CHANNEL_ACCESS_FAILURE);
		break;
	case S31_WPAN_TX_NO_ACK:
		ieee802154_xmit_error(wpan->hw, skb, IEEE802154_NO_ACK);
		break;
	default:
		ieee802154_xmit_hw_error(wpan->hw, skb);
		break;
	}
}

static void s31_wpan_tx_work(struct work_struct *work)
{
	struct s31_wpan *wpan = container_of(work, struct s31_wpan, tx_work);
	struct s31_wifi_control request = { .operation = S31_RADIO_WPAN_TX };
	unsigned long flags;
	struct sk_buff *skb;
	int ret = -ESHUTDOWN;
	u8 status = S31_WPAN_TX_FAILED;

	mutex_lock(&wpan->config_lock);
	spin_lock_irqsave(&wpan->event_lock, flags);
	skb = wpan->tx_pending;
	spin_unlock_irqrestore(&wpan->event_lock, flags);
	if (skb && READ_ONCE(wpan->started)) {
		request.length = skb->len;
		ret = skb_copy_bits(skb, 0, request.data, skb->len);
		if (!ret) {
			reinit_completion(&wpan->tx_ready);
			request.total = wpan->tx_token + 1;
			WRITE_ONCE(wpan->tx_token, request.total);
			ret = esp32s31_radio_wpan_control(&request);
		}
		if (!ret && !wait_for_completion_timeout(&wpan->tx_ready,
							  msecs_to_jiffies(S31_WPAN_WAIT_MS)))
			ret = -ETIMEDOUT;
		if (!ret)
			status = READ_ONCE(wpan->tx_status);
	}
	spin_lock_irqsave(&wpan->event_lock, flags);
	wpan->tx_pending = NULL;
	spin_unlock_irqrestore(&wpan->event_lock, flags);
	mutex_unlock(&wpan->config_lock);
	if (skb)
		s31_wpan_report_tx(wpan, skb, ret, status);
}

static int s31_wpan_xmit(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	struct s31_wpan *wpan = hw->priv;
	unsigned long flags;
	int ret = 0;

	if (!skb->len || skb->len > S31_WPAN_MAX_PSDU - S31_WPAN_FCS_BYTES)
		return -EMSGSIZE;

	spin_lock_irqsave(&wpan->event_lock, flags);
	if (!wpan->started) {
		ret = -ESHUTDOWN;
	} else if (wpan->tx_pending) {
		ret = -EBUSY;
	} else {
		wpan->tx_pending = skb;
		schedule_work(&wpan->tx_work);
	}
	spin_unlock_irqrestore(&wpan->event_lock, flags);
	return ret;
}

static int s31_wpan_config(struct s31_wpan *wpan, u32 operation,
			    const struct ieee802154_hw_addr_filt *filter, bool promisc)
{
	struct s31_wifi_control request = {
		.operation = operation,
		.channel = wpan->channel,
		.field = le16_to_cpu(filter->pan_id),
		.offset = le16_to_cpu(filter->short_addr),
		.total = filter->pan_coord,
		.hidden = promisc,
	};

	memcpy(request.data, &filter->ieee_addr, sizeof(filter->ieee_addr));
	return esp32s31_radio_wpan_control(&request);
}

static int s31_wpan_start(struct ieee802154_hw *hw)
{
	struct s31_wpan *wpan = hw->priv;
	unsigned long flags;
	int ret;

	mutex_lock(&wpan->config_lock);
	ret = s31_wpan_config(wpan, S31_RADIO_WPAN_START, &wpan->filter, wpan->promiscuous);
	if (!ret) {
		spin_lock_irqsave(&wpan->event_lock, flags);
		wpan->started = true;
		spin_unlock_irqrestore(&wpan->event_lock, flags);
	}
	mutex_unlock(&wpan->config_lock);
	return ret;
}

static int s31_wpan_disable(struct ieee802154_hw *hw)
{
	struct s31_wpan *wpan = hw->priv;
	struct s31_wifi_control request = { .operation = S31_RADIO_WPAN_STOP };
	unsigned long flags;
	struct sk_buff *skb;
	bool started;
	int ret;

	spin_lock_irqsave(&wpan->event_lock, flags);
	started = wpan->started;
	wpan->started = false;
	spin_unlock_irqrestore(&wpan->event_lock, flags);
	if (!started)
		return 0;

	cancel_work_sync(&wpan->tx_work);
	cancel_work_sync(&wpan->rx_work);
	spin_lock_irqsave(&wpan->event_lock, flags);
	skb = wpan->tx_pending;
	wpan->tx_pending = NULL;
	wpan->rx_head = wpan->rx_tail = 0;
	spin_unlock_irqrestore(&wpan->event_lock, flags);
	if (skb)
		ieee802154_xmit_hw_error(hw, skb);

	mutex_lock(&wpan->config_lock);
	ret = esp32s31_radio_wpan_control(&request);
	mutex_unlock(&wpan->config_lock);
	return ret;
}

static void s31_wpan_stop(struct ieee802154_hw *hw)
{
	int ret = s31_wpan_disable(hw);

	if (ret)
		dev_err(hw->parent, "failed to stop WPAN radio (%d)\n", ret);
}

static int s31_wpan_ed(struct ieee802154_hw *hw, u8 *level)
{
	struct s31_wpan *wpan = hw->priv;
	struct s31_wifi_control request = { .operation = S31_RADIO_WPAN_ED };
	int ret;

	mutex_lock(&wpan->config_lock);
	if (!READ_ONCE(wpan->started)) {
		ret = -ENETDOWN;
		goto out;
	}
	reinit_completion(&wpan->ed_ready);
	request.total = wpan->ed_token + 1;
	WRITE_ONCE(wpan->ed_token, request.total);
	ret = esp32s31_radio_wpan_control(&request);
	if (!ret && !wait_for_completion_timeout(&wpan->ed_ready,
						  msecs_to_jiffies(S31_WPAN_WAIT_MS)))
		ret = -ETIMEDOUT;
	if (!ret)
		*level = clamp_t(int, (READ_ONCE(wpan->ed_value) + 100) * 255 / 80, 0, 255);
out:
	mutex_unlock(&wpan->config_lock);
	return ret;
}

static int s31_wpan_set_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
	struct s31_wpan *wpan = hw->priv;
	struct s31_wifi_control request = {
		.operation = S31_RADIO_WPAN_CHANNEL,
		.channel = channel,
	};
	int ret = 0;

	if (page || channel < S31_WPAN_MIN_CHANNEL || channel > S31_WPAN_MAX_CHANNEL)
		return -EINVAL;
	mutex_lock(&wpan->config_lock);
	if (READ_ONCE(wpan->started))
		ret = esp32s31_radio_wpan_control(&request);
	if (!ret)
		wpan->channel = channel;
	mutex_unlock(&wpan->config_lock);
	return ret;
}

static int s31_wpan_set_filter(struct ieee802154_hw *hw,
			      struct ieee802154_hw_addr_filt *filter,
			      unsigned long changed)
{
	struct s31_wpan *wpan = hw->priv;
	struct ieee802154_hw_addr_filt next;
	int ret = 0;

	mutex_lock(&wpan->config_lock);
	next = wpan->filter;
	if (changed & IEEE802154_AFILT_PANID_CHANGED)
		next.pan_id = filter->pan_id;
	if (changed & IEEE802154_AFILT_SADDR_CHANGED)
		next.short_addr = filter->short_addr;
	if (changed & IEEE802154_AFILT_IEEEADDR_CHANGED)
		next.ieee_addr = filter->ieee_addr;
	if (changed & IEEE802154_AFILT_PANC_CHANGED)
		next.pan_coord = filter->pan_coord;

	if (READ_ONCE(wpan->started))
		ret = s31_wpan_config(wpan, S31_RADIO_WPAN_FILTER, &next, wpan->promiscuous);
	if (!ret)
		wpan->filter = next;
	mutex_unlock(&wpan->config_lock);
	return ret;
}

static int s31_wpan_set_promisc(struct ieee802154_hw *hw, bool enable)
{
	struct s31_wpan *wpan = hw->priv;
	int ret = 0;

	mutex_lock(&wpan->config_lock);
	if (READ_ONCE(wpan->started))
		ret = s31_wpan_config(wpan, S31_RADIO_WPAN_FILTER, &wpan->filter, enable);
	if (!ret)
		wpan->promiscuous = enable;
	mutex_unlock(&wpan->config_lock);
	return ret;
}

static const struct ieee802154_ops s31_wpan_ops = {
	.owner = THIS_MODULE,
	.start = s31_wpan_start,
	.stop = s31_wpan_stop,
	.xmit_async = s31_wpan_xmit,
	.ed = s31_wpan_ed,
	.set_channel = s31_wpan_set_channel,
	.set_hw_addr_filt = s31_wpan_set_filter,
	.set_promiscuous_mode = s31_wpan_set_promisc,
};

static int s31_wpan_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s31_wifi_control request = { .operation = S31_RADIO_WPAN_CAP };
	struct device_node *radio_node __free(device_node) =
		of_parse_phandle(dev->of_node, "espressif,radio", 0);
	struct platform_device *radio;
	struct ieee802154_hw *hw;
	struct s31_wpan *wpan;
	struct device_link *link;
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

	ret = esp32s31_radio_wpan_control(&request);
	if (ret)
		return dev_err_probe(dev, ret == -ENODEV ? -EPROBE_DEFER : ret,
				     "IEEE 802.15.4 service unavailable\n");
	if (request.field != S31_RADIO_WPAN_ABI)
		return -EPROTONOSUPPORT;

	hw = ieee802154_alloc_hw(sizeof(*wpan), &s31_wpan_ops);
	if (!hw)
		return -ENOMEM;
	wpan = hw->priv;
	wpan->hw = hw;
	mutex_init(&wpan->config_lock);
	spin_lock_init(&wpan->event_lock);
	INIT_WORK(&wpan->rx_work, s31_wpan_rx_work);
	INIT_WORK(&wpan->tx_work, s31_wpan_tx_work);
	init_completion(&wpan->tx_ready);
	init_completion(&wpan->ed_ready);

	hw->parent = dev;
	hw->flags = IEEE802154_HW_OMIT_CKSUM | IEEE802154_HW_AFILT | IEEE802154_HW_PROMISCUOUS;
	hw->phy->supported.channels[0] = GENMASK(S31_WPAN_MAX_CHANNEL, S31_WPAN_MIN_CHANNEL);
	hw->phy->current_page = 0;
	hw->phy->current_channel = S31_WPAN_MIN_CHANNEL;
	wpan->channel = hw->phy->current_channel;
	ieee802154_random_extended_addr(&hw->phy->perm_extended_addr);
	wpan->filter.ieee_addr = hw->phy->perm_extended_addr;
	wpan->filter.pan_id = cpu_to_le16(IEEE802154_PAN_ID_BROADCAST);
	wpan->filter.short_addr = cpu_to_le16(IEEE802154_ADDR_SHORT_BROADCAST);

	ret = esp32s31_radio_wpan_register(&s31_wpan_events, wpan);
	if (ret)
		goto free_hw;
	ret = ieee802154_register_hw(hw);
	if (ret)
		goto unregister;
	platform_set_drvdata(pdev, hw);
	return 0;

unregister:
	esp32s31_radio_wpan_unregister(&s31_wpan_events);
free_hw:
	ieee802154_free_hw(hw);
	return ret;
}

static void s31_wpan_remove(struct platform_device *pdev)
{
	struct ieee802154_hw *hw = platform_get_drvdata(pdev);

	ieee802154_unregister_hw(hw);
	esp32s31_radio_wpan_unregister(&s31_wpan_events);
	ieee802154_free_hw(hw);
}

static int s31_wpan_suspend(struct device *dev)
{
	struct ieee802154_hw *hw = dev_get_drvdata(dev);
	struct s31_wpan *wpan = hw->priv;

	wpan->suspended = READ_ONCE(wpan->started);
	return s31_wpan_disable(hw);
}

static int s31_wpan_resume(struct device *dev)
{
	struct ieee802154_hw *hw = dev_get_drvdata(dev);
	struct s31_wpan *wpan = hw->priv;

	return wpan->suspended ? s31_wpan_start(hw) : 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(s31_wpan_pm, s31_wpan_suspend, s31_wpan_resume);

static const struct of_device_id s31_wpan_of[] = {
	{ .compatible = "espressif,esp32s31-wpan" },
	{ }
};
MODULE_DEVICE_TABLE(of, s31_wpan_of);

static struct platform_driver s31_wpan_driver = {
	.probe = s31_wpan_probe,
	.remove = s31_wpan_remove,
	.driver = {
		.name = "esp32s31-wpan",
		.of_match_table = s31_wpan_of,
		.pm = pm_sleep_ptr(&s31_wpan_pm),
	},
};
module_platform_driver(s31_wpan_driver);

MODULE_DESCRIPTION("Espressif ESP32-S31 IEEE 802.15.4 radio");
MODULE_LICENSE("GPL");
