// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 cfg80211 fullmac front end. */

#include <linux/etherdevice.h>
#include <linux/esp32s31-radio.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/workqueue.h>
#include <net/cfg80211.h>

struct s31_wifi {
	struct wiphy *wiphy;
	struct wireless_dev wdev;
	struct net_device *netdev;
	struct cfg80211_scan_request *scan_request;
	struct delayed_work register_work;
};

#define S31_CHANNEL(_ch, _freq) { \
	.band = NL80211_BAND_2GHZ, .center_freq = (_freq), \
	.hw_value = (_ch), .max_power = 20 }

static struct ieee80211_channel s31_channels[] = {
	S31_CHANNEL(1, 2412), S31_CHANNEL(2, 2417),
	S31_CHANNEL(3, 2422), S31_CHANNEL(4, 2427),
	S31_CHANNEL(5, 2432), S31_CHANNEL(6, 2437),
	S31_CHANNEL(7, 2442), S31_CHANNEL(8, 2447),
	S31_CHANNEL(9, 2452), S31_CHANNEL(10, 2457),
	S31_CHANNEL(11, 2462), S31_CHANNEL(12, 2467),
	S31_CHANNEL(13, 2472), S31_CHANNEL(14, 2484),
};

static struct ieee80211_rate s31_rates[] = {
	{ .bitrate = 10 }, { .bitrate = 20 }, { .bitrate = 55 },
	{ .bitrate = 110 }, { .bitrate = 60 }, { .bitrate = 90 },
	{ .bitrate = 120 }, { .bitrate = 180 }, { .bitrate = 240 },
	{ .bitrate = 360 }, { .bitrate = 480 }, { .bitrate = 540 },
};

static struct ieee80211_supported_band s31_band_2ghz = {
	.band = NL80211_BAND_2GHZ,
	.channels = s31_channels,
	.n_channels = ARRAY_SIZE(s31_channels),
	.bitrates = s31_rates,
	.n_bitrates = ARRAY_SIZE(s31_rates),
	.ht_cap = {
		.ht_supported = true,
		.cap = IEEE80211_HT_CAP_SGI_20,
		.mcs = { .rx_mask = { 0xff },
			 .tx_params = IEEE80211_HT_MCS_TX_DEFINED },
	},
};

static void s31_wifi_scan_complete(void *context, int status,
				   const struct esp32s31_radio_wifi_ap *aps,
				   size_t count)
{
	struct s31_wifi *wifi = context;
	struct cfg80211_scan_info info = { .aborted = status != 0 };
	size_t i;

	for (i = 0; !status && i < count; i++) {
		const struct esp32s31_radio_wifi_ap *ap = &aps[i];
		struct ieee80211_channel *channel;
		struct cfg80211_bss *bss;
		u8 ies[2 + IEEE80211_MAX_SSID_LEN];
		u16 capability = WLAN_CAPABILITY_ESS;

		if (!ap->channel || ap->channel > ARRAY_SIZE(s31_channels))
			continue;
		channel = &s31_channels[ap->channel - 1];
		ies[0] = WLAN_EID_SSID;
		ies[1] = ap->ssid_length;
		memcpy(ies + 2, ap->ssid, ap->ssid_length);
		if (ap->authmode)
			capability |= WLAN_CAPABILITY_PRIVACY;
		bss = cfg80211_inform_bss(wifi->wiphy, channel,
					   CFG80211_BSS_FTYPE_UNKNOWN,
					   ap->bssid, 0, capability, 100,
					   ies, ap->ssid_length + 2,
					   ap->signal * 100, GFP_KERNEL);
		if (bss)
			cfg80211_put_bss(wifi->wiphy, bss);
	}
	if (wifi->scan_request) {
		cfg80211_scan_done(wifi->scan_request, &info);
		wifi->scan_request = NULL;
	}
	pr_info("esp32s31-wifi: scan complete status=%d aps=%zu\n",
		status, count);
}

static const struct esp32s31_radio_wifi_ops s31_radio_wifi_ops = {
	.scan_complete = s31_wifi_scan_complete,
};

static int s31_cfg_scan(struct wiphy *wiphy,
			struct cfg80211_scan_request *request)
{
	struct s31_wifi *wifi = wiphy_priv(wiphy);
	int ret;

	if (wifi->scan_request)
		return -EBUSY;
	wifi->scan_request = request;
	ret = esp32s31_radio_wifi_scan();
	if (ret)
		wifi->scan_request = NULL;
	return ret;
}

static const struct cfg80211_ops s31_cfg80211_ops = {
	.scan = s31_cfg_scan,
};

static netdev_tx_t s31_wifi_xmit(struct sk_buff *skb, struct net_device *dev)
{
	dev->stats.tx_dropped++;
	dev_kfree_skb(skb);
	return NETDEV_TX_OK;
}

static int s31_wifi_open(struct net_device *dev)
{
	netif_start_queue(dev);
	netif_carrier_off(dev);
	return 0;
}

static int s31_wifi_stop(struct net_device *dev)
{
	netif_stop_queue(dev);
	return 0;
}

static const struct net_device_ops s31_netdev_ops = {
	.ndo_open = s31_wifi_open,
	.ndo_stop = s31_wifi_stop,
	.ndo_start_xmit = s31_wifi_xmit,
};

static void s31_wifi_setup(struct net_device *dev)
{
	ether_setup(dev);
	dev->netdev_ops = &s31_netdev_ops;
	dev->needs_free_netdev = true;
}

static void s31_wifi_register_workfn(struct work_struct *work)
{
	struct s31_wifi *wifi = container_of(to_delayed_work(work),
					       struct s31_wifi, register_work);
	struct net_device *netdev;
	int ret;

	ret = esp32s31_radio_wifi_register(&s31_radio_wifi_ops, wifi);
	if (ret == -EAGAIN) {
		schedule_delayed_work(&wifi->register_work, msecs_to_jiffies(100));
		return;
	}
	if (ret)
		return;

	wifi->wiphy->max_scan_ssids = 4;
	wifi->wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;
	wifi->wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION);
	wifi->wiphy->bands[NL80211_BAND_2GHZ] = &s31_band_2ghz;
	ret = wiphy_register(wifi->wiphy);
	if (ret)
		return;

	netdev = alloc_netdev(0, "wlan%d", NET_NAME_ENUM, s31_wifi_setup);
	if (!netdev)
		return;
	wifi->netdev = netdev;
	wifi->wdev.wiphy = wifi->wiphy;
	wifi->wdev.iftype = NL80211_IFTYPE_STATION;
	wifi->wdev.netdev = netdev;
	netdev->ieee80211_ptr = &wifi->wdev;
	SET_NETDEV_DEV(netdev, wiphy_dev(wifi->wiphy));
	eth_hw_addr_random(netdev);
	/*
	 * This worker runs outside a cfg80211 callback and does not hold either
	 * RTNL or the wiphy mutex.  register_netdev() supplies the RTNL locking;
	 * the cfg80211 netdev notifier attaches the already initialized wdev.
	 */
	ret = register_netdev(netdev);
	if (ret) {
		free_netdev(netdev);
		wifi->netdev = NULL;
		return;
	}
	pr_info("esp32s31-wifi: registered %s fullmac scan interface\n",
		netdev->name);
}

static int __init s31_wifi_init(void)
{
	struct wiphy *wiphy;
	struct s31_wifi *wifi;

	wiphy = wiphy_new(&s31_cfg80211_ops, sizeof(*wifi));
	if (!wiphy)
		return -ENOMEM;
	wifi = wiphy_priv(wiphy);
	wifi->wiphy = wiphy;
	INIT_DELAYED_WORK(&wifi->register_work, s31_wifi_register_workfn);
	schedule_delayed_work(&wifi->register_work, 0);
	return 0;
}
late_initcall(s31_wifi_init);
