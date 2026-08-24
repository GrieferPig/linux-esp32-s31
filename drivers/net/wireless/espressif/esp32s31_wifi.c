// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 cfg80211 fullmac front end. */

#include <linux/etherdevice.h>
#include <linux/esp32s31-radio.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/smp.h>
#include <linux/workqueue.h>
#include <net/cfg80211.h>

#define S31_WIFI_RX_SKB_POOL_TARGET	128
#define S31_WIFI_RX_SKB_POOL_LOW		64
#define S31_WIFI_RX_RING_SIZE		512
#define S31_WIFI_RX_RING_MASK		(S31_WIFI_RX_RING_SIZE - 1)

struct s31_wifi_skb_ring {
	struct sk_buff *slot[S31_WIFI_RX_RING_SIZE];
	u32 head ____cacheline_aligned_in_smp;
	u32 tail ____cacheline_aligned_in_smp;
};

struct s31_wifi {
	struct wiphy *wiphy;
	struct wireless_dev wdev;
	struct net_device *netdev;
	struct cfg80211_scan_request *scan_request;
	struct delayed_work register_work;
	struct napi_struct napi;
	struct s31_wifi_skb_ring rx_skb_pool;
	struct s31_wifi_skb_ring rx_ready_queue;
	struct work_struct rx_refill_work;
	u8 connect_ssid[IEEE80211_MAX_SSID_LEN];
	u8 connect_ssid_len;
	bool connect_privacy;
	bool connecting;
	bool connected;
	u32 napi_polls;
	u32 napi_packets;
	u32 napi_budget_exhausted;
	u32 napi_alloc_drops;
	u32 napi_dequeue_errors;
	s32 napi_last_dequeue_error;
	u32 napi_max_batch;
	u32 rx_pool_empty;
	u32 rx_refill_failures;
	u32 rx_work_refills;
	u32 rx_wrong_cpu;
	u32 napi_wrong_cpu;
	u32 refill_wrong_cpu;
};

/* NAPI cannot sleep in the page allocator.  This no-MMU target was already
 * observed to reject full-sized GFP_ATOMIC skb allocations while megabytes of
 * ordinary memory remained available.  Keep a process-context reserve ahead
 * of the hardware-facing SRAM ring; refill work changes allocation context,
 * not packet-delivery context. */
/* Allocation runs on hart 1 so it cannot steal cycles from the closed radio.
 * The 128/64 reserve absorbs normal BA bursts while that CFS worker refills. */
/* rx_skb_pool is an SPSC handoff from the hart-1 allocator to the hart-0 closed
 * callback.  Its producer/consumer indices occupy separate cache lines and it
 * never takes a cross-hart lock.  rx_ready_queue remains entirely on hart 0,
 * running from that callback to NAPI.  Release/acquire publication keeps the
 * slot visible before its monotonically increasing index. */
static unsigned int s31_wifi_ring_len(const struct s31_wifi_skb_ring *ring)
{
	return smp_load_acquire(&ring->head) - READ_ONCE(ring->tail);
}

static bool s31_wifi_ring_push(struct s31_wifi_skb_ring *ring,
			       struct sk_buff *skb)
{
	u32 head = READ_ONCE(ring->head);

	if (head - smp_load_acquire(&ring->tail) >= S31_WIFI_RX_RING_SIZE)
		return false;
	WRITE_ONCE(ring->slot[head & S31_WIFI_RX_RING_MASK], skb);
	smp_store_release(&ring->head, head + 1);
	return true;
}

static struct sk_buff *s31_wifi_ring_pop(struct s31_wifi_skb_ring *ring)
{
	u32 tail = READ_ONCE(ring->tail);
	struct sk_buff *skb;

	if (tail == smp_load_acquire(&ring->head))
		return NULL;
	skb = READ_ONCE(ring->slot[tail & S31_WIFI_RX_RING_MASK]);
	WRITE_ONCE(ring->slot[tail & S31_WIFI_RX_RING_MASK], NULL);
	smp_store_release(&ring->tail, tail + 1);
	return skb;
}

static void s31_wifi_ring_purge(struct s31_wifi_skb_ring *ring)
{
	struct sk_buff *skb;

	while ((skb = s31_wifi_ring_pop(ring)))
		dev_kfree_skb(skb);
}

static const u32 s31_cipher_suites[] = {
	WLAN_CIPHER_SUITE_CCMP,
};

static const u32 s31_akm_suites[] = {
	WLAN_AKM_SUITE_PSK,
	WLAN_AKM_SUITE_SAE,
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
		pr_info("esp32s31-wifi: BSS %pM channel=%u authmode=%u ssid=%.*s\n",
			ap->bssid, ap->channel, ap->authmode, ap->ssid_length,
			ap->ssid);
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

static void s31_wifi_connected(void *context, int status, const u8 *bssid,
			       u8 channel)
{
	struct s31_wifi *wifi = context;
	struct cfg80211_bss *bss = NULL;

	if (!wifi->connecting)
		return;
	wifi->connecting = false;
	wifi->connected = !status;
	pr_info("esp32s31-wifi: association complete status=%d channel=%u\n",
		status, channel);
	if (!status) {
		u8 ies[2 + IEEE80211_MAX_SSID_LEN];
		u16 capability = WLAN_CAPABILITY_ESS;

		if (wifi->connect_privacy)
			capability |= WLAN_CAPABILITY_PRIVACY;
		ies[0] = WLAN_EID_SSID;
		ies[1] = wifi->connect_ssid_len;
		memcpy(ies + 2, wifi->connect_ssid, wifi->connect_ssid_len);
		if (channel && channel <= ARRAY_SIZE(s31_channels))
			bss = cfg80211_inform_bss(wifi->wiphy,
				&s31_channels[channel - 1],
				CFG80211_BSS_FTYPE_UNKNOWN, bssid, 0,
				capability, 100, ies,
				wifi->connect_ssid_len + 2, 0, GFP_KERNEL);
		netif_carrier_on(wifi->netdev);
	}
	/* __cfg80211_connect_result() uses links[0].bssid as the connected
	 * address even when an exact BSS is supplied, so keep both populated. */
	cfg80211_connect_bss(wifi->netdev, bssid, bss,
		NULL, 0, NULL, 0,
		status ? WLAN_STATUS_UNSPECIFIED_FAILURE : WLAN_STATUS_SUCCESS,
		GFP_KERNEL, NL80211_TIMEOUT_UNSPECIFIED);
}

static void s31_wifi_disconnected(void *context, u16 reason)
{
	struct s31_wifi *wifi = context;

	pr_info("esp32s31-wifi: disconnected reason=%u connecting=%u\n",
		reason, wifi->connecting);
	pr_info("esp32s31-wifi: NAPI polls=%u packets=%u max_batch=%u budget_exhausted=%u alloc_drops=%u dequeue_errors=%u last_error=%d pool_empty=%u refill_fail=%u pool_len=%u work_refills=%u wrong_cpu(rx/napi/refill)=%u/%u/%u\n",
		READ_ONCE(wifi->napi_polls), READ_ONCE(wifi->napi_packets),
		READ_ONCE(wifi->napi_max_batch),
		READ_ONCE(wifi->napi_budget_exhausted),
		READ_ONCE(wifi->napi_alloc_drops),
		READ_ONCE(wifi->napi_dequeue_errors),
		READ_ONCE(wifi->napi_last_dequeue_error),
		READ_ONCE(wifi->rx_pool_empty),
		READ_ONCE(wifi->rx_refill_failures),
		s31_wifi_ring_len(&wifi->rx_skb_pool),
		READ_ONCE(wifi->rx_work_refills),
		READ_ONCE(wifi->rx_wrong_cpu),
		READ_ONCE(wifi->napi_wrong_cpu),
		READ_ONCE(wifi->refill_wrong_cpu));
	netif_carrier_off(wifi->netdev);
	if (wifi->connecting) {
		wifi->connecting = false;
		cfg80211_connect_result(wifi->netdev, NULL, NULL, 0, NULL, 0,
					WLAN_STATUS_UNSPECIFIED_FAILURE, GFP_KERNEL);
	} else if (wifi->connected) {
		wifi->connected = false;
		cfg80211_disconnected(wifi->netdev, reason, NULL, 0, false,
				      GFP_KERNEL);
	}
}

static void s31_wifi_rx_ready(void *context)
{
	struct s31_wifi *wifi = context;

	if (!wifi || !wifi->netdev || !netif_running(wifi->netdev))
		return;
	if (in_hardirq())
		napi_schedule_irqoff(&wifi->napi);
	else
		napi_schedule(&wifi->napi);
}

static void s31_wifi_schedule_rx_refill(struct s31_wifi *wifi);

static int s31_wifi_rx_copy(void *context, const u8 *frame, size_t length)
{
	struct s31_wifi *wifi = context;
	struct sk_buff *skb;

	if (!wifi || !wifi->netdev || !netif_running(wifi->netdev) ||
	    !frame || length < ETH_HLEN ||
	    length > ESP32S31_RADIO_WIFI_FRAME_MAX)
		return -EINVAL;
	if (unlikely(raw_smp_processor_id() != 0))
		wifi->rx_wrong_cpu++;
	skb = s31_wifi_ring_pop(&wifi->rx_skb_pool);
	if (unlikely(!skb)) {
		wifi->rx_pool_empty++;
		wifi->napi_alloc_drops++;
		wifi->netdev->stats.rx_dropped++;
		s31_wifi_schedule_rx_refill(wifi);
		return -ENOSPC;
	}
	memcpy(skb_put(skb, length), frame, length);
	if (unlikely(!s31_wifi_ring_push(&wifi->rx_ready_queue, skb))) {
		wifi->napi_alloc_drops++;
		wifi->netdev->stats.rx_dropped++;
		dev_kfree_skb_any(skb);
		return -ENOSPC;
	}
	return 0;
}

static struct sk_buff *s31_wifi_alloc_rx_skb(struct s31_wifi *wifi,
					     gfp_t gfp)
{
	struct sk_buff *skb;

	skb = __netdev_alloc_skb(wifi->netdev,
		ESP32S31_RADIO_WIFI_FRAME_MAX + NET_IP_ALIGN, gfp);
	if (skb)
		skb_reserve(skb, NET_IP_ALIGN);
	return skb;
}

static void s31_wifi_refill_rx_pool(struct s31_wifi *wifi, gfp_t gfp)
{
	while (s31_wifi_ring_len(&wifi->rx_skb_pool) <
	       S31_WIFI_RX_SKB_POOL_TARGET) {
		struct sk_buff *skb = s31_wifi_alloc_rx_skb(wifi, gfp);

		if (!skb) {
			wifi->rx_refill_failures++;
			break;
		}
		if (unlikely(!s31_wifi_ring_push(&wifi->rx_skb_pool, skb))) {
			dev_kfree_skb(skb);
			break;
		}
		wifi->rx_work_refills++;
		if (gfpflags_allow_blocking(gfp))
			cond_resched();
	}
}

static void s31_wifi_rx_refill_workfn(struct work_struct *work)
{
	struct s31_wifi *wifi = container_of(work, struct s31_wifi,
					     rx_refill_work);

	if (unlikely(raw_smp_processor_id() != 1))
		wifi->refill_wrong_cpu++;
	if (wifi->netdev && netif_running(wifi->netdev))
		s31_wifi_refill_rx_pool(wifi, GFP_KERNEL);
}

static void s31_wifi_schedule_rx_refill(struct s31_wifi *wifi)
{
	/* Only this hart-1 worker produces pool entries; the closed callback is the
	 * sole hart-0 consumer.  The SPSC ring has no shared spinlock. */
	schedule_work_on(1, &wifi->rx_refill_work);
}

static int s31_wifi_napi_poll(struct napi_struct *napi, int budget)
{
	struct s31_wifi *wifi = container_of(napi, struct s31_wifi, napi);
	struct net_device *dev = wifi->netdev;
	int work_done = 0;

	if (unlikely(raw_smp_processor_id() != 0))
		wifi->napi_wrong_cpu++;
	while (work_done < budget) {
		struct sk_buff *skb = s31_wifi_ring_pop(&wifi->rx_ready_queue);

		if (!skb)
			break;
		esp32s31_radio_wifi_rx_complete();
		skb->protocol = eth_type_trans(skb, dev);
		dev->stats.rx_packets++;
		dev->stats.rx_bytes += skb->len;
		napi_gro_receive(napi, skb);
		work_done++;
	}
	if (s31_wifi_ring_len(&wifi->rx_skb_pool) <= S31_WIFI_RX_SKB_POOL_LOW)
		s31_wifi_schedule_rx_refill(wifi);

	wifi->napi_polls++;
	wifi->napi_packets += work_done;
	wifi->napi_max_batch = max_t(u32, wifi->napi_max_batch, work_done);
	if (work_done == budget) {
		wifi->napi_budget_exhausted++;
		return budget;
	}

	if (napi_complete_done(napi, work_done) &&
	    s31_wifi_ring_len(&wifi->rx_ready_queue))
		napi_schedule(napi);

	return work_done;
}

static void s31_wifi_tx_wakeup(void *context)
{
	struct s31_wifi *wifi = context;

	if (wifi && wifi->netdev)
		netif_wake_queue(wifi->netdev);
}

static const struct esp32s31_radio_wifi_ops s31_radio_wifi_ops = {
	.scan_complete = s31_wifi_scan_complete,
	.connected = s31_wifi_connected,
	.disconnected = s31_wifi_disconnected,
	.rx_copy = s31_wifi_rx_copy,
	.rx_ready = s31_wifi_rx_ready,
	.tx_wakeup = s31_wifi_tx_wakeup,
};

/* The closed radio runtime, its worker and all routed device IRQs live on
 * hart 0.  Keep cfg80211 control submission on the same hart as well; data
 * path callers remain free to run on either CPU. */
static long s31_wifi_scan_cpu0(void *unused)
{
	return esp32s31_radio_wifi_scan();
}

static long s31_wifi_connect_cpu0(void *arg)
{
	return esp32s31_radio_wifi_connect(arg);
}

static long s31_wifi_disconnect_cpu0(void *arg)
{
	return esp32s31_radio_wifi_disconnect(*(u16 *)arg);
}

static int s31_cfg_scan(struct wiphy *wiphy,
			struct cfg80211_scan_request *request)
{
	struct s31_wifi *wifi = wiphy_priv(wiphy);
	int ret;

	if (wifi->scan_request)
		return -EBUSY;
	wifi->scan_request = request;
	ret = work_on_cpu(0, s31_wifi_scan_cpu0, NULL);
	if (ret)
		wifi->scan_request = NULL;
	return ret;
}

static int s31_cfg_connect(struct wiphy *wiphy, struct net_device *dev,
			   struct cfg80211_connect_params *request)
{
	struct s31_wifi *wifi = wiphy_priv(wiphy);
	struct esp32s31_radio_wifi_connect_params params = { };

	(void)dev;
	if (!request->ssid || !request->ssid_len || request->ssid_len > 32)
		return -EINVAL;
	if (wifi->connecting || wifi->connected)
		return -EBUSY;
	if (request->privacy && !request->crypto.psk &&
	    !request->crypto.sae_pwd)
		return -EOPNOTSUPP;
	memcpy(params.ssid, request->ssid, request->ssid_len);
	params.ssid_length = request->ssid_len;
	memcpy(wifi->connect_ssid, request->ssid, request->ssid_len);
	wifi->connect_ssid_len = request->ssid_len;
	wifi->connect_privacy = request->privacy;
	if (request->bssid) {
		memcpy(params.bssid, request->bssid, ETH_ALEN);
		params.has_bssid = true;
	}
	if (request->channel)
		params.channel = ieee80211_frequency_to_channel(
					request->channel->center_freq);
	if (request->crypto.psk) {
		memcpy(params.psk, request->crypto.psk, sizeof(params.psk));
		params.has_psk = true;
	}
	if (request->crypto.sae_pwd) {
		if (!request->crypto.sae_pwd_len ||
		    request->crypto.sae_pwd_len > sizeof(params.password))
			return -EINVAL;
		memcpy(params.password, request->crypto.sae_pwd,
		       request->crypto.sae_pwd_len);
		params.password_length = request->crypto.sae_pwd_len;
		params.has_password = true;
	}
	wifi->connecting = true;
	{
		int ret = work_on_cpu(0, s31_wifi_connect_cpu0, &params);

		if (!ret)
			return 0;
		wifi->connecting = false;
		return ret;
	}
}

static int s31_cfg_disconnect(struct wiphy *wiphy, struct net_device *dev,
			      u16 reason)
{
	struct s31_wifi *wifi = wiphy_priv(wiphy);

	(void)dev;
	if (!wifi->connecting && !wifi->connected)
		return 0;
	return work_on_cpu(0, s31_wifi_disconnect_cpu0, &reason);
}

static const struct cfg80211_ops s31_cfg80211_ops = {
	.scan = s31_cfg_scan,
	.connect = s31_cfg_connect,
	.disconnect = s31_cfg_disconnect,
};

static netdev_tx_t s31_wifi_xmit(struct sk_buff *skb, struct net_device *dev)
{
	int ret = esp32s31_radio_wifi_send(skb->data, skb->len);

	if (ret == -ENOSPC) {
		/* Apply real netdev backpressure instead of letting the qdisc spin on
		 * NETDEV_TX_BUSY while the radio worker is trying to drain the ring.
		 * Recheck after stopping to close the race with the worker's wake. */
		netif_stop_queue(dev);
		if (esp32s31_radio_wifi_tx_has_space())
			netif_wake_queue(dev);
		return NETDEV_TX_BUSY;
	}
	if (ret) {
		dev->stats.tx_dropped++;
	} else {
		dev->stats.tx_packets++;
		dev->stats.tx_bytes += skb->len;
		/* Stop as soon as this enqueue consumes the final slot.  This avoids
		 * one guaranteed BUSY/requeue round trip per full ring; the worker
		 * wakes the queue after it advances the tail. */
		if (!esp32s31_radio_wifi_tx_has_space()) {
			netif_stop_queue(dev);
			if (esp32s31_radio_wifi_tx_has_space())
				netif_wake_queue(dev);
		}
	}
	dev_kfree_skb(skb);
	return NETDEV_TX_OK;
}

static int s31_wifi_open(struct net_device *dev)
{
	struct s31_wifi *wifi = wiphy_priv(dev->ieee80211_ptr->wiphy);

	s31_wifi_refill_rx_pool(wifi, GFP_KERNEL);
	if (!s31_wifi_ring_len(&wifi->rx_skb_pool))
		return -ENOMEM;
	napi_enable(&wifi->napi);
	netif_start_queue(dev);
	netif_carrier_off(dev);
	if (s31_wifi_ring_len(&wifi->rx_ready_queue))
		napi_schedule(&wifi->napi);
	return 0;
}

static int s31_wifi_stop(struct net_device *dev)
{
	struct s31_wifi *wifi = wiphy_priv(dev->ieee80211_ptr->wiphy);
	struct sk_buff *skb;

	netif_stop_queue(dev);
	napi_disable(&wifi->napi);
	cancel_work_sync(&wifi->rx_refill_work);
	/* Keep the platform-side throttle count paired with every frame removed
	 * without passing through NAPI (for example during interface shutdown). */
	while ((skb = s31_wifi_ring_pop(&wifi->rx_ready_queue))) {
		esp32s31_radio_wifi_rx_complete();
		dev_kfree_skb(skb);
	}
	s31_wifi_ring_purge(&wifi->rx_skb_pool);
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
	u8 mac[ETH_ALEN];
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
	wifi->wiphy->cipher_suites = s31_cipher_suites;
	wifi->wiphy->n_cipher_suites = ARRAY_SIZE(s31_cipher_suites);
	wifi->wiphy->akm_suites = s31_akm_suites;
	wifi->wiphy->n_akm_suites = ARRAY_SIZE(s31_akm_suites);
	wifi->wiphy->bands[NL80211_BAND_2GHZ] = &s31_band_2ghz;
	wiphy_ext_feature_set(wifi->wiphy,
			      NL80211_EXT_FEATURE_4WAY_HANDSHAKE_STA_PSK);
	wiphy_ext_feature_set(wifi->wiphy, NL80211_EXT_FEATURE_SAE_OFFLOAD);
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
	INIT_WORK(&wifi->rx_refill_work, s31_wifi_rx_refill_workfn);
	netif_napi_add_weight(netdev, &wifi->napi, s31_wifi_napi_poll, 64);
	netdev->features |= NETIF_F_GRO;
	netdev->hw_features |= NETIF_F_GRO;
	ret = esp32s31_radio_wifi_get_mac(mac);
	if (ret || !is_valid_ether_addr(mac)) {
		pr_err("esp32s31-wifi: failed to read STA MAC: %d\n", ret);
		netif_napi_del(&wifi->napi);
		free_netdev(netdev);
		wifi->netdev = NULL;
		return;
	}
	eth_hw_addr_set(netdev, mac);
	/*
	 * This worker runs outside a cfg80211 callback and does not hold either
	 * RTNL or the wiphy mutex.  register_netdev() supplies the RTNL locking;
	 * the cfg80211 netdev notifier attaches the already initialized wdev.
	 */
	ret = register_netdev(netdev);
	if (ret) {
		netif_napi_del(&wifi->napi);
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

	if (esp32s31_radio_is_disabled()) {
		pr_info("esp32s31-wifi: disabled with radio core\n");
		return 0;
	}

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
