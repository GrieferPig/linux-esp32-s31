// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 cfg80211 fullmac front end. */

#include <linux/etherdevice.h>
#include <linux/esp32s31-radio.h>
#include <linux/interrupt.h>
#include <linux/if_arp.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rtnetlink.h>
#include <linux/smp.h>
#include <linux/workqueue.h>
#include <linux/unaligned.h>
#include <net/cfg80211.h>
#include <net/ieee80211_radiotap.h>

#define S31_WIFI_RX_SKB_POOL_TARGET	128
#define S31_WIFI_RX_SKB_POOL_LOW		112
#define S31_WIFI_RX_RING_SIZE		512
#define S31_WIFI_RX_RING_MASK		(S31_WIFI_RX_RING_SIZE - 1)
#define S31_WIFI_MAX_SCAN_IE_LEN	512

/* Values are part of the serialized ESP-IDF wifi_auth_mode_t ABI. */
#define S31_WIFI_AUTH_WPA2_PSK		3
#define S31_WIFI_AUTH_WPA_WPA2_PSK	4

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
	struct wiphy_work channel_work;
	u8 pending_channel;
	struct napi_struct napi;
	struct s31_wifi_skb_ring rx_skb_pool;
	struct s31_wifi_skb_ring rx_ready_queue;
	struct work_struct rx_refill_work;
	struct work_struct rx_napi_work;
	atomic_t rx_napi_armed;
	u8 connect_ssid[IEEE80211_MAX_SSID_LEN];
	u8 connect_ssid_len;
	bool connect_privacy;
	bool connecting;
	bool connected;
	bool radio_registered;
	bool wiphy_registered;
	bool suspended;
	bool pm_napi_stopped;
	bool ap_active;
	u8 monitor_channel;
	struct net_device __rcu *ap_dev;
	struct net_device __rcu *monitor_dev;
	struct s31_wifi_control ap_config;
	bool enterprise_enabled;
	u8 *eap_data[S31_EAP_FIELDS];
	u32 eap_length[S31_EAP_FIELDS];
	u32 eap_received[S31_EAP_FIELDS];
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

static struct s31_wifi *s31_wifi_frontend;

/* NAPI cannot sleep in the page allocator.  This no-MMU target was already
 * observed to reject full-sized GFP_ATOMIC skb allocations while megabytes of
 * ordinary memory remained available.  Keep a process-context reserve ahead
 * of the hardware-facing SRAM ring; refill work changes allocation context,
 * not packet-delivery context. */
/* Allocation runs on hart 1 so it cannot steal cycles from the closed radio.
 * The 128/64 reserve absorbs normal BA bursts while that CFS worker refills. */
/* rx_skb_pool is an SPSC handoff from the hart-1 allocator to the hart-0 closed
 * callback.  Its producer/consumer indices occupy separate cache lines and it
 * never takes a cross-hart lock.  rx_ready_queue is the complementary SPSC
 * handoff from the hart-0 callback to hart-1 NAPI.  Release/acquire publication
 * keeps the slot visible before its monotonically increasing index. */
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

static size_t s31_wifi_add_rsn_ie(u8 *ies, u8 authmode)
{
	static const u8 wpa2_psk_ccmp_rsn[] = {
		WLAN_EID_RSN, 20,
		0x01, 0x00,
		0x00, 0x0f, 0xac, 0x04,
		0x01, 0x00,
		0x00, 0x0f, 0xac, 0x04,
		0x01, 0x00,
		0x00, 0x0f, 0xac, 0x02,
		0x00, 0x00,
	};

	if (authmode != S31_WIFI_AUTH_WPA2_PSK &&
	    authmode != S31_WIFI_AUTH_WPA_WPA2_PSK)
		return 0;

	memcpy(ies, wpa2_psk_ccmp_rsn, sizeof(wpa2_psk_ccmp_rsn));
	return sizeof(wpa2_psk_ccmp_rsn);
}

static void s31_wifi_scan_complete(void *context, int status,
				   const struct esp32s31_radio_wifi_ap *aps,
				   size_t count)
{
	struct s31_wifi *wifi = context;
	struct wiphy *wiphy = READ_ONCE(wifi->wdev.wiphy);
	struct cfg80211_scan_info info = { .aborted = status != 0 };
	size_t i;

	/* wdev is the cfg80211-owned association after registration.  Use it for
	 * completion callbacks instead of the duplicate bootstrap pointer kept in
	 * the driver's private state. */
	if (WARN_ON_ONCE(!wiphy)) {
		info.aborted = true;
		status = -ENODEV;
	}

	for (i = 0; !status && i < count; i++) {
		const struct esp32s31_radio_wifi_ap *ap = &aps[i];
		struct ieee80211_channel *channel;
		struct cfg80211_bss *bss;
		u8 ies[2 + IEEE80211_MAX_SSID_LEN + 22];
		size_t ie_len;
		u16 capability = WLAN_CAPABILITY_ESS;

		if (!ap->channel || ap->channel > ARRAY_SIZE(s31_channels))
			continue;
		if (ap->ssid_length > IEEE80211_MAX_SSID_LEN) {
			pr_warn_ratelimited("esp32s31-wifi: ignoring invalid SSID length %u\n",
					    ap->ssid_length);
			continue;
		}
		pr_info("esp32s31-wifi: BSS %pM channel=%u authmode=%u ssid=%.*s\n",
			ap->bssid, ap->channel, ap->authmode, ap->ssid_length,
			ap->ssid);
		channel = &s31_channels[ap->channel - 1];
		ies[0] = WLAN_EID_SSID;
		ies[1] = ap->ssid_length;
		memcpy(ies + 2, ap->ssid, ap->ssid_length);
		ie_len = ap->ssid_length + 2;
		ie_len += s31_wifi_add_rsn_ie(ies + ie_len, ap->authmode);
		if (ap->authmode)
			capability |= WLAN_CAPABILITY_PRIVACY;
		bss = cfg80211_inform_bss(wiphy, channel,
					   CFG80211_BSS_FTYPE_UNKNOWN,
					   ap->bssid, 0, capability, 100,
					   ies, ie_len,
					   ap->signal * 100, GFP_KERNEL);
		if (bss)
			cfg80211_put_bss(wiphy, bss);
	}
	if (wifi->scan_request) {
		cfg80211_scan_done(wifi->scan_request, &info);
		wifi->scan_request = NULL;
	}
	pr_info("esp32s31-wifi: scan complete status=%d aps=%zu\n",
		status, count);
}

static void s31_wifi_channel_work(struct wiphy *wiphy, struct wiphy_work *work)
{
	struct s31_wifi *wifi = wiphy_priv(wiphy);
	struct net_device *ap = rcu_dereference_protected(wifi->ap_dev, 1);
	struct cfg80211_chan_def chandef;
	u8 channel = READ_ONCE(wifi->pending_channel);

	if (!ap || !wifi->ap_active || wifi->suspended || !channel ||
	    channel > ARRAY_SIZE(s31_channels) || channel == wifi->ap_config.channel)
		return;
	cfg80211_chandef_create(&chandef, &s31_channels[channel - 1], NL80211_CHAN_HT20);
	wifi->ap_config.channel = channel;
	cfg80211_ch_switch_notify(ap, &chandef, 0);
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
	if (!status) {
		WRITE_ONCE(wifi->pending_channel, channel);
		wiphy_work_queue(wifi->wiphy, &wifi->channel_work);
	}
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
	/* The radio callback and A2DP HCI drain both occupy hart 0.  Schedule NAPI
	 * from the otherwise-idle hart 1 so GRO/TCP ACK processing can proceed in
	 * parallel with the serialized radio core; rx_ready_queue is already an
	 * acquire/release SPSC queue across precisely these two harts. */
	/* rx_ready is called once per frame, while one NAPI instance can consume a
	 * complete RX burst.  Keep the cross-hart work queued from the first frame
	 * until NAPI has drained the ready ring instead of enqueueing another work
	 * item for every frame after the worker itself has returned. */
	if (atomic_cmpxchg(&wifi->rx_napi_armed, 0, 1) == 0)
		schedule_work_on(1, &wifi->rx_napi_work);
}

static void s31_wifi_schedule_rx_refill(struct s31_wifi *wifi);

static int s31_wifi_rx_copy(void *context, const u8 *frame, size_t length)
{
	struct s31_wifi *wifi = context;
	struct sk_buff *skb;

	if (!wifi || READ_ONCE(wifi->suspended) || !wifi->netdev || !netif_running(wifi->netdev) ||
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

static void s31_wifi_rx_napi_workfn(struct work_struct *work)
{
	struct s31_wifi *wifi = container_of(work, struct s31_wifi, rx_napi_work);

	if (wifi->netdev && netif_running(wifi->netdev))
		napi_schedule(&wifi->napi);
	else
		atomic_set(&wifi->rx_napi_armed, 0);
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

	if (unlikely(raw_smp_processor_id() != 1))
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

	if (napi_complete_done(napi, work_done)) {
		/* Publish the idle state before checking the SPSC queue again.  A
		 * producer racing this window either owns the 0 -> 1 transition and
		 * queues the hart-1 work, or leaves a visible packet for this CPU to
		 * reschedule directly. */
		atomic_set(&wifi->rx_napi_armed, 0);
		smp_mb__after_atomic();
		if (s31_wifi_ring_len(&wifi->rx_ready_queue) &&
		    atomic_cmpxchg(&wifi->rx_napi_armed, 0, 1) == 0)
			napi_schedule(napi);
	}

	return work_done;
}

static void s31_wifi_tx_wakeup(void *context)
{
	struct s31_wifi *wifi = context;

	if (wifi && wifi->netdev && !READ_ONCE(wifi->suspended))
		netif_wake_queue(wifi->netdev);
	if (wifi) {
		struct net_device *ap;

		rcu_read_lock();
		ap = rcu_dereference(wifi->ap_dev);
		if (ap && wifi->ap_active && !READ_ONCE(wifi->suspended))
			netif_wake_queue(ap);
		rcu_read_unlock();
	}
}

static void s31_wifi_receive_aux(void *context, u8 interface, const u8 *frame,
				 size_t length, u8 channel, s8 signal)
{
	struct s31_wifi *wifi = context;
	struct net_device *dev;
	struct sk_buff *skb;
	struct {
		struct ieee80211_radiotap_header header;
		u8 flags, pad;
		__le16 frequency, channel_flags;
		s8 signal;
	} __packed radiotap = {
		.header.it_len = cpu_to_le16(sizeof(radiotap)),
		.header.it_present = cpu_to_le32(BIT(IEEE80211_RADIOTAP_FLAGS) |
			BIT(IEEE80211_RADIOTAP_CHANNEL) | BIT(IEEE80211_RADIOTAP_DBM_ANTSIGNAL)),
		.flags = IEEE80211_RADIOTAP_F_FCS,
		.channel_flags = cpu_to_le16(IEEE80211_CHAN_2GHZ),
		.signal = signal,
	};

	if (READ_ONCE(wifi->suspended) || !frame || length > 4096)
		return;
	rcu_read_lock();
	dev = interface == S31_WIFI_IF_AP ? rcu_dereference(wifi->ap_dev) :
		rcu_dereference(wifi->monitor_dev);
	if (!dev || !netif_running(dev) ||
	    (interface == S31_WIFI_IF_AP && length < ETH_HLEN))
		goto out;
	skb = netdev_alloc_skb(dev, length + sizeof(radiotap));
	if (!skb) {
		dev->stats.rx_dropped++;
		goto out;
	}
	if (interface == S31_WIFI_IF_MONITOR) {
		radiotap.frequency = cpu_to_le16(ieee80211_channel_to_frequency(
			channel, NL80211_BAND_2GHZ));
		skb_put_data(skb, &radiotap, sizeof(radiotap));
	}
	skb_put_data(skb, frame, length);
	skb->dev = dev;
	if (interface == S31_WIFI_IF_MONITOR) {
		skb_reset_mac_header(skb);
		skb->protocol = htons(ETH_P_802_2);
		skb->pkt_type = PACKET_OTHERHOST;
	} else {
		skb->protocol = eth_type_trans(skb, dev);
	}
	dev->stats.rx_packets++;
	dev->stats.rx_bytes += length;
	netif_rx(skb);
out:
	rcu_read_unlock();
}

static void s31_wifi_ap_station(void *context, const u8 *mac, bool joined)
{
	struct s31_wifi *wifi = context;
	struct net_device *dev;
	struct station_info info = { };

	rcu_read_lock();
	dev = rcu_dereference(wifi->ap_dev);
	if (dev && !READ_ONCE(wifi->suspended)) {
		if (joined)
			cfg80211_new_sta(dev, mac, &info, GFP_ATOMIC);
		else
			cfg80211_del_sta(dev, mac, GFP_ATOMIC);
	}
	rcu_read_unlock();
}

static const struct esp32s31_radio_wifi_ops s31_radio_wifi_ops = {
	.ap_station = s31_wifi_ap_station,
	.scan_complete = s31_wifi_scan_complete,
	.connected = s31_wifi_connected,
	.disconnected = s31_wifi_disconnected,
	.rx_copy = s31_wifi_rx_copy,
	.rx_ready = s31_wifi_rx_ready,
	.tx_wakeup = s31_wifi_tx_wakeup,
	.receive_aux = s31_wifi_receive_aux,
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

	if (READ_ONCE(wifi->suspended))
		return -EBUSY;
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
	if (READ_ONCE(wifi->suspended) || dev->ieee80211_ptr->iftype != NL80211_IFTYPE_STATION)
		return -EBUSY;
	if (!request->ssid || !request->ssid_len || request->ssid_len > 32)
		return -EINVAL;
	if (wifi->connecting || wifi->connected)
		return -EBUSY;
	if (request->privacy && !request->crypto.psk &&
	    !request->crypto.sae_pwd && !wifi->enterprise_enabled)
		return -EOPNOTSUPP;
	if (wifi->enterprise_enabled && (request->crypto.psk || request->crypto.sae_pwd))
		return -EINVAL;
	params.enterprise = wifi->enterprise_enabled;
	memcpy(params.ssid, request->ssid, request->ssid_len);
	params.ssid_length = request->ssid_len;
	memcpy(wifi->connect_ssid, request->ssid, request->ssid_len);
	wifi->connect_ssid_len = request->ssid_len;
	wifi->connect_privacy = request->privacy || params.enterprise;
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

static int s31_cfg_start_ap(struct wiphy *wiphy, struct net_device *dev,
			    struct cfg80211_ap_settings *settings)
{
	struct s31_wifi *wifi = wiphy_priv(wiphy);
	struct s31_wifi_control request = { .operation = S31_WIFI_AP_START };
	int ret;

	if (wifi->suspended || wifi->ap_active)
		return -EBUSY;
	if (!settings->ssid || !settings->ssid_len || settings->ssid_len > 32 ||
	    !settings->chandef.chan || settings->chandef.chan->band != NL80211_BAND_2GHZ ||
	    (settings->chandef.width != NL80211_CHAN_WIDTH_20 &&
	     settings->chandef.width != NL80211_CHAN_WIDTH_20_NOHT) ||
	    settings->beacon_interval < 100 || settings->beacon_interval > 60000 ||
	    settings->dtim_period < 1 || settings->dtim_period > 10)
		return -EINVAL;
	if (settings->privacy) {
		if (settings->crypto.cipher_group != WLAN_CIPHER_SUITE_CCMP ||
		    settings->crypto.n_akm_suites != 1)
			return -EOPNOTSUPP;
		if (settings->crypto.akm_suites[0] == WLAN_AKM_SUITE_PSK && settings->crypto.psk) {
			memcpy(request.psk, settings->crypto.psk, sizeof(request.psk));
			request.has_psk = 1;
		} else if (settings->crypto.akm_suites[0] == WLAN_AKM_SUITE_SAE &&
			   settings->crypto.sae_pwd && settings->crypto.sae_pwd_len &&
			   settings->crypto.sae_pwd_len <= 63) {
			request.password_length = settings->crypto.sae_pwd_len;
			memcpy(request.password, settings->crypto.sae_pwd, request.password_length);
		} else {
			return -EOPNOTSUPP;
		}
	}
	memcpy(request.ssid, settings->ssid, settings->ssid_len);
	request.ssid_length = settings->ssid_len;
	request.channel = ieee80211_frequency_to_channel(settings->chandef.chan->center_freq);
	request.hidden = settings->hidden_ssid != NL80211_HIDDEN_SSID_NOT_IN_USE;
	request.max_connections = 4;
	request.beacon_interval = settings->beacon_interval;
	request.dtim_period = settings->dtim_period;
	memcpy(request.mac, dev->dev_addr, ETH_ALEN);
	ret = esp32s31_radio_wifi_control(&request);
	if (!ret) {
		wifi->ap_config = request;
		wifi->ap_active = true;
		netif_carrier_on(dev);
		netif_wake_queue(dev);
	}
	memzero_explicit(&request, sizeof(request));
	return ret;
}

static int s31_cfg_stop_ap(struct wiphy *wiphy, struct net_device *dev, unsigned int link)
{
	struct s31_wifi *wifi = wiphy_priv(wiphy);
	struct s31_wifi_control request = { .operation = S31_WIFI_AP_STOP };
	int ret = 0;

	if (link)
		return -EINVAL;
	if (wifi->ap_active && !wifi->suspended)
		ret = esp32s31_radio_wifi_control(&request);
	if (!ret) {
		wifi->ap_active = false;
		memzero_explicit(&wifi->ap_config, sizeof(wifi->ap_config));
		netif_carrier_off(dev);
	}
	return ret;
}

static int s31_cfg_monitor_channel(struct wiphy *wiphy, struct net_device *dev,
				   struct cfg80211_chan_def *chandef)
{
	struct s31_wifi *wifi = wiphy_priv(wiphy);
	struct s31_wifi_control request = { .operation = S31_WIFI_SET_CHANNEL };
	int ret;

	if (wifi->suspended || wifi->connected || wifi->connecting || wifi->ap_active)
		return -EBUSY;
	if (!chandef->chan || chandef->chan->band != NL80211_BAND_2GHZ ||
	    (chandef->width != NL80211_CHAN_WIDTH_20 && chandef->width != NL80211_CHAN_WIDTH_20_NOHT))
		return -EINVAL;
	request.channel = ieee80211_frequency_to_channel(chandef->chan->center_freq);
	ret = esp32s31_radio_wifi_control(&request);
	if (!ret)
		wifi->monitor_channel = request.channel;
	return ret;
}

static struct wireless_dev *s31_cfg_add_interface(struct wiphy *wiphy, const char *name,
		unsigned char assign_type, enum nl80211_iftype type, struct vif_params *params);
static int s31_cfg_del_interface(struct wiphy *wiphy, struct wireless_dev *wdev);
static int s31_cfg_change_interface(struct wiphy *wiphy, struct net_device *dev,
				   enum nl80211_iftype type, struct vif_params *params);

static int s31_cfg_del_station(struct wiphy *wiphy, struct net_device *dev,
			       struct station_del_parameters *params)
{
	struct s31_wifi *wifi = wiphy_priv(wiphy);
	struct s31_wifi_control request = { .operation = S31_WIFI_AP_DEAUTH };

	if (wifi->suspended || !wifi->ap_active)
		return -ENETDOWN;
	if (params->mac)
		ether_addr_copy(request.mac, params->mac);
	else
		eth_broadcast_addr(request.mac);
	return esp32s31_radio_wifi_control(&request);
}

static const struct cfg80211_ops s31_cfg80211_ops = {
	.del_station = s31_cfg_del_station,
	.scan = s31_cfg_scan,
	.connect = s31_cfg_connect,
	.disconnect = s31_cfg_disconnect,
	.start_ap = s31_cfg_start_ap,
	.stop_ap = s31_cfg_stop_ap,
	.set_monitor_channel = s31_cfg_monitor_channel,
	.add_virtual_intf = s31_cfg_add_interface,
	.del_virtual_intf = s31_cfg_del_interface,
	.change_virtual_intf = s31_cfg_change_interface,
};

static netdev_tx_t s31_wifi_xmit(struct sk_buff *skb, struct net_device *dev)
{
	int ret = esp32s31_radio_wifi_send_interface(
		dev->ieee80211_ptr->iftype == NL80211_IFTYPE_AP ? S31_WIFI_IF_AP : S31_WIFI_IF_STA,
		skb->data, skb->len);

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
		dev_warn_ratelimited(&dev->dev,
				     "TX enqueue rejected len=%u ret=%d\n",
				     skb->len, ret);
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
	atomic_set(&wifi->rx_napi_armed, 0);
	napi_enable(&wifi->napi);
	netif_start_queue(dev);
	netif_carrier_off(dev);
	if (s31_wifi_ring_len(&wifi->rx_ready_queue)) {
		atomic_set(&wifi->rx_napi_armed, 1);
		napi_schedule(&wifi->napi);
	}
	return 0;
}

static int s31_wifi_stop(struct net_device *dev)
{
	struct s31_wifi *wifi = wiphy_priv(dev->ieee80211_ptr->wiphy);
	struct sk_buff *skb;

	netif_stop_queue(dev);
	if (wifi->pm_napi_stopped) {
		wifi->pm_napi_stopped = false;
		return 0;
	}
	cancel_work_sync(&wifi->rx_napi_work);
	napi_disable(&wifi->napi);
	atomic_set(&wifi->rx_napi_armed, 0);
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
	dev->needs_free_netdev = false;
}

static int s31_aux_open(struct net_device *dev)
{
	struct s31_wifi *wifi = wiphy_priv(dev->ieee80211_ptr->wiphy);
	struct s31_wifi_control request = { .operation = S31_WIFI_MONITOR_START };
	int ret;

	if (wifi->suspended)
		return -EBUSY;
	if (dev->ieee80211_ptr->iftype == NL80211_IFTYPE_MONITOR) {
		ret = esp32s31_radio_wifi_control(&request);
		if (ret)
			return ret;
		netif_carrier_on(dev);
	} else {
		netif_carrier_off(dev);
	}
	netif_start_queue(dev);
	return 0;
}

static int s31_aux_stop(struct net_device *dev)
{
	struct s31_wifi *wifi = wiphy_priv(dev->ieee80211_ptr->wiphy);
	struct s31_wifi_control request = { .operation = S31_WIFI_MONITOR_STOP };

	netif_stop_queue(dev);
	netif_carrier_off(dev);
	if (dev->ieee80211_ptr->iftype == NL80211_IFTYPE_MONITOR && !wifi->suspended)
		return esp32s31_radio_wifi_control(&request);
	return 0;
}

static netdev_tx_t s31_aux_xmit(struct sk_buff *skb, struct net_device *dev)
{
	/* Promiscuous reception does not imply arbitrary 802.11 injection. */
	if (dev->ieee80211_ptr->iftype == NL80211_IFTYPE_MONITOR) {
		dev->stats.tx_dropped++;
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}
	return s31_wifi_xmit(skb, dev);
}

static const struct net_device_ops s31_aux_netdev_ops = {
	.ndo_open = s31_aux_open,
	.ndo_stop = s31_aux_stop,
	.ndo_start_xmit = s31_aux_xmit,
};

static void s31_aux_setup(struct net_device *dev)
{
	ether_setup(dev);
	dev->netdev_ops = &s31_aux_netdev_ops;
	dev->needs_free_netdev = true;
}

static struct wireless_dev *s31_cfg_add_interface(struct wiphy *wiphy, const char *name,
		unsigned char assign_type, enum nl80211_iftype type, struct vif_params *params)
{
	struct s31_wifi *wifi = wiphy_priv(wiphy);
	struct wireless_dev *wdev;
	struct net_device *dev;
	u8 mac[ETH_ALEN];
	int ret;

	if (wifi->suspended)
		return ERR_PTR(-EBUSY);
	if (type != NL80211_IFTYPE_AP && type != NL80211_IFTYPE_MONITOR)
		return ERR_PTR(-EOPNOTSUPP);
	if ((type == NL80211_IFTYPE_AP && rcu_access_pointer(wifi->ap_dev)) ||
	    (type == NL80211_IFTYPE_MONITOR && rcu_access_pointer(wifi->monitor_dev)))
		return ERR_PTR(-EBUSY);
	dev = alloc_netdev(sizeof(*wdev), name, assign_type, s31_aux_setup);
	if (!dev)
		return ERR_PTR(-ENOMEM);
	wdev = netdev_priv(dev);
	wdev->wiphy = wiphy;
	wdev->iftype = type;
	wdev->netdev = dev;
	dev->ieee80211_ptr = wdev;
	SET_NETDEV_DEV(dev, wiphy_dev(wiphy));
	ether_addr_copy(mac, wifi->netdev->dev_addr);
	mac[0] |= 2;
	mac[5] ^= type == NL80211_IFTYPE_AP ? 1 : 2;
	eth_hw_addr_set(dev, mac);
	if (type == NL80211_IFTYPE_MONITOR) {
		dev->type = ARPHRD_IEEE80211_RADIOTAP;
		dev->header_ops = NULL;
		dev->hard_header_len = 0;
		dev->flags = IFF_NOARP;
	}
	ret = cfg80211_register_netdevice(dev);
	if (ret) {
		free_netdev(dev);
		return ERR_PTR(ret);
	}
	if (type == NL80211_IFTYPE_AP)
		rcu_assign_pointer(wifi->ap_dev, dev);
	else
		rcu_assign_pointer(wifi->monitor_dev, dev);
	return wdev;
}

static int s31_cfg_del_interface(struct wiphy *wiphy, struct wireless_dev *wdev)
{
	struct s31_wifi *wifi = wiphy_priv(wiphy);
	int ret;

	if (wdev == &wifi->wdev)
		return -EOPNOTSUPP;
	if (wdev->iftype == NL80211_IFTYPE_AP) {
		ret = s31_cfg_stop_ap(wiphy, wdev->netdev, 0);
		if (ret)
			return ret;
		RCU_INIT_POINTER(wifi->ap_dev, NULL);
	} else {
		RCU_INIT_POINTER(wifi->monitor_dev, NULL);
	}
	synchronize_net();
	cfg80211_unregister_netdevice(wdev->netdev);
	return 0;
}

static int s31_cfg_change_interface(struct wiphy *wiphy, struct net_device *dev,
				   enum nl80211_iftype type, struct vif_params *params)
{
	struct s31_wifi *wifi = wiphy_priv(wiphy);

	if (dev->ieee80211_ptr->iftype == type)
		return 0;
	if (dev != wifi->netdev || (type != NL80211_IFTYPE_STATION && type != NL80211_IFTYPE_AP))
		return -EOPNOTSUPP;
	if (wifi->suspended || wifi->connected || wifi->connecting || wifi->scan_request ||
	    wifi->ap_active || (type == NL80211_IFTYPE_AP && rcu_access_pointer(wifi->ap_dev)))
		return -EBUSY;
	if (type == NL80211_IFTYPE_AP)
		rcu_assign_pointer(wifi->ap_dev, dev);
	else {
		RCU_INIT_POINTER(wifi->ap_dev, NULL);
		synchronize_net();
	}
	dev->ieee80211_ptr->iftype = type;
	return 0;
}

static const struct ieee80211_iface_limit s31_iface_limits[] = {
	{ .max = 1, .types = BIT(NL80211_IFTYPE_STATION) },
	{ .max = 1, .types = BIT(NL80211_IFTYPE_AP) },
	{ .max = 1, .types = BIT(NL80211_IFTYPE_MONITOR) },
};

static const struct ieee80211_iface_combination s31_iface_combinations[] = {
	{ .limits = s31_iface_limits, .n_limits = ARRAY_SIZE(s31_iface_limits),
	  .max_interfaces = 3, .num_different_channels = 1 },
};

static void s31_wifi_eap_free(struct s31_wifi *wifi)
{
	unsigned int i;

	for (i = 0; i < S31_EAP_FIELDS; i++) {
		kfree_sensitive(wifi->eap_data[i]);
		wifi->eap_data[i] = NULL;
		wifi->eap_length[i] = wifi->eap_received[i] = 0;
	}
	wifi->enterprise_enabled = false;
}

/* Vendor provisioning is needed because cfg80211 connect has no CA, client
 * certificate or EAP identity fields for a firmware-owned EAP supplicant.
 * Wire header: five little-endian u32 values, then at most 512 data bytes. */
static int s31_wifi_vendor_eap(struct wiphy *wiphy, struct wireless_dev *wdev,
			       const void *data, int data_len)
{
	struct s31_wifi *wifi = wiphy_priv(wiphy);
	struct s31_wifi_control request = { };
	const u8 *wire = data;
	u8 *new_field = NULL;
	unsigned int i;
	int ret;

	if (wifi->suspended || wifi->connected || wifi->connecting ||
	    wdev != &wifi->wdev || wdev->iftype != NL80211_IFTYPE_STATION)
		return -EBUSY;
	if (data_len < 20)
		return -EINVAL;
	request.operation = get_unaligned_le32(wire);
	request.field = get_unaligned_le32(wire + 4);
	request.offset = get_unaligned_le32(wire + 8);
	request.total = get_unaligned_le32(wire + 12);
	request.length = get_unaligned_le32(wire + 16);
	if (request.length > S31_EAP_CHUNK || data_len != 20 + request.length)
		return -EINVAL;
	if (request.operation == S31_WIFI_EAP_WRITE) {
		if (wifi->enterprise_enabled || request.field >= S31_EAP_FIELDS ||
		    !request.total || request.total > S31_EAP_MAX_FIELD || !request.length ||
		    request.offset > request.total || request.length > request.total - request.offset)
			return -EINVAL;
		i = request.field;
		if (!request.offset) {
			if (wifi->eap_data[i])
				return -EALREADY;
			new_field = kzalloc(request.total + 1, GFP_KERNEL);
			if (!new_field)
				return -ENOMEM;
		} else if (!wifi->eap_data[i] || request.total != wifi->eap_length[i] ||
			   request.offset != wifi->eap_received[i]) {
			return -EINVAL;
		}
		memcpy(request.data, wire + 20, request.length);
	} else if (request.operation == S31_WIFI_EAP_COMMIT) {
		if (request.length || request.offset || request.total || request.field)
			return -EINVAL;
		for (i = 0; i < S31_EAP_FIELDS; i++)
			if (wifi->eap_length[i] != wifi->eap_received[i])
				return -EINVAL;
		if (!wifi->eap_data[S31_EAP_DOMAIN] || wifi->eap_length[S31_EAP_DOMAIN] > 253 ||
		    memchr(wifi->eap_data[S31_EAP_DOMAIN], 0, wifi->eap_length[S31_EAP_DOMAIN]))
			return -EINVAL;
	} else if (request.operation != S31_WIFI_EAP_CLEAR || request.length ||
		   request.field || request.offset || request.total) {
		return -EINVAL;
	}
	ret = esp32s31_radio_wifi_control(&request);
	if (!ret) {
		if (request.operation == S31_WIFI_EAP_CLEAR) {
			s31_wifi_eap_free(wifi);
		} else if (request.operation == S31_WIFI_EAP_COMMIT) {
			wifi->enterprise_enabled = true;
		} else {
			i = request.field;
			if (new_field) {
				wifi->eap_data[i] = new_field;
				new_field = NULL;
				wifi->eap_length[i] = request.total;
			}
			memcpy(wifi->eap_data[i] + request.offset, request.data, request.length);
			wifi->eap_received[i] += request.length;
		}
	}
	kfree_sensitive(new_field);
	memzero_explicit(&request, sizeof(request));
	return ret;
}

static int s31_wifi_eap_restore(struct s31_wifi *wifi)
{
	struct s31_wifi_control request = { .operation = S31_WIFI_EAP_WRITE };
	unsigned int i, offset;
	int ret = 0;

	if (!wifi->enterprise_enabled)
		return 0;
	for (i = 0; i < S31_EAP_FIELDS; i++) {
		request.field = i;
		request.total = wifi->eap_length[i];
		for (offset = 0; offset < request.total; offset += request.length) {
			request.offset = offset;
			request.length = min_t(u32, S31_EAP_CHUNK, request.total - offset);
			memcpy(request.data, wifi->eap_data[i] + offset, request.length);
			ret = esp32s31_radio_wifi_control(&request);
			if (ret)
				goto out;
		}
	}
	memzero_explicit(&request, sizeof(request));
	request.operation = S31_WIFI_EAP_COMMIT;
	ret = esp32s31_radio_wifi_control(&request);
out:
	memzero_explicit(&request, sizeof(request));
	return ret;
}

static const struct wiphy_vendor_command s31_vendor_commands[] = {
	{ .info = { .vendor_id = S31_WIFI_VENDOR_ID, .subcmd = S31_WIFI_VENDOR_EAP },
	  .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	  .doit = s31_wifi_vendor_eap, .policy = VENDOR_CMD_RAW_DATA },
};

static void s31_wifi_register_workfn(struct work_struct *work)
{
	struct s31_wifi *wifi = container_of(to_delayed_work(work),
					       struct s31_wifi, register_work);
	struct net_device *netdev;
	u8 mac[ETH_ALEN];
	int ret;

	ret = wifi->radio_registered ? 0 :
		esp32s31_radio_wifi_register(&s31_radio_wifi_ops, wifi);
	if (ret == -EAGAIN) {
		schedule_delayed_work(&wifi->register_work, msecs_to_jiffies(100));
		return;
	}
	if (ret)
		return;
	wifi->radio_registered = true;

	wifi->wiphy->max_scan_ssids = 4;
	/* The closed scan engine does not consume probe-request IEs, but the
	 * cfg80211 contract still requires a non-zero limit so standard clients
	 * such as wpa_supplicant are not rejected before .scan is called. */
	wifi->wiphy->max_scan_ie_len = S31_WIFI_MAX_SCAN_IE_LEN;
	wifi->wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;
	wifi->wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION) |
		BIT(NL80211_IFTYPE_AP) | BIT(NL80211_IFTYPE_MONITOR);
	wifi->wiphy->iface_combinations = s31_iface_combinations;
	wifi->wiphy->n_iface_combinations = ARRAY_SIZE(s31_iface_combinations);
	wifi->wiphy->vendor_commands = s31_vendor_commands;
	wifi->wiphy->n_vendor_commands = ARRAY_SIZE(s31_vendor_commands);
	wifi->wiphy->cipher_suites = s31_cipher_suites;
	wifi->wiphy->n_cipher_suites = ARRAY_SIZE(s31_cipher_suites);
	wifi->wiphy->akm_suites = s31_akm_suites;
	wifi->wiphy->n_akm_suites = ARRAY_SIZE(s31_akm_suites);
	wifi->wiphy->bands[NL80211_BAND_2GHZ] = &s31_band_2ghz;
	wiphy_ext_feature_set(wifi->wiphy,
			      NL80211_EXT_FEATURE_4WAY_HANDSHAKE_STA_PSK);
	wiphy_ext_feature_set(wifi->wiphy, NL80211_EXT_FEATURE_SAE_OFFLOAD);
	wiphy_ext_feature_set(wifi->wiphy, NL80211_EXT_FEATURE_4WAY_HANDSHAKE_AP_PSK);
	wiphy_ext_feature_set(wifi->wiphy, NL80211_EXT_FEATURE_SAE_OFFLOAD_AP);
	ret = wifi->wiphy_registered ? 0 : wiphy_register(wifi->wiphy);
	if (ret)
		return;
	wifi->wiphy_registered = true;

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
	INIT_WORK(&wifi->rx_napi_work, s31_wifi_rx_napi_workfn);
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

int s31_radio_wifi_frontend_init(struct device *parent)
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
	s31_wifi_frontend = wifi;
	set_wiphy_dev(wiphy, parent);
	INIT_DELAYED_WORK(&wifi->register_work, s31_wifi_register_workfn);
	wiphy_work_init(&wifi->channel_work, s31_wifi_channel_work);
	schedule_delayed_work(&wifi->register_work, 0);
	return 0;
}

int s31_radio_wifi_frontend_suspend(void)
{
	struct s31_wifi *wifi = s31_wifi_frontend;
	struct net_device *ap, *monitor;
	struct cfg80211_scan_info info = { .aborted = true };

	if (!wifi)
		return 0;
	cancel_delayed_work_sync(&wifi->register_work);
	rtnl_lock();
	wiphy_lock(wifi->wiphy);
	WRITE_ONCE(wifi->suspended, true);
	ap = rcu_dereference_protected(wifi->ap_dev, 1);
	monitor = rcu_dereference_protected(wifi->monitor_dev, 1);
	if (ap)
		netif_device_detach(ap);
	if (monitor)
		netif_device_detach(monitor);
	if (wifi->netdev) {
		netif_device_detach(wifi->netdev);
		netif_tx_disable(wifi->netdev);
		if (netif_running(wifi->netdev) && !wifi->pm_napi_stopped) {
			s31_wifi_stop(wifi->netdev);
			wifi->pm_napi_stopped = true;
		}
		if (wifi->connecting || wifi->connected)
			s31_wifi_disconnected(wifi, WLAN_REASON_DEAUTH_LEAVING);
	}
	if (wifi->scan_request) {
		cfg80211_scan_done(wifi->scan_request, &info);
		wifi->scan_request = NULL;
	}
	wiphy_unlock(wifi->wiphy);
	rtnl_unlock();
	synchronize_net();
	return 0;
}

int s31_radio_wifi_frontend_resume(void)
{
	struct s31_wifi *wifi = s31_wifi_frontend;
	struct s31_wifi_control request = { .operation = S31_WIFI_MONITOR_START };
	struct net_device *ap, *monitor;
	int ret = 0;

	if (!wifi)
		return 0;
	rtnl_lock();
	wiphy_lock(wifi->wiphy);
	ap = rcu_dereference_protected(wifi->ap_dev, 1);
	monitor = rcu_dereference_protected(wifi->monitor_dev, 1);
	ret = s31_wifi_eap_restore(wifi);
	if (ret)
		goto out;
	if (wifi->pm_napi_stopped) {
		ret = s31_wifi_open(wifi->netdev);
		if (ret)
			goto out;
		wifi->pm_napi_stopped = false;
	}
	if (ap && wifi->ap_active) {
		ret = esp32s31_radio_wifi_control(&wifi->ap_config);
		if (ret)
			goto out;
		netif_carrier_on(ap);
	}
	if (monitor && netif_running(monitor)) {
		ret = esp32s31_radio_wifi_control(&request);
		if (ret)
			goto out;
		if (!wifi->ap_active && wifi->monitor_channel) {
			request.operation = S31_WIFI_SET_CHANNEL;
			request.channel = wifi->monitor_channel;
			ret = esp32s31_radio_wifi_control(&request);
			if (ret)
				goto out;
		}
	}
	WRITE_ONCE(wifi->suspended, false);
	if (wifi->netdev)
		netif_device_attach(wifi->netdev);
	if (ap)
		netif_device_attach(ap);
	if (monitor)
		netif_device_attach(monitor);
out:
	wiphy_unlock(wifi->wiphy);
	rtnl_unlock();
	if (!ret && !wifi->netdev)
		schedule_delayed_work(&wifi->register_work, 0);
	return ret;
}

void s31_radio_wifi_frontend_exit(void)
{
	struct s31_wifi *wifi = xchg(&s31_wifi_frontend, NULL);
	struct cfg80211_scan_info info = { .aborted = true };

	if (!wifi)
		return;
	cancel_delayed_work_sync(&wifi->register_work);
	if (wifi->wiphy_registered) {
		struct net_device *ap, *monitor;

		rtnl_lock();
		wiphy_lock(wifi->wiphy);
		WRITE_ONCE(wifi->suspended, true);
		wiphy_work_cancel(wifi->wiphy, &wifi->channel_work);
		ap = rcu_dereference_protected(wifi->ap_dev, 1);
		monitor = rcu_dereference_protected(wifi->monitor_dev, 1);
		if (ap && ap != wifi->netdev)
			s31_cfg_del_interface(wifi->wiphy, ap->ieee80211_ptr);
		if (monitor)
			s31_cfg_del_interface(wifi->wiphy, monitor->ieee80211_ptr);
		RCU_INIT_POINTER(wifi->ap_dev, NULL);
		synchronize_net();
		wiphy_unlock(wifi->wiphy);
		rtnl_unlock();
	}
	if (wifi->scan_request) {
		cfg80211_scan_done(wifi->scan_request, &info);
		wifi->scan_request = NULL;
	}
	if (wifi->netdev) {
		unregister_netdev(wifi->netdev);
		netif_napi_del(&wifi->napi);
		free_netdev(wifi->netdev);
		wifi->netdev = NULL;
	}
	if (wifi->radio_registered) {
		esp32s31_radio_wifi_unregister(&s31_radio_wifi_ops, wifi);
		wifi->radio_registered = false;
	}
	if (wifi->wiphy_registered) {
		wiphy_unregister(wifi->wiphy);
		wifi->wiphy_registered = false;
	}
	s31_wifi_eap_free(wifi);
	memzero_explicit(&wifi->ap_config, sizeof(wifi->ap_config));
	wiphy_free(wifi->wiphy);
}
