// SPDX-License-Identifier: GPL-2.0-only
/* Native open/WPA2-CCMP STA mac80211 frontend. */
#include <linux/etherdevice.h>
#include <linux/esp32s31-radio.h>
#include <linux/workqueue.h>
#include <linux/moduleparam.h>
#include <linux/cpu.h>
#include <linux/completion.h>
#include <linux/bitmap.h>
#include <linux/kernel_stat.h>
#include <linux/io.h>
#include <linux/sched/signal.h>
#include <linux/sched/task.h>
#include <linux/smp.h>
#include <linux/delay.h>
#include <net/mac80211.h>
#include <net/gro.h>
#include "esp32s31-rx-copy.h"

static const unsigned int sm_io_cpu = 1;
/* skb_copy_bits() in the SRAM bridge consumes page frags without retaining
 * them. Let TCP keep those frags until that single mandatory staging copy. */
static const bool sm_tx_sg = true;
static unsigned int sm_tx_sg_frames;


/* Submit only to the Linux SRAM TX ring from mac80211's atomic context.
 * Native PP still runs exclusively under its existing owner/blob guards. */
/* Consume native completions on the sole RX/TX-status NAPI owner.
 * Submission still uses only the Linux SRAM ring, never PP or the blob gate. */

static const bool sm_direct_status = true;
static unsigned int sm_direct_status_calls;

/* Consume scheduled TXQs on the existing sole NAPI owner. No retained TXQ
 * pointer, extra worker, timer delay, or additional packet pool. */
static const bool sm_txq_napi = true;

static const bool sm_direct_tx = true;

static const unsigned int sm_data_tx_rate = 11;
static const int sm_data_tx_mcs = 9;
static const bool sm_tx_amsdu = true;
static const unsigned int sm_tx_ampdu = 16;


static const unsigned int sm_tx_pending_limit = 16;

static unsigned int sm_direct_tx_calls;

static const bool sm_direct_rx = true;

/* Preserve the producer's IRQ-disabled section, but release the shared
 * consumer lock during its bounded copy/checksum. The separate producer
 * lock serializes callbacks; .stop joins it before resetting RX slots. */

static const bool sm_rx_view = true;

static unsigned int sm_rx_view_hits;

static const bool sm_use_napi = true;
/* Keep the measured CPU0 IRQ wake behavior only while associated. */
static const bool sm_auto_idle_poll = true;

static const bool sm_ht = true;
static const bool sm_he = true;
static const bool sm_he_tx = true;

static const bool sm_rx_ba = true;

static const bool sm_hw_ccmp = true;

static const bool sm_hw_tx_ccmp = true;

static const bool sm_hw_tx_ccmp_fast = true;


static unsigned int sm_rx_buf_len = 1600;

static const unsigned int sm_napi_budget = 64;

static const unsigned int sm_gro_batch = 16;

static const bool sm_hw_bssid_filter = true;

static bool sm_hw_bssid_filter_active;

static unsigned int sm_hw_bssid_filter_updates;

static const bool sm_rx_filter = true;
static const bool sm_native_rx_filter = true;
static const bool sm_native_rx_error_filter = true;

static const bool sm_native_bssid_filter = true;


static const bool sm_tx_highpri = true;

static struct workqueue_struct *sm_io_wq(void)
{

 return sm_use_napi && sm_tx_highpri ? system_highpri_wq : system_percpu_wq;
}
static unsigned int sm_io_target(void)
{
 unsigned int cpu=READ_ONCE(sm_io_cpu);
 return cpu<nr_cpu_ids && cpu_online(cpu)?cpu:0;
}

#define SM_RX_SLOTS 128
static const unsigned int sm_rx_ring_slots = 128;

static const unsigned int sm_rx_idle_skb_limit = 64;

#define SM_TX_QUEUED 64
/* One submission may be requeued while producers fill the freed slot. */
struct sm_rx {
	u16 length;
	u8 channel;
	s8 signal;
	u8 rate;
	u8 encoding, phy_flags;
	struct sk_buff *skb;
};
struct sm_pending {
	struct sk_buff *skb;
	u32 cookie;
	unsigned long expires;
	bool done, ack;
	u8 attempts, rate;
};

struct s31_sm {
	struct ieee80211_hw *hw;
	struct delayed_work register_work, timer;
	struct work_struct io, rx_io, status_io, filter_io;
	struct net_device *napi_dev;
	struct napi_struct napi;
	call_single_data_t rx_csd;
	struct completion rx_kick_done;
	bool rx_kick_pending;

 bool scanning, associated;
	u8 bssid[ETH_ALEN];
	u8 rx_ba_tid[4];
	u8 rx_ba_mask;
	u8 tx_key_generation;
	u32 tx_fast_frames, tx_fast_amsdu;
	/* lock protects overlapping wake callbacks and the final flush. */
	u32 txq_batch_users, txq_batch_frames, txq_batch_flushes;
 u8 txq_pending_ac; /* wake callbacks publish only AC bits, under lock */
 u32 txq_napi_wakes, txq_napi_passes, txq_napi_frames;
	unsigned int filter_flags;
	u32 rx_filtered, rx_kicks;
	bool rx_need_refill;
	struct ieee80211_sta __rcu *rx_sta;
	bool napi_active;
	u32 napi_polls, napi_budget_hits;
	spinlock_t lock;
	spinlock_t submit_lock;
	struct sk_buff_head tx, status_queue;
	/* Protected by lock, advanced with the FIFO skb queue. */
	struct sm_pending pending[S31_SOFTMAC_PENDING];
	unsigned int status_cursor;
 DECLARE_BITMAP(status_done, S31_SOFTMAC_PENDING);
	struct sm_rx rx[SM_RX_SLOTS];
	unsigned int head, tail;
 unsigned int rx_slots, rx_mask;
	unsigned int rx_idle_limit, rx_emergency_alloc;
	unsigned long rx_last_data;
	bool rx_trimmed;
	u32 cookie, epoch;
	u32 rx_queued, rx_delivered, rx_dropped, rx_alloc_failed, rx_peak;
	u32 tx_completed, tx_failed, tx_timeout;
	u32 tx_outstanding, tx_depth_peak, tx_gate_stops, tx_gate_wakes;
	bool tx_gate_stopped;
	bool running, registered, radio_registered, has_vif;
	bool tx_ht_peer, tx_he_peer;
	bool tx_ba_requested, tx_ba_operational, tx_native_peer, tx_ba_flushing;
	u32 tx_ampdu_submitted;
	u8 channel;
	struct ieee80211_channel channels[11];
	struct ieee80211_rate rates[12];
	struct ieee80211_supported_band band;
	struct ieee80211_sband_iftype_data iftype;
};
/* Caller holds sm->lock, or producers/NAPI have been joined for stop. */
static void sm_pending_clear(struct s31_sm *sm, unsigned int i)
{
 sm->pending[i].skb = NULL;
 __clear_bit(i, sm->status_done);
}

static struct s31_sm *s31_sm_frontend;

/* Caller holds sm->lock. DRIVER wake schedules a mac80211 tasklet, never
 * invokes .tx inline; stop/wake ordering therefore stays under this lock. */
static void sm_tx_retire_locked(struct s31_sm *sm)
{
	if (WARN_ON_ONCE(!sm->tx_outstanding)) return;
	sm->tx_outstanding--;

	if (sm->running && sm->tx_gate_stopped &&
	    sm->tx_outstanding <= sm_tx_pending_limit / 2) {
		sm->tx_gate_stopped = false;
		sm->tx_gate_wakes++;
		ieee80211_wake_queues(sm->hw);
	}
}

/* A coalesced IPI only schedules the CPU1 NAPI, never runs the stack in
 * the radio callback. The producer lock serializes CSD submission and stop. */
/* Publication-to-service latency includes requests made during a poll. It
 * does not claim CPU self time or that a new IRQ was required. sm->lock
 * serializes producers, the single poll owner, and stop. */
/* Caller holds sm->lock with local IRQs disabled. Remote counters and
 * NAPI list fields are approximate READ_ONCE snapshots, not atomic pairs. */

static void sm_rx_kick(void *arg)
{
 struct s31_sm *sm = arg;
 unsigned long flags;
 spin_lock_irqsave(&sm->lock, flags);

 if (sm->running) napi_schedule(&sm->napi);
 sm->rx_kick_pending = false;
 complete(&sm->rx_kick_done);
 spin_unlock_irqrestore(&sm->lock, flags);
}

static void sm_schedule_rx_locked(struct s31_sm *sm)
{
 unsigned int cpu = cpu_online(1) ? 1 : 0;
 int ret;

 if (test_bit(NAPI_STATE_SCHED, &sm->napi.state)) return;
 if (cpu == smp_processor_id()) {
  napi_schedule(&sm->napi);
  return;
 }
 if (sm->rx_kick_pending) return;
 reinit_completion(&sm->rx_kick_done);
 sm->rx_kick_pending = true;
 ret = smp_call_function_single_async(cpu, &sm->rx_csd);
 if (!ret) {
  sm->rx_kicks++;
  return;
 }
 sm->rx_kick_pending = false;
 complete(&sm->rx_kick_done);
 queue_work_on(cpu, system_percpu_wq, &sm->rx_io);
}

/* Replacements are allocated on the Linux consumer stack. The radio callback
 * only copies into an already allocated skb; it never allocates or retains
 * a native buffer. Retry rare allocation failures without poisoning a slot. */
static struct sk_buff *sm_alloc_rx_skb(struct s31_sm *sm, bool use_napi)
{
 unsigned int size = clamp_t(unsigned int, sm_rx_buf_len, 1600, S31_SOFTMAC_FRAME_MAX) + 8;
 struct sk_buff *skb = use_napi ? napi_alloc_skb(&sm->napi, size) :
                                __dev_alloc_skb(size, GFP_KERNEL);
 /* Align the 802.11 payload, rather than relying on the Ethernet default
  * headroom of napi_alloc_skb(). QoS's extra two bytes are handled at RX. */
 if (skb) skb_reserve(skb, -(unsigned long)skb->data & 3);
 return skb;
}
static struct sk_buff *sm_alloc_rx_skb_atomic(void)
{
 unsigned int size = clamp_t(unsigned int, sm_rx_buf_len, 1600, S31_SOFTMAC_FRAME_MAX) + 8;
 struct sk_buff *skb = __dev_alloc_skb(size, GFP_ATOMIC);

 if (skb) skb_reserve(skb, -(unsigned long)skb->data & 3);
 return skb;
}
static void sm_refill_empty(struct s31_sm *sm, bool use_napi)
{
 unsigned int i;
 unsigned long flags;
 bool failed = false;
 if (!READ_ONCE(sm->rx_need_refill)) return;
 for (i = 0; i < sm->rx_slots; i++) {
  struct sk_buff *skb;
  if (READ_ONCE(sm->rx[i].skb)) continue;
  skb = sm_alloc_rx_skb(sm, use_napi);
  if (!skb) { failed = true; continue; }
  spin_lock_irqsave(&sm->lock, flags);
  if (!sm->rx[i].skb) {
   sm->rx[i].skb = skb;
   skb = NULL;
  }
  spin_unlock_irqrestore(&sm->lock, flags);
  if (skb) dev_kfree_skb(skb);
 }
 WRITE_ONCE(sm->rx_need_refill, failed);
 /* A failed replacement is exceptional. Restore a complete ring before
  * trying the idle policy again. */
 if (!failed) WRITE_ONCE(sm->rx_trimmed, false);
}
static void sm_install_rx_replacement(struct s31_sm *sm, struct sm_rx *rx,
                                      struct sk_buff *replacement)
{
 unsigned long flags;
 unsigned int target;

 if (READ_ONCE(sm->rx_trimmed)) {
  /* Move the spare window with the producer instead of leaving all 64
   * buffers behind after one lap of the logical 128-slot ring. */
  target = (sm->tail + sm->rx_idle_limit) & sm->rx_mask;
  spin_lock_irqsave(&sm->lock, flags);
  if (sm->rx_trimmed && !sm->rx[target].skb) {
   sm->rx[target].skb = replacement;
   replacement = NULL;
  }
  spin_unlock_irqrestore(&sm->lock, flags);
 }
 rx->skb = replacement;
}
static void sm_trim_idle_rx_skbs(struct s31_sm *sm)
{
 unsigned long flags;
 struct sk_buff *freed[SM_RX_SLOTS];
 unsigned int i, count = 0;

 if (sm->rx_idle_limit >= sm->rx_slots || READ_ONCE(sm->rx_trimmed) ||
     !time_after(jiffies, READ_ONCE(sm->rx_last_data) + 5 * HZ))
  return;
 spin_lock_irqsave(&sm->lock, flags);
 if (!sm->running || sm->head != sm->tail || sm->rx_trimmed ||
     sm->rx_need_refill ||
     !time_after(jiffies, sm->rx_last_data + 5 * HZ)) {
  spin_unlock_irqrestore(&sm->lock, flags);
  return;
 }
 for (i = 0; i < sm->rx_slots; i++) {
  if (((i - sm->head) & sm->rx_mask) >= sm->rx_idle_limit) {
   if (sm->rx[i].skb) freed[count++] = sm->rx[i].skb;
   sm->rx[i].skb = NULL;
  }
 }
 sm->rx_trimmed = true;
 spin_unlock_irqrestore(&sm->lock, flags);
 for (i = 0; i < count; i++) dev_kfree_skb(freed[i]);
}

static void sm_status_io(struct work_struct *work)
{
 struct s31_sm *sm=container_of(work,struct s31_sm,status_io);
 struct sk_buff *skb;
 while ((skb=skb_dequeue(&sm->status_queue)))
  ieee80211_tx_status_irqsafe(sm->hw,skb);
}
static bool sm_tx_rate_is_he(unsigned int rate)
{
 return (rate & ~15U) == S31_SOFTMAC_TX_HE_RATE && (rate & 15) <= 9;
}
static void sm_prepare_status(struct sk_buff *skb, bool ack, u8 attempts, u8 rate)
{
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);

	ieee80211_tx_info_clear_status(info);
	info->status.status_driver_data[0] = NULL;
	info->status.rates[0].idx = rate & ~S31_SOFTMAC_TX_HT_RATE;
	if (sm_tx_rate_is_he(rate)) {
		info->status.status_driver_data[0] = (void *)(unsigned long)rate;
		info->status.rates[0].idx = -1; /* HE is reported by tx_status_ext. */
	}
	info->status.rates[0].flags = (rate & S31_SOFTMAC_TX_HT_RATE) ? IEEE80211_TX_RC_MCS : 0;
	info->status.rates[0].count = max_t(u8, 1, attempts);
	info->status.rates[1].idx = -1;
	if (ack && !(info->flags & IEEE80211_TX_CTL_NO_ACK))
		info->flags |= IEEE80211_TX_STAT_ACK;
	/* Native completion reports each MPDU separately, including BA misses. */
	if (info->flags & IEEE80211_TX_CTL_AMPDU) {
		info->flags |= IEEE80211_TX_STAT_AMPDU;
		info->status.ampdu_len = 1;
		info->status.ampdu_ack_len = ack ? 1 : 0;
	}
}
static void sm_status(struct s31_sm *sm, struct sk_buff *skb, bool ack, u8 attempts, u8 rate)
{
 sm_prepare_status(skb, ack, attempts, rate);
 {
  unsigned long flags;
  spin_lock_irqsave(&sm->lock, flags);
  if (sm->running) {
   skb_queue_tail(&sm->status_queue, skb);
   sm_schedule_rx_locked(sm);
   skb = NULL;
  }
  spin_unlock_irqrestore(&sm->lock, flags);
  if (skb) ieee80211_free_txskb(sm->hw, skb);
 }
}

/* submit_lock serializes Linux queue publication across .tx and
 * the fallback/status work item. This helper never enters native firmware,
 * sleeps, or acquires the blob gate. IRQs remain enabled outside sm->lock. */
static void sm_submit_queued(struct s31_sm *sm)
{
 struct sk_buff *skb;
 unsigned long flags;
 unsigned int i;
 int ret;
 spin_lock_bh(&sm->submit_lock);
	for (;;) {
		u32 cookie;
		struct s31_softmac_tx_header header = { 0 };
		s8 rate;

		spin_lock_irqsave(&sm->lock, flags);
		/* A wake callback already owns the final flush. Neither NAPI nor
		 * fallback work may drain its partially prepared burst. */
		if (sm->txq_batch_users) {
			spin_unlock_irqrestore(&sm->lock, flags);
			break;
		}
		for (i = 0; i < S31_SOFTMAC_PENDING; i++)
			if (!sm->pending[i].skb)
				break;
		if (!sm->running || sm->tx_ba_flushing || i == S31_SOFTMAC_PENDING ||
		    skb_queue_empty(&sm->tx)) {
			spin_unlock_irqrestore(&sm->lock, flags);
			break;
		}
		skb = __skb_dequeue(&sm->tx);

		rate = IEEE80211_SKB_CB(skb)->control.rates[0].idx;
		/* A stopped BA session sends already queued Linux frames normally.
		 * This is serialized with ampdu_action by submit_lock. */
		if (!sm->tx_ba_operational)
			IEEE80211_SKB_CB(skb)->flags &= ~IEEE80211_TX_CTL_AMPDU;
		if (IEEE80211_SKB_CB(skb)->flags & IEEE80211_TX_CTL_AMPDU)
			sm->tx_ampdu_submitted++;
		/* RX may be HT/HE aggregated. Initial QoS TX remains unaggregated
		 * legacy OFDM; never interpret an MCS index as a legacy rate index. */
		if (ieee80211_is_data(((struct ieee80211_hdr *)skb->data)->frame_control))
			rate = clamp_t(unsigned int, READ_ONCE(sm_data_tx_rate), 4, 11);
		if (rate < 0 || rate >= ARRAY_SIZE(sm->rates)) {
			sm_tx_retire_locked(sm);
			spin_unlock_irqrestore(&sm->lock, flags);
			sm_status(sm, skb, false, 1, 0);
			continue;
		}

		cookie = ++sm->cookie ?: ++sm->cookie;
		sm->pending[i] = (struct sm_pending) {
			.skb = skb, .cookie = cookie, .expires = jiffies + 2 * HZ,
			.rate = rate,
		};

		spin_unlock_irqrestore(&sm->lock, flags);
		header.cookie = cookie;
		header.epoch = sm->epoch;
		header.rate_index = rate;
		if (READ_ONCE(sm->associated) && READ_ONCE(sm->tx_ht_peer) &&
		    ieee80211_is_data(((struct ieee80211_hdr *)skb->data)->frame_control) &&
		    !is_multicast_ether_addr(((struct ieee80211_hdr *)skb->data)->addr1)) {
			header.reserved[0] = 1;
			header.rate_index = min(sm_data_tx_mcs, 7);
		}
		if (header.reserved[0] &&
		    (IEEE80211_SKB_CB(skb)->flags & IEEE80211_TX_CTL_AMPDU))
			header.reserved[1] = 1;
		if (header.reserved[1] && READ_ONCE(sm->tx_he_peer)) {
			header.reserved[0] = 2;
			header.rate_index = sm_data_tx_mcs;
		}
		header.reserved[2] = (unsigned long)IEEE80211_SKB_CB(skb)->rate_driver_data[0];
		ret = esp32s31_radio_softmac_send_skb(&header, skb);

		if (ret) {
			spin_lock_irqsave(&sm->lock, flags);
			sm_pending_clear(sm, i);
			if (ret == -ENOSPC) {
				__skb_queue_head(&sm->tx, skb);
			} else sm_tx_retire_locked(sm);

			spin_unlock_irqrestore(&sm->lock, flags);
			if (ret != -ENOSPC)
				sm_status(sm, skb, false, 1, rate);
			if (ret == -ENOSPC)
				break;
		}
	}
 spin_unlock_bh(&sm->submit_lock);
}

/* No native/blob entry: stage the ready burst, retaining the existing
 * event/work fallback only when SRAM publication is temporarily full. */
static void sm_flush_tx(struct s31_sm *sm)
{
	unsigned long flags;
	sm_submit_queued(sm);
	spin_lock_irqsave(&sm->lock, flags);
	if (sm->running && !sm->txq_batch_users && !skb_queue_empty(&sm->tx))
		queue_work_on(sm_io_target(), sm_io_wq(), &sm->io);
	spin_unlock_irqrestore(&sm->lock, flags);
}

static int sm_tx_ba_flush(struct s31_sm *sm, const u8 *addr);

static void sm_io(struct work_struct *work)
{
	struct s31_sm *sm = container_of(work, struct s31_sm, io);
	struct sk_buff *skb;
	unsigned long flags;
	unsigned int i;
	bool aggregate_timeout = false;
	u8 ap[ETH_ALEN];

	/* A Linux timeout cannot release the native EB. Cancel the aggregate
	 * before retiring its cookie or reopening the pending gate. A failed
	 * cancel keeps publication closed and ownership intact. */
	spin_lock_irqsave(&sm->lock, flags);
	for (i = 0; i < S31_SOFTMAC_PENDING; i++)
		if (sm->pending[i].skb && !sm->pending[i].done &&
		    time_after_eq(jiffies, sm->pending[i].expires) &&
		    (IEEE80211_SKB_CB(sm->pending[i].skb)->flags & IEEE80211_TX_CTL_AMPDU)) {
			aggregate_timeout = true;
			ether_addr_copy(ap, sm->bssid);
			sm->tx_timeout++;
			break;
		}
	spin_unlock_irqrestore(&sm->lock, flags);
	if (aggregate_timeout) {
		int ret = sm_tx_ba_flush(sm, ap);
		pr_warn_ratelimited("esp32s31-softmac: TX BA timed out, cancel rc=%d; reconnect required\n", ret);
	}

	/* TX submission is serialized by submit_lock; RX callbacks use bounded rings. */
	for (i = 0; i < S31_SOFTMAC_PENDING; i++) {
		bool ack;
		u8 attempts, rate;

		spin_lock_irqsave(&sm->lock, flags);
		if (!sm->pending[i].skb || (!sm->pending[i].done &&
		    time_before(jiffies, sm->pending[i].expires))) {
			spin_unlock_irqrestore(&sm->lock, flags);
			continue;
		}
		if (!sm->pending[i].done &&
		    (IEEE80211_SKB_CB(sm->pending[i].skb)->flags & IEEE80211_TX_CTL_AMPDU)) {
			spin_unlock_irqrestore(&sm->lock, flags);
			continue;
		}
        if (sm_direct_status && sm_use_napi && sm->running && sm->pending[i].done) {
            sm_schedule_rx_locked(sm);
            spin_unlock_irqrestore(&sm->lock, flags);
            continue;
        }
		if (!sm->pending[i].done) sm->tx_timeout++;
		skb = sm->pending[i].skb;
		ack = sm->pending[i].done && sm->pending[i].ack;
		attempts = sm->pending[i].attempts;
		rate = sm->pending[i].rate;
		sm_pending_clear(sm, i);
		sm_tx_retire_locked(sm);
		if (ack) sm->tx_completed++; else sm->tx_failed++;
		spin_unlock_irqrestore(&sm->lock, flags);

		sm_status(sm, skb, ack, attempts, rate);
	}
	sm_submit_queued(sm);
}
/* Caller holds sm->lock. Used in the NAPI completion race recheck. */
/* Same rotating fairness as the original scan; native32 reads two words. */
static unsigned int sm_pending_done_index(struct s31_sm *sm)
{
 unsigned int i;
 {
  i = find_next_bit(sm->status_done, S31_SOFTMAC_PENDING, sm->status_cursor);
  return i < S31_SOFTMAC_PENDING ? i :
         find_first_bit(sm->status_done, S31_SOFTMAC_PENDING);
 }
 for (i = 0; i < S31_SOFTMAC_PENDING; i++) {
  unsigned int slot = (sm->status_cursor + i) % S31_SOFTMAC_PENDING;
  if (sm->pending[slot].skb && sm->pending[slot].done) return slot;
 }
 return S31_SOFTMAC_PENDING;
}

static bool sm_pending_status_ready_locked(struct s31_sm *sm)
{
 unsigned int i;
 if (!sm->running || !sm_direct_status || !sm_use_napi) return false;
 return !bitmap_empty(sm->status_done, S31_SOFTMAC_PENDING);
 for (i=0; i<S31_SOFTMAC_PENDING; i++)
  if (sm->pending[i].skb && sm->pending[i].done) return true;
 return false;
}
static struct sk_buff *sm_napi_take_status(struct s31_sm *sm)
{
 struct sm_pending completed;
 unsigned long flags;
 unsigned int i;
 spin_lock_irqsave(&sm->lock, flags);
 if (!sm->running) goto empty;
 /* Allocation reuses low slots. Rotate delivery so ongoing traffic cannot
  * indefinitely postpone a completed frame in a higher slot. */
 i = sm_pending_done_index(sm);
 if (i < S31_SOFTMAC_PENDING) {
  completed=sm->pending[i];
  sm_pending_clear(sm, i);
  sm_tx_retire_locked(sm);
  sm->status_cursor = (i + 1) % S31_SOFTMAC_PENDING;
  if (completed.ack) sm->tx_completed++; else sm->tx_failed++;
  sm_direct_status_calls++;
  spin_unlock_irqrestore(&sm->lock, flags);

  sm_prepare_status(completed.skb, completed.ack, completed.attempts, completed.rate);
  return completed.skb;
 }
empty:
 spin_unlock_irqrestore(&sm->lock, flags);
 return NULL;
}
static void sm_rx_status(struct ieee80211_rx_status *status, const struct sm_rx *rx)
{
	status->rate_idx = rx->rate;
 if (rx->phy_flags & S31_SOFTMAC_RX_CCMP_VALID)
  status->flag |= RX_FLAG_DECRYPTED | RX_FLAG_MIC_STRIPPED;
 /* Deliberately retain CCMP IV and omit RX_FLAG_PN_VALIDATED: mac80211
  * verifies replay counters even though TX keys remain software-only. */
	if (!rx->encoding) return;
	status->encoding = rx->encoding == 1 ? RX_ENC_HT : RX_ENC_HE;
	status->nss = 1;
	status->bw = rx->phy_flags & 2 ? RATE_INFO_BW_40 : RATE_INFO_BW_20;
	if (rx->encoding == 1 && (rx->phy_flags & 1))
		status->enc_flags |= RX_ENC_FLAG_SHORT_GI;
	if (rx->encoding == 2) status->he_gi = (rx->phy_flags >> 2) & 3;
}

/* RX and TX status share one poll: mac80211 forbids concurrent delivery.
 * No irqsafe handoff is used in NAPI mode. TX submission remains separate.
 */
static void sm_napi_status(struct s31_sm *sm, struct sk_buff *skb)
{
 struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
 unsigned int rate = (unsigned long)info->status.status_driver_data[0];
 if (sm_tx_rate_is_he(rate)) {
  pr_info_once("esp32s31-softmac: HE TX completion MCS%u GI1.6\n", rate & 15);
  struct ieee80211_rate_status rs = {
   .rate_idx = { .flags = RATE_INFO_FLAGS_HE_MCS, .mcs = rate & 15,
                 .nss = 1, .bw = RATE_INFO_BW_20,
                 .he_gi = NL80211_RATE_INFO_HE_GI_1_6 },
   .try_count = info->status.rates[0].count,
  };
  struct ieee80211_tx_status st = {
   .info = info, .skb = skb, .rates = &rs, .n_rates = 1,
  };
  rcu_read_lock();
  st.sta = rcu_dereference(sm->rx_sta);
  ieee80211_tx_status_ext(sm->hw, &st);
  rcu_read_unlock();
 } else {
  ieee80211_tx_status_skb(sm->hw, skb);
 }

}
static void sm_tx(struct ieee80211_hw *hw, struct ieee80211_tx_control *ctl,
                  struct sk_buff *skb);

/* The sole NAPI instance serializes scheduling rounds. Public mac80211
 * dequeue preserves AQL/airtime, stop reasons and key/PN handling. RCU spans
 * each returned queue's station/vif use; no pointer survives this pass.
 * A stopped gate sleeps until the existing completion -> wake_queues path.
 * Preserve undrained ACs at a gate/budget boundary, never poll an empty TXQ. */
static int sm_napi_txqs(struct s31_sm *sm, int budget)
{
 struct ieee80211_hw *hw = sm->hw;
 struct ieee80211_txq *queue;
 unsigned long flags;
 unsigned int ac, count = 0;
 u8 ready;
 bool flush;

 spin_lock_irqsave(&sm->lock, flags);
 if (!sm->running || !sm->txq_pending_ac || sm->tx_gate_stopped) {
  spin_unlock_irqrestore(&sm->lock, flags);
  return 0;
 }
 ready = sm->txq_pending_ac;
 sm->txq_pending_ac = 0;
 sm->txq_batch_users++;
 sm->txq_napi_passes++;
 spin_unlock_irqrestore(&sm->lock, flags);

 rcu_read_lock();
 for (ac = 0; ac < IEEE80211_NUM_ACS; ac++) {
  if (!(ready & BIT(ac))) continue;
  if (count == budget || READ_ONCE(sm->tx_gate_stopped)) break;
  ieee80211_txq_schedule_start(hw, ac);
  while (count < budget && (queue = ieee80211_next_txq(hw, ac))) {
   struct ieee80211_tx_control ctl = { .sta = queue->sta };
   struct sk_buff *skb;
   while (count < budget && (skb = ieee80211_tx_dequeue(hw, queue))) {
    sm_tx(hw, &ctl, skb);
    count++;
   }
   ieee80211_return_txq(hw, queue, false);
  }
  ieee80211_txq_schedule_end(hw, ac);
  if (count < budget && !READ_ONCE(sm->tx_gate_stopped)) ready &= ~BIT(ac);
 }
 rcu_read_unlock();

 spin_lock_irqsave(&sm->lock, flags);
 sm->txq_batch_users--;
 sm->txq_napi_frames += count;
 /* Preserve ACs not visited before the gate/budget closed. They may not
  * yet be marked DIRTY by dequeue. A full gate excludes these bits from
  * NAPI readiness; real completion reopens it before the next drain. */
 if (sm->running) sm->txq_pending_ac |= ready;
 flush = !sm->txq_batch_users && sm->running && !skb_queue_empty(&sm->tx);
 if (flush) sm->txq_batch_flushes++;
 spin_unlock_irqrestore(&sm->lock, flags);
 if (flush) sm_flush_tx(sm);

 return count;
}

static int sm_napi_poll(struct napi_struct *napi, int budget)
{
 struct s31_sm *sm = container_of(napi, struct s31_sm, napi);
 unsigned long flags;
 int done = 0, rx_since_flush = 0;

 sm->napi_polls++;
 sm_refill_empty(sm, true);
 while (done < budget) {
  struct sk_buff *skb;
  struct sm_rx *rx;
  struct ieee80211_rx_status *status;

  {
   skb=sm_napi_take_status(sm);
   if (skb) {
    sm_napi_status(sm, skb);
    sm_submit_queued(sm);
    done++;
    continue;
   }
  }
  skb = skb_dequeue(&sm->status_queue);
  if (skb) {
   sm_napi_status(sm, skb);
   done++;
   continue;
  }
  /* Avoid another driver-lock round trip on RX-only or gated iterations.
   * The helper and completion recheck validate readiness under lock. */
  if (sm_txq_napi && READ_ONCE(sm->txq_pending_ac) &&
      !READ_ONCE(sm->tx_gate_stopped)) {
   int sent = sm_napi_txqs(sm, budget - done);
   if (sent) { done += sent; if (done >= budget) break; }
  }
  spin_lock_irqsave(&sm->lock, flags);
  if (!sm->running || sm->head == sm->tail) {
   if (skb_queue_empty(&sm->status_queue) && !sm_pending_status_ready_locked(sm) &&
       !(sm_txq_napi && sm->txq_pending_ac && !sm->tx_gate_stopped)) {
    spin_unlock_irqrestore(&sm->lock, flags);
    /* Completion can flush GRO and generate TX; never hold sm->lock.
     * Recheck after completion so a producer racing SCHED clear cannot
     * lose the final wakeup. All delivery remains on this one NAPI.
     */
    napi_complete_done(napi, done);
    spin_lock_irqsave(&sm->lock, flags);
    if (sm->running && (sm->head != sm->tail ||
        !skb_queue_empty(&sm->status_queue) || sm_pending_status_ready_locked(sm) ||
        (sm_txq_napi && sm->txq_pending_ac && !sm->tx_gate_stopped)))
     napi_schedule(napi);
    spin_unlock_irqrestore(&sm->lock, flags);

    return done;
   }
   spin_unlock_irqrestore(&sm->lock, flags);
   continue;
  }
  rx = &sm->rx[sm->tail];
  spin_unlock_irqrestore(&sm->lock, flags);

  skb = rx->skb;
  {
   struct sk_buff *replacement = sm_alloc_rx_skb(sm, true);
   if (replacement) sm_install_rx_replacement(sm, rx, replacement);
   else { rx->skb = NULL; sm->rx_alloc_failed++; WRITE_ONCE(sm->rx_need_refill, true); }
  }

  if (skb) {
   status = IEEE80211_SKB_RXCB(skb);
   memset(status, 0, sizeof(*status));
   status->band = NL80211_BAND_2GHZ;
   status->freq = ieee80211_channel_to_frequency(rx->channel, NL80211_BAND_2GHZ);
   status->signal = rx->signal;
   sm_rx_status(status, rx);
  }

  /* Free the producer slot before synchronous decrypt/stack processing. */
  spin_lock_irqsave(&sm->lock, flags);
  sm->tail = (sm->tail + 1) & sm->rx_mask;
  s31_radio_softmac_rx_pending_set(sm->head != sm->tail);
  if (skb) sm->rx_delivered++; else sm->rx_alloc_failed++;
  spin_unlock_irqrestore(&sm->lock, flags);
  if (skb) {
   struct ieee80211_sta *sta;
   struct ieee80211_hdr *hdr = (void *)skb->data;

   /* The pre-RCU removal callback withdraws this hint before mac80211's
    * grace period. Unknown senders retain normal mac80211 lookup. */
   rcu_read_lock();
   sta = rcu_dereference(sm->rx_sta);
   if (!sta || !ieee80211_is_data(hdr->frame_control) ||
       !ether_addr_equal(hdr->addr2, sta->addr)) sta = NULL;

    ieee80211_rx_napi(sm->hw, sta, skb, napi);
   rcu_read_unlock();
   if (READ_ONCE(sm_gro_batch) &&
       ++rx_since_flush >= READ_ONCE(sm_gro_batch)) {
    /* Same sole NAPI owner; no driver lock or RCU read section held. */
    napi_gro_flush(napi, false);
    if (napi->gro.rx_count) {
     netif_receive_skb_list(&napi->gro.rx_list);
     INIT_LIST_HEAD(&napi->gro.rx_list);
     napi->gro.rx_count = 0;
    }
    rx_since_flush = 0;
   }

  }
  done++;
 }
 sm->napi_budget_hits++;

 return done;
}

static void sm_rx_io(struct work_struct *work)
{
 struct s31_sm *sm = container_of(work, struct s31_sm, rx_io);
 unsigned long flags;

 spin_lock_irqsave(&sm->lock, flags);
 if (sm->running) napi_schedule(&sm->napi);
 spin_unlock_irqrestore(&sm->lock, flags);
}
/* TX acceptance, watchdog rearm and stop all serialize on sm->lock. */

static void sm_timer(struct work_struct *work)
{
	struct s31_sm *sm = container_of(to_delayed_work(work), struct s31_sm, timer);

	unsigned long flags;

	if (!READ_ONCE(sm->running))
		return;
	sm_trim_idle_rx_skbs(sm);
	spin_lock_irqsave(&sm->lock, flags);
	if (sm->running) {
		queue_work_on(sm_io_target(), sm_io_wq(), &sm->io);
		/* Stop prevents rearm before cancel_delayed_work_sync(). */
		schedule_delayed_work(&sm->timer, msecs_to_jiffies(100));
	}
	spin_unlock_irqrestore(&sm->lock, flags);
}
/* One value status; caller holds sm->lock. No packet/native pointer retained. */
static bool sm_tx_status_locked(struct s31_sm *sm, struct s31_softmac_tx_status status)
{
 unsigned int i;
		for (i = 0; i < S31_SOFTMAC_PENDING; i++) {
			if (sm->pending[i].skb && sm->pending[i].cookie == status.cookie) {
				sm->pending[i].ack = status.success;
				sm->pending[i].attempts = status.attempts;
				if (status.rate < ARRAY_SIZE(sm->rates) ||
				    ((status.rate & ~7U) == S31_SOFTMAC_TX_HT_RATE) ||
				    sm_tx_rate_is_he(status.rate))
					sm->pending[i].rate = status.rate;

				sm->pending[i].done = true;
                __set_bit(i, sm->status_done);

				return true;
			}
		}
 return false;
}
static void sm_aux(void *context, u8 interface, const u8 *frame,
		   size_t length, u8 channel, s8 signal)
{
	struct s31_sm *sm = context;
	unsigned long flags;
	unsigned int next;
	struct s31_softmac_rx_header meta = {0};
	bool borrowed = false;

	if (!sm)
		return;
	if (interface == S31_WIFI_IF_SOFTMAC_VIEW) {
		const struct s31_softmac_rx_view *view = (const void *)frame;

		if (length < 24 || length > S31_SOFTMAC_FRAME_MAX || !view->frame)
			return;
		meta = view->header;
		frame = view->frame;
		length += sizeof(meta);
		interface = S31_WIFI_IF_SOFTMAC;
		borrowed = true;
	} else if (interface == S31_WIFI_IF_SOFTMAC && length >= sizeof(meta)) {
		memcpy(&meta, frame, sizeof(meta));
	}

	spin_lock_irqsave(&sm->lock, flags);

	if (!sm->running)
		goto out;
	if (borrowed)
		sm_rx_view_hits++;
	if (interface == S31_WIFI_IF_TX_STATUS) {
		bool matched = false;
		struct s31_softmac_tx_status status;
		if (length != sizeof(status))
			goto out;
		memcpy(&status, frame, sizeof(status));
		matched = sm_tx_status_locked(sm, status);
		if (sm_direct_status && sm_use_napi && matched) {
			sm_schedule_rx_locked(sm);
			goto out;
		}
	} else if (interface == S31_WIFI_IF_SOFTMAC &&
		   length >= sizeof(struct s31_softmac_rx_header) + 24 &&
		   length <= sizeof(struct s31_softmac_rx_header) + S31_SOFTMAC_FRAME_MAX &&
		   meta.encoding <= 2 && meta.rate_index < (meta.encoding == 1 ? 8 : meta.encoding == 2 ? 10 : ARRAY_SIZE(sm->rates)) &&
		   channel >= 1 && channel <= 11) {
		u8 rate = meta.rate_index;
		u8 encoding = meta.encoding, phy_flags = meta.flags;

		if (!borrowed)
			frame += sizeof(struct s31_softmac_rx_header);
		length -= sizeof(struct s31_softmac_rx_header);

        {
         const struct ieee80211_hdr *hdr = (const void *)frame;
         bool own = ether_addr_equal(hdr->addr1, sm->hw->wiphy->perm_addr);
         bool group = is_multicast_ether_addr(hdr->addr1);
         bool selected = is_valid_ether_addr(sm->bssid);
         /* STA-only frontend: unrelated unicast cannot reach this vif.
          * Preserve all BSS management during scan; in normal operation
          * mac80211 requests only the selected BSS beacon/probe response. */
         if ((!sm->scanning && !selected &&
              !(sm->filter_flags & FIF_OTHER_BSS) &&
              (ieee80211_is_beacon(hdr->frame_control) ||
               ieee80211_is_probe_resp(hdr->frame_control))) ||
             (!own && !group) ||
             (selected && ieee80211_is_data(hdr->frame_control) && group &&
              !ether_addr_equal(hdr->addr2, sm->bssid)) ||
             (selected && !(sm->filter_flags & FIF_BCN_PRBRESP_PROMISC) &&
              (ieee80211_is_beacon(hdr->frame_control) ||
               ieee80211_is_probe_resp(hdr->frame_control)) &&
              !ether_addr_equal(hdr->addr3, sm->bssid))) {
          sm->rx_filtered++;
          goto out;
         }
        }
		/* Keep the full pool warm for bulk downlink data. Beacons,
		 * multicast, and small background packets should not pin it. */
		if (sm->rx_idle_limit < sm->rx_slots && length >= 512 &&
		    ether_addr_equal(((const struct ieee80211_hdr *)frame)->addr1,
				     sm->hw->wiphy->perm_addr) &&
		    ieee80211_is_data_present(((const struct ieee80211_hdr *)frame)->frame_control)) {
			sm->rx_last_data = jiffies;
			if (sm->rx_trimmed) {
				sm->rx_trimmed = false;
				sm->rx_need_refill = true;
			}
		}
        next = (sm->head + 1) & sm->rx_mask;
		if (!sm->rx[sm->head].skb && sm->rx_idle_limit < sm->rx_slots) {
			sm->rx[sm->head].skb = sm_alloc_rx_skb_atomic();
			if (sm->rx[sm->head].skb) {
				sm->rx_emergency_alloc++;
				sm->rx_trimmed = false;
			}
		}
		if (!sm->rx[sm->head].skb) {
			sm->rx_alloc_failed++;
			sm_schedule_rx_locked(sm);
			goto out;
		}
		if (next == sm->tail) {
			sm->rx_dropped++;

			goto out;
		}
		/* The common MPDU uses the small preallocated slab. Only a larger
		 * A-MSDU needs an atomic expansion; failure drops this frame before
		 * copying and leaves the producer slot reusable. */
		if (length + 4 > skb_tailroom(sm->rx[sm->head].skb) &&
		    pskb_expand_head(sm->rx[sm->head].skb, 0,
			length + 4 - skb_tailroom(sm->rx[sm->head].skb), GFP_ATOMIC)) {
			sm->rx_alloc_failed++;
			goto out;
		}
		sm->rx[sm->head].length = length;
		sm->rx[sm->head].channel = channel;
		sm->rx[sm->head].signal = signal;
		sm->rx[sm->head].rate = rate;
		sm->rx[sm->head].encoding = encoding;
		sm->rx[sm->head].phy_flags = phy_flags;
		if (ieee80211_hdrlen(get_unaligned((const __le16 *)frame)) & 2)
			skb_reserve(sm->rx[sm->head].skb, 2);

        s31_sm_rx_copy(skb_put(sm->rx[sm->head].skb, length), frame, length);

        /* Only verified plaintext IP/TCP checksums can bypass the later
         * transport checksum. CCMP replay and normal MAC checks still run. */
        sm->rx[sm->head].skb->ip_summed=CHECKSUM_NONE;

		sm->head = next;
		s31_radio_softmac_rx_pending_set(true);
		sm->rx_queued++;
		sm->rx_peak = max(sm->rx_peak, (sm->head + sm->rx_slots - sm->tail) & sm->rx_mask);
		sm_schedule_rx_locked(sm);
		goto out;
	} else {
		goto out;
	}
	/* Queue while locked so .stop can forbid all new work before cancelling. */
	queue_work_on(sm_io_target(), sm_io_wq(), &sm->io);
out:

	spin_unlock_irqrestore(&sm->lock, flags);

}
static void sm_wakeup(void *context)
{
	struct s31_sm *sm = context;
	unsigned long flags;

	if (!sm)
		return;
	spin_lock_irqsave(&sm->lock, flags);
	if (sm->running && !skb_queue_empty(&sm->tx))
		queue_work_on(sm_io_target(), sm_io_wq(), &sm->io);
	spin_unlock_irqrestore(&sm->lock, flags);
}
static void sm_scan(void *ctx, int rc, const struct esp32s31_radio_wifi_ap *aps,
		    size_t count) {}
static void sm_unused_rx(void *ctx) {}
static void sm_connected(void *ctx, int rc, const u8 *bssid, u8 channel) {}
static void sm_disconnected(void *ctx, u16 reason) {}
static const struct esp32s31_radio_wifi_ops sm_radio_ops = {
	.scan_complete = sm_scan, .connected = sm_connected,
	.disconnected = sm_disconnected, .rx_ready = sm_unused_rx,
	.tx_wakeup = sm_wakeup, .receive_aux = sm_aux,
};
static int sm_start(struct ieee80211_hw *hw)
{
	struct s31_sm *sm = hw->priv;
	struct s31_wifi_control ctl = { .operation = S31_WIFI_SOFTMAC_START };
	int ret;
	if (sm_data_tx_mcs < -1 || sm_data_tx_mcs > 9 ||
	    (sm_data_tx_mcs > 7 && !sm_he_tx)) return -EINVAL;
	/* Without a credit gate generic TXQ drain is unbounded. */

	if (sm_tx_pending_limit > S31_SOFTMAC_PENDING ||
	    (sm_tx_amsdu && (!sm_ht || sm_data_tx_mcs < 0 || !sm_tx_pending_limit)))
		return -EINVAL;
	if (sm_tx_ampdu && (!sm_ht || sm_data_tx_mcs < 0 ||
	    (sm_tx_ampdu != 2 && sm_tx_ampdu != 4 &&
	     sm_tx_ampdu != 8 && sm_tx_ampdu != 16) ||
        sm_tx_pending_limit < sm_tx_ampdu ||
        sm_tx_pending_limit > (sm_tx_sg && sm_hw_tx_ccmp && sm_hw_tx_ccmp_fast ? 16 : 8)))
		return -EINVAL;

	{
		ctl.operation = sm_data_tx_mcs > 7 ? S31_WIFI_SOFTMAC_TX_HE9_CAP :
		                                  S31_WIFI_SOFTMAC_TX_HE_CAP;
		ret = esp32s31_radio_wifi_control(&ctl);
		if (ret) return -EOPNOTSUPP;
		ctl.operation = S31_WIFI_SOFTMAC_START;
	}
	WRITE_ONCE(sm->tx_he_peer, false);

	{
		ctl.operation = S31_WIFI_SOFTMAC_TX_AGG_CAP;
		ret = esp32s31_radio_wifi_control(&ctl);
		if (ret) return -EOPNOTSUPP;
		ctl.operation = S31_WIFI_SOFTMAC_START;
	}

	{
		ctl.operation = S31_WIFI_SOFTMAC_TX_HT_CAP;
		ret = esp32s31_radio_wifi_control(&ctl);
		if (ret) return -EOPNOTSUPP;
		ctl.operation = S31_WIFI_SOFTMAC_START;
	}
	WRITE_ONCE(sm->tx_ht_peer, false);
	sm->tx_ba_requested = sm->tx_ba_operational = sm->tx_native_peer = false;
	sm->tx_ba_flushing = false;

	memcpy(ctl.mac, hw->wiphy->perm_addr, ETH_ALEN);
	ctl.channel = sm->channel ?: 1;
	ctl.offset = ++sm->epoch;
	ctl.field = (sm_ht ? 1 : 0) | (sm_ht && sm_he ? 2 : 0) |
		(sm_rx_view ? S31_SOFTMAC_START_RX_VIEW : 0) |
		(sm_rx_filter && sm_native_rx_filter ? S31_SOFTMAC_START_RX_RA_FILTER : 0) |
  (sm_native_rx_error_filter ? S31_SOFTMAC_START_RX_DROP_ERRORS : 0) |
  (sm_native_bssid_filter && sm_rx_filter && sm_hw_bssid_filter ? S31_SOFTMAC_START_RX_BSSID_FILTER : 0);
 ctl.total = sm_direct_rx;
	ret = esp32s31_radio_wifi_control(&ctl);
	if (ret)
		return ret;
	{
		napi_enable(&sm->napi);
		sm->napi_active = true;
	}
	WRITE_ONCE(sm->running, true);
	sm->rx_ba_mask = 0;
	schedule_delayed_work(&sm->timer, msecs_to_jiffies(100));
	return 0;
}
/* NAPI and producers are stopped. Keep the retained skb window anchored
 * to the old consumer cursor; .stop clears all retained skb payloads below.
 * Rewinding to zero can put a trimmed window behind the new producer. */
static void sm_restart_rx_ring(struct s31_sm *sm)
{
	sm->head = sm->tail;
	s31_radio_softmac_rx_pending_set(false);
}

static void sm_stop(struct ieee80211_hw *hw, bool suspend)
{
	struct s31_sm *sm = hw->priv;
	struct s31_wifi_control ctl = { .operation = S31_WIFI_SOFTMAC_STOP };
	struct sk_buff *skb;
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&sm->lock, flags);
	sm->running = false;
 sm->txq_pending_ac = 0;
	s31_radio_softmac_rx_pending_set(false);
 sm->associated = false;
 sm->scanning = false;
	spin_unlock_irqrestore(&sm->lock, flags);
	/* Join an atomic .tx submission before stopping payload/rings. New
         * submissions see running=false and cannot reserve another skb. */
        spin_lock_bh(&sm->submit_lock);
        spin_unlock_bh(&sm->submit_lock);
	/* No producer can submit after running=false. Completion occurs under
         * sm->lock; acquire it once more before disabling/freeing NAPI. */
	wait_for_completion(&sm->rx_kick_done);
	spin_lock_irqsave(&sm->lock, flags);
	spin_unlock_irqrestore(&sm->lock, flags);
	cancel_delayed_work_sync(&sm->timer);
	cancel_work_sync(&sm->io);
	cancel_work_sync(&sm->rx_io);
	cancel_work_sync(&sm->status_io);
 cancel_work_sync(&sm->filter_io);
 WRITE_ONCE(sm_hw_bssid_filter_active,false);
	if (sm->napi_active) {
		napi_disable(&sm->napi);
		sm->napi_active = false;
	}
	if (esp32s31_radio_wifi_control(&ctl))
		pr_warn("esp32s31-softmac: radio stop failed\n");
	esp32s31_radio_idle_poll_set(false);
	while ((skb = __skb_dequeue(&sm->tx)))
		ieee80211_free_txskb(hw, skb);
	for (i = 0; i < S31_SOFTMAC_PENDING; i++) {
		if (sm->pending[i].skb)
			ieee80211_free_txskb(hw, sm->pending[i].skb);
		sm_pending_clear(sm, i);
	}
	while ((skb=skb_dequeue(&sm->status_queue)))
		ieee80211_free_txskb(hw,skb);
	sm->tx_outstanding = 0;
	if (sm->tx_gate_stopped) {
		sm->tx_gate_stopped = false;
		ieee80211_wake_queues(hw);
	}
	sm->status_cursor = 0;
	sm_restart_rx_ring(sm);
	for (i = 0; i < sm->rx_slots; i++) {
		struct sk_buff *skb = sm->rx[i].skb;
		if (!skb) continue;
		skb_trim(skb, 0);
		/* Undo the per-frame QoS alignment for a retained empty slot. */
		skb->data = PTR_ALIGN_DOWN(skb->data, 4);
		skb_reset_tail_pointer(skb);
	}
}
static void sm_tx_inner(struct ieee80211_hw *hw, struct ieee80211_tx_control *ctl,
		  struct sk_buff *skb)
{
	struct s31_sm *sm = hw->priv;
	struct ieee80211_hdr *hdr = (void *)skb->data;
	unsigned long flags;
	bool accepted = false;
	bool defer_submit = false;
	bool request_ba = false;
	struct ieee80211_tx_info *txinfo = IEEE80211_SKB_CB(skb);
	u8 key_token = txinfo->control.hw_key ? txinfo->control.hw_key->hw_key_idx : 0;
	{
		u32 basic = txinfo->control.vif ?
			READ_ONCE(txinfo->control.vif->bss_conf.basic_rates) : 0;
		/* All advertised rates are 2.4 GHz indices0..11. Before an AP
		 * supplies a basic set, retain the mandatory 1 Mbps legacy rate. */
		basic &= GENMASK(11, 0);
		txinfo->control.rates[0].idx = basic ? __ffs(basic) : 0;
		txinfo->control.rates[0].count = 1;
		txinfo->control.rates[0].flags = 0;
	}
	/* .tx owns the driver area. Preserve rates, but never retain/dereference
	 * a key_conf pointer after this callback or after key removal. */
	txinfo->rate_driver_data[0] = (void *)(unsigned long)key_token;

	if (skb->len < 24 || skb->len > S31_SOFTMAC_FRAME_MAX ||
	    (!ieee80211_is_mgmt(hdr->frame_control) &&
	     !ieee80211_is_data(hdr->frame_control)) ||
	    (ieee80211_has_protected(hdr->frame_control) &&
	     (!ieee80211_is_data(hdr->frame_control) || skb->len < 40)) ||
	    ieee80211_has_a4(hdr->frame_control) ||
	    ieee80211_has_morefrags(hdr->frame_control) ||
	    (le16_to_cpu(hdr->seq_ctrl) & IEEE80211_SCTL_FRAG)) {
		sm_status(sm, skb, false, 1, 0);
		return;
	}
	spin_lock_irqsave(&sm->lock, flags);
	if (sm_tx_ampdu && sm->associated && sm->tx_ht_peer && ctl->sta &&
	    ctl->sta->wme && !sm->tx_ba_requested &&
	    ieee80211_is_data_qos(hdr->frame_control) &&
	    !(*ieee80211_get_qos_ctl(hdr) & IEEE80211_QOS_CTL_TID_MASK) &&
	    skb->protocol != cpu_to_be16(ETH_P_PAE)) {
		sm->tx_ba_requested = true;
		request_ba = true;
	}
	if (sm->running && !sm->tx_ba_flushing && skb_queue_len(&sm->tx) < SM_TX_QUEUED) {
		__skb_queue_tail(&sm->tx, skb);
		if (sm_tx_sg && (skb_shinfo(skb)->nr_frags ||
		    (skb_shinfo(skb)->frag_list &&
		     skb_shinfo(skb_shinfo(skb)->frag_list)->nr_frags)))
			sm_tx_sg_frames++;
		if (txinfo->control.flags & IEEE80211_TX_CTRL_FAST_XMIT) {
			sm->tx_fast_frames++;
			if (ieee80211_is_data_qos(hdr->frame_control) &&
			    (*ieee80211_get_qos_ctl(hdr) & IEEE80211_QOS_CTL_A_MSDU_PRESENT))
				sm->tx_fast_amsdu++;
		}
		sm->tx_outstanding++;

		sm->tx_depth_peak = max(sm->tx_depth_peak, sm->tx_outstanding);
		if (sm_tx_pending_limit && !sm->tx_gate_stopped &&
		    sm->tx_outstanding >= sm_tx_pending_limit) {
			sm->tx_gate_stopped = true;
			sm->tx_gate_stops++;
			ieee80211_stop_queues(hw);
		}
		defer_submit = sm->txq_batch_users != 0;
		if (defer_submit) sm->txq_batch_frames++;
		sm_direct_tx_calls++;
		accepted = true;
	}
	spin_unlock_irqrestore(&sm->lock, flags);
	/* Uses mac80211's existing BA work; never invokes native PP here. */
	if (request_ba) ieee80211_start_tx_ba_session(ctl->sta, 0, 0);
	if (!accepted)
		sm_status(sm, skb, false, 1, 0);
	else if (sm_direct_tx && !defer_submit)
		sm_flush_tx(sm);
}

static void sm_tx(struct ieee80211_hw *hw, struct ieee80211_tx_control *ctl,
                  struct sk_buff *skb)
{
 sm_tx_inner(hw, ctl, skb);

}
static void sm_wake_txq(struct ieee80211_hw *hw, struct ieee80211_txq *txq)
{
 struct s31_sm *sm = hw->priv;
 unsigned long flags;
 bool flush = false;
 {
  spin_lock_irqsave(&sm->lock, flags);
  if (sm->running && txq->ac < IEEE80211_NUM_ACS) {
   sm->txq_pending_ac |= BIT(txq->ac);
   sm->txq_napi_wakes++;
   sm_schedule_rx_locked(sm);
  }
  spin_unlock_irqrestore(&sm->lock, flags);
  return;
 }
 {
  spin_lock_irqsave(&sm->lock, flags);
  sm->txq_batch_users++;
  spin_unlock_irqrestore(&sm->lock, flags);
 }
 /* Generic dequeue retains airtime fairness and queue-stop handling. Its
  * existing credit gate bounds this burst; singleton traffic flushes now. */
 ieee80211_handle_wake_tx_queue(hw, txq);
 {
  spin_lock_irqsave(&sm->lock, flags);
  if (!WARN_ON_ONCE(!sm->txq_batch_users)) {
   sm->txq_batch_users--;
   flush = !sm->txq_batch_users && sm->running && !skb_queue_empty(&sm->tx);
   if (flush) sm->txq_batch_flushes++;
  }
  spin_unlock_irqrestore(&sm->lock, flags);
  if (flush) sm_flush_tx(sm);
 }

}

static int sm_add(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	struct s31_sm *sm = hw->priv;

	if (vif->type != NL80211_IFTYPE_STATION || sm->has_vif ||
	    !ether_addr_equal(vif->addr, hw->wiphy->perm_addr))
		return -EOPNOTSUPP;
	/* Include MAC/LLC plus software CCMP IV (8) and MIC (8). */
	ieee80211_vif_to_wdev(vif)->netdev->max_mtu = S31_SOFTMAC_FRAME_MAX - 48;
	ieee80211_vif_to_wdev(vif)->netdev->mtu = 1400;
	sm->has_vif = true;
	return 0;
}
static void sm_remove(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	((struct s31_sm *)hw->priv)->has_vif = false;
}
static int sm_config(struct ieee80211_hw *hw, int radio_idx, u32 changed)
{
	struct s31_sm *sm = hw->priv;
	struct s31_wifi_control ctl = { .operation = S31_WIFI_SET_CHANNEL };
	int ret;

	if (!(changed & IEEE80211_CONF_CHANGE_CHANNEL))
		return 0;
	if (!hw->conf.chandef.chan ||
	    (hw->conf.chandef.width != NL80211_CHAN_WIDTH_20_NOHT &&
	     hw->conf.chandef.width != NL80211_CHAN_WIDTH_20))
		return -EOPNOTSUPP;
	ctl.channel = hw->conf.chandef.chan->hw_value;
	ret = esp32s31_radio_wifi_control(&ctl);
	if (!ret)
		sm->channel = ctl.channel;
	return ret;
}
/* configure_filter can run atomically. Native register changes therefore
 * use a separate work item; scan_start joins it before scanning begins. */
static void sm_filter_io(struct work_struct *work)
{
 struct s31_sm *sm=container_of(work,struct s31_sm,filter_io);
 struct s31_wifi_control ctl={.operation=S31_WIFI_SOFTMAC_BSSID_FILTER};
 unsigned long flags;
 bool running;
 spin_lock_irqsave(&sm->lock,flags);
 running=sm->running;
 ctl.field=running && sm->associated && !sm->scanning &&
   !(sm->filter_flags & (FIF_OTHER_BSS | FIF_BCN_PRBRESP_PROMISC)) &&
   is_valid_ether_addr(sm->bssid);
 ether_addr_copy(ctl.mac,sm->bssid);
 spin_unlock_irqrestore(&sm->lock,flags);
 if (!running || !sm_hw_bssid_filter) return;
 if (esp32s31_radio_wifi_control(&ctl))
  pr_warn("esp32s31-softmac: hardware BSSID filter update failed\n");
 else {
  WRITE_ONCE(sm_hw_bssid_filter_active,!!ctl.field);
  sm_hw_bssid_filter_updates++;
 }
}
static void sm_queue_filter_locked(struct s31_sm *sm)
{
 if (sm_hw_bssid_filter && sm->running)
  queue_work(system_dfl_wq,&sm->filter_io);
}
static void sm_filter(struct ieee80211_hw *hw, unsigned int changed,
		      unsigned int *total, u64 multicast)
{
	unsigned long flags;
	*total &= FIF_ALLMULTI | FIF_BCN_PRBRESP_PROMISC | FIF_OTHER_BSS;
	spin_lock_irqsave(&((struct s31_sm *)hw->priv)->lock, flags);
	((struct s31_sm *)hw->priv)->filter_flags = *total;
 sm_queue_filter_locked(hw->priv);
	spin_unlock_irqrestore(&((struct s31_sm *)hw->priv)->lock, flags);
}
/* Avoid unsolicited beacon updates destabilizing a multipart BSS dump.
 * Scan callbacks bracket every mac80211 software scan; selected BSS beacons
 * remain enabled outside scans for association and connection monitoring. */
static void sm_sw_scan_start(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
                             const u8 *addr)
{
 struct s31_sm *sm = hw->priv;
 unsigned long flags;
 spin_lock_irqsave(&sm->lock, flags);
 sm->scanning = true;
 sm_queue_filter_locked(sm);
 spin_unlock_irqrestore(&sm->lock, flags);
 flush_work(&sm->filter_io);
}
static void sm_sw_scan_complete(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
 struct s31_sm *sm = hw->priv;
 unsigned long flags;
 spin_lock_irqsave(&sm->lock, flags);
 sm->scanning = false;
 sm_queue_filter_locked(sm);
 spin_unlock_irqrestore(&sm->lock, flags);
}
static void sm_bss_changed(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
                           struct ieee80211_bss_conf *info, u64 changed)
{
 struct s31_sm *sm = hw->priv;
 unsigned long flags;
 spin_lock_irqsave(&sm->lock,flags);
 if (changed & BSS_CHANGED_ASSOC) sm->associated=vif->cfg.assoc;
 if (changed & BSS_CHANGED_BSSID) {
  if (info->bssid) ether_addr_copy(sm->bssid,info->bssid);
  else eth_zero_addr(sm->bssid);
 }
 if ((changed & BSS_CHANGED_ASSOC) && !vif->cfg.assoc)
  sm_queue_filter_locked(sm);
 spin_unlock_irqrestore(&sm->lock,flags);
 if (changed & BSS_CHANGED_ASSOC)
  esp32s31_radio_idle_poll_set(sm_auto_idle_poll && vif->cfg.assoc &&
                                READ_ONCE(sm->running));
 if (sm_hw_bssid_filter && (changed & BSS_CHANGED_ASSOC) && !vif->cfg.assoc)
  flush_work(&sm->filter_io);
 if ((sm_hw_ccmp || sm_hw_tx_ccmp) && (changed & BSS_CHANGED_ASSOC) && !vif->cfg.assoc && READ_ONCE(sm->running)) {
  struct s31_wifi_control ctl = { .operation = S31_WIFI_SOFTMAC_RX_KEY_DEL, .length = 16 };
  /* Software TX keys do not receive DISABLE_KEY callbacks. Disassociation
   * therefore explicitly retires all RX-only hardware slots. */
  for (ctl.field = 0; ctl.field < 5; ctl.field++)
   if (esp32s31_radio_wifi_control(&ctl)) pr_warn("esp32s31-softmac: RX key retirement failed\n");
 }
 if (sm_he && READ_ONCE(sm->running) &&
     (changed & (BSS_CHANGED_ASSOC | BSS_CHANGED_BSSID | BSS_CHANGED_HE_BSS_COLOR))) {
  struct s31_wifi_control ctl = { .operation = S31_WIFI_SOFTMAC_HE_BSS };
  ctl.length = vif->cfg.assoc && info->he_support;
  ctl.field = vif->cfg.aid;
  ctl.offset = info->he_bss_color.color;
  ctl.total = info->he_bss_color.enabled;
  ctl.hidden = info->he_bss_color.partial;
  if (info->bssid) ether_addr_copy(ctl.mac, info->bssid);
  if (esp32s31_radio_wifi_control(&ctl))
   pr_warn("esp32s31-softmac: HE BSS setup failed\n");
 }
 spin_lock_irqsave(&sm->lock,flags);
 sm_queue_filter_locked(sm);
 spin_unlock_irqrestore(&sm->lock,flags);
}
/* Single AP frontend. Join SRAM publication before any native cancel.
 * No submit/spin lock is held while entering the sleeping control API. */
static void sm_tx_ba_disable(struct s31_sm *sm, bool flushing)
{
	unsigned long flags;
	spin_lock_bh(&sm->submit_lock);
	spin_lock_irqsave(&sm->lock, flags);
	sm->tx_ba_operational = false;
	sm->tx_ba_flushing = flushing;
	spin_unlock_irqrestore(&sm->lock, flags);
	spin_unlock_bh(&sm->submit_lock);
}

static bool sm_tx_ba_pending(struct s31_sm *sm)
{
	unsigned long flags;
	unsigned int i;
	bool busy = false;
	spin_lock_irqsave(&sm->lock, flags);
	for (i = 0; i < S31_SOFTMAC_PENDING; i++)
		if (sm->pending[i].skb &&
		    (IEEE80211_SKB_CB(sm->pending[i].skb)->flags & IEEE80211_TX_CTL_AMPDU)) {
			busy = true; break;
		}
	spin_unlock_irqrestore(&sm->lock, flags);
	return busy;
}

static int sm_tx_ba_flush(struct s31_sm *sm, const u8 *addr)
{
	struct s31_wifi_control c = { .operation = S31_WIFI_SOFTMAC_TX_BA_FLUSH };
	struct sk_buff_head dropped;
	struct sk_buff *skb;
	unsigned long flags;
	unsigned int i;
	int ret = 0;
	sm_tx_ba_disable(sm, true);
	if (sm->tx_native_peer) {
		memcpy(c.mac, addr, ETH_ALEN);
		ret = esp32s31_radio_wifi_control(&c);
		if (ret) return ret;
	}
	skb_queue_head_init(&dropped);
	spin_lock_irqsave(&sm->lock, flags);
	while ((skb = __skb_dequeue(&sm->tx))) {
		__skb_queue_tail(&dropped, skb);
		sm_tx_retire_locked(sm);
	}
	for (i = 0; i < S31_SOFTMAC_PENDING; i++) {
		if (!sm->pending[i].skb) continue;
		__skb_queue_tail(&dropped, sm->pending[i].skb);
		sm_pending_clear(sm, i);
		sm_tx_retire_locked(sm);
	}
	spin_unlock_irqrestore(&sm->lock, flags);
	/* A later SRAM-ring rejection/old cookie is harmless after unpublish. */
	while ((skb = __skb_dequeue(&dropped))) ieee80211_free_txskb(sm->hw, skb);
	return 0;
}

static int sm_tx_ba_action(struct s31_sm *sm, struct ieee80211_vif *vif,
			   struct ieee80211_ampdu_params *p)
{
	struct s31_wifi_control c = {0};
	unsigned long deadline;
	int ret = 0;
	if (!sm_tx_ampdu || p->tid || !READ_ONCE(sm->tx_ht_peer)) {
		/* Stop callbacks remain valid after sta_pre_rcu_remove. */
		if (p->action == IEEE80211_AMPDU_TX_START ||
		    p->action == IEEE80211_AMPDU_TX_OPERATIONAL) return -EOPNOTSUPP;
		if (!sm_tx_ampdu || p->tid) return -EOPNOTSUPP;
	}
	memcpy(c.mac, p->sta->addr, ETH_ALEN);
	switch (p->action) {
	case IEEE80211_AMPDU_TX_START:
		if (!sm->tx_native_peer) {
			c.operation = S31_WIFI_SOFTMAC_TX_PEER_ADD;
			/* sta->aid is what an AP assigned to its client. Our station's
			 * association response AID belongs to the managed interface. */
			c.offset = vif->cfg.aid;
			c.field = READ_ONCE(sm->tx_he_peer) ? sm_data_tx_mcs : min(sm_data_tx_mcs, 7);
			c.total = p->sta->deflink.ht_cap.ampdu_factor;
			c.length = p->sta->deflink.ht_cap.ampdu_density;
			c.dtim_period = sm_tx_ampdu;
			c.hidden = READ_ONCE(sm->tx_he_peer);
			ret = esp32s31_radio_wifi_control(&c);
			if (!ret) sm->tx_native_peer = true;
		}
		if (!ret) ret = IEEE80211_AMPDU_TX_START_IMMEDIATE;
		break;
	case IEEE80211_AMPDU_TX_OPERATIONAL:
		if (p->buf_size < sm_tx_pending_limit) return -EOPNOTSUPP;
		c.operation = S31_WIFI_SOFTMAC_TX_BA_ON;
		c.length = p->buf_size;
		ret = esp32s31_radio_wifi_control(&c);
		if (!ret) {
			spin_lock_bh(&sm->submit_lock);
			WRITE_ONCE(sm->tx_ba_operational, true);
			spin_unlock_bh(&sm->submit_lock);
		}
		break;
	case IEEE80211_AMPDU_TX_STOP_CONT:
		sm_tx_ba_disable(sm, false);
		/* Existing native EBs finish before BA is disabled. The native
		 * check also covers cookies already expired by the Linux timer. */
		deadline = jiffies + msecs_to_jiffies(2200);
		c.operation = S31_WIFI_SOFTMAC_TX_BA_OFF;
		while (sm->tx_native_peer) {
			ret = sm_tx_ba_pending(sm) ? -EBUSY : esp32s31_radio_wifi_control(&c);
			if (!ret) break;
			if (time_after_eq(jiffies, deadline)) {
				ret = sm_tx_ba_flush(sm, p->sta->addr);
				if (!ret) sm_tx_ba_disable(sm, false);
				break;
			}
			usleep_range(1000, 2000);
		}
		if (!ret) ieee80211_stop_tx_ba_cb_irqsafe(vif, p->sta->addr, p->tid);
		break;
	case IEEE80211_AMPDU_TX_STOP_FLUSH:
	case IEEE80211_AMPDU_TX_STOP_FLUSH_CONT:
		ret = sm_tx_ba_flush(sm, p->sta->addr);
		break;
	default:
		return -EOPNOTSUPP;
	}
	pr_info("esp32s31-softmac: TX BA action=%u tid=%u ssn=%u window=%u cap=%u aid=%u rc=%d\n",
		p->action, p->tid, p->ssn, p->buf_size, sm_tx_ampdu, vif->cfg.aid, ret);
	return ret;
}

static int sm_sta_add(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
                      struct ieee80211_sta *sta)
{
 struct s31_sm *sm = hw->priv;
 WRITE_ONCE(sm->tx_ht_peer, sm_data_tx_mcs >= 0 &&
            sta->deflink.ht_cap.ht_supported &&
            (sta->deflink.ht_cap.mcs.rx_mask[0] & BIT(min(sm_data_tx_mcs, 7))));
 WRITE_ONCE(sm->tx_he_peer, sm_he_tx && sm->tx_ht_peer &&
            sta->deflink.he_cap.has_he &&
            (le16_to_cpu(sta->deflink.he_cap.he_mcs_nss_supp.rx_mcs_80) & 3) != 3 &&
            (sm_data_tx_mcs <= 7 ||
             (le16_to_cpu(sta->deflink.he_cap.he_mcs_nss_supp.rx_mcs_80) & 3) >= 1));
 pr_info("esp32s31-softmac: peer HT=%u HE=%u rxHE=%04x selectedHE=%u\n",
         sta->deflink.ht_cap.ht_supported, sta->deflink.he_cap.has_he,
         le16_to_cpu(sta->deflink.he_cap.he_mcs_nss_supp.rx_mcs_80), sm->tx_he_peer);
 /* A single associated AP is an RX lookup hint, never a substitute for
  * mac80211's address, key, duplicate or replay checks. */
 if (!rcu_access_pointer(sm->rx_sta)) rcu_assign_pointer(sm->rx_sta, sta);
 return 0;
}
static void sm_sta_pre_remove(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
                              struct ieee80211_sta *sta)
{
 struct s31_sm *sm = hw->priv;
 WRITE_ONCE(sm->tx_ht_peer, false);
 WRITE_ONCE(sm->tx_he_peer, false);
 if (rcu_access_pointer(sm->rx_sta) == sta) RCU_INIT_POINTER(sm->rx_sta, NULL);
}
static int sm_sta_remove(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
                         struct ieee80211_sta *sta)
{
 struct s31_sm *sm = hw->priv;
 struct s31_wifi_control c = { .operation = S31_WIFI_SOFTMAC_TX_PEER_DEL };
 int ret = 0;
 if (sm->tx_native_peer) {
  ret = sm_tx_ba_flush(sm, sta->addr);
  if (!ret) {
   memcpy(c.mac, sta->addr, ETH_ALEN);
   ret = esp32s31_radio_wifi_control(&c);
  }
  if (ret) return ret;
  sm->tx_native_peer = false;
 }
 sm->tx_ba_requested = false;
 sm_tx_ba_disable(sm, false);
 return ret;
}
static int sm_rts(struct ieee80211_hw *hw, int radio_idx, u32 threshold)
{
	return threshold == IEEE80211_MAX_RTS_THRESHOLD ? 0 : -EOPNOTSUPP;
}

static int sm_key(struct ieee80211_hw *hw, enum set_key_cmd cmd,
		  struct ieee80211_vif *vif, struct ieee80211_sta *sta,
		  struct ieee80211_key_conf *key)
{
	struct s31_wifi_control ctl = {0};
	struct s31_sm *sm = hw->priv;
	bool pairwise = key->flags & IEEE80211_KEY_FLAG_PAIRWISE;
	bool tx_offload = sm_hw_tx_ccmp && pairwise && sm->tx_key_generation < 127;
	u8 token = 0;
	int ret;
	if (key->cipher != WLAN_CIPHER_SUITE_CCMP)
		return -EOPNOTSUPP;
	{
		if (key->keylen != 16 || key->keyidx > 3 || (pairwise && !sta))
			return -EOPNOTSUPP;
		ctl.operation = cmd == DISABLE_KEY ? S31_WIFI_SOFTMAC_RX_KEY_DEL :
			S31_WIFI_SOFTMAC_RX_KEY_ADD;
		ctl.field = pairwise ? 4 : key->keyidx;
		ctl.offset = key->keyidx;
		ctl.length = key->keylen;
		ctl.total = (sm_hw_ccmp || sm_hw_tx_ccmp) ? S31_SOFTMAC_KEY_RX_CCMP : 0;
		if (cmd != DISABLE_KEY && tx_offload) {
			/* Never wrap a token during one frontend lifetime. At exhaustion
			 * new keys retain software TX; old queued plaintext is rejected. */
			token = ((sm->tx_key_generation + 1) << 1) | S31_SOFTMAC_TX_MAC_CCMP;
			ctl.beacon_interval = token;
			ctl.total |= S31_SOFTMAC_KEY_TX_CCMP;
		}
		memcpy(ctl.mac, pairwise ? sta->addr : vif->cfg.ap_addr, ETH_ALEN);
		memcpy(ctl.data, key->key, key->keylen);
		ret = esp32s31_radio_wifi_control(&ctl);
		memzero_explicit(&ctl, sizeof(ctl));
		if (ret) return ret;
	}
	if (cmd == DISABLE_KEY)
		return 0;
	if (tx_offload) {
		sm->tx_key_generation++;
		/* Driver-private token; the payload selects physical MAC slot 4. */
		key->hw_key_idx = token;
		key->flags |= IEEE80211_KEY_FLAG_GENERATE_IV |
			IEEE80211_KEY_FLAG_SW_MGMT_TX;
		pr_info("esp32s31-softmac: pairwise MAC TX CCMP, Linux IV/PN\n");
		return 0;
	}
	pr_info("esp32s31-softmac: CCMP %s key index=%u uses software crypto\n",
		key->flags & IEEE80211_KEY_FLAG_PAIRWISE ? "pairwise" : "group",
		key->keyidx);
	return 1;
}
static int sm_conf_tx(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		     unsigned int link, u16 ac, const struct ieee80211_tx_queue_params *p)
{
	struct s31_sm *sm = hw->priv;
	struct s31_wifi_control ctl = { .operation = S31_WIFI_SOFTMAC_EDCA };
	if (link || ac > 3 || p->cw_min > p->cw_max || p->cw_max > 1023 ||
	    p->aifs > 15 || p->uapsd || p->txop > 2047) return -EOPNOTSUPP;
	if (!READ_ONCE(sm->running)) return 0; /* Native initialization defaults. */
	ctl.field = ac; ctl.offset = p->aifs;
	ctl.total = fls(p->cw_min + 1) - 1;
	ctl.length = fls(p->cw_max + 1) - 1;
	ctl.beacon_interval = p->txop * 32;
	return esp32s31_radio_wifi_control(&ctl);
}
static const unsigned int sm_rx_ba_window = 64;

static int sm_ampdu(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		    struct ieee80211_ampdu_params *p)
{
	struct s31_sm *sm = hw->priv;
	struct s31_wifi_control ctl = {0};
	unsigned int slot;
	int ret;
	if (p->tid > 15) return -EINVAL;
	if (!READ_ONCE(sm_rx_ba) && p->action == IEEE80211_AMPDU_RX_START)
		return -EOPNOTSUPP;
	if (p->action != IEEE80211_AMPDU_RX_START && p->action != IEEE80211_AMPDU_RX_STOP)
		return sm_tx_ba_action(sm, vif, p);
	for (slot = 0; slot < 4; slot++)
		if ((sm->rx_ba_mask & BIT(slot)) && sm->rx_ba_tid[slot] == p->tid) break;
	if (p->action == IEEE80211_AMPDU_RX_STOP && slot == 4) return 0;
	if (p->action == IEEE80211_AMPDU_RX_START && slot == 4)
		for (slot = 0; slot < 4; slot++) if (!(sm->rx_ba_mask & BIT(slot))) break;
	if (slot == 4) return -ENOSPC;
	ctl.operation = p->action == IEEE80211_AMPDU_RX_START ? S31_WIFI_SOFTMAC_RX_BA_ADD : S31_WIFI_SOFTMAC_RX_BA_DEL;
	ctl.field = slot; ctl.offset = p->ssn; ctl.total = p->tid; ctl.length = p->buf_size;
	memcpy(ctl.mac, p->sta->addr, ETH_ALEN);
	ret = esp32s31_radio_wifi_control(&ctl);
	if (!ret) {
		if (p->action == IEEE80211_AMPDU_RX_START) { sm->rx_ba_mask |= BIT(slot); sm->rx_ba_tid[slot] = p->tid; }
		else sm->rx_ba_mask &= ~BIT(slot);
	}
	pr_info("esp32s31-softmac: RX BA action=%u tid=%u ssn=%u window=%u slot=%u rc=%d\n",
		p->action, p->tid, p->ssn, p->buf_size, slot, ret);
	return ret;
}
/* skb_copy_bits() already flattens the accepted frag_list directly into the
 * radio bridge. Bound aggregation before mac80211 appends another subframe;
 * do not add a linearization copy or enlarge the native MPDU pools. */
static bool sm_can_aggregate(struct ieee80211_hw *hw, struct sk_buff *head,
			     struct sk_buff *skb)
{

	/* One frag_list member means there are already two MSDUs. Reserve
	 * 16 bytes for the first subframe's conversion and alignment. */
	return !skb_has_frag_list(head) && !skb_has_frag_list(skb) &&
		head->len + skb->len + 16 <= 3839;
}
static const struct ieee80211_ops sm_ops = {
	.tx = sm_tx, .start = sm_start, .stop = sm_stop,
	.add_interface = sm_add, .remove_interface = sm_remove,
	.config = sm_config, .configure_filter = sm_filter,
 .sw_scan_start = sm_sw_scan_start, .sw_scan_complete = sm_sw_scan_complete,
	.bss_info_changed = sm_bss_changed,
	.sta_add = sm_sta_add, .sta_remove = sm_sta_remove,
	.sta_pre_rcu_remove = sm_sta_pre_remove,
	.wake_tx_queue = sm_wake_txq,
	.can_aggregate_in_amsdu = sm_can_aggregate,
	.set_rts_threshold = sm_rts, .set_key = sm_key,
	.conf_tx = sm_conf_tx, .ampdu_action = sm_ampdu,
	.add_chanctx = ieee80211_emulate_add_chanctx,
	.remove_chanctx = ieee80211_emulate_remove_chanctx,
	.change_chanctx = ieee80211_emulate_change_chanctx,
};
static void sm_register(struct work_struct *work)
{
	struct s31_sm *sm = container_of(to_delayed_work(work), struct s31_sm,
					register_work);
	u8 mac[ETH_ALEN];
	int ret;

	ret = esp32s31_radio_wifi_register(&sm_radio_ops, sm);
	if (ret == -EAGAIN) {
		schedule_delayed_work(&sm->register_work, msecs_to_jiffies(100));
		return;
	}
	if (ret)
		goto failed;
	sm->radio_registered = true;
	ret = esp32s31_radio_wifi_get_mac(mac);
	if (ret)
		goto failed;
	SET_IEEE80211_PERM_ADDR(sm->hw, mac);
	ret = ieee80211_register_hw(sm->hw);
	if (ret)
		goto failed;
	sm->registered = true;
	pr_info("esp32s31-softmac: experimental mac80211 frontend registered\n");
	return;
failed:
	pr_err("esp32s31-softmac: registration failed: %d\n", ret);
	if (sm->radio_registered) {
		esp32s31_radio_wifi_unregister(&sm_radio_ops, sm);
		sm->radio_registered = false;
	}
}
int s31_radio_wifi_frontend_init(struct device *parent)
{
	struct ieee80211_hw *hw;
	struct s31_sm *sm;
	unsigned int i;

	if (esp32s31_radio_is_disabled())
		return 0;
	hw = ieee80211_alloc_hw(sizeof(*sm), &sm_ops);
	if (!hw)
		return -ENOMEM;
	sm = hw->priv;
	sm->hw = hw;
	sm->rx_slots=roundup_pow_of_two(clamp_t(unsigned int,sm_rx_ring_slots,32,SM_RX_SLOTS));
 sm->rx_mask=sm->rx_slots-1;
	sm->rx_idle_limit = sm_rx_idle_skb_limit ?
		clamp_t(unsigned int, sm_rx_idle_skb_limit, 32, sm->rx_slots) : sm->rx_slots;
	sm->rx_last_data = jiffies;
	sm->rx_trimmed = sm->rx_idle_limit < sm->rx_slots;
	spin_lock_init(&sm->lock);
	spin_lock_init(&sm->submit_lock);

	INIT_CSD(&sm->rx_csd, sm_rx_kick, sm);
	init_completion(&sm->rx_kick_done);
	complete(&sm->rx_kick_done);
	skb_queue_head_init(&sm->tx);
	skb_queue_head_init(&sm->status_queue);
	for (i = 0; i < sm->rx_slots; i++) {
		if (i >= sm->rx_idle_limit) continue;
		sm->rx[i].skb = sm_alloc_rx_skb(sm, false);
		if (!sm->rx[i].skb) {
			while (i) dev_kfree_skb(sm->rx[--i].skb);
			ieee80211_free_hw(hw);
			return -ENOMEM;
		}
	}
	sm->napi_dev = alloc_netdev_dummy(0);
	if (!sm->napi_dev) {
		for (i = 0; i < sm->rx_slots; i++) dev_kfree_skb(sm->rx[i].skb);
		ieee80211_free_hw(hw);
		return -ENOMEM;
	}
	netif_napi_add_weight(sm->napi_dev, &sm->napi, sm_napi_poll, clamp_t(unsigned int, sm_napi_budget, 1, 64));

	INIT_WORK(&sm->io, sm_io);
	INIT_WORK(&sm->rx_io, sm_rx_io);
	INIT_WORK(&sm->status_io, sm_status_io);
 INIT_WORK(&sm->filter_io, sm_filter_io);
	INIT_DELAYED_WORK(&sm->timer, sm_timer);
	INIT_DELAYED_WORK(&sm->register_work, sm_register);
	for (i = 0; i < ARRAY_SIZE(sm->channels); i++) {
		sm->channels[i].band = NL80211_BAND_2GHZ;
		sm->channels[i].hw_value = i + 1;
		sm->channels[i].center_freq = 2412 + 5 * i;
		sm->channels[i].max_power = 20;
	}
	/* mac80211 selects legacy rates; PP receives the selected rate per MPDU. */
	for (i = 0; i < ARRAY_SIZE(sm->rates); i++) {
		static const u16 bitrate[] = { 10, 20, 55, 110, 60, 90, 120, 180, 240, 360, 480, 540 };
		sm->rates[i].bitrate = bitrate[i];
		sm->rates[i].hw_value = i;
	}
	sm->band.channels = sm->channels;
	sm->band.n_channels = ARRAY_SIZE(sm->channels);
	sm->band.bitrates = sm->rates;
	sm->band.n_bitrates = ARRAY_SIZE(sm->rates);
	{
		sm->band.ht_cap.ht_supported = true;
		sm->band.ht_cap.cap = IEEE80211_HT_CAP_SGI_20;
		sm->band.ht_cap.ampdu_factor = IEEE80211_HT_MAX_AMPDU_64K;
		sm->band.ht_cap.ampdu_density = IEEE80211_HT_MPDU_DENSITY_4;
		sm->band.ht_cap.mcs.rx_mask[0] = 0xff;
		sm->band.ht_cap.mcs.rx_highest = cpu_to_le16(72);
		sm->band.ht_cap.mcs.tx_params = IEEE80211_HT_MCS_TX_DEFINED;
		ieee80211_hw_set(hw, AMPDU_AGGREGATION);
		hw->max_rx_aggregation_subframes = sm_rx_ba_window <= 32 ? 32 : 64;
		/* BA window is distinct from the bounded native assembly cap.
		 * Outstanding gate <=8, and operational callback rejects windows
		 * smaller than that gate to bound losses/retries within the window. */
		hw->max_tx_aggregation_subframes = sm_tx_ampdu ? 64 : 1;
		{
			ieee80211_hw_set(hw, SUPPORT_FAST_XMIT);
			hw->netdev_features |= NETIF_F_SG;
			{
				ieee80211_hw_set(hw, TX_AMSDU);
				ieee80211_hw_set(hw, TX_FRAG_LIST);
			}
		}
	}
	{
		sm->iftype.types_mask = BIT(NL80211_IFTYPE_STATION);
		sm->iftype.he_cap.has_he = true;
		sm->iftype.he_cap.he_cap_elem.phy_cap_info[1] =
			IEEE80211_HE_PHY_CAP1_DEVICE_CLASS_A |
			IEEE80211_HE_PHY_CAP1_HE_LTF_AND_GI_FOR_HE_PPDUS_0_8US;
		sm->iftype.he_cap.he_mcs_nss_supp.rx_mcs_80 = cpu_to_le16(0xfffc); /* NSS1 MCS0..7 */
		sm->iftype.he_cap.he_mcs_nss_supp.tx_mcs_80 =
			cpu_to_le16(sm_he_tx && sm_data_tx_mcs > 7 ? 0xfffd : 0xfffc);
		/* RX remains NSS1 MCS0..7; TX advertises MCS0..9. */
		sm->iftype.he_cap.he_mcs_nss_supp.rx_mcs_160 = cpu_to_le16(0xffff);
		sm->iftype.he_cap.he_mcs_nss_supp.tx_mcs_160 = cpu_to_le16(0xffff);
		sm->iftype.he_cap.he_mcs_nss_supp.rx_mcs_80p80 = cpu_to_le16(0xffff);
		sm->iftype.he_cap.he_mcs_nss_supp.tx_mcs_80p80 = cpu_to_le16(0xffff);
		sm->band.iftype_data = &sm->iftype;
		sm->band.n_iftype_data = 1;
	}
	hw->wiphy->bands[NL80211_BAND_2GHZ] = &sm->band;
	hw->wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION);
	hw->queues = sm_ht ? 4 : 1;
	ieee80211_hw_set(hw, HAS_RATE_CONTROL);
	/* Feed the bounded MPDU gate across native completion/ACK latency.
	 * A 1/32-second TSQ budget improves already-queued A-MSDU pairing;
	 * the radio gate and socket memory limits still bound queue growth. */
	hw->tx_sk_pacing_shift = 5;
	hw->max_rates = 1;
	hw->max_report_rates = 1;
	hw->max_rate_tries = 1;
	/* mac80211 fast TX requires disabled fragmentation, not the largest
	 * finite threshold. Native raw TX already rejects fragmented MPDUs. */
	hw->wiphy->frag_threshold = sm_hw_tx_ccmp && sm_hw_tx_ccmp_fast ?
		(u32)-1 : IEEE80211_MAX_FRAG_THRESHOLD;
	hw->wiphy->rts_threshold = IEEE80211_MAX_RTS_THRESHOLD;
	ieee80211_hw_set(hw, SIGNAL_DBM);

	{
		static const u32 ciphers[] = { WLAN_CIPHER_SUITE_CCMP };
		hw->wiphy->cipher_suites = ciphers;
		hw->wiphy->n_cipher_suites = ARRAY_SIZE(ciphers);
	}
	SET_IEEE80211_DEV(hw, parent);

	s31_sm_frontend = sm;
	schedule_delayed_work(&sm->register_work, 0);
	return 0;
}
void s31_radio_wifi_frontend_exit(void)
{
	struct s31_sm *sm = xchg(&s31_sm_frontend, NULL);
	unsigned int i;

	if (!sm)
		return;
	cancel_delayed_work_sync(&sm->register_work);
	esp32s31_radio_idle_poll_set(false);
	if (sm->registered)
		ieee80211_unregister_hw(sm->hw);
	cancel_delayed_work_sync(&sm->timer);
	cancel_work_sync(&sm->io);
	cancel_work_sync(&sm->rx_io);
	cancel_work_sync(&sm->status_io);
 cancel_work_sync(&sm->filter_io);
 WRITE_ONCE(sm_hw_bssid_filter_active,false);
	if (sm->napi_active) {
		napi_disable(&sm->napi);
		sm->napi_active = false;
	}
	if (sm->radio_registered)
		esp32s31_radio_wifi_unregister(&sm_radio_ops, sm);
	netif_napi_del(&sm->napi);
	free_netdev(sm->napi_dev);
	for (i = 0; i < sm->rx_slots; i++) dev_kfree_skb(sm->rx[i].skb);

	ieee80211_free_hw(sm->hw);
}
int s31_radio_wifi_frontend_suspend(void)
{
	/* Radio reset/replay while MLME is active is outside this prototype. */
	return s31_sm_frontend && READ_ONCE(s31_sm_frontend->running) ? -EBUSY : 0;
}
int s31_radio_wifi_frontend_resume(void)
{
	return 0;
}
