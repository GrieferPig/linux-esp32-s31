// SPDX-License-Identifier: GPL-2.0-only
/* Native open/WPA2-CCMP STA mac80211 frontend. */
#include <linux/etherdevice.h>
#include <linux/esp32s31-radio.h>
#include <linux/workqueue.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
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
static_assert(S31_SOFTMAC_PENDING &&
              !(S31_SOFTMAC_PENDING & (S31_SOFTMAC_PENDING - 1)));

/* The bounded source lease retains fragments until CPU0 copies directly
 * into the final vendor allocation. No intermediate linearization. */

/* Publish only bounded metadata/source leases from atomic context.
 * CPU0 reserves/fills/submits under its persistent radio owner. */
/* Consume native completions on the sole RX/TX-status NAPI owner.
 * Submission publishes metadata, never calls PP from NAPI. */

static const bool sm_direct_status = true;

/* Consume scheduled TXQs on the existing sole NAPI owner. No retained TXQ
 * pointer, extra worker, timer delay, or additional packet pool. */
static const bool sm_txq_napi = true;

static const bool sm_direct_tx = true;

static const unsigned int sm_data_tx_rate = 11;
static const int sm_data_tx_mcs = 9;
static const bool sm_tx_amsdu = true;
static const unsigned int sm_tx_ampdu = 16;

/* Allow the next aggregate to be prepared while one is outstanding.
 * The source ring, native allocator and per-pass batch retain their own
 * backpressure; this window uses the existing pending/flight tables. */
static const unsigned int sm_tx_pending_limit = 64;

static unsigned int sm_direct_tx_calls;

static const bool sm_direct_rx = true;

/* Preserve the producer's IRQ-disabled section, but release the shared
 * consumer lock during its bounded copy/checksum. The separate producer
 * lock serializes callbacks; .stop joins it before resetting RX slots. */

static const bool sm_rx_view = true;

static unsigned int sm_rx_view_hits;

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

static const unsigned int sm_gro_batch = 32;

/* The sole NAPI owner drains a small run per ring-lock round trip. The
 * producer only advances head, so entries below tail stay owned by this
 * consumer until the batch publishes replacements and advances tail. */
#define SM_NAPI_RX_BATCH 8
#define SM_NAPI_STATUS_BATCH 8

static const bool sm_hw_bssid_filter = true;

static bool sm_hw_bssid_filter_active;

static unsigned int sm_hw_bssid_filter_updates;

static const bool sm_rx_filter = true;
static const bool sm_native_rx_filter = true;
static const bool sm_native_rx_error_filter = true;

static const bool sm_native_bssid_filter = true;

#define SM_RX_SLOTS 128
#define SM_RX_LARGE_RESERVE 8
#define SM_RX_LARGE_ROOM (S31_SOFTMAC_FRAME_MAX + 4)
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
	bool done, ack, timeout_requested;
	u8 attempts, rate;
};

struct s31_sm {
	struct ieee80211_hw *hw;
	struct delayed_work register_work, timer;
	unsigned long timer_deadline;
	bool timer_armed;
	struct work_struct io, filter_io;
	struct net_device *napi_dev;
	struct napi_struct napi;
	call_single_data_t rx_csd;
	struct completion rx_kick_done;
	struct completion tx_ba_drained;
	struct mutex tx_ba_mutex;
	bool tx_ba_waiting;
	int tx_drain_error;
	bool tx_native_stopped;

 bool scanning, associated, connecting;
	u8 bssid[ETH_ALEN];
	u8 rx_ba_tid[4];
	u8 rx_ba_mask;
	u8 tx_key_generation;

	/* lock protects overlapping wake callbacks and the final flush. */
	u32 txq_batch_users;
 u8 txq_pending_ac; /* wake callbacks publish only AC bits, under lock */

	unsigned int filter_flags;

	bool rx_need_refill, rx_large_needed, rx_capture_active, rx_trim_pending;
	u32 rx_large_ready[SM_RX_SLOTS / 32];
	unsigned int rx_large_count;
	struct ieee80211_sta __rcu *rx_sta;
	bool napi_active;

	spinlock_t lock;
	spinlock_t submit_lock;
	/* RX producers reserve/copy/publish; stop joins before resetting skbs. */
	spinlock_t rx_capture_lock;
	struct sk_buff_head tx, status_queue;
	/* Protected by lock, advanced with the FIFO skb queue. */
	struct sm_pending pending[S31_SOFTMAC_PENDING];
	unsigned int status_cursor;
 DECLARE_BITMAP(status_done, S31_SOFTMAC_PENDING);
	struct sm_rx rx[SM_RX_SLOTS];
	unsigned int head, tail;
 unsigned int rx_slots, rx_mask;
	unsigned int rx_idle_limit;

	unsigned long rx_last_data;
	bool rx_trimmed;
	u32 cookie, epoch;

	u32 tx_outstanding;
	bool tx_gate_stopped;
	bool running, registered, radio_registered, has_vif;
	/* Replay identity, owned only by mac80211's serialized interface hooks. */
	struct ieee80211_vif *vif;
	bool tx_ht_peer, tx_he_peer;
	bool tx_ba_requested, tx_ba_operational, tx_native_peer, tx_ba_flushing;

	u8 channel;
	u16 coex_beacon_interval;
	u16 coex_aid;
	struct ieee80211_channel channels[13];
	struct ieee80211_rate rates[12];
	struct ieee80211_supported_band band;
	struct ieee80211_sband_iftype_data iftype;
};

/* Caller holds sm->lock, or producers/NAPI have been joined for stop. */
static void sm_pending_clear(struct s31_sm *sm, unsigned int i)
{
 unsigned int n;
 sm->pending[i].skb = NULL;
 __clear_bit(i, sm->status_done);
 /* Only the cold BA stop waiter needs an aggregate scan or a wake. The
  * same lock protects waiter publication and every pending retirement. */
 if (sm->tx_ba_waiting) {
  for (n = 0; n < S31_SOFTMAC_PENDING; n++)
   if (sm->pending[n].skb &&
       (IEEE80211_SKB_CB(sm->pending[n].skb)->flags & IEEE80211_TX_CTL_AMPDU))
    return;
  complete_all(&sm->tx_ba_drained);
 }
}

static struct s31_sm *s31_sm_frontend;
/* Cold publication/snapshot ownership only. Packet paths never take this
 * mutex. Unpublish before teardown joins; readers retain no frontend pointer.
 */
static DEFINE_MUTEX(s31_sm_frontend_mutex);

/* Caller holds sm->lock. A newer earlier deadline must replace an idle
 * deadline already pending. Later requests coalesce into that one wake. */
static void sm_arm_deadline_locked(struct s31_sm *sm, unsigned long deadline)
{
	unsigned long delay, now = jiffies;

	if (!sm->running || (sm->timer_armed &&
	    !time_before(deadline, sm->timer_deadline)))
		return;
	delay = time_after(deadline, now) ? deadline - now : 1;
	sm->timer_deadline = deadline;
	sm->timer_armed = true;
	mod_delayed_work(system_wq, &sm->timer, delay);
}

/* Only real packet expiry or an untrimmed RX idle window owns a deadline.
 * A requested source cancellation completes through its existing status
 * event; do not periodically poll the source lease while it is joined. */
static void sm_idle_deadline_locked(struct s31_sm *sm)
{
	if (!sm->rx_trimmed && sm->rx_idle_limit < sm->rx_slots &&
	    sm->head == sm->tail && !sm->rx_need_refill &&
	    !sm->rx_large_needed && !sm->rx_capture_active && !sm->rx_trim_pending)
		sm_arm_deadline_locked(sm, sm->rx_last_data + 5 * HZ);
}

static void sm_next_deadline_locked(struct s31_sm *sm)
{
	unsigned int i;

	sm_idle_deadline_locked(sm);
	for (i = 0; i < S31_SOFTMAC_PENDING; i++)
		if (sm->pending[i].skb && !sm->pending[i].done &&
		    !sm->pending[i].timeout_requested)
			sm_arm_deadline_locked(sm, sm->pending[i].expires);
}

/* Caller holds sm->lock. DRIVER wake schedules a mac80211 tasklet, never
 * invokes .tx inline; stop/wake ordering therefore stays under this lock. */
static void sm_tx_retire_locked(struct s31_sm *sm)
{
	if (WARN_ON_ONCE(!sm->tx_outstanding)) return;
	sm->tx_outstanding--;

	/* A retired frame is usable credit immediately. Waiting for half the
	 * window to retire withholds slots while a partial aggregate completes.
	 * NAPI drains ready statuses before TXQ service, so completions still
	 * coalesce into a refill batch without an artificial low watermark. */
	if (sm->running && sm->tx_gate_stopped &&
	    sm->tx_outstanding < sm_tx_pending_limit) {
		sm->tx_gate_stopped = false;

		ieee80211_wake_queues(sm->hw);
	}
}

/* The completion is the sole CSD admission token: one means idle, zero
 * means the submitted callback still owns it. Producers hold sm->lock, but
 * the IPI never takes that contended RX/TX ring lock. Stop closes producers,
 * joins the token, then disables NAPI. The callback's final access is complete;
 * its waitqueue lock orders both token reuse and stop against that access. */
static void sm_rx_kick(void *arg)
{
 struct s31_sm *sm = arg;

 /* This callback is invoked only by async CSD, with IRQs disabled. */
 napi_schedule_irqoff(&sm->napi);
 /* Record before the final completion access; stop joins that access. */

 complete(&sm->rx_kick_done);
}

static void sm_schedule_rx_locked(struct s31_sm *sm)
{
 const unsigned int cpu = sm_io_cpu;
 int ret;

 if (test_bit(NAPI_STATE_SCHED, &sm->napi.state)) return;
 if (cpu == smp_processor_id()) {
  napi_schedule(&sm->napi);
  return;
 }
 if (!try_wait_for_completion(&sm->rx_kick_done)) return;
 ret = smp_call_function_single_async(cpu, &sm->rx_csd);
 if (!ret) {

  return;
 }
 /* CPU1 is protected by the active radio hotplug lease. The private
  * CSD cannot be busy here: the token covers submission through callback,
  * and the kernel unlocks an async CSD before invoking that callback.
  * A failure is an ownership/admission defect, never a second RX path.
  */
 panic("esp32s31-softmac: mandatory CPU1 NAPI kick failed (%d)", ret);
}

/* Replacements are allocated on the Linux consumer stack. The radio callback
 * only copies into an already allocated skb; it never allocates or retains
 * a native buffer. Retry rare allocation failures without poisoning a slot. */
/* Only empty, unpublished buffers are marked. A replacement in the current
 * consumer slot is already empty even before tail advances. */
static void sm_rx_mark_ready(struct s31_sm *sm, unsigned int i,
                             struct sk_buff *skb)
{
 u32 bit = 1U << (i & 31);
 bool old = !!(sm->rx_large_ready[i >> 5] & bit);
 bool ready = skb && skb_tailroom(skb) >= SM_RX_LARGE_ROOM;
 if (old == ready) return;
 if (ready) { sm->rx_large_ready[i >> 5] |= bit; sm->rx_large_count++; }
 else { sm->rx_large_ready[i >> 5] &= ~bit; sm->rx_large_count--; }
}
static struct sk_buff *sm_alloc_rx_skb(struct s31_sm *sm, bool use_napi)
{
 unsigned int size = clamp_t(unsigned int, sm_rx_buf_len, 1600, S31_SOFTMAC_FRAME_MAX) + 8;
 bool large = READ_ONCE(sm->rx_large_count) < SM_RX_LARGE_RESERVE;
 struct sk_buff *skb;
 if (large) size = S31_SOFTMAC_FRAME_MAX + 8;
 skb = use_napi ? napi_alloc_skb(&sm->napi, size) :
                  __dev_alloc_skb(size, GFP_KERNEL);
 /* Keep ordinary MPDUs moving if the larger allocation cannot be met. */
 if (!skb && large) {
  size = clamp_t(unsigned int, sm_rx_buf_len, 1600, S31_SOFTMAC_FRAME_MAX) + 8;
  skb = use_napi ? napi_alloc_skb(&sm->napi, size) :
                   __dev_alloc_skb(size, GFP_KERNEL);
 }
 if (skb) skb_reserve(skb, -(unsigned long)skb->data & 3);
 return skb;
}
static void sm_refill_empty(struct s31_sm *sm, bool use_napi)
{
 unsigned int i;
 unsigned long flags;
 bool failed = false;
 struct sk_buff *upgrade = NULL;
 unsigned int upgrade_slot = 0;

 /* Steady state has no empty slot: avoid the ring lock twice per poll. */
 if (!READ_ONCE(sm->rx_need_refill) && !READ_ONCE(sm->rx_large_needed))
  return;
 spin_lock_irqsave(&sm->lock, flags);
 if (!sm->rx_need_refill && !sm->rx_large_needed) {
  spin_unlock_irqrestore(&sm->lock, flags);
  return;
 }
 /* Clear before doing work: a concurrent producer's new request must survive. */
 sm->rx_need_refill = false;
 if (sm->rx_large_needed) {
  if (sm->rx_large_count) sm->rx_large_needed = false;
  else if (!sm->rx_capture_active) {
   /* The producer cannot advance an unpublished head without its buffer.
    * Expand this empty buffer on the consumer stack, outside the lock. */
   upgrade_slot = sm->head;
   upgrade = sm->rx[upgrade_slot].skb;
   sm->rx[upgrade_slot].skb = NULL;
  }
 }
 spin_unlock_irqrestore(&sm->lock, flags);

 if (upgrade) {
  int extra = SM_RX_LARGE_ROOM - skb_tailroom(upgrade);
  if (extra > 0 && pskb_expand_head(upgrade, 0, extra, GFP_ATOMIC)) failed = true;
  spin_lock_irqsave(&sm->lock, flags);
  sm->rx[upgrade_slot].skb = upgrade;
  sm_rx_mark_ready(sm, upgrade_slot, upgrade);
  if (sm->rx_large_count) sm->rx_large_needed = false;
  spin_unlock_irqrestore(&sm->lock, flags);
 }
 for (i = 0; i < sm->rx_slots; i++) {
  struct sk_buff *skb;
  if (READ_ONCE(sm->rx[i].skb)) continue;
  skb = sm_alloc_rx_skb(sm, use_napi);
  if (!skb) { failed = true; continue; }
  spin_lock_irqsave(&sm->lock, flags);
  if (!sm->rx[i].skb) {
   sm->rx[i].skb = skb;
   sm_rx_mark_ready(sm, i, skb);
   skb = NULL;
  }
  spin_unlock_irqrestore(&sm->lock, flags);
  if (skb) dev_kfree_skb(skb);
 }
 spin_lock_irqsave(&sm->lock, flags);
 sm->rx_need_refill |= failed;

 if (!failed && !sm->rx_need_refill) sm->rx_trimmed = false;
 spin_unlock_irqrestore(&sm->lock, flags);
}
/* Peek a consecutive run of owned RX slots. Only the NAPI consumer advances
 * tail, so the metadata stays valid until sm_finish_rx_batch() commits. The
 * producer cannot overwrite these slots because tail has not moved. */
static unsigned int __aligned(64) sm_napi_take_rx(struct s31_sm *sm, struct sm_rx *rx,
                                    unsigned int max)
{
 unsigned long flags;
 unsigned int n = 0, tail, head;

 spin_lock_irqsave(&sm->lock, flags);
 if (!sm->running)
  goto out;

 head = sm->head;
 tail = sm->tail;
 while (n < max && tail != head) {
  rx[n++] = sm->rx[tail];
  tail = (tail + 1) & sm->rx_mask;
 }
out:
 spin_unlock_irqrestore(&sm->lock, flags);
 return n;
}

/* The sole NAPI owner has consumed every metadata field before calling.
 * Publish all replacements and advance tail in one fence, so capture cannot
 * reuse a slot before status construction has finished. */
static void __aligned(64) sm_finish_rx_batch(struct s31_sm *sm, struct sm_rx *rx,
                               struct sk_buff **replacement, unsigned int n)
{
 unsigned long flags;
 unsigned int i, target;

 spin_lock_irqsave(&sm->lock, flags);
 for (i = 0; i < n; i++) {
  struct sk_buff *repl = replacement[i];
  unsigned int slot = sm->tail;
  bool allocation_failed = !repl;

  if (sm->rx_trimmed && repl) {
   target = (sm->tail + sm->rx_idle_limit) & sm->rx_mask;
   if (!sm->rx[target].skb) {
    sm->rx[target].skb = repl;
    sm_rx_mark_ready(sm, target, repl);
    repl = NULL;
   }
  }
  sm->rx[slot].skb = repl;
  sm_rx_mark_ready(sm, slot, repl);
  if (allocation_failed) {

   sm->rx_need_refill = true;

  }
  sm->tail = (sm->tail + 1) & sm->rx_mask;

 }
 spin_unlock_irqrestore(&sm->lock, flags);
}
/* Empty ring and no capture owner; caller holds sm->lock. Retain
 * already prepared large buffers inside the idle window before freeing the
 * rest. Only pointers move: no allocation or frame copy under the lock. */
static void sm_keep_idle_rx_reserve(struct s31_sm *sm)
{
 unsigned int offset, cursor = 0, kept = 0;

 for (offset = 0; offset < sm->rx_idle_limit; offset++) {
  unsigned int slot = (sm->head + offset) & sm->rx_mask;
  if (sm->rx_large_ready[slot >> 5] & BIT(slot & 31)) kept++;
 }
 for (offset = sm->rx_idle_limit;
      offset < sm->rx_slots && kept < SM_RX_LARGE_RESERVE; offset++) {
  unsigned int source = (sm->head + offset) & sm->rx_mask;
  if (!(sm->rx_large_ready[source >> 5] & BIT(source & 31))) continue;
  while (cursor < sm->rx_idle_limit) {
   unsigned int target = (sm->head + cursor++) & sm->rx_mask;
   struct sk_buff *small;
   if (sm->rx_large_ready[target >> 5] & BIT(target & 31)) continue;
   small = sm->rx[target].skb;
   sm->rx[target].skb = sm->rx[source].skb;
   sm->rx[source].skb = small;
   sm_rx_mark_ready(sm, source, small);
   sm_rx_mark_ready(sm, target, sm->rx[target].skb);
   kept++;
   break;
  }
 }
}
/* Only the sole NAPI owner allocates/refills/trims prepared RX slots. The
 * deadline worker requests this cold operation; it never moves/free buffers
 * concurrently with refill publication. Capture remains independently joined. */
static bool sm_rx_idle_trim_due_locked(struct s31_sm *sm)
{
 return sm->running && !sm->rx_trimmed &&
        sm->rx_idle_limit < sm->rx_slots && sm->head == sm->tail &&
        !sm->rx_need_refill && !sm->rx_large_needed && !sm->rx_capture_active &&
        time_after_eq(jiffies, sm->rx_last_data + 5 * HZ);
}

static void sm_trim_idle_rx_skbs(struct s31_sm *sm)
{
 unsigned long flags;
 struct sk_buff *freed[SM_RX_SLOTS];
 unsigned int i, count = 0;

 spin_lock_irqsave(&sm->lock, flags);
 /* Keep the request published through the final state check, so a parallel
  * TX deadline pass cannot rearm an already expired idle window meanwhile. */
 sm->rx_trim_pending = false;
 if (!sm_rx_idle_trim_due_locked(sm)) {
  spin_unlock_irqrestore(&sm->lock, flags);
  return;
 }
 sm_keep_idle_rx_reserve(sm);
 for (i = 0; i < sm->rx_slots; i++) {
  if (((i - sm->head) & sm->rx_mask) >= sm->rx_idle_limit) {
   if (sm->rx[i].skb) freed[count++] = sm->rx[i].skb;
   sm->rx[i].skb = NULL;
   sm_rx_mark_ready(sm, i, NULL);
  }
 }
 sm->rx_trimmed = true;

 spin_unlock_irqrestore(&sm->lock, flags);
 for (i = 0; i < count; i++) dev_kfree_skb(freed[i]);
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
static void __aligned(64) sm_submit_queued(struct s31_sm *sm)
{
 struct sk_buff *skb;
 unsigned long flags;
 unsigned int i;
 int ret;
 bool published = false;
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
		/* Keep cookies opaque to native code, but choose their low bits
		 * as the Linux pending slot. Skip occupied slots without reserving
		 * generation bits or overwriting an owner. The full nonzero cookie
		 * is still checked when a completion returns. This rotates normal
		 * allocation and removes the cross-hart completion's PSRAM scan. */
		cookie = sm->cookie;
		for (i = 0; i < S31_SOFTMAC_PENDING; i++) {
			if (!++cookie)
				++cookie;
			if (!sm->pending[cookie & (S31_SOFTMAC_PENDING - 1)].skb)
				break;
		}
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

		sm->cookie = cookie;
		i = cookie & (S31_SOFTMAC_PENDING - 1);
		sm->pending[i] = (struct sm_pending) {
			.skb = skb, .cookie = cookie, .expires = jiffies + 2 * HZ,
			.rate = rate,
		};
		sm_arm_deadline_locked(sm, sm->pending[i].expires);

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
		if (!ret) published = true;

		if (ret) {
			spin_lock_irqsave(&sm->lock, flags);
			sm_pending_clear(sm, i);
			if (ret == -ENOSPC) {
				__skb_queue_head(&sm->tx, skb);
				/* The CPU0 consumer may have returned space and notified
				 * us between refusal and this requeue. Join that edge by
				 * rechecking under the normal lock order. If still full,
				 * the ring's published interest owns the next notification.
				 * NAPI's completion recheck covers an already scheduled poll.
				 */
				if (esp32s31_radio_wifi_tx_has_space())
					sm_schedule_rx_locked(sm);
			} else sm_tx_retire_locked(sm);

			spin_unlock_irqrestore(&sm->lock, flags);
			if (ret != -ENOSPC)
				sm_status(sm, skb, false, 1, rate);
			if (ret == -ENOSPC)
				break;
		}
	}
 spin_unlock_bh(&sm->submit_lock);
 if (published) esp32s31_radio_softmac_tx_kick();
}

/* Publish the ready metadata batch without entering vendor code. The
 * source lease ring wakes NAPI on consumed entries. A full ring must not
 * queue a second worker which immediately retries the same unavailable slot. */
static void sm_flush_tx(struct s31_sm *sm)
{
	sm_submit_queued(sm);
}

static int sm_tx_ba_flush(struct s31_sm *sm, const u8 *addr);
static void sm_tx_drain_failed(struct s31_sm *sm, int error);

static void sm_io(struct work_struct *work)
{
	struct s31_sm *sm = container_of(work, struct s31_sm, io);
	struct sk_buff *skb;
	unsigned long flags;
	unsigned int i;
	bool aggregate_timeout = false;
	u32 timeout_cookie = 0, timeout_epoch = 0, timeout_bytes = 0, timeout_depth = 0;
	unsigned int timeout_slot = 0;
	unsigned long timeout_late = 0;
	u8 ap[ETH_ALEN];

	/* A Linux timeout cannot release the native EB. Cancel the aggregate
	 * before retiring its cookie or reopening the pending gate. A failed
	 * cancel keeps publication closed and ownership intact. */
	spin_lock_irqsave(&sm->lock, flags);
	for (i = 0; i < S31_SOFTMAC_PENDING; i++)
		if (sm->pending[i].skb && !sm->pending[i].done &&
		    !sm->pending[i].timeout_requested &&
		    time_after_eq(jiffies, sm->pending[i].expires) &&
		    (IEEE80211_SKB_CB(sm->pending[i].skb)->flags & IEEE80211_TX_CTL_AMPDU)) {
			aggregate_timeout = true;
			sm->pending[i].timeout_requested = true;
			timeout_cookie = sm->pending[i].cookie;
			timeout_slot = i;
			timeout_epoch = sm->epoch;
			timeout_bytes = sm->pending[i].skb->len;
			timeout_depth = sm->tx_outstanding;
			timeout_late = jiffies - sm->pending[i].expires;
			ether_addr_copy(ap, sm->bssid);

			break;
		}
	spin_unlock_irqrestore(&sm->lock, flags);
	if (aggregate_timeout) {
		int ret = 0;

		mutex_lock(&sm->tx_ba_mutex);
		/* Selection precedes the sleepable BA lock. While we wait, another
		 * teardown may retire this cookie or a real completion may finish it.
		 * A stale deadline must not drain a replacement slot/station or restart
		 * a link whose old aggregate has already completed. The pending lock
		 * pairs this recheck with status publication and retirement; the BA
		 * mutex excludes another teardown until recovery owns publication.
		 */
		spin_lock_irqsave(&sm->lock, flags);
		aggregate_timeout = sm->running && sm->epoch == timeout_epoch &&
			!sm->tx_drain_error && !sm->tx_native_stopped &&
			sm->pending[timeout_slot].skb &&
			sm->pending[timeout_slot].cookie == timeout_cookie &&
			!sm->pending[timeout_slot].done &&
			sm->pending[timeout_slot].timeout_requested;
		spin_unlock_irqrestore(&sm->lock, flags);
		if (aggregate_timeout) {

			ret = sm_tx_ba_flush(sm, ap);
			/* A successful forced drain stops native SW/HW queues as well as
			 * retiring buffers. It is not a running link: keep publication closed
			 * and replay the normal hardware lifecycle. Failed drains already
			 * requested this recovery from sm_tx_ba_flush(). Never silently leave
			 * an associated interface waiting for credits from stopped hardware.
			 */
			if (!ret)
				sm_tx_drain_failed(sm, -ETIMEDOUT);
		} else {

		}
		mutex_unlock(&sm->tx_ba_mutex);
		if (aggregate_timeout)
			pr_warn_ratelimited("esp32s31-softmac: TX BA timed out cookie=%u epoch=%u bytes=%u depth=%u late_jiffies=%lu cancel_rc=%d; hardware recovery requested\n",
				    timeout_cookie, timeout_epoch, timeout_bytes,
				    timeout_depth, timeout_late, ret);
	}

	/* TX submission is serialized by submit_lock; RX callbacks use bounded rings. */
	for (i = 0; i < S31_SOFTMAC_PENDING; i++) {
		bool ack;
		u8 attempts, rate;

		spin_lock_irqsave(&sm->lock, flags);
		if (!sm->pending[i].skb || (!sm->pending[i].done &&
		    (sm->pending[i].timeout_requested ||
		     time_before(jiffies, sm->pending[i].expires)))) {
			spin_unlock_irqrestore(&sm->lock, flags);
			continue;
		}
		if (!sm->pending[i].done &&
		    (IEEE80211_SKB_CB(sm->pending[i].skb)->flags & IEEE80211_TX_CTL_AMPDU)) {
			spin_unlock_irqrestore(&sm->lock, flags);
			continue;
		}
        if (sm_direct_status && sm->running && sm->pending[i].done) {
            sm_schedule_rx_locked(sm);
            spin_unlock_irqrestore(&sm->lock, flags);
            continue;
        }
		/* mac80211 may change skb layout during status reporting. A source
		 * lease must finish reading before timeout gives that skb back. */
		if (!sm->pending[i].done &&
		    esp32s31_radio_softmac_tx_cancel(sm->pending[i].cookie, sm->epoch)) {
			sm->pending[i].timeout_requested = true;
			spin_unlock_irqrestore(&sm->lock, flags);
			continue;
		}

		skb = sm->pending[i].skb;
		ack = sm->pending[i].done && sm->pending[i].ack;
		attempts = sm->pending[i].attempts;
		rate = sm->pending[i].rate;
		sm_pending_clear(sm, i);
		sm_tx_retire_locked(sm);

		spin_unlock_irqrestore(&sm->lock, flags);

		sm_status(sm, skb, ack, attempts, rate);
	}
	sm_submit_queued(sm);
	spin_lock_irqsave(&sm->lock, flags);
	sm_next_deadline_locked(sm);
	spin_unlock_irqrestore(&sm->lock, flags);
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

}

static bool sm_pending_status_ready_locked(struct s31_sm *sm)
{
 if (!sm->running) return false;
 return !bitmap_empty(sm->status_done, S31_SOFTMAC_PENDING);
}
static unsigned int sm_napi_take_status(struct s31_sm *sm,
                                        struct sm_pending *completed,
                                        unsigned int max)
{
 unsigned long flags;
 unsigned int i, n = 0;
 /* Most downlink iterations have no TX completion. This is only an
  * empty hint: take the lock before consuming any bit/record. A racing
  * producer is observed by a later iteration or the locked NAPI completion
  * recheck, so skipping this lock cannot strand a completion. */
 for (i = 0; i < ARRAY_SIZE(sm->status_done); i++)
  if (READ_ONCE(sm->status_done[i])) break;
 if (i == ARRAY_SIZE(sm->status_done)) return 0;
 spin_lock_irqsave(&sm->lock, flags);
 if (!sm->running) goto out;
 /* Allocation reuses low slots. Rotate delivery so ongoing traffic cannot
  * indefinitely postpone a completed frame in a higher slot. */
 while (n < max) {
  i = sm_pending_done_index(sm);
  if (i >= S31_SOFTMAC_PENDING) break;
  completed[n] = sm->pending[i];
  sm_pending_clear(sm, i);
  sm_tx_retire_locked(sm);
  sm->status_cursor = (i + 1) % S31_SOFTMAC_PENDING;

  n++;
 }
out:
 spin_unlock_irqrestore(&sm->lock, flags);
 for (i = 0; i < n; i++) {
  sm_prepare_status(completed[i].skb, completed[i].ack,
                    completed[i].attempts, completed[i].rate);
 }
 return n;
}
static void sm_rx_status(struct ieee80211_rx_status *status, const struct sm_rx *rx)
{
	status->rate_idx = rx->rate;
 if (rx->phy_flags & S31_SOFTMAC_RX_CCMP_VALID)
  status->flag |= RX_FLAG_DECRYPTED | RX_FLAG_MIC_STRIPPED;
 /* Retain CCMP IV and omit RX_FLAG_PN_VALIDATED: mac80211 verifies RX
  * replay counters even when the MAC provides CCMP encryption/decryption. */
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
static int __aligned(64) sm_napi_txqs(struct s31_sm *sm, int budget)
{
 struct ieee80211_hw *hw = sm->hw;
 struct ieee80211_txq *queue;
 unsigned long flags;
 unsigned int ac, count = 0;
 u8 ready;
 bool flush;

 spin_lock_irqsave(&sm->lock, flags);
 if (!sm->running || !sm->txq_pending_ac || sm->tx_gate_stopped || sm->tx_ba_flushing) {
  spin_unlock_irqrestore(&sm->lock, flags);
  return 0;
 }
 ready = sm->txq_pending_ac;
 sm->txq_pending_ac = 0;
 sm->txq_batch_users++;

 spin_unlock_irqrestore(&sm->lock, flags);

 rcu_read_lock();
 for (ac = 0; ac < IEEE80211_NUM_ACS; ac++) {
  if (!(ready & BIT(ac))) continue;
  if (count == budget || READ_ONCE(sm->tx_gate_stopped) || READ_ONCE(sm->tx_ba_flushing)) break;
  ieee80211_txq_schedule_start(hw, ac);
  while (count < budget && (queue = ieee80211_next_txq(hw, ac))) {
   struct ieee80211_tx_control ctl = { .sta = queue->sta };
   struct sk_buff *skb;
   while (count < budget && !READ_ONCE(sm->tx_gate_stopped) && !READ_ONCE(sm->tx_ba_flushing) &&
          (skb = ieee80211_tx_dequeue(hw, queue))) {
    sm_tx(hw, &ctl, skb);
    count++;
   }
   ieee80211_return_txq(hw, queue, false);
  }
  ieee80211_txq_schedule_end(hw, ac);
  if (count < budget && !READ_ONCE(sm->tx_gate_stopped) && !READ_ONCE(sm->tx_ba_flushing)) ready &= ~BIT(ac);
 }
 rcu_read_unlock();

 spin_lock_irqsave(&sm->lock, flags);
 sm->txq_batch_users--;

 /* Preserve ACs not visited before the gate/budget closed. They may not
  * yet be marked DIRTY by dequeue. A full gate excludes these bits from
  * NAPI readiness; real completion reopens it before the next drain. */
 if (sm->running) sm->txq_pending_ac |= ready;
 flush = !sm->txq_batch_users && sm->running && !skb_queue_empty(&sm->tx);

 spin_unlock_irqrestore(&sm->lock, flags);
 if (flush) sm_flush_tx(sm);

 return count;
}

/* Caller holds sm->lock. A full source ring sleeps until its consumption
 * notification; queued metadata alone must never keep NAPI busy polling. */
static bool sm_tx_batch_ready_locked(struct s31_sm *sm)
{
 return sm->running && !sm->tx_ba_flushing && !sm->txq_batch_users &&
        !skb_queue_empty(&sm->tx) &&
        esp32s31_radio_wifi_tx_has_space();
}

static int __aligned(64) sm_napi_poll(struct napi_struct *napi, int budget)
{

 struct s31_sm *sm = container_of(napi, struct s31_sm, napi);

 struct sm_pending status_batch[SM_NAPI_STATUS_BATCH];
 struct sm_rx rx_batch[SM_NAPI_RX_BATCH];
 struct sk_buff *replacement[SM_NAPI_RX_BATCH];
 struct sk_buff *deliver_skb[SM_NAPI_RX_BATCH];
 unsigned long flags;
 int done = 0, rx_since_flush = 0;

 sm_refill_empty(sm, true);
 if (!skb_queue_empty(&sm->tx)) sm_submit_queued(sm);
 while (done < budget) {
  struct sk_buff *skb;
  unsigned int i, n;

  n = sm_napi_take_status(sm, status_batch,
                          min_t(unsigned int, SM_NAPI_STATUS_BATCH,
                                budget - done));
  if (n) {
   for (i = 0; i < n; i++) {
    sm_napi_status(sm, status_batch[i].skb);
   }
   sm_submit_queued(sm);
   done += n;
   continue;
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
      !READ_ONCE(sm->tx_gate_stopped) && !READ_ONCE(sm->tx_ba_flushing)) {
   int sent = sm_napi_txqs(sm, budget - done);
   if (sent) { done += sent; if (done >= budget) break; }
  }
  if (!skb_queue_empty(&sm->tx)) sm_submit_queued(sm);

  /* Drain one run of owned RX slots per ring-lock round trip. Replacements
   * are allocated outside the lock; the producer cannot overwrite the
   * peeked slots while tail is still behind them. */
  n = sm_napi_take_rx(sm, rx_batch,
                      min_t(unsigned int, SM_NAPI_RX_BATCH,
                            budget - done));
  if (n) {
   for (i = 0; i < n; i++) {

    replacement[i] = sm_alloc_rx_skb(sm, true);

   }
   for (i = 0; i < n; i++) {
    struct ieee80211_rx_status *status;

    skb = rx_batch[i].skb;
    deliver_skb[i] = NULL;
    if (!skb)
     continue;
    status = IEEE80211_SKB_RXCB(skb);
    memset(status, 0, sizeof(*status));
    status->band = NL80211_BAND_2GHZ;
    status->freq = ieee80211_channel_to_frequency(rx_batch[i].channel,
                                                  NL80211_BAND_2GHZ);
    status->signal = rx_batch[i].signal;
    sm_rx_status(status, &rx_batch[i]);
    deliver_skb[i] = skb;
   }
   /* Free the producer slots before synchronous decrypt/stack processing. */
   sm_finish_rx_batch(sm, rx_batch, replacement, n);
   {
    /* One RCU section for the run. mac80211's per-frame reorder/PN/
     * decrypt, GRO and delivery semantics are unchanged. */

    rcu_read_lock();
    for (i = 0; i < n; i++) {
     struct ieee80211_sta *sta;
     struct ieee80211_hdr *hdr;

     skb = deliver_skb[i];
     if (!skb)
      continue;
     hdr = (void *)skb->data;
     /* The pre-RCU removal callback withdraws this hint before mac80211's
      * grace period. Unknown senders retain normal mac80211 lookup. */
     sta = rcu_dereference(sm->rx_sta);
     if (!sta || !ieee80211_is_data(hdr->frame_control) ||
         !ether_addr_equal(hdr->addr2, sta->addr)) sta = NULL;
     {
       ieee80211_rx_napi(sm->hw, sta, skb, napi);
     }
    }
    rcu_read_unlock();

   }
   rx_since_flush += n;
   if (READ_ONCE(sm_gro_batch) &&
       rx_since_flush >= READ_ONCE(sm_gro_batch)) {
    /* Same sole NAPI owner; no driver lock or RCU read section held. */

    {
      napi_gro_flush(napi, false);
      if (napi->gro.rx_count) {
       netif_receive_skb_list(&napi->gro.rx_list);
       INIT_LIST_HEAD(&napi->gro.rx_list);
       napi->gro.rx_count = 0;
      }
    }

    rx_since_flush = 0;
   }
   done += n;
   continue;
  }

  spin_lock_irqsave(&sm->lock, flags);
  if (!sm->running || sm->head == sm->tail) {
   /* Service the idle request at the same ownership boundary as refill.
    * A request arriving after this check is caught by completion's locked
    * recheck below; it must not leave an already scheduled NAPI spinning. */
   if (sm->rx_trim_pending) {
    spin_unlock_irqrestore(&sm->lock, flags);
    sm_trim_idle_rx_skbs(sm);
    continue;
   }
   if (skb_queue_empty(&sm->status_queue) && !sm_pending_status_ready_locked(sm) &&
       !sm_tx_batch_ready_locked(sm) &&
       !(sm_txq_napi && sm->txq_pending_ac && !sm->tx_gate_stopped && !sm->tx_ba_flushing)) {
    spin_unlock_irqrestore(&sm->lock, flags);
    /* Completion can flush GRO and generate TX; never hold sm->lock.
     * Recheck after completion so a producer racing SCHED clear cannot
     * lose the final wakeup. All delivery remains on this one NAPI.
     */

    napi_complete_done(napi, done);

    spin_lock_irqsave(&sm->lock, flags);
    if (sm->running && (sm->rx_trim_pending || sm->head != sm->tail ||
        !skb_queue_empty(&sm->status_queue) || sm_pending_status_ready_locked(sm) ||
        sm_tx_batch_ready_locked(sm) ||
        (sm_txq_napi && sm->txq_pending_ac && !sm->tx_gate_stopped && !sm->tx_ba_flushing)))
     napi_schedule(napi);
    sm_idle_deadline_locked(sm);
    spin_unlock_irqrestore(&sm->lock, flags);

    return done;
   }
   spin_unlock_irqrestore(&sm->lock, flags);
   continue;
  }
  spin_unlock_irqrestore(&sm->lock, flags);
 }

 return done;
}

/* TX acceptance, watchdog rearm and stop all serialize on sm->lock. */

static void sm_timer(struct work_struct *work)
{
	struct s31_sm *sm = container_of(to_delayed_work(work), struct s31_sm, timer);

	unsigned long flags;
	unsigned int i;
	bool expired = false;

	spin_lock_irqsave(&sm->lock, flags);
	sm->timer_armed = false;
	if (!sm->running) {
		spin_unlock_irqrestore(&sm->lock, flags);
		return;
	}
	for (i = 0; i < S31_SOFTMAC_PENDING; i++)
		if (sm->pending[i].skb && !sm->pending[i].done &&
		    !sm->pending[i].timeout_requested &&
		    time_after_eq(jiffies, sm->pending[i].expires)) {
			expired = true;
			break;
		}
	if (expired)
		queue_work_on(sm_io_cpu, system_highpri_wq, &sm->io);
	if (!sm->rx_trim_pending && sm_rx_idle_trim_due_locked(sm)) {
		sm->rx_trim_pending = true;

		sm_schedule_rx_locked(sm);
	}
	/* The expiry worker arms remaining TX deadlines after joining sources;
	 * do not rearm an already expired packet while that worker is queued. */
	if (!expired)
		sm_next_deadline_locked(sm);
	spin_unlock_irqrestore(&sm->lock, flags);
}
/* One value status; caller holds sm->lock. No packet/native pointer retained. */
static bool sm_tx_status_locked(struct s31_sm *sm, struct s31_softmac_tx_status status)
{
	unsigned int i = status.cookie & (S31_SOFTMAC_PENDING - 1);
	struct sm_pending *pending = &sm->pending[i];

	/* Indexing is only a lookup. Stale/zero/retired cookies must never
	 * complete another frame after the slot has been reused. */
	if (!pending->skb || pending->cookie != status.cookie)
		return false;
	pending->ack = status.success;
	pending->attempts = status.attempts;
	if (status.rate < ARRAY_SIZE(sm->rates) ||
	    ((status.rate & ~7U) == S31_SOFTMAC_TX_HT_RATE) ||
	    sm_tx_rate_is_he(status.rate))
		pending->rate = status.rate;
	pending->done = true;
	__set_bit(i, sm->status_done);
	return true;
}

static void __aligned(64) sm_aux(void *context, u8 interface, const u8 *frame,
		   size_t length, u8 channel, s8 signal)
{
	struct s31_sm *sm = context;
	unsigned long flags;
	unsigned int next;
	struct s31_softmac_rx_header meta = {0};
	bool borrowed = false, capture_locked;

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

	/* One IRQ-mask span, and no vendor execution gate. TX status needs only
	 * the short ring lock. RX holds its resource lock across the borrowed
	 * copy so another producer and .stop cannot reuse the reserved skb. */
	capture_locked = interface == S31_WIFI_IF_SOFTMAC;
	local_irq_save(flags);
	if (capture_locked) spin_lock(&sm->rx_capture_lock);
	spin_lock(&sm->lock);

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
		if (sm_direct_status && matched) {
			sm_schedule_rx_locked(sm);
			goto out;
		}
	} else if (interface == S31_WIFI_IF_SOFTMAC &&
		   length >= sizeof(struct s31_softmac_rx_header) + 24 &&
		   length <= sizeof(struct s31_softmac_rx_header) + S31_SOFTMAC_FRAME_MAX &&
		   meta.encoding <= 2 && meta.rate_index < (meta.encoding == 1 ? 8 : meta.encoding == 2 ? 10 : ARRAY_SIZE(sm->rates)) &&
		   channel >= 1 && channel <= ARRAY_SIZE(sm->channels)) {
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
        if (next == sm->tail) {  goto out; }
        if (!sm->rx[sm->head].skb) {

         sm->rx_need_refill = true;
         sm_schedule_rx_locked(sm);
         goto out;
        }
        if (length + 4 > skb_tailroom(sm->rx[sm->head].skb)) {
         unsigned int word;
         for (word = 0; word < ARRAY_SIZE(sm->rx_large_ready); word++) {
          u32 ready = sm->rx_large_ready[word];
          if (ready) {
           unsigned int slot = word * 32 + __ffs(ready);
           struct sk_buff *small = sm->rx[sm->head].skb;
           sm->rx[sm->head].skb = sm->rx[slot].skb;
           sm->rx[slot].skb = small;
           sm_rx_mark_ready(sm, slot, small);

           break;
          }
         }
         if (word == ARRAY_SIZE(sm->rx_large_ready)) {

          sm->rx_large_needed = true;

          sm_schedule_rx_locked(sm);
          goto out;
         }
        }
        /* No allocation/expansion or retained vendor ownership in capture. */
        sm_rx_mark_ready(sm, sm->head, NULL);
		sm->rx[sm->head].length = length;
		sm->rx[sm->head].channel = channel;
		sm->rx[sm->head].signal = signal;
		sm->rx[sm->head].rate = rate;
		sm->rx[sm->head].encoding = encoding;
		sm->rx[sm->head].phy_flags = phy_flags;
		if (ieee80211_hdrlen(get_unaligned((const __le16 *)frame)) & 2)
			skb_reserve(sm->rx[sm->head].skb, 2);

        {
         struct sm_rx *rx = &sm->rx[sm->head];
         struct sk_buff *skb = rx->skb;

         /* head is unpublished until the copy completes. CPU1 can consume
          * older slots/status while this producer retains the borrowed view.
          * Refill must not steal/expand this active head. */
         sm->rx_capture_active = true;
         spin_unlock(&sm->lock);

         s31_sm_rx_copy(skb_put(skb, length), frame, length);
         skb->ip_summed = CHECKSUM_NONE;
         spin_lock(&sm->lock);
         sm->rx_capture_active = false;

         if (!sm->running) {
          /* .stop has closed admission and is joining rx_capture_lock. */
          skb_trim(skb, 0);
          skb->data = PTR_ALIGN_DOWN(skb->data, 4);
          skb_reset_tail_pointer(skb);
          sm_rx_mark_ready(sm, sm->head, skb);

          goto out;
         }
        }

		sm->head = next;

		sm_schedule_rx_locked(sm);
		goto out;
	} else {
		goto out;
	}
	/* Queue while locked so .stop can forbid all new work before cancelling. */
	queue_work_on(sm_io_cpu, system_highpri_wq, &sm->io);
out:

	/* Restore IRQs before the final preempt-enable, as irqrestore normally
	 * does, so a higher-priority wakeup is not left until another interrupt. */
	if (capture_locked) {
		spin_unlock(&sm->lock);
		spin_unlock_irqrestore(&sm->rx_capture_lock, flags);
	} else {
		spin_unlock_irqrestore(&sm->lock, flags);
	}

}
static void sm_wakeup(void *context)
{
	struct s31_sm *sm = context;
	unsigned long flags;

	spin_lock_irqsave(&sm->lock, flags);
	if (sm->running && !skb_queue_empty(&sm->tx))
		sm_schedule_rx_locked(sm);
	spin_unlock_irqrestore(&sm->lock, flags);
}
static void sm_scan(void *ctx, int rc, const struct esp32s31_radio_wifi_ap *aps,
		    size_t count) {}
static void sm_unused_rx(void *ctx) {}
static const struct esp32s31_radio_wifi_ops sm_radio_ops = {
	/* mac80211 owns auth/assoc and cfg80211 notifications. Firmware STA
	 * lifecycle events must not clear its connection during hardware replay. */
	.scan_complete = sm_scan, .rx_ready = sm_unused_rx,
	.tx_wakeup = sm_wakeup, .receive_aux = sm_aux,
};
static void sm_stop(struct ieee80211_hw *hw, bool suspend);
static int sm_start(struct ieee80211_hw *hw)
{
	struct s31_sm *sm = hw->priv;
	struct s31_wifi_control ctl = { .operation = S31_WIFI_SOFTMAC_START };
	int ret;
	unsigned long flags;

	/* CPU1 is the sole Linux NAPI/TX-status owner. The radio lease pins
	 * its online lifetime while this interface runs; refuse cold admission
	 * without it rather than moving network work onto the radio CPU.
	 */
	if (!cpu_online(sm_io_cpu))
		return -EHOSTDOWN;
	/* mac80211 hardware reconfiguration calls .start without .stop. Join
	 * Linux producers/NAPI and native ownership before replay; enabling an
	 * already enabled NAPI instance is a BUG, not an idempotent operation. */
	if (READ_ONCE(sm->running))
		sm_stop(hw, false);
	/* A failed PP drain is not a completed hardware stop. Retry the full
	 * native stop before replaying a new epoch or replacing its peer/keys. */
	if (READ_ONCE(sm->tx_drain_error)) {
		ctl.operation = S31_WIFI_SOFTMAC_STOP;
		ret = esp32s31_radio_wifi_control(&ctl);
		if (ret) return ret < 0 ? ret : -EIO;
		WRITE_ONCE(sm->tx_drain_error, 0);
		ctl.operation = S31_WIFI_SOFTMAC_START;
	}
	if (sm_data_tx_mcs < -1 || sm_data_tx_mcs > 9 ||
	    (sm_data_tx_mcs > 7 && !sm_he_tx)) return -EINVAL;
	/* Without a credit gate generic TXQ drain is unbounded. */

	if (sm_tx_pending_limit > S31_SOFTMAC_PENDING ||
	    (sm_tx_amsdu && (!sm_ht || sm_data_tx_mcs < 0 || !sm_tx_pending_limit)))
		return -EINVAL;
	if (sm_tx_ampdu && (!sm_ht || sm_data_tx_mcs < 0 ||
	    (sm_tx_ampdu != 2 && sm_tx_ampdu != 4 &&
	     sm_tx_ampdu != 8 && sm_tx_ampdu != 16) ||
        sm_tx_pending_limit < sm_tx_ampdu))
		return -EINVAL;

	{
		ctl.operation = sm_data_tx_mcs > 7 ? S31_WIFI_SOFTMAC_TX_HE9_CAP :
		                                  S31_WIFI_SOFTMAC_TX_HE_CAP;
		ret = esp32s31_radio_wifi_control(&ctl);
		if (ret) return ret == -EIO ? -EOPNOTSUPP : ret;
		ctl.operation = S31_WIFI_SOFTMAC_START;
	}
	ctl.operation = S31_WIFI_SOFTMAC_TX_BATCH_CAP;
	ret = esp32s31_radio_wifi_control(&ctl);
	if (ret) return ret == -EIO ? -EOPNOTSUPP : ret;
	ctl.operation = S31_WIFI_SOFTMAC_START;
	WRITE_ONCE(sm->tx_he_peer, false);

	{
		ctl.operation = S31_WIFI_SOFTMAC_TX_AGG_CAP;
		ret = esp32s31_radio_wifi_control(&ctl);
		if (ret) return ret == -EIO ? -EOPNOTSUPP : ret;
		ctl.operation = S31_WIFI_SOFTMAC_START;
	}

	{
		ctl.operation = S31_WIFI_SOFTMAC_TX_HT_CAP;
		ret = esp32s31_radio_wifi_control(&ctl);
		if (ret) return ret == -EIO ? -EOPNOTSUPP : ret;
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
	WRITE_ONCE(sm->tx_native_stopped, false);
	{
		napi_enable(&sm->napi);
		sm->napi_active = true;
	}
	WRITE_ONCE(sm->running, true);

	sm->rx_ba_mask = 0;
	spin_lock_irqsave(&sm->lock, flags);
	sm->rx_last_data = jiffies;
	sm->rx_trim_pending = false;
	sm_next_deadline_locked(sm);
	spin_unlock_irqrestore(&sm->lock, flags);
	return 0;
}
/* NAPI and producers are stopped. Keep the retained skb window anchored
 * to the old consumer cursor; .stop clears all retained skb payloads below.
 * Rewinding to zero can put a trimmed window behind the new producer. */
static void sm_restart_rx_ring(struct s31_sm *sm)
{
	sm->head = sm->tail;
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
 sm->rx_trim_pending = false;
 complete_all(&sm->tx_ba_drained);
 sm->txq_pending_ac = 0;
 sm->associated = false;
 sm->scanning = false;
 sm->connecting = false;
	spin_unlock_irqrestore(&sm->lock, flags);
	/* A reserved head may be copying without the shared ring lock. Join it
	 * before disabling NAPI, stopping firmware or resetting retained buffers.
	 * Never wait for this producer while holding sm->lock. */
	spin_lock_irqsave(&sm->rx_capture_lock, flags);
	spin_unlock_irqrestore(&sm->rx_capture_lock, flags);
	/* Join an atomic .tx submission before stopping payload/rings. New
         * submissions see running=false and cannot reserve another skb. */
        spin_lock_bh(&sm->submit_lock);
        spin_unlock_bh(&sm->submit_lock);

	/* No producer can submit after running=false. Join the callback's last
	 * memory access and return the idle token for the next .start epoch. */
	wait_for_completion(&sm->rx_kick_done);
	complete(&sm->rx_kick_done);
	cancel_delayed_work_sync(&sm->timer);
	sm->timer_armed = false;
	cancel_work_sync(&sm->io);
 cancel_work_sync(&sm->filter_io);
 WRITE_ONCE(sm_hw_bssid_filter_active,false);
	if (sm->napi_active) {
		napi_disable(&sm->napi);
		sm->napi_active = false;
	}
	esp32s31_radio_softmac_tx_quiesce(sm->epoch);
	{
		int ret = esp32s31_radio_wifi_control(&ctl);
		WRITE_ONCE(sm->tx_drain_error, ret);
		if (!ret) WRITE_ONCE(sm->tx_native_stopped, true);
		if (ret) pr_warn("esp32s31-softmac: radio stop failed: %d; replay requires native stop\n", ret);
	}
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
	/* A successful STA_STOP joins native EBs/callbacks; quiesce above joined
	 * source leases, and every driver-owned skb has now been discarded.
	 * No old plaintext token can cross the next .start epoch. Bound token
	 * uniqueness to this retired hardware lifetime, rather than exhausting
	 * the seven-bit space after127 otherwise successful hardware restarts.
	 * A failed stop retains the generation and its ownership protection. */
	if (!READ_ONCE(sm->tx_drain_error) && READ_ONCE(sm->tx_native_stopped))
		sm->tx_key_generation = 0;
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
		sm_rx_mark_ready(sm, i, skb);
	}
}
static void __aligned(64) sm_tx_inner(struct ieee80211_hw *hw, struct ieee80211_tx_control *ctl,
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
		sm->tx_outstanding++;

		if (sm_tx_pending_limit && !sm->tx_gate_stopped &&
		    sm->tx_outstanding >= sm_tx_pending_limit) {
			sm->tx_gate_stopped = true;

			ieee80211_stop_queues(hw);
		}
		defer_submit = sm->txq_batch_users != 0;

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

 spin_lock_irqsave(&sm->lock, flags);
 if (sm->running && txq->ac < IEEE80211_NUM_ACS) {
  sm->txq_pending_ac |= BIT(txq->ac);

  sm_schedule_rx_locked(sm);
 }
 spin_unlock_irqrestore(&sm->lock, flags);
}

static int sm_add(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	struct s31_sm *sm = hw->priv;

	if (vif->type != NL80211_IFTYPE_STATION ||
	    !ether_addr_equal(vif->addr, hw->wiphy->perm_addr))
		return -EOPNOTSUPP;
	/* Hardware reconfiguration replays the same persistent vif; it does
	 * not call .remove_interface first. Still reject a second interface. */
	if (sm->has_vif && sm->vif != vif) {
		return -EBUSY;
	}
	/* Include MAC/LLC plus software CCMP IV (8) and MIC (8). */
	ieee80211_vif_to_wdev(vif)->netdev->max_mtu = S31_SOFTMAC_FRAME_MAX - 48;
	if (!sm->has_vif)
		ieee80211_vif_to_wdev(vif)->netdev->mtu = ETH_DATA_LEN;
	sm->vif = vif;
	sm->has_vif = true;
	return 0;
}
static void sm_remove(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	struct s31_sm *sm = hw->priv;

	if (sm->vif == vif) {
		sm->vif = NULL;
		sm->has_vif = false;
	}
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
 struct s31_wifi_control ctl={.operation=S31_WIFI_SOFTMAC_COEX_BSS};
 unsigned long flags;
 bool running, filter;
 spin_lock_irqsave(&sm->lock,flags);
 running=sm->running;
 ctl.field=sm->scanning ? 1 : sm->connecting ? 3 : sm->associated ? 2 : 0;
 ctl.beacon_interval=sm->coex_beacon_interval;
 ctl.offset=sm->coex_aid;
 filter=running && sm->associated && !sm->scanning &&
   !(sm->filter_flags & (FIF_OTHER_BSS | FIF_BCN_PRBRESP_PROMISC)) &&
   is_valid_ether_addr(sm->bssid);
 ether_addr_copy(ctl.mac,sm->bssid);
 spin_unlock_irqrestore(&sm->lock,flags);
 if (!running) return;
 /* Reuse the existing cold BSS/filter owner. Firmware coalesces unchanged
  * state; no per-packet work or duplicate MLME/power-save timers are added. */
 if (esp32s31_radio_wifi_control(&ctl))
  pr_warn("esp32s31-softmac: coexistence BSS update failed\n");
 if (!sm_hw_bssid_filter) return;
 ctl.operation=S31_WIFI_SOFTMAC_BSSID_FILTER;
 ctl.field=filter;
 if (esp32s31_radio_wifi_control(&ctl))
  pr_warn("esp32s31-softmac: hardware BSSID filter update failed\n");
 else {
  WRITE_ONCE(sm_hw_bssid_filter_active,!!ctl.field);
  sm_hw_bssid_filter_updates++;
 }
}
static void sm_queue_filter_locked(struct s31_sm *sm)
{
 if (sm->running)
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
/* mac80211 calls this sleepable hook before publishing each auth/(re)assoc
 * frame. Join the existing cold filter/coex owner so the shared radio policy
 * reserves connecting airtime before that frame can reach PP. */
static void sm_mgd_prepare_tx(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
                             struct ieee80211_prep_tx_info *info)
{
 struct s31_sm *sm = hw->priv;
 unsigned long flags;
 if (info->subtype != IEEE80211_STYPE_AUTH &&
     info->subtype != IEEE80211_STYPE_ASSOC_REQ &&
     info->subtype != IEEE80211_STYPE_REASSOC_REQ) return;
 spin_lock_irqsave(&sm->lock, flags);
 if (!sm->associated) {
  sm->connecting = true;
  sm_queue_filter_locked(sm);
 }
 spin_unlock_irqrestore(&sm->lock, flags);
 flush_work(&sm->filter_io);
}
static void sm_mgd_complete_tx(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
                              struct ieee80211_prep_tx_info *info)
{
 struct s31_sm *sm = hw->priv;
 unsigned long flags;
 /* Successful auth must retain connecting policy through association.
  * Successful association is published by BSS_CHANGED_ASSOC. */
 if (info->success || (info->subtype != IEEE80211_STYPE_AUTH &&
     info->subtype != IEEE80211_STYPE_ASSOC_REQ &&
     info->subtype != IEEE80211_STYPE_REASSOC_REQ)) return;
 spin_lock_irqsave(&sm->lock, flags);
 sm->connecting = false;
 sm_queue_filter_locked(sm);
 spin_unlock_irqrestore(&sm->lock, flags);
 flush_work(&sm->filter_io);
}
static void sm_bss_changed(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
                           struct ieee80211_bss_conf *info, u64 changed)
{
 struct s31_sm *sm = hw->priv;
 unsigned long flags;
 spin_lock_irqsave(&sm->lock,flags);
 if (changed & BSS_CHANGED_ASSOC) {
  sm->associated=vif->cfg.assoc;
  if (!sm->associated) sm->connecting=false;
 }
 if (changed & (BSS_CHANGED_ASSOC | BSS_CHANGED_BEACON_INT)) {
  sm->coex_beacon_interval=info->beacon_int;
  sm->coex_aid=vif->cfg.assoc ? vif->cfg.aid : 0;
 }
 if (changed & BSS_CHANGED_BSSID) {
  if (info->bssid) ether_addr_copy(sm->bssid,info->bssid);
  else eth_zero_addr(sm->bssid);
  if (!is_valid_ether_addr(sm->bssid)) sm->connecting=false;
 }
 if ((changed & BSS_CHANGED_ASSOC) && !vif->cfg.assoc)
  sm_queue_filter_locked(sm);
 spin_unlock_irqrestore(&sm->lock,flags);
 if (changed & BSS_CHANGED_ASSOC)
  esp32s31_radio_idle_poll_set(sm_auto_idle_poll && vif->cfg.assoc &&
                                READ_ONCE(sm->running));
 if ((changed & BSS_CHANGED_ASSOC) && !vif->cfg.assoc)
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
 if (changed & (BSS_CHANGED_ASSOC | BSS_CHANGED_BEACON_INT | BSS_CHANGED_BSSID))
  flush_work(&sm->filter_io);
}
/* Single AP frontend. Join SRAM publication before any native cancel.
 * No submit/spin lock is held while entering the sleeping control API. */
static void sm_tx_ba_disable(struct s31_sm *sm, bool flushing)
{
	unsigned long flags;
	bool was_flushing;
	spin_lock_bh(&sm->submit_lock);
	spin_lock_irqsave(&sm->lock, flags);
	sm->tx_ba_operational = false;
	was_flushing = sm->tx_ba_flushing;
	sm->tx_ba_flushing = flushing || sm->tx_drain_error;
	if (was_flushing && !sm->tx_ba_flushing && sm->running &&
	    (sm->txq_pending_ac || !skb_queue_empty(&sm->tx)))
		sm_schedule_rx_locked(sm);
	spin_unlock_irqrestore(&sm->lock, flags);
	spin_unlock_bh(&sm->submit_lock);
}

static int sm_tx_ba_wait(struct s31_sm *sm)
{
	unsigned long flags;
	unsigned int i;
	bool busy = false, running;
	spin_lock_irqsave(&sm->lock, flags);
	reinit_completion(&sm->tx_ba_drained);
	sm->tx_ba_waiting = true;
	for (i = 0; i < S31_SOFTMAC_PENDING; i++)
		if (sm->pending[i].skb &&
		    (IEEE80211_SKB_CB(sm->pending[i].skb)->flags & IEEE80211_TX_CTL_AMPDU)) {
			busy = true; break;
		}
	spin_unlock_irqrestore(&sm->lock, flags);
	if (busy)
		wait_for_completion_timeout(&sm->tx_ba_drained, msecs_to_jiffies(2200));
	/* Recheck under the publication lock: completion can race the timeout. */
	spin_lock_irqsave(&sm->lock, flags);
	sm->tx_ba_waiting = false;
	busy = false;
	for (i = 0; i < S31_SOFTMAC_PENDING; i++)
		if (sm->pending[i].skb &&
		    (IEEE80211_SKB_CB(sm->pending[i].skb)->flags & IEEE80211_TX_CTL_AMPDU)) {
			busy = true; break;
		}
	running = sm->running;
	spin_unlock_irqrestore(&sm->lock, flags);
	return !running ? -ESHUTDOWN : busy ? -ETIMEDOUT : 0;
}

static void sm_tx_drain_failed(struct s31_sm *sm, int error)
{
	unsigned long flags;
	bool restart;

	spin_lock_irqsave(&sm->lock, flags);
	restart = !sm->tx_drain_error && sm->running;
	sm->tx_drain_error = error;
	sm->tx_ba_flushing = true;

	spin_unlock_irqrestore(&sm->lock, flags);
	pr_warn_ratelimited("esp32s31-softmac: native TX retirement failed: %d, publication closed, restart=%u\n",
			    error, restart);
	if (restart) ieee80211_restart_hw(sm->hw);
}

static noinline int sm_tx_ba_flush(struct s31_sm *sm, const u8 *addr)
{
	struct s31_wifi_control c = { .operation = S31_WIFI_SOFTMAC_TX_BA_FLUSH };
	struct sk_buff_head dropped;
	struct sk_buff *skb;
	unsigned long flags;
	unsigned int i;
	int ret = 0;
	sm_tx_ba_disable(sm, true);

	esp32s31_radio_softmac_tx_quiesce(sm->epoch);
	/* Peerless/unaggregated flights still need the real PP drain before
	 * keys or BSS ownership can retire. Never infer empty from BA state. */
	if (!READ_ONCE(sm->tx_native_stopped)) {
		memcpy(c.mac, addr, ETH_ALEN);
		ret = esp32s31_radio_wifi_control(&c);
	}
	if (ret)
		pr_warn_ratelimited("esp32s31-softmac: TX drain control rejected, frames dropped locally: %d\n",
				    ret);
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
	/* Local skb retirement is separate from the native PP ownership result.
	 * Never reopen publication merely because mac80211 must finish teardown. */
	if (ret) sm_tx_drain_failed(sm, ret);
	return ret;
}

static noinline int sm_tx_ba_action_locked(struct s31_sm *sm, struct ieee80211_vif *vif,
			   struct ieee80211_ampdu_params *p)
{
	struct s31_wifi_control c = {0};
	int ret = 0;
	if (READ_ONCE(sm->tx_drain_error) &&
	    (p->action == IEEE80211_AMPDU_TX_START ||
	     p->action == IEEE80211_AMPDU_TX_OPERATIONAL)) return -EIO;
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
		/* The aggregate cap must fit the negotiated buffer. This check
		 * alone does not establish the native retry sequence window: the
		 * shim enables TRC BA without conveying the full buffer size.
		 * Small-window/retry-hole qualification remains separate from
		 * the frontend's global pending/source/descriptor ownership. */
		if (p->buf_size < sm_tx_ampdu) return -EOPNOTSUPP;
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
		c.operation = S31_WIFI_SOFTMAC_TX_BA_OFF;
		if (sm->tx_native_peer) {
			ret = sm_tx_ba_wait(sm);
			if (!ret) ret = esp32s31_radio_wifi_control(&c);
			if (ret == -ESHUTDOWN) { ret = 0; break; }
			/* A native EB may outlive its Linux timeout. One synchronous PP
			 * drain resolves it; do not poll the closed native flight table. */
			if (ret) ret = sm_tx_ba_flush(sm, p->sta->addr);
			if (!ret) sm_tx_ba_disable(sm, false);
		}
		ieee80211_stop_tx_ba_cb_irqsafe(vif, p->sta->addr, p->tid);
		/* The failed native drain is quarantined/restarted separately. */
		ret = 0;
		break;
	case IEEE80211_AMPDU_TX_STOP_FLUSH:
	case IEEE80211_AMPDU_TX_STOP_FLUSH_CONT:
		ret = sm_tx_ba_flush(sm, p->sta->addr);
		/* mac80211 must finish destroying its session even if the backend
		 * needs recovery. tx_drain_error keeps native publication closed. */
		ret = 0;
		break;
	default:
		return -EOPNOTSUPP;
	}

	return ret;
}

/* Only cold BA/control retirement is serialized. The packet path never
 * takes this mutex; a timeout worker cannot reopen a concurrent teardown. */
static int sm_tx_ba_action(struct s31_sm *sm, struct ieee80211_vif *vif,
			   struct ieee80211_ampdu_params *p)
{
	int ret;
	mutex_lock(&sm->tx_ba_mutex);
	ret = sm_tx_ba_action_locked(sm, vif, p);
	mutex_unlock(&sm->tx_ba_mutex);
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
static int sm_sta_remove_locked(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
                         struct ieee80211_sta *sta)
{
 struct s31_sm *sm = hw->priv;
 struct s31_wifi_control c = { .operation = S31_WIFI_SOFTMAC_TX_PEER_DEL };
 int ret = 0;
 ret = sm_tx_ba_flush(sm, sta->addr);
 if (!ret && sm->tx_native_peer && !READ_ONCE(sm->tx_native_stopped)) {
   memcpy(c.mac, sta->addr, ETH_ALEN);
   ret = esp32s31_radio_wifi_control(&c);
 }
 if (!ret) sm->tx_native_peer = false;
 else sm_tx_drain_failed(sm, ret);
 sm->tx_ba_requested = false;
 sm_tx_ba_disable(sm, false);
 /* Native failure remains sticky; the Linux station can retire safely
  * after source join because PP owns final copies, not Linux skb data. */
 return 0;
}
static int sm_sta_remove(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
                         struct ieee80211_sta *sta)
{
 struct s31_sm *sm = hw->priv;
 int ret;
 mutex_lock(&sm->tx_ba_mutex);
 ret = sm_sta_remove_locked(hw, vif, sta);
 mutex_unlock(&sm->tx_ba_mutex);
 return ret;
}
static int sm_sta_state(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
                        struct ieee80211_sta *sta,
                        enum ieee80211_sta_state old,
                        enum ieee80211_sta_state new)
{
 struct s31_sm *sm = hw->priv;
 unsigned long flags;
 int ret = 0;
 /* Preserve the old sta_add/remove timing used by mac80211's fallback.
  * Peer capabilities and native BA retirement keep their existing owners. */
 if (old == IEEE80211_STA_AUTH && new == IEEE80211_STA_ASSOC)
  ret = sm_sta_add(hw, vif, sta);
 else if (old == IEEE80211_STA_ASSOC && new == IEEE80211_STA_AUTH)
  ret = sm_sta_remove(hw, vif, sta);
 if (ret) return ret;
 if (new == IEEE80211_STA_AUTHORIZED || old == IEEE80211_STA_AUTHORIZED ||
     new == IEEE80211_STA_NOTEXIST) {
  /* Association alone does not complete a WPA/802.1X connection. Keep
   * connecting airtime through the key exchange; authorization is the
   * controlled-port boundary for both protected and open networks. */
  spin_lock_irqsave(&sm->lock, flags);
  sm->connecting = new != IEEE80211_STA_AUTHORIZED &&
                   new != IEEE80211_STA_NOTEXIST && sm->associated;
  sm_queue_filter_locked(sm);
  spin_unlock_irqrestore(&sm->lock, flags);
  flush_work(&sm->filter_io);
 }
 return 0;
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
	bool tx_offload = sm_hw_tx_ccmp && pairwise;
	u8 token = 0;
	int ret;
	if (key->cipher != WLAN_CIPHER_SUITE_CCMP)
		return -EOPNOTSUPP;
	/* Reusing a token while old plaintext may still be queued is unsafe.
	 * Do not silently change crypto policy when a live epoch exhausts it.
	 * Only the fully joined stop boundary above permits token reuse. */
	if (cmd != DISABLE_KEY && tx_offload && sm->tx_key_generation >= 127)
		return -ENOSPC;
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
			/* Never wrap a token within an active hardware lifetime. */
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

		return 0;
	}

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
 .mgd_prepare_tx = sm_mgd_prepare_tx, .mgd_complete_tx = sm_mgd_complete_tx,
	.bss_info_changed = sm_bss_changed,
	.sta_state = sm_sta_state,
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
	spin_lock_init(&sm->rx_capture_lock);
	mutex_init(&sm->tx_ba_mutex);

	INIT_CSD(&sm->rx_csd, sm_rx_kick, sm);
	init_completion(&sm->rx_kick_done);
	init_completion(&sm->tx_ba_drained);
	sm->tx_native_stopped = true;
	complete(&sm->rx_kick_done);
	skb_queue_head_init(&sm->tx);
	skb_queue_head_init(&sm->status_queue);
	for (i = 0; i < sm->rx_slots; i++) {
		if (i >= sm->rx_idle_limit) continue;
		sm->rx[i].skb = sm_alloc_rx_skb(sm, false);
		sm_rx_mark_ready(sm, i, sm->rx[i].skb);
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
			/* Finish Internet checksums at the final private SRAM copy,
			 * before native publication/CCMP. Avoid a second PSRAM payload pass. */
			hw->netdev_features |= NETIF_F_SG | NETIF_F_HW_CSUM;
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
		/* S31 HE20 is one stream, MCS0..9. The native HE decoder accepts
		 * indices0..9; do not artificially restrict AP downlink to MCS7. */
		sm->iftype.he_cap.he_mcs_nss_supp.rx_mcs_80 = cpu_to_le16(0xfffd);
		sm->iftype.he_cap.he_mcs_nss_supp.tx_mcs_80 =
			cpu_to_le16(sm_he_tx && sm_data_tx_mcs > 7 ? 0xfffd : 0xfffc);
		/* HE160 and80+80 are unsupported by this20MHz radio. */
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
	/* Group keys deliberately use mac80211 TX crypto (set_key returns1).
	 * A failed pairwise hardware install must not silently select it too. */
	ieee80211_hw_set(hw, SW_CRYPTO_CONTROL);

	{
		static const u32 ciphers[] = { WLAN_CIPHER_SUITE_CCMP };
		hw->wiphy->cipher_suites = ciphers;
		hw->wiphy->n_cipher_suites = ARRAY_SIZE(ciphers);
	}
	SET_IEEE80211_DEV(hw, parent);

	mutex_lock(&s31_sm_frontend_mutex);
	s31_sm_frontend = sm;
	mutex_unlock(&s31_sm_frontend_mutex);
	schedule_delayed_work(&sm->register_work, 0);
	return 0;
}
void s31_radio_wifi_frontend_exit(void)
{
	struct s31_sm *sm;
	unsigned int i;

	mutex_lock(&s31_sm_frontend_mutex);
	sm = xchg(&s31_sm_frontend, NULL);
	mutex_unlock(&s31_sm_frontend_mutex);
	if (!sm)
		return;
	cancel_delayed_work_sync(&sm->register_work);
	esp32s31_radio_idle_poll_set(false);
	if (sm->registered)
		ieee80211_unregister_hw(sm->hw);
	cancel_delayed_work_sync(&sm->timer);
	cancel_work_sync(&sm->io);
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
	/* The wiphy child's PM callback stops mac80211 before the parent radio
	 * resets. Refuse an out-of-order reset while its producer is still live. */
	return s31_sm_frontend && READ_ONCE(s31_sm_frontend->running) ? -EBUSY : 0;
}
int s31_radio_wifi_frontend_resume(void)
{
	return 0;
}
