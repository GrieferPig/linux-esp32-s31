/* SPDX-License-Identifier: GPL-2.0-only */
/* hdev->send transfers ownership on success. EAGAIN cannot be returned for
 * transient ring pressure: hci_send_frame frees that skb instead of retrying.
 * Keep a bounded native FIFO in Linux memory, then copy to the SRAM ring.
 * The existing radio owner retries after a real slot-release edge, on its
 * Linux stack after leaving the foreign window. No additional worker/poll.
 */
#define S31_NATIVE_TX_MAX_PENDING 64U
#define S31_NATIVE_TX_DRAIN_BUDGET 8U
static unsigned long s31_native_tx_accepted, s31_native_tx_forwarded;
static unsigned long s31_native_tx_deferred, s31_native_tx_cancelled;
static unsigned long s31_native_tx_failed, s31_native_tx_peak, s31_native_tx_pending;
module_param_named(bt_native_tx_accepted, s31_native_tx_accepted, ulong, 0400);
module_param_named(bt_native_tx_forwarded, s31_native_tx_forwarded, ulong, 0400);
module_param_named(bt_native_tx_deferred, s31_native_tx_deferred, ulong, 0400);
module_param_named(bt_native_tx_cancelled, s31_native_tx_cancelled, ulong, 0400);
module_param_named(bt_native_tx_failed, s31_native_tx_failed, ulong, 0400);
module_param_named(bt_native_tx_peak, s31_native_tx_peak, ulong, 0400);
module_param_named(bt_native_tx_pending, s31_native_tx_pending, ulong, 0400);

static unsigned int s31_hci_native_tx_limit(struct hci_dev *hdev)
{
    /* Normal HCI producers already obey these credits. One additional slot
     * admits the serialized command. Cap raw-socket injection independently;
     * it must not turn controller backpressure into unbounded kernel memory.
     */
    unsigned int limit = 1;
    limit += min_t(unsigned int, READ_ONCE(hdev->acl_pkts), S31_NATIVE_TX_MAX_PENDING);
    limit += min_t(unsigned int, READ_ONCE(hdev->le_pkts), S31_NATIVE_TX_MAX_PENDING);
    limit += min_t(unsigned int, READ_ONCE(hdev->sco_pkts), S31_NATIVE_TX_MAX_PENDING);
    limit += min_t(unsigned int, READ_ONCE(hdev->iso_pkts), S31_NATIVE_TX_MAX_PENDING);
    return min_t(unsigned int, limit, S31_NATIVE_TX_MAX_PENDING);
}

/* tx_queue.lock serializes admission, retries and close/flush. The typed ring
 * operation copies bounded bytes under its own short lock and never sleeps.
 */
static void s31_hci_native_tx_drain_locked(struct s31_hci *radio)
{
    unsigned int budget = S31_NATIVE_TX_DRAIN_BUDGET;
    struct sk_buff *skb;
    while (budget-- && (skb = skb_peek(&radio->tx_queue))) {
        u8 type = hci_skb_pkt_type(skb);
        int ret = esp32s31_radio_hci_send(type, skb->data, skb->len);
        if (ret == -EAGAIN) {
            s31_native_tx_deferred++;
            break; /* Preserve this skb and all suffixes in FIFO order. */
        }
        __skb_unlink(skb, &radio->tx_queue);
        s31_native_tx_pending--;
        if (ret) {
            s31_native_tx_failed++;
            radio->hdev->stat.err_tx++;
        } else {
            switch (type) {
            case HCI_COMMAND_PKT: radio->hdev->stat.cmd_tx++; break;
            case HCI_ACLDATA_PKT: radio->hdev->stat.acl_tx++; break;
            case HCI_SCODATA_PKT: radio->hdev->stat.sco_tx++; break;
            }
            radio->hdev->stat.byte_tx += skb->len;
            s31_native_tx_forwarded++;
        }
        kfree_skb(skb); /* Ring copied it, or a permanent failure retired it. */
    }
}

static void s31_hci_native_tx_ready(void *context)
{
    struct s31_hci *radio = context;
    unsigned long flags;
    spin_lock_irqsave(&radio->tx_queue.lock, flags);
    if (!radio->tx_stopped && !READ_ONCE(radio->suspended))
        s31_hci_native_tx_drain_locked(radio);
    spin_unlock_irqrestore(&radio->tx_queue.lock, flags);
}

static void s31_hci_native_tx_purge(struct s31_hci *radio, bool stop)
{
    struct sk_buff *skb;
    unsigned long flags;
    spin_lock_irqsave(&radio->tx_queue.lock, flags);
    if (stop)
        radio->tx_stopped = true;
    while ((skb = __skb_dequeue(&radio->tx_queue))) {
        s31_native_tx_pending--;
        s31_native_tx_cancelled++;
        kfree_skb(skb);
    }
    spin_unlock_irqrestore(&radio->tx_queue.lock, flags);
}

static void s31_hci_native_tx_start(struct s31_hci *radio)
{
    unsigned long flags;
    spin_lock_irqsave(&radio->tx_queue.lock, flags);
    radio->tx_stopped = false;
    spin_unlock_irqrestore(&radio->tx_queue.lock, flags);
}

static int s31_hci_native_tx_submit(struct hci_dev *hdev, struct sk_buff *skb)
{
    struct s31_hci *radio = hci_get_drvdata(hdev);
    unsigned long flags;
    int ret = 0;
    if (!skb || skb->len > ESP32S31_RADIO_HCI_FRAME_MAX - 1)
        return -EMSGSIZE;
    if (!s31_hci_frame_valid(hci_skb_pkt_type(skb), skb->data, skb->len))
        return -EINVAL;
    spin_lock_irqsave(&radio->tx_queue.lock, flags);
    if (radio->tx_stopped || READ_ONCE(radio->suspended)) {
        ret = -ENODEV;
        goto out;
    }
    if (skb_queue_len(&radio->tx_queue) >= s31_hci_native_tx_limit(hdev)) {
        ret = -ENOBUFS; /* Rejected: caller still owns this skb. */
        goto out;
    }
    __skb_queue_tail(&radio->tx_queue, skb);
    s31_native_tx_accepted++;
    s31_native_tx_pending++;
    s31_native_tx_peak = max(s31_native_tx_peak, s31_native_tx_pending);
    s31_hci_native_tx_drain_locked(radio);
out:
    spin_unlock_irqrestore(&radio->tx_queue.lock, flags);
    return ret;
}
