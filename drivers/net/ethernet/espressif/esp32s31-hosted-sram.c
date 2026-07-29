// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S31 in-package ESP-Hosted-FG transport.
 *
 * hart0 owns the Wi-Fi/Bluetooth controller and hart1 owns this driver.  The
 * transport is intentionally small: two SPSC rings in reserved HP SRAM plus
 * CPU_INT_FROM_CPU doorbells.  ESP-Hosted payload headers are retained above
 * the transport.
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/poll.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <asm/fixmap.h>
#include <asm/sbi.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "s31_hosted_sram.h"

#define S31_HP_SYSTEM_CPU_INT_FROM_CPU_2	0x20586018U
#define S31_HP_SYSTEM_CPU_INT_FROM_CPU_3	0x2058601cU
#define S31_HOSTED_NAPI_WEIGHT		16
#define S31_HOSTED_FIXMAP_PAGES		17
#define S31_HOSTED_SELFTEST_ROUNDS	(S31_HOSTED_SLOT_COUNT + 1)
#define S31_HOSTED_SELFTEST_SIZE	ETH_DATA_LEN
#define S31_HOSTED_SERIAL_FRAGMENT	ETH_DATA_LEN
#define S31_HOSTED_SERIAL_MAX_WRITE	SZ_64K
#define S31_HOSTED_SERIAL_QUEUE_DEPTH	32
#define S31_HOSTED_MORE_FRAGMENT		BIT(0)
#define S31_HOSTED_PRIV_EVENT_PACKET	0x33
#define S31_HOSTED_PRIV_EVENT_INIT	0x22
#define S31_HOSTED_TLV_CAPABILITY	0x11
#define S31_HOSTED_TLV_CHIP_ID		0x12
#define S31_HOSTED_TLV_CAP_EXT		0x16
#define S31_HOSTED_HOST_CAPABILITY	0x44
#define S31_HOSTED_HOST_CHIP_ID		0x45
#define S31_HOSTED_HOST_RAW_TP		0x46
#define S31_HOSTED_HOST_THROTTLE_HIGH	0x47
#define S31_HOSTED_HOST_THROTTLE_LOW	0x48
#define S31_SBI_EXT_HOSTED		0x09000001UL
#define S31_SBI_HOSTED_TX		0
#define S31_SBI_HOSTED_RX_ACK		1
#define S31_SBI_HOSTED_H1_READY		2

struct s31_hosted {
	struct device *dev;
	struct net_device *ndev;
	struct hci_dev *hdev;
	struct work_struct hci_open_work;
	struct napi_struct napi;
	void __iomem *shmem;
	void __iomem *db_h0_to_h1;
	void __iomem *db_h1_to_h0;
	int irq;
	spinlock_t tx_lock;
	u8 *rx_frame;
	atomic_t irq_disabled;
	struct completion pong;
	struct completion selftest_done;
	struct miscdevice serial_misc;
	struct sk_buff_head serial_rx;
	u8 *serial_reassembly;
	size_t serial_reassembly_length;
	u16 serial_reassembly_sequence;
	bool serial_reassembly_active;
	wait_queue_head_t serial_wait;
	struct mutex serial_rx_mutex;
	struct mutex serial_tx_mutex;
	u32 generation;
	u32 capabilities_ext;
	u32 rx_errors;
	u32 tx_drops;
	u16 tx_sequence;
	u16 serial_tx_sequence;
	u16 selftest_length;
	u16 selftest_round;
	u8 capabilities;
	u8 chip_id;
	bool selftest_ok;
	bool hosted_ready;
};

static u16 s31_frame_checksum(const u8 *frame, size_t length)
{
	u16 checksum = 0;

	while (length--)
		checksum += *frame++;
	return checksum;
}

static int s31_send_payload_meta(struct s31_hosted *hosted, u8 if_type,
				 const void *payload, size_t length, u8 flags,
				 u16 sequence, u8 packet_type);
static int s31_send_payload(struct s31_hosted *hosted, u8 if_type,
			    const void *payload, size_t length, u8 hci_type);

static int s31_hci_open(struct hci_dev *hdev)
{
	return 0;
}

static int s31_hci_close(struct hci_dev *hdev)
{
	return 0;
}

static int s31_hci_flush(struct hci_dev *hdev)
{
	return 0;
}

static int s31_hci_send(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct s31_hosted *hosted = hci_get_drvdata(hdev);
	u8 packet_type = hci_skb_pkt_type(skb);
	int ret;

	if (!hosted->hosted_ready) {
		ret = -EHOSTDOWN;
		goto out;
	}

	switch (packet_type) {
	case HCI_COMMAND_PKT:
		hdev->stat.cmd_tx++;
		break;
	case HCI_ACLDATA_PKT:
		hdev->stat.acl_tx++;
		break;
	case HCI_SCODATA_PKT:
		hdev->stat.sco_tx++;
		break;
	case HCI_ISODATA_PKT:
		break;
	default:
		ret = -EINVAL;
		goto out;
	}

	ret = s31_send_payload(hosted, S31_HOSTED_HCI_IF, skb->data,
			       skb->len, packet_type);
	if (!ret)
		hdev->stat.byte_tx += skb->len;
out:
	kfree_skb(skb);
	return ret;
}

static int s31_hci_register(struct s31_hosted *hosted)
{
	struct hci_dev *hdev;
	int ret;

	hdev = hci_alloc_dev();
	if (!hdev)
		return -ENOMEM;

	hdev->bus = HCI_VIRTUAL;
	hdev->open = s31_hci_open;
	hdev->close = s31_hci_close;
	hdev->flush = s31_hci_flush;
	hdev->send = s31_hci_send;
	SET_HCIDEV_DEV(hdev, hosted->dev);
	hci_set_drvdata(hdev, hosted);

	ret = hci_register_dev(hdev);
	if (ret) {
		hci_free_dev(hdev);
		return ret;
	}
	hosted->hdev = hdev;
	return 0;
}

static void s31_hci_unregister(struct s31_hosted *hosted)
{
	if (!hosted->hdev)
		return;
	hci_unregister_dev(hosted->hdev);
	hci_free_dev(hosted->hdev);
	hosted->hdev = NULL;
}

static void s31_hci_open_work(struct work_struct *work)
{
	struct s31_hosted *hosted =
		container_of(work, struct s31_hosted, hci_open_work);
	int ret;

	if (!hosted->hdev || !hosted->hosted_ready)
		return;
	ret = hci_dev_open(hosted->hdev->id);
	if (ret && ret != -EALREADY)
		dev_err(hosted->dev, "Bluetooth HCI initialization failed: %d\n",
			ret);
	else
		dev_info(hosted->dev,
			 "Bluetooth HCI initialized: commands=%u tx=%u rx=%u bytes\n",
			 hosted->hdev->stat.cmd_tx,
			 hosted->hdev->stat.byte_tx,
			 hosted->hdev->stat.byte_rx);
}

static void s31_process_hci(struct s31_hosted *hosted, const u8 *payload,
			    u16 length, u8 packet_type)
{
	struct sk_buff *skb;

	if (!hosted->hdev || !length ||
	    (packet_type != HCI_EVENT_PKT &&
	     packet_type != HCI_ACLDATA_PKT &&
	     packet_type != HCI_SCODATA_PKT &&
	     packet_type != HCI_ISODATA_PKT)) {
		hosted->rx_errors++;
		return;
	}

	skb = bt_skb_alloc(length, GFP_ATOMIC);
	if (!skb) {
		hosted->rx_errors++;
		return;
	}
	skb_put_data(skb, payload, length);
	hci_skb_pkt_type(skb) = packet_type;
	hosted->hdev->stat.byte_rx += length;
	if (hci_recv_frame(hosted->hdev, skb))
		hosted->rx_errors++;
}

static void s31_hosted_serial_queue_purge(void *data)
{
	struct s31_hosted *hosted = data;

	skb_queue_purge(&hosted->serial_rx);
}

static u8 s31_selftest_byte(unsigned int round, unsigned int offset)
{
	return (u8)(round * 29U + offset * 17U + (offset >> 3));
}

static void s31_hosted_clear_fixmap(void *unused)
{
	unsigned int i;

	for (i = 0; i < S31_HOSTED_FIXMAP_PAGES; i++)
		clear_fixmap(FIX_S31_HOSTED_BEGIN - i);
}

static void __iomem *s31_hosted_fixmap_resource(struct platform_device *pdev,
						struct resource *res)
{
	phys_addr_t page_start = res->start & PAGE_MASK;
	unsigned long page_offset = offset_in_page(res->start);
	unsigned int i;
	int ret;

	if (resource_size(res) != S31_HOSTED_SRAM_SIZE ||
	    PAGE_ALIGN(page_offset + resource_size(res)) / PAGE_SIZE !=
		    S31_HOSTED_FIXMAP_PAGES)
		return ERR_PTR(-EINVAL);

	if (!devm_request_mem_region(&pdev->dev, res->start,
				     resource_size(res), dev_name(&pdev->dev)))
		return ERR_PTR(-EBUSY);

	for (i = 0; i < S31_HOSTED_FIXMAP_PAGES; i++)
		set_fixmap_io(FIX_S31_HOSTED_BEGIN - i,
			      page_start + i * PAGE_SIZE);

	ret = devm_add_action_or_reset(&pdev->dev, s31_hosted_clear_fixmap,
				       NULL);
	if (ret)
		return ERR_PTR(ret);

	return (void __iomem *)(fix_to_virt(FIX_S31_HOSTED_BEGIN) +
				page_offset);
}

static inline void __iomem *s31_ctrl_ptr(struct s31_hosted *hosted,
					size_t offset)
{
	return hosted->shmem + offset;
}

static inline void __iomem *s31_ring_ptr(struct s31_hosted *hosted,
					bool h0_to_h1, size_t offset)
{
	size_t base = offsetof(struct s31_hosted_control,
			       h0_to_h1);

	if (!h0_to_h1)
		base = offsetof(struct s31_hosted_control, h1_to_h0);
	return hosted->shmem + base + offset;
}

static inline void __iomem *s31_slot_ptr(struct s31_hosted *hosted,
					bool h0_to_h1, u32 index)
{
	size_t base = h0_to_h1 ? S31_HOSTED_H0_TO_H1_OFFSET :
				    S31_HOSTED_H1_TO_H0_OFFSET;

	return hosted->shmem + base + index * S31_HOSTED_SLOT_SIZE;
}

static int s31_send_frame(struct s31_hosted *hosted, void *frame,
			  size_t length)
{
	struct s31_esp_payload_header *header = frame;
	struct sbiret sbi_ret;
	unsigned long flags;

	if (length < sizeof(*header) || length > S31_HOSTED_SLOT_DATA_SIZE)
		return -EMSGSIZE;

	spin_lock_irqsave(&hosted->tx_lock, flags);
	if (!header->seq_num)
		header->seq_num = cpu_to_le16(++hosted->tx_sequence);
	header->checksum = 0;
	header->checksum = cpu_to_le16(s31_frame_checksum(frame, length));
	sbi_ret = sbi_ecall(S31_SBI_EXT_HOSTED, S31_SBI_HOSTED_TX,
			    (unsigned long)frame, length, 0, 0, 0, 0);
	spin_unlock_irqrestore(&hosted->tx_lock, flags);

	if (sbi_ret.error) {
		if (sbi_ret.error == -9) {
			hosted->tx_drops++;
			return -ENOSPC;
		}
		return -EIO;
	}
	return 0;
}

static int s31_send_payload(struct s31_hosted *hosted, u8 if_type,
			    const void *payload, size_t length, u8 hci_type)
{
	return s31_send_payload_meta(hosted, if_type, payload, length, 0, 0,
				     hci_type);
}

static int s31_send_payload_meta(struct s31_hosted *hosted, u8 if_type,
				 const void *payload, size_t length, u8 flags,
				 u16 sequence, u8 packet_type)
{
	struct s31_esp_payload_header *header;
	u8 *frame;
	size_t frame_length = sizeof(*header) + length;
	int ret;

	if (!payload || frame_length > S31_HOSTED_SLOT_DATA_SIZE)
		return -EMSGSIZE;

	frame = kmalloc(frame_length, GFP_ATOMIC);
	if (!frame)
		return -ENOMEM;
	header = (void *)frame;
	memset(header, 0, sizeof(*header));
	header->if_type = if_type;
	header->flags = flags;
	header->len = cpu_to_le16(length);
	header->offset = cpu_to_le16(sizeof(*header));
	header->seq_num = cpu_to_le16(sequence);
	header->hci_pkt_type = packet_type;
	memcpy(frame + sizeof(*header), payload, length);

	ret = s31_send_frame(hosted, frame, frame_length);
	kfree(frame);
	return ret;
}

static void s31_send_host_init(struct s31_hosted *hosted)
{
	u8 event[] = {
		S31_HOSTED_PRIV_EVENT_INIT, 15,
		S31_HOSTED_HOST_CAPABILITY, 1, 0,
		S31_HOSTED_HOST_CHIP_ID, 1, hosted->chip_id,
		S31_HOSTED_HOST_RAW_TP, 1, 0,
		S31_HOSTED_HOST_THROTTLE_HIGH, 1, 80,
		S31_HOSTED_HOST_THROTTLE_LOW, 1, 20,
	};

	if (s31_send_payload_meta(hosted, S31_HOSTED_PRIV_IF, event,
				  sizeof(event), 0, 0,
				  S31_HOSTED_PRIV_EVENT_PACKET))
		dev_warn_ratelimited(hosted->dev,
				     "failed to send Hosted host-init event\n");
}

static void s31_update_link(struct s31_hosted *hosted, bool up)
{
	u8 mac[ETH_ALEN];

	if (up) {
		memcpy_fromio(mac,
			     s31_ctrl_ptr(hosted,
					  offsetof(struct s31_hosted_control,
						   sta_mac)),
			     sizeof(mac));
		if (is_valid_ether_addr(mac) &&
		    !ether_addr_equal(mac, hosted->ndev->dev_addr))
			eth_hw_addr_set(hosted->ndev, mac);
		netif_carrier_on(hosted->ndev);
		netif_wake_queue(hosted->ndev);
	} else {
		netif_carrier_off(hosted->ndev);
	}

	dev_info(hosted->dev, "radio link %s\n", up ? "up" : "down");
}

static bool s31_process_hosted_init(struct s31_hosted *hosted,
				    const u8 *payload, u16 length,
				    u8 packet_type)
{
	const u8 *pos;
	u8 left;

	if (packet_type != S31_HOSTED_PRIV_EVENT_PACKET || length < 2 ||
	    payload[0] != S31_HOSTED_PRIV_EVENT_INIT ||
	    payload[1] > length - 2)
		return false;

	pos = payload + 2;
	left = payload[1];
	while (left >= 2) {
		u8 tag = pos[0];
		u8 len = pos[1];

		if (len + 2 > left)
			break;
		if (tag == S31_HOSTED_TLV_CAPABILITY && len == 1)
			hosted->capabilities = pos[2];
		else if (tag == S31_HOSTED_TLV_CHIP_ID && len == 1)
			hosted->chip_id = pos[2];
		else if (tag == S31_HOSTED_TLV_CAP_EXT && len == 4)
			hosted->capabilities_ext = get_unaligned_le32(pos + 2);
		pos += len + 2;
		left -= len + 2;
	}

	hosted->hosted_ready = true;
	s31_send_host_init(hosted);
	schedule_work(&hosted->hci_open_work);
	dev_info(hosted->dev,
		 "ESP-Hosted ready: chip=%u capabilities=%#02x ext=%#08x\n",
		 hosted->chip_id, hosted->capabilities,
		 hosted->capabilities_ext);
	return true;
}

static void s31_process_private(struct s31_hosted *hosted, const u8 *payload,
				u16 length, u8 packet_type)
{
	const struct s31_hosted_control_msg *msg;

	if (s31_process_hosted_init(hosted, payload, length, packet_type))
		return;
	if (length < sizeof(*msg))
		return;
	msg = (const void *)payload;
	if (le32_to_cpu(msg->generation) != hosted->generation)
		return;

	switch (msg->type) {
	case S31_HOSTED_CTRL_PONG:
		complete(&hosted->pong);
		dev_info(hosted->dev, "SRAM transport pong, generation %u\n",
			 hosted->generation);
		break;
	case S31_HOSTED_CTRL_LINK:
		s31_update_link(hosted, msg->value == S31_HOSTED_LINK_UP);
		break;
	case S31_HOSTED_CTRL_RADIO_READY:
		dev_info(hosted->dev, "hart0 radio transport ready\n");
		break;
	default:
		dev_dbg(hosted->dev, "unknown private message %u\n", msg->type);
		break;
	}
}

static void s31_process_serial(struct s31_hosted *hosted, const u8 *payload,
			       u16 length, u8 flags, u16 sequence)
{
	struct sk_buff *skb;

	if (!hosted->serial_reassembly_active ||
	    hosted->serial_reassembly_sequence != sequence) {
		if (hosted->serial_reassembly_active)
			hosted->rx_errors++;
		hosted->serial_reassembly_active = true;
		hosted->serial_reassembly_sequence = sequence;
		hosted->serial_reassembly_length = 0;
	}
	if (length > S31_HOSTED_SERIAL_MAX_WRITE -
		     hosted->serial_reassembly_length) {
		hosted->rx_errors++;
		dev_warn_ratelimited(hosted->dev,
				     "serial RPC reassembly overflow\n");
		hosted->serial_reassembly_active = false;
		return;
	}
	memcpy(hosted->serial_reassembly + hosted->serial_reassembly_length,
	       payload, length);
	hosted->serial_reassembly_length += length;
	if (flags & S31_HOSTED_MORE_FRAGMENT)
		return;

	if (skb_queue_len(&hosted->serial_rx) >=
	    S31_HOSTED_SERIAL_QUEUE_DEPTH) {
		hosted->rx_errors++;
		hosted->serial_reassembly_active = false;
		dev_warn_ratelimited(hosted->dev,
				     "serial RPC message queue overflow\n");
		return;
	}
	skb = alloc_skb(hosted->serial_reassembly_length, GFP_ATOMIC);
	if (!skb) {
		hosted->rx_errors++;
		hosted->serial_reassembly_active = false;
		return;
	}
	skb_put_data(skb, hosted->serial_reassembly,
		     hosted->serial_reassembly_length);
	hosted->serial_reassembly_active = false;
	skb_queue_tail(&hosted->serial_rx, skb);
	wake_up_interruptible(&hosted->serial_wait);
}

static void s31_process_frame(struct s31_hosted *hosted, u8 *frame,
			      u16 frame_length)
{
	struct s31_esp_payload_header *header = (void *)frame;
	struct sk_buff *skb;
	const u8 *payload;
	u16 received_checksum;
	u16 offset;
	u16 length;

	if (frame_length < sizeof(*header))
		goto malformed;
	received_checksum = le16_to_cpu(header->checksum);
	header->checksum = 0;
	if (s31_frame_checksum(frame, frame_length) != received_checksum)
		goto malformed;
	offset = le16_to_cpu(header->offset);
	length = le16_to_cpu(header->len);
	if (offset < sizeof(*header) || offset + length > frame_length)
		goto malformed;
	payload = frame + offset;

	switch (header->if_type) {
	case S31_HOSTED_STA_IF:
	case S31_HOSTED_AP_IF:
		if (length < ETH_HLEN || length > ETH_FRAME_LEN)
			goto malformed;
		skb = napi_alloc_skb(&hosted->napi, length + NET_IP_ALIGN);
		if (!skb)
			return;
		skb_reserve(skb, NET_IP_ALIGN);
		skb_put_data(skb, payload, length);
		skb->protocol = eth_type_trans(skb, hosted->ndev);
		skb->ip_summed = CHECKSUM_NONE;
		hosted->ndev->stats.rx_packets++;
		hosted->ndev->stats.rx_bytes += length;
		napi_gro_receive(&hosted->napi, skb);
		break;
	case S31_HOSTED_PRIV_IF:
		s31_process_private(hosted, payload, length,
				    header->priv_pkt_type);
		break;
	case S31_HOSTED_SERIAL_IF:
		s31_process_serial(hosted, payload, length, header->flags,
				   le16_to_cpu(header->seq_num));
		break;
	case S31_HOSTED_HCI_IF:
		s31_process_hci(hosted, payload, length,
				header->hci_pkt_type);
		break;
	case S31_HOSTED_TEST_IF: {
		unsigned int round = READ_ONCE(hosted->selftest_round);
		unsigned int i;

		hosted->selftest_ok = length == hosted->selftest_length;
		for (i = 0; hosted->selftest_ok && i < length; i++)
			if (payload[i] != s31_selftest_byte(round, i))
				hosted->selftest_ok = false;
		complete(&hosted->selftest_done);
		break;
	}
	default:
		dev_dbg_ratelimited(hosted->dev, "unhandled Hosted interface %u\n",
				    header->if_type);
		break;
	}
	return;

malformed:
	hosted->rx_errors++;
	hosted->ndev->stats.rx_errors++;
}

static ssize_t s31_serial_read(struct file *file, char __user *buffer,
			       size_t count, loff_t *ppos)
{
	struct miscdevice *misc = file->private_data;
	struct s31_hosted *hosted =
		container_of(misc, struct s31_hosted, serial_misc);
	struct sk_buff *skb;
	int ret;

	if (!count)
		return 0;
	for (;;) {
		if (skb_queue_empty(&hosted->serial_rx)) {
			if (file->f_flags & O_NONBLOCK)
				return -EAGAIN;
			ret = wait_event_interruptible(
				hosted->serial_wait,
				!skb_queue_empty(&hosted->serial_rx));
			if (ret)
				return ret;
		}
		mutex_lock(&hosted->serial_rx_mutex);
		skb = skb_dequeue(&hosted->serial_rx);
		if (!skb) {
			mutex_unlock(&hosted->serial_rx_mutex);
			continue;
		}
		if (count < skb->len) {
			skb_queue_head(&hosted->serial_rx, skb);
			mutex_unlock(&hosted->serial_rx_mutex);
			return -EMSGSIZE;
		}
		ret = copy_to_user(buffer, skb->data, skb->len);
		count = skb->len;
		kfree_skb(skb);
		mutex_unlock(&hosted->serial_rx_mutex);
		return ret ? -EFAULT : count;
	}
}

static ssize_t s31_serial_write(struct file *file, const char __user *buffer,
				size_t count, loff_t *ppos)
{
	struct miscdevice *misc = file->private_data;
	struct s31_hosted *hosted =
		container_of(misc, struct s31_hosted, serial_misc);
	u8 *data;
	size_t offset = 0;
	u16 sequence;
	int ret = 0;

	if (!count)
		return 0;
	if (count > S31_HOSTED_SERIAL_MAX_WRITE)
		return -EMSGSIZE;
	if (!hosted->hosted_ready)
		return -EHOSTDOWN;

	data = memdup_user(buffer, count);
	if (IS_ERR(data))
		return PTR_ERR(data);

	mutex_lock(&hosted->serial_tx_mutex);
	sequence = ++hosted->serial_tx_sequence;
	while (offset < count) {
		size_t length = min_t(size_t, count - offset,
				      S31_HOSTED_SERIAL_FRAGMENT);
		u8 flags = offset + length < count ?
			   S31_HOSTED_MORE_FRAGMENT : 0;
		unsigned int retry;

		for (retry = 0; retry < 100; retry++) {
			ret = s31_send_payload_meta(hosted,
					S31_HOSTED_SERIAL_IF, data + offset,
					length, flags, sequence, 0);
			if (ret != -ENOSPC)
				break;
			usleep_range(500, 1000);
		}
		if (ret)
			break;
		offset += length;
	}
	mutex_unlock(&hosted->serial_tx_mutex);
	kfree(data);
	return ret ? ret : count;
}

static __poll_t s31_serial_poll(struct file *file, poll_table *wait)
{
	struct miscdevice *misc = file->private_data;
	struct s31_hosted *hosted =
		container_of(misc, struct s31_hosted, serial_misc);
	__poll_t mask = EPOLLOUT | EPOLLWRNORM;

	poll_wait(file, &hosted->serial_wait, wait);
	if (!skb_queue_empty(&hosted->serial_rx))
		mask |= EPOLLIN | EPOLLRDNORM;
	if (!hosted->hosted_ready)
		mask |= EPOLLERR;
	return mask;
}

static const struct file_operations s31_serial_fops = {
	.owner = THIS_MODULE,
	.read = s31_serial_read,
	.write = s31_serial_write,
	.poll = s31_serial_poll,
	.llseek = noop_llseek,
};

static int s31_hosted_selftest(struct s31_hosted *hosted)
{
	void __iomem *h0_ring = s31_ring_ptr(hosted, true, 0);
	void __iomem *h1_ring = s31_ring_ptr(hosted, false, 0);
	u8 *payload;
	unsigned int round;
	unsigned int i;
	int ret = 0;

	payload = kmalloc(S31_HOSTED_SELFTEST_SIZE, GFP_KERNEL);
	if (!payload)
		return -ENOMEM;

	hosted->selftest_length = S31_HOSTED_SELFTEST_SIZE;
	for (round = 0; round < S31_HOSTED_SELFTEST_ROUNDS; round++) {
		for (i = 0; i < S31_HOSTED_SELFTEST_SIZE; i++)
			payload[i] = s31_selftest_byte(round, i);

		reinit_completion(&hosted->selftest_done);
		WRITE_ONCE(hosted->selftest_round, round);
		hosted->selftest_ok = false;
		ret = s31_send_payload(hosted, S31_HOSTED_TEST_IF, payload,
				       S31_HOSTED_SELFTEST_SIZE, 0);
		if (ret)
			break;
		if (!wait_for_completion_timeout(&hosted->selftest_done,
						  msecs_to_jiffies(250))) {
			dev_warn(hosted->dev,
				 "self-test round %u timeout: rx p=%u c=%u "
				 "tx p=%u c=%u; h0 seen p=%u seq=%u "
				 "tests=%u echo_ret=%d db=%u\n",
				 round,
				 readl(h0_ring + offsetof(
					 struct s31_hosted_ring_state, producer)),
				 readl(h0_ring + offsetof(
					 struct s31_hosted_ring_state, consumer)),
				 readl(h1_ring + offsetof(
					 struct s31_hosted_ring_state, producer)),
				 readl(h1_ring + offsetof(
					 struct s31_hosted_ring_state, consumer)),
				 readl(s31_ctrl_ptr(hosted, offsetof(
					 struct s31_hosted_control,
					 h0_seen_h1_producer))),
				 readl(s31_ctrl_ptr(hosted, offsetof(
					 struct s31_hosted_control,
					 h0_seen_h1_sequence))),
				 readl(s31_ctrl_ptr(hosted, offsetof(
					 struct s31_hosted_control,
					 h0_apm_status))),
				 (s32)readl(s31_ctrl_ptr(hosted, offsetof(
					 struct s31_hosted_control,
					 h0_h1_doorbell))),
				 readl(hosted->db_h0_to_h1));
			ret = -ETIMEDOUT;
			break;
		}
		if (!hosted->selftest_ok) {
			ret = -EBADMSG;
			break;
		}
	}

	kfree(payload);
	return ret;
}

static int s31_hosted_poll(struct napi_struct *napi, int budget)
{
	struct s31_hosted *hosted = container_of(napi, struct s31_hosted, napi);
	void __iomem *ring = s31_ring_ptr(hosted, true, 0);
	u8 *frame = hosted->rx_frame;
	int work = 0;

	while (work < budget) {
		void __iomem *slot;
		u32 consumer;
		u32 producer;
		u32 sequence;
		u32 index;
		u16 length;

		consumer = readl(ring + offsetof(struct s31_hosted_ring_state,
					       consumer));
		rmb();
		producer = readl(ring + offsetof(struct s31_hosted_ring_state,
					       producer));
		if (consumer == producer)
			break;
		if (producer - consumer > S31_HOSTED_SLOT_COUNT) {
			u32 count = producer - consumer;

			hosted->rx_errors += count;
			hosted->ndev->stats.rx_errors += count;
			if (sbi_ecall(S31_SBI_EXT_HOSTED,
				      S31_SBI_HOSTED_RX_ACK, count,
				      0, 0, 0, 0, 0).error)
				dev_err_ratelimited(hosted->dev,
						    "failed to recover RX ring\n");
			break;
		}

		index = consumer & (S31_HOSTED_SLOT_COUNT - 1);
		slot = s31_slot_ptr(hosted, true, index);
		rmb();
		sequence = readl(slot + offsetof(struct s31_hosted_slot,
					       sequence));
		length = readw(slot + offsetof(struct s31_hosted_slot, length));
		if (sequence == consumer + 1 && length &&
		    length <= S31_HOSTED_SLOT_DATA_SIZE) {
			memcpy_fromio(frame,
				      slot + offsetof(struct s31_hosted_slot, data),
				      length);
			s31_process_frame(hosted, frame, length);
		} else {
			hosted->rx_errors++;
		}
		if (sbi_ecall(S31_SBI_EXT_HOSTED, S31_SBI_HOSTED_RX_ACK,
			      1, 0, 0, 0, 0, 0).error) {
			dev_err_ratelimited(hosted->dev,
					    "failed to acknowledge RX slot\n");
			break;
		}
		work++;
	}

	if (work < budget && napi_complete_done(napi, work)) {
		if (atomic_xchg(&hosted->irq_disabled, 0))
			enable_irq(hosted->irq);
		/* Cover a producer racing with the empty check above. */
		if (readl(ring + offsetof(struct s31_hosted_ring_state,
					 producer)) !=
		    readl(ring + offsetof(struct s31_hosted_ring_state,
					 consumer)) &&
		    napi_schedule_prep(napi)) {
			disable_irq_nosync(hosted->irq);
			atomic_set(&hosted->irq_disabled, 1);
			__napi_schedule(napi);
		}
	}

	return work;
}

static irqreturn_t s31_hosted_irq(int irq, void *data)
{
	struct s31_hosted *hosted = data;

	writel(0, hosted->db_h0_to_h1);
	if (napi_schedule_prep(&hosted->napi)) {
		disable_irq_nosync(irq);
		atomic_set(&hosted->irq_disabled, 1);
		__napi_schedule(&hosted->napi);
	}
	return IRQ_HANDLED;
}

static netdev_tx_t s31_hosted_xmit(struct sk_buff *skb,
				   struct net_device *ndev)
{
	struct s31_hosted *hosted = netdev_priv(ndev);
	int ret;

	if (unlikely(skb->len > ETH_FRAME_LEN)) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	ret = s31_send_payload(hosted, S31_HOSTED_STA_IF, skb->data,
			       skb->len, 0);
	if (ret == -ENOSPC) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}
	if (ret) {
		ndev->stats.tx_dropped++;
	} else {
		ndev->stats.tx_packets++;
		ndev->stats.tx_bytes += skb->len;
	}
	dev_kfree_skb(skb);
	return NETDEV_TX_OK;
}

static int s31_hosted_open(struct net_device *ndev)
{
	netif_start_queue(ndev);
	return 0;
}

static int s31_hosted_stop(struct net_device *ndev)
{
	netif_stop_queue(ndev);
	return 0;
}

static const struct net_device_ops s31_hosted_netdev_ops = {
	.ndo_open = s31_hosted_open,
	.ndo_stop = s31_hosted_stop,
	.ndo_start_xmit = s31_hosted_xmit,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_validate_addr = eth_validate_addr,
};

static int s31_hosted_probe(struct platform_device *pdev)
{
	struct net_device *ndev;
	struct s31_hosted *hosted;
	struct resource *res;
	u32 magic;
	u32 version;
	int ret;

	ndev = devm_alloc_etherdev(&pdev->dev, sizeof(*hosted));
	if (!ndev)
		return -ENOMEM;
	SET_NETDEV_DEV(ndev, &pdev->dev);
	hosted = netdev_priv(ndev);
	hosted->dev = &pdev->dev;
	hosted->ndev = ndev;
	INIT_WORK(&hosted->hci_open_work, s31_hci_open_work);
	spin_lock_init(&hosted->tx_lock);
	skb_queue_head_init(&hosted->serial_rx);
	mutex_init(&hosted->serial_rx_mutex);
	mutex_init(&hosted->serial_tx_mutex);
	init_waitqueue_head(&hosted->serial_wait);
	atomic_set(&hosted->irq_disabled, 0);
	init_completion(&hosted->pong);
	init_completion(&hosted->selftest_done);
	hosted->rx_frame = devm_kmalloc(&pdev->dev,
					S31_HOSTED_SLOT_DATA_SIZE, GFP_KERNEL);
	if (!hosted->rx_frame)
		return -ENOMEM;
	hosted->serial_reassembly = devm_kmalloc(
		&pdev->dev, S31_HOSTED_SERIAL_MAX_WRITE, GFP_KERNEL);
	if (!hosted->serial_reassembly)
		return -ENOMEM;
	ret = devm_add_action_or_reset(&pdev->dev,
				       s31_hosted_serial_queue_purge,
				       hosted);
	if (ret)
		return ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hosted->shmem = s31_hosted_fixmap_resource(pdev, res);
	if (IS_ERR(hosted->shmem))
		return PTR_ERR(hosted->shmem);

	hosted->db_h0_to_h1 = devm_ioremap(&pdev->dev,
					   S31_HP_SYSTEM_CPU_INT_FROM_CPU_2,
					   sizeof(u32));
	hosted->db_h1_to_h0 = devm_ioremap(&pdev->dev,
					   S31_HP_SYSTEM_CPU_INT_FROM_CPU_3,
					   sizeof(u32));
	if (!hosted->db_h0_to_h1 || !hosted->db_h1_to_h0)
		return -ENOMEM;

	magic = readl(s31_ctrl_ptr(hosted,
				  offsetof(struct s31_hosted_control, magic)));
	version = readl(s31_ctrl_ptr(hosted,
				    offsetof(struct s31_hosted_control,
					     abi_version)));
	if (magic != S31_HOSTED_MAGIC || version != S31_HOSTED_ABI_VERSION)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "hart0 transport unavailable (%08x/%u)\n",
				     magic, version);
	hosted->generation =
		readl(s31_ctrl_ptr(hosted,
				   offsetof(struct s31_hosted_control, generation)));

	ndev->netdev_ops = &s31_hosted_netdev_ops;
	ndev->min_mtu = ETH_MIN_MTU;
	ndev->max_mtu = ETH_DATA_LEN;
	strscpy(ndev->name, "ethsta%d", IFNAMSIZ);
	eth_hw_addr_random(ndev);
	netif_carrier_off(ndev);
	netif_napi_add(ndev, &hosted->napi, s31_hosted_poll);

	hosted->irq = platform_get_irq(pdev, 0);
	if (hosted->irq < 0)
		return hosted->irq;
	napi_enable(&hosted->napi);
	ret = devm_request_irq(&pdev->dev, hosted->irq, s31_hosted_irq, 0,
			       dev_name(&pdev->dev), hosted);
	if (ret) {
		napi_disable(&hosted->napi);
		return dev_err_probe(&pdev->dev, ret,
				     "failed to request doorbell IRQ\n");
	}

	ret = register_netdev(ndev);
	if (ret) {
		napi_disable(&hosted->napi);
		return ret;
	}

	hosted->serial_misc.minor = MISC_DYNAMIC_MINOR;
	hosted->serial_misc.name = "esps0";
	hosted->serial_misc.fops = &s31_serial_fops;
	hosted->serial_misc.parent = &pdev->dev;
	ret = misc_register(&hosted->serial_misc);
	if (ret) {
		unregister_netdev(ndev);
		napi_disable(&hosted->napi);
		return ret;
	}

	ret = s31_hci_register(hosted);
	if (ret) {
		misc_deregister(&hosted->serial_misc);
		unregister_netdev(ndev);
		napi_disable(&hosted->napi);
		return dev_err_probe(&pdev->dev, ret,
				     "failed to register Bluetooth HCI\n");
	}
	if (hosted->hosted_ready)
		schedule_work(&hosted->hci_open_work);

	platform_set_drvdata(pdev, hosted);
	ret = sbi_ecall(S31_SBI_EXT_HOSTED, S31_SBI_HOSTED_H1_READY,
			0, 0, 0, 0, 0, 0).error;
	if (ret)
		dev_warn(&pdev->dev, "OpenSBI Hosted bridge unavailable: %d\n",
			 ret);
	if (readl(s31_ring_ptr(hosted, true,
			       offsetof(struct s31_hosted_ring_state, producer))) !=
	    readl(s31_ring_ptr(hosted, true,
			       offsetof(struct s31_hosted_ring_state, consumer)))) {
		disable_irq(hosted->irq);
		atomic_set(&hosted->irq_disabled, 1);
		napi_schedule(&hosted->napi);
	}
	ret = s31_hosted_selftest(hosted);
	if (ret)
		dev_warn(&pdev->dev, "transport self-test failed: %d\n", ret);
	else
		dev_info(&pdev->dev,
			 "transport self-test passed: %u x %u-byte frames\n",
			 S31_HOSTED_SELFTEST_ROUNDS,
			 S31_HOSTED_SELFTEST_SIZE);

	dev_info(&pdev->dev,
		 "SRAM transport generation %u, netdev %s, RPC /dev/%s, HCI %s, IRQ %d\n",
		 hosted->generation, ndev->name, hosted->serial_misc.name,
		 hosted->hdev->name, hosted->irq);
	return 0;
}

static void s31_hosted_remove(struct platform_device *pdev)
{
	struct s31_hosted *hosted = platform_get_drvdata(pdev);

	cancel_work_sync(&hosted->hci_open_work);
	s31_hci_unregister(hosted);
	misc_deregister(&hosted->serial_misc);
	unregister_netdev(hosted->ndev);
	napi_disable(&hosted->napi);
	netif_napi_del(&hosted->napi);
}

static const struct of_device_id s31_hosted_of_match[] = {
	{ .compatible = "espressif,esp32s31-hosted-sram" },
	{ }
};
MODULE_DEVICE_TABLE(of, s31_hosted_of_match);

static struct platform_driver s31_hosted_driver = {
	.probe = s31_hosted_probe,
	.remove_new = s31_hosted_remove,
	.driver = {
		.name = "esp32s31-hosted-sram",
		.of_match_table = s31_hosted_of_match,
	},
};
module_platform_driver(s31_hosted_driver);

MODULE_DESCRIPTION("ESP32-S31 in-package ESP-Hosted SRAM transport");
MODULE_LICENSE("GPL");
