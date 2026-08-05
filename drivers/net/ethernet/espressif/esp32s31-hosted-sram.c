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
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ioctl.h>
#include <linux/ktime.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/poll.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <asm/fixmap.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "s31_hosted_sram.h"

#define S31_HP_SYSTEM_CPU_INT_FROM_CPU_2	0x20586018U
#define S31_HP_SYSTEM_CPU_INT_FROM_CPU_3	0x2058601cU
#define S31_HOSTED_NAPI_WEIGHT		16
#define S31_HOSTED_FIXMAP_PAGES		((S31_HOSTED_SRAM_SIZE + 0x1f7f) >> PAGE_SHIFT)
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
#define S31_HOSTED_IOC_CLOCK_TEST \
	_IOWR('S', 0x31, struct s31_hosted_clock_test)
#define S31_HOSTED_IOC_MEM_STATS \
	_IOR('S', 0x32, struct s31_hosted_mem_stats)
#define S31_HOSTED_IOC_WIFI_SLOT_SET \
	_IOW('S', 0x33, struct s31_hosted_wifi_slot_request)
#define S31_HOSTED_IOC_WIFI_SLOT_GET \
	_IOWR('S', 0x34, struct s31_hosted_wifi_slot_request)
#define S31_HOSTED_IOC_WIFI_STATE_SET \
	_IOW('S', 0x35, struct s31_hosted_wifi_state_request)
#define S31_HOSTED_IOC_WIFI_STATE_GET \
	_IOR('S', 0x36, struct s31_hosted_wifi_state_request)

static struct cpufreq_frequency_table s31_cpu_freq_table[] = {
	{ .frequency = 53000 },
	{ .frequency = 80000 },
	{ .frequency = 160000 },
	{ .frequency = 240000 },
	{ .frequency = 320000 },
	{ .frequency = CPUFREQ_TABLE_END },
};

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
	struct completion clock_start;
	struct completion clock_stop;
	struct completion mem_stats_done;
	struct completion wifi_cfg_done;
	struct completion cpu_freq_done;
	struct miscdevice serial_misc;
	struct sk_buff_head serial_rx;
	u8 *serial_reassembly;
	size_t serial_reassembly_length;
	u16 serial_reassembly_sequence;
	bool serial_reassembly_active;
	wait_queue_head_t serial_wait;
	struct mutex serial_rx_mutex;
	struct mutex serial_tx_mutex;
	struct mutex clock_mutex;
	struct mutex mem_stats_mutex;
	struct mutex wifi_cfg_mutex;
	struct mutex cpu_freq_mutex;
	u32 generation;
	u32 clock_cookie;
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
	struct s31_hosted_clock_test clock_test;
	struct s31_hosted_mem_stats mem_stats;
	struct s31_hosted_wifi_slot wifi_slot;
	struct s31_hosted_wifi_state wifi_state;
	u8 wifi_cfg_response_type;
	u32 wifi_cfg_status;
	u32 cpu_freq_status;
	u32 cpu_freq_mhz;
	u32 cpu_freq_floor_mhz;
};

static struct s31_hosted *s31_cpufreq_hosted;

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

static ssize_t transport_stats_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct s31_hosted *hosted = dev_get_drvdata(dev);

	(void)attr;
	return sysfs_emit(buf, "h0_irq_count=%u h0_rx_polls=%u\n",
			readl(s31_ctrl_ptr(hosted, offsetof(
				struct s31_hosted_control, h0_irq_count))),
			readl(s31_ctrl_ptr(hosted, offsetof(
				struct s31_hosted_control, h0_rx_polls))));
}

static DEVICE_ATTR_RO(transport_stats);

static struct attribute *s31_hosted_attrs[] = {
	&dev_attr_transport_stats.attr,
	NULL,
};

static const struct attribute_group s31_hosted_attr_group = {
	.attrs = s31_hosted_attrs,
};

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
	void __iomem *ring;
	void __iomem *slot;
	u32 producer;
	u32 consumer;
	u32 index;
	unsigned long flags;

	if (length < sizeof(*header) || length > S31_HOSTED_SLOT_DATA_SIZE)
		return -EMSGSIZE;

	spin_lock_irqsave(&hosted->tx_lock, flags);
	if (!header->seq_num)
		header->seq_num = cpu_to_le16(++hosted->tx_sequence);
	header->checksum = 0;
	header->checksum = cpu_to_le16(s31_frame_checksum(frame, length));

	/* Linux owns the producer side of the H1->H0 ring. */
	ring = s31_ring_ptr(hosted, false, 0);
	producer = readl(ring + offsetof(struct s31_hosted_ring_state,
					producer));
	consumer = readl(ring + offsetof(struct s31_hosted_ring_state,
					consumer));
	if (producer - consumer >= S31_HOSTED_SLOT_COUNT) {
		hosted->tx_drops++;
		spin_unlock_irqrestore(&hosted->tx_lock, flags);
		return -ENOSPC;
	}

	index = producer & (S31_HOSTED_SLOT_COUNT - 1);
	slot = s31_slot_ptr(hosted, false, index);
	memcpy_toio(slot + offsetof(struct s31_hosted_slot, data),
			frame, length);
	writew(length, slot + offsetof(struct s31_hosted_slot, length));
	writeb(0, slot + offsetof(struct s31_hosted_slot, flags));
	writeb(0, slot + offsetof(struct s31_hosted_slot, reserved));
	wmb();
	/* Publish sequence last, then commit the ring producer. */
	writel(producer + 1,
	       slot + offsetof(struct s31_hosted_slot, sequence));
	wmb();
	writel(producer + 1,
	       ring + offsetof(struct s31_hosted_ring_state, producer));
	wmb();
	writel(1, hosted->db_h1_to_h0);
	spin_unlock_irqrestore(&hosted->tx_lock, flags);
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
	if (msg->type == S31_HOSTED_CTRL_WIFI_SLOT_SET_RESPONSE ||
	    msg->type == S31_HOSTED_CTRL_WIFI_SLOT_GET_RESPONSE ||
	    msg->type == S31_HOSTED_CTRL_WIFI_STATE_SET_RESPONSE ||
	    msg->type == S31_HOSTED_CTRL_WIFI_STATE_GET_RESPONSE) {
		const struct s31_hosted_wifi_msg *wifi_msg = (const void *)payload;

		hosted->wifi_cfg_status = le32_to_cpu(wifi_msg->status);
		if ((msg->type == S31_HOSTED_CTRL_WIFI_SLOT_SET_RESPONSE ||
		     msg->type == S31_HOSTED_CTRL_WIFI_SLOT_GET_RESPONSE) &&
		    le16_to_cpu(wifi_msg->length) == sizeof(hosted->wifi_slot))
			memcpy(&hosted->wifi_slot, wifi_msg->data,
			       sizeof(hosted->wifi_slot));
		else if ((msg->type == S31_HOSTED_CTRL_WIFI_STATE_SET_RESPONSE ||
			  msg->type == S31_HOSTED_CTRL_WIFI_STATE_GET_RESPONSE) &&
			 le16_to_cpu(wifi_msg->length) == sizeof(hosted->wifi_state))
			memcpy(&hosted->wifi_state, wifi_msg->data,
			       sizeof(hosted->wifi_state));
		if (msg->type == hosted->wifi_cfg_response_type)
			complete(&hosted->wifi_cfg_done);
		return;
	}
	if (msg->type == S31_HOSTED_CTRL_CPU_FREQ_SET_RESPONSE) {
		const struct s31_hosted_cpu_freq_msg *freq = (const void *)msg->data;

		hosted->cpu_freq_status = le32_to_cpu(freq->status);
		if (!hosted->cpu_freq_status) {
			hosted->cpu_freq_mhz = le32_to_cpu(freq->actual_mhz);
			hosted->cpu_freq_floor_mhz = le32_to_cpu(freq->target_mhz);
		}
		complete(&hosted->cpu_freq_done);
		return;
	}

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
	case S31_HOSTED_CTRL_CLOCK_START:
	case S31_HOSTED_CTRL_CLOCK_STOP: {
		const struct s31_hosted_clock_stamp *stamp;

		stamp = (const void *)msg->data;
		if (le32_to_cpu(stamp->cookie) != hosted->clock_test.cookie)
			break;
		if (msg->type == S31_HOSTED_CTRL_CLOCK_START) {
			hosted->clock_test.freertos_start_us =
				get_unaligned_le64(&stamp->freertos_us);
			hosted->clock_test.linux_start_ns = ktime_get_raw_ns();
			complete(&hosted->clock_start);
		} else {
			hosted->clock_test.freertos_end_us =
				get_unaligned_le64(&stamp->freertos_us);
			hosted->clock_test.linux_end_ns = ktime_get_raw_ns();
			complete(&hosted->clock_stop);
		}
		break;
	}
	case S31_HOSTED_CTRL_MEM_STATS_RESPONSE: {
		const struct s31_hosted_mem_stats *stats = (const void *)msg->data;

		hosted->mem_stats.total_bytes =
			get_unaligned_le32(&stats->total_bytes);
		hosted->mem_stats.free_bytes =
			get_unaligned_le32(&stats->free_bytes);
		hosted->mem_stats.minimum_free_bytes =
			get_unaligned_le32(&stats->minimum_free_bytes);
		hosted->mem_stats.largest_free_block =
			get_unaligned_le32(&stats->largest_free_block);
		complete(&hosted->mem_stats_done);
		break;
	}
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

static int s31_wifi_cfg_exchange(struct s31_hosted *hosted,
				 struct s31_hosted_wifi_msg *message,
				 u8 response_type)
{
	unsigned long timeout;
	int ret;

	ret = mutex_lock_interruptible(&hosted->wifi_cfg_mutex);
	if (ret)
		return ret;
	hosted->wifi_cfg_response_type = response_type;
	hosted->wifi_cfg_status = 0;
	reinit_completion(&hosted->wifi_cfg_done);
	ret = s31_send_payload(hosted, S31_HOSTED_PRIV_IF, message,
				       sizeof(*message), 0);
	if (!ret) {
		timeout = wait_for_completion_interruptible_timeout(
			&hosted->wifi_cfg_done, msecs_to_jiffies(3000));
		if (!timeout)
			ret = -ETIMEDOUT;
		else if ((long)timeout < 0)
			ret = (long)timeout;
		else if (hosted->wifi_cfg_status)
			ret = -EREMOTEIO;
	}
	mutex_unlock(&hosted->wifi_cfg_mutex);
	return ret;
}

static int s31_hosted_set_cpu_freq(struct s31_hosted *hosted,
				   unsigned int khz)
{
	struct s31_hosted_control_msg message = {
		.type = S31_HOSTED_CTRL_CPU_FREQ_SET,
		.length = sizeof(message),
		.generation = hosted->generation,
	};
	struct s31_hosted_cpu_freq_msg request = {
		.target_mhz = khz / 1000,
	};
	unsigned long timeout;
	int ret;

	if (khz % 1000 || !hosted->hosted_ready)
		return -EINVAL;
	memcpy(message.data, &request, sizeof(request));
	ret = mutex_lock_interruptible(&hosted->cpu_freq_mutex);
	if (ret)
		return ret;
	/* Governors may request the same floor on every sampling tick.  The
	 * transition is a remote RPC and needlessly reprogramming the PM policy
	 * stalls the Linux CPU even when FreeRTOS is currently running faster. */
	if (hosted->cpu_freq_floor_mhz == request.target_mhz) {
		mutex_unlock(&hosted->cpu_freq_mutex);
		return 0;
	}
	reinit_completion(&hosted->cpu_freq_done);
	hosted->cpu_freq_status = ~0U;
	ret = s31_send_payload(hosted, S31_HOSTED_PRIV_IF, &message,
				       sizeof(message), 0);
	if (!ret) {
		timeout = wait_for_completion_interruptible_timeout(
			&hosted->cpu_freq_done, msecs_to_jiffies(3000));
		if (!timeout)
			ret = -ETIMEDOUT;
		else if ((long)timeout < 0)
			ret = (long)timeout;
		else if (hosted->cpu_freq_status)
			ret = -EREMOTEIO;
	}
	mutex_unlock(&hosted->cpu_freq_mutex);
	return ret;
}

static int s31_cpufreq_init(struct cpufreq_policy *policy)
{
	int ret;

	if (policy->cpu != 0 || !s31_cpufreq_hosted)
		return -ENODEV;
	policy->freq_table = s31_cpu_freq_table;
	ret = cpufreq_table_validate_and_sort(policy);
	if (ret)
		return ret;
	/* Frequency changes cross the H0/H1 RPC boundary.  Report a coarse
	 * transition interval so ondemand does not poll every scheduler tick. */
	policy->cpuinfo.transition_latency = 100000000;
	policy->cur = s31_cpufreq_hosted->cpu_freq_mhz * 1000;
	return 0;
}

static int s31_cpufreq_target_index(struct cpufreq_policy *policy,
					unsigned int index)
{
	unsigned int target = s31_cpu_freq_table[index].frequency;
	int ret;

	ret = s31_hosted_set_cpu_freq(s31_cpufreq_hosted, target);
	if (!ret)
		policy->cur = s31_cpufreq_hosted->cpu_freq_mhz * 1000;
	return ret;
}

static unsigned int s31_cpufreq_get(unsigned int cpu)
{
	if (cpu != 0 || !s31_cpufreq_hosted)
		return 0;
	return s31_cpufreq_hosted->cpu_freq_mhz * 1000;
}

static struct cpufreq_driver s31_cpufreq_driver = {
	.flags = CPUFREQ_NEED_INITIAL_FREQ_CHECK | CPUFREQ_CONST_LOOPS,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = s31_cpufreq_target_index,
	.get = s31_cpufreq_get,
	.init = s31_cpufreq_init,
	.attr = cpufreq_generic_attr,
	.name = "esp32s31-idf",
};

static long s31_serial_ioctl(struct file *file, unsigned int command,
			     unsigned long argument)
{
	struct miscdevice *misc = file->private_data;
	struct s31_hosted *hosted =
		container_of(misc, struct s31_hosted, serial_misc);
	struct s31_hosted_control_msg message = {
		.type = S31_HOSTED_CTRL_CLOCK_START,
		.length = sizeof(message),
		.generation = hosted->generation,
	};
	struct s31_hosted_clock_stamp stamp = { 0 };
	struct s31_hosted_clock_test request;
	struct s31_hosted_wifi_msg wifi_message = { 0 };
	struct s31_hosted_wifi_slot_request slot_request;
	struct s31_hosted_wifi_state_request state_request;
	unsigned long timeout;
	int ret;

	if (command == S31_HOSTED_IOC_MEM_STATS) {
		message.type = S31_HOSTED_CTRL_MEM_STATS_REQUEST;
		ret = mutex_lock_interruptible(&hosted->mem_stats_mutex);
		if (ret)
			return ret;
		reinit_completion(&hosted->mem_stats_done);
		ret = s31_send_payload(hosted, S31_HOSTED_PRIV_IF, &message,
				       sizeof(message), 0);
		if (ret)
			goto out_mem_stats;
		timeout = wait_for_completion_interruptible_timeout(
			&hosted->mem_stats_done, msecs_to_jiffies(2000));
		if (!timeout)
			ret = -ETIMEDOUT;
		else if ((long)timeout < 0)
			ret = (long)timeout;
		else
			ret = copy_to_user((void __user *)argument,
					   &hosted->mem_stats,
					   sizeof(hosted->mem_stats)) ?
				-EFAULT : 0;
out_mem_stats:
		mutex_unlock(&hosted->mem_stats_mutex);
		return ret;
	}

	if (command == S31_HOSTED_IOC_WIFI_SLOT_SET ||
	    command == S31_HOSTED_IOC_WIFI_SLOT_GET) {
		if (copy_from_user(&slot_request, (void __user *)argument,
				   sizeof(slot_request)))
			return -EFAULT;
		if (slot_request.slot >= S31_HOSTED_WIFI_SLOT_COUNT)
			return -EINVAL;
		wifi_message.type = command == S31_HOSTED_IOC_WIFI_SLOT_SET ?
			S31_HOSTED_CTRL_WIFI_SLOT_SET : S31_HOSTED_CTRL_WIFI_SLOT_GET;
		wifi_message.slot = slot_request.slot;
		wifi_message.length = cpu_to_le16(sizeof(slot_request.config));
		wifi_message.generation = cpu_to_le32(hosted->generation);
		if (command == S31_HOSTED_IOC_WIFI_SLOT_SET)
			memcpy(wifi_message.data, &slot_request.config,
			       sizeof(slot_request.config));
		ret = s31_wifi_cfg_exchange(hosted, &wifi_message,
			command == S31_HOSTED_IOC_WIFI_SLOT_SET ?
			S31_HOSTED_CTRL_WIFI_SLOT_SET_RESPONSE :
			S31_HOSTED_CTRL_WIFI_SLOT_GET_RESPONSE);
		if (!ret && command == S31_HOSTED_IOC_WIFI_SLOT_GET) {
			slot_request.config = hosted->wifi_slot;
			if (copy_to_user((void __user *)argument, &slot_request,
					 sizeof(slot_request)))
				ret = -EFAULT;
		}
		return ret;
	}

	if (command == S31_HOSTED_IOC_WIFI_STATE_SET ||
	    command == S31_HOSTED_IOC_WIFI_STATE_GET) {
		if (command == S31_HOSTED_IOC_WIFI_STATE_SET &&
		    copy_from_user(&state_request, (void __user *)argument,
				   sizeof(state_request)))
			return -EFAULT;
		wifi_message.type = command == S31_HOSTED_IOC_WIFI_STATE_SET ?
			S31_HOSTED_CTRL_WIFI_STATE_SET : S31_HOSTED_CTRL_WIFI_STATE_GET;
		wifi_message.length = cpu_to_le16(sizeof(state_request.state));
		wifi_message.generation = cpu_to_le32(hosted->generation);
		if (command == S31_HOSTED_IOC_WIFI_STATE_SET)
			memcpy(wifi_message.data, &state_request.state,
			       sizeof(state_request.state));
		ret = s31_wifi_cfg_exchange(hosted, &wifi_message,
			command == S31_HOSTED_IOC_WIFI_STATE_SET ?
			S31_HOSTED_CTRL_WIFI_STATE_SET_RESPONSE :
			S31_HOSTED_CTRL_WIFI_STATE_GET_RESPONSE);
		if (!ret) {
			state_request.state = hosted->wifi_state;
			if (copy_to_user((void __user *)argument, &state_request,
					 sizeof(state_request)))
				ret = -EFAULT;
		}
		return ret;
	}

	if (command != S31_HOSTED_IOC_CLOCK_TEST)
		return -ENOTTY;
	if (copy_from_user(&request, (void __user *)argument, sizeof(request)))
		return -EFAULT;
	if (!request.duration_sec || request.duration_sec > 600)
		return -EINVAL;

	ret = mutex_lock_interruptible(&hosted->clock_mutex);
	if (ret)
		return ret;
	memset(&hosted->clock_test, 0, sizeof(hosted->clock_test));
	hosted->clock_test.duration_sec = request.duration_sec;
	hosted->clock_test.cookie = ++hosted->clock_cookie;
	if (!hosted->clock_test.cookie)
		hosted->clock_test.cookie = ++hosted->clock_cookie;
	stamp.cookie = cpu_to_le32(hosted->clock_test.cookie);
	stamp.duration_sec = cpu_to_le32(request.duration_sec);
	memcpy(message.data, &stamp, sizeof(stamp));
	reinit_completion(&hosted->clock_start);
	reinit_completion(&hosted->clock_stop);
	ret = s31_send_payload(hosted, S31_HOSTED_PRIV_IF, &message,
			       sizeof(message), 0);
	if (ret)
		goto out;
	timeout = wait_for_completion_interruptible_timeout(
		&hosted->clock_start, msecs_to_jiffies(2000));
	if (!timeout) {
		ret = -ETIMEDOUT;
		goto out;
	}
	if ((long)timeout < 0) {
		ret = (long)timeout;
		goto out;
	}
	timeout = wait_for_completion_interruptible_timeout(
		&hosted->clock_stop,
		msecs_to_jiffies((request.duration_sec + 5) * 1000));
	if (!timeout) {
		ret = -ETIMEDOUT;
		goto out;
	}
	if ((long)timeout < 0) {
		ret = (long)timeout;
		goto out;
	}
	ret = copy_to_user((void __user *)argument, &hosted->clock_test,
			   sizeof(hosted->clock_test)) ? -EFAULT : 0;
out:
	mutex_unlock(&hosted->clock_mutex);
	return ret;
}

static const struct file_operations s31_serial_fops = {
	.owner = THIS_MODULE,
	.read = s31_serial_read,
	.write = s31_serial_write,
	.poll = s31_serial_poll,
	.unlocked_ioctl = s31_serial_ioctl,
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
			writel(producer,
			       ring + offsetof(struct s31_hosted_ring_state,
					       consumer));
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
		writel(consumer + 1,
		       ring + offsetof(struct s31_hosted_ring_state, consumer));
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
	mutex_init(&hosted->clock_mutex);
	mutex_init(&hosted->mem_stats_mutex);
	mutex_init(&hosted->wifi_cfg_mutex);
	mutex_init(&hosted->cpu_freq_mutex);
	init_waitqueue_head(&hosted->serial_wait);
	atomic_set(&hosted->irq_disabled, 0);
	init_completion(&hosted->pong);
	init_completion(&hosted->selftest_done);
	init_completion(&hosted->clock_start);
	init_completion(&hosted->clock_stop);
	init_completion(&hosted->mem_stats_done);
	init_completion(&hosted->wifi_cfg_done);
	init_completion(&hosted->cpu_freq_done);
	hosted->cpu_freq_mhz = 320;
	hosted->cpu_freq_floor_mhz = 320;
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
	ret = devm_device_add_group(&pdev->dev, &s31_hosted_attr_group);
	if (ret)
		dev_warn(&pdev->dev, "failed to expose transport stats: %d\n", ret);
	s31_cpufreq_hosted = hosted;
	ret = cpufreq_register_driver(&s31_cpufreq_driver);
	if (ret)
		dev_warn(&pdev->dev, "CPU frequency scaling unavailable: %d\n", ret);
	/* Publish H1 readiness directly in the shared control block. */
	writel(readl(s31_ctrl_ptr(hosted,
				  offsetof(struct s31_hosted_control, state))) |
		       S31_HOSTED_H1_READY,
	       s31_ctrl_ptr(hosted,
			     offsetof(struct s31_hosted_control, state)));
	wmb();
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
	else {
		dev_info(&pdev->dev,
			 "transport self-test passed: %u x %u-byte frames; "
			 "h0 doorbell irq=%u rx-wake=%u\n",
			 S31_HOSTED_SELFTEST_ROUNDS,
			 S31_HOSTED_SELFTEST_SIZE,
			 readl(s31_ctrl_ptr(hosted, offsetof(
				 struct s31_hosted_control, h0_irq_count))),
			 readl(s31_ctrl_ptr(hosted, offsetof(
				 struct s31_hosted_control, h0_rx_polls))));
	}

	dev_info(&pdev->dev,
		 "SRAM transport generation %u, netdev %s, RPC /dev/%s, HCI %s, IRQ %d\n",
		 hosted->generation, ndev->name, hosted->serial_misc.name,
		 hosted->hdev->name, hosted->irq);
	return 0;
}

static void s31_hosted_remove(struct platform_device *pdev)
{
	struct s31_hosted *hosted = platform_get_drvdata(pdev);

	if (s31_cpufreq_hosted == hosted) {
		cpufreq_unregister_driver(&s31_cpufreq_driver);
		s31_cpufreq_hosted = NULL;
	}

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
