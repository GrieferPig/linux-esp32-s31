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
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <asm/fixmap.h>
#include <asm/sbi.h>

#include "s31_hosted_sram.h"

#define S31_HP_SYSTEM_CPU_INT_FROM_CPU_2	0x20587018U
#define S31_HP_SYSTEM_CPU_INT_FROM_CPU_3	0x2058701cU
#define S31_HOSTED_NAPI_WEIGHT		16
#define S31_HOSTED_FIXMAP_PAGES		17
#define S31_SBI_EXT_HOSTED		0x09000001UL
#define S31_SBI_HOSTED_TX		0
#define S31_SBI_HOSTED_RX_ACK		1
#define S31_SBI_HOSTED_H1_READY		2

struct s31_hosted {
	struct device *dev;
	struct net_device *ndev;
	struct napi_struct napi;
	void __iomem *shmem;
	void __iomem *db_h0_to_h1;
	void __iomem *db_h1_to_h0;
	int irq;
	spinlock_t tx_lock;
	u8 *rx_frame;
	atomic_t irq_disabled;
	struct completion pong;
	u32 generation;
	u32 rx_errors;
	u32 tx_drops;
};

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

static int s31_send_frame(struct s31_hosted *hosted, const void *frame,
			  size_t length)
{
	struct sbiret sbi_ret;
	unsigned long flags;

	if (!length || length > S31_HOSTED_SLOT_DATA_SIZE)
		return -EMSGSIZE;

	spin_lock_irqsave(&hosted->tx_lock, flags);
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
	header->len = cpu_to_le16(length);
	header->offset = cpu_to_le16(sizeof(*header));
	header->hci_pkt_type = hci_type;
	memcpy(frame + sizeof(*header), payload, length);

	ret = s31_send_frame(hosted, frame, frame_length);
	kfree(frame);
	return ret;
}

static void s31_send_control(struct s31_hosted *hosted, u8 type, u8 value)
{
	struct s31_hosted_control_msg msg = {
		.type = type,
		.value = value,
		.length = cpu_to_le16(sizeof(msg)),
		.generation = cpu_to_le32(hosted->generation),
	};

	s31_send_payload(hosted, S31_HOSTED_PRIV_IF, &msg, sizeof(msg), 0);
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

static void s31_process_private(struct s31_hosted *hosted, const u8 *payload,
				u16 length)
{
	const struct s31_hosted_control_msg *msg;

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

static void s31_process_frame(struct s31_hosted *hosted, const u8 *frame,
			      u16 frame_length)
{
	const struct s31_esp_payload_header *header = (const void *)frame;
	struct sk_buff *skb;
	const u8 *payload;
	u16 offset;
	u16 length;

	if (frame_length < sizeof(*header))
		goto malformed;
	offset = le16_to_cpu(header->offset);
	length = le16_to_cpu(header->len);
	if (offset < sizeof(*header) || offset + length > frame_length)
		goto malformed;
	payload = frame + offset;

	switch (header->if_type) {
	case S31_HOSTED_STA_IF:
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
		s31_process_private(hosted, payload, length);
		break;
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
	spin_lock_init(&hosted->tx_lock);
	atomic_set(&hosted->irq_disabled, 0);
	init_completion(&hosted->pong);
	hosted->rx_frame = devm_kmalloc(&pdev->dev,
					S31_HOSTED_SLOT_DATA_SIZE, GFP_KERNEL);
	if (!hosted->rx_frame)
		return -ENOMEM;

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

	platform_set_drvdata(pdev, hosted);
	ret = sbi_ecall(S31_SBI_EXT_HOSTED, S31_SBI_HOSTED_H1_READY,
			0, 0, 0, 0, 0, 0).error;
	if (ret)
		dev_warn(&pdev->dev, "OpenSBI Hosted bridge unavailable: %d\n",
			 ret);
	s31_send_control(hosted, S31_HOSTED_CTRL_PING, 0);
	if (readl(s31_ring_ptr(hosted, true,
			       offsetof(struct s31_hosted_ring_state, producer))) !=
	    readl(s31_ring_ptr(hosted, true,
			       offsetof(struct s31_hosted_ring_state, consumer)))) {
		disable_irq(hosted->irq);
		atomic_set(&hosted->irq_disabled, 1);
		napi_schedule(&hosted->napi);
	}
	if (!wait_for_completion_timeout(&hosted->pong,
					 msecs_to_jiffies(500))) {
		void __iomem *h0_ring = s31_ring_ptr(hosted, true, 0);
		void __iomem *h1_ring = s31_ring_ptr(hosted, false, 0);

		dev_warn(&pdev->dev,
			 "hart0 ping timeout: tx p=%u c=%u rx p=%u c=%u; h0 irq=%u polls=%u seen_p=%u seq=%u pma=%#x/%#x apm=%#x@%#x worker=%u\n",
			 readl(h0_ring +
			       offsetof(struct s31_hosted_ring_state, producer)),
			 readl(h0_ring +
			       offsetof(struct s31_hosted_ring_state, consumer)),
			 readl(h1_ring +
			       offsetof(struct s31_hosted_ring_state, producer)),
			 readl(h1_ring +
			       offsetof(struct s31_hosted_ring_state, consumer)),
			 readl(s31_ctrl_ptr(hosted,
					    offsetof(struct s31_hosted_control,
						     h0_irq_count))),
			 readl(s31_ctrl_ptr(hosted,
					    offsetof(struct s31_hosted_control,
						     h0_rx_polls))),
			 readl(s31_ctrl_ptr(hosted,
					    offsetof(struct s31_hosted_control,
						     h0_seen_h1_producer))),
			 readl(s31_ctrl_ptr(hosted,
					    offsetof(struct s31_hosted_control,
						     h0_seen_h1_sequence))),
			 readl(s31_ctrl_ptr(hosted,
					    offsetof(struct s31_hosted_control,
						     h0_apm_status))),
			 readl(s31_ctrl_ptr(hosted,
					    offsetof(struct s31_hosted_control,
						     h0_h1_doorbell))),
			 readl(hosted->shmem + 0x408),
			 readl(hosted->shmem + 0x40c),
			 readl(hosted->shmem + 0x418));
	}

	/* Read trampoline test patterns at different SRAM offsets */
	dev_info(&pdev->dev,
		 "XHART test: off256=%08x off512=%08x off1K=%08x off2K=%08x off4K=%08x off60K=%08x\n",
		 readl(hosted->shmem + 256),
		 readl(hosted->shmem + 512),
		 readl(hosted->shmem + 1024),
		 readl(hosted->shmem + 2048),
		 readl(hosted->shmem + 4096),
		 readl(hosted->shmem + 61440));

	dev_info(&pdev->dev,
		 "SRAM transport generation %u, netdev %s, IRQ %d\n",
		 hosted->generation, ndev->name, hosted->irq);
	return 0;
}

static void s31_hosted_remove(struct platform_device *pdev)
{
	struct s31_hosted *hosted = platform_get_drvdata(pdev);

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
