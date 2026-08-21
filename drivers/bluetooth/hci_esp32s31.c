// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 Bluetooth controller HCI front end. */

#include <linux/delay.h>
#include <linux/esp32s31-radio.h>
#include <linux/init.h>
#include <linux/printk.h>
#include <linux/skbuff.h>
#include <linux/workqueue.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

struct s31_hci {
	struct hci_dev *hdev;
	struct delayed_work register_work;
};

static struct s31_hci s31_hci;

static int s31_hci_open(struct hci_dev *hdev)
{
	int ret;

	/* Powering on hci0 enables the integrated controller via the serialized
	 * radio core. */
	ret = esp32s31_radio_bt_enable();
	if (ret && ret != -EALREADY)
		return ret;
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

static int s31_hci_send_frame(struct hci_dev *hdev, struct sk_buff *skb)
{
	u8 type = hci_skb_pkt_type(skb);
	int ret;

	ret = esp32s31_radio_hci_send(type, skb->data, skb->len);
	if (ret) {
		hdev->stat.err_tx++;
		return ret;
	}
	switch (type) {
	case HCI_COMMAND_PKT:
		hdev->stat.cmd_tx++;
		break;
	case HCI_ACLDATA_PKT:
		hdev->stat.acl_tx++;
		break;
	case HCI_SCODATA_PKT:
		hdev->stat.sco_tx++;
		break;
	}
	hdev->stat.byte_tx += skb->len;
	kfree_skb(skb);
	return 0;
}

static void s31_hci_receive(void *context, const u8 *frame, size_t length)
{
	struct s31_hci *radio = context;
	struct sk_buff *skb;

	if (!radio->hdev || length < 2)
		return;
	/* The BTDM controller emits an unsolicited NOP command-complete
	 * (04 0e 03 01 00 00) before the first host command is sent.  If we
	 * forward it, the HCI core sees opcode 0 while a real command is
	 * pending and logs "unexpected event for opcode 0x0000".  It is a
	 * boot quirk, not a transport error; drop only that exact frame.
	 */
	if (length == 6 && frame[0] == 0x04 && frame[1] == 0x0e &&
	    frame[2] == 0x03 && frame[3] == 0x01 &&
	    frame[4] == 0x00 && frame[5] == 0x00) {
		pr_info_once("esp32s31-hci: dropped unsolicited initial NOP complete\n");
		return;
	}
	skb = bt_skb_alloc(length - 1, GFP_KERNEL);
	if (!skb) {
		radio->hdev->stat.err_rx++;
		return;
	}
	hci_skb_pkt_type(skb) = frame[0];
	skb_put_data(skb, frame + 1, length - 1);
	radio->hdev->stat.byte_rx += length - 1;
	if (hci_recv_frame(radio->hdev, skb) < 0)
		radio->hdev->stat.err_rx++;
}

static const struct esp32s31_radio_hci_ops s31_hci_ops = {
	.receive = s31_hci_receive,
};

static void s31_hci_register_work(struct work_struct *work)
{
	struct s31_hci *radio = container_of(to_delayed_work(work),
					       struct s31_hci, register_work);
	struct hci_dev *hdev;
	int ret;

	ret = esp32s31_radio_hci_register(&s31_hci_ops, radio);
	if (ret == -EAGAIN) {
		schedule_delayed_work(&radio->register_work, msecs_to_jiffies(100));
		return;
	}
	if (ret) {
		pr_err("esp32s31-hci: radio registration failed: %d\n", ret);
		return;
	}

	hdev = hci_alloc_dev();
	if (!hdev) {
		esp32s31_radio_hci_unregister(&s31_hci_ops, radio);
		return;
	}
	radio->hdev = hdev;
	hdev->bus = HCI_VIRTUAL;
	hci_set_drvdata(hdev, radio);
	hdev->open = s31_hci_open;
	hdev->close = s31_hci_close;
	hdev->flush = s31_hci_flush;
	hdev->send = s31_hci_send_frame;

	ret = hci_register_dev(hdev);
	if (ret) {
		pr_err("esp32s31-hci: cannot register HCI device: %d\n", ret);
		radio->hdev = NULL;
		hci_free_dev(hdev);
		esp32s31_radio_hci_unregister(&s31_hci_ops, radio);
		return;
	}
	pr_info("esp32s31-hci: registered hci%d over serialized VHCI\n", hdev->id);
}

static int __init s31_hci_init(void)
{
	if (esp32s31_radio_is_disabled()) {
		pr_info("esp32s31-hci: disabled with radio core\n");
		return 0;
	}

	INIT_DELAYED_WORK(&s31_hci.register_work, s31_hci_register_work);
	schedule_delayed_work(&s31_hci.register_work, 0);
	return 0;
}
late_initcall(s31_hci_init);
