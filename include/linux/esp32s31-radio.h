/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_ESP32S31_RADIO_H
#define _LINUX_ESP32S31_RADIO_H

#include <linux/types.h>
#include <linux/esp32s31-radio-control.h>

struct device;
struct sk_buff;

/* The SoftMAC association toggles the CPU0 IRQ-latency idle policy. */
void esp32s31_radio_idle_poll_set(bool active);
/* Internal frontend hint; publish while holding the RX ring lock. */

#define ESP32S31_RADIO_CORE_ABI_VERSION	1U

enum esp32s31_radio_state {
	ESP32S31_RADIO_OFFLINE,
	ESP32S31_RADIO_STARTING,
	ESP32S31_RADIO_READY,
	ESP32S31_RADIO_FAILED,
};

/* A stable, blob-independent snapshot for Linux radio front ends. */

struct esp32s31_radio_hci_ops {
	void (*receive)(void *context, const u8 *frame, size_t length);
	void (*rx_ready)(void *context);
	/* Owner callback after SRAM/foreign context is restored to Linux. */
	void (*tx_ready)(void *context);
};

#define ESP32S31_RADIO_HCI_FRAME_MAX	1029

#define ESP32S31_RADIO_WIFI_MAX_APS	32

struct esp32s31_radio_wifi_ap {
	u8 bssid[6];
	u8 ssid[32];
	u8 ssid_length;
	u8 channel;
	s8 signal;
	u8 authmode;
};

struct esp32s31_radio_wifi_connect_params {
	u8 ssid[32];
	u8 ssid_length;
	u8 bssid[6];
	u8 channel;
	u8 psk[32];
	u8 password[64];
	u8 password_length;
	bool has_bssid;
	bool has_psk;
	bool has_password;
	bool enterprise;
	u8 akm;
	u8 mfp;
	u8 sae_pwe;
};

struct esp32s31_radio_wifi_ops {
	void (*scan_complete)(void *context, int status,
			      const struct esp32s31_radio_wifi_ap *aps,
			      size_t count);
	/* Firmware-managed association notifications only. SoftMAC clients
	 * leave these NULL: mac80211 owns the cfg80211 connection state. */
	void (*connected)(void *context, int status, const u8 *bssid,
			  u8 channel);
	void (*disconnected)(void *context, u16 reason);
	/* Copy a closed-driver-owned frame into preallocated Linux ownership.
	 * This may run in hardirq context and therefore must not allocate or
	 * sleep.  rx_ready() schedules the NAPI consumer after a successful copy. */
	int (*rx_copy)(void *context, const u8 *frame, size_t length);
	void (*rx_ready)(void *context);
	void (*tx_wakeup)(void *context);
	void (*receive_aux)(void *context, u8 interface, const u8 *frame,
			    size_t length, u8 channel, s8 signal);
	void (*ap_station)(void *context, const u8 *mac, bool joined);
	/* Borrowed management body (no 802.11 header/FCS), consumed before
	 * callback return. Used only during FullMAC scan/connection setup. */
	void (*mgmt)(void *context, const u8 *bssid, u8 subtype,
		     const u8 *body, size_t length, u8 channel, s8 signal);
};

#define ESP32S31_RADIO_WIFI_FRAME_MAX	1600

bool esp32s31_radio_is_disabled(void);
void s31_linux_pmu_radio_vote(bool active);

/* Keep Linux frontends on typed operations rather than raw payload calls. */
int esp32s31_radio_hci_register(const struct esp32s31_radio_hci_ops *ops,
				void *context);
void esp32s31_radio_hci_unregister(const struct esp32s31_radio_hci_ops *ops,
				   void *context);
int esp32s31_radio_hci_send(u8 packet_type, const u8 *data, size_t length);
/* 0xff followed by up to eight LE16-length-prefixed H4 frames. */
int esp32s31_radio_hci_send_batch(const u8 *data, size_t length);
bool esp32s31_radio_hci_tx_has_space(void);
bool esp32s31_radio_hci_rx_pending(void);
int esp32s31_radio_hci_dequeue(u8 *frame, size_t capacity);
int esp32s31_radio_hci_peek(const u8 **frame, size_t *length);
void esp32s31_radio_hci_consume(void);
void esp32s31_radio_hci_purge(void);
/* Close RX admission only when retiring the controller generation. */
void s31_radio_hci_rx_quiesce(void);
int esp32s31_radio_wifi_register(const struct esp32s31_radio_wifi_ops *ops,
				 void *context);
void esp32s31_radio_wifi_unregister(const struct esp32s31_radio_wifi_ops *ops,
				    void *context);
int esp32s31_radio_wifi_get_mac(u8 mac[6]);
int esp32s31_radio_wifi_scan(const struct s31_wifi_scan_params *params);
int esp32s31_radio_wifi_connect(
		const struct esp32s31_radio_wifi_connect_params *params);
int esp32s31_radio_wifi_disconnect(u16 reason);
int esp32s31_radio_wifi_send(const u8 *frame, size_t length);
int esp32s31_radio_wifi_send_interface(u8 interface, const u8 *frame, size_t length);
/* Publishes bounded metadata and retains a source skb reference. CPU0 fills
 * the final vendor buffer directly from skb fragments before PP submission.
 * Source reference is released after submission/cancellation, separately from
 * mac80211 completion ownership. Retryable resource failure retains it. */
int esp32s31_radio_softmac_send_skb(const struct s31_softmac_tx_header *header,
                                 const struct sk_buff *skb);
/* Required after publishing one or more SoftMAC source leases. */
void esp32s31_radio_softmac_tx_kick(void);
/* Retain original skb while cancellation reports a pending source lease. */
bool esp32s31_radio_softmac_tx_cancel(u32 cookie, u32 epoch);
/* Sleepable cold drain; callers close/join submitters before calling. */
void esp32s31_radio_softmac_tx_quiesce(u32 epoch);
int esp32s31_radio_wifi_control(const struct s31_wifi_control *control);
bool esp32s31_radio_wifi_tx_has_space(void);
bool esp32s31_radio_wifi_rx_pending(void);
void esp32s31_radio_wifi_rx_complete(void);
/* Return one Ethernet-frame length, -ENODATA when the ring is empty, or a
 * negative error.  A NULL frame deliberately consumes and discards one slot,
 * which lets an allocation-starved NAPI poll keep the hardware-facing ring
 * moving. */
int esp32s31_radio_wifi_rx_dequeue(u8 *frame, size_t capacity);
int esp32s31_radio_bt_enable(void);
int esp32s31_radio_bt_disable(void);
/* Integrated frontend lifecycle, owned by the common platform module. */
int s31_radio_wifi_frontend_init(struct device *parent);
void s31_radio_wifi_frontend_exit(void);
int s31_radio_wifi_frontend_suspend(void);
int s31_radio_wifi_frontend_resume(void);
int s31_radio_btdm_frontend_init(struct device *parent, bool direct_hci);
void s31_radio_btdm_frontend_exit(void);
int s31_radio_btdm_frontend_suspend(void);
int s31_radio_btdm_frontend_resume(void);

#endif /* _LINUX_ESP32S31_RADIO_H */
