/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_ESP32S31_RADIO_H
#define _LINUX_ESP32S31_RADIO_H

#include <linux/types.h>

enum esp32s31_radio_state {
	ESP32S31_RADIO_OFFLINE,
	ESP32S31_RADIO_STARTING,
	ESP32S31_RADIO_READY,
	ESP32S31_RADIO_FAILED,
};

/* A stable, blob-independent snapshot for Linux radio front ends. */
struct esp32s31_radio_health {
	enum esp32s31_radio_state state;
	s32 wifi_init_result;
	s32 bt_init_result;
	s32 bt_enable_result;
	u32 tick_irqs;
	u32 worker_passes;
	u32 commands_completed;
	u32 heap_used;
	u32 heap_peak;
	u32 heap_total;
};

struct esp32s31_radio_hci_ops {
	void (*receive)(void *context, const u8 *frame, size_t length);
};

#define ESP32S31_RADIO_WIFI_MAX_APS	32

struct esp32s31_radio_wifi_ap {
	u8 bssid[6];
	u8 ssid[32];
	u8 ssid_length;
	u8 channel;
	s8 signal;
	u8 authmode;
};

struct esp32s31_radio_wifi_ops {
	void (*scan_complete)(void *context, int status,
			      const struct esp32s31_radio_wifi_ap *aps,
			      size_t count);
};

/*
 * This is intentionally the only public operation in the core's first
 * revision. Bluetooth HCI and cfg80211 add typed entry points here rather
 * than exposing raw ESP-IDF symbols or an arbitrary function-call gateway.
 */
int esp32s31_radio_get_health(struct esp32s31_radio_health *health);
int esp32s31_radio_hci_register(const struct esp32s31_radio_hci_ops *ops,
				void *context);
void esp32s31_radio_hci_unregister(const struct esp32s31_radio_hci_ops *ops,
				   void *context);
int esp32s31_radio_hci_send(u8 packet_type, const u8 *data, size_t length);
int esp32s31_radio_wifi_register(const struct esp32s31_radio_wifi_ops *ops,
				 void *context);
int esp32s31_radio_wifi_scan(void);

#endif /* _LINUX_ESP32S31_RADIO_H */
