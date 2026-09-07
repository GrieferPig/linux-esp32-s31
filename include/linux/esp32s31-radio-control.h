/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#ifndef ESP32S31_RADIO_CONTROL_H
#define ESP32S31_RADIO_CONTROL_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

/* Fixed-width payload ABI.  No pointers or compiler-sized enums cross it. */
enum s31_wifi_control_op {
	S31_WIFI_AP_START = 1,
	S31_WIFI_AP_STOP,
	S31_WIFI_MONITOR_START,
	S31_WIFI_MONITOR_STOP,
	S31_WIFI_SET_CHANNEL,
	S31_WIFI_EAP_WRITE,
	S31_WIFI_EAP_COMMIT,
	S31_WIFI_EAP_CLEAR,
	S31_WIFI_AP_DEAUTH,
};

enum s31_wifi_eap_field {
	S31_EAP_IDENTITY,
	S31_EAP_USERNAME,
	S31_EAP_PASSWORD,
	S31_EAP_CA,
	S31_EAP_DOMAIN,
	S31_EAP_CERT,
	S31_EAP_KEY,
	S31_EAP_FIELDS,
};

#define S31_WIFI_VENDOR_ID 0x18fe34U
#define S31_WIFI_VENDOR_EAP 1U
#define S31_EAP_CHUNK 512U
#define S31_EAP_MAX_FIELD 4095U

struct s31_wifi_control {
	uint32_t operation;
	uint8_t ssid[32];
	uint8_t ssid_length;
	uint8_t channel;
	uint8_t hidden;
	uint8_t max_connections;
	uint8_t password[64];
	uint8_t password_length;
	uint8_t has_psk;
	uint8_t mac[6];
	uint8_t psk[32];
	uint16_t beacon_interval;
	uint8_t dtim_period;
	uint8_t reserved;
	uint32_t field;
	uint32_t offset;
	uint32_t total;
	uint32_t length;
	uint8_t data[S31_EAP_CHUNK];
};

#define S31_WIFI_IF_STA 0U
#define S31_WIFI_IF_AP 1U
#define S31_WIFI_IF_MONITOR 2U

#endif
