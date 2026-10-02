/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#ifndef ESP32S31_RADIO_CONTROL_H
/* Fixed Wi-Fi-only native RX layout, passed in radio task features. */
#define S31_RADIO_FEATURE_RX_LEAN16 (1U << 9)
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
	S31_WIFI_SOFTMAC_START,
	S31_WIFI_SOFTMAC_STOP,
	S31_WIFI_SOFTMAC_RX_BA_ADD,
	S31_WIFI_SOFTMAC_RX_BA_DEL,
	S31_WIFI_SOFTMAC_EDCA,
	S31_WIFI_SOFTMAC_RX_KEY_ADD,
	S31_WIFI_SOFTMAC_RX_KEY_DEL,
	S31_WIFI_SOFTMAC_HE_BSS,
	S31_WIFI_SOFTMAC_BSSID_FILTER,
	S31_WIFI_SOFTMAC_TX_HT_CAP, /* Success advertises raw HT20 TX ABI support. */
	S31_WIFI_SOFTMAC_TX_AGG_CAP,
	S31_WIFI_SOFTMAC_TX_PEER_ADD, /* mac=AP, offset=AID, field=MCS, total=factor, length=density, dtim_period=MPDU limit, hidden=HE */
	S31_WIFI_SOFTMAC_TX_PEER_DEL,
	S31_WIFI_SOFTMAC_TX_BA_ON, /* initial prototype supports TID0 only */
	S31_WIFI_SOFTMAC_TX_BA_OFF,
	S31_WIFI_SOFTMAC_TX_BA_FLUSH,
	S31_WIFI_SOFTMAC_TX_HE_CAP, /* HE-SU20 GI1.6, negotiated TID0 BA only. */
	S31_WIFI_SOFTMAC_TX_HE9_CAP, /* HE MCS8/9 GI1.6 support. */
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
#define S31_WIFI_IF_SOFTMAC 3U
#define S31_WIFI_IF_TX_STATUS 4U
#define S31_WIFI_IF_SOFTMAC_VIEW 5U
#define S31_SOFTMAC_START_RX_VIEW (1U << 2)
#define S31_SOFTMAC_START_RX_RA_FILTER (1U << 3)
#define S31_SOFTMAC_START_RX_DROP_ERRORS (1U << 4)
#define S31_SOFTMAC_START_RX_BSSID_FILTER (1U << 5)
#define S31_SOFTMAC_FRAME_MAX 4096U
/* Keep enough native submissions in flight to cover the controller completion window. */
#define S31_SOFTMAC_PENDING 64U
#define S31_SOFTMAC_TX_HE_RATE 0x40U /* HE-SU20, NSS1, GI1.6; low four bits MCS0..9. */
#define S31_SOFTMAC_TX_HT_RATE 0x80U /* TX status rate: low three bits are MCS. */
#define S31_SOFTMAC_KEY_RX_CCMP 1U
#define S31_SOFTMAC_KEY_TX_CCMP 2U /* Pairwise slot 4 only. */
#define S31_SOFTMAC_TX_MAC_CCMP 1U /* Bit0: Linux IV/PN, plaintext, no MIC tail. */

/* Cookie values, never pointers. Native little endian on the RV32 bridge. */
struct s31_softmac_tx_header {
	uint32_t cookie;
	uint32_t epoch;
	uint8_t rate_index;
	uint8_t reserved[3]; /* [0]=1 HT20 LGI, 2 HE-SU20 GI1.6; [1]=TID0 A-MPDU; [2]=odd CCMP key token or 0. */
};
#define S31_SOFTMAC_RX_CCMP_VALID (1U << 4) /* Decrypted and MIC stripped; PN retained. */
struct s31_softmac_rx_header {
	uint8_t rate_index;
	uint8_t encoding; /* 0=legacy, 1=HT, 2=HE SU; rate_index is MCS for HT/HE. */
	uint8_t flags; /* bit0=HT short GI, bit1=40MHz, bits2..3=HE GI enum, bit4=validated CCMP RX. */
	uint8_t reserved;
};
/* Borrowed native DMA frame. Valid only during the synchronous receive_aux
 * callback; the Linux consumer must copy before returning and never retain it.
 * Enabled by the matched native START capability. */
struct s31_softmac_rx_view {
	struct s31_softmac_rx_header header;
	const uint8_t *frame;
};
struct s31_softmac_tx_status {
	uint32_t cookie;
	uint8_t success;
	uint8_t rate;
	uint8_t attempts;
	uint8_t reserved;
};

#endif
