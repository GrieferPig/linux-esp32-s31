// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
#ifndef _UAPI_LINUX_ESP32S31_DS_H
#define _UAPI_LINUX_ESP32S31_DS_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define ESP32S31_DS_MAX_RSA_BYTES 512U
#define ESP32S31_DS_CIPHERTEXT_BYTES 1584U
#define ESP32S31_DS_IV_BYTES 16U

/*
 * Encrypted key as produced by ESP-IDF esp_ds_encrypt_params().
 * rsa_length is the modulus length in 32-bit words, minus one.
 */
struct esp32s31_ds_key {
	__le32 rsa_length;
	__u8 iv[ESP32S31_DS_IV_BYTES];
	__u8 ciphertext[ESP32S31_DS_CIPHERTEXT_BYTES];
};

/*
 * Select a pre-provisioned DS eFuse slot, 0 through 4. The padded input
 * representative and the returned signature are little-endian. Only the
 * first (rsa_length + 1) * 4 bytes of each are significant.
 */
struct esp32s31_ds_sign {
	__u32 slot;
	__u32 reserved;
	struct esp32s31_ds_key key;
	__u8 message[ESP32S31_DS_MAX_RSA_BYTES];
	__u8 signature[ESP32S31_DS_MAX_RSA_BYTES];
};

#define ESP32S31_DS_IOC_SIGN _IOWR('D', 0, struct esp32s31_ds_sign)

#endif
