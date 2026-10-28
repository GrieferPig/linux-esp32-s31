/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_ESP32S31_CRYPTO_H
#define _UAPI_LINUX_ESP32S31_CRYPTO_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define ESP32S31_CRYPTO_ABI_VERSION 1
#define ESP32S31_CRYPTO_KEY_PRIVATE 1

enum esp32s31_crypto_algorithm {
	ESP32S31_CRYPTO_RSA = 1,
	ESP32S31_CRYPTO_ECDH_P256,
	ESP32S31_CRYPTO_ECDH_P384,
	ESP32S31_CRYPTO_HMAC_EFUSE,
	ESP32S31_CRYPTO_DS_RSA,
	ESP32S31_CRYPTO_ECDSA_P192,
	ESP32S31_CRYPTO_ECDSA_P256,
	ESP32S31_CRYPTO_ECDSA_P384,
	ESP32S31_CRYPTO_HMAC_KEY_MANAGER,
};

enum esp32s31_crypto_request_operation {
	/* Raw modular exponentiation: callers supply padding/signature encoding. */
	ESP32S31_CRYPTO_RSA_PUBLIC = 1,
	ESP32S31_CRYPTO_RSA_PRIVATE,
	ESP32S31_CRYPTO_ECDH_PUBLIC,
	ESP32S31_CRYPTO_ECDH_SHARED,
	ESP32S31_CRYPTO_HMAC_SHA256,
	ESP32S31_CRYPTO_DS_PRIVATE,
	ESP32S31_CRYPTO_ECDSA_PUBLIC,
	ESP32S31_CRYPTO_ECDSA_SIGN,
	ESP32S31_CRYPTO_ECDSA_VERIFY,
};

struct esp32s31_crypto_caps {
	__u32 version;
	__u32 algorithms; /* BIT(algorithm - 1) */
	__u32 max_key_bytes;
	__u32 reserved;
};

/* One key per open file. Every import attempt invalidates the previous key.
 * RSA uses PKCS#1 DER (RSAPublicKey/RSAPrivateKey). ECDH uses a fixed-width
 * big-endian private scalar; public points are uncompressed X || Y, with no
 * SEC1 prefix. No ioctl provisions eFuses or accepts arbitrary MMIO access.
 */
struct esp32s31_crypto_import {
	__u32 algorithm;
	__u32 flags;
	__u32 key_len;
	__u32 reserved;
	__aligned_u64 key;
};

struct esp32s31_crypto_request {
	__u32 operation;
	__u32 reserved;
	__u32 input_len;
	__u32 output_len; /* Capacity in; bytes produced/required out. */
	__aligned_u64 input;
	__aligned_u64 output;
};

#define ESP32S31_CRYPTO_EFUSE_SLOTS 5
#define ESP32S31_CRYPTO_SLOT_READ_PROTECTED 1
#define ESP32S31_CRYPTO_HMAC_MAX_MESSAGE 65536

/* Public metadata only. No key bytes are read. HMAC_EFUSE imports a __u32
 * slot index (0..4), with KEY_PRIVATE set, and requires HMAC_UP purpose (8).
 * HMAC_SHA256 accepts at most HMAC_MAX_MESSAGE bytes and produces 32 bytes.
 * A selected slot belongs to its open file, like an ordinary imported key.
 */
struct esp32s31_crypto_slot {
	__u32 index;
	__u32 purpose;
	__u32 flags;
	__u32 reserved;
};

#define ESP32S31_CRYPTO_DS_WRAPPED_BYTES 1584
/* DS_RSA imports this header followed by a PKCS#1 DER RSAPublicKey of
 * public_key_len bytes. Fields use the native little-endian S31 ABI. IV and
 * wrapped bytes match ESP-IDF esp_ds_data_t; rsa_bits replaces rsa_length.
 * Supported widths are 1024/2048/3072/4096 bits. Only existing HMAC_DOWN_DS
 * or HMAC_DOWN_ALL eFuse slots are accepted. No plaintext private key input.
 * DS_PRIVATE is a raw RSA operation on one full-width big-endian integer;
 * padding/blinding belongs to the application backend. Each result is checked
 * against the supplied public key before release. RSA_PUBLIC is also allowed.
 */
struct esp32s31_crypto_ds_key {
	__u32 slot;
	__u32 rsa_bits;
	__u32 public_key_len;
	__u32 reserved;
	__u8 iv[16];
	__u8 wrapped[ESP32S31_CRYPTO_DS_WRAPPED_BYTES];
	__u8 public_key[];
};

#define ESP32S31_CRYPTO_SLOT_UNUSED ((__u32)~0U)
/* ECDSA private imports select existing eFuse slots, never scalar bytes.
 * P192 requires purpose16 and P256 purpose1 in slot; slot_high must be UNUSED.
 * P384 requires purpose17 in slot and purpose18 in a distinct slot_high.
 * Reserved fields are zero. Public imports use SEC1 uncompressed points.
 * ECDSA_PUBLIC exports that same SEC1 point. SIGN accepts a 1..64-byte digest
 * (leftmost curve-width bytes, left-padded when shorter) and returns fixed-width
 * big-endian r || s. VERIFY consumes digest || r || s and produces no output.
 * Existing-slot signing uses the hardware deterministic nonce generator and
 * independently verifies each result in software before returning it.
 */
struct esp32s31_crypto_ecdsa_key {
	__u32 slot;
	__u32 slot_high;
	__u32 reserved[2];
};

/* SDK-compatible opaque recovery record; no plaintext deployed key is read.
 * Only ECDSA (type 0, lengths 0/1/2), HMAC (type 2, length 3), and DS
 * (type 3, length 3) are accepted. CRC32 is IEEE CRC-32, matching the SDK.
 * Recovery is chip-bound, not an authenticated file format: protect the file
 * against replacement and independently authenticate the public key/result.
 */
#define ESP32S31_CRYPTO_KM_MAGIC 0xDEA5CE5AU
#define ESP32S31_CRYPTO_KM_HUK_BYTES 660
#define ESP32S31_CRYPTO_KM_INFO_BYTES 64
struct esp32s31_crypto_km_huk {
	__u8 info[ESP32S31_CRYPTO_KM_HUK_BYTES];
	__u32 crc;
};
struct esp32s31_crypto_km_info {
	__u8 info[ESP32S31_CRYPTO_KM_INFO_BYTES];
	__u32 crc;
};
struct esp32s31_crypto_km_key {
	__u32 magic;
	__u32 version;
	__u8 type;
	__u8 length;
	__u8 mode;
	__u8 reserved[13];
	struct esp32s31_crypto_km_huk huk;
	struct esp32s31_crypto_km_info key[2];
};
/* Volatile AES-mode deployment with a software init key. No eFuse writes,
 * hardware policy locks, or flash/PSRAM key purposes are exposed. Set flags
 * to KM_REUSE_HUK and supply a checked HUK record to recover an existing root;
 * otherwise use this provider's volatile root, generating it on first use.
 * Memory-encryption key purposes and active encrypted-memory keys are refused.
 * P384 uses both encrypted_key entries (high stage followed by low stage).
 * The output is an 824-byte recovery record; all reserved fields are zero.
 */
#define ESP32S31_CRYPTO_KM_REUSE_HUK 1
#define ESP32S31_CRYPTO_DS_KEY_MANAGER 1
struct esp32s31_crypto_km_deploy {
	__u32 type;
	__u32 length;
	__u32 flags;
	__u32 reserved;
	__u8 init_key[32];
	__u8 k2_info[64];
	__u8 encrypted_key[2][32];
	struct esp32s31_crypto_km_key recovery;
};
/* HMAC_KEY_MANAGER imports a complete KM key with KEY_PRIVATE. ECDSA
 * private imports accept either the legacy slot descriptor or a complete KM
 * key. DS bundles set reserved=DS_KEY_MANAGER, slot=UNUSED and place one KM
 * key before the public-key DER; public_key_len counts the DER alone.
 */
#define ESP32S31_CRYPTO_IOC_KM_DEPLOY _IOWR(0xd3, 4, struct esp32s31_crypto_km_deploy)

#define ESP32S31_CRYPTO_IOC_CAPS _IOR(0xd3, 0, struct esp32s31_crypto_caps)
#define ESP32S31_CRYPTO_IOC_IMPORT _IOW(0xd3, 1, struct esp32s31_crypto_import)
#define ESP32S31_CRYPTO_IOC_REQUEST _IOWR(0xd3, 2, struct esp32s31_crypto_request)
#define ESP32S31_CRYPTO_IOC_SLOT _IOWR(0xd3, 3, struct esp32s31_crypto_slot)

#endif
