/* SPDX-License-Identifier: (GPL-2.0-only OR MIT) */
#ifndef _LINUX_SOC_ESP32S31_CRYPTO_H
#define _LINUX_SOC_ESP32S31_CRYPTO_H

/* Shared by the native provider and the SDK adapter; not a userspace ABI.
 * Composite domains acquire all dependencies in this total order.
 */
enum esp32s31_crypto_domain {
	S31_CRYPTO_KEY_MANAGER,
	S31_CRYPTO_BULK,
	S31_CRYPTO_MPI,
	S31_CRYPTO_ECC,
	S31_CRYPTO_ECDSA,
	S31_CRYPTO_HMAC,
	S31_CRYPTO_DS,
	S31_CRYPTO_DOMAINS,
	S31_CRYPTO_AES = S31_CRYPTO_DOMAINS,
	S31_CRYPTO_SHA,
};

enum esp32s31_crypto_operation {
	S31_CRYPTO_ACQUIRE,
	S31_CRYPTO_RELEASE,
	S31_CRYPTO_CLOCK_ON,
	S31_CRYPTO_CLOCK_OFF,
	S31_CRYPTO_QUIESCENT,
	/* A leased SDK operation could not establish completion. Quarantine
	 * only this task's held engines before its normal clock/unlock unwind.
	 */
	S31_CRYPTO_FAULT,
};

#ifdef __KERNEL__
long esp32s31_crypto_radio_operation(unsigned int operation, unsigned int domain);
#endif

#endif
