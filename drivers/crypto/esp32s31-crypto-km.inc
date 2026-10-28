// SPDX-License-Identifier: GPL-2.0-only
/* Volatile Key Manager deployment and per-operation recovery. The HUK sequence
 * matches the S31 ROM km_huk_conf implementation, with bounded waits instead
 * of ROM's unbounded loops. No eFuse or hardware policy-lock writes occur.
 */
#define S31_KM_STATIC 0x18
#define S31_KM_LOCK 0x1c
#define S31_KM_CONF 0x20
#define S31_KM_START 0x24
#define S31_KM_STATE 0x28
#define S31_KM_VALID 0x30
#define S31_KM_HUK_VALID 0x34
#define S31_KM_ASSIST 0x100
#define S31_KM_PUBLIC 0x140
#define S31_KM_INIT 0x180
#define S31_HUK_CLOCK 0x04
#define S31_HUK_CONF 0x20
#define S31_HUK_START 0x24
#define S31_HUK_STATE 0x28
#define S31_HUK_INFO 0x100

#include "esp32s31-crypto-km-crc.inc"

static int s31_km_purpose(unsigned int type, unsigned int length)
{
	if (!type && length <= 2)
		return length == 2 ? 12 : length + 1;
	if (length == 3 && (type == 2 || type == 3))
		return type == 2 ? 6 : 7;
	return -EINVAL;
}

static u32 s31_km_valid_mask(unsigned int type, unsigned int length)
{
	return type ? BIT(type + 1) : BIT(length == 2 ? 6 : length);
}

static int s31_km_validate(const struct esp32s31_crypto_km_key *key,
			 unsigned int type, unsigned int length)
{
	unsigned int stages = !type && length == 2 ? 2 : 1, i;

	BUILD_BUG_ON(sizeof(*key) != 824);
	if (key->magic != ESP32S31_CRYPTO_KM_MAGIC || key->version ||
	    key->type != type || key->length != length ||
	    s31_km_purpose(type, length) < 0 || key->mode > 3 ||
	    memchr_inv(key->reserved, 0, sizeof(key->reserved)))
		return -EINVAL;
	if (key->huk.crc != s31_km_crc(key->huk.info, sizeof(key->huk.info)))
		return -EBADMSG;
	for (i = 0; i < stages; i++)
		if (key->key[i].crc != s31_km_crc(key->key[i].info, sizeof(key->key[i].info)))
			return -EBADMSG;
	if (stages == 1 && memchr_inv(&key->key[1], 0, sizeof(key->key[1])))
		return -EINVAL;
	return 0;
}

static int s31_km_wait(struct s31_crypto_dev *dev, void __iomem *base,
		       unsigned int reg, u32 expected)
{
	u32 value;
	int ret;

	/* Waits finish the hardware protocol even after a userspace signal;
	 * cleanup/reset is mandatory before the lease can be released.
	 * The usual state transition takes microseconds. A short bounded spin
	 * avoids a scheduler round trip when the first read is still busy;
	 * delayed or failed hardware keeps the sleepable timeout below.
	 */
	value = readl(base + reg);
	if ((value & 3) == expected)
		return 0;
	ret = readl_poll_timeout(base + reg, value, (value & 3) == expected, 0, 20);
	if (ret)
		ret = readl_poll_timeout(base + reg, value,
					 (value & 3) == expected, 10, 100000);
	if (ret) {
		dev_err_ratelimited(dev->dev, "%s state %#x, expected %#x; timed out\n",
			base == dev->huk ? "HUK" : "Key Manager", value & 3, expected);
		s31_crypto_fault(dev, s31_crypto_owned_mask(dev));
	}
	return ret;
}

static int s31_km_huk(struct s31_crypto_dev *dev,
		      const struct esp32s31_crypto_km_huk *recovery,
		      struct esp32s31_crypto_km_huk *generated)
{
	const struct esp32s31_crypto_km_huk *info = recovery ?: generated;
	bool generate = !recovery;
	void __iomem *huk = dev->huk;
	u32 clock = readl(huk + S31_HUK_CLOCK);
	int ret;

	/* Never replace a root/key serving memory encryption. The native crypto
	 * provider is the sole KM owner; encrypted-memory users cannot share it.
	 */
	if (readl(dev->key_manager + S31_KM_VALID) & (BIT(2) | BIT(5)))
		return -EBUSY;
	/* AES deployment explicitly requests a fresh volatile root when no
	 * recovery record is supplied. As in IDF, a prior volatile HUK need
	 * not be invalidated first; existing records can recover their roots.
	 */
	if (generate && dev->huk_owned) {
		*generated = dev->huk_recovery;
		generate = false;
	}
	/* The composite lease is the sole HUK writer. Its recovery record is
	 * public helper data, not a private root. Reuse only the exact root we
	 * last established and only while silicon still reports it valid. KM
	 * key resets do not reset HUK; loss of power or a provider reprobe must
	 * recover it again. Different records always run the full protocol.
	 */
	if (!generate && dev->huk_owned &&
	    !memcmp(info, &dev->huk_recovery, sizeof(*info)) &&
	    (readl(dev->key_manager + S31_KM_HUK_VALID) & 1))
		return 0;
	writel(clock | BIT(0) | BIT(1), huk + S31_HUK_CLOCK);
	ret = s31_km_wait(dev, huk, S31_HUK_STATE, 0);
	if (ret)
		goto out;
	writel((readl(huk + S31_HUK_CONF) & ~BIT(0)) | generate, huk + S31_HUK_CONF);
	writel(1, huk + S31_HUK_START);
	ret = s31_km_wait(dev, huk, S31_HUK_STATE, 1);
	if (ret)
		goto out;
	if (!generate)
		s31_write_words(huk, S31_HUK_INFO, info->info, sizeof(info->info));
	writel(2, huk + S31_HUK_START);
	ret = s31_km_wait(dev, huk, S31_HUK_STATE, 2);
	if (ret)
		goto out;
	if (generate)
		s31_read_words(huk, S31_HUK_INFO, generated->info, sizeof(generated->info));
	writel(2, huk + S31_HUK_START);
	ret = s31_km_wait(dev, huk, S31_HUK_STATE, 0);
	if (!ret && !(readl(dev->key_manager + S31_KM_HUK_VALID) & 1))
		ret = -EKEYREJECTED;
	if (!ret) {
		if (generate)
			generated->crc = s31_km_crc(generated->info, sizeof(generated->info));
		dev->huk_recovery = *info;
		dev->huk_owned = true;
	}
out:
	/* Recovery info is not a secret key; do not touch HUK state memory
	 * while the state machine is stuck. Quarantine prevents its reuse.
	 */
	if (!ret)
		s31_clear_words(huk + S31_HUK_INFO, sizeof(info->info));
	writel(clock, huk + S31_HUK_CLOCK);
	return ret;
}

static int s31_km_stage(struct s31_crypto_dev *dev, unsigned int purpose,
			struct esp32s31_crypto_km_info *info,
			const struct esp32s31_crypto_km_deploy *deploy, unsigned int stage)
{
	void __iomem *km = dev->key_manager;
	int ret;

	ret = s31_km_wait(dev, km, S31_KM_STATE, 0);
	if (ret)
		return ret;
	writel((purpose << 3) | (deploy ? 1 : 4), km + S31_KM_CONF);
	if (deploy)
		writel(readl(km + S31_KM_STATIC) | BIT(10), km + S31_KM_STATIC);
	if (deploy && !(readl(km + S31_KM_STATIC) & BIT(10)))
		return -EACCES;
	writel(1, km + S31_KM_START);
	ret = s31_km_wait(dev, km, S31_KM_STATE, 1);
	if (ret)
		return ret;
	if (deploy) {
		s31_write_words(km, S31_KM_INIT, deploy->init_key, sizeof(deploy->init_key));
		s31_write_words(km, S31_KM_ASSIST, deploy->k2_info, sizeof(deploy->k2_info));
		s31_write_words(km, S31_KM_PUBLIC, deploy->encrypted_key[stage], 32);
	} else {
		s31_write_words(km, S31_KM_ASSIST, info->info, sizeof(info->info));
	}
	writel(2, km + S31_KM_START);
	ret = s31_km_wait(dev, km, S31_KM_STATE, 2);
	if (ret)
		return ret;
	if (deploy) {
		s31_read_words(km, S31_KM_PUBLIC, info->info, sizeof(info->info));
		info->crc = s31_km_crc(info->info, sizeof(info->info));
	}
	writel(2, km + S31_KM_START);
	return s31_km_wait(dev, km, S31_KM_STATE, 0);
}

/*
 * Only accepts an immutable kernel-owned record validated at key import.
 * The caller retains that key for the operation, holds the engine's composite
 * lease and has enabled the KM clock. Untrusted records must be validated
 * before they are copied into the imported key, not on this restore path.
 */
static int s31_km_recover_imported(struct s31_crypto_dev *dev,
			const struct esp32s31_crypto_km_key *key)
{
	struct esp32s31_crypto_km_info info;
	void __iomem *km = dev->key_manager;
	unsigned int stages = !key->type && key->length == 2 ? 2 : 1, i;
	u32 bit = BIT(key->type == 3 ? 3 : key->type == 2 ? 2 : 0);
	int ret;

	if (readl(km + S31_KM_LOCK) & bit)
		return -EACCES;
	/* Immutable validated record remains owned by the imported key while
	 * this operation holds its composite lease. No scratch copy is needed.
	 */
	ret = s31_km_huk(dev, &key->huk, NULL);
	for (i = 0; !ret && i < stages; i++) {
		info = key->key[i];
		ret = s31_km_stage(dev, i ? 11 : s31_km_purpose(key->type, key->length),
				   &info, NULL, i);
	}
	if (!ret && !(readl(km + S31_KM_VALID) & s31_km_valid_mask(key->type, key->length)))
		ret = -EKEYREJECTED;
	if (!ret) {
		writel(readl(km + S31_KM_STATIC) & ~bit, km + S31_KM_STATIC);
		if (readl(km + S31_KM_STATIC) & bit)
			ret = -EACCES;
	}
	memzero_explicit(&info, sizeof(info));
	return ret;
}

static int s31_km_deploy(struct esp32s31_crypto_km_deploy *job)
{
	static const unsigned int clocks[] = { S31_CRYPTO_ECC, S31_CRYPTO_SHA,
		S31_CRYPTO_KEY_MANAGER };
	struct esp32s31_crypto_km_huk huk = job->recovery.huk;
	struct esp32s31_crypto_km_key *key = &job->recovery;
	struct s31_crypto_dev *dev;
	unsigned int stages, i, enabled = 0;
	int purpose = s31_km_purpose(job->type, job->length), ret, error;

	if (job->reserved || job->flags & ~ESP32S31_CRYPTO_KM_REUSE_HUK || purpose < 0)
		return -EINVAL;
	if ((job->flags & ESP32S31_CRYPTO_KM_REUSE_HUK) &&
	    huk.crc != s31_km_crc(huk.info, sizeof(huk.info)))
		return -EBADMSG;
	memzero_explicit(key, sizeof(*key));
	stages = !job->type && job->length == 2 ? 2 : 1;
	ret = s31_crypto_sdk_acquire(S31_CRYPTO_DS);
	if (ret)
		goto out;
	dev = READ_ONCE(s31_crypto_provider);
	/* Check before enabling/resetting KM, not after destroying another key. */
	if (readl(dev->key_manager + S31_KM_VALID) & (BIT(2) | BIT(5))) {
		ret = -EBUSY;
		goto release;
	}
	for (i = 0; i < ARRAY_SIZE(clocks); i++) {
		ret = s31_crypto_sdk_clock(dev, clocks[i], true);
		if (ret)
			goto cleanup;
		enabled++;
	}
	ret = s31_km_huk(dev, job->flags & ESP32S31_CRYPTO_KM_REUSE_HUK ?
			 &huk : NULL, &huk);
	key->huk = huk;
	for (i = 0; !ret && i < stages; i++)
		ret = s31_km_stage(dev, i ? 11 : purpose, &key->key[i], job, i);
	if (!ret && !(readl(dev->key_manager + S31_KM_VALID) &
		      s31_km_valid_mask(job->type, job->length)))
		ret = -EKEYREJECTED;
	if (!ret) {
		key->magic = ESP32S31_CRYPTO_KM_MAGIC;
		key->type = job->type;
		key->length = job->length;
		key->mode = 1;
	}
cleanup:
	while (enabled) {
		error = s31_crypto_sdk_clock(dev, clocks[--enabled], false);
		if (!ret)
			ret = error;
	}
release:
	error = s31_crypto_sdk_release(dev, S31_CRYPTO_DS);
	if (!ret)
		ret = error;
out:
	memzero_explicit(&huk, sizeof(huk));
	if (ret)
		memzero_explicit(key, sizeof(*key));
	return ret;
}
