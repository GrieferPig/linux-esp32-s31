// SPDX-License-Identifier: GPL-2.0-only
/* Signature API plus existing eFuse signing handles. No fuse programming or
 * secret-shadow reads: the only eFuse command issued here is a read reload.
 */

#define S31_ECDSA_BASE		0x6000
#define S31_ECDSA_CONF		0x04
#define S31_ECDSA_START		0x1c
#define S31_ECDSA_STATE		0x20
#define S31_ECDSA_RESULT		0x24
#define S31_ECDSA_R		0x3e0
#define S31_ECDSA_S		0x410
#define S31_ECDSA_Z		0x440
#define S31_ECDSA_QX		0x470
#define S31_ECDSA_QY		0x4a0
#define S31_ECDSA_LOAD		1
#define S31_ECDSA_GET		2
#define S31_ECDSA_SIGN		1
#define S31_ECDSA_EXPORT		2

/* Resource begins at EFUSE_CONF, beyond the NVMEM read shadows. */
#define S31_EFUSE_SELECTOR	0x04
#define S31_EFUSE_COMMAND	0x0c
#define S31_EFUSE_UPDATE		0x30
#define S31_EFUSE_READ_OPCODE	0x5aa5

struct s31_ecdsa_ctx {
	struct crypto_sig *check;
	const struct ecc_curve *curve;
	struct esp32s31_crypto_ecdsa_key slots;
	struct esp32s31_crypto_km_key *km;
	u8 point[1 + 2 * S31_ECC_MAX_BYTES];
	unsigned int bytes, curve_id, mode;
	bool have_public, private;
};

static int s31_ecdsa_wait(void __iomem *reg, u32 mask, u32 expected,
			  unsigned long deadline)
{
	for (;;) {
		if ((readl(reg) & mask) == expected)
			return 0;
		if (fatal_signal_pending(current))
			return -EINTR;
		if (time_after_eq(jiffies, deadline))
			return -ETIMEDOUT;
		usleep_range(10, 20);
	}
}

static int s31_ecdsa_slots(struct s31_crypto_dev *dev,
			   const struct s31_ecdsa_ctx *ctx,
			   const struct esp32s31_crypto_ecdsa_key *key)
{
	struct esp32s31_crypto_slot low = { .index = key->slot };
	struct esp32s31_crypto_slot high = { .index = key->slot_high };
	int ret;

	if (key->reserved[0] || key->reserved[1])
		return -EINVAL;
	if (ctx->bytes != 48 && key->slot_high != ESP32S31_CRYPTO_SLOT_UNUSED)
		return -EINVAL;
	ret = s31_crypto_slot_read(dev, &low);
	if (ret)
		return ret;
	if (low.purpose != (ctx->bytes == 24 ? 16 : ctx->bytes == 32 ? 1 : 17))
		return -ENOKEY;
	if (ctx->bytes != 48)
		return 0;
	if (key->slot == key->slot_high)
		return -EINVAL;
	ret = s31_crypto_slot_read(dev, &high);
	if (!ret && high.purpose != 18)
		ret = -ENOKEY;
	return ret;
}

static int s31_ecdsa_select(struct s31_crypto_dev *dev,
			    const struct s31_ecdsa_ctx *ctx,
			    const struct esp32s31_crypto_ecdsa_key *key,
			    unsigned long deadline)
{
	void __iomem *efuse = dev->efuse_read_control;
	u32 mask, selected, value;
	int ret;

	ret = s31_ecdsa_wait(efuse + S31_EFUSE_COMMAND, 3, 0, deadline);
	if (ret)
		return ret;
	/* API slot indices name KEY0..KEY4; ECDSA selects physical BLK4..BLK8. */
	if (ctx->bytes == 48) {
		mask = GENMASK(15, 8);
		selected = ((key->slot + 4) << 8) | ((key->slot_high + 4) << 12);
	} else {
		unsigned int shift = ctx->bytes == 24 ? 0 : 4;

		mask = 15U << shift;
		selected = (key->slot + 4) << shift;
	}
	value = readl(efuse + S31_EFUSE_SELECTOR);
	writel((value & ~mask) | selected, efuse + S31_EFUSE_SELECTOR);
	/* This only latches volatile selector fields; it is not a program strobe. */
	writel(readl(efuse + S31_EFUSE_UPDATE) | BIT(12), efuse + S31_EFUSE_UPDATE);
	writel(S31_EFUSE_READ_OPCODE, efuse);
	writel(1, efuse + S31_EFUSE_COMMAND);
	ret = s31_ecdsa_wait(efuse + S31_EFUSE_COMMAND, 1, 0, deadline);
	if (!ret)
		/* The S31 SDK requires a second read to confirm the eFuse clock. */
		ret = s31_ecdsa_wait(efuse + S31_EFUSE_COMMAND, 1, 0, deadline);
	if (!ret && ((readl(efuse + S31_EFUSE_SELECTOR) >> 16) & mask) != selected)
		ret = -EKEYREJECTED;
	return ret;
}

static void s31_ecdsa_write(void __iomem *ecdsa, unsigned int reg,
			    const u8 *big_endian, unsigned int bytes)
{
	u8 little[S31_ECC_MAX_BYTES] = { };

	s31_reverse(little, big_endian, bytes);
	s31_write_words(ecdsa, reg, little, sizeof(little));
	memzero_explicit(little, sizeof(little));
}

static void s31_ecdsa_read(void __iomem *ecdsa, unsigned int reg,
			   u8 *big_endian, unsigned int bytes)
{
	u8 little[S31_ECC_MAX_BYTES];

	s31_read_words(ecdsa, reg, little, bytes);
	s31_reverse(big_endian, little, bytes);
	memzero_explicit(little, sizeof(little));
}

static int s31_ecdsa_hardware(struct s31_ecdsa_ctx *ctx, unsigned int mode,
			      const struct esp32s31_crypto_ecdsa_key *slots,
			      const struct esp32s31_crypto_km_key *km, const u8 *hash, const u8 *signature, u8 *output)
{
	static const unsigned int clocks[] = {
		S31_CRYPTO_ECDSA, S31_CRYPTO_KEY_MANAGER,
	};
	struct s31_crypto_dev *dev;
	void __iomem *ecdsa;
	unsigned int enabled = 0, count = mode ? ARRAY_SIZE(clocks) : 1;
	unsigned long deadline;
	u32 value, config;
	int ret, error;

	ret = s31_crypto_sdk_acquire(S31_CRYPTO_ECDSA);
	if (ret)
		return ret;
	dev = READ_ONCE(s31_crypto_provider);
	ecdsa = dev->base + S31_ECDSA_BASE;
	deadline = jiffies + msecs_to_jiffies(2000);
	if (mode && !km) {
		ret = s31_ecdsa_slots(dev, ctx, slots);
		if (ret)
			goto release;
	}
	while (enabled < count) {
		ret = s31_crypto_sdk_clock(dev, clocks[enabled], true);
		if (ret)
			goto clear;
		enabled++;
	}
	if (mode && km) {
		ret = s31_km_recover_imported(dev, km);
		if (ret)
			goto clear;
	} else if (mode) {
		value = readl(dev->key_manager + S31_KEY_MANAGER_STATIC) | BIT(0);
		writel(value, dev->key_manager + S31_KEY_MANAGER_STATIC);
		if (!(readl(dev->key_manager + S31_KEY_MANAGER_STATIC) & BIT(0))) {
			ret = -EACCES;
			goto clear;
		}
		ret = s31_ecdsa_select(dev, ctx, slots, deadline);
		if (ret)
			goto clear;
	}
	config = mode | (ctx->mode << 2) | BIT(5);
	if (mode)
		config |= BIT(7);
	if (mode == S31_ECDSA_SIGN)
		config |= BIT(6); /* Hardware deterministic nonce; never software k. */
	writel(config, ecdsa + S31_ECDSA_CONF);
	writel(1, ecdsa + S31_ECDSA_START);
	ret = s31_ecdsa_wait(ecdsa + S31_ECDSA_STATE, 3, S31_ECDSA_LOAD, deadline);
	if (ret)
		goto clear;
	if (mode != S31_ECDSA_EXPORT)
		s31_ecdsa_write(ecdsa, S31_ECDSA_Z, hash, ctx->bytes);
	if (!mode) {
		s31_ecdsa_write(ecdsa, S31_ECDSA_R, signature, ctx->bytes);
		s31_ecdsa_write(ecdsa, S31_ECDSA_S, signature + ctx->bytes, ctx->bytes);
		s31_ecdsa_write(ecdsa, S31_ECDSA_QX, ctx->point + 1, ctx->bytes);
		s31_ecdsa_write(ecdsa, S31_ECDSA_QY, ctx->point + 1 + ctx->bytes, ctx->bytes);
	}
	writel(2, ecdsa + S31_ECDSA_START);
	ret = s31_ecdsa_wait(ecdsa + S31_ECDSA_STATE, 3,
			     mode ? S31_ECDSA_GET : 0, deadline);
	if (ret)
		goto clear;
	if (mode) {
		s31_ecdsa_read(ecdsa, mode == S31_ECDSA_EXPORT ? S31_ECDSA_QX : S31_ECDSA_R,
			       output, ctx->bytes);
		s31_ecdsa_read(ecdsa, mode == S31_ECDSA_EXPORT ? S31_ECDSA_QY : S31_ECDSA_S,
			       output + ctx->bytes, ctx->bytes);
		writel(4, ecdsa + S31_ECDSA_START);
		ret = s31_ecdsa_wait(ecdsa + S31_ECDSA_STATE, 3, 0, deadline);
	}
	if (!ret && !mode && !(readl(ecdsa + S31_ECDSA_RESULT) & BIT(0)))
		ret = -EKEYREJECTED;
clear:
	/* Reset first, even after a partial transaction. The composite lease
	 * owns every engine touched by reset or temporary private/nonce state.
	 */
	while (enabled) {
		error = s31_crypto_sdk_clock(dev, clocks[--enabled], false);
		if (!ret)
			ret = error;
	}
	if (!(atomic_read(&dev->failed_domains) & BIT(S31_CRYPTO_ECC))) {
		s31_clear_words(dev->base + S31_ECC_BASE + S31_ECC_K_MEM, 3 * S31_ECC_MAX_BYTES);
		readl(dev->base + S31_ECC_BASE + S31_ECC_PY_MEM +
		      S31_ECC_MAX_BYTES - sizeof(u32));
	}
	if (!(atomic_read(&dev->failed_domains) & BIT(S31_CRYPTO_BULK))) {
		s31_clear_words(dev->base + S31_SHA_BASE + S31_SHA_H_MEM, SHA512_DIGEST_SIZE);
		s31_clear_words(dev->base + S31_SHA_BASE + S31_SHA_M_MEM, SHA512_BLOCK_SIZE);
		readl(dev->base + S31_SHA_BASE + S31_SHA_M_MEM +
		      SHA512_BLOCK_SIZE - sizeof(u32));
	}
	if (!(atomic_read(&dev->failed_domains) & BIT(S31_CRYPTO_MPI))) {
		s31_crypto_reset(dev, S31_CRYPTO_RSA_RST);
		error = readl_poll_timeout(dev->base + S31_RSA_BASE + S31_RSA_QUERY_CLEAN,
					  value, value & BIT(0), 20, 10000);
		if (!error) {
			s31_clear_words(dev->base + S31_RSA_BASE, 4 * S31_RSA_MAX_BYTES);
			readl(dev->base + S31_RSA_BASE + S31_RSA_X_MEM +
			      S31_RSA_MAX_BYTES - sizeof(u32));
		} else {
			s31_crypto_fault(dev, BIT(S31_CRYPTO_MPI));
		}
		if (!ret)
			ret = error;
	}
release:
	error = s31_crypto_sdk_release(dev, S31_CRYPTO_ECDSA);
	return ret ?: error;
}

static void s31_ecdsa_invalidate(struct s31_ecdsa_ctx *ctx)
{
	kfree_sensitive(ctx->km);
	ctx->km = NULL;
	ctx->have_public = false;
	ctx->private = false;
	memset(&ctx->slots, 0, sizeof(ctx->slots));
	memzero_explicit(ctx->point, sizeof(ctx->point));
}

static int s31_ecdsa_set_public(struct crypto_sig *tfm, const void *key,
			       unsigned int length)
{
	struct s31_ecdsa_ctx *ctx = crypto_sig_ctx(tfm);
	int ret;

	s31_ecdsa_invalidate(ctx);
	if (length != 1 + 2 * ctx->bytes || *(const u8 *)key != 4)
		return -EINVAL;
	ret = crypto_sig_set_pubkey(ctx->check, key, length);
	if (ret)
		return ret;
	memcpy(ctx->point, key, length);
	ctx->have_public = true;
	return 0;
}

static int s31_ecdsa_set_private(struct crypto_sig *tfm, const void *key,
				unsigned int length)
{
	struct s31_ecdsa_ctx *ctx = crypto_sig_ctx(tfm);
	struct esp32s31_crypto_ecdsa_key slots;
	u8 point[1 + 2 * S31_ECC_MAX_BYTES] = { 4 };
	int ret;

	s31_ecdsa_invalidate(ctx);
	if (length == sizeof(struct esp32s31_crypto_km_key)) {
		ret = s31_km_validate(key, 0, ctx->mode);
		if (ret)
			return ret;
		ctx->km = kmemdup(key, length, GFP_KERNEL);
		if (!ctx->km)
			return -ENOMEM;
		memset(&slots, 0, sizeof(slots));
	} else if (length == sizeof(slots)) {
		memcpy(&slots, key, sizeof(slots));
	} else {
		return -EINVAL;
	}
	ret = s31_ecdsa_hardware(ctx, S31_ECDSA_EXPORT, &slots, ctx->km,
				 NULL, NULL, point + 1);
	if (!ret)
		ret = crypto_sig_set_pubkey(ctx->check, point, 1 + 2 * ctx->bytes);
	if (!ret) {
		memcpy(ctx->point, point, 1 + 2 * ctx->bytes);
		ctx->have_public = true;
		ctx->slots = slots;
		ctx->private = true;
	} else {
		s31_ecdsa_invalidate(ctx);
	}
	memzero_explicit(point, sizeof(point));
	return ret;
}

static void s31_ecdsa_raw_to_bytes(const struct ecdsa_raw_sig *raw, u8 *bytes,
				  unsigned int width)
{
	unsigned int i, digits = width / sizeof(u64);

	for (i = 0; i < digits; i++) {
		put_unaligned_be64(raw->r[digits - 1 - i], bytes + 8 * i);
		put_unaligned_be64(raw->s[digits - 1 - i], bytes + width + 8 * i);
	}
}

static int s31_ecdsa_verify(struct crypto_sig *tfm, const void *signature,
			   unsigned int length, const void *digest, unsigned int dlen)
{
	struct s31_ecdsa_ctx *ctx = crypto_sig_ctx(tfm);
	const struct ecdsa_raw_sig *raw = signature;
	u8 hash[S31_ECC_MAX_BYTES] = { }, bytes[2 * S31_ECC_MAX_BYTES];
	unsigned int digits = ctx->bytes / sizeof(u64);
	int ret;

	if (!ctx->have_public)
		return -ENOKEY;
	if (length != sizeof(*raw) || !dlen || dlen > SHA512_DIGEST_SIZE)
		return -EINVAL;
	if (vli_is_zero(raw->r, digits) || vli_cmp(raw->r, ctx->curve->n, digits) >= 0 ||
	    vli_is_zero(raw->s, digits) || vli_cmp(raw->s, ctx->curve->n, digits) >= 0)
		return -EBADMSG;
	/* Signature API consumers can run in softirq context. Only public
	 * verification has a software representation; never sleep there.
	 */
	if (in_atomic() || irqs_disabled())
		return crypto_sig_verify(ctx->check, signature, length, digest, dlen);
	dlen = min(dlen, ctx->bytes);
	memcpy(hash + ctx->bytes - dlen, digest, dlen);
	s31_ecdsa_raw_to_bytes(raw, bytes, ctx->bytes);
	ret = s31_ecdsa_hardware(ctx, 0, NULL, NULL, hash, bytes, NULL);
	memzero_explicit(hash, sizeof(hash));
	return ret;
}

static int s31_ecdsa_sign(struct crypto_sig *tfm, const void *digest, unsigned int dlen,
			 void *signature, unsigned int capacity)
{
	struct s31_ecdsa_ctx *ctx = crypto_sig_ctx(tfm);
	struct ecdsa_raw_sig raw = { };
	u8 hash[S31_ECC_MAX_BYTES] = { }, bytes[2 * S31_ECC_MAX_BYTES] = { };
	unsigned int length = min(dlen, ctx->bytes), digits = ctx->bytes / sizeof(u64);
	int ret;

	if (!ctx->private || !ctx->have_public)
		return -ENOKEY;
	if (!dlen || dlen > SHA512_DIGEST_SIZE || capacity < sizeof(raw))
		return -EINVAL;
	if (in_atomic() || irqs_disabled())
		return -EWOULDBLOCK;
	memcpy(hash + ctx->bytes - length, digest, length);
	ret = s31_ecdsa_hardware(ctx, S31_ECDSA_SIGN, &ctx->slots, ctx->km, hash, NULL, bytes);
	if (!ret) {
		ecc_digits_from_bytes(bytes, ctx->bytes, raw.r, digits);
		ecc_digits_from_bytes(bytes + ctx->bytes, ctx->bytes, raw.s, digits);
		/* No result is released if an independent implementation rejects it. */
		ret = crypto_sig_verify(ctx->check, &raw, sizeof(raw), digest, dlen);
	}
	if (!ret) {
		memcpy(signature, &raw, sizeof(raw));
		ret = sizeof(raw);
	}
	memzero_explicit(&raw, sizeof(raw));
	memzero_explicit(hash, sizeof(hash));
	memzero_explicit(bytes, sizeof(bytes));
	return ret;
}

static int s31_ecdsa_init(struct crypto_sig *tfm, unsigned int bytes,
			 unsigned int curve_id, const char *generic)
{
	struct s31_ecdsa_ctx *ctx = crypto_sig_ctx(tfm);

	ctx->bytes = bytes;
	ctx->mode = bytes == 24 ? 0 : bytes == 32 ? 1 : 2;
	ctx->curve_id = curve_id;
	ctx->curve = ecc_get_curve(curve_id);
	if (!ctx->curve)
		return -EINVAL;
	ctx->check = crypto_alloc_sig(generic, 0, 0);
	return PTR_ERR_OR_ZERO(ctx->check);
}

static void s31_ecdsa_exit(struct crypto_sig *tfm)
{
	struct s31_ecdsa_ctx *ctx = crypto_sig_ctx(tfm);

	crypto_free_sig(ctx->check);
	s31_ecdsa_invalidate(ctx);
}

static unsigned int s31_ecdsa_bits(struct crypto_sig *tfm)
{
	return ((struct s31_ecdsa_ctx *)crypto_sig_ctx(tfm))->bytes * 8;
}

static unsigned int s31_ecdsa_digest_size(struct crypto_sig *tfm)
{
	return SHA512_DIGEST_SIZE;
}

static unsigned int s31_ecdsa_signature_size(struct crypto_sig *tfm)
{
	return sizeof(struct ecdsa_raw_sig);
}

#define S31_ECDSA_ALG(bits, width, curve) \
static int s31_ecdsa_init_##bits(struct crypto_sig *tfm) \
{ \
	return s31_ecdsa_init(tfm, width, curve, "ecdsa-nist-p" #bits "-generic"); \
}
S31_ECDSA_ALG(192, 24, ECC_CURVE_NIST_P192)
S31_ECDSA_ALG(256, 32, ECC_CURVE_NIST_P256)
S31_ECDSA_ALG(384, 48, ECC_CURVE_NIST_P384)

#define S31_ECDSA_DESCRIPTOR(bits) { \
	.sign = s31_ecdsa_sign, .verify = s31_ecdsa_verify, \
	.set_pub_key = s31_ecdsa_set_public, .set_priv_key = s31_ecdsa_set_private, \
	.key_size = s31_ecdsa_bits, .digest_size = s31_ecdsa_digest_size, \
	.max_size = s31_ecdsa_signature_size, .init = s31_ecdsa_init_##bits, \
	.exit = s31_ecdsa_exit, .base = { .cra_name = "ecdsa-nist-p" #bits, \
	.cra_driver_name = "ecdsa-nist-p" #bits "-esp32s31", .cra_priority = 300, \
	.cra_module = THIS_MODULE, .cra_ctxsize = sizeof(struct s31_ecdsa_ctx) } \
}

static struct sig_alg s31_ecdsa_algs[] = {
	S31_ECDSA_DESCRIPTOR(192),
	S31_ECDSA_DESCRIPTOR(256),
	S31_ECDSA_DESCRIPTOR(384),
};
