// SPDX-License-Identifier: GPL-2.0-only
#include <crypto/internal/hash.h>
#include <crypto/sha2.h>
#include <linux/fips.h>
#include <linux/module.h>
#include <linux/unaligned.h>

#include "esp32s31-radio-internal.h"

static int s31_sha256_blocks(struct crypto_sha256_state *ctx, const u8 *data,
			     unsigned int length)
{
	while (length >= SHA256_BLOCK_SIZE) {
		__be32 registers[SHA256_STATE_WORDS];
		u32 block[SHA256_BLOCK_SIZE / sizeof(u32)];
		int i, ret;

		for (i = 0; i < SHA256_STATE_WORDS; i++)
			registers[i] = cpu_to_be32(ctx->state[i]);
		memcpy(block, data, sizeof(block));
		ret = s31_radio_fw_sha256_block(registers, block);
		if (!ret) {
			for (i = 0; i < SHA256_STATE_WORDS; i++)
				ctx->state[i] = be32_to_cpu(registers[i]);
		} else if (ret == -EBUSY || ret == -ENODEV) {
			struct sha256_ctx software = {};

			/*
			 * Firmware can hold SHA across sleeping HMAC and DS
			 * operations. A synchronous shash cannot wait for it.
			 */
			memcpy(software.ctx.state.h, ctx->state, sizeof(ctx->state));
			software.ctx.bytecount = ctx->count;
			sha256_update(&software, data, SHA256_BLOCK_SIZE);
			memcpy(ctx->state, software.ctx.state.h, sizeof(ctx->state));
			memzero_explicit(&software, sizeof(software));
		}
		memzero_explicit(registers, sizeof(registers));
		memzero_explicit(block, sizeof(block));
		if (ret && ret != -EBUSY && ret != -ENODEV)
			return ret;
		data += SHA256_BLOCK_SIZE;
		length -= SHA256_BLOCK_SIZE;
		ctx->count += SHA256_BLOCK_SIZE;
	}

	return length;
}

static int s31_sha256_finish(struct crypto_sha256_state *ctx, const u8 *data,
			      unsigned int length, u8 *digest)
{
	u8 padded[SHA256_BLOCK_SIZE * 2] = {};
	unsigned int size = SHA256_BLOCK_SIZE;
	int i, ret;

	ret = s31_sha256_blocks(ctx, data, length);
	if (ret < 0)
		return ret;
	if (ret)
		memcpy(padded, data + length - ret, ret);
	padded[ret] = 0x80;
	if (ret >= SHA256_BLOCK_SIZE - sizeof(u64))
		size *= 2;
	put_unaligned_be64((ctx->count + ret) * 8, padded + size - sizeof(u64));
	ret = s31_sha256_blocks(ctx, padded, size);
	memzero_explicit(padded, sizeof(padded));
	if (ret)
		return ret;
	for (i = 0; i < SHA256_STATE_WORDS; i++)
		put_unaligned_be32(ctx->state[i], digest + sizeof(u32) * i);
	return 0;
}

static int s31_hmac_setkey(struct crypto_shash *tfm, const u8 *raw_key, unsigned int length)
{
	struct hmac_sha256_key *key = crypto_shash_ctx(tfm);

	if (fips_enabled && length < 14)
		return -EINVAL;
	hmac_sha256_preparekey(key, raw_key, length);
	return 0;
}

static int s31_hmac_init(struct shash_desc *desc)
{
	const struct hmac_sha256_key *key = crypto_shash_ctx(desc->tfm);
	struct crypto_sha256_state *ctx = shash_desc_ctx(desc);

	memcpy(ctx->state, key->key.istate.h, sizeof(ctx->state));
	ctx->count = SHA256_BLOCK_SIZE;
	return 0;
}

static int s31_hmac_update(struct shash_desc *desc, const u8 *data, unsigned int length)
{
	return s31_sha256_blocks(shash_desc_ctx(desc), data, length);
}

static int s31_hmac_finup(struct shash_desc *desc, const u8 *data, unsigned int length, u8 *digest)
{
	const struct hmac_sha256_key *key = crypto_shash_ctx(desc->tfm);
	struct crypto_sha256_state *ctx = shash_desc_ctx(desc);
	u8 inner[SHA256_DIGEST_SIZE];
	int ret;

	ret = s31_sha256_finish(ctx, data, length, inner);
	if (!ret) {
		memcpy(ctx->state, key->key.ostate.h, sizeof(ctx->state));
		ctx->count = SHA256_BLOCK_SIZE;
		ret = s31_sha256_finish(ctx, inner, sizeof(inner), digest);
	}
	memzero_explicit(inner, sizeof(inner));
	return ret;
}

static struct shash_alg s31_hmac_alg = {
	.setkey		= s31_hmac_setkey,
	.init		= s31_hmac_init,
	.update		= s31_hmac_update,
	.finup		= s31_hmac_finup,
	.digestsize	= SHA256_DIGEST_SIZE,
	.base		= {
		.cra_name	= "hmac(sha256)",
		.cra_driver_name	= "hmac-sha256-esp32s31",
		.cra_priority	= 350,
		.cra_flags	= CRYPTO_AHASH_ALG_BLOCK_ONLY,
		.cra_blocksize	= SHA256_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct hmac_sha256_key),
		.cra_module	= THIS_MODULE,
	},
};

int s31_radio_hmac_register(void)
{
	/* Registration adds the block-only buffering space to these sizes. */
	s31_hmac_alg.descsize = sizeof(struct crypto_sha256_state);
	s31_hmac_alg.statesize = 0;
	return crypto_register_shash(&s31_hmac_alg);
}

void s31_radio_hmac_unregister(void)
{
	crypto_unregister_shash(&s31_hmac_alg);
}
