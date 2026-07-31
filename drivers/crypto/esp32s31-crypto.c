// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 AES and SHA Crypto API providers. */

#include <crypto/aes.h>
#include <crypto/akcipher.h>
#include <crypto/algapi.h>
#include <crypto/ecdh.h>
#include <crypto/internal/ecc.h>
#include <crypto/hash.h>
#include <crypto/internal/akcipher.h>
#include <crypto/internal/hash.h>
#include <crypto/internal/kpp.h>
#include <crypto/internal/rsa.h>
#include <crypto/internal/skcipher.h>
#include <crypto/kpp.h>
#include <crypto/scatterwalk.h>
#include <crypto/sha1.h>
#include <crypto/sha2.h>
#include <linux/bitops.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>

#define S31_AES_KEY		0x000
#define S31_AES_TEXT_IN		0x020
#define S31_AES_TEXT_OUT	0x030
#define S31_AES_MODE		0x040
#define S31_AES_TRIGGER		0x048
#define S31_AES_STATE		0x04c
#define S31_AES_IDLE		0

#define S31_SHA_BASE		0x1000
#define S31_SHA_MODE		0x000
#define S31_SHA_START		0x010
#define S31_SHA_CONTINUE	0x014
#define S31_SHA_BUSY		0x018
#define S31_SHA_DMA_BLOCK_NUM	0x00c
#define S31_SHA_DMA_START	0x01c
#define S31_SHA_DMA_CONTINUE	0x020
#define S31_SHA_H_MEM		0x040
#define S31_SHA_M_MEM		0x080

#define S31_RSA_BASE		0x2000
#define S31_RSA_M_MEM		0x000
#define S31_RSA_Z_MEM		0x200
#define S31_RSA_Y_MEM		0x400
#define S31_RSA_X_MEM		0x600
#define S31_RSA_M_PRIME		0x800
#define S31_RSA_MODE		0x804
#define S31_RSA_QUERY_CLEAN	0x808
#define S31_RSA_START_MODEXP	0x80c
#define S31_RSA_QUERY_IDLE	0x818
#define S31_RSA_INT_CLR		0x81c
#define S31_RSA_CONSTANT_TIME	0x820
#define S31_RSA_SEARCH_ENABLE	0x824
#define S31_RSA_SEARCH_POS	0x828

#define S31_ECC_BASE		0x3000
#define S31_ECC_INT_RAW		0x00c
#define S31_ECC_INT_CLR		0x018
#define S31_ECC_CONF		0x01c
#define S31_ECC_K_MEM		0x100
#define S31_ECC_PX_MEM		0x130
#define S31_ECC_PY_MEM		0x160

#define S31_RSA_MAX_BYTES	512
#define S31_RSA_MAX_WORDS	(S31_RSA_MAX_BYTES / sizeof(u32))
#define S31_ECC_MAX_BYTES	48

#define S31_ECC_START		BIT(0)
#define S31_ECC_KEY_LENGTH_SHIFT	2
#define S31_ECC_WORK_MODE_SHIFT	5
#define S31_ECC_SECURITY_MODE	BIT(9)
#define S31_ECC_VERIFY_RESULT	BIT(29)

#define S31_ECC_MODE_POINT_MUL		0
#define S31_ECC_MODE_VERIFY_THEN_POINT_MUL	3
#define S31_ECC_CURVE_P256		1
#define S31_ECC_CURVE_P384		2

/* HP_SYS_CLKRST.crypto_ctrl0, from the S31 IDF register definitions. */
#define S31_CRYPTO_SYS_CLK	BIT(0)
#define S31_CRYPTO_AES_CLK	BIT(4)
#define S31_CRYPTO_SHA_CLK	BIT(6)
#define S31_CRYPTO_RSA_CLK	BIT(8)
#define S31_CRYPTO_ECC_CLK	BIT(12)

enum s31_aes_mode {
	S31_AES_ECB,
	S31_AES_CBC,
	S31_AES_CTR,
};

struct s31_crypto_dev {
	struct device *dev;
	void __iomem *base;
	void __iomem *clkrst;
	struct dma_chan *sha_dma;
	raw_spinlock_t lock;
};

struct s31_aes_ctx {
	u8 key[AES_MAX_KEY_SIZE];
	u32 keylen;
};

struct s31_sha_ctx {
	u32 state[16];
	u64 total;
	u8 buffer[SHA512_BLOCK_SIZE];
	u8 type;
	u8 digestsize;
	u8 blocksize;
	bool first;
};

struct s31_rsa_key {
	u32 n[S31_RSA_MAX_WORDS];
	u32 e[S31_RSA_MAX_WORDS];
	u32 d[S31_RSA_MAX_WORDS];
	u32 rinv[S31_RSA_MAX_WORDS];
	u32 mprime;
	unsigned int words;
	unsigned int bytes;
	unsigned int e_bits;
	unsigned int d_bits;
};

struct s31_rsa_ctx {
	struct crypto_akcipher *fallback;
	struct s31_rsa_key key;
};

struct s31_rsa_request {
	u8 input[S31_RSA_MAX_BYTES];
	u8 output[S31_RSA_MAX_BYTES];
	u32 number[S31_RSA_MAX_WORDS];
	u32 result[S31_RSA_MAX_WORDS];
};

struct s31_ecdh_ctx {
	u8 private[S31_ECC_MAX_BYTES];
	unsigned int bytes;
	unsigned int curve;
	unsigned int digits;
	bool have_private;
};

static struct s31_crypto_dev *s31_crypto;

static void s31_write_words(void __iomem *base, unsigned int offset,
			    const u8 *src, unsigned int len)
{
	u32 value;
	unsigned int i;

	for (i = 0; i < len; i += sizeof(value)) {
		memcpy(&value, src + i, sizeof(value));
		writel(value, base + offset + i);
	}
}

static void s31_read_words(void __iomem *base, unsigned int offset,
			   u8 *dst, unsigned int len)
{
	u32 value;
	unsigned int i;

	for (i = 0; i < len; i += sizeof(value)) {
		value = readl(base + offset + i);
		memcpy(dst + i, &value, sizeof(value));
	}
}

static int s31_aes_block(struct s31_aes_ctx *ctx, const u8 *src, u8 *dst,
			 bool encrypt)
{
	struct s31_crypto_dev *dev = READ_ONCE(s31_crypto);
	void __iomem *aes;
	unsigned long flags;
	u32 state, mode;
	int ret;

	if (!dev)
		return -ENODEV;
	mode = (encrypt ? 0 : BIT(2)) + ctx->keylen / 8 - 2;
	raw_spin_lock_irqsave(&dev->lock, flags);
	aes = dev->base;
	s31_write_words(aes, S31_AES_KEY, ctx->key, ctx->keylen);
	writel(mode, aes + S31_AES_MODE);
	s31_write_words(aes, S31_AES_TEXT_IN, src, AES_BLOCK_SIZE);
	writel(1, aes + S31_AES_TRIGGER);
	ret = readl_poll_timeout_atomic(aes + S31_AES_STATE, state,
				       state == S31_AES_IDLE, 1, 1000);
	if (!ret)
		s31_read_words(aes, S31_AES_TEXT_OUT, dst, AES_BLOCK_SIZE);
	/* Keys must not survive another tenant's request in the shared engine. */
	for (state = 0; state < AES_MAX_KEY_SIZE; state += sizeof(u32))
		writel(0, aes + S31_AES_KEY + state);
	raw_spin_unlock_irqrestore(&dev->lock, flags);
	return ret;
}

static int s31_aes_setkey(struct crypto_skcipher *tfm, const u8 *key,
			  unsigned int keylen)
{
	struct s31_aes_ctx *ctx = crypto_skcipher_ctx(tfm);

	if (keylen != AES_KEYSIZE_128 && keylen != AES_KEYSIZE_256)
		return -EINVAL;
	memzero_explicit(ctx->key, sizeof(ctx->key));
	memcpy(ctx->key, key, keylen);
	ctx->keylen = keylen;
	return 0;
}

static int s31_aes_crypt(struct skcipher_request *req, bool encrypt,
			 enum s31_aes_mode mode)
{
	struct s31_aes_ctx *ctx = crypto_skcipher_ctx(
		crypto_skcipher_reqtfm(req));
	u8 input[AES_BLOCK_SIZE], output[AES_BLOCK_SIZE], iv[AES_BLOCK_SIZE];
	unsigned int offset, n;
	int ret = 0;

	if (!ctx->keylen)
		return -ENOKEY;
	if (mode != S31_AES_CTR && req->cryptlen & (AES_BLOCK_SIZE - 1))
		return -EINVAL;
	if (mode != S31_AES_ECB)
		memcpy(iv, req->iv, AES_BLOCK_SIZE);

	for (offset = 0; offset < req->cryptlen; offset += n) {
		n = min_t(unsigned int, AES_BLOCK_SIZE, req->cryptlen - offset);
		scatterwalk_map_and_copy(input, req->src, offset, n, 0);
		if (mode == S31_AES_CTR) {
			ret = s31_aes_block(ctx, iv, output, true);
			if (!ret) {
				crypto_xor(output, input, n);
				crypto_inc(iv, AES_BLOCK_SIZE);
			}
		} else if (mode == S31_AES_CBC && encrypt) {
			crypto_xor(input, iv, AES_BLOCK_SIZE);
			ret = s31_aes_block(ctx, input, output, true);
			memcpy(iv, output, AES_BLOCK_SIZE);
		} else if (mode == S31_AES_CBC) {
			ret = s31_aes_block(ctx, input, output, false);
			if (!ret)
				crypto_xor(output, iv, AES_BLOCK_SIZE);
			memcpy(iv, input, AES_BLOCK_SIZE);
		} else {
			ret = s31_aes_block(ctx, input, output, encrypt);
		}
		if (ret)
			break;
		scatterwalk_map_and_copy(output, req->dst, offset, n, 1);
	}
	if (mode != S31_AES_ECB)
		memcpy(req->iv, iv, AES_BLOCK_SIZE);
	memzero_explicit(input, sizeof(input));
	memzero_explicit(output, sizeof(output));
	memzero_explicit(iv, sizeof(iv));
	return ret;
}

#define S31_AES_CRYPT(_name, _enc, _mode) \
static int _name(struct skcipher_request *req) \
{ return s31_aes_crypt(req, _enc, _mode); }

S31_AES_CRYPT(s31_ecb_encrypt, true, S31_AES_ECB)
S31_AES_CRYPT(s31_ecb_decrypt, false, S31_AES_ECB)
S31_AES_CRYPT(s31_cbc_encrypt, true, S31_AES_CBC)
S31_AES_CRYPT(s31_cbc_decrypt, false, S31_AES_CBC)
S31_AES_CRYPT(s31_ctr_crypt, true, S31_AES_CTR)

#define S31_AES_ALG(_name, _driver, _enc, _dec, _ivsize, _block) { \
	.setkey = s31_aes_setkey, .encrypt = _enc, .decrypt = _dec, \
	.min_keysize = AES_KEYSIZE_128, .max_keysize = AES_KEYSIZE_256, \
	.ivsize = _ivsize, .chunksize = AES_BLOCK_SIZE, .walksize = AES_BLOCK_SIZE, \
	.base = { .cra_name = _name, .cra_driver_name = _driver, \
		.cra_priority = 300, .cra_blocksize = _block, \
		.cra_ctxsize = sizeof(struct s31_aes_ctx), .cra_module = THIS_MODULE } \
}

static struct skcipher_alg s31_aes_algs[] = {
	S31_AES_ALG("ecb(aes)", "ecb-aes-esp32s31", s31_ecb_encrypt,
		    s31_ecb_decrypt, 0, AES_BLOCK_SIZE),
	S31_AES_ALG("cbc(aes)", "cbc-aes-esp32s31", s31_cbc_encrypt,
		    s31_cbc_decrypt, AES_BLOCK_SIZE, AES_BLOCK_SIZE),
	S31_AES_ALG("ctr(aes)", "ctr-aes-esp32s31", s31_ctr_crypt,
		    s31_ctr_crypt, AES_BLOCK_SIZE, 1),
};

static void s31_sha_config(struct s31_sha_ctx *ctx, struct crypto_shash *tfm,
			   u8 type)
{
	ctx->digestsize = crypto_shash_digestsize(tfm);
	ctx->blocksize = crypto_shash_blocksize(tfm);
	ctx->type = type;
}

static int s31_sha_process_pio(struct s31_sha_ctx *ctx, const u8 *data,
			       unsigned int blocks)
{
	struct s31_crypto_dev *dev = READ_ONCE(s31_crypto);
	void __iomem *sha;
	unsigned long flags;
	unsigned int words = ctx->blocksize == SHA512_BLOCK_SIZE ? 16 : 8;
	u32 busy;
	unsigned int i;
	int ret;

	if (!dev)
		return -ENODEV;
	raw_spin_lock_irqsave(&dev->lock, flags);
	sha = dev->base + S31_SHA_BASE;
	writel(ctx->type, sha + S31_SHA_MODE);
	if (!ctx->first)
		for (i = 0; i < words; i++)
			writel(ctx->state[i], sha + S31_SHA_H_MEM + i * sizeof(u32));
	for (i = 0; i < blocks; i++) {
		s31_write_words(sha, S31_SHA_M_MEM, data + i * ctx->blocksize,
				ctx->blocksize);
		writel(1, sha + (ctx->first ? S31_SHA_START : S31_SHA_CONTINUE));
		ret = readl_poll_timeout_atomic(sha + S31_SHA_BUSY, busy, !busy,
					       1, 1000);
		if (ret)
			goto out;
		ctx->first = false;
	}
	for (i = 0; i < words; i++)
		ctx->state[i] = readl(sha + S31_SHA_H_MEM + i * sizeof(u32));
	ret = 0;
out:
	raw_spin_unlock_irqrestore(&dev->lock, flags);
	return ret;
}

static int s31_sha_process_dma(struct s31_sha_ctx *ctx, const u8 *data,
			       unsigned int blocks)
{
	struct s31_crypto_dev *dev = READ_ONCE(s31_crypto);
	struct scatterlist sg;
	struct dma_async_tx_descriptor *desc;
	void __iomem *sha;
	dma_addr_t dma;
	dma_cookie_t cookie;
	unsigned int words = ctx->blocksize == SHA512_BLOCK_SIZE ? 16 : 8;
	unsigned int len = blocks * ctx->blocksize;
	u32 busy;
	int i, ret = 0;

	if (!dev || !dev->sha_dma || blocks > U16_MAX)
		return -ENODEV;
	dma = dma_map_single(dev->dev, (void *)data, len, DMA_TO_DEVICE);
	if (dma_mapping_error(dev->dev, dma))
		return -EIO;
	sg_init_table(&sg, 1);
	sg_dma_address(&sg) = dma;
	sg_dma_len(&sg) = len;

	/* The crypto engines share state, so keep the transaction contiguous. */
	/* GDMA completion is reported from an IRQ, so do not mask it here. */
	raw_spin_lock(&dev->lock);
	sha = dev->base + S31_SHA_BASE;
	writel(ctx->type, sha + S31_SHA_MODE);
	if (!ctx->first)
		for (i = 0; i < words; i++)
			writel(ctx->state[i], sha + S31_SHA_H_MEM + i * sizeof(u32));
	desc = dmaengine_prep_slave_sg(dev->sha_dma, &sg, 1, DMA_MEM_TO_DEV,
				       DMA_CTRL_ACK);
	if (!desc) {
		ret = -EIO;
		goto out_unlock;
	}
	cookie = dmaengine_submit(desc);
	if (dma_submit_error(cookie)) {
		ret = -EIO;
		goto out_unlock;
	}
	dma_async_issue_pending(dev->sha_dma);
	writel(blocks, sha + S31_SHA_DMA_BLOCK_NUM);
	writel(1, sha + (ctx->first ? S31_SHA_DMA_START : S31_SHA_DMA_CONTINUE));
	ret = readl_poll_timeout_atomic(sha + S31_SHA_BUSY, busy,
				       dma_async_is_tx_complete(dev->sha_dma, cookie,
						       NULL, NULL) == DMA_COMPLETE,
				       1, 10000);
	if (ret) {
		dmaengine_terminate_async(dev->sha_dma);
		goto out_unlock;
	}
	ret = readl_poll_timeout_atomic(sha + S31_SHA_BUSY, busy, !busy, 1, 10000);
	if (ret)
		goto out_unlock;
	ctx->first = false;
	for (i = 0; i < words; i++)
		ctx->state[i] = readl(sha + S31_SHA_H_MEM + i * sizeof(u32));
out_unlock:
	raw_spin_unlock(&dev->lock);
	dma_unmap_single(dev->dev, dma, len, DMA_TO_DEVICE);
	return ret;
}

static int s31_sha_process(struct s31_sha_ctx *ctx, const u8 *data,
			   unsigned int blocks)
{
	if (READ_ONCE(s31_crypto) && READ_ONCE(s31_crypto)->sha_dma)
		return s31_sha_process_dma(ctx, data, blocks);
	return s31_sha_process_pio(ctx, data, blocks);
}

static int s31_sha_update_bytes(struct s31_sha_ctx *ctx, const u8 *data,
				unsigned int len)
{
	unsigned int left = ctx->total & (ctx->blocksize - 1);
	unsigned int fill = ctx->blocksize - left;
	unsigned int blocks;
	int ret;

	ctx->total += len;
	if (left && len >= fill) {
		memcpy(ctx->buffer + left, data, fill);
		ret = s31_sha_process(ctx, ctx->buffer, 1);
		if (ret)
			return ret;
		data += fill;
		len -= fill;
		left = 0;
	}
	blocks = len / ctx->blocksize;
	if (blocks) {
		ret = s31_sha_process(ctx, data, blocks);
		if (ret)
			return ret;
		data += blocks * ctx->blocksize;
		len -= blocks * ctx->blocksize;
	}
	if (len)
		memcpy(ctx->buffer + left, data, len);
	return 0;
}

static int s31_sha_init_type(struct shash_desc *desc, u8 type)
{
	struct s31_sha_ctx *ctx = shash_desc_ctx(desc);

	memset(ctx, 0, sizeof(*ctx));
	s31_sha_config(ctx, desc->tfm, type);
	ctx->first = true;
	return 0;
}

#define S31_SHA_INIT(_name, _type) \
static int _name(struct shash_desc *desc) \
{ return s31_sha_init_type(desc, _type); }

S31_SHA_INIT(s31_sha1_init, 0)
S31_SHA_INIT(s31_sha224_init, 1)
S31_SHA_INIT(s31_sha256_init, 2)
S31_SHA_INIT(s31_sha384_init, 3)
S31_SHA_INIT(s31_sha512_init, 4)
S31_SHA_INIT(s31_sha512_224_init, 5)
S31_SHA_INIT(s31_sha512_256_init, 6)

static int s31_sha_update(struct shash_desc *desc, const u8 *data,
			  unsigned int len)
{
	return s31_sha_update_bytes(shash_desc_ctx(desc), data, len);
}

static int s31_sha_final(struct shash_desc *desc, u8 *out)
{
	struct s31_sha_ctx *ctx = shash_desc_ctx(desc);
	u8 padding[SHA512_BLOCK_SIZE] = { 0x80 };
	u8 length[16] = { };
	u64 bits = ctx->total << 3;
	unsigned int last = ctx->total & (ctx->blocksize - 1);
	unsigned int limit = ctx->blocksize == SHA512_BLOCK_SIZE ? 112 : 56;
	unsigned int lenbytes = ctx->blocksize == SHA512_BLOCK_SIZE ? 16 : 8;
	unsigned int padlen = last < limit ? limit - last :
			2 * ctx->blocksize - last - lenbytes;
	int i, ret;

	for (i = 0; i < 8; i++)
		length[lenbytes - 1 - i] = bits >> (i * 8);
	ret = s31_sha_update_bytes(ctx, padding, padlen);
	if (!ret)
		ret = s31_sha_update_bytes(ctx, length, lenbytes);
	if (!ret)
		memcpy(out, ctx->state, ctx->digestsize);
	memzero_explicit(ctx, sizeof(*ctx));
	return ret;
}

static int s31_sha_finup(struct shash_desc *desc, const u8 *data,
			 unsigned int len, u8 *out)
{
	int ret = s31_sha_update(desc, data, len);

	return ret ?: s31_sha_final(desc, out);
}

#define S31_SHA_ALG(_name, _driver, _digest, _block, _init) { \
	.digestsize = _digest, .init = _init, .update = s31_sha_update, \
	.final = s31_sha_final, .finup = s31_sha_finup, \
	.descsize = sizeof(struct s31_sha_ctx), \
	.base = { .cra_name = _name, .cra_driver_name = _driver, \
		.cra_priority = 300, .cra_blocksize = _block, .cra_module = THIS_MODULE } \
}

static struct shash_alg s31_sha_algs[] = {
	S31_SHA_ALG("sha1", "sha1-esp32s31", SHA1_DIGEST_SIZE, SHA1_BLOCK_SIZE,
		    s31_sha1_init),
	S31_SHA_ALG("sha224", "sha224-esp32s31", SHA224_DIGEST_SIZE, SHA224_BLOCK_SIZE,
		    s31_sha224_init),
	S31_SHA_ALG("sha256", "sha256-esp32s31", SHA256_DIGEST_SIZE, SHA256_BLOCK_SIZE,
		    s31_sha256_init),
	S31_SHA_ALG("sha384", "sha384-esp32s31", SHA384_DIGEST_SIZE, SHA384_BLOCK_SIZE,
		    s31_sha384_init),
	S31_SHA_ALG("sha512", "sha512-esp32s31", SHA512_DIGEST_SIZE, SHA512_BLOCK_SIZE,
		    s31_sha512_init),
	S31_SHA_ALG("sha512-224", "sha512-224-esp32s31", SHA224_DIGEST_SIZE,
		    SHA512_BLOCK_SIZE, s31_sha512_224_init),
	S31_SHA_ALG("sha512-256", "sha512-256-esp32s31", SHA256_DIGEST_SIZE,
		    SHA512_BLOCK_SIZE, s31_sha512_256_init),
};

static void s31_be_to_limbs(u32 *dst, unsigned int words, const u8 *src,
			    unsigned int len)
{
	unsigned int i;

	memset(dst, 0, words * sizeof(*dst));
	for (i = 0; i < len; i++)
		dst[i / sizeof(u32)] |= (u32)src[len - 1 - i] << ((i % 4) * 8);
}

static void s31_limbs_to_be(u8 *dst, unsigned int len, const u32 *src)
{
	unsigned int i;

	for (i = 0; i < len; i++)
		dst[len - 1 - i] = src[i / sizeof(u32)] >> ((i % 4) * 8);
}

static int s31_limbs_cmp(const u32 *a, const u32 *b, unsigned int words)
{
	while (words--) {
		if (a[words] != b[words])
			return a[words] > b[words] ? 1 : -1;
	}
	return 0;
}

static void s31_limbs_sub(u32 *a, const u32 *b, unsigned int words)
{
	u64 borrow = 0;
	unsigned int i;

	for (i = 0; i < words; i++) {
		u64 sub = (u64)b[i] + borrow;
		borrow = a[i] < sub;
		a[i] -= sub;
	}
}

static unsigned int s31_limbs_bits(const u32 *a, unsigned int words)
{
	while (words && !a[words - 1])
		words--;
	return words ? (words - 1) * 32 + fls(a[words - 1]) : 0;
}

static u32 s31_rsa_mprime(u32 modulus)
{
	u32 inverse = 1;
	int i;

	/* Each iteration doubles the number of correct bits modulo 2^32. */
	for (i = 0; i < 5; i++)
		inverse *= 2 - modulus * inverse;
	return 0 - inverse;
}

static void s31_rsa_compute_rinv(struct s31_rsa_key *key)
{
	u64 value;
	u32 carry;
	unsigned int i, j;

	memset(key->rinv, 0, sizeof(key->rinv));
	key->rinv[0] = 1;
	for (i = 0; i < key->words * 64; i++) {
		carry = 0;
		for (j = 0; j < key->words; j++) {
			value = ((u64)key->rinv[j] << 1) | carry;
			key->rinv[j] = value;
			carry = value >> 32;
		}
		if (carry || s31_limbs_cmp(key->rinv, key->n, key->words) >= 0)
			s31_limbs_sub(key->rinv, key->n, key->words);
	}
}

static int s31_rsa_modexp(struct s31_crypto_dev *dev,
			  const struct s31_rsa_key *key, const u32 *input,
			  const u32 *exponent, unsigned int exponent_bits,
			  bool constant_time, u32 *output)
{
	void __iomem *rsa = dev->base + S31_RSA_BASE;
	unsigned long flags;
	u32 value;
	int ret;

	if (!exponent_bits)
		return -EINVAL;
	raw_spin_lock_irqsave(&dev->lock, flags);
	ret = readl_poll_timeout_atomic(rsa + S31_RSA_QUERY_CLEAN, value,
					 value & BIT(0), 1, 10000);
	if (ret)
		goto out_unlock;
	writel(key->words - 1, rsa + S31_RSA_MODE);
	s31_write_words(rsa, S31_RSA_M_MEM, (const u8 *)key->n, key->bytes);
	s31_write_words(rsa, S31_RSA_X_MEM, (const u8 *)input, key->bytes);
	s31_write_words(rsa, S31_RSA_Y_MEM, (const u8 *)exponent, key->bytes);
	s31_write_words(rsa, S31_RSA_Z_MEM, (const u8 *)key->rinv, key->bytes);
	writel(key->mprime, rsa + S31_RSA_M_PRIME);
	writel(constant_time ? BIT(0) : 0, rsa + S31_RSA_CONSTANT_TIME);
	writel(constant_time ? 0 : BIT(0), rsa + S31_RSA_SEARCH_ENABLE);
	if (!constant_time)
		writel(exponent_bits - 1, rsa + S31_RSA_SEARCH_POS);
	writel(BIT(0), rsa + S31_RSA_INT_CLR);
	writel(BIT(0), rsa + S31_RSA_START_MODEXP);
	ret = readl_poll_timeout_atomic(rsa + S31_RSA_QUERY_IDLE, value,
					 value & BIT(0), 1, 2000000);
	if (!ret)
		s31_read_words(rsa, S31_RSA_Z_MEM, (u8 *)output, key->bytes);
	writel(0, rsa + S31_RSA_SEARCH_ENABLE);
	memset_io(rsa + S31_RSA_M_MEM, 0, S31_RSA_MAX_BYTES);
	memset_io(rsa + S31_RSA_Z_MEM, 0, S31_RSA_MAX_BYTES);
	memset_io(rsa + S31_RSA_Y_MEM, 0, S31_RSA_MAX_BYTES);
	memset_io(rsa + S31_RSA_X_MEM, 0, S31_RSA_MAX_BYTES);
out_unlock:
	raw_spin_unlock_irqrestore(&dev->lock, flags);
	return ret;
}

static void s31_rsa_clear_key(struct s31_rsa_key *key)
{
	memzero_explicit(key, sizeof(*key));
}

static int s31_rsa_load_key(struct s31_rsa_key *key,
			    const struct rsa_key *raw, bool private)
{
	const u8 *modulus = raw->n;
	unsigned int modulus_len = raw->n_sz;

	while (modulus_len > 1 && !*modulus) {
		modulus++;
		modulus_len--;
	}
	if (modulus_len > S31_RSA_MAX_BYTES || !(*modulus & 1))
		return -EOPNOTSUPP;
	key->words = DIV_ROUND_UP(modulus_len, sizeof(u32));
	key->bytes = key->words * sizeof(u32);
	s31_be_to_limbs(key->n, key->words, modulus, modulus_len);
	s31_be_to_limbs(key->e, key->words, raw->e, raw->e_sz);
	key->e_bits = s31_limbs_bits(key->e, key->words);
	if (!key->e_bits)
		return -EINVAL;
	if (private) {
		s31_be_to_limbs(key->d, key->words, raw->d, raw->d_sz);
		key->d_bits = s31_limbs_bits(key->d, key->words);
		if (!key->d_bits)
			return -EINVAL;
	}
	key->mprime = s31_rsa_mprime(key->n[0]);
	s31_rsa_compute_rinv(key);
	return 0;
}

static int s31_rsa_setkey(struct crypto_akcipher *tfm, const void *data,
			  unsigned int len, bool private)
{
	struct s31_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct rsa_key raw = { };
	int ret;

	ret = private ? crypto_akcipher_set_priv_key(ctx->fallback, data, len) :
		crypto_akcipher_set_pub_key(ctx->fallback, data, len);
	if (ret)
		return ret;
	ret = private ? rsa_parse_priv_key(&raw, data, len) :
		rsa_parse_pub_key(&raw, data, len);
	if (ret)
		return ret;
	s31_rsa_clear_key(&ctx->key);
	ret = s31_rsa_load_key(&ctx->key, &raw, private);
	if (ret == -EOPNOTSUPP)
		return 0;
	if (ret)
		s31_rsa_clear_key(&ctx->key);
	return ret;
}

static int s31_rsa_set_pub_key(struct crypto_akcipher *tfm, const void *data,
			       unsigned int len)
{
	return s31_rsa_setkey(tfm, data, len, false);
}

static int s31_rsa_set_priv_key(struct crypto_akcipher *tfm, const void *data,
				unsigned int len)
{
	return s31_rsa_setkey(tfm, data, len, true);
}

static int s31_rsa_crypt(struct akcipher_request *req, bool private)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
	struct s31_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct s31_crypto_dev *dev = READ_ONCE(s31_crypto);
	struct s31_rsa_request *rctx;
	const u32 *exponent;
	unsigned int exponent_bits;
	int ret;

	if (!ctx->key.words) {
		akcipher_request_set_tfm(req, ctx->fallback);
		ret = private ? crypto_akcipher_decrypt(req) :
			crypto_akcipher_encrypt(req);
		akcipher_request_set_tfm(req, tfm);
		return ret;
	}
	if (!dev)
		return -ENODEV;
	if (req->src_len > ctx->key.bytes)
		return -EINVAL;
	if (req->dst_len < ctx->key.bytes) {
		req->dst_len = ctx->key.bytes;
		return -EOVERFLOW;
	}
	rctx = kzalloc(sizeof(*rctx), GFP_ATOMIC);
	if (!rctx)
		return -ENOMEM;
	if (sg_copy_to_buffer(req->src, sg_nents_for_len(req->src, req->src_len),
			      rctx->input + ctx->key.bytes - req->src_len, req->src_len) !=
	    req->src_len)
		goto invalid;
	s31_be_to_limbs(rctx->number, ctx->key.words, rctx->input, ctx->key.bytes);
	if (s31_limbs_cmp(rctx->number, ctx->key.n, ctx->key.words) >= 0)
		goto invalid;
	exponent = private ? ctx->key.d : ctx->key.e;
	exponent_bits = private ? ctx->key.d_bits : ctx->key.e_bits;
	ret = s31_rsa_modexp(dev, &ctx->key, rctx->number, exponent, exponent_bits,
			     private, rctx->result);
	if (!ret) {
		s31_limbs_to_be(rctx->output, ctx->key.bytes, rctx->result);
		if (sg_copy_from_buffer(req->dst,
				sg_nents_for_len(req->dst, ctx->key.bytes), rctx->output,
				ctx->key.bytes) != ctx->key.bytes)
			ret = -EINVAL;
		else
			req->dst_len = ctx->key.bytes;
	}
	kfree_sensitive(rctx);
	return ret;

invalid:
	kfree_sensitive(rctx);
	return -EINVAL;
}

static int s31_rsa_encrypt(struct akcipher_request *req)
{
	return s31_rsa_crypt(req, false);
}

static int s31_rsa_decrypt(struct akcipher_request *req)
{
	return s31_rsa_crypt(req, true);
}

static unsigned int s31_rsa_max_size(struct crypto_akcipher *tfm)
{
	struct s31_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);

	return ctx->key.bytes ?: crypto_akcipher_maxsize(ctx->fallback);
}

static int s31_rsa_init_tfm(struct crypto_akcipher *tfm)
{
	struct s31_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);

	ctx->fallback = crypto_alloc_akcipher("rsa-generic", 0,
					  CRYPTO_ALG_NEED_FALLBACK);
	return IS_ERR(ctx->fallback) ? PTR_ERR(ctx->fallback) : 0;
}

static void s31_rsa_exit_tfm(struct crypto_akcipher *tfm)
{
	struct s31_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);

	crypto_free_akcipher(ctx->fallback);
	s31_rsa_clear_key(&ctx->key);
}

static struct akcipher_alg s31_rsa_alg = {
	.encrypt = s31_rsa_encrypt,
	.decrypt = s31_rsa_decrypt,
	.sign = s31_rsa_decrypt,
	.verify = s31_rsa_encrypt,
	.set_pub_key = s31_rsa_set_pub_key,
	.set_priv_key = s31_rsa_set_priv_key,
	.max_size = s31_rsa_max_size,
	.init = s31_rsa_init_tfm,
	.exit = s31_rsa_exit_tfm,
	.base = {
		.cra_name = "rsa",
		.cra_driver_name = "rsa-esp32s31",
		.cra_priority = 300,
		.cra_flags = CRYPTO_ALG_NEED_FALLBACK,
		.cra_module = THIS_MODULE,
		.cra_ctxsize = sizeof(struct s31_rsa_ctx),
	},
};

static const u8 s31_p256_gx[32] = {
	0x6b, 0x17, 0xd1, 0xf2, 0xe1, 0x2c, 0x42, 0x47,
	0xf8, 0xbc, 0xe6, 0xe5, 0x63, 0xa4, 0x40, 0xf2,
	0x77, 0x03, 0x7d, 0x81, 0x2d, 0xeb, 0x33, 0xa0,
	0xf4, 0xa1, 0x39, 0x45, 0xd8, 0x98, 0xc2, 0x96,
};

static const u8 s31_p256_gy[32] = {
	0x4f, 0xe3, 0x42, 0xe2, 0xfe, 0x1a, 0x7f, 0x9b,
	0x8e, 0xe7, 0xeb, 0x4a, 0x7c, 0x0f, 0x9e, 0x16,
	0x2b, 0xce, 0x33, 0x57, 0x6b, 0x31, 0x5e, 0xce,
	0xcb, 0xb6, 0x40, 0x68, 0x37, 0xbf, 0x51, 0xf5,
};

static const u8 s31_p384_gx[48] = {
	0xaa, 0x87, 0xca, 0x22, 0xbe, 0x8b, 0x05, 0x37,
	0x8e, 0xb1, 0xc7, 0x1e, 0xf3, 0x20, 0xad, 0x74,
	0x6e, 0x1d, 0x3b, 0x62, 0x8b, 0xa7, 0x9b, 0x98,
	0x59, 0xf7, 0x41, 0xe0, 0x82, 0x54, 0x2a, 0x38,
	0x55, 0x02, 0xf2, 0x5d, 0xbf, 0x55, 0x29, 0x6c,
	0x3a, 0x54, 0x5e, 0x38, 0x72, 0x76, 0x0a, 0xb7,
};

static const u8 s31_p384_gy[48] = {
	0x36, 0x17, 0xde, 0x4a, 0x96, 0x26, 0x2c, 0x6f,
	0x5d, 0x9e, 0x98, 0xbf, 0x92, 0x92, 0xdc, 0x29,
	0xf8, 0xf4, 0x1d, 0xbd, 0x28, 0x9a, 0x14, 0x7c,
	0xe9, 0xda, 0x31, 0x13, 0xb5, 0xf0, 0xb8, 0xc0,
	0x0a, 0x60, 0xb1, 0xce, 0x1d, 0x7e, 0x81, 0x9d,
	0x7a, 0x43, 0x1d, 0x7c, 0x90, 0xea, 0x0e, 0x5f,
};

static void s31_reverse(u8 *dst, const u8 *src, unsigned int len)
{
	unsigned int i;

	for (i = 0; i < len; i++)
		dst[i] = src[len - 1 - i];
}

static int s31_ecc_point_mul(struct s31_crypto_dev *dev, unsigned int bytes,
			     const u8 *scalar, const u8 *px, const u8 *py,
			     bool verify, u8 *qx, u8 *qy)
{
	u8 scalar_le[S31_ECC_MAX_BYTES] = { };
	u8 px_le[S31_ECC_MAX_BYTES] = { };
	u8 py_le[S31_ECC_MAX_BYTES] = { };
	u8 qx_le[S31_ECC_MAX_BYTES];
	u8 qy_le[S31_ECC_MAX_BYTES];
	void __iomem *ecc = dev->base + S31_ECC_BASE;
	unsigned long flags;
	u32 value;
	int ret;

	if (bytes != 32 && bytes != 48)
		return -EINVAL;
	s31_reverse(scalar_le, scalar, bytes);
	s31_reverse(px_le, px, bytes);
	s31_reverse(py_le, py, bytes);
	raw_spin_lock_irqsave(&dev->lock, flags);
	s31_write_words(ecc, S31_ECC_K_MEM, scalar_le, S31_ECC_MAX_BYTES);
	s31_write_words(ecc, S31_ECC_PX_MEM, px_le, S31_ECC_MAX_BYTES);
	s31_write_words(ecc, S31_ECC_PY_MEM, py_le, S31_ECC_MAX_BYTES);
	writel(BIT(0), ecc + S31_ECC_INT_CLR);
	writel((bytes == 48 ? S31_ECC_CURVE_P384 : S31_ECC_CURVE_P256)
		       << S31_ECC_KEY_LENGTH_SHIFT |
		       (verify ? S31_ECC_MODE_VERIFY_THEN_POINT_MUL :
			 S31_ECC_MODE_POINT_MUL) << S31_ECC_WORK_MODE_SHIFT |
		       S31_ECC_SECURITY_MODE, ecc + S31_ECC_CONF);
	writel(readl(ecc + S31_ECC_CONF) | S31_ECC_START, ecc + S31_ECC_CONF);
	ret = readl_poll_timeout_atomic(ecc + S31_ECC_INT_RAW, value,
					 value & BIT(0), 1, 2000000);
	if (!ret && verify && !(readl(ecc + S31_ECC_CONF) & S31_ECC_VERIFY_RESULT))
		ret = -EINVAL;
	if (!ret) {
		s31_read_words(ecc, S31_ECC_PX_MEM, qx_le, bytes);
		s31_read_words(ecc, S31_ECC_PY_MEM, qy_le, bytes);
		s31_reverse(qx, qx_le, bytes);
		s31_reverse(qy, qy_le, bytes);
	}
	memset_io(ecc + S31_ECC_K_MEM, 0, S31_ECC_MAX_BYTES);
	memset_io(ecc + S31_ECC_PX_MEM, 0, S31_ECC_MAX_BYTES);
	memset_io(ecc + S31_ECC_PY_MEM, 0, S31_ECC_MAX_BYTES);
	writel(BIT(0), ecc + S31_ECC_INT_CLR);
	raw_spin_unlock_irqrestore(&dev->lock, flags);
	memzero_explicit(scalar_le, sizeof(scalar_le));
	memzero_explicit(px_le, sizeof(px_le));
	memzero_explicit(py_le, sizeof(py_le));
	memzero_explicit(qx_le, sizeof(qx_le));
	memzero_explicit(qy_le, sizeof(qy_le));
	return ret;
}

static int s31_ecdh_set_secret(struct crypto_kpp *tfm, const void *data,
			       unsigned int len)
{
	struct s31_ecdh_ctx *ctx = kpp_tfm_ctx(tfm);
	struct ecdh params;
	u64 private[ECC_MAX_DIGITS];

	if (crypto_ecdh_decode_key(data, len, &params) || !params.key_size ||
	    params.key_size > ctx->bytes)
		return -EINVAL;
	memzero_explicit(ctx->private, sizeof(ctx->private));
	memcpy(ctx->private + ctx->bytes - params.key_size, params.key,
	       params.key_size);
	ecc_digits_from_bytes(ctx->private, ctx->bytes, private, ctx->digits);
	if (ecc_is_key_valid(ctx->curve, ctx->digits, private, ctx->bytes)) {
		memzero_explicit(ctx->private, sizeof(ctx->private));
		memzero_explicit(private, sizeof(private));
		return -EINVAL;
	}
	memzero_explicit(private, sizeof(private));
	ctx->have_private = true;
	return 0;
}

static int s31_ecdh_generate_public_key(struct kpp_request *req)
{
	struct s31_ecdh_ctx *ctx = kpp_tfm_ctx(crypto_kpp_reqtfm(req));
	struct s31_crypto_dev *dev = READ_ONCE(s31_crypto);
	const u8 *gx = ctx->bytes == 48 ? s31_p384_gx : s31_p256_gx;
	const u8 *gy = ctx->bytes == 48 ? s31_p384_gy : s31_p256_gy;
	u8 point[2 * S31_ECC_MAX_BYTES];
	unsigned int len = 2 * ctx->bytes;
	int ret;

	if (!ctx->have_private || !dev)
		return -ENOKEY;
	if (req->dst_len < len) {
		req->dst_len = len;
		return -EOVERFLOW;
	}
	ret = s31_ecc_point_mul(dev, ctx->bytes, ctx->private, gx, gy, false,
				point, point + ctx->bytes);
	if (!ret && sg_copy_from_buffer(req->dst,
			sg_nents_for_len(req->dst, len), point, len) != len)
		ret = -EINVAL;
	if (!ret)
		req->dst_len = len;
	memzero_explicit(point, sizeof(point));
	return ret;
}

static int s31_ecdh_compute_shared_secret(struct kpp_request *req)
{
	struct s31_ecdh_ctx *ctx = kpp_tfm_ctx(crypto_kpp_reqtfm(req));
	struct s31_crypto_dev *dev = READ_ONCE(s31_crypto);
	u8 point[2 * S31_ECC_MAX_BYTES];
	u8 secret[S31_ECC_MAX_BYTES];
	unsigned int len = ctx->bytes;
	int ret;

	if (!ctx->have_private || !dev)
		return -ENOKEY;
	if (req->src_len != 2 * len)
		return -EINVAL;
	if (req->dst_len < len) {
		req->dst_len = len;
		return -EOVERFLOW;
	}
	if (sg_copy_to_buffer(req->src, sg_nents_for_len(req->src, 2 * len),
			      point, 2 * len) != 2 * len)
		return -EINVAL;
	ret = s31_ecc_point_mul(dev, len, ctx->private, point, point + len,
				true, secret, point + len);
	if (!ret && sg_copy_from_buffer(req->dst,
			sg_nents_for_len(req->dst, len), secret, len) != len)
		ret = -EINVAL;
	if (!ret)
		req->dst_len = len;
	memzero_explicit(point, sizeof(point));
	memzero_explicit(secret, sizeof(secret));
	return ret;
}

static unsigned int s31_ecdh_max_size(struct crypto_kpp *tfm)
{
	return 2 * ((struct s31_ecdh_ctx *)kpp_tfm_ctx(tfm))->bytes;
}

static int s31_ecdh_p256_init(struct crypto_kpp *tfm)
{
	struct s31_ecdh_ctx *ctx = kpp_tfm_ctx(tfm);

	ctx->bytes = 32;
	ctx->curve = ECC_CURVE_NIST_P256;
	ctx->digits = ECC_CURVE_NIST_P256_DIGITS;
	return 0;
}

static int s31_ecdh_p384_init(struct crypto_kpp *tfm)
{
	struct s31_ecdh_ctx *ctx = kpp_tfm_ctx(tfm);

	ctx->bytes = 48;
	ctx->curve = ECC_CURVE_NIST_P384;
	ctx->digits = ECC_CURVE_NIST_P384_DIGITS;
	return 0;
}

#define S31_ECDH_ALG(_name, _driver, _init) { \
	.set_secret = s31_ecdh_set_secret, \
	.generate_public_key = s31_ecdh_generate_public_key, \
	.compute_shared_secret = s31_ecdh_compute_shared_secret, \
	.max_size = s31_ecdh_max_size, .init = _init, \
	.base = { .cra_name = _name, .cra_driver_name = _driver, \
		.cra_priority = 300, .cra_module = THIS_MODULE, \
		.cra_ctxsize = sizeof(struct s31_ecdh_ctx) } \
}

static struct kpp_alg s31_ecdh_algs[] = {
	S31_ECDH_ALG("ecdh-nist-p256", "ecdh-nist-p256-esp32s31",
		     s31_ecdh_p256_init),
	S31_ECDH_ALG("ecdh-nist-p384", "ecdh-nist-p384-esp32s31",
		     s31_ecdh_p384_init),
};

static int s31_crypto_selftest(struct s31_crypto_dev *dev)
{
	struct s31_rsa_key *key;
	u32 input = 65;
	u32 output;
	u8 scalar[32] = { 0 };
	u8 qx[32], qy[32];
	u8 scalar384[48] = { 0 };
	u8 qx384[48], qy384[48];
	int ret;

	key = kzalloc(sizeof(*key), GFP_KERNEL);
	if (!key)
		return -ENOMEM;
	key->words = 1;
	key->bytes = sizeof(u32);
	key->n[0] = 3233;
	key->e[0] = 17;
	key->d[0] = 2753;
	key->e_bits = s31_limbs_bits(key->e, key->words);
	key->d_bits = s31_limbs_bits(key->d, key->words);
	key->mprime = s31_rsa_mprime(key->n[0]);
	s31_rsa_compute_rinv(key);
	ret = s31_rsa_modexp(dev, key, &input, key->e, key->e_bits, false, &output);
	if (ret || output != 2790)
		goto fail;
	ret = s31_rsa_modexp(dev, key, &output, key->d, key->d_bits, true, &input);
	if (ret || input != 65)
		goto fail;
	scalar[31] = 1;
	ret = s31_ecc_point_mul(dev, sizeof(scalar), scalar, s31_p256_gx,
			s31_p256_gy, true, qx, qy);
	if (ret || memcmp(qx, s31_p256_gx, sizeof(qx)) ||
	    memcmp(qy, s31_p256_gy, sizeof(qy)))
		goto fail;
	scalar384[47] = 1;
	ret = s31_ecc_point_mul(dev, sizeof(scalar384), scalar384, s31_p384_gx,
			s31_p384_gy, true, qx384, qy384);
	if (ret || memcmp(qx384, s31_p384_gx, sizeof(qx384)) ||
	    memcmp(qy384, s31_p384_gy, sizeof(qy384)))
		goto fail;
	ret = 0;
out:
	kfree_sensitive(key);
	memzero_explicit(qx, sizeof(qx));
	memzero_explicit(qy, sizeof(qy));
	memzero_explicit(scalar384, sizeof(scalar384));
	memzero_explicit(qx384, sizeof(qx384));
	memzero_explicit(qy384, sizeof(qy384));
	return ret;
fail:
	ret = ret ?: -EIO;
	goto out;
}

static int s31_crypto_probe(struct platform_device *pdev)
{
	struct s31_crypto_dev *dev;
	u32 value;
	int ret;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	dev->dev = &pdev->dev;
	dev->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dev->base))
		return PTR_ERR(dev->base);
	dev->clkrst = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(dev->clkrst))
		return PTR_ERR(dev->clkrst);
	raw_spin_lock_init(&dev->lock);
	dev->sha_dma = dma_request_chan(&pdev->dev, "sha");
	if (IS_ERR(dev->sha_dma)) {
		ret = PTR_ERR(dev->sha_dma);
		dev->sha_dma = NULL;
		if (ret == -EPROBE_DEFER)
			return ret;
		dev_warn(&pdev->dev, "SHA DMA unavailable (%d), using PIO\n", ret);
	}
	value = readl(dev->clkrst);
	writel(value | S31_CRYPTO_SYS_CLK | S31_CRYPTO_AES_CLK |
	       S31_CRYPTO_SHA_CLK | S31_CRYPTO_RSA_CLK | S31_CRYPTO_ECC_CLK,
	       dev->clkrst);
	ret = s31_crypto_selftest(dev);
	if (ret) {
		if (dev->sha_dma)
			dma_release_channel(dev->sha_dma);
		return dev_err_probe(&pdev->dev, ret,
				     "RSA/MPI or ECC hardware self-test failed\n");
	}
	WRITE_ONCE(s31_crypto, dev);
	ret = crypto_register_skciphers(s31_aes_algs, ARRAY_SIZE(s31_aes_algs));
	if (ret) {
		WRITE_ONCE(s31_crypto, NULL);
		if (dev->sha_dma)
			dma_release_channel(dev->sha_dma);
		return ret;
	}
	ret = crypto_register_shashes(s31_sha_algs, ARRAY_SIZE(s31_sha_algs));
	if (ret) {
		crypto_unregister_skciphers(s31_aes_algs, ARRAY_SIZE(s31_aes_algs));
		WRITE_ONCE(s31_crypto, NULL);
		if (dev->sha_dma)
			dma_release_channel(dev->sha_dma);
		return ret;
	}
	ret = crypto_register_akcipher(&s31_rsa_alg);
	if (ret)
		goto unregister_shashes;
	ret = crypto_register_kpp(&s31_ecdh_algs[0]);
	if (ret)
		goto unregister_rsa;
	ret = crypto_register_kpp(&s31_ecdh_algs[1]);
	if (ret)
		goto unregister_ecdh_p256;
	platform_set_drvdata(pdev, dev);
	dev_info(&pdev->dev,
		 "AES PIO, SHA %s, RSA/MPI and P-256/P-384 ECDH providers registered\n",
		 dev->sha_dma ? "DMA" : "PIO");
	return 0;

unregister_ecdh_p256:
	crypto_unregister_kpp(&s31_ecdh_algs[0]);
unregister_rsa:
	crypto_unregister_akcipher(&s31_rsa_alg);
unregister_shashes:
	crypto_unregister_shashes(s31_sha_algs, ARRAY_SIZE(s31_sha_algs));
	crypto_unregister_skciphers(s31_aes_algs, ARRAY_SIZE(s31_aes_algs));
	WRITE_ONCE(s31_crypto, NULL);
	if (dev->sha_dma)
		dma_release_channel(dev->sha_dma);
	return ret;
}

static void s31_crypto_remove(struct platform_device *pdev)
{
	struct s31_crypto_dev *dev = platform_get_drvdata(pdev);

	crypto_unregister_kpp(&s31_ecdh_algs[1]);
	crypto_unregister_kpp(&s31_ecdh_algs[0]);
	crypto_unregister_akcipher(&s31_rsa_alg);
	crypto_unregister_shashes(s31_sha_algs, ARRAY_SIZE(s31_sha_algs));
	crypto_unregister_skciphers(s31_aes_algs, ARRAY_SIZE(s31_aes_algs));
	WRITE_ONCE(s31_crypto, NULL);
	if (dev->sha_dma)
		dma_release_channel(dev->sha_dma);
}

static const struct of_device_id s31_crypto_of_match[] = {
	{ .compatible = "espressif,esp32s31-crypto" },
	{ }
};
MODULE_DEVICE_TABLE(of, s31_crypto_of_match);

static struct platform_driver s31_crypto_driver = {
	.probe = s31_crypto_probe,
	.remove = s31_crypto_remove,
	.driver = {
		.name = "esp32s31-crypto",
		.of_match_table = s31_crypto_of_match,
	},
};
module_platform_driver(s31_crypto_driver);

MODULE_DESCRIPTION("ESP32-S31 AES, SHA, RSA/MPI and ECC Crypto API driver");
MODULE_LICENSE("GPL");
