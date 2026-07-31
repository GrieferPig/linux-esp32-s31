// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 AES and SHA Crypto API providers. */

#include <crypto/aes.h>
#include <crypto/algapi.h>
#include <crypto/hash.h>
#include <crypto/internal/hash.h>
#include <crypto/internal/skcipher.h>
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

/* HP_SYS_CLKRST.crypto_ctrl0, from the S31 IDF register definitions. */
#define S31_CRYPTO_SYS_CLK	BIT(0)
#define S31_CRYPTO_AES_CLK	BIT(4)
#define S31_CRYPTO_SHA_CLK	BIT(6)

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

static void s31_sha_config(struct s31_sha_ctx *ctx, struct crypto_shash *tfm)
{
	ctx->digestsize = crypto_shash_digestsize(tfm);
	ctx->blocksize = crypto_shash_blocksize(tfm);
	switch (ctx->digestsize) {
	case SHA1_DIGEST_SIZE:
		ctx->type = 0;
		break;
	case SHA224_DIGEST_SIZE:
		ctx->type = 1;
		break;
	case SHA256_DIGEST_SIZE:
		ctx->type = 2;
		break;
	case SHA384_DIGEST_SIZE:
		ctx->type = 3;
		break;
	default:
		ctx->type = 4;
		break;
	}
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

static int s31_sha_init(struct shash_desc *desc)
{
	struct s31_sha_ctx *ctx = shash_desc_ctx(desc);

	memset(ctx, 0, sizeof(*ctx));
	s31_sha_config(ctx, desc->tfm);
	ctx->first = true;
	return 0;
}

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

#define S31_SHA_ALG(_name, _driver, _digest, _block) { \
	.digestsize = _digest, .init = s31_sha_init, .update = s31_sha_update, \
	.final = s31_sha_final, .finup = s31_sha_finup, \
	.descsize = sizeof(struct s31_sha_ctx), \
	.base = { .cra_name = _name, .cra_driver_name = _driver, \
		.cra_priority = 300, .cra_blocksize = _block, .cra_module = THIS_MODULE } \
}

static struct shash_alg s31_sha_algs[] = {
	S31_SHA_ALG("sha1", "sha1-esp32s31", SHA1_DIGEST_SIZE, SHA1_BLOCK_SIZE),
	S31_SHA_ALG("sha224", "sha224-esp32s31", SHA224_DIGEST_SIZE, SHA224_BLOCK_SIZE),
	S31_SHA_ALG("sha256", "sha256-esp32s31", SHA256_DIGEST_SIZE, SHA256_BLOCK_SIZE),
	S31_SHA_ALG("sha384", "sha384-esp32s31", SHA384_DIGEST_SIZE, SHA384_BLOCK_SIZE),
	S31_SHA_ALG("sha512", "sha512-esp32s31", SHA512_DIGEST_SIZE, SHA512_BLOCK_SIZE),
};

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
	       S31_CRYPTO_SHA_CLK, dev->clkrst);
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
	platform_set_drvdata(pdev, dev);
	dev_info(&pdev->dev, "AES PIO, SHA %s Crypto API providers registered\n",
		 dev->sha_dma ? "DMA" : "PIO");
	return 0;
}

static void s31_crypto_remove(struct platform_device *pdev)
{
	struct s31_crypto_dev *dev = platform_get_drvdata(pdev);

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

MODULE_DESCRIPTION("ESP32-S31 AES/SHA Crypto API driver");
MODULE_LICENSE("GPL");
