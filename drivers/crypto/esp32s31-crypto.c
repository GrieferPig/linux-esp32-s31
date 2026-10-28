// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 Crypto API providers. */

#include <crypto/aes.h>
#include <crypto/akcipher.h>
#include <crypto/algapi.h>
#include <crypto/ecdh.h>
#include <crypto/engine.h>
#include <crypto/internal/ecc.h>
#include <crypto/internal/engine.h>
#include <crypto/hash.h>
#include <crypto/internal/akcipher.h>
#include <crypto/internal/hash.h>
#include <crypto/internal/kpp.h>
#include <crypto/internal/rsa.h>
#include <crypto/internal/skcipher.h>
#include <crypto/internal/sig.h>
#include <crypto/kpp.h>
#include <crypto/scatterwalk.h>
#include <crypto/sha1.h>
#include <crypto/sha2.h>
#include <crypto/utils.h>
#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/clk.h>
#include <linux/compat.h>
#include <linux/crc32.h>
#include <linux/fs.h>
#include <linux/highmem.h>
#include <linux/hrtimer.h>
#include <linux/miscdevice.h>
#include <linux/mpi.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <uapi/linux/esp32s31-crypto.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/of_platform.h>
#include <linux/device/driver.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/rtmutex.h>
#include <linux/sched.h>
#include <linux/soc/espressif/esp32s31-crypto.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#define S31_AES_KEY		0x000
#define S31_AES_TEXT_IN		0x020
#define S31_AES_TEXT_OUT	0x030
#define S31_AES_MODE		0x040
#define S31_AES_TRIGGER		0x048
#define S31_AES_STATE		0x04c
#define S31_AES_IDLE		0
#define S31_AES_DMA_DONE	2
#define S31_AES_IV		0x050
#define S31_AES_DMA_ENABLE	0x090
#define S31_AES_DMA_EXIT	0x0b8
#define S31_AES_BLOCK_MODE	0x094
#define S31_AES_BLOCK_NUM	0x098
#define S31_AES_INC_SEL		0x09c
#define S31_AES_INT_CLR		0x0ac
#define S31_AES_INT_ENA		0x0b0
#define S31_AES_RX_RESET		0x0c0
#define S31_AES_TX_RESET		0x0c4
#define S31_AES_DMA_THRESHOLD	PAGE_SIZE
#define S31_CRYPTO_DMA_MAX_BYTES	SZ_64K
#define S31_CRYPTO_DMA_MAX_NENTS	64
#define S31_AES_CACHE_LINE	64

#define S31_SHA_BASE		0x1000
#define S31_SHA_MODE		0x000
#define S31_SHA_START		0x010
#define S31_SHA_CONTINUE	0x014
#define S31_SHA_BUSY		0x018
#define S31_SHA_DMA_BLOCK_NUM	0x00c
#define S31_SHA_DMA_START	0x01c
#define S31_SHA_DMA_CONTINUE	0x020
#define S31_SHA_DMA_RX_RESET	0x030
#define S31_SHA_DMA_TX_RESET	0x034
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
#define S31_ECC_INT_ST		0x010
#define S31_ECC_INT_ENA		0x014
#define S31_ECC_INT_CLR		0x018
#define S31_ECC_CONF		0x01c
#define S31_ECC_K_MEM		0x100
#define S31_ECC_PX_MEM		0x130
#define S31_ECC_PY_MEM		0x160

#define S31_RSA_MAX_BYTES	512
#define S31_RSA_MAX_WORDS	(S31_RSA_MAX_BYTES / sizeof(u32))
#define S31_ECC_MAX_BYTES	48
/* Bound the staging path to one descriptor; direct DMA can use a chain. */
#define S31_SHA_BATCH_BYTES	(PAGE_SIZE - SHA512_BLOCK_SIZE)
#define S31_SHA_DMA_THRESHOLD	PAGE_SIZE

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
#define S31_CRYPTO_CLK_SEL	GENMASK(21, 20)
#define S31_CRYPTO_CLK_240M	BIT(21)
#define S31_CRYPTO_SYS_CLK	BIT(0)
#define S31_CRYPTO_SEC_CLK	BIT(2)
#define S31_CRYPTO_DS_CLK	BIT(10)
#define S31_CRYPTO_HMAC_CLK	BIT(14)
#define S31_CRYPTO_ECDSA_CLK	BIT(16)
#define S31_CRYPTO_EXTRA_CLOCKS (S31_CRYPTO_DS_CLK | S31_CRYPTO_HMAC_CLK | \
				 S31_CRYPTO_ECDSA_CLK)
#define S31_ECC_MEM_LP_EN	BIT(2)
#define S31_ECC_MEM_FORCE	BIT(3)
#define S31_CRYPTO_AES_CLK	BIT(4)
#define S31_CRYPTO_SHA_CLK	BIT(6)
#define S31_CRYPTO_RSA_CLK	BIT(8)
#define S31_CRYPTO_ECC_CLK	BIT(12)
#define S31_CRYPTO_AES_RST	BIT(5)
#define S31_CRYPTO_SHA_RST	BIT(7)
#define S31_CRYPTO_RSA_RST	BIT(9)
#define S31_CRYPTO_ECC_RST	BIT(13)
#define S31_CRYPTO_CLOCKS	(S31_CRYPTO_AES_CLK | S31_CRYPTO_SHA_CLK | \
				 S31_CRYPTO_RSA_CLK | S31_CRYPTO_ECC_CLK)
#define S31_CRYPTO_PARENT_RESETS (BIT(1) | BIT(11) | BIT(15) | BIT(17))

enum s31_aes_mode {
	S31_AES_ECB,
	S31_AES_CBC,
	S31_AES_CTR,
	S31_AES_CFB,
	S31_AES_OFB,
};

enum s31_request_type {
	S31_REQUEST_AES,
	S31_REQUEST_SHA,
	S31_REQUEST_RSA,
	S31_REQUEST_ECDH,
};

struct s31_crypto_dev;

struct s31_crypto_queue {
	struct s31_crypto_dev *dev;
	struct crypto_engine *engine;
	struct tasklet_struct done;
	struct crypto_async_request *completed;
	int error;
	enum s31_request_type type;
};

struct s31_crypto_dev {
	struct device *dev;
	struct clk *clock;
	struct miscdevice miscdev;
	void __iomem *base;
	void __iomem *clkrst;
	void __iomem *ecc_mem;
	void __iomem *key_manager;
	void __iomem *huk;
	struct esp32s31_crypto_km_huk huk_recovery;
	bool huk_owned;
	void __iomem *efuse_read_control;
	struct nvmem_cell *key_purposes;
	struct nvmem_cell *key_read_protect;
	struct dma_chan *sha_dma;
	struct completion ecc_done;
	int ecc_irq;
	u8 *sha_buffer;
	raw_spinlock_t clock_lock;
	struct rt_mutex engine_lock[S31_CRYPTO_DOMAINS];
	struct task_struct *engine_owner[S31_CRYPTO_DOMAINS];
	unsigned int engine_depth[S31_CRYPTO_DOMAINS];
	struct task_struct *domain_owner[S31_CRYPTO_DOMAINS];
	unsigned int domain_depth[S31_CRYPTO_DOMAINS];
	unsigned int sdk_clock_depth[S31_CRYPTO_SHA + 1];
	struct s31_crypto_queue bulk;
	struct s31_crypto_queue asym;
	atomic_t users;
	atomic_t failed_domains;
	wait_queue_head_t idle;
	bool suspending;
};

struct s31_aes_request {
	bool encrypt;
	enum s31_aes_mode mode;
};

struct s31_asym_request {
	bool private;
};

struct s31_aes_ctx {
	u8 key[AES_MAX_KEY_SIZE];
	u32 keylen;
};

struct s31_sha_ctx {
	u32 state[16];
	u64 total;
	u64 total_hi;
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
	bool have_key;
	bool have_private;
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
/* Releases remain valid after new admission closes during remove. */
static struct s31_crypto_dev *s31_crypto_provider;
static DEFINE_SPINLOCK(s31_crypto_admission);
static DEFINE_MUTEX(s31_crypto_lifecycle);

static void __s31_crypto_put(struct s31_crypto_dev *dev)
{
	if (atomic_dec_and_test(&dev->users))
		wake_up(&dev->idle);
}

static void s31_crypto_put(struct s31_crypto_dev *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&s31_crypto_admission, flags);
	__s31_crypto_put(dev);
	spin_unlock_irqrestore(&s31_crypto_admission, flags);
}

/* Algorithms can outlive device removal through an allocated transform. */
static void s31_crypto_alg_destroy(struct crypto_alg *alg);

static void s31_crypto_done(struct tasklet_struct *tasklet)
{
	struct s31_crypto_queue *queue = from_tasklet(queue, tasklet, done);
	struct crypto_async_request *req = queue->completed;
	int err = queue->error;

	switch (queue->type) {
	case S31_REQUEST_AES:
		crypto_finalize_skcipher_request(queue->engine,
			container_of(req, struct skcipher_request, base), err);
		break;
	case S31_REQUEST_SHA:
		crypto_finalize_hash_request(queue->engine,
			container_of(req, struct ahash_request, base), err);
		break;
	case S31_REQUEST_RSA:
		crypto_finalize_akcipher_request(queue->engine,
			container_of(req, struct akcipher_request, base), err);
		break;
	case S31_REQUEST_ECDH:
		crypto_finalize_kpp_request(queue->engine,
			container_of(req, struct kpp_request, base), err);
		break;
	}
	s31_crypto_put(queue->dev);
}

static int s31_crypto_execute(struct s31_crypto_dev *dev,
			      struct crypto_async_request *req,
			      enum s31_request_type type);

static int s31_crypto_submit(struct crypto_async_request *req,
			      enum s31_request_type type)
{
	struct s31_crypto_dev *dev;
	struct crypto_engine *engine;
	unsigned long flags;
	unsigned int held = 0, domain, i;
	bool may_sleep = (req->flags & CRYPTO_TFM_REQ_MAY_SLEEP) &&
			 !in_atomic() && !irqs_disabled();
	int ret;

	/* Serialize admission with removal, not with the hardware operation. */
	spin_lock_irqsave(&s31_crypto_admission, flags);
	dev = s31_crypto;
	if (!dev || dev->suspending) {
		ret = dev ? -ESHUTDOWN : -ENODEV;
		goto out;
	}
	if (may_sleep) {
		domain = type < S31_REQUEST_RSA ? S31_CRYPTO_BULK :
			type == S31_REQUEST_RSA ? S31_CRYPTO_MPI : S31_CRYPTO_ECC;
		for (i = 0; i < S31_CRYPTO_DOMAINS; i++)
			if (dev->engine_owner[i] == current)
				held |= BIT(i);
		/* A synchronous request cannot wait on its own SDK lease or
		 * acquire an earlier engine while holding a later one.
		 */
		if ((held & BIT(domain)) || (held && domain < fls(held) - 1)) {
			ret = -EDEADLK;
			goto out;
		}
	}
	atomic_inc(&dev->users);
	if (may_sleep) {
		spin_unlock_irqrestore(&s31_crypto_admission, flags);
		ret = s31_crypto_execute(dev, req, type);
		s31_crypto_put(dev);
		/* EBUSY denotes an admitted backlog to CryptoAPI waiters. No
		 * callback is pending for an operation completed in this caller.
		 */
		return ret == -EBUSY ? -EAGAIN : ret;
	}
	engine = type < S31_REQUEST_RSA ? dev->bulk.engine : dev->asym.engine;
	switch (type) {
	case S31_REQUEST_AES:
		ret = crypto_transfer_skcipher_request_to_engine(engine,
			container_of(req, struct skcipher_request, base));
		break;
	case S31_REQUEST_SHA:
		ret = crypto_transfer_hash_request_to_engine(engine,
			container_of(req, struct ahash_request, base));
		break;
	case S31_REQUEST_RSA:
		ret = crypto_transfer_akcipher_request_to_engine(engine,
			container_of(req, struct akcipher_request, base));
		break;
	case S31_REQUEST_ECDH:
		ret = crypto_transfer_kpp_request_to_engine(engine,
			container_of(req, struct kpp_request, base));
		break;
	default:
		ret = -EINVAL;
	}
	if (ret != -EINPROGRESS &&
	    !(ret == -EBUSY && (req->flags & CRYPTO_TFM_REQ_MAY_BACKLOG)))
		__s31_crypto_put(dev);
out:
	spin_unlock_irqrestore(&s31_crypto_admission, flags);
	return ret;
}

static void s31_crypto_complete(struct crypto_engine *engine,
			       struct crypto_async_request *req, int err,
			       enum s31_request_type type)
{
	struct s31_crypto_queue *queue = engine->priv_data;

	queue->completed = req;
	queue->error = err;
	queue->type = type;
	tasklet_schedule(&queue->done);
}

/* Only clock/reset RMW is shared; independent engines can execute together. */
static void s31_crypto_reset(struct s31_crypto_dev *dev, u32 reset)
{
	unsigned long flags;
	u32 value;

	raw_spin_lock_irqsave(&dev->clock_lock, flags);
	value = readl(dev->clkrst);
	writel(value | reset, dev->clkrst);
	readl(dev->clkrst);
	writel(value & ~reset, dev->clkrst);
	readl(dev->clkrst);
	raw_spin_unlock_irqrestore(&dev->clock_lock, flags);
}

/* Crypto operand windows and every supplied count are word-aligned.
 * Match memset_io's raw access width and ordering, but clear eight words
 * per loop. All original ranges and completion readbacks remain intact.
 */
static void __s31_clear_words(void __iomem *address, unsigned int bytes)
{
	/* Use a single base register for the eight RV32 MMIO stores. */
#define S31_CLEAR_WORD(offset) \
	asm volatile("sw zero, %c1(%0)" : : "r"(address), \
		     "i"(offset) : "memory")
	while (bytes >= 32) {
		S31_CLEAR_WORD(0); S31_CLEAR_WORD(4);
		S31_CLEAR_WORD(8); S31_CLEAR_WORD(12);
		S31_CLEAR_WORD(16); S31_CLEAR_WORD(20);
		S31_CLEAR_WORD(24); S31_CLEAR_WORD(28);
		address += 32;
		bytes -= 32;
	}
#undef S31_CLEAR_WORD
	while (bytes) {
		__raw_writel(0, address);
		address += sizeof(u32);
		bytes -= sizeof(u32);
	}
}

#define s31_clear_words(address, bytes) do { \
	BUILD_BUG_ON(__builtin_constant_p(bytes) && (bytes) % sizeof(u32)); \
	__s31_clear_words(address, bytes); \
} while (0)

#include "esp32s31-crypto-owner.inc"

/* Most PIO operations finish before the first status read. Avoid setting up
 * a timeout clock in that case; a busy engine retains the bounded poll and
 * the caller's normal reset/quarantine path.
 */
static int s31_crypto_poll(void __iomem *status, u32 mask, u32 ready,
			   unsigned int sleep_us, unsigned int timeout_us)
{
	u32 value = readl(status);

	if ((value & mask) == ready)
		return 0;
	return readl_poll_timeout(status, value, (value & mask) == ready,
				  sleep_us, timeout_us);
}

/* Reset failure must not release an engine whose private scratch or stop state
 * is unproven. Callers hold its PI lock; independent domains remain usable.
 */
static int s31_crypto_reset_wait(struct s31_crypto_dev *dev, u32 reset,
				void __iomem *status, u32 mask, u32 ready,
				unsigned int domain)
{
	int ret;

	if (atomic_read(&dev->failed_domains) & BIT(domain))
		return -EIO;
	s31_crypto_reset(dev, reset);
	ret = s31_crypto_poll(status, mask, ready, 20, 10000);
	if (ret)
		s31_crypto_fault(dev, BIT(domain));
	return ret;
}

static void s31_write_words_impl(void __iomem *base, unsigned int offset,
				 const u8 *src, unsigned int len,
				 bool defer_mmiowb_to_command)
{
	void __iomem *dst = base + offset;

	/* Callers own the engine's operand window. Keep 32-bit accesses and
	 * the same memory/IO boundaries as __iowrite32_copy, but amortize loop
	 * bookkeeping across eight aligned words. Control registers retain
	 * readl/writel and unaligned transfers retain memcpy_toio.
	 */
	wmb();
	if (IS_ALIGNED((unsigned long)src | offset | len, sizeof(u32))) {
		const u32 *words = (const u32 *)src;

		/* Keep one MMIO base register for the eight RV32 word stores. */
#define S31_WRITE_WORD(n) \
		asm volatile("sw %1, %c2(%0)" : : "r"(dst), \
			     "r"(words[n]), "i"((n) * sizeof(u32)) : "memory")
		while (len >= 32) {
			S31_WRITE_WORD(0); S31_WRITE_WORD(1);
			S31_WRITE_WORD(2); S31_WRITE_WORD(3);
			S31_WRITE_WORD(4); S31_WRITE_WORD(5);
			S31_WRITE_WORD(6); S31_WRITE_WORD(7);
			words += 8;
			dst += 32;
			len -= 32;
		}
#undef S31_WRITE_WORD
		while (len) {
			__raw_writel(*words++, dst);
			dst += sizeof(u32);
			len -= sizeof(u32);
		}
	} else {
		memcpy_toio(dst, src, len);
	}
	wmb();
	/* Deferral is valid only when an unconditional ordered MMIO command
	 * immediately follows, with no intervening lock release or error path.
	 */
	if (!defer_mmiowb_to_command)
		mmiowb_set_pending();
}

static void s31_write_words(void __iomem *base, unsigned int offset,
			    const u8 *src, unsigned int len)
{
	s31_write_words_impl(base, offset, src, len, false);
}

static void s31_read_words(void __iomem *base, unsigned int offset,
			   u8 *dst, unsigned int len)
{
	const void __iomem *src = base + offset;

	/* Aligned operands need neither byte assembly nor per-word loop
	 * bookkeeping. Preserve memcpy_fromio's access width and final rmb.
	 */
	if (IS_ALIGNED((unsigned long)dst | offset | len, sizeof(u32))) {
		u32 *words = (u32 *)dst;

		while (len >= 32) {
			words[0] = __raw_readl(src);
			words[1] = __raw_readl(src + 4);
			words[2] = __raw_readl(src + 8);
			words[3] = __raw_readl(src + 12);
			words[4] = __raw_readl(src + 16);
			words[5] = __raw_readl(src + 20);
			words[6] = __raw_readl(src + 24);
			words[7] = __raw_readl(src + 28);
			words += 8;
			src += 32;
			len -= 32;
		}
		while (len) {
			*words++ = __raw_readl(src);
			src += sizeof(u32);
			len -= sizeof(u32);
		}
	} else {
		memcpy_fromio(dst, src, len);
	}
	rmb();
}

/* The S31 DMA FIFOs are outside the ordinary AES/SHA engine reset. An
 * aborted transfer can leave input queued for the next request, even after
 * resetting the engine and the GDMA controller. Call only with BULK ownership
 * (or before probe publishes the provider), after stopping and joining DMA.
 */
static void s31_aes_dma_reset_fifos(struct s31_crypto_dev *dev)
{
	writel(1, dev->base + S31_AES_RX_RESET);
	writel(1, dev->base + S31_AES_TX_RESET);
	readl(dev->base + S31_AES_STATE);
}

static void s31_sha_dma_reset_fifos(struct s31_crypto_dev *dev)
{
	void __iomem *sha = dev->base + S31_SHA_BASE;

	writel(1, sha + S31_SHA_DMA_RX_RESET);
	writel(1, sha + S31_SHA_DMA_TX_RESET);
	readl(sha + S31_SHA_BUSY);
}

static int s31_aes_block(struct s31_crypto_dev *dev, const u8 *src, u8 *dst)
{
	void __iomem *aes;
	int ret;

	if (!dev)
		return -ENODEV;
	aes = dev->base;
	s31_write_words(aes, S31_AES_TEXT_IN, src, AES_BLOCK_SIZE);
	writel(1, aes + S31_AES_TRIGGER);
	ret = s31_crypto_poll(aes + S31_AES_STATE, U32_MAX,
			      S31_AES_IDLE, 0, 1000);
	if (!ret)
		s31_read_words(aes, S31_AES_TEXT_OUT, dst, AES_BLOCK_SIZE);
	if (ret && s31_crypto_reset_wait(dev, S31_CRYPTO_AES_RST,
					 aes + S31_AES_STATE, U32_MAX,
					 S31_AES_IDLE, S31_CRYPTO_BULK))
		return ret;
	return ret;
}

static int s31_aes_setkey(struct crypto_skcipher *tfm, const u8 *key,
			  unsigned int keylen)
{
	struct s31_aes_ctx *ctx = crypto_skcipher_ctx(tfm);

	memzero_explicit(ctx->key, sizeof(ctx->key));
	ctx->keylen = 0;
	if (keylen != AES_KEYSIZE_128 && keylen != AES_KEYSIZE_256)
		return -EINVAL;
	memcpy(ctx->key, key, keylen);
	ctx->keylen = keylen;
	return 0;
}

static void s31_aes_exit(struct crypto_skcipher *tfm)
{
	memzero_explicit(crypto_skcipher_ctx(tfm), sizeof(struct s31_aes_ctx));
}

/* Two ordinary DMAengine slave channels carry the AES input and output.
 * The shared BULK lease also owns sha_buffer, so only one extra bounce page
 * is needed. Isolated pages avoid DMA cache-line aliasing with arbitrary
 * user/kernel scatterlist offsets, including in-place requests.
 */
struct s31_dma_wait {
	struct completion done;
	enum dmaengine_tx_result result;
};

static void s31_dma_complete(void *arg, const struct dmaengine_result *result)
{
	struct s31_dma_wait *wait = arg;

	wait->result = result ? result->result : DMA_TRANS_ABORTED;
	complete(&wait->done);
}

static int s31_dma_wait_done(struct s31_dma_wait *wait)
{
	if (!wait_for_completion_timeout(&wait->done, msecs_to_jiffies(1000)))
		return -ETIMEDOUT;
	return wait->result == DMA_TRANS_NOERROR ? 0 : -EIO;
}

static int s31_aes_dma_submit(struct s31_crypto_dev *dev,
			     struct dma_chan *tx, struct dma_chan *rx,
			     struct scatterlist *src, unsigned int src_nents,
			     struct scatterlist *dst, unsigned int dst_nents,
			     unsigned int len)
{
	struct dma_async_tx_descriptor *desc;
	struct s31_dma_wait tx_wait = {}, rx_wait = {};
	dma_cookie_t tx_cookie, rx_cookie;
	u32 state;
	bool dma_started = false;
	int ret;

	init_completion(&rx_wait.done);
	init_completion(&tx_wait.done);
	desc = dmaengine_prep_slave_sg(rx, dst, dst_nents, DMA_DEV_TO_MEM,
				       DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
	if (!desc) {
		ret = -ENOMEM;
		goto out;
	}
	desc->callback_result = s31_dma_complete;
	desc->callback_param = &rx_wait;
	rx_cookie = dmaengine_submit(desc);
	ret = dma_submit_error(rx_cookie);
	if (ret)
		goto out;
	desc = dmaengine_prep_slave_sg(tx, src, src_nents, DMA_MEM_TO_DEV,
				       DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
	if (!desc) {
		ret = -ENOMEM;
		goto out;
	}
	desc->callback_result = s31_dma_complete;
	desc->callback_param = &tx_wait;
	tx_cookie = dmaengine_submit(desc);
	ret = dma_submit_error(tx_cookie);
	if (ret)
		goto out;

	dma_async_issue_pending(rx);
	dma_async_issue_pending(tx);
	writel(1, dev->base + S31_AES_DMA_ENABLE); /* AES_DMA_ENABLE */
	dma_started = true;
	writel(len / AES_BLOCK_SIZE, dev->base + S31_AES_BLOCK_NUM); /* AES_BLOCK_NUM */
	writel(1, dev->base + S31_AES_TRIGGER);
	ret = s31_dma_wait_done(&rx_wait);
	if (ret)
		goto out;
	ret = s31_dma_wait_done(&tx_wait);
	if (ret)
		goto out;
	ret = readl_poll_timeout(dev->base + S31_AES_STATE, state,
				state == S31_AES_DMA_DONE, 10, 100000);
	if (!ret) {
		/* Match the S31 IDF DMA finish sequence before the next reset. */
		writel(0, dev->base + S31_AES_DMA_EXIT);
		writel(0, dev->base + S31_AES_DMA_ENABLE);
	}
out:
	/* A queued RX also needs cancellation if TX preparation failed. Stop
	 * and join both channels before returning either page to the CPU.
	 */
	if (ret) {
		dmaengine_terminate_sync(rx);
		dmaengine_terminate_sync(tx);
		if (dma_started) {
			writel(0, dev->base + S31_AES_DMA_EXIT);
			writel(0, dev->base + S31_AES_DMA_ENABLE);
		}
		s31_aes_dma_reset_fifos(dev);
	}
	return ret;
}

static int s31_aes_dma_block(struct s31_crypto_dev *dev,
			   struct dma_chan *tx, struct dma_chan *rx,
			   u8 *output, unsigned int len)
{
	struct device *dma_dev = dmaengine_get_dma_device(tx);
	struct scatterlist src, dst;
	dma_addr_t input_dma, output_dma;
	int ret;

	input_dma = dma_map_single(dma_dev, dev->sha_buffer, len, DMA_TO_DEVICE);
	if (dma_mapping_error(dma_dev, input_dma))
		return -EIO;
	output_dma = dma_map_single(dma_dev, output, len, DMA_FROM_DEVICE);
	if (dma_mapping_error(dma_dev, output_dma)) {
		ret = -EIO;
		goto unmap_input;
	}
	sg_init_table(&src, 1);
	sg_dma_address(&src) = input_dma;
	sg_dma_len(&src) = len;
	sg_init_table(&dst, 1);
	sg_dma_address(&dst) = output_dma;
	sg_dma_len(&dst) = len;
	ret = s31_aes_dma_submit(dev, tx, rx, &src, 1, &dst, 1, len);
	dma_unmap_single(dma_dev, output_dma, len, DMA_FROM_DEVICE);
unmap_input:
	dma_unmap_single(dma_dev, input_dma, len, DMA_TO_DEVICE);
	return ret;
}

struct s31_aes_dma_edge {
	unsigned int offset;
	unsigned int len;
};

static void s31_aes_counter_add(u8 *iv, unsigned int blocks)
{
	int i;

	for (i = AES_BLOCK_SIZE - 1; i >= 0 && blocks; i--) {
		blocks += iv[i];
		iv[i] = blocks;
		blocks >>= 8;
	}
}

/* Adjacent physical pages need one cache-maintenance range, not one SBI
 * call per page. The request bound also bounds the resulting DMA segment.
 */
static void s31_dma_append(struct scatterlist *list, unsigned int *count,
			   struct page *page, unsigned int offset, unsigned int len)
{
	if (*count) {
		struct scatterlist *last = &list[*count - 1];

		if (sg_phys(last) + last->length == page_to_phys(page) + offset) {
			last->length += len;
			return;
		}
	}
	sg_set_page(&list[(*count)++], page, len, offset);
}

/* Whole-request DMA removes the staging copy and per-page restart. Keep
 * partial destination cache lines in private buffers: cache invalidation
 * must never discard a concurrent CPU write outside the caller's output.
 * The source and destination scatterlists remain owned by the caller.
 */
static int s31_aes_dma_direct(struct s31_crypto_dev *dev,
			    struct skcipher_request *req,
			    struct dma_chan *tx, struct dma_chan *rx,
			    bool encrypt, enum s31_aes_mode mode, bool *used)
{
	struct s31_aes_ctx *ctx = crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));
	struct device *dma_dev = dmaengine_get_dma_device(tx);
	struct scatterlist *src = NULL, *dst = NULL, *sg;
	struct s31_aes_dma_edge *edges = NULL;
	u8 *allocation = NULL, *edge_data;
	u8 iv[AES_BLOCK_SIZE];
	unsigned int src_count, dst_count, left, n, offset, head, middle;
	unsigned int i, src_used = 0, nr = 0, edge_count = 0, edge_capacity = 0;
	int src_mapped = 0, dst_mapped = 0, ret = 0;

	*used = false;
	if (req->cryptlen > S31_CRYPTO_DMA_MAX_BYTES ||
	    !IS_ALIGNED(req->cryptlen, AES_BLOCK_SIZE))
		return 0;
	ret = sg_nents_for_len(req->src, req->cryptlen);
	if (ret <= 0 || ret > S31_CRYPTO_DMA_MAX_NENTS)
		return 0;
	src_count = ret;
	ret = sg_nents_for_len(req->dst, req->cryptlen);
	if (ret <= 0 || ret > S31_CRYPTO_DMA_MAX_NENTS)
		return 0;
	dst_count = ret;
	for (i = 0; i < 2; i++) {
		unsigned int count = i ? dst_count : src_count;

		left = req->cryptlen;
		for_each_sg(i ? req->dst : req->src, sg, count, offset) {
			n = min(left, sg->length);
			if (!n || !IS_ALIGNED(sg->offset | n, sizeof(u32)))
				return 0;
			left -= n;
			if (i) {
				head = min(n, (S31_AES_CACHE_LINE - sg->offset) &
						(S31_AES_CACHE_LINE - 1));
				edge_capacity += !!head;
				edge_capacity += !!((n - head) & (S31_AES_CACHE_LINE - 1));
			}
		}
	}
	/* This representation is chosen before touching the hardware. Allocation
	 * pressure can still use the existing bounded staging path.
	 */
	allocation = kmalloc((src_count + dst_count + edge_capacity) * sizeof(*src) +
			     edge_capacity * (sizeof(*edges) + S31_AES_CACHE_LINE) +
			     S31_AES_CACHE_LINE - 1, GFP_NOWAIT);
	if (!allocation)
		goto free;
	src = (struct scatterlist *)allocation;
	dst = src + src_count;
	edges = (struct s31_aes_dma_edge *)(dst + dst_count + edge_capacity);
	edge_data = PTR_ALIGN((u8 *)(edges + edge_capacity), S31_AES_CACHE_LINE);
	memset(edge_data, 0, edge_capacity * S31_AES_CACHE_LINE);
	sg_init_table(src, src_count);
	sg_init_table(dst, dst_count + edge_capacity);
	left = req->cryptlen;
	for_each_sg(req->src, sg, src_count, i) {
		n = min(left, sg->length);
		s31_dma_append(src, &src_used, sg_page(sg), sg->offset, n);
		left -= n;
	}
	sg_mark_end(&src[src_used - 1]);
	left = req->cryptlen;
	offset = 0;
	for_each_sg(req->dst, sg, dst_count, i) {
		unsigned int pos = sg->offset;

		n = min(left, sg->length);
		left -= n;
		head = min(n, (S31_AES_CACHE_LINE - pos) &
				(S31_AES_CACHE_LINE - 1));
		if (head) {
			edges[edge_count] = (struct s31_aes_dma_edge) { offset, head };
			sg_set_buf(&dst[nr++], edge_data +
				   edge_count++ * S31_AES_CACHE_LINE, head);
			pos += head;
			offset += head;
			n -= head;
		}
		middle = round_down(n, S31_AES_CACHE_LINE);
		if (middle) {
			s31_dma_append(dst, &nr, sg_page(sg), pos, middle);
			offset += middle;
			n -= middle;
		}
		if (n) {
			edges[edge_count] = (struct s31_aes_dma_edge) { offset, n };
			sg_set_buf(&dst[nr++], edge_data +
				   edge_count++ * S31_AES_CACHE_LINE, n);
			offset += n;
		}
	}
	sg_mark_end(&dst[nr - 1]);
	*used = true;
	if (mode == S31_AES_CBC && !encrypt)
		scatterwalk_map_and_copy(iv, req->src,
				       req->cryptlen - AES_BLOCK_SIZE, AES_BLOCK_SIZE, 0);
	src_mapped = dma_map_sg(dma_dev, src, src_used, DMA_TO_DEVICE);
	if (!src_mapped) {
		ret = -EIO;
		goto free;
	}
	dst_mapped = dma_map_sg(dma_dev, dst, nr, DMA_FROM_DEVICE);
	if (!dst_mapped) {
		ret = -EIO;
		goto unmap;
	}
	ret = s31_crypto_reset_wait(dev, S31_CRYPTO_AES_RST,
		dev->base + S31_AES_STATE, U32_MAX, S31_AES_IDLE, S31_CRYPTO_BULK);
	if (!ret) {
		s31_write_words(dev->base, S31_AES_KEY, ctx->key, ctx->keylen);
		writel(((encrypt || mode == S31_AES_CTR) && mode != S31_AES_OFB ? 0 : BIT(2)) +
		       ctx->keylen / 8 - 2, dev->base + S31_AES_MODE);
		writel(mode == S31_AES_CTR ? 3 : mode == S31_AES_CFB ? 5 :
		       mode == S31_AES_OFB ? 2 : mode, dev->base + S31_AES_BLOCK_MODE);
		writel(1, dev->base + S31_AES_INC_SEL);
		writel(0, dev->base + S31_AES_INT_ENA);
		writel(1, dev->base + S31_AES_INT_CLR);
		if (mode != S31_AES_ECB)
			s31_write_words(dev->base, S31_AES_IV, req->iv, AES_BLOCK_SIZE);
		ret = s31_aes_dma_submit(dev, tx, rx, src, src_mapped,
					 dst, dst_mapped, req->cryptlen);
		if (!ret && (mode == S31_AES_CFB || mode == S31_AES_OFB))
			s31_read_words(dev->base, S31_AES_IV, req->iv, AES_BLOCK_SIZE);
	}
	dma_unmap_sg(dma_dev, dst, nr, DMA_FROM_DEVICE);
unmap:
	dma_unmap_sg(dma_dev, src, src_used, DMA_TO_DEVICE);
	if (!ret) {
		for (i = 0; i < edge_count; i++)
			scatterwalk_map_and_copy(edge_data + i * S31_AES_CACHE_LINE,
					req->dst, edges[i].offset, edges[i].len, 1);
		if (mode == S31_AES_CBC) {
			if (encrypt)
				scatterwalk_map_and_copy(iv, req->dst,
					req->cryptlen - AES_BLOCK_SIZE, AES_BLOCK_SIZE, 0);
			memcpy(req->iv, iv, AES_BLOCK_SIZE);
		} else if (mode == S31_AES_CTR) {
			s31_aes_counter_add(req->iv, req->cryptlen / AES_BLOCK_SIZE);
		}
	}
free:
	memzero_explicit(iv, sizeof(iv));
	kfree_sensitive(allocation);
	return *used ? ret : 0;
}

static int s31_aes_dma_crypt(struct s31_crypto_dev *dev,
			   struct skcipher_request *req, bool encrypt,
			   enum s31_aes_mode mode, bool *used)
{
	struct s31_aes_ctx *ctx = crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));
	struct dma_slave_config config = {
		.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.src_maxburst = 4, .dst_maxburst = 2,
	};
	struct dma_chan *tx, *rx;
	u8 iv[AES_BLOCK_SIZE], *output;
	unsigned int offset = 0, n, padded;
	bool direct;
	int ret;

	*used = false;
	/* A busy shared GDMA controller must not reserve two fixed pairs away
	 * from other peripherals. PIO remains the native short/busy path.
	 */
	tx = of_dma_request_slave_channel(dev->dev->of_node, "aes-tx");
	if (IS_ERR(tx))
		return 0;
	rx = of_dma_request_slave_channel(dev->dev->of_node, "aes-rx");
	if (IS_ERR(rx)) {
		dma_release_channel(tx);
		return 0;
	}
	*used = true;
	if (dmaengine_get_dma_device(tx) != dmaengine_get_dma_device(rx)) {
		ret = -EINVAL;
		goto release;
	}
	ret = dmaengine_slave_config(tx, &config);
	if (!ret)
		ret = dmaengine_slave_config(rx, &config);
	if (ret)
		goto release;
	ret = s31_aes_dma_direct(dev, req, tx, rx, encrypt, mode, &direct);
	if (direct)
		goto release;
	/* Do not enter reclaim while holding a radio-shared engine lease. */
	output = kmalloc(PAGE_SIZE, GFP_NOWAIT);
	if (!output) {
		ret = -ENOMEM;
		goto release;
	}
	if (mode != S31_AES_ECB)
		memcpy(iv, req->iv, AES_BLOCK_SIZE);

	while (offset < req->cryptlen) {
		n = min_t(unsigned int, PAGE_SIZE, req->cryptlen - offset);
		padded = ALIGN(n, AES_BLOCK_SIZE);
		scatterwalk_map_and_copy(dev->sha_buffer, req->src, offset, n, 0);
		memset(dev->sha_buffer + n, 0, padded - n);
		ret = s31_crypto_reset_wait(dev, S31_CRYPTO_AES_RST,
			dev->base + S31_AES_STATE, U32_MAX, S31_AES_IDLE,
			S31_CRYPTO_BULK);
		if (ret)
			break;
		s31_write_words(dev->base, S31_AES_KEY, ctx->key, ctx->keylen);
		writel(((encrypt || mode == S31_AES_CTR) && mode != S31_AES_OFB ? 0 : BIT(2)) +
		       ctx->keylen / 8 - 2, dev->base + S31_AES_MODE);
		writel(mode == S31_AES_CTR ? 3 : mode == S31_AES_CFB ? 5 :
		       mode == S31_AES_OFB ? 2 : mode, dev->base + S31_AES_BLOCK_MODE);
		writel(1, dev->base + S31_AES_INC_SEL); /* CTR uses INC128, not IDF's INC32. */
		writel(0, dev->base + S31_AES_INT_ENA); /* Completion comes from the DMA channel IRQs. */
		writel(1, dev->base + S31_AES_INT_CLR);
		if (mode != S31_AES_ECB)
			s31_write_words(dev->base, S31_AES_IV, iv, AES_BLOCK_SIZE);
		ret = s31_aes_dma_block(dev, tx, rx, output, padded);
		if (ret)
			break;
		if (mode == S31_AES_CBC)
			memcpy(iv, (encrypt ? output : dev->sha_buffer) +
			       n - AES_BLOCK_SIZE, AES_BLOCK_SIZE);
		else if (mode == S31_AES_CTR)
			s31_aes_counter_add(iv, padded / AES_BLOCK_SIZE);
		else if (mode == S31_AES_CFB || mode == S31_AES_OFB)
			s31_read_words(dev->base, S31_AES_IV, iv, AES_BLOCK_SIZE);
		scatterwalk_map_and_copy(output, req->dst, offset, n, 1);
		offset += n;
		cond_resched();
	}
	if (mode != S31_AES_ECB)
		memcpy(req->iv, iv, AES_BLOCK_SIZE);
	memzero_explicit(iv, sizeof(iv));
	memzero_explicit(dev->sha_buffer, PAGE_SIZE);
	kfree_sensitive(output);
release:
	dma_release_channel(rx);
	dma_release_channel(tx);
	return ret;
}

static int s31_aes_crypt(struct s31_crypto_dev *dev,
			 struct skcipher_request *req, bool encrypt,
			 enum s31_aes_mode mode)
{
	struct s31_aes_ctx *ctx = crypto_skcipher_ctx(
		crypto_skcipher_reqtfm(req));
	u8 input[AES_BLOCK_SIZE], output[AES_BLOCK_SIZE], iv[AES_BLOCK_SIZE];
	unsigned int offset, n;
	int ret = 0;

	if (!ctx->keylen)
		return -ENOKEY;
	if ((mode == S31_AES_ECB || mode == S31_AES_CBC) &&
	    req->cryptlen & (AES_BLOCK_SIZE - 1))
		return -EINVAL;
	/* DMA setup is relatively expensive for feedback modes: measured
	 * 4 KiB CFB/OFB requests are faster through the block interface.
	 * Use whole-block DMA from 8 KiB, leaving partial requests to PIO.
	 */
	if (req->cryptlen >= S31_AES_DMA_THRESHOLD &&
	    ((mode != S31_AES_CFB && mode != S31_AES_OFB) ||
	     (req->cryptlen >= 2 * S31_AES_DMA_THRESHOLD &&
	      IS_ALIGNED(req->cryptlen, AES_BLOCK_SIZE)))) {
		bool used;

		ret = s31_aes_dma_crypt(dev, req, encrypt, mode, &used);
		if (used)
			return ret;
	}
	if (mode != S31_AES_ECB)
		memcpy(iv, req->iv, AES_BLOCK_SIZE);
	/* The BULK lease covers this whole request, including rescheduling.
	 * Program one key/mode for its blocks; s31_aes_run resets the engine
	 * before releasing ownership on every success or failure.
	 */
	s31_write_words(dev->base, S31_AES_KEY, ctx->key, ctx->keylen);
	writel(((encrypt || mode == S31_AES_CTR || mode == S31_AES_CFB ||
		 mode == S31_AES_OFB) ? 0 : BIT(2)) +
	       ctx->keylen / 8 - 2, dev->base + S31_AES_MODE);

	for (offset = 0; offset < req->cryptlen; offset += n) {
		n = min_t(unsigned int, AES_BLOCK_SIZE, req->cryptlen - offset);
		scatterwalk_map_and_copy(input, req->src, offset, n, 0);
		if (mode == S31_AES_CTR) {
			ret = s31_aes_block(dev, iv, output);
			if (!ret) {
				crypto_xor(output, input, n);
				crypto_inc(iv, AES_BLOCK_SIZE);
			}
		} else if (mode == S31_AES_CFB || mode == S31_AES_OFB) {
			ret = s31_aes_block(dev, iv, output);
			if (!ret) {
				if (mode == S31_AES_OFB)
					memcpy(iv, output, AES_BLOCK_SIZE);
				crypto_xor(output, input, n);
				if (mode == S31_AES_CFB)
					memcpy(iv, encrypt ? output : input, n);
			}
		} else if (mode == S31_AES_CBC && encrypt) {
			crypto_xor(input, iv, AES_BLOCK_SIZE);
			ret = s31_aes_block(dev, input, output);
			if (!ret)
				memcpy(iv, output, AES_BLOCK_SIZE);
		} else if (mode == S31_AES_CBC) {
			ret = s31_aes_block(dev, input, output);
			if (!ret)
				crypto_xor(output, iv, AES_BLOCK_SIZE);
			memcpy(iv, input, AES_BLOCK_SIZE);
		} else {
			ret = s31_aes_block(dev, input, output);
		}
		if (ret)
			break;
		scatterwalk_map_and_copy(output, req->dst, offset, n, 1);
		if (!(offset & (PAGE_SIZE - 1)))
			cond_resched();
	}
	if (mode != S31_AES_ECB)
		memcpy(req->iv, iv, AES_BLOCK_SIZE);
	memzero_explicit(input, sizeof(input));
	memzero_explicit(output, sizeof(output));
	memzero_explicit(iv, sizeof(iv));
	return ret;
}

static int s31_aes_enqueue(struct skcipher_request *req, bool encrypt,
			   enum s31_aes_mode mode)
{
	struct s31_aes_request *rctx = skcipher_request_ctx(req);

	rctx->encrypt = encrypt;
	rctx->mode = mode;
	return s31_crypto_submit(&req->base, S31_REQUEST_AES);
}

static int s31_aes_run(struct s31_crypto_dev *dev, struct skcipher_request *req)
{
	struct s31_aes_request *rctx = skcipher_request_ctx(req);
	int ret;

	ret = pm_runtime_resume_and_get(dev->dev);
	if (ret >= 0) {
		rt_mutex_lock(&dev->engine_lock[S31_CRYPTO_BULK]);
		if (atomic_read(&dev->failed_domains) & BIT(S31_CRYPTO_BULK)) {
			ret = -EIO;
			goto unlock;
		}
		ret = s31_aes_crypt(dev, req, rctx->encrypt, rctx->mode);
		/* Result latches ignore clearing writes on S31 silicon. Reset after
		 * copying the output, while the engine is still exclusively owned.
		 */
		{
			int reset_ret = s31_crypto_reset_wait(dev,
				S31_CRYPTO_AES_RST, dev->base + S31_AES_STATE,
				U32_MAX, S31_AES_IDLE, S31_CRYPTO_BULK);

			if (!reset_ret) {
				/* Clear writable key and IV words while ownership is
				 * held, even when reset leaves their latches intact.
				 */
				s31_clear_words(dev->base + S31_AES_KEY, AES_KEYSIZE_256);
				s31_clear_words(dev->base + S31_AES_IV, AES_BLOCK_SIZE);
				wmb();
				readl(dev->base + S31_AES_STATE);
			}
			if (reset_ret)
				ret = reset_ret;
		}
unlock:
		rt_mutex_unlock(&dev->engine_lock[S31_CRYPTO_BULK]);
		pm_runtime_mark_last_busy(dev->dev);
		pm_runtime_put_autosuspend(dev->dev);
	}
	return ret;
}

static int s31_aes_do_one(struct crypto_engine *engine, void *areq)
{
	struct s31_crypto_queue *queue = engine->priv_data;
	struct skcipher_request *req = container_of(areq, struct skcipher_request, base);
	int ret = s31_aes_run(queue->dev, req);

	s31_crypto_complete(engine, &req->base, ret, S31_REQUEST_AES);
	return 0;
}

static int s31_aes_init(struct crypto_skcipher *tfm)
{
	crypto_skcipher_set_reqsize(tfm, sizeof(struct s31_aes_request));
	return 0;
}

#define S31_AES_CRYPT(_name, _enc, _mode) \
static int _name(struct skcipher_request *req) \
{ return s31_aes_enqueue(req, _enc, _mode); }

S31_AES_CRYPT(s31_ecb_encrypt, true, S31_AES_ECB)
S31_AES_CRYPT(s31_ecb_decrypt, false, S31_AES_ECB)
S31_AES_CRYPT(s31_cbc_encrypt, true, S31_AES_CBC)
S31_AES_CRYPT(s31_cbc_decrypt, false, S31_AES_CBC)
S31_AES_CRYPT(s31_ctr_crypt, true, S31_AES_CTR)
S31_AES_CRYPT(s31_cfb_encrypt, true, S31_AES_CFB)
S31_AES_CRYPT(s31_cfb_decrypt, false, S31_AES_CFB)
S31_AES_CRYPT(s31_ofb_crypt, true, S31_AES_OFB)

#define S31_AES_ALG(_name, _driver, _enc, _dec, _ivsize, _block) { .base = { \
	.setkey = s31_aes_setkey, .encrypt = _enc, .decrypt = _dec, \
	.init = s31_aes_init, .exit = s31_aes_exit, \
	.min_keysize = AES_KEYSIZE_128, .max_keysize = AES_KEYSIZE_256, \
	.ivsize = _ivsize, .chunksize = AES_BLOCK_SIZE, .walksize = AES_BLOCK_SIZE, \
	.base = { .cra_name = _name, .cra_driver_name = _driver, \
		.cra_priority = 300, .cra_blocksize = _block, \
		.cra_flags = CRYPTO_ALG_ASYNC, \
		.cra_ctxsize = sizeof(struct s31_aes_ctx), .cra_module = THIS_MODULE } }, \
	.op.do_one_request = s31_aes_do_one \
}

static struct skcipher_engine_alg s31_aes_algs[] = {
	S31_AES_ALG("ecb(aes)", "ecb-aes-esp32s31", s31_ecb_encrypt,
		    s31_ecb_decrypt, 0, AES_BLOCK_SIZE),
	S31_AES_ALG("cbc(aes)", "cbc-aes-esp32s31", s31_cbc_encrypt,
		    s31_cbc_decrypt, AES_BLOCK_SIZE, AES_BLOCK_SIZE),
	S31_AES_ALG("ctr(aes)", "ctr-aes-esp32s31", s31_ctr_crypt,
		    s31_ctr_crypt, AES_BLOCK_SIZE, 1),
	S31_AES_ALG("cfb(aes)", "cfb-aes-esp32s31", s31_cfb_encrypt,
		    s31_cfb_decrypt, AES_BLOCK_SIZE, 1),
	S31_AES_ALG("ofb(aes)", "ofb-aes-esp32s31", s31_ofb_crypt,
		    s31_ofb_crypt, AES_BLOCK_SIZE, 1),
};

enum s31_sha_operation {
	S31_SHA_UPDATE,
	S31_SHA_FINAL,
	S31_SHA_FINUP,
};

struct s31_sha_request {
	struct s31_sha_ctx hash;
	enum s31_sha_operation operation;
	bool initialized;
};

static void s31_sha_config(struct s31_sha_ctx *ctx, struct crypto_ahash *tfm)
{
	ctx->digestsize = crypto_ahash_digestsize(tfm);
	ctx->blocksize = crypto_ahash_blocksize(tfm);
	if (ctx->blocksize == SHA256_BLOCK_SIZE)
		ctx->type = ctx->digestsize == SHA1_DIGEST_SIZE ? 0 :
			    ctx->digestsize == SHA224_DIGEST_SIZE ? 1 : 2;
	else
		ctx->type = ctx->digestsize == SHA384_DIGEST_SIZE ? 3 :
			    ctx->digestsize == SHA512_DIGEST_SIZE ? 4 :
			    ctx->digestsize == SHA224_DIGEST_SIZE ? 5 : 6;
}

static int s31_sha_process_pio(struct s31_crypto_dev *dev,
			       struct s31_sha_ctx *ctx, const u8 *data,
			       unsigned int blocks)
{
	void __iomem *sha = dev->base + S31_SHA_BASE;
	unsigned int words = ctx->blocksize == SHA512_BLOCK_SIZE ? 16 : 8;
	unsigned int i;
	int ret;

	writel(ctx->type, sha + S31_SHA_MODE);
	if (!ctx->first)
		for (i = 0; i < words; i++)
			writel(ctx->state[i], sha + S31_SHA_H_MEM + i * sizeof(u32));
	for (i = 0; i < blocks; i++) {
		s31_write_words(sha, S31_SHA_M_MEM, data + i * ctx->blocksize,
				ctx->blocksize);
		writel(1, sha + (ctx->first ? S31_SHA_START : S31_SHA_CONTINUE));
		ret = s31_crypto_poll(sha + S31_SHA_BUSY, U32_MAX, 0, 0, 1000);
		if (ret) {
			s31_crypto_reset_wait(dev, S31_CRYPTO_SHA_RST,
				sha + S31_SHA_BUSY, U32_MAX, 0, S31_CRYPTO_BULK);
			return ret;
		}
		ctx->first = false;
		cond_resched();
	}
	for (i = 0; i < words; i++)
		ctx->state[i] = readl(sha + S31_SHA_H_MEM + i * sizeof(u32));
	return 0;
}

static int s31_sha_dma_submit(struct s31_crypto_dev *dev,
			      struct s31_sha_ctx *ctx, struct scatterlist *sg,
			      unsigned int nents, unsigned int blocks)
{
	struct dma_async_tx_descriptor *desc;
	void __iomem *sha = dev->base + S31_SHA_BASE;
	dma_cookie_t cookie;
	unsigned int words = ctx->blocksize == SHA512_BLOCK_SIZE ? 16 : 8;
	struct s31_dma_wait dma_wait = {};
	u32 busy;
	int i, ret;

	writel(ctx->type, sha + S31_SHA_MODE);
	if (!ctx->first)
		for (i = 0; i < words; i++)
			writel(ctx->state[i], sha + S31_SHA_H_MEM + i * sizeof(u32));
	init_completion(&dma_wait.done);
	desc = dmaengine_prep_slave_sg(dev->sha_dma, sg, nents, DMA_MEM_TO_DEV,
				       DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
	if (!desc) {
		ret = -EIO;
		goto out;
	}
	desc->callback_result = s31_dma_complete;
	desc->callback_param = &dma_wait;
	cookie = dmaengine_submit(desc);
	if (dma_submit_error(cookie)) {
		ret = dma_submit_error(cookie);
		goto out;
	}
	dma_async_issue_pending(dev->sha_dma);
	writel(blocks, sha + S31_SHA_DMA_BLOCK_NUM);
	writel(1, sha + (ctx->first ? S31_SHA_DMA_START : S31_SHA_DMA_CONTINUE));
	ret = s31_dma_wait_done(&dma_wait);
	if (ret)
		goto out;
	ret = readl_poll_timeout(sha + S31_SHA_BUSY, busy, !busy, 10, 10000);
	if (ret)
		goto out;
	ctx->first = false;
	for (i = 0; i < words; i++)
		ctx->state[i] = readl(sha + S31_SHA_H_MEM + i * sizeof(u32));
out:
	/* Join callbacks before returning DMA buffers after errors or timeout. */
	if (ret) {
		dmaengine_terminate_sync(dev->sha_dma);
		s31_sha_dma_reset_fifos(dev);
		s31_crypto_reset_wait(dev, S31_CRYPTO_SHA_RST,
			sha + S31_SHA_BUSY, U32_MAX, 0, S31_CRYPTO_BULK);
	}
	return ret;
}

static int s31_sha_process_dma(struct s31_crypto_dev *dev,
			       struct s31_sha_ctx *ctx, unsigned int blocks)
{
	struct device *dma_dev = dmaengine_get_dma_device(dev->sha_dma);
	struct scatterlist sg;
	unsigned int len = blocks * ctx->blocksize;
	dma_addr_t dma;
	int ret;

	dma = dma_map_single(dma_dev, dev->sha_buffer, len, DMA_TO_DEVICE);
	if (dma_mapping_error(dma_dev, dma))
		return -EIO;
	sg_init_table(&sg, 1);
	sg_dma_address(&sg) = dma;
	sg_dma_len(&sg) = len;
	ret = s31_sha_dma_submit(dev, ctx, &sg, 1, blocks);
	dma_unmap_single(dma_dev, dma, len, DMA_TO_DEVICE);
	return ret;
}

/* S31 SHA_DMA_BLOCK_NUM is 16 bits, so a bounded 64-KiB request can run
 * as one DMA chain. Only a partial hash block needs the staging buffer.
 */
static int s31_sha_direct_update(struct s31_crypto_dev *dev,
				 struct ahash_request *req, unsigned int offset,
				 unsigned int *processed)
{
	struct s31_sha_request *rctx = ahash_request_ctx(req);
	struct s31_sha_ctx *ctx = &rctx->hash;
	struct device *dma_dev;
	struct scatterlist head[2], *source = req->src, *sg, *input = NULL;
	unsigned int bytes, nents, count = 0, left, n, i;
	int mapped, ret = 0;

	*processed = 0;
	bytes = min_t(unsigned int, S31_CRYPTO_DMA_MAX_BYTES,
		      round_down(req->nbytes - offset, ctx->blocksize));
	if (!dev->sha_dma || bytes <= S31_SHA_DMA_THRESHOLD ||
	    !IS_ALIGNED(ctx->total, ctx->blocksize))
		return 0;
	if (ahash_request_isvirt(req)) {
		if (!virt_addr_valid(req->svirt + offset) ||
		    !virt_addr_valid(req->svirt + offset + bytes - 1) ||
		    !IS_ALIGNED((unsigned long)req->svirt + offset, sizeof(u32)))
			return 0;
		nents = 1;
	} else {
		if (offset && sg_nents_for_len(req->src, offset + bytes) < 0)
			return 0;
		source = offset ? scatterwalk_ffwd(head, req->src, offset) : req->src;
		left = bytes;
		nents = 0;
		sg = source;
		while (left && sg) {
			n = min(left, sg->length);
			if (!n || !IS_ALIGNED(sg->offset | n, sizeof(u32)) ||
			    ++nents > S31_CRYPTO_DMA_MAX_NENTS)
				return 0;
			left -= n;
			if (left)
				sg = sg_next(sg);
		}
		if (left)
			return 0;
	}
	input = kmalloc_array(nents, sizeof(*input), GFP_NOWAIT);
	if (!input)
		return 0;
	sg_init_table(input, nents);
	if (ahash_request_isvirt(req)) {
		sg_set_buf(input, req->svirt + offset, bytes);
		count = 1;
	} else {
		left = bytes;
		for_each_sg(source, sg, nents, i) {
			n = min(left, sg->length);
			s31_dma_append(input, &count, sg_page(sg), sg->offset, n);
			left -= n;
		}
		sg_mark_end(&input[count - 1]);
	}
	*processed = bytes;
	dma_dev = dmaengine_get_dma_device(dev->sha_dma);
	mapped = dma_map_sg(dma_dev, input, count, DMA_TO_DEVICE);
	if (!mapped) {
		ret = -EIO;
		goto free;
	}
	ret = s31_sha_dma_submit(dev, ctx, input, mapped,
				bytes / ctx->blocksize);
	dma_unmap_sg(dma_dev, input, count, DMA_TO_DEVICE);
	if (!ret)
		ctx->total += bytes;
free:
	kfree(input);
	return ret;
}

static int s31_sha_copy(struct ahash_request *req, unsigned int offset,
			u8 *out, unsigned int len)
{
	int nents;

	if (ahash_request_isvirt(req)) {
		memcpy(out, req->svirt + offset, len);
		return 0;
	}
	nents = sg_nents_for_len(req->src, req->nbytes);
	if (nents < 0 || sg_pcopy_to_buffer(req->src, nents, out, len, offset) != len)
		return -EINVAL;
	return 0;
}

static int s31_sha_update_request(struct s31_crypto_dev *dev,
				  struct ahash_request *req)
{
	struct s31_sha_request *rctx = ahash_request_ctx(req);
	struct s31_sha_ctx *ctx = &rctx->hash;
	unsigned int offset = 0, left, n;
	int ret;

	if (req->nbytes > U64_MAX - ctx->total) {
		if (ctx->blocksize != SHA512_BLOCK_SIZE)
			return -EOVERFLOW;
		ctx->total_hi++;
	}
	while (offset < req->nbytes) {
		left = ctx->total & (ctx->blocksize - 1);
		if (!left && dev->sha_dma) {
			ret = s31_sha_direct_update(dev, req, offset, &n);
			if (ret)
				return ret;
			if (n) {
				offset += n;
				cond_resched();
				continue;
			}
		}
		if (left || req->nbytes - offset < ctx->blocksize) {
			n = min_t(unsigned int, ctx->blocksize - left,
				  req->nbytes - offset);
			ret = s31_sha_copy(req, offset, ctx->buffer + left, n);
			if (ret)
				return ret;
			ctx->total += n;
			if (left + n == ctx->blocksize) {
				ret = s31_sha_process_pio(dev, ctx, ctx->buffer, 1);
				if (ret)
					return ret;
				memzero_explicit(ctx->buffer, sizeof(ctx->buffer));
			}
		} else {
			n = min_t(unsigned int, S31_SHA_BATCH_BYTES,
				  round_down(req->nbytes - offset, ctx->blocksize));
			ret = s31_sha_copy(req, offset, dev->sha_buffer, n);
			if (ret) {
				/* A short scatterlist copy can already have written data. */
				memzero_explicit(dev->sha_buffer, n);
				return ret;
			}
			ctx->total += n;
			if (dev->sha_dma && n > ctx->blocksize)
				ret = s31_sha_process_dma(dev, ctx, n / ctx->blocksize);
			else
				ret = s31_sha_process_pio(dev, ctx, dev->sha_buffer,
							 n / ctx->blocksize);
			memzero_explicit(dev->sha_buffer, n);
			if (ret)
				return ret;
		}
		offset += n;
		cond_resched();
	}
	return 0;
}

static int s31_sha_final_request(struct s31_crypto_dev *dev,
				 struct ahash_request *req)
{
	struct s31_sha_request *rctx = ahash_request_ctx(req);
	struct s31_sha_ctx *ctx = &rctx->hash;
	unsigned int last = ctx->total & (ctx->blocksize - 1);
	unsigned int lenbytes = ctx->blocksize == SHA512_BLOCK_SIZE ? 16 : 8;
	u64 bits = ctx->total << 3;
	int i, ret;

	ctx->buffer[last++] = 0x80;
	if (last > ctx->blocksize - lenbytes) {
		memset(ctx->buffer + last, 0, ctx->blocksize - last);
		ret = s31_sha_process_pio(dev, ctx, ctx->buffer, 1);
		if (ret)
			return ret;
		last = 0;
	}
	memset(ctx->buffer + last, 0, ctx->blocksize - last);
	for (i = 0; i < 8; i++)
		ctx->buffer[ctx->blocksize - 1 - i] = bits >> (i * 8);
	if (lenbytes == 16) {
		bits = (ctx->total_hi << 3) | (ctx->total >> 61);
		for (i = 0; i < 8; i++)
			ctx->buffer[ctx->blocksize - 9 - i] = bits >> (i * 8);
	}
	ret = s31_sha_process_pio(dev, ctx, ctx->buffer, 1);
	if (!ret)
		memcpy(req->result, ctx->state, ctx->digestsize);
	return ret;
}

static int s31_sha_init(struct ahash_request *req)
{
	struct s31_sha_request *rctx = ahash_request_ctx(req);

	memset(rctx, 0, sizeof(*rctx));
	s31_sha_config(&rctx->hash, crypto_ahash_reqtfm(req));
	rctx->hash.first = true;
	rctx->initialized = true;
	return 0;
}

static int s31_sha_enqueue(struct ahash_request *req,
			   enum s31_sha_operation operation)
{
	struct s31_sha_request *rctx = ahash_request_ctx(req);

	if (!rctx->initialized)
		return -EINVAL;
	if (operation == S31_SHA_UPDATE && !req->nbytes)
		return 0;
	rctx->operation = operation;
	return s31_crypto_submit(&req->base, S31_REQUEST_SHA);
}

static int s31_sha_update(struct ahash_request *req)
{
	return s31_sha_enqueue(req, S31_SHA_UPDATE);
}

static int s31_sha_final(struct ahash_request *req)
{
	return s31_sha_enqueue(req, S31_SHA_FINAL);
}

static int s31_sha_finup(struct ahash_request *req)
{
	return s31_sha_enqueue(req, S31_SHA_FINUP);
}

static int s31_sha_digest(struct ahash_request *req)
{
	s31_sha_init(req);
	return s31_sha_finup(req);
}

/*
 * Match Linux's standard SHA export format, including its partial-byte suffix.
 * The hardware digest registers contain big-endian bytes, not host-endian words.
 */
static int s31_sha_export_state(struct ahash_request *req, void *out, bool core)
{
	struct s31_sha_request *rctx = ahash_request_ctx(req);
	struct s31_sha_ctx *ctx = &rctx->hash;
	union {
		struct sha1_ctx sha1;
		struct sha256_ctx sha256;
		struct sha224_ctx sha224;
		struct sha512_ctx sha512;
		struct sha384_ctx sha384;
	} state = { };
	unsigned int partial, size, head, i;
	u8 *buffer;

	if (!rctx->initialized)
		return -EINVAL;
	if (ctx->type > 4) {
		memcpy(out, ctx, sizeof(*ctx));
		return 0;
	}
	partial = ctx->total & (ctx->blocksize - 1);
	if (ctx->type == 0) {
		if (ctx->first)
			sha1_init(&state.sha1);
		else
			for (i = 0; i < 5; i++)
				state.sha1.state.h[i] = get_unaligned_be32((u8 *)ctx->state + i * 4);
		state.sha1.bytecount = core ? ctx->total : ctx->total - partial;
		buffer = state.sha1.buf;
		size = sizeof(state.sha1);
		head = offsetof(struct sha1_ctx, buf);
	} else if (ctx->type < 3) {
		if (ctx->first) {
			if (ctx->type == 1)
				sha224_init(&state.sha224);
			else
				sha256_init(&state.sha256);
		} else {
			for (i = 0; i < 8; i++)
				state.sha256.ctx.state.h[i] = get_unaligned_be32((u8 *)ctx->state + i * 4);
		}
		state.sha256.ctx.bytecount = core ? ctx->total : ctx->total - partial;
		buffer = state.sha256.ctx.buf;
		size = sizeof(state.sha256.ctx);
		head = offsetof(struct __sha256_ctx, buf);
	} else {
		if (ctx->first) {
			if (ctx->type == 3)
				sha384_init(&state.sha384);
			else
				sha512_init(&state.sha512);
		} else {
			for (i = 0; i < 8; i++)
				state.sha512.ctx.state.h[i] = get_unaligned_be64((u8 *)ctx->state + i * 8);
		}
		state.sha512.ctx.bytecount_lo = core ? ctx->total : ctx->total - partial;
		state.sha512.ctx.bytecount_hi = ctx->total_hi;
		buffer = state.sha512.ctx.buf;
		size = sizeof(state.sha512.ctx);
		head = offsetof(struct __sha512_ctx, buf);
	}
	if (core) {
		memcpy(out, &state, head);
	} else {
		memcpy(buffer, ctx->buffer, partial);
		memcpy(out, &state, size);
		((u8 *)out)[size] = partial;
	}
	memzero_explicit(&state, sizeof(state));
	return 0;
}

static int s31_sha_import_state(struct ahash_request *req, const void *in,
				bool core)
{
	struct s31_sha_request *rctx = ahash_request_ctx(req);
	struct s31_sha_ctx *ctx = &rctx->hash;
	union {
		struct sha1_ctx sha1;
		struct __sha256_ctx sha256;
		struct __sha512_ctx sha512;
	} state = { };
	unsigned int size, head, partial = 0, i;
	const u8 *buffer;
	int ret = 0;

	s31_sha_init(req);
	if (ctx->type > 4) {
		memcpy(ctx, in, sizeof(*ctx));
		s31_sha_config(ctx, crypto_ahash_reqtfm(req));
		ctx->first = !ctx->total_hi && ctx->total < ctx->blocksize;
		return 0;
	}
	if (ctx->type == 0) {
		size = sizeof(state.sha1);
		head = offsetof(struct sha1_ctx, buf);
		memcpy(&state.sha1, in, core ? head : size);
		ctx->total = state.sha1.bytecount;
		buffer = state.sha1.buf;
		for (i = 0; i < 5; i++)
			put_unaligned_be32(state.sha1.state.h[i], (u8 *)ctx->state + i * 4);
	} else if (ctx->type < 3) {
		size = sizeof(state.sha256);
		head = offsetof(struct __sha256_ctx, buf);
		memcpy(&state.sha256, in, core ? head : size);
		ctx->total = state.sha256.bytecount;
		buffer = state.sha256.buf;
		for (i = 0; i < 8; i++)
			put_unaligned_be32(state.sha256.state.h[i], (u8 *)ctx->state + i * 4);
	} else {
		size = sizeof(state.sha512);
		head = offsetof(struct __sha512_ctx, buf);
		memcpy(&state.sha512, in, core ? head : size);
		ctx->total = state.sha512.bytecount_lo;
		ctx->total_hi = state.sha512.bytecount_hi;
		buffer = state.sha512.buf;
		for (i = 0; i < 8; i++)
			put_unaligned_be64(state.sha512.state.h[i], (u8 *)ctx->state + i * 8);
	}
	if (!core)
		partial = ((const u8 *)in)[size];
	if ((ctx->total & (ctx->blocksize - 1)) || partial >= ctx->blocksize) {
		memzero_explicit(rctx, sizeof(*rctx));
		ret = -EINVAL;
	} else {
		ctx->total += partial;
		memcpy(ctx->buffer, buffer, partial);
		ctx->first = !ctx->total_hi && ctx->total < ctx->blocksize;
	}
	memzero_explicit(&state, sizeof(state));
	return ret;
}

static int s31_sha_export(struct ahash_request *req, void *out)
{
	return s31_sha_export_state(req, out, false);
}

static int s31_sha_export_core(struct ahash_request *req, void *out)
{
	return s31_sha_export_state(req, out, true);
}

static int s31_sha_import(struct ahash_request *req, const void *in)
{
	return s31_sha_import_state(req, in, false);
}

static int s31_sha_import_core(struct ahash_request *req, const void *in)
{
	return s31_sha_import_state(req, in, true);
}

static int s31_sha_run(struct s31_crypto_dev *dev, struct ahash_request *req)
{
	struct s31_sha_request *rctx = ahash_request_ctx(req);
	int ret;

	ret = pm_runtime_resume_and_get(dev->dev);
	if (ret >= 0) {
		rt_mutex_lock(&dev->engine_lock[S31_CRYPTO_BULK]);
		if (atomic_read(&dev->failed_domains) & BIT(S31_CRYPTO_BULK)) {
			ret = -EIO;
			goto unlock;
		}
		/* Borrow an available pair only for this request. Other peripheral
		 * clients must not lose a fixed pair merely because SHA is enabled.
		 */
		if (rctx->operation != S31_SHA_FINAL &&
		    req->nbytes > S31_SHA_DMA_THRESHOLD) {
			/* This DT-only consumer borrows transient channels; no per-job
			 * sysfs client links are needed. Keep normal DMA ownership/PM.
			 */
			dev->sha_dma = of_dma_request_slave_channel(dev->dev->of_node,
							  "sha");
			if (IS_ERR(dev->sha_dma))
				dev->sha_dma = NULL;
		}
		ret = 0;
		if (rctx->operation != S31_SHA_FINAL)
			ret = s31_sha_update_request(dev, req);
		if (!ret && rctx->operation != S31_SHA_UPDATE)
			ret = s31_sha_final_request(dev, req);
		if (dev->sha_dma) {
			dma_release_channel(dev->sha_dma);
			dev->sha_dma = NULL;
		}
		/* Do this once per request under the lease, on every exit path.
		 * Intermediate HMAC states and message blocks must not survive a
		 * PIO/DMA failure or the handoff to a different SHA variant/tenant.
		 */
		s31_clear_words(dev->base + S31_SHA_BASE + S31_SHA_H_MEM, SHA512_DIGEST_SIZE);
		s31_clear_words(dev->base + S31_SHA_BASE + S31_SHA_M_MEM, SHA512_BLOCK_SIZE);
		readl(dev->base + S31_SHA_BASE + S31_SHA_M_MEM +
		      SHA512_BLOCK_SIZE - sizeof(u32));
unlock:
		/* Staging paths clear their populated range after the hardware is
		 * stopped. PIO finalization and direct DMA never populate this page.
		 */
		rt_mutex_unlock(&dev->engine_lock[S31_CRYPTO_BULK]);
		pm_runtime_mark_last_busy(dev->dev);
		pm_runtime_put_autosuspend(dev->dev);
	}
	if (ret || rctx->operation != S31_SHA_UPDATE)
		memzero_explicit(rctx, sizeof(*rctx));
	return ret;
}

static int s31_sha_do_one(struct crypto_engine *engine, void *areq)
{
	struct s31_crypto_queue *queue = engine->priv_data;
	struct ahash_request *req = container_of(areq, struct ahash_request, base);
	int ret = s31_sha_run(queue->dev, req);

	s31_crypto_complete(engine, &req->base, ret, S31_REQUEST_SHA);
	return 0;
}

#define S31_SHA_ALG(_name, _driver, _digest, _block, _state, _flags, _core) { .base = { \
	.init = s31_sha_init, .update = s31_sha_update, .final = s31_sha_final, \
	.finup = s31_sha_finup, .digest = s31_sha_digest, \
	.export = s31_sha_export, .import = s31_sha_import, \
	.export_core = _core ? s31_sha_export_core : NULL, \
	.import_core = _core ? s31_sha_import_core : NULL, \
	.halg = { .digestsize = _digest, .statesize = _state, \
		.base = { .cra_name = _name, .cra_driver_name = _driver, \
			.cra_priority = 300, \
			.cra_flags = CRYPTO_ALG_ASYNC | CRYPTO_ALG_REQ_VIRT | _flags, \
			.cra_reqsize = sizeof(struct s31_sha_request), \
			.cra_blocksize = _block, .cra_module = THIS_MODULE } } }, \
	.op.do_one_request = s31_sha_do_one \
}

static struct ahash_engine_alg s31_sha_algs[] = {
	S31_SHA_ALG("sha1", "sha1-esp32s31", SHA1_DIGEST_SIZE, SHA1_BLOCK_SIZE,
		    sizeof(struct sha1_ctx) + 1, 0, true),
	S31_SHA_ALG("sha224", "sha224-esp32s31", SHA224_DIGEST_SIZE, SHA224_BLOCK_SIZE,
		    sizeof(struct __sha256_ctx) + 1, 0, true),
	S31_SHA_ALG("sha256", "sha256-esp32s31", SHA256_DIGEST_SIZE, SHA256_BLOCK_SIZE,
		    sizeof(struct __sha256_ctx) + 1, 0, true),
	S31_SHA_ALG("sha384", "sha384-esp32s31", SHA384_DIGEST_SIZE, SHA384_BLOCK_SIZE,
		    sizeof(struct __sha512_ctx) + 1, 0, true),
	S31_SHA_ALG("sha512", "sha512-esp32s31", SHA512_DIGEST_SIZE, SHA512_BLOCK_SIZE,
		    sizeof(struct __sha512_ctx) + 1, 0, true),
	/* Linux has no synchronous provider with these variants' exported state. */
	S31_SHA_ALG("sha512-224", "sha512-224-esp32s31", SHA224_DIGEST_SIZE,
		    SHA512_BLOCK_SIZE, sizeof(struct s31_sha_ctx), CRYPTO_ALG_NO_FALLBACK, false),
	S31_SHA_ALG("sha512-256", "sha512-256-esp32s31", SHA256_DIGEST_SIZE,
		    SHA512_BLOCK_SIZE, sizeof(struct s31_sha_ctx), CRYPTO_ALG_NO_FALLBACK, false),
};

/* Buffers may be unaligned and lengths need not be a complete limb. */
static void s31_be_to_limbs(u32 *dst, unsigned int words, const u8 *src,
			    unsigned int len)
{
	unsigned int full = len / sizeof(u32), tail = len % sizeof(u32), i;

	for (i = 0; i < full; i++)
		dst[i] = get_unaligned_be32(src + len - sizeof(u32) * (i + 1));
	if (tail) {
		u32 value = 0;

		for (i = 0; i < tail; i++)
			value = (value << 8) | src[i];
		dst[full++] = value;
	}
	memset(dst + full, 0, (words - full) * sizeof(*dst));
}

static void s31_limbs_to_be(u8 *dst, unsigned int len, const u32 *src)
{
	unsigned int full = len / sizeof(u32), tail = len % sizeof(u32), i;

	for (i = 0; i < full; i++)
		put_unaligned_be32(src[i], dst + len - sizeof(u32) * (i + 1));
	if (tail) {
		u32 value = src[full];

		for (i = 0; i < tail; i++) {
			dst[tail - 1 - i] = value;
			value >>= 8;
		}
	}
}

static int s31_limbs_cmp(const u32 *a, const u32 *b, unsigned int words)
{
	while (words--) {
		if (a[words] != b[words])
			return a[words] > b[words] ? 1 : -1;
	}
	return 0;
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

/* R^2 mod n depends only on the public modulus. Use the existing MPI long
 * division instead of repeatedly doubling every limb for twice its bit width.
 */
static int s31_rsa_compute_rinv(struct s31_rsa_key *key)
{
	MPI modulus = NULL, power = NULL;
	int ret = -ENOMEM;

	BUILD_BUG_ON(sizeof(mpi_limb_t) != sizeof(u32));
	memset(key->rinv, 0, sizeof(key->rinv));
	modulus = mpi_alloc(key->words);
	power = mpi_alloc(2 * key->words + 2);
	if (!modulus || !power)
		goto out;
	memcpy(modulus->d, key->n, key->words * sizeof(u32));
	modulus->nlimbs = key->words;
	/* Include the normalization carry limb to avoid a later resize. */
	ret = mpi_set_bit(power, 64 * key->words);
	if (ret)
		goto out;
	ret = mpi_fdiv_r(power, power, modulus);
	if (!ret) {
		if (power->sign || power->nlimbs > key->words)
			ret = -EINVAL;
		else
			memcpy(key->rinv, power->d, power->nlimbs * sizeof(u32));
	}
out:
	/* These temporaries contain only the public modulus and its remainder. */
	mpi_free(power);
	mpi_free(modulus);
	return ret;
}

/* Caller owns MPI, including any composite DS lease. */
static int s31_rsa_modexp_locked(struct s31_crypto_dev *dev,
			  const struct s31_rsa_key *key, const u32 *input,
			  const u32 *exponent, unsigned int exponent_bits,
			  bool constant_time, u32 *output)
{
	void __iomem *rsa = dev->base + S31_RSA_BASE;
	u32 value;
	int ret;

	if (!exponent_bits)
		return -EINVAL;
	if (atomic_read(&dev->failed_domains) & BIT(S31_CRYPTO_MPI))
		return -EIO;
	ret = readl_poll_timeout(rsa + S31_RSA_QUERY_CLEAN, value,
				value & BIT(0), 20, 10000);
	if (ret)
		goto recover;
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
	ret = readl_poll_timeout(rsa + S31_RSA_QUERY_IDLE, value,
				value & BIT(0), 20, 2000000);
	if (!ret)
		s31_read_words(rsa, S31_RSA_Z_MEM, (u8 *)output, key->bytes);
recover:
	if (ret) {
		if (s31_crypto_reset_wait(dev, S31_CRYPTO_RSA_RST,
				rsa + S31_RSA_QUERY_CLEAN, BIT(0), BIT(0),
				S31_CRYPTO_MPI))
			return ret;
	}
	writel(0, rsa + S31_RSA_SEARCH_ENABLE);
	s31_clear_words(rsa + S31_RSA_M_MEM, S31_RSA_MAX_BYTES);
	s31_clear_words(rsa + S31_RSA_Z_MEM, S31_RSA_MAX_BYTES);
	s31_clear_words(rsa + S31_RSA_Y_MEM, S31_RSA_MAX_BYTES);
	s31_clear_words(rsa + S31_RSA_X_MEM, S31_RSA_MAX_BYTES);
	readl(rsa + S31_RSA_X_MEM + S31_RSA_MAX_BYTES - sizeof(u32));
	return ret;
}

static int s31_rsa_modexp(struct s31_crypto_dev *dev,
			  const struct s31_rsa_key *key, const u32 *input,
			  const u32 *exponent, unsigned int exponent_bits,
			  bool constant_time, u32 *output)
{
	int ret;

	rt_mutex_lock(&dev->engine_lock[S31_CRYPTO_MPI]);
	ret = s31_rsa_modexp_locked(dev, key, input, exponent, exponent_bits,
				    constant_time, output);
	rt_mutex_unlock(&dev->engine_lock[S31_CRYPTO_MPI]);
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
	const u8 *exponent = raw->e;
	const u8 *secret = raw->d;
	unsigned int modulus_len = raw->n_sz;
	unsigned int exponent_len = raw->e_sz;
	unsigned int secret_len = raw->d_sz;

	if (!modulus_len || !exponent_len || (private && !secret_len))
		return -EINVAL;
	while (modulus_len > 1 && !*modulus) {
		modulus++;
		modulus_len--;
	}
	if (modulus_len > S31_RSA_MAX_BYTES || !(modulus[modulus_len - 1] & 1))
		return -EOPNOTSUPP;
	while (exponent_len > 1 && !*exponent) {
		exponent++;
		exponent_len--;
	}
	while (secret_len > 1 && !*secret) {
		secret++;
		secret_len--;
	}
	key->words = DIV_ROUND_UP(modulus_len, sizeof(u32));
	key->bytes = key->words * sizeof(u32);
	if (exponent_len > key->bytes || secret_len > key->bytes)
		return -EINVAL;
	s31_be_to_limbs(key->n, key->words, modulus, modulus_len);
	s31_be_to_limbs(key->e, key->words, exponent, exponent_len);
	key->e_bits = s31_limbs_bits(key->e, key->words);
	if (!key->e_bits)
		return -EINVAL;
	if (private) {
		s31_be_to_limbs(key->d, key->words, secret, secret_len);
		key->d_bits = s31_limbs_bits(key->d, key->words);
		if (!key->d_bits)
			return -EINVAL;
	}
	key->mprime = s31_rsa_mprime(key->n[0]);
	return s31_rsa_compute_rinv(key);
}

/* DS import may supply an immutable public key validated from the same DER.
 * Copy its cached arithmetic into this independently owned transform; ordinary
 * Crypto API key callbacks always decode and prepare their own input.
 */
static int s31_rsa_setkey(struct crypto_akcipher *tfm, const void *data,
			  unsigned int len, bool private,
			  const struct s31_rsa_key *validated_public)
{
	struct s31_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct rsa_key raw = { };
	int ret;

	ctx->have_key = false;
	ctx->have_private = false;
	s31_rsa_clear_key(&ctx->key);
	crypto_free_akcipher(ctx->fallback);
	ctx->fallback = crypto_alloc_akcipher("rsa-generic", 0,
					    CRYPTO_ALG_NEED_FALLBACK);
	if (IS_ERR(ctx->fallback)) {
		ret = PTR_ERR(ctx->fallback);
		ctx->fallback = NULL;
		return ret;
	}
	ret = private ? crypto_akcipher_set_priv_key(ctx->fallback, data, len) :
		crypto_akcipher_set_pub_key(ctx->fallback, data, len);
	if (ret)
		goto invalid;
	if (validated_public) {
		ctx->key = *validated_public;
	} else {
		ret = private ? rsa_parse_priv_key(&raw, data, len) :
			rsa_parse_pub_key(&raw, data, len);
		if (ret)
			goto invalid;
		ret = s31_rsa_load_key(&ctx->key, &raw, private);
		if (ret == -EOPNOTSUPP) {
			s31_rsa_clear_key(&ctx->key);
			ret = 0;
		}
	}
	if (ret)
		goto invalid;
	ctx->have_key = true;
	ctx->have_private = private;
	return 0;
invalid:
	s31_rsa_clear_key(&ctx->key);
	crypto_free_akcipher(ctx->fallback);
	ctx->fallback = NULL;
	return ret;
}

static int s31_rsa_set_pub_key(struct crypto_akcipher *tfm, const void *data,
			       unsigned int len)
{
	return s31_rsa_setkey(tfm, data, len, false, NULL);
}

static int s31_rsa_set_priv_key(struct crypto_akcipher *tfm, const void *data,
				unsigned int len)
{
	return s31_rsa_setkey(tfm, data, len, true, NULL);
}

static int s31_rsa_crypt(struct s31_crypto_dev *dev,
			 struct akcipher_request *req, bool private)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
	struct s31_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct s31_rsa_request *rctx;
	const u32 *exponent;
	unsigned int exponent_bits;
	int ret;

	if (!ctx->have_key || (private && !ctx->have_private))
		return -ENOKEY;
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
	rctx = kzalloc(sizeof(*rctx), GFP_KERNEL);
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
	struct s31_asym_request *rctx = akcipher_request_ctx(req);

	rctx->private = false;
	return s31_crypto_submit(&req->base, S31_REQUEST_RSA);
}

static int s31_rsa_decrypt(struct akcipher_request *req)
{
	struct s31_asym_request *rctx = akcipher_request_ctx(req);

	rctx->private = true;
	return s31_crypto_submit(&req->base, S31_REQUEST_RSA);
}

static int s31_rsa_run(struct s31_crypto_dev *dev, struct akcipher_request *req)
{
	struct s31_asym_request *rctx = akcipher_request_ctx(req);
	int ret;

	ret = pm_runtime_resume_and_get(dev->dev);
	if (ret >= 0) {
		ret = s31_rsa_crypt(dev, req, rctx->private);
		pm_runtime_mark_last_busy(dev->dev);
		pm_runtime_put_autosuspend(dev->dev);
	}

	return ret;
}

static int s31_rsa_do_one(struct crypto_engine *engine, void *areq)
{
	struct s31_crypto_queue *queue = engine->priv_data;
	struct akcipher_request *req = container_of(areq, struct akcipher_request, base);
	int ret = s31_rsa_run(queue->dev, req);

	s31_crypto_complete(engine, &req->base, ret, S31_REQUEST_RSA);
	return 0;
}

static unsigned int s31_rsa_max_size(struct crypto_akcipher *tfm)
{
	struct s31_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);

	if (!ctx->have_key)
		return 0;
	return ctx->key.words ? ctx->key.bytes :
		crypto_akcipher_maxsize(ctx->fallback);
}

static int s31_rsa_init_tfm(struct crypto_akcipher *tfm)
{
	struct s31_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);

	ctx->fallback = crypto_alloc_akcipher("rsa-generic", 0,
					  CRYPTO_ALG_NEED_FALLBACK);
	if (IS_ERR(ctx->fallback))
		return PTR_ERR(ctx->fallback);
	akcipher_set_reqsize(tfm, max_t(unsigned int,
				      sizeof(struct s31_asym_request),
				      crypto_akcipher_reqsize(ctx->fallback)));
	return 0;
}

static void s31_rsa_exit_tfm(struct crypto_akcipher *tfm)
{
	struct s31_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);

	crypto_free_akcipher(ctx->fallback);
	memzero_explicit(ctx, sizeof(*ctx));
}

static struct akcipher_engine_alg s31_rsa_alg = {
	.base = {
		.encrypt = s31_rsa_encrypt,
		.decrypt = s31_rsa_decrypt,
		.set_pub_key = s31_rsa_set_pub_key,
		.set_priv_key = s31_rsa_set_priv_key,
		.max_size = s31_rsa_max_size,
		.init = s31_rsa_init_tfm,
		.exit = s31_rsa_exit_tfm,
		.base = {
			.cra_name = "rsa",
			.cra_driver_name = "rsa-esp32s31",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_NEED_FALLBACK | CRYPTO_ALG_ASYNC,
			.cra_module = THIS_MODULE,
			.cra_ctxsize = sizeof(struct s31_rsa_ctx),
		},
	},
	.op.do_one_request = s31_rsa_do_one,
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

static irqreturn_t s31_ecc_irq(int irq, void *arg)
{
	struct s31_crypto_dev *dev = arg;
	void __iomem *ecc = dev->base + S31_ECC_BASE;

	if (!(readl(ecc + S31_ECC_INT_ST) & BIT(0)))
		return IRQ_NONE;
	writel(0, ecc + S31_ECC_INT_ENA);
	writel(BIT(0), ecc + S31_ECC_INT_CLR);
	complete(&dev->ecc_done);
	return IRQ_HANDLED;
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
	int ret;

	if (bytes != 32 && bytes != 48)
		return -EINVAL;
	s31_reverse(scalar_le, scalar, bytes);
	s31_reverse(px_le, px, bytes);
	s31_reverse(py_le, py, bytes);
	rt_mutex_lock(&dev->engine_lock[S31_CRYPTO_ECC]);
	if (atomic_read(&dev->failed_domains) & BIT(S31_CRYPTO_ECC)) {
		ret = -EIO;
		goto unlock;
	}
	s31_write_words(ecc, S31_ECC_K_MEM, scalar_le, S31_ECC_MAX_BYTES);
	s31_write_words(ecc, S31_ECC_PX_MEM, px_le, S31_ECC_MAX_BYTES);
	s31_write_words(ecc, S31_ECC_PY_MEM, py_le, S31_ECC_MAX_BYTES);
	writel(0, ecc + S31_ECC_INT_ENA);
	writel(BIT(0), ecc + S31_ECC_INT_CLR);
	reinit_completion(&dev->ecc_done);
	writel(BIT(0), ecc + S31_ECC_INT_ENA);
	writel((bytes == 48 ? S31_ECC_CURVE_P384 : S31_ECC_CURVE_P256)
		       << S31_ECC_KEY_LENGTH_SHIFT |
		       (verify ? S31_ECC_MODE_VERIFY_THEN_POINT_MUL :
			 S31_ECC_MODE_POINT_MUL) << S31_ECC_WORK_MODE_SHIFT |
		       S31_ECC_SECURITY_MODE, ecc + S31_ECC_CONF);
	writel(readl(ecc + S31_ECC_CONF) | S31_ECC_START, ecc + S31_ECC_CONF);
	ret = wait_for_completion_timeout(&dev->ecc_done, msecs_to_jiffies(2000)) ? 0 : -ETIMEDOUT;
	writel(0, ecc + S31_ECC_INT_ENA);
	if (ret)
		synchronize_irq(dev->ecc_irq);
	if (ret) {
		if (s31_crypto_reset_wait(dev, S31_CRYPTO_ECC_RST,
				ecc + S31_ECC_CONF, S31_ECC_START, 0, S31_CRYPTO_ECC))
			goto unlock;
	}
	if (!ret && verify && !(readl(ecc + S31_ECC_CONF) & S31_ECC_VERIFY_RESULT))
		ret = -EINVAL;
	if (!ret) {
		s31_read_words(ecc, S31_ECC_PX_MEM, qx_le, bytes);
		s31_read_words(ecc, S31_ECC_PY_MEM, qy_le, bytes);
		s31_reverse(qx, qx_le, bytes);
		s31_reverse(qy, qy_le, bytes);
	}
	s31_clear_words(ecc + S31_ECC_K_MEM, S31_ECC_MAX_BYTES);
	s31_clear_words(ecc + S31_ECC_PX_MEM, S31_ECC_MAX_BYTES);
	s31_clear_words(ecc + S31_ECC_PY_MEM, S31_ECC_MAX_BYTES);
	readl(ecc + S31_ECC_PY_MEM + S31_ECC_MAX_BYTES - sizeof(u32));
	writel(BIT(0), ecc + S31_ECC_INT_CLR);
unlock:
	rt_mutex_unlock(&dev->engine_lock[S31_CRYPTO_ECC]);
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

	ctx->have_private = false;
	memzero_explicit(ctx->private, sizeof(ctx->private));
	if (crypto_ecdh_decode_key(data, len, &params) || !params.key_size ||
	    params.key_size > ctx->bytes)
		return -EINVAL;
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

static int s31_ecdh_generate(struct s31_crypto_dev *dev,
			     struct kpp_request *req)
{
	struct s31_ecdh_ctx *ctx = kpp_tfm_ctx(crypto_kpp_reqtfm(req));
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

static int s31_ecdh_compute(struct s31_crypto_dev *dev,
			    struct kpp_request *req)
{
	struct s31_ecdh_ctx *ctx = kpp_tfm_ctx(crypto_kpp_reqtfm(req));
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

static int s31_ecdh_generate_public_key(struct kpp_request *req)
{
	struct s31_asym_request *rctx = kpp_request_ctx(req);

	rctx->private = false;
	return s31_crypto_submit(&req->base, S31_REQUEST_ECDH);
}

static int s31_ecdh_compute_shared_secret(struct kpp_request *req)
{
	struct s31_asym_request *rctx = kpp_request_ctx(req);

	rctx->private = true;
	return s31_crypto_submit(&req->base, S31_REQUEST_ECDH);
}

static int s31_ecdh_run(struct s31_crypto_dev *dev, struct kpp_request *req)
{
	struct s31_asym_request *rctx = kpp_request_ctx(req);
	int ret;

	ret = pm_runtime_resume_and_get(dev->dev);
	if (ret >= 0) {
		ret = rctx->private ? s31_ecdh_compute(dev, req) :
			s31_ecdh_generate(dev, req);
		pm_runtime_mark_last_busy(dev->dev);
		pm_runtime_put_autosuspend(dev->dev);
	}
	return ret;
}

static int s31_ecdh_do_one(struct crypto_engine *engine, void *areq)
{
	struct s31_crypto_queue *queue = engine->priv_data;
	struct kpp_request *req = container_of(areq, struct kpp_request, base);
	int ret = s31_ecdh_run(queue->dev, req);

	s31_crypto_complete(engine, &req->base, ret, S31_REQUEST_ECDH);
	return 0;
}

/* Direct callers and queued workers share all hardware and cleanup paths. */
static int s31_crypto_execute(struct s31_crypto_dev *dev,
			      struct crypto_async_request *req,
			      enum s31_request_type type)
{
	switch (type) {
	case S31_REQUEST_AES:
		return s31_aes_run(dev, container_of(req, struct skcipher_request, base));
	case S31_REQUEST_SHA:
		return s31_sha_run(dev, container_of(req, struct ahash_request, base));
	case S31_REQUEST_RSA:
		return s31_rsa_run(dev, container_of(req, struct akcipher_request, base));
	case S31_REQUEST_ECDH:
		return s31_ecdh_run(dev, container_of(req, struct kpp_request, base));
	}
	return -EINVAL;
}

static void s31_ecdh_exit(struct crypto_kpp *tfm)
{
	memzero_explicit(kpp_tfm_ctx(tfm), sizeof(struct s31_ecdh_ctx));
}

static unsigned int s31_ecdh_max_size(struct crypto_kpp *tfm)
{
	return 2 * ((struct s31_ecdh_ctx *)kpp_tfm_ctx(tfm))->bytes;
}

static int s31_ecdh_p256_init(struct crypto_kpp *tfm)
{
	struct s31_ecdh_ctx *ctx = kpp_tfm_ctx(tfm);

	kpp_set_reqsize(tfm, sizeof(struct s31_asym_request));
	ctx->bytes = 32;
	ctx->curve = ECC_CURVE_NIST_P256;
	ctx->digits = ECC_CURVE_NIST_P256_DIGITS;
	return 0;
}

static int s31_ecdh_p384_init(struct crypto_kpp *tfm)
{
	struct s31_ecdh_ctx *ctx = kpp_tfm_ctx(tfm);

	kpp_set_reqsize(tfm, sizeof(struct s31_asym_request));
	ctx->bytes = 48;
	ctx->curve = ECC_CURVE_NIST_P384;
	ctx->digits = ECC_CURVE_NIST_P384_DIGITS;
	return 0;
}

#define S31_ECDH_ALG(_name, _driver, _init) { .base = { \
	.set_secret = s31_ecdh_set_secret, \
	.generate_public_key = s31_ecdh_generate_public_key, \
	.compute_shared_secret = s31_ecdh_compute_shared_secret, \
	.max_size = s31_ecdh_max_size, .init = _init, .exit = s31_ecdh_exit, \
	.base = { .cra_name = _name, .cra_driver_name = _driver, \
		.cra_priority = 300, .cra_flags = CRYPTO_ALG_ASYNC, \
		.cra_module = THIS_MODULE, .cra_destroy = s31_crypto_alg_destroy, \
		.cra_ctxsize = sizeof(struct s31_ecdh_ctx) } }, \
	.op.do_one_request = s31_ecdh_do_one \
}

static struct kpp_engine_alg s31_ecdh_algs[] = {
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
	ret = s31_rsa_compute_rinv(key);
	if (ret)
		goto fail;
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

#include "esp32s31-crypto-km.inc"
#include "esp32s31-crypto-protected.inc"
#include "esp32s31-crypto-ecdsa.inc"

/*
 * Keep static algorithm descriptors intact until old transforms are freed.
 * A removed instance rejects new hardware work; rebind waits for those users.
 */
#define S31_AES_ALGS ARRAY_SIZE(s31_aes_algs)
#define S31_SHA_ALGS ARRAY_SIZE(s31_sha_algs)
#define S31_RSA_INDEX (S31_AES_ALGS + S31_SHA_ALGS)
#define S31_KPP_INDEX (S31_RSA_INDEX + 1)
#define S31_SIG_INDEX (S31_KPP_INDEX + ARRAY_SIZE(s31_ecdh_algs))
#define S31_ALGS (S31_SIG_INDEX + ARRAY_SIZE(s31_ecdsa_algs))

static bool s31_alg_live[S31_ALGS];

static struct crypto_alg *s31_crypto_algorithm(unsigned int index)
{
	if (index < S31_AES_ALGS)
		return &s31_aes_algs[index].base.base;
	if (index < S31_RSA_INDEX)
		return &s31_sha_algs[index - S31_AES_ALGS].base.halg.base;
	if (index == S31_RSA_INDEX)
		return &s31_rsa_alg.base.base;
	if (index < S31_SIG_INDEX)
		return &s31_ecdh_algs[index - S31_KPP_INDEX].base.base;
	return &s31_ecdsa_algs[index - S31_SIG_INDEX].base;
}

static void s31_crypto_alg_destroy(struct crypto_alg *alg)
{
	unsigned int i;

	for (i = 0; i < S31_ALGS; i++)
		if (s31_crypto_algorithm(i) == alg) {
			WRITE_ONCE(s31_alg_live[i], false);
			return;
		}
}

static int s31_crypto_register_algorithm(unsigned int i)
{
	int ret;

	s31_crypto_algorithm(i)->cra_destroy = s31_crypto_alg_destroy;
	WRITE_ONCE(s31_alg_live[i], true);
	if (i < S31_AES_ALGS)
		ret = crypto_engine_register_skcipher(&s31_aes_algs[i]);
	else if (i < S31_RSA_INDEX)
		ret = crypto_engine_register_ahash(&s31_sha_algs[i - S31_AES_ALGS]);
	else if (i == S31_RSA_INDEX)
		ret = crypto_engine_register_akcipher(&s31_rsa_alg);
	else if (i < S31_SIG_INDEX)
		ret = crypto_engine_register_kpp(&s31_ecdh_algs[i - S31_KPP_INDEX]);
	else
		ret = crypto_register_sig(&s31_ecdsa_algs[i - S31_SIG_INDEX]);
	if (ret)
		WRITE_ONCE(s31_alg_live[i], false);
	return ret;
}

static void s31_crypto_unregister_algorithm(unsigned int i)
{
	if (i < S31_AES_ALGS)
		crypto_engine_unregister_skcipher(&s31_aes_algs[i]);
	else if (i < S31_RSA_INDEX)
		crypto_engine_unregister_ahash(&s31_sha_algs[i - S31_AES_ALGS]);
	else if (i == S31_RSA_INDEX)
		crypto_engine_unregister_akcipher(&s31_rsa_alg);
	else if (i < S31_SIG_INDEX)
		crypto_engine_unregister_kpp(&s31_ecdh_algs[i - S31_KPP_INDEX]);
	else
		crypto_unregister_sig(&s31_ecdsa_algs[i - S31_SIG_INDEX]);
}

static void s31_crypto_unpublish(struct s31_crypto_dev *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&s31_crypto_admission, flags);
	WRITE_ONCE(s31_crypto, NULL);
	spin_unlock_irqrestore(&s31_crypto_admission, flags);
	wait_event(dev->idle, !atomic_read(&dev->users));
	/* Finish the last put's wakeup before its waitqueue can be released. */
	spin_lock_irqsave(&s31_crypto_admission, flags);
	WRITE_ONCE(s31_crypto_provider, NULL);
	spin_unlock_irqrestore(&s31_crypto_admission, flags);
}

static int s31_crypto_queue_init(struct s31_crypto_dev *dev,
				 struct s31_crypto_queue *queue)
{
	int ret;

	queue->dev = dev;
	tasklet_setup(&queue->done, s31_crypto_done);
	queue->engine = crypto_engine_alloc_init(dev->dev, false);
	if (!queue->engine)
		return -ENOMEM;
	queue->engine->priv_data = queue;
	ret = crypto_engine_start(queue->engine);
	if (ret)
		crypto_engine_exit(queue->engine);
	return ret;
}

static void s31_crypto_queue_exit(struct s31_crypto_queue *queue)
{
	crypto_engine_exit(queue->engine);
	tasklet_kill(&queue->done);
}

static int s31_crypto_runtime_suspend(struct device *device)
{
	struct s31_crypto_dev *dev = dev_get_drvdata(device);
	unsigned long flags;
	u32 value;

	/* No request holds a runtime reference; all intermediate state is in RAM. */
	raw_spin_lock_irqsave(&dev->clock_lock, flags);
	value = readl(dev->clkrst);
	writel(value & ~(S31_CRYPTO_CLOCKS | S31_CRYPTO_EXTRA_CLOCKS), dev->clkrst);
	readl(dev->clkrst);
	writel(readl(dev->clkrst + 4) & ~(BIT(0) | BIT(2)), dev->clkrst + 4);
	readl(dev->clkrst + 4);
	writel(readl(dev->ecc_mem) | S31_ECC_MEM_LP_EN | S31_ECC_MEM_FORCE,
	       dev->ecc_mem);
	raw_spin_unlock_irqrestore(&dev->clock_lock, flags);
	clk_disable_unprepare(dev->clock);
	return 0;
}

static int s31_crypto_runtime_resume(struct device *device)
{
	struct s31_crypto_dev *dev = dev_get_drvdata(device);
	unsigned long flags;
	u32 value, failed;
	int ret;

	ret = clk_prepare_enable(dev->clock);
	if (ret)
		return ret;
	raw_spin_lock_irqsave(&dev->clock_lock, flags);
	failed = s31_crypto_failed_clocks(dev);
	value = readl(dev->clkrst) & ~(S31_CRYPTO_CLK_SEL | failed);
	writel((value | (failed << 1) | S31_CRYPTO_SYS_CLK | S31_CRYPTO_SEC_CLK |
		(S31_CRYPTO_CLOCKS & ~failed) | S31_CRYPTO_CLK_240M) &
	       ~(S31_CRYPTO_PARENT_RESETS & ~(failed << 1)), dev->clkrst);
	readl(dev->clkrst);
	if (atomic_read(&dev->failed_domains) & BIT(S31_CRYPTO_KEY_MANAGER)) {
		value = readl(dev->clkrst + 4);
		writel((value | BIT(1)) & ~(BIT(0) | BIT(2)), dev->clkrst + 4);
		readl(dev->clkrst + 4);
	}
	writel((readl(dev->ecc_mem) | S31_ECC_MEM_FORCE) & ~S31_ECC_MEM_LP_EN,
	       dev->ecc_mem);
	raw_spin_unlock_irqrestore(&dev->clock_lock, flags);
	/* Autosuspend powers ECC SRAM down. Initialize its power-on contents
	 * before runtime PM publishes the resumed device to a new owner.
	 */
	s31_clear_words(dev->base + S31_ECC_BASE + S31_ECC_K_MEM, 3 * S31_ECC_MAX_BYTES);
	readl(dev->base + S31_ECC_BASE + S31_ECC_PY_MEM +
	      S31_ECC_MAX_BYTES - sizeof(u32));
	return 0;
}

static void s31_crypto_quiesce(struct s31_crypto_dev *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&s31_crypto_admission, flags);
	dev->suspending = true;
	spin_unlock_irqrestore(&s31_crypto_admission, flags);
	wait_event(dev->idle, !atomic_read(&dev->users));
}

static int s31_crypto_suspend(struct device *device)
{
	struct s31_crypto_dev *dev = dev_get_drvdata(device);
	unsigned long flags;
	int ret;

	s31_crypto_quiesce(dev);
	ret = pm_runtime_force_suspend(device);
	if (ret) {
		spin_lock_irqsave(&s31_crypto_admission, flags);
		dev->suspending = false;
		spin_unlock_irqrestore(&s31_crypto_admission, flags);
	}
	return ret;
}

static int s31_crypto_resume(struct device *device)
{
	struct s31_crypto_dev *dev = dev_get_drvdata(device);
	unsigned long flags;
	int ret;

	ret = pm_runtime_force_resume(device);
	if (ret)
		return ret;
	spin_lock_irqsave(&s31_crypto_admission, flags);
	dev->suspending = false;
	spin_unlock_irqrestore(&s31_crypto_admission, flags);
	return 0;
}

static const struct dev_pm_ops s31_crypto_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(s31_crypto_suspend, s31_crypto_resume)
	RUNTIME_PM_OPS(s31_crypto_runtime_suspend, s31_crypto_runtime_resume, NULL)
};

#include "esp32s31-crypto-user.inc"

static int s31_crypto_probe_device(struct platform_device *pdev)
{
	struct platform_device *rng;
	struct device_node *rng_node;
	struct device_link *rng_link;
	struct s31_crypto_dev *dev;
	unsigned long flags;
	unsigned int i;
	int ret;

	for (i = 0; i < S31_ALGS; i++)
		if (READ_ONCE(s31_alg_live[i]))
			return dev_err_probe(&pdev->dev, -EBUSY,
					     "crypto transforms still reference the previous instance\n");
	rng_node = of_parse_phandle(pdev->dev.of_node, "rng", 0);
	if (!rng_node)
		return dev_err_probe(&pdev->dev, -EINVAL, "missing conditioned TRNG supplier\n");
	rng = of_find_device_by_node(rng_node);
	of_node_put(rng_node);
	if (!rng)
		return -EPROBE_DEFER;
	if (!device_is_bound(&rng->dev)) {
		put_device(&rng->dev);
		return -EPROBE_DEFER;
	}
	rng_link = device_link_add(&pdev->dev, &rng->dev, DL_FLAG_AUTOPROBE_CONSUMER);
	put_device(&rng->dev);
	if (!rng_link)
		return -ENOMEM;
	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	dev->dev = &pdev->dev;
	dev->key_purposes = devm_nvmem_cell_get(&pdev->dev, "key-purposes");
	if (IS_ERR(dev->key_purposes))
		return dev_err_probe(&pdev->dev, PTR_ERR(dev->key_purposes), "key purposes\n");
	dev->key_read_protect = devm_nvmem_cell_get(&pdev->dev, "key-read-protect");
	if (IS_ERR(dev->key_read_protect))
		return dev_err_probe(&pdev->dev, PTR_ERR(dev->key_read_protect), "key protection\n");
	dev->clock = devm_clk_get(&pdev->dev, "crypto");
	if (IS_ERR(dev->clock))
		return dev_err_probe(&pdev->dev, PTR_ERR(dev->clock), "crypto parent clock\n");
	if (clk_get_rate(dev->clock) != 240000000) {
		ret = clk_set_rate(dev->clock, 240000000);
		if (ret)
			return dev_err_probe(&pdev->dev, ret, "crypto requires 240 MHz\n");
	}
	dev->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dev->base))
		return PTR_ERR(dev->base);
	dev->clkrst = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(dev->clkrst))
		return PTR_ERR(dev->clkrst);
	dev->ecc_mem = devm_platform_ioremap_resource(pdev, 2);
	if (IS_ERR(dev->ecc_mem))
		return PTR_ERR(dev->ecc_mem);
	dev->key_manager = devm_platform_ioremap_resource(pdev, 3);
	if (IS_ERR(dev->key_manager))
		return PTR_ERR(dev->key_manager);
	dev->huk = devm_platform_ioremap_resource_byname(pdev, "huk");
	if (IS_ERR(dev->huk))
		return PTR_ERR(dev->huk);
	dev->efuse_read_control = devm_platform_ioremap_resource(pdev, 4);
	if (IS_ERR(dev->efuse_read_control))
		return PTR_ERR(dev->efuse_read_control);

	dev->sha_buffer = devm_kzalloc(&pdev->dev, PAGE_SIZE, GFP_KERNEL);
	if (!dev->sha_buffer)
		return -ENOMEM;
	raw_spin_lock_init(&dev->clock_lock);
	for (i = 0; i < S31_CRYPTO_DOMAINS; i++)
		rt_mutex_init(&dev->engine_lock[i]);
	atomic_set(&dev->users, 0);
	atomic_set(&dev->failed_domains, 0);
	init_waitqueue_head(&dev->idle);
	platform_set_drvdata(pdev, dev);
	ret = s31_crypto_runtime_resume(&pdev->dev);
	if (ret)
		return ret;
	s31_crypto_reset(dev, S31_CRYPTO_AES_RST | S31_CRYPTO_SHA_RST |
			 S31_CRYPTO_RSA_RST | S31_CRYPTO_ECC_RST);
	init_completion(&dev->ecc_done);
	dev->ecc_irq = platform_get_irq_byname(pdev, "ecc");
	if (dev->ecc_irq < 0) {
		ret = dev->ecc_irq;
		goto release_dma;
	}
	ret = devm_request_irq(&pdev->dev, dev->ecc_irq, s31_ecc_irq, 0,
			       "esp32s31-crypto-ecc", dev);
	if (ret)
		goto release_dma;
	s31_aes_dma_reset_fifos(dev);
	s31_sha_dma_reset_fifos(dev);
	ret = s31_crypto_selftest(dev);
	if (ret) {
		dev_err(&pdev->dev, "RSA/MPI or ECC hardware self-test failed\n");
		goto release_dma;
	}
	ret = s31_crypto_queue_init(dev, &dev->bulk);
	if (ret)
		goto release_dma;
	ret = s31_crypto_queue_init(dev, &dev->asym);
	if (ret)
		goto release_bulk;
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, 100);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	spin_lock_irqsave(&s31_crypto_admission, flags);
	WRITE_ONCE(s31_crypto_provider, dev);
	WRITE_ONCE(s31_crypto, dev);
	spin_unlock_irqrestore(&s31_crypto_admission, flags);
	for (i = 0; i < S31_ALGS; i++) {
		ret = s31_crypto_register_algorithm(i);
		if (ret)
			goto unregister;
	}
	dev->miscdev.minor = MISC_DYNAMIC_MINOR;
	dev->miscdev.name = "s31-crypto";
	dev->miscdev.fops = &s31_crypto_fops;
	dev->miscdev.parent = &pdev->dev;
	dev->miscdev.mode = 0600;
	ret = misc_register(&dev->miscdev);
	if (ret)
		goto unregister;
	dev_info(&pdev->dev,
		 "AES/SHA (shared DMA), RSA/MPI and P-256/P-384 ECDH at 240 MHz\n");
	pm_runtime_mark_last_busy(&pdev->dev);
	pm_runtime_put_autosuspend(&pdev->dev);
	return 0;

unregister:
	s31_crypto_unpublish(dev);
	while (i)
		s31_crypto_unregister_algorithm(--i);
	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	s31_crypto_queue_exit(&dev->asym);
release_bulk:
	s31_crypto_queue_exit(&dev->bulk);
release_dma:
	s31_crypto_runtime_suspend(&pdev->dev);
	if (dev->sha_dma)
		dma_release_channel(dev->sha_dma);
	return ret;
}

static int s31_crypto_probe(struct platform_device *pdev)
{
	int ret;

	mutex_lock(&s31_crypto_lifecycle);
	ret = s31_crypto_probe_device(pdev);
	mutex_unlock(&s31_crypto_lifecycle);
	return ret;
}

static void s31_crypto_remove(struct platform_device *pdev)
{
	struct s31_crypto_dev *dev = platform_get_drvdata(pdev);
	unsigned int i = S31_ALGS;

	mutex_lock(&s31_crypto_lifecycle);
	misc_deregister(&dev->miscdev);
	s31_crypto_unpublish(dev);
	while (i)
		s31_crypto_unregister_algorithm(--i);
	s31_crypto_queue_exit(&dev->asym);
	s31_crypto_queue_exit(&dev->bulk);
	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		s31_crypto_runtime_suspend(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	if (dev->sha_dma)
		dma_release_channel(dev->sha_dma);
	mutex_unlock(&s31_crypto_lifecycle);
}

static void s31_crypto_shutdown(struct platform_device *pdev)
{
	struct s31_crypto_dev *dev = platform_get_drvdata(pdev);
	int ret;

	/* Device links stop the radio consumer first. Deny new native work,
	 * drain accepted jobs and SDK leases, then stop clocks/DMA before the
	 * final platform power transition. Unlike failed suspend, shutdown must
	 * never reopen admission to still-runnable userspace or kernel callers.
	 */
	s31_crypto_quiesce(dev);
	ret = pm_runtime_force_suspend(&pdev->dev);
	if (ret)
		dev_err(&pdev->dev, "crypto shutdown failed: %d\n", ret);
}

static const struct of_device_id s31_crypto_of_match[] = {
	{ .compatible = "espressif,esp32s31-crypto" },
	{ }
};
MODULE_DEVICE_TABLE(of, s31_crypto_of_match);

static struct platform_driver s31_crypto_driver = {
	.probe = s31_crypto_probe,
	.remove = s31_crypto_remove,
	.shutdown = s31_crypto_shutdown,
	.driver = {
		.name = "esp32s31-crypto",
		.of_match_table = s31_crypto_of_match,
		.pm = pm_ptr(&s31_crypto_pm_ops),
	},
};
module_platform_driver(s31_crypto_driver);

MODULE_DESCRIPTION("ESP32-S31 AES, SHA, RSA/MPI and ECC Crypto API driver");
MODULE_LICENSE("GPL");
