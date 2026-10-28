// SPDX-License-Identifier: GPL-2.0-only
/* Opaque eFuse key operations. Only purpose/protection metadata is read via
 * NVMEM. No path reads secret key shadows, provisions keys or writes JTAG
 * authorization registers.
 */

#define S31_HMAC_BASE		0x5000
#define S31_HMAC_START		0x40
#define S31_HMAC_PURPOSE		0x44
#define S31_HMAC_KEY		0x48
#define S31_HMAC_CONFIGURE	0x4c
#define S31_HMAC_MESSAGE		0x50
#define S31_HMAC_CONTINUE		0x54
#define S31_HMAC_FINISH		0x5c
#define S31_HMAC_ERROR		0x68
#define S31_HMAC_BUSY		0x6c
#define S31_HMAC_INPUT		0x80
#define S31_HMAC_OUTPUT		0xc0
#define S31_HMAC_PADDING		0xf0
#define S31_HMAC_ONE_BLOCK	0xf4
#define S31_HMAC_UP		8
#define S31_KEY_MANAGER_STATIC	0x18
#define S31_KEY_MANAGER_HMAC_EFUSE BIT(2)

static int s31_crypto_metadata_word(struct nvmem_cell *cell, u32 *word)
{
	size_t length;
	void *data = nvmem_cell_read(cell, &length);
	int ret = 0;

	if (IS_ERR(data))
		return PTR_ERR(data);
	if (length != sizeof(*word))
		ret = -EINVAL;
	else
		*word = get_unaligned_le32(data);
	kfree(data);
	return ret;
}

static int s31_crypto_slot_read(struct s31_crypto_dev *dev,
				struct esp32s31_crypto_slot *slot)
{
	u32 purposes, protected;
	int ret;

	if (slot->index >= ESP32S31_CRYPTO_EFUSE_SLOTS)
		return -EINVAL;
	ret = s31_crypto_metadata_word(dev->key_purposes, &purposes);
	if (ret)
		return ret;
	ret = s31_crypto_metadata_word(dev->key_read_protect, &protected);
	if (ret)
		return ret;
	slot->purpose = (purposes >> (slot->index * 5)) & 31;
	slot->flags = protected & BIT(slot->index) ?
		ESP32S31_CRYPTO_SLOT_READ_PROTECTED : 0;
	slot->reserved = 0;
	return 0;
}

static int s31_crypto_slot_info(struct esp32s31_crypto_slot *slot)
{
	struct s31_crypto_dev *dev;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&s31_crypto_admission, flags);
	dev = s31_crypto;
	if (!dev || dev->suspending) {
		ret = dev ? -ESHUTDOWN : -ENODEV;
		spin_unlock_irqrestore(&s31_crypto_admission, flags);
		return ret;
	}
	atomic_inc(&dev->users);
	spin_unlock_irqrestore(&s31_crypto_admission, flags);
	ret = s31_crypto_slot_read(dev, slot);
	s31_crypto_put(dev);
	return ret;
}

static __always_inline int s31_hmac_wait(void __iomem *hmac,
					unsigned long deadline)
{
	if (task_sigpending(current) && __fatal_signal_pending(current))
		return -EINTR;
	if (time_after_eq(jiffies, deadline))
		return -ETIMEDOUT;
	/* Most blocks complete before this first read. Keep the common path
	 * local while retaining the shared bounded poll for a busy engine.
	 */
	if (!readl(hmac + S31_HMAC_BUSY))
		return 0;
	return s31_crypto_poll(hmac + S31_HMAC_BUSY, U32_MAX, 0, 20, 10000);
}

/* GCC's size optimization outlines the MMIO bookkeeping and signal checks
 * on every block. Keep those checks intact but inline them for this hot path.
 */
#if defined(CONFIG_CC_IS_GCC) && defined(CONFIG_CC_OPTIMIZE_FOR_SIZE)
#define S31_HMAC_HOT __attribute__((optimize("O2")))
#else
#define S31_HMAC_HOT
#endif

static __always_inline S31_HMAC_HOT int
s31_hmac_block_inline(void __iomem *hmac, const u8 *input,
		      unsigned long deadline)
{
	int ret = s31_hmac_wait(hmac, deadline);
	void __iomem *dst = hmac + S31_HMAC_INPUT;

	if (ret)
		return ret;
	/* The following writel records both the operand and command writes
	 * before any lock handover; keep the operand fences unchanged.
	 */
	if (IS_ALIGNED((unsigned long)input, sizeof(u32))) {
		const u32 *words = (const u32 *)input;

		/* Use immediate offsets from one base register. The generic raw
		 * accessor can materialize a separate address for each word on
		 * RV32. These are still ordered, individual 32-bit MMIO stores.
		 */
#define S31_HMAC_WRITE_WORD(n) \
		asm volatile("sw %1, %c2(%0)" : : "r"(dst), \
			     "r"(words[n]), "i"((n) * sizeof(u32)) : "memory")
		wmb();
		S31_HMAC_WRITE_WORD(0); S31_HMAC_WRITE_WORD(1);
		S31_HMAC_WRITE_WORD(2); S31_HMAC_WRITE_WORD(3);
		S31_HMAC_WRITE_WORD(4); S31_HMAC_WRITE_WORD(5);
		S31_HMAC_WRITE_WORD(6); S31_HMAC_WRITE_WORD(7);
		S31_HMAC_WRITE_WORD(8); S31_HMAC_WRITE_WORD(9);
		S31_HMAC_WRITE_WORD(10); S31_HMAC_WRITE_WORD(11);
		S31_HMAC_WRITE_WORD(12); S31_HMAC_WRITE_WORD(13);
		S31_HMAC_WRITE_WORD(14); S31_HMAC_WRITE_WORD(15);
		wmb();
#undef S31_HMAC_WRITE_WORD
	} else {
		s31_write_words_impl(hmac, S31_HMAC_INPUT, input,
				     SHA256_BLOCK_SIZE, true);
	}
	writel(1, hmac + S31_HMAC_MESSAGE);
	return 0;
}

/* Keep non-bulk call sites compact while inlining the page loop below. */
static noinline S31_HMAC_HOT int s31_hmac_block(void __iomem *hmac,
					      const u8 *input,
					      unsigned long deadline)
{
	return s31_hmac_block_inline(hmac, input, deadline);
}

static __always_inline int s31_hmac_command(void __iomem *hmac,
					    unsigned int reg,
					    unsigned long deadline)
{
	int ret = s31_hmac_wait(hmac, deadline);

	if (!ret)
		writel(1, hmac + reg);
	return ret;
}

struct s31_hmac_input {
	const u8 *linear;
	struct page **pages;
	unsigned int offset;
};

/* The synchronous caller pins message pages before acquiring the hardware
 * lease and keeps them until cleanup has completed. No key pages are mapped.
 */
static const u8 *s31_hmac_paged_map(const struct s31_hmac_input *input,
				   unsigned int offset, unsigned int length,
				   u8 *scratch, void **mapping)
{
	unsigned int pos = input->offset + offset;
	unsigned int first = min(length, PAGE_SIZE - offset_in_page(pos));
	u8 *data;

	*mapping = kmap_local_page(input->pages[pos >> PAGE_SHIFT]);
	data = (u8 *)*mapping + offset_in_page(pos);
	if (first == length)
		return data;
	memcpy(scratch, data, first);
	kunmap_local(*mapping);
	*mapping = kmap_local_page(input->pages[(pos >> PAGE_SHIFT) + 1]);
	memcpy(scratch + first, *mapping, length - first);
	return scratch;
}

static void s31_hmac_paged_copy(u8 *output, const struct s31_hmac_input *input,
				unsigned int offset, unsigned int length)
{
	const u8 *data;
	void *mapping;

	if (!length)
		return;
	data = s31_hmac_paged_map(input, offset, length, output, &mapping);
	if (data != output)
		memcpy(output, data, length);
	kunmap_local(mapping);
}

/* Keep page iteration out of the short-message loop. The lease and the
 * final padding/cleanup protocol remain shared with linear messages.
 */
static noinline S31_HMAC_HOT int s31_hmac_paged_blocks(void __iomem *hmac,
				const struct s31_hmac_input *input,
				unsigned int length, unsigned long deadline,
				u8 *scratch)
{
	unsigned int offset = 0, remaining = length;
	int ret = 0;

	while (remaining >= SHA256_BLOCK_SIZE) {
		unsigned int pos = input->offset + offset;
		unsigned int bytes = min(remaining,
			PAGE_SIZE - offset_in_page(pos));
		unsigned int blocks;
		const u8 *data;
		void *mapping;

		/* Reuse this thread-local mapping for complete blocks in one
		 * pinned page. kmap_local_page permits scheduling while mapped;
		 * no mapped pointer escapes this synchronous owner. A block that
		 * straddles pages still uses the bounded scratch-copy helper.
		 */
		if (bytes < SHA256_BLOCK_SIZE) {
			data = s31_hmac_paged_map(input, offset,
				SHA256_BLOCK_SIZE, scratch, &mapping);
			blocks = 1;
		} else {
			mapping = kmap_local_page(input->pages[pos >> PAGE_SHIFT]);
			data = (const u8 *)mapping + offset_in_page(pos);
			blocks = bytes / SHA256_BLOCK_SIZE;
		}
		while (blocks--) {
			ret = s31_hmac_block_inline(hmac, data, deadline);
			if (ret)
				break;
			data += SHA256_BLOCK_SIZE;
			offset += SHA256_BLOCK_SIZE;
			remaining -= SHA256_BLOCK_SIZE;
			if (remaining >= SHA256_BLOCK_SIZE) {
				ret = s31_hmac_command(hmac, S31_HMAC_CONTINUE, deadline);
				if (ret)
					break;
			}
		}
		kunmap_local(mapping);
		if (ret)
			return ret;
	}
	return 0;
}
#undef S31_HMAC_HOT

static int s31_crypto_protected_hmac(unsigned int index,
				const struct esp32s31_crypto_km_key *km_key,
				const struct s31_hmac_input *input,
				unsigned int length, u8 output[SHA256_DIGEST_SIZE])
{
	/* As in IDF, SHA and DS must be enabled for HMAC. The composite lease
	 * covers DS reset's dependencies and pins the provider until cleanup.
	 */
	static const unsigned int clocks[] = {
		S31_CRYPTO_HMAC, S31_CRYPTO_SHA, S31_CRYPTO_DS, S31_CRYPTO_KEY_MANAGER,
	};
	struct esp32s31_crypto_slot slot = { .index = index };
	struct s31_crypto_dev *dev;
	void __iomem *hmac;
	unsigned long deadline;
	u8 block[SHA256_BLOCK_SIZE] = { };
	const u8 *linear = input->linear;
	u64 bits = ((u64)length + SHA256_BLOCK_SIZE) * 8;
	unsigned int enabled = 0, remaining = length, i;
	bool started = false;
	int ret, error;

	ret = s31_crypto_sdk_acquire(S31_CRYPTO_HMAC);
	if (ret)
		return ret;
	dev = READ_ONCE(s31_crypto_provider);
	deadline = jiffies + msecs_to_jiffies(2000);
	hmac = dev->base + S31_HMAC_BASE;
	if (!km_key) {
		ret = s31_crypto_slot_read(dev, &slot);
		if (ret)
			goto release;
		if (slot.purpose != S31_HMAC_UP) {
			ret = -ENOKEY;
			goto release;
		}
	}
	for (i = 0; i < ARRAY_SIZE(clocks); i++) {
		ret = s31_crypto_sdk_clock(dev, clocks[i], true);
		if (ret)
			goto cleanup;
		enabled++;
	}
	if (km_key) {
		ret = s31_km_recover_imported(dev, km_key);
		if (ret)
			goto cleanup;
	} else {
		/* Volatile key-source selection only; no fuse programming. */
		writel(readl(dev->key_manager + S31_KEY_MANAGER_STATIC) |
		       S31_KEY_MANAGER_HMAC_EFUSE, dev->key_manager + S31_KEY_MANAGER_STATIC);
		if (!(readl(dev->key_manager + S31_KEY_MANAGER_STATIC) & S31_KEY_MANAGER_HMAC_EFUSE)) {
			ret = -EACCES;
			goto cleanup;
		}
	}
	ret = s31_hmac_command(hmac, S31_HMAC_START, deadline);
	if (ret)
		goto cleanup;
	started = true;
	ret = s31_hmac_wait(hmac, deadline);
	if (ret)
		goto cleanup;
	writel(S31_HMAC_UP, hmac + S31_HMAC_PURPOSE);
	writel(km_key ? 7 : index, hmac + S31_HMAC_KEY);
	writel(1, hmac + S31_HMAC_CONFIGURE);
	ret = s31_hmac_wait(hmac, deadline);
	if (ret)
		goto cleanup;
	if (readl(hmac + S31_HMAC_ERROR)) {
		ret = -EKEYREJECTED;
		goto cleanup;
	}
	if (length < SHA256_BLOCK_SIZE - sizeof(bits)) {
		if (length)
			memcpy(block, linear, length);
		block[length] = 0x80;
		put_unaligned_be64(bits, block + sizeof(block) - sizeof(bits));
		ret = s31_hmac_block(hmac, block, deadline);
		if (!ret)
			ret = s31_hmac_command(hmac, S31_HMAC_ONE_BLOCK, deadline);
	} else {
		if (input->pages) {
			ret = s31_hmac_paged_blocks(hmac, input, length, deadline, block);
			if (ret)
				goto cleanup;
			remaining = length % SHA256_BLOCK_SIZE;
			memset(block, 0, sizeof(block));
			s31_hmac_paged_copy(block, input, length - remaining, remaining);
		} else {
			while (remaining >= SHA256_BLOCK_SIZE) {
				ret = s31_hmac_block(hmac, linear, deadline);
				if (ret)
					goto cleanup;
				linear += SHA256_BLOCK_SIZE;
				remaining -= SHA256_BLOCK_SIZE;
				if (remaining >= SHA256_BLOCK_SIZE) {
					ret = s31_hmac_command(hmac, S31_HMAC_CONTINUE, deadline);
					if (ret)
						goto cleanup;
				}
			}
			memcpy(block, linear, remaining);
		}
		block[remaining] = 0x80;
		if (remaining >= SHA256_BLOCK_SIZE - sizeof(bits)) {
			ret = s31_hmac_command(hmac, S31_HMAC_CONTINUE, deadline);
			if (!ret)
				ret = s31_hmac_block(hmac, block, deadline);
			if (ret)
				goto cleanup;
			memset(block, 0, sizeof(block));
		}
		put_unaligned_be64(bits, block + sizeof(block) - sizeof(bits));
		ret = s31_hmac_command(hmac, S31_HMAC_PADDING, deadline);
		if (!ret)
			ret = s31_hmac_block(hmac, block, deadline);
	}
	if (!ret)
		ret = s31_hmac_wait(hmac, deadline);
	if (!ret)
		for (i = 0; i < SHA256_DIGEST_SIZE; i += sizeof(u32))
			put_unaligned_le32(readl(hmac + S31_HMAC_OUTPUT + i), output + i);
cleanup:
	if (started) {
		writel(2, hmac + S31_HMAC_FINISH);
		s31_clear_words(hmac + S31_HMAC_INPUT, SHA256_BLOCK_SIZE);
	}
	/* Clock disable resets and clears the engine even after a timeout.
	 * No generic HMAC clean call: it would also change JTAG authorization.
	 */
	while (enabled) {
		error = s31_crypto_sdk_clock(dev, clocks[--enabled], false);
		if (!ret)
			ret = error;
	}
release:
	error = s31_crypto_sdk_release(dev, S31_CRYPTO_HMAC);
	if (!ret)
		ret = error;
	if (ret)
		memzero_explicit(output, SHA256_DIGEST_SIZE);
	memzero_explicit(block, sizeof(block));
	return ret;
}

#define S31_DS_BASE		0x4000
#define S31_DS_INPUT		0x800
#define S31_DS_OUTPUT		0xa00
#define S31_DS_START		0xe00
#define S31_DS_CONTINUE		0xe04
#define S31_DS_FINISH		0xe08
#define S31_DS_BUSY		0xe0c
#define S31_DS_CHECK		0xe14
#define S31_DS_STATIC		0xe18
#define S31_DS_IV		0x630
#define S31_HMAC_INVALIDATE_DS	0x64
#define S31_HMAC_DOWN_DS		7
#define S31_HMAC_DOWN_ALL		5
#define S31_KEY_MANAGER_DS_EFUSE	BIT(3)

struct s31_ds_key {
	struct s31_rsa_key public;
	u32 slot;
	bool key_manager;
	struct esp32s31_crypto_km_key recovery;
	u8 iv[16];
	u8 wrapped[ESP32S31_CRYPTO_DS_WRAPPED_BYTES];
};

static int s31_ds_wait_complete(void __iomem *ds)
{
	ktime_t start = ktime_get();
	ktime_t deadline = ktime_add_ms(start, 2000);
	ktime_t next = ktime_add_us(start, 10000);

	/* Use the SDK's 10-ms cadence, anchored to operation start.
	 * Absolute deadlines avoid accumulating scheduler/dispatch time on
	 * every poll. An early unrelated wakeup retains the same deadline.
	 */
	while (readl(ds + S31_DS_BUSY)) {
		ktime_t now = ktime_get(), expires;

		if (ktime_compare(now, deadline) >= 0)
			return -ETIMEDOUT;
		while (ktime_compare(next, now) <= 0)
			next = ktime_add_us(next, 10000);
		expires = ktime_before(next, deadline) ? next : deadline;
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_hrtimeout_range(&expires, 250 * NSEC_PER_USEC,
					HRTIMER_MODE_ABS);
	}
	return 0;
}

static bool s31_crypto_ds_purpose(unsigned int purpose)
{
	return purpose == S31_HMAC_DOWN_DS || purpose == S31_HMAC_DOWN_ALL;
}

static struct s31_ds_key *s31_crypto_ds_import(const void *data, unsigned int length)
{
	const struct esp32s31_crypto_ds_key *blob = data;
	struct esp32s31_crypto_slot slot;
	struct rsa_key raw = { };
	struct s31_ds_key *key;
	size_t extra;
	const u8 *public;
	int ret;

	if (length < sizeof(*blob))
		return ERR_PTR(-EINVAL);
	extra = blob->reserved == ESP32S31_CRYPTO_DS_KEY_MANAGER ?
		sizeof(struct esp32s31_crypto_km_key) : 0;
	public = blob->public_key + extra;
	if (length <= sizeof(*blob) + extra ||
	    (blob->reserved && !extra) ||
	    blob->public_key_len != length - sizeof(*blob) - extra ||
	    (blob->rsa_bits != 1024 && blob->rsa_bits != 2048 &&
	     blob->rsa_bits != 3072 && blob->rsa_bits != 4096))
		return ERR_PTR(-EINVAL);
	if (extra) {
		if (blob->slot != ESP32S31_CRYPTO_SLOT_UNUSED)
			return ERR_PTR(-EINVAL);
		ret = s31_km_validate((const void *)blob->public_key, 3, 3);
		if (ret)
			return ERR_PTR(ret);
	} else {
		slot.index = blob->slot;
		ret = s31_crypto_slot_info(&slot);
		if (ret)
			return ERR_PTR(ret);
		if (!s31_crypto_ds_purpose(slot.purpose))
			return ERR_PTR(-ENOKEY);
	}
	key = kzalloc(sizeof(*key), GFP_KERNEL);
	if (!key)
		return ERR_PTR(-ENOMEM);
	ret = rsa_parse_pub_key(&raw, public, blob->public_key_len);
	if (!ret)
		ret = s31_rsa_load_key(&key->public, &raw, false);
	if (!ret && (s31_limbs_bits(key->public.n, key->public.words) != blob->rsa_bits ||
		     key->public.e_bits < 2 || !(key->public.e[0] & 1) ||
		     s31_limbs_cmp(key->public.e, key->public.n, key->public.words) >= 0))
		ret = -EINVAL;
	if (ret) {
		kfree_sensitive(key);
		return ERR_PTR(ret);
	}
	key->key_manager = !!extra;
	if (extra)
		memcpy(&key->recovery, blob->public_key, extra);
	key->slot = blob->slot;
	memcpy(key->iv, blob->iv, sizeof(key->iv));
	memcpy(key->wrapped, blob->wrapped, sizeof(key->wrapped));
	return key;
}

static int s31_crypto_ds_private(const struct s31_ds_key *key, const u8 *input,
				u8 *output)
{
	static const unsigned int clocks[] = {
		S31_CRYPTO_HMAC, S31_CRYPTO_SHA, S31_CRYPTO_MPI,
		S31_CRYPTO_DS, S31_CRYPTO_KEY_MANAGER,
	};
	struct esp32s31_crypto_slot slot = { .index = key->slot };
	struct s31_crypto_dev *dev;
	u32 *work, *message, *result, *check;
	void __iomem *ds, *hmac;
	unsigned int enabled = 0, i;
	unsigned long deadline;
	u32 value;
	bool started = false, configured = false;
	int ret, error;

	/* All three arrays are bounded by the imported modulus size. */
	work = kcalloc(3, key->public.bytes, GFP_KERNEL);
	if (!work)
		return -ENOMEM;
	message = work;
	result = work + key->public.words;
	check = result + key->public.words;
	s31_be_to_limbs(message, key->public.words, input, key->public.bytes);
	if (s31_limbs_cmp(message, key->public.n, key->public.words) >= 0) {
		ret = -EINVAL;
		goto free;
	}
	ret = s31_crypto_sdk_acquire(S31_CRYPTO_DS);
	if (ret)
		goto free;
	dev = READ_ONCE(s31_crypto_provider);
	ds = dev->base + S31_DS_BASE;
	hmac = dev->base + S31_HMAC_BASE;
	if (!key->key_manager) {
		ret = s31_crypto_slot_read(dev, &slot);
		if (ret)
			goto release;
		if (!s31_crypto_ds_purpose(slot.purpose)) {
			ret = -ENOKEY;
			goto release;
		}
	}
	for (i = 0; i < ARRAY_SIZE(clocks); i++) {
		ret = s31_crypto_sdk_clock(dev, clocks[i], true);
		if (ret)
			goto cleanup;
		enabled++;
	}
	if (key->key_manager) {
		ret = s31_km_recover_imported(dev, &key->recovery);
		if (ret)
			goto cleanup;
		writel(1, ds + S31_DS_STATIC); /* Key Manager AES key */
	} else {
		value = readl(dev->key_manager + S31_KEY_MANAGER_STATIC) |
			S31_KEY_MANAGER_HMAC_EFUSE | S31_KEY_MANAGER_DS_EFUSE;
		writel(value, dev->key_manager + S31_KEY_MANAGER_STATIC);
		if ((readl(dev->key_manager + S31_KEY_MANAGER_STATIC) &
		     (S31_KEY_MANAGER_HMAC_EFUSE | S31_KEY_MANAGER_DS_EFUSE)) !=
		    (S31_KEY_MANAGER_HMAC_EFUSE | S31_KEY_MANAGER_DS_EFUSE)) {
			ret = -EACCES;
			goto cleanup;
		}
		writel(0, ds + S31_DS_STATIC); /* eFuse HMAC downstream key source */
		deadline = jiffies + msecs_to_jiffies(100);
		ret = s31_hmac_command(hmac, S31_HMAC_START, deadline);
		if (ret)
			goto cleanup;
		configured = true;
		ret = s31_hmac_wait(hmac, deadline);
		if (ret)
			goto cleanup;
		writel(S31_HMAC_DOWN_DS, hmac + S31_HMAC_PURPOSE);
		writel(key->slot, hmac + S31_HMAC_KEY);
		writel(1, hmac + S31_HMAC_CONFIGURE);
		ret = s31_hmac_wait(hmac, deadline);
		if (ret)
			goto cleanup;
		if (readl(hmac + S31_HMAC_ERROR)) {
			ret = -EKEYREJECTED;
			goto cleanup;
		}
	}
	writel(1, ds + S31_DS_START);
	started = true;
	/* IDF bounds this key handoff at 1100us; allow Linux polling latency. */
	ret = readl_poll_timeout(ds + S31_DS_BUSY, value, !value, 20, 10000);
	if (ret)
		goto cleanup;
	s31_write_words(ds, 0, key->wrapped, sizeof(key->wrapped));
	s31_write_words(ds, S31_DS_IV, key->iv, sizeof(key->iv));
	/* The load phase cannot consume input until DS_CONTINUE. Overwrite
	 * every active word and clear the unused tail before that command.
	 */
	s31_clear_words(ds + S31_DS_INPUT + key->public.bytes,
			S31_RSA_MAX_BYTES - key->public.bytes);
	s31_write_words(ds, S31_DS_INPUT, (u8 *)message, key->public.bytes);
	writel(1, ds + S31_DS_CONTINUE);
	ret = s31_ds_wait_complete(ds);
	if (ret)
		goto cleanup;
	if (readl(ds + S31_DS_CHECK)) {
		ret = -EKEYREJECTED;
		goto cleanup;
	}
	s31_read_words(ds, S31_DS_OUTPUT, (u8 *)result, key->public.bytes);
cleanup:
	/* IDF invalidates the downstream key before issuing DS_FINISH. */
	if (configured) {
		writel(1, hmac + S31_HMAC_INVALIDATE_DS);
		writel(2, hmac + S31_HMAC_FINISH);
	}
	if (started) {
		writel(1, ds + S31_DS_FINISH);
		error = readl_poll_timeout(ds + S31_DS_BUSY, value, !value, 20, 10000);
		if (!ret)
			ret = error;
		if (!error) {
			s31_clear_words(ds, S31_DS_IV + sizeof(key->iv));
			s31_clear_words(ds + S31_DS_INPUT, S31_RSA_MAX_BYTES);
			s31_clear_words(ds + S31_DS_OUTPUT, S31_RSA_MAX_BYTES);
		} else {
			s31_crypto_fault(dev, s31_crypto_domain_mask(S31_CRYPTO_DS));
		}
	}
	/* Release/reset DS before the independent MPI verification. All engine
	 * locks remain held, so no other client can see a partially cleaned key.
	 */
	while (enabled) {
		error = s31_crypto_sdk_clock(dev, clocks[--enabled], false);
		if (!ret)
			ret = error;
	}
	if (!ret) {
		/* DS uses MPI internally. Reset it into the native public-operation
		 * state without dropping this composite lease or recursively locking.
		 */
		s31_crypto_reset(dev, S31_CRYPTO_RSA_RST);
		ret = s31_rsa_modexp_locked(dev, &key->public, result,
					   key->public.e, key->public.e_bits, false, check);
		if (!ret && crypto_memneq(message, check, key->public.bytes))
			ret = -EKEYREJECTED;
		if (!ret)
			s31_limbs_to_be(output, key->public.bytes, result);
	}
release:
	error = s31_crypto_sdk_release(dev, S31_CRYPTO_DS);
	if (!ret)
		ret = error;
free:
	if (ret)
		memzero_explicit(output, key->public.bytes);
	kfree_sensitive(work);
	return ret;
}
