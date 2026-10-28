// SPDX-License-Identifier: GPL-2.0-only
/* Asymmetric application backend. Symmetric/hash applications use AF_ALG.
 * A file owns its transforms, never the platform allocation; open keys can
 * safely outlive unbind and requests then fail through normal admission.
 */

#define S31_USER_MAX_KEY 8192U

struct s31_crypto_file {
	struct mutex lock;
	struct crypto_akcipher *rsa;
	struct crypto_kpp *ecdh;
	struct crypto_sig *ecdsa;
	struct s31_ds_key *ds;
	struct esp32s31_crypto_km_key *km;
	bool private;
	int slot;
};

static void s31_crypto_file_clear(struct s31_crypto_file *ctx)
{
	if (ctx->rsa)
		crypto_free_akcipher(ctx->rsa);
	if (ctx->ecdh)
		crypto_free_kpp(ctx->ecdh);
	if (ctx->ecdsa)
		crypto_free_sig(ctx->ecdsa);
	ctx->ecdsa = NULL;
	ctx->rsa = NULL;
	ctx->ecdh = NULL;
	kfree_sensitive(ctx->ds);
	ctx->ds = NULL;
	kfree_sensitive(ctx->km);
	ctx->km = NULL;
	ctx->private = false;
	ctx->slot = -1;
}

static int s31_crypto_file_open(struct inode *inode, struct file *file)
{
	struct s31_crypto_file *ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);

	if (!ctx)
		return -ENOMEM;
	mutex_init(&ctx->lock);
	ctx->slot = -1;
	file->private_data = ctx;
	return nonseekable_open(inode, file);
}

static int s31_crypto_file_release(struct inode *inode, struct file *file)
{
	struct s31_crypto_file *ctx = file->private_data;

	s31_crypto_file_clear(ctx);
	kfree_sensitive(ctx);
	return 0;
}

static int s31_crypto_file_import(struct s31_crypto_file *ctx, void __user *arg)
{
	struct esp32s31_crypto_import key;
	struct ecdh secret;
	void *bytes, *encoded;
	unsigned int length;
	int ret;

	s31_crypto_file_clear(ctx);
	if (copy_from_user(&key, arg, sizeof(key)))
		return -EFAULT;
	if (key.reserved || key.flags & ~ESP32S31_CRYPTO_KEY_PRIVATE ||
	    !key.key_len || key.key_len > S31_USER_MAX_KEY)
		return -EINVAL;
	bytes = memdup_user(u64_to_user_ptr(key.key), key.key_len);
	if (IS_ERR(bytes))
		return PTR_ERR(bytes);
	ctx->private = key.flags & ESP32S31_CRYPTO_KEY_PRIVATE;
	ret = -EINVAL;
	if (key.algorithm == ESP32S31_CRYPTO_HMAC_KEY_MANAGER) {
		if (!ctx->private || key.key_len != sizeof(*ctx->km))
			goto out;
		ret = s31_km_validate(bytes, 2, 3);
		if (ret)
			goto out;
		ctx->km = kmemdup(bytes, key.key_len, GFP_KERNEL);
		ret = ctx->km ? 0 : -ENOMEM;
	} else if (key.algorithm == ESP32S31_CRYPTO_HMAC_EFUSE) {
		struct esp32s31_crypto_slot slot = { };

		if (key.key_len != sizeof(u32) || !ctx->private)
			goto out;
		slot.index = get_unaligned((u32 *)bytes);
		ret = s31_crypto_slot_info(&slot);
		if (!ret && slot.purpose != S31_HMAC_UP)
			ret = -ENOKEY;
		if (!ret)
			ctx->slot = slot.index;
	} else if (key.algorithm == ESP32S31_CRYPTO_DS_RSA && ctx->private) {
		const struct esp32s31_crypto_ds_key *blob = bytes;

		ctx->ds = s31_crypto_ds_import(bytes, key.key_len);
		if (IS_ERR(ctx->ds)) {
			ret = PTR_ERR(ctx->ds);
			ctx->ds = NULL;
			goto out;
		}
		ctx->rsa = crypto_alloc_akcipher("rsa-esp32s31", 0, 0);
		if (IS_ERR(ctx->rsa)) {
			ret = PTR_ERR(ctx->rsa);
			ctx->rsa = NULL;
			goto out;
		}
		ret = s31_rsa_setkey(ctx->rsa, blob->public_key +
			(ctx->ds->key_manager ? sizeof(ctx->ds->recovery) : 0),
			blob->public_key_len, false, &ctx->ds->public);
	} else if (key.algorithm >= ESP32S31_CRYPTO_ECDSA_P192 &&
		   key.algorithm <= ESP32S31_CRYPTO_ECDSA_P384) {
		static const char * const names[] = {
			"ecdsa-nist-p192-esp32s31", "ecdsa-nist-p256-esp32s31",
			"ecdsa-nist-p384-esp32s31",
		};

		ctx->ecdsa = crypto_alloc_sig(names[key.algorithm - ESP32S31_CRYPTO_ECDSA_P192],
					      0, 0);
		if (IS_ERR(ctx->ecdsa)) {
			ret = PTR_ERR(ctx->ecdsa);
			ctx->ecdsa = NULL;
			goto out;
		}
		ret = ctx->private ? crypto_sig_set_privkey(ctx->ecdsa, bytes, key.key_len) :
			crypto_sig_set_pubkey(ctx->ecdsa, bytes, key.key_len);
	} else if (key.algorithm == ESP32S31_CRYPTO_RSA) {
		ctx->rsa = crypto_alloc_akcipher("rsa-esp32s31", 0, 0);
		if (IS_ERR(ctx->rsa)) {
			ret = PTR_ERR(ctx->rsa);
			ctx->rsa = NULL;
			goto out;
		}
		ret = ctx->private ? crypto_akcipher_set_priv_key(ctx->rsa, bytes, key.key_len) :
			crypto_akcipher_set_pub_key(ctx->rsa, bytes, key.key_len);
		if (!ret && !((struct s31_rsa_ctx *)akcipher_tfm_ctx(ctx->rsa))->key.words)
			ret = -EOPNOTSUPP;
	} else if (ctx->private &&
		   ((key.algorithm == ESP32S31_CRYPTO_ECDH_P256 && key.key_len == 32) ||
		    (key.algorithm == ESP32S31_CRYPTO_ECDH_P384 && key.key_len == 48))) {
		ctx->ecdh = crypto_alloc_kpp(key.key_len == 32 ?
			"ecdh-nist-p256-esp32s31" : "ecdh-nist-p384-esp32s31", 0, 0);
		if (IS_ERR(ctx->ecdh)) {
			ret = PTR_ERR(ctx->ecdh);
			ctx->ecdh = NULL;
			goto out;
		}
		secret.key = bytes;
		secret.key_size = key.key_len;
		length = crypto_ecdh_key_len(&secret);
		encoded = kmalloc(length, GFP_KERNEL);
		if (!encoded) {
			ret = -ENOMEM;
			goto out;
		}
		ret = crypto_ecdh_encode_key(encoded, length, &secret);
		if (!ret)
			ret = crypto_kpp_set_secret(ctx->ecdh, encoded, length);
		kfree_sensitive(encoded);
	}
out:
	kfree_sensitive(bytes);
	if (ret)
		s31_crypto_file_clear(ctx);
	return ret;
}

static int s31_crypto_file_hmac(struct s31_crypto_file *ctx,
				struct esp32s31_crypto_request *job, void __user *arg)
{
	u8 output[SHA256_DIGEST_SIZE];
	struct s31_hmac_input input = { };
	u8 *copy = NULL;
	unsigned int nr_pages = 0;
	int ret;

	if (ctx->slot < 0 && !ctx->km)
		return -ENOKEY;
	if (job->input_len > ESP32S31_CRYPTO_HMAC_MAX_MESSAGE)
		return -EMSGSIZE;
	if (job->output_len < sizeof(output)) {
		job->output_len = sizeof(output);
		return copy_to_user(arg, job, sizeof(*job)) ? -EFAULT : -EMSGSIZE;
	}
	job->output_len = 0;
	if (job->input_len >= SZ_16K) {
		unsigned long start = (unsigned long)u64_to_user_ptr(job->input);
		int pinned;

		input.offset = offset_in_page(start);
		nr_pages = DIV_ROUND_UP(input.offset + job->input_len, PAGE_SIZE);
		input.pages = kcalloc(nr_pages, sizeof(*input.pages), GFP_KERNEL);
		if (!input.pages) {
			ret = -ENOMEM;
			goto out;
		}
		ret = mmap_read_lock_killable(current->mm);
		if (ret) {
			nr_pages = 0;
			goto out;
		}
		pinned = pin_user_pages(start & PAGE_MASK, nr_pages, 0, input.pages);
		mmap_read_unlock(current->mm);
		if (pinned != nr_pages) {
			if (pinned > 0)
				unpin_user_pages(input.pages, pinned);
			nr_pages = 0;
			ret = pinned < 0 ? pinned : -EFAULT;
			goto out;
		}
	} else if (job->input_len) {
		copy = memdup_user(u64_to_user_ptr(job->input), job->input_len);
		if (IS_ERR(copy)) {
			ret = PTR_ERR(copy);
			copy = NULL;
			goto out;
		}
		input.linear = copy;
	}
	ret = s31_crypto_protected_hmac(ctx->slot, ctx->km, &input, job->input_len, output);
	if (!ret) {
		if (copy_to_user(u64_to_user_ptr(job->output), output, sizeof(output)))
			ret = -EFAULT;
		else
			job->output_len = sizeof(output);
	}
out:
	memzero_explicit(output, sizeof(output));
	if (nr_pages && input.pages)
		unpin_user_pages(input.pages, nr_pages);
	kfree(input.pages);
	kfree_sensitive(copy);
	return copy_to_user(arg, job, sizeof(*job)) ? -EFAULT : ret;
}

static int s31_crypto_file_ecdsa(struct s31_crypto_file *ctx,
				struct esp32s31_crypto_request *job, void __user *arg)
{
	struct ecdsa_raw_sig signature = { };
	struct s31_ecdsa_ctx *key;
	u8 input[SHA512_DIGEST_SIZE + 2 * S31_ECC_MAX_BYTES];
	u8 output[1 + 2 * S31_ECC_MAX_BYTES] = { };
	unsigned int needed, digest, digits;
	int ret;

	if (!ctx->ecdsa)
		return -ENOKEY;
	key = crypto_sig_ctx(ctx->ecdsa);
	digits = key->bytes / sizeof(u64);
	if (job->operation == ESP32S31_CRYPTO_ECDSA_PUBLIC) {
		if (job->input_len)
			return -EINVAL;
		needed = 1 + 2 * key->bytes;
	} else if (job->operation == ESP32S31_CRYPTO_ECDSA_SIGN) {
		if (!ctx->private)
			return -ENOKEY;
		if (!job->input_len || job->input_len > SHA512_DIGEST_SIZE)
			return -EINVAL;
		needed = 2 * key->bytes;
	} else {
		if (job->input_len <= 2 * key->bytes ||
		    job->input_len > 2 * key->bytes + SHA512_DIGEST_SIZE)
			return -EINVAL;
		needed = 0;
	}
	if (job->output_len < needed) {
		job->output_len = needed;
		return copy_to_user(arg, job, sizeof(*job)) ? -EFAULT : -EMSGSIZE;
	}
	job->output_len = 0;
	if (job->input_len && copy_from_user(input, u64_to_user_ptr(job->input),
					    job->input_len)) {
		ret = -EFAULT;
		goto out;
	}
	if (job->operation == ESP32S31_CRYPTO_ECDSA_PUBLIC) {
		memcpy(output, key->point, needed);
		ret = 0;
	} else if (job->operation == ESP32S31_CRYPTO_ECDSA_SIGN) {
		ret = crypto_sig_sign(ctx->ecdsa, input, job->input_len,
				      &signature, sizeof(signature));
		if (ret == sizeof(signature)) {
			s31_ecdsa_raw_to_bytes(&signature, output, key->bytes);
			ret = 0;
		} else if (ret >= 0) {
			ret = -EIO;
		}
	} else {
		digest = job->input_len - 2 * key->bytes;
		ecc_digits_from_bytes(input + digest, key->bytes, signature.r, digits);
		ecc_digits_from_bytes(input + digest + key->bytes, key->bytes,
				      signature.s, digits);
		ret = crypto_sig_verify(ctx->ecdsa, &signature, sizeof(signature),
					input, digest);
	}
	if (!ret && needed && copy_to_user(u64_to_user_ptr(job->output), output, needed))
		ret = -EFAULT;
	if (!ret)
		job->output_len = needed;
out:
	memzero_explicit(&signature, sizeof(signature));
	memzero_explicit(input, sizeof(input));
	memzero_explicit(output, sizeof(output));
	return copy_to_user(arg, job, sizeof(*job)) ? -EFAULT : ret;
}

static int s31_crypto_file_request(struct s31_crypto_file *ctx, void __user *arg)
{
	struct esp32s31_crypto_request job;
	struct akcipher_request *rsa_req;
	struct kpp_request *ecdh_req;
	struct scatterlist source, destination;
	DECLARE_CRYPTO_WAIT(wait);
	u8 *input = NULL, *output = NULL;
	unsigned int needed;
	int ret;

	if (copy_from_user(&job, arg, sizeof(job)))
		return -EFAULT;
	if (job.reserved)
		return -EINVAL;
	if (job.operation == ESP32S31_CRYPTO_HMAC_SHA256)
		return s31_crypto_file_hmac(ctx, &job, arg);
	if (job.operation >= ESP32S31_CRYPTO_ECDSA_PUBLIC &&
	    job.operation <= ESP32S31_CRYPTO_ECDSA_VERIFY)
		return s31_crypto_file_ecdsa(ctx, &job, arg);
	if (job.input_len > S31_RSA_MAX_BYTES)
		return -EINVAL;
	switch (job.operation) {
	case ESP32S31_CRYPTO_RSA_PUBLIC:
	case ESP32S31_CRYPTO_RSA_PRIVATE:
		if (job.operation == ESP32S31_CRYPTO_RSA_PRIVATE && ctx->ds)
			return -EOPNOTSUPP;
		if (!ctx->rsa || (job.operation == ESP32S31_CRYPTO_RSA_PRIVATE && !ctx->private))
			return -ENOKEY;
		needed = crypto_akcipher_maxsize(ctx->rsa);
		if (!job.input_len || job.input_len > needed)
			return -EINVAL;
		break;
	case ESP32S31_CRYPTO_DS_PRIVATE:
		if (!ctx->ds)
			return -ENOKEY;
		needed = ctx->ds->public.bytes;
		if (job.input_len != needed)
			return -EINVAL;
		break;
	case ESP32S31_CRYPTO_ECDH_PUBLIC:
	case ESP32S31_CRYPTO_ECDH_SHARED:
		if (!ctx->ecdh)
			return -ENOKEY;
		needed = crypto_kpp_maxsize(ctx->ecdh);
		if (job.operation == ESP32S31_CRYPTO_ECDH_PUBLIC) {
			if (job.input_len)
				return -EINVAL;
		} else {
			if (job.input_len != needed)
				return -EINVAL;
			needed /= 2;
		}
		break;
	default:
		return -EINVAL;
	}
	if (needed > S31_RSA_MAX_BYTES)
		return -EOVERFLOW;
	if (job.output_len < needed) {
		job.output_len = needed;
		return copy_to_user(arg, &job, sizeof(job)) ? -EFAULT : -EMSGSIZE;
	}
	job.output_len = 0;
	if (job.input_len) {
		input = memdup_user(u64_to_user_ptr(job.input), job.input_len);
		if (IS_ERR(input)) {
			ret = PTR_ERR(input);
			input = NULL;
			goto out;
		}
		sg_init_one(&source, input, job.input_len);
	}
	output = kzalloc(needed, GFP_KERNEL);
	if (!output) {
		ret = -ENOMEM;
		goto out;
	}
	sg_init_one(&destination, output, needed);
	if (job.operation == ESP32S31_CRYPTO_DS_PRIVATE) {
		ret = s31_crypto_ds_private(ctx->ds, input, output);
		if (!ret)
			job.output_len = needed;
	} else if (ctx->rsa) {
		rsa_req = akcipher_request_alloc(ctx->rsa, GFP_KERNEL);
		if (!rsa_req) {
			ret = -ENOMEM;
			goto out;
		}
		akcipher_request_set_callback(rsa_req, CRYPTO_TFM_REQ_MAY_SLEEP |
				       CRYPTO_TFM_REQ_MAY_BACKLOG,
					       crypto_req_done, &wait);
		akcipher_request_set_crypt(rsa_req, &source, &destination, job.input_len, needed);
		ret = crypto_wait_req(job.operation == ESP32S31_CRYPTO_RSA_PRIVATE ?
			crypto_akcipher_decrypt(rsa_req) : crypto_akcipher_encrypt(rsa_req), &wait);
		if (!ret)
			job.output_len = rsa_req->dst_len;
		akcipher_request_free(rsa_req);
	} else {
		ecdh_req = kpp_request_alloc(ctx->ecdh, GFP_KERNEL);
		if (!ecdh_req) {
			ret = -ENOMEM;
			goto out;
		}
		kpp_request_set_callback(ecdh_req, CRYPTO_TFM_REQ_MAY_SLEEP |
				  CRYPTO_TFM_REQ_MAY_BACKLOG, crypto_req_done, &wait);
		kpp_request_set_input(ecdh_req, input ? &source : NULL, job.input_len);
		kpp_request_set_output(ecdh_req, &destination, needed);
		ret = crypto_wait_req(job.operation == ESP32S31_CRYPTO_ECDH_SHARED ?
			crypto_kpp_compute_shared_secret(ecdh_req) : crypto_kpp_generate_public_key(ecdh_req), &wait);
		if (!ret)
			job.output_len = ecdh_req->dst_len;
		kpp_request_free(ecdh_req);
	}
	if (!ret && job.output_len > needed) {
		job.output_len = 0;
		ret = -EIO;
	}
	if (!ret && copy_to_user(u64_to_user_ptr(job.output), output, job.output_len))
		ret = -EFAULT;
out:
	/* An admitted request must not advertise result bytes on failure, including input
	 * allocation failure and a partial result copy to an invalid user page.
	 */
	if (ret)
		job.output_len = 0;
	kfree_sensitive(input);
	kfree_sensitive(output);
	if (copy_to_user(arg, &job, sizeof(job)))
		return -EFAULT;
	return ret;
}

static long s31_crypto_file_ioctl(struct file *file, unsigned int command, unsigned long arg)
{
	struct s31_crypto_file *ctx = file->private_data;
	struct esp32s31_crypto_caps caps = {
		.version = ESP32S31_CRYPTO_ABI_VERSION,
		.algorithms = BIT(ESP32S31_CRYPTO_RSA - 1) | BIT(ESP32S31_CRYPTO_ECDH_P256 - 1) |
			      BIT(ESP32S31_CRYPTO_ECDH_P384 - 1) | BIT(ESP32S31_CRYPTO_HMAC_EFUSE - 1) |
			      BIT(ESP32S31_CRYPTO_DS_RSA - 1) |
			      BIT(ESP32S31_CRYPTO_ECDSA_P192 - 1) |
			      BIT(ESP32S31_CRYPTO_ECDSA_P256 - 1) |
			      BIT(ESP32S31_CRYPTO_ECDSA_P384 - 1) |
			      BIT(ESP32S31_CRYPTO_HMAC_KEY_MANAGER - 1),
		.max_key_bytes = S31_USER_MAX_KEY,
	};
	void __user *ptr = (void __user *)arg;
	long ret;

	if (command == ESP32S31_CRYPTO_IOC_CAPS)
		return copy_to_user(ptr, &caps, sizeof(caps)) ? -EFAULT : 0;
	if (command == ESP32S31_CRYPTO_IOC_SLOT) {
		struct esp32s31_crypto_slot slot;

		if (copy_from_user(&slot, ptr, sizeof(slot)))
			return -EFAULT;
		if (slot.reserved)
			return -EINVAL;
		ret = s31_crypto_slot_info(&slot);
		return ret ? ret : copy_to_user(ptr, &slot, sizeof(slot)) ? -EFAULT : 0;
	}
	if (command == ESP32S31_CRYPTO_IOC_KM_DEPLOY) {
		struct esp32s31_crypto_km_deploy *job;

		job = memdup_user(ptr, sizeof(*job));
		if (IS_ERR(job))
			return PTR_ERR(job);
		ret = s31_km_deploy(job);
		/* Never echo software init keys back through the ioctl. */
		memzero_explicit(job->init_key, sizeof(job->init_key));
		memzero_explicit(job->k2_info, sizeof(job->k2_info));
		memzero_explicit(job->encrypted_key, sizeof(job->encrypted_key));
		if (copy_to_user(ptr, job, sizeof(*job)))
			ret = -EFAULT;
		kfree_sensitive(job);
		return ret;
	}
	if (mutex_lock_interruptible(&ctx->lock))
		return -ERESTARTSYS;
	if (command == ESP32S31_CRYPTO_IOC_IMPORT)
		ret = s31_crypto_file_import(ctx, ptr);
	else if (command == ESP32S31_CRYPTO_IOC_REQUEST)
		ret = s31_crypto_file_request(ctx, ptr);
	else
		ret = -ENOTTY;
	mutex_unlock(&ctx->lock);
	return ret;
}

static const struct file_operations s31_crypto_fops = {
	.owner = THIS_MODULE,
	.open = s31_crypto_file_open,
	.release = s31_crypto_file_release,
	.unlocked_ioctl = s31_crypto_file_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = compat_ptr_ioctl,
#endif
};
