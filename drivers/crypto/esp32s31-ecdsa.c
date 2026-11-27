// SPDX-License-Identifier: GPL-2.0-only

#include <linux/math.h>
#include <crypto/ecdh.h>
#include <crypto/internal/ecc.h>
#include <crypto/internal/sig.h>
#include <crypto/sha2.h>
#include <linux/esp32s31-radio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/unaligned.h>

struct s31_ecdsa_key {
	const struct ecc_curve *curve;
	u64 x[ECC_MAX_DIGITS];
	u64 y[ECC_MAX_DIGITS];
	bool valid;
};

static int s31_ecdsa_set_pubkey(struct crypto_sig *tfm, const void *key,
			       unsigned int len)
{
	struct s31_ecdsa_key *ctx = crypto_sig_ctx(tfm);
	unsigned int ndigits = ctx->curve->g.ndigits;
	unsigned int bytes;
	const u8 *data = key;
	struct ecc_point point = ECC_POINT_INIT(ctx->x, ctx->y, ndigits);
	int ret;

	ctx->valid = false;
	if (len < 3 || !(len & 1) || data[0] != 4)
		return -EINVAL;

	bytes = (len - 1) / 2;
	if (DIV_ROUND_UP(bytes, sizeof(u64)) != ndigits)
		return -EINVAL;
	ecc_digits_from_bytes(data + 1, bytes, ctx->x, ndigits);
	ecc_digits_from_bytes(data + 1 + bytes, bytes, ctx->y, ndigits);
	ret = ecc_is_pubkey_valid_full(ctx->curve, &point);
	if (!ret)
		ctx->valid = true;
	return ret;
}

static void s31_ecdsa_to_le(u8 *dst, const u64 *digits, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		put_unaligned_le64(digits[i], dst + i * sizeof(u64));
}

static int s31_ecdsa_verify(struct crypto_sig *tfm, const void *signature,
			    unsigned int siglen, const void *digest,
			    unsigned int digestlen)
{
	const struct ecdsa_raw_sig *sig = signature;
	struct s31_ecdsa_key *ctx = crypto_sig_ctx(tfm);
	unsigned int n = ctx->curve->g.ndigits;
	struct s31_wifi_control request = {
		.operation = S31_RADIO_ECDSA_VERIFY,
		.length = n * sizeof(u64),
	};
	struct s31_radio_ecdsa_request *wire = (void *)request.data;
	u64 hash[ECC_MAX_DIGITS];
	int ret;

	if (!ctx->valid || siglen != sizeof(*sig))
		return -EINVAL;
	if (vli_is_zero(sig->r, n) || vli_cmp(sig->r, ctx->curve->n, n) >= 0 ||
	    vli_is_zero(sig->s, n) || vli_cmp(sig->s, ctx->curve->n, n) >= 0)
		return -EBADMSG;

	digestlen = min(digestlen, request.length);
	ecc_digits_from_bytes(digest, digestlen, hash, n);
	s31_ecdsa_to_le(wire->digest, hash, n);
	s31_ecdsa_to_le(wire->r, sig->r, n);
	s31_ecdsa_to_le(wire->s, sig->s, n);
	s31_ecdsa_to_le(wire->x, ctx->x, n);
	s31_ecdsa_to_le(wire->y, ctx->y, n);

	/* Keep ECC ownership with the ESP-IDF crypto lock used by the radio. */
	ret = esp32s31_radio_security_control(&request);
	if (ret)
		return ret;
	return request.field ? 0 : -EKEYREJECTED;
}

static int s31_ecdsa_p256_init(struct crypto_sig *tfm)
{
	struct s31_ecdsa_key *ctx = crypto_sig_ctx(tfm);

	ctx->curve = ecc_get_curve(ECC_CURVE_NIST_P256);
	return ctx->curve ? 0 : -ENODEV;
}

static int s31_ecdsa_p384_init(struct crypto_sig *tfm)
{
	struct s31_ecdsa_key *ctx = crypto_sig_ctx(tfm);

	ctx->curve = ecc_get_curve(ECC_CURVE_NIST_P384);
	return ctx->curve ? 0 : -ENODEV;
}

static unsigned int s31_ecdsa_key_size(struct crypto_sig *tfm)
{
	struct s31_ecdsa_key *ctx = crypto_sig_ctx(tfm);

	return ctx->curve->nbits;
}

static unsigned int s31_ecdsa_digest_size(struct crypto_sig *tfm)
{
	return SHA512_DIGEST_SIZE;
}

#define S31_ECDSA_ALG(curve) {						\
	.verify = s31_ecdsa_verify,					\
	.set_pub_key = s31_ecdsa_set_pubkey,				\
	.key_size = s31_ecdsa_key_size,					\
	.digest_size = s31_ecdsa_digest_size,				\
	.init = s31_ecdsa_##curve##_init,					\
	.base = {							\
		.cra_name = "ecdsa-nist-" #curve,			\
		.cra_driver_name = "ecdsa-nist-" #curve "-esp32s31",	\
		.cra_priority = 300,					\
		.cra_module = THIS_MODULE,				\
		.cra_ctxsize = sizeof(struct s31_ecdsa_key),		\
	},								\
}

static struct sig_alg s31_ecdsa_algs[] = {
	S31_ECDSA_ALG(p256),
	S31_ECDSA_ALG(p384),
};

static int s31_ecdsa_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *radio_node __free(device_node) =
		of_parse_phandle(dev->of_node, "espressif,radio", 0);
	struct s31_wifi_control control = { .operation = S31_RADIO_SECURITY_CAP };
	struct platform_device *radio;
	struct device_link *link;
	int ret;

	if (!radio_node)
		return -EINVAL;
	radio = of_find_device_by_node(radio_node);
	if (!radio)
		return -EPROBE_DEFER;
	link = device_link_add(dev, &radio->dev, DL_FLAG_AUTOREMOVE_CONSUMER);
	put_device(&radio->dev);
	if (!link)
		return -EINVAL;

	ret = esp32s31_radio_security_control(&control);
	if (ret)
		return dev_err_probe(&pdev->dev, ret == -ENODEV ? -EPROBE_DEFER : ret,
				     "S31 security service unavailable\n");
	if (control.field != S31_RADIO_SECURITY_ABI)
		return -EPROTONOSUPPORT;

	ret = crypto_register_sig(&s31_ecdsa_algs[0]);
	if (ret)
		return ret;
	ret = crypto_register_sig(&s31_ecdsa_algs[1]);
	if (ret)
		crypto_unregister_sig(&s31_ecdsa_algs[0]);
	return ret;
}

static void s31_ecdsa_remove(struct platform_device *pdev)
{
	crypto_unregister_sig(&s31_ecdsa_algs[1]);
	crypto_unregister_sig(&s31_ecdsa_algs[0]);
}

static const struct of_device_id s31_ecdsa_ids[] = {
	{ .compatible = "espressif,esp32s31-ecdsa" },
	{ }
};
MODULE_DEVICE_TABLE(of, s31_ecdsa_ids);

static struct platform_driver s31_ecdsa_driver = {
	.probe = s31_ecdsa_probe,
	.remove = s31_ecdsa_remove,
	.driver = {
		.name = "esp32s31-ecdsa",
		.of_match_table = s31_ecdsa_ids,
	},
};
module_platform_driver(s31_ecdsa_driver);

MODULE_DESCRIPTION("Espressif ESP32-S31 ECDSA verification accelerator");
MODULE_LICENSE("GPL");
