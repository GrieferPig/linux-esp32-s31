// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-P4 rev1.3 Image Signal Processor (ISP) V4L2 control interface.
 *
 * Exposes standard V4L2 controls (brightness, contrast, saturation, hue,
 * sharpness, AWB) and ISP pipeline ioctls (CCM, BF, AWB/AE stats) on 0x500a1000.
 *
 * Hardware reference: ESP32-P4 rev1.3 TRM ch.39 ISP + ESP-IDF
 * soc/esp32p4/register/hw_ver1/soc/isp_reg.h and esp_hal_cam/esp32p4/isp_ll.h.
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <uapi/linux/esp-isp.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>

/* ESP32-P4 rev1.3 TRM ch.39 ISP / IDF hw_ver1/soc/isp_reg.h */
#define ISP_CLK_EN		0x04
#define ISP_CNTL		0x08
#define ISP_CCM_EN		BIT(7)
#define ISP_BF_EN		BIT(8)
#define ISP_SHARP_EN		BIT(12)
#define ISP_COLOR_EN		BIT(14)
#define ISP_AWB_EN		BIT(17)
#define ISP_AE_EN		BIT(19)

#define ISP_CCM_COEF0		0x14
#define ISP_CCM_COEF1		0x18
#define ISP_CCM_COEF3		0x1c
#define ISP_CCM_COEF4		0x20
#define ISP_CCM_COEF5		0x24

#define ISP_BF_SIGMA		0x2c
#define ISP_BF_GAU0		0x30
#define ISP_BF_GAU1		0x34

#define ISP_AE_BLOCK_MEAN_0	0xd8
#define ISP_SHARP_CTRL0		0xf4
#define ISP_AWB_MODE		0x164
#define ISP_AWB_RESULT0		0x17c
#define ISP_AWB_RESULT1		0x180
#define ISP_AWB_RESULT2		0x184
#define ISP_AWB_RESULT3		0x188

#define ISP_COLOR_CTRL		0x18c
#define ISP_COLOR_SATURATION	GENMASK(7, 0)
#define ISP_COLOR_HUE		GENMASK(15, 8)
#define ISP_COLOR_CONTRAST	GENMASK(23, 16)
#define ISP_COLOR_BRIGHTNESS	GENMASK(31, 24)

struct p4_isp {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct v4l2_device v4l2;
	struct video_device video;
	struct v4l2_ctrl_handler ctrls;
	struct mutex lock;
};

static void isp_rmw(struct p4_isp *isp, u32 reg, u32 mask, u32 val)
{
	u32 cur = readl(isp->base + reg);

	writel((cur & ~mask) | (val & mask), isp->base + reg);
}

static int p4_isp_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct p4_isp *isp = container_of(ctrl->handler, struct p4_isp, ctrls);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		isp_rmw(isp, ISP_CNTL, ISP_COLOR_EN, ISP_COLOR_EN);
		isp_rmw(isp, ISP_COLOR_CTRL, ISP_COLOR_BRIGHTNESS,
			FIELD_PREP(ISP_COLOR_BRIGHTNESS, (u8)(s8)ctrl->val));
		return 0;
	case V4L2_CID_CONTRAST:
		isp_rmw(isp, ISP_CNTL, ISP_COLOR_EN, ISP_COLOR_EN);
		isp_rmw(isp, ISP_COLOR_CTRL, ISP_COLOR_CONTRAST,
			FIELD_PREP(ISP_COLOR_CONTRAST, (u8)ctrl->val));
		return 0;
	case V4L2_CID_SATURATION:
		isp_rmw(isp, ISP_CNTL, ISP_COLOR_EN, ISP_COLOR_EN);
		isp_rmw(isp, ISP_COLOR_CTRL, ISP_COLOR_SATURATION,
			FIELD_PREP(ISP_COLOR_SATURATION, (u8)ctrl->val));
		return 0;
	case V4L2_CID_HUE:
		isp_rmw(isp, ISP_CNTL, ISP_COLOR_EN, ISP_COLOR_EN);
		isp_rmw(isp, ISP_COLOR_CTRL, ISP_COLOR_HUE,
			FIELD_PREP(ISP_COLOR_HUE, (u8)ctrl->val));
		return 0;
	case V4L2_CID_SHARPNESS:
		isp_rmw(isp, ISP_CNTL, ISP_SHARP_EN, ctrl->val ? ISP_SHARP_EN : 0);
		if (ctrl->val)
			writel(ctrl->val & 0xff, isp->base + ISP_SHARP_CTRL0);
		return 0;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		isp_rmw(isp, ISP_CNTL, ISP_AWB_EN, ctrl->val ? ISP_AWB_EN : 0);
		isp_rmw(isp, ISP_AWB_MODE, BIT(4), ctrl->val ? BIT(4) : 0);
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops p4_isp_ctrl_ops = {
	.s_ctrl = p4_isp_s_ctrl,
};

static int p4_isp_querycap(struct file *file, void *fh,
			   struct v4l2_capability *cap)
{
	struct p4_isp *isp = video_drvdata(file);

	strscpy(cap->driver, "esp32p4-isp", sizeof(cap->driver));
	strscpy(cap->card, "ESP32-P4 ISP Control", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(isp->dev));
	return 0;
}

static long p4_isp_default_ioctl(struct file *file, void *fh, bool valid_prio,
				 unsigned int cmd, void *arg)
{
	struct p4_isp *isp = video_drvdata(file);

	switch (cmd) {
	case ESP_ISP_IOC_SET_CCM: {
		const struct esp_isp_ccm_cfg *c = arg;
		u32 r0, r1, r2, r3, r4;

		r0 = ((u32)(c->matrix[0][0] & 0x1fff)) |
		     ((u32)(c->matrix[0][1] & 0x1fff) << 13);
		r1 = ((u32)(c->matrix[0][2] & 0x1fff)) |
		     ((u32)(c->matrix[1][0] & 0x1fff) << 13);
		r2 = ((u32)(c->matrix[1][1] & 0x1fff)) |
		     ((u32)(c->matrix[1][2] & 0x1fff) << 13);
		r3 = ((u32)(c->matrix[2][0] & 0x1fff)) |
		     ((u32)(c->matrix[2][1] & 0x1fff) << 13);
		r4 = (u32)(c->matrix[2][2] & 0x1fff);
		writel(r0, isp->base + ISP_CCM_COEF0);
		writel(r1, isp->base + ISP_CCM_COEF1);
		writel(r2, isp->base + ISP_CCM_COEF3);
		writel(r3, isp->base + ISP_CCM_COEF4);
		writel(r4, isp->base + ISP_CCM_COEF5);
		isp_rmw(isp, ISP_CNTL, ISP_CCM_EN, c->enable ? ISP_CCM_EN : 0);
		return 0;
	}
	case ESP_ISP_IOC_SET_BF: {
		const struct esp_isp_bf_cfg *b = arg;
		u32 g0, g1;

		writel(b->sigma & 0x3f, isp->base + ISP_BF_SIGMA);
		g0 = ((u32)(b->kernel[0][0] & 0xf)) |
		     ((u32)(b->kernel[0][1] & 0xf) << 4) |
		     ((u32)(b->kernel[0][2] & 0xf) << 8) |
		     ((u32)(b->kernel[1][0] & 0xf) << 12) |
		     ((u32)(b->kernel[1][1] & 0xf) << 16) |
		     ((u32)(b->kernel[1][2] & 0xf) << 20);
		g1 = ((u32)(b->kernel[2][0] & 0xf)) |
		     ((u32)(b->kernel[2][1] & 0xf) << 4) |
		     ((u32)(b->kernel[2][2] & 0xf) << 8);
		writel(g0, isp->base + ISP_BF_GAU0);
		writel(g1, isp->base + ISP_BF_GAU1);
		isp_rmw(isp, ISP_CNTL, ISP_BF_EN, b->enable ? ISP_BF_EN : 0);
		return 0;
	}
	case ESP_ISP_IOC_GET_AWB_STATS: {
		struct esp_isp_awb_stats *st = arg;

		st->white_patch_cnt = readl(isp->base + ISP_AWB_RESULT0);
		st->sum_r = readl(isp->base + ISP_AWB_RESULT1);
		st->sum_g = readl(isp->base + ISP_AWB_RESULT2);
		st->sum_b = readl(isp->base + ISP_AWB_RESULT3);
		return 0;
	}
	case ESP_ISP_IOC_GET_AE_STATS: {
		struct esp_isp_ae_stats *ae = arg;
		unsigned int i;

		for (i = 0; i < 6; i++) {
			u32 v = readl(isp->base + ISP_AE_BLOCK_MEAN_0 + i * 4);
			ae->block_mean[i * 4 + 0] = v & 0xff;
			ae->block_mean[i * 4 + 1] = (v >> 8) & 0xff;
			ae->block_mean[i * 4 + 2] = (v >> 16) & 0xff;
			ae->block_mean[i * 4 + 3] = (v >> 24) & 0xff;
		}
		ae->block_mean[24] = readl(isp->base + ISP_AE_BLOCK_MEAN_0 + 24) & 0xff;
		return 0;
	}
	default:
		return -ENOTTY;
	}
}

static const struct v4l2_ioctl_ops p4_isp_ioctl_ops = {
	.vidioc_querycap = p4_isp_querycap,
	.vidioc_default = p4_isp_default_ioctl,
};

static const struct v4l2_file_operations p4_isp_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = v4l2_fh_release,
	.unlocked_ioctl = video_ioctl2,
};

static int p4_isp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct p4_isp *isp;
	int ret;

	isp = devm_kzalloc(dev, sizeof(*isp), GFP_KERNEL);
	if (!isp)
		return -ENOMEM;
	isp->dev = dev;
	mutex_init(&isp->lock);
	platform_set_drvdata(pdev, isp);

	/* Non-exclusive ioremap so ISP controls coexist with CSI/ISP capture. */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;
	isp->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!isp->base)
		return -ENOMEM;

	isp->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(isp->clk))
		return PTR_ERR(isp->clk);

	isp_rmw(isp, ISP_CLK_EN, BIT(0), BIT(0));

	ret = v4l2_device_register(dev, &isp->v4l2);
	if (ret)
		return ret;

	v4l2_ctrl_handler_init(&isp->ctrls, 6);
	v4l2_ctrl_new_std(&isp->ctrls, &p4_isp_ctrl_ops, V4L2_CID_BRIGHTNESS, -128, 127, 1, 0);
	v4l2_ctrl_new_std(&isp->ctrls, &p4_isp_ctrl_ops, V4L2_CID_CONTRAST, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&isp->ctrls, &p4_isp_ctrl_ops, V4L2_CID_SATURATION, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&isp->ctrls, &p4_isp_ctrl_ops, V4L2_CID_HUE, 0, 255, 1, 0);
	v4l2_ctrl_new_std(&isp->ctrls, &p4_isp_ctrl_ops, V4L2_CID_SHARPNESS, 0, 255, 1, 0);
	v4l2_ctrl_new_std(&isp->ctrls, &p4_isp_ctrl_ops, V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 0);
	ret = isp->ctrls.error;
	if (ret)
		goto err_ctrl;
	isp->v4l2.ctrl_handler = &isp->ctrls;

	isp->video = (struct video_device) {
		.name = "esp32p4-isp",
		.vfl_dir = VFL_DIR_RX,
		.fops = &p4_isp_fops,
		.ioctl_ops = &p4_isp_ioctl_ops,
		.release = video_device_release_empty,
		.v4l2_dev = &isp->v4l2,
		.ctrl_handler = &isp->ctrls,
		.lock = &isp->lock,
		.device_caps = V4L2_CAP_META_CAPTURE,
	};
	video_set_drvdata(&isp->video, isp);
	ret = video_register_device(&isp->video, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_ctrl;

	dev_info(dev, "registered ISP control interface /dev/%s\n",
		 video_device_node_name(&isp->video));
	return 0;

err_ctrl:
	v4l2_ctrl_handler_free(&isp->ctrls);
	v4l2_device_unregister(&isp->v4l2);
	return ret;
}

static void p4_isp_remove(struct platform_device *pdev)
{
	struct p4_isp *isp = platform_get_drvdata(pdev);

	video_unregister_device(&isp->video);
	v4l2_ctrl_handler_free(&isp->ctrls);
	v4l2_device_unregister(&isp->v4l2);
}

static const struct of_device_id p4_isp_match[] = {
	{ .compatible = "espressif,esp32p4-isp" },
	{ }
};
MODULE_DEVICE_TABLE(of, p4_isp_match);

static struct platform_driver p4_isp_driver = {
	.probe = p4_isp_probe,
	.remove = p4_isp_remove,
	.driver = {
		.name = "esp32p4-isp",
		.of_match_table = p4_isp_match,
	},
};
module_platform_driver(p4_isp_driver);

MODULE_DESCRIPTION("ESP32-P4 ISP V4L2 control interface driver");
MODULE_LICENSE("GPL");
