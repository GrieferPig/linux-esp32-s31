/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_ESP_ISP_H
#define _UAPI_LINUX_ESP_ISP_H

#include <linux/ioctl.h>
#include <linux/types.h>

/* 3x3 Color Correction Matrix (Q2.10 signed fixed-point, IDF isp_ll_ccm_gain_t). */
struct esp_isp_ccm_cfg {
	__u8 enable;
	__s16 matrix[3][3];
};

/* Bilateral Filter (denoising) configuration (IDF isp_ll_bf_*). */
struct esp_isp_bf_cfg {
	__u8 enable;
	__u8 sigma;
	__u8 kernel[3][3];
};

/* Auto White Balance accumulated statistics (IDF isp_ll_awb_get_*). */
struct esp_isp_awb_stats {
	__u32 white_patch_cnt;
	__u32 sum_r;
	__u32 sum_g;
	__u32 sum_b;
};

/* Auto Exposure 5x5 block mean luminance statistics (IDF ISP_AE_BLOCK_MEAN_0..6). */
struct esp_isp_ae_stats {
	__u8 block_mean[25];
};

#define ESP_ISP_IOC_SET_CCM		_IOW('I', 0x01, struct esp_isp_ccm_cfg)
#define ESP_ISP_IOC_SET_BF		_IOW('I', 0x02, struct esp_isp_bf_cfg)
#define ESP_ISP_IOC_GET_AWB_STATS	_IOR('I', 0x03, struct esp_isp_awb_stats)
#define ESP_ISP_IOC_GET_AE_STATS	_IOR('I', 0x04, struct esp_isp_ae_stats)

#endif /* _UAPI_LINUX_ESP_ISP_H */
