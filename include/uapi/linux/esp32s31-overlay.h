/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_ESP32S31_OVERLAY_H
#define _UAPI_LINUX_ESP32S31_OVERLAY_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define S31_OVERLAY_MAX_SIZE (128U * 1024U)
#define S31_OVERLAY_MAX_ACTIVE 32
#define S31_OVERLAY_NAME_LEN 32

struct s31_overlay_name {
	char name[S31_OVERLAY_NAME_LEN];
};

struct s31_overlay_item {
	__s32 id;
	char name[S31_OVERLAY_NAME_LEN];
	__aligned_u64 gpios;
};

struct s31_overlay_list {
	__u32 count;
	struct s31_overlay_item items[S31_OVERLAY_MAX_ACTIVE];
};

#define S31_OVERLAY_IOC_MAGIC 'O'
#define S31_OVERLAY_IOC_REMOVE_ALL _IO(S31_OVERLAY_IOC_MAGIC, 0)
#define S31_OVERLAY_IOC_GET_LAST_ID _IOR(S31_OVERLAY_IOC_MAGIC, 1, int)
#define S31_OVERLAY_IOC_REMOVE_NAME \
	_IOW(S31_OVERLAY_IOC_MAGIC, 2, struct s31_overlay_name)
#define S31_OVERLAY_IOC_LIST \
	_IOR(S31_OVERLAY_IOC_MAGIC, 3, struct s31_overlay_list)

#endif
