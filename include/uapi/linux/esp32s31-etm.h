/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_ESP32S31_ETM_H
#define _UAPI_LINUX_ESP32S31_ETM_H

#include <linux/ioctl.h>
#include <linux/types.h>

/* Hardware event and task IDs from the ESP32-S31 ETM matrix. */
#define ESP32S31_ETM_TG0_T0_COMPARE	73
#define ESP32S31_ETM_TG0_T1_COMPARE	74
#define ESP32S31_ETM_TG1_T0_COMPARE	75
#define ESP32S31_ETM_TG1_T1_COMPARE	76
#define ESP32S31_ETM_GPIO_SET(slot)	(1U + (slot))
#define ESP32S31_ETM_GPIO_CLEAR(slot)	(9U + (slot))
#define ESP32S31_ETM_GPIO_TOGGLE(slot)	(17U + (slot))

/* A dynamic channel belongs to its open file and is released on close. */
struct esp32s31_etm_route {
	__u32 channel;
	__u32 event;
	__u32 task;
};

struct esp32s31_etm_level {
	__u32 slot;
	__u32 value;
};

#define ESP32S31_ETM_IOC_BIND	_IOW('E', 0x71, struct esp32s31_etm_route)
#define ESP32S31_ETM_IOC_UNBIND	_IOW('E', 0x72, __u32)
#define ESP32S31_ETM_IOC_LEVEL	_IOWR('E', 0x73, struct esp32s31_etm_level)

#endif
