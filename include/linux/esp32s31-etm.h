/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_ESP32S31_ETM_H
#define _LINUX_ESP32S31_ETM_H

#include <uapi/linux/esp32s31-etm.h>

struct device;

/*
 * The ETM device must remain referenced during these calls. The caller
 * supplies an ownership token and unbinds before releasing that token.
 */
int esp32s31_etm_bind(struct device *dev, const struct esp32s31_etm_route *route,
		      const void *owner);
int esp32s31_etm_unbind(struct device *dev, unsigned int channel, const void *owner);

#endif
