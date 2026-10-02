/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef ESP32S31_RADIO_XIP_H
#define ESP32S31_RADIO_XIP_H

#define S31_XIP_MAGIC 0x58313353U
#define S31_XIP_MAP_BASE 0xbe000000U
#define S31_XIP_MAP_PHYS 0x40000000U
#define S31_XIP_MAP_SIZE 0x400000U
#define S31_XIP_BASE 0xbe210000U
#define S31_XIP_PHYS 0x40210000U
#define S31_XIP_HEADER_SIZE 4096U
#define S31_XIP_SLOT_SIZE 0x1f0000U
#define S31_XIP_RAM_SIZE 40960U
#define S31_XIP_EXPORTS 23U
#define S31_XIP_MAX_IMPORTS ((S31_XIP_HEADER_SIZE - 156U - 8U) / 68U)
#define S31_XIP_WIFI_IRAM_FIELDS_OFFSET (156U + S31_XIP_MAX_IMPORTS * 68U)

struct s31_xip_import {
	u32 offset;
	char name[64];
};

struct s31_xip_header {
	u32 magic, version, abi, image_size;
	u32 base, ram, ram_capacity, data_offset;
	u32 data_size, bss_offset, bss_size, vectors_offset;
	u32 import_count, export_count, body_crc, header_crc;
	u32 exports[S31_XIP_EXPORTS];
	struct s31_xip_import imports[S31_XIP_MAX_IMPORTS];
	/* Reserved image fields, required to be zero. */
	u32 wifi_iram_offset, wifi_iram_size;
};

/* Pure bounds/address preflight, also compiled by the host tests. */
static inline bool s31_xip_range(u32 offset, u32 size, u32 capacity)
{
	return offset <= capacity && size <= capacity - offset;
}

static inline bool s31_xip_valid(const struct s31_xip_header *h, u32 ram)
{
	u32 i, body_end;

	if (h->magic != S31_XIP_MAGIC ||
	    h->version != 1 || h->abi != 1 ||
	    h->base != S31_XIP_BASE || h->ram != ram ||
	    h->ram_capacity != S31_XIP_RAM_SIZE ||
	    h->image_size <= S31_XIP_HEADER_SIZE ||
	    h->image_size > S31_XIP_SLOT_SIZE ||
	    h->export_count != S31_XIP_EXPORTS ||
	    h->import_count > S31_XIP_MAX_IMPORTS ||
	    h->data_offset < S31_XIP_HEADER_SIZE ||
	    !s31_xip_range(h->data_offset, h->data_size, h->image_size) ||
	    !s31_xip_range(0, h->data_size, S31_XIP_RAM_SIZE) ||
	    h->bss_offset != h->data_size ||
	    !s31_xip_range(h->bss_offset, h->bss_size, S31_XIP_RAM_SIZE) ||
	    !s31_xip_range(h->vectors_offset, 192, h->data_size))
		return false;
	if (h->wifi_iram_offset || h->wifi_iram_size)
		return false;
	body_end = S31_XIP_BASE + h->data_offset;
	for (i = 0; i < S31_XIP_EXPORTS - 1; i++) {
		if (h->exports[i] < S31_XIP_BASE + S31_XIP_HEADER_SIZE ||
		    h->exports[i] >= body_end)
			return false;
		if (h->exports[i] & 1)
			return false;
	}
	if (h->exports[22] < ram ||
	    !s31_xip_range(h->exports[22] - ram, 4,
			  h->bss_offset + h->bss_size) || (h->exports[22] & 3))
		return false;
	for (i = 0; i < h->import_count; i++) {
		const struct s31_xip_import *p = &h->imports[i];
		u32 j;

		if ((p->offset & 3) ||
		    !s31_xip_range(p->offset, 4, h->data_size) ||
		    !p->name[0] || !memchr(p->name, 0, sizeof(p->name)))
			return false;
		for (j = 0; j < i; j++)
			if (p->offset == h->imports[j].offset ||
			    !strcmp(p->name, h->imports[j].name))
				return false;
	}
	return true;
}

#endif
