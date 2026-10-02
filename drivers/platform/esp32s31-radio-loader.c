// SPDX-License-Identifier: GPL-2.0-only
/* Validated flash-XIP radio loader. */
#include <linux/crc32.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <asm/cacheflush.h>
#include "esp32s31-radio-internal.h"
#include "esp32s31-radio-xip.h"
extern u8 esp32s31_radio_xip_ram[S31_XIP_RAM_SIZE];
extern const void *esp32s31_radio_xip_get(void);
extern const unsigned long _mtvt_table[48];
static const struct s31_xip_header *s31_xip_image;
struct s31_fw_import {
	const char *name;
	unsigned long address;
};

struct s31_radio_fw_exports {
	void (*stack_task)(void *arg);
	void (*bt_enable_task)(void *arg);
	void (*bt_disable_task)(void *arg);
	void (*shutdown_task)(void *arg);
	int (*vhci_try_send)(u8 *frame, u16 length);
	int (*coex_status)(u8 type, u8 op, u8 status);
	void (*wifi_scan_task)(void *arg);
	void (*wifi_connect_task)(void *arg);
	void (*wifi_disconnect_task)(void *arg);
	void (*wifi_control_task)(void *arg);
	int (*wifi_try_send_interface)(u8 interface, u8 *frame, u16 length);
	int (*wifi_read_mac)(u8 *mac);
	void (*wifi_clock_enable)(void);
	void (*wifi_clock_disable)(void);
	int (*wifi_try_send)(u8 *frame, u16 length);
	void (*rtos_init)(void);
	void (*rtos_tick)(void);
	u32 (*timer_next_due_us)(void);
	void (*rtos_hard_tick)(void);
	void (*rtos_free)(void *ptr);
	void (*rtos_task_release)(void *cookie);
	int (*task_create_pinned)(void (*task)(void *), const char *name,
				  u32 stack_size, void *arg, u32 priority,
				  void *task_handle, int core_id);
	u32 *rtos_isr_depth;
};

extern const struct s31_fw_import s31_fw_import_table[];
extern const u32 s31_fw_import_count;

static struct s31_radio_fw_exports s31_fw;
static unsigned long s31_fw_import_lookup(const char *name, bool weak)
{
	u32 index;

	for (index = 0; index < s31_fw_import_count; index++)
		if (!strcmp(name, s31_fw_import_table[index].name))
			return s31_fw_import_table[index].address;
	return weak ? 0 : ULONG_MAX;
}

static int s31_fw_xip_reset(void)
{
	const struct s31_xip_header *h = s31_xip_image;
	u32 i;

	/* Validate all bindings before overwriting the stable writable arena. */
	for (i = 0; i < h->import_count; i++) {
		unsigned long target = s31_fw_import_lookup(h->imports[i].name, false);

		if (!target || target == ULONG_MAX)
			return -ENOENT;
	}
	memcpy(esp32s31_radio_xip_ram, (const u8 *)h + h->data_offset, h->data_size);
	memset(esp32s31_radio_xip_ram + h->bss_offset, 0, h->bss_size);
	for (i = 0; i < h->import_count; i++)
		*(u32 *)(esp32s31_radio_xip_ram + h->imports[i].offset) =
			s31_fw_import_lookup(h->imports[i].name, false);
	memcpy(esp32s31_radio_xip_ram + h->vectors_offset, _mtvt_table, 192);
	return 0;
}

static int s31_fw_xip_load(struct device *device)
{
	const struct s31_xip_header *h = esp32s31_radio_xip_get();
	u32 zero = 0, crc;
	int ret;

	static_assert(sizeof(s31_fw) == S31_XIP_EXPORTS * sizeof(u32));
	static_assert(sizeof(struct s31_xip_header) <= S31_XIP_HEADER_SIZE);
	static_assert(offsetof(struct s31_xip_header, wifi_iram_offset) ==
		      S31_XIP_WIFI_IRAM_FIELDS_OFFSET);
	if (!h || !s31_xip_valid(h, (u32)(uintptr_t)esp32s31_radio_xip_ram))
		return dev_err_probe(device, -ENOEXEC, "invalid XIP image/layout\n");
	crc = crc32_le(~0U, (const u8 *)h, 60);
	crc = crc32_le(crc, (const u8 *)&zero, 4);
	crc = crc32_le(crc, (const u8 *)h + 64, S31_XIP_HEADER_SIZE - 64) ^ ~0U;
	if (crc != h->header_crc ||
	    (crc32_le(~0U, (const u8 *)h + S31_XIP_HEADER_SIZE,
		      h->image_size - S31_XIP_HEADER_SIZE) ^ ~0U) != h->body_crc)
		return dev_err_probe(device, -EBADMSG, "XIP image CRC mismatch\n");
	s31_xip_image = h;
	ret = s31_fw_xip_reset();
	if (ret) {
		s31_xip_image = NULL;
		return dev_err_probe(device, ret, "XIP reset failed\n");
	}
	memcpy(&s31_fw, h->exports, sizeof(s31_fw));
	flush_icache_range(S31_XIP_BASE, S31_XIP_BASE + h->data_offset);
	dev_info(device, "radio XIP at %px: %u flash bytes, %u/%u RAM bytes, ABI %u\n",
		h, h->image_size, h->data_size + h->bss_size,
		S31_XIP_RAM_SIZE, h->abi);
	return 0;
}
int s31_radio_fw_reset(void)
{
	return s31_xip_image ? s31_fw_xip_reset() : -ENODEV;
}
int s31_radio_fw_load(struct device *device)
{
	return s31_fw_xip_load(device);
}
void s31_radio_fw_unload(void)
{
	s31_xip_image = NULL;
	memset(&s31_fw, 0, sizeof(s31_fw));
}

void s31_radio_stack_task(void *arg) { s31_fw.stack_task(arg); }
void s31_radio_bt_enable_task(void *arg) { s31_fw.bt_enable_task(arg); }
void s31_radio_bt_disable_task(void *arg) { s31_fw.bt_disable_task(arg); }
void s31_radio_shutdown_task(void *arg) { s31_fw.shutdown_task(arg); }
int s31_radio_vhci_try_send(u8 *frame, u16 length)
{
	return s31_fw.vhci_try_send(frame, length);
}
int s31_radio_coex_status(u8 type, u8 op, u8 status)
{
	if (!s31_fw.coex_status)
		return -EOPNOTSUPP;
	return s31_fw.coex_status(type, op, status);
}
void s31_radio_wifi_scan_task(void *arg) { s31_fw.wifi_scan_task(arg); }
void s31_radio_wifi_control_task(void *arg) { s31_fw.wifi_control_task(arg); }
int s31_radio_wifi_try_send_interface(u8 interface, u8 *frame, u16 length)
{
	return s31_fw.wifi_try_send_interface(interface, frame, length);
}
void s31_radio_wifi_connect_task(void *arg) { s31_fw.wifi_connect_task(arg); }
void s31_radio_wifi_disconnect_task(void *arg)
{
	s31_fw.wifi_disconnect_task(arg);
}
int s31_radio_wifi_read_mac(u8 *mac) { return s31_fw.wifi_read_mac(mac); }
void s31_radio_wifi_clock_enable(void) { s31_fw.wifi_clock_enable(); }
void s31_radio_wifi_clock_disable(void) { s31_fw.wifi_clock_disable(); }
int s31_radio_wifi_try_send(u8 *frame, u16 length)
{
	return s31_fw.wifi_try_send(frame, length);
}
void s31_rtos_init(void) { s31_fw.rtos_init(); }
void s31_rtos_tick(void) { s31_fw.rtos_tick(); }
u32 s31_linux_timer_next_due_us(void) { return s31_fw.timer_next_due_us(); }
void s31_rtos_hard_tick(void) { s31_fw.rtos_hard_tick(); }
void s31_rtos_free(void *ptr) { s31_fw.rtos_free(ptr); }
void s31_rtos_task_release(void *cookie) { s31_fw.rtos_task_release(cookie); }
int xTaskCreatePinnedToCore(void (*task)(void *), const char *name,
			  u32 stack_size, void *arg, u32 priority,
			  void *task_handle, int core_id)
{
	return s31_fw.task_create_pinned(task, name, stack_size, arg, priority,
					 task_handle, core_id);
}
u32 *s31_radio_fw_isr_depth(void) { return s31_fw.rtos_isr_depth; }
