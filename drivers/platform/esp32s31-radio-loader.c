// SPDX-License-Identifier: GPL-2.0-only
/* Versioned loader for the externally packaged ESP32-S31 IDF radio closure. */

#include <linux/elf.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/overflow.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include <asm/cacheflush.h>

#include "esp32s31-radio-internal.h"

#define S31_RADIO_FW_ABI_VERSION 1U
#define S31_RADIO_FW_DEFAULT_NAME "esp32s31-radio-fw-v1.o"

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
static void *s31_fw_image;
static size_t s31_fw_image_size;
struct s31_fw_reset_section {
	void *address;
	void *initial;
	size_t size;
};
static struct s31_fw_reset_section *s31_fw_reset_sections;
static unsigned int s31_fw_reset_count;

static void s31_fw_free_reset_sections(void)
{
	unsigned int i;

	for (i = 0; i < s31_fw_reset_count; i++)
		kvfree(s31_fw_reset_sections[i].initial);
	kfree(s31_fw_reset_sections);
	s31_fw_reset_sections = NULL;
	s31_fw_reset_count = 0;
}

/* The code stays at the same executable address.  Reset only mutable data
 * after stopping every payload task/IRQ, without firmware I/O during resume. */
int s31_radio_fw_reset(void)
{
	unsigned int i;

	if (!s31_fw_image || !s31_fw_reset_sections)
		return -ENODEV;
	for (i = 0; i < s31_fw_reset_count; i++)
		memcpy(s31_fw_reset_sections[i].address, s31_fw_reset_sections[i].initial,
		       s31_fw_reset_sections[i].size);
	return 0;
}
static char *s31_fw_name = S31_RADIO_FW_DEFAULT_NAME;
module_param_named(firmware, s31_fw_name, charp, 0400);
MODULE_PARM_DESC(firmware, "external ESP32-S31 radio closure filename");

void *esp32s31_radio_fw_execmem_alloc(size_t size);
void esp32s31_radio_fw_execmem_free(void *address);
int esp32s31_radio_fw_execmem_enable_x(void *address, size_t size);
int esp32s31_radio_fw_apply_relocate_add(Elf_Shdr *sections,
					 const char *strings,
					 unsigned int symbol_section,
					 unsigned int relocation_section,
					 struct module *owner);

static unsigned long s31_fw_import_lookup(const char *name, bool weak)
{
	u32 index;

	for (index = 0; index < s31_fw_import_count; index++)
		if (!strcmp(name, s31_fw_import_table[index].name))
			return s31_fw_import_table[index].address;
	return weak ? 0 : ULONG_MAX;
}

static bool s31_fw_range_valid(size_t file_size, u32 offset, u32 length)
{
	size_t end;

	return !check_add_overflow((size_t)offset, (size_t)length, &end) &&
	       end <= file_size;
}

static int s31_fw_validate_call_relocations(Elf32_Shdr *sections,
					     unsigned int symbol_section,
					     unsigned int relocation_section,
					     const char *strings)
{
	Elf32_Rela *relocations =
		(void *)(uintptr_t)sections[relocation_section].sh_addr;
	Elf32_Sym *symbols = (void *)(uintptr_t)sections[symbol_section].sh_addr;
	size_t count = sections[relocation_section].sh_size / sizeof(*relocations);
	size_t index;

	for (index = 0; index < count; index++) {
		Elf32_Rela *relocation = &relocations[index];
		Elf32_Sym *symbol;
		void *location;
		unsigned long expected, actual;
		u32 auipc, jalr;
		s32 upper, lower;
		u32 type = ELF32_R_TYPE(relocation->r_info);

		if (type != R_RISCV_CALL && type != R_RISCV_CALL_PLT)
			continue;
		symbol = &symbols[ELF32_R_SYM(relocation->r_info)];
		location = (void *)(uintptr_t)
			(sections[sections[relocation_section].sh_info].sh_addr +
			 relocation->r_offset);
		auipc = get_unaligned_le32(location);
		jalr = get_unaligned_le32(location + sizeof(u32));
		if ((auipc & 0x7f) != 0x17 || (jalr & 0x707f) != 0x67) {
			pr_err("esp32s31-radio: malformed relocated call to %s\n",
			       strings + symbol->st_name);
			return -ENOEXEC;
		}
		upper = (s32)(auipc & 0xfffff000);
		lower = (s32)jalr >> 20;
		expected = symbol->st_value + relocation->r_addend;
		actual = (uintptr_t)location + upper + lower;
		if (actual != expected) {
			pr_err("esp32s31-radio: bad call relocation %s: %px != %px\n",
			       strings + symbol->st_name, (void *)actual,
			       (void *)expected);
			return -ENOEXEC;
		}
	}
	return 0;
}

static Elf32_Sym *s31_fw_find_symbol(Elf32_Sym *symbols, size_t count,
				     const char *strings, size_t strings_size,
				     const char *name)
{
	size_t index;

	for (index = 1; index < count; index++) {
		if (symbols[index].st_name >= strings_size)
			continue;
		if (!strcmp(strings + symbols[index].st_name, name))
			return &symbols[index];
	}
	return NULL;
}

static int s31_fw_export(void **destination, Elf32_Sym *symbols,
			 size_t symbol_count, const char *strings,
			 size_t strings_size, const char *name)
{
	Elf32_Sym *symbol = s31_fw_find_symbol(symbols, symbol_count, strings,
					       strings_size, name);

	if (!symbol || symbol->st_shndx == SHN_UNDEF || !symbol->st_value) {
		pr_err("esp32s31-radio: firmware export %s is missing\n", name);
		return -ENOEXEC;
	}
	*destination = (void *)(uintptr_t)symbol->st_value;
	return 0;
}

static void s31_fw_export_optional(void **destination, Elf32_Sym *symbols,
				   size_t symbol_count, const char *strings,
				   size_t strings_size, const char *name)
{
	Elf32_Sym *symbol = s31_fw_find_symbol(symbols, symbol_count, strings,
					       strings_size, name);

	if (symbol && symbol->st_shndx != SHN_UNDEF && symbol->st_value)
		*destination = (void *)(uintptr_t)symbol->st_value;
}

#define S31_FW_EXPORT(field, name) \
	s31_fw_export((void **)&s31_fw.field, symbols, symbol_count, strings, \
		      strings_size, name)

static int s31_fw_collect_exports(Elf32_Sym *symbols, size_t symbol_count,
				  const char *strings, size_t strings_size)
{
	Elf32_Sym *abi;
	int ret;

	abi = s31_fw_find_symbol(symbols, symbol_count, strings, strings_size,
				 "s31_radio_fw_abi_version");
	if (!abi || abi->st_shndx == SHN_UNDEF || !abi->st_value ||
	    *(u32 *)(uintptr_t)abi->st_value != S31_RADIO_FW_ABI_VERSION) {
		pr_err("esp32s31-radio: incompatible or missing firmware ABI\n");
		return -EPROTO;
	}

	ret = S31_FW_EXPORT(stack_task, "s31_radio_stack_task");
	ret = ret ?: S31_FW_EXPORT(bt_enable_task, "s31_radio_bt_enable_task");
	ret = ret ?: S31_FW_EXPORT(bt_disable_task, "s31_radio_bt_disable_task");
	ret = ret ?: S31_FW_EXPORT(shutdown_task, "s31_radio_shutdown_task");
	ret = ret ?: S31_FW_EXPORT(vhci_try_send, "s31_radio_vhci_try_send");
	ret = ret ?: S31_FW_EXPORT(wifi_scan_task, "s31_radio_wifi_scan_task");
	ret = ret ?: S31_FW_EXPORT(wifi_connect_task, "s31_radio_wifi_connect_task");
	ret = ret ?: S31_FW_EXPORT(wifi_disconnect_task,
				    "s31_radio_wifi_disconnect_task");
	ret = ret ?: S31_FW_EXPORT(wifi_read_mac, "s31_radio_wifi_read_mac");
	ret = ret ?: S31_FW_EXPORT(wifi_clock_enable,
				    "s31_radio_wifi_clock_enable");
	ret = ret ?: S31_FW_EXPORT(wifi_clock_disable,
				    "s31_radio_wifi_clock_disable");
	ret = ret ?: S31_FW_EXPORT(wifi_try_send, "s31_radio_wifi_try_send");
	ret = ret ?: S31_FW_EXPORT(wifi_control_task, "s31_radio_wifi_control_task");
	ret = ret ?: S31_FW_EXPORT(wifi_try_send_interface,
				    "s31_radio_wifi_try_send_interface");
	ret = ret ?: S31_FW_EXPORT(rtos_init, "s31_rtos_init");
	ret = ret ?: S31_FW_EXPORT(rtos_tick, "s31_rtos_tick");
	ret = ret ?: S31_FW_EXPORT(timer_next_due_us,
				    "s31_linux_timer_next_due_us");
	ret = ret ?: S31_FW_EXPORT(rtos_hard_tick, "s31_rtos_hard_tick");
	ret = ret ?: S31_FW_EXPORT(rtos_free, "s31_rtos_free");
	ret = ret ?: S31_FW_EXPORT(rtos_task_release, "s31_rtos_task_release");
	ret = ret ?: S31_FW_EXPORT(task_create_pinned, "xTaskCreatePinnedToCore");
	ret = ret ?: S31_FW_EXPORT(rtos_isr_depth, "s31_rtos_isr_depth");
	if (!ret)
		s31_fw_export_optional((void **)&s31_fw.coex_status, symbols,
				       symbol_count, strings, strings_size,
				       "s31_radio_coex_status");
	return ret;
}

int s31_radio_fw_load(struct device *device)
{
	const struct firmware *firmware;
	const Elf32_Ehdr *header;
	const Elf32_Shdr *file_sections;
	Elf32_Shdr *sections = NULL;
	Elf32_Sym *symbols = NULL;
	const char *strings;
	size_t strings_size, symbol_count;
	size_t image_size = 0;
	u32 symbol_section = UINT_MAX;
	u32 index;
	int ret;

	if (s31_fw_image)
		return -EBUSY;
	ret = request_firmware(&firmware, s31_fw_name, device);
	if (ret)
		return dev_err_probe(device, ret, "cannot load firmware %s\n",
				     s31_fw_name);
	if (firmware->size < sizeof(*header)) {
		ret = -ENOEXEC;
		goto out_release;
	}
	header = (const Elf32_Ehdr *)firmware->data;
	if (memcmp(header->e_ident, ELFMAG, SELFMAG) ||
	    header->e_ident[EI_CLASS] != ELFCLASS32 ||
	    header->e_ident[EI_DATA] != ELFDATA2LSB ||
	    header->e_type != ET_REL || header->e_machine != EM_RISCV ||
	    header->e_shentsize != sizeof(Elf32_Shdr) || !header->e_shnum ||
	    header->e_shnum >= SHN_LORESERVE ||
	    !s31_fw_range_valid(firmware->size, header->e_shoff,
				header->e_shnum * sizeof(Elf32_Shdr))) {
		ret = -ENOEXEC;
		goto out_release;
	}
	file_sections = (const Elf32_Shdr *)(firmware->data + header->e_shoff);
	sections = kmemdup(file_sections,
			    header->e_shnum * sizeof(*sections), GFP_KERNEL);
	if (!sections) {
		ret = -ENOMEM;
		goto out_release;
	}

	for (index = 1; index < header->e_shnum; index++) {
		size_t aligned;

		if (!(sections[index].sh_flags & SHF_ALLOC))
			continue;
		if (sections[index].sh_addralign &&
		    !is_power_of_2(sections[index].sh_addralign)) {
			ret = -ENOEXEC;
			goto out_free_sections;
		}
		aligned = ALIGN(image_size, max_t(u32, 1,
						sections[index].sh_addralign));
		if (check_add_overflow(aligned, (size_t)sections[index].sh_size,
				       &image_size)) {
			ret = -EOVERFLOW;
			goto out_free_sections;
		}
		sections[index].sh_addr = aligned;
	}
	if (!image_size) {
		ret = -ENOEXEC;
		goto out_free_sections;
	}
	s31_fw_image = esp32s31_radio_fw_execmem_alloc(image_size);
	if (!s31_fw_image) {
		ret = -ENOMEM;
		goto out_free_sections;
	}
	s31_fw_image_size = image_size;
	memset(s31_fw_image, 0, image_size);

	for (index = 1; index < header->e_shnum; index++) {
		if (sections[index].sh_flags & SHF_ALLOC) {
			sections[index].sh_addr += (uintptr_t)s31_fw_image;
			if (sections[index].sh_type == SHT_NOBITS)
				continue;
			if (!s31_fw_range_valid(firmware->size,
						sections[index].sh_offset,
						sections[index].sh_size)) {
				ret = -ENOEXEC;
				goto out_free_image;
			}
			memcpy((void *)(uintptr_t)sections[index].sh_addr,
			       firmware->data + sections[index].sh_offset,
			       sections[index].sh_size);
		} else if (sections[index].sh_type != SHT_NOBITS) {
			if (!s31_fw_range_valid(firmware->size,
						sections[index].sh_offset,
						sections[index].sh_size)) {
				ret = -ENOEXEC;
				goto out_free_image;
			}
			sections[index].sh_addr =
				(uintptr_t)firmware->data + sections[index].sh_offset;
		}
		if (sections[index].sh_type == SHT_SYMTAB) {
			if (symbol_section != UINT_MAX ||
			    sections[index].sh_entsize != sizeof(Elf32_Sym) ||
			    sections[index].sh_link >= header->e_shnum) {
				ret = -ENOEXEC;
				goto out_free_image;
			}
			symbol_section = index;
		}
	}
	if (symbol_section == UINT_MAX) {
		ret = -ENOEXEC;
		goto out_free_image;
	}
	symbol_count = sections[symbol_section].sh_size / sizeof(*symbols);
	/* The IDF closure has a large relocation symbol table.  A physically
	 * contiguous kmemdup() works at cold boot but can require an order-8 page
	 * after the persistent executable arena has fragmented 16 MiB RAM during
	 * a mode reload.  The table is loader-only, so permit vmalloc fallback. */
	symbols = kvmemdup((void *)(uintptr_t)sections[symbol_section].sh_addr,
			   sections[symbol_section].sh_size, GFP_KERNEL);
	if (!symbols) {
		ret = -ENOMEM;
		goto out_free_image;
	}
	sections[symbol_section].sh_addr = (uintptr_t)symbols;
	strings = (const char *)(uintptr_t)
		sections[sections[symbol_section].sh_link].sh_addr;
	strings_size = sections[sections[symbol_section].sh_link].sh_size;

	for (index = 1; index < symbol_count; index++) {
		Elf32_Sym *symbol = &symbols[index];
		const char *name;
		unsigned long address;
		bool weak = ELF32_ST_BIND(symbol->st_info) == STB_WEAK;

		if (symbol->st_name >= strings_size) {
			ret = -ENOEXEC;
			goto out_free_symbols;
		}
		name = strings + symbol->st_name;
		if (symbol->st_shndx == SHN_UNDEF) {
			address = s31_fw_import_lookup(name, weak);
			if (address == ULONG_MAX) {
				pr_err("esp32s31-radio: firmware import %s is not allowed\n",
				       name);
				ret = -ENOENT;
				goto out_free_symbols;
			}
			symbol->st_value = address;
		} else if (symbol->st_shndx == SHN_ABS) {
			continue;
		} else if (symbol->st_shndx >= header->e_shnum) {
			ret = -ENOEXEC;
			goto out_free_symbols;
		} else {
			symbol->st_value += sections[symbol->st_shndx].sh_addr;
		}
	}

	for (index = 1; index < header->e_shnum; index++) {
		if (sections[index].sh_type != SHT_RELA)
			continue;
		if (sections[index].sh_info >= header->e_shnum ||
		    !(sections[sections[index].sh_info].sh_flags & SHF_ALLOC))
			continue;
		ret = esp32s31_radio_fw_apply_relocate_add(sections, strings,
							 symbol_section, index,
							 THIS_MODULE);
		if (!ret)
			ret = s31_fw_validate_call_relocations(sections,
							 symbol_section, index,
							 strings);
		if (ret)
			goto out_free_symbols;
	}
	ret = esp32s31_radio_fw_execmem_enable_x(s31_fw_image, image_size);
	if (ret) {
		dev_err(device, "cannot enable firmware execution: %d\n", ret);
		goto out_free_symbols;
	}
	flush_icache_range((unsigned long)s31_fw_image,
			   (unsigned long)s31_fw_image + image_size);
	ret = s31_fw_collect_exports(symbols, symbol_count, strings, strings_size);
	if (ret)
		goto out_free_symbols;
	s31_fw_reset_sections = kcalloc(header->e_shnum,
				       sizeof(*s31_fw_reset_sections), GFP_KERNEL);
	if (!s31_fw_reset_sections) {
		ret = -ENOMEM;
		goto out_free_symbols;
	}
	for (index = 1; index < header->e_shnum; index++) {
		struct s31_fw_reset_section *reset;

		if ((sections[index].sh_flags & (SHF_ALLOC | SHF_WRITE)) !=
		    (SHF_ALLOC | SHF_WRITE) || !sections[index].sh_size)
			continue;
		reset = &s31_fw_reset_sections[s31_fw_reset_count];
		reset->address = (void *)(uintptr_t)sections[index].sh_addr;
		reset->size = sections[index].sh_size;
		reset->initial = kvmalloc(reset->size, GFP_KERNEL);
		if (!reset->initial) {
			ret = -ENOMEM;
			goto out_free_symbols;
		}
		memcpy(reset->initial, reset->address, reset->size);
		s31_fw_reset_count++;
	}
	dev_info(device,
		 "external firmware %s loaded at %px: %zu bytes, ABI %u, init=%ps\n",
		 s31_fw_name, s31_fw_image, image_size,
		 S31_RADIO_FW_ABI_VERSION, s31_fw.rtos_init);
	kvfree(symbols);
	kfree(sections);
	release_firmware(firmware);
	return 0;

out_free_symbols:
	kvfree(symbols);
out_free_image:
	s31_fw_free_reset_sections();
	memset(&s31_fw, 0, sizeof(s31_fw));
	esp32s31_radio_fw_execmem_free(s31_fw_image);
	s31_fw_image = NULL;
	s31_fw_image_size = 0;
out_free_sections:
	kfree(sections);
out_release:
	release_firmware(firmware);
	return ret;
}

void s31_radio_fw_unload(void)
{
	s31_fw_free_reset_sections();
	memset(&s31_fw, 0, sizeof(s31_fw));
	if (s31_fw_image)
		esp32s31_radio_fw_execmem_free(s31_fw_image);
	s31_fw_image = NULL;
	s31_fw_image_size = 0;
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
