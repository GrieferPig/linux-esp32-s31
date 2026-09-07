// SPDX-License-Identifier: GPL-2.0-only
/* Built-in executable-memory and relocation bridge for the loadable S31 radio. */

#include <linux/elf.h>
#include <linux/execmem.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleloader.h>
#include <linux/mutex.h>
#include <linux/set_memory.h>
#include <linux/vmalloc.h>

void *esp32s31_radio_fw_execmem_alloc(size_t size);
void esp32s31_radio_fw_execmem_free(void *address);
int esp32s31_radio_fw_execmem_enable_x(void *address, size_t size);
int esp32s31_radio_fw_apply_relocate_add(Elf_Shdr *sections,
					 const char *strings,
					 unsigned int symbol_section,
					 unsigned int relocation_section,
					 struct module *owner);

static DEFINE_MUTEX(esp32s31_radio_fw_execmem_lock);
static void *esp32s31_radio_fw_execmem;
static size_t esp32s31_radio_fw_execmem_size;

void *esp32s31_radio_fw_execmem_alloc(size_t size)
{
	void *address = NULL;
	size_t aligned = PAGE_ALIGN(size);

	mutex_lock(&esp32s31_radio_fw_execmem_lock);
	if (esp32s31_radio_fw_execmem) {
		if (aligned <= esp32s31_radio_fw_execmem_size)
			address = esp32s31_radio_fw_execmem;
		goto out;
	}
	address = execmem_alloc_rw(EXECMEM_MODULE_TEXT, aligned);
	if (address) {
		esp32s31_radio_fw_execmem = address;
		esp32s31_radio_fw_execmem_size = aligned;
	}
out:
	mutex_unlock(&esp32s31_radio_fw_execmem_lock);
	return address;
}
EXPORT_SYMBOL_GPL(esp32s31_radio_fw_execmem_alloc);

void esp32s31_radio_fw_execmem_free(void *address)
{
	/* ESP-IDF keeps process-lifetime objects containing callbacks into the
	 * payload even after the public Wi-Fi/BT deinit sequence.  The module
	 * teardown stops every task and IRQ before reaching this hook, but a later
	 * controller init may legitimately reuse one of those private objects.
	 * Keep one executable arena at a stable virtual address until reboot and
	 * overwrite/relocate it from the pristine external ELF on every load.  This
	 * is the firmware-domain equivalent of IDF retaining its application image
	 * while controllers are disabled; it also prevents stale closed-library
	 * callbacks from jumping into a freed previous image after module reload.
	 */
	WARN_ON_ONCE(address && address != esp32s31_radio_fw_execmem);
}
EXPORT_SYMBOL_GPL(esp32s31_radio_fw_execmem_free);

int esp32s31_radio_fw_execmem_enable_x(void *address, size_t size)
{
	/* execmem_alloc_rw() returns PAGE_KERNEL on RISC-V.  Mirror the normal
	 * module loader's module_enable_text_rox() transition before entering the
	 * dynamically relocated closure.  STRICT_MODULE_RWX is disabled on this
	 * target, so set_memory_x() retains the writable data in the same image. */
	set_vm_flush_reset_perms(address);
	return set_memory_x((unsigned long)address,
			    PAGE_ALIGN(size) >> PAGE_SHIFT);
}
EXPORT_SYMBOL_GPL(esp32s31_radio_fw_execmem_enable_x);

int esp32s31_radio_fw_apply_relocate_add(Elf_Shdr *sections,
					 const char *strings,
					 unsigned int symbol_section,
					 unsigned int relocation_section,
					 struct module *owner)
{
	return apply_relocate_add(sections, strings, symbol_section,
				  relocation_section, owner);
}
EXPORT_SYMBOL_GPL(esp32s31_radio_fw_apply_relocate_add);
