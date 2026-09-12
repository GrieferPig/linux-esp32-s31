/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef ESP32S31_RADIO_ELF_H
#define ESP32S31_RADIO_ELF_H

/* Pure validation, shared with the host malformed-firmware test harness.
 * No allocation, relocation or firmware execution is allowed before this pass.
 */
/* ELF SHT_INIT_ARRAY is not named by the kernel ELF headers. */
#define S31_FW_SHT_INIT_ARRAY 14U
#define S31_FW_MAX_IMAGE_SIZE (16U * 1024U * 1024U)

static bool s31_fw_range_valid(size_t size, u32 offset, u32 length)
{
	return offset <= size && length <= size - offset;
}

static bool s31_fw_string_valid(const char *strings, size_t size, u32 offset)
{
	return offset < size && memchr(strings + offset, '\0', size - offset);
}

/* Allow only bounded, implemented static RISC-V relocations. GOT/PLT section
 * generation and variable-length ULEB writes are not part of this payload ABI.
 * A new relocation needs a checked write width and a host test before use.
 */
static int s31_fw_relocation_width(u32 type)
{
	switch (type) {
	case R_RISCV_RELAX:
		return 0;
	case R_RISCV_SUB6: case R_RISCV_SET6:
	case R_RISCV_ADD8: case R_RISCV_SUB8: case R_RISCV_SET8:
		return 1;
	case R_RISCV_RVC_BRANCH: case R_RISCV_RVC_JUMP:
	case R_RISCV_ADD16: case R_RISCV_SUB16: case R_RISCV_SET16:
		return 2;
	case R_RISCV_32: case R_RISCV_BRANCH: case R_RISCV_JAL:
	case R_RISCV_PCREL_HI20: case R_RISCV_PCREL_LO12_I:
	case R_RISCV_PCREL_LO12_S: case R_RISCV_HI20:
	case R_RISCV_LO12_I: case R_RISCV_LO12_S:
	case R_RISCV_ADD32: case R_RISCV_SUB32: case R_RISCV_SET32:
	case R_RISCV_32_PCREL:
		return 4;
	case R_RISCV_CALL: case R_RISCV_CALL_PLT:
	case R_RISCV_64: case R_RISCV_ADD64: case R_RISCV_SUB64:
		return 8;
	default:
		return -ENOEXEC;
	}
}

static int s31_fw_validate_elf(const void *data, size_t size)
{
	const Elf32_Ehdr *header = data;
	const Elf32_Shdr *sections, *table, *strtab;
	const Elf32_Sym *symbols;
	const char *strings;
	size_t image_size = 0, symbol_count;
	u32 index, symindex = 0;

	if (size < sizeof(*header) ||
	    memcmp(header->e_ident, ELFMAG, SELFMAG) ||
	    header->e_ident[EI_CLASS] != ELFCLASS32 ||
	    header->e_ident[EI_DATA] != ELFDATA2LSB ||
	    header->e_ident[EI_VERSION] != EV_CURRENT ||
	    header->e_version != EV_CURRENT || header->e_type != ET_REL ||
	    header->e_machine != EM_RISCV || header->e_ehsize != sizeof(*header) ||
	    header->e_phnum || header->e_shentsize != sizeof(Elf32_Shdr) ||
	    !header->e_shnum || header->e_shnum >= SHN_LORESERVE ||
	    header->e_shoff % 4 ||
	    !s31_fw_range_valid(size, header->e_shoff,
			       header->e_shnum * sizeof(Elf32_Shdr)))
		return -ENOEXEC;
	sections = (const void *)((const u8 *)data + header->e_shoff);
	if (sections[0].sh_type != SHT_NULL)
		return -ENOEXEC;
	for (index = 1; index < header->e_shnum; index++) {
		const Elf32_Shdr *s = &sections[index];
		u32 align = s->sh_addralign ? s->sh_addralign : 1;

		if ((align & (align - 1)) || align > S31_FW_MAX_IMAGE_SIZE ||
		    (s->sh_type != SHT_NOBITS &&
		     !s31_fw_range_valid(size, s->sh_offset, s->sh_size)))
			return -ENOEXEC;
		if (s->sh_flags & SHF_ALLOC) {
			if (s->sh_type != SHT_PROGBITS && s->sh_type != SHT_NOBITS &&
			    s->sh_type != S31_FW_SHT_INIT_ARRAY)
				return -ENOEXEC;
			image_size = (image_size + align - 1) & ~(size_t)(align - 1);
			if (image_size > S31_FW_MAX_IMAGE_SIZE ||
			    s->sh_size > S31_FW_MAX_IMAGE_SIZE - image_size)
				return -EFBIG;
			image_size += s->sh_size;
		}
		if (s->sh_type == SHT_SYMTAB) {
			if (symindex || s->sh_entsize != sizeof(Elf32_Sym) ||
			    s->sh_size % sizeof(Elf32_Sym) || s->sh_offset % 4 ||
			    s->sh_size < sizeof(Elf32_Sym) ||
			    !s->sh_link || s->sh_link >= header->e_shnum)
				return -ENOEXEC;
			symindex = index;
		}
		if (s->sh_type == SHT_REL)
			return -ENOEXEC;
	}
	if (!symindex || !image_size)
		return -ENOEXEC;
	if (header->e_shstrndx != SHN_UNDEF &&
	    (header->e_shstrndx >= header->e_shnum ||
	     sections[header->e_shstrndx].sh_type != SHT_STRTAB))
		return -ENOEXEC;
	table = &sections[symindex];
	strtab = &sections[table->sh_link];
	if (strtab->sh_type != SHT_STRTAB || !strtab->sh_size)
		return -ENOEXEC;
	strings = (const char *)data + strtab->sh_offset;
	/* Every in-range string offset now has a terminator within the table. */
	if (strings[0] || strings[strtab->sh_size - 1])
		return -ENOEXEC;
	symbols = (const void *)((const u8 *)data + table->sh_offset);
	symbol_count = table->sh_size / sizeof(*symbols);
	for (index = 0; index < symbol_count; index++) {
		const Elf32_Sym *s = &symbols[index];

		if (s->st_name >= strtab->sh_size)
			return -ENOEXEC;
		if (s->st_shndx == SHN_UNDEF || s->st_shndx == SHN_ABS)
			continue;
		if (s->st_shndx >= header->e_shnum ||
		    !s31_fw_range_valid(sections[s->st_shndx].sh_size,
				       s->st_value, s->st_size))
			return -ENOEXEC;
	}
	for (index = 1; index < header->e_shnum; index++) {
		const Elf32_Shdr *s = &sections[index];
		const Elf32_Rela *relocations;
		size_t i, count;

		if (s->sh_type != SHT_RELA)
			continue;
		if (s->sh_link != symindex || !s->sh_info ||
		    s->sh_info >= header->e_shnum || s->sh_offset % 4 ||
		    s->sh_entsize != sizeof(Elf32_Rela) ||
		    s->sh_size % sizeof(Elf32_Rela))
			return -ENOEXEC;
		relocations = (const void *)((const u8 *)data + s->sh_offset);
		count = s->sh_size / sizeof(*relocations);
		for (i = 0; i < count; i++) {
			const Elf32_Rela *rel = &relocations[i];
			u32 symbol = ELF32_R_SYM(rel->r_info);
			int width;

			if (symbol >= symbol_count)
				return -ENOEXEC;
			if (!(sections[s->sh_info].sh_flags & SHF_ALLOC))
				continue;
			width = s31_fw_relocation_width(ELF32_R_TYPE(rel->r_info));
			if (width < 0 || !s31_fw_range_valid(sections[s->sh_info].sh_size,
							   rel->r_offset, width))
				return -ENOEXEC;
			if (symbols[symbol].st_shndx != SHN_UNDEF &&
			    symbols[symbol].st_shndx != SHN_ABS &&
			    !(sections[symbols[symbol].st_shndx].sh_flags & SHF_ALLOC))
				return -ENOEXEC;
		}
	}
	return 0;
}

/* Runtime exports must remain inside a loaded allocation, not an ABS symbol
 * or a one-past-the-end linker marker. Function and data exports are distinct.
 */
static bool s31_fw_export_valid(const Elf32_Sym *symbol,
			       const Elf32_Shdr *sections, size_t count,
			       bool function, bool writable)
{
	const Elf32_Shdr *section;
	u32 offset;

	if (!symbol || symbol->st_shndx == SHN_UNDEF || symbol->st_shndx >= count)
		return false;
	section = &sections[symbol->st_shndx];
	if (!(section->sh_flags & SHF_ALLOC) ||
	    (function && !(section->sh_flags & SHF_EXECINSTR)) ||
	    (writable && !(section->sh_flags & SHF_WRITE)) ||
	    symbol->st_value < section->sh_addr)
		return false;
	offset = symbol->st_value - section->sh_addr;
	return s31_fw_range_valid(section->sh_size, offset, function ? 2 : 4) &&
	       !(symbol->st_value & (function ? 1 : 3));
}
#endif
