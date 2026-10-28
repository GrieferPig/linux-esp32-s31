# SPDX-License-Identifier: GPL-2.0-only
lib-y			+= delay.o
lib-y			+= memcpy.o
lib-y			+= memset.o
lib-y			+= memmove.o
ifeq ($(CONFIG_KASAN_GENERIC)$(CONFIG_KASAN_SW_TAGS),)
lib-y			+= strcmp.o
lib-y			+= strlen.o
lib-y			+= strncmp.o
endif
lib-y			+= csum.o
lib-$(CONFIG_SOC_ESP32S31) += s31_csum_copy.o
ifeq ($(CONFIG_MMU), y)
lib-$(CONFIG_RISCV_ISA_V)	+= uaccess_vector.o
endif
lib-$(CONFIG_MMU)	+= uaccess.o
lib-$(CONFIG_64BIT)	+= tishift.o
lib-$(CONFIG_RISCV_ISA_ZICBOZ)	+= clear_page.o
obj-$(CONFIG_FUNCTION_ERROR_INJECTION) += error-inject.o
lib-$(CONFIG_RISCV_ISA_V)	+= xor.o
lib-$(CONFIG_RISCV_ISA_V)	+= riscv_v_helpers.o

ifeq ($(CONFIG_SOC_ESP32S31)$(CONFIG_XIP_KERNEL),yy)
lib-y += s31_memcpy_unaligned.o
ifneq ($(CONFIG_KASAN),y)
lib-y += s31_memcmp.o
endif
# A memcpy implementation must never synthesize a recursive memcpy call.
CFLAGS_s31_memcpy_unaligned.o += -fno-tree-loop-distribute-patterns
endif
