# SPDX-License-Identifier: GPL-2.0
#
# Makefile for linux/drivers/platform
#

obj-$(CONFIG_ESP32S31_DT_OVERLAY) += esp32s31-overlay.o
obj-$(CONFIG_SOC_ESP32S31) += esp32s31-radio-loader-core.o
obj-$(CONFIG_ESP32S31_RADIO_SMODE_DRIVER) += esp32s31-radio.o
esp32s31-radio-y := esp32s31-radio-module.o \
	esp32s31-radio-smode.o esp32s31-radio-rtos.o \
	esp32s31-radio-loader.o esp32s31-radio-imports.o \
	esp32s31-radio-wifi.o esp32s31-radio-btdm.o

LDFLAGS_esp32s31-radio.o += --gc-sections \
	-T $(src)/esp32s31-radio-core.lds --undefined=init_module \
	--undefined=s31_radio_modinfo_anchor \
	--undefined=s31_radio_timing_param_anchor

obj-$(CONFIG_X86)		+= x86/
obj-$(CONFIG_LOONGARCH)		+= loongarch/
obj-$(CONFIG_MELLANOX_PLATFORM)	+= mellanox/
obj-$(CONFIG_MIPS)		+= mips/
obj-$(CONFIG_OLPC_EC)		+= olpc/
obj-$(CONFIG_GOLDFISH)		+= goldfish/
obj-$(CONFIG_CHROME_PLATFORMS)	+= chrome/
obj-$(CONFIG_CZNIC_PLATFORMS)	+= cznic/
obj-$(CONFIG_SURFACE_PLATFORMS)	+= surface/
obj-$(CONFIG_ARM64_PLATFORM_DEVICES)	+= arm64/
