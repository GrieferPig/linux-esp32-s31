# SPDX-License-Identifier: GPL-2.0
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31.dtb
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31_generic.dtb
DTC_FLAGS_esp32s31_generic := -@
dtb-$(CONFIG_SOC_ESP32S31) += esp32s31-generic.dtb
dtb-$(CONFIG_SOC_ESP32S31) += esp32s31-coreboard.dtb
dtb-$(CONFIG_SOC_ESP32S31) += esp32s31-korvo.dtb
DTC_FLAGS_esp32s31-generic := -@
DTC_FLAGS_esp32s31-coreboard := -@
DTC_FLAGS_esp32s31-korvo := -@
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-gmac.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-sdmmc0.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-sdmmc1.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-sdmmc-dual.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-pwm-counter.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-analog.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-timers.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-watchdogs.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-uart1.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-uart2.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-uart3.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-uart3-dma.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-gdma.dtbo
ifneq ($(CONFIG_FB_ESP32S31_I80),)
dtb-$(CONFIG_SOC_ESP32S31) += esp32s31-overlay-i80-ili9341.dtbo
dtb-$(CONFIG_SOC_ESP32S31) += esp32s31-i80-ili9341.dtb
endif
esp32s31-i80-ili9341-dtbs := esp32s31_generic.dtb esp32s31-overlay-i80-ili9341.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-gpspi2.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-gpspi3.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-gpspi2-target.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-gpspi3-target.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-i2s0.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-i2s1.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-i2c0.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-i2c1.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-twai0.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-twai1.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-sdmmc-uhs.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-usb-device.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-radio-wifi.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-radio-bluetooth.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-radio-combo.dtbo
dtb-$(CONFIG_ARCH_ESPRESSIF) += esp32s31-overlay-lp.dtbo

dtb-$(CONFIG_SOC_ESP32P4) += esp32p4-generic.dtb
dtb-$(CONFIG_SOC_ESP32P4) += esp32p4-waveshare-wifi6-dev-kit.dtb
