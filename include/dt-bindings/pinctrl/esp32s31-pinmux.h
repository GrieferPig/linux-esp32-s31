/* SPDX-License-Identifier: (GPL-2.0-only OR MIT) */
#ifndef _DT_BINDINGS_PINCTRL_ESP32S31_H
#define _DT_BINDINGS_PINCTRL_ESP32S31_H

/*
 * ESP32-S31 pinmux entries.  A pinctrl group's pinmux property may contain
 * any mixture of native IO_MUX selections and GPIO-matrix routes.
 */
#define ESP32S31_PINMUX_TYPE_SHIFT	28
#define ESP32S31_PINMUX_TYPE_IOMUX	0
#define ESP32S31_PINMUX_TYPE_MATRIX_OUT	1
#define ESP32S31_PINMUX_TYPE_MATRIX_IN	2
#define ESP32S31_PINMUX_TYPE_IOMUX_IN	3

#define ESP32S31_PINMUX_INVERT		(1U << 17)
#define ESP32S31_PINMUX_OEN_INVERT	(1U << 18)
#define ESP32S31_PINMUX_GPIO_OEN		(1U << 19)

#define ESP32S31_IOMUX(pin, function) \
	(((pin) & 0xff) | (((function) & 0x7) << 8) | \
	 (ESP32S31_PINMUX_TYPE_IOMUX << ESP32S31_PINMUX_TYPE_SHIFT))

#define ESP32S31_MATRIX_OUT(pin, signal, flags) \
	(((pin) & 0xff) | (((signal) & 0x1ff) << 8) | \
	 ((flags) & (ESP32S31_PINMUX_INVERT | ESP32S31_PINMUX_OEN_INVERT | \
		     ESP32S31_PINMUX_GPIO_OEN)) | \
	 (ESP32S31_PINMUX_TYPE_MATRIX_OUT << ESP32S31_PINMUX_TYPE_SHIFT))

#define ESP32S31_MATRIX_IN(pin, signal, flags) \
	(((pin) & 0xff) | (((signal) & 0x1ff) << 8) | \
	 ((flags) & ESP32S31_PINMUX_INVERT) | \
	 (ESP32S31_PINMUX_TYPE_MATRIX_IN << ESP32S31_PINMUX_TYPE_SHIFT))

/* Select a native IO_MUX input and bypass the GPIO matrix for signal. */
#define ESP32S31_IOMUX_IN(pin, function, signal) \
	(((pin) & 0xff) | (((signal) & 0xff) << 8) | \
	 (((function) & 0x7) << 20) | \
	 (ESP32S31_PINMUX_TYPE_IOMUX_IN << ESP32S31_PINMUX_TYPE_SHIFT))

/* Disconnect peripheral output and select GPIO_OUT/GPIO_ENABLE for the pad. */
#define ESP32S31_GPIO(pin) \
	ESP32S31_MATRIX_OUT(pin, 256, ESP32S31_PINMUX_GPIO_OEN)

#define ESP32S31_MATRIX_CONST_ONE	0x80
#define ESP32S31_MATRIX_CONST_ZERO	0xc0
#define ESP32S31_MATRIX_GPIO_OUT		256

#endif
