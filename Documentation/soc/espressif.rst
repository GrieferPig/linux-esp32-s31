.. SPDX-License-Identifier: GPL-2.0

==============================
Espressif SoC driver naming
==============================

This tree carries support for two Espressif RISC-V SoCs that share several
hardware blocks but reach some of them through different privilege models:
the ESP32-S31 (S-mode Linux, OpenSBI below it, MMU on) and the ESP32-P4
(M-mode noMMU Linux, cold-init firmware retained in SRAM).

File naming
===========

Shared drivers are named after the class directory convention followed by the
vendor::

    drivers/i2c/busses/i2c-espressif.c
    drivers/spi/spi-espressif.c
    drivers/cache/espressif_cache.c
    drivers/counter/espressif-gptimer.c
    drivers/pwm/pwm-espressif-ledc.c
    drivers/watchdog/espressif_wdt.c
    drivers/phy/phy-espressif-usb.c

Files that implement one SoC carry the SoC token, because the hardware
programming model - not the class - is what differs::

    drivers/irqchip/irq-esp-clic-s31.c      S-mode CLIC view (OpenSBI delegated)
    drivers/irqchip/irq-esp-intmtx-s31.c    S-mode interrupt matrix
    drivers/irqchip/irq-esp-smp-s31.c       per-hart doorbell / direct routes
    drivers/irqchip/irq-esp-clic-p4.c       M-mode CLIC + matrix, rev 1.3
    drivers/clocksource/timer-espressif-systimer-s31.c
    drivers/clocksource/timer-espressif-systimer-p4.c
    drivers/clk/clk-espressif.c             (S31 clock tree; P4 has no clk driver)
    drivers/pinctrl/pinctrl-espressif.c     (S31 pin tables)

Do not merge two SoC files just because the class matches.  The ESP32-S31 CLIC
driver programs the S-mode window at 0x10A00000 that OpenSBI has delegated;
the ESP32-P4 driver owns the M-mode controller, its hart-relative windows, and
the rev 1.3 register layout.  A single driver would carry two disjoint sets of
register definitions behind a runtime switch and would be testable only on one
board at a time.

DT compatible rules
===================

Compatibles are SoC specific; fallbacks are SoC specific too::

    compatible = "espressif,esp32p4-i2c";
    compatible = "espressif,esp32p4-i2c", "espressif,esp32s31-i2c";  /* identical IP */

A bare IP name ("espressif,i2c") is intentionally not used: the binding rules
reject device-family names in compatible strings.

Retained firmware services
==========================

The ESP32-P4 cold-init firmware keeps running and owns a few engines.  It is
described by its own node rather than by suffixing hardware compatibles::

    firmware-services {
        compatible = "espressif,esp32p4-idf-services";
        #address-cells = <1>;
        #size-cells = <0>;

        service@0 { reg = <0>; espressif,entry = <0x4ff00000>; };
    };

Service ids are the retained firmware ABI (0 executable cache sync, 1 display
writeback, 2 camera invalidate); the callback addresses stay in the device tree
so the kernel never hardcodes firmware addresses.

Kconfig
=======

    CONFIG_SOC_ESP32S31 / CONFIG_SOC_ESP32P4   only SoC switches
    CONFIG_<CLASS>_ESPRESSIF                   shared driver
    CONFIG_ESP32S31_* / CONFIG_ESP32P4_*       SoC-private subsystem

Display block boundaries
========================

``drivers/video/fbdev/espressif-fb.c`` keeps the P4 framebuffer, the DSI host
and the PPA rotation in one file.  They are not separate drivers: the present
path shares one lock, one completion and one IRQ pair between the PPA job and
the framebuffer writeback, and the DSI host has to be programmed before the
first scanout starts.  Splitting them would export those internals without
giving either part an independent lifetime, so the file stays whole.  The
device itself is described by the framebuffer node, with the DSI and PPA
controllers referenced as phandles.
