ESP32-P4 rev1 Waveshare native peripherals
=========================================

The physical UART0 boot console remains at 0x500ca000, matrix IRQ source 31,
115200n8 on ttyS0. Neither the UART driver nor the retained IDF display DT
is changed. The new board DT derives from the minimal U-Boot handoff, including
the mailbox, reserved addresses and XTAL rate. Use only with the Waveshare
ESP32-P4-WIFI6-DEV-KIT routing and rev1 silicon; PCB and SoC revisions are
separate facts.

Resources and constraints
-------------------------

* I2C1 (HP): 0x500c5000, source 45, 40 MHz XTAL parent, SDA=GPIO7/SCL=GPIO8.
  GPIO matrix signals 71/70, open drain. The board shares this bus with the
  codec and DSI/CSI connectors. No panel or camera is assumed to be installed.
  Linux controls the clock, reset and pinmux without an IDF handoff.
* GPSPI2: 0x500d0000, source 25, 20 MHz divider input and 2 MHz bus limit.
  Only 8-bit, single-line PIO master and native CS0. The opt-in expansion
  header route is GPIO20=SCK, GPIO21=MOSI, GPIO22=MISO, GPIO23=CS0. Check for
  conflicting HAT wiring before selecting this DT. The driver never acquires
  DMA, touches MSPI/boot flash/PSRAM, or instantiates an SPI child.
* UART0 (CH343P): TXD/RXD on GPIO37/GPIO38. These pads are not managed by the
  pinctrl provider, which exposes only the I2C1 and GPSPI2 groups.

Clock, reset, matrix and bus registers use ESP-IDF a602e67b hw_ver1 headers,
available at:
https://github.com/espressif/esp-idf/tree/a602e67b0bf9ee0806dc4e1df7afc9affedf5c33/components/soc/esp32p4/register/hw_ver1/soc

The P6 expansion pins and shared I2C wires are from the Waveshare schematic:
https://files.waveshare.com/wiki/ESP32-P4-WIFI6-DEV-KIT/ESP32-P4-WIFI6-DEV-KIT-datasheet.pdf

Build
-----

Apply the fragment over a working P4 rev1 NOMMU configuration with an
existing bFLT rootfs cpio. From the kernel tree, with the same ARCH=riscv
and CROSS_COMPILE=riscv32-esp-elf- used by the baseline::

    mkdir -p "$O"
    scripts/kconfig/merge_config.sh -m -O "$O" "$BASE_CONFIG" \
        arch/riscv/configs/esp32p4_native_peripherals.config
    make O="$O" olddefconfig
    make O="$O" -j8 Image espressif/esp32p4-waveshare-native-peripherals.dtb \
        espressif/esp32p4-waveshare-native-peripherals-spi.dtb

The first DTB enables I2C1; GPSPI2 and its pads remain untouched. The second
also enables GPSPI2 and routes P6. Neither declares an SPI slave. Keep the
recovery DTB and rich IDF display/camera DT available independently.

Validation
----------

For an esp-emu device specified with
--i2c-dev 1:0x34,fill=0xa5,reg=0x03=0x4a, verify::

    i2ctransfer -y 1 w1@0x34 0x03 r1    # 0x4a
    i2ctransfer -y 1 w1@0x34 0x20 r40   # 40 bytes of 0xa5
    i2ctransfer -y 1 w1@0x35 0x03 r1    # NACK
    i2ctransfer -y 1 w1@0x34 0x03 r1    # 0x4a again

In the SPI DTB, /sys/class/spi_master/spi2 and interrupt source 25 should
appear. Probe success is not proof of data transfer. For future bench
acceptance, first verify the PCB model and P6 wiring, exercise UART0 RX/TX,
then read a known I2C device ID at 100 kHz without scanning or writing its
configuration. Check low-speed SPI with a fixture at 1, 63, 64, 65 and 257
bytes, comparing data, observing chip-select and counting interrupts.
Tests here do not establish electrical edge quality, high rates or recovery
from shorted I2C lines.
