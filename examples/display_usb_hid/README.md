# BSP: Display USB HID Example

## Overview

<table>
<tr><td valign="top">

This example demonstrates usage of the USB HID (keyboard, mouse or GamePad) with Board Support Package.

</td><td width="200" valign="top">
  <img src="/examples/display_usb_hid/doc/pic.webp">
</td></tr>
</table>

## How to use the example

### Hardware Required

* Board [ESP32-S3-USB-OTG](https://docs.espressif.com/projects/espressif-esp-dev-kits/en/latest/esp32s3/esp32-s3-usb-otg/user_guide.html)
  - connect 5V USB power supply to the left "USB DEV" connector
  - connect USB keyboard/mouse to the right "USB Host" connector

## Build and Flash

To build and flash the example for a specific `{board}` and `{port}`, use the following command:

```
idf.py -D SDKCONFIG_DEFAULTS=sdkconfig.bsp.{board} -p {port} flash monitor
```
Make sure the correct board name is set in the `main/idf_component.yml` file under the `dependencies` section.

### Example outputs

```
I (0) cpu_start: App cpu up.
I (235) cpu_start: Pro cpu start user code
I (235) cpu_start: cpu freq: 160000000 Hz
I (235) cpu_start: Application information:
I (238) cpu_start: Project name:     display-usb-hid
I (244) cpu_start: App version:      squareline-latest-21-g87b0e54
I (251) cpu_start: Compile time:     Apr  5 2023 14:15:08
I (257) cpu_start: ELF file SHA256:  a74d139d1fcf326b...
I (263) cpu_start: ESP-IDF:          v5.0-dev-8835-g29737e1bc8a-dirt
I (270) cpu_start: Min chip rev:     v0.0
I (275) cpu_start: Max chip rev:     v0.99
I (279) cpu_start: Chip rev:         v0.1
I (284) heap_init: Initializing. RAM available for dynamic allocation:
I (291) heap_init: At 3FC976B8 len 00052058 (328 KiB): DRAM
I (298) heap_init: At 3FCE9710 len 00005724 (21 KiB): STACK/DRAM
I (304) heap_init: At 3FCF0000 len 00008000 (32 KiB): DRAM
I (310) heap_init: At 600FE010 len 00001FF0 (7 KiB): RTCRAM
I (318) spi_flash: detected chip: gd
I (321) spi_flash: flash io: dio
W (325) spi_flash: Detected size(8192k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
I (338) sleep: Configure to isolate all GPIO pins in sleep state
I (345) sleep: Enable automatic switching of GPIO sleep configuration
I (352) app_start: Starting scheduler on CPU0
I (357) app_start: Starting scheduler on CPU1
I (357) main_task: Started on CPU0
I (367) main_task: Calling app_main()
I (367) gpio: GPIO[18]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (377) gpio: GPIO[12]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (387) gpio: GPIO[13]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (397) gpio: GPIO[17]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (467) USB-OTG: Installing USB Host
I (497) LVGL: Starting LVGL task
I (497) gpio: GPIO[4]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (497) gpio: GPIO[8]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (627) USB-OTG: Setting LCD backlight: 100%
I (667) example: Keyboard input device group was set.
I (667) example: Example initialization done.
I (667) main_task: Returned from app_main()
```
