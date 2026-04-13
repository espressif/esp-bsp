# BSP: Display Rotation Example

## Overview

<table>
<tr><td valign="top">

This example demonstrates usage of ESP-BOX Board Support Package. This is a single purpose example, which is focused on rotating LCD display: user can rotating display by buttons.

</td><td width="200" valign="top">
  <img src="/examples/display_rotation/doc/pic.webp">
</td></tr>
</table>

## Build and Flash

To build and flash the example for a specific `{board}` and `{port}`, use the following command:

```
idf.py -D SDKCONFIG_DEFAULTS=sdkconfig.bsp.{board} -p {port} flash monitor
```
Make sure the correct board name is set in the `main/idf_component.yml` file under the `dependencies` section.

### Example outputs

```
I (241) cpu_start: ESP-IDF:          v5.0-dev-3434-g75b80d7a23
I (247) heap_init: Initializing. RAM available for dynamic allocation:
I (255) heap_init: At 3FC975C0 len 00048A40 (290 KiB): D/IRAM
I (261) heap_init: At 3FCE0000 len 0000EE34 (59 KiB): STACK/DRAM
I (268) heap_init: At 3FCF0000 len 00008000 (32 KiB): DRAM
I (274) heap_init: At 600FE000 len 00002000 (8 KiB): RTCRAM
I (281) spi_flash: detected chip: gd
I (284) spi_flash: flash io: dio
I (289) sleep: Configure to isolate all GPIO pins in sleep state
I (295) sleep: Enable automatic switching of GPIO sleep configuration
I (303) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
I (325) gpio: GPIO[4]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (325) gpio: GPIO[48]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (455) ESP-BOX: Setting LCD backlight: 100%
I (455) ESP-BOX: Starting LVGL task
I (495) ESP-BOX: Example initialization done.
```

## Launch Example

You can also try this example using ESP Launchpad:

<a href="https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_rotation-">
    <img alt="Try it with ESP Launchpad" src="https://espressif.github.io/esp-launchpad/assets/try_with_launchpad.png" width="250" height="70">
</a>
