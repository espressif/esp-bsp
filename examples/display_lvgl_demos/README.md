# Display LVGL Demos

This example shows LVGL internal demos with RGB LCD.

For common issues about RGB LCD, please refer to [LCD Development Guide](https://docs.espressif.com/projects/esp-iot-solution/en/latest/display/lcd/lcd_development_guide.html#common-problems).

## How to use the example

### Hardware Required

* ESP32-S3-LCD-EV-Board or ESP32-S3-LCD-EV-Board-2
* USB-C Cable

### Compile and flash

```
idf.py -p COMx build flash monitor
```

### Example outputs

```
...
I (0) cpu_start: App cpu up.
I (923) esp_psram: SPI SRAM memory test OK
I (932) cpu_start: Pro cpu start user code
I (932) cpu_start: cpu freq: 240000000 Hz
I (932) cpu_start: Application information:
I (935) cpu_start: Project name:     display_lvgl_demos
I (941) cpu_start: App version:      squareline-latest-29-geed37e1
I (948) cpu_start: Compile time:     Dec  2 2023 15:41:34
I (954) cpu_start: ELF file SHA256:  fb3ec2c6026f1bb4...
I (960) cpu_start: ESP-IDF:          v5.1.2
I (965) cpu_start: Min chip rev:     v0.0
I (969) cpu_start: Max chip rev:     v0.99
I (974) cpu_start: Chip rev:         v0.2
I (979) heap_init: Initializing. RAM available for dynamic allocation:
I (986) heap_init: At 3FC9BF28 len 0004D7E8 (309 KiB): DRAM
I (992) heap_init: At 3FCE9710 len 00005724 (21 KiB): STACK/DRAM
I (999) heap_init: At 3FCF0000 len 00008000 (32 KiB): DRAM
I (1005) heap_init: At 600FE010 len 00001FD8 (7 KiB): RTCRAM
I (1011) esp_psram: Adding pool of 15552K of PSRAM memory to heap allocator
I (1019) spi_flash: detected chip: gd
I (1023) spi_flash: flash io: qio
W (1027) spi_flash: Detected size(16384k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
I (1041) sleep: Configure to isolate all GPIO pins in sleep state
I (1048) sleep: Enable automatic switching of GPIO sleep configuration
I (1055) app_start: Starting scheduler on CPU0
I (1060) app_start: Starting scheduler on CPU1
I (1060) main_task: Started on CPU0
I (1070) esp_psram: Reserving pool of 32K of internal memory for DMA/internal allocations
I (1078) main_task: Calling app_main()
I (1083) bsp_probe: Detect module with 16MB PSRAM
I (1088) bsp_probe: Detect sub_board3 with 800x480 LCD (ST7262), Touch (GT1151)
I (1096) bsp_sub_board: Initialize RGB panel
I (1134) gt1151: IC version: GT1158_000101(Patch)_0102(Mask)_00(SensorID)
I (1136) bsp_lvgl_port: Create LVGL task
I (1136) bsp_lvgl_port: Starting LVGL task
I (1164) app_main: Avoid lcd tearing effect
I (1165) app_main: LVGL direct-mode
W (1165) S3-LCD-EV-BOARD: This board doesn't support to change brightness of LCD
I (1171) app_main: Display LVGL demo
I (1370) main_task: Returned from app_main()
```

<a href="https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_lvgl_demos">
    <img alt="Try it with ESP Launchpad" src="https://espressif.github.io/esp-launchpad/assets/try_with_launchpad.png" width="250" height="70">
</a>
