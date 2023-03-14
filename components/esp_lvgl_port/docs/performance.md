# LCD & LVGL Performance

This document provides steps, how to set up your LCD and LVGL port for the best performance and comparison of different settings. All settings and measurements are valid for Espressif's chips.

## Performance metrics

In this document we will use following metrics for performance evaluation:

1. Measure time needed for refreshing the whole screen.
2. Use LVGL's `lv_demo_benchmark()` -test suite- to measure FPS (weighted FPS).
3. Use LVGL's `lv_demo_music()` -demo application- to measure FPS (average FPS).

## Settings on ESP32 chips which have impact on LCD and LVGL performance

Following options and settings have impact on LCD performance (FPS). Some options yield only small difference in FPS (e.g. ~1 FPS), and some of them are more significant. Usually it depends on application, resources and size of screen. Also the HW interface and clock speed is really important (not included in this sheet).

### LVGL Buffer size

* The size of **LVGL buffer** and **double buffering enables** has big impact on performance. 
* The best is use the same size of the buffer, like size of the LCD in pixels and enable double buffering. 
* If there is not enough RAM inside the CPU, you can use PSRAM (DMA is not available).

### Compiler optimization level

Recommended level is "Performance" for good results. The "Performance" setting causes the compiled code to be larger and faster, but will be easier to correlate code addresses to source file lines.

* `CONFIG_COMPILER_OPTIMIZATION_PERF=y`

### CPU frequency

The CPU frequency has a big impact on LCD performance. The recommended value is 240 MHz.

* `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y`

### Flash frequency

The flash clock frequency has a big impact on LCD performance. When PSRAM is used, the PSRAM clock frequency must be the same as a flash clock frequency. The recommended value is 120 MHz if it is supported by flash and RAM chip. 

* `CONFIG_ESPTOOLPY_FLASHFREQ_120M=y`

### Flash mode

The recommended flash mode is QIO, which uses two additional GPIOs for SPI flash communication. (It depends on HW)

* `CONFIG_ESPTOOLPY_FLASHMODE_QIO=y`

### Fast LVGL memory

This option puts some LVGL functions into IRAM for speed up.

* `CONFIG_LV_ATTRIBUTE_FAST_MEM_USE_IRAM=y`

### Affinity main task to second core

The main LVGL task can be processed on the second core of the CPU. It can increase performance. (It is available only on dual-core chips)

* `CONFIG_ESP_MAIN_TASK_AFFINITY_CPU1=y`

### Using LVGL memcpy instead of standard memcpy

You can select between using standard memcpy and memset functions or LVGL implementation of the memcpy and memset functions. For some tasks it can be faster but other tasks are slower. It depends on the application.

* `CONFIG_LV_MEMCPY_MEMSET_STD=n`

### Default LVGL display refresh period

Reducing the value of this param won't improve actual LVGL performance normally, it will increase weighted or average FPS.

* `CONFIG_LV_DISP_DEF_REFR_PERIOD=10`

## Example of changing FPS when changing settings

<img src="https://github.com/espressif/esp-bsp/blob/master/docu/pics/7inch-Capacitive-Touch-LCD-C_l.jpg?raw=true" align="right" width="300px" />

Default settings:
* BSP example `display_lvgl_demos` with `ws_7inch` component
* LCD: 7" 800x480
* Intarface: 16bit parallel Intel 8080
* Clock: 20 MHz
* LVGL buffer size: 800 x 50
* LVGL double buffer: YES
* Optimization: Debug
* CPU frequency: 160 MHz
* Flash frequency: 80 MHz
* Flash mode: DIO
* Memcpy: standard
* LVGL display refresh period: 30 ms

### Internal RAM with DMA

| Average FPS | Weighted FPS | Changed settings |
| :---: | :---: | ---------------- |
|  17   |  32   | Default          |
|  18   |  36   | + Optimization: Performance (`CONFIG_COMPILER_OPTIMIZATION_PERF=y`) |
|  21   |  46   | + CPU frequency: 240 MHz (`CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y`) |
|  26   |  56   | + Flash frequency: 120 MHz, Flash mode: QIO (`CONFIG_ESPTOOLPY_FLASHFREQ_120M=y` + `CONFIG_ESPTOOLPY_FLASHMODE_QIO=y`) |
|  28   |  **60**   | + LVGL fast memory enabled (`CONFIG_LV_ATTRIBUTE_FAST_MEM_USE_IRAM=y`) |
|  28   |  59   | + Affinity main task to CPU1 (`CONFIG_ESP_MAIN_TASK_AFFINITY_CPU1=y`) |
|  27   |  57   | + Memcpy from LVGL instead of standard (`CONFIG_LV_MEMCPY_MEMSET_STD=n`) |
|  41   |  55   | + (`CONFIG_LV_DISP_DEF_REFR_PERIOD=10`) |

### PSRAM (QUAD) without DMA

Default changes:
* LCD clock: 2 MHz
* PSRAM frequency: 80 MHz

| Average FPS |  Weighted FPS  | Changed settings |
| :---: | :---: | ---------------- |
|  11   |   7   | Default          |
|  11   |   7   | + Optimization: Performance (`CONFIG_COMPILER_OPTIMIZATION_PERF=y`) |
|  12   |   8   | + CPU frequency: 240 MHz   (`CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y`) |
|  12   |   9   | + Flash frequency: 120 MHz, PSRAM frequency: 120 MHz, Flash mode: QIO (`CONFIG_ESPTOOLPY_FLASHFREQ_120M=y` + `CONFIG_SPIRAM_SPEED_120M=y` + `CONFIG_ESPTOOLPY_FLASHMODE_QIO=y`) |
|  12   |   9   | + LVGL fast memory enabled (`CONFIG_LV_ATTRIBUTE_FAST_MEM_USE_IRAM=y`) |
|  12   |   9   | + Affinity main task to CPU1 (`CONFIG_ESP_MAIN_TASK_AFFINITY_CPU1=y`) |
|  12   |   9   | + Memcpy from LVGL instead of standard (`CONFIG_LV_MEMCPY_MEMSET_STD=n`) |
|  12   |   8   | + LVGL buffer size: 800 x 480 |
|  26   |   8   | + (`CONFIG_LV_DISP_DEF_REFR_PERIOD=10`) |
|  31   |  23   | + LCD clock: 10 MHz [^1] |

[^1]: This is not working in defualt and sometimes in fast changes on screen is not working properly.

### RGB LCD (without LVGL port), PSRAM (octal) with GDMA - ESP32-S3-LCD-EV-BOARD

<img src="https://github.com/espressif/esp-bsp/blob/master/docu/pics/esp32-s3-lcd-ev-board_800x480.png?raw=true" align="right" width="300px" />

Default settings:
* BSP example `display_lvgl_demos`
* LCD: 4.3" 800x480
* Intarface: RGB
* LVGL buffer size: 800 x 100
* LVGL double buffer: NO
* Optimization: Debug
* CPU frequency: 160 MHz
* Flash frequency: 80 MHz
* PSRAM frequency: 80 MHz
* Flash mode: DIO
* Memcpy: standard
* LVGL display refresh period: 30 ms

| Average FPS |  Weighted FPS  | Changed settings |
| :---: | :---: | ---------------- |
|  18   |  24   | Default          |
|  18   |  26   | + Optimization: Performance (`CONFIG_COMPILER_OPTIMIZATION_PERF=y`) |
|  21   |  32   | + CPU frequency: 240 MHz   (`CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y`) |
|  21   |  32   | + Flash mode: QIO (`CONFIG_ESPTOOLPY_FLASHMODE_QIO=y`) |
|  21   |  32   | + LVGL fast memory enabled (`CONFIG_LV_ATTRIBUTE_FAST_MEM_USE_IRAM=y`) |
|  21   |  33   | + Affinity main task to CPU1 (`CONFIG_ESP_MAIN_TASK_AFFINITY_CPU1=y`) |
|  21   |  33   | + Memcpy from LVGL instead of standard (`CONFIG_LV_MEMCPY_MEMSET_STD=n`) |
|  35   |  34   | + (`CONFIG_LV_DISP_DEF_REFR_PERIOD=10`) |

## Conclusion

The LCD performance depends on a lot of things and settings. The settings mentioned in this document are only part of the all things, which can be done for better performance. It depends on a lot of other things, like HW interface, clocks, LCD controller and more. 

The screen FPS is impacted too by the count of LVGL objects and their styles.
