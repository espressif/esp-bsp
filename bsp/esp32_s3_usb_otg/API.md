# API Reference

<div align="center">




| :1234: [CAPABILITIES](#1234-capabilities) | :floppy_disk: [SD CARD AND SPIFFS](#floppy_disk-sd-card-and-spiffs) | :pager: [DISPLAY AND TOUCH](#pager-display-and-touch) | :radio_button: [BUTTONS](#radio_button-buttons) | :bulb: [LEDS](#bulb-leds) | :electric_plug: [USB](#electric_plug-usb) | :battery: [BATTERY](#battery-battery) | 
| :-------------------------: | :-------------------------: | :-------------------------: | :-------------------------: | :-------------------------: | :-------------------------: | :-------------------------: | 

</div>



## Overview

This document provides an overview of the ESP-BSP (Board Support Package) API as implemented by this board.

While the ESP-BSP framework defines a unified API shared across multiple boards, this documentation focuses only on the APIs supported by the current board. Any APIs not applicable to this board's hardware are excluded or may not be functional.

The goal of this document is to make it easier for developers to understand the available APIs and how to use them consistently across different boards.

## General

### Pinout

Each BSP defines a set of macros for default pin assignments used by its hardware peripherals.
These macros allow users to configure or reference standard interfaces like I2C, SPI, LCD, audio, or SD cards easily.

- I2C: `BSP_I2C_*`
- Display: `BSP_LCD_*`
- Audio I2S: `BSP_I2S_*`
- USB: `BSP_USB_*`
- SD Card (MMC): `BSP_SD_*`
- SD Card (SPI): `BSP_SD_SPI_*`

> [!NOTE]
> Not all boards support all interfaces. You should always check if the related capability macro (e.g., BSP_CAPS_SDCARD) is defined.

### I2C

Some devices included in BSPs (e.g., sensors, displays, audio codecs) communicate via the I2C interface. In many cases, I2C is initialized automatically as part of the device setup. However, you can manually initialize or deinitialize the I2C peripheral using the following API:

```
/* Initialize the default I2C bus used by the BSP */
bsp_i2c_init();

...

/* Deinitialize the I2C bus */
bsp_i2c_deinit();
```

If you need direct access to the initialized I2C bus (e.g., to communicate with an external peripheral not handled by the BSP), you can retrieve the I2C bus handle:

```
i2c_master_bus_handle_t i2c = bsp_i2c_get_handle();
```

> [!NOTE]
> The BSP ensures that I2C initialization is performed only once, even if called multiple times. This helps avoid conflicts when multiple components rely on the same I2C bus.


### ADC

Some devices included in BSPs (such as buttons, battery monitoring, etc.) use the ADC peripheral. In most cases, the ADC is automatically initialized as part of the specific device setup. However, you can manually initialize the ADC using the following API:

```
/* Initialize the ADC peripheral */
bsp_adc_initialize();
```

If you need direct access to the ADC instance (e.g., for custom measurements), you can retrieve the handle:

```
adc_oneshot_unit_handle_t adc = bsp_adc_get_handle();
```

> [!NOTE]
> The BSP ensures the ADC is initialized only once, even if `bsp_adc_initialize()` is called multiple times.

### Features

Some boards support enabling or disabling specific hardware features (such as LCD, SD card, camera, etc.) to reduce power consumption or manage shared resources. The BSP provides a unified API to control these features:

```
/* Enable the LCD feature */
bsp_feature_enable(BSP_FEATURE_LCD, true);

/* Disable the speaker to reduce power usage */
bsp_feature_enable(BSP_FEATURE_SPEAKER, false);
```

Supported feature flags (may vary depending on the board):
- `BSP_FEATURE_LCD` - Display module
- `BSP_FEATURE_TOUCH` - Touch controller
- `BSP_FEATURE_SD` - SD card interface
- `BSP_FEATURE_SPEAKER`-  Audio speaker
- `BSP_FEATURE_BATTERY` - Battery monitoring
- `BSP_FEATURE_VIBRATION` - Vibration motor

> [!NOTE]
> Not all BSPs support feature toggling, and some features may not be available or controllable via this API. Always check the BSP header or documentation for supported features.

> [!TIP]
> Disabling unused features can help reduce power consumption, especially in battery-powered applications.




## Identification

Each BSP defines an identifier macro in the form of `BSP_BOARD_*`.

### Board Name API Reference



## Macros

| Type | Name |
| ---: | :--- |
| define  | [**BSP\_BOARD\_ESP32\_S3\_USB\_OTG**](#define-bsp_board_esp32_s3_usb_otg)  <br> |









## :1234: Capabilities

Each BSP defines a set of capability macros that indicate which features are supported.
The list may look like this.
You can use these macros to conditionally compile code depending on feature availability.

### Capabilities API Reference



## Macros

| Type | Name |
| ---: | :--- |
| define  | [**BSP\_CAPS\_AUDIO**](#define-bsp_caps_audio)  0<br> |
| define  | [**BSP\_CAPS\_AUDIO\_MIC**](#define-bsp_caps_audio_mic)  0<br> |
| define  | [**BSP\_CAPS\_AUDIO\_SPEAKER**](#define-bsp_caps_audio_speaker)  0<br> |
| define  | [**BSP\_CAPS\_BAT**](#define-bsp_caps_bat)  1<br> |
| define  | [**BSP\_CAPS\_BUTTONS**](#define-bsp_caps_buttons)  1<br> |
| define  | [**BSP\_CAPS\_DISPLAY**](#define-bsp_caps_display)  1<br> |
| define  | [**BSP\_CAPS\_IMU**](#define-bsp_caps_imu)  0<br> |
| define  | [**BSP\_CAPS\_LED**](#define-bsp_caps_led)  1<br> |
| define  | [**BSP\_CAPS\_SDCARD**](#define-bsp_caps_sdcard)  1<br> |
| define  | [**BSP\_CAPS\_TOUCH**](#define-bsp_caps_touch)  0<br> |











### ADC API Reference


## Functions

| Type | Name |
| ---: | :--- |
|  adc\_oneshot\_unit\_handle\_t | [**bsp\_adc\_get\_handle**](#function-bsp_adc_get_handle) (void) <br>_Get ADC handle._ |
|  esp\_err\_t | [**bsp\_adc\_initialize**](#function-bsp_adc_initialize) (void) <br>_Initialize ADC._ |

## Macros

| Type | Name |
| ---: | :--- |
| define  | [**BSP\_ADC\_UNIT**](#define-bsp_adc_unit)  ADC\_UNIT\_1<br> |



## Functions Documentation

### function `bsp_adc_get_handle`

_Get ADC handle._
```c
adc_oneshot_unit_handle_t bsp_adc_get_handle (
    void
) 
```


**Note:**

This function is available only in IDF5 and higher



**Returns:**

ADC handle
### function `bsp_adc_initialize`

_Initialize ADC._
```c
esp_err_t bsp_adc_initialize (
    void
) 
```


The ADC can be initialized inside BSP, when needed.






## :floppy_disk: SD Card and SPIFFS

### SPIFFS Initialization / Deinitialization

Each BSP provides a simple API for mounting and unmounting the SPI Flash File System (SPIFFS).

```
/* Mount SPIFFS to the virtual file system */
bsp_spiffs_mount();

/* ... perform file operations ... */

/* Unmount SPIFFS from the virtual file system */
bsp_spiffs_unmount();
```

### SD Card Initialization / Deinitialization

The BSP offers a flexible API for working with SD cards. In addition to the default mount and unmount functions, you can also use a configuration structure or access preconfigured `host` and `slot` structures.

Mount with Default Configuration

```
/* Mount microSD card to the virtual file system */
bsp_sdcard_mount();

/* ... perform file operations ... */

/* Unmount microSD card */
bsp_sdcard_unmount();
```

Mount with Custom Configuration

Some BSPs allow selecting between SDMMC and SPI interfaces for the SD card. Use the appropriate API function based on your hardware:
```
bsp_sdcard_cfg_t cfg = {0};
/* Mount SD card using SDMMC interface */
bsp_sdcard_sdmmc_mount(&cfg);
```

or

```
bsp_sdcard_cfg_t cfg = {0};
/* Mount SD card using SPI interface */
bsp_sdcard_sdspi_mount(&cfg)
```

> [!NOTE]
> Not all BSPs support both SDMMC and SPI modes. Check the board documentation to see which interfaces are available.
> If an unsupported interface is used, the API will return `ESP_ERR_NOT_SUPPORTED` error.

### After Mounting

Once the SD card or SPIFFS is mounted, you can use standard file I/O functions (`fopen`, `fread`, `fwrite`, `fclose`, etc.) provided by ESP-IDF's VFS (Virtual File System).

To print basic SD card information (after mounting), you can use:
```
sdmmc_card_t *sdcard = bsp_sdcard_get_handle();
sdmmc_card_print_info(stdout, sdcard);
```

> [!TIP]
> The bsp_sdcard_get_handle() function returns a pointer to the sdmmc_card_t structure, which contains detailed information about the connected SD card.

### SD Card and SPIFFS API Reference

## Structures and Types

| Type | Name |
| ---: | :--- |
| struct | [**bsp\_sdcard\_cfg\_t**](#struct-bsp_sdcard_cfg_t) <br>_BSP SD card configuration structure._ |

## Functions

| Type | Name |
| ---: | :--- |
|  sdmmc\_card\_t \* | [**bsp\_sdcard\_get\_handle**](#function-bsp_sdcard_get_handle) (void) <br>_Get SD card handle._ |
|  void | [**bsp\_sdcard\_get\_sdmmc\_host**](#function-bsp_sdcard_get_sdmmc_host) (const int slot, sdmmc\_host\_t \*config) <br>_Get SD card MMC host config._ |
|  void | [**bsp\_sdcard\_get\_sdspi\_host**](#function-bsp_sdcard_get_sdspi_host) (const int slot, sdmmc\_host\_t \*config) <br>_Get SD card SPI host config._ |
|  esp\_err\_t | [**bsp\_sdcard\_mount**](#function-bsp_sdcard_mount) (void) <br>_Mount microSD card to virtual file system._ |
|  void | [**bsp\_sdcard\_sdmmc\_get\_slot**](#function-bsp_sdcard_sdmmc_get_slot) (const int slot, sdmmc\_slot\_config\_t \*config) <br>_Get SD card MMC slot config._ |
|  esp\_err\_t | [**bsp\_sdcard\_sdmmc\_mount**](#function-bsp_sdcard_sdmmc_mount) ([**bsp\_sdcard\_cfg\_t**](#struct-bsp_sdcard_cfg_t) \*cfg) <br>_Mount microSD card to virtual file system (MMC mode)_ |
|  void | [**bsp\_sdcard\_sdspi\_get\_slot**](#function-bsp_sdcard_sdspi_get_slot) (const spi\_host\_device\_t spi\_host, sdspi\_device\_config\_t \*config) <br>_Get SD card SPI slot config._ |
|  esp\_err\_t | [**bsp\_sdcard\_sdspi\_mount**](#function-bsp_sdcard_sdspi_mount) ([**bsp\_sdcard\_cfg\_t**](#struct-bsp_sdcard_cfg_t) \*cfg) <br>_Mount microSD card to virtual file system (SPI mode)_ |
|  esp\_err\_t | [**bsp\_sdcard\_unmount**](#function-bsp_sdcard_unmount) (void) <br>_Unmount microSD card from virtual file system._ |
|  esp\_err\_t | [**bsp\_spiffs\_mount**](#function-bsp_spiffs_mount) (void) <br>_Mount SPIFFS to virtual file system._ |
|  esp\_err\_t | [**bsp\_spiffs\_unmount**](#function-bsp_spiffs_unmount) (void) <br>_Unmount SPIFFS from virtual file system._ |

## Macros

| Type | Name |
| ---: | :--- |
| define  | [**BSP\_SDSPI\_HOST**](#define-bsp_sdspi_host)  (SPI2\_HOST)<br> |
| define  | [**BSP\_SD\_CLK**](#define-bsp_sd_clk)  (GPIO\_NUM\_36)<br> |
| define  | [**BSP\_SD\_CMD**](#define-bsp_sd_cmd)  (GPIO\_NUM\_35)<br> |
| define  | [**BSP\_SD\_D0**](#define-bsp_sd_d0)  (GPIO\_NUM\_37)<br> |
| define  | [**BSP\_SD\_D1**](#define-bsp_sd_d1)  (GPIO\_NUM\_38)<br> |
| define  | [**BSP\_SD\_D2**](#define-bsp_sd_d2)  (GPIO\_NUM\_33)<br> |
| define  | [**BSP\_SD\_D3**](#define-bsp_sd_d3)  (GPIO\_NUM\_34)<br> |
| define  | [**BSP\_SD\_MOUNT\_POINT**](#define-bsp_sd_mount_point)  CONFIG\_BSP\_SD\_MOUNT\_POINT<br> |
| define  | [**BSP\_SD\_SPI\_CLK**](#define-bsp_sd_spi_clk)  (GPIO\_NUM\_36)<br> |
| define  | [**BSP\_SD\_SPI\_CS**](#define-bsp_sd_spi_cs)  (GPIO\_NUM\_34)<br> |
| define  | [**BSP\_SD\_SPI\_MISO**](#define-bsp_sd_spi_miso)  (GPIO\_NUM\_37)<br> |
| define  | [**BSP\_SD\_SPI\_MOSI**](#define-bsp_sd_spi_mosi)  (GPIO\_NUM\_35)<br> |
| define  | [**BSP\_SPIFFS\_MOUNT\_POINT**](#define-bsp_spiffs_mount_point)  CONFIG\_BSP\_SPIFFS\_MOUNT\_POINT<br> |


## Structures and Types Documentation

### struct `bsp_sdcard_cfg_t`

_BSP SD card configuration structure._

Variables:

-  sdmmc\_host\_t \* host  

-  const esp\_vfs\_fat\_sdmmc\_mount\_config\_t \* mount  

-  const sdmmc\_slot\_config\_t \* sdmmc  

-  const sdspi\_device\_config\_t \* sdspi  

-  union [**bsp\_sdcard\_cfg\_t**](#struct-bsp_sdcard_cfg_t) slot  


## Functions Documentation

### function `bsp_sdcard_get_handle`

_Get SD card handle._
```c
sdmmc_card_t * bsp_sdcard_get_handle (
    void
) 
```


**Returns:**

SD card handle
### function `bsp_sdcard_get_sdmmc_host`

_Get SD card MMC host config._
```c
void bsp_sdcard_get_sdmmc_host (
    const int slot,
    sdmmc_host_t *config
) 
```


**Parameters:**


* `slot` SD card slot 
* `config` Structure which will be filled
### function `bsp_sdcard_get_sdspi_host`

_Get SD card SPI host config._
```c
void bsp_sdcard_get_sdspi_host (
    const int slot,
    sdmmc_host_t *config
) 
```


**Parameters:**


* `slot` SD card slot 
* `config` Structure which will be filled
### function `bsp_sdcard_mount`

_Mount microSD card to virtual file system._
```c
esp_err_t bsp_sdcard_mount (
    void
) 
```


**Returns:**



* ESP\_OK on success
* ESP\_ERR\_INVALID\_STATE if esp\_vfs\_fat\_sdmmc\_mount was already called
* ESP\_ERR\_NO\_MEM if memory cannot be allocated
* ESP\_FAIL if partition cannot be mounted
* other error codes from SDMMC or SPI drivers, SDMMC protocol, or FATFS drivers
### function `bsp_sdcard_sdmmc_get_slot`

_Get SD card MMC slot config._
```c
void bsp_sdcard_sdmmc_get_slot (
    const int slot,
    sdmmc_slot_config_t *config
) 
```


**Parameters:**


* `slot` SD card slot 
* `config` Structure which will be filled
### function `bsp_sdcard_sdmmc_mount`

_Mount microSD card to virtual file system (MMC mode)_
```c
esp_err_t bsp_sdcard_sdmmc_mount (
    bsp_sdcard_cfg_t *cfg
) 
```


**Parameters:**


* `cfg` SD card configuration


**Returns:**



* ESP\_OK on success
* ESP\_ERR\_INVALID\_STATE if esp\_vfs\_fat\_sdmmc\_mount was already called
* ESP\_ERR\_NO\_MEM if memory cannot be allocated
* ESP\_FAIL if partition cannot be mounted
* other error codes from SDMMC or SPI drivers, SDMMC protocol, or FATFS drivers
### function `bsp_sdcard_sdspi_get_slot`

_Get SD card SPI slot config._
```c
void bsp_sdcard_sdspi_get_slot (
    const spi_host_device_t spi_host,
    sdspi_device_config_t *config
) 
```


**Parameters:**


* `spi_host` SPI host ID 
* `config` Structure which will be filled
### function `bsp_sdcard_sdspi_mount`

_Mount microSD card to virtual file system (SPI mode)_
```c
esp_err_t bsp_sdcard_sdspi_mount (
    bsp_sdcard_cfg_t *cfg
) 
```


**Parameters:**


* `cfg` SD card configuration


**Returns:**



* ESP\_OK on success
* ESP\_ERR\_INVALID\_STATE if esp\_vfs\_fat\_sdmmc\_mount was already called
* ESP\_ERR\_NO\_MEM if memory cannot be allocated
* ESP\_FAIL if partition cannot be mounted
* other error codes from SDMMC or SPI drivers, SDMMC protocol, or FATFS drivers
### function `bsp_sdcard_unmount`

_Unmount microSD card from virtual file system._
```c
esp_err_t bsp_sdcard_unmount (
    void
) 
```


**Returns:**



* ESP\_OK on success
* ESP\_ERR\_NOT\_FOUND if the partition table does not contain FATFS partition with given label
* ESP\_ERR\_INVALID\_STATE if esp\_vfs\_fat\_spiflash\_mount was already called
* ESP\_ERR\_NO\_MEM if memory can not be allocated
* ESP\_FAIL if partition can not be mounted
* other error codes from wear levelling library, SPI flash driver, or FATFS drivers
### function `bsp_spiffs_mount`

_Mount SPIFFS to virtual file system._
```c
esp_err_t bsp_spiffs_mount (
    void
) 
```


**Returns:**



* ESP\_OK on success
* ESP\_ERR\_INVALID\_STATE if esp\_vfs\_spiffs\_register was already called
* ESP\_ERR\_NO\_MEM if memory can not be allocated
* ESP\_FAIL if partition can not be mounted
* other error codes
### function `bsp_spiffs_unmount`

_Unmount SPIFFS from virtual file system._
```c
esp_err_t bsp_spiffs_unmount (
    void
) 
```


**Returns:**



* ESP\_OK on success
* ESP\_ERR\_NOT\_FOUND if the partition table does not contain SPIFFS partition with given label
* ESP\_ERR\_INVALID\_STATE if esp\_vfs\_spiffs\_unregister was already called
* ESP\_ERR\_NO\_MEM if memory can not be allocated
* ESP\_FAIL if partition can not be mounted
* other error codes






## :pager: Display and Touch

### Initialization

ESP-BSP provides two ways to initialize the **display**, **touch** and **LVGL**.

Simple method:

```
/* Initialize display, touch, and LVGL */
lv_display_t display = bsp_display_start();
```

Configurable method:

```
bsp_display_cfg_t cfg = {
    .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),   /* See LVGL Port for more info */
    .buffer_size = BSP_LCD_V_RES * BSP_LCD_H_RES,   /* Screen buffer size in pixels */
    .double_buffer = true,                          /* Allocate two buffers if true */
    .flags = {
        .buff_dma = true,                           /* Use DMA-capable LVGL buffer */
        .buff_spiram = false,                       /* Allocate buffer in PSRAM if true */
    }
};
cfg.lvgl_port_cfg.task_stack = 10000;   /* Example: change LVGL task stack size */
/* Initialize display, touch, and LVGL */
lv_display_t display = bsp_display_start_with_config(&cfg);
```

After initialization, you can use the [LVGL](https://docs.lvgl.io/master/) API or [LVGL Port](../components/esp_lvgl_port/README.md) API.

### Initialization without LVGL - NoGLIB BSP

To initialize the LCD without LVGL, use:

```
esp_lcd_panel_handle_t panel_handle;
esp_lcd_panel_io_handle_t io_handle;
const bsp_display_config_t bsp_disp_cfg = {
    .max_transfer_sz = (BSP_LCD_H_RES * 100) * sizeof(uint16_t),
};
BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new(&bsp_disp_cfg, &panel_handle, &io_handle));
```

To initialize the LCD touch without LVGL, use:

```
esp_lcd_touch_handle_t tp;
bsp_touch_new(NULL, &tp);
```

After initialization, you can use the [ESP-LCD](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/lcd/index.html) API and [ESP-LCD Touch](../components/lcd_touch/README.md) API.

### Set Brightness

```
/* Set display brightness to 100% */
bsp_display_backlight_on();

/* Set display brightness to 0% */
bsp_display_backlight_off();

/* Set display brightness to 50% */
bsp_display_brightness_set(50);
```

> [!NOTE]
> Some boards do not support changing brightness. They return an `ESP_ERR_NOT_SUPPORTED` error.

### LVGL API Usage (only when initialized with LVGL)

All LVGL calls must be protected using lock/unlock:

```
/* Wait until other tasks finish screen operations */
bsp_display_lock(0);
...
lv_obj_t * screen = lv_disp_get_scr_act(disp_handle);
lv_obj_t * obj = lv_label_create(screen);
...
/* Unlock after screen operations are done */
bsp_display_unlock();
```

### Screen rotation (only when initialized with LVGL)

```
bsp_display_lock(0);
/* Rotate display to 90 */
bsp_display_rotate(display, LV_DISPLAY_ROTATION_90);
bsp_display_unlock();
```

> [!NOTE]
> Some LCDs do not support hardware rotation and instead use software rotation, which consumes more memory.

### Available constants

Constants like screen resolution, pin configuration, and other options are defined in the BSP header files (`{bsp_name}.h`, `display.h`, `touch.h`).
Below are some of the most relevant predefined constants:

- `BSP_LCD_H_RES` - Horizontal resolution in pixels
- `BSP_LCD_V_RES` - Vertical resolution in pixels
- `BSP_LCD_SPI_NUM` - SPI bus used by the LCD (if applicable)


### Display and Touch API Reference

## Structures and Types

| Type | Name |
| ---: | :--- |
| struct | [**bsp\_display\_cfg\_t**](#struct-bsp_display_cfg_t) <br>_BSP display configuration structure._ |
| struct | [**bsp\_display\_config\_t**](#struct-bsp_display_config_t) <br>_BSP display configuration structure._ |

## Functions

| Type | Name |
| ---: | :--- |
|  esp\_err\_t | [**bsp\_display\_backlight\_off**](#function-bsp_display_backlight_off) (void) <br>_Turn off display backlight._ |
|  esp\_err\_t | [**bsp\_display\_backlight\_on**](#function-bsp_display_backlight_on) (void) <br>_Turn on display backlight._ |
|  esp\_err\_t | [**bsp\_display\_brightness\_init**](#function-bsp_display_brightness_init) (void) <br>_Initialize display's brightness._ |
|  esp\_err\_t | [**bsp\_display\_brightness\_set**](#function-bsp_display_brightness_set) (int brightness\_percent) <br>_Set display's brightness._ |
|  lv\_indev\_t \* | [**bsp\_display\_get\_input\_dev**](#function-bsp_display_get_input_dev) (void) <br>_Get pointer to input device (touch, buttons, ...)_ |
|  bool | [**bsp\_display\_lock**](#function-bsp_display_lock) (uint32\_t timeout\_ms) <br>_Take LVGL mutex._ |
|  esp\_err\_t | [**bsp\_display\_new**](#function-bsp_display_new) (const [**bsp\_display\_config\_t**](#struct-bsp_display_config_t) \*config, esp\_lcd\_panel\_handle\_t \*ret\_panel, esp\_lcd\_panel\_io\_handle\_t \*ret\_io) <br>_Create new display panel._ |
|  void | [**bsp\_display\_rotate**](#function-bsp_display_rotate) (lv\_display\_t \*disp, lv\_disp\_rotation\_t rotation) <br>_Rotate screen._ |
|  lv\_display\_t \* | [**bsp\_display\_start**](#function-bsp_display_start) (void) <br>_Initialize display._ |
|  lv\_display\_t \* | [**bsp\_display\_start\_with\_config**](#function-bsp_display_start_with_config) (const [**bsp\_display\_cfg\_t**](#struct-bsp_display_cfg_t) \*cfg) <br>_Initialize display._ |
|  void | [**bsp\_display\_unlock**](#function-bsp_display_unlock) (void) <br>_Give LVGL mutex._ |

## Macros

| Type | Name |
| ---: | :--- |
| define  | [**BSP\_LCD\_BACKLIGHT**](#define-bsp_lcd_backlight)  (GPIO\_NUM\_9)<br> |
| define  | [**BSP\_LCD\_BIGENDIAN**](#define-bsp_lcd_bigendian)  (1)<br> |
| define  | [**BSP\_LCD\_BITS\_PER\_PIXEL**](#define-bsp_lcd_bits_per_pixel)  (16)<br> |
| define  | [**BSP\_LCD\_COLOR\_FORMAT**](#define-bsp_lcd_color_format)  (ESP\_LCD\_COLOR\_FORMAT\_RGB565)<br> |
| define  | [**BSP\_LCD\_COLOR\_SPACE**](#define-bsp_lcd_color_space)  (LCD\_RGB\_ELEMENT\_ORDER\_RGB)<br> |
| define  | [**BSP\_LCD\_DC**](#define-bsp_lcd_dc)  (GPIO\_NUM\_4)<br> |
| define  | [**BSP\_LCD\_DRAW\_BUFF\_DOUBLE**](#define-bsp_lcd_draw_buff_double)  (1)<br> |
| define  | [**BSP\_LCD\_DRAW\_BUFF\_SIZE**](#define-bsp_lcd_draw_buff_size)  (BSP\_LCD\_H\_RES \* 30)<br> |
| define  | [**BSP\_LCD\_H\_RES**](#define-bsp_lcd_h_res)  (240)<br> |
| define  | [**BSP\_LCD\_PIXEL\_CLOCK\_HZ**](#define-bsp_lcd_pixel_clock_hz)  (40 \* 1000 \* 1000)<br> |
| define  | [**BSP\_LCD\_RST**](#define-bsp_lcd_rst)  (GPIO\_NUM\_8)<br> |
| define  | [**BSP\_LCD\_SPI\_CLK**](#define-bsp_lcd_spi_clk)  (GPIO\_NUM\_6)<br> |
| define  | [**BSP\_LCD\_SPI\_CS**](#define-bsp_lcd_spi_cs)  (GPIO\_NUM\_5)<br> |
| define  | [**BSP\_LCD\_SPI\_MOSI**](#define-bsp_lcd_spi_mosi)  (GPIO\_NUM\_7)<br> |
| define  | [**BSP\_LCD\_SPI\_NUM**](#define-bsp_lcd_spi_num)  (SPI3\_HOST)<br> |
| define  | [**BSP\_LCD\_V\_RES**](#define-bsp_lcd_v_res)  (240)<br> |
| define  | [**ESP\_LCD\_COLOR\_FORMAT\_RGB565**](#define-esp_lcd_color_format_rgb565)  (1)<br> |
| define  | [**ESP\_LCD\_COLOR\_FORMAT\_RGB888**](#define-esp_lcd_color_format_rgb888)  (2)<br> |


## Structures and Types Documentation

### struct `bsp_display_cfg_t`

_BSP display configuration structure._

Variables:

-  unsigned int buff_dma  <br>Allocated LVGL buffer will be DMA capable

-  unsigned int buff_spiram  <br>Allocated LVGL buffer will be in PSRAM

-  uint32\_t buffer_size  <br>Size of the buffer for the screen in pixels

-  bool double_buffer  <br>True, if should be allocated two buffers

-  struct [**bsp\_display\_cfg\_t**](#struct-bsp_display_cfg_t) flags  

-  lvgl\_port\_cfg\_t lvgl_port_cfg  <br>LVGL port configuration

### struct `bsp_display_config_t`

_BSP display configuration structure._

Variables:

-  int max_transfer_sz  <br>Maximum transfer size, in bytes.


## Functions Documentation

### function `bsp_display_backlight_off`

_Turn off display backlight._
```c
esp_err_t bsp_display_backlight_off (
    void
) 
```


Brightness is controlled with PWM signal to a pin controlling backlight. Brightness must be already initialized by calling [**bsp\_display\_brightness\_init()**](#function-bsp_display_brightness_init) or[**bsp\_display\_new()**](#function-bsp_display_new)



**Returns:**



* ESP\_OK On success
* ESP\_ERR\_INVALID\_ARG Parameter error
### function `bsp_display_backlight_on`

_Turn on display backlight._
```c
esp_err_t bsp_display_backlight_on (
    void
) 
```


Brightness is controlled with PWM signal to a pin controlling backlight. Brightness must be already initialized by calling [**bsp\_display\_brightness\_init()**](#function-bsp_display_brightness_init) or[**bsp\_display\_new()**](#function-bsp_display_new)



**Returns:**



* ESP\_OK On success
* ESP\_ERR\_INVALID\_ARG Parameter error
### function `bsp_display_brightness_init`

_Initialize display's brightness._
```c
esp_err_t bsp_display_brightness_init (
    void
) 
```


Brightness is controlled with PWM signal to a pin controlling backlight.



**Returns:**



* ESP\_OK On success
* ESP\_ERR\_INVALID\_ARG Parameter error
### function `bsp_display_brightness_set`

_Set display's brightness._
```c
esp_err_t bsp_display_brightness_set (
    int brightness_percent
) 
```


Brightness is controlled with PWM signal to a pin controlling backlight. Brightness must be already initialized by calling [**bsp\_display\_brightness\_init()**](#function-bsp_display_brightness_init) or[**bsp\_display\_new()**](#function-bsp_display_new)



**Parameters:**


* `brightness_percent` Brightness in [%] 


**Returns:**



* ESP\_OK On success
* ESP\_ERR\_INVALID\_ARG Parameter error
### function `bsp_display_get_input_dev`

_Get pointer to input device (touch, buttons, ...)_
```c
lv_indev_t * bsp_display_get_input_dev (
    void
) 
```


**Note:**

The LVGL input device is initialized in [**bsp\_display\_start()**](#function-bsp_display_start) function.



**Returns:**

Pointer to LVGL input device or NULL when not initialized
### function `bsp_display_lock`

_Take LVGL mutex._
```c
bool bsp_display_lock (
    uint32_t timeout_ms
) 
```


**Parameters:**


* `timeout_ms` Timeout in [ms]. 0 will block indefinitely. 


**Returns:**

true Mutex was taken 



**Returns:**

false Mutex was NOT taken
### function `bsp_display_new`

_Create new display panel._
```c
esp_err_t bsp_display_new (
    const bsp_display_config_t *config,
    esp_lcd_panel_handle_t *ret_panel,
    esp_lcd_panel_io_handle_t *ret_io
) 
```


For maximum flexibility, this function performs only reset and initialization of the display. You must turn on the display explicitly by calling esp\_lcd\_panel\_disp\_on\_off(). The display's backlight is not turned on either. You can use bsp\_display\_backlight\_on/off(), [**bsp\_display\_brightness\_set()**](#function-bsp_display_brightness_set) (on supported boards) or implement your own backlight control.

If you want to free resources allocated by this function, you can use esp\_lcd API, ie.:


````cpp
esp_lcd_panel_del(panel);
esp_lcd_panel_io_del(io);
spi_bus_free(spi_num_from_configuration);
````





**Parameters:**


* `config` display configuration 
* `ret_panel` esp\_lcd panel handle 
* `ret_io` esp\_lcd IO handle 


**Returns:**



* ESP\_OK On success
* Else esp\_lcd failure
### function `bsp_display_rotate`

_Rotate screen._
```c
void bsp_display_rotate (
    lv_display_t *disp,
    lv_disp_rotation_t rotation
) 
```


Display must be already initialized by calling [**bsp\_display\_start()**](#function-bsp_display_start)



**Parameters:**


* `disp` Pointer to LVGL display 
* `rotation` Angle of the display rotation
### function `bsp_display_start`

_Initialize display._
```c
lv_display_t * bsp_display_start (
    void
) 
```


This function initializes SPI, display controller and starts LVGL handling task.



**Returns:**

Pointer to LVGL display or NULL when error occurred
### function `bsp_display_start_with_config`

_Initialize display._
```c
lv_display_t * bsp_display_start_with_config (
    const bsp_display_cfg_t *cfg
) 
```


This function initializes SPI, display controller and starts LVGL handling task. LCD backlight must be enabled separately by calling [**bsp\_display\_brightness\_set()**](#function-bsp_display_brightness_set)



**Parameters:**


* `cfg` display configuration


**Returns:**

Pointer to LVGL display or NULL when error occurred
### function `bsp_display_unlock`

_Give LVGL mutex._
```c
void bsp_display_unlock (
    void
) 
```







## :radio_button: Buttons

Most boards include one or more user buttons. The BSP provides a simple API to initialize and use them.
Internally, it utilizes the [button](https://components.espressif.com/components/espressif/button/) component for event handling and debouncing.

```
/* Initialize all available buttons */
button_handle_t btns[BSP_BUTTON_NUM] = {NULL};
bsp_iot_button_create(btns, NULL, BSP_BUTTON_NUM);

/* Register a callback for button press */
for (int i = 0; i < BSP_BUTTON_NUM; i++) {
    iot_button_register_cb(btns[i], BUTTON_PRESS_DOWN, NULL, btn_handler, (void *) i);
}

/* Called on button press */
static void btn_handler(void *button_handle, void *usr_data)
{
    int button_index = (int)usr_data;
    ESP_LOGI(TAG, "Button %d pressed", button_index);
}
```

**Notes:**
- `BSP_BUTTON_NUM` defines the number of available buttons on the board

- Instead of relying on a numeric index, you can use the `bsp_button_t` structure defined in the BSP header to reference buttons by name (e.g., BSP_BUTTON_MUTE, BSP_BUTTON_MAIN, BSP_BUTTON_BOOT, etc.)

- The callback can be registered for different button events, such as `BUTTON_PRESS_DOWN`, `BUTTON_PRESS_UP`, or `BUTTON_LONG_PRESS_START`, `BUTTON_DOUBLE_CLICK`, ...

- Button features (e.g. long-press support, active level) are configured automatically by the BSP.

### Buttons API Reference

## Structures and Types

| Type | Name |
| ---: | :--- |
| enum  | [**bsp\_button\_t**](#enum-bsp_button_t)  <br> |

## Functions

| Type | Name |
| ---: | :--- |
|  bool | [**bsp\_button\_get**](#function-bsp_button_get) (const bsp\_button\_t btn) <br>_Get button's state._ |
|  esp\_err\_t | [**bsp\_button\_init**](#function-bsp_button_init) (void) <br>_Set button's GPIO as input._ |
|  esp\_err\_t | [**bsp\_iot\_button\_create**](#function-bsp_iot_button_create) (button\_handle\_t btn\_array, int \*btn\_cnt, int btn\_array\_size) <br>_Initialize all buttons._ |



## Structures and Types Documentation

### enum `bsp_button_t`

```c
enum bsp_button_t {
    BSP_BUTTON_OK = GPIO_NUM_0,
    BSP_BUTTON_DW = GPIO_NUM_11,
    BSP_BUTTON_UP = GPIO_NUM_10,
    BSP_BUTTON_MENU = GPIO_NUM_14,
    BSP_USB_OVERCURRENT = GPIO_NUM_21,
    BSP_BUTTON_NUM = 5
};
```


## Functions Documentation

### function `bsp_button_get`

_Get button's state._
```c
bool bsp_button_get (
    const bsp_button_t btn
) 
```


Note: For LCD panel button which is defined as BSP\_BUTTON\_MAIN, bsp\_display\_start should be called before call this function.



**Parameters:**


* `btn` Button to read 


**Returns:**

true Button pressed 



**Returns:**

false Button released
### function `bsp_button_init`

_Set button's GPIO as input._
```c
esp_err_t bsp_button_init (
    void
) 
```


**Returns:**



* ESP\_OK Success
* ESP\_ERR\_INVALID\_ARG Parameter error
### function `bsp_iot_button_create`

_Initialize all buttons._
```c
esp_err_t bsp_iot_button_create (
    button_handle_t btn_array,
    int *btn_cnt,
    int btn_array_size
) 
```


Returned button handlers must be used with espressif/button component API



**Note:**

For LCD panel button which is defined as BSP\_BUTTON\_MAIN, bsp\_display\_start should be called before call this function.



**Parameters:**


* `btn_array` Output button array 
* `btn_cnt` Number of button handlers saved to btn\_array, can be NULL 
* `btn_array_size` Size of output button array. Must be at least BSP\_BUTTON\_NUM 


**Returns:**



* ESP\_OK All buttons initialized
* ESP\_ERR\_INVALID\_ARG btn\_array is too small or NULL
* ESP\_FAIL Underlaying iot\_button\_create failed






## :bulb: LEDs

LEDs are handled similarly to buttons in BSP. The BSP uses the [led_indicator](https://components.espressif.com/components/espressif/led_indicator) component, which provides simple control over LED states and built-in effects such as blinking, breathing, and more. It also supports addressable RGB LEDs.

```
/* Initialize all LEDs */
bsp_led_indicator_create(leds, NULL, BSP_LED_NUM);

/* Set color of the first LED (for addressable RGB LEDs only) */
led_indicator_set_rgb(leds[0], SET_IRGB(0, 0x00, 0x64, 0x64));

/*
Start a predefined LED effect:
- BSP_LED_ON
- BSP_LED_OFF
- BSP_LED_BLINK_FAST
- BSP_LED_BLINK_SLOW
- BSP_LED_BREATHE_FAST
- BSP_LED_BREATHE_SLOW
*/
led_indicator_start(leds[0], BSP_LED_BREATHE_SLOW);
```

**Notes:**
- `BSP_LED_NUM` defines the total number of available LEDs on the board
- LEDs are automatically configured by the BSP (no need to set GPIO or direction manually)

### Leds API Reference

## Structures and Types

| Type | Name |
| ---: | :--- |
| enum  | [**bsp\_led\_t**](#enum-bsp_led_t)  <br> |
| typedef enum bsp\_led\_t | [**bsp\_led\_t**](#typedef-bsp_led_t)  <br> |

## Functions

| Type | Name |
| ---: | :--- |
|  esp\_err\_t | [**bsp\_led\_set**](#function-bsp_led_set) (const bsp\_led\_t led\_io, const bool on) <br>_Turn LED on/off._ |
|  esp\_err\_t | [**bsp\_leds\_init**](#function-bsp_leds_init) (void) <br>_Set LED's GPIOs as output push-pull._ |



## Structures and Types Documentation

### enum `bsp_led_t`

```c
enum bsp_led_t {
    BSP_LED_GREEN = GPIO_NUM_15,
    BSP_LED_YELLOW = GPIO_NUM_16
};
```

### typedef `bsp_led_t`

```c
typedef enum bsp_led_t bsp_led_t;
```


## Functions Documentation

### function `bsp_led_set`

_Turn LED on/off._
```c
esp_err_t bsp_led_set (
    const bsp_led_t led_io,
    const bool on
) 
```


**Parameters:**


* `led_io` LED io 
* `on` Switch LED on/off 


**Returns:**



* ESP\_OK On success
* ESP\_ERR\_INVALID\_ARG Parameter error
### function `bsp_leds_init`

_Set LED's GPIOs as output push-pull._
```c
esp_err_t bsp_leds_init (
    void
) 
```


**Returns:**



* ESP\_OK On success
* ESP\_ERR\_INVALID\_ARG Parameter error






## :electric_plug: USB

Boards with USB support define macros for USB pins, such as `BSP_USB_POS` and `BSP_USB_NEG`, and may also provide control APIs for enabling or disabling USB functionality.

```
/* Initialize USB in device mode and enable power */
bsp_usb_host_start(BSP_USB_HOST_POWER_MODE_USB_DEV, true);

...
/* Deinitialize and stop USB */
bsp_usb_host_stop();
```

> [!NOTE]
> Not all BSPs implement USB support or provide power control. Refer to the board's documentation and the BSP header files for available functions and supported modes.

For more USB-related APIs and configuration options, check the corresponding BSP header files.

### USB API Reference

## Structures and Types

| Type | Name |
| ---: | :--- |
| enum  | [**bsp\_usb\_host\_power\_mode\_t**](#enum-bsp_usb_host_power_mode_t)  <br>_Power modes of USB Host connector._ |
| typedef enum [**bsp\_usb\_host\_power\_mode\_t**](#enum-bsp_usb_host_power_mode_t) | [**bsp\_usb\_host\_power\_mode\_t**](#typedef-bsp_usb_host_power_mode_t)  <br>_Power modes of USB Host connector._ |

## Functions

| Type | Name |
| ---: | :--- |
|  esp\_err\_t | [**bsp\_usb\_host\_power\_mode**](#function-bsp_usb_host_power_mode) ([**bsp\_usb\_host\_power\_mode\_t**](#enum-bsp_usb_host_power_mode_t) mode, bool limit\_500mA) <br>_Select power source of USB Host connector._ |
|  esp\_err\_t | [**bsp\_usb\_host\_start**](#function-bsp_usb_host_start) ([**bsp\_usb\_host\_power\_mode\_t**](#enum-bsp_usb_host_power_mode_t) mode, bool limit\_500mA) <br>_Start USB host._ |
|  esp\_err\_t | [**bsp\_usb\_host\_stop**](#function-bsp_usb_host_stop) (void) <br>_Stop USB host._ |
|  esp\_err\_t | [**bsp\_usb\_mode\_select\_device**](#function-bsp_usb_mode_select_device) (void) <br>_Switch ESP32-S3-USB-OTG to USB device mode._ |
|  esp\_err\_t | [**bsp\_usb\_mode\_select\_host**](#function-bsp_usb_mode_select_host) (void) <br>_Switch ESP32-S3-USB-OTG to USB host mode._ |

## Macros

| Type | Name |
| ---: | :--- |
| define  | [**BSP\_USB\_DEV\_VBUS\_EN**](#define-bsp_usb_dev_vbus_en)  (GPIO\_NUM\_12)<br> |
| define  | [**BSP\_USB\_HOST\_VOLTAGE**](#define-bsp_usb_host_voltage)  (GPIO\_NUM\_1)<br> |
| define  | [**BSP\_USB\_HOST\_VOLTAGE\_DIV**](#define-bsp_usb_host_voltage_div)  (3.7f)<br> |
| define  | [**BSP\_USB\_LIMIT\_EN**](#define-bsp_usb_limit_en)  (GPIO\_NUM\_17)<br> |
| define  | [**BSP\_USB\_MODE\_SEL**](#define-bsp_usb_mode_sel)  (GPIO\_NUM\_18)<br> |
| define  | [**BSP\_USB\_NEG**](#define-bsp_usb_neg)  (GPIO\_NUM\_19)<br> |
| define  | [**BSP\_USB\_POS**](#define-bsp_usb_pos)  (GPIO\_NUM\_20)<br> |


## Structures and Types Documentation

### enum `bsp_usb_host_power_mode_t`

_Power modes of USB Host connector._
```c
enum bsp_usb_host_power_mode_t {
    BSP_USB_HOST_POWER_MODE_OFF,
    BSP_USB_HOST_POWER_MODE_BATTERY,
    BSP_USB_HOST_POWER_MODE_USB_DEV
};
```


For easy setup of USB host mode use [**bsp\_usb\_host\_start()**](#function-bsp_usb_host_start) function.

Use this function only if you want to change power mode on already initialized board, or in custom USB Host lib configurations.



**Note:**

USB Host connector can't be powered from debugging USB port (USB-UART0) 



**Note:**

If selecting battery mode, the battery slide switch must be switched on
### typedef `bsp_usb_host_power_mode_t`

_Power modes of USB Host connector._
```c
typedef enum bsp_usb_host_power_mode_t bsp_usb_host_power_mode_t;
```


For easy setup of USB host mode use [**bsp\_usb\_host\_start()**](#function-bsp_usb_host_start) function.

Use this function only if you want to change power mode on already initialized board, or in custom USB Host lib configurations.



**Note:**

USB Host connector can't be powered from debugging USB port (USB-UART0) 



**Note:**

If selecting battery mode, the battery slide switch must be switched on

## Functions Documentation

### function `bsp_usb_host_power_mode`

_Select power source of USB Host connector._
```c
esp_err_t bsp_usb_host_power_mode (
    bsp_usb_host_power_mode_t mode,
    bool limit_500mA
) 
```


**Parameters:**


* `mode` USB Host connector power mode 
* `limit_500mA` Limit output current to 500mA 


**Returns:**



* ESP\_OK On success
* ESP\_ERR\_INVALID\_ARG Parameter error
### function `bsp_usb_host_start`

_Start USB host._
```c
esp_err_t bsp_usb_host_start (
    bsp_usb_host_power_mode_t mode,
    bool limit_500mA
) 
```


This is a one-stop-shop function that will configure the board for USB Host mode and start USB Host library



**Parameters:**


* `mode` USB Host connector power mode 
* `limit_500mA` Limit output current to 500mA 


**Returns:**



* ESP\_OK On success
* ESP\_ERR\_INVALID\_ARG Parameter error
* ESP\_ERR\_NO\_MEM Memory cannot be allocated
### function `bsp_usb_host_stop`

_Stop USB host._
```c
esp_err_t bsp_usb_host_stop (
    void
) 
```


USB Host lib will be uninstalled and power from connector removed.



**Returns:**



* ESP\_OK On success
* ESP\_ERR\_INVALID\_ARG Parameter error
### function `bsp_usb_mode_select_device`

_Switch ESP32-S3-USB-OTG to USB device mode._
```c
esp_err_t bsp_usb_mode_select_device (
    void
) 
```


**Returns:**



* ESP\_OK On success
* ESP\_ERR\_INVALID\_ARG Parameter error
### function `bsp_usb_mode_select_host`

_Switch ESP32-S3-USB-OTG to USB host mode._
```c
esp_err_t bsp_usb_mode_select_host (
    void
) 
```


For easy setup of USB host mode use [**bsp\_usb\_host\_start()**](#function-bsp_usb_host_start) function.

Use this in custom USB Host lib configurations.



**Returns:**



* ESP\_OK On success
* ESP\_ERR\_INVALID\_ARG Parameter error






## :battery: Battery

Some boards with battery support can measure the battery voltage using an ADC channel. BSP provides a simple API for this:

```
/* Initialize the battery voltage measurement */
bsp_voltage_init();

/* Read battery voltage in millivolts */
int voltage = bsp_voltage_battery_get();
```

### Battery API Reference


## Functions

| Type | Name |
| ---: | :--- |
|  int | [**bsp\_voltage\_battery\_get**](#function-bsp_voltage_battery_get) (void) <br>_Get battery voltage._ |
|  esp\_err\_t | [**bsp\_voltage\_init**](#function-bsp_voltage_init) (void) <br>_Init voltage measurements._ |
|  int | [**bsp\_voltage\_usb\_get**](#function-bsp_voltage_usb_get) (void) <br>_Get USB device connector voltage._ |

## Macros

| Type | Name |
| ---: | :--- |
| define  | [**BSP\_BATTERY\_BOOST\_EN**](#define-bsp_battery_boost_en)  (GPIO\_NUM\_13)<br> |
| define  | [**BSP\_BATTERY\_VOLTAGE**](#define-bsp_battery_voltage)  (GPIO\_NUM\_2)<br> |
| define  | [**BSP\_BATTERY\_VOLTAGE\_DIV**](#define-bsp_battery_voltage_div)  (2)<br> |



## Functions Documentation

### function `bsp_voltage_battery_get`

_Get battery voltage._
```c
int bsp_voltage_battery_get (
    void
) 
```


**Note:**

[**bsp\_voltage\_init()**](#function-bsp_voltage_init) must be called first



**Returns:**

Resulting voltage in [mV] or -1 on error
### function `bsp_voltage_init`

_Init voltage measurements._
```c
esp_err_t bsp_voltage_init (
    void
) 
```


ADC configuration and calibration



**Note:**

If the calibration fails, voltage can't be measured 



**Returns:**

true Calibration OK 



**Returns:**

false Calibration failed 



**Returns:**



* ESP\_OK On success
* ESP\_ERR\_INVALID\_ARG Invalid arguments
* ESP\_ERR\_NO\_MEM No memory
* ESP\_ERR\_NOT\_FOUND ADC peripheral to be claimed is already in use
* ESP\_ERR\_NOT\_SUPPORTED ADC scheme required eFuse bits not burnt
### function `bsp_voltage_usb_get`

_Get USB device connector voltage._
```c
int bsp_voltage_usb_get (
    void
) 
```


**Note:**

[**bsp\_voltage\_init()**](#function-bsp_voltage_init) must be called first



**Returns:**

Resulting voltage in [mV] or -1 on error



