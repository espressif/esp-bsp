# API Reference

## Header files

- [bsp/esp_wrover_kit/include/bsp/display.h](#file-bspesp_wrover_kitincludebspdisplayh)
- [bsp/esp_wrover_kit/include/bsp/esp_wrover_kit.h](#file-bspesp_wrover_kitincludebspesp_wrover_kith)

## File bsp/esp_wrover_kit/include/bsp/display.h

_BSP LCD._

This file offers API for basic LCD control. It is useful for users who want to use the LCD without the default Graphical Library LVGL.

For standard LCD initialization with LVGL graphical library, you can call all-in-one function [**bsp\_display\_start()**](#function-bsp_display_start).

## Structures and Types

| Type | Name |
| ---: | :--- |
| struct | [**bsp\_display\_config\_t**](#struct-bsp_display_config_t) <br>_BSP display configuration structure._ |

## Functions

| Type | Name |
| ---: | :--- |
|  esp\_err\_t | [**bsp\_display\_backlight\_off**](#function-bsp_display_backlight_off) (void) <br>_Turn off display backlight._ |
|  esp\_err\_t | [**bsp\_display\_backlight\_on**](#function-bsp_display_backlight_on) (void) <br>_Turn on display backlight._ |
|  esp\_err\_t | [**bsp\_display\_brightness\_init**](#function-bsp_display_brightness_init) (void) <br>_Initialize display's brightness._ |
|  esp\_err\_t | [**bsp\_display\_brightness\_set**](#function-bsp_display_brightness_set) (int brightness\_percent) <br>_Set display's brightness._ |
|  esp\_err\_t | [**bsp\_display\_new**](#function-bsp_display_new) (const [**bsp\_display\_config\_t**](#struct-bsp_display_config_t) \*config, esp\_lcd\_panel\_handle\_t \*ret\_panel, esp\_lcd\_panel\_io\_handle\_t \*ret\_io) <br>_Create new display panel._ |

## Macros

| Type | Name |
| ---: | :--- |
| define  | [**BSP\_LCD\_BIGENDIAN**](#define-bsp_lcd_bigendian)  (1)<br> |
| define  | [**BSP\_LCD\_BITS\_PER\_PIXEL**](#define-bsp_lcd_bits_per_pixel)  (16)<br> |
| define  | [**BSP\_LCD\_COLOR\_FORMAT**](#define-bsp_lcd_color_format)  (ESP\_LCD\_COLOR\_FORMAT\_RGB565)<br> |
| define  | [**BSP\_LCD\_COLOR\_SPACE**](#define-bsp_lcd_color_space)  (ESP\_LCD\_COLOR\_SPACE\_RGB)<br> |
| define  | [**BSP\_LCD\_H\_RES**](#define-bsp_lcd_h_res)  (240)<br> |
| define  | [**BSP\_LCD\_V\_RES**](#define-bsp_lcd_v_res)  (320)<br> |
| define  | [**ESP\_LCD\_COLOR\_FORMAT\_RGB565**](#define-esp_lcd_color_format_rgb565)  (1)<br> |
| define  | [**ESP\_LCD\_COLOR\_FORMAT\_RGB888**](#define-esp_lcd_color_format_rgb888)  (2)<br> |

## Structures and Types Documentation

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

## Macros Documentation

### define `BSP_LCD_BIGENDIAN`

```c
#define BSP_LCD_BIGENDIAN (1)
```

### define `BSP_LCD_BITS_PER_PIXEL`

```c
#define BSP_LCD_BITS_PER_PIXEL (16)
```

### define `BSP_LCD_COLOR_FORMAT`

```c
#define BSP_LCD_COLOR_FORMAT (ESP_LCD_COLOR_FORMAT_RGB565)
```

### define `BSP_LCD_COLOR_SPACE`

```c
#define BSP_LCD_COLOR_SPACE (ESP_LCD_COLOR_SPACE_RGB)
```

### define `BSP_LCD_H_RES`

```c
#define BSP_LCD_H_RES (240)
```

### define `BSP_LCD_V_RES`

```c
#define BSP_LCD_V_RES (320)
```

### define `ESP_LCD_COLOR_FORMAT_RGB565`

```c
#define ESP_LCD_COLOR_FORMAT_RGB565 (1)
```

### define `ESP_LCD_COLOR_FORMAT_RGB888`

```c
#define ESP_LCD_COLOR_FORMAT_RGB888 (2)
```


## File bsp/esp_wrover_kit/include/bsp/esp_wrover_kit.h

_ESP BSP: ESP-WROVER-KIT._



## Structures and Types

| Type | Name |
| ---: | :--- |
| enum  | [**bsp\_button\_t**](#enum-bsp_button_t)  <br> |
| struct | [**bsp\_display\_cfg\_t**](#struct-bsp_display_cfg_t) <br>_BSP display configuration structure._ |
| enum  | [**bsp\_led\_t**](#enum-bsp_led_t)  <br> |
| typedef enum bsp\_led\_t | [**bsp\_led\_t**](#typedef-bsp_led_t)  <br> |
| struct | [**bsp\_sdcard\_cfg\_t**](#struct-bsp_sdcard_cfg_t) <br>_BSP SD card configuration structure._ |

## Functions

| Type | Name |
| ---: | :--- |
|  bool | [**bsp\_button\_get**](#function-bsp_button_get) (const bsp\_button\_t btn) <br>_Get button's state._ |
|  esp\_err\_t | [**bsp\_button\_init**](#function-bsp_button_init) (const bsp\_button\_t btn) <br>_Set button's GPIO as input._ |
|  lv\_indev\_t \* | [**bsp\_display\_get\_input\_dev**](#function-bsp_display_get_input_dev) (void) <br>_Get pointer to input device (touch, buttons, ...)_ |
|  bool | [**bsp\_display\_lock**](#function-bsp_display_lock) (uint32\_t timeout\_ms) <br>_Take LVGL mutex._ |
|  void | [**bsp\_display\_rotate**](#function-bsp_display_rotate) (lv\_display\_t \*disp, lv\_disp\_rotation\_t rotation) <br>_Rotate screen._ |
|  lv\_display\_t \* | [**bsp\_display\_start**](#function-bsp_display_start) (void) <br>_Initialize display._ |
|  lv\_display\_t \* | [**bsp\_display\_start\_with\_config**](#function-bsp_display_start_with_config) (const [**bsp\_display\_cfg\_t**](#struct-bsp_display_cfg_t) \*cfg) <br>_Initialize display._ |
|  void | [**bsp\_display\_unlock**](#function-bsp_display_unlock) (void) <br>_Give LVGL mutex._ |
|  esp\_err\_t | [**bsp\_iot\_button\_create**](#function-bsp_iot_button_create) (button\_handle\_t btn\_array, int \*btn\_cnt, int btn\_array\_size) <br>_Initialize all buttons._ |
|  esp\_err\_t | [**bsp\_led\_set**](#function-bsp_led_set) (const bsp\_led\_t led\_io, const bool on) <br>_Turn LED on/off._ |
|  esp\_err\_t | [**bsp\_leds\_init**](#function-bsp_leds_init) (void) <br>_Set LED's GPIOs as output push-pull._ |
|  sdmmc\_card\_t \* | [**bsp\_sdcard\_get\_handle**](#function-bsp_sdcard_get_handle) (void) <br>_Get SD card handle._ |
|  void | [**bsp\_sdcard\_get\_sdmmc\_host**](#function-bsp_sdcard_get_sdmmc_host) (const int slot, sdmmc\_host\_t \*config) <br>_Get SD card MMC host config._ |
|  void | [**bsp\_sdcard\_get\_sdspi\_host**](#function-bsp_sdcard_get_sdspi_host) (const int slot, sdmmc\_host\_t \*config) <br>_Get SD card SPI host config._ |
|  esp\_err\_t | [**bsp\_sdcard\_mount**](#function-bsp_sdcard_mount) (void) <br>_Mount microSD card to virtual file system._ |
|  void | [**bsp\_sdcard\_sdmmc\_get\_slot**](#function-bsp_sdcard_sdmmc_get_slot) (const int slot, sdmmc\_slot\_config\_t \*config) <br>_Get SD card MMC slot config._ |
|  esp\_err\_t | [**bsp\_sdcard\_sdmmc\_mount**](#function-bsp_sdcard_sdmmc_mount) ([**bsp\_sdcard\_cfg\_t**](#struct-bsp_sdcard_cfg_t) \*cfg) <br>_Mount microSD card to virtual file system (MMC mode)_ |
|  void | [**bsp\_sdcard\_sdspi\_get\_slot**](#function-bsp_sdcard_sdspi_get_slot) (const spi\_host\_device\_t spi\_host, sdspi\_device\_config\_t \*config) <br>_Get SD card SPI slot config._ |
|  esp\_err\_t | [**bsp\_sdcard\_sdspi\_mount**](#function-bsp_sdcard_sdspi_mount) ([**bsp\_sdcard\_cfg\_t**](#struct-bsp_sdcard_cfg_t) \*cfg) <br>_Mount microSD card to virtual file system (SPI mode)_ |
|  esp\_err\_t | [**bsp\_sdcard\_unmount**](#function-bsp_sdcard_unmount) (void) <br>_Unmount micorSD card from virtual file system._ |
|  esp\_err\_t | [**bsp\_spiffs\_mount**](#function-bsp_spiffs_mount) (void) <br>_Mount SPIFFS to virtual file system._ |
|  esp\_err\_t | [**bsp\_spiffs\_unmount**](#function-bsp_spiffs_unmount) (void) <br>_Unmount SPIFFS from virtual file system._ |

## Macros

| Type | Name |
| ---: | :--- |
| define  | [**BSP\_BOARD\_ESP\_WROVER\_KIT**](#define-bsp_board_esp_wrover_kit)  <br> |
| define  | [**BSP\_BUTTON\_BOOT\_IO**](#define-bsp_button_boot_io)  (GPIO\_NUM\_0)<br> |
| define  | [**BSP\_CAPS\_AUDIO**](#define-bsp_caps_audio)  0<br> |
| define  | [**BSP\_CAPS\_AUDIO\_MIC**](#define-bsp_caps_audio_mic)  0<br> |
| define  | [**BSP\_CAPS\_AUDIO\_SPEAKER**](#define-bsp_caps_audio_speaker)  0<br> |
| define  | [**BSP\_CAPS\_BUTTONS**](#define-bsp_caps_buttons)  1<br> |
| define  | [**BSP\_CAPS\_CAMERA**](#define-bsp_caps_camera)  0<br> |
| define  | [**BSP\_CAPS\_DISPLAY**](#define-bsp_caps_display)  1<br> |
| define  | [**BSP\_CAPS\_IMU**](#define-bsp_caps_imu)  0<br> |
| define  | [**BSP\_CAPS\_LED**](#define-bsp_caps_led)  1<br> |
| define  | [**BSP\_CAPS\_SDCARD**](#define-bsp_caps_sdcard)  1<br> |
| define  | [**BSP\_CAPS\_TOUCH**](#define-bsp_caps_touch)  0<br> |
| define  | [**BSP\_LCD\_BACKLIGHT**](#define-bsp_lcd_backlight)  (GPIO\_NUM\_5)<br> |
| define  | [**BSP\_LCD\_DC**](#define-bsp_lcd_dc)  (GPIO\_NUM\_21)<br> |
| define  | [**BSP\_LCD\_DRAW\_BUFF\_DOUBLE**](#define-bsp_lcd_draw_buff_double)  (1)<br> |
| define  | [**BSP\_LCD\_DRAW\_BUFF\_SIZE**](#define-bsp_lcd_draw_buff_size)  (BSP\_LCD\_H\_RES \* 30)<br> |
| define  | [**BSP\_LCD\_PIXEL\_CLOCK\_HZ**](#define-bsp_lcd_pixel_clock_hz)  (40 \* 1000 \* 1000)<br> |
| define  | [**BSP\_LCD\_RST**](#define-bsp_lcd_rst)  (GPIO\_NUM\_18)<br> |
| define  | [**BSP\_LCD\_SPI\_CLK**](#define-bsp_lcd_spi_clk)  (GPIO\_NUM\_19)<br> |
| define  | [**BSP\_LCD\_SPI\_CS**](#define-bsp_lcd_spi_cs)  (GPIO\_NUM\_22)<br> |
| define  | [**BSP\_LCD\_SPI\_MISO**](#define-bsp_lcd_spi_miso)  (GPIO\_NUM\_25)<br> |
| define  | [**BSP\_LCD\_SPI\_MOSI**](#define-bsp_lcd_spi_mosi)  (GPIO\_NUM\_23)<br> |
| define  | [**BSP\_LCD\_SPI\_NUM**](#define-bsp_lcd_spi_num)  (SPI2\_HOST)<br> |
| define  | [**BSP\_SDSPI\_HOST**](#define-bsp_sdspi_host)  (SPI3\_HOST)<br> |
| define  | [**BSP\_SD\_CLK**](#define-bsp_sd_clk)  (GPIO\_NUM\_14)<br> |
| define  | [**BSP\_SD\_CMD**](#define-bsp_sd_cmd)  (GPIO\_NUM\_15)<br> |
| define  | [**BSP\_SD\_D0**](#define-bsp_sd_d0)  (GPIO\_NUM\_2)<br> |
| define  | [**BSP\_SD\_D1**](#define-bsp_sd_d1)  (GPIO\_NUM\_4)<br> |
| define  | [**BSP\_SD\_D2**](#define-bsp_sd_d2)  (GPIO\_NUM\_12)<br> |
| define  | [**BSP\_SD\_D3**](#define-bsp_sd_d3)  (GPIO\_NUM\_13)<br> |
| define  | [**BSP\_SD\_DET**](#define-bsp_sd_det)  (GPIO\_NUM\_21)<br> |
| define  | [**BSP\_SD\_MOUNT\_POINT**](#define-bsp_sd_mount_point)  CONFIG\_BSP\_SD\_MOUNT\_POINT<br> |
| define  | [**BSP\_SD\_SPI\_CLK**](#define-bsp_sd_spi_clk)  (GPIO\_NUM\_14)<br> |
| define  | [**BSP\_SD\_SPI\_CS**](#define-bsp_sd_spi_cs)  (GPIO\_NUM\_13)<br> |
| define  | [**BSP\_SD\_SPI\_MISO**](#define-bsp_sd_spi_miso)  (GPIO\_NUM\_2)<br> |
| define  | [**BSP\_SD\_SPI\_MOSI**](#define-bsp_sd_spi_mosi)  (GPIO\_NUM\_15)<br> |
| define  | [**BSP\_SPIFFS\_MOUNT\_POINT**](#define-bsp_spiffs_mount_point)  CONFIG\_BSP\_SPIFFS\_MOUNT\_POINT<br> |

## Structures and Types Documentation

### enum `bsp_button_t`

```c
enum bsp_button_t {
    BSP_BUTTON_BOOT,
    BSP_BUTTON_NUM
};
```

### struct `bsp_display_cfg_t`

_BSP display configuration structure._

Variables:

-  unsigned int buff_dma  <br>Allocated LVGL buffer will be DMA capable

-  unsigned int buff_spiram  <br>Allocated LVGL buffer will be in PSRAM

-  uint32\_t buffer_size  <br>Size of the buffer for the screen in pixels

-  bool double_buffer  <br>True, if should be allocated two buffers

-  struct [**bsp\_display\_cfg\_t**](#struct-bsp_display_cfg_t) flags  

-  lvgl\_port\_cfg\_t lvgl_port_cfg  <br>LVGL port configuration

### enum `bsp_led_t`

```c
enum bsp_led_t {
    BSP_LED_RED = GPIO_NUM_0,
    BSP_LED_GREEN = GPIO_NUM_2,
    BSP_LED_BLUE = GPIO_NUM_4
};
```

### typedef `bsp_led_t`

```c
typedef enum bsp_led_t bsp_led_t;
```

### struct `bsp_sdcard_cfg_t`

_BSP SD card configuration structure._

Variables:

-  sdmmc\_host\_t \* host  

-  const esp\_vfs\_fat\_sdmmc\_mount\_config\_t \* mount  

-  const sdmmc\_slot\_config\_t \* sdmmc  

-  const sdspi\_device\_config\_t \* sdspi  

-  union [**bsp\_sdcard\_cfg\_t**](#struct-bsp_sdcard_cfg_t) slot  


## Functions Documentation

### function `bsp_button_get`

_Get button's state._
```c
bool bsp_button_get (
    const bsp_button_t btn
) 
```


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
    const bsp_button_t btn
) 
```


**Parameters:**


* `btn` Button to be initialized 


**Returns:**



* ESP\_OK Success
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



* ESP\_OK Success
* ESP\_ERR\_INVALID\_ARG Parameter error
### function `bsp_leds_init`

_Set LED's GPIOs as output push-pull._
```c
esp_err_t bsp_leds_init (
    void
) 
```


**Returns:**



* ESP\_OK Success
* ESP\_ERR\_INVALID\_ARG Parameter error
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
* ESP\_ERR\_NO\_MEM if memory can not be allocated
* ESP\_FAIL if partition can not be mounted
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

_Unmount micorSD card from virtual file system._
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

## Macros Documentation

### define `BSP_BOARD_ESP_WROVER_KIT`

```c
#define BSP_BOARD_ESP_WROVER_KIT 
```

### define `BSP_BUTTON_BOOT_IO`

```c
#define BSP_BUTTON_BOOT_IO (GPIO_NUM_0)
```

### define `BSP_CAPS_AUDIO`

```c
#define BSP_CAPS_AUDIO 0
```

### define `BSP_CAPS_AUDIO_MIC`

```c
#define BSP_CAPS_AUDIO_MIC 0
```

### define `BSP_CAPS_AUDIO_SPEAKER`

```c
#define BSP_CAPS_AUDIO_SPEAKER 0
```

### define `BSP_CAPS_BUTTONS`

```c
#define BSP_CAPS_BUTTONS 1
```

### define `BSP_CAPS_CAMERA`

```c
#define BSP_CAPS_CAMERA 0
```

### define `BSP_CAPS_DISPLAY`

```c
#define BSP_CAPS_DISPLAY 1
```

### define `BSP_CAPS_IMU`

```c
#define BSP_CAPS_IMU 0
```

### define `BSP_CAPS_LED`

```c
#define BSP_CAPS_LED 1
```

### define `BSP_CAPS_SDCARD`

```c
#define BSP_CAPS_SDCARD 1
```

### define `BSP_CAPS_TOUCH`

```c
#define BSP_CAPS_TOUCH 0
```

### define `BSP_LCD_BACKLIGHT`

```c
#define BSP_LCD_BACKLIGHT (GPIO_NUM_5)
```

### define `BSP_LCD_DC`

```c
#define BSP_LCD_DC (GPIO_NUM_21)
```

### define `BSP_LCD_DRAW_BUFF_DOUBLE`

```c
#define BSP_LCD_DRAW_BUFF_DOUBLE (1)
```

### define `BSP_LCD_DRAW_BUFF_SIZE`

```c
#define BSP_LCD_DRAW_BUFF_SIZE (BSP_LCD_H_RES * 30)
```

### define `BSP_LCD_PIXEL_CLOCK_HZ`

```c
#define BSP_LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)
```

### define `BSP_LCD_RST`

```c
#define BSP_LCD_RST (GPIO_NUM_18)
```

### define `BSP_LCD_SPI_CLK`

```c
#define BSP_LCD_SPI_CLK (GPIO_NUM_19)
```

### define `BSP_LCD_SPI_CS`

```c
#define BSP_LCD_SPI_CS (GPIO_NUM_22)
```

### define `BSP_LCD_SPI_MISO`

```c
#define BSP_LCD_SPI_MISO (GPIO_NUM_25)
```

### define `BSP_LCD_SPI_MOSI`

```c
#define BSP_LCD_SPI_MOSI (GPIO_NUM_23)
```

### define `BSP_LCD_SPI_NUM`

```c
#define BSP_LCD_SPI_NUM (SPI2_HOST)
```

### define `BSP_SDSPI_HOST`

```c
#define BSP_SDSPI_HOST (SPI3_HOST)
```

### define `BSP_SD_CLK`

```c
#define BSP_SD_CLK (GPIO_NUM_14)
```

### define `BSP_SD_CMD`

```c
#define BSP_SD_CMD (GPIO_NUM_15)
```

### define `BSP_SD_D0`

```c
#define BSP_SD_D0 (GPIO_NUM_2)
```

### define `BSP_SD_D1`

```c
#define BSP_SD_D1 (GPIO_NUM_4)
```

### define `BSP_SD_D2`

```c
#define BSP_SD_D2 (GPIO_NUM_12)
```

### define `BSP_SD_D3`

```c
#define BSP_SD_D3 (GPIO_NUM_13)
```

### define `BSP_SD_DET`

```c
#define BSP_SD_DET (GPIO_NUM_21)
```

### define `BSP_SD_MOUNT_POINT`

```c
#define BSP_SD_MOUNT_POINT CONFIG_BSP_SD_MOUNT_POINT
```

### define `BSP_SD_SPI_CLK`

```c
#define BSP_SD_SPI_CLK (GPIO_NUM_14)
```

### define `BSP_SD_SPI_CS`

```c
#define BSP_SD_SPI_CS (GPIO_NUM_13)
```

### define `BSP_SD_SPI_MISO`

```c
#define BSP_SD_SPI_MISO (GPIO_NUM_2)
```

### define `BSP_SD_SPI_MOSI`

```c
#define BSP_SD_SPI_MOSI (GPIO_NUM_15)
```

### define `BSP_SPIFFS_MOUNT_POINT`

```c
#define BSP_SPIFFS_MOUNT_POINT CONFIG_BSP_SPIFFS_MOUNT_POINT
```


