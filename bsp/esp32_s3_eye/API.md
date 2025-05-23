# API Reference

## Header files

- [bsp/esp32_s3_eye/include/bsp/display.h](#file-bspesp32_s3_eyeincludebspdisplayh)
- [bsp/esp32_s3_eye/include/bsp/esp32_s3_eye.h](#file-bspesp32_s3_eyeincludebspesp32_s3_eyeh)

## File bsp/esp32_s3_eye/include/bsp/display.h

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
| define  | [**BSP\_LCD\_V\_RES**](#define-bsp_lcd_v_res)  (240)<br> |
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
#define BSP_LCD_V_RES (240)
```

### define `ESP_LCD_COLOR_FORMAT_RGB565`

```c
#define ESP_LCD_COLOR_FORMAT_RGB565 (1)
```

### define `ESP_LCD_COLOR_FORMAT_RGB888`

```c
#define ESP_LCD_COLOR_FORMAT_RGB888 (2)
```


## File bsp/esp32_s3_eye/include/bsp/esp32_s3_eye.h

_ESP BSP: ESP32-S3-EYE._



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
|  adc\_oneshot\_unit\_handle\_t | [**bsp\_adc\_get\_handle**](#function-bsp_adc_get_handle) (void) <br>_Get ADC handle._ |
|  esp\_err\_t | [**bsp\_adc\_initialize**](#function-bsp_adc_initialize) (void) <br>_Initialize ADC._ |
|  esp\_codec\_dev\_handle\_t | [**bsp\_audio\_codec\_microphone\_init**](#function-bsp_audio_codec_microphone_init) (void) <br>_Initialize microphone codec device._ |
|  const audio\_codec\_data\_if\_t \* | [**bsp\_audio\_get\_codec\_itf**](#function-bsp_audio_get_codec_itf) (void) <br>_Get codec I2S interface (initialized in bsp\_audio\_init)_ |
|  esp\_err\_t | [**bsp\_audio\_init**](#function-bsp_audio_init) (const i2s\_std\_config\_t \*i2s\_config) <br>_Init audio._ |
|  lv\_indev\_t \* | [**bsp\_display\_get\_input\_dev**](#function-bsp_display_get_input_dev) (void) <br>_Get pointer to input device (touch, buttons, ...)_ |
|  bool | [**bsp\_display\_lock**](#function-bsp_display_lock) (uint32\_t timeout\_ms) <br>_Take LVGL mutex._ |
|  void | [**bsp\_display\_rotate**](#function-bsp_display_rotate) (lv\_display\_t \*disp, lv\_disp\_rotation\_t rotation) <br>_Rotate screen._ |
|  lv\_display\_t \* | [**bsp\_display\_start**](#function-bsp_display_start) (void) <br>_Initialize display._ |
|  lv\_display\_t \* | [**bsp\_display\_start\_with\_config**](#function-bsp_display_start_with_config) (const [**bsp\_display\_cfg\_t**](#struct-bsp_display_cfg_t) \*cfg) <br>_Initialize display._ |
|  void | [**bsp\_display\_unlock**](#function-bsp_display_unlock) (void) <br>_Give LVGL mutex._ |
|  esp\_err\_t | [**bsp\_i2c\_deinit**](#function-bsp_i2c_deinit) (void) <br>_Deinit I2C driver and free its resources._ |
|  i2c\_master\_bus\_handle\_t | [**bsp\_i2c\_get\_handle**](#function-bsp_i2c_get_handle) (void) <br>_Get I2C driver handle._ |
|  esp\_err\_t | [**bsp\_i2c\_init**](#function-bsp_i2c_init) (void) <br>_Init I2C driver._ |
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
|  esp\_err\_t | [**bsp\_sdcard\_unmount**](#function-bsp_sdcard_unmount) (void) <br>_Unmount microSD card from virtual file system._ |
|  esp\_err\_t | [**bsp\_spiffs\_mount**](#function-bsp_spiffs_mount) (void) <br>_Mount SPIFFS to virtual file system._ |
|  esp\_err\_t | [**bsp\_spiffs\_unmount**](#function-bsp_spiffs_unmount) (void) <br>_Unmount SPIFFS from virtual file system._ |

## Macros

| Type | Name |
| ---: | :--- |
| define  | [**BSP\_ADC\_UNIT**](#define-bsp_adc_unit)  ADC\_UNIT\_1<br> |
| define  | [**BSP\_BOARD\_ESP32\_S3\_EYE**](#define-bsp_board_esp32_s3_eye)  <br> |
| define  | [**BSP\_BUTTONS\_IO**](#define-bsp_buttons_io)  (GPIO\_NUM\_1)<br> |
| define  | [**BSP\_BUTTON\_BOOT\_IO**](#define-bsp_button_boot_io)  (GPIO\_NUM\_0)<br> |
| define  | [**BSP\_CAMERA\_D0**](#define-bsp_camera_d0)  (GPIO\_NUM\_11)<br> |
| define  | [**BSP\_CAMERA\_D1**](#define-bsp_camera_d1)  (GPIO\_NUM\_9)<br> |
| define  | [**BSP\_CAMERA\_D2**](#define-bsp_camera_d2)  (GPIO\_NUM\_8)<br> |
| define  | [**BSP\_CAMERA\_D3**](#define-bsp_camera_d3)  (GPIO\_NUM\_10)<br> |
| define  | [**BSP\_CAMERA\_D4**](#define-bsp_camera_d4)  (GPIO\_NUM\_12)<br> |
| define  | [**BSP\_CAMERA\_D5**](#define-bsp_camera_d5)  (GPIO\_NUM\_18)<br> |
| define  | [**BSP\_CAMERA\_D6**](#define-bsp_camera_d6)  (GPIO\_NUM\_17)<br> |
| define  | [**BSP\_CAMERA\_D7**](#define-bsp_camera_d7)  (GPIO\_NUM\_16)<br> |
| define  | [**BSP\_CAMERA\_DEFAULT\_CONFIG**](#define-bsp_camera_default_config)  <br>_ESP32-S3-EYE camera default configuration._ |
| define  | [**BSP\_CAMERA\_HMIRROR**](#define-bsp_camera_hmirror)  0<br> |
| define  | [**BSP\_CAMERA\_HSYNC**](#define-bsp_camera_hsync)  (GPIO\_NUM\_7)<br> |
| define  | [**BSP\_CAMERA\_PCLK**](#define-bsp_camera_pclk)  (GPIO\_NUM\_13)<br> |
| define  | [**BSP\_CAMERA\_VFLIP**](#define-bsp_camera_vflip)  1<br> |
| define  | [**BSP\_CAMERA\_VSYNC**](#define-bsp_camera_vsync)  (GPIO\_NUM\_6)<br> |
| define  | [**BSP\_CAMERA\_XCLK**](#define-bsp_camera_xclk)  (GPIO\_NUM\_15)<br> |
| define  | [**BSP\_CAPS\_AUDIO**](#define-bsp_caps_audio)  1<br> |
| define  | [**BSP\_CAPS\_AUDIO\_MIC**](#define-bsp_caps_audio_mic)  1<br> |
| define  | [**BSP\_CAPS\_AUDIO\_SPEAKER**](#define-bsp_caps_audio_speaker)  0<br> |
| define  | [**BSP\_CAPS\_BUTTONS**](#define-bsp_caps_buttons)  1<br> |
| define  | [**BSP\_CAPS\_CAMERA**](#define-bsp_caps_camera)  1<br> |
| define  | [**BSP\_CAPS\_DISPLAY**](#define-bsp_caps_display)  1<br> |
| define  | [**BSP\_CAPS\_IMU**](#define-bsp_caps_imu)  1<br> |
| define  | [**BSP\_CAPS\_SDCARD**](#define-bsp_caps_sdcard)  1<br> |
| define  | [**BSP\_CAPS\_TOUCH**](#define-bsp_caps_touch)  0<br> |
| define  | [**BSP\_I2C\_NUM**](#define-bsp_i2c_num)  CONFIG\_BSP\_I2C\_NUM<br> |
| define  | [**BSP\_I2C\_SCL**](#define-bsp_i2c_scl)  (GPIO\_NUM\_5)<br> |
| define  | [**BSP\_I2C\_SDA**](#define-bsp_i2c_sda)  (GPIO\_NUM\_4)<br> |
| define  | [**BSP\_I2S\_DIN**](#define-bsp_i2s_din)  (GPIO\_NUM\_2)<br> |
| define  | [**BSP\_I2S\_LCLK**](#define-bsp_i2s_lclk)  (GPIO\_NUM\_42)<br> |
| define  | [**BSP\_I2S\_SCLK**](#define-bsp_i2s_sclk)  (GPIO\_NUM\_41)<br> |
| define  | [**BSP\_LCD\_BACKLIGHT**](#define-bsp_lcd_backlight)  (GPIO\_NUM\_48)<br> |
| define  | [**BSP\_LCD\_DC**](#define-bsp_lcd_dc)  (GPIO\_NUM\_43)<br> |
| define  | [**BSP\_LCD\_DRAW\_BUFF\_DOUBLE**](#define-bsp_lcd_draw_buff_double)  (0)<br> |
| define  | [**BSP\_LCD\_DRAW\_BUFF\_SIZE**](#define-bsp_lcd_draw_buff_size)  (BSP\_LCD\_H\_RES \* BSP\_LCD\_V\_RES)<br> |
| define  | [**BSP\_LCD\_PIXEL\_CLOCK\_HZ**](#define-bsp_lcd_pixel_clock_hz)  (80 \* 1000 \* 1000)<br> |
| define  | [**BSP\_LCD\_RST**](#define-bsp_lcd_rst)  (GPIO\_NUM\_NC)<br> |
| define  | [**BSP\_LCD\_SPI\_CLK**](#define-bsp_lcd_spi_clk)  (GPIO\_NUM\_21)<br> |
| define  | [**BSP\_LCD\_SPI\_CS**](#define-bsp_lcd_spi_cs)  (GPIO\_NUM\_44)<br> |
| define  | [**BSP\_LCD\_SPI\_MOSI**](#define-bsp_lcd_spi_mosi)  (GPIO\_NUM\_47)<br> |
| define  | [**BSP\_LCD\_SPI\_NUM**](#define-bsp_lcd_spi_num)  (SPI3\_HOST)<br> |
| define  | [**BSP\_SDSPI\_HOST**](#define-bsp_sdspi_host)  (SPI3\_HOST)<br> |
| define  | [**BSP\_SD\_CLK**](#define-bsp_sd_clk)  (GPIO\_NUM\_39)<br> |
| define  | [**BSP\_SD\_CMD**](#define-bsp_sd_cmd)  (GPIO\_NUM\_38)<br> |
| define  | [**BSP\_SD\_D0**](#define-bsp_sd_d0)  (GPIO\_NUM\_40)<br> |
| define  | [**BSP\_SD\_MOUNT\_POINT**](#define-bsp_sd_mount_point)  CONFIG\_BSP\_SD\_MOUNT\_POINT<br> |
| define  | [**BSP\_SPIFFS\_MOUNT\_POINT**](#define-bsp_spiffs_mount_point)  CONFIG\_BSP\_SPIFFS\_MOUNT\_POINT<br> |

## Structures and Types Documentation

### enum `bsp_button_t`

```c
enum bsp_button_t {
    BSP_BUTTON_MENU = 0,
    BSP_BUTTON_PLAY,
    BSP_BUTTON_DOWN,
    BSP_BUTTON_UP,
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
    BSP_LED_GREEN = GPIO_NUM_3
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

### function `bsp_adc_get_handle`

_Get ADC handle._
```c
adc_oneshot_unit_handle_t bsp_adc_get_handle (
    void
) 
```


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
### function `bsp_audio_codec_microphone_init`

_Initialize microphone codec device._
```c
esp_codec_dev_handle_t bsp_audio_codec_microphone_init (
    void
) 
```


**Returns:**

Pointer to codec device handle or NULL when error occurred
### function `bsp_audio_get_codec_itf`

_Get codec I2S interface (initialized in bsp\_audio\_init)_
```c
const audio_codec_data_if_t * bsp_audio_get_codec_itf (
    void
) 
```


**Returns:**



* Pointer to codec I2S interface handle or NULL when error occurred
### function `bsp_audio_init`

_Init audio._
```c
esp_err_t bsp_audio_init (
    const i2s_std_config_t *i2s_config
) 
```


**Note:**

There is no deinit audio function. Users can free audio resources by calling i2s\_del\_channel() 



**Warning:**

The type of i2s\_config param is depending on IDF version. 



**Parameters:**


* `i2s_config` I2S configuration. Pass NULL to use default values (Mono, duplex, 16bit, 22050 Hz) 


**Returns:**



* ESP\_OK On success
* ESP\_ERR\_NOT\_SUPPORTED The communication mode is not supported on the current chip
* ESP\_ERR\_INVALID\_ARG NULL pointer or invalid configuration
* ESP\_ERR\_NOT\_FOUND No available I2S channel found
* ESP\_ERR\_NO\_MEM No memory for storing the channel information
* ESP\_ERR\_INVALID\_STATE This channel has not initialized or already started
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

### function `bsp_i2c_deinit`

_Deinit I2C driver and free its resources._
```c
esp_err_t bsp_i2c_deinit (
    void
) 
```


**Returns:**



* ESP\_OK On success
* ESP\_ERR\_INVALID\_ARG I2C parameter error
### function `bsp_i2c_get_handle`

_Get I2C driver handle._
```c
i2c_master_bus_handle_t bsp_i2c_get_handle (
    void
) 
```


**Returns:**



* I2C handle
### function `bsp_i2c_init`

_Init I2C driver._
```c
esp_err_t bsp_i2c_init (
    void
) 
```


**Returns:**



* ESP\_OK On success
* ESP\_ERR\_INVALID\_ARG I2C parameter error
* ESP\_FAIL I2C driver installation error
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
* ESP\_ERR\_INVALID\_STATE if already unmounted

## Macros Documentation

### define `BSP_ADC_UNIT`

```c
#define BSP_ADC_UNIT ADC_UNIT_1
```

### define `BSP_BOARD_ESP32_S3_EYE`

```c
#define BSP_BOARD_ESP32_S3_EYE 
```

### define `BSP_BUTTONS_IO`

```c
#define BSP_BUTTONS_IO (GPIO_NUM_1)
```

### define `BSP_BUTTON_BOOT_IO`

```c
#define BSP_BUTTON_BOOT_IO (GPIO_NUM_0)
```

### define `BSP_CAMERA_D0`

```c
#define BSP_CAMERA_D0 (GPIO_NUM_11)
```

### define `BSP_CAMERA_D1`

```c
#define BSP_CAMERA_D1 (GPIO_NUM_9)
```

### define `BSP_CAMERA_D2`

```c
#define BSP_CAMERA_D2 (GPIO_NUM_8)
```

### define `BSP_CAMERA_D3`

```c
#define BSP_CAMERA_D3 (GPIO_NUM_10)
```

### define `BSP_CAMERA_D4`

```c
#define BSP_CAMERA_D4 (GPIO_NUM_12)
```

### define `BSP_CAMERA_D5`

```c
#define BSP_CAMERA_D5 (GPIO_NUM_18)
```

### define `BSP_CAMERA_D6`

```c
#define BSP_CAMERA_D6 (GPIO_NUM_17)
```

### define `BSP_CAMERA_D7`

```c
#define BSP_CAMERA_D7 (GPIO_NUM_16)
```

### define `BSP_CAMERA_DEFAULT_CONFIG`

_ESP32-S3-EYE camera default configuration._
```c
#define BSP_CAMERA_DEFAULT_CONFIG {                                     \
        .pin_pwdn = GPIO_NUM_NC,          \
        .pin_reset = GPIO_NUM_NC,         \
        .pin_xclk = BSP_CAMERA_XCLK,      \
        .pin_sccb_sda = GPIO_NUM_NC,      \
        .pin_sccb_scl = GPIO_NUM_NC,      \
        .pin_d7 = BSP_CAMERA_D7,          \
        .pin_d6 = BSP_CAMERA_D6,          \
        .pin_d5 = BSP_CAMERA_D5,          \
        .pin_d4 = BSP_CAMERA_D4,          \
        .pin_d3 = BSP_CAMERA_D3,          \
        .pin_d2 = BSP_CAMERA_D2,          \
        .pin_d1 = BSP_CAMERA_D1,          \
        .pin_d0 = BSP_CAMERA_D0,          \
        .pin_vsync = BSP_CAMERA_VSYNC,    \
        .pin_href = BSP_CAMERA_HSYNC,     \
        .pin_pclk = BSP_CAMERA_PCLK,      \
        .xclk_freq_hz = 16000000,         \
        .ledc_timer = LEDC_TIMER_0,       \
        .ledc_channel = LEDC_CHANNEL_0,   \
        .pixel_format = PIXFORMAT_RGB565, \
        .frame_size = FRAMESIZE_240X240,  \
        .jpeg_quality = 12,               \
        .fb_count = 2,                    \
        .fb_location = CAMERA_FB_IN_PSRAM,\
        .sccb_i2c_port = BSP_I2C_NUM,     \
    }
```


In this configuration we select RGB565 color format and 240x240 image size - matching the display. We use double-buffering for the best performance. Since we don't want to waste internal SRAM, we allocate the framebuffers in external PSRAM. By setting XCLK to 16MHz, we configure the esp32-camera driver to use EDMA when accessing the PSRAM.



**Attention:**

I2C must be enabled by [**bsp\_i2c\_init()**](#function-bsp_i2c_init), before camera is initialized
### define `BSP_CAMERA_HMIRROR`

```c
#define BSP_CAMERA_HMIRROR 0
```

### define `BSP_CAMERA_HSYNC`

```c
#define BSP_CAMERA_HSYNC (GPIO_NUM_7)
```

### define `BSP_CAMERA_PCLK`

```c
#define BSP_CAMERA_PCLK (GPIO_NUM_13)
```

### define `BSP_CAMERA_VFLIP`

```c
#define BSP_CAMERA_VFLIP 1
```

### define `BSP_CAMERA_VSYNC`

```c
#define BSP_CAMERA_VSYNC (GPIO_NUM_6)
```

### define `BSP_CAMERA_XCLK`

```c
#define BSP_CAMERA_XCLK (GPIO_NUM_15)
```

### define `BSP_CAPS_AUDIO`

```c
#define BSP_CAPS_AUDIO 1
```

### define `BSP_CAPS_AUDIO_MIC`

```c
#define BSP_CAPS_AUDIO_MIC 1
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
#define BSP_CAPS_CAMERA 1
```

### define `BSP_CAPS_DISPLAY`

```c
#define BSP_CAPS_DISPLAY 1
```

### define `BSP_CAPS_IMU`

```c
#define BSP_CAPS_IMU 1
```

### define `BSP_CAPS_SDCARD`

```c
#define BSP_CAPS_SDCARD 1
```

### define `BSP_CAPS_TOUCH`

```c
#define BSP_CAPS_TOUCH 0
```

### define `BSP_I2C_NUM`

```c
#define BSP_I2C_NUM CONFIG_BSP_I2C_NUM
```

### define `BSP_I2C_SCL`

```c
#define BSP_I2C_SCL (GPIO_NUM_5)
```

### define `BSP_I2C_SDA`

```c
#define BSP_I2C_SDA (GPIO_NUM_4)
```

### define `BSP_I2S_DIN`

```c
#define BSP_I2S_DIN (GPIO_NUM_2)
```

### define `BSP_I2S_LCLK`

```c
#define BSP_I2S_LCLK (GPIO_NUM_42)
```

### define `BSP_I2S_SCLK`

```c
#define BSP_I2S_SCLK (GPIO_NUM_41)
```

### define `BSP_LCD_BACKLIGHT`

```c
#define BSP_LCD_BACKLIGHT (GPIO_NUM_48)
```

### define `BSP_LCD_DC`

```c
#define BSP_LCD_DC (GPIO_NUM_43)
```

### define `BSP_LCD_DRAW_BUFF_DOUBLE`

```c
#define BSP_LCD_DRAW_BUFF_DOUBLE (0)
```

### define `BSP_LCD_DRAW_BUFF_SIZE`

```c
#define BSP_LCD_DRAW_BUFF_SIZE (BSP_LCD_H_RES * BSP_LCD_V_RES)
```

### define `BSP_LCD_PIXEL_CLOCK_HZ`

```c
#define BSP_LCD_PIXEL_CLOCK_HZ (80 * 1000 * 1000)
```

### define `BSP_LCD_RST`

```c
#define BSP_LCD_RST (GPIO_NUM_NC)
```

### define `BSP_LCD_SPI_CLK`

```c
#define BSP_LCD_SPI_CLK (GPIO_NUM_21)
```

### define `BSP_LCD_SPI_CS`

```c
#define BSP_LCD_SPI_CS (GPIO_NUM_44)
```

### define `BSP_LCD_SPI_MOSI`

```c
#define BSP_LCD_SPI_MOSI (GPIO_NUM_47)
```

### define `BSP_LCD_SPI_NUM`

```c
#define BSP_LCD_SPI_NUM (SPI3_HOST)
```

### define `BSP_SDSPI_HOST`

```c
#define BSP_SDSPI_HOST (SPI3_HOST)
```

### define `BSP_SD_CLK`

```c
#define BSP_SD_CLK (GPIO_NUM_39)
```

### define `BSP_SD_CMD`

```c
#define BSP_SD_CMD (GPIO_NUM_38)
```

### define `BSP_SD_D0`

```c
#define BSP_SD_D0 (GPIO_NUM_40)
```

### define `BSP_SD_MOUNT_POINT`

```c
#define BSP_SD_MOUNT_POINT CONFIG_BSP_SD_MOUNT_POINT
```

### define `BSP_SPIFFS_MOUNT_POINT`

```c
#define BSP_SPIFFS_MOUNT_POINT CONFIG_BSP_SPIFFS_MOUNT_POINT
```


