# API Reference

## Header files

- [bsp/esp32_s2_kaluga_kit/include/bsp/display.h](#file-bspesp32_s2_kaluga_kitincludebspdisplayh)
- [bsp/esp32_s2_kaluga_kit/include/bsp/esp32_s2_kaluga_kit.h](#file-bspesp32_s2_kaluga_kitincludebspesp32_s2_kaluga_kith)

## File bsp/esp32_s2_kaluga_kit/include/bsp/display.h

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
|  esp\_err\_t | [**bsp\_display\_backlight\_off**](#function-bsp_display_backlight_off) (void) <br> |
|  esp\_err\_t | [**bsp\_display\_backlight\_on**](#function-bsp_display_backlight_on) (void) <br> |
|  esp\_err\_t | [**bsp\_display\_brightness\_init**](#function-bsp_display_brightness_init) (void) <br> |
|  esp\_err\_t | [**bsp\_display\_brightness\_set**](#function-bsp_display_brightness_set) (int brightness\_percent) <br> |
|  esp\_err\_t | [**bsp\_display\_new**](#function-bsp_display_new) (const [**bsp\_display\_config\_t**](#struct-bsp_display_config_t) \*config, esp\_lcd\_panel\_handle\_t \*ret\_panel, esp\_lcd\_panel\_io\_handle\_t \*ret\_io) <br>_Create new display panel._ |

## Macros

| Type | Name |
| ---: | :--- |
| define  | [**BSP\_LCD\_BIGENDIAN**](#define-bsp_lcd_bigendian)  (1)<br> |
| define  | [**BSP\_LCD\_BITS\_PER\_PIXEL**](#define-bsp_lcd_bits_per_pixel)  (16)<br> |
| define  | [**BSP\_LCD\_COLOR\_FORMAT**](#define-bsp_lcd_color_format)  (ESP\_LCD\_COLOR\_FORMAT\_RGB565)<br> |
| define  | [**BSP\_LCD\_COLOR\_SPACE**](#define-bsp_lcd_color_space)  (ESP\_LCD\_COLOR\_SPACE\_RGB)<br> |
| define  | [**BSP\_LCD\_H\_RES**](#define-bsp_lcd_h_res)  (320)<br> |
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

```c
esp_err_t bsp_display_backlight_off (
    void
) 
```

### function `bsp_display_backlight_on`

```c
esp_err_t bsp_display_backlight_on (
    void
) 
```

### function `bsp_display_brightness_init`

```c
esp_err_t bsp_display_brightness_init (
    void
) 
```

### function `bsp_display_brightness_set`

```c
esp_err_t bsp_display_brightness_set (
    int brightness_percent
) 
```

### function `bsp_display_new`

_Create new display panel._
```c
esp_err_t bsp_display_new (
    const bsp_display_config_t *config,
    esp_lcd_panel_handle_t *ret_panel,
    esp_lcd_panel_io_handle_t *ret_io
) 
```


For maximum flexibility, this function performs only reset and initialization of the display. You must turn on the display explicitly by calling esp\_lcd\_panel\_disp\_on\_off(). The display's backlight is not turned on either. You can use bsp\_display\_backlight\_on/off(), bsp\_display\_brightness\_set() (on supported boards) or implement your own backlight control.

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
#define BSP_LCD_H_RES (320)
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


## File bsp/esp32_s2_kaluga_kit/include/bsp/esp32_s2_kaluga_kit.h

_ESP BSP: ESP32-S2-Kaluga Kit._



## Structures and Types

| Type | Name |
| ---: | :--- |
| enum  | [**bsp\_button\_t**](#enum-bsp_button_t)  <br> |
| struct | [**bsp\_display\_cfg\_t**](#struct-bsp_display_cfg_t) <br>_BSP display configuration structure._ |
| enum  | [**bsp\_touchpad\_button\_t**](#enum-bsp_touchpad_button_t)  <br> |

## Functions

| Type | Name |
| ---: | :--- |
|  adc\_oneshot\_unit\_handle\_t | [**bsp\_adc\_get\_handle**](#function-bsp_adc_get_handle) (void) <br>_Get ADC handle._ |
|  esp\_err\_t | [**bsp\_adc\_initialize**](#function-bsp_adc_initialize) (void) <br>_Initialize ADC._ |
|  esp\_codec\_dev\_handle\_t | [**bsp\_audio\_codec\_microphone\_init**](#function-bsp_audio_codec_microphone_init) (void) <br>_Initialize microphone codec device._ |
|  esp\_codec\_dev\_handle\_t | [**bsp\_audio\_codec\_speaker\_init**](#function-bsp_audio_codec_speaker_init) (void) <br>_Initialize speaker codec device._ |
|  const audio\_codec\_data\_if\_t \* | [**bsp\_audio\_get\_codec\_itf**](#function-bsp_audio_get_codec_itf) (void) <br>_Get codec I2S interface (initialized in bsp\_audio\_init)_ |
|  esp\_err\_t | [**bsp\_audio\_init**](#function-bsp_audio_init) (const i2s\_std\_config\_t \*i2s\_config) <br>_Init audio._ |
|  lv\_indev\_t \* | [**bsp\_display\_get\_input\_dev**](#function-bsp_display_get_input_dev) (void) <br>_Get pointer to input device (touch, buttons, ...)_ |
|  bool | [**bsp\_display\_lock**](#function-bsp_display_lock) (uint32\_t timeout\_ms) <br>_Take LVGL mutex._ |
|  void | [**bsp\_display\_rotate**](#function-bsp_display_rotate) (lv\_display\_t \*disp, lv\_disp\_rotation\_t rotation) <br>_Rotate screen._ |
|  lv\_display\_t \* | [**bsp\_display\_start**](#function-bsp_display_start) (void) <br>_Initialize display and graphics library._ |
|  lv\_display\_t \* | [**bsp\_display\_start\_with\_config**](#function-bsp_display_start_with_config) (const [**bsp\_display\_cfg\_t**](#struct-bsp_display_cfg_t) \*cfg) <br>_Initialize display._ |
|  void | [**bsp\_display\_unlock**](#function-bsp_display_unlock) (void) <br>_Give LVGL mutex._ |
|  esp\_err\_t | [**bsp\_i2c\_deinit**](#function-bsp_i2c_deinit) (void) <br>_Deinit I2C driver and free its resources._ |
|  i2c\_master\_bus\_handle\_t | [**bsp\_i2c\_get\_handle**](#function-bsp_i2c_get_handle) (void) <br>_Get I2C driver handle._ |
|  esp\_err\_t | [**bsp\_i2c\_init**](#function-bsp_i2c_init) (void) <br>_Init I2C driver._ |
|  esp\_err\_t | [**bsp\_iot\_button\_create**](#function-bsp_iot_button_create) (button\_handle\_t btn\_array, int \*btn\_cnt, int btn\_array\_size) <br>_Initialize all buttons._ |
|  esp\_err\_t | [**bsp\_spiffs\_mount**](#function-bsp_spiffs_mount) (void) <br>_Mount SPIFFS to virtual file system._ |
|  esp\_err\_t | [**bsp\_spiffs\_unmount**](#function-bsp_spiffs_unmount) (void) <br>_Unmount SPIFFS from virtual file system._ |
|  esp\_err\_t | [**bsp\_touchpad\_calibrate**](#function-bsp_touchpad_calibrate) (bsp\_touchpad\_button\_t tch\_pad, float tch\_threshold) <br>_Calibrate touch threshold._ |
|  esp\_err\_t | [**bsp\_touchpad\_deinit**](#function-bsp_touchpad_deinit) (void) <br>_Deinit buttons on Touchpad board._ |
|  esp\_err\_t | [**bsp\_touchpad\_init**](#function-bsp_touchpad_init) (intr\_handler\_t fn) <br>_Init buttons on Touchpad board._ |

## Macros

| Type | Name |
| ---: | :--- |
| define  | [**BSP\_ADC\_UNIT**](#define-bsp_adc_unit)  ADC\_UNIT\_1<br> |
| define  | [**BSP\_BOARD\_ESP32\_S2\_KALUGA\_KIT**](#define-bsp_board_esp32_s2_kaluga_kit)  <br> |
| define  | [**BSP\_BUTTONS\_IO**](#define-bsp_buttons_io)  (GPIO\_NUM\_6)<br> |
| define  | [**BSP\_CAMERA\_D0**](#define-bsp_camera_d0)  (GPIO\_NUM\_36)<br> |
| define  | [**BSP\_CAMERA\_D1**](#define-bsp_camera_d1)  (GPIO\_NUM\_37)<br> |
| define  | [**BSP\_CAMERA\_D2**](#define-bsp_camera_d2)  (GPIO\_NUM\_41)<br> |
| define  | [**BSP\_CAMERA\_D3**](#define-bsp_camera_d3)  (GPIO\_NUM\_42)<br> |
| define  | [**BSP\_CAMERA\_D4**](#define-bsp_camera_d4)  (GPIO\_NUM\_39)<br> |
| define  | [**BSP\_CAMERA\_D5**](#define-bsp_camera_d5)  (GPIO\_NUM\_40)<br> |
| define  | [**BSP\_CAMERA\_D6**](#define-bsp_camera_d6)  (GPIO\_NUM\_21)<br> |
| define  | [**BSP\_CAMERA\_D7**](#define-bsp_camera_d7)  (GPIO\_NUM\_38)<br> |
| define  | [**BSP\_CAMERA\_DEFAULT\_CONFIG**](#define-bsp_camera_default_config)  <br>_Kaluga camera default configuration._ |
| define  | [**BSP\_CAMERA\_HMIRROR**](#define-bsp_camera_hmirror)  0<br> |
| define  | [**BSP\_CAMERA\_HSYNC**](#define-bsp_camera_hsync)  (GPIO\_NUM\_3)<br> |
| define  | [**BSP\_CAMERA\_PCLK**](#define-bsp_camera_pclk)  (GPIO\_NUM\_33)<br> |
| define  | [**BSP\_CAMERA\_VFLIP**](#define-bsp_camera_vflip)  1<br> |
| define  | [**BSP\_CAMERA\_VSYNC**](#define-bsp_camera_vsync)  (GPIO\_NUM\_2)<br> |
| define  | [**BSP\_CAMERA\_XCLK**](#define-bsp_camera_xclk)  (GPIO\_NUM\_1)<br> |
| define  | [**BSP\_CAPS\_AUDIO**](#define-bsp_caps_audio)  1<br> |
| define  | [**BSP\_CAPS\_AUDIO\_MIC**](#define-bsp_caps_audio_mic)  1<br> |
| define  | [**BSP\_CAPS\_AUDIO\_SPEAKER**](#define-bsp_caps_audio_speaker)  1<br> |
| define  | [**BSP\_CAPS\_BUTTONS**](#define-bsp_caps_buttons)  1<br> |
| define  | [**BSP\_CAPS\_CAMERA**](#define-bsp_caps_camera)  1<br> |
| define  | [**BSP\_CAPS\_DISPLAY**](#define-bsp_caps_display)  1<br> |
| define  | [**BSP\_CAPS\_IMU**](#define-bsp_caps_imu)  0<br> |
| define  | [**BSP\_CAPS\_LED**](#define-bsp_caps_led)  1<br> |
| define  | [**BSP\_CAPS\_SDCARD**](#define-bsp_caps_sdcard)  0<br> |
| define  | [**BSP\_CAPS\_TOUCH**](#define-bsp_caps_touch)  0<br> |
| define  | [**BSP\_I2C\_NUM**](#define-bsp_i2c_num)  CONFIG\_BSP\_I2C\_NUM<br> |
| define  | [**BSP\_I2C\_SCL**](#define-bsp_i2c_scl)  (GPIO\_NUM\_7)<br> |
| define  | [**BSP\_I2C\_SDA**](#define-bsp_i2c_sda)  (GPIO\_NUM\_8)<br> |
| define  | [**BSP\_I2S\_DOUT**](#define-bsp_i2s_dout)  (GPIO\_NUM\_12)<br> |
| define  | [**BSP\_I2S\_DSIN**](#define-bsp_i2s_dsin)  (GPIO\_NUM\_34)<br> |
| define  | [**BSP\_I2S\_LCLK**](#define-bsp_i2s_lclk)  (GPIO\_NUM\_17)<br> |
| define  | [**BSP\_I2S\_MCLK**](#define-bsp_i2s_mclk)  (GPIO\_NUM\_35)<br> |
| define  | [**BSP\_I2S\_SCLK**](#define-bsp_i2s_sclk)  (GPIO\_NUM\_18)<br> |
| define  | [**BSP\_LCD\_DC**](#define-bsp_lcd_dc)  (GPIO\_NUM\_13)<br> |
| define  | [**BSP\_LCD\_DRAW\_BUFF\_DOUBLE**](#define-bsp_lcd_draw_buff_double)  (0)<br> |
| define  | [**BSP\_LCD\_DRAW\_BUFF\_SIZE**](#define-bsp_lcd_draw_buff_size)  (BSP\_LCD\_H\_RES \* 20)<br> |
| define  | [**BSP\_LCD\_PIXEL\_CLOCK\_HZ**](#define-bsp_lcd_pixel_clock_hz)  (40 \* 1000 \* 1000)<br> |
| define  | [**BSP\_LCD\_RST**](#define-bsp_lcd_rst)  (GPIO\_NUM\_16)<br> |
| define  | [**BSP\_LCD\_SPI\_CLK**](#define-bsp_lcd_spi_clk)  (GPIO\_NUM\_15)<br> |
| define  | [**BSP\_LCD\_SPI\_CS**](#define-bsp_lcd_spi_cs)  (GPIO\_NUM\_11)<br> |
| define  | [**BSP\_LCD\_SPI\_MOSI**](#define-bsp_lcd_spi_mosi)  (GPIO\_NUM\_9)<br> |
| define  | [**BSP\_LCD\_SPI\_NUM**](#define-bsp_lcd_spi_num)  (SPI3\_HOST)<br> |
| define  | [**BSP\_LEDSTRIP\_IO**](#define-bsp_ledstrip_io)  (GPIO\_NUM\_45)<br> |
| define  | [**BSP\_POWER\_AMP\_IO**](#define-bsp_power_amp_io)  (GPIO\_NUM\_10)<br> |
| define  | [**BSP\_SPIFFS\_MOUNT\_POINT**](#define-bsp_spiffs_mount_point)  CONFIG\_BSP\_SPIFFS\_MOUNT\_POINT<br> |
| define  | [**BSP\_TOUCH\_BUTTON\_GUARD**](#define-bsp_touch_button_guard)  (GPIO\_NUM\_4)<br> |
| define  | [**BSP\_TOUCH\_BUTTON\_NETWORK**](#define-bsp_touch_button_network)  (GPIO\_NUM\_11)<br> |
| define  | [**BSP\_TOUCH\_BUTTON\_PHOTO**](#define-bsp_touch_button_photo)  (GPIO\_NUM\_6)<br> |
| define  | [**BSP\_TOUCH\_BUTTON\_PLAY**](#define-bsp_touch_button_play)  (GPIO\_NUM\_2)<br> |
| define  | [**BSP\_TOUCH\_BUTTON\_RECORD**](#define-bsp_touch_button_record)  (GPIO\_NUM\_5)<br> |
| define  | [**BSP\_TOUCH\_BUTTON\_VOLDOWN**](#define-bsp_touch_button_voldown)  (GPIO\_NUM\_3)<br> |
| define  | [**BSP\_TOUCH\_BUTTON\_VOLUP**](#define-bsp_touch_button_volup)  (GPIO\_NUM\_1)<br> |
| define  | [**BSP\_TOUCH\_SHELED\_ELECT**](#define-bsp_touch_sheled_elect)  (GPIO\_NUM\_14)<br> |

## Structures and Types Documentation

### enum `bsp_button_t`

```c
enum bsp_button_t {
    BSP_BUTTON_REC = 0,
    BSP_BUTTON_MODE,
    BSP_BUTTON_PLAY,
    BSP_BUTTON_SET,
    BSP_BUTTON_VOLDOWN,
    BSP_BUTTON_VOLUP,
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

### enum `bsp_touchpad_button_t`

```c
enum bsp_touchpad_button_t {
    TOUCH_BUTTON_PLAY = TOUCH_PAD_NUM2,
    TOUCH_BUTTON_PHOTO = TOUCH_PAD_NUM6,
    TOUCH_BUTTON_NETWORK = TOUCH_PAD_NUM11,
    TOUCH_BUTTON_RECORD = TOUCH_PAD_NUM5,
    TOUCH_BUTTON_VOLUP = TOUCH_PAD_NUM1,
    TOUCH_BUTTON_VOLDOWN = TOUCH_PAD_NUM3,
    TOUCH_BUTTON_GUARD = TOUCH_PAD_NUM4,
    TOUCH_SHELED_ELECT = TOUCH_PAD_NUM14,
    TOUCH_BUTTON_NUM = 7
};
```


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
### function `bsp_audio_codec_microphone_init`

_Initialize microphone codec device._
```c
esp_codec_dev_handle_t bsp_audio_codec_microphone_init (
    void
) 
```


**Returns:**

Pointer to codec device handle or NULL when error occurred
### function `bsp_audio_codec_speaker_init`

_Initialize speaker codec device._
```c
esp_codec_dev_handle_t bsp_audio_codec_speaker_init (
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

_Initialize display and graphics library._
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


This function initializes SPI, display controller and starts LVGL handling task.



**Parameters:**


* `cfg` Display configuration


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
### function `bsp_touchpad_calibrate`

_Calibrate touch threshold._
```c
esp_err_t bsp_touchpad_calibrate (
    bsp_touchpad_button_t tch_pad,
    float tch_threshold
) 
```


**Parameters:**


* `tch_pad` Touch pad from bsp\_touchpad\_button\_t enum. 
* `tch_threshold` Interrupt threshold ratio. Min.: 0, max.: 1. 


**Returns:**



* ESP\_OK On success
* ESP\_ERR\_INVALID\_ARG NULL pointer or invalid configuration
### function `bsp_touchpad_deinit`

_Deinit buttons on Touchpad board._
```c
esp_err_t bsp_touchpad_deinit (
    void
) 
```

### function `bsp_touchpad_init`

_Init buttons on Touchpad board._
```c
esp_err_t bsp_touchpad_init (
    intr_handler_t fn
) 
```


**Attention:**

This function initializes all touchpad buttons, including 'Photo' and 'Network' buttons, which have conflicts with Audio board buttons and LCD respectively. 



**Parameters:**


* `fn` Interrupt callback for touchpad peripheral. 


**Returns:**



* ESP\_OK On success
* ESP\_ERR\_INVALID\_ARG NULL pointer or invalid configuration
* ESP\_FAIL Touch pad not initialized
* ESP\_ERR\_NO\_MEM No memory

## Macros Documentation

### define `BSP_ADC_UNIT`

```c
#define BSP_ADC_UNIT ADC_UNIT_1
```

### define `BSP_BOARD_ESP32_S2_KALUGA_KIT`

```c
#define BSP_BOARD_ESP32_S2_KALUGA_KIT 
```

### define `BSP_BUTTONS_IO`

```c
#define BSP_BUTTONS_IO (GPIO_NUM_6)
```

### define `BSP_CAMERA_D0`

```c
#define BSP_CAMERA_D0 (GPIO_NUM_36)
```


Labeled as: D2
### define `BSP_CAMERA_D1`

```c
#define BSP_CAMERA_D1 (GPIO_NUM_37)
```


Labeled as: D3
### define `BSP_CAMERA_D2`

```c
#define BSP_CAMERA_D2 (GPIO_NUM_41)
```


Labeled as: D4
### define `BSP_CAMERA_D3`

```c
#define BSP_CAMERA_D3 (GPIO_NUM_42)
```


Labeled as: D5
### define `BSP_CAMERA_D4`

```c
#define BSP_CAMERA_D4 (GPIO_NUM_39)
```


Labeled as: D6
### define `BSP_CAMERA_D5`

```c
#define BSP_CAMERA_D5 (GPIO_NUM_40)
```


Labeled as: D7
### define `BSP_CAMERA_D6`

```c
#define BSP_CAMERA_D6 (GPIO_NUM_21)
```


Labeled as: D8
### define `BSP_CAMERA_D7`

```c
#define BSP_CAMERA_D7 (GPIO_NUM_38)
```


Labeled as: D9
### define `BSP_CAMERA_DEFAULT_CONFIG`

_Kaluga camera default configuration._
```c
#define BSP_CAMERA_DEFAULT_CONFIG {                                     \
        .pin_pwdn = GPIO_NUM_NC,          \
        .pin_reset = GPIO_NUM_NC,         \
        .pin_xclk = BSP_CAMERA_XCLK,      \
        .pin_sccb_sda = GPIO_NUM_NC,      \
        .pin_sccb_scl = GPIO_NUM_NC,      \
        .pin_d7 = BSP_CAMERA_D7 ,          \
        .pin_d6 = BSP_CAMERA_D6 ,          \
        .pin_d5 = BSP_CAMERA_D5 ,          \
        .pin_d4 = BSP_CAMERA_D4 ,          \
        .pin_d3 = BSP_CAMERA_D3 ,          \
        .pin_d2 = BSP_CAMERA_D2 ,          \
        .pin_d1 = BSP_CAMERA_D1 ,          \
        .pin_d0 = BSP_CAMERA_D0 ,          \
        .pin_vsync = BSP_CAMERA_VSYNC,    \
        .pin_href = BSP_CAMERA_HSYNC,     \
        .pin_pclk = BSP_CAMERA_PCLK,      \
        .xclk_freq_hz = 16000000,         \
        .ledc_timer = LEDC_TIMER_0,       \
        .ledc_channel = LEDC_CHANNEL_0,   \
        .pixel_format = PIXFORMAT_RGB565, \
        .frame_size = FRAMESIZE_QVGA,     \
        .jpeg_quality = 12,               \
        .fb_count = 2,                    \
        .fb_location = CAMERA_FB_IN_PSRAM,\
        .sccb_i2c_port = BSP_I2C_NUM,     \
    }
```


In this configuration we select RGB565 color format and 320x240 image size - matching the display. We use double-buffering for the best performance. Since ESP32-S2 has only 320kB of internal SRAM, we allocate the framebuffers in external PSRAM. By setting XCLK to 16MHz, we configure the esp32-camera driver to use EDMA when accessing the PSRAM.



**Attention:**

I2C must be enabled by [**bsp\_i2c\_init()**](#function-bsp_i2c_init), before camera is initialized
### define `BSP_CAMERA_HMIRROR`

```c
#define BSP_CAMERA_HMIRROR 0
```

### define `BSP_CAMERA_HSYNC`

```c
#define BSP_CAMERA_HSYNC (GPIO_NUM_3)
```

### define `BSP_CAMERA_PCLK`

```c
#define BSP_CAMERA_PCLK (GPIO_NUM_33)
```

### define `BSP_CAMERA_VFLIP`

```c
#define BSP_CAMERA_VFLIP 1
```

### define `BSP_CAMERA_VSYNC`

```c
#define BSP_CAMERA_VSYNC (GPIO_NUM_2)
```

### define `BSP_CAMERA_XCLK`

```c
#define BSP_CAMERA_XCLK (GPIO_NUM_1)
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
#define BSP_CAPS_AUDIO_SPEAKER 1
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
#define BSP_CAPS_IMU 0
```

### define `BSP_CAPS_LED`

```c
#define BSP_CAPS_LED 1
```

### define `BSP_CAPS_SDCARD`

```c
#define BSP_CAPS_SDCARD 0
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
#define BSP_I2C_SCL (GPIO_NUM_7)
```

### define `BSP_I2C_SDA`

```c
#define BSP_I2C_SDA (GPIO_NUM_8)
```

### define `BSP_I2S_DOUT`

```c
#define BSP_I2S_DOUT (GPIO_NUM_12)
```

### define `BSP_I2S_DSIN`

```c
#define BSP_I2S_DSIN (GPIO_NUM_34)
```

### define `BSP_I2S_LCLK`

```c
#define BSP_I2S_LCLK (GPIO_NUM_17)
```

### define `BSP_I2S_MCLK`

```c
#define BSP_I2S_MCLK (GPIO_NUM_35)
```

### define `BSP_I2S_SCLK`

```c
#define BSP_I2S_SCLK (GPIO_NUM_18)
```

### define `BSP_LCD_DC`

```c
#define BSP_LCD_DC (GPIO_NUM_13)
```

### define `BSP_LCD_DRAW_BUFF_DOUBLE`

```c
#define BSP_LCD_DRAW_BUFF_DOUBLE (0)
```

### define `BSP_LCD_DRAW_BUFF_SIZE`

```c
#define BSP_LCD_DRAW_BUFF_SIZE (BSP_LCD_H_RES * 20)
```

### define `BSP_LCD_PIXEL_CLOCK_HZ`

```c
#define BSP_LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)
```

### define `BSP_LCD_RST`

```c
#define BSP_LCD_RST (GPIO_NUM_16)
```

### define `BSP_LCD_SPI_CLK`

```c
#define BSP_LCD_SPI_CLK (GPIO_NUM_15)
```

### define `BSP_LCD_SPI_CS`

```c
#define BSP_LCD_SPI_CS (GPIO_NUM_11)
```

### define `BSP_LCD_SPI_MOSI`

```c
#define BSP_LCD_SPI_MOSI (GPIO_NUM_9)
```

### define `BSP_LCD_SPI_NUM`

```c
#define BSP_LCD_SPI_NUM (SPI3_HOST)
```

### define `BSP_LEDSTRIP_IO`

```c
#define BSP_LEDSTRIP_IO (GPIO_NUM_45)
```

### define `BSP_POWER_AMP_IO`

```c
#define BSP_POWER_AMP_IO (GPIO_NUM_10)
```

### define `BSP_SPIFFS_MOUNT_POINT`

```c
#define BSP_SPIFFS_MOUNT_POINT CONFIG_BSP_SPIFFS_MOUNT_POINT
```

### define `BSP_TOUCH_BUTTON_GUARD`

```c
#define BSP_TOUCH_BUTTON_GUARD (GPIO_NUM_4)
```

### define `BSP_TOUCH_BUTTON_NETWORK`

```c
#define BSP_TOUCH_BUTTON_NETWORK (GPIO_NUM_11)
```

### define `BSP_TOUCH_BUTTON_PHOTO`

```c
#define BSP_TOUCH_BUTTON_PHOTO (GPIO_NUM_6)
```

### define `BSP_TOUCH_BUTTON_PLAY`

```c
#define BSP_TOUCH_BUTTON_PLAY (GPIO_NUM_2)
```

### define `BSP_TOUCH_BUTTON_RECORD`

```c
#define BSP_TOUCH_BUTTON_RECORD (GPIO_NUM_5)
```

### define `BSP_TOUCH_BUTTON_VOLDOWN`

```c
#define BSP_TOUCH_BUTTON_VOLDOWN (GPIO_NUM_3)
```

### define `BSP_TOUCH_BUTTON_VOLUP`

```c
#define BSP_TOUCH_BUTTON_VOLUP (GPIO_NUM_1)
```

### define `BSP_TOUCH_SHELED_ELECT`

```c
#define BSP_TOUCH_SHELED_ELECT (GPIO_NUM_14)
```


