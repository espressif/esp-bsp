# API Reference

## Header files

- [bsp/esp32_s3_lcd_ev_board/include/bsp/display.h](#file-bspesp32_s3_lcd_ev_boardincludebspdisplayh)
- [bsp/esp32_s3_lcd_ev_board/include/bsp/esp32_s3_lcd_ev_board.h](#file-bspesp32_s3_lcd_ev_boardincludebspesp32_s3_lcd_ev_boardh)
- [bsp/esp32_s3_lcd_ev_board/include/bsp/touch.h](#file-bspesp32_s3_lcd_ev_boardincludebsptouchh)

## File bsp/esp32_s3_lcd_ev_board/include/bsp/display.h

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
|  esp\_err\_t | [**bsp\_display\_backlight\_off**](#function-bsp_display_backlight_off) (void) <br>_Turn off display backlight (Useless, just for compatibility)_ |
|  esp\_err\_t | [**bsp\_display\_backlight\_on**](#function-bsp_display_backlight_on) (void) <br>_Turn on display backlight (Useless, just for compatibility)_ |
|  esp\_err\_t | [**bsp\_display\_brightness\_init**](#function-bsp_display_brightness_init) (void) <br>_Initialize display's brightness (Useless, just for compatibility)_ |
|  esp\_err\_t | [**bsp\_display\_brightness\_set**](#function-bsp_display_brightness_set) (int brightness\_percent) <br>_Set display's brightness (Useless, just for compatibility)_ |
|  esp\_err\_t | [**bsp\_display\_new**](#function-bsp_display_new) (const [**bsp\_display\_config\_t**](#struct-bsp_display_config_t) \*config, esp\_lcd\_panel\_handle\_t \*ret\_panel, esp\_lcd\_panel\_io\_handle\_t \*ret\_io) <br>_Create new display panel._ |


## Structures and Types Documentation

### struct `bsp_display_config_t`

_BSP display configuration structure._

Variables:

-  int max_transfer_sz  <br>Maximum transfer size, in bytes.


## Functions Documentation

### function `bsp_display_backlight_off`

_Turn off display backlight (Useless, just for compatibility)_
```c
esp_err_t bsp_display_backlight_off (
    void
) 
```


**Returns:**



* ESP\_ERR\_NOT\_SUPPORTED: Always
### function `bsp_display_backlight_on`

_Turn on display backlight (Useless, just for compatibility)_
```c
esp_err_t bsp_display_backlight_on (
    void
) 
```


**Returns:**



* ESP\_ERR\_NOT\_SUPPORTED: Always
### function `bsp_display_brightness_init`

_Initialize display's brightness (Useless, just for compatibility)_
```c
esp_err_t bsp_display_brightness_init (
    void
) 
```


**Returns:**



* ESP\_ERR\_NOT\_SUPPORTED: Always
### function `bsp_display_brightness_set`

_Set display's brightness (Useless, just for compatibility)_
```c
esp_err_t bsp_display_brightness_set (
    int brightness_percent
) 
```


**Parameters:**


* `brightness_percent` Brightness in [%] 


**Returns:**



* ESP\_ERR\_NOT\_SUPPORTED: Always
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


* `config` display configuration. Set to NULL if not needed. 
* `ret_panel` esp\_lcd panel handle. Set to NULL if not needed. 
* `ret_io` esp\_lcd IO handle. Set to NULL if not needed. 


**Returns:**



* ESP\_OK On success
* Else esp\_lcd failure


## File bsp/esp32_s3_lcd_ev_board/include/bsp/esp32_s3_lcd_ev_board.h

_ESP BSP: ESP32-S3-LCD-EV-Board._



## Structures and Types

| Type | Name |
| ---: | :--- |
| enum  | [**bsp\_button\_t**](#enum-bsp_button_t)  <br> |
| struct | [**bsp\_display\_cfg\_t**](#struct-bsp_display_cfg_t) <br>_BSP display configuration structure._ |

## Functions

| Type | Name |
| ---: | :--- |
|  adc\_oneshot\_unit\_handle\_t | [**bsp\_adc\_get\_handle**](#function-bsp_adc_get_handle) (void) <br>_Get ADC handle._ |
|  esp\_err\_t | [**bsp\_adc\_initialize**](#function-bsp_adc_initialize) (void) <br>_Initialize ADC._ |
|  esp\_codec\_dev\_handle\_t | [**bsp\_audio\_codec\_microphone\_init**](#function-bsp_audio_codec_microphone_init) (void) <br>_Initialize microphone codec device._ |
|  esp\_codec\_dev\_handle\_t | [**bsp\_audio\_codec\_speaker\_init**](#function-bsp_audio_codec_speaker_init) (void) <br>_Initialize speaker codec device._ |
|  esp\_err\_t | [**bsp\_audio\_init**](#function-bsp_audio_init) (const i2s\_std\_config\_t \*i2s\_config) <br>_Init audio._ |
|  esp\_err\_t | [**bsp\_audio\_poweramp\_enable**](#function-bsp_audio_poweramp_enable) (bool enable) <br>_Enable/disable audio power amplifier (deprecated)_ |
|  bool | [**bsp\_button\_get**](#function-bsp_button_get) (const bsp\_button\_t btn) <br>_Get button's state._ |
|  esp\_err\_t | [**bsp\_button\_init**](#function-bsp_button_init) (const bsp\_button\_t btn) <br>_Set button's GPIO as input._ |
|  uint16\_t | [**bsp\_display\_get\_h\_res**](#function-bsp_display_get_h_res) (void) <br>_Get display horizontal resolution._ |
|  lv\_indev\_t \* | [**bsp\_display\_get\_input\_dev**](#function-bsp_display_get_input_dev) (void) <br>_Get pointer to input device (touch, buttons, ...)_ |
|  uint16\_t | [**bsp\_display\_get\_v\_res**](#function-bsp_display_get_v_res) (void) <br>_Get display vertical resolution._ |
|  bool | [**bsp\_display\_lock**](#function-bsp_display_lock) (uint32\_t timeout\_ms) <br>_Take LVGL mutex._ |
|  void | [**bsp\_display\_rotate**](#function-bsp_display_rotate) (lv\_display\_t \*disp, lv\_display\_rotation\_t rotation) <br>_Rotate screen._ |
|  lv\_display\_t \* | [**bsp\_display\_start**](#function-bsp_display_start) (void) <br>_Initialize display._ |
|  lv\_display\_t \* | [**bsp\_display\_start\_with\_config**](#function-bsp_display_start_with_config) (const [**bsp\_display\_cfg\_t**](#struct-bsp_display_cfg_t) \*cfg) <br>_Initialize display._ |
|  void | [**bsp\_display\_unlock**](#function-bsp_display_unlock) (void) <br>_Give LVGL mutex._ |
|  esp\_err\_t | [**bsp\_i2c\_deinit**](#function-bsp_i2c_deinit) (void) <br>_Deinit I2C driver and free its resources._ |
|  i2c\_master\_bus\_handle\_t | [**bsp\_i2c\_get\_handle**](#function-bsp_i2c_get_handle) (void) <br>_Get I2C driver handle._ |
|  esp\_err\_t | [**bsp\_i2c\_init**](#function-bsp_i2c_init) (void) <br>_Init I2C driver._ |
|  esp\_io\_expander\_handle\_t | [**bsp\_io\_expander\_init**](#function-bsp_io_expander_init) (void) <br>_Init IO expander chip TCA9554._ |
|  esp\_err\_t | [**bsp\_iot\_button\_create**](#function-bsp_iot_button_create) (button\_handle\_t btn\_array, int \*btn\_cnt, int btn\_array\_size) <br>_Initialize all buttons._ |
|  esp\_err\_t | [**bsp\_spiffs\_mount**](#function-bsp_spiffs_mount) (void) <br>_Mount SPIFFS to virtual file system._ |
|  esp\_err\_t | [**bsp\_spiffs\_unmount**](#function-bsp_spiffs_unmount) (void) <br>_Unmount SPIFFS from virtual file system._ |

## Macros

| Type | Name |
| ---: | :--- |
| define  | [**BSP\_ADC\_UNIT**](#define-bsp_adc_unit)  ADC\_UNIT\_1<br> |
| define  | [**BSP\_BOARD\_ESP32\_S3\_LCD\_EV\_BOARD**](#define-bsp_board_esp32_s3_lcd_ev_board)  <br> |
| define  | [**BSP\_BUTTON\_BOOT\_IO**](#define-bsp_button_boot_io)  (GPIO\_NUM\_0)<br> |
| define  | [**BSP\_CAPS\_AUDIO**](#define-bsp_caps_audio)  1<br> |
| define  | [**BSP\_CAPS\_AUDIO\_MIC**](#define-bsp_caps_audio_mic)  1<br> |
| define  | [**BSP\_CAPS\_AUDIO\_SPEAKER**](#define-bsp_caps_audio_speaker)  1<br> |
| define  | [**BSP\_CAPS\_BUTTONS**](#define-bsp_caps_buttons)  1<br> |
| define  | [**BSP\_CAPS\_DISPLAY**](#define-bsp_caps_display)  1<br> |
| define  | [**BSP\_CAPS\_IMU**](#define-bsp_caps_imu)  0<br> |
| define  | [**BSP\_CAPS\_SDCARD**](#define-bsp_caps_sdcard)  0<br> |
| define  | [**BSP\_CAPS\_TOUCH**](#define-bsp_caps_touch)  1<br> |
| define  | [**BSP\_I2C\_NUM**](#define-bsp_i2c_num)  (CONFIG\_BSP\_I2C\_NUM)<br> |
| define  | [**BSP\_I2C\_SCL**](#define-bsp_i2c_scl)  (GPIO\_NUM\_18)<br> |
| define  | [**BSP\_I2C\_SCL\_R16**](#define-bsp_i2c_scl_r16)  (GPIO\_NUM\_48)<br> |
| define  | [**BSP\_I2C\_SDA**](#define-bsp_i2c_sda)  (GPIO\_NUM\_8)<br> |
| define  | [**BSP\_I2C\_SDA\_R16**](#define-bsp_i2c_sda_r16)  (GPIO\_NUM\_47)<br> |
| define  | [**BSP\_I2S\_DOUT**](#define-bsp_i2s_dout)  (GPIO\_NUM\_6)<br> |
| define  | [**BSP\_I2S\_DSIN**](#define-bsp_i2s_dsin)  (GPIO\_NUM\_15)<br> |
| define  | [**BSP\_I2S\_LCLK**](#define-bsp_i2s_lclk)  (GPIO\_NUM\_7)<br> |
| define  | [**BSP\_I2S\_MCLK**](#define-bsp_i2s_mclk)  (GPIO\_NUM\_5)<br> |
| define  | [**BSP\_I2S\_SCLK**](#define-bsp_i2s_sclk)  (GPIO\_NUM\_16)<br> |
| define  | [**BSP\_IO\_EXPANDER\_I2C\_ADDRESS**](#define-bsp_io_expander_i2c_address)  (ESP\_IO\_EXPANDER\_I2C\_TCA9554\_ADDRESS\_000)<br> |
| define  | [**BSP\_LCD\_H\_RES**](#define-bsp_lcd_h_res)  [**bsp\_display\_get\_h\_res**](#function-bsp_display_get_h_res)()<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_3\_DATA0**](#define-bsp_lcd_sub_board_2_3_data0)  (GPIO\_NUM\_10)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_3\_DATA1**](#define-bsp_lcd_sub_board_2_3_data1)  (GPIO\_NUM\_11)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_3\_DATA10**](#define-bsp_lcd_sub_board_2_3_data10)  (GPIO\_NUM\_39)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_3\_DATA11**](#define-bsp_lcd_sub_board_2_3_data11)  (GPIO\_NUM\_40)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_3\_DATA12**](#define-bsp_lcd_sub_board_2_3_data12)  (GPIO\_NUM\_41)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_3\_DATA13**](#define-bsp_lcd_sub_board_2_3_data13)  (GPIO\_NUM\_42)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_3\_DATA14**](#define-bsp_lcd_sub_board_2_3_data14)  (GPIO\_NUM\_2)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_3\_DATA15**](#define-bsp_lcd_sub_board_2_3_data15)  (GPIO\_NUM\_1)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_3\_DATA2**](#define-bsp_lcd_sub_board_2_3_data2)  (GPIO\_NUM\_12)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_3\_DATA3**](#define-bsp_lcd_sub_board_2_3_data3)  (GPIO\_NUM\_13)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_3\_DATA4**](#define-bsp_lcd_sub_board_2_3_data4)  (GPIO\_NUM\_14)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_3\_DATA5**](#define-bsp_lcd_sub_board_2_3_data5)  (GPIO\_NUM\_21)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_3\_DATA6**](#define-bsp_lcd_sub_board_2_3_data6)  (GPIO\_NUM\_47)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_3\_DATA6\_R16**](#define-bsp_lcd_sub_board_2_3_data6_r16)  (GPIO\_NUM\_8)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_3\_DATA7**](#define-bsp_lcd_sub_board_2_3_data7)  (GPIO\_NUM\_48)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_3\_DATA7\_R16**](#define-bsp_lcd_sub_board_2_3_data7_r16)  (GPIO\_NUM\_18)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_3\_DATA8**](#define-bsp_lcd_sub_board_2_3_data8)  (GPIO\_NUM\_45)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_3\_DATA9**](#define-bsp_lcd_sub_board_2_3_data9)  (GPIO\_NUM\_38)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_3\_DE**](#define-bsp_lcd_sub_board_2_3_de)  (GPIO\_NUM\_17)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_3\_DISP**](#define-bsp_lcd_sub_board_2_3_disp)  (GPIO\_NUM\_NC)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_3\_HSYNC**](#define-bsp_lcd_sub_board_2_3_hsync)  (GPIO\_NUM\_46)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_3\_PCLK**](#define-bsp_lcd_sub_board_2_3_pclk)  (GPIO\_NUM\_9)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_3\_VSYNC**](#define-bsp_lcd_sub_board_2_3_vsync)  (GPIO\_NUM\_3)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_H\_RES**](#define-bsp_lcd_sub_board_2_h_res)  (480)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_SPI\_CS**](#define-bsp_lcd_sub_board_2_spi_cs)  (IO\_EXPANDER\_PIN\_NUM\_1)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_SPI\_SCK**](#define-bsp_lcd_sub_board_2_spi_sck)  (IO\_EXPANDER\_PIN\_NUM\_2)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_SPI\_SDO**](#define-bsp_lcd_sub_board_2_spi_sdo)  (IO\_EXPANDER\_PIN\_NUM\_3)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_2\_V\_RES**](#define-bsp_lcd_sub_board_2_v_res)  (480)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_3\_H\_RES**](#define-bsp_lcd_sub_board_3_h_res)  (800)<br> |
| define  | [**BSP\_LCD\_SUB\_BOARD\_3\_V\_RES**](#define-bsp_lcd_sub_board_3_v_res)  (480)<br> |
| define  | [**BSP\_LCD\_V\_RES**](#define-bsp_lcd_v_res)  [**bsp\_display\_get\_v\_res**](#function-bsp_display_get_v_res)()<br> |
| define  | [**BSP\_POWER\_AMP\_IO**](#define-bsp_power_amp_io)  (IO\_EXPANDER\_PIN\_NUM\_0)<br> |
| define  | [**BSP\_SPIFFS\_MOUNT\_POINT**](#define-bsp_spiffs_mount_point)  CONFIG\_BSP\_SPIFFS\_MOUNT\_POINT<br> |
| define  | [**BSP\_USB\_NEG**](#define-bsp_usb_neg)  ((GPIO\_NUM\_19))<br> |
| define  | [**BSP\_USB\_POS**](#define-bsp_usb_pos)  ((GPIO\_NUM\_20))<br> |
| define  | [**LVGL\_BUFFER\_HEIGHT**](#define-lvgl_buffer_height)  (CONFIG\_BSP\_DISPLAY\_LVGL\_BUF\_HEIGHT)<br> |
| define  | [**SUB\_BOARD2\_480\_480\_PANEL\_60HZ\_RGB\_TIMING**](#define-sub_board2_480_480_panel_60hz_rgb_timing) () GC9503\_480\_480\_PANEL\_60HZ\_RGB\_TIMING()<br> |
| define  | [**SUB\_BOARD2\_480\_480\_PANEL\_SCL\_ACTIVE\_EDGE**](#define-sub_board2_480_480_panel_scl_active_edge)  (0)<br> |
| define  | [**SUB\_BOARD3\_800\_480\_PANEL\_35HZ\_RGB\_TIMING**](#define-sub_board3_800_480_panel_35hz_rgb_timing) () <br> |

## Structures and Types Documentation

### enum `bsp_button_t`

```c
enum bsp_button_t {
    BSP_BUTTON_BOOT = 0,
    BSP_BUTTON_NUM
};
```

### struct `bsp_display_cfg_t`

_BSP display configuration structure._

Variables:

-  lvgl\_port\_cfg\_t lvgl_port_cfg  


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


**Note:**

This function will call `bsp_audio_init()` if it has not been called already.



**Returns:**

Pointer to codec device handle or NULL when error occurred
### function `bsp_audio_codec_speaker_init`

_Initialize speaker codec device._
```c
esp_codec_dev_handle_t bsp_audio_codec_speaker_init (
    void
) 
```


**Note:**

This function will call `bsp_audio_init()` if it has not been called already.



**Returns:**

Pointer to codec device handle or NULL when error occurred
### function `bsp_audio_init`

_Init audio._
```c
esp_err_t bsp_audio_init (
    const i2s_std_config_t *i2s_config
) 
```


**Note:**

There is no deinit audio function. Users can free audio resources by calling `i2s_del_channel()`.



**Note:**

This function wiil call `bsp_io_expander_init()` to setup and enable the control pin of audio power amplifier.



**Note:**

This function will be called in `bsp_audio_codec_speaker_init()` and`bsp_audio_codec_microphone_init()`.



**Parameters:**


* `i2s_config` I2S configuration. Pass NULL to use default values (Mono, duplex, 16bit, 22050 Hz) 


**Returns:**



* ESP\_OK On success
* ESP\_ERR\_NOT\_SUPPORTED The communication mode is not supported on the current chip
* ESP\_ERR\_INVALID\_ARG NULL pointer or invalid configuration
* ESP\_ERR\_NOT\_FOUND No available I2S channel found
* ESP\_ERR\_NO\_MEM No memory for storing the channel information
* ESP\_ERR\_INVALID\_STATE This channel has not initialized or already started
* other error codes
### function `bsp_audio_poweramp_enable`

_Enable/disable audio power amplifier (deprecated)_
```c
esp_err_t bsp_audio_poweramp_enable (
    bool enable
) 
```


**Parameters:**


* `enable` Enable/disable audio power amplifier


**Returns:**



* ESP\_OK: On success
* ESP\_ERR\_INVALID\_ARG: Invalid GPIO number
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



* true: Button pressed
* false: Button released
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



* ESP\_OK: Success
* ESP\_ERR\_INVALID\_ARG: Parameter error
### function `bsp_display_get_h_res`

_Get display horizontal resolution._
```c
uint16_t bsp_display_get_h_res (
    void
) 
```


**Note:**

This function should be called after calling `bsp_display_new()` or`bsp_display_start()`



**Returns:**

Horizontal resolution. Return 0 if error occurred.
### function `bsp_display_get_input_dev`

_Get pointer to input device (touch, buttons, ...)_
```c
lv_indev_t * bsp_display_get_input_dev (
    void
) 
```


**Note:**

The LVGL input device is initialized in `bsp_display_start()` function.



**Note:**

This function should be called after calling `bsp_display_start()`.



**Returns:**

Pointer to LVGL input device or NULL when not initialized
### function `bsp_display_get_v_res`

_Get display vertical resolution._
```c
uint16_t bsp_display_get_v_res (
    void
) 
```


**Note:**

This function should be called after calling `bsp_display_new()` or`bsp_display_start()`



**Returns:**

Vertical resolution. Return 0 if error occurred.
### function `bsp_display_lock`

_Take LVGL mutex._
```c
bool bsp_display_lock (
    uint32_t timeout_ms
) 
```


**Note:**

Display must be already initialized by calling `bsp_display_start()`



**Parameters:**


* `timeout_ms` Timeout in [ms]. 0 will block indefinitely.


**Returns:**



* true: Mutex was taken
* false: Mutex was NOT taken
### function `bsp_display_rotate`

_Rotate screen._
```c
void bsp_display_rotate (
    lv_display_t *disp,
    lv_display_rotation_t rotation
) 
```


**Note:**

Display must be already initialized by calling `bsp_display_start()`



**Note:**

This function can't work with the anti-tearing function. Please use the `BSP_DISPLAY_LVGL_ROTATION` configuration instead.



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


**Note:**

This function initializes display controller and starts LVGL handling task. 



**Note:**

Users can get LCD panel handle from `user_data` in returned display.



**Returns:**

Pointer to LVGL display or NULL when error occurred
### function `bsp_display_start_with_config`

_Initialize display._
```c
lv_display_t * bsp_display_start_with_config (
    const bsp_display_cfg_t *cfg
) 
```


This function initializes SPI, display controller and starts LVGL handling task. LCD backlight must be enabled separately by calling `bsp_display_brightness_set()`



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


**Note:**

Display must be already initialized by calling `bsp_display_start()`
### function `bsp_i2c_deinit`

_Deinit I2C driver and free its resources._
```c
esp_err_t bsp_i2c_deinit (
    void
) 
```


**Returns:**



* ESP\_OK: On success
* ESP\_ERR\_INVALID\_ARG: I2C parameter error
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



* ESP\_OK: On success
* ESP\_ERR\_INVALID\_ARG: I2C parameter error
* ESP\_FAIL: I2C driver installation error
### function `bsp_io_expander_init`

_Init IO expander chip TCA9554._
```c
esp_io_expander_handle_t bsp_io_expander_init (
    void
) 
```


**Note:**

If the device was already initialized, users can also use it to get handle. 



**Note:**

This function will be called in `bsp_display_start()` when using LCD sub-board 2 with the resolution of 480x480.



**Note:**

This function will be called in `bsp_audio_init()`.



**Returns:**

Pointer to device handle or NULL when error occurred
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

## Macros Documentation

### define `BSP_ADC_UNIT`

```c
#define BSP_ADC_UNIT ADC_UNIT_1
```

### define `BSP_BOARD_ESP32_S3_LCD_EV_BOARD`

```c
#define BSP_BOARD_ESP32_S3_LCD_EV_BOARD 
```

### define `BSP_BUTTON_BOOT_IO`

```c
#define BSP_BUTTON_BOOT_IO (GPIO_NUM_0)
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

### define `BSP_CAPS_DISPLAY`

```c
#define BSP_CAPS_DISPLAY 1
```

### define `BSP_CAPS_IMU`

```c
#define BSP_CAPS_IMU 0
```

### define `BSP_CAPS_SDCARD`

```c
#define BSP_CAPS_SDCARD 0
```

### define `BSP_CAPS_TOUCH`

```c
#define BSP_CAPS_TOUCH 1
```

### define `BSP_I2C_NUM`

```c
#define BSP_I2C_NUM (CONFIG_BSP_I2C_NUM)
```

### define `BSP_I2C_SCL`

```c
#define BSP_I2C_SCL (GPIO_NUM_18)
```

### define `BSP_I2C_SCL_R16`

```c
#define BSP_I2C_SCL_R16 (GPIO_NUM_48)
```

### define `BSP_I2C_SDA`

```c
#define BSP_I2C_SDA (GPIO_NUM_8)
```

### define `BSP_I2C_SDA_R16`

```c
#define BSP_I2C_SDA_R16 (GPIO_NUM_47)
```

### define `BSP_I2S_DOUT`

```c
#define BSP_I2S_DOUT (GPIO_NUM_6)
```

### define `BSP_I2S_DSIN`

```c
#define BSP_I2S_DSIN (GPIO_NUM_15)
```

### define `BSP_I2S_LCLK`

```c
#define BSP_I2S_LCLK (GPIO_NUM_7)
```

### define `BSP_I2S_MCLK`

```c
#define BSP_I2S_MCLK (GPIO_NUM_5)
```

### define `BSP_I2S_SCLK`

```c
#define BSP_I2S_SCLK (GPIO_NUM_16)
```

### define `BSP_IO_EXPANDER_I2C_ADDRESS`

```c
#define BSP_IO_EXPANDER_I2C_ADDRESS (ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000)
```

### define `BSP_LCD_H_RES`

```c
#define BSP_LCD_H_RES bsp_display_get_h_res ()
```

### define `BSP_LCD_SUB_BOARD_2_3_DATA0`

```c
#define BSP_LCD_SUB_BOARD_2_3_DATA0 (GPIO_NUM_10)
```

### define `BSP_LCD_SUB_BOARD_2_3_DATA1`

```c
#define BSP_LCD_SUB_BOARD_2_3_DATA1 (GPIO_NUM_11)
```

### define `BSP_LCD_SUB_BOARD_2_3_DATA10`

```c
#define BSP_LCD_SUB_BOARD_2_3_DATA10 (GPIO_NUM_39)
```

### define `BSP_LCD_SUB_BOARD_2_3_DATA11`

```c
#define BSP_LCD_SUB_BOARD_2_3_DATA11 (GPIO_NUM_40)
```

### define `BSP_LCD_SUB_BOARD_2_3_DATA12`

```c
#define BSP_LCD_SUB_BOARD_2_3_DATA12 (GPIO_NUM_41)
```

### define `BSP_LCD_SUB_BOARD_2_3_DATA13`

```c
#define BSP_LCD_SUB_BOARD_2_3_DATA13 (GPIO_NUM_42)
```

### define `BSP_LCD_SUB_BOARD_2_3_DATA14`

```c
#define BSP_LCD_SUB_BOARD_2_3_DATA14 (GPIO_NUM_2)
```

### define `BSP_LCD_SUB_BOARD_2_3_DATA15`

```c
#define BSP_LCD_SUB_BOARD_2_3_DATA15 (GPIO_NUM_1)
```

### define `BSP_LCD_SUB_BOARD_2_3_DATA2`

```c
#define BSP_LCD_SUB_BOARD_2_3_DATA2 (GPIO_NUM_12)
```

### define `BSP_LCD_SUB_BOARD_2_3_DATA3`

```c
#define BSP_LCD_SUB_BOARD_2_3_DATA3 (GPIO_NUM_13)
```

### define `BSP_LCD_SUB_BOARD_2_3_DATA4`

```c
#define BSP_LCD_SUB_BOARD_2_3_DATA4 (GPIO_NUM_14)
```

### define `BSP_LCD_SUB_BOARD_2_3_DATA5`

```c
#define BSP_LCD_SUB_BOARD_2_3_DATA5 (GPIO_NUM_21)
```

### define `BSP_LCD_SUB_BOARD_2_3_DATA6`

```c
#define BSP_LCD_SUB_BOARD_2_3_DATA6 (GPIO_NUM_47)
```

### define `BSP_LCD_SUB_BOARD_2_3_DATA6_R16`

```c
#define BSP_LCD_SUB_BOARD_2_3_DATA6_R16 (GPIO_NUM_8)
```

### define `BSP_LCD_SUB_BOARD_2_3_DATA7`

```c
#define BSP_LCD_SUB_BOARD_2_3_DATA7 (GPIO_NUM_48)
```

### define `BSP_LCD_SUB_BOARD_2_3_DATA7_R16`

```c
#define BSP_LCD_SUB_BOARD_2_3_DATA7_R16 (GPIO_NUM_18)
```

### define `BSP_LCD_SUB_BOARD_2_3_DATA8`

```c
#define BSP_LCD_SUB_BOARD_2_3_DATA8 (GPIO_NUM_45)
```

### define `BSP_LCD_SUB_BOARD_2_3_DATA9`

```c
#define BSP_LCD_SUB_BOARD_2_3_DATA9 (GPIO_NUM_38)
```

### define `BSP_LCD_SUB_BOARD_2_3_DE`

```c
#define BSP_LCD_SUB_BOARD_2_3_DE (GPIO_NUM_17)
```

### define `BSP_LCD_SUB_BOARD_2_3_DISP`

```c
#define BSP_LCD_SUB_BOARD_2_3_DISP (GPIO_NUM_NC)
```

### define `BSP_LCD_SUB_BOARD_2_3_HSYNC`

```c
#define BSP_LCD_SUB_BOARD_2_3_HSYNC (GPIO_NUM_46)
```

### define `BSP_LCD_SUB_BOARD_2_3_PCLK`

```c
#define BSP_LCD_SUB_BOARD_2_3_PCLK (GPIO_NUM_9)
```

### define `BSP_LCD_SUB_BOARD_2_3_VSYNC`

```c
#define BSP_LCD_SUB_BOARD_2_3_VSYNC (GPIO_NUM_3)
```

### define `BSP_LCD_SUB_BOARD_2_H_RES`

```c
#define BSP_LCD_SUB_BOARD_2_H_RES (480)
```

### define `BSP_LCD_SUB_BOARD_2_SPI_CS`

```c
#define BSP_LCD_SUB_BOARD_2_SPI_CS (IO_EXPANDER_PIN_NUM_1)
```

### define `BSP_LCD_SUB_BOARD_2_SPI_SCK`

```c
#define BSP_LCD_SUB_BOARD_2_SPI_SCK (IO_EXPANDER_PIN_NUM_2)
```

### define `BSP_LCD_SUB_BOARD_2_SPI_SDO`

```c
#define BSP_LCD_SUB_BOARD_2_SPI_SDO (IO_EXPANDER_PIN_NUM_3)
```

### define `BSP_LCD_SUB_BOARD_2_V_RES`

```c
#define BSP_LCD_SUB_BOARD_2_V_RES (480)
```

### define `BSP_LCD_SUB_BOARD_3_H_RES`

```c
#define BSP_LCD_SUB_BOARD_3_H_RES (800)
```

### define `BSP_LCD_SUB_BOARD_3_V_RES`

```c
#define BSP_LCD_SUB_BOARD_3_V_RES (480)
```

### define `BSP_LCD_V_RES`

```c
#define BSP_LCD_V_RES bsp_display_get_v_res ()
```

### define `BSP_POWER_AMP_IO`

```c
#define BSP_POWER_AMP_IO (IO_EXPANDER_PIN_NUM_0)
```

### define `BSP_SPIFFS_MOUNT_POINT`

```c
#define BSP_SPIFFS_MOUNT_POINT CONFIG_BSP_SPIFFS_MOUNT_POINT
```

### define `BSP_USB_NEG`

```c
#define BSP_USB_NEG ((GPIO_NUM_19))
```

### define `BSP_USB_POS`

```c
#define BSP_USB_POS ((GPIO_NUM_20))
```

### define `LVGL_BUFFER_HEIGHT`

```c
#define LVGL_BUFFER_HEIGHT (CONFIG_BSP_DISPLAY_LVGL_BUF_HEIGHT)
```

### define `SUB_BOARD2_480_480_PANEL_60HZ_RGB_TIMING`

```c
#define SUB_BOARD2_480_480_PANEL_60HZ_RGB_TIMING (
    
) GC9503_480_480_PANEL_60HZ_RGB_TIMING()
```

### define `SUB_BOARD2_480_480_PANEL_SCL_ACTIVE_EDGE`

```c
#define SUB_BOARD2_480_480_PANEL_SCL_ACTIVE_EDGE (0)
```

### define `SUB_BOARD3_800_480_PANEL_35HZ_RGB_TIMING`

```c
#define SUB_BOARD3_800_480_PANEL_35HZ_RGB_TIMING (
    
) {                                               \
        .pclk_hz = 18 * 1000 * 1000,                \
        .h_res = BSP_LCD_SUB_BOARD_3_H_RES,         \
        .v_res = BSP_LCD_SUB_BOARD_3_V_RES,         \
        .hsync_pulse_width = 40,                    \
        .hsync_back_porch = 40,                     \
        .hsync_front_porch = 48,                    \
        .vsync_pulse_width = 23,                    \
        .vsync_back_porch = 32,                     \
        .vsync_front_porch = 13,                    \
        .flags.pclk_active_neg = true,              \
    }
```


## File bsp/esp32_s3_lcd_ev_board/include/bsp/touch.h

_BSP Touchscreen._

This file offers API for basic touchscreen initialization. It is useful for users who want to use the touchscreen without the default Graphical Library LVGL.

For standard LCD initialization with LVGL graphical library, you can call all-in-one function [**bsp\_display\_start()**](#function-bsp_display_start).

## Structures and Types

| Type | Name |
| ---: | :--- |
| struct | [**bsp\_touch\_config\_t**](#struct-bsp_touch_config_t) <br>_BSP touch configuration structure._ |

## Functions

| Type | Name |
| ---: | :--- |
|  esp\_err\_t | [**bsp\_touch\_new**](#function-bsp_touch_new) (const [**bsp\_touch\_config\_t**](#struct-bsp_touch_config_t) \*config, esp\_lcd\_touch\_handle\_t \*ret\_touch) <br>_Create new touchscreen._ |


## Structures and Types Documentation

### struct `bsp_touch_config_t`

_BSP touch configuration structure._

Variables:

-  void \* dummy  <br>Prepared for future use.


## Functions Documentation

### function `bsp_touch_new`

_Create new touchscreen._
```c
esp_err_t bsp_touch_new (
    const bsp_touch_config_t *config,
    esp_lcd_touch_handle_t *ret_touch
) 
```


If you want to free resources allocated by this function, you can use esp\_lcd\_touch API, ie.:


````cpp
esp_lcd_touch_del(tp);
````





**Parameters:**


* `config` touch configuration. Set to NULL if not needed. 
* `ret_touch` esp\_lcd\_touch touchscreen handle. Set to NULL if not needed. 


**Returns:**



* ESP\_OK On success
* Else esp\_lcd\_touch failure


