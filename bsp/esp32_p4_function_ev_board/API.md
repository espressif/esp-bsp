# API Reference

## Header files

- [bsp/esp32_p4_function_ev_board/include/bsp/display.h](#file-bspesp32_p4_function_ev_boardincludebspdisplayh)
- [bsp/esp32_p4_function_ev_board/include/bsp/esp32_p4_function_ev_board.h](#file-bspesp32_p4_function_ev_boardincludebspesp32_p4_function_ev_boardh)
- [bsp/esp32_p4_function_ev_board/include/bsp/touch.h](#file-bspesp32_p4_function_ev_boardincludebsptouchh)

## File bsp/esp32_p4_function_ev_board/include/bsp/display.h

_BSP LCD._

This file offers API for basic LCD control. It is useful for users who want to use the LCD without the default Graphical Library LVGL.

For standard LCD initialization with LVGL graphical library, you can call all-in-one function [**bsp\_display\_start()**](#function-bsp_display_start).

## Structures and Types

| Type | Name |
| ---: | :--- |
| struct | [**bsp\_display\_config\_t**](#struct-bsp_display_config_t) <br>_BSP display configuration structure._ |
| enum  | [**bsp\_hdmi\_resolution\_t**](#enum-bsp_hdmi_resolution_t)  <br>_BSP HDMI resolution types._ |
| struct | [**bsp\_lcd\_handles\_t**](#struct-bsp_lcd_handles_t) <br>_BSP display return handles._ |

## Functions

| Type | Name |
| ---: | :--- |
|  esp\_err\_t | [**bsp\_display\_backlight\_off**](#function-bsp_display_backlight_off) (void) <br>_Turn off display backlight._ |
|  esp\_err\_t | [**bsp\_display\_backlight\_on**](#function-bsp_display_backlight_on) (void) <br>_Turn on display backlight._ |
|  esp\_err\_t | [**bsp\_display\_brightness\_deinit**](#function-bsp_display_brightness_deinit) (void) <br>_Deinitialize display's brightness._ |
|  esp\_err\_t | [**bsp\_display\_brightness\_init**](#function-bsp_display_brightness_init) (void) <br>_Initialize display's brightness._ |
|  esp\_err\_t | [**bsp\_display\_brightness\_set**](#function-bsp_display_brightness_set) (int brightness\_percent) <br>_Set display's brightness._ |
|  void | [**bsp\_display\_delete**](#function-bsp_display_delete) (void) <br>_Delete display panel._ |
|  esp\_err\_t | [**bsp\_display\_new**](#function-bsp_display_new) (const [**bsp\_display\_config\_t**](#struct-bsp_display_config_t) \*config, esp\_lcd\_panel\_handle\_t \*ret\_panel, esp\_lcd\_panel\_io\_handle\_t \*ret\_io) <br>_Create new display panel._ |
|  esp\_err\_t | [**bsp\_display\_new\_with\_handles**](#function-bsp_display_new_with_handles) (const [**bsp\_display\_config\_t**](#struct-bsp_display_config_t) \*config, [**bsp\_lcd\_handles\_t**](#struct-bsp_lcd_handles_t) \*ret\_handles) <br>_Create new display panel._ |

## Macros

| Type | Name |
| ---: | :--- |
| define  | [**BSP\_LCD\_BIGENDIAN**](#define-bsp_lcd_bigendian)  (0)<br> |
| define  | [**BSP\_LCD\_BITS\_PER\_PIXEL**](#define-bsp_lcd_bits_per_pixel)  (16)<br> |
| define  | [**BSP\_LCD\_COLOR\_FORMAT**](#define-bsp_lcd_color_format)  (ESP\_LCD\_COLOR\_FORMAT\_RGB565)<br> |
| define  | [**BSP\_LCD\_COLOR\_SPACE**](#define-bsp_lcd_color_space)  (ESP\_LCD\_COLOR\_SPACE\_RGB)<br> |
| define  | [**BSP\_LCD\_H\_RES**](#define-bsp_lcd_h_res)  (800)<br> |
| define  | [**BSP\_LCD\_MIPI\_DSI\_LANE\_BITRATE\_MBPS**](#define-bsp_lcd_mipi_dsi_lane_bitrate_mbps)  (1000)<br> |
| define  | [**BSP\_LCD\_MIPI\_DSI\_LANE\_NUM**](#define-bsp_lcd_mipi_dsi_lane_num)  (2)<br> |
| define  | [**BSP\_LCD\_V\_RES**](#define-bsp_lcd_v_res)  (1280)<br> |
| define  | [**BSP\_MIPI\_DSI\_PHY\_PWR\_LDO\_CHAN**](#define-bsp_mipi_dsi_phy_pwr_ldo_chan)  (3)<br> |
| define  | [**BSP\_MIPI\_DSI\_PHY\_PWR\_LDO\_VOLTAGE\_MV**](#define-bsp_mipi_dsi_phy_pwr_ldo_voltage_mv)  (2500)<br> |
| define  | [**ESP\_LCD\_COLOR\_FORMAT\_RGB565**](#define-esp_lcd_color_format_rgb565)  (1)<br> |
| define  | [**ESP\_LCD\_COLOR\_FORMAT\_RGB888**](#define-esp_lcd_color_format_rgb888)  (2)<br> |

## Structures and Types Documentation

### struct `bsp_display_config_t`

_BSP display configuration structure._

Variables:

-  struct [**bsp\_display\_config\_t**](#struct-bsp_display_config_t) dsi_bus  

-  [**bsp\_hdmi\_resolution\_t**](#enum-bsp_hdmi_resolution_t) hdmi_resolution  <br>HDMI resolution selection

-  uint32\_t lane_bit_rate_mbps  <br>DSI bus config - lane bit rate

-  mipi\_dsi\_phy\_clock\_source\_t phy_clk_src  <br>DSI bus config - clock source

### enum `bsp_hdmi_resolution_t`

_BSP HDMI resolution types._
```c
enum bsp_hdmi_resolution_t {
    BSP_HDMI_RES_NONE = 0,
    BSP_HDMI_RES_800x600,
    BSP_HDMI_RES_1024x768,
    BSP_HDMI_RES_1280x720,
    BSP_HDMI_RES_1280x800,
    BSP_HDMI_RES_1920x1080
};
```

### struct `bsp_lcd_handles_t`

_BSP display return handles._

Variables:

-  esp\_lcd\_panel\_handle\_t control  <br>ESP LCD panel (control) handle

-  esp\_lcd\_panel\_io\_handle\_t io  <br>ESP LCD IO handle

-  esp\_lcd\_dsi\_bus\_handle\_t mipi_dsi_bus  <br>MIPI DSI bus handle

-  esp\_lcd\_panel\_handle\_t panel  <br>ESP LCD panel (color) handle


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
### function `bsp_display_brightness_deinit`

_Deinitialize display's brightness._
```c
esp_err_t bsp_display_brightness_deinit (
    void
) 
```

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
### function `bsp_display_delete`

_Delete display panel._
```c
void bsp_display_delete (
    void
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


For maximum flexibility, this function performs only reset and initialization of the display. You must turn on the display explicitly by calling esp\_lcd\_panel\_disp\_on\_off(). The display's backlight is not turned on either. You can use bsp\_display\_backlight\_on/off(), [**bsp\_display\_brightness\_set()**](#function-bsp_display_brightness_set) (on supported boards) or implement your own backlight control.

If you want to free resources allocated by this function, you can use esp\_lcd API, ie.:


````cpp
esp_lcd_panel_del(panel);
esp_lcd_panel_io_del(io);
esp_lcd_del_dsi_bus(mipi_dsi_bus);
````





**Parameters:**


* `config` display configuration 
* `ret_panel` esp\_lcd panel handle 
* `ret_io` esp\_lcd IO handle 


**Returns:**



* ESP\_OK On success
* Else esp\_lcd failure
### function `bsp_display_new_with_handles`

_Create new display panel._
```c
esp_err_t bsp_display_new_with_handles (
    const bsp_display_config_t *config,
    bsp_lcd_handles_t *ret_handles
) 
```


For maximum flexibility, this function performs only reset and initialization of the display. You must turn on the display explicitly by calling esp\_lcd\_panel\_disp\_on\_off(). The display's backlight is not turned on either. You can use bsp\_display\_backlight\_on/off(), [**bsp\_display\_brightness\_set()**](#function-bsp_display_brightness_set) (on supported boards) or implement your own backlight control.

If you want to free resources allocated by this function, you can use API:


````cpp
bsp_display_delete();
````





**Parameters:**


* `config` display configuration 
* `ret_handles` all esp\_lcd handles in one structure 


**Returns:**



* ESP\_OK On success
* Else esp\_lcd failure

## Macros Documentation

### define `BSP_LCD_BIGENDIAN`

```c
#define BSP_LCD_BIGENDIAN (0)
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
#define BSP_LCD_H_RES (800)
```

### define `BSP_LCD_MIPI_DSI_LANE_BITRATE_MBPS`

```c
#define BSP_LCD_MIPI_DSI_LANE_BITRATE_MBPS (1000)
```

### define `BSP_LCD_MIPI_DSI_LANE_NUM`

```c
#define BSP_LCD_MIPI_DSI_LANE_NUM (2)
```

### define `BSP_LCD_V_RES`

```c
#define BSP_LCD_V_RES (1280)
```

### define `BSP_MIPI_DSI_PHY_PWR_LDO_CHAN`

```c
#define BSP_MIPI_DSI_PHY_PWR_LDO_CHAN (3)
```

### define `BSP_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV`

```c
#define BSP_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV (2500)
```

### define `ESP_LCD_COLOR_FORMAT_RGB565`

```c
#define ESP_LCD_COLOR_FORMAT_RGB565 (1)
```

### define `ESP_LCD_COLOR_FORMAT_RGB888`

```c
#define ESP_LCD_COLOR_FORMAT_RGB888 (2)
```


## File bsp/esp32_p4_function_ev_board/include/bsp/esp32_p4_function_ev_board.h

_ESP BSP: ESP32-P4 Function EV Board._



## Structures and Types

| Type | Name |
| ---: | :--- |
| struct | [**bsp\_display\_cfg\_t**](#struct-bsp_display_cfg_t) <br>_BSP display configuration structure._ |
| struct | [**bsp\_sdcard\_cfg\_t**](#struct-bsp_sdcard_cfg_t) <br>_BSP SD card configuration structure._ |
| enum  | [**bsp\_usb\_host\_power\_mode\_t**](#enum-bsp_usb_host_power_mode_t)  <br>_Power modes of USB Host connector._ |
| typedef enum [**bsp\_usb\_host\_power\_mode\_t**](#enum-bsp_usb_host_power_mode_t) | [**bsp\_usb\_host\_power\_mode\_t**](#typedef-bsp_usb_host_power_mode_t)  <br>_Power modes of USB Host connector._ |

## Functions

| Type | Name |
| ---: | :--- |
|  esp\_codec\_dev\_handle\_t | [**bsp\_audio\_codec\_microphone\_init**](#function-bsp_audio_codec_microphone_init) (void) <br>_Initialize microphone codec device._ |
|  esp\_codec\_dev\_handle\_t | [**bsp\_audio\_codec\_speaker\_init**](#function-bsp_audio_codec_speaker_init) (void) <br>_Initialize speaker codec device._ |
|  esp\_err\_t | [**bsp\_audio\_init**](#function-bsp_audio_init) (const i2s\_std\_config\_t \*i2s\_config) <br>_Init audio._ |
|  lv\_indev\_t \* | [**bsp\_display\_get\_input\_dev**](#function-bsp_display_get_input_dev) (void) <br>_Get pointer to input device (touch, buttons, ...)_ |
|  bool | [**bsp\_display\_lock**](#function-bsp_display_lock) (uint32\_t timeout\_ms) <br>_Take LVGL mutex._ |
|  void | [**bsp\_display\_rotate**](#function-bsp_display_rotate) (lv\_display\_t \*disp, lv\_disp\_rotation\_t rotation) <br>_Rotate screen._ |
|  lv\_display\_t \* | [**bsp\_display\_start**](#function-bsp_display_start) (void) <br>_Initialize display._ |
|  lv\_display\_t \* | [**bsp\_display\_start\_with\_config**](#function-bsp_display_start_with_config) (const [**bsp\_display\_cfg\_t**](#struct-bsp_display_cfg_t) \*cfg) <br>_Initialize display._ |
|  void | [**bsp\_display\_stop**](#function-bsp_display_stop) (lv\_display\_t \*display) <br>_Deinitialize display._ |
|  void | [**bsp\_display\_unlock**](#function-bsp_display_unlock) (void) <br>_Give LVGL mutex._ |
|  esp\_err\_t | [**bsp\_i2c\_deinit**](#function-bsp_i2c_deinit) (void) <br>_Deinit I2C driver and free its resources._ |
|  i2c\_master\_bus\_handle\_t | [**bsp\_i2c\_get\_handle**](#function-bsp_i2c_get_handle) (void) <br>_Get I2C driver handle._ |
|  esp\_err\_t | [**bsp\_i2c\_init**](#function-bsp_i2c_init) (void) <br>_Init I2C driver._ |
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
|  esp\_err\_t | [**bsp\_usb\_host\_start**](#function-bsp_usb_host_start) ([**bsp\_usb\_host\_power\_mode\_t**](#enum-bsp_usb_host_power_mode_t) mode, bool limit\_500mA) <br>_Start USB host._ |
|  esp\_err\_t | [**bsp\_usb\_host\_stop**](#function-bsp_usb_host_stop) (void) <br>_Stop USB host._ |

## Macros

| Type | Name |
| ---: | :--- |
| define  | [**BSP\_BOARD\_ESP32\_P4\_FUNCTION\_EV\_BOARD**](#define-bsp_board_esp32_p4_function_ev_board)  <br> |
| define  | [**BSP\_CAPS\_AUDIO**](#define-bsp_caps_audio)  1<br> |
| define  | [**BSP\_CAPS\_AUDIO\_MIC**](#define-bsp_caps_audio_mic)  1<br> |
| define  | [**BSP\_CAPS\_AUDIO\_SPEAKER**](#define-bsp_caps_audio_speaker)  1<br> |
| define  | [**BSP\_CAPS\_BUTTONS**](#define-bsp_caps_buttons)  0<br> |
| define  | [**BSP\_CAPS\_DISPLAY**](#define-bsp_caps_display)  1<br> |
| define  | [**BSP\_CAPS\_IMU**](#define-bsp_caps_imu)  0<br> |
| define  | [**BSP\_CAPS\_SDCARD**](#define-bsp_caps_sdcard)  1<br> |
| define  | [**BSP\_CAPS\_TOUCH**](#define-bsp_caps_touch)  1<br> |
| define  | [**BSP\_I2C\_NUM**](#define-bsp_i2c_num)  CONFIG\_BSP\_I2C\_NUM<br> |
| define  | [**BSP\_I2C\_SCL**](#define-bsp_i2c_scl)  (GPIO\_NUM\_8)<br> |
| define  | [**BSP\_I2C\_SDA**](#define-bsp_i2c_sda)  (GPIO\_NUM\_7)<br> |
| define  | [**BSP\_I2S\_DOUT**](#define-bsp_i2s_dout)  (GPIO\_NUM\_9)<br> |
| define  | [**BSP\_I2S\_DSIN**](#define-bsp_i2s_dsin)  (GPIO\_NUM\_11)<br> |
| define  | [**BSP\_I2S\_LCLK**](#define-bsp_i2s_lclk)  (GPIO\_NUM\_10)<br> |
| define  | [**BSP\_I2S\_MCLK**](#define-bsp_i2s_mclk)  (GPIO\_NUM\_13)<br> |
| define  | [**BSP\_I2S\_SCLK**](#define-bsp_i2s_sclk)  (GPIO\_NUM\_12)<br> |
| define  | [**BSP\_LCD\_BACKLIGHT**](#define-bsp_lcd_backlight)  (GPIO\_NUM\_23)<br> |
| define  | [**BSP\_LCD\_DRAW\_BUFF\_DOUBLE**](#define-bsp_lcd_draw_buff_double)  (0)<br> |
| define  | [**BSP\_LCD\_DRAW\_BUFF\_SIZE**](#define-bsp_lcd_draw_buff_size)  (BSP\_LCD\_H\_RES \* 50)<br> |
| define  | [**BSP\_LCD\_PIXEL\_CLOCK\_MHZ**](#define-bsp_lcd_pixel_clock_mhz)  (80)<br> |
| define  | [**BSP\_LCD\_RST**](#define-bsp_lcd_rst)  (GPIO\_NUM\_NC)<br> |
| define  | [**BSP\_LCD\_TOUCH\_INT**](#define-bsp_lcd_touch_int)  (GPIO\_NUM\_NC)<br> |
| define  | [**BSP\_LCD\_TOUCH\_RST**](#define-bsp_lcd_touch_rst)  (GPIO\_NUM\_NC)<br> |
| define  | [**BSP\_POWER\_AMP\_IO**](#define-bsp_power_amp_io)  (GPIO\_NUM\_53)<br> |
| define  | [**BSP\_SDSPI\_HOST**](#define-bsp_sdspi_host)  (SDSPI\_DEFAULT\_HOST)<br> |
| define  | [**BSP\_SD\_CLK**](#define-bsp_sd_clk)  (GPIO\_NUM\_43)<br> |
| define  | [**BSP\_SD\_CMD**](#define-bsp_sd_cmd)  (GPIO\_NUM\_44)<br> |
| define  | [**BSP\_SD\_D0**](#define-bsp_sd_d0)  (GPIO\_NUM\_39)<br> |
| define  | [**BSP\_SD\_D1**](#define-bsp_sd_d1)  (GPIO\_NUM\_40)<br> |
| define  | [**BSP\_SD\_D2**](#define-bsp_sd_d2)  (GPIO\_NUM\_41)<br> |
| define  | [**BSP\_SD\_D3**](#define-bsp_sd_d3)  (GPIO\_NUM\_42)<br> |
| define  | [**BSP\_SD\_MOUNT\_POINT**](#define-bsp_sd_mount_point)  CONFIG\_BSP\_SD\_MOUNT\_POINT<br> |
| define  | [**BSP\_SD\_SPI\_CLK**](#define-bsp_sd_spi_clk)  (GPIO\_NUM\_43)<br> |
| define  | [**BSP\_SD\_SPI\_CS**](#define-bsp_sd_spi_cs)  (GPIO\_NUM\_42)<br> |
| define  | [**BSP\_SD\_SPI\_MISO**](#define-bsp_sd_spi_miso)  (GPIO\_NUM\_39)<br> |
| define  | [**BSP\_SD\_SPI\_MOSI**](#define-bsp_sd_spi_mosi)  (GPIO\_NUM\_44)<br> |
| define  | [**BSP\_SPIFFS\_MOUNT\_POINT**](#define-bsp_spiffs_mount_point)  CONFIG\_BSP\_SPIFFS\_MOUNT\_POINT<br> |

## Structures and Types Documentation

### struct `bsp_display_cfg_t`

_BSP display configuration structure._

Variables:

-  unsigned int buff_dma  <br>Allocated LVGL buffer will be DMA capable

-  unsigned int buff_spiram  <br>Allocated LVGL buffer will be in PSRAM

-  uint32\_t buffer_size  <br>Size of the buffer for the screen in pixels

-  bool double_buffer  <br>True, if should be allocated two buffers

-  struct [**bsp\_display\_cfg\_t**](#struct-bsp_display_cfg_t) flags  

-  [**bsp\_display\_config\_t**](#struct-bsp_display_config_t) hw_cfg  <br>Display HW configuration

-  lvgl\_port\_cfg\_t lvgl_port_cfg  <br>LVGL port configuration

-  unsigned int sw_rotate  <br>Use software rotation (slower), The feature is unavailable under avoid-tear mode

### struct `bsp_sdcard_cfg_t`

_BSP SD card configuration structure._

Variables:

-  sdmmc\_host\_t \* host  

-  const esp\_vfs\_fat\_sdmmc\_mount\_config\_t \* mount  

-  const sdmmc\_slot\_config\_t \* sdmmc  

-  const sdspi\_device\_config\_t \* sdspi  

-  union [**bsp\_sdcard\_cfg\_t**](#struct-bsp_sdcard_cfg_t) slot  

### enum `bsp_usb_host_power_mode_t`

_Power modes of USB Host connector._
```c
enum bsp_usb_host_power_mode_t {
    BSP_USB_HOST_POWER_MODE_USB_DEV
};
```

### typedef `bsp_usb_host_power_mode_t`

_Power modes of USB Host connector._
```c
typedef enum bsp_usb_host_power_mode_t bsp_usb_host_power_mode_t;
```


## Functions Documentation

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


This function initializes MIPI-DSI, display controller and starts LVGL handling task. LCD backlight must be enabled separately by calling [**bsp\_display\_brightness\_set()**](#function-bsp_display_brightness_set)



**Returns:**

Pointer to LVGL display or NULL when error occured
### function `bsp_display_start_with_config`

_Initialize display._
```c
lv_display_t * bsp_display_start_with_config (
    const bsp_display_cfg_t *cfg
) 
```


This function initializes MIPI-DSI, display controller and starts LVGL handling task. LCD backlight must be enabled separately by calling [**bsp\_display\_brightness\_set()**](#function-bsp_display_brightness_set)



**Parameters:**


* `cfg` display configuration


**Returns:**

Pointer to LVGL display or NULL when error occured
### function `bsp_display_stop`

_Deinitialize display._
```c
void bsp_display_stop (
    lv_display_t *display
) 
```


This function deinitializes MIPI-DSI, display controller and stops LVGL.



**Parameters:**


* `display` Pointer to LVGL display
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


* `mode` USB Host connector power mode (Not used on this board) 
* `limit_500mA` Limit output current to 500mA (Not used on this board) 


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

## Macros Documentation

### define `BSP_BOARD_ESP32_P4_FUNCTION_EV_BOARD`

```c
#define BSP_BOARD_ESP32_P4_FUNCTION_EV_BOARD 
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
#define BSP_CAPS_BUTTONS 0
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
#define BSP_CAPS_SDCARD 1
```

### define `BSP_CAPS_TOUCH`

```c
#define BSP_CAPS_TOUCH 1
```

### define `BSP_I2C_NUM`

```c
#define BSP_I2C_NUM CONFIG_BSP_I2C_NUM
```

### define `BSP_I2C_SCL`

```c
#define BSP_I2C_SCL (GPIO_NUM_8)
```

### define `BSP_I2C_SDA`

```c
#define BSP_I2C_SDA (GPIO_NUM_7)
```

### define `BSP_I2S_DOUT`

```c
#define BSP_I2S_DOUT (GPIO_NUM_9)
```

### define `BSP_I2S_DSIN`

```c
#define BSP_I2S_DSIN (GPIO_NUM_11)
```

### define `BSP_I2S_LCLK`

```c
#define BSP_I2S_LCLK (GPIO_NUM_10)
```

### define `BSP_I2S_MCLK`

```c
#define BSP_I2S_MCLK (GPIO_NUM_13)
```

### define `BSP_I2S_SCLK`

```c
#define BSP_I2S_SCLK (GPIO_NUM_12)
```

### define `BSP_LCD_BACKLIGHT`

```c
#define BSP_LCD_BACKLIGHT (GPIO_NUM_23)
```

### define `BSP_LCD_DRAW_BUFF_DOUBLE`

```c
#define BSP_LCD_DRAW_BUFF_DOUBLE (0)
```

### define `BSP_LCD_DRAW_BUFF_SIZE`

```c
#define BSP_LCD_DRAW_BUFF_SIZE (BSP_LCD_H_RES * 50)
```

### define `BSP_LCD_PIXEL_CLOCK_MHZ`

```c
#define BSP_LCD_PIXEL_CLOCK_MHZ (80)
```

### define `BSP_LCD_RST`

```c
#define BSP_LCD_RST (GPIO_NUM_NC)
```

### define `BSP_LCD_TOUCH_INT`

```c
#define BSP_LCD_TOUCH_INT (GPIO_NUM_NC)
```

### define `BSP_LCD_TOUCH_RST`

```c
#define BSP_LCD_TOUCH_RST (GPIO_NUM_NC)
```

### define `BSP_POWER_AMP_IO`

```c
#define BSP_POWER_AMP_IO (GPIO_NUM_53)
```

### define `BSP_SDSPI_HOST`

```c
#define BSP_SDSPI_HOST (SDSPI_DEFAULT_HOST)
```

### define `BSP_SD_CLK`

```c
#define BSP_SD_CLK (GPIO_NUM_43)
```

### define `BSP_SD_CMD`

```c
#define BSP_SD_CMD (GPIO_NUM_44)
```

### define `BSP_SD_D0`

```c
#define BSP_SD_D0 (GPIO_NUM_39)
```

### define `BSP_SD_D1`

```c
#define BSP_SD_D1 (GPIO_NUM_40)
```

### define `BSP_SD_D2`

```c
#define BSP_SD_D2 (GPIO_NUM_41)
```

### define `BSP_SD_D3`

```c
#define BSP_SD_D3 (GPIO_NUM_42)
```

### define `BSP_SD_MOUNT_POINT`

```c
#define BSP_SD_MOUNT_POINT CONFIG_BSP_SD_MOUNT_POINT
```

### define `BSP_SD_SPI_CLK`

```c
#define BSP_SD_SPI_CLK (GPIO_NUM_43)
```

### define `BSP_SD_SPI_CS`

```c
#define BSP_SD_SPI_CS (GPIO_NUM_42)
```

### define `BSP_SD_SPI_MISO`

```c
#define BSP_SD_SPI_MISO (GPIO_NUM_39)
```

### define `BSP_SD_SPI_MOSI`

```c
#define BSP_SD_SPI_MOSI (GPIO_NUM_44)
```

### define `BSP_SPIFFS_MOUNT_POINT`

```c
#define BSP_SPIFFS_MOUNT_POINT CONFIG_BSP_SPIFFS_MOUNT_POINT
```


## File bsp/esp32_p4_function_ev_board/include/bsp/touch.h

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
|  void | [**bsp\_touch\_delete**](#function-bsp_touch_delete) (void) <br>_Deinitialize touch._ |
|  esp\_err\_t | [**bsp\_touch\_new**](#function-bsp_touch_new) (const [**bsp\_touch\_config\_t**](#struct-bsp_touch_config_t) \*config, esp\_lcd\_touch\_handle\_t \*ret\_touch) <br>_Create new touchscreen._ |


## Structures and Types Documentation

### struct `bsp_touch_config_t`

_BSP touch configuration structure._

Variables:

-  void \* dummy  <br>Prepared for future use.


## Functions Documentation

### function `bsp_touch_delete`

_Deinitialize touch._
```c
void bsp_touch_delete (
    void
) 
```

### function `bsp_touch_new`

_Create new touchscreen._
```c
esp_err_t bsp_touch_new (
    const bsp_touch_config_t *config,
    esp_lcd_touch_handle_t *ret_touch
) 
```


If you want to free resources allocated by this function, you can use API:


````cpp
bsp_touch_delete();
````





**Parameters:**


* `config` touch configuration 
* `ret_touch` esp\_lcd\_touch touchscreen handle 


**Returns:**



* ESP\_OK On success
* Else esp\_lcd\_touch failure


