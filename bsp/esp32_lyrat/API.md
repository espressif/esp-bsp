# API Reference

## Header files

- [bsp/esp32_lyrat/include/bsp/esp32_lyrat.h](#file-bspesp32_lyratincludebspesp32_lyrath)

## File bsp/esp32_lyrat/include/bsp/esp32_lyrat.h

_ESP BSP: ESP32-LyraT._



## Structures and Types

| Type | Name |
| ---: | :--- |
| enum  | [**bsp\_button\_t**](#enum-bsp_button_t)  <br> |
| enum  | [**bsp\_led\_t**](#enum-bsp_led_t)  <br> |
| typedef enum bsp\_led\_t | [**bsp\_led\_t**](#typedef-bsp_led_t)  <br> |
| struct | [**bsp\_sdcard\_cfg\_t**](#struct-bsp_sdcard_cfg_t) <br>_BSP SD card configuration structure._ |

## Functions

| Type | Name |
| ---: | :--- |
|  esp\_codec\_dev\_handle\_t | [**bsp\_audio\_codec\_microphone\_init**](#function-bsp_audio_codec_microphone_init) (void) <br>_Initialize microphone codec device._ |
|  esp\_codec\_dev\_handle\_t | [**bsp\_audio\_codec\_speaker\_init**](#function-bsp_audio_codec_speaker_init) (void) <br>_Initialize speaker codec device._ |
|  const audio\_codec\_data\_if\_t \* | [**bsp\_audio\_get\_codec\_itf**](#function-bsp_audio_get_codec_itf) (void) <br>_Get codec I2S interface (initialized in bsp\_audio\_init)_ |
|  esp\_err\_t | [**bsp\_audio\_init**](#function-bsp_audio_init) (const i2s\_std\_config\_t \*i2s\_config) <br>_Init audio._ |
|  esp\_err\_t | [**bsp\_i2c\_deinit**](#function-bsp_i2c_deinit) (void) <br>_Deinit I2C driver and free its resources._ |
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
| define  | [**BSP\_BOARD\_ESP32\_LYRAT**](#define-bsp_board_esp32_lyrat)  <br> |
| define  | [**BSP\_BUTTON\_MODE\_IO**](#define-bsp_button_mode_io)  (GPIO\_NUM\_39)<br> |
| define  | [**BSP\_BUTTON\_PLAY\_TOUCH**](#define-bsp_button_play_touch)  (TOUCH\_PAD\_NUM8)<br> |
| define  | [**BSP\_BUTTON\_REC\_IO**](#define-bsp_button_rec_io)  (GPIO\_NUM\_36)<br> |
| define  | [**BSP\_BUTTON\_SET\_TOUCH**](#define-bsp_button_set_touch)  (TOUCH\_PAD\_NUM9)<br> |
| define  | [**BSP\_BUTTON\_VOLDOWN\_TOUCH**](#define-bsp_button_voldown_touch)  (TOUCH\_PAD\_NUM4)<br> |
| define  | [**BSP\_BUTTON\_VOLUP\_TOUCH**](#define-bsp_button_volup_touch)  (TOUCH\_PAD\_NUM7)<br> |
| define  | [**BSP\_CAPS\_AUDIO**](#define-bsp_caps_audio)  1<br> |
| define  | [**BSP\_CAPS\_AUDIO\_MIC**](#define-bsp_caps_audio_mic)  1<br> |
| define  | [**BSP\_CAPS\_AUDIO\_SPEAKER**](#define-bsp_caps_audio_speaker)  1<br> |
| define  | [**BSP\_CAPS\_BUTTONS**](#define-bsp_caps_buttons)  1<br> |
| define  | [**BSP\_CAPS\_DISPLAY**](#define-bsp_caps_display)  0<br> |
| define  | [**BSP\_CAPS\_IMU**](#define-bsp_caps_imu)  0<br> |
| define  | [**BSP\_CAPS\_LED**](#define-bsp_caps_led)  1<br> |
| define  | [**BSP\_CAPS\_SDCARD**](#define-bsp_caps_sdcard)  1<br> |
| define  | [**BSP\_CAPS\_TOUCH**](#define-bsp_caps_touch)  0<br> |
| define  | [**BSP\_I2C\_NUM**](#define-bsp_i2c_num)  CONFIG\_BSP\_I2C\_NUM<br> |
| define  | [**BSP\_I2C\_SCL**](#define-bsp_i2c_scl)  (GPIO\_NUM\_23)<br> |
| define  | [**BSP\_I2C\_SDA**](#define-bsp_i2c_sda)  (GPIO\_NUM\_18)<br> |
| define  | [**BSP\_I2S\_DOUT**](#define-bsp_i2s_dout)  (GPIO\_NUM\_26)<br> |
| define  | [**BSP\_I2S\_DSIN**](#define-bsp_i2s_dsin)  (GPIO\_NUM\_35)<br> |
| define  | [**BSP\_I2S\_LCLK**](#define-bsp_i2s_lclk)  (GPIO\_NUM\_25)<br> |
| define  | [**BSP\_I2S\_MCLK**](#define-bsp_i2s_mclk)  (GPIO\_NUM\_0)<br> |
| define  | [**BSP\_I2S\_SCLK**](#define-bsp_i2s_sclk)  (GPIO\_NUM\_5)<br> |
| define  | [**BSP\_POWER\_AMP\_IO**](#define-bsp_power_amp_io)  (GPIO\_NUM\_21)<br> |
| define  | [**BSP\_SDSPI\_HOST**](#define-bsp_sdspi_host)  (SDSPI\_DEFAULT\_HOST)<br> |
| define  | [**BSP\_SD\_CLK**](#define-bsp_sd_clk)  (GPIO\_NUM\_14)<br> |
| define  | [**BSP\_SD\_CMD**](#define-bsp_sd_cmd)  (GPIO\_NUM\_15)<br> |
| define  | [**BSP\_SD\_D0**](#define-bsp_sd_d0)  (GPIO\_NUM\_2)<br> |
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
    BSP_BUTTON_REC = 0,
    BSP_BUTTON_MODE,
    BSP_BUTTON_PLAY,
    BSP_BUTTON_SET,
    BSP_BUTTON_VOLUP,
    BSP_BUTTON_VOLDOWN,
    BSP_BUTTON_NUM
};
```

### enum `bsp_led_t`

```c
enum bsp_led_t {
    BSP_LED_GREEN = GPIO_NUM_22
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
* ESP\_ERR\_INVALID\_STATE Could not init IO expander
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
* ESP\_ERR\_NOT\_FOUND if the partition table does not contain SPIFFS partition with given label
* ESP\_ERR\_INVALID\_STATE if esp\_vfs\_spiffs\_unregister was already called
* ESP\_ERR\_NO\_MEM if memory can not be allocated
* ESP\_FAIL if partition can not be mounted
* other error codes

## Macros Documentation

### define `BSP_BOARD_ESP32_LYRAT`

```c
#define BSP_BOARD_ESP32_LYRAT 
```

### define `BSP_BUTTON_MODE_IO`

```c
#define BSP_BUTTON_MODE_IO (GPIO_NUM_39)
```

### define `BSP_BUTTON_PLAY_TOUCH`

```c
#define BSP_BUTTON_PLAY_TOUCH (TOUCH_PAD_NUM8)
```

### define `BSP_BUTTON_REC_IO`

```c
#define BSP_BUTTON_REC_IO (GPIO_NUM_36)
```

### define `BSP_BUTTON_SET_TOUCH`

```c
#define BSP_BUTTON_SET_TOUCH (TOUCH_PAD_NUM9)
```

### define `BSP_BUTTON_VOLDOWN_TOUCH`

```c
#define BSP_BUTTON_VOLDOWN_TOUCH (TOUCH_PAD_NUM4)
```

### define `BSP_BUTTON_VOLUP_TOUCH`

```c
#define BSP_BUTTON_VOLUP_TOUCH (TOUCH_PAD_NUM7)
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
#define BSP_CAPS_DISPLAY 0
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

### define `BSP_I2C_NUM`

```c
#define BSP_I2C_NUM CONFIG_BSP_I2C_NUM
```

### define `BSP_I2C_SCL`

```c
#define BSP_I2C_SCL (GPIO_NUM_23)
```

### define `BSP_I2C_SDA`

```c
#define BSP_I2C_SDA (GPIO_NUM_18)
```

### define `BSP_I2S_DOUT`

```c
#define BSP_I2S_DOUT (GPIO_NUM_26)
```

### define `BSP_I2S_DSIN`

```c
#define BSP_I2S_DSIN (GPIO_NUM_35)
```

### define `BSP_I2S_LCLK`

```c
#define BSP_I2S_LCLK (GPIO_NUM_25)
```

### define `BSP_I2S_MCLK`

```c
#define BSP_I2S_MCLK (GPIO_NUM_0)
```

### define `BSP_I2S_SCLK`

```c
#define BSP_I2S_SCLK (GPIO_NUM_5)
```

### define `BSP_POWER_AMP_IO`

```c
#define BSP_POWER_AMP_IO (GPIO_NUM_21)
```

### define `BSP_SDSPI_HOST`

```c
#define BSP_SDSPI_HOST (SDSPI_DEFAULT_HOST)
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


