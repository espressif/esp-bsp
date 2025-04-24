# Board Support Package development guide

This guide describes necessary steps to add a new BSP to the esp-bsp project.

### New BSP requirements
* We want the BSPs to be used by as many users and projects as possible. All BSP must support IDF from version 4.4 up to latest release. Boards that were released after IDF v5.0 do not have to be backward compatible with v4.4.
* We want the BSPs to provide a 'board abstraction layer' that allows the user to run his application on a range of different boards. Public API for each BSP must be the same. The API consistency is ensured by conventions described below.
* We want the main features to be clearly visible for new users. Each new BSP must be added to a table of supported boards in root [README.md](https://github.com/espressif/esp-bsp/blob/master/README.md) it must contain a license (Apache-2) and a README.md file. Conventions for the readme file are described below
* All BSPs are uploaded to [ESP Registry](https://components.espressif.com/). So each BSP must contain `idf_component.yml` file and must be added to [CI upload job](https://github.com/espressif/esp-bsp/blob/master/.github/workflows/upload_component.yml). The `idf_component.yml` must contain: version, description, url, target and (optional) dependencies.
* We want to provide LVGL users with easy, out-of-the-box working solution for their GUI applications. Each board that has a display must be added to LVGL's [SquareLineStudio](https://squareline.io/) pre-defined boards. For more information refer to [this readme](https://github.com/espressif/esp-bsp/blob/master/SquareLine/README.md).
    * List of currently implemented LCD drivers is maintained in a [separate table](https://github.com/espressif/esp-bsp/blob/master/docu/LCD.md)
* We have an experimental idf.py extension, [bsp_ext.py](https://github.com/espressif/esp-bsp/blob/master/examples/bsp_ext.py) that allows changing of BSP during project configuration. Please add new BSPs to the list in this file.

### API conventions

BSP is a standard IDF component. You can start development by `idf.py create-component your_bsp_name`.

The APIs that a BSP can offer depend on the boards capabilities. For example, a board that contains audio codec will expose audio API and a board with display and touch controller will expose display and touch API.

#### General
* The boards capabilities must be listed in its header file, similarly to `SOC_CAPS_*` macro used in esp-idf. For example:
``` c
/**************************************************************************************************
 *  BSP Capabilities
 **************************************************************************************************/

#define BSP_CAPS_DISPLAY        1
#define BSP_CAPS_TOUCH          1
#define BSP_CAPS_BUTTONS        1
#define BSP_CAPS_AUDIO          1
#define BSP_CAPS_AUDIO_SPEAKER  1
#define BSP_CAPS_AUDIO_MIC      1
#define BSP_CAPS_SDCARD         0
#define BSP_CAPS_IMU            1
```
* Each BSP must provide a wrapper header file that can be included by `#include "bsp/esp-bsp.h"`
* BSPs that expose display and touch APIs must provide a header file `bsp/display.h` and `bsp/touch.h` that contain functions for display/touch initialization without graphical library (LVGL)
* Complete board's pinout must be in the beginning of the BSP's header file

#### I2C
* I2C can be used by many devices on the bus. The 'init' call must support multiple invocations
* Uses Kconfig for I2C peripheral selection
* Functions, which must be defined in BSP *.c file:
``` c
esp_err_t bsp_i2c_init(void);
esp_err_t bsp_i2c_deinit(void);
```
* See any BSP

#### Display
* BSPs with display are shipped with [LVGL](https://components.espressif.com/components/lvgl/lvgl) component by default. For version without LVGL, you can use BSP version with `noglib` suffix (eg. `esp32_s3_eye_noglib`)
* Display functions are divided into two parts. The first part is HW initialization only (include/bsp/display.h), second part is initialization of the display and LVGL (bsp/esp-bsp.h)
* HW initialization functions, which must be defined in bsp/display.h file:
``` c
esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel, esp_lcd_panel_io_handle_t *ret_io);
esp_err_t bsp_display_brightness_set(int brightness_percent);
esp_err_t bsp_display_backlight_on(void);
esp_err_t bsp_display_backlight_off(void);
```
* LVGL init function, which must be defined in bsp/esp-bsp.h file:
``` c
lv_disp_t *bsp_display_start(void);
lv_disp_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg);
bool bsp_display_lock(uint32_t timeout_ms);
void bsp_display_unlock(void);
void bsp_display_rotate(lv_disp_t *disp, lv_disp_rot_t rotation);
```
* See esp_wrover_kit, esp32_s2_kaluga_kit ...

#### LCD Touch
* LCD touch drivers are build around base [esp_lcd_touch](https://components.espressif.com/components/espressif/esp_lcd_touch) component
* BSPs with touch display are shipped with [LVGL](https://components.espressif.com/components/lvgl/lvgl) component by default. For version without LVGL, you can use BSP version with `noglib` suffix (eg. `esp32_s3_eye_noglib`)
* If LVGL is used, the touch controller is initialized together with display. If LVGL is not used you can use HW initialization functions from bsp/touch.h
* HW initialization function, which must be defined in bsp/touch.h file:
``` c
esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch);
```
* LVGL get input device function for advanced scenarios in bsp/esp-bsp.h
``` c
lv_indev_t *bsp_display_get_input_dev(void);
```
* See esp-box, esp-box-3 ...

#### SPI Flash File System
* The SPIFFS is supported by all ESP chips internally. It allows the users to store large files (pictures, recordings...)
* Functions, which must be defined in BSP *.c file:
``` c
esp_err_t bsp_spiffs_mount(void);
esp_err_t bsp_spiffs_unmount(void);
```
* Uses Kconfig for SPIFFS settings
* See any BSP

#### SD card
* Functions, which should be defined in BSP *.c file:
``` c
esp_err_t bsp_sdcard_mount(void);
esp_err_t bsp_sdcard_unmount(void);
```
* Uses Kconfig for mountpoint and possible `format_on_fail` settings
* See esp_wrover_kit

> @todo We still use `extern sdmmc_card_t *bsp_sdcard;` which is very ugly :( Tracked in BSP-189

> @todo We could reuse code from https://github.com/espressif/esp-box/blob/master/components/bsp/src/storage/bsp_sdcard.c

#### Buttons
* We want to provide unified API for all types of buttons
* Buttons feature is covered by [button](https://components.espressif.com/components/espressif/button) component. It allows usage of GPIO, ADC, io_expander and even capacitive buttons with various events (pressed, released, double click, long press...)
* All buttons must be added to enum `bsp_button_t`
* All buttons must be initialized in the following call
``` c
esp_err_t bsp_iot_button_create(button_handle_t btn_array[], int *btn_cnt, int btn_array_size);
```
* See esp32_s2_kaluga_kit

#### LEDs
* Similarly to buttons, we want to provide unified API for all types of LEDs (classic, RGB, addressable RGB)
* LEDs feature is covered by [led_indicator](https://components.espressif.com/components/espressif/led_indicator) component. It support all aforementioned types of LEDs with various control options (brightness, color, transitions...)
* All LEDs must be initialized in the following call
``` c
esp_err_t bsp_led_indicator_create(led_indicator_handle_t led_array[], int *led_cnt, int led_array_size);
```
* See esp32_s3_korvo_1


#### Audio
* Audio feature is covered by [esp_codec_dev](https://components.espressif.com/components/espressif/esp_codec_dev) component
* BSP provides API for easy initialization:
``` c
esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config);
const audio_codec_data_if_t *bsp_audio_get_codec_itf(void);
esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void);
esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void);
```
* See esp32_s3_korvo_2, esp32_s2_kaluga_kit

#### Camera
* Camera feature is covered by [esp32-camera](https://components.espressif.com/components/espressif/esp32-camera) component
* There is no API needed. We only provide pinout and default camera configuration macro `BSP_CAMERA_DEFAULT_CONFIG`
* See esp32_s2_kaluga_kit, esp32_s3_eye

#### USB
* Only provide pinout for completeness
* See esp32_s3_usb_otg

### README.md conventions
> @todo tracked in BSP-362

