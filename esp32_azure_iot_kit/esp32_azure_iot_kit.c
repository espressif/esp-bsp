// Copyright 2015-2021 Espressif Systems (Shanghai) CO LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "bsp/esp32_azure_iot_kit.h"
#include "esp_vfs_fat.h"
#include "driver/ledc.h"

/* Assert on error, if selected in menuconfig. Otherwise return error code. */
//TODO: Move this code into common header
#if CONFIG_BSP_ERROR_CHECK
#define BSP_ERROR_CHECK_RETURN_ERR(x)    ESP_ERROR_CHECK(x)
#define BSP_ERROR_CHECK_RETURN_NULL(x)   ESP_ERROR_CHECK(x)
#define BSP_NULL_CHECK(x, ret)           assert(x)
#define BSP_NULL_CHECK_GOTO(x, goto_tag) assert(x)
#else
#define BSP_ERROR_CHECK_RETURN_ERR(x) do { \
        esp_err_t err_rc_ = (x);            \
        if (unlikely(err_rc_ != ESP_OK)) {  \
            return err_rc_;                 \
        }                                   \
    } while(0)

#define BSP_ERROR_CHECK_RETURN_NULL(x)  do { \
        if (unlikely((x) != ESP_OK)) {      \
            return NULL;                    \
        }                                   \
    } while(0)

#define BSP_NULL_CHECK(x, ret) do { \
        if ((x) == NULL) {          \
            return ret;             \
        }                           \
    } while(0)

#define BSP_NULL_CHECK_GOTO(x, goto_tag) do { \
        if ((x) == NULL) {      \
            goto goto_tag;      \
        }                       \
    } while(0)
#endif

sdmmc_card_t *bsp_sdcard = NULL;

esp_err_t bsp_i2c_init(void)
{
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BSP_I2C_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = BSP_I2C_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = CONFIG_BSP_I2C_CLK_SPEED_HZ
    };
    BSP_ERROR_CHECK_RETURN_ERR(i2c_param_config(BSP_I2C_NUM, &i2c_conf));
    BSP_ERROR_CHECK_RETURN_ERR(i2c_driver_install(BSP_I2C_NUM, i2c_conf.mode, 0, 0, 0));

    return ESP_OK;
}

esp_err_t bsp_i2c_deinit(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(i2c_driver_delete(BSP_I2C_NUM));
    return ESP_OK;
}

esp_err_t bsp_leds_init(void)
{
    const gpio_config_t led_io_config = {
        .pin_bit_mask = BIT64(BSP_LED_AZURE) | BIT64(BSP_LED_WIFI),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    BSP_ERROR_CHECK_RETURN_ERR(gpio_config(&led_io_config));
    return ESP_OK;
}

esp_err_t bsp_led_set(const bsp_led_t led_io, const bool on)
{
    BSP_ERROR_CHECK_RETURN_ERR(gpio_set_level((gpio_num_t) led_io, (uint32_t) on));
    return ESP_OK;
}

esp_err_t bsp_sdcard_mount(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_BSP_uSD_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;

    return esp_vfs_fat_sdmmc_mount(CONFIG_BSP_uSD_MOUNT_POINT, &host, &slot_config, &mount_config, &bsp_sdcard);
}

esp_err_t bsp_sdcard_unmount(void)
{
    return esp_vfs_fat_sdcard_unmount(CONFIG_BSP_uSD_MOUNT_POINT, bsp_sdcard);
}

esp_err_t bsp_buzzer_init(void)
{
    const ledc_timer_config_t buzzer_timer_conf = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = (ledc_timer_t)CONFIG_BSP_BUZZER_LEDC_TIMER_NUM,
        .freq_hz = CONFIG_BSP_BUZZER_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };

    const esp_err_t ret = ledc_timer_config(&buzzer_timer_conf);
    if (ESP_OK != ret) {
        return ret;
    }

    const ledc_channel_config_t buzzer_channel_conf = {
        .gpio_num = (int)BSP_BUZZER_IO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel =  (ledc_channel_t)CONFIG_BSP_BUZZER_LEDC_CHANNEL_NUM,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = (ledc_timer_t)CONFIG_BSP_BUZZER_LEDC_TIMER_NUM,
        .duty = 0,
        .hpoint = 0
    };
    return ledc_channel_config(&buzzer_channel_conf);
}

esp_err_t bsp_buzzer_set(const bool on)
{
    BSP_ERROR_CHECK_RETURN_ERR(ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)CONFIG_BSP_BUZZER_LEDC_CHANNEL_NUM, on ? 127 : 0));
    BSP_ERROR_CHECK_RETURN_ERR(ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)CONFIG_BSP_BUZZER_LEDC_CHANNEL_NUM));
    return ESP_OK;
}

esp_err_t bsp_button_init(void)
{
    const gpio_config_t led_io_config = {
        .pin_bit_mask = BIT64(BSP_BUTTON_IO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    return gpio_config(&led_io_config);
}

bool bsp_button_get(void)
{
    return !(bool)gpio_get_level(BSP_BUTTON_IO);
}
