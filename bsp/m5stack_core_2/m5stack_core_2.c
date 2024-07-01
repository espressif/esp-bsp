/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_spiffs.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "driver/i2c.h"

#include "bsp/m5stack_core_2.h"
#include "bsp/display.h"
#include "bsp/touch.h"
#include "esp_lcd_ili9341.h"
#include "esp_lcd_touch_ft5x06.h"
#include "bsp_err_check.h"
#include "esp_codec_dev_defaults.h"

static const char *TAG = "M5Stack";

#define BSP_AXP192_ADDR 0x34
#define BSP_AXP2101_ADDR 0x34

/* Features */
typedef enum {
    BSP_FEATURE_LCD,
    BSP_FEATURE_TOUCH,
    BSP_FEATURE_SD,
    BSP_FEATURE_SPEAKER,
} bsp_feature_t;

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_display_t *disp;
static lv_indev_t *disp_indev = NULL;
#endif                               // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static esp_lcd_touch_handle_t tp;    // LCD touch handle
sdmmc_card_t *bsp_sdcard    = NULL;  // Global SD card handler
static bool i2c_initialized = false;
static bool spi_initialized = false;

esp_err_t bsp_i2c_init(void)
{
    /* I2C was initialized before */
    if (i2c_initialized) {
        return ESP_OK;
    }

    const i2c_config_t i2c_conf = {.mode             = I2C_MODE_MASTER,
                                   .sda_io_num       = BSP_I2C_SDA,
                                   .sda_pullup_en    = GPIO_PULLUP_DISABLE,
                                   .scl_io_num       = BSP_I2C_SCL,
                                   .scl_pullup_en    = GPIO_PULLUP_DISABLE,
                                   .master.clk_speed = CONFIG_BSP_I2C_CLK_SPEED_HZ
                                  };
    BSP_ERROR_CHECK_RETURN_ERR(i2c_param_config(BSP_I2C_NUM, &i2c_conf));
    BSP_ERROR_CHECK_RETURN_ERR(i2c_driver_install(BSP_I2C_NUM, i2c_conf.mode, 0, 0, 0));

    i2c_initialized = true;

    return ESP_OK;
}

esp_err_t bsp_i2c_deinit(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(i2c_driver_delete(BSP_I2C_NUM));
    i2c_initialized = false;
    return ESP_OK;
}

uint8_t read8bit(uint8_t sub_addr)
{
    // Read register data
    uint8_t reg_data[1] = {0};
    esp_err_t err = i2c_master_write_read_device(BSP_I2C_NUM, BSP_AXP192_ADDR, &sub_addr, 1, reg_data, sizeof(reg_data), 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write & read register address: %s", esp_err_to_name(err));
    }
    ESP_LOGE(TAG, "AXP192 register %x: 0x%x", sub_addr, reg_data[0]);

    return reg_data[0];
}

static esp_err_t bsp_enable_feature(bsp_feature_t feature)
{
    esp_err_t err = ESP_OK;

    /* Initilize I2C */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());

    switch (feature) {
    case BSP_FEATURE_LCD://ldo2
    case BSP_FEATURE_TOUCH:
    case BSP_FEATURE_SD:
#if defined ( CONFIG_BSP_PMU_AXP2101 )
        /* AXP ALDO4 voltage / SD Card / Touch Pad / 3V3 */
        const uint8_t feature_ctr[] = {0x95, 0b00011100};  // axp: lcd logic and sdcard voltage preset to 3.3v
        err |= i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP2101_ADDR, feature_ctr, sizeof(feature_ctr),
                                          1000 / portTICK_PERIOD_MS);

#elif defined (CONFIG_BSP_PMU_AXP192)
        const uint8_t pmu_ldo2[] = {0x28, (read8bit(0x28) & 0x0f) | 0xf0}; // axp: lcd logic and sdcard voltage preset to 3.3v
        err |= i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP192_ADDR, pmu_ldo2, sizeof(pmu_ldo2),
                                          1000 / portTICK_PERIOD_MS);
#endif
        break;
    case BSP_FEATURE_SPEAKER:
#if defined ( CONFIG_BSP_PMU_AXP2101 )
        /* AXP ALDO3 voltage / Codec+Mic / 3V3 */
        const uint8_t spk_ctr[] = {0x94, 0b00011100};  // axp: lcd logic and sdcard voltage preset to 3.3v
        err |= i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP2101_ADDR, spk_ctr, sizeof(spk_ctr),
                                          1000 / portTICK_PERIOD_MS);
#elif defined (CONFIG_BSP_PMU_AXP192)
        const uint8_t led_gpio_set[] = {0x94, (read8bit(0x94) | 0x04)};
        ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP192_ADDR, led_gpio_set, sizeof(led_gpio_set),
                            1000 / portTICK_PERIOD_MS), TAG, "I2C write failed");
#endif
        break;
    }
    return err;
}

static esp_err_t bsp_spi_init(uint32_t max_transfer_sz)
{
    /* SPI was initialized before */
    if (spi_initialized) {
        return ESP_OK;
    }

    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num     = BSP_LCD_PCLK,
        .mosi_io_num     = BSP_LCD_MOSI,
        .miso_io_num     = BSP_LCD_MISO,
        .quadwp_io_num   = GPIO_NUM_NC,
        .quadhd_io_num   = GPIO_NUM_NC,
        .max_transfer_sz = max_transfer_sz,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(BSP_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    spi_initialized = true;

    return ESP_OK;
}

esp_err_t bsp_spiffs_mount(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path       = CONFIG_BSP_SPIFFS_MOUNT_POINT,
        .partition_label = CONFIG_BSP_SPIFFS_PARTITION_LABEL,
        .max_files       = CONFIG_BSP_SPIFFS_MAX_FILES,
#ifdef CONFIG_BSP_SPIFFS_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
    };

    esp_err_t ret_val = esp_vfs_spiffs_register(&conf);

    BSP_ERROR_CHECK_RETURN_ERR(ret_val);

    size_t total = 0, used = 0;
    ret_val = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret_val != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret_val));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    return ret_val;
}

esp_err_t bsp_spiffs_unmount(void)
{
    return esp_vfs_spiffs_unregister(CONFIG_BSP_SPIFFS_PARTITION_LABEL);
}

esp_err_t bsp_sdcard_mount(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(bsp_enable_feature(BSP_FEATURE_SD));

    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_BSP_SD_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files            = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host                 = SDSPI_HOST_DEFAULT();
    host.slot                         = BSP_LCD_SPI_NUM;
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs               = BSP_SD_CS;
    slot_config.host_id               = host.slot;

    ESP_RETURN_ON_ERROR(bsp_spi_init((BSP_LCD_H_RES * BSP_LCD_V_RES) * sizeof(uint16_t)), TAG, "");

    return esp_vfs_fat_sdspi_mount(BSP_SD_MOUNT_POINT, &host, &slot_config, &mount_config, &bsp_sdcard);
}

esp_err_t bsp_sdcard_unmount(void)
{
    return esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, bsp_sdcard);
}

esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void)
{
    const audio_codec_data_if_t *i2s_data_if = bsp_audio_get_codec_itf();
    if (i2s_data_if == NULL) {
        /* Initilize I2C */
        BSP_ERROR_CHECK_RETURN_NULL(bsp_i2c_init());
        /* Configure I2S peripheral and Power Amplifier */
        BSP_ERROR_CHECK_RETURN_NULL(bsp_audio_init(NULL));
        i2s_data_if = bsp_audio_get_codec_itf();
    }
    assert(i2s_data_if);

    BSP_ERROR_CHECK_RETURN_ERR(bsp_enable_feature(BSP_FEATURE_SPEAKER));

    esp_codec_dev_cfg_t codec_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = NULL,
        .data_if  = i2s_data_if,
    };
    return esp_codec_dev_new(&codec_dev_cfg);
}


// Bit number used to represent command and parameter
#define LCD_CMD_BITS   8
#define LCD_PARAM_BITS 8
#define LCD_LEDC_CH    CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH

esp_err_t bsp_display_brightness_init(void)
{
    /* Initilize I2C */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());

#if defined ( CONFIG_BSP_PMU_AXP2101 )
    ESP_LOGE("example", "AXP2101");
    const uint8_t lcd_bl_en[] = {0x90, 0x3F};  // AXP ALDO1~4 BLDO1~2 Enable
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP2101_ADDR, lcd_bl_en, sizeof(lcd_bl_en),
                        1000 / portTICK_PERIOD_MS),
                        TAG, "I2C write failed");
    const uint8_t lcd_bl_val[] = {0x96, 0b000011000};  // AXP BLDO1 voltage
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP2101_ADDR, lcd_bl_val, sizeof(lcd_bl_val),
                        1000 / portTICK_PERIOD_MS),
                        TAG, "I2C write failed");

    // AXP2101
    // PowerKey Hold=1sec / PowerOff=4sec IRQLEVEL/OFFLEVEL/ONLEVEL setting
    const uint8_t axp_pwr_val[] = {0x27, 0b00000000};
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP2101_ADDR, axp_pwr_val, sizeof(axp_pwr_val),
                        1000 / portTICK_PERIOD_MS),
                        TAG, "I2C write failed");
    // Internal off-discharge enable for DCDC & LDO & SWITCH
    const uint8_t axp_en_val[] = {0x10, 0b00110000};
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP2101_ADDR, axp_en_val, sizeof(axp_en_val),
                        1000 / portTICK_PERIOD_MS),
                        TAG, "I2C write failed");
    // BATFET disable
    const uint8_t axp_bat_val[] = {0x12, 0b00000000};
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP2101_ADDR, axp_bat_val, sizeof(axp_bat_val),
                        1000 / portTICK_PERIOD_MS),
                        TAG, "I2C write failed");

    // Battery detection enabled.
    const uint8_t axp_bte_val[] = {0x68, 0b00110000};
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP2101_ADDR, axp_bte_val, sizeof(axp_bte_val),
                        1000 / portTICK_PERIOD_MS),
                        TAG, "I2C write failed");
    // CHGLED setting
    const uint8_t axp_hgled_val[] = {0x69, 0b00010011};
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP2101_ADDR, axp_hgled_val, sizeof(axp_hgled_val),
                        1000 / portTICK_PERIOD_MS),
                        TAG, "I2C write failed");
    // DLDO1 set 0.5v (vibration motor)
    const uint8_t axp_vib_val[] = {0x99, 0b00000000};
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP2101_ADDR, axp_vib_val, sizeof(axp_vib_val),
                        1000 / portTICK_PERIOD_MS),
                        TAG, "I2C write failed");
#elif defined (CONFIG_BSP_PMU_AXP192)
    ESP_LOGE("example", "AXP192");
    read8bit(0x12);

    const uint8_t vbus_limit[] = {0x30, (read8bit(0x30) & 0x04) | 0x02}; // axp: vbus limit off
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP192_ADDR, vbus_limit, sizeof(vbus_limit),
                        1000 / portTICK_PERIOD_MS), TAG, "I2C write failed");
    const uint8_t pmu_gpio1[] = {0x92, read8bit(0x92) & 0xf8}; // AXP192 GPIO1:OD OUTPUT
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP192_ADDR, pmu_gpio1, sizeof(pmu_gpio1),
                        1000 / portTICK_PERIOD_MS), TAG, "I2C write failed");
    const uint8_t pmu_gpio2[] = {0x93, read8bit(0x93) & 0xf8}; // AXP192 GPIO2:OD OUTPUT
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP192_ADDR, pmu_gpio2, sizeof(pmu_gpio2),
                        1000 / portTICK_PERIOD_MS), TAG, "I2C write failed");
    const uint8_t pmu_gpio3[] = {0x35, (read8bit(0x35) & 0x1c) | 0xa2}; // AXP192 RTC CHG
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP192_ADDR, pmu_gpio3, sizeof(pmu_gpio3),
                        1000 / portTICK_PERIOD_MS), TAG, "I2C write failed");
    const uint8_t esp_vol[] = {0x26, 0x6a};  // AXP192 RTC CHG
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP192_ADDR, esp_vol, sizeof(esp_vol),
                        1000 / portTICK_PERIOD_MS), TAG, "I2C write failed");
    const uint8_t lcd_bl_val[] = {0x27, 0x68};  // Lcd backlight voltage
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP192_ADDR, lcd_bl_val, sizeof(lcd_bl_val),
                        1000 / portTICK_PERIOD_MS), TAG, "I2C write failed");
    const uint8_t pmu_ldo2[] = {0x28, (read8bit(0x28) & 0x0f) | 0xf0}; // axp: lcd logic and sdcard voltage preset to 3.3v
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP192_ADDR, pmu_ldo2, sizeof(pmu_ldo2),
                        1000 / portTICK_PERIOD_MS), TAG, "I2C write failed");
    const uint8_t pmu_ldo3[] = {0x28, (read8bit(0x28) & 0xf0) | 0x02}; // Vibrator power voltage preset
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP192_ADDR, pmu_ldo3, sizeof(pmu_ldo3),
                        1000 / portTICK_PERIOD_MS), TAG, "I2C write failed");
    const uint8_t pmu_ldo2_en[] = {0x12, (read8bit(0x12) & 0xfb) | 0x04}; // ldo2 enable
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP192_ADDR, pmu_ldo2_en, sizeof(pmu_ldo2_en),
                        1000 / portTICK_PERIOD_MS), TAG, "I2C write failed");
    read8bit(0x12);
    const uint8_t lcd_bl_en[] = {0x12, (read8bit(0x12) & 0xfd) | 0x02}; // Lcd backlight enable
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP192_ADDR, lcd_bl_en, sizeof(lcd_bl_en),
                        1000 / portTICK_PERIOD_MS), TAG, "I2C write failed");

    const uint8_t chg_current[] = {0x33, ((read8bit(0x33) & 0xf0)) | 0x0};
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP192_ADDR, chg_current, sizeof(chg_current),
                        1000 / portTICK_PERIOD_MS), TAG, "I2C write failed");

    const uint8_t apx_gpio4[] = {0x95, (read8bit(0x95) & 0x72) | 0x84};
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP192_ADDR, apx_gpio4, sizeof(apx_gpio4),
                        1000 / portTICK_PERIOD_MS), TAG, "I2C write failed");
    read8bit(0x95);
    const uint8_t pek_key[] = {0x36, 0x4c};  // Lcd backlight enable
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP192_ADDR, pek_key, sizeof(pek_key),
                        1000 / portTICK_PERIOD_MS), TAG, "I2C write failed");
    const uint8_t adc_en[] = {0x82, 0xff};  // Lcd backlight enable
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP192_ADDR, adc_en, sizeof(adc_en),
                        1000 / portTICK_PERIOD_MS), TAG, "I2C write failed");
    const uint8_t lcd_rset2[] = {0x96, read8bit(0x96) &(~0x02)}; // Lcd backlight enable
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP192_ADDR, lcd_rset2, sizeof(lcd_rset2),
                        1000 / portTICK_PERIOD_MS), TAG, "I2C write failed");
    vTaskDelay(pdMS_TO_TICKS(100));  // 延迟100ms
    const uint8_t lcd_rset[] = {0x96, read8bit(0x96) | 0x02}; // Lcd backlight enable
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP192_ADDR, lcd_rset, sizeof(lcd_rset),
                        1000 / portTICK_PERIOD_MS), TAG, "I2C write failed");
#endif
    return ESP_OK;
}

esp_err_t bsp_display_brightness_set(int brightness_percent)
{
    if (brightness_percent > 100) {
        brightness_percent = 100;
    }
    if (brightness_percent < 0) {
        brightness_percent = 0;
    }

    ESP_LOGI(TAG, "Setting LCD backlight: %d%%", brightness_percent);
#if defined ( CONFIG_BSP_PMU_AXP2101 )
    const uint8_t reg_val      = 20 + ((8 * brightness_percent) / 100);  // 0b00000 ~ 0b11100; under 20, it is too dark
    const uint8_t lcd_bl_val[] = {0x96, reg_val};                        // AXP DLDO1 voltage
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP2101_ADDR, lcd_bl_val, sizeof(lcd_bl_val),
                        1000 / portTICK_PERIOD_MS),
                        TAG, "I2C write failed");
#elif defined (CONFIG_BSP_PMU_AXP192)
    const uint8_t reg_val      = 90 + ((8 * brightness_percent) / 100);  // 0b00000 ~ 0b11100; under 20, it is too dark
    const uint8_t lcd_bl_val[] = {0x27, reg_val};                        // AXP DCDC3 voltage
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(BSP_I2C_NUM, BSP_AXP192_ADDR, lcd_bl_val, sizeof(lcd_bl_val),
                        1000 / portTICK_PERIOD_MS),
                        TAG, "I2C write failed");
#endif

    return ESP_OK;
}

esp_err_t bsp_display_backlight_off(void)
{
    return bsp_display_brightness_set(0);
}

esp_err_t bsp_display_backlight_on(void)
{
    return bsp_display_brightness_set(100);
}

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel,
                          esp_lcd_panel_io_handle_t *ret_io)
{
    esp_err_t ret = ESP_OK;
    assert(config != NULL && config->max_transfer_sz > 0);

    BSP_ERROR_CHECK_RETURN_ERR(bsp_enable_feature(BSP_FEATURE_LCD));

    /* Initialize SPI */
    ESP_RETURN_ON_ERROR(bsp_spi_init(config->max_transfer_sz), TAG, "");

    ESP_LOGI(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num       = BSP_LCD_DC,
        .cs_gpio_num       = BSP_LCD_CS,
        .pclk_hz           = BSP_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits      = LCD_CMD_BITS,
        .lcd_param_bits    = LCD_PARAM_BITS,
        .spi_mode          = 0,
        .trans_queue_depth = 10,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_NUM, &io_config, ret_io), err, TAG,
                      "New panel IO failed");

    ESP_LOGI(TAG, "Install LCD driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST,  // Shared with Touch reset
        .color_space    = BSP_LCD_COLOR_SPACE,
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL,
    };

    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_ili9341(*ret_io, &panel_config, ret_panel), err, TAG, "New panel failed");

    esp_lcd_panel_reset(*ret_panel);
    esp_lcd_panel_init(*ret_panel);
    esp_lcd_panel_invert_color(*ret_panel, true);
    return ret;

err:
    if (*ret_panel) {
        esp_lcd_panel_del(*ret_panel);
    }
    if (*ret_io) {
        esp_lcd_panel_io_del(*ret_io);
    }
    spi_bus_free(BSP_LCD_SPI_NUM);
    return ret;
}

esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch)
{
    BSP_ERROR_CHECK_RETURN_ERR(bsp_enable_feature(BSP_FEATURE_TOUCH));

    /* Initialize touch */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max        = BSP_LCD_H_RES,
        .y_max        = BSP_LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC,  // Shared with LCD reset
        .int_gpio_num = BSP_LCD_TOUCH_INT,
        .levels =
        {
            .reset     = 0,
            .interrupt = 0,
        },
        .flags =
        {
            .swap_xy  = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    esp_lcd_panel_io_handle_t tp_io_handle           = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)BSP_I2C_NUM, &tp_io_config, &tp_io_handle),
                        TAG, "");
    return esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, ret_touch);
}

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_display_t *bsp_display_lcd_init(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);
    esp_lcd_panel_io_handle_t io_handle     = NULL;
    esp_lcd_panel_handle_t panel_handle     = NULL;
    const bsp_display_config_t bsp_disp_cfg = {
        .max_transfer_sz = BSP_LCD_DRAW_BUFF_SIZE * sizeof(uint16_t),
    };
    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new(&bsp_disp_cfg, &panel_handle, &io_handle));

    esp_lcd_panel_disp_on_off(panel_handle, true);

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle     = io_handle,
        .panel_handle  = panel_handle,
        .buffer_size   = cfg->buffer_size,
        .double_buffer = cfg->double_buffer,
        .hres          = BSP_LCD_H_RES,
        .vres          = BSP_LCD_V_RES,
        .monochrome    = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation =
        {
            .swap_xy  = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma    = cfg->flags.buff_dma,
            .buff_spiram = cfg->flags.buff_spiram,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = (BSP_LCD_BIGENDIAN ? true : false),
#endif
        }
    };

    return lvgl_port_add_disp(&disp_cfg);
}

static lv_indev_t *bsp_display_indev_init(lv_display_t *disp)
{
    BSP_ERROR_CHECK_RETURN_NULL(bsp_touch_new(NULL, &tp));
    assert(tp);

    /* Add touch input (for selected screen) */
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp   = disp,
        .handle = tp,
    };

    return lvgl_port_add_touch(&touch_cfg);
}

lv_display_t *bsp_display_start(void)
{
    bsp_display_cfg_t cfg           = {.lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
                                       .buffer_size   = BSP_LCD_DRAW_BUFF_SIZE,
                                       .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
    .flags         = {
        .buff_dma    = true,
        .buff_spiram = false,
    }
                                      };
    cfg.lvgl_port_cfg.task_affinity = 1; /* For camera */
    return bsp_display_start_with_config(&cfg);
}

lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_init(&cfg->lvgl_port_cfg));

    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_brightness_init());

    BSP_NULL_CHECK(disp = bsp_display_lcd_init(cfg), NULL);

    BSP_NULL_CHECK(disp_indev = bsp_display_indev_init(disp), NULL);

    return disp;
}

lv_indev_t *bsp_display_get_input_dev(void)
{
    return disp_indev;
}

void bsp_display_rotate(lv_display_t *disp, lv_display_rotation_t rotation)
{
    lv_disp_set_rotation(disp, rotation);
}

bool bsp_display_lock(uint32_t timeout_ms)
{
    return lvgl_port_lock(timeout_ms);
}

void bsp_display_unlock(void)
{
    lvgl_port_unlock();
}

#endif  // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
