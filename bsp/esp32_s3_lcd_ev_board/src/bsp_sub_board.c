/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_check.h"
#include "esp_io_expander_tca9554.h"
#include "esp_lcd_gc9503.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_io_additions.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_touch_ft5x06.h"
#include "esp_lcd_touch_gt1151.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include "sdkconfig.h"
#include "bsp_err_check.h"
#include "bsp_probe.h"
#include "bsp/display.h"
#include "bsp/esp32_s3_lcd_ev_board.h"
#include "bsp/touch.h"

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 1, 2)
#warning "Due to significant updates of the RGB LCD drivers, it's recommended to develop using ESP-IDF v5.1.2 or later"
#endif

#if CONFIG_ESP32S3_DATA_CACHE_LINE_64B && !(CONFIG_SPIRAM_SPEED_120M || CONFIG_BSP_LCD_RGB_BOUNCE_BUFFER_MODE)
#warning "Enabling the `ESP32S3_DATA_CACHE_LINE_64B` configuration when the PSRAM speed is not set to 120MHz (`SPIRAM_SPEED_120M`) and the LCD is not in bounce buffer mode (`BSP_LCD_RGB_BOUNCE_BUFFER_MODE`) may result in screen drift, please enable `ESP32S3_DATA_CACHE_LINE_32B` instead"
#endif

static const char *TAG = "bsp_sub_board";

/**************************************************************************************************
 *
 * Display Panel Function
 *
 **************************************************************************************************/

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel, esp_lcd_panel_io_handle_t *ret_io)
{
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 1, 2)
    ESP_LOGW(TAG, "Due to significant updates of the RGB LCD drivers, it's recommended to develop using ESP-IDF v5.1.2 or later");
#endif

#if CONFIG_ESP32S3_DATA_CACHE_LINE_64B && !(CONFIG_SPIRAM_SPEED_120M || CONFIG_BSP_LCD_RGB_BOUNCE_BUFFER_MODE)
    ESP_LOGW(TAG, "Enabling the `ESP32S3_DATA_CACHE_LINE_64B` configuration when the PSRAM speed is not set to 120MHz \
(`SPIRAM_SPEED_120M`) and the LCD is not in bounce buffer mode (`BSP_LCD_RGB_BOUNCE_BUFFER_MODE`) may result in screen \
drift, please enable `ESP32S3_DATA_CACHE_LINE_32B` instead");
#endif

    esp_io_expander_handle_t expander = NULL;
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_io_handle_t io_handle = NULL;

    bsp_module_type_t module_type = bsp_probe_module_type();
    if (module_type == MODULE_TYPE_UNKNOW) {
        ESP_LOGE(TAG, "Unknow module type");
        return ESP_FAIL;
    }

    bsp_sub_board_type_t sub_board_type = bsp_probe_sub_board_type();
    if (sub_board_type == SUB_BOARD_TYPE_UNKNOW) {
        ESP_LOGE(TAG, "Unknow sub-board type");
        return ESP_FAIL;
    }

    switch (sub_board_type) {
    case SUB_BOARD_TYPE_2_480_480: {
        // For the latest version of the sub-board 2, it's necessary to set `BSP_LCD_SUB_BOARD_2_3_VSYNC` pin to a high level before initializing the LCD
        // This is a specific requirement of this LCD module and not apply to all LCD modules
        gpio_config_t io_conf = {
            .pin_bit_mask = BIT64(BSP_LCD_SUB_BOARD_2_3_VSYNC),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = true,
        };
        gpio_config(&io_conf);
        gpio_set_level(BSP_LCD_SUB_BOARD_2_3_VSYNC, 1);

        BSP_NULL_CHECK(expander = bsp_io_expander_init(), ESP_FAIL);
        ESP_LOGI(TAG, "Install panel IO");
        spi_line_config_t line_config = {
            .cs_io_type = IO_TYPE_EXPANDER,
            .cs_expander_pin = BSP_LCD_SUB_BOARD_2_SPI_CS,
            .scl_io_type = IO_TYPE_EXPANDER,
            .scl_expander_pin = BSP_LCD_SUB_BOARD_2_SPI_SCK,
            .sda_io_type = IO_TYPE_EXPANDER,
            .sda_expander_pin = BSP_LCD_SUB_BOARD_2_SPI_SDO,
            .io_expander = expander,
        };
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 2)
        esp_lcd_panel_io_3wire_spi_config_t io_config = GC9503_PANEL_IO_3WIRE_SPI_CONFIG(
                    line_config, SUB_BOARD2_480_480_PANEL_SCL_ACTIVE_EDGE);
#else
        esp_lcd_panel_io_3wire_spi_config_t io_config = GC9503_PANEL_IO_3WIRE_SPI_CONFIG(line_config);
#endif
        BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_new_panel_io_3wire_spi(&io_config, &io_handle));

        ESP_LOGI(TAG, "Initialize RGB panel");
        esp_lcd_rgb_panel_config_t rgb_conf = {
            .clk_src = LCD_CLK_SRC_PLL160M,
            .dma_burst_size = 64,
            .data_width = 16,
            .bits_per_pixel = 16,
            .de_gpio_num = BSP_LCD_SUB_BOARD_2_3_DE,
            .pclk_gpio_num = BSP_LCD_SUB_BOARD_2_3_PCLK,
            .vsync_gpio_num = BSP_LCD_SUB_BOARD_2_3_VSYNC,
            .hsync_gpio_num = BSP_LCD_SUB_BOARD_2_3_HSYNC,
            .disp_gpio_num = BSP_LCD_SUB_BOARD_2_3_DISP,
            .data_gpio_nums = {
                BSP_LCD_SUB_BOARD_2_3_DATA0,
                BSP_LCD_SUB_BOARD_2_3_DATA1,
                BSP_LCD_SUB_BOARD_2_3_DATA2,
                BSP_LCD_SUB_BOARD_2_3_DATA3,
                BSP_LCD_SUB_BOARD_2_3_DATA4,
                BSP_LCD_SUB_BOARD_2_3_DATA5,
                BSP_LCD_SUB_BOARD_2_3_DATA6,
                BSP_LCD_SUB_BOARD_2_3_DATA7,
                BSP_LCD_SUB_BOARD_2_3_DATA8,
                BSP_LCD_SUB_BOARD_2_3_DATA9,
                BSP_LCD_SUB_BOARD_2_3_DATA10,
                BSP_LCD_SUB_BOARD_2_3_DATA11,
                BSP_LCD_SUB_BOARD_2_3_DATA12,
                BSP_LCD_SUB_BOARD_2_3_DATA13,
                BSP_LCD_SUB_BOARD_2_3_DATA14,
                BSP_LCD_SUB_BOARD_2_3_DATA15,
            },
            .timings = SUB_BOARD2_480_480_PANEL_60HZ_RGB_TIMING(),
            .flags.fb_in_psram = 1,
            .num_fbs = CONFIG_BSP_LCD_RGB_BUFFER_NUMS,
#if CONFIG_BSP_LCD_RGB_BOUNCE_BUFFER_MODE
            .bounce_buffer_size_px = BSP_LCD_SUB_BOARD_2_H_RES * CONFIG_BSP_LCD_RGB_BOUNCE_BUFFER_HEIGHT,
#endif
        };
        // To compatible with ESP32-S3-WROOM-N16R16V module
        if (module_type == MODULE_TYPE_R16) {
            rgb_conf.data_gpio_nums[6] = BSP_LCD_SUB_BOARD_2_3_DATA6_R16;
            rgb_conf.data_gpio_nums[7] = BSP_LCD_SUB_BOARD_2_3_DATA7_R16;
        }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 2)
        gc9503_vendor_config_t vendor_config = {
            .rgb_config = &rgb_conf,
            .flags = {
                .mirror_by_cmd = 0,
                .auto_del_panel_io = 1,
            },
        };
        const esp_lcd_panel_dev_config_t panel_conf = {
            .reset_gpio_num = -1,
            .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
            .bits_per_pixel = 18,
            .vendor_config = &vendor_config,
        };
        BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_new_panel_gc9503(io_handle, &panel_conf, &panel_handle));
#else
        BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_new_panel_gc9503(io_handle, &rgb_conf, &panel_handle));
#endif
        break;
    }
    case SUB_BOARD_TYPE_3_800_480: {
        ESP_LOGI(TAG, "Initialize RGB panel");
        esp_lcd_rgb_panel_config_t panel_conf = {
            .clk_src = LCD_CLK_SRC_PLL160M,
            .dma_burst_size = 64,
            .data_width = 16,
            .bits_per_pixel = 16,
            .de_gpio_num = BSP_LCD_SUB_BOARD_2_3_DE,
            .pclk_gpio_num = BSP_LCD_SUB_BOARD_2_3_PCLK,
            .vsync_gpio_num = BSP_LCD_SUB_BOARD_2_3_VSYNC,
            .hsync_gpio_num = BSP_LCD_SUB_BOARD_2_3_HSYNC,
            .disp_gpio_num = BSP_LCD_SUB_BOARD_2_3_DISP,
            .data_gpio_nums = {
                BSP_LCD_SUB_BOARD_2_3_DATA0,
                BSP_LCD_SUB_BOARD_2_3_DATA1,
                BSP_LCD_SUB_BOARD_2_3_DATA2,
                BSP_LCD_SUB_BOARD_2_3_DATA3,
                BSP_LCD_SUB_BOARD_2_3_DATA4,
                BSP_LCD_SUB_BOARD_2_3_DATA5,
                BSP_LCD_SUB_BOARD_2_3_DATA6,
                BSP_LCD_SUB_BOARD_2_3_DATA7,
                BSP_LCD_SUB_BOARD_2_3_DATA8,
                BSP_LCD_SUB_BOARD_2_3_DATA9,
                BSP_LCD_SUB_BOARD_2_3_DATA10,
                BSP_LCD_SUB_BOARD_2_3_DATA11,
                BSP_LCD_SUB_BOARD_2_3_DATA12,
                BSP_LCD_SUB_BOARD_2_3_DATA13,
                BSP_LCD_SUB_BOARD_2_3_DATA14,
                BSP_LCD_SUB_BOARD_2_3_DATA15,
            },
            .timings = SUB_BOARD3_800_480_PANEL_35HZ_RGB_TIMING(),
            .flags.fb_in_psram = 1,
            .num_fbs = CONFIG_BSP_LCD_RGB_BUFFER_NUMS,
#if CONFIG_BSP_LCD_RGB_BOUNCE_BUFFER_MODE
            .bounce_buffer_size_px = BSP_LCD_SUB_BOARD_3_H_RES * CONFIG_BSP_LCD_RGB_BOUNCE_BUFFER_HEIGHT,
#endif
        };
        // To compatible with ESP32-S3-WROOM-N16R16V module
        if (module_type == MODULE_TYPE_R16) {
            panel_conf.data_gpio_nums[6] = BSP_LCD_SUB_BOARD_2_3_DATA6_R16;
            panel_conf.data_gpio_nums[7] = BSP_LCD_SUB_BOARD_2_3_DATA7_R16;
        }
        BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_new_rgb_panel(&panel_conf, &panel_handle));
        break;
    }
    default:
        break;
    }
    BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_panel_init(panel_handle));

    if (ret_panel) {
        *ret_panel = panel_handle;
    }
    if (ret_io) {
        *ret_io = io_handle;
    }

    return ESP_OK;
}

/**************************************************************************************************
 *
 * Touch Panel Function
 *
 **************************************************************************************************/
esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch)
{
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_touch_handle_t tp_handle = NULL;

    bsp_sub_board_type_t sub_board_type = bsp_probe_sub_board_type();
    if (sub_board_type == SUB_BOARD_TYPE_UNKNOW) {
        ESP_LOGE(TAG, "Unknow sub-board type");
        return ESP_FAIL;
    }

    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());
    i2c_master_bus_handle_t i2c_handle = bsp_i2c_get_handle();

    switch (sub_board_type) {
    case SUB_BOARD_TYPE_2_480_480: {
        esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
        tp_io_config.scl_speed_hz = CONFIG_BSP_I2C_CLK_SPEED_HZ; // This parameter was introduced together with I2C Driver-NG in IDF v5.2
        const esp_lcd_touch_config_t tp_cfg = {
            .x_max = BSP_LCD_SUB_BOARD_2_H_RES,
            .y_max = BSP_LCD_SUB_BOARD_2_V_RES,
            .rst_gpio_num = GPIO_NUM_NC,
            .int_gpio_num = GPIO_NUM_NC,
            .levels = {
                .reset = 0,
                .interrupt = 0,
            },
            .flags = {
                .swap_xy = 0,
                .mirror_x = 0,
                .mirror_y = 0,
            },
        };
        BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_new_panel_io_i2c(i2c_handle, &tp_io_config, &tp_io_handle));
        BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, &tp_handle));
        break;
    }
    case SUB_BOARD_TYPE_3_800_480: {
        esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT1151_CONFIG();
        tp_io_config.scl_speed_hz = CONFIG_BSP_I2C_CLK_SPEED_HZ; // This parameter was introduced together with I2C Driver-NG in IDF v5.2
        const esp_lcd_touch_config_t tp_cfg = {
            .x_max = BSP_LCD_SUB_BOARD_3_H_RES,
            .y_max = BSP_LCD_SUB_BOARD_3_V_RES,
            .rst_gpio_num = GPIO_NUM_NC,
            .int_gpio_num = GPIO_NUM_NC,
            .levels = {
                .reset = 0,
                .interrupt = 0,
            },
            .flags = {
                .swap_xy = 0,
                .mirror_x = 0,
                .mirror_y = 0,
            },
        };

        BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_new_panel_io_i2c(i2c_handle, &tp_io_config, &tp_io_handle));
        BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_touch_new_i2c_gt1151(tp_io_handle, &tp_cfg, &tp_handle));
        break;
    }
    default:
        break;
    }

    if (ret_touch) {
        *ret_touch = tp_handle;
    }

    return ESP_OK;
}

/**************************************************************************************************
 *
 * Other Function
 *
 **************************************************************************************************/
uint16_t bsp_display_get_h_res(void)
{
    bsp_sub_board_type_t sub_board_type = bsp_probe_sub_board_type();
    if (sub_board_type == SUB_BOARD_TYPE_UNKNOW) {
        ESP_LOGE(TAG, "Unknow sub-board type");
        return ESP_FAIL;
    }

    switch (sub_board_type) {
    case SUB_BOARD_TYPE_2_480_480:
        return BSP_LCD_SUB_BOARD_2_H_RES;
    case SUB_BOARD_TYPE_3_800_480:
        return BSP_LCD_SUB_BOARD_3_H_RES;
    default:
        ESP_LOGE(TAG, "Failed to get horizontal resolution, unknow sub-board");
        return 0;
    }
}

uint16_t bsp_display_get_v_res(void)
{
    bsp_sub_board_type_t sub_board_type = bsp_probe_sub_board_type();
    if (sub_board_type == SUB_BOARD_TYPE_UNKNOW) {
        ESP_LOGE(TAG, "Unknow sub-board type");
        return ESP_FAIL;
    }

    switch (sub_board_type) {
    case SUB_BOARD_TYPE_2_480_480:
        return BSP_LCD_SUB_BOARD_2_V_RES;
    case SUB_BOARD_TYPE_3_800_480:
        return BSP_LCD_SUB_BOARD_3_V_RES;
    default:
        ESP_LOGE(TAG, "Failed to get vertical resolution, unknow sub-board");
        return 0;
    }
}
