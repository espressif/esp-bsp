/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "soc/soc_caps.h"
#include "esp_check.h"
#include "esp_lcd_types.h"

#include "esp_lcd_st7796.h"
#include "esp_lcd_st7796_interface.h"

static const char *TAG = "st7796";

esp_err_t esp_lcd_new_panel_st7796(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config,
                                   esp_lcd_panel_handle_t *ret_panel)
{
    ESP_LOGI(TAG, "version: %d.%d.%d", ESP_LCD_ST7796_VER_MAJOR, ESP_LCD_ST7796_VER_MINOR, ESP_LCD_ST7796_VER_PATCH);
    ESP_RETURN_ON_FALSE(panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    st7796_vendor_config_t *vendor_config = (st7796_vendor_config_t *)panel_dev_config->vendor_config;

    esp_err_t ret = ESP_ERR_NOT_SUPPORTED;

    if (vendor_config && vendor_config->flags.use_mipi_interface) {
#if SOC_MIPI_DSI_SUPPORTED
        ret = esp_lcd_new_panel_st7796_mipi(io, panel_dev_config, ret_panel);
#else
        ESP_LOGE(TAG, "The chip does not support MIPI-DSI interface");
#endif
    } else {
        ret = esp_lcd_new_panel_st7796_general(io, panel_dev_config, ret_panel);
    }

    return ret;
}
