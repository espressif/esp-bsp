/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_err_check.h"
#include "bsp/m5stack_core_s3.h"

static const char *TAG = "M5Stack Core S3";

static bool spi_initialized = false;

esp_err_t bsp_spi_init(const bsp_spi_cfg_t *config)
{
    /* SPI was initialized before */
    if (spi_initialized) {
        return ESP_OK;
    }

    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num = BSP_SD_SPI_CLK,
        .mosi_io_num = BSP_SD_SPI_MOSI,
        .miso_io_num = BSP_SD_SPI_MISO,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,

        .max_transfer_sz = config->max_transfer_sz,
    };
    BSP_ERROR_CHECK_RETURN_ERR(spi_bus_initialize(BSP_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO));
    spi_initialized = true;

    return ESP_OK;
}

esp_err_t bsp_spi_deinit(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(spi_bus_free(BSP_LCD_SPI_NUM));
    spi_initialized = false;
    return ESP_OK;
}
