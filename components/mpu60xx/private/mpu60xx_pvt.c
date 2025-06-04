/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * mpu60xx_pvt.c
 *
 *  Created on: 4-Jun-2025
 *      Author: rohan
 */
#include "mpu60xx_pvt.h"

esp_err_t mpu60xx_write_register(mpu60xx_dev_t *mpu_handle, uint8_t reg, const uint8_t *data, size_t length)
{
    esp_err_t ret = i2c_master_probe(mpu_handle->bus_handle, mpu_handle->i2c_address, 50);
    if (ret == ESP_OK) {
        ret = i2c_master_transmit(mpu_handle->i2c_dev_handle, data, length, 50);
    } else {
        ESP_LOGE(s_TAG, "mpu60xx not detected on i2c");
    }

    return ret;
}

esp_err_t mpu60xx_read_register(mpu60xx_dev_t *mpu_handle, const uint8_t data_addr,  uint8_t *data, size_t length)
{
    esp_err_t ret = i2c_master_probe(mpu_handle->bus_handle, mpu_handle->i2c_address, 50);
    if (ret == ESP_OK) {
        ret = i2c_master_transmit_receive(mpu_handle->i2c_dev_handle, (uint8_t *)&data_addr, 1, data, length, 50);
    } else {
        ESP_LOGE(s_TAG, "mpu60xx not detected on i2c");
    }

    return ret;
}

esp_err_t register_isr( mpu60xx_intrpt_config_t *interrupt_configuration)
{
    gpio_install_isr_service(0);
    ESP_RETURN_ON_ERROR ( gpio_isr_handler_add( interrupt_configuration->interrupt_pin, ((gpio_isr_t) * (interrupt_configuration->isr)), NULL), s_TAG, "unable to register isr");

    return ESP_OK;
}
