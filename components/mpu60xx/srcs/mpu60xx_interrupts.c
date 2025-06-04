/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * mpu60xx.c
 *
 *  Created on: 4-Jun-2025
 *      Author: Rohan Jeet <jeetrohan92@gmail.com>
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

#include "freertos/FreeRTOS.h"

#include "mpu60xx.h"
#include "mpu60xx_pvt.h"



esp_err_t mpu60xx_getMotionInterruptStatus(mpu60xx_handle_t mpu60xx, bool *status)
{
    ESP_RETURN_ON_FALSE((mpu60xx != NULL), ESP_ERR_INVALID_ARG, s_TAG, "mpu60xx handle is not initiated");
    mpu60xx_dev_t *mpu_handle = (mpu60xx_dev_t *) mpu60xx;
    ESP_RETURN_ON_ERROR (i2c_master_probe(mpu_handle->bus_handle, mpu_handle->i2c_address, 50), s_TAG, "mpu60xx not detected on i2c");

    uint8_t reg_value;

    ESP_RETURN_ON_ERROR ( mpu60xx_read_register(mpu_handle, MPU60xx_INT_STATUS, &reg_value, 1), s_TAG, "unable to read MPU60xx_INT_EN_REG register");

    reg_value = reg_value &  ((uint8_t)(64));
    if (reg_value) {
        *status = true;
    } else {
        *status = false;
    }

    return ESP_OK;

}

esp_err_t mpu60xx_Interrupt_pin_configuration (mpu60xx_handle_t mpu60xx, mpu60xx_intrpt_config_t *interrupt_configuration)
{
    ESP_RETURN_ON_FALSE((mpu60xx != NULL), ESP_ERR_INVALID_ARG, s_TAG, "mpu60xx handle is not initiated");
    mpu60xx_dev_t *mpu_handle = (mpu60xx_dev_t *) mpu60xx;
    ESP_RETURN_ON_ERROR (i2c_master_probe(mpu_handle->bus_handle, mpu_handle->i2c_address, 50), s_TAG, "mpu60xx not detected on i2c");

    uint8_t  cmd[2];

    if (NULL != interrupt_configuration) {
        if (GPIO_IS_VALID_GPIO(interrupt_configuration->interrupt_pin)) {
            // Set GPIO connected to mpu60xx INT pin only when user configures interrupts.
            // mpu60xx_dev_t *sensor_device = (mpu60xx_dev_t *) sensor;
            // sensor_device->int_pin = interrupt_configuration->interrupt_pin;
        } else {
            esp_err_t ret = ESP_ERR_INVALID_ARG;
            return ret;
        }
    }
    uint8_t int_pin_cfg = 0x00;

    ESP_RETURN_ON_ERROR ( mpu60xx_read_register(mpu_handle, MPU60xx_INT_PIN_CONFIG, &int_pin_cfg, 1), s_TAG, "unable to read MPU60xx interrupt pin configuration");


    int_pin_cfg &= (uint8_t)~(BIT4 | BIT5 | BIT6 | BIT7);

    if (MPU60XX_INTERRUPT_PIN_ACTIVE_LOW == interrupt_configuration->active_level) {
        int_pin_cfg |= BIT7;
    }

    if (MPU60XX_INTERRUPT_PIN_OPEN_DRAIN == interrupt_configuration->pin_mode) {
        int_pin_cfg |= BIT6;
    }

    if (MPU60XX_INTERRUPT_LATCH_UNTIL_CLEARED == interrupt_configuration->interrupt_latch) {
        int_pin_cfg |= BIT5;
    }

    if (MPU60XX_INTERRUPT_CLEAR_ON_ANY_READ == interrupt_configuration->interrupt_clear_behavior) {
        int_pin_cfg |= BIT4;
    }

    cmd[0] = MPU60xx_INT_PIN_CONFIG;
    cmd[1] = int_pin_cfg;
    ESP_RETURN_ON_ERROR (mpu60xx_write_register(mpu_handle, MPU60xx_INT_PIN_CONFIG, cmd, 2), s_TAG, "unable to configure MPU60xx interrupt pin behaviour");

    gpio_int_type_t gpio_intr_type;
    uint8_t pull_up = 0, pull_down = 0;

    if (MPU60XX_INTERRUPT_PIN_ACTIVE_LOW == interrupt_configuration->active_level) {
        gpio_intr_type = GPIO_INTR_NEGEDGE;
        pull_up = 1;
    } else {
        gpio_intr_type = GPIO_INTR_POSEDGE;
        pull_down = 1;
    }

    gpio_config_t int_gpio_config = {
        .mode = GPIO_MODE_INPUT,
        .intr_type = gpio_intr_type,
        .pin_bit_mask = (BIT0 << interrupt_configuration->interrupt_pin),
        .pull_up_en = pull_up,
        .pull_down_en = pull_down,
    };

    ESP_RETURN_ON_ERROR (gpio_config(&int_gpio_config), s_TAG, "unable to configure ESP32 GPIO interrupt pin behaviour");
    ESP_RETURN_ON_ERROR (register_isr(interrupt_configuration), s_TAG, "unable to register isr");
    return ESP_OK;
}


esp_err_t mpu60xx_en_EventDetection(mpu60xx_handle_t mpu60xx, mpu60xx_event_detect_config_t *mdc, mpu60xx_event_t event)
{
    ESP_RETURN_ON_FALSE((mpu60xx != NULL), ESP_ERR_INVALID_ARG, s_TAG, "mpu60xx handle is not initiated");
    mpu60xx_dev_t *mpu_handle = (mpu60xx_dev_t *) mpu60xx;
    ESP_RETURN_ON_ERROR (i2c_master_probe(mpu_handle->bus_handle, mpu_handle->i2c_address, 50), s_TAG, "mpu60xx not detected on i2c");

    uint8_t  cmd[2];

    uint8_t mot_thr_reg = 0, mot_dur_reg = 0;
    switch ( event ) {
    case MPU60XX_EVENT_MOTION_DETECT:
        mot_thr_reg = MPU60xx_MOT_THR;
        mot_dur_reg = MPU60xx_MOT_DUR;
        break;
    }

    cmd[0] = mot_thr_reg;
    cmd[1] = mdc->event_threshold;
    ESP_RETURN_ON_ERROR (mpu60xx_write_register(mpu_handle, mot_thr_reg, cmd, 2), s_TAG, "unable to write to mot_threshold");


    cmd[0] = mot_dur_reg;
    cmd[1] = mdc->event_duration;
    ESP_RETURN_ON_ERROR (mpu60xx_write_register(mpu_handle, mot_dur_reg, cmd, 2), s_TAG, "unable to write to mot_duration");

    en_DHPF(mpu_handle, mdc->dhpf_bw);
    ESP_RETURN_ON_ERROR (mpu60xx_enEventDetectInterrupt(mpu_handle, true, event), "", "");

    return ESP_OK;
}


esp_err_t mpu60xx_enEventDetectInterrupt(mpu60xx_handle_t mpu60xx, bool active, mpu60xx_event_t event)
{
    ESP_RETURN_ON_FALSE((mpu60xx != NULL), ESP_ERR_INVALID_ARG, s_TAG, "mpu60xx handle is not initiated");
    mpu60xx_dev_t *mpu_handle = (mpu60xx_dev_t *) mpu60xx;
    ESP_RETURN_ON_ERROR (i2c_master_probe(mpu_handle->bus_handle, mpu_handle->i2c_address, 50), s_TAG, "mpu60xx not detected on i2c");

    uint8_t reg_value, cmd[2];

    ESP_RETURN_ON_ERROR ( mpu60xx_read_register(mpu_handle, MPU60xx_INT_EN_REG, &reg_value, 1), s_TAG, "unable to read MPU60xx_INT_EN_REG  register");

    uint8_t event_bit = 0;
    switch ( event ) {
    case MPU60XX_EVENT_MOTION_DETECT:
        event_bit = BIT6;
        break;
    }

    if (active) {
        reg_value |= (event_bit);
    } else {
        reg_value = reg_value &  ((uint8_t)(~event_bit));
    }

    cmd[0] = MPU60xx_INT_EN_REG;
    cmd[1] = reg_value;

    ESP_RETURN_ON_ERROR ( mpu60xx_write_register(mpu_handle, MPU60xx_INT_EN_REG, cmd, 2), s_TAG, "unable to write to MPU60xx_INT_EN_REG register");

    return ESP_OK;
}
