/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * mpu60xx.h
 *
 *  Created on: 4-Jun-2025
 *      Author: rohan
 */

#ifndef __MPU_60XX_PVT_H
#define __MPU_60XX_PVT_H


#ifdef __cplusplus
extern "C" {
#endif

#include "mpu60xx.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

static const char *s_TAG = "MPU60xx"; // tag used in logs

/**
 * @brief Handle for mpu60xx operations
 */
typedef struct {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t i2c_dev_handle;    /*!< i2c device handle. */
    uint8_t i2c_address;
    mpu60xx_event_status_t event_status;
} mpu60xx_dev_t;


/**
 * @brief Write command to MPU60xx.
 *
 * @param[in] mpu_handle Handle of mpu60xx initialized by calling "mpu60xx_init"
 *
 * @param[in] reg MPU60xx register address to write to.
 *
 * @param[in] data Data bytes to send to MPU60xx.
 *
 * @param[in] length Size, in bytes, of data.
 *
 * @return
 *      - ESP_OK: gyrometer resolution successfully set
 *      - other error codes : from the underlying i2c driver, check log
 *
 * @note only values defined in "mpu60xx_gyro_range_t" are valid
 */
esp_err_t mpu60xx_write_register(mpu60xx_dev_t *mpu_handle, uint8_t reg, const uint8_t *data, size_t length);

/**
 * @brief Read data from MPU60xx
 *
 * @param[in] mpu_handle Handle of mpu60xx initialized by calling "mpu60xx_init"
 *
 * @param[in] data_addr MPU60xx register address to read from
 *
 * @param[out] data Data buffer store values read from the MPU60xx.
 *
 * @param[in] length Size, in bytes, of data.
 *
 * @return
 *      - ESP_OK: gyrometer resolution successfully set
 *      - other error codes : from the underlying i2c driver, check log
 *
 * @note only values defined in "mpu60xx_gyro_range_t" are valid
 */
esp_err_t mpu60xx_read_register(mpu60xx_dev_t *mpu_handle, const uint8_t data_addr,  uint8_t *data, size_t length);

/**
 * @brief Register ISR to handle interrupt from MPU60xx
 *
 * @param[in] interrupt_configuration configuration passed through "mpu60xx_intrpt_config_t"
 *
 * @return
 *      - ESP_OK: successfully registered ISR
 *      - other error codes : from the underlying i2c driver or ISR mechanism, check log
 */
esp_err_t register_isr(mpu60xx_intrpt_config_t *interrupt_configuration);

#ifdef __cplusplus
}
#endif

#endif // __MPU_60XX_H
