/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c.h"

#define ICM42670_I2C_ADDRESS         0x68 /*!< I2C address with AD0 pin low */
#define ICM42670_I2C_ADDRESS_1       0x69 /*!< I2C address with AD0 pin high */

typedef enum {
    ACCE_FS_16G = 0,     /*!< Accelerometer full scale range is +/- 16g */
    ACCE_FS_8G  = 1,     /*!< Accelerometer full scale range is +/- 8g */
    ACCE_FS_4G  = 2,     /*!< Accelerometer full scale range is +/- 4g */
    ACCE_FS_2G  = 3,     /*!< Accelerometer full scale range is +/- 2g */
} icm42670_acce_fs_t;

typedef enum {
    ACCE_PWR_OFF      = 0,     /*!< Accelerometer power off state */
    ACCE_PWR_ON       = 1,     /*!< Accelerometer power on state */
    ACCE_PWR_LOWPOWER = 2,     /*!< Accelerometer low-power mode */
    ACCE_PWR_LOWNOISE = 3,     /*!< Accelerometer low noise state */
} icm42670_acce_pwr_t;

typedef enum {
    ACCE_ODR_1600HZ   = 5,  /*!< Accelerometer ODR 1.6 kHz */
    ACCE_ODR_800HZ    = 6,  /*!< Accelerometer ODR 800 Hz */
    ACCE_ODR_400HZ    = 7,  /*!< Accelerometer ODR 400 Hz */
    ACCE_ODR_200HZ    = 8,  /*!< Accelerometer ODR 200 Hz */
    ACCE_ODR_100HZ    = 9,  /*!< Accelerometer ODR 100 Hz */
    ACCE_ODR_50HZ     = 10, /*!< Accelerometer ODR 50 Hz */
    ACCE_ODR_25HZ     = 11, /*!< Accelerometer ODR 25 Hz */
    ACCE_ODR_12_5HZ   = 12, /*!< Accelerometer ODR 12.5 Hz */
    ACCE_ODR_6_25HZ   = 13, /*!< Accelerometer ODR 6.25 Hz */
    ACCE_ODR_3_125HZ  = 14, /*!< Accelerometer ODR 3.125 Hz */
    ACCE_ODR_1_5625HZ = 15, /*!< Accelerometer ODR 1.5625 Hz */
} icm42670_acce_odr_t;

typedef enum {
    GYRO_FS_2000DPS = 0,     /*!< Gyroscope full scale range is +/- 2000 degree per sencond */
    GYRO_FS_1000DPS = 1,     /*!< Gyroscope full scale range is +/- 1000 degree per sencond */
    GYRO_FS_500DPS  = 2,     /*!< Gyroscope full scale range is +/- 500 degree per sencond */
    GYRO_FS_250DPS  = 3,     /*!< Gyroscope full scale range is +/- 250 degree per sencond */
} icm42670_gyro_fs_t;

typedef enum {
    GYRO_PWR_OFF      = 0,     /*!< Gyroscope power off state */
    GYRO_PWR_STANDBY  = 1,     /*!< Gyroscope power standby state */
    GYRO_PWR_LOWNOISE = 3,     /*!< Gyroscope power low noise state */
} icm42670_gyro_pwr_t;

typedef enum {
    GYRO_ODR_1600HZ = 5,  /*!< Gyroscope ODR 1.6 kHz */
    GYRO_ODR_800HZ  = 6,  /*!< Gyroscope ODR 800 Hz */
    GYRO_ODR_400HZ  = 7,  /*!< Gyroscope ODR 400 Hz */
    GYRO_ODR_200HZ  = 8,  /*!< Gyroscope ODR 200 Hz */
    GYRO_ODR_100HZ  = 9,  /*!< Gyroscope ODR 100 Hz */
    GYRO_ODR_50HZ   = 10, /*!< Gyroscope ODR 50 Hz */
    GYRO_ODR_25HZ   = 11, /*!< Gyroscope ODR 25 Hz */
    GYRO_ODR_12_5HZ = 12, /*!< Gyroscope ODR 12.5 Hz */
} icm42670_gyro_odr_t;

typedef struct {
    icm42670_acce_fs_t  acce_fs;    /*!< Accelerometer full scale range */
    icm42670_acce_odr_t acce_odr;   /*!< Accelerometer ODR selection */
    icm42670_gyro_fs_t  gyro_fs;    /*!< Gyroscope full scale range */
    icm42670_gyro_odr_t gyro_odr;   /*!< Gyroscope ODR selection */
} icm42670_cfg_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} icm42670_raw_value_t;

typedef struct {
    float x;
    float y;
    float z;
} icm42670_value_t;

typedef struct {
    float roll;
    float pitch;
} complimentary_angle_t;

typedef void *icm42670_handle_t;

/**
 * @brief Create and init sensor object and return a sensor handle
 *
 * @param port I2C port number
 * @param dev_addr I2C device address of sensor
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
icm42670_handle_t icm42670_create(i2c_port_t port, const uint8_t dev_addr);

/**
 * @brief Delete and release a sensor object
 *
 * @param sensor object handle of icm42670
 */
void icm42670_delete(icm42670_handle_t sensor);

/**
 * @brief Get device identification of ICM42670
 *
 * @param sensor object handle of icm42670
 * @param deviceid a pointer of device ID
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42670_get_deviceid(icm42670_handle_t sensor, uint8_t *deviceid);

/**
 * @brief Set accelerometer power mode
 *
 * @param sensor object handle of icm42670
 * @param state power mode of accelerometer
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42670_acce_set_pwr(icm42670_handle_t sensor, icm42670_acce_pwr_t state);

/**
 * @brief Set gyroscope power mode
 *
 * @param sensor object handle of icm42670
 * @param state power mode of gyroscope
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42670_gyro_set_pwr(icm42670_handle_t sensor, icm42670_gyro_pwr_t state);

/**
 * @brief Set accelerometer and gyroscope full scale range
 *
 * @param sensor object handle of icm42670
 * @param config Accelerometer and gyroscope configuration structure
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42670_config(icm42670_handle_t sensor, const icm42670_cfg_t *config);

/**
 * @brief Get accelerometer sensitivity
 *
 * @param sensor object handle of icm42670
 * @param sensitivity accelerometer sensitivity
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42670_get_acce_sensitivity(icm42670_handle_t sensor, float *sensitivity);

/**
 * @brief Get gyroscope sensitivity
 *
 * @param sensor object handle of icm42670
 * @param sensitivity gyroscope sensitivity
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42670_get_gyro_sensitivity(icm42670_handle_t sensor, float *sensitivity);

/**
 * @brief Read raw temperature measurements
 *
 * @param sensor object handle of icm42670
 * @param value raw temperature measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42670_get_temp_raw_value(icm42670_handle_t sensor, uint16_t *value);

/**
 * @brief Read raw accelerometer measurements
 *
 * @param sensor object handle of icm42670
 * @param value raw accelerometer measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42670_get_acce_raw_value(icm42670_handle_t sensor, icm42670_raw_value_t *value);

/**
 * @brief Read raw gyroscope measurements
 *
 * @param sensor object handle of icm42670
 * @param value raw gyroscope measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42670_get_gyro_raw_value(icm42670_handle_t sensor, icm42670_raw_value_t *value);

/**
 * @brief Read accelerometer measurements
 *
 * @param sensor object handle of icm42670
 * @param value accelerometer measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42670_get_acce_value(icm42670_handle_t sensor, icm42670_value_t *value);

/**
 * @brief Read gyro values
 *
 * @param sensor object handle of icm42670
 * @param value gyroscope measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42670_get_gyro_value(icm42670_handle_t sensor, icm42670_value_t *value);

/**
 * @brief Read temperature value
 *
 * @param sensor object handle of icm42670
 * @param value temperature measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42670_get_temp_value(icm42670_handle_t sensor, float *value);

/**
 * @brief use complimentory filter to caculate roll and pitch
 *
 * @param acce_value accelerometer measurements
 * @param gyro_value gyroscope measurements
 * @param complimentary_angle complimentary angle
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42670_complimentory_filter(icm42670_handle_t sensor, const icm42670_value_t *acce_value,
                                        const icm42670_value_t *gyro_value, complimentary_angle_t *complimentary_angle);

#ifdef __cplusplus
}
#endif
