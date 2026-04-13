/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c_types.h"
#include "driver/spi_master.h"

#define BMI270_CHIP_ID                      0x24
#define BMI270_I2C_ADDRESS_L                0x68
#define BMI270_I2C_ADDRESS_H                0x69

/**
 * @brief Selection for the BMI270 communication interface
 * @note The SPI interface is currently not supported
 */
typedef enum {
    BMI270_USE_I2C,
    BMI270_USE_SPI,
} bmi270_interface_e;

/**
 * @brief BMI270 accelerometer output data rates
 */
typedef enum {
    BMI270_ACC_ODR_0_78_HZ = 1,
    BMI270_ACC_ODR_1_5_HZ,
    BMI270_ACC_ODR_3_1_HZ,
    BMI270_ACC_ODR_6_25_HZ,
    BMI270_ACC_ODR_12_5_HZ,
    BMI270_ACC_ODR_25_HZ,
    BMI270_ACC_ODR_50_HZ,
    BMI270_ACC_ODR_100_HZ,
    BMI270_ACC_ODR_200_HZ,
    BMI270_ACC_ODR_400_HZ,
    BMI270_ACC_ODR_800_HZ,
    BMI270_ACC_ODR_1600_HZ
} bmi270_acce_odr_e;

/**
 * @brief BMI270 gyroscope output data rates
 */
typedef enum {
    BMI270_GYR_ODR_25_HZ = 6,
    BMI270_GYR_ODR_50_HZ,
    BMI270_GYR_ODR_100_HZ,
    BMI270_GYR_ODR_200_HZ,
    BMI270_GYR_ODR_400_HZ,
    BMI270_GYR_ODR_800_HZ,
    BMI270_GYR_ODR_1600_HZ,
    BMI270_GYR_ODR_3200_HZ
} bmi270_gyro_odr_e;

/**
 * @brief BMI270 accelerometer data ranges
 */
typedef enum {
    BMI270_ACC_RANGE_2_G = 0,
    BMI270_ACC_RANGE_4_G,
    BMI270_ACC_RANGE_8_G,
    BMI270_ACC_RANGE_16_G
} bmi270_acce_range_e;

/**
 * @brief BMI270 gyroscope data ranges
 */
typedef enum {
    BMI270_GYR_RANGE_2000_DPS = 0,
    BMI270_GYR_RANGE_1000_DPS,
    BMI270_GYR_RANGE_500_DPS,
    BMI270_GYR_RANGE_250_DPS,
    BMI270_GYR_RANGE_125_DPS
} bmi270_gyro_range_e;

/**
 * @brief Driver interface configuration structure
 * @note The SPI interface is currently not supported
 */
typedef struct {
    uint8_t addr;
    bmi270_interface_e interface;
    union {
        i2c_master_bus_handle_t i2c_bus;
    };
} bmi270_driver_config_t;

/**
 * @brief BMI270 data acquisition configuration
 */
typedef struct {
    bmi270_acce_odr_e acce_odr;
    bmi270_acce_range_e acce_range;
    bmi270_gyro_odr_e gyro_odr;
    bmi270_gyro_range_e gyro_range;
} bmi270_config_t;

/**
 * @brief BMI270 handle
 */
typedef struct {
    bool initialized;
    bmi270_interface_e interface;
    union {
        i2c_master_dev_handle_t i2c_handle;
        spi_device_handle_t spi_handle;
    };
    bmi270_acce_range_e acce_range;
    bmi270_gyro_range_e gyro_range;
} bmi270_handle_t;

/**
 * @brief Create and initialize the BMI270 sensor object
 *
 * @param[in]  config       BMI270 sensor driver configuration
 * @param[out] dev_handle   Pointer to the allocated memory for the BMI270 handle
 *
 * @return
 *     - ESP_OK             Success
 *     - ESP_ERR_NO_MEM     Memory allocation failure
 *     - ESP_ERR_NOT_FOUND  Device was not found on the data bus
 *     - Or other errors from the underlying I2C driver
 */
esp_err_t bmi270_create(const bmi270_driver_config_t *config, bmi270_handle_t **dev_handle);

/**
 * @brief Free and delete the BMI270 sensor object
 *
 * @param[in]  dev_handle   Pointer to an initialized BMI270 handle
 *
 * @return
 *     - ESP_OK             Success
 *     - Or other errors from the underlying I2C driver
 */
esp_err_t bmi270_delete(bmi270_handle_t *dev_handle);

/**
 * @brief Read the chip ID register
 *
 * @param[in]  dev_handle   Initialized BMI270 handle
 * @param[out] chip_id      Value read from the chip ID register
 *
 * @return
 *     - ESP_OK             Success
 *     - Or other errors from the underlying I2C driver
 */
esp_err_t bmi270_get_chip_id(const bmi270_handle_t *dev_handle, uint8_t *chip_id);

/**
 * @brief Start accelerometer, gyroscope and temperature sensor measurement
 *
 * @param[in]  dev_handle   Initialized BMI270 handle
 * @param[out] config       Configuration of data acquisition
 *
 * @return
 *     - ESP_OK             Success
 *     - Or other errors from the underlying I2C driver
 */
esp_err_t bmi270_start(bmi270_handle_t *dev_handle, const bmi270_config_t *config);

/**
 * @brief Stop accelerometer, gyroscope and temperature sensor measurement
 *
 * @param[in]  dev_handle   Initialized BMI270 handle
 *
 * @return
 *     - ESP_OK             Success
 *     - Or other errors from the underlying I2C driver
 */
esp_err_t bmi270_stop(const bmi270_handle_t *dev_handle);

/**
 * @brief Read out scaled accelerometer data
 *
 * @param[in]  dev_handle   Initialized BMI270 handle
 * @param[out] x            X axis accelerometer value in g
 * @param[out] y            Y axis accelerometer value in g
 * @param[out] z            Z axis accelerometer value in g
 *
 * @return
 *     - ESP_OK             Success
 *     - Or other errors from the underlying I2C driver
 */
esp_err_t bmi270_get_acce_data(const bmi270_handle_t *dev_handle, float *x, float *y, float *z);

/**
 * @brief Read out scaled gyroscope data
 *
 * @param[in]  dev_handle   Initialized BMI270 handle
 * @param[out] x            X axis gyroscope value in dps
 * @param[out] y            Y axis gyroscope value in dps
 * @param[out] z            Z axis gyroscope value in dps
 *
 * @return
 *     - ESP_OK             Success
 *     - Or other errors from the underlying I2C driver
 */
esp_err_t bmi270_get_gyro_data(const bmi270_handle_t *dev_handle, float *x, float *y, float *z);

/**
 * @brief Set accelerometer output data rate
 *
 * @param[in]  dev_handle   Initialized BMI270 handle
 * @param[in]  odr          Accelerometer output data rate
 *
 * @return
 *     - ESP_OK             Success
 *     - Or other errors from the underlying I2C driver
 */
esp_err_t bmi270_set_acce_odr(const bmi270_handle_t *dev_handle, bmi270_acce_odr_e odr);

/**
 * @brief Set gyroscope output data rate
 *
 * @param[in]  dev_handle   Initialized BMI270 handle
 * @param[in]  odr          Gyroscope output data rate
 *
 * @return
 *     - ESP_OK             Success
 *     - Or other errors from the underlying I2C driver
 */
esp_err_t bmi270_set_gyro_odr(const bmi270_handle_t *dev_handle, bmi270_gyro_odr_e odr);

/**
 * @brief Set accelerometer measurement range
 *
 * @param[in]  dev_handle   Initialized BMI270 handle
 * @param[in]  odr          Accelerometer measurement range
 *
 * @return
 *     - ESP_OK             Success
 *     - Or other errors from the underlying I2C driver
 */
esp_err_t bmi270_set_acce_range(bmi270_handle_t *dev_handle, bmi270_acce_range_e range);

/**
 * @brief Set gyroscope measurement range
 *
 * @param[in]  dev_handle   Initialized BMI270 handle
 * @param[in]  odr          Gyroscope measurement range
 *
 * @return
 *     - ESP_OK             Success
 *     - Or other errors from the underlying I2C driver
 */
esp_err_t bmi270_set_gyro_range(bmi270_handle_t *dev_handle, bmi270_gyro_range_e range);

#ifdef __cplusplus
}
#endif
