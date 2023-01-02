/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief MPU6050 driver
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c.h"
#include "driver/gpio.h"

#define MPU6050_I2C_ADDRESS         0x68u /*!< I2C address with AD0 pin low */
#define MPU6050_I2C_ADDRESS_1       0x69u /*!< I2C address with AD0 pin high */
#define MPU6050_WHO_AM_I_VAL        0x68u

typedef enum {
    ACCE_FS_2G  = 0,     /*!< Accelerometer full scale range is +/- 2g */
    ACCE_FS_4G  = 1,     /*!< Accelerometer full scale range is +/- 4g */
    ACCE_FS_8G  = 2,     /*!< Accelerometer full scale range is +/- 8g */
    ACCE_FS_16G = 3,     /*!< Accelerometer full scale range is +/- 16g */
} mpu6050_acce_fs_t;

typedef enum {
    GYRO_FS_250DPS  = 0,     /*!< Gyroscope full scale range is +/- 250 degree per sencond */
    GYRO_FS_500DPS  = 1,     /*!< Gyroscope full scale range is +/- 500 degree per sencond */
    GYRO_FS_1000DPS = 2,     /*!< Gyroscope full scale range is +/- 1000 degree per sencond */
    GYRO_FS_2000DPS = 3,     /*!< Gyroscope full scale range is +/- 2000 degree per sencond */
} mpu6050_gyro_fs_t;

typedef enum {
    INTERRUPT_PIN_ACTIVE_HIGH = 0,
    INTERRUPT_PIN_ACTIVE_LOW  = 1
} mpu6050_int_pin_active_level_t;

typedef enum {
    INTERRUPT_PIN_PUSH_PULL   = 0,
    INTERRUPT_PIN_OPEN_DRAIN  = 1
} mpu6050_int_pin_mode_t;

typedef enum {
    INTERRUPT_LATCH_50US            = 0,
    INTERRUPT_LATCH_UNTIL_CLEARED   = 1
} mpu6050_int_latch_t;

typedef enum {
    INTERRUPT_CLEAR_ON_ANY_READ     = 0,
    INTERRUPT_CLEAR_ON_STATUS_READ  = 1
} mpu6050_int_clear_t;

typedef struct {
    gpio_num_t interrupt_pin;
    mpu6050_int_pin_active_level_t active_level;
    mpu6050_int_pin_mode_t pin_mode;
    mpu6050_int_latch_t interrupt_latch;
    mpu6050_int_clear_t interrupt_clear_behavior;
} mpu6050_int_config_t;

extern const uint8_t MPU6050_DATA_RDY_INT_BIT;
extern const uint8_t MPU6050_I2C_MASTER_INT_BIT;
extern const uint8_t MPU6050_FIFO_OVERFLOW_INT_BIT;
extern const uint8_t MPU6050_MOT_DETECT_INT_BIT;

typedef struct {
    int16_t raw_acce_x;
    int16_t raw_acce_y;
    int16_t raw_acce_z;
} mpu6050_raw_acce_value_t;

typedef struct {
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;
} mpu6050_raw_gyro_value_t;

typedef struct {
    float acce_x;
    float acce_y;
    float acce_z;
} mpu6050_acce_value_t;

typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
} mpu6050_gyro_value_t;

typedef struct {
    float temp;
} mpu6050_temp_value_t;


typedef struct {
    float roll;
    float pitch;
} complimentary_angle_t;

typedef void *mpu6050_handle_t;

typedef gpio_isr_t mpu6050_isr_t;

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
mpu6050_handle_t mpu6050_create(i2c_port_t port, const uint16_t dev_addr);

/**
 * @brief Delete and release a sensor object
 *
 * @param sensor object handle of mpu6050
 */
void mpu6050_delete(mpu6050_handle_t sensor);

/**
 * @brief Get device identification of MPU6050
 *
 * @param sensor object handle of mpu6050
 * @param deviceid a pointer of device ID
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6050_get_deviceid(mpu6050_handle_t sensor, uint8_t *const deviceid);

/**
 * @brief Wake up MPU6050
 *
 * @param sensor object handle of mpu6050
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6050_wake_up(mpu6050_handle_t sensor);

/**
 * @brief Enter sleep mode
 *
 * @param sensor object handle of mpu6050
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6050_sleep(mpu6050_handle_t sensor);

/**
 * @brief Set accelerometer and gyroscope full scale range
 *
 * @param sensor object handle of mpu6050
 * @param acce_fs accelerometer full scale range
 * @param gyro_fs gyroscope full scale range
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6050_config(mpu6050_handle_t sensor, const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs);

/**
 * @brief Get accelerometer sensitivity
 *
 * @param sensor object handle of mpu6050
 * @param acce_sensitivity accelerometer sensitivity
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6050_get_acce_sensitivity(mpu6050_handle_t sensor, float *const acce_sensitivity);

/**
 * @brief Get gyroscope sensitivity
 *
 * @param sensor object handle of mpu6050
 * @param gyro_sensitivity gyroscope sensitivity
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6050_get_gyro_sensitivity(mpu6050_handle_t sensor, float *const gyro_sensitivity);

esp_err_t mpu6050_config_interrupts(mpu6050_handle_t sensor, const mpu6050_int_config_t* const interrupt_configuration);

esp_err_t mpu6050_register_isr(mpu6050_handle_t sensor, const mpu6050_isr_t isr);

esp_err_t mpu6050_enable_interrupts(mpu6050_handle_t sensor, uint8_t interrupt_sources);

esp_err_t mpu6050_disable_interrupts(mpu6050_handle_t sensor, uint8_t interrupt_sources);

uint8_t mpu6050_is_data_ready_interrupt(uint8_t interrupt_status);

uint8_t mpu6050_is_i2c_master_interrupt(uint8_t interrupt_status);

uint8_t mpu6050_is_fifo_overflow_interrupt(uint8_t interrupt_status);

/**
 * @brief Read raw accelerometer measurements
 *
 * @param sensor object handle of mpu6050
 * @param raw_acce_value raw accelerometer measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6050_get_raw_acce(mpu6050_handle_t sensor, mpu6050_raw_acce_value_t *const raw_acce_value);

/**
 * @brief Read raw gyroscope measurements
 *
 * @param sensor object handle of mpu6050
 * @param raw_gyro_value raw gyroscope measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6050_get_raw_gyro(mpu6050_handle_t sensor, mpu6050_raw_gyro_value_t *const raw_gyro_value);

/**
 * @brief Read accelerometer measurements
 *
 * @param sensor object handle of mpu6050
 * @param acce_value accelerometer measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6050_get_acce(mpu6050_handle_t sensor, mpu6050_acce_value_t *const acce_value);

/**
 * @brief Read gyro values
 *
 * @param sensor object handle of mpu6050
 * @param gyro_value gyroscope measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6050_get_gyro(mpu6050_handle_t sensor, mpu6050_gyro_value_t *const gyro_value);

/**
 * @brief Read temperature values
 *
 * @param sensor object handle of mpu6050
 * @param temp_value temperature measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6050_get_temp(mpu6050_handle_t sensor, mpu6050_temp_value_t *const temp_value);

/**
 * @brief Use complimentory filter to calculate roll and pitch
 *
 * @param sensor object handle of mpu6050
 * @param acce_value accelerometer measurements
 * @param gyro_value gyroscope measurements
 * @param complimentary_angle complimentary angle
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6050_complimentory_filter(mpu6050_handle_t sensor, const mpu6050_acce_value_t *const acce_value,
                                       const mpu6050_gyro_value_t *const gyro_value, complimentary_angle_t *const complimentary_angle);

#ifdef __cplusplus
}
#endif
