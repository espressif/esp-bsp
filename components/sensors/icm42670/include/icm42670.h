/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c_master.h"

#define ICM42670_I2C_ADDRESS         0x68 /*!< I2C address with AD0 pin low */
#define ICM42670_I2C_ADDRESS_1       0x69 /*!< I2C address with AD0 pin high */

#define ICM42670_SIGNAL_PATH_RESET   0x02 /*!< Signal path reset */
#define ICM42670_INT_CONFIG          0x06 /*!< Interrupt configuration */
#define ICM42670_PWR_MGMT0           0x1F /*!< Power management 0 */
#define ICM42670_APEX_CONFIG0        0x25 /*!< APEX configuration 0 */
#define ICM42670_APEX_CONFIG1        0x26 /*!< APEX configuration 1 */
#define ICM42670_WOM_CONFIG          0x27 /*!< Wake on Motion configuration */
#define ICM42670_INT_SOURCE0         0x2B /*!< Interrupt source 0 */
#define ICM42670_INT_SOURCE1         0x2C /*!< Interrupt source 1 */
#define ICM42670_INTF_CONFIG0        0x35 /*!< Interface configuration 0 */
#define ICM42670_INTF_CONFIG1        0x36 /*!< Interface configuration 1 */
#define ICM42670_INT_STATUS          0x3A /*!< Interrupt status */
#define ICM42670_INT_STATUS2         0x3B /*!< Interrupt status 2 */
#define ICM42670_INT_STATUS3         0x3C /*!< Interrupt status 3 */
#define ICM42670_BLK_SEL_W           0x79 /*! Select MREG1, MREG2, or MREG3 bank for writing */
#define ICM42670_MADDR_W             0x7A /*! Set MREG* register address for writing */
#define ICM42670_M_W                 0x7B /*! Write MREG* register value */
#define ICM42670_BLK_SEL_R           0x7C /*! Select MREG1, MREG2, or MREG3 bank for reading */
#define ICM42670_MADDR_R             0x7D /*! Set MREG* register address for reading */
#define ICM42670_M_R                 0x7E /*! Read MREG* register value */

// MREG1 Registers
#define ICM42670_MREG1_INT_CONFIG0      0x04 /*!< Interrupt configuration 0 */
#define ICM42670_MREG1_INT_CONFIG1      0x05 /*!< Interrupt configuration 1 */
#define ICM42670_MREG1_ACCEL_WOM_X_THR  0x4B /*!< WOM X threshold */
#define ICM42670_MREG1_ACCEL_WOM_Y_THR  0x4C /*!< WOM Y threshold */
#define ICM42670_MREG1_ACCEL_WOM_Z_THR  0x4D /*!< WOM Z threshold */


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
 * @brief Create and init sensor object
 *
 * @param[in]  i2c_bus    I2C bus handle. Obtained from i2c_new_master_bus()
 * @param[in]  dev_addr   I2C device address of sensor. Can be ICM42670_I2C_ADDRESS or ICM42670_I2C_ADDRESS_1
 * @param[out] handle_ret Handle to created ICM42670 driver object
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_NO_MEM Not enough memory for the driver
 *     - ESP_ERR_NOT_FOUND Sensor not found on the I2C bus
 *     - Others Error from underlying I2C driver
 */
esp_err_t icm42670_create(i2c_master_bus_handle_t i2c_bus, const uint8_t dev_addr, icm42670_handle_t *handle_ret);

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
 * @param value raw value of a temperature measurement in two's complement
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

/**
 * @brief Read a register
 *
 * @param sensor object handle of icm42670
 * @param reg register address
 * @param val value of the register
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42670_read_register(icm42670_handle_t sensor, uint8_t reg, uint8_t *val);

/**
 * @brief Write to a register
 *
 * @param sensor object handle of icm42670
 * @param reg register address
 * @param val value to write
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42670_write_register(icm42670_handle_t sensor, uint8_t reg, uint8_t val);

/**
 * @brief Read from a MREG register
 *
 * @param sensor object handle of icm42670
 * @param mreg which MREG bank to write (1-3)
 * @param reg register address
 * @param data data read
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_INVALID_ARG Invalid MREG
 *     - ESP_FAIL Fail
 */
esp_err_t icm42670_read_mreg_register(icm42670_handle_t sensor, uint8_t mreg, uint8_t reg,
                                      uint8_t *val);

/**
 * @brief Write to a MREG register
 *
 * @param sensor object handle of icm42670
 * @param mreg which MREG bank to write (1-3)
 * @param reg register address
 * @param data data to write
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_INVALID_ARG Invalid MREG
 *     - ESP_FAIL Fail
 */
esp_err_t icm42670_write_mreg_register(icm42670_handle_t sensor, uint8_t mreg, uint8_t reg,
                                       uint8_t val);

#ifdef __cplusplus
}
#endif
