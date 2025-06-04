/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * mpu60xx.h
 *
 *  Created on: 23-May-2025
 *      Author: Rohan Jeet <jeetrohan92@gmail.com>
 */

#ifndef __MPU_60XX_REGS_BITS_H
#define __MPU_60XX_REGS_BITS_H






#ifdef __cplusplus
extern "C" {
#endif

//MPU60XX Addresses
#define MPU60xx_ADDR            0x68    // MPU60XX I2C address
#define MPU60xx_ADDR_LOW            0x68    // MPU60XX I2C address
#define MPU60xx_ADDR_HIGH           0x69    // MPU60XX I2C address

//MPU60XX registers
#define MPU60xx_DEVICE_ID_REG       0X75    // Device ID Register

#define MPU60xx_USER_CTRL_REG       0X6A    // User Control Register
#define MPU60xx_PWR_MGMT1_REG       0X6B    // Power Management Registers 1
#define MPU60xx_PWR_MGMT2_REG       0X6C    // Power Management Registers 2

#define MPU60xx_SMPRT_DIV           0X19    // Sample Rate Divider
#define MPU60xx_CFG_REG             0X1A    // Configuration Register
#define MPU60xx_GYRO_CFG_REG        0X1B    // Gyroscope Configuration Register
#define MPU60xx_ACCEL_CFG_REG       0X1C    // Accelerometer Configuration Register

#define MPU60xx_MOT_THR             0x1F    // Motion detection threshold bits [7:0]
#define MPU60xx_MOT_DUR             0x20    // Duration counter, threshold for Motion detection int. 1 kHz rate, LSB = 1 ms

#define MPU60xx_INT_PIN_CONFIG      0X37    // Interrupt/Bypass Setting Register
#define MPU60xx_INT_EN_REG          0X38    // Interrupt Enable Register
#define MPU60xx_INT_STATUS          0x3A    // Interrupt status register

#define MPU60xx_ACCEL_XOUTH_REG     0X3B    // Accelerometer X-axis High Byte Register
#define MPU60xx_ACCEL_XOUTL_REG     0X3C    // Accelerometer X-axis Low Byte Register
#define MPU60xx_ACCEL_YOUTH_REG     0X3D    // Accelerometer Y-axis High Byte Register
#define MPU60xx_ACCEL_YOUTL_REG     0X3E    // Accelerometer Y-axis Low Byte Register
#define MPU60xx_ACCEL_ZOUTH_REG     0X3F    // Accelerometer Z-axis High Byte Register
#define MPU60xx_ACCEL_ZOUTL_REG     0X40    // Accelerometer Z-axis Low Byte Register

#define MPU60xx_TEMP_OUTH_REG       0X41    // Temperature Value High Byte Register
#define MPU60xx_TEMP_OUTL_REG       0X42    // Temperature Value Low Byte Register

#define MPU60xx_GYRO_XOUTH_REG      0X43    // Gyroscope X-axis High Byte Register
#define MPU60xx_GYRO_XOUTL_REG      0X44    // Gyroscope X-axis Low Byte Register
#define MPU60xx_GYRO_YOUTH_REG      0X45    // Gyroscope Y-axis High Byte Register
#define MPU60xx_GYRO_YOUTL_REG      0X46    // Gyroscope Y-axis Low Byte Register
#define MPU60xx_GYRO_ZOUTH_REG      0X47    // Gyroscope Z-axis High Byte Register
#define MPU60xx_GYRO_ZOUTL_REG      0X48    // Gyroscope Z-axis Low Byte Register
#define MPU60xx_SIGNAL_PATH_RESET   0x68    //Signal path reset register



/**
 * @brief Get accelerometer resolution
 *
 * @param[in] mpu_handle Handle of mpu60xx initialized by calling "mpu60xx_init"
 *
 * @param[out] range resolution read from accelerometer
 *
 * @return
 *      - ESP_OK: accelerometer resolution successfully read
 *      - other error codes : from the underlying i2c driver, check log
 */
esp_err_t getAccelerometerRange(mpu60xx_handle_t mpu_handle, mpu60xx_accel_range_t *range );

/**
 * @brief Set accelerometer resolution
 *
 * @param[in] mpu_handle Handle of mpu60xx initialized by calling "mpu60xx_init"
 *
 * @param[in] range resolution of accelerometer
 *
 * @return
 *      - ESP_OK: accelerometer resolution successfully set
 *      - other error codes : from the underlying i2c driver, check log
 */
esp_err_t mpu60xx_setAccelerometerRange(mpu60xx_handle_t mpu_handle, mpu60xx_accel_range_t range);

/**
 * @brief Get gyrometer resolution
 *
 * @param[in] mpu_handle Handle of mpu60xx initialized by calling "mpu60xx_init"
 *
 * @param[out] range resolution read from gyrometer
 *
 * @return
 *      - ESP_OK: gyrometer resolution successfully read
 *      - other error codes : from the underlying i2c driver, check log
 */
esp_err_t getGyrometerRange(mpu60xx_handle_t mpu_handle, mpu60xx_gyro_range_t *range );

/**
 * @brief Set gyrometer resolution
 *
 * @param[in] mpu_handle Handle of mpu60xx initialized by calling "mpu60xx_init"
 *
 * @param[in] range resolution of gyrometer
 *
 * @return
 *      - ESP_OK: gyrometer resolution successfully set
 *      - other error codes : from the underlying i2c driver, check log
 */
esp_err_t mpu60xx_setGyrometerRange(mpu60xx_handle_t mpu_handle, mpu60xx_gyro_range_t range);

/**
 * @brief Enable digital low pass filter on MPU60xx
 *
 * @param[in] mpu_handle Handle of mpu60xx initialized by calling "mpu60xx_init"
 *
 * @param[in] b_width low pass filter bandwidth options
 *
 * @return
 *      - ESP_OK: successfully enabled dlpf with provided b_width
 *      - other error codes : from the underlying i2c driver, check log
 */
esp_err_t en_DLPF( mpu60xx_handle_t mpu_handle, mpu60xx_lowpass_t b_width);

/**
 * @brief Enable digital high pass filter on MPU60xx
 *
 * @param[in] mpu_handle Handle of mpu60xx initialized by calling "mpu60xx_init"
 *
 * @param[in] b_width high pass filter bandwidth options
 *
 * @return
 *      - ESP_OK: successfully enabled dhpf with provided b_width
 *      - other error codes : from the underlying i2c driver, check log
 */
esp_err_t en_DHPF( mpu60xx_handle_t mpu_handle, mpu60xx_highpass_t bandwidth);



/**
 * @brief Set sampling rate
 * @note called by "mpu60xx_init",but provided for further flexibility
 *
 * @param[in] mpu_handle MPU60xx device handle
 *
 * @param[in] sample_rate Sampling rate of mpu60xx
 *
 * @param[in] dlpf_bw Low pass frequency
 * @note using value other than MPU60XX_BAND_260_HZ (default), puts an upper limit of 1KHz on 'sample_rate',
 * @note the default behavior allows 'sample_rate' upper limit of
 *       gyrometer as 8KHz and accelerometer readings as 1KHz.
 *       Corresponding accelerometer readings are repeated for samples, if 'sample_rate'>1KHz.
 *
 * @return
 *      - ESP_OK: sampling rate successfully set
 *      - other error codes : from the underlying i2c driver, check log
 */
esp_err_t mpu60xx_setSampleRate ( mpu60xx_handle_t mpu_handle, uint32_t sample_rate, mpu60xx_lowpass_t dlpf_bw);



#ifdef __cplusplus
}
#endif

#endif // __MPU_60XX_H
