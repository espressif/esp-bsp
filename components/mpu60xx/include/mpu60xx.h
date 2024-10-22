/*
 * mpu60xx.h
 *
 *  Created on: 15-Oct-2024
 *      Author: rohan
 */
 
#ifndef __MPU_60XX_H
#define __MPU_60XX_H


#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"



#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Accelerometer range options
 *Allowed values for 'setAccelerometerRange'.
 */
typedef enum {
    MPU60XX_RANGE_2_G  = 0x00, /*!< +/- 2g (default) */   
    MPU60XX_RANGE_4_G  = 0x08, /*!< +/- 4g  */   
    MPU60XX_RANGE_8_G  = 0x10, /*!< +/- 8g  */  
    MPU60XX_RANGE_16_G = 0x18, /*!< +/- 16g */ 
} mpu60xx_accel_range_t;

/**
 * @brief Gyroscope range options
 * Allowed values for 'setGyrometerRange'.
 */
typedef enum {
    MPU60XX_RANGE_250_DEG  = 0x00,  /*< +/- 250 deg/s (default) */ 
    MPU60XX_RANGE_500_DEG  = 0x08,  /*< +/- 500 deg/s */  
    MPU60XX_RANGE_1000_DEG = 0x10,  /*< +/- 1000 deg/s */ 
    MPU60XX_RANGE_2000_DEG = 0x18,  /*< +/- 2000 deg/s */
} mpu60xx_gyro_range_t;

/**
 * @brief Digital low pass filter cutoff frequencies 
 * Allowed values for  'en_DLPF'.
 */
typedef enum {
  MPU60XX_BAND_260_HZ,  /*!< Default */
  MPU60XX_BAND_184_HZ,  /*!< 184 Hz */
  MPU60XX_BAND_94_HZ,   /*!<  94 Hz */
  MPU60XX_BAND_44_HZ,   /*!<  44 Hz */
  MPU60XX_BAND_21_HZ,   /*!<  21 Hz */
  MPU60XX_BAND_10_HZ,   /*!<  10 Hz */
  MPU60XX_BAND_5_HZ,    /*!<   5 Hz */
} mpu60xx_lowpass_t;

/**
 * @brief High pass filter cutoff frequencies 
 * Allowed values for 'en_DHPF'.
 */
typedef enum {
  MPU60XX_HIGHPASS_DISABLE, /*!< The filter output settles to zero within one sample. This effectively disables the high pass filter.*/
  MPU60XX_HIGHPASS_5_HZ,    /*!< 5 Hz */
  MPU60XX_HIGHPASS_2_5_HZ,  /*!< 2.5 Hz */
  MPU60XX_HIGHPASS_1_25_HZ, /*!< 1.25 Hz */
  MPU60XX_HIGHPASS_0_63_HZ, /*!< 0.63 Hz */
  MPU60XX_HIGHPASS_UNUSED_1,/*!< Unused value. */
  MPU60XX_HIGHPASS_UNUSED_2,/*!< Unused value. */
  MPU60XX_HIGHPASS_HOLD,    /*!< When triggered, the filter holds the present sample. The filter output will be the difference between the input sample and the held sample.  */
} mpu60xx_highpass_t;

/**
 * @brief Interrupt pin active levels on mpu60xx
 * Allowed values for 'active_level' in 'mpu60xx_intrpt_config_t'.
 */
typedef enum {
    MPU60XX_INTERRUPT_PIN_ACTIVE_HIGH = 0,          /*!<  The mpu60xx sets its INT pin HIGH on interrupt. */
    MPU60XX_INTERRUPT_PIN_ACTIVE_LOW  = 1           /*!<  The mpu60xx sets its INT pin LOW on interrupt.  */
} mpu60xx_int_pin_active_level_t;

/**
 * @brief Interrupt pin modes on mpu60xx
 * @note Allowed values for 'pin_mode' in 'mpu60xx_intrpt_config_t'.
 */
typedef enum {
    MPU60XX_INTERRUPT_PIN_PUSH_PULL   = 0,          /*!<  The mpu60xx configures its INT pin as push-pull.  */
    MPU60XX_INTERRUPT_PIN_OPEN_DRAIN  = 1           /*!<  The mpu60xx configures its INT pin as open drain. */
} mpu60xx_int_pin_mode_t;

/**
 * @brief Interrupt pulse behavior on mpu60xx
 *
 * Allowed values for 'interrupt_latch' in 'mpu60xx_intrpt_config_t'.
 */
typedef enum {
    MPU60XX_INTERRUPT_LATCH_50US            = 0,    /*!<  The mpu60xx produces a 50 microsecond pulse on interrupt. */
    MPU60XX_INTERRUPT_LATCH_UNTIL_CLEARED   = 1     /*!<  The mpu60xx latches its INT pin to its active level, until interrupt is cleared. */
} mpu60xx_int_latch_t;

/**
 * @brief Interrupt clear conditions on mpu60xx
 * @note Allowed values for 'interrupt_clear_behavior' in 'mpu60xx_intrpt_config_t'.
 */
typedef enum {
    MPU60XX_INTERRUPT_CLEAR_ON_ANY_READ     = 0,    /*!<  INT_STATUS register bits are cleared on any register read. */
    MPU60XX_INTERRUPT_CLEAR_ON_STATUS_READ  = 1     /*!<  INT_STATUS register bits are cleared only by reading INT_STATUS value. */
} mpu60xx_int_clear_t;

/**
 * @brief Interrupt configurations for mpu60xx operations
 * @note Implemented only for motion detection
 */
typedef struct {
    gpio_num_t interrupt_pin;                       /*!<   esp32 GPIO pin connected to MPU60xx INT pin */
    const gpio_isr_t isr;                           /*!<   ISR routine to handle the interrupt */
    mpu60xx_int_pin_active_level_t active_level;    /*!<   Active level of MPU60xx INT pin */
    mpu60xx_int_pin_mode_t pin_mode;                /*!<   Push-pull or open drain mode for MPU60xx INT pin */ 
    mpu60xx_int_latch_t interrupt_latch;            /*!<   The interrupt pulse behavior of MPU60xx INT pin */
    mpu60xx_int_clear_t interrupt_clear_behavior;   /*!<   MPU60xx Interrupt status clear behavior */
} mpu60xx_intrpt_config_t;

/**
 * @brief Motion detection configurations for mpu60xx 
 */
typedef struct{
    uint8_t motion_threshold;    /*!< Specifies the Motion detection threshold. Unit of 1 LSB = 1mg. */
    uint8_t motion_duration;     /*!< Specifies the duration counter threshold. Unit of 1 LSB = 1ms. */
    mpu60xx_highpass_t dhpf_bw;  /*!< Cutoff frequency for digital high pass filter. */
} mpu60xx_motion_detect_config_t;

/**
 * @brief Structure to store raw values read from mpu60xx
 */
typedef struct {
    int16_t
    temp_raw,   /*!< Last reading's temperature raw */     
    accX_raw,   /*!< Last reading's accelerometer X axis raw */ 
    accY_raw,   /*!< Last reading's accelerometer Y axis raw */
    accZ_raw,   /*!< Last reading's accelerometer Z axis raw */ 
    gyroX_raw,  /*!< Last reading's gyro X axis raw */ 
    gyroY_raw,  /*!< Last reading's gyro Y axis raw */
    gyroZ_raw;  /*!< Last reading's gyro Z axis raw */ 
} mpu60xx_reading_raw_t;


/**
 * @brief Structure to store float values read from mpu60xx
 */
typedef struct {
    float_t
    temperature,    /*!< Last reading's temperature (°C) */
    accX,   /*!< Last reading's accelerometer X axis m/s^2 */
    accY,   /*!< Last reading's accelerometer Y axis m/s^2 */
    accZ,   /*!< Last reading's accelerometer Z axis m/s^2 */
    gyroX,  /*!< Last reading's gyro X axis in degrees/s */
    gyroY,  /*!< Last reading's gyro Y axis in degrees/s */
    gyroZ;  /*!< Last reading's gyro Z axis in degrees/s */
} mpu60xx_reading_t;

/**
 * @brief Minimal configurations for mpu60xx operations
 */
typedef struct{
    uint32_t scl_speed_hz;           /*!< MPU60xx i2c clock speed. */  
    mpu60xx_gyro_range_t gyro_res;   /*!< Gyrometer resolution. */
    mpu60xx_accel_range_t accel_res; /*!< Accelerometer resolution. */
    bool temp_sensor;                /*!< Enable on-chip temperature sensor. Default is off. */
    mpu60xx_lowpass_t dlpf_bw;       /*!< Cutoff frequency for digital low pass filter. */
    uint32_t sample_rate;            /*!< Sampling rate. */
} mpu60xx_init_config_t;


/**
 * @brief Handle for mpu60xx operations
 */
typedef struct {
    i2c_master_bus_handle_t * bus_handle;   /*!< i2c bus handle.    */ 
    i2c_master_dev_handle_t * dev_handle;   /*!< i2c device handle. */
}mpu60xx_handle_t;


/**
 * @brief Set accelerometer resolution
 * @note called by "mpu60xx_init",but provided for further flexibility 
 *
 * @param[in] mpu_handle Handle of mpu60xx initialized by calling "mpu60xx_init"
 *
 * @param[in] range Accelerometer range options 
 *
 * @return  
 *      - ESP_OK: accelerometer resolution successfully set
 *      - other error codes : from the underlying i2c driver, check log   
 */
esp_err_t mpu60xx_setAccelerometerRange(mpu60xx_handle_t mpu_handle, mpu60xx_accel_range_t range);


/**
 * @brief Set gyrometer resolution
 * @note called by "mpu60xx_init",but provided for further flexibility 
 *
 * @param[in] mpu_handle Handle of mpu60xx initialized by calling "mpu60xx_init"
 *
 * @param[in] range Gyrometer range options 
 *
 * @return  
 *      - ESP_OK: gyrometer resolution successfully set
 *      - other error codes : from the underlying i2c driver, check log  
 *
 * 
 */
esp_err_t mpu60xx_setGyrometerRange(mpu60xx_handle_t mpu_handle, mpu60xx_gyro_range_t range);

/**
 * @brief Set sampling rate
 * @note called by "mpu60xx_init",but provided for further flexibility 
 *
 * @param[in] mpu_handle Handle of mpu60xx initialized by calling "mpu60xx_init"
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


/**
 * @brief enable motion detection
 *
 * @param[in] mpu_handle Handle of mpu60xx initialized by calling "mpu60xx_init"
 *
 * @param[in] mdc Configuration for motion detection
 *
 * @param[in] interrupt_configuration configuration for motion detection interrupt behavior
 * @note  can be NULL, use "mpu60xx_en_MotionDetection" for polling     
 *
 * @return  
 *      - ESP_OK: successfully read the Interrupt status register
 *      - other: failed to set value, check log 
 */
esp_err_t mpu60xx_en_MotionDetection(mpu60xx_handle_t * mpu_handle,mpu60xx_motion_detect_config_t *mdc, mpu60xx_intrpt_config_t* interrupt_configuration);

/**
 * @brief enable/disable motion detection interrupt generation on MPU60xx.
 * @note internally called by mpu60xx_en_MotionDetection but provided for flexibility.
 *
 * @param[in] mpu_handle Handle of mpu60xx initialized by calling "mpu60xx_init".
 *
 * @param[in] active if 'true' enables interrupt generation, 
 *                   if 'false' disables interrupt generation.
 *
 * @return  
 *      - ESP_OK: successfully read the Interrupt status register.
 *      - other: failed to set value, check log. 
 */
esp_err_t mpu60xx_enMotionDetectInterrupt(mpu60xx_handle_t * mpu_handle, bool active);

/**
 * @brief read motion detect interrupt status
 * @note if "MPU60XX_INTERRUPT_LATCH_UNTIL_CLEARED" was used in "mpu60xx_intrpt_config_t"
 *       this function needs to be called to clear the interrupt flag on mpu60xx,
 *       otherwise further interrupts will not be notified
 *
 * @param[in] mpu_handle Handle of mpu60xx initialized by calling "mpu60xx_init"
 *
 * @param[out] status 'true' if motion detected otherwise 'false'  
 *
 * @return  
 *      - ESP_OK: successfully read the Interrupt status register
 *      - other: failed to read value, check log 
 */
esp_err_t mpu60xx_getMotionInterruptStatus(mpu60xx_handle_t mpu_handle, bool * status);

/**
 * @brief initialize MPU60xx and also the handle used to interact with it.
 *
 * @param[in] config_handle initial configuration of MPU60xx
 *
 * @param[out] mpu_handle Handle of mpu60xx
 *
 * @return  
 *      - ESP_OK: successfully read the Interrupt status register
 *      - other: failed to read value, check log 
 */
esp_err_t mpu60xx_init(mpu60xx_init_config_t config_handle, mpu60xx_handle_t mpu_handle);

/**
 * @brief get float values of accelerometer, gyrometer and temperature sensor readings from MPU60xx
 *
 * @param[in] mpu_handle Handle of mpu60xx initialized by calling "mpu60xx_init"
 *
 * @param[out] sensor_read structure to hold the values read  
 *
 * @return  
 *      - ESP_OK: successfully read the Interrupt status register
 *      - other: failed to read value, check log 
 */
esp_err_t mpu60xx_read_sensor(mpu60xx_handle_t mpu_handle, mpu60xx_reading_t * sensor_read);

/**
 * @brief get raw values of accelerometer, gyrometer and temperature sensor readings from MPU60xx
 *
 * @param[in] mpu_handle Handle of mpu60xx initialized by calling "mpu60xx_init"
 *
 * @param[out] sensor_raw_read structure to hold the raw values read  
 *
 * @return  
 *      - ESP_OK: successfully read the Interrupt status register
 *      - other: failed to read value, check log 
 */
esp_err_t mpu60xx_read_sensor_raw (mpu60xx_handle_t mpu_handle, mpu60xx_reading_raw_t * sensor_raw_read);


#ifdef __cplusplus
}
#endif

#endif // __MPU_60XX_H
