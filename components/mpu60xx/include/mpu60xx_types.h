/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * mpu60xx_types.h
 *
 *  Created on: 27-May-2025
 *      Author: Rohan Jeet <jeetrohan92@gmail.com>
 */

#ifndef __MPU_60XX_TYPES_H
#define __MPU_60XX_TYPES_H


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief magnetic sensor type raw readings
 *
 */
#ifndef AXIS3_RAW_T
typedef union {
    struct {
        int16_t x_raw; /*!< X axis raw reading */
        int16_t y_raw; /*!< Y axis raw reading */
        int16_t z_raw; /*!< Z axis raw reading */
    };
    int16_t axis_raw[3];
} axis3_raw_t;
#define AXIS3_RAW_T axis3_raw_t
#endif


/**
 * @brief imu sensor type scaled readings
 *
 */
#ifndef AXIS3_T
typedef union {
    struct {
        float x; /*!< x axis */
        float y; /*!< y axis */
        float z; /*!< z axis */
    };
    float axis[3];
} axis3_t;
#define AXIS3_T axis3_t
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
 * @brief Interrupt events from mpu60xx operations
 */
typedef enum {
    MPU60XX_EVENT_MOTION_DETECT,
} mpu60xx_event_t;

/**
 * @brief Interrupt events from mpu60xx operations
 */
typedef struct {
    bool motion_detect;
    bool zero_motion;
    bool free_fall;
} mpu60xx_event_status_t;

/**
 * @brief Interrupt configurations for mpu60xx operations
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
typedef struct {
    uint8_t event_threshold;    /*!< Specifies the Motion detection threshold. Unit of 1 LSB = 1mg. */
    uint8_t event_duration;     /*!< Specifies the duration counter threshold. Unit of 1 LSB = 1ms. */
    mpu60xx_highpass_t dhpf_bw;  /*!< Cutoff frequency for digital high pass filter. */
} mpu60xx_event_detect_config_t;

/**
 * @brief Structure to store raw values read from mpu60xx
 */
typedef struct {
    int16_t temp_raw;   /*!< raw temperature reading */
    axis3_raw_t accel; /*!< raw accelerometer reading */
    axis3_raw_t gyro; /*!< raw gyrometer reading */
} mpu60xx_reading_raw_t;

/*
 * @brief Structure to store scaled values read from mpu60xx
 */
typedef struct {
    float_t temperature; /*!< temperature reading °C */
    axis3_t accel; /*!< accelerometer reading m/s^2*/
    axis3_t gyro; /*!< gyrometer reading m/s^2 */
} mpu60xx_reading_t;

/**
 * @brief Minimal configurations for mpu60xx operations
 */
typedef struct {
    mpu60xx_gyro_range_t gyro_res;   /*!< Gyrometer resolution. */
    mpu60xx_accel_range_t accel_res; /*!< Accelerometer resolution. */
    bool temp_sensor;                /*!< Enable on-chip temperature sensor. Default is off. */
    mpu60xx_lowpass_t dlpf_bw;       /*!< Cutoff frequency for digital low pass filter. */
    uint32_t sample_rate;            /*!< Sampling rate. */
} mpu60xx_init_config_t;


typedef void *mpu60xx_handle_t;


#ifdef __cplusplus
}
#endif

#endif // __MPU_60XX_TYPES_H
