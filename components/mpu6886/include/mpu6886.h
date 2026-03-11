/*
 * SPDX-FileCopyrightText: 2026 Rashed Talukder
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief MPU6886 driver
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_idf_version.h"
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
#include "driver/i2c_master.h"
#else
#include "driver/i2c.h"
#endif
#include "driver/gpio.h"

#define MPU6886_I2C_ADDRESS         0x68u /*!< I2C address with AD0/SA0 pin low */
#define MPU6886_I2C_ADDRESS_1       0x69u /*!< I2C address with AD0/SA0 pin high */
#define MPU6886_WHO_AM_I_VAL        0x19u /*!< WHO_AM_I register default value */

#ifndef MPU6886_I2C_CLK_SPEED_HZ
#define MPU6886_I2C_CLK_SPEED_HZ   400000u /*!< Default I2C clock speed for MPU6886 (max 400 kHz) */
#endif

typedef enum {
    MPU6886_ACCE_FS_2G  = 0,     /*!< Accelerometer full scale range is +/- 2g */
    MPU6886_ACCE_FS_4G  = 1,     /*!< Accelerometer full scale range is +/- 4g */
    MPU6886_ACCE_FS_8G  = 2,     /*!< Accelerometer full scale range is +/- 8g */
    MPU6886_ACCE_FS_16G = 3,     /*!< Accelerometer full scale range is +/- 16g */
} mpu6886_acce_fs_t;

typedef enum {
    MPU6886_GYRO_FS_250DPS  = 0,     /*!< Gyroscope full scale range is +/- 250 degree per second */
    MPU6886_GYRO_FS_500DPS  = 1,     /*!< Gyroscope full scale range is +/- 500 degree per second */
    MPU6886_GYRO_FS_1000DPS = 2,     /*!< Gyroscope full scale range is +/- 1000 degree per second */
    MPU6886_GYRO_FS_2000DPS = 3,     /*!< Gyroscope full scale range is +/- 2000 degree per second */
} mpu6886_gyro_fs_t;

/** Gyroscope DLPF bandwidth (register CONFIG bits [2:0], when FCHOICE_B=0) */
typedef enum {
    MPU6886_DLPF_0 = 0,     /*!< Gyro BW 250 Hz, Rate 8 kHz */
    MPU6886_DLPF_1 = 1,     /*!< Gyro BW 176 Hz, Rate 1 kHz */
    MPU6886_DLPF_2 = 2,     /*!< Gyro BW 92 Hz,  Rate 1 kHz */
    MPU6886_DLPF_3 = 3,     /*!< Gyro BW 41 Hz,  Rate 1 kHz */
    MPU6886_DLPF_4 = 4,     /*!< Gyro BW 20 Hz,  Rate 1 kHz */
    MPU6886_DLPF_5 = 5,     /*!< Gyro BW 10 Hz,  Rate 1 kHz */
    MPU6886_DLPF_6 = 6,     /*!< Gyro BW 5 Hz,   Rate 1 kHz */
    MPU6886_DLPF_7 = 7,     /*!< Gyro BW 3281 Hz, Rate 8 kHz */
} mpu6886_dlpf_t;

/** Accelerometer DLPF bandwidth (register ACCEL_CONFIG2 bits [2:0], when ACCEL_FCHOICE_B=0) */
typedef enum {
    MPU6886_ACCE_DLPF_0 = 0, /*!< Accel BW 218.1 Hz, Rate 1 kHz */
    MPU6886_ACCE_DLPF_1 = 1, /*!< Accel BW 218.1 Hz, Rate 1 kHz */
    MPU6886_ACCE_DLPF_2 = 2, /*!< Accel BW 99.0 Hz,  Rate 1 kHz */
    MPU6886_ACCE_DLPF_3 = 3, /*!< Accel BW 44.8 Hz,  Rate 1 kHz */
    MPU6886_ACCE_DLPF_4 = 4, /*!< Accel BW 21.2 Hz,  Rate 1 kHz */
    MPU6886_ACCE_DLPF_5 = 5, /*!< Accel BW 10.2 Hz,  Rate 1 kHz */
    MPU6886_ACCE_DLPF_6 = 6, /*!< Accel BW 5.1 Hz,   Rate 1 kHz */
    MPU6886_ACCE_DLPF_7 = 7, /*!< Accel BW 420.0 Hz, Rate 1 kHz */
} mpu6886_acce_dlpf_t;

/** Clock source selection (register PWR_MGMT_1 bits [2:0]) */
typedef enum {
    MPU6886_CLK_INTERNAL_20MHZ = 0,  /*!< Internal 20 MHz oscillator */
    MPU6886_CLK_AUTO           = 1,  /*!< Auto select best available clock (PLL if ready, else internal) */
    MPU6886_CLK_STOP           = 7,  /*!< Stop clock, keep timing generator in reset */
} mpu6886_clk_src_t;

/** FSYNC pin data sampling location (register CONFIG bits [5:3]) */
typedef enum {
    MPU6886_FSYNC_DISABLED     = 0,  /*!< FSYNC function disabled */
    MPU6886_FSYNC_TEMP_OUT_L   = 1,  /*!< FSYNC on TEMP_OUT_L[0] */
    MPU6886_FSYNC_GYRO_XOUT_L  = 2,  /*!< FSYNC on GYRO_XOUT_L[0] */
    MPU6886_FSYNC_GYRO_YOUT_L  = 3,  /*!< FSYNC on GYRO_YOUT_L[0] */
    MPU6886_FSYNC_GYRO_ZOUT_L  = 4,  /*!< FSYNC on GYRO_ZOUT_L[0] */
    MPU6886_FSYNC_ACCEL_XOUT_L = 5,  /*!< FSYNC on ACCEL_XOUT_L[0] */
    MPU6886_FSYNC_ACCEL_YOUT_L = 6,  /*!< FSYNC on ACCEL_YOUT_L[0] */
    MPU6886_FSYNC_ACCEL_ZOUT_L = 7,  /*!< FSYNC on ACCEL_ZOUT_L[0] */
} mpu6886_fsync_out_t;

/** Averaging filter for low-power gyroscope mode (register LP_MODE_CFG bits [6:4]) */
typedef enum {
    MPU6886_GYRO_AVG_1   = 0,   /*!< Average 1 sample */
    MPU6886_GYRO_AVG_2   = 1,   /*!< Average 2 samples */
    MPU6886_GYRO_AVG_4   = 2,   /*!< Average 4 samples */
    MPU6886_GYRO_AVG_8   = 3,   /*!< Average 8 samples */
    MPU6886_GYRO_AVG_16  = 4,   /*!< Average 16 samples */
    MPU6886_GYRO_AVG_32  = 5,   /*!< Average 32 samples */
    MPU6886_GYRO_AVG_64  = 6,   /*!< Average 64 samples */
    MPU6886_GYRO_AVG_128 = 7,   /*!< Average 128 samples */
} mpu6886_gyro_avg_t;

/** Averaging filter for low-power accelerometer mode (register ACCEL_CONFIG2 bits [5:4]) */
typedef enum {
    MPU6886_ACCE_AVG_4  = 0,    /*!< Average 4 samples */
    MPU6886_ACCE_AVG_8  = 1,    /*!< Average 8 samples */
    MPU6886_ACCE_AVG_16 = 2,    /*!< Average 16 samples */
    MPU6886_ACCE_AVG_32 = 3,    /*!< Average 32 samples */
} mpu6886_acce_avg_t;

/** Wake-on-motion threshold comparison mode (register ACCEL_INTEL_CTRL bit [0]) */
typedef enum {
    MPU6886_WOM_OR  = 0,  /*!< Interrupt on OR of all enabled accel thresholds */
    MPU6886_WOM_AND = 1,  /*!< Interrupt on AND of all enabled accel thresholds */
} mpu6886_wom_threshold_mode_t;

typedef enum {
    MPU6886_INTERRUPT_PIN_ACTIVE_HIGH = 0,  /*!< INT pin is active HIGH */
    MPU6886_INTERRUPT_PIN_ACTIVE_LOW  = 1   /*!< INT pin is active LOW */
} mpu6886_int_pin_active_level_t;

typedef enum {
    MPU6886_INTERRUPT_PIN_PUSH_PULL   = 0,  /*!< INT pin is push-pull */
    MPU6886_INTERRUPT_PIN_OPEN_DRAIN  = 1   /*!< INT pin is open drain */
} mpu6886_int_pin_mode_t;

typedef enum {
    MPU6886_INTERRUPT_LATCH_50US            = 0, /*!< 50 microsecond pulse on interrupt */
    MPU6886_INTERRUPT_LATCH_UNTIL_CLEARED   = 1  /*!< Latched until cleared */
} mpu6886_int_latch_t;

typedef enum {
    MPU6886_INTERRUPT_CLEAR_ON_ANY_READ     = 0, /*!< INT_STATUS bits cleared on any register read */
    MPU6886_INTERRUPT_CLEAR_ON_STATUS_READ  = 1  /*!< INT_STATUS bits cleared only by reading INT_STATUS */
} mpu6886_int_clear_t;

typedef struct {
    gpio_num_t interrupt_pin;                           /*!< GPIO connected to MPU6886 INT pin       */
    mpu6886_int_pin_active_level_t active_level;        /*!< Active level of INT pin                 */
    mpu6886_int_pin_mode_t pin_mode;                    /*!< Push-pull or open drain mode            */
    mpu6886_int_latch_t interrupt_latch;                /*!< Interrupt pulse behavior of INT pin     */
    mpu6886_int_clear_t interrupt_clear_behavior;       /*!< Interrupt status clear behavior         */
} mpu6886_int_config_t;

extern const uint8_t MPU6886_DATA_RDY_INT_BIT;      /*!< DATA READY interrupt bit               */
extern const uint8_t MPU6886_GDRIVE_INT_BIT;        /*!< Gyroscope Drive System Ready interrupt  */
extern const uint8_t MPU6886_FIFO_OVERFLOW_INT_BIT;  /*!< FIFO Overflow interrupt bit            */
extern const uint8_t MPU6886_WOM_X_INT_BIT;          /*!< Wake-on-motion X interrupt bit         */
extern const uint8_t MPU6886_WOM_Y_INT_BIT;          /*!< Wake-on-motion Y interrupt bit         */
extern const uint8_t MPU6886_WOM_Z_INT_BIT;          /*!< Wake-on-motion Z interrupt bit         */
extern const uint8_t MPU6886_ALL_INTERRUPTS;         /*!< All interrupts supported by MPU6886    */

typedef struct {
    int16_t raw_acce_x;
    int16_t raw_acce_y;
    int16_t raw_acce_z;
} mpu6886_raw_acce_value_t;

typedef struct {
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;
} mpu6886_raw_gyro_value_t;

typedef struct {
    float acce_x;
    float acce_y;
    float acce_z;
} mpu6886_acce_value_t;

typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
} mpu6886_gyro_value_t;

typedef struct {
    float temp;
} mpu6886_temp_value_t;

typedef struct {
    float roll;
    float pitch;
} mpu6886_complementary_angle_t;

typedef void *mpu6886_handle_t;

typedef gpio_isr_t mpu6886_isr_t;

/**
 * @brief Create and init sensor object and return a sensor handle
 *
 * @param bus I2C master bus handle (IDF >= 5.2) or I2C port number (legacy)
 * @param dev_addr I2C device address of sensor (e.g. MPU6886_I2C_ADDRESS)
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
mpu6886_handle_t mpu6886_create(i2c_master_bus_handle_t bus, const uint16_t dev_addr);
#else
mpu6886_handle_t mpu6886_create(i2c_port_t port, const uint16_t dev_addr);
#endif

/**
 * @brief Delete and release a sensor object
 *
 * @param sensor object handle of mpu6886
 */
void mpu6886_delete(mpu6886_handle_t sensor);

/**
 * @brief Get device identification of MPU6886
 *
 * @param sensor object handle of mpu6886
 * @param deviceid a pointer of device ID
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_get_deviceid(mpu6886_handle_t sensor, uint8_t *const deviceid);

/**
 * @brief Reset the device (all registers restored to defaults)
 *
 * @param sensor object handle of mpu6886
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_reset(mpu6886_handle_t sensor);

/**
 * @brief Wake up MPU6886
 *
 * @param sensor object handle of mpu6886
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_wake_up(mpu6886_handle_t sensor);

/**
 * @brief Enter sleep mode
 *
 * @param sensor object handle of mpu6886
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_sleep(mpu6886_handle_t sensor);

/**
 * @brief Set clock source (CLKSEL must be set to 1 for full gyro performance)
 *
 * @param sensor object handle of mpu6886
 * @param clk_src clock source selection
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_set_clock_source(mpu6886_handle_t sensor, mpu6886_clk_src_t clk_src);

/**
 * @brief Set accelerometer and gyroscope full scale range
 *
 * @param sensor object handle of mpu6886
 * @param acce_fs accelerometer full scale range
 * @param gyro_fs gyroscope full scale range
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_config(mpu6886_handle_t sensor, const mpu6886_acce_fs_t acce_fs, const mpu6886_gyro_fs_t gyro_fs);

/**
 * @brief Get accelerometer sensitivity
 *
 * @param sensor object handle of mpu6886
 * @param acce_sensitivity accelerometer sensitivity
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_get_acce_sensitivity(mpu6886_handle_t sensor, float *const acce_sensitivity);

/**
 * @brief Get gyroscope sensitivity
 *
 * @param sensor object handle of mpu6886
 * @param gyro_sensitivity gyroscope sensitivity
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_get_gyro_sensitivity(mpu6886_handle_t sensor, float *const gyro_sensitivity);

/**
 * @brief Set sample rate divider. SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + divider)
 *        where INTERNAL_SAMPLE_RATE = 1 kHz. Only effective when FCHOICE_B = 0 and 0 < DLPF_CFG < 7.
 *
 * @param sensor object handle of mpu6886
 * @param divider sample rate divider (0-255)
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_set_sample_rate_divider(mpu6886_handle_t sensor, uint8_t divider);

/**
 * @brief Set gyroscope/temperature digital low-pass filter bandwidth
 *
 * @param sensor object handle of mpu6886
 * @param dlpf DLPF configuration value
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_set_gyro_dlpf(mpu6886_handle_t sensor, mpu6886_dlpf_t dlpf);

/**
 * @brief Set accelerometer digital low-pass filter bandwidth
 *
 * @param sensor object handle of mpu6886
 * @param dlpf accelerometer DLPF configuration value
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_set_acce_dlpf(mpu6886_handle_t sensor, mpu6886_acce_dlpf_t dlpf);

/**
 * @brief Configure FSYNC pin data sampling location
 *
 * @param sensor object handle of mpu6886
 * @param fsync_out FSYNC pin configuration
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_set_fsync(mpu6886_handle_t sensor, mpu6886_fsync_out_t fsync_out);

/**
 * @brief Configure gyroscope low-power mode
 *
 * @param sensor object handle of mpu6886
 * @param enable true to enable low-power gyro mode, false for low-noise mode
 * @param avg averaging filter configuration for low-power mode
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_set_gyro_low_power(mpu6886_handle_t sensor, bool enable, mpu6886_gyro_avg_t avg);

/**
 * @brief Set accelerometer averaging filter for low-power mode
 *
 * @param sensor object handle of mpu6886
 * @param avg averaging filter configuration
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_set_acce_averaging(mpu6886_handle_t sensor, mpu6886_acce_avg_t avg);

/**
 * @brief Configure accelerometer low-power cycle mode
 *
 * @param sensor object handle of mpu6886
 * @param enable true to enable accelerometer duty-cycle mode
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_set_acce_low_power_cycle(mpu6886_handle_t sensor, bool enable);

/**
 * @brief Configure wake-on-motion detection
 *
 * @param sensor object handle of mpu6886
 * @param wom_x_thr X-axis WoM threshold (0-255, in mg scaled by accel FSR)
 * @param wom_y_thr Y-axis WoM threshold (0-255)
 * @param wom_z_thr Z-axis WoM threshold (0-255)
 * @param mode threshold comparison mode (OR or AND of enabled axes)
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_config_wom(mpu6886_handle_t sensor, uint8_t wom_x_thr, uint8_t wom_y_thr,
                             uint8_t wom_z_thr, mpu6886_wom_threshold_mode_t mode);

/**
 * @brief Enable or disable individual accelerometer/gyroscope axes via PWR_MGMT_2
 *        Each bit disables the corresponding axis: XA(BIT5), YA(BIT4), ZA(BIT3), XG(BIT2), YG(BIT1), ZG(BIT0)
 *
 * @param sensor object handle of mpu6886
 * @param disable_mask bitmask of axes to disable (0 = all on, 0x3F = all off)
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_set_axes_disable(mpu6886_handle_t sensor, uint8_t disable_mask);

/**
 * @brief Disable temperature sensor
 *
 * @param sensor object handle of mpu6886
 * @param disable true to disable, false to enable
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_set_temp_disable(mpu6886_handle_t sensor, bool disable);

/**
 * @brief Put gyroscope in standby mode (drive and PLL enabled, sense paths disabled)
 *
 * @param sensor object handle of mpu6886
 * @param enable true to enable gyro standby, false to disable
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_set_gyro_standby(mpu6886_handle_t sensor, bool enable);

/**
 * @brief Enable FIFO and select which sensor data to buffer
 *
 * @param sensor object handle of mpu6886
 * @param enable_fifo true to enable FIFO, false to disable
 * @param gyro_fifo_en true to buffer gyro (and temp) data
 * @param acce_fifo_en true to buffer accel (and temp) data
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_enable_fifo(mpu6886_handle_t sensor, bool enable_fifo, bool gyro_fifo_en, bool acce_fifo_en);

/**
 * @brief Reset FIFO buffer
 *
 * @param sensor object handle of mpu6886
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_reset_fifo(mpu6886_handle_t sensor);

/**
 * @brief Get number of bytes currently in FIFO
 *
 * @param sensor object handle of mpu6886
 * @param[out] count number of bytes in FIFO
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_get_fifo_count(mpu6886_handle_t sensor, uint16_t *count);

/**
 * @brief Read data from FIFO buffer
 *
 * @param sensor object handle of mpu6886
 * @param buf buffer to store FIFO data
 * @param len number of bytes to read
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_read_fifo(mpu6886_handle_t sensor, uint8_t *buf, uint16_t len);

/**
 * @brief Set FIFO watermark threshold. Watermark interrupt disabled when threshold is 0.
 *
 * @param sensor object handle of mpu6886
 * @param threshold watermark threshold in bytes (0-1023)
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_set_fifo_watermark(mpu6886_handle_t sensor, uint16_t threshold);

/**
 * @brief Get FIFO watermark interrupt status
 *
 * @param sensor object handle of mpu6886
 * @param[out] status true if watermark reached
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_get_fifo_wm_status(mpu6886_handle_t sensor, bool *status);

/**
 * @brief Reset accelerometer and temperature digital signal paths
 *
 * @param sensor object handle of mpu6886
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_reset_signal_path(mpu6886_handle_t sensor);

/**
 * @brief Reset all sensor signal paths and sensor registers
 *
 * @param sensor object handle of mpu6886
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_reset_signal_cond(mpu6886_handle_t sensor);

/**
 * @brief Set OUTPUT_LIMIT bit (required per datasheet to avoid limiting sensor output)
 *        Should be called after every power-up.
 *
 * @param sensor object handle of mpu6886
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_set_output_limit(mpu6886_handle_t sensor);

/**
 * @brief Set gyroscope user offset (16-bit, 2's complement, added to sensor output)
 *
 * @param sensor object handle of mpu6886
 * @param x_offset X-axis gyro offset
 * @param y_offset Y-axis gyro offset
 * @param z_offset Z-axis gyro offset
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_set_gyro_offset(mpu6886_handle_t sensor, int16_t x_offset, int16_t y_offset, int16_t z_offset);

/**
 * @brief Set accelerometer offset (15-bit, 2's complement, 0.98-mg steps)
 *
 * @param sensor object handle of mpu6886
 * @param x_offset X-axis accel offset
 * @param y_offset Y-axis accel offset
 * @param z_offset Z-axis accel offset
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_set_acce_offset(mpu6886_handle_t sensor, int16_t x_offset, int16_t y_offset, int16_t z_offset);

/**
 * @brief Run accelerometer self-test and get results
 *
 * @param sensor object handle of mpu6886
 * @param[out] x_result X-axis self-test result (factory stored value)
 * @param[out] y_result Y-axis self-test result
 * @param[out] z_result Z-axis self-test result
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_get_acce_self_test(mpu6886_handle_t sensor, uint8_t *x_result, uint8_t *y_result, uint8_t *z_result);

/**
 * @brief Run gyroscope self-test and get results
 *
 * @param sensor object handle of mpu6886
 * @param[out] x_result X-axis self-test result (factory stored value)
 * @param[out] y_result Y-axis self-test result
 * @param[out] z_result Z-axis self-test result
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_get_gyro_self_test(mpu6886_handle_t sensor, uint8_t *x_result, uint8_t *y_result, uint8_t *z_result);

/**
 * @brief Configure INT pin behavior and setup target GPIO
 *
 * @param sensor object handle of mpu6886
 * @param interrupt_configuration INT pin configuration parameters
 *
 * @return
 *      - ESP_OK Success
 *      - ESP_ERR_INVALID_ARG A parameter is NULL or incorrect
 *      - ESP_FAIL Failed to configure INT pin
 */
esp_err_t mpu6886_config_interrupts(mpu6886_handle_t sensor, const mpu6886_int_config_t *const interrupt_configuration);

/**
 * @brief Register an ISR to handle mpu6886 interrupts
 *
 * @param sensor object handle of mpu6886
 * @param isr function to handle interrupts
 *
 * @return
 *      - ESP_OK Success
 *      - ESP_ERR_INVALID_ARG A parameter is NULL or not valid
 *      - ESP_FAIL Failed to register ISR
 */
esp_err_t mpu6886_register_isr(mpu6886_handle_t sensor, const mpu6886_isr_t isr);

/**
 * @brief Enable specific interrupts from mpu6886
 *
 * @param sensor object handle of mpu6886
 * @param interrupt_sources bit mask with interrupt sources to enable
 *
 * @return
 *      - ESP_OK Success
 *      - ESP_ERR_INVALID_ARG A parameter is NULL or not valid
 *      - ESP_FAIL Failed to enable interrupt sources
 */
esp_err_t mpu6886_enable_interrupts(mpu6886_handle_t sensor, uint8_t interrupt_sources);

/**
 * @brief Disable specific interrupts from mpu6886
 *
 * @param sensor object handle of mpu6886
 * @param interrupt_sources bit mask with interrupt sources to disable
 *
 * @return
 *      - ESP_OK Success
 *      - ESP_ERR_INVALID_ARG A parameter is NULL or not valid
 *      - ESP_FAIL Failed to disable interrupt sources
 */
esp_err_t mpu6886_disable_interrupts(mpu6886_handle_t sensor, uint8_t interrupt_sources);

/**
 * @brief Get the interrupt status of mpu6886
 *
 * @param sensor object handle of mpu6886
 * @param[out] out_intr_status bit mask representing active interrupts
 *
 * @return
 *      - ESP_OK Success
 *      - ESP_ERR_INVALID_ARG A parameter is NULL or not valid
 *      - ESP_FAIL Failed to retrieve interrupt status
 */
esp_err_t mpu6886_get_interrupt_status(mpu6886_handle_t sensor, uint8_t *const out_intr_status);

/**
 * @brief Read raw accelerometer measurements
 *
 * @param sensor object handle of mpu6886
 * @param raw_acce_value raw accelerometer measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_get_raw_acce(mpu6886_handle_t sensor, mpu6886_raw_acce_value_t *const raw_acce_value);

/**
 * @brief Read raw gyroscope measurements
 *
 * @param sensor object handle of mpu6886
 * @param raw_gyro_value raw gyroscope measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_get_raw_gyro(mpu6886_handle_t sensor, mpu6886_raw_gyro_value_t *const raw_gyro_value);

/**
 * @brief Read accelerometer measurements
 *
 * @param sensor object handle of mpu6886
 * @param acce_value accelerometer measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_get_acce(mpu6886_handle_t sensor, mpu6886_acce_value_t *const acce_value);

/**
 * @brief Read gyro values
 *
 * @param sensor object handle of mpu6886
 * @param gyro_value gyroscope measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_get_gyro(mpu6886_handle_t sensor, mpu6886_gyro_value_t *const gyro_value);

/**
 * @brief Read temperature value
 *
 * @param sensor object handle of mpu6886
 * @param temp_value temperature measurement
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_get_temp(mpu6886_handle_t sensor, mpu6886_temp_value_t *const temp_value);

/**
 * @brief Use complementary filter to calculate roll and pitch
 *
 * @param sensor object handle of mpu6886
 * @param acce_value accelerometer measurements
 * @param gyro_value gyroscope measurements
 * @param complementary_angle complementary angle
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6886_complementary_filter(mpu6886_handle_t sensor, const mpu6886_acce_value_t *const acce_value,
                                       const mpu6886_gyro_value_t *const gyro_value, mpu6886_complementary_angle_t *const complementary_angle);

#ifdef __cplusplus
}
#endif
