/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief qma6100p driver
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c_master.h"
#include "driver/gpio.h"

#define QMA6100P_I2C_ADDRESS         0x12u /*!< I2C address with AD0 pin low */
#define QMA6100P_I2C_ADDRESS_1       0x13u /*!< I2C address with AD0 pin high */
#define QMA6100P_WHO_AM_I_VAL        0x90u

typedef enum {
    ACCE_FS_2G  = 0b0001,     /*!< Accelerometer full scale range is +/- 2g */
    ACCE_FS_4G  = 0b0010,     /*!< Accelerometer full scale range is +/- 4g */
    ACCE_FS_8G  = 0b0100,     /*!< Accelerometer full scale range is +/- 8g */
    ACCE_FS_16G = 0b1000,     /*!< Accelerometer full scale range is +/- 16g */
    ACCE_FS_32G = 0b1111,     /*!< Accelerometer full scale range is +/- 32g */
} qma6100p_acce_fs_t;

typedef enum {
    INTERRUPT_PIN_ACTIVE_HIGH = 0,          /*!< The qma6100p sets its INT pin HIGH on interrupt */
    INTERRUPT_PIN_ACTIVE_LOW  = 1           /*!< The qma6100p sets its INT pin LOW on interrupt */
} qma6100p_int_pin_active_level_t;

typedef enum {
    INTERRUPT_PIN_PUSH_PULL   = 0,          /*!< The qma6100p configures its INT pin as push-pull */
    INTERRUPT_PIN_OPEN_DRAIN  = 1           /*!< The qma6100p configures its INT pin as open drain */
} qma6100p_int_pin_mode_t;

typedef enum {
    INTERRUPT_NON_LATCH_MODE  = 0,          /*!< The qma6100p configures the INT to non-latch mode */
    INTERRUPT_LATCH_MODE      = 1           /*!< The qma6100p configures the INT to latch mode */
} qma6100p_int_latch_t;

typedef enum {
    INTERRUPT_CLEAR_ALL_INTERRUPTS = 0,     /*!< INT_STATUS register bits are cleared */
    INTERRUPT_CLEAR_LATCHED        = 1      /*!< INT_STATUS register bits are cleared only if latched */
} qma6100p_int_clear_t;

typedef gpio_isr_t qma6100p_isr_t;

typedef struct {
    gpio_num_t interrupt_pin;                          /*!< GPIO connected to qma6100p INT pin        */
    qma6100p_int_pin_active_level_t active_level_int;  /*!< Active level of qma6100p INT pin          */
    qma6100p_int_pin_mode_t pin_mode_int;              /*!< Push-pull or open drain mode for INT pin  */
    qma6100p_int_latch_t interrupt_latch;              /*!< The interrupt pulse behavior of INT pin   */
    qma6100p_int_clear_t interrupt_clear_behavior;     /*!< Interrupt status clear behavior           */
    qma6100p_isr_t isr;                                /*!< Interrupt handler                         */
    uint8_t interrupt_sources;                         /*!< Sources; use QMA6100P_*_INT_BIT macros    */
} qma6100p_int_config_t;

typedef enum {
    FIFO_BYPASS_MODE = 0,
    FIFO_FIFO_MODE   = 1,
    FIFO_STREAM_MODE = 2
} qma6100p_fifo_mode_t;

extern const uint8_t QMA6100P_DATA_RDY_INT_BIT;      /*!< DATA READY interrupt bit               */
extern const uint8_t QMA6100P_FIFO_FULL_INT_BIT;     /*!< FIFO Full interrupt bit                */
extern const uint8_t QMA6100P_FIFO_WM_INT_BIT;       /*!< FIFO Watermark interrupt bit           */
extern const uint8_t QMA6100P_FIFO_OF_INT_BIT;       /*!< FIFO Overflow interrupt bit            */
extern const uint8_t QMA6100P_ALL_INTERRUPTS;        /*!< All interrupts supported by qma6100p   */

typedef struct {
    int16_t raw_acce_x;
    int16_t raw_acce_y;
    int16_t raw_acce_z;
} qma6100p_raw_acce_value_t;

typedef struct {
    float acce_x;
    float acce_y;
    float acce_z;
} qma6100p_acce_value_t;

typedef void *qma6100p_handle_t;

/**
 * @brief Create and init sensor object
 *
 * @param[in]  i2c_bus    I2C bus handle. Obtained from i2c_new_master_bus()
 * @param[in]  dev_addr   I2C device address of sensor. Can be QMA6100P_I2C_ADDRESS or QMA6100P_I2C_ADDRESS_1
 * @param[out] handle_ret Handle to created QMA6100P driver object*
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_NO_MEM Not enough memory for the driver
 *     - ESP_ERR_NOT_FOUND Sensor not found on the I2C bus
 *     - Others Error from underlying I2C driver
 */
esp_err_t qma6100p_create(i2c_master_bus_handle_t i2c_bus, const uint8_t dev_addr, qma6100p_handle_t *handle_ret);

/**
 * @brief Delete and release a sensor object
 *
 * @param sensor object handle of qma6100p
 */
void qma6100p_delete(qma6100p_handle_t sensor);

/**
 * @brief Get device identification of qma6100p
 *
 * @param sensor object handle of qma6100p
 * @param deviceid a pointer of device ID
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t qma6100p_get_deviceid(qma6100p_handle_t sensor, uint8_t *const deviceid);

/**
 * @brief Load non volatile memory
 *
 * @param sensor object handle of qma6100p
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t qma6100p_nvm_load(qma6100p_handle_t sensor);

/**
 * @brief Wake up qma6100p
 *
 * @param sensor object handle of qma6100p
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t qma6100p_wake_up(qma6100p_handle_t sensor);

/**
 * @brief Enter sleep mode
 *
 * @param sensor object handle of qma6100p
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t qma6100p_sleep(qma6100p_handle_t sensor);

/**
 * @brief Set accelerometer full scale range
 *
 * @param sensor object handle of qma6100p
 * @param acce_fs accelerometer full scale range
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t qma6100p_config(qma6100p_handle_t sensor, const qma6100p_acce_fs_t acce_fs);

/**
 * @brief Get accelerometer sensitivity
 *
 * @param sensor object handle of qma6100p
 * @param acce_sensitivity accelerometer sensitivity
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t qma6100p_get_acce_sensitivity(qma6100p_handle_t sensor, float *const acce_sensitivity);

/**
 * @brief Configure INT pin behavior and setup target GPIO.
 *
 * @warning Not tested, implemented according to datasheet.
 *
 * @param sensor object handle of qma6100p
 * @param interrupt_configuration qma6100p INT pin configuration parameters
 *
 * @return
 *      - ESP_OK Success
 *      - ESP_ERR_INVALID_ARG A parameter is NULL or incorrect
 *      - ESP_FAIL Failed to configure INT pin on qma6100p
 */
esp_err_t qma6100p_config_interrupt(qma6100p_handle_t sensor, int int_num, const qma6100p_int_config_t *const interrupt_configuration);

/**
 * @brief Enable specific interrupts from qma6100p
 *
 * @param sensor object handle of qma6100p
 * @param interrupt_sources bit mask with interrupt sources to enable
 *
 * This function does not disable interrupts not set in interrupt_sources. To disable
 * specific qma6100p interrupts, use qma6100p_disable_interrupts().
 *
 * To enable all qma6100p interrupts, pass qma6100p_ALL_INTERRUPTS as the argument
 * for interrupt_sources.
 *
 * @return
 *      - ESP_OK Success
 *      - ESP_ERR_INVALID_ARG A parameter is NULL or not valid
 *      - ESP_FAIL Failed to enable interrupt sources on qma6100p
 */
esp_err_t qma6100p_enable_interrupts(qma6100p_handle_t sensor, uint8_t interrupt_sources);

/**
 * @brief Disable specific interrupts from qma6100p
 *
 * @param sensor object handle of qma6100p
 * @param interrupt_sources bit mask with interrupt sources to disable
 *
 * This function does not enable interrupts not set in interrupt_sources. To enable
 * specific qma6100p interrupts, use qma6100p_enable_interrupts().
 *
 * To disable all qma6100p interrupts, pass qma6100p_ALL_INTERRUPTS as the
 * argument for interrupt_sources.
 *
 * @return
 *      - ESP_OK Success
 *      - ESP_ERR_INVALID_ARG A parameter is NULL or not valid
 *      - ESP_FAIL Failed to enable interrupt sources on qma6100p
 */
esp_err_t qma6100p_disable_interrupts(qma6100p_handle_t sensor, uint8_t interrupt_sources);

/**
 * @brief Get the interrupt status of qma6100p
 *
 * @param sensor object handle of qma6100p
 * @param out_intr_status[out] bit mask that is assigned a value representing the interrupts triggered by the qma6100p
 *
 * This function can be used by the qma6100p ISR to determine the source of
 * the qma6100p interrupt that it is handling.
 *
 * After this function returns, the bits set in out_intr_status are
 * the sources of the latest interrupt triggered by the qma6100p. For example,
 * if QMA6100P_DATA_RDY_INT_BIT is set in out_intr_status, the last interrupt
 * from the qma6100p was a DATA READY interrupt.
 *
 * The behavior of the INT_STATUS register of the qma6100p may change depending on
 * the value of qma6100p_int_clear_t used on interrupt configuration.
 *
 * @return
 *      - ESP_OK Success
 *      - ESP_ERR_INVALID_ARG A parameter is NULL or not valid
 *      - ESP_FAIL Failed to retrieve interrupt status from qma6100p
 */
esp_err_t qma6100p_get_interrupt_status(qma6100p_handle_t sensor, uint8_t *const out_intr_status);

/**
 * @brief Determine if the last qma6100p interrupt was due to data ready.
 *
 * @param interrupt_status qma6100p interrupt status, obtained by invoking qma6100p_get_interrupt_status()
 *
 * @return
 *      - 0: The interrupt was not produced due to data ready
 *      - Any other positive integer: Interrupt was a DATA_READY interrupt
 */
extern uint8_t qma6100p_is_data_ready_interrupt(uint8_t interrupt_status);

/**
 * @brief Determine if the last qma6100p interrupt was triggered by a fifo overflow.
 *
 * @param interrupt_status qma6100p interrupt status, obtained by invoking qma6100p_get_interrupt_status()
 *
 * @return
 *      - 0: The interrupt is not a fifo overflow interrupt
 *      - Any other positive integer: Interrupt was triggered by a fifo overflow
 */
extern uint8_t qma6100p_is_fifo_overflow_interrupt(uint8_t interrupt_status);

/**
 * @brief Determine if the last qma6100p interrupt was triggered due to fifo full.
 *
 * @param interrupt_status qma6100p interrupt status, obtained by invoking qma6100p_get_interrupt_status()
 *
 * @return
 *      - 0: The interrupt is not a fifo full interrupt
 *      - Any other positive integer: Interrupt was triggered due to fifo full
 */
extern uint8_t qma6100p_is_fifo_full_interrupt(uint8_t interrupt_status);

/**
 * @brief Read raw accelerometer measurements
 *
 * @param sensor object handle of qma6100p
 * @param raw_acce_value raw accelerometer measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t qma6100p_get_raw_acce(qma6100p_handle_t sensor, qma6100p_raw_acce_value_t *const raw_acce_value);

/**
 * @brief Read accelerometer measurements
 *
 * @param sensor object handle of qma6100p
 * @param acce_value accelerometer measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t qma6100p_get_acce(qma6100p_handle_t sensor, qma6100p_acce_value_t *const acce_value);

/**
 * @brief Get fifo frame counter, 64 is full
 *
 * @param sensor object handle of qma6100p
 * @param counter frame counter
 *
 * @return
 *      - ESP_OK Success
 *      - ESP_FAIL Fail
 */
esp_err_t qma6100p_get_fifo_frame_counter(qma6100p_handle_t sensor, uint8_t *counter);

/**
 * @brief Get fifo data pointer
 *
 * @param sensor object handle of qma6100p
 * @param counter frame counter
 *
 * @return
 *      - ESP_OK Success
 *      - ESP_FAIL Fail
 */
esp_err_t qma6100p_get_fifo_data(qma6100p_handle_t sensor, uint8_t *data);

#ifdef __cplusplus
}
#endif
