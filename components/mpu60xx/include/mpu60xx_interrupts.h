/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * mpu60xx.h
 *
 *  Created on: 4-Jun-2025
 *      Author: Rohan Jeet <jeetrohan9@gmail.com>
 */

#ifndef __MPU_60XX_INTERRUPTS_H
#define __MPU_60XX_INTERRUPTS_H

#include "mpu60xx_types.h"
#include "mpu60xx_regs_bits.h"


#ifdef __cplusplus
extern "C" {
#endif



/**
 * @brief enable event detection
 *
 * @param[in] mpu_handle MPU60xx device handle.
 *
 * @param[in] mdc Configuration for event detection.
 *
 * @param[in] interrupt_configuration configuration for event detection interrupt behavior.
 * @note  can be NULL, use "mpu60xx_en_EventDetection" for polling.
 *
 *
 * @return
 *      - ESP_OK: successfully read the Interrupt status register.
 *      - other: failed to set value, check log.
 */
esp_err_t mpu60xx_Interrupt_pin_configuration (mpu60xx_handle_t mpu_handle, mpu60xx_intrpt_config_t *interrupt_configuration);

/**
 * @brief enable event detection
 *
 * @param[in] mpu_handle MPU60xx device handle.
 *
 * @param[in] mdc Configuration for event detection.
 *
 * @param [in] event interrupt to be enabled.
 *
 * @return
 *      - ESP_OK: successfully read the Interrupt status register.
 *      - other: failed to set value, check log.
 */
esp_err_t mpu60xx_en_EventDetection(mpu60xx_handle_t mpu_handle, mpu60xx_event_detect_config_t *mdc, mpu60xx_event_t event);



/**
 * @brief enable/disable event detection interrupt generation on MPU60xx.
 * @note internally called by mpu60xx_en_EventDetection but provided for flexibility.
 *
 * @param[in] mpu_handle MPU60xx device handle.
 *
 * @param[in] active if 'true' enables interrupt generation,
 *                   if 'false' disables interrupt generation.
 *
 * @param [in] event interrupt to be enabled/disabled.
 *
 * @return
 *      - ESP_OK: successfully read the Interrupt status register.
 *      - other: failed to set value, check log.
 */
esp_err_t mpu60xx_enEventDetectInterrupt(mpu60xx_handle_t mpu_handle, bool active, mpu60xx_event_t event);

/**
 * @brief read event detect interrupt status
 * @note if "MPU60XX_INTERRUPT_LATCH_UNTIL_CLEARED" was used in "mpu60xx_intrpt_config_t"
 *       this function needs to be called to clear the interrupt flag on mpu60xx,
 *       otherwise further interrupts will not be notified
 *
 * @param[in] mpu_handle MPU60xx device handle
 *
 * @param[out] status 'true' if event detected otherwise 'false'
 *
 * @return
 *      - ESP_OK: successfully read the Interrupt status register
 *      - other: failed to read value, check log
 */
esp_err_t mpu60xx_getEventInterruptStatus(mpu60xx_handle_t mpu_handle, bool *status);

/**
 * @brief read Motion detect interrupt status
 * @note if "MPU60XX_INTERRUPT_LATCH_UNTIL_CLEARED" was used in "mpu60xx_intrpt_config_t"
 *       this function needs to be called to clear the interrupt flag on mpu60xx,
 *       otherwise further interrupts will not be notified
 *
 * @param[in] mpu_handle MPU60xx device handle
 *
 * @param[out] status 'true' if event detected otherwise 'false'
 *
 * @return
 *      - ESP_OK: successfully read the Interrupt status register
 *      - other: failed to read value, check log
 */
esp_err_t mpu60xx_getMotionInterruptStatus(mpu60xx_handle_t mpu_handle, bool *status);

/**
 * @brief read Zero motion detect interrupt status
 *
 * @param[in] mpu_handle MPU60xx device handle
 *
 * @param[out] status 'true' if event detected otherwise 'false'
 *
 * @return
 *      - ESP_OK: successfully read the Interrupt status register
 *      - other: failed to read value, check log
 */
esp_err_t mpu60xx_getZeroMotionInterruptStatus(mpu60xx_handle_t mpu_handle, bool *status);

/**
 * @brief read Free Fall motion detect interrupt status
 * @note if "MPU60XX_INTERRUPT_LATCH_UNTIL_CLEARED" was used in "mpu60xx_intrpt_config_t"
 *       this function needs to be called to clear the interrupt flag on mpu60xx,
 *       otherwise further interrupts will not be notified
 *
 * @param[in] mpu_handle MPU60xx device handle
 *
 * @param[out] status 'true' if event detected otherwise 'false'
 *
 * @return
 *      - ESP_OK: successfully read the Interrupt status register
 *      - other: failed to read value, check log
 */
esp_err_t mpu60xx_getFreeFallMotionInterruptStatus(mpu60xx_handle_t mpu_handle, bool *status);


#ifdef __cplusplus
}
#endif

#endif // __MPU_60XX_INTERRUPTS_H
