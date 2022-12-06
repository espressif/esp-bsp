/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief HTS221 driver
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c.h"
#include "driver/gpio.h"

/**
* @brief  Humidity average.
*/
typedef enum {
    HTS221_AVGH_4         = 0x00,         /*!< Internal average on 4 samples */
    HTS221_AVGH_8         = 0x01,         /*!< Internal average on 8 samples */
    HTS221_AVGH_16        = 0x02,         /*!< Internal average on 16 samples */
    HTS221_AVGH_32        = 0x03,         /*!< Internal average on 32 samples */
    HTS221_AVGH_64        = 0x04,         /*!< Internal average on 64 samples */
    HTS221_AVGH_128       = 0x05,         /*!< Internal average on 128 samples */
    HTS221_AVGH_256       = 0x06,         /*!< Internal average on 256 samples */
    HTS221_AVGH_512       = 0x07          /*!< Internal average on 512 samples */
} hts221_avgh_t;

/**
* @brief  Temperature average.
*/
typedef enum {
    HTS221_AVGT_2         = 0x00,        /*!< Internal average on 2 samples */
    HTS221_AVGT_4         = 0x08,        /*!< Internal average on 4 samples */
    HTS221_AVGT_8         = 0x10,        /*!< Internal average on 8 samples */
    HTS221_AVGT_16        = 0x18,        /*!< Internal average on 16 samples */
    HTS221_AVGT_32        = 0x20,        /*!< Internal average on 32 samples */
    HTS221_AVGT_64        = 0x28,        /*!< Internal average on 64 samples */
    HTS221_AVGT_128       = 0x30,        /*!< Internal average on 128 samples */
    HTS221_AVGT_256       = 0x38         /*!< Internal average on 256 samples */
} hts221_avgt_t;

/**
* @brief  Output data rate configuration.
*/
typedef enum {
    HTS221_ODR_ONE_SHOT  = 0x00,         /*!< Output Data Rate: one shot */
    HTS221_ODR_1HZ       = 0x01,         /*!< Output Data Rate: 1Hz */
    HTS221_ODR_7HZ       = 0x02,         /*!< Output Data Rate: 7Hz */
    HTS221_ODR_12_5HZ    = 0x03,         /*!< Output Data Rate: 12.5Hz */
} hts221_odr_t;

/**
* @brief  Push-pull/Open Drain selection on DRDY pin.
*/
typedef enum {
    HTS221_PUSHPULL   = 0x00,   /*!< DRDY pin in push pull */
    HTS221_OPENDRAIN  = 0x40    /*!< DRDY pin in open drain */
} hts221_outputtype_t;

/**
* @brief  Active level of DRDY pin.
*/
typedef enum {
    HTS221_HIGH_LVL   = 0x00,   /*!< HIGH state level for DRDY pin */
    HTS221_LOW_LVL    = 0x80    /*!< LOW state level for DRDY pin */
} hts221_drdylevel_t;

/**
* @brief  HTS221 Init structure definition.
*/
typedef struct {
    hts221_avgh_t        avg_h;            /*!< Humidity average */
    hts221_avgt_t        avg_t;            /*!< Temperature average */
    hts221_odr_t         odr;              /*!< Output data rate */
    bool                 bdu_status;       /*!< Enable/disable the block data update */
} hts221_config_t;

/**
 * @brief Callback function type for DRDY mode
 */
typedef void (*hts221_drdy_callback_t)(int16_t humidity, int16_t temperature);

/**
 * @brief Configuration structure for DRDY mode
 */
typedef struct {
    hts221_drdylevel_t     irq_level;        /*!< HTS221_HIGH_LVL/HTS221_LOW_LVL the level for DRDY pin */
    hts221_outputtype_t    irq_output_type;  /*!< Output configuration for DRDY pin */
    gpio_num_t             drdy_pin;
    hts221_drdy_callback_t drdy_callback;
    UBaseType_t            drdy_task_priority;
} hts221_drdy_config_t;


/**
* @brief Device Identification value.
*/
#define HTS221_WHO_AM_I_VAL         ((uint8_t)0xBC)

typedef void *hts221_handle_t;

/**
 * @brief Get device identification of HTS221
 *
 * @param sensor object handle of hts221
 * @param[out] deviceid a pointer to device ID
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t hts221_get_deviceid(hts221_handle_t sensor, uint8_t *const deviceid);

/**
 * @brief Set configration of HTS221
 *
 * @param sensor object handle of hts221
 * @param[in] hts221_config a structure pointer of configration
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t hts221_set_config(hts221_handle_t sensor, const hts221_config_t *const hts221_config);

/**
 * @brief Get configration of HTS221
 *
 * @param sensor object handle of hts221
 * @param[out] hts221_config a structure pointer of configration
 *
 * @return
 *     - ESP_OK Success
 *     - others Fail
 */
esp_err_t hts221_get_config(hts221_handle_t sensor, hts221_config_t *const hts221_config);

/**
 * @brief Activate HTS221
 *
 * @param sensor object handle of hts221
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t hts221_set_activate(hts221_handle_t sensor);

/**
 * @brief Set HTS221 as power down mode
 *
 * @param sensor object handle of hts221
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t hts221_set_powerdown(hts221_handle_t sensor);

/**
 * @brief Set output data rate
 *
 * @param sensor object handle of hts221
 * @param[in] odr output data rate value
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t hts221_set_odr(hts221_handle_t sensor, const hts221_odr_t odr);

/**
 * @brief Set humidity average
 *
 * @param sensor object handle of hts221
 * @param[in] avgh selections of the numbers of averaged humidity samples
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t hts221_set_avgh(hts221_handle_t sensor, const hts221_avgh_t avgh);

/**
 * @brief Set temperature average
 *
 * @param sensor object handle of hts221
 * @param[in] avgt  selections of the numbers of averaged temperature samples
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t hts221_set_avgt(hts221_handle_t sensor, const hts221_avgt_t avgt);

/**
 * @brief Enable block data update
 *
 * @param sensor object handle of hts221
 * @param[in] status enable/diable status
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t hts221_set_bdumode(hts221_handle_t sensor, const bool status);

/**
 * @brief Enable heater
 *
 * @param sensor object handle of hts221
 * @param[in] status enable/diable status
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t hts221_set_heaterstate(hts221_handle_t sensor, const bool status);

/**
 * @brief  Enable one-shot mode
 *
 * @param sensor object handle of hts221
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t hts221_start_oneshot(hts221_handle_t sensor);

/**
 * @brief Read HTS221 Humidity output registers, and calculate humidity
 *
 * @param sensor object handle of hts221
 * @param[out] humidity pointer to the returned humidity value that must be divided by 10 to get the value in [%]
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t hts221_get_humidity(hts221_handle_t sensor, int16_t *const humidity);

/**
 * @brief Read HTS221 temperature output registers, and calculate temperature
 *
 * @param sensor object handle of hts221
 * @param[out] temperature pointer to the returned temperature value that must be divided by 10 to get the value in ['C]
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t hts221_get_temperature(hts221_handle_t sensor, int16_t *const temperature);

/**
 * @brief Create sensor object and return a sensor handle
 *
 * @param port     I2C port object handle
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
hts221_handle_t hts221_create(const i2c_port_t port);

/**
 * @brief Init HTS221 sensor object
 *
 * This function verifies content of WHO_AM_I register, loads calibration data into RAM, configures and activates the device.
 *
 * @param sensor object handle of hts221
 * @param[in] hts221_config a structure pointer of configration
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_RESPONSE WHO_AM_I register contains invalid data
 *     - ESP_FAIL Fail
 */
esp_err_t hts221_init(hts221_handle_t sensor, const hts221_config_t *const hts221_config);

/**
 * @brief Enable DRDY mode
 *
 * @param sensor Object handle of HTS221
 * @param[in] config DRDY mode configuration structure
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Callback function is NULL
 *     - ESP_ERR_INVALID_STATE Driver is not initialized
 *     - ESP_ERR_NO_MEM Failed to create a FreeRTOS task
 */
esp_err_t hts221_drdy_enable(hts221_handle_t sensor, const hts221_drdy_config_t *const config);

/**
 * @brief Disable DRDY mode
 *
 * @param sensor Object handle of HTS221
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_STATE DRDY mode was not enabled
 *     - else I2C transmission failed
 */
esp_err_t hts221_drdy_disable(hts221_handle_t sensor);

/**
 * @brief Delete and release a sensor object
 *
 * @param sensor object handle of hts221
 */
void hts221_delete(hts221_handle_t sensor);

#ifdef __cplusplus
}
#endif
