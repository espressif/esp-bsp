/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief DS18B20 supported resolutions
 */
typedef enum {
    DS18B20_RESOLUTION_9B,  /*!<  9bit, needs ~93.75ms convert time */
    DS18B20_RESOLUTION_10B, /*!< 10bit, needs ~187.5ms convert time */
    DS18B20_RESOLUTION_11B, /*!< 11bit, needs ~375ms convert time */
    DS18B20_RESOLUTION_12B, /*!< 12bit, needs ~750ms convert time */
} ds18b20_resolution_t;

#ifdef __cplusplus
}
#endif
