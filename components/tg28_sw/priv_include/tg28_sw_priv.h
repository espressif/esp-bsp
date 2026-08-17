/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Pure conversion helpers from tg28_sw.c, shared with the test app.
 *
 * These declarations are deliberately not part of the public header: they
 * exist so the test app can exercise the conversion logic without I2C
 * hardware. They are not a stable API.
 */

#pragma once

#include "tg28_sw.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t tg28_sw_encode_charge_current(uint16_t milliamps, uint8_t *code);
uint16_t tg28_sw_decode_charge_current(uint8_t code);
esp_err_t tg28_sw_encode_input_current_limit(uint16_t milliamps, uint8_t *code);
uint16_t tg28_sw_decode_input_current_limit(uint8_t code);
esp_err_t tg28_sw_encode_charge_voltage(uint16_t millivolts, uint8_t *code);
uint16_t tg28_sw_decode_charge_voltage(uint8_t code);
esp_err_t tg28_sw_encode_vindpm(uint16_t millivolts, uint8_t *code);
uint16_t tg28_sw_decode_vindpm(uint8_t code);
esp_err_t tg28_sw_encode_precharge_current(uint16_t milliamps, uint8_t *code);
uint16_t tg28_sw_decode_precharge_current(uint8_t code);
esp_err_t tg28_sw_encode_termination_current(uint16_t milliamps, uint8_t *code);
uint16_t tg28_sw_decode_termination_current(uint8_t code);
esp_err_t tg28_sw_encode_low_battery_warning(uint8_t level1_percent,
        uint8_t level2_percent, uint8_t *value);
void tg28_sw_decode_low_battery_warning(uint8_t value, uint8_t *level1_percent,
                                        uint8_t *level2_percent);
esp_err_t tg28_sw_encode_powerkey_levels(uint16_t irqlevel_ms,
        uint16_t offlevel_ms, uint16_t onlevel_ms, uint8_t *value);
void tg28_sw_decode_powerkey_levels(uint8_t value, uint16_t *irqlevel_ms,
                                    uint16_t *offlevel_ms, uint16_t *onlevel_ms);
esp_err_t tg28_sw_encode_regulator_voltage(tg28_sw_regulator_t regulator,
        uint16_t millivolts, uint8_t *code);
uint16_t tg28_sw_decode_regulator_voltage(tg28_sw_regulator_t regulator,
        uint8_t code);
esp_err_t tg28_sw_encode_ts_current(uint16_t microamps, uint8_t *code);
uint16_t tg28_sw_decode_adc_channel(tg28_sw_adc_channel_t channel,
                                    uint8_t high, uint8_t low);

#ifdef __cplusplus
}
#endif
