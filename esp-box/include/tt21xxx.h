/**
 * @file tt21xxx.h
 * @brief
 * @version 0.1
 * @date 2021-09-06
 *
 * @copyright Copyright 2021 Espressif Systems (Shanghai) Co. Ltd.
 *
 *      Licensed under the Apache License, Version 2.0 (the "License");
 *      you may not use this file except in compliance with the License.
 *      You may obtain a copy of the License at
 *
 *               http://www.apache.org/licenses/LICENSE-2.0
 *
 *      Unless required by applicable law or agreed to in writing, software
 *      distributed under the License is distributed on an "AS IS" BASIS,
 *      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *      See the License for the specific language governing permissions and
 *      limitations under the License.
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

typedef void *tt21xxx_handle_t;

typedef struct {
    gpio_num_t rst_pin; // Touch IC reset pin. Set to GPIO_NUM_NC if not used
    gpio_num_t int_pin; // Touch IC interrupt pin. Set to GPIO_NUM_NC if not used
} tt21xxx_config_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Init tt21xxx touch panel
 *
 * @return
 *    - ESP_OK: Success
 *    - Others: Fail
 */
esp_err_t tt21xxx_create(tt21xxx_handle_t *tp, const i2c_port_t port, const tt21xxx_config_t *config);

/**
 * @brief
 *
 * @param tp
 */
void tt21xxx_delete(tt21xxx_handle_t tp);

/**
 * @brief Read packet from tt21xxx
 *
 * @return
 *    - ESP_OK: Success
 *    - Others: Fail
 */
esp_err_t tt21xxx_tp_read(tt21xxx_handle_t tp);

/**
 * @brief
 *
 * @param p_tp_num
 * @param p_x
 * @param p_y
 * @return esp_err_t
 */
esp_err_t tt21xxx_get_touch_point(tt21xxx_handle_t tp, uint8_t *p_tp_num, uint16_t *p_x, uint16_t *p_y);

/**
 * @brief
 *
 * @param p_btn_val
 * @param p_btn_signal
 * @return esp_err_t
 */
esp_err_t tt21xxx_get_btn_val(tt21xxx_handle_t tp, uint8_t *p_btn_val, uint16_t *p_btn_signal);

/**
 * @brief tt21xxx will keep COMM_INT low until all data read.
 *        So if the INT line is low after read the packet, read again.
 *
 * @return true Data avaliable
 * @return false All data has been read
 */
bool tt21xxx_data_avaliable(tt21xxx_handle_t tp);

#ifdef __cplusplus
}
#endif

