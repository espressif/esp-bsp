// Copyright 2015-2021 Espressif Systems (Shanghai) CO LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#include <stdint.h>
#include <stdbool.h>

// BSP related
#include "hts221.h" // Temperature and Humidity
#include "ssd1306.h" // OLED Display
#include "esp32_azure_iot_kit.h" // BSP
// Rainmaker related
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>
#include <app_wifi.h>

#define REPORTING_PERIOD    30 /* Seconds */

extern esp_rmaker_device_t *temp_sensor_device;
extern esp_rmaker_device_t *humid_sensor_device;

void app_driver_init(void);
void app_start_data_collect(void);
float app_get_current_temperature();
float app_get_current_humidity();
void app_show_prov_msg(void);
