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

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include "app_priv.h"

static const char *TAG = "Rainmaker Example";

esp_rmaker_device_t *temp_sensor_device;
esp_rmaker_device_t *humid_sensor_device;

/* Callback to handle commands received from the RainMaker cloud */
static esp_err_t write_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
            const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx)
{
    if (ctx) {
        ESP_LOGI(TAG, "Received write request via : %s", esp_rmaker_device_cb_src_to_str(ctx->src));
        
        if (strcmp(esp_rmaker_param_get_name(param), "LED") == 0) {
            bsp_led_set(BSP_LED_AZURE, val.val.b);
        } else if (strcmp(esp_rmaker_param_get_name(param), "BUZZER") == 0) {
            bsp_buzzer_set(val.val.b);
        }
        esp_rmaker_param_update_and_report(param, val);
    }
    
    return ESP_OK;
}

void app_main()
{
    /* Initialize Application specific hardware drivers and
     * set initial state.
     */
    app_driver_init();

    /* Initialize NVS. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    /* Initialize Wi-Fi. Note that, this should be called before esp_rmaker_init()
     */
    app_wifi_init();

    /* Initialize the ESP RainMaker Agent.
     * Note that this should be called after app_wifi_init() but before app_wifi_start()
     * */
    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = false,
    };
    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "ESP RainMaker Device", "Azure-Kit");
    if (!node) {
        ESP_LOGE(TAG, "Could not initialise node. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    /* Create a device and add the relevant parameters to it */
    esp_rmaker_device_t *rmk_device = esp_rmaker_device_create("Azure-Kit", NULL, NULL);
    esp_rmaker_device_add_cb(rmk_device, write_cb, NULL);
    // Add LED toggle UI
    esp_rmaker_param_t *led_param = esp_rmaker_param_create("LED", NULL, esp_rmaker_bool(false), PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(led_param, ESP_RMAKER_UI_TOGGLE);
    esp_rmaker_device_add_param(rmk_device, led_param);
    // Add buzzer toggle UI
    esp_rmaker_param_t *buzz_param = esp_rmaker_param_create("BUZZER", NULL, esp_rmaker_bool(false), PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(buzz_param, ESP_RMAKER_UI_TOGGLE);
    esp_rmaker_device_add_param(rmk_device, buzz_param);

    esp_rmaker_node_add_device(node, rmk_device);

    /* Create a Temperature Sensor device and add the relevant parameters to it */
    temp_sensor_device = esp_rmaker_temp_sensor_device_create("Temperature", NULL, app_get_current_temperature());
    esp_rmaker_node_add_device(node, temp_sensor_device);
    /* Create a Temperature Sensor device and add the relevant parameters to it */
    humid_sensor_device = esp_rmaker_temp_sensor_device_create("Humidity", NULL, app_get_current_humidity());
    esp_rmaker_node_add_device(node, humid_sensor_device);

    /* Start the ESP RainMaker Agent */
    esp_rmaker_start();

    app_show_prov_msg();

    /* Start the Wi-Fi.
     * If the node is provisioned, it will start connection attempts,
     * else, it will start Wi-Fi provisioning. The function will return
     * after a connection has been successfully established
     */
    err = app_wifi_start(POP_TYPE_MAC);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start Wifi. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    bsp_led_set(BSP_LED_WIFI, true);
    ESP_LOGI(TAG, "Starting Application!");
    app_start_data_collect();
}
