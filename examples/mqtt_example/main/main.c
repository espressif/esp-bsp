/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/**
 * @file
 * @brief BSP MQTT Example
 * @details Collect sensor data and publish to an MQTT server
 */

#include <stdio.h>
#include <inttypes.h>
#include "hts221.h"
#include "fbm320.h"
#include "bh1750.h"
#include "bsp/esp-bsp.h"
#include "esp_log.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "mqtt_client.h"

void wifi_init_sta(void);

static const char *TAG = "Azure";

static bool mqtt_connected = false;
static bh1750_handle_t bh1750_dev = NULL;
static hts221_handle_t hts221_dev = NULL;
static fbm320_handle_t fbm320_dev = NULL;

typedef struct {
    int16_t temperature;
    int16_t humidity;
    float luminescence;
    int32_t pressure;
} sensor_data_t;

static void app_sensors_init()
{
    bh1750_dev = bh1750_create(BSP_I2C_NUM, BH1750_I2C_ADDRESS_DEFAULT);
    assert(bh1750_dev != NULL);
    ESP_ERROR_CHECK(bh1750_power_on(bh1750_dev));
    ESP_ERROR_CHECK(bh1750_set_measure_mode(bh1750_dev, BH1750_CONTINUE_4LX_RES));

    hts221_dev = hts221_create(BSP_I2C_NUM);
    assert(hts221_dev != NULL);
    const hts221_config_t hts221_config = {
        .avg_h = HTS221_AVGH_32, .avg_t = HTS221_AVGT_16, .odr = HTS221_ODR_1HZ, .bdu_status = true
    };
    ESP_ERROR_CHECK(hts221_init(hts221_dev, &hts221_config));

    fbm320_dev = fbm320_create(BSP_I2C_NUM, FBM320_I2C_ADDRESS_1);
    assert(fbm320_dev != NULL);
    ESP_ERROR_CHECK(fbm320_init(fbm320_dev));
}

static void app_sensors_get(sensor_data_t *data)
{
    int32_t temp;
    hts221_get_humidity(hts221_dev, &data->humidity);
    hts221_get_temperature(hts221_dev, &data->temperature);
    bh1750_get_data(bh1750_dev, &data->luminescence);
    fbm320_get_data(fbm320_dev, FBM320_MEAS_PRESS_OSR_2048, &temp, &data->pressure);
}

/**
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRId32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        mqtt_connected = true;
        bsp_led_set(BSP_LED_AZURE, true);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        mqtt_connected = false;
        bsp_led_set(BSP_LED_AZURE, false);
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;

    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(bsp_i2c_init());
    ESP_ERROR_CHECK(bsp_leds_init());
    lv_disp_t *disp = bsp_display_start();
    app_sensors_init();
    ESP_ERROR_CHECK(nvs_flash_init());

    /* Write labels on display */
    bsp_display_lock(0);
    lv_obj_t *main_screen = lv_disp_get_scr_act(NULL);
    lv_obj_t *main_label = lv_label_create(main_screen);
    lv_label_set_text_static(main_label, LV_SYMBOL_WIFI"\nWifi\nconnecting...");
    lv_obj_set_style_text_align(main_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(main_label, lv_display_get_physical_horizontal_resolution(disp));
    lv_obj_align(main_label, LV_ALIGN_TOP_MID, 0, 0);
    bsp_display_unlock();

    wifi_init_sta();

    const esp_mqtt_client_config_t mqtt_cfg = {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
        .uri = CONFIG_BROKER_URL,
#else
        .broker.address.uri = CONFIG_BROKER_URL,
#endif
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    /* Write labels on display */
    bsp_display_lock(0);
    lv_label_set_text_static(main_label, "Temp:\nHumi:\nLumi:\nPress:");
    lv_obj_set_style_text_align(main_label, LV_TEXT_ALIGN_LEFT, 0);
    bsp_display_unlock();

    while (1) {
        sensor_data_t sensor_data;
        app_sensors_get(&sensor_data);

        /* Strings to be sent to MQTT and display */
        char t[10], h[10], l[10], p[10];
        sprintf(t, "%4.2f", ((float)sensor_data.temperature) / 10); // In degree Celsius
        sprintf(h, "%4.2f", ((float)sensor_data.humidity) / 10); // In percent
        sprintf(l, "%4.2f", ((float)sensor_data.luminescence)); // In lux
        sprintf(p, "%4.2f", ((float)sensor_data.pressure) / 1000); // in kPa

        /* Publish sensor data, if connected */
        if (mqtt_connected) {
            esp_mqtt_client_publish(client, "esp-azure/temperature", t, 0, 1, 0);
            esp_mqtt_client_publish(client, "esp-azure/humidity", h, 0, 1, 0);
            esp_mqtt_client_publish(client, "esp-azure/luminescence", l, 0, 1, 0);
            esp_mqtt_client_publish(client, "esp-azure/pressure", p, 0, 1, 0);
        }

        bsp_display_lock(0);
        lv_label_set_text_fmt(main_label, "Temp: %s\nHumi: %s\nLumi: %s\nPress: %s", t, h, l, p);
        bsp_display_unlock();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
