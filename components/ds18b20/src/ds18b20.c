/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "onewire_bus.h"
#include "onewire_cmd.h"
#include "onewire_crc.h"
#include "ds18b20.h"

static const char *TAG = "ds18b20";

#define DS18B20_CMD_CONVERT_TEMP      0x44
#define DS18B20_CMD_WRITE_SCRATCHPAD  0x4E
#define DS18B20_CMD_READ_SCRATCHPAD   0xBE

/**
 * @brief Structure of DS18B20's scratchpad
 */
typedef struct  {
    uint8_t temp_lsb;      /*!< lsb of temperature */
    uint8_t temp_msb;      /*!< msb of temperature */
    uint8_t th_user1;      /*!< th register or user byte 1 */
    uint8_t tl_user2;      /*!< tl register or user byte 2 */
    uint8_t configuration; /*!< resolution configuration register */
    uint8_t _reserved1;
    uint8_t _reserved2;
    uint8_t _reserved3;
    uint8_t crc_value;     /*!< crc value of scratchpad data */
} __attribute__((packed)) ds18b20_scratchpad_t;

typedef struct ds18b20_device_t {
    onewire_bus_handle_t bus;
    onewire_device_address_t addr; // if the addr is 0, we will send "ONEWIRE_CMD_SKIP_ROM" command
    uint8_t th_user1;
    uint8_t tl_user2;
    ds18b20_resolution_t resolution;
} ds18b20_device_t;

esp_err_t ds18b20_new_device_from_enumeration(onewire_device_t *device, const ds18b20_config_t *config, ds18b20_device_handle_t *ret_ds18b20)
{
    ds18b20_device_t *ds18b20 = NULL;
    ESP_RETURN_ON_FALSE(device && config && ret_ds18b20, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    // check ROM ID, the family code of DS18B20 is 0x28
    if ((device->address & 0xFF) != 0x28) {
        ESP_LOGD(TAG, "%016llX is not a DS18B20 device", device->address);
        return ESP_ERR_NOT_SUPPORTED;
    }

    ds18b20 = calloc(1, sizeof(ds18b20_device_t));
    ESP_RETURN_ON_FALSE(ds18b20, ESP_ERR_NO_MEM, TAG, "no mem for ds18b20");
    ds18b20->bus = device->bus;
    ds18b20->addr = device->address;
    ds18b20->resolution = DS18B20_RESOLUTION_12B; // DS18B20 default resolution is 12 bits

    *ret_ds18b20 = ds18b20;
    return ESP_OK;
}

esp_err_t ds18b20_new_device_from_bus(onewire_bus_handle_t bus, const ds18b20_config_t *config, ds18b20_device_handle_t *ret_ds18b20)
{
    ds18b20_device_t *ds18b20 = NULL;
    ESP_RETURN_ON_FALSE(bus && config && ret_ds18b20, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    ds18b20 = calloc(1, sizeof(ds18b20_device_t));
    ESP_RETURN_ON_FALSE(ds18b20, ESP_ERR_NO_MEM, TAG, "no mem for ds18b20");
    ds18b20->bus = bus;
    ds18b20->resolution = DS18B20_RESOLUTION_12B; // DS18B20 default resolution is 12 bits
    // we don't know the device address because there is no enumeration
    ds18b20->addr = 0;

    *ret_ds18b20 = ds18b20;
    return ESP_OK;
}

esp_err_t ds18b20_del_device(ds18b20_device_handle_t ds18b20)
{
    ESP_RETURN_ON_FALSE(ds18b20, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    free(ds18b20);
    return ESP_OK;
}

static esp_err_t ds18b20_send_command(ds18b20_device_handle_t ds18b20, uint8_t cmd)
{
    // No address mode (single device connected to the bus)
    // use "Skip ROM" command
    if (ds18b20->addr == 0) {
        uint8_t tx_buffer[2] = {ONEWIRE_CMD_SKIP_ROM, cmd};
        return onewire_bus_write_bytes(ds18b20->bus, tx_buffer, sizeof(tx_buffer));
    }

    // otherwise,
    // send device address first, then send the command
    uint8_t tx_buffer[10] = {0};
    tx_buffer[0] = ONEWIRE_CMD_MATCH_ROM;
    memcpy(&tx_buffer[1], &ds18b20->addr, sizeof(ds18b20->addr));
    tx_buffer[sizeof(ds18b20->addr) + 1] = cmd;

    return onewire_bus_write_bytes(ds18b20->bus, tx_buffer, sizeof(tx_buffer));
}

esp_err_t ds18b20_set_resolution(ds18b20_device_handle_t ds18b20, ds18b20_resolution_t resolution)
{
    ESP_RETURN_ON_FALSE(ds18b20, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    // reset bus and check if the ds18b20 is present
    ESP_RETURN_ON_ERROR(onewire_bus_reset(ds18b20->bus), TAG, "reset bus error");

    // send command: DS18B20_CMD_WRITE_SCRATCHPAD
    ESP_RETURN_ON_ERROR(ds18b20_send_command(ds18b20, DS18B20_CMD_WRITE_SCRATCHPAD), TAG, "send DS18B20_CMD_WRITE_SCRATCHPAD failed");

    // write new resolution to scratchpad
    const uint8_t resolution_data[] = {0x1F, 0x3F, 0x5F, 0x7F};
    uint8_t tx_buffer[3] = {0};
    tx_buffer[0] = ds18b20->th_user1;
    tx_buffer[1] = ds18b20->tl_user2;
    tx_buffer[2] = resolution_data[resolution];
    ESP_RETURN_ON_ERROR(onewire_bus_write_bytes(ds18b20->bus, tx_buffer, sizeof(tx_buffer)), TAG, "send new resolution failed");

    ds18b20->resolution = resolution;
    return ESP_OK;
}

esp_err_t ds18b20_trigger_temperature_conversion(ds18b20_device_handle_t ds18b20)
{
    ESP_RETURN_ON_FALSE(ds18b20, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    // reset bus and check if the ds18b20 is present
    ESP_RETURN_ON_ERROR(onewire_bus_reset(ds18b20->bus), TAG, "reset bus error");

    // send command: DS18B20_CMD_CONVERT_TEMP
    ESP_RETURN_ON_ERROR(ds18b20_send_command(ds18b20, DS18B20_CMD_CONVERT_TEMP), TAG, "send DS18B20_CMD_CONVERT_TEMP failed");

    // delay proper time based on its resolution
    const uint32_t delays_ms[] = {100, 200, 400, 800};
    vTaskDelay(pdMS_TO_TICKS(delays_ms[ds18b20->resolution]));

    return ESP_OK;
}

esp_err_t ds18b20_trigger_temperature_conversion_for_all(onewire_bus_handle_t bus)
{
    ESP_RETURN_ON_FALSE(bus, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    // reset bus and check if devices are present
    ESP_RETURN_ON_ERROR(onewire_bus_reset(bus), TAG, "reset bus error");

    // use Skip ROM command to trigger conversion for all sensors
    uint8_t tx_buffer[2] = {ONEWIRE_CMD_SKIP_ROM, DS18B20_CMD_CONVERT_TEMP};
    ESP_RETURN_ON_ERROR(onewire_bus_write_bytes(bus, tx_buffer, sizeof(tx_buffer)), TAG, "send DS18B20_CMD_CONVERT_TEMP failed");

    // delay proper time for temperature conversion
    vTaskDelay(pdMS_TO_TICKS(800));

    return ESP_OK;
}

esp_err_t ds18b20_get_temperature(ds18b20_device_handle_t ds18b20, float *ret_temperature)
{
    ESP_RETURN_ON_FALSE(ds18b20 && ret_temperature, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    // reset bus and check if the ds18b20 is present
    ESP_RETURN_ON_ERROR(onewire_bus_reset(ds18b20->bus), TAG, "reset bus error");

    // send command: DS18B20_CMD_READ_SCRATCHPAD
    ESP_RETURN_ON_ERROR(ds18b20_send_command(ds18b20, DS18B20_CMD_READ_SCRATCHPAD), TAG, "send DS18B20_CMD_READ_SCRATCHPAD failed");

    // read scratchpad data
    ds18b20_scratchpad_t scratchpad;
    ESP_RETURN_ON_ERROR(onewire_bus_read_bytes(ds18b20->bus, (uint8_t *)&scratchpad, sizeof(scratchpad)),
                        TAG, "error while reading scratchpad data");
    // check crc
    ESP_RETURN_ON_FALSE(onewire_crc8(0, (uint8_t *)&scratchpad, 8) == scratchpad.crc_value, ESP_ERR_INVALID_CRC, TAG, "scratchpad crc error");

    const uint8_t lsb_mask[4] = {0x07, 0x03, 0x01, 0x00}; // mask bits not used in low resolution
    uint8_t lsb_masked = scratchpad.temp_lsb & (~lsb_mask[scratchpad.configuration >> 5]);
    // Combine the MSB and masked LSB into a signed 16-bit integer
    int16_t temperature_raw = (((int16_t)scratchpad.temp_msb << 8) | lsb_masked);
    // Convert the raw temperature to a float,
    *ret_temperature = temperature_raw / 16.0f;

    return ESP_OK;
}

esp_err_t ds18b20_get_device_address(ds18b20_device_handle_t ds18b20, onewire_device_address_t *ret_address)
{
    ESP_RETURN_ON_FALSE(ds18b20 && ret_address, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    *ret_address = ds18b20->addr;
    return ESP_OK;
}
