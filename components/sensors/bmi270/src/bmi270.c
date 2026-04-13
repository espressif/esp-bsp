/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bmi270.h"
#include "bmi270_priv.h"

#define I2C_CLK_SPEED 400000

/*******************************************************************************
* Function definitions
*******************************************************************************/

static esp_err_t soft_reset(const bmi270_handle_t *dev_handle);
static esp_err_t set_adv_pwr_save(const bmi270_handle_t *dev_handle, const bool enable);
static esp_err_t set_config_load(const bmi270_handle_t *dev_handle, const bool enable);
static esp_err_t upload_file(const bmi270_handle_t *dev_handle, const uint8_t *file, const size_t file_size);
static esp_err_t upload_chunk(const bmi270_handle_t *dev_handle, const uint8_t *file, uint16_t index,
                              uint16_t write_len);
static esp_err_t read_register(const bmi270_handle_t *dev_handle, const uint8_t reg, uint8_t data[],
                               const size_t data_len);
static esp_err_t write_register(const bmi270_handle_t *dev_handle, const uint8_t reg, const uint8_t data[],
                                const size_t data_len);

/*******************************************************************************
* Local variables
*******************************************************************************/

static const char *TAG = "BMI270";

/*******************************************************************************
* Public functions
*******************************************************************************/

esp_err_t bmi270_create(const bmi270_driver_config_t *config, bmi270_handle_t **dev_handle)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "Pointer to the configuration structure must not be NULL");
    ESP_RETURN_ON_FALSE(dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to the handle pointer must not be NULL");

    esp_err_t ret = ESP_OK;
    uint8_t data = 0;

    bmi270_handle_t *handle = calloc(1, sizeof(bmi270_handle_t));
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_NO_MEM, TAG, "Not enough memory");

    if (config->interface == BMI270_USE_I2C) {
        const i2c_device_config_t i2c_dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = config->addr,
            .scl_speed_hz = I2C_CLK_SPEED,
        };
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(config->i2c_bus, &i2c_dev_cfg, &handle->i2c_handle), err, TAG,
                          "Failed to add new I2C device");
        assert(handle->i2c_handle);
    } else if (config->interface == BMI270_USE_SPI) {

        // TODO: Reminder when implementing the SPI interface
        // Dummy SPI read which sets the device to SPI mode
        // ESP_GOTO_ON_ERROR(bmi270_get_chip_id(handle, &data), err, TAG,
        // "Failed to perform a dummy read");
        ESP_RETURN_ON_ERROR(bmi270_delete(handle), TAG, "Clean up failure");
        return ESP_ERR_NOT_SUPPORTED;
    }

    // Check for the BMI270 device on I2C bus
    ESP_GOTO_ON_ERROR(bmi270_get_chip_id(handle, &data), err, TAG,
                      "Failed to read ID");
    ESP_GOTO_ON_FALSE(data == BMI270_CHIP_ID, ESP_ERR_NOT_FOUND, err, TAG, "Unexpected chip ID");

    // Start the BMI270 and prepare it to load the configuration file
    ESP_GOTO_ON_ERROR(soft_reset(handle), err, TAG, "Failed to request a soft reset");
    ESP_GOTO_ON_ERROR(set_adv_pwr_save(handle, false), err, TAG, "Failed to disable power saving");
    ESP_GOTO_ON_ERROR(set_config_load(handle, false), err, TAG, "Failed to disable configuration loading");

    ESP_GOTO_ON_ERROR(upload_file(handle, bmi270_config_file, sizeof(bmi270_config_file)), err, TAG,
                      "Failed to send the configuration file");

    handle->initialized = true;
    *dev_handle = handle;

    ESP_LOGI(TAG, "Successfully initialized the BMI270 sensor driver");

    return ret;

err:
    ESP_RETURN_ON_ERROR(bmi270_delete(handle), TAG, "Clean up failure");
    return ret;
}

esp_err_t bmi270_delete(bmi270_handle_t *dev_handle)
{
    ESP_RETURN_ON_FALSE(dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to the device handle must not be NULL");

    esp_err_t ret = ESP_OK;
    if (dev_handle->interface == BMI270_USE_I2C && dev_handle->i2c_handle) {
        ret = i2c_master_bus_rm_device(dev_handle->i2c_handle);
    } else if (dev_handle->interface == BMI270_USE_SPI && dev_handle->spi_handle) {
        // SPI interface is currently not supported
        return ESP_ERR_NOT_SUPPORTED;
    }
    free(dev_handle);
    return ret;
}

esp_err_t bmi270_get_chip_id(const bmi270_handle_t *dev_handle, uint8_t *chip_id)
{
    ESP_RETURN_ON_FALSE(dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to the device handle must not be NULL");
    ESP_RETURN_ON_FALSE(chip_id != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to the chip ID variable must not be NULL");
    return read_register(dev_handle, BMI270_CHIP_ID_REG, chip_id, 1);
}

esp_err_t bmi270_start(bmi270_handle_t *dev_handle, const bmi270_config_t *config)
{

    ESP_RETURN_ON_FALSE(dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to the device handle must not be NULL");
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "Pointer to the configuration structure must not be NULL");

    uint8_t data;

    if (!dev_handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Enable all internal BMI270 blocks (except for FIFO)
    data = BMI270_ACC_EN_MSK | BMI270_GYR_EN_MSK | BMI270_TEMP_EN_MSK;
    ESP_RETURN_ON_ERROR(write_register(dev_handle, BMI270_PWR_CTRL_REG, &data, 1), TAG, "Failed to start acquisition");

    // Configure the accelerometer and gyroscope
    ESP_RETURN_ON_ERROR(bmi270_set_acce_odr(dev_handle, config->acce_odr), TAG, "");
    ESP_RETURN_ON_ERROR(bmi270_set_acce_range(dev_handle, config->acce_range), TAG, "");
    ESP_RETURN_ON_ERROR(bmi270_set_gyro_odr(dev_handle, config->gyro_odr), TAG, "");
    ESP_RETURN_ON_ERROR(bmi270_set_gyro_range(dev_handle, config->gyro_range), TAG, "");

    return ESP_OK;
}

esp_err_t bmi270_stop(const bmi270_handle_t *dev_handle)
{
    ESP_RETURN_ON_FALSE(dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to the device handle must not be NULL");

    uint8_t data = 0;

    if (!dev_handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Disable all internal BMI270 blocks
    ESP_RETURN_ON_ERROR(write_register(dev_handle, BMI270_PWR_CTRL_REG, &data, 1), TAG, "Failed to stop");

    return ESP_OK;
}

esp_err_t bmi270_get_acce_data(const bmi270_handle_t *dev_handle, float *x, float *y, float *z)
{
    ESP_RETURN_ON_FALSE(dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to the device handle must not be NULL");
    ESP_RETURN_ON_FALSE(x != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to an axi data variable must not be NULL");
    ESP_RETURN_ON_FALSE(y != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to an axi data variable must not be NULL");
    ESP_RETURN_ON_FALSE(z != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to an axi data variable must not be NULL");

    uint8_t data[BMI270_RAW_DATA_READ_LEN];
    float sensitivity;

    if (!dev_handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(read_register(dev_handle, BMI270_ACC_X_LSB_REG, data, BMI270_RAW_DATA_READ_LEN), TAG,
                        "Failed to read raw accelerometer data");

    // Get LSB sensitivity depending on the range
    switch (dev_handle->acce_range) {
    case BMI270_ACC_RANGE_2_G:
        sensitivity = (float)INT16_MAX / 2.0f;
        break;
    case BMI270_ACC_RANGE_4_G:
        sensitivity = (float)INT16_MAX / 4.0f;
        break;
    case BMI270_ACC_RANGE_8_G:
        sensitivity = (float)INT16_MAX / 8.0f;
        break;
    case BMI270_ACC_RANGE_16_G:
        sensitivity = (float)INT16_MAX / 16.0f;
        break;
    default:
        sensitivity = 0;
    }

    // Convert and normalize all values
    *x = ((int16_t)(data[1] << 8 | data[0])) / sensitivity;
    *y = ((int16_t)(data[3] << 8 | data[2])) / sensitivity;
    *z = ((int16_t)(data[5] << 8 | data[4])) / sensitivity;

    return ESP_OK;
}

esp_err_t bmi270_get_gyro_data(const bmi270_handle_t *dev_handle, float *x, float *y, float *z)
{
    ESP_RETURN_ON_FALSE(dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to the device handle must not be NULL");
    ESP_RETURN_ON_FALSE(x != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to an axi data variable must not be NULL");
    ESP_RETURN_ON_FALSE(y != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to an axi data variable must not be NULL");
    ESP_RETURN_ON_FALSE(z != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to an axi data variable must not be NULL");

    uint8_t data[BMI270_RAW_DATA_READ_LEN];
    float sensitivity;

    if (!dev_handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(read_register(dev_handle, BMI270_GYR_X_LSB_REG, data, BMI270_RAW_DATA_READ_LEN), TAG,
                        "Failed to read raw gyroscope data");

    // Get LSB sensitivity depending on the range
    switch (dev_handle->gyro_range) {
    case BMI270_GYR_RANGE_125_DPS:
        sensitivity = (float)INT16_MAX / 125.0f;
        break;
    case BMI270_GYR_RANGE_250_DPS:
        sensitivity = (float)INT16_MAX / 250.0f;
        break;
    case BMI270_GYR_RANGE_500_DPS:
        sensitivity = (float)INT16_MAX / 500.0f;
        break;
    case BMI270_GYR_RANGE_1000_DPS:
        sensitivity = (float)INT16_MAX / 1000.0f;
        break;
    case BMI270_GYR_RANGE_2000_DPS:
        sensitivity = (float)INT16_MAX / 2000.0f;
        break;
    default:
        sensitivity = 0;
    }

    // Convert and normalize all values
    *x = ((int16_t)(data[1] << 8 | data[0])) / sensitivity;
    *y = ((int16_t)(data[3] << 8 | data[2])) / sensitivity;
    *z = ((int16_t)(data[5] << 8 | data[4])) / sensitivity;

    return ESP_OK;
}

esp_err_t bmi270_set_acce_odr(const bmi270_handle_t *dev_handle, bmi270_acce_odr_e odr)
{
    ESP_RETURN_ON_FALSE(dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to the device handle must not be NULL");

    uint8_t data;

    if (!dev_handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(read_register(dev_handle, BMI270_ACC_CONF_REG, &data, 1), TAG,
                        "Failed to read accelerometer configuration");
    data = (data & BMI270_ODR_MSK) | odr;
    ESP_RETURN_ON_ERROR(write_register(dev_handle, BMI270_ACC_CONF_REG, &data, 1), TAG,
                        "Failed to configure accelerometer ODR");

    return ESP_OK;
}

esp_err_t bmi270_set_gyro_odr(const bmi270_handle_t *dev_handle, bmi270_gyro_odr_e odr)
{
    ESP_RETURN_ON_FALSE(dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to the device handle must not be NULL");

    uint8_t data;

    if (!dev_handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(read_register(dev_handle, BMI270_GYR_CONF_REG, &data, 1), TAG,
                        "Failed to read gyroscope configuration");
    data = (data & BMI270_ODR_MSK) | odr;
    ESP_RETURN_ON_ERROR(write_register(dev_handle, BMI270_GYR_CONF_REG, &data, 1), TAG,
                        "Failed to configure gyroscope ODR");

    return ESP_OK;
}

esp_err_t bmi270_set_acce_range(bmi270_handle_t *dev_handle, bmi270_acce_range_e range)
{
    ESP_RETURN_ON_FALSE(dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to the device handle must not be NULL");

    uint8_t data;

    if (!dev_handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    data = range;
    ESP_RETURN_ON_ERROR(write_register(dev_handle, BMI270_ACC_RANGE_REG, &data, 1), TAG,
                        "Failed to configure accelerometer range");
    dev_handle->acce_range = range;

    return ESP_OK;
}

esp_err_t bmi270_set_gyro_range(bmi270_handle_t *dev_handle, bmi270_gyro_range_e range)
{
    ESP_RETURN_ON_FALSE(dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to the device handle must not be NULL");

    uint8_t data;

    if (!dev_handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(read_register(dev_handle, BMI270_GYR_RANGE_REG, &data, 1), TAG,
                        "Failed to read gyroscope range register");
    data = (data & BMI270_GYR_RANGE_MSK) | range;
    ESP_RETURN_ON_ERROR(write_register(dev_handle, BMI270_GYR_RANGE_REG, &data, 1), TAG,
                        "Failed to configure gyroscope range");
    dev_handle->gyro_range = range;

    return ESP_OK;
}

/*******************************************************************************
* Private functions
*******************************************************************************/

static esp_err_t soft_reset(const bmi270_handle_t *dev_handle)
{
    assert(dev_handle != NULL);

    uint8_t data = BMI270_CMD_SOFT_RESET;

    ESP_RETURN_ON_ERROR(write_register(dev_handle, BMI270_CMD_REG, &data, 1), TAG,
                        "Failed to soft reset the device");
    vTaskDelay(pdMS_TO_TICKS(BMI270_SOFT_RESET_TIME_MS));

    return ESP_OK;
}

static esp_err_t set_adv_pwr_save(const bmi270_handle_t *dev_handle, const bool enable)
{
    assert(dev_handle != NULL);

    uint8_t data;

    ESP_RETURN_ON_ERROR(read_register(dev_handle, BMI270_PWR_CONF_REG, &data, 1), TAG,
                        "Failed to read the power config");
    data = enable ? data | BIT0 : data & ~BIT0;
    ESP_RETURN_ON_ERROR(write_register(dev_handle, BMI270_PWR_CONF_REG, &data, 1), TAG,
                        "Failed to set the power config");
    vTaskDelay(pdMS_TO_TICKS(BMI270_POWER_ON_TIME_MS));

    return ESP_OK;
}

static esp_err_t set_config_load(const bmi270_handle_t *dev_handle, const bool enable)
{
    assert(dev_handle != NULL);

    uint8_t data;

    ESP_RETURN_ON_ERROR(read_register(dev_handle, BMI270_INIT_CTRL_REG, &data, 1), TAG,
                        "Failed to read the initialization control register");
    data = enable ? data | BIT0 : data & ~BIT0;
    ESP_RETURN_ON_ERROR(write_register(dev_handle, BMI270_INIT_CTRL_REG, &data, 1), TAG,
                        "Failed to set the initialization control register");
    return ESP_OK;
}

static esp_err_t upload_file(const bmi270_handle_t *dev_handle, const uint8_t *file, const size_t file_size)
{
    assert(dev_handle != NULL);
    assert(file != NULL);

    uint8_t data;
    uint16_t remainder = 0;
    uint16_t chunk_size = 0;
    uint32_t file_index = 0;
    uint8_t status_checks;

    remainder = file_size % BMI270_FILE_WRITE_LEN;

    // Pick write approach depending on it's divisibility by the write length
    if (!remainder) {
        for (file_index = 0; file_index < file_size; file_index += BMI270_FILE_WRITE_LEN) {
            ESP_RETURN_ON_ERROR(upload_chunk(dev_handle, &file[file_index], file_index, BMI270_FILE_WRITE_LEN), TAG,
                                "Failed to upload a file chunk");
        }
    } else {
        chunk_size = file_size - remainder;
        // Bulk write parts of the file
        for (file_index = 0; file_index < chunk_size; file_index += BMI270_FILE_WRITE_LEN) {
            ESP_RETURN_ON_ERROR(upload_chunk(dev_handle, &file[file_index], file_index, BMI270_FILE_WRITE_LEN), TAG,
                                "Failed to upload a file chunk");
        }

        // Switch to half-word granular write
        for (file_index = chunk_size; file_index < file_size; file_index += 2) {
            ESP_RETURN_ON_ERROR(upload_chunk(dev_handle, &file[file_index], file_index, BMI270_FILE_WRITE_LEN), TAG,
                                "Failed to upload a file chunk");
        }
    }

    // Re-enable loading of the configuration file
    ESP_RETURN_ON_ERROR(set_config_load(dev_handle, true), TAG, "Failed to enable configuration loading");

    // Wait for the BMI270 to initialize
    status_checks = BMI270_STATUS_CHECK_CNT;
    do {
        ESP_RETURN_ON_ERROR(read_register(dev_handle, BMI270_INTERNAL_STATUS_REG, &data, 1), TAG,
                            "Failed to read internal status");

        vTaskDelay(pdMS_TO_TICKS(BMI270_STATUS_CHECK_PERIOD_MS));
        status_checks--;
    } while ((data != BMI270_STATUS_INIT_OK) & (status_checks > 0));

    ESP_RETURN_ON_FALSE(status_checks > 0, ESP_ERR_TIMEOUT, TAG, "Could not initialize the device in time");

    return ESP_OK;
}

static esp_err_t upload_chunk(const bmi270_handle_t *dev_handle, const uint8_t *file, uint16_t index,
                              uint16_t write_len)
{
    assert(dev_handle != NULL);
    assert(file != NULL);

    uint8_t write_addr[2];

    write_addr[0] = (uint8_t)((index / 2) & 0x0F);
    write_addr[1] = (uint8_t)((index / 2) >> 4);

    ESP_RETURN_ON_ERROR(write_register(dev_handle, BMI270_INIT_ADDR_0_REG, write_addr, 2), TAG,
                        "Failed to set address for the config file");

    ESP_RETURN_ON_ERROR(write_register(dev_handle, BMI270_INIT_DATA_REG, file, BMI270_FILE_WRITE_LEN), TAG,
                        "Failed to send config file data");

    return ESP_OK;
}

static esp_err_t read_register(const bmi270_handle_t *dev_handle, const uint8_t reg, uint8_t data[],
                               const size_t data_len)
{
    assert(dev_handle != NULL);

    if (dev_handle->interface == BMI270_USE_I2C) {
        return i2c_master_transmit_receive(dev_handle->i2c_handle, &reg, 1, data, data_len, -1);
    } else if (dev_handle->interface == BMI270_USE_SPI) {
        // SPI interface is currently not supported
        return ESP_ERR_NOT_SUPPORTED;
    } else {
        return ESP_FAIL;
    }
}

static esp_err_t write_register(const bmi270_handle_t *dev_handle, const uint8_t reg, const uint8_t data[],
                                const size_t data_len)
{
    assert(dev_handle != NULL);

    size_t packet_len = data_len + 1;
    uint8_t packet[BMI270_FILE_WRITE_LEN + 1];

    ESP_RETURN_ON_FALSE(data_len <= BMI270_FILE_WRITE_LEN, ESP_ERR_INVALID_ARG, TAG, "Exceeded expected data length");

    packet[0] = reg;
    memcpy(&packet[1], data, data_len);
    if (dev_handle->interface == BMI270_USE_I2C) {
        return i2c_master_transmit(dev_handle->i2c_handle, packet, packet_len, -1);
    } else if (dev_handle->interface == BMI270_USE_SPI) {
        // SPI interface is currently not supported
        return ESP_ERR_NOT_SUPPORTED;
    } else {
        return ESP_FAIL;
    }
}
