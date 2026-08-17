/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "fusb303b.h"

#define FUSB303B_REG_DEVICE_ID          0x01
#define FUSB303B_REG_PORT_ROLE          0x03
#define FUSB303B_REG_CONTROL            0x04
#define FUSB303B_REG_CONTROL1           0x05
#define FUSB303B_REG_MANUAL             0x09
#define FUSB303B_REG_MASK               0x0E
#define FUSB303B_REG_STATUS             0x11
#define FUSB303B_REG_INTERRUPT          0x14

#define FUSB303B_PORT_ROLE_MASK         0x07
#define FUSB303B_CONTROL_HOST_CUR_MASK  (3U << 1)
#define FUSB303B_CONTROL_INT_MASK       (1U << 0)
#define FUSB303B_CONTROL1_ENABLE        (1U << 3)
#define FUSB303B_MANUAL_DISABLED        (1U << 1)

#define FUSB303B_STATUS_VSAFE0V         (1U << 6)
#define FUSB303B_STATUS_ORIENT_MASK     (3U << 4)
#define FUSB303B_STATUS_VBUSOK          (1U << 3)
#define FUSB303B_STATUS_BC_LEVEL_MASK   (3U << 1)
#define FUSB303B_STATUS_ATTACH          (1U << 0)
#define FUSB303B_STATUS1_FAULT          (1U << 1)
#define FUSB303B_STATUS1_REMEDY         (1U << 0)
#define FUSB303B_TIMEOUT_MS             100

struct fusb303b_device_t {
    i2c_master_dev_handle_t i2c_device;
    SemaphoreHandle_t lock;  /* Serializes the compound register sequences */
    uint8_t device_address;
    uint8_t device_id;      /* Read-only identity, cached by create */
    uint8_t device_type;    /* Read-only identity, cached by create */
};

static const char *TAG = "fusb303b";

static esp_err_t lock_device(fusb303b_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL && handle->i2c_device != NULL &&
                        handle->lock != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid device handle");
    ESP_RETURN_ON_FALSE(xSemaphoreTake(handle->lock, portMAX_DELAY) == pdTRUE,
                        ESP_ERR_TIMEOUT, TAG, "device lock failed");
    return ESP_OK;
}

static void unlock_device(fusb303b_handle_t handle)
{
    xSemaphoreGive(handle->lock);
}

static esp_err_t fusb303b_read(fusb303b_handle_t handle, uint8_t reg,
                               void *data, size_t size)
{
    ESP_RETURN_ON_FALSE(handle != NULL && handle->i2c_device != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid device handle");
    ESP_RETURN_ON_FALSE(data != NULL && size > 0, ESP_ERR_INVALID_ARG, TAG,
                        "invalid read buffer");
    return i2c_master_transmit_receive(handle->i2c_device, &reg, 1, data,
                                       size, FUSB303B_TIMEOUT_MS);
}

static esp_err_t fusb303b_write(fusb303b_handle_t handle, uint8_t reg,
                                const void *data, size_t size)
{
    ESP_RETURN_ON_FALSE(handle != NULL && handle->i2c_device != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid device handle");
    ESP_RETURN_ON_FALSE(data != NULL && size > 0 && size <= 2,
                        ESP_ERR_INVALID_ARG, TAG, "invalid write buffer");
    uint8_t buffer[3];
    buffer[0] = reg;
    memcpy(&buffer[1], data, size);
    return i2c_master_transmit(handle->i2c_device, buffer, size + 1,
                               FUSB303B_TIMEOUT_MS);
}

static esp_err_t fusb303b_update_bits(fusb303b_handle_t handle, uint8_t reg,
                                      uint8_t mask, uint8_t value)
{
    uint8_t current = 0;
    ESP_RETURN_ON_ERROR(fusb303b_read(handle, reg, &current, 1), TAG,
                        "register read failed");
    current = (current & ~mask) | (value & mask);
    return fusb303b_write(handle, reg, &current, 1);
}

bool fusb303b_is_supported_identity(uint8_t device_id, uint8_t device_type)
{
    return (device_id >> 4) == FUSB303B_DEVICE_VERSION &&
           device_type == FUSB303B_DEVICE_TYPE_VALUE;
}

fusb303b_current_t fusb303b_decode_bc_level(uint8_t bc_level)
{
    /* BC_LVL=00 reports Ra or nothing attached (datasheet Table 21): it is
     * not a valid current advertisement. */
    switch (bc_level & 0x03) {
    case 1:
        return FUSB303B_CURRENT_DEFAULT;
    case 2:
        return FUSB303B_CURRENT_1_5_A;
    case 3:
        return FUSB303B_CURRENT_3_0_A;
    default:
        return FUSB303B_CURRENT_NONE;
    }
}

esp_err_t fusb303b_create(i2c_master_bus_handle_t bus,
                          const fusb303b_config_t *config,
                          fusb303b_handle_t *ret_handle)
{
    ESP_RETURN_ON_FALSE(bus != NULL && config != NULL && ret_handle != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid create argument");
    ESP_RETURN_ON_FALSE((config->device_address == FUSB303B_I2C_ADDRESS_LOW ||
                         config->device_address == FUSB303B_I2C_ADDRESS_HIGH) &&
                        config->scl_speed_hz > 0 &&
                        config->scl_speed_hz <= FUSB303B_I2C_CLOCK_HZ,
                        ESP_ERR_INVALID_ARG, TAG,
                        "invalid I2C configuration");
    *ret_handle = NULL;

    fusb303b_handle_t handle = calloc(1, sizeof(*handle));
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_NO_MEM, TAG,
                        "device allocation failed");
    handle->lock = xSemaphoreCreateMutex();
    if (handle->lock == NULL) {
        free(handle);
        return ESP_ERR_NO_MEM;
    }
    handle->device_address = config->device_address;
    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->device_address,
        .scl_speed_hz = config->scl_speed_hz,
    };
    esp_err_t error = i2c_master_bus_add_device(bus, &device_config,
                      &handle->i2c_device);
    uint8_t identity[2] = {0};
    if (error == ESP_OK) {
        error = fusb303b_read(handle, FUSB303B_REG_DEVICE_ID, identity,
                              sizeof(identity));
    }
    if (error == ESP_OK &&
            !fusb303b_is_supported_identity(identity[0], identity[1])) {
        error = ESP_ERR_INVALID_RESPONSE;
    }
    if (error == ESP_OK) {
        /* The identity registers are read-only: read them once here and
         * serve them from the handle in fusb303b_get_status(). */
        handle->device_id = identity[0];
        handle->device_type = identity[1];
    }
    if (error != ESP_OK) {
        if (handle->i2c_device != NULL) {
            i2c_master_bus_rm_device(handle->i2c_device);
        }
        vSemaphoreDelete(handle->lock);
        free(handle);
        return error;
    }
    *ret_handle = handle;
    return ESP_OK;
}

esp_err_t fusb303b_delete(fusb303b_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL && handle->i2c_device != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid device handle");
    const esp_err_t error = i2c_master_bus_rm_device(handle->i2c_device);
    if (error == ESP_OK) {
        vSemaphoreDelete(handle->lock);
        free(handle);
    }
    return error;
}

esp_err_t fusb303b_set_enabled(fusb303b_handle_t handle, bool enabled)
{
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "lock");
    const esp_err_t error = fusb303b_update_bits(handle, FUSB303B_REG_CONTROL1,
                            FUSB303B_CONTROL1_ENABLE,
                            enabled ? FUSB303B_CONTROL1_ENABLE : 0);
    unlock_device(handle);
    return error;
}

esp_err_t fusb303b_set_global_interrupt_mask(fusb303b_handle_t handle,
        bool masked)
{
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "lock");
    const esp_err_t error = fusb303b_update_bits(handle, FUSB303B_REG_CONTROL,
                            FUSB303B_CONTROL_INT_MASK,
                            masked ? FUSB303B_CONTROL_INT_MASK : 0);
    unlock_device(handle);
    return error;
}

esp_err_t fusb303b_set_interrupt_mask(fusb303b_handle_t handle, uint8_t mask,
                                      uint8_t mask1)
{
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "lock");
    /* Mask (0Eh) and Mask1 (0Fh) are consecutive plain R/W registers and the
     * FUSB303B auto-increments the register address on multi-byte writes, so
     * both are programmed in one transfer. */
    const uint8_t masks[2] = {mask, mask1};
    const esp_err_t error = fusb303b_write(handle, FUSB303B_REG_MASK, masks,
                                           sizeof(masks));
    unlock_device(handle);
    return error;
}

esp_err_t fusb303b_set_role(fusb303b_handle_t handle,
                            fusb303b_role_t role,
                            fusb303b_current_t current)
{
    ESP_RETURN_ON_FALSE(role >= FUSB303B_ROLE_DISABLED &&
                        role <= FUSB303B_ROLE_DRP &&
                        current >= FUSB303B_CURRENT_DEFAULT &&
                        current <= FUSB303B_CURRENT_3_0_A,
                        ESP_ERR_INVALID_ARG, TAG, "invalid role request");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "lock");
    esp_err_t ret = ESP_OK;
    if (role == FUSB303B_ROLE_DISABLED) {
        ESP_GOTO_ON_ERROR(fusb303b_update_bits(handle, FUSB303B_REG_MANUAL,
                                               FUSB303B_MANUAL_DISABLED,
                                               FUSB303B_MANUAL_DISABLED),
                          out, TAG, "cannot enter disabled state");
    } else {
        /* DISABLED has priority over PORT_ROLE. Keep it asserted until both
         * the requested current and role are committed, so any failed I2C
         * operation leaves the CC state machine safely disabled instead of
         * resuming the previously programmed role. */
        ESP_GOTO_ON_ERROR(fusb303b_update_bits(handle, FUSB303B_REG_MANUAL,
                                               FUSB303B_MANUAL_DISABLED,
                                               FUSB303B_MANUAL_DISABLED),
                          out, TAG, "cannot enter disabled state");
        const uint8_t current_value = current == FUSB303B_CURRENT_3_0_A ? 3 :
                                      current == FUSB303B_CURRENT_1_5_A ? 2 : 1;
        ESP_GOTO_ON_ERROR(fusb303b_update_bits(handle, FUSB303B_REG_CONTROL,
                                               FUSB303B_CONTROL_HOST_CUR_MASK,
                                               (uint8_t)(current_value << 1)),
                          out, TAG, "source current update failed");

        const uint8_t role_value = role == FUSB303B_ROLE_SINK ? (1U << 1) :
                                   role == FUSB303B_ROLE_SOURCE ? (1U << 0) :
                                   (1U << 2);
        ESP_GOTO_ON_ERROR(fusb303b_update_bits(handle, FUSB303B_REG_PORT_ROLE,
                                               FUSB303B_PORT_ROLE_MASK, role_value),
                          out, TAG, "port role update failed");
        ESP_GOTO_ON_ERROR(fusb303b_update_bits(handle, FUSB303B_REG_MANUAL,
                                               FUSB303B_MANUAL_DISABLED, 0),
                          out, TAG, "cannot leave disabled state");
    }
out:
    unlock_device(handle);
    return ret;
}

esp_err_t fusb303b_get_role(fusb303b_handle_t handle, fusb303b_role_t *role)
{
    ESP_RETURN_ON_FALSE(handle != NULL && role != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid role output");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "lock");
    esp_err_t ret = ESP_OK;
    uint8_t manual = 0;
    ESP_GOTO_ON_ERROR(fusb303b_read(handle, FUSB303B_REG_MANUAL,
                                    &manual, sizeof(manual)),
                      out, TAG, "manual register read failed");
    if ((manual & FUSB303B_MANUAL_DISABLED) != 0) {
        *role = FUSB303B_ROLE_DISABLED;
        goto out;
    }

    uint8_t port_role = 0;
    ESP_GOTO_ON_ERROR(fusb303b_read(handle, FUSB303B_REG_PORT_ROLE,
                                    &port_role, sizeof(port_role)),
                      out, TAG, "port role register read failed");
    /* Combined Portrole bits resolve by the chip priority: DRP, then sink,
     * then source (datasheet Table 14). */
    const uint8_t port = port_role & FUSB303B_PORT_ROLE_MASK;
    if ((port & (1U << 2)) != 0) {
        *role = FUSB303B_ROLE_DRP;
    } else if ((port & (1U << 1)) != 0) {
        *role = FUSB303B_ROLE_SINK;
    } else if ((port & (1U << 0)) != 0) {
        *role = FUSB303B_ROLE_SOURCE;
    } else {
        ret = ESP_ERR_INVALID_RESPONSE;
    }
out:
    unlock_device(handle);
    return ret;
}

esp_err_t fusb303b_get_and_clear_interrupts(fusb303b_handle_t handle,
        uint8_t *interrupt,
        uint8_t *interrupt1)
{
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "lock");
    esp_err_t ret = ESP_OK;
    uint8_t events[2] = {0};
    ESP_GOTO_ON_ERROR(fusb303b_read(handle, FUSB303B_REG_INTERRUPT,
                                    events, sizeof(events)),
                      out, TAG, "interrupt read failed");
    if (interrupt != NULL) {
        *interrupt = events[0];
    }
    if (interrupt1 != NULL) {
        *interrupt1 = events[1];
    }
    if ((events[0] | events[1]) != 0) {
        ESP_GOTO_ON_ERROR(fusb303b_write(handle, FUSB303B_REG_INTERRUPT,
                                         events, sizeof(events)),
                          out, TAG, "interrupt clear failed");
    }
out:
    unlock_device(handle);
    return ret;
}

esp_err_t fusb303b_get_status(fusb303b_handle_t handle,
                              fusb303b_status_t *status,
                              bool clear_interrupts)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid device handle");
    ESP_RETURN_ON_FALSE(status != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "status is NULL");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "lock");
    esp_err_t ret = ESP_OK;
    memset(status, 0, sizeof(*status));
    status->i2c_address = handle->device_address;
    /* The identity registers are read-only: report the values verified and
     * cached by fusb303b_create() instead of re-reading them on every poll. */
    status->device_id = handle->device_id;
    status->device_type = handle->device_type;

    uint8_t connection[3] = {0};
    uint8_t events[2] = {0};
    ESP_GOTO_ON_ERROR(fusb303b_read(handle, FUSB303B_REG_STATUS,
                                    connection, sizeof(connection)),
                      out, TAG, "connection status read failed");
    ESP_GOTO_ON_ERROR(fusb303b_read(handle, FUSB303B_REG_INTERRUPT,
                                    events, sizeof(events)),
                      out, TAG, "interrupt status read failed");

    status->status = connection[0];
    status->status1 = connection[1];
    status->type = connection[2];
    status->interrupt = events[0];
    status->interrupt1 = events[1];
    status->attached = (status->status & FUSB303B_STATUS_ATTACH) != 0;
    status->vbus_ok = (status->status & FUSB303B_STATUS_VBUSOK) != 0;
    status->vbus_safe_0v = (status->status & FUSB303B_STATUS_VSAFE0V) != 0;
    status->fault = (status->status1 & FUSB303B_STATUS1_FAULT) != 0;
    status->remedy_active = (status->status1 & FUSB303B_STATUS1_REMEDY) != 0;
    status->orientation = (status->status & FUSB303B_STATUS_ORIENT_MASK) >> 4;
    status->advertised_current = fusb303b_decode_bc_level(
                                     (status->status & FUSB303B_STATUS_BC_LEVEL_MASK) >> 1);

    if (clear_interrupts && (events[0] | events[1]) != 0) {
        ESP_GOTO_ON_ERROR(fusb303b_write(handle, FUSB303B_REG_INTERRUPT,
                                         events, sizeof(events)),
                          out, TAG, "interrupt clear failed");
    }
out:
    unlock_device(handle);
    return ret;
}

esp_err_t fusb303b_read_register(fusb303b_handle_t handle, uint8_t reg,
                                 uint8_t *value)
{
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "lock");
    const esp_err_t error = fusb303b_read(handle, reg, value, 1);
    unlock_device(handle);
    return error;
}

esp_err_t fusb303b_write_register(fusb303b_handle_t handle, uint8_t reg,
                                  uint8_t value)
{
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "lock");
    const esp_err_t error = fusb303b_write(handle, reg, &value, 1);
    unlock_device(handle);
    return error;
}
