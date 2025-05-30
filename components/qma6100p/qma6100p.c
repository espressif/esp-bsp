/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "driver/i2c_master.h"
#include "qma6100p.h"
#include "esp_err.h"
#include "esp_check.h"

#define I2C_CLK_SPEED 400000

#define QMA6100P_WHO_AM_I             0x00u
#define QMA6100P_ACCEL_CONFIG         0x0Fu
#define QMA6100P_ACCEL_XOUT_H         0x01u
#define QMA6100P_PWR_MGMT_1           0x11u
#define QMA6100P_NVM_LOAD             0x33u

#define QMA6100P_INTR_PIN_CFG         0x20u
#define QMA6100P_INTR_CFG             0X21u
#define QMA6100P_INTR_ENABLE          0x16u
#define QMA6100P_INTR_TAP_EN          0x16u
#define QMA6100P_INTR_FIFO_EN         0x17u
#define QMA6100P_INTR_MOT_EN          0x18u

// status 0x09 <-> 0x0D
#define QMA6100P_INTR_STATUS          0x09u
#define QMA6100P_INTR_FIFO_ST         0x0Bu

// map 0x19 <-> 0x1C
#define QMA6100P_INT1_MAP_TAP         0x19u
#define QMA6100P_INT1_MAP_FIFO        0x1Au
#define QMA6100P_INT2_MAP_TAP         0x1Bu
#define QMA6100P_INT2_MAP_FIFO        0x1Cu

#define QMA6100P_FIFO_FRAME_CTR       0x0Eu
#define QMA6100P_FIFO_DATA            0x3Fu
#define QMA6100P_FIFO_MODE            0x3Eu

const uint8_t QMA6100P_DATA_RDY_INT_BIT =      (uint8_t) BIT4;
// FIFO full interrupt
const uint8_t QMA6100P_FIFO_FULL_INT_BIT =     (uint8_t) BIT5;
// FIFO watermark interrupt
const uint8_t QMA6100P_FIFO_WM_INT_BIT =       (uint8_t) BIT6;
const uint8_t QMA6100P_FIFO_OF_INT_BIT =       (uint8_t) BIT7;
const uint8_t QMA6100P_ALL_INTERRUPTS = (QMA6100P_DATA_RDY_INT_BIT | QMA6100P_FIFO_FULL_INT_BIT | QMA6100P_FIFO_WM_INT_BIT | QMA6100P_FIFO_OF_INT_BIT);

static const char *TAG = "QMA6100P";

typedef struct {
    i2c_master_dev_handle_t i2c_handle;
    gpio_num_t int_pin;
    uint32_t counter;
    float dt;  /*!< delay time between two measurements, dt should be small (ms level) */
    struct timeval *timer;
} qma6100p_dev_t;

static esp_err_t qma6100p_write(qma6100p_handle_t sensor, const uint8_t reg_start_addr, const uint8_t data_buf)
{
    qma6100p_dev_t *sens = (qma6100p_dev_t *) sensor;

    uint8_t write_buff[2] = {reg_start_addr, data_buf};
    return i2c_master_transmit(sens->i2c_handle, write_buff, 2, -1);
}

static esp_err_t qma6100p_read(qma6100p_handle_t sensor, const uint8_t reg_start_addr, uint8_t *const data_buf, const uint8_t data_len)
{
    qma6100p_dev_t *sens = (qma6100p_dev_t *) sensor;

    uint8_t reg_buff[] = {reg_start_addr};
    assert(sens);

    return i2c_master_transmit_receive(sens->i2c_handle, reg_buff, sizeof(reg_buff), data_buf, data_len, -1);
}

esp_err_t qma6100p_create(i2c_master_bus_handle_t i2c_bus, const uint8_t dev_addr, qma6100p_handle_t *handle_ret)
{
    esp_err_t ret = ESP_OK;

    qma6100p_dev_t *sensor = (qma6100p_dev_t *) calloc(1, sizeof(qma6100p_dev_t));
    struct timeval *timer = (struct timeval *) calloc(1, sizeof(struct timeval));
    ESP_RETURN_ON_FALSE(sensor != NULL && timer != NULL, ESP_ERR_NO_MEM, TAG, "Not enough memory");
    sensor->timer = timer;

    // Add new I2C device
    const i2c_device_config_t i2c_dev_cfg = {
        .device_address = dev_addr,
        .scl_speed_hz = I2C_CLK_SPEED,
    };
    ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &i2c_dev_cfg, &sensor->i2c_handle), err, TAG, "Failed to add new I2C device");
    assert(sensor->i2c_handle);

    *handle_ret = sensor;
    return ret;

err:
    qma6100p_delete(sensor);
    return ret;
}

void qma6100p_delete(qma6100p_handle_t sensor)
{
    qma6100p_dev_t *sens = (qma6100p_dev_t *) sensor;
    if (sens->i2c_handle) {
        i2c_master_bus_rm_device(sens->i2c_handle);
    }
    if (sens->timer) {
        free(sens->timer);
    }
    free(sens);
}

esp_err_t qma6100p_get_deviceid(qma6100p_handle_t sensor, uint8_t *const deviceid)
{
    return qma6100p_read(sensor, QMA6100P_WHO_AM_I, deviceid, 1);
}

esp_err_t qma6100p_nvm_load(qma6100p_handle_t sensor)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = qma6100p_read(sensor, QMA6100P_NVM_LOAD, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp |= BIT3;
    ret = qma6100p_write(sensor, QMA6100P_NVM_LOAD, tmp);
    return ret;
}

esp_err_t qma6100p_wake_up(qma6100p_handle_t sensor)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = qma6100p_read(sensor, QMA6100P_PWR_MGMT_1, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp |= BIT7;
    ret = qma6100p_write(sensor, QMA6100P_PWR_MGMT_1, tmp);

    ret = qma6100p_read(sensor, QMA6100P_PWR_MGMT_1, &tmp, 1);
    return ret;
}

esp_err_t qma6100p_config(qma6100p_handle_t sensor, const qma6100p_acce_fs_t acce_fs)
{
    uint8_t config_reg;
    esp_err_t ret;
    ret = qma6100p_read(sensor, QMA6100P_ACCEL_CONFIG, &config_reg, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    config_reg = (config_reg & 0xf0) | (acce_fs & 0x0f);
    return qma6100p_write(sensor, QMA6100P_ACCEL_CONFIG, config_reg);
}

esp_err_t qma6100p_get_acce_sensitivity(qma6100p_handle_t sensor, float *const acce_sensitivity)
{
    esp_err_t ret;
    uint8_t acce_fs;
    ret = qma6100p_read(sensor, QMA6100P_ACCEL_CONFIG, &acce_fs, 1);
    acce_fs &= 0b1111;
    switch (acce_fs) {
    case ACCE_FS_2G:
        *acce_sensitivity = 4096;
        break;

    case ACCE_FS_4G:
        *acce_sensitivity = 2048;
        break;

    case ACCE_FS_8G:
        *acce_sensitivity = 1024;
        break;

    case ACCE_FS_16G:
        *acce_sensitivity = 512;
        break;

    case ACCE_FS_32G:
        *acce_sensitivity = 256;
        break;

    default:
        *acce_sensitivity = 4096;
        break;
    }
    return ret;
}

esp_err_t qma6100p_register_isr(qma6100p_handle_t sensor, const qma6100p_isr_t isr)
{
    esp_err_t ret;
    qma6100p_dev_t *sensor_device = (qma6100p_dev_t *) sensor;

    if (NULL == sensor_device) {
        return ESP_ERR_INVALID_ARG;
    }

    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);

    ret = gpio_isr_handler_add(
              sensor_device->int_pin,
              ((gpio_isr_t) * (isr)),
              ((void *) sensor)
          );

    if (ESP_OK != ret) {
        return ret;
    }

    ret = gpio_intr_enable(sensor_device->int_pin);

    return ret;
}

esp_err_t qma6100p_enable_interrupts(qma6100p_handle_t sensor, uint8_t interrupt_sources)
{
    esp_err_t ret;
    uint8_t enabled_interrupts = 0x00;

    ret = qma6100p_read(sensor, QMA6100P_INTR_FIFO_EN, &enabled_interrupts, 1);

    if (ESP_OK != ret) {
        return ret;
    }

    if (enabled_interrupts != interrupt_sources) {
        enabled_interrupts |= interrupt_sources;
        ret = qma6100p_write(sensor, QMA6100P_INTR_FIFO_EN, enabled_interrupts);
    }

    return ret;
}

esp_err_t qma6100p_map_interrupts(qma6100p_handle_t sensor, int int_num, uint8_t interrupt_sources)
{
    esp_err_t ret;
    int int_map_reg = 0;

    if (int_num == 0) {
        int_map_reg = QMA6100P_INT1_MAP_FIFO;
    } else if (int_num == 1) {
        int_map_reg = QMA6100P_INT2_MAP_FIFO;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t int_map = 0;
    ret = qma6100p_read(sensor, int_map_reg, &int_map, 1);

    if (ret != ESP_OK) {
        return ret;
    }

    int_map |= interrupt_sources;

    ret = qma6100p_write(sensor, int_map_reg, int_map);

    return ret;
}

esp_err_t qma6100p_config_interrupt(qma6100p_handle_t sensor, int int_num, const qma6100p_int_config_t *const interrupt_configuration)
{
    esp_err_t ret = ESP_OK;

    if (NULL == interrupt_configuration) {
        return ESP_ERR_INVALID_ARG;
    }

    qma6100p_dev_t *sensor_device = (qma6100p_dev_t *) sensor;

    if (GPIO_IS_VALID_GPIO(interrupt_configuration->interrupt_pin)) {
        // Set GPIO connected to qma6100p INT pin only when user configures interrupts.
        sensor_device->int_pin = interrupt_configuration->interrupt_pin;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t int_pin_cfg = 0x00;
    uint8_t int_cfg     = 0x00;

    ret = qma6100p_read(sensor, QMA6100P_INTR_PIN_CFG, &int_pin_cfg, 1);

    if (ESP_OK != ret) {
        return ret;
    }

    ret = qma6100p_read(sensor, QMA6100P_INTR_CFG, &int_cfg, 1);

    if (ESP_OK != ret) {
        return ret;
    }

    if (int_num == 0) {
        if (INTERRUPT_PIN_ACTIVE_LOW == interrupt_configuration->active_level_int) {
            int_pin_cfg |= BIT0;
        }
        if (INTERRUPT_PIN_OPEN_DRAIN == interrupt_configuration->pin_mode_int) {
            int_pin_cfg |= BIT1;
        }
    } else if (int_num == 1) {
        if (INTERRUPT_PIN_ACTIVE_LOW == interrupt_configuration->active_level_int) {
            int_pin_cfg |= BIT2;
        }
        if (INTERRUPT_PIN_OPEN_DRAIN == interrupt_configuration->pin_mode_int) {
            int_pin_cfg |= BIT3;
        }
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    if (INTERRUPT_LATCH_MODE == interrupt_configuration->interrupt_latch) {
        int_cfg |= BIT0;
    }
    if (INTERRUPT_CLEAR_LATCHED == interrupt_configuration->interrupt_clear_behavior) {
        int_cfg |= BIT7;
    }

    ret = qma6100p_write(sensor, QMA6100P_INTR_PIN_CFG, int_pin_cfg);

    if (ESP_OK != ret) {
        return ret;
    }

    ret = qma6100p_write(sensor, QMA6100P_INTR_CFG, int_cfg);

    if (ESP_OK != ret) {
        return ret;
    }

    gpio_int_type_t gpio_intr_type;

    if (INTERRUPT_PIN_ACTIVE_LOW == interrupt_configuration->active_level_int) {
        gpio_intr_type = GPIO_INTR_NEGEDGE;
    } else {
        gpio_intr_type = GPIO_INTR_POSEDGE;
    }

    gpio_config_t int_gpio_config = {
        .mode = GPIO_MODE_INPUT,
        .intr_type = gpio_intr_type,
        .pin_bit_mask = (BIT0 << interrupt_configuration->interrupt_pin)
    };

    ret = gpio_config(&int_gpio_config);

    if (ret != ESP_OK) {
        return ret;
    }

    ret = qma6100p_register_isr(sensor_device, interrupt_configuration->isr);

    if (ret != ESP_OK) {
        return ret;
    }

    ret = qma6100p_map_interrupts(sensor, int_num, interrupt_configuration->interrupt_sources);

    if (ret != ESP_OK) {
        return ret;
    }

    ret = qma6100p_enable_interrupts(sensor, interrupt_configuration->interrupt_sources);

    return ret;
}

esp_err_t qma6100p_disable_interrupts(qma6100p_handle_t sensor, uint8_t interrupt_sources)
{
    esp_err_t ret;
    uint8_t enabled_interrupts = 0x00;

    ret = qma6100p_read(sensor, QMA6100P_INTR_ENABLE, &enabled_interrupts, 1);

    if (ESP_OK != ret) {
        return ret;
    }

    if (0 != (enabled_interrupts & interrupt_sources)) {
        enabled_interrupts &= (~interrupt_sources);

        ret = qma6100p_write(sensor, QMA6100P_INTR_ENABLE, enabled_interrupts);
    }

    return ret;
}

esp_err_t qma6100p_get_interrupt_status(qma6100p_handle_t sensor, uint8_t *const out_intr_status)
{
    esp_err_t ret;

    if (NULL == out_intr_status) {
        ret = ESP_ERR_INVALID_ARG;
        return ret;
    }

    ret = qma6100p_read(sensor, QMA6100P_INTR_FIFO_ST, out_intr_status, 1);

    return ret;
}

inline uint8_t qma6100p_is_data_ready_interrupt(uint8_t interrupt_status)
{
    return (QMA6100P_DATA_RDY_INT_BIT == (QMA6100P_DATA_RDY_INT_BIT & interrupt_status));
}

inline uint8_t qma6100p_is_fifo_overflow_interrupt(uint8_t interrupt_status)
{
    return (QMA6100P_FIFO_OF_INT_BIT == (QMA6100P_FIFO_OF_INT_BIT & interrupt_status));
}

inline uint8_t qma6100p_is_fifo_full_interrupt(uint8_t interrupt_status)
{
    return (QMA6100P_FIFO_FULL_INT_BIT == (QMA6100P_FIFO_FULL_INT_BIT & interrupt_status));
}

esp_err_t qma6100p_get_raw_acce(qma6100p_handle_t sensor, qma6100p_raw_acce_value_t *const raw_acce_value)
{
    uint8_t data_rd[6];
    esp_err_t ret = qma6100p_read(sensor, QMA6100P_ACCEL_XOUT_H, data_rd, sizeof(data_rd));

    raw_acce_value->raw_acce_x = (int16_t)((data_rd[1] << 8) + (data_rd[0])) / 4;
    raw_acce_value->raw_acce_y = (int16_t)((data_rd[3] << 8) + (data_rd[2])) / 4;
    raw_acce_value->raw_acce_z = (int16_t)((data_rd[5] << 8) + (data_rd[4])) / 4;
    return ret;
}

esp_err_t qma6100p_get_acce(qma6100p_handle_t sensor, qma6100p_acce_value_t *const acce_value)
{
    esp_err_t ret;
    float acce_sensitivity;
    qma6100p_raw_acce_value_t raw_acce;

    ret = qma6100p_get_acce_sensitivity(sensor, &acce_sensitivity);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = qma6100p_get_raw_acce(sensor, &raw_acce);
    if (ret != ESP_OK) {
        return ret;
    }

    acce_value->acce_x = raw_acce.raw_acce_x / acce_sensitivity;
    acce_value->acce_y = raw_acce.raw_acce_y / acce_sensitivity;
    acce_value->acce_z = raw_acce.raw_acce_z / acce_sensitivity;
    return ESP_OK;
}

esp_err_t qma6100p_get_fifo_frame_counter(qma6100p_handle_t sensor, uint8_t *counter)
{
    return qma6100p_read(sensor, QMA6100P_FIFO_FRAME_CTR, counter, 1);
}

esp_err_t qma6100p_get_fifo_data(qma6100p_handle_t sensor, uint8_t *data)
{
    return qma6100p_read(sensor, QMA6100P_FIFO_DATA, data, 1);
}
