/*
 * SPDX-FileCopyrightText: 2026 Rashed Talukder
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "mpu6886.h"

#define ALPHA                       0.99f        /*!< Weight of gyroscope */
#define RAD_TO_DEG                  57.29577951f  /*!< Radians to degrees (180/π) */

/* MPU6886 register addresses (from datasheet DS-000193 Rev 1.1) */
#define MPU6886_SELF_TEST_X_ACCEL   0x0Du
#define MPU6886_SELF_TEST_Y_ACCEL   0x0Eu
#define MPU6886_SELF_TEST_Z_ACCEL   0x0Fu
#define MPU6886_XG_OFFS_USRH        0x13u
#define MPU6886_XG_OFFS_USRL        0x14u
#define MPU6886_YG_OFFS_USRH        0x15u
#define MPU6886_YG_OFFS_USRL        0x16u
#define MPU6886_ZG_OFFS_USRH        0x17u
#define MPU6886_ZG_OFFS_USRL        0x18u
#define MPU6886_SMPLRT_DIV          0x19u
#define MPU6886_CONFIG              0x1Au
#define MPU6886_GYRO_CONFIG         0x1Bu
#define MPU6886_ACCEL_CONFIG        0x1Cu
#define MPU6886_ACCEL_CONFIG2       0x1Du
#define MPU6886_LP_MODE_CFG         0x1Eu
#define MPU6886_ACCEL_WOM_X_THR     0x20u
#define MPU6886_ACCEL_WOM_Y_THR     0x21u
#define MPU6886_ACCEL_WOM_Z_THR     0x22u
#define MPU6886_FIFO_EN             0x23u
#define MPU6886_FSYNC_INT           0x36u
#define MPU6886_INTR_PIN_CFG        0x37u
#define MPU6886_INTR_ENABLE         0x38u
#define MPU6886_FIFO_WM_INT_STATUS  0x39u
#define MPU6886_INTR_STATUS         0x3Au
#define MPU6886_ACCEL_XOUT_H        0x3Bu
#define MPU6886_TEMP_XOUT_H         0x41u
#define MPU6886_GYRO_XOUT_H         0x43u
#define MPU6886_SELF_TEST_X_GYRO    0x50u
#define MPU6886_SELF_TEST_Y_GYRO    0x51u
#define MPU6886_SELF_TEST_Z_GYRO    0x52u
#define MPU6886_FIFO_WM_TH1         0x60u
#define MPU6886_FIFO_WM_TH2         0x61u
#define MPU6886_SIGNAL_PATH_RESET   0x68u
#define MPU6886_ACCEL_INTEL_CTRL    0x69u
#define MPU6886_USER_CTRL           0x6Au
#define MPU6886_PWR_MGMT_1          0x6Bu
#define MPU6886_PWR_MGMT_2          0x6Cu
#define MPU6886_FIFO_COUNTH         0x72u
#define MPU6886_FIFO_COUNTL         0x73u
#define MPU6886_FIFO_R_W            0x74u
#define MPU6886_WHO_AM_I            0x75u
#define MPU6886_XA_OFFSET_H         0x77u
#define MPU6886_XA_OFFSET_L         0x78u
#define MPU6886_YA_OFFSET_H         0x7Au
#define MPU6886_YA_OFFSET_L         0x7Bu
#define MPU6886_ZA_OFFSET_H         0x7Du
#define MPU6886_ZA_OFFSET_L         0x7Eu

/* Temperature sensor constants from datasheet */
#define MPU6886_TEMP_SENSITIVITY    326.8f
#define MPU6886_TEMP_OFFSET         25.0f

const uint8_t MPU6886_DATA_RDY_INT_BIT =      (uint8_t) BIT0;
const uint8_t MPU6886_GDRIVE_INT_BIT =        (uint8_t) BIT2;
const uint8_t MPU6886_FIFO_OVERFLOW_INT_BIT =  (uint8_t) BIT4;
const uint8_t MPU6886_WOM_Z_INT_BIT =          (uint8_t) BIT5;
const uint8_t MPU6886_WOM_Y_INT_BIT =          (uint8_t) BIT6;
const uint8_t MPU6886_WOM_X_INT_BIT =          (uint8_t) BIT7;
const uint8_t MPU6886_ALL_INTERRUPTS =         (uint8_t)(BIT0 | BIT2 | BIT4 | BIT5 | BIT6 | BIT7);

typedef struct {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
    i2c_master_dev_handle_t i2c_dev;
#else
    i2c_port_t bus;
    uint16_t dev_addr;
#endif
    gpio_num_t int_pin;
    uint32_t counter;
    float dt;  /*!< delay time between two measurements, dt should be small (ms level) */
    struct timeval *timer;
} mpu6886_dev_t;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)

static esp_err_t mpu6886_write(mpu6886_handle_t sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf,
                               const uint8_t data_len)
{
    mpu6886_dev_t *sens = (mpu6886_dev_t *) sensor;
    uint8_t write_buf[data_len + 1];
    write_buf[0] = reg_start_addr;
    memcpy(&write_buf[1], data_buf, data_len);
    return i2c_master_transmit(sens->i2c_dev, write_buf, data_len + 1, 1000);
}

static esp_err_t mpu6886_read(mpu6886_handle_t sensor, const uint8_t reg_start_addr, uint8_t *const data_buf,
                              const uint16_t data_len)
{
    mpu6886_dev_t *sens = (mpu6886_dev_t *) sensor;
    return i2c_master_transmit_receive(sens->i2c_dev, &reg_start_addr, 1, data_buf, data_len, 1000);
}

mpu6886_handle_t mpu6886_create(i2c_master_bus_handle_t bus, const uint16_t dev_addr)
{
    mpu6886_dev_t *sensor = (mpu6886_dev_t *) calloc(1, sizeof(mpu6886_dev_t));
    if (!sensor) {
        return NULL;
    }

    const i2c_device_config_t i2c_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = MPU6886_I2C_CLK_SPEED_HZ,
    };
    if (i2c_master_bus_add_device(bus, &i2c_dev_cfg, &sensor->i2c_dev) != ESP_OK) {
        free(sensor);
        return NULL;
    }

    sensor->counter = 0;
    sensor->dt = 0;
    sensor->timer = (struct timeval *) calloc(1, sizeof(struct timeval));
    return (mpu6886_handle_t) sensor;
}

void mpu6886_delete(mpu6886_handle_t sensor)
{
    mpu6886_dev_t *sens = (mpu6886_dev_t *) sensor;
    i2c_master_bus_rm_device(sens->i2c_dev);
    free(sens->timer);
    free(sens);
}

#else /* Legacy I2C API for IDF < 5.2 */

static esp_err_t mpu6886_write(mpu6886_handle_t sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf,
                               const uint8_t data_len)
{
    mpu6886_dev_t *sens = (mpu6886_dev_t *) sensor;
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_WRITE, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, reg_start_addr, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write(cmd, data_buf, data_len, true);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(sens->bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t mpu6886_read(mpu6886_handle_t sensor, const uint8_t reg_start_addr, uint8_t *const data_buf,
                              const uint16_t data_len)
{
    mpu6886_dev_t *sens = (mpu6886_dev_t *) sensor;
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_WRITE, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, reg_start_addr, true);
    assert(ESP_OK == ret);
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_READ, true);
    assert(ESP_OK == ret);
    ret = i2c_master_read(cmd, data_buf, data_len, I2C_MASTER_LAST_NACK);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(sens->bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

mpu6886_handle_t mpu6886_create(i2c_port_t port, const uint16_t dev_addr)
{
    mpu6886_dev_t *sensor = (mpu6886_dev_t *) calloc(1, sizeof(mpu6886_dev_t));
    if (!sensor) {
        return NULL;
    }
    sensor->bus = port;
    sensor->dev_addr = dev_addr << 1;
    sensor->counter = 0;
    sensor->dt = 0;
    sensor->timer = (struct timeval *) calloc(1, sizeof(struct timeval));
    return (mpu6886_handle_t) sensor;
}

void mpu6886_delete(mpu6886_handle_t sensor)
{
    mpu6886_dev_t *sens = (mpu6886_dev_t *) sensor;
    free(sens->timer);
    free(sens);
}

#endif /* ESP_IDF_VERSION check */

esp_err_t mpu6886_get_deviceid(mpu6886_handle_t sensor, uint8_t *const deviceid)
{
    return mpu6886_read(sensor, MPU6886_WHO_AM_I, deviceid, 1);
}

esp_err_t mpu6886_reset(mpu6886_handle_t sensor)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6886_read(sensor, MPU6886_PWR_MGMT_1, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp |= BIT7;
    return mpu6886_write(sensor, MPU6886_PWR_MGMT_1, &tmp, 1);
}

esp_err_t mpu6886_wake_up(mpu6886_handle_t sensor)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6886_read(sensor, MPU6886_PWR_MGMT_1, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp &= (~BIT6);
    ret = mpu6886_write(sensor, MPU6886_PWR_MGMT_1, &tmp, 1);
    return ret;
}

esp_err_t mpu6886_sleep(mpu6886_handle_t sensor)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6886_read(sensor, MPU6886_PWR_MGMT_1, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp |= BIT6;
    ret = mpu6886_write(sensor, MPU6886_PWR_MGMT_1, &tmp, 1);
    return ret;
}

esp_err_t mpu6886_config(mpu6886_handle_t sensor, const mpu6886_acce_fs_t acce_fs, const mpu6886_gyro_fs_t gyro_fs)
{
    uint8_t gyro_config = gyro_fs << 3;
    uint8_t accel_config = acce_fs << 3;
    esp_err_t ret;

    ret = mpu6886_write(sensor, MPU6886_GYRO_CONFIG, &gyro_config, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    ret = mpu6886_write(sensor, MPU6886_ACCEL_CONFIG, &accel_config, 1);
    return ret;
}

esp_err_t mpu6886_get_acce_sensitivity(mpu6886_handle_t sensor, float *const acce_sensitivity)
{
    esp_err_t ret;
    uint8_t acce_fs;
    ret = mpu6886_read(sensor, MPU6886_ACCEL_CONFIG, &acce_fs, 1);
    acce_fs = (acce_fs >> 3) & 0x03;
    switch (acce_fs) {
    case MPU6886_ACCE_FS_2G:
        *acce_sensitivity = 16384;
        break;
    case MPU6886_ACCE_FS_4G:
        *acce_sensitivity = 8192;
        break;
    case MPU6886_ACCE_FS_8G:
        *acce_sensitivity = 4096;
        break;
    case MPU6886_ACCE_FS_16G:
        *acce_sensitivity = 2048;
        break;
    default:
        break;
    }
    return ret;
}

esp_err_t mpu6886_get_gyro_sensitivity(mpu6886_handle_t sensor, float *const gyro_sensitivity)
{
    esp_err_t ret;
    uint8_t gyro_fs;
    ret = mpu6886_read(sensor, MPU6886_GYRO_CONFIG, &gyro_fs, 1);
    gyro_fs = (gyro_fs >> 3) & 0x03;
    switch (gyro_fs) {
    case MPU6886_GYRO_FS_250DPS:
        *gyro_sensitivity = 131;
        break;
    case MPU6886_GYRO_FS_500DPS:
        *gyro_sensitivity = 65.5;
        break;
    case MPU6886_GYRO_FS_1000DPS:
        *gyro_sensitivity = 32.8;
        break;
    case MPU6886_GYRO_FS_2000DPS:
        *gyro_sensitivity = 16.4;
        break;
    default:
        break;
    }
    return ret;
}

esp_err_t mpu6886_set_clock_source(mpu6886_handle_t sensor, mpu6886_clk_src_t clk_src)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6886_read(sensor, MPU6886_PWR_MGMT_1, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp = (tmp & 0xF8) | (clk_src & 0x07);
    return mpu6886_write(sensor, MPU6886_PWR_MGMT_1, &tmp, 1);
}

esp_err_t mpu6886_set_sample_rate_divider(mpu6886_handle_t sensor, uint8_t divider)
{
    return mpu6886_write(sensor, MPU6886_SMPLRT_DIV, &divider, 1);
}

esp_err_t mpu6886_set_gyro_dlpf(mpu6886_handle_t sensor, mpu6886_dlpf_t dlpf)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6886_read(sensor, MPU6886_CONFIG, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp = (tmp & 0xF8) | (dlpf & 0x07);
    return mpu6886_write(sensor, MPU6886_CONFIG, &tmp, 1);
}

esp_err_t mpu6886_set_acce_dlpf(mpu6886_handle_t sensor, mpu6886_acce_dlpf_t dlpf)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6886_read(sensor, MPU6886_ACCEL_CONFIG2, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp = (tmp & 0xF0) | (dlpf & 0x07);
    /* Clear ACCEL_FCHOICE_B (bit 3) to enable DLPF */
    tmp &= ~BIT3;
    return mpu6886_write(sensor, MPU6886_ACCEL_CONFIG2, &tmp, 1);
}

esp_err_t mpu6886_set_fsync(mpu6886_handle_t sensor, mpu6886_fsync_out_t fsync_out)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6886_read(sensor, MPU6886_CONFIG, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp = (tmp & 0xC7) | ((fsync_out & 0x07) << 3);
    return mpu6886_write(sensor, MPU6886_CONFIG, &tmp, 1);
}

esp_err_t mpu6886_set_gyro_low_power(mpu6886_handle_t sensor, bool enable, mpu6886_gyro_avg_t avg)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6886_read(sensor, MPU6886_LP_MODE_CFG, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    if (enable) {
        tmp |= BIT7;
    } else {
        tmp &= ~BIT7;
    }
    tmp = (tmp & 0x8F) | ((avg & 0x07) << 4);
    return mpu6886_write(sensor, MPU6886_LP_MODE_CFG, &tmp, 1);
}

esp_err_t mpu6886_set_acce_averaging(mpu6886_handle_t sensor, mpu6886_acce_avg_t avg)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6886_read(sensor, MPU6886_ACCEL_CONFIG2, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp = (tmp & 0xCF) | ((avg & 0x03) << 4);
    return mpu6886_write(sensor, MPU6886_ACCEL_CONFIG2, &tmp, 1);
}

esp_err_t mpu6886_set_acce_low_power_cycle(mpu6886_handle_t sensor, bool enable)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6886_read(sensor, MPU6886_PWR_MGMT_1, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    if (enable) {
        tmp |= BIT5;
    } else {
        tmp &= ~BIT5;
    }
    return mpu6886_write(sensor, MPU6886_PWR_MGMT_1, &tmp, 1);
}

esp_err_t mpu6886_config_wom(mpu6886_handle_t sensor, uint8_t wom_x_thr, uint8_t wom_y_thr,
                             uint8_t wom_z_thr, mpu6886_wom_threshold_mode_t mode)
{
    esp_err_t ret;

    ret = mpu6886_write(sensor, MPU6886_ACCEL_WOM_X_THR, &wom_x_thr, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    ret = mpu6886_write(sensor, MPU6886_ACCEL_WOM_Y_THR, &wom_y_thr, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    ret = mpu6886_write(sensor, MPU6886_ACCEL_WOM_Z_THR, &wom_z_thr, 1);
    if (ESP_OK != ret) {
        return ret;
    }

    /* Enable accel intelligence, set mode to compare current vs previous, set threshold mode */
    uint8_t intel_ctrl;
    ret = mpu6886_read(sensor, MPU6886_ACCEL_INTEL_CTRL, &intel_ctrl, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    intel_ctrl |= BIT7;  /* ACCEL_INTEL_EN */
    intel_ctrl |= BIT6;  /* ACCEL_INTEL_MODE = 1 (compare current with previous) */
    if (mode == MPU6886_WOM_AND) {
        intel_ctrl |= BIT0;
    } else {
        intel_ctrl &= ~BIT0;
    }
    return mpu6886_write(sensor, MPU6886_ACCEL_INTEL_CTRL, &intel_ctrl, 1);
}

esp_err_t mpu6886_set_axes_disable(mpu6886_handle_t sensor, uint8_t disable_mask)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6886_read(sensor, MPU6886_PWR_MGMT_2, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp = (tmp & 0xC0) | (disable_mask & 0x3F);
    return mpu6886_write(sensor, MPU6886_PWR_MGMT_2, &tmp, 1);
}

esp_err_t mpu6886_set_temp_disable(mpu6886_handle_t sensor, bool disable)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6886_read(sensor, MPU6886_PWR_MGMT_1, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    if (disable) {
        tmp |= BIT3;
    } else {
        tmp &= ~BIT3;
    }
    return mpu6886_write(sensor, MPU6886_PWR_MGMT_1, &tmp, 1);
}

esp_err_t mpu6886_set_gyro_standby(mpu6886_handle_t sensor, bool enable)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6886_read(sensor, MPU6886_PWR_MGMT_1, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    if (enable) {
        tmp |= BIT4;
    } else {
        tmp &= ~BIT4;
    }
    return mpu6886_write(sensor, MPU6886_PWR_MGMT_1, &tmp, 1);
}

esp_err_t mpu6886_enable_fifo(mpu6886_handle_t sensor, bool enable_fifo, bool gyro_fifo_en, bool acce_fifo_en)
{
    esp_err_t ret;
    uint8_t tmp;

    /* Set FIFO_EN bit in USER_CTRL (register 0x6A) */
    ret = mpu6886_read(sensor, MPU6886_USER_CTRL, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    if (enable_fifo) {
        tmp |= BIT6;
    } else {
        tmp &= ~BIT6;
    }
    ret = mpu6886_write(sensor, MPU6886_USER_CTRL, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }

    /* Configure which data to write to FIFO (register 0x23) */
    uint8_t fifo_en = 0;
    if (gyro_fifo_en) {
        fifo_en |= BIT4;
    }
    if (acce_fifo_en) {
        fifo_en |= BIT3;
    }
    return mpu6886_write(sensor, MPU6886_FIFO_EN, &fifo_en, 1);
}

esp_err_t mpu6886_reset_fifo(mpu6886_handle_t sensor)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6886_read(sensor, MPU6886_USER_CTRL, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp |= BIT2;
    return mpu6886_write(sensor, MPU6886_USER_CTRL, &tmp, 1);
}

esp_err_t mpu6886_get_fifo_count(mpu6886_handle_t sensor, uint16_t *count)
{
    uint8_t data[2];
    esp_err_t ret = mpu6886_read(sensor, MPU6886_FIFO_COUNTH, data, 2);
    *count = ((uint16_t)data[0] << 8) | data[1];
    return ret;
}

esp_err_t mpu6886_read_fifo(mpu6886_handle_t sensor, uint8_t *buf, uint16_t len)
{
    return mpu6886_read(sensor, MPU6886_FIFO_R_W, buf, len);
}

esp_err_t mpu6886_set_fifo_watermark(mpu6886_handle_t sensor, uint16_t threshold)
{
    esp_err_t ret;
    uint8_t th1 = (threshold >> 8) & 0x03;
    uint8_t th2 = threshold & 0xFF;
    ret = mpu6886_write(sensor, MPU6886_FIFO_WM_TH1, &th1, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    return mpu6886_write(sensor, MPU6886_FIFO_WM_TH2, &th2, 1);
}

esp_err_t mpu6886_get_fifo_wm_status(mpu6886_handle_t sensor, bool *status)
{
    uint8_t tmp;
    esp_err_t ret = mpu6886_read(sensor, MPU6886_FIFO_WM_INT_STATUS, &tmp, 1);
    *status = (tmp & BIT6) != 0;
    return ret;
}

esp_err_t mpu6886_reset_signal_path(mpu6886_handle_t sensor)
{
    uint8_t tmp = BIT1 | BIT0;  /* ACCEL_RST | TEMP_RST */
    return mpu6886_write(sensor, MPU6886_SIGNAL_PATH_RESET, &tmp, 1);
}

esp_err_t mpu6886_reset_signal_cond(mpu6886_handle_t sensor)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6886_read(sensor, MPU6886_USER_CTRL, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp |= BIT0;  /* SIG_COND_RST */
    return mpu6886_write(sensor, MPU6886_USER_CTRL, &tmp, 1);
}

esp_err_t mpu6886_set_output_limit(mpu6886_handle_t sensor)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6886_read(sensor, MPU6886_ACCEL_INTEL_CTRL, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp |= BIT1;  /* OUTPUT_LIMIT = 1 per datasheet requirement */
    return mpu6886_write(sensor, MPU6886_ACCEL_INTEL_CTRL, &tmp, 1);
}

esp_err_t mpu6886_set_gyro_offset(mpu6886_handle_t sensor, int16_t x_offset, int16_t y_offset, int16_t z_offset)
{
    esp_err_t ret;
    uint8_t data[2];

    data[0] = (x_offset >> 8) & 0xFF;
    data[1] = x_offset & 0xFF;
    ret = mpu6886_write(sensor, MPU6886_XG_OFFS_USRH, data, 2);
    if (ESP_OK != ret) {
        return ret;
    }

    data[0] = (y_offset >> 8) & 0xFF;
    data[1] = y_offset & 0xFF;
    ret = mpu6886_write(sensor, MPU6886_YG_OFFS_USRH, data, 2);
    if (ESP_OK != ret) {
        return ret;
    }

    data[0] = (z_offset >> 8) & 0xFF;
    data[1] = z_offset & 0xFF;
    return mpu6886_write(sensor, MPU6886_ZG_OFFS_USRH, data, 2);
}

esp_err_t mpu6886_set_acce_offset(mpu6886_handle_t sensor, int16_t x_offset, int16_t y_offset, int16_t z_offset)
{
    esp_err_t ret;
    uint8_t data[2];

    /* Accel offset is 15-bit (bits [14:0] across two registers, bit 0 of low byte is reserved) */
    data[0] = (x_offset >> 7) & 0xFF;
    data[1] = (x_offset << 1) & 0xFE;
    ret = mpu6886_write(sensor, MPU6886_XA_OFFSET_H, data, 2);
    if (ESP_OK != ret) {
        return ret;
    }

    data[0] = (y_offset >> 7) & 0xFF;
    data[1] = (y_offset << 1) & 0xFE;
    ret = mpu6886_write(sensor, MPU6886_YA_OFFSET_H, data, 2);
    if (ESP_OK != ret) {
        return ret;
    }

    data[0] = (z_offset >> 7) & 0xFF;
    data[1] = (z_offset << 1) & 0xFE;
    return mpu6886_write(sensor, MPU6886_ZA_OFFSET_H, data, 2);
}

esp_err_t mpu6886_get_acce_self_test(mpu6886_handle_t sensor, uint8_t *x_result, uint8_t *y_result, uint8_t *z_result)
{
    esp_err_t ret;
    ret = mpu6886_read(sensor, MPU6886_SELF_TEST_X_ACCEL, x_result, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    ret = mpu6886_read(sensor, MPU6886_SELF_TEST_Y_ACCEL, y_result, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    return mpu6886_read(sensor, MPU6886_SELF_TEST_Z_ACCEL, z_result, 1);
}

esp_err_t mpu6886_get_gyro_self_test(mpu6886_handle_t sensor, uint8_t *x_result, uint8_t *y_result, uint8_t *z_result)
{
    esp_err_t ret;
    ret = mpu6886_read(sensor, MPU6886_SELF_TEST_X_GYRO, x_result, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    ret = mpu6886_read(sensor, MPU6886_SELF_TEST_Y_GYRO, y_result, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    return mpu6886_read(sensor, MPU6886_SELF_TEST_Z_GYRO, z_result, 1);
}

esp_err_t mpu6886_config_interrupts(mpu6886_handle_t sensor, const mpu6886_int_config_t *const interrupt_configuration)
{
    esp_err_t ret = ESP_OK;

    if (NULL == interrupt_configuration) {
        return ESP_ERR_INVALID_ARG;
    }

    if (GPIO_IS_VALID_GPIO(interrupt_configuration->interrupt_pin)) {
        mpu6886_dev_t *sensor_device = (mpu6886_dev_t *) sensor;
        sensor_device->int_pin = interrupt_configuration->interrupt_pin;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t int_pin_cfg = 0x00;

    ret = mpu6886_read(sensor, MPU6886_INTR_PIN_CFG, &int_pin_cfg, 1);
    if (ESP_OK != ret) {
        return ret;
    }

    if (MPU6886_INTERRUPT_PIN_ACTIVE_LOW == interrupt_configuration->active_level) {
        int_pin_cfg |= BIT7;
    } else {
        int_pin_cfg &= ~BIT7;
    }

    if (MPU6886_INTERRUPT_PIN_OPEN_DRAIN == interrupt_configuration->pin_mode) {
        int_pin_cfg |= BIT6;
    } else {
        int_pin_cfg &= ~BIT6;
    }

    if (MPU6886_INTERRUPT_LATCH_UNTIL_CLEARED == interrupt_configuration->interrupt_latch) {
        int_pin_cfg |= BIT5;
    } else {
        int_pin_cfg &= ~BIT5;
    }

    if (MPU6886_INTERRUPT_CLEAR_ON_ANY_READ == interrupt_configuration->interrupt_clear_behavior) {
        int_pin_cfg |= BIT4;
    } else {
        int_pin_cfg &= ~BIT4;
    }

    ret = mpu6886_write(sensor, MPU6886_INTR_PIN_CFG, &int_pin_cfg, 1);
    if (ESP_OK != ret) {
        return ret;
    }

    gpio_int_type_t gpio_intr_type;
    if (MPU6886_INTERRUPT_PIN_ACTIVE_LOW == interrupt_configuration->active_level) {
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
    return ret;
}

esp_err_t mpu6886_register_isr(mpu6886_handle_t sensor, const mpu6886_isr_t isr)
{
    esp_err_t ret;
    mpu6886_dev_t *sensor_device = (mpu6886_dev_t *) sensor;

    if (NULL == sensor_device) {
        return ESP_ERR_INVALID_ARG;
    }

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

esp_err_t mpu6886_enable_interrupts(mpu6886_handle_t sensor, uint8_t interrupt_sources)
{
    esp_err_t ret;
    uint8_t enabled_interrupts = 0x00;

    ret = mpu6886_read(sensor, MPU6886_INTR_ENABLE, &enabled_interrupts, 1);
    if (ESP_OK != ret) {
        return ret;
    }

    if (enabled_interrupts != interrupt_sources) {
        enabled_interrupts |= interrupt_sources;
        ret = mpu6886_write(sensor, MPU6886_INTR_ENABLE, &enabled_interrupts, 1);
    }

    return ret;
}

esp_err_t mpu6886_disable_interrupts(mpu6886_handle_t sensor, uint8_t interrupt_sources)
{
    esp_err_t ret;
    uint8_t enabled_interrupts = 0x00;

    ret = mpu6886_read(sensor, MPU6886_INTR_ENABLE, &enabled_interrupts, 1);
    if (ESP_OK != ret) {
        return ret;
    }

    if (0 != (enabled_interrupts & interrupt_sources)) {
        enabled_interrupts &= (~interrupt_sources);
        ret = mpu6886_write(sensor, MPU6886_INTR_ENABLE, &enabled_interrupts, 1);
    }

    return ret;
}

esp_err_t mpu6886_get_interrupt_status(mpu6886_handle_t sensor, uint8_t *const out_intr_status)
{
    if (NULL == out_intr_status) {
        return ESP_ERR_INVALID_ARG;
    }

    return mpu6886_read(sensor, MPU6886_INTR_STATUS, out_intr_status, 1);
}

esp_err_t mpu6886_get_raw_acce(mpu6886_handle_t sensor, mpu6886_raw_acce_value_t *const raw_acce_value)
{
    uint8_t data_rd[6];
    esp_err_t ret = mpu6886_read(sensor, MPU6886_ACCEL_XOUT_H, data_rd, sizeof(data_rd));

    raw_acce_value->raw_acce_x = (int16_t)((data_rd[0] << 8) | (data_rd[1]));
    raw_acce_value->raw_acce_y = (int16_t)((data_rd[2] << 8) | (data_rd[3]));
    raw_acce_value->raw_acce_z = (int16_t)((data_rd[4] << 8) | (data_rd[5]));
    return ret;
}

esp_err_t mpu6886_get_raw_gyro(mpu6886_handle_t sensor, mpu6886_raw_gyro_value_t *const raw_gyro_value)
{
    uint8_t data_rd[6];
    esp_err_t ret = mpu6886_read(sensor, MPU6886_GYRO_XOUT_H, data_rd, sizeof(data_rd));

    raw_gyro_value->raw_gyro_x = (int16_t)((data_rd[0] << 8) | (data_rd[1]));
    raw_gyro_value->raw_gyro_y = (int16_t)((data_rd[2] << 8) | (data_rd[3]));
    raw_gyro_value->raw_gyro_z = (int16_t)((data_rd[4] << 8) | (data_rd[5]));
    return ret;
}

esp_err_t mpu6886_get_acce(mpu6886_handle_t sensor, mpu6886_acce_value_t *const acce_value)
{
    esp_err_t ret;
    float acce_sensitivity;
    mpu6886_raw_acce_value_t raw_acce;

    ret = mpu6886_get_acce_sensitivity(sensor, &acce_sensitivity);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = mpu6886_get_raw_acce(sensor, &raw_acce);
    if (ret != ESP_OK) {
        return ret;
    }

    acce_value->acce_x = raw_acce.raw_acce_x / acce_sensitivity;
    acce_value->acce_y = raw_acce.raw_acce_y / acce_sensitivity;
    acce_value->acce_z = raw_acce.raw_acce_z / acce_sensitivity;
    return ESP_OK;
}

esp_err_t mpu6886_get_gyro(mpu6886_handle_t sensor, mpu6886_gyro_value_t *const gyro_value)
{
    esp_err_t ret;
    float gyro_sensitivity;
    mpu6886_raw_gyro_value_t raw_gyro;

    ret = mpu6886_get_gyro_sensitivity(sensor, &gyro_sensitivity);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = mpu6886_get_raw_gyro(sensor, &raw_gyro);
    if (ret != ESP_OK) {
        return ret;
    }

    gyro_value->gyro_x = raw_gyro.raw_gyro_x / gyro_sensitivity;
    gyro_value->gyro_y = raw_gyro.raw_gyro_y / gyro_sensitivity;
    gyro_value->gyro_z = raw_gyro.raw_gyro_z / gyro_sensitivity;
    return ESP_OK;
}

esp_err_t mpu6886_get_temp(mpu6886_handle_t sensor, mpu6886_temp_value_t *const temp_value)
{
    uint8_t data_rd[2];
    esp_err_t ret = mpu6886_read(sensor, MPU6886_TEMP_XOUT_H, data_rd, sizeof(data_rd));
    /* MPU-6886 temperature formula: TEMP_degC = (TEMP_OUT / 326.8) + 25 */
    temp_value->temp = (int16_t)((data_rd[0] << 8) | (data_rd[1])) / MPU6886_TEMP_SENSITIVITY + MPU6886_TEMP_OFFSET;
    return ret;
}

esp_err_t mpu6886_complementary_filter(mpu6886_handle_t sensor, const mpu6886_acce_value_t *const acce_value,
                                       const mpu6886_gyro_value_t *const gyro_value, mpu6886_complementary_angle_t *const complementary_angle)
{
    float acce_angle[2];
    float gyro_angle[2];
    float gyro_rate[2];
    mpu6886_dev_t *sens = (mpu6886_dev_t *) sensor;

    sens->counter++;
    if (sens->counter == 1) {
        acce_angle[0] = (atan2(acce_value->acce_y, acce_value->acce_z) * RAD_TO_DEG);
        acce_angle[1] = (atan2(acce_value->acce_x, acce_value->acce_z) * RAD_TO_DEG);
        complementary_angle->roll = acce_angle[0];
        complementary_angle->pitch = acce_angle[1];
        gettimeofday(sens->timer, NULL);
        return ESP_OK;
    }

    struct timeval now, dt_t;
    gettimeofday(&now, NULL);
    timersub(&now, sens->timer, &dt_t);
    sens->dt = (float) (dt_t.tv_sec) + (float)dt_t.tv_usec / 1000000;
    gettimeofday(sens->timer, NULL);

    acce_angle[0] = (atan2(acce_value->acce_y, acce_value->acce_z) * RAD_TO_DEG);
    acce_angle[1] = (atan2(acce_value->acce_x, acce_value->acce_z) * RAD_TO_DEG);

    gyro_rate[0] = gyro_value->gyro_x;
    gyro_rate[1] = gyro_value->gyro_y;
    gyro_angle[0] = gyro_rate[0] * sens->dt;
    gyro_angle[1] = gyro_rate[1] * sens->dt;

    complementary_angle->roll = (ALPHA * (complementary_angle->roll + gyro_angle[0])) + ((1 - ALPHA) * acce_angle[0]);
    complementary_angle->pitch = (ALPHA * (complementary_angle->pitch + gyro_angle[1])) + ((1 - ALPHA) * acce_angle[1]);

    return ESP_OK;
}
