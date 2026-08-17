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
#include "freertos/task.h"

#include "rx8130ce.h"

#define RX8130CE_REG_SECONDS          0x10
#define RX8130CE_REG_ALARM_MINUTE     0x17
#define RX8130CE_REG_TIMER_COUNTER0   0x1A
#define RX8130CE_REG_EXTENSION        0x1C
#define RX8130CE_REG_FLAGS            0x1D
#define RX8130CE_REG_CONTROL0         0x1E
#define RX8130CE_REG_CONTROL1         0x1F
#define RX8130CE_REG_RAM              0x20
#define RX8130CE_REG_DIGITAL_OFFSET   0x30
#define RX8130CE_REG_EXTENSION1       0x31
#define RX8130CE_ALARM_AE             (1U << 7)
#define RX8130CE_EXTENSION_TE         (1U << 4)
#define RX8130CE_EXTENSION_WADA       (1U << 3)
#define RX8130CE_EXTENSION_TSEL_MASK  0x07
#define RX8130CE_CONTROL0_STOP        (1U << 6)
#define RX8130CE_CONTROL0_UIE         (1U << 5)
#define RX8130CE_CONTROL0_TIE         (1U << 4)
#define RX8130CE_CONTROL0_AIE         (1U << 3)
#define RX8130CE_CONTROL1_CHGEN       (1U << 5)
#define RX8130CE_CONTROL1_INIEN       (1U << 4)
#define RX8130CE_CONTROL1_BFVSEL_MASK 0x03
#define RX8130CE_EXTENSION1_VBLFE     (1U << 0)
#define RX8130CE_INTERRUPT_FLAGS      (RX8130CE_FLAG_UF | RX8130CE_FLAG_TF | RX8130CE_FLAG_AF)
#define RX8130CE_TIMEOUT_MS           100
#define RX8130CE_BACKUP_RECOVERY_MS   35
/* Oscillation start time t_str is 1.0 s max (appman 8); appman 10.2
 * requires waiting it out before initializing when VLF=1. */
#define RX8130CE_OSCILLATOR_START_MS  1100

struct rx8130ce_device_t {
    i2c_master_dev_handle_t i2c_device;
    SemaphoreHandle_t lock;
    /* Backup policy copied from the create-time configuration so the
     * runtime charge switch re-applies the same settings. */
    rx8130ce_charge_cutoff_t charge_cutoff;
    bool backup_voltage_low_detect;
};

static const char *TAG = "rx8130ce";

static esp_err_t lock_device(rx8130ce_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL && handle->i2c_device != NULL &&
                        handle->lock != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid device handle");
    ESP_RETURN_ON_FALSE(xSemaphoreTake(handle->lock, portMAX_DELAY) == pdTRUE,
                        ESP_ERR_TIMEOUT, TAG, "device lock failed");
    return ESP_OK;
}

static void unlock_device(rx8130ce_handle_t handle)
{
    xSemaphoreGive(handle->lock);
}

/* The register helpers and the *_locked workers below expect the caller to
 * hold the device lock (or to run before the handle is published). */

static esp_err_t rx8130ce_read(rx8130ce_handle_t handle, uint8_t reg,
                               void *data, size_t size)
{
    ESP_RETURN_ON_FALSE(handle != NULL && handle->i2c_device != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid device handle");
    ESP_RETURN_ON_FALSE(data != NULL && size > 0, ESP_ERR_INVALID_ARG, TAG,
                        "invalid read buffer");
    return i2c_master_transmit_receive(handle->i2c_device, &reg, 1, data,
                                       size, RX8130CE_TIMEOUT_MS);
}

static esp_err_t rx8130ce_write(rx8130ce_handle_t handle, uint8_t reg,
                                const void *data, size_t size)
{
    ESP_RETURN_ON_FALSE(handle != NULL && handle->i2c_device != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid device handle");
    ESP_RETURN_ON_FALSE(data != NULL && size > 0 && size <= 7,
                        ESP_ERR_INVALID_ARG, TAG, "invalid write buffer");
    uint8_t buffer[8];
    buffer[0] = reg;
    memcpy(&buffer[1], data, size);
    return i2c_master_transmit(handle->i2c_device, buffer, size + 1,
                               RX8130CE_TIMEOUT_MS);
}

static uint8_t bcd_to_binary(uint8_t value)
{
    return (uint8_t)((value >> 4) * 10 + (value & 0x0F));
}

static uint8_t binary_to_bcd(uint8_t value)
{
    return (uint8_t)(((value / 10) << 4) | (value % 10));
}

static bool bcd_is_valid(uint8_t value)
{
    return (value & 0x0F) <= 9 && (value >> 4) <= 9;
}

bool rx8130ce_time_is_valid(const rx8130ce_time_t *time)
{
    if (time == NULL || time->year < 2000 || time->year > 2099 ||
            time->month < 1 || time->month > 12 || time->weekday > 6 ||
            time->hour > 23 || time->minute > 59 || time->second > 59) {
        return false;
    }
    static const uint8_t days_per_month[] = {
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
    };
    uint8_t days = days_per_month[time->month - 1];
    if (time->month == 2 && (time->year % 4) == 0) {
        ++days;
    }
    return time->day >= 1 && time->day <= days;
}

bool rx8130ce_alarm_is_valid(const rx8130ce_alarm_t *alarm)
{
    if (alarm == NULL || (alarm->minute_en && alarm->minute > 59) ||
            (alarm->hour_en && alarm->hour > 23) ||
            (alarm->day_en && (alarm->day < 1 || alarm->day > 31)) ||
            (alarm->weekday_en && alarm->weekday > 6)) {
        return false;
    }
    /* Day and weekday share register 19h, so they cannot both compare. */
    return !(alarm->day_en && alarm->weekday_en);
}

void rx8130ce_alarm_encode(const rx8130ce_alarm_t *alarm,
                           uint8_t registers[3], bool *use_day_alarm)
{
    /* AE is active-low: a set bit excludes the field from the comparison. */
    registers[0] = alarm->minute_en ? binary_to_bcd(alarm->minute)
                   : RX8130CE_ALARM_AE;
    registers[1] = alarm->hour_en ? binary_to_bcd(alarm->hour)
                   : RX8130CE_ALARM_AE;
    if (alarm->day_en) {
        registers[2] = binary_to_bcd(alarm->day);
    } else if (alarm->weekday_en) {
        registers[2] = (uint8_t)(1U << alarm->weekday);
    } else {
        registers[2] = RX8130CE_ALARM_AE;
    }
    *use_day_alarm = alarm->day_en;
}

/* Days since 2000-01-01 for a validated calendar value. Inputs always pass
 * rx8130ce_time_is_valid(), so the year stays within 2000-2099 and the only
 * leap-year rule needed is year % 4 == 0 (2000 is leap, 2100 out of range). */
static uint32_t days_since_2000(const rx8130ce_time_t *time)
{
    static const uint16_t month_offset[12] = {
        0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334,
    };
    const uint16_t year = time->year;
    uint32_t days = (uint32_t)(year - 2000) * 365 +
                    (uint32_t)((year - 1) / 4 - 499);
    days += month_offset[time->month - 1];
    if (time->month > 2 && (year % 4) == 0) {
        ++days;
    }
    return days + time->day - 1;
}

esp_err_t rx8130ce_alarm_from_time(const rx8130ce_time_t *now,
                                   const rx8130ce_time_t *target,
                                   rx8130ce_alarm_t *out_alarm)
{
    ESP_RETURN_ON_FALSE(rx8130ce_time_is_valid(now) &&
                        rx8130ce_time_is_valid(target) && out_alarm != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid time argument");
    const uint32_t now_days = days_since_2000(now);
    const uint32_t target_days = days_since_2000(target);
    /* Minute granularity: the target minute must be strictly ahead. */
    ESP_RETURN_ON_FALSE(target_days > now_days ||
                        (target_days == now_days &&
                         (target->hour > now->hour ||
                          (target->hour == now->hour &&
                           target->minute > now->minute))),
                        ESP_ERR_INVALID_ARG, TAG, "target is not in the future");
    /* Beyond this horizon an earlier monthly recurrence of the compare
     * pattern could match first; see RX8130CE_ALARM_MAX_FUTURE_DAYS. */
    ESP_RETURN_ON_FALSE(target_days - now_days <= RX8130CE_ALARM_MAX_FUTURE_DAYS,
                        ESP_ERR_INVALID_ARG, TAG, "target beyond alarm horizon");
    *out_alarm = (rx8130ce_alarm_t) {
        .minute_en = true,
        .minute = target->minute,
        .hour_en = true,
        .hour = target->hour,
        .day_en = true,
        .day = target->day,
    };
    return ESP_OK;
}

bool rx8130ce_timer_is_valid(const rx8130ce_timer_t *timer)
{
    return timer != NULL &&
           timer->source_clock <= RX8130CE_TIMER_SOURCE_1_3600HZ &&
           timer->count >= 1;
}

void rx8130ce_timer_encode(const rx8130ce_timer_t *timer,
                           uint8_t registers[2], uint8_t *tsel_bits)
{
    /* Timer Counter 0/1 hold the preset low byte first (appman 14.2.2). */
    registers[0] = (uint8_t)(timer->count & 0xFF);
    registers[1] = (uint8_t)(timer->count >> 8);
    *tsel_bits = (uint8_t)timer->source_clock;
}

static esp_err_t decode_time(const uint8_t data[7], rx8130ce_time_t *time)
{
    const uint8_t second = data[0] & 0x7F;
    const uint8_t minute = data[1] & 0x7F;
    const uint8_t hour = data[2] & 0x3F;
    const uint8_t weekday_bits = data[3] & 0x7F;
    const uint8_t day = data[4] & 0x3F;
    const uint8_t month = data[5] & 0x1F;
    const uint8_t year = data[6];

    ESP_RETURN_ON_FALSE(bcd_is_valid(second) && bcd_is_valid(minute) &&
                        bcd_is_valid(hour) && bcd_is_valid(day) &&
                        bcd_is_valid(month) && bcd_is_valid(year),
                        ESP_ERR_INVALID_RESPONSE, TAG, "invalid BCD calendar data");
    ESP_RETURN_ON_FALSE(weekday_bits != 0 &&
                        (weekday_bits & (weekday_bits - 1U)) == 0,
                        ESP_ERR_INVALID_RESPONSE, TAG, "invalid weekday data");

    uint8_t weekday = 0;
    while ((weekday_bits & (1U << weekday)) == 0) {
        ++weekday;
    }
    *time = (rx8130ce_time_t) {
        .year = (uint16_t)(2000 + bcd_to_binary(year)),
        .month = bcd_to_binary(month),
        .day = bcd_to_binary(day),
        .weekday = weekday,
        .hour = bcd_to_binary(hour),
        .minute = bcd_to_binary(minute),
        .second = bcd_to_binary(second),
    };
    ESP_RETURN_ON_FALSE(rx8130ce_time_is_valid(time), ESP_ERR_INVALID_RESPONSE,
                        TAG, "calendar value is out of range");
    return ESP_OK;
}

static esp_err_t decode_alarm(const uint8_t data[3], bool use_day_alarm,
                              rx8130ce_alarm_t *alarm)
{
    *alarm = (rx8130ce_alarm_t) {
        .minute_en = (data[0] & RX8130CE_ALARM_AE) == 0,
        .minute = bcd_to_binary(data[0] & 0x7F),
        .hour_en = (data[1] & RX8130CE_ALARM_AE) == 0,
        .hour = bcd_to_binary(data[1] & 0x3F),
    };
    ESP_RETURN_ON_FALSE(bcd_is_valid(data[0] & 0x7F) &&
                        bcd_is_valid(data[1] & 0x3F),
                        ESP_ERR_INVALID_RESPONSE, TAG, "invalid BCD alarm data");
    if (use_day_alarm) {
        ESP_RETURN_ON_FALSE(bcd_is_valid(data[2] & 0x3F),
                            ESP_ERR_INVALID_RESPONSE, TAG,
                            "invalid BCD alarm data");
        alarm->day_en = (data[2] & RX8130CE_ALARM_AE) == 0;
        alarm->day = bcd_to_binary(data[2] & 0x3F);
    } else {
        const uint8_t weekday_bits = data[2] & 0x7F;
        alarm->weekday_en = (data[2] & RX8130CE_ALARM_AE) == 0 &&
                            weekday_bits != 0;
        /* The hardware accepts a weekday mask; report the lowest set day. */
        while (alarm->weekday < 6 &&
                (weekday_bits & (1U << alarm->weekday)) == 0) {
            ++alarm->weekday;
        }
    }
    ESP_RETURN_ON_FALSE(rx8130ce_alarm_is_valid(alarm),
                        ESP_ERR_INVALID_RESPONSE, TAG,
                        "alarm value is out of range");
    return ESP_OK;
}

static void decode_status(uint8_t flags, rx8130ce_status_t *status)
{
    *status = (rx8130ce_status_t) {
        .raw = flags,
        .time_valid = (flags & RX8130CE_FLAG_VLF) == 0,
        .alarm = (flags & RX8130CE_FLAG_AF) != 0,
        .timer = (flags & RX8130CE_FLAG_TF) != 0,
        .update = (flags & RX8130CE_FLAG_UF) != 0,
        .reset = (flags & RX8130CE_FLAG_RSF) != 0,
        .backup_voltage_low = (flags & RX8130CE_FLAG_VBLF) != 0,
        .backup_battery_full = (flags & RX8130CE_FLAG_VBFF) != 0,
    };
}

static esp_err_t rx8130ce_configure_backup_supply(rx8130ce_handle_t handle,
        bool charge_enable)
{
    /* Appman 14.7.2: INIEN=1 enables automatic supply switchover; CHGEN
     * selects charging, which suits only rechargeable backup sources.
     * BFVSEL changes in the same write as CHGEN, so charging never runs
     * with a retained setting (the cutoff-less 11b is unreachable). */
    uint8_t control1 = 0;
    ESP_RETURN_ON_ERROR(rx8130ce_read(handle, RX8130CE_REG_CONTROL1,
                                      &control1, 1), TAG,
                        "backup control read failed");
    control1 &= ~RX8130CE_CONTROL1_BFVSEL_MASK;
    control1 |= (uint8_t)handle->charge_cutoff;
    if (charge_enable) {
        control1 |= RX8130CE_CONTROL1_CHGEN;
    } else {
        control1 &= ~RX8130CE_CONTROL1_CHGEN;
    }
    control1 |= RX8130CE_CONTROL1_INIEN;
    ESP_RETURN_ON_ERROR(rx8130ce_write(handle, RX8130CE_REG_CONTROL1,
                                       &control1, 1), TAG,
                        "backup control write failed");
    /* Appman Table 42: VBLFE=1 makes VBLF detectable while CHGEN=0;
     * bits 7-1 of 31h are manufacturer test bits kept at 0 (Table 12). */
    const uint8_t extension1 = handle->backup_voltage_low_detect
                               ? RX8130CE_EXTENSION1_VBLFE : 0;
    return rx8130ce_write(handle, RX8130CE_REG_EXTENSION1, &extension1, 1);
}

static esp_err_t rx8130ce_initialize_all_registers(rx8130ce_handle_t handle,
        bool charge_enable)
{
    /* Known-safe epoch: 2000-01-01 00:00:00, Saturday. */
    static const uint8_t calendar[7] = {
        0x00, 0x00, 0x00, 1U << 6, 0x01, 0x01, 0x00,
    };
    static const uint8_t alarm_timer_extension[6] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
    };
    static const uint8_t ram[4] = {0};
    const uint8_t control0_stopped = RX8130CE_CONTROL0_STOP;
    const uint8_t control0_running = 0;
    const uint8_t flags = 0;
    const uint8_t control1 = RX8130CE_CONTROL1_INIEN |
                             (uint8_t)handle->charge_cutoff |
                             (charge_enable ? RX8130CE_CONTROL1_CHGEN : 0);
    const uint8_t digital_offset = 0;
    /* Appman Table 42: VBLFE gates the VBLF detection for a primary cell;
     * bits 7-1 of 31h are manufacturer test bits kept at 0 (Table 12). */
    const uint8_t extension1 = handle->backup_voltage_low_detect
                               ? RX8130CE_EXTENSION1_VBLFE : 0;

    esp_err_t ret = ESP_OK;
    esp_err_t restore_error;
    ESP_GOTO_ON_ERROR(rx8130ce_write(handle, RX8130CE_REG_CONTROL0,
                                     &control0_stopped, 1), restore_stop, TAG,
                      "initial counter stop failed");
    ESP_GOTO_ON_ERROR(rx8130ce_write(handle, RX8130CE_REG_SECONDS,
                                     calendar, sizeof(calendar)), restore_stop,
                      TAG, "initial calendar write failed");
    ESP_GOTO_ON_ERROR(rx8130ce_write(handle, RX8130CE_REG_ALARM_MINUTE,
                                     alarm_timer_extension,
                                     sizeof(alarm_timer_extension)),
                      restore_stop, TAG,
                      "alarm/timer initialization failed");
    ESP_GOTO_ON_ERROR(rx8130ce_write(handle, RX8130CE_REG_CONTROL1,
                                     &control1, 1), restore_stop, TAG,
                      "backup control initialization failed");
    ESP_GOTO_ON_ERROR(rx8130ce_write(handle, RX8130CE_REG_RAM,
                                     ram, sizeof(ram)), restore_stop, TAG,
                      "RAM initialization failed");
    ESP_GOTO_ON_ERROR(rx8130ce_write(handle, RX8130CE_REG_DIGITAL_OFFSET,
                                     &digital_offset, 1), restore_stop, TAG,
                      "digital offset initialization failed");
    ESP_GOTO_ON_ERROR(rx8130ce_write(handle, RX8130CE_REG_EXTENSION1,
                                     &extension1, 1), restore_stop, TAG,
                      "backup flag detection initialization failed");
    /* Restart the counter before clearing VLF: while VLF stays set, a
     * failure makes the next create() retry this whole sequence instead
     * of succeeding with a halted clock. */
    ESP_GOTO_ON_ERROR(rx8130ce_write(handle, RX8130CE_REG_CONTROL0,
                                     &control0_running, 1), restore_stop,
                      TAG, "initial counter start failed");
    /* Written last, so VLF clears only after the initialization fully
     * succeeded and the time counter runs again. */
    return rx8130ce_write(handle, RX8130CE_REG_FLAGS, &flags, 1);

restore_stop:
    /* Do not leave the time counter stopped on a failed initialization;
     * preserve the programming error if the restart fails too. */
    restore_error = rx8130ce_write(handle, RX8130CE_REG_CONTROL0,
                                   &control0_running, 1);
    return ret != ESP_OK ? ret : restore_error;
}

esp_err_t rx8130ce_create(i2c_master_bus_handle_t bus,
                          const rx8130ce_config_t *config,
                          rx8130ce_handle_t *ret_handle)
{
    ESP_RETURN_ON_FALSE(bus != NULL && ret_handle != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid create argument");
    *ret_handle = NULL;

    const rx8130ce_config_t default_config = RX8130CE_CONFIG_DEFAULT();
    if (config == NULL) {
        config = &default_config;
    }
    ESP_RETURN_ON_FALSE(config->device_address <= 0x7F &&
                        config->scl_speed_hz > 0 &&
                        config->scl_speed_hz <= RX8130CE_I2C_CLOCK_HZ,
                        ESP_ERR_INVALID_ARG, TAG,
                        "invalid I2C configuration");
    /* Keeps the cutoff-less 11b encoding out of BFVSEL. */
    ESP_RETURN_ON_FALSE(config->backup_charge_cutoff <=
                        RX8130CE_CHARGE_CUTOFF_2_92V,
                        ESP_ERR_INVALID_ARG, TAG,
                        "invalid backup charge cutoff");

    rx8130ce_handle_t handle = calloc(1, sizeof(*handle));
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_NO_MEM, TAG,
                        "device allocation failed");
    handle->lock = xSemaphoreCreateMutex();
    if (handle->lock == NULL) {
        free(handle);
        return ESP_ERR_NO_MEM;
    }
    handle->charge_cutoff = config->backup_charge_cutoff;
    handle->backup_voltage_low_detect = config->backup_voltage_low_detect;
    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->device_address,
        .scl_speed_hz = config->scl_speed_hz,
    };
    esp_err_t error = i2c_master_bus_add_device(bus, &device_config,
                      &handle->i2c_device);
    if (error == ESP_OK) {
        /* Covers the specified t_int after return from backup operation.
         * pdMS_TO_TICKS rounds down, so one extra tick guarantees the
         * wait is never shorter than the specification. */
        vTaskDelay(pdMS_TO_TICKS(RX8130CE_BACKUP_RECOVERY_MS) + 1);
        uint8_t flags = 0;
        error = rx8130ce_read(handle, RX8130CE_REG_FLAGS, &flags, 1);
        if (error == ESP_OK && (flags & RX8130CE_FLAG_VLF) != 0) {
            /* Appman 10.2: with VLF=1, initialize only after waiting out
             * the oscillator start time t_str (1.0 s max, appman 8). */
            vTaskDelay(pdMS_TO_TICKS(RX8130CE_OSCILLATOR_START_MS));
            error = rx8130ce_initialize_all_registers(handle,
                    config->backup_charge_enable);
        } else if (error == ESP_OK) {
            /* Enable switchover; charge only rechargeable backup sources. */
            error = rx8130ce_configure_backup_supply(handle,
                    config->backup_charge_enable);
        }
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

esp_err_t rx8130ce_delete(rx8130ce_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "handle is NULL");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = i2c_master_bus_rm_device(handle->i2c_device);
    if (error != ESP_OK) {
        unlock_device(handle);
        return error;
    }
    handle->i2c_device = NULL;
    unlock_device(handle);
    vSemaphoreDelete(handle->lock);
    handle->lock = NULL;
    free(handle);
    return ESP_OK;
}

/* The caller must already hold the device lock. */
static esp_err_t get_status_locked(rx8130ce_handle_t handle,
                                   rx8130ce_status_t *status)
{
    uint8_t flags = 0;
    ESP_RETURN_ON_ERROR(rx8130ce_read(handle, RX8130CE_REG_FLAGS, &flags, 1),
                        TAG, "flag read failed");
    decode_status(flags, status);
    return ESP_OK;
}

esp_err_t rx8130ce_get_status(rx8130ce_handle_t handle,
                              rx8130ce_status_t *status)
{
    ESP_RETURN_ON_FALSE(status != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "status is NULL");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = get_status_locked(handle, status);
    unlock_device(handle);
    return error;
}

esp_err_t rx8130ce_check_power(rx8130ce_handle_t handle,
                               rx8130ce_power_check_t *out_check)
{
    ESP_RETURN_ON_FALSE(out_check != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "check result is NULL");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t flags = 0;
    const esp_err_t error = rx8130ce_read(handle, RX8130CE_REG_FLAGS, &flags,
                                          1);
    if (error == ESP_OK) {
        /* Appman 14.5.1: VLF=1 means the register contents are invalid and
         * all registers should be initialized before use. */
        *out_check = (rx8130ce_power_check_t) {
            .voltage_low = (flags & RX8130CE_FLAG_VLF) != 0,
            .reset_detected = (flags & RX8130CE_FLAG_RSF) != 0,
            .init_recommended = (flags & RX8130CE_FLAG_VLF) != 0,
        };
    }
    unlock_device(handle);
    return error;
}

esp_err_t rx8130ce_set_backup_charge(rx8130ce_handle_t handle, bool enable)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid device handle");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = rx8130ce_configure_backup_supply(handle, enable);
    unlock_device(handle);
    return error;
}

esp_err_t rx8130ce_get_backup_charge(rx8130ce_handle_t handle,
                                     bool *out_enabled)
{
    ESP_RETURN_ON_FALSE(handle != NULL && out_enabled != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t control1 = 0;
    const esp_err_t error = rx8130ce_read(handle, RX8130CE_REG_CONTROL1,
                                          &control1, 1);
    if (error == ESP_OK) {
        *out_enabled = (control1 & RX8130CE_CONTROL1_CHGEN) != 0;
    }
    unlock_device(handle);
    return error;
}

/* The caller must already hold the device lock. */
static esp_err_t get_time_locked(rx8130ce_handle_t handle,
                                 rx8130ce_time_t *time,
                                 rx8130ce_status_t *status)
{
    uint8_t data[7] = {0};
    ESP_RETURN_ON_ERROR(rx8130ce_read(handle, RX8130CE_REG_SECONDS, data,
                                      sizeof(data)), TAG, "calendar read failed");
    ESP_RETURN_ON_ERROR(decode_time(data, time), TAG,
                        "calendar decode failed");
    return status != NULL ? get_status_locked(handle, status) : ESP_OK;
}

esp_err_t rx8130ce_get_time(rx8130ce_handle_t handle,
                            rx8130ce_time_t *time,
                            rx8130ce_status_t *status)
{
    ESP_RETURN_ON_FALSE(time != NULL, ESP_ERR_INVALID_ARG, TAG, "time is NULL");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = get_time_locked(handle, time, status);
    unlock_device(handle);
    return error;
}

/* The caller must already hold the device lock. */
static esp_err_t set_time_locked(rx8130ce_handle_t handle,
                                 const rx8130ce_time_t *time)
{
    const uint8_t data[7] = {
        binary_to_bcd(time->second),
        binary_to_bcd(time->minute),
        binary_to_bcd(time->hour),
        (uint8_t)(1U << time->weekday),
        binary_to_bcd(time->day),
        binary_to_bcd(time->month),
        binary_to_bcd((uint8_t)(time->year - 2000)),
    };

    uint8_t control0 = 0;
    ESP_RETURN_ON_ERROR(rx8130ce_read(handle, RX8130CE_REG_CONTROL0,
                                      &control0, 1), TAG,
                        "control register read failed");
    const uint8_t stopped_control0 = control0 | RX8130CE_CONTROL0_STOP;
    ESP_RETURN_ON_ERROR(rx8130ce_write(handle, RX8130CE_REG_CONTROL0,
                                       &stopped_control0, 1), TAG,
                        "time counter stop failed");

    esp_err_t error = rx8130ce_write(handle, RX8130CE_REG_SECONDS, data,
                                     sizeof(data));
    if (error == ESP_OK) {
        uint8_t flags = 0;
        error = rx8130ce_read(handle, RX8130CE_REG_FLAGS, &flags, 1);
        if (error == ESP_OK) {
            flags &= ~RX8130CE_FLAG_VLF;
            error = rx8130ce_write(handle, RX8130CE_REG_FLAGS, &flags, 1);
        }
    }

    /* Always clear STOP when restoring: a leftover STOP bit would keep the
     * clock halted while rx8130ce_set_time() reports success. */
    const uint8_t restart_control0 = control0 & ~RX8130CE_CONTROL0_STOP;
    const esp_err_t restart_error = rx8130ce_write(handle,
                                    RX8130CE_REG_CONTROL0,
                                    &restart_control0, 1);
    return error != ESP_OK ? error : restart_error;
}

esp_err_t rx8130ce_set_time(rx8130ce_handle_t handle,
                            const rx8130ce_time_t *time)
{
    ESP_RETURN_ON_FALSE(rx8130ce_time_is_valid(time), ESP_ERR_INVALID_ARG,
                        TAG, "invalid calendar time");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = set_time_locked(handle, time);
    unlock_device(handle);
    return error;
}

/* The caller must already hold the device lock. */
static esp_err_t set_alarm_locked(rx8130ce_handle_t handle,
                                  const rx8130ce_alarm_t *alarm)
{
    uint8_t registers[3];
    bool use_day_alarm = false;
    rx8130ce_alarm_encode(alarm, registers, &use_day_alarm);

    uint8_t control0 = 0;
    ESP_RETURN_ON_ERROR(rx8130ce_read(handle, RX8130CE_REG_CONTROL0,
                                      &control0, 1), TAG,
                        "control register read failed");
    /* Appman 14.3.1 recommends holding AIE cleared while settings change. */
    const uint8_t disabled_control0 = control0 & ~RX8130CE_CONTROL0_AIE;
    ESP_RETURN_ON_ERROR(rx8130ce_write(handle, RX8130CE_REG_CONTROL0,
                                       &disabled_control0, 1), TAG,
                        "alarm interrupt disable failed");

    esp_err_t ret = ESP_OK;
    esp_err_t restore_error;
    ESP_GOTO_ON_ERROR(rx8130ce_write(handle, RX8130CE_REG_ALARM_MINUTE,
                                     registers, sizeof(registers)), restore_aie,
                      TAG, "alarm register write failed");

    uint8_t extension = 0;
    ESP_GOTO_ON_ERROR(rx8130ce_read(handle, RX8130CE_REG_EXTENSION,
                                    &extension, 1), restore_aie, TAG,
                      "extension register read failed");
    if (use_day_alarm) {
        extension |= RX8130CE_EXTENSION_WADA;
    } else {
        extension &= ~RX8130CE_EXTENSION_WADA;
    }
    ESP_GOTO_ON_ERROR(rx8130ce_write(handle, RX8130CE_REG_EXTENSION,
                                     &extension, 1), restore_aie, TAG,
                      "alarm target select failed");

    /* Drop any alarm event latched while the registers changed. */
    uint8_t flags = 0;
    ESP_GOTO_ON_ERROR(rx8130ce_read(handle, RX8130CE_REG_FLAGS, &flags, 1),
                      restore_aie, TAG, "flag read failed");
    flags &= ~RX8130CE_FLAG_AF;
    ESP_GOTO_ON_ERROR(rx8130ce_write(handle, RX8130CE_REG_FLAGS, &flags, 1),
                      restore_aie, TAG, "alarm flag clear failed");

restore_aie:
    /* Once AIE has been gated, restore its previous state on every exit path.
     * Preserve the programming error if both it and restoration fail. */
    restore_error = rx8130ce_write(handle, RX8130CE_REG_CONTROL0,
                                   &control0, 1);
    return ret != ESP_OK ? ret : restore_error;
}

esp_err_t rx8130ce_set_alarm(rx8130ce_handle_t handle,
                             const rx8130ce_alarm_t *alarm)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid device handle");
    ESP_RETURN_ON_FALSE(rx8130ce_alarm_is_valid(alarm), ESP_ERR_INVALID_ARG,
                        TAG, "invalid alarm");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = set_alarm_locked(handle, alarm);
    unlock_device(handle);
    return error;
}

/* The caller must already hold the device lock. */
static esp_err_t get_alarm_locked(rx8130ce_handle_t handle,
                                  rx8130ce_alarm_t *out_alarm)
{
    uint8_t data[3] = {0};
    ESP_RETURN_ON_ERROR(rx8130ce_read(handle, RX8130CE_REG_ALARM_MINUTE,
                                      data, sizeof(data)), TAG,
                        "alarm read failed");
    uint8_t extension = 0;
    ESP_RETURN_ON_ERROR(rx8130ce_read(handle, RX8130CE_REG_EXTENSION,
                                      &extension, 1), TAG,
                        "extension register read failed");
    return decode_alarm(data, (extension & RX8130CE_EXTENSION_WADA) != 0,
                        out_alarm);
}

esp_err_t rx8130ce_get_alarm(rx8130ce_handle_t handle,
                             rx8130ce_alarm_t *out_alarm)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid device handle");
    ESP_RETURN_ON_FALSE(out_alarm != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "alarm is NULL");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = get_alarm_locked(handle, out_alarm);
    unlock_device(handle);
    return error;
}

/* The caller must already hold the device lock. */
static esp_err_t alarm_irq_enable_locked(rx8130ce_handle_t handle, bool enable)
{
    uint8_t control0 = 0;
    ESP_RETURN_ON_ERROR(rx8130ce_read(handle, RX8130CE_REG_CONTROL0,
                                      &control0, 1), TAG,
                        "control register read failed");
    if (enable) {
        control0 |= RX8130CE_CONTROL0_AIE;
    } else {
        control0 &= ~RX8130CE_CONTROL0_AIE;
    }
    return rx8130ce_write(handle, RX8130CE_REG_CONTROL0, &control0, 1);
}

esp_err_t rx8130ce_alarm_irq_enable(rx8130ce_handle_t handle, bool enable)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid device handle");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = alarm_irq_enable_locked(handle, enable);
    unlock_device(handle);
    return error;
}

/* The caller must already hold the device lock. */
static esp_err_t clear_alarm_locked(rx8130ce_handle_t handle)
{
    /* Every compare field ignored: the hardware then matches once per minute
     * (appman 14.3.1 note *3), so AIE is held cleared to keep /IRQ released
     * and AF is dropped afterwards. */
    const uint8_t registers[3] = {
        RX8130CE_ALARM_AE, RX8130CE_ALARM_AE, RX8130CE_ALARM_AE,
    };
    uint8_t control0 = 0;
    ESP_RETURN_ON_ERROR(rx8130ce_read(handle, RX8130CE_REG_CONTROL0,
                                      &control0, 1), TAG,
                        "control register read failed");
    const uint8_t disabled_control0 = control0 & ~RX8130CE_CONTROL0_AIE;
    ESP_RETURN_ON_ERROR(rx8130ce_write(handle, RX8130CE_REG_CONTROL0,
                                       &disabled_control0, 1), TAG,
                        "alarm interrupt disable failed");
    ESP_RETURN_ON_ERROR(rx8130ce_write(handle, RX8130CE_REG_ALARM_MINUTE,
                                       registers, sizeof(registers)), TAG,
                        "alarm register write failed");
    uint8_t flags = 0;
    ESP_RETURN_ON_ERROR(rx8130ce_read(handle, RX8130CE_REG_FLAGS, &flags, 1),
                        TAG, "flag read failed");
    flags &= ~RX8130CE_FLAG_AF;
    return rx8130ce_write(handle, RX8130CE_REG_FLAGS, &flags, 1);
}

esp_err_t rx8130ce_clear_alarm(rx8130ce_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid device handle");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = clear_alarm_locked(handle);
    unlock_device(handle);
    return error;
}

esp_err_t rx8130ce_get_and_clear_alarm_flag(rx8130ce_handle_t handle,
        bool *alarm_flag)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid device handle");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t flags = 0;
    esp_err_t error = rx8130ce_read(handle, RX8130CE_REG_FLAGS, &flags, 1);
    if (error == ESP_OK) {
        if (alarm_flag != NULL) {
            *alarm_flag = (flags & RX8130CE_FLAG_AF) != 0;
        }
        /* Appman 14.3.1: writing 0 clears AF, writing 1 is ignored, so this
         * read-modify-write cannot disturb the other flags. */
        flags &= ~RX8130CE_FLAG_AF;
        error = rx8130ce_write(handle, RX8130CE_REG_FLAGS, &flags, 1);
    }
    unlock_device(handle);
    return error;
}

/* The caller must already hold the device lock. */
static esp_err_t set_timer_locked(rx8130ce_handle_t handle,
                                  const rx8130ce_timer_t *timer)
{
    uint8_t registers[2];
    uint8_t tsel_bits = 0;
    rx8130ce_timer_encode(timer, registers, &tsel_bits);

    uint8_t extension = 0;
    ESP_RETURN_ON_ERROR(rx8130ce_read(handle, RX8130CE_REG_EXTENSION,
                                      &extension, 1), TAG,
                        "extension register read failed");
    /* Appman 14.2.2 requires TE=0 before the preset value is written. */
    const uint8_t stopped_extension = (extension & ~RX8130CE_EXTENSION_TE &
                                       ~RX8130CE_EXTENSION_TSEL_MASK) |
                                      tsel_bits;
    ESP_RETURN_ON_ERROR(rx8130ce_write(handle, RX8130CE_REG_EXTENSION,
                                       &stopped_extension, 1), TAG,
                        "timer stop failed");
    ESP_RETURN_ON_ERROR(rx8130ce_write(handle, RX8130CE_REG_TIMER_COUNTER0,
                                       registers, sizeof(registers)), TAG,
                        "timer counter write failed");

    /* Drop any timer event latched while the registers changed. */
    uint8_t flags = 0;
    ESP_RETURN_ON_ERROR(rx8130ce_read(handle, RX8130CE_REG_FLAGS, &flags, 1),
                        TAG, "flag read failed");
    flags &= ~RX8130CE_FLAG_TF;
    ESP_RETURN_ON_ERROR(rx8130ce_write(handle, RX8130CE_REG_FLAGS, &flags, 1),
                        TAG, "timer flag clear failed");

    /* The TE 0->1 transition starts the countdown from the preset value. */
    if (timer->enable) {
        const uint8_t running_extension = stopped_extension |
                                          RX8130CE_EXTENSION_TE;
        ESP_RETURN_ON_ERROR(rx8130ce_write(handle, RX8130CE_REG_EXTENSION,
                                           &running_extension, 1), TAG,
                            "timer start failed");
    }
    return ESP_OK;
}

esp_err_t rx8130ce_set_timer(rx8130ce_handle_t handle,
                             const rx8130ce_timer_t *timer)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid device handle");
    ESP_RETURN_ON_FALSE(rx8130ce_timer_is_valid(timer), ESP_ERR_INVALID_ARG,
                        TAG, "invalid timer");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = set_timer_locked(handle, timer);
    unlock_device(handle);
    return error;
}

/* The caller must already hold the device lock. */
static esp_err_t get_timer_locked(rx8130ce_handle_t handle,
                                  rx8130ce_timer_t *out_timer)
{
    uint8_t data[2] = {0};
    ESP_RETURN_ON_ERROR(rx8130ce_read(handle, RX8130CE_REG_TIMER_COUNTER0,
                                      data, sizeof(data)), TAG,
                        "timer counter read failed");
    uint8_t extension = 0;
    ESP_RETURN_ON_ERROR(rx8130ce_read(handle, RX8130CE_REG_EXTENSION,
                                      &extension, 1), TAG,
                        "extension register read failed");
    const uint8_t tsel_bits = extension & RX8130CE_EXTENSION_TSEL_MASK;
    ESP_RETURN_ON_FALSE(tsel_bits <= RX8130CE_TIMER_SOURCE_1_3600HZ,
                        ESP_ERR_INVALID_RESPONSE, TAG,
                        "invalid timer source data");
    *out_timer = (rx8130ce_timer_t) {
        .enable = (extension & RX8130CE_EXTENSION_TE) != 0,
        .source_clock = (rx8130ce_timer_source_t)tsel_bits,
        .count = (uint16_t)(data[0] | ((uint16_t)data[1] << 8)),
    };
    return ESP_OK;
}

esp_err_t rx8130ce_get_timer(rx8130ce_handle_t handle,
                             rx8130ce_timer_t *out_timer)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid device handle");
    ESP_RETURN_ON_FALSE(out_timer != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "timer is NULL");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = get_timer_locked(handle, out_timer);
    unlock_device(handle);
    return error;
}

/* The caller must already hold the device lock. */
static esp_err_t timer_irq_enable_locked(rx8130ce_handle_t handle, bool enable)
{
    uint8_t control0 = 0;
    ESP_RETURN_ON_ERROR(rx8130ce_read(handle, RX8130CE_REG_CONTROL0,
                                      &control0, 1), TAG,
                        "control register read failed");
    if (enable) {
        control0 |= RX8130CE_CONTROL0_TIE;
    } else {
        control0 &= ~RX8130CE_CONTROL0_TIE;
    }
    return rx8130ce_write(handle, RX8130CE_REG_CONTROL0, &control0, 1);
}

esp_err_t rx8130ce_timer_irq_enable(rx8130ce_handle_t handle, bool enable)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid device handle");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = timer_irq_enable_locked(handle, enable);
    unlock_device(handle);
    return error;
}

/* The caller must already hold the device lock. */
static esp_err_t update_irq_enable_locked(rx8130ce_handle_t handle, bool enable)
{
    uint8_t control0 = 0;
    ESP_RETURN_ON_ERROR(rx8130ce_read(handle, RX8130CE_REG_CONTROL0,
                                      &control0, 1), TAG,
                        "control register read failed");
    if (enable) {
        control0 |= RX8130CE_CONTROL0_UIE;
    } else {
        control0 &= ~RX8130CE_CONTROL0_UIE;
    }
    return rx8130ce_write(handle, RX8130CE_REG_CONTROL0, &control0, 1);
}

esp_err_t rx8130ce_update_irq_enable(rx8130ce_handle_t handle, bool enable)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid device handle");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = update_irq_enable_locked(handle, enable);
    unlock_device(handle);
    return error;
}

/* The caller must already hold the device lock. */
static esp_err_t get_and_clear_interrupts_locked(rx8130ce_handle_t handle,
        uint8_t *flags)
{
    uint8_t value = 0;
    ESP_RETURN_ON_ERROR(rx8130ce_read(handle, RX8130CE_REG_FLAGS, &value, 1),
                        TAG, "flag read failed");
    if (flags != NULL) {
        *flags = value;
    }
    value &= ~RX8130CE_INTERRUPT_FLAGS;
    return rx8130ce_write(handle, RX8130CE_REG_FLAGS, &value, 1);
}

esp_err_t rx8130ce_get_and_clear_interrupts(rx8130ce_handle_t handle,
        uint8_t *flags)
{
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = get_and_clear_interrupts_locked(handle, flags);
    unlock_device(handle);
    return error;
}

#define RX8130CE_EXTENSION_FSEL_MASK      0xC0
#define RX8130CE_EXTENSION_USEL           (1U << 5)
#define RX8130CE_CONTROL0_TSTP            (1U << 2)
#define RX8130CE_CONTROL0_TBKON           (1U << 1)
#define RX8130CE_CONTROL0_TBKE            (1U << 0)
#define RX8130CE_DIGITAL_OFFSET_DTE       (1U << 7)
#define RX8130CE_DIGITAL_OFFSET_L_MASK    0x7F
#define RX8130CE_REG_USER_REGISTERS_FIRST 0x10
#define RX8130CE_REG_USER_REGISTERS_LAST  0x23

static bool rx8130ce_reg_is_user_accessible(uint8_t reg)
{
    /* appman 13.2.1 note *6: only the documented user registers (10h-23h,
     * 30h and 31h) may be accessed; the manufacturer registers are
     * excluded. */
    return (reg >= RX8130CE_REG_USER_REGISTERS_FIRST && reg <= RX8130CE_REG_USER_REGISTERS_LAST) ||
           reg == RX8130CE_REG_DIGITAL_OFFSET ||
           reg == RX8130CE_REG_EXTENSION1;
}

static bool rx8130ce_reg_window_is_user_accessible(uint8_t start, size_t length)
{
    /* The whole transfer must stay inside the user registers: crossing a
     * gap would read or write a reserved register, because the address
     * counter wraps only at 1Fh, 2Fh and 3Fh (appman 14.12.4). */
    for (size_t i = 0; i < length; ++i) {
        if (!rx8130ce_reg_is_user_accessible((uint8_t)(start + i))) {
            return false;
        }
    }
    return true;
}

/* The caller must already hold the device lock. */
static esp_err_t update_bits_locked(rx8130ce_handle_t handle, uint8_t reg,
                                    uint8_t mask, uint8_t value)
{
    uint8_t image = 0;
    ESP_RETURN_ON_ERROR(rx8130ce_read(handle, reg, &image, 1), TAG,
                        "register read failed");
    image = (uint8_t)((image & ~mask) | (value & mask));
    return rx8130ce_write(handle, reg, &image, 1);
}

esp_err_t rx8130ce_set_fout(rx8130ce_handle_t handle, rx8130ce_fout_t frequency)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid device handle");
    ESP_RETURN_ON_FALSE(frequency >= RX8130CE_FOUT_32768HZ && frequency <= RX8130CE_FOUT_OFF,
                        ESP_ERR_INVALID_ARG, TAG, "invalid FOUT selection");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = update_bits_locked(handle, RX8130CE_REG_EXTENSION,
                            RX8130CE_EXTENSION_FSEL_MASK,
                            (uint8_t)frequency << 6);
    unlock_device(handle);
    return error;
}

esp_err_t rx8130ce_get_fout(rx8130ce_handle_t handle, rx8130ce_fout_t *out_frequency)
{
    ESP_RETURN_ON_FALSE(handle != NULL && out_frequency != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid argument");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t extension = 0;
    const esp_err_t error = rx8130ce_read(handle, RX8130CE_REG_EXTENSION, &extension, 1);
    unlock_device(handle);
    ESP_RETURN_ON_ERROR(error, TAG, "extension register read failed");
    *out_frequency = (rx8130ce_fout_t)((extension & RX8130CE_EXTENSION_FSEL_MASK) >> 6);
    return ESP_OK;
}

esp_err_t rx8130ce_set_update_irq_mode(rx8130ce_handle_t handle, rx8130ce_update_irq_mode_t mode)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid device handle");
    ESP_RETURN_ON_FALSE(mode >= RX8130CE_UPDATE_IRQ_PER_SECOND && mode <= RX8130CE_UPDATE_IRQ_PER_MINUTE,
                        ESP_ERR_INVALID_ARG, TAG, "invalid update interrupt mode");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = update_bits_locked(handle, RX8130CE_REG_EXTENSION,
                            RX8130CE_EXTENSION_USEL,
                            (uint8_t)mode << 5);
    unlock_device(handle);
    return error;
}

esp_err_t rx8130ce_get_update_irq_mode(rx8130ce_handle_t handle, rx8130ce_update_irq_mode_t *out_mode)
{
    ESP_RETURN_ON_FALSE(handle != NULL && out_mode != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid argument");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t extension = 0;
    const esp_err_t error = rx8130ce_read(handle, RX8130CE_REG_EXTENSION, &extension, 1);
    unlock_device(handle);
    ESP_RETURN_ON_ERROR(error, TAG, "extension register read failed");
    *out_mode = (extension & RX8130CE_EXTENSION_USEL) != 0 ?
                RX8130CE_UPDATE_IRQ_PER_MINUTE : RX8130CE_UPDATE_IRQ_PER_SECOND;
    return ESP_OK;
}

esp_err_t rx8130ce_set_timer_count_mode(rx8130ce_handle_t handle, rx8130ce_timer_count_mode_t mode)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid device handle");
    ESP_RETURN_ON_FALSE(mode >= RX8130CE_TIMER_COUNT_ALWAYS && mode <= RX8130CE_TIMER_COUNT_BACKUP,
                        ESP_ERR_INVALID_ARG, TAG, "invalid timer count mode");
    /* appman 14.2.2 6): ALWAYS = TBKE 0 / TBKON don't care (driven 0),
     * MAIN = TBKE 1 / TBKON 0, BACKUP = TBKE 1 / TBKON 1. */
    const uint8_t bits = mode == RX8130CE_TIMER_COUNT_ALWAYS ? 0 :
                         (RX8130CE_CONTROL0_TBKE |
                          (mode == RX8130CE_TIMER_COUNT_BACKUP ? RX8130CE_CONTROL0_TBKON : 0));
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = update_bits_locked(handle, RX8130CE_REG_CONTROL0,
                            RX8130CE_CONTROL0_TBKE | RX8130CE_CONTROL0_TBKON,
                            bits);
    unlock_device(handle);
    return error;
}

esp_err_t rx8130ce_get_timer_count_mode(rx8130ce_handle_t handle, rx8130ce_timer_count_mode_t *out_mode)
{
    ESP_RETURN_ON_FALSE(handle != NULL && out_mode != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid argument");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t control0 = 0;
    const esp_err_t error = rx8130ce_read(handle, RX8130CE_REG_CONTROL0, &control0, 1);
    unlock_device(handle);
    ESP_RETURN_ON_ERROR(error, TAG, "control register read failed");
    if ((control0 & RX8130CE_CONTROL0_TBKE) == 0) {
        *out_mode = RX8130CE_TIMER_COUNT_ALWAYS;
    } else {
        *out_mode = (control0 & RX8130CE_CONTROL0_TBKON) != 0 ?
                    RX8130CE_TIMER_COUNT_BACKUP : RX8130CE_TIMER_COUNT_MAIN;
    }
    return ESP_OK;
}

esp_err_t rx8130ce_timer_pause(rx8130ce_handle_t handle, bool pause)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid device handle");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = update_bits_locked(handle, RX8130CE_REG_CONTROL0,
                            RX8130CE_CONTROL0_TSTP,
                            pause ? RX8130CE_CONTROL0_TSTP : 0);
    unlock_device(handle);
    return error;
}

esp_err_t rx8130ce_write_user_ram(rx8130ce_handle_t handle, uint8_t offset,
                                  const uint8_t *data, size_t length)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid device handle");
    ESP_RETURN_ON_FALSE(data != NULL && length > 0, ESP_ERR_INVALID_ARG, TAG,
                        "invalid write buffer");
    ESP_RETURN_ON_FALSE(offset < RX8130CE_USER_RAM_SIZE &&
                        length <= RX8130CE_USER_RAM_SIZE - offset,
                        ESP_ERR_INVALID_SIZE, TAG, "user RAM range exceeded");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = rx8130ce_write(handle, RX8130CE_REG_RAM + offset, data, length);
    unlock_device(handle);
    return error;
}

esp_err_t rx8130ce_read_user_ram(rx8130ce_handle_t handle, uint8_t offset,
                                 uint8_t *data, size_t length)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid device handle");
    ESP_RETURN_ON_FALSE(data != NULL && length > 0, ESP_ERR_INVALID_ARG, TAG,
                        "invalid read buffer");
    ESP_RETURN_ON_FALSE(offset < RX8130CE_USER_RAM_SIZE &&
                        length <= RX8130CE_USER_RAM_SIZE - offset,
                        ESP_ERR_INVALID_SIZE, TAG, "user RAM range exceeded");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = rx8130ce_read(handle, RX8130CE_REG_RAM + offset, data, length);
    unlock_device(handle);
    return error;
}

esp_err_t rx8130ce_digital_offset_encode(bool enable, int8_t offset_steps,
        uint8_t *out_register)
{
    ESP_RETURN_ON_FALSE(out_register != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid output pointer");
    ESP_RETURN_ON_FALSE(offset_steps >= RX8130CE_DIGITAL_OFFSET_MIN_STEPS &&
                        offset_steps <= RX8130CE_DIGITAL_OFFSET_MAX_STEPS,
                        ESP_ERR_INVALID_ARG, TAG, "offset steps out of range");
    /* appman 14.10.1: positive offsets are L = steps; negative offsets are
     * L = 128 + steps (two regions of the 7-bit L field). */
    const uint8_t field = offset_steps >= 0 ? (uint8_t)offset_steps
                          : (uint8_t)(128 + offset_steps);
    *out_register = (enable ? RX8130CE_DIGITAL_OFFSET_DTE : 0) |
                    (field & RX8130CE_DIGITAL_OFFSET_L_MASK);
    return ESP_OK;
}

void rx8130ce_digital_offset_decode(uint8_t register_value, bool *out_enabled,
                                    int8_t *out_offset_steps)
{
    const uint8_t field = register_value & RX8130CE_DIGITAL_OFFSET_L_MASK;
    if (out_enabled != NULL) {
        *out_enabled = (register_value & RX8130CE_DIGITAL_OFFSET_DTE) != 0;
    }
    if (out_offset_steps != NULL) {
        *out_offset_steps = field <= (uint8_t)RX8130CE_DIGITAL_OFFSET_MAX_STEPS ?
                            (int8_t)field : (int8_t)(field - 128);
    }
}

esp_err_t rx8130ce_set_digital_offset(rx8130ce_handle_t handle, bool enable,
                                      int8_t offset_steps)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid device handle");
    uint8_t image = 0;
    ESP_RETURN_ON_ERROR(rx8130ce_digital_offset_encode(enable, offset_steps, &image),
                        TAG, "invalid digital offset");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = rx8130ce_write(handle, RX8130CE_REG_DIGITAL_OFFSET, &image, 1);
    unlock_device(handle);
    return error;
}

esp_err_t rx8130ce_get_digital_offset(rx8130ce_handle_t handle, bool *out_enabled,
                                      int8_t *out_offset_steps)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid device handle");
    ESP_RETURN_ON_FALSE(out_enabled != NULL || out_offset_steps != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "no output requested");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t image = 0;
    const esp_err_t error = rx8130ce_read(handle, RX8130CE_REG_DIGITAL_OFFSET, &image, 1);
    unlock_device(handle);
    ESP_RETURN_ON_ERROR(error, TAG, "digital offset register read failed");
    rx8130ce_digital_offset_decode(image, out_enabled, out_offset_steps);
    return ESP_OK;
}

esp_err_t rx8130ce_read_registers(rx8130ce_handle_t handle, uint8_t start_register,
                                  uint8_t *data, size_t length)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid device handle");
    ESP_RETURN_ON_FALSE(data != NULL && length > 0, ESP_ERR_INVALID_ARG, TAG,
                        "invalid read buffer");
    ESP_RETURN_ON_FALSE(
        rx8130ce_reg_window_is_user_accessible(start_register, length),
        ESP_ERR_INVALID_ARG, TAG, "not a user register address");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = rx8130ce_read(handle, start_register, data, length);
    unlock_device(handle);
    return error;
}

esp_err_t rx8130ce_write_register(rx8130ce_handle_t handle, uint8_t reg,
                                  uint8_t value)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid device handle");
    ESP_RETURN_ON_FALSE(rx8130ce_reg_is_user_accessible(reg),
                        ESP_ERR_INVALID_ARG, TAG, "not a user register address");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = rx8130ce_write(handle, reg, &value, 1);
    unlock_device(handle);
    return error;
}
