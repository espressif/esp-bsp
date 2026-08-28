/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "tg28_sw.h"
#include "tg28_sw_priv.h"

#define TG28_SW_REG_COMMON_STATUS0       0x00
#define TG28_SW_REG_CHIP_ID              0x03
#define TG28_SW_REG_COMMON_CONFIG        0x10
#define TG28_SW_REG_BATFET_CONTROL       0x12
#define TG28_SW_REG_MIN_SYS_VOLTAGE      0x14
#define TG28_SW_REG_VINDPM               0x15
#define TG28_SW_REG_INPUT_CURRENT_LIMIT  0x16
#define TG28_SW_REG_MODE                 0x17
#define TG28_SW_REG_MODULE_ENABLE        0x18
#define TG28_SW_REG_WATCHDOG             0x19
#define TG28_SW_REG_LOW_BATTERY_WARNING  0x1A
#define TG28_SW_REG_GPIO1_CONFIG         0x1B
#define TG28_SW_REG_POWER_ON_SOURCE      0x20
#define TG28_SW_REG_POWER_OFF_SOURCE     0x21
#define TG28_SW_REG_PWROFF_EN            0x22
#define TG28_SW_REG_DCDC_PWROFF_EN       0x23
#define TG28_SW_REG_VOFF_THRESHOLD       0x24
#define TG28_SW_REG_SLEEP_WAKEUP         0x26
#define TG28_SW_REG_POWER_KEY_LEVELS     0x27
#define TG28_SW_REG_ADC_CHANNEL_ENABLE   0x30
#define TG28_SW_REG_VBAT_H               0x34
#define TG28_SW_REG_TS_H                 0x36
#define TG28_SW_REG_VBUS_H               0x38
#define TG28_SW_REG_VSYS_H               0x3A
#define TG28_SW_REG_TDIE_H               0x3C
#define TG28_SW_REG_INT_ENABLE0          0x40
#define TG28_SW_REG_INT_ENABLE1          0x41
#define TG28_SW_REG_INT_ENABLE2          0x42
#define TG28_SW_REG_INT_STATUS0          0x48
#define TG28_SW_REG_TS_CONFIG            0x50
#define TG28_SW_REG_TS_HYST_LOW          0x52
#define TG28_SW_REG_TS_HYST_HIGH         0x53
#define TG28_SW_REG_VLTF_CHARGE          0x54
#define TG28_SW_REG_VHTF_CHARGE          0x55
#define TG28_SW_REG_VLTF_WORK            0x56
#define TG28_SW_REG_VHTF_WORK            0x57
#define TG28_SW_REG_JEITA_ENABLE         0x58
#define TG28_SW_REG_JEITA_CV             0x59
#define TG28_SW_REG_JEITA_COOL           0x5A
#define TG28_SW_REG_JEITA_WARM           0x5B
#define TG28_SW_REG_TS_FIXED_H           0x5C
#define TG28_SW_REG_TS_FIXED_L           0x5D
#define TG28_SW_REG_PRECHARGE_CURRENT    0x61
#define TG28_SW_REG_CHARGE_CURRENT       0x62
#define TG28_SW_REG_TERMINATION_CURRENT  0x63
#define TG28_SW_REG_CHARGE_VOLTAGE       0x64
#define TG28_SW_REG_THERMAL_REGULATION   0x65
#define TG28_SW_REG_CHARGE_TIMERS        0x67
#define TG28_SW_REG_BATTERY_DETECT       0x68
#define TG28_SW_REG_CHARGE_LED           0x69
#define TG28_SW_REG_BACKUP_CHARGE        0x6A
#define TG28_SW_REG_DCDC_ENABLE          0x80
#define TG28_SW_REG_DCDC_FORCE_PWM       0x81
#define TG28_SW_REG_DCDC1_VOLTAGE        0x82
#define TG28_SW_REG_DCDC2_VOLTAGE        0x83
#define TG28_SW_REG_DCDC3_VOLTAGE        0x84
#define TG28_SW_REG_DCDC4_VOLTAGE        0x85
#define TG28_SW_REG_LDO_ENABLE0          0x90
#define TG28_SW_REG_LDO_ENABLE1          0x91
#define TG28_SW_REG_ALDO1_VOLTAGE        0x92
#define TG28_SW_REG_ALDO2_VOLTAGE        0x93
#define TG28_SW_REG_ALDO3_VOLTAGE        0x94
#define TG28_SW_REG_ALDO4_VOLTAGE        0x95
#define TG28_SW_REG_BLDO1_VOLTAGE        0x96
#define TG28_SW_REG_BLDO2_VOLTAGE        0x97
#define TG28_SW_REG_CPUSLDO_VOLTAGE      0x98
#define TG28_SW_REG_DLDO1_VOLTAGE        0x99
#define TG28_SW_REG_DLDO2_VOLTAGE        0x9A
#define TG28_SW_REG_BATTERY_MODEL        0xA1
#define TG28_SW_REG_FUEL_GAUGE_CONTROL   0xA2
#define TG28_SW_REG_SOC                  0xA4

#define TG28_SW_VBUS_PRESENT_MASK        (1U << 5)
#define TG28_SW_BATTERY_PRESENT_MASK     (1U << 3)
#define TG28_SW_CHARGE_STATE_MASK        0x07
#define TG28_SW_CHARGE_STATE_DONE        0x04
#define TG28_SW_CHARGER_ENABLE_MASK      (1U << 1)
#define TG28_SW_GAUGE_MCU_RESET_MASK     (1U << 2)
/* REG10 common configuration: soft PWROFF and software reset are
 * self-clearing command bits (datasheet 6.13.2.7). */
#define TG28_SW_SOFT_PWROFF_MASK         (1U << 0)
#define TG28_SW_SOFTWARE_RESET_MASK      (1U << 1)
#define TG28_SW_POWER_KEY_IRQ_MASK       0x0F
#define TG28_SW_TS_MODE_MASK             (1U << 4)
#define TG28_SW_TS_CURRENT_SOURCE_MASK   (3U << 2)
#define TG28_SW_TS_CURRENT_VALUE_MASK    0x03
#define TG28_SW_TS_CONFIG_MASK           0x1F
#define TG28_SW_ADC_VALUE_MASK           0x3F
#define TG28_SW_PRECHARGE_CURRENT_MASK   0x0F
#define TG28_SW_CHARGE_CURRENT_MASK      0x1F
#define TG28_SW_TERMINATION_CURRENT_MASK 0x0F
#define TG28_SW_TERMINATION_ENABLE_MASK  (1U << 4)
#define TG28_SW_INPUT_CURRENT_LIMIT_MASK 0x07
#define TG28_SW_CHARGE_VOLTAGE_MASK      0x07
/* REG15 bits 7:4 are read-only; VINDPM lives in bits 3:0 (6.13.2.11). */
#define TG28_SW_VINDPM_MASK              0x0F
#define TG28_SW_BROM_UPDATE_MARK_MASK    (1U << 4)
#define TG28_SW_BROM_WRITER_ENABLE_MASK  (1U << 0)
#define TG28_SW_GAUGE_CONTROL_WRITE_MASK ((1U << 5) | \
                                           TG28_SW_BROM_UPDATE_MARK_MASK | \
                                           TG28_SW_BROM_WRITER_ENABLE_MASK)
/* REG18 module enables (datasheet 6.13.2.14). */
#define TG28_SW_GAUGE_ENABLE_MASK        (1U << 3)
#define TG28_SW_BACKUP_CHARGE_ENABLE_MASK (1U << 2)
#define TG28_SW_WATCHDOG_ENABLE_MASK     (1U << 0)
#define TG28_SW_MODULE_ENABLE_WRITE_MASK  0x0F
/* REG19 watchdog control (datasheet 6.13.2.15). */
#define TG28_SW_WATCHDOG_ACTION_MASK     0x30
#define TG28_SW_WATCHDOG_CLEAR_MASK      (1U << 3)
#define TG28_SW_WATCHDOG_PERIOD_MASK     0x07
/* REG12 BATFET off-state keep (datasheet 6.13.2.8). */
#define TG28_SW_BATFET_OFF_ENABLE_MASK   (1U << 3)
/* REG14 minimum system voltage codes 0-7 = 3.2-3.9V (6.13.2.10). */
#define TG28_SW_MIN_SYS_VOLTAGE_MASK     0x07
#define TG28_SW_MIN_SYS_VOLTAGE_BASE_MV  3200
#define TG28_SW_MIN_SYS_VOLTAGE_STEP_MV  100
/* REG22 PWROFF_EN bits (datasheet 6.13.2.20). */
#define TG28_SW_PWROFF_DIE_OT2_MASK      (1U << 2)
#define TG28_SW_PWROFF_OFFLEVEL_MASK     (1U << 1)
#define TG28_SW_PWROFF_BUTTON_RESTART_MASK (1U << 0)
/* REG23 DCDC OVP/UVP power-off enables (datasheet 6.13.2.21). */
#define TG28_SW_DCDC_OVP_PWROFF_MASK     (1U << 5)
#define TG28_SW_DCDC_UVP_PWROFF_MASK     0x0F
/* REG24 battery voltage for PWROFF: 2.6-3.3V in 100mV steps (6.13.2.22). */
#define TG28_SW_VOFF_VOLTAGE_MASK        0x07
#define TG28_SW_VOFF_VOLTAGE_BASE_MV     2600
#define TG28_SW_VOFF_VOLTAGE_STEP_MV     100
/* REG26 sleep/wakeup control bits (datasheet 6.13.2.24). */
#define TG28_SW_WAKEUP_IRQ_MASK          (1U << 4)
#define TG28_SW_WAKEUP_PWROK_LOW_MASK    (1U << 3)
#define TG28_SW_WAKEUP_VOLTAGE_MASK      (1U << 2)
#define TG28_SW_WAKEUP_ENABLE_MASK       (1U << 1)
#define TG28_SW_SLEEP_ENABLE_MASK        (1U << 0)
/* REG27 IRQLEVEL/OFFLEVEL/ONLEVEL fields (datasheet 6.13.2.25). */
#define TG28_SW_IRQLEVEL_MASK            0x30
#define TG28_SW_OFFLEVEL_MASK            0x0C
#define TG28_SW_ONLEVEL_MASK             0x03
/* REG1B GPIO1 output field in bits 3:2 (datasheet 6.13.2.17). */
#define TG28_SW_GPIO1_OUTPUT_MASK        0x0C
/* REG65 thermal regulation threshold codes (6.13.2.64). */
#define TG28_SW_THERMAL_REGULATION_MASK  0x03
/* REG67 charger safety-timer fields (6.13.2.65). */
#define TG28_SW_TIMER_SLOW_DPM_MASK      (1U << 7)
#define TG28_SW_CHARGE_TIMER_ENABLE_MASK (1U << 6)
#define TG28_SW_CHARGE_TIMER_MASK        0x30
#define TG28_SW_PRECHARGE_TIMER_ENABLE_MASK (1U << 2)
#define TG28_SW_PRECHARGE_TIMER_MASK     0x03
/* REG68 battery detection enable (6.13.2.66). */
#define TG28_SW_BATTERY_DETECT_MASK      (1U << 0)
/* REG69 CHGLED fields (6.13.2.67). */
#define TG28_SW_CHARGE_LED_MANUAL_MASK   0x30
#define TG28_SW_CHARGE_LED_MODE_MASK     0x06
#define TG28_SW_CHARGE_LED_ENABLE_MASK   (1U << 0)
/* REG6A backup termination voltage codes 0-7 = 2.6-3.3V (6.13.2.68), the
 * same 2600mV base and 100mV step as REG24. */
#define TG28_SW_BACKUP_VOLTAGE_MASK      0x07
/* REG80 force-CCM and DVM ramp bits (6.13.2.69). */
#define TG28_SW_DCDC_FORCE_CCM_MASK      (1U << 6)
#define TG28_SW_DCDC_DVM_RAMP_MASK       (1U << 5)
/* REG81 force-PWM bits for DCDC4..DCDC1 in bits 5:2 and spread-spectrum
 * bits 7:6 (6.13.2.70). */
#define TG28_SW_DCDC_SPREAD_ENABLE_MASK  (1U << 7)
#define TG28_SW_DCDC_SPREAD_RANGE_MASK   (1U << 6)
#define TG28_SW_DCDC_FORCE_PWM_MASK      0x3C
/* REG5C ts_cfg_data high field (6.13.2.58): bits 5:0 hold code bits 13:8. */
#define TG28_SW_TS_FIXED_H_MASK          0x3F
/* IRQ slot 21 (REG42 bit5) is a reserved read-only bit on the switch-charger
 * variant and deliberately has no tg28_sw_irq_t symbol. */
#define TG28_SW_IRQ_RESERVED_SLOT        21
#define TG28_SW_REGISTER_TIMEOUT_MS      100
#define TG28_SW_CHARGER_SETTLE_MS        1000
#define TG28_SW_WRITE_BUFFER_SIZE        4

typedef struct {
    const char *name;
    uint8_t voltage_register;
    uint8_t enable_register;
    uint8_t enable_mask;
    uint8_t voltage_mask;
    uint16_t minimum_mv;
    uint16_t maximum_mv;
    uint16_t step_mv;
    uint16_t second_range_start_mv;
    uint16_t second_step_mv;
} regulator_config_t;

struct tg28_sw_device_t {
    i2c_master_dev_handle_t i2c_device;
    SemaphoreHandle_t lock;
};

static const char *TAG = "tg28_sw";

static const regulator_config_t s_regulators[TG28_SW_REGULATOR_COUNT] = {
    [TG28_SW_DCDC1] = {"dcdc1", TG28_SW_REG_DCDC1_VOLTAGE, TG28_SW_REG_DCDC_ENABLE,
        1U << 0, 0x1F, 1500, 3400, 100, 0, 0
    },
    [TG28_SW_DCDC2] = {"dcdc2", TG28_SW_REG_DCDC2_VOLTAGE, TG28_SW_REG_DCDC_ENABLE,
        1U << 1, 0x7F, 500, 1540, 10, 1200, 20
    },
    [TG28_SW_DCDC3] = {"dcdc3", TG28_SW_REG_DCDC3_VOLTAGE, TG28_SW_REG_DCDC_ENABLE,
        1U << 2, 0x7F, 500, 1540, 10, 1200, 20
    },
    [TG28_SW_DCDC4] = {"dcdc4", TG28_SW_REG_DCDC4_VOLTAGE, TG28_SW_REG_DCDC_ENABLE,
        1U << 3, 0x7F, 500, 1840, 10, 1200, 20
    },
    [TG28_SW_ALDO1] = {"aldo1", TG28_SW_REG_ALDO1_VOLTAGE, TG28_SW_REG_LDO_ENABLE0,
        1U << 0, 0x1F, 500, 3500, 100, 0, 0
    },
    [TG28_SW_ALDO2] = {"aldo2", TG28_SW_REG_ALDO2_VOLTAGE, TG28_SW_REG_LDO_ENABLE0,
        1U << 1, 0x1F, 500, 3500, 100, 0, 0
    },
    [TG28_SW_ALDO3] = {"aldo3", TG28_SW_REG_ALDO3_VOLTAGE, TG28_SW_REG_LDO_ENABLE0,
        1U << 2, 0x1F, 500, 3500, 100, 0, 0
    },
    /* The vendor linear-range table for ALDO4 (second segment 3400/200mV)
     * makes 3500mV unreachable. The single 500-3500mV / 100mV range used
     * here matches the vendor decode for every code (code 30 = 3500mV,
     * code 31 clamps to 3500mV) and keeps the full range settable. */
    [TG28_SW_ALDO4] = {"aldo4", TG28_SW_REG_ALDO4_VOLTAGE, TG28_SW_REG_LDO_ENABLE0,
        1U << 3, 0x1F, 500, 3500, 100, 0, 0
    },
    [TG28_SW_BLDO1] = {"bldo1", TG28_SW_REG_BLDO1_VOLTAGE, TG28_SW_REG_LDO_ENABLE0,
        1U << 4, 0x1F, 500, 3500, 100, 0, 0
    },
    [TG28_SW_BLDO2] = {"bldo2", TG28_SW_REG_BLDO2_VOLTAGE, TG28_SW_REG_LDO_ENABLE0,
        1U << 5, 0x1F, 500, 3500, 100, 0, 0
    },
    /* REG98 (6.13.2.83) is self-contradictory: the headline allows
     * 0.5-1.4V in 50mV steps while the last enumeration entry maps code 19
     * to 1.40V; the step interpretation wins (code 18 = 1400mV), matching
     * the vendor driver behavior. */
    [TG28_SW_CPUSLDO] = {"cpusldo", TG28_SW_REG_CPUSLDO_VOLTAGE, TG28_SW_REG_LDO_ENABLE0,
        1U << 6, 0x1F, 500, 1400, 50, 0, 0
    },
    /* DLDO1/DLDO2 correspond to the vendor driver's LDO10/LDO11. The DLDO1
     * datasheet section (REG99, 6.13.2.84) is self-contradictory: the
     * headline allows 0.5-3.4V while its own enumeration stops at 3.3V
     * (code 28) and marks codes 29-31 reserved; the enumeration wins, so the
     * maximum here is 3300mV. On boards whose OTP straps DLDO1/DLDO2 as
     * load switches, the voltage register is inert and
     * tg28_sw_switch_enable() controls the rail. */
    [TG28_SW_DLDO1] = {"dldo1", TG28_SW_REG_DLDO1_VOLTAGE, TG28_SW_REG_LDO_ENABLE0,
        1U << 7, 0x1F, 500, 3300, 100, 0, 0
    },
    /* REG9A (6.13.2.85) carries the same enumeration/step contradiction as
     * REG98; the step interpretation wins here too. */
    [TG28_SW_DLDO2] = {"dldo2", TG28_SW_REG_DLDO2_VOLTAGE, TG28_SW_REG_LDO_ENABLE1,
        1U << 0, 0x1F, 500, 1400, 50, 0, 0
    },
};

/* ADC channel layout from datasheet 6.10 Table 6-7: the 14-bit result reads
 * as high[5:0] then low[7:0]; VBAT/VBUS/VSYS convert at 1mV/LSB, TS at
 * 0.5mV/LSB, and TDIE at 0.1mV/LSB. The enable bits live in REG30. */
typedef struct {
    uint8_t high_register;
    uint8_t enable_mask;
    uint8_t numerator;
    uint8_t denominator;
} adc_channel_config_t;

static const adc_channel_config_t s_adc_channels[TG28_SW_ADC_CHANNEL_COUNT] = {
    [TG28_SW_ADC_CHANNEL_VBAT] = {TG28_SW_REG_VBAT_H, 1U << 0, 1, 1},
    [TG28_SW_ADC_CHANNEL_TS] = {TG28_SW_REG_TS_H, 1U << 1, 1, 2},
    [TG28_SW_ADC_CHANNEL_VBUS] = {TG28_SW_REG_VBUS_H, 1U << 2, 1, 1},
    [TG28_SW_ADC_CHANNEL_VSYS] = {TG28_SW_REG_VSYS_H, 1U << 3, 1, 1},
    [TG28_SW_ADC_CHANNEL_TDIE] = {TG28_SW_REG_TDIE_H, 1U << 4, 1, 10},
};

static const uint8_t s_irq_enable_registers[TG28_SW_IRQ_BANK_COUNT] = {
    TG28_SW_REG_INT_ENABLE0, TG28_SW_REG_INT_ENABLE1, TG28_SW_REG_INT_ENABLE2,
};

/* The switch channels share the physical enable bits of the DLDO1/DLDO2
 * LDOs: DC1SW is REG90 bit7 and DC4SW is REG91 bit0 (datasheet 6.13.2.75-76);
 * there is no separate switch control bit. */
typedef struct {
    const char *name;
    uint8_t enable_register;
    uint8_t enable_mask;
} power_switch_config_t;

static const power_switch_config_t s_power_switches[TG28_SW_SWITCH_COUNT] = {
    [TG28_SW_SWITCH_DC1SW] = {"dc1sw", TG28_SW_REG_LDO_ENABLE0, 1U << 7},
    [TG28_SW_SWITCH_DC4SW] = {"dc4sw", TG28_SW_REG_LDO_ENABLE1, 1U << 0},
};

static bool regulator_is_valid(tg28_sw_regulator_t regulator)
{
    return regulator >= 0 && regulator < TG28_SW_REGULATOR_COUNT;
}

static bool switch_is_valid(tg28_sw_power_switch_t sw)
{
    return sw >= 0 && sw < TG28_SW_SWITCH_COUNT;
}

static bool irq_bank_is_valid(tg28_sw_irq_bank_t bank)
{
    return bank >= 0 && bank < TG28_SW_IRQ_BANK_COUNT;
}

static bool irq_is_valid(tg28_sw_irq_t irq)
{
    return irq >= 0 && irq < TG28_SW_IRQ_COUNT &&
           irq != TG28_SW_IRQ_RESERVED_SLOT;
}

static bool adc_channel_is_valid(tg28_sw_adc_channel_t channel)
{
    return channel >= 0 && channel < TG28_SW_ADC_CHANNEL_COUNT;
}

static esp_err_t lock_device(tg28_sw_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL && handle->i2c_device != NULL && handle->lock != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid device handle");
    ESP_RETURN_ON_FALSE(xSemaphoreTake(handle->lock, portMAX_DELAY) == pdTRUE,
                        ESP_ERR_TIMEOUT, TAG, "device lock failed");
    return ESP_OK;
}

static void unlock_device(tg28_sw_handle_t handle)
{
    xSemaphoreGive(handle->lock);
}

static esp_err_t read_registers(tg28_sw_handle_t handle, uint8_t reg,
                                void *data, size_t size)
{
    return i2c_master_transmit_receive(handle->i2c_device, &reg, sizeof(reg),
                                       data, size, TG28_SW_REGISTER_TIMEOUT_MS);
}

static esp_err_t write_registers(tg28_sw_handle_t handle, uint8_t reg,
                                 const void *data, size_t size)
{
    ESP_RETURN_ON_FALSE(size <= TG28_SW_WRITE_BUFFER_SIZE - 1,
                        ESP_ERR_INVALID_SIZE, TAG, "register write is too large");
    uint8_t buffer[TG28_SW_WRITE_BUFFER_SIZE] = {reg};
    memcpy(&buffer[1], data, size);
    return i2c_master_transmit(handle->i2c_device, buffer, size + 1,
                               TG28_SW_REGISTER_TIMEOUT_MS);
}

static esp_err_t update_bits(tg28_sw_handle_t handle, uint8_t reg,
                             uint8_t mask, uint8_t value)
{
    uint8_t current = 0;
    ESP_RETURN_ON_ERROR(read_registers(handle, reg, &current, sizeof(current)),
                        TAG, "register read failed");
    current = (current & ~mask) | (value & mask);
    return write_registers(handle, reg, &current, sizeof(current));
}

static esp_err_t device_io_read(void *context, uint8_t reg, void *data,
                                size_t size)
{
    return read_registers((tg28_sw_handle_t)context, reg, data, size);
}

static esp_err_t device_io_write(void *context, uint8_t reg, const void *data,
                                 size_t size)
{
    return write_registers((tg28_sw_handle_t)context, reg, data, size);
}

static void device_io_delay_ms(void *context, uint32_t milliseconds)
{
    (void)context;
    vTaskDelay(pdMS_TO_TICKS(milliseconds));
}

static esp_err_t io_write_register_verified(const tg28_sw_register_io_t *io,
        uint8_t reg, uint8_t value, uint8_t verify_mask)
{
    const esp_err_t write_error = io->write(io->context, reg, &value,
                                            sizeof(value));
    uint8_t current = 0;
    const esp_err_t read_error = io->read(io->context, reg, &current,
                                          sizeof(current));
    if (read_error != ESP_OK) {
        return write_error != ESP_OK ? write_error : read_error;
    }
    if ((current & verify_mask) != (value & verify_mask)) {
        return write_error != ESP_OK ? write_error : ESP_ERR_INVALID_RESPONSE;
    }
    /* An I2C timeout can be reported after the PMIC accepted the byte. A
     * matching readback proves the requested register state is in place. */
    return ESP_OK;
}

static esp_err_t io_reset_gauge_mcu(const tg28_sw_register_io_t *io,
                                    bool *reset_released)
{
    *reset_released = true;
    uint8_t mode = 0;
    ESP_RETURN_ON_ERROR(io->read(io->context, TG28_SW_REG_MODE, &mode,
                                 sizeof(mode)), TAG,
                        "fuel-gauge MCU mode read failed");

    /* Once reset assertion is attempted, always try to release it even when
     * the write readback fails: the PMIC may have accepted the write before
     * the I2C error was reported. A second deassert attempt handles one
     * transient write/readback failure. The out parameter lets cleanup keep
     * the watchdog disabled if release still cannot be confirmed. */
    *reset_released = false;
    const esp_err_t assert_error = io_write_register_verified(
                                       io, TG28_SW_REG_MODE,
                                       mode | TG28_SW_GAUGE_MCU_RESET_MASK,
                                       TG28_SW_GAUGE_MCU_RESET_MASK);
    const uint8_t released_mode = mode & ~TG28_SW_GAUGE_MCU_RESET_MASK;
    esp_err_t release_error = io_write_register_verified(
                                  io, TG28_SW_REG_MODE, released_mode,
                                  TG28_SW_GAUGE_MCU_RESET_MASK);
    if (release_error != ESP_OK) {
        const esp_err_t retry_error = io_write_register_verified(
                                          io, TG28_SW_REG_MODE,
                                          released_mode,
                                          TG28_SW_GAUGE_MCU_RESET_MASK);
        if (retry_error == ESP_OK) {
            *reset_released = true;
        } else {
            return retry_error;
        }
    } else {
        *reset_released = true;
    }

    return assert_error != ESP_OK ? assert_error : release_error;
}

static esp_err_t io_set_gauge_control(const tg28_sw_register_io_t *io,
                                      uint8_t value)
{
    return io_write_register_verified(io, TG28_SW_REG_FUEL_GAUGE_CONTROL,
                                      value, TG28_SW_GAUGE_CONTROL_WRITE_MASK);
}

static esp_err_t finish_battery_model_io(const tg28_sw_register_io_t *io,
        uint8_t final_gauge_control, bool gauge_control_touched,
        uint8_t original_modules, bool modules_touched,
        bool reset_released, esp_err_t operation_error)
{
    esp_err_t gauge_cleanup_error = ESP_OK;
    bool gauge_state_safe = reset_released;
    if (gauge_control_touched) {
        /* One REGA2 transaction both closes BROM and selects the source that
         * the final reset must activate. This mirrors the vendor sequence and
         * avoids a reset between two partially-applied control writes. */
        gauge_cleanup_error = io_set_gauge_control(
                                  io, final_gauge_control &
                                  ~TG28_SW_BROM_WRITER_ENABLE_MASK);
        if (gauge_cleanup_error == ESP_OK) {
            gauge_cleanup_error = io_reset_gauge_mcu(io, &gauge_state_safe);
        } else {
            gauge_state_safe = false;
        }
    }

    esp_err_t module_restore_error = ESP_OK;
    if (modules_touched && gauge_state_safe) {
        /* Restore the complete entry state only after BROM is closed and
         * REG17 reset release is confirmed. Otherwise leave the prepared
         * charger/watchdog-off, gauge-on state for explicit recovery. */
        module_restore_error = io_write_register_verified(
                                   io, TG28_SW_REG_MODULE_ENABLE,
                                   original_modules,
                                   TG28_SW_MODULE_ENABLE_WRITE_MASK);
    }

    if (gauge_cleanup_error != ESP_OK) {
        return gauge_cleanup_error;
    }
    if (module_restore_error != ESP_OK) {
        return module_restore_error;
    }
    return operation_error;
}

/* The encode/decode helpers below are intentionally non-static so the
 * test app can exercise the pure conversion logic without I2C hardware.
 * Their declarations live in priv_include/tg28_sw_priv.h. */
esp_err_t tg28_sw_encode_charge_current(uint16_t milliamps, uint8_t *code)
{
    ESP_RETURN_ON_FALSE(code != NULL, ESP_ERR_INVALID_ARG, TAG, "code is NULL");
    uint16_t value = 0;
    if (milliamps <= 200) {
        ESP_RETURN_ON_FALSE(milliamps % 25 == 0, ESP_ERR_INVALID_ARG,
                            TAG, "charge current is not representable");
        value = milliamps / 25;
    } else {
        ESP_RETURN_ON_FALSE(milliamps >= 300 && milliamps <= 1500 &&
                            milliamps % 100 == 0,
                            ESP_ERR_INVALID_ARG, TAG,
                            "charge current is not representable");
        value = 8 + (milliamps - 200) / 100;
    }
    *code = (uint8_t)value;
    return ESP_OK;
}

uint16_t tg28_sw_decode_charge_current(uint8_t code)
{
    code &= TG28_SW_CHARGE_CURRENT_MASK;
    return code <= 8 ? (uint16_t)code * 25 :
           (uint16_t)(200 + (code - 8) * 100);
}

/* REG16 input current limit levels (mA) indexed by the 3-bit code. */
static const uint16_t s_input_current_limits[] = {100, 500, 900, 1000, 1500, 2000};
#define TG28_SW_INPUT_CURRENT_LIMIT_COUNT \
    (sizeof(s_input_current_limits) / sizeof(s_input_current_limits[0]))

esp_err_t tg28_sw_encode_input_current_limit(uint16_t milliamps, uint8_t *code)
{
    ESP_RETURN_ON_FALSE(code != NULL, ESP_ERR_INVALID_ARG, TAG, "code is NULL");
    for (uint8_t i = 0; i < TG28_SW_INPUT_CURRENT_LIMIT_COUNT; ++i) {
        if (s_input_current_limits[i] == milliamps) {
            *code = i;
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG, "input current limit is not representable");
    return ESP_ERR_INVALID_ARG;
}

uint16_t tg28_sw_decode_input_current_limit(uint8_t code)
{
    code &= TG28_SW_INPUT_CURRENT_LIMIT_MASK;
    return code < TG28_SW_INPUT_CURRENT_LIMIT_COUNT ?
           s_input_current_limits[code] : 0;
}

/* REG64 charge termination voltage levels (mV) indexed by the 3-bit code.
 * On the switch-charger variant code 0 is reserved (datasheet 6.13.2.63),
 * unlike the linear-charger variant, so the entry stays 0 and can be neither
 * encoded nor decoded. Codes 6-7 are reserved as well. */
static const uint16_t s_charge_voltages[] = {0, 4000, 4100, 4200, 4350, 4400};
#define TG28_SW_CHARGE_VOLTAGE_COUNT \
    (sizeof(s_charge_voltages) / sizeof(s_charge_voltages[0]))

esp_err_t tg28_sw_encode_charge_voltage(uint16_t millivolts, uint8_t *code)
{
    ESP_RETURN_ON_FALSE(code != NULL, ESP_ERR_INVALID_ARG, TAG, "code is NULL");
    for (uint8_t i = 0; i < TG28_SW_CHARGE_VOLTAGE_COUNT; ++i) {
        if (s_charge_voltages[i] != 0 && s_charge_voltages[i] == millivolts) {
            *code = i;
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG, "charge voltage is not representable");
    return ESP_ERR_INVALID_ARG;
}

uint16_t tg28_sw_decode_charge_voltage(uint8_t code)
{
    code &= TG28_SW_CHARGE_VOLTAGE_MASK;
    return code < TG28_SW_CHARGE_VOLTAGE_COUNT ? s_charge_voltages[code] : 0;
}

/* REG15 VINDPM: millivolts = 3880 + 80 * code, codes 0-15 (3880-5080mV,
 * datasheet 6.13.2.11). The narrower 4040-4680mV window of the vendor driver
 * is a software choice, not a hardware limit, and is not enforced here. */
#define TG28_SW_VINDPM_BASE_MV    3880
#define TG28_SW_VINDPM_STEP_MV    80
#define TG28_SW_VINDPM_MAX_CODE   15

esp_err_t tg28_sw_encode_vindpm(uint16_t millivolts, uint8_t *code)
{
    ESP_RETURN_ON_FALSE(code != NULL, ESP_ERR_INVALID_ARG, TAG, "code is NULL");
    ESP_RETURN_ON_FALSE(millivolts >= TG28_SW_VINDPM_BASE_MV &&
                        (millivolts - TG28_SW_VINDPM_BASE_MV) %
                        TG28_SW_VINDPM_STEP_MV == 0,
                        ESP_ERR_INVALID_ARG, TAG,
                        "VINDPM voltage is not representable");
    const uint16_t value =
        (millivolts - TG28_SW_VINDPM_BASE_MV) / TG28_SW_VINDPM_STEP_MV;
    ESP_RETURN_ON_FALSE(value <= TG28_SW_VINDPM_MAX_CODE,
                        ESP_ERR_INVALID_ARG, TAG,
                        "VINDPM voltage is out of range");
    *code = (uint8_t)value;
    return ESP_OK;
}

uint16_t tg28_sw_decode_vindpm(uint8_t code)
{
    code &= TG28_SW_VINDPM_MASK;
    return TG28_SW_VINDPM_BASE_MV + (uint16_t)code * TG28_SW_VINDPM_STEP_MV;
}

/* REG61 precharge and REG63 termination currents share the same 4-bit
 * coding: 25mA per step for codes 0-8 (0-200mA); codes 9-15 are reserved
 * (datasheet 6.13.2.60 and 6.13.2.62). */
esp_err_t tg28_sw_encode_precharge_current(uint16_t milliamps, uint8_t *code)
{
    ESP_RETURN_ON_FALSE(code != NULL, ESP_ERR_INVALID_ARG, TAG, "code is NULL");
    ESP_RETURN_ON_FALSE(milliamps <= 200 && milliamps % 25 == 0,
                        ESP_ERR_INVALID_ARG, TAG,
                        "precharge current is not representable");
    *code = (uint8_t)(milliamps / 25);
    return ESP_OK;
}

uint16_t tg28_sw_decode_precharge_current(uint8_t code)
{
    code &= TG28_SW_PRECHARGE_CURRENT_MASK;
    return code <= 8 ? (uint16_t)code * 25 : 0;
}

esp_err_t tg28_sw_encode_termination_current(uint16_t milliamps, uint8_t *code)
{
    ESP_RETURN_ON_FALSE(code != NULL, ESP_ERR_INVALID_ARG, TAG, "code is NULL");
    ESP_RETURN_ON_FALSE(milliamps <= 200 && milliamps % 25 == 0,
                        ESP_ERR_INVALID_ARG, TAG,
                        "termination current is not representable");
    *code = (uint8_t)(milliamps / 25);
    return ESP_OK;
}

uint16_t tg28_sw_decode_termination_current(uint8_t code)
{
    code &= TG28_SW_TERMINATION_CURRENT_MASK;
    return code <= 8 ? (uint16_t)code * 25 : 0;
}

/* REG1A low-battery warning thresholds (datasheet 6.13.2.16): bits 3:0 select
 * level1 as 0-15% in 1% steps; bits 7:4 select level2 as 5-20% in 1% steps
 * (code = percent - 5). Every byte value decodes to a valid pair. */
esp_err_t tg28_sw_encode_low_battery_warning(uint8_t level1_percent,
        uint8_t level2_percent, uint8_t *value)
{
    ESP_RETURN_ON_FALSE(value != NULL, ESP_ERR_INVALID_ARG, TAG, "value is NULL");
    ESP_RETURN_ON_FALSE(level1_percent <= 15, ESP_ERR_INVALID_ARG, TAG,
                        "level1 threshold is out of range");
    ESP_RETURN_ON_FALSE(level2_percent >= 5 && level2_percent <= 20,
                        ESP_ERR_INVALID_ARG, TAG,
                        "level2 threshold is out of range");
    *value = (uint8_t)(((level2_percent - 5) << 4) | level1_percent);
    return ESP_OK;
}

void tg28_sw_decode_low_battery_warning(uint8_t value, uint8_t *level1_percent,
                                        uint8_t *level2_percent)
{
    *level1_percent = value & 0x0F;
    *level2_percent = (uint8_t)(((value >> 4) & 0x0F) + 5);
}

static esp_err_t encode_from_table(uint16_t value, const uint16_t *table,
                                   uint8_t table_size, uint8_t *code)
{
    for (uint8_t i = 0; i < table_size; i++) {
        if (table[i] == value) {
            *code = i;
            return ESP_OK;
        }
    }
    return ESP_ERR_INVALID_ARG;
}

/* REG27 power-key timing windows (datasheet 6.13.2.25): bits5:4 IRQLEVEL
 * 1000/1500/2000/2500 ms, bits3:2 OFFLEVEL 4000/6000/8000/10000 ms, and
 * bits1:0 ONLEVEL 128/512/1000/2000 ms. Every byte value decodes to a
 * valid window triplet. */
static const uint16_t s_irqlevel_windows[4] = {1000, 1500, 2000, 2500};
static const uint16_t s_offlevel_windows[4] = {4000, 6000, 8000, 10000};
static const uint16_t s_onlevel_windows[4] = {128, 512, 1000, 2000};

esp_err_t tg28_sw_encode_powerkey_levels(uint16_t irqlevel_ms,
        uint16_t offlevel_ms, uint16_t onlevel_ms, uint8_t *value)
{
    ESP_RETURN_ON_FALSE(value != NULL, ESP_ERR_INVALID_ARG, TAG, "value is NULL");
    uint8_t irqlevel = 0, offlevel = 0, onlevel = 0;
    esp_err_t error = encode_from_table(irqlevel_ms, s_irqlevel_windows, 4, &irqlevel);
    if (error == ESP_OK) {
        error = encode_from_table(offlevel_ms, s_offlevel_windows, 4, &offlevel);
    }
    if (error == ESP_OK) {
        error = encode_from_table(onlevel_ms, s_onlevel_windows, 4, &onlevel);
    }
    if (error == ESP_OK) {
        *value = (uint8_t)((irqlevel << 4) | (offlevel << 2) | onlevel);
    }
    return error;
}

void tg28_sw_decode_powerkey_levels(uint8_t value, uint16_t *irqlevel_ms,
                                    uint16_t *offlevel_ms, uint16_t *onlevel_ms)
{
    *irqlevel_ms = s_irqlevel_windows[(value & TG28_SW_IRQLEVEL_MASK) >> 4];
    *offlevel_ms = s_offlevel_windows[(value & TG28_SW_OFFLEVEL_MASK) >> 2];
    *onlevel_ms = s_onlevel_windows[value & TG28_SW_ONLEVEL_MASK];
}

static esp_err_t encode_voltage(const regulator_config_t *config,
                                uint16_t millivolts, uint8_t *code)
{
    if (millivolts < config->minimum_mv || millivolts > config->maximum_mv) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t encoded = 0;
    if (config->second_range_start_mv != 0 &&
            millivolts > config->second_range_start_mv) {
        const uint16_t first_codes =
            (config->second_range_start_mv - config->minimum_mv) / config->step_mv;
        if ((millivolts - config->second_range_start_mv) % config->second_step_mv != 0) {
            return ESP_ERR_INVALID_ARG;
        }
        encoded = first_codes +
                  (millivolts - config->second_range_start_mv) / config->second_step_mv;
    } else {
        if ((millivolts - config->minimum_mv) % config->step_mv != 0) {
            return ESP_ERR_INVALID_ARG;
        }
        encoded = (millivolts - config->minimum_mv) / config->step_mv;
    }

    if (encoded > config->voltage_mask) {
        return ESP_ERR_INVALID_ARG;
    }
    *code = (uint8_t)encoded;
    return ESP_OK;
}

static uint16_t decode_voltage(const regulator_config_t *config, uint8_t code)
{
    uint16_t millivolts = 0;
    if (config->second_range_start_mv != 0) {
        const uint16_t first_codes =
            (config->second_range_start_mv - config->minimum_mv) / config->step_mv;
        if (code > first_codes) {
            millivolts = config->second_range_start_mv +
                         (code - first_codes) * config->second_step_mv;
            return millivolts < config->maximum_mv ? millivolts : config->maximum_mv;
        }
    }
    millivolts = config->minimum_mv + code * config->step_mv;
    return millivolts < config->maximum_mv ? millivolts : config->maximum_mv;
}

esp_err_t tg28_sw_encode_regulator_voltage(tg28_sw_regulator_t regulator,
        uint16_t millivolts, uint8_t *code)
{
    ESP_RETURN_ON_FALSE(regulator_is_valid(regulator) && code != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid voltage encode request");
    return encode_voltage(&s_regulators[regulator], millivolts, code);
}

uint16_t tg28_sw_decode_regulator_voltage(tg28_sw_regulator_t regulator,
        uint8_t code)
{
    if (!regulator_is_valid(regulator)) {
        return 0;
    }
    const regulator_config_t *config = &s_regulators[regulator];
    return decode_voltage(config, code & config->voltage_mask);
}

/* REG50 TS current-source values (uA) indexed by the 2-bit code. */
static const uint16_t s_ts_currents[] = {20, 40, 50, 60};
#define TG28_SW_TS_CURRENT_COUNT (sizeof(s_ts_currents) / sizeof(s_ts_currents[0]))

esp_err_t tg28_sw_encode_ts_current(uint16_t microamps, uint8_t *code)
{
    ESP_RETURN_ON_FALSE(code != NULL, ESP_ERR_INVALID_ARG, TAG, "code is NULL");
    for (uint8_t i = 0; i < TG28_SW_TS_CURRENT_COUNT; ++i) {
        if (s_ts_currents[i] == microamps) {
            *code = i;
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG, "TS current is not representable");
    return ESP_ERR_INVALID_ARG;
}

uint16_t tg28_sw_decode_adc_channel(tg28_sw_adc_channel_t channel,
                                    uint8_t high, uint8_t low)
{
    if (!adc_channel_is_valid(channel)) {
        return 0;
    }
    const uint16_t raw = ((uint16_t)(high & TG28_SW_ADC_VALUE_MASK) << 8) | low;
    const adc_channel_config_t *config = &s_adc_channels[channel];
    return (uint16_t)(((uint32_t)raw * config->numerator) / config->denominator);
}

esp_err_t tg28_sw_create(i2c_master_bus_handle_t bus,
                         const tg28_sw_config_t *config,
                         tg28_sw_handle_t *ret_handle)
{
    ESP_RETURN_ON_FALSE(bus != NULL && ret_handle != NULL, ESP_ERR_INVALID_ARG,
                        TAG, "invalid create arguments");
    *ret_handle = NULL;

    const tg28_sw_config_t default_config = TG28_SW_CONFIG_DEFAULT();
    const tg28_sw_config_t *device_config = config != NULL ? config : &default_config;
    ESP_RETURN_ON_FALSE(device_config->device_address <= 0x7F &&
                        device_config->scl_speed_hz > 0 &&
                        ((device_config->battery_model == NULL &&
                          device_config->battery_model_size == 0) ||
                         (device_config->battery_model != NULL &&
                          device_config->battery_model_size == TG28_SW_BATTERY_MODEL_SIZE)),
                        ESP_ERR_INVALID_ARG, TAG, "invalid device configuration");

    tg28_sw_handle_t handle = calloc(1, sizeof(*handle));
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_NO_MEM, TAG, "allocation failed");
    handle->lock = xSemaphoreCreateMutex();
    if (handle->lock == NULL) {
        free(handle);
        return ESP_ERR_NO_MEM;
    }

    const i2c_device_config_t i2c_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_config->device_address,
        .scl_speed_hz = device_config->scl_speed_hz,
    };
    esp_err_t error = i2c_master_bus_add_device(bus, &i2c_config, &handle->i2c_device);
    if (error != ESP_OK) {
        vSemaphoreDelete(handle->lock);
        free(handle);
        return error;
    }

    uint8_t chip_id = 0;
    error = read_registers(handle, TG28_SW_REG_CHIP_ID, &chip_id, sizeof(chip_id));
    if (error != ESP_OK) {
        i2c_master_bus_rm_device(handle->i2c_device);
        vSemaphoreDelete(handle->lock);
        free(handle);
        return error;
    }
    if (!tg28_sw_is_supported_chip_id(chip_id)) {
        ESP_LOGW(TAG, "unexpected chip ID: 0x%02x", chip_id);
    }

    if (device_config->battery_model != NULL && device_config->battery_model_size > 0) {
        error = tg28_sw_program_battery_model(handle, device_config->battery_model,
                                              device_config->battery_model_size);
        if (error != ESP_OK) {
            i2c_master_bus_rm_device(handle->i2c_device);
            vSemaphoreDelete(handle->lock);
            free(handle);
            return error;
        }
    }

    *ret_handle = handle;
    return ESP_OK;
}

esp_err_t tg28_sw_delete(tg28_sw_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle is NULL");
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

esp_err_t tg28_sw_get_chip_id(tg28_sw_handle_t handle, uint8_t *chip_id)
{
    ESP_RETURN_ON_FALSE(chip_id != NULL, ESP_ERR_INVALID_ARG, TAG, "chip ID is NULL");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = read_registers(handle, TG28_SW_REG_CHIP_ID,
                                           chip_id, sizeof(*chip_id));
    unlock_device(handle);
    return error;
}

bool tg28_sw_is_supported_chip_id(uint8_t chip_id)
{
    return chip_id == 0x47 || chip_id == 0x4A;
}

esp_err_t tg28_sw_read_registers(tg28_sw_handle_t handle, uint8_t register_address,
                                 uint8_t *values, size_t count)
{
    ESP_RETURN_ON_FALSE(handle != NULL && values != NULL && count > 0,
                        ESP_ERR_INVALID_ARG, TAG, "invalid raw read request");
    ESP_RETURN_ON_FALSE((size_t)UINT8_MAX - register_address + 1 >= count,
                        ESP_ERR_INVALID_SIZE, TAG, "raw read crosses the register map");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = read_registers(handle, register_address, values, count);
    unlock_device(handle);
    return error;
}

/* The caller must already hold the device lock. */
static esp_err_t read_adc_channel_locked(tg28_sw_handle_t handle,
        tg28_sw_adc_channel_t channel, uint16_t *millivolts)
{
    const adc_channel_config_t *config = &s_adc_channels[channel];
    uint8_t raw[2] = {0};
    ESP_RETURN_ON_ERROR(read_registers(handle, config->high_register,
                                       raw, sizeof(raw)),
                        TAG, "ADC channel read failed");
    *millivolts = tg28_sw_decode_adc_channel(channel, raw[0], raw[1]);
    return ESP_OK;
}

esp_err_t tg28_sw_get_status(tg28_sw_handle_t handle, tg28_sw_status_t *status)
{
    ESP_RETURN_ON_FALSE(status != NULL, ESP_ERR_INVALID_ARG, TAG, "status is NULL");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");

    uint8_t raw[2] = {0};
    memset(status, 0, sizeof(*status));
    esp_err_t error = read_registers(handle, TG28_SW_REG_CHIP_ID,
                                     &status->chip_id, sizeof(status->chip_id));
    if (error == ESP_OK) {
        error = read_registers(handle, TG28_SW_REG_COMMON_STATUS0, raw, sizeof(raw));
    }
    if (error == ESP_OK) {
        status->common_status0 = raw[0];
        status->common_status1 = raw[1];
        status->vbus_present = (raw[0] & TG28_SW_VBUS_PRESENT_MASK) != 0;
        status->battery_present = (raw[0] & TG28_SW_BATTERY_PRESENT_MASK) != 0;
        /* REG01 bits2:0: 0 trickle, 1 pre-charge, 2 CC, 3 CV - all active
         * charging; 4 charge done, 5 not charging. */
        const uint8_t charge_state = raw[1] & TG28_SW_CHARGE_STATE_MASK;
        status->charging = charge_state < TG28_SW_CHARGE_STATE_DONE;
        status->charge_done = charge_state == TG28_SW_CHARGE_STATE_DONE;
        error = read_adc_channel_locked(handle, TG28_SW_ADC_CHANNEL_VBAT,
                                        &status->battery_mv);
    }
    if (error == ESP_OK) {
        error = read_registers(handle, TG28_SW_REG_SOC,
                               &status->battery_percent,
                               sizeof(status->battery_percent));
    }
    if (status->battery_percent > 100) {
        status->battery_percent = 100;
    }

    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_power_on_source(tg28_sw_handle_t handle, uint8_t *source)
{
    ESP_RETURN_ON_FALSE(source != NULL, ESP_ERR_INVALID_ARG, TAG, "source is NULL");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = read_registers(handle, TG28_SW_REG_POWER_ON_SOURCE,
                                           source, sizeof(*source));
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_power_off(tg28_sw_handle_t handle)
{
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = update_bits(handle, TG28_SW_REG_COMMON_CONFIG,
                                        TG28_SW_SOFT_PWROFF_MASK,
                                        TG28_SW_SOFT_PWROFF_MASK);
    /* On success the write is acknowledged and the PMU enters its off
     * state, so this call often does not return. The lock is still
     * released unconditionally: a board can survive the command (external
     * supply, delayed rail collapse, tests), and a held mutex would
     * deadlock every later TG28 call. */
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_power_off_source(tg28_sw_handle_t handle, uint8_t *source)
{
    ESP_RETURN_ON_FALSE(source != NULL, ESP_ERR_INVALID_ARG, TAG, "source is NULL");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t value = 0;
    const esp_err_t error = read_registers(handle, TG28_SW_REG_POWER_OFF_SOURCE,
                                           &value, sizeof(value));
    if (error == ESP_OK) {
        /* Return the raw REG21 byte: all eight latched power-off sources
         * (datasheet 6.13.2.19) matter for forensics; the TG28_SW_POWER_OFF_SOURCE_*
         * bit definitions name them. */
        *source = value;
    }
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_set_irq_enable(tg28_sw_handle_t handle,
                                 tg28_sw_irq_bank_t bank, uint8_t mask)
{
    ESP_RETURN_ON_FALSE(irq_bank_is_valid(bank), ESP_ERR_INVALID_ARG,
                        TAG, "invalid IRQ bank");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = write_registers(handle, s_irq_enable_registers[bank],
                                            &mask, sizeof(mask));
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_irq_enable(tg28_sw_handle_t handle,
                                 tg28_sw_irq_bank_t bank, uint8_t *mask)
{
    ESP_RETURN_ON_FALSE(irq_bank_is_valid(bank) && mask != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid IRQ bank request");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = read_registers(handle, s_irq_enable_registers[bank],
                                           mask, sizeof(*mask));
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_configure_power_key_interrupts(tg28_sw_handle_t handle,
        uint8_t enabled_mask)
{
    ESP_RETURN_ON_FALSE((enabled_mask & ~TG28_SW_POWER_KEY_IRQ_MASK) == 0,
                        ESP_ERR_INVALID_ARG, TAG, "invalid power-key IRQ mask");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t bank_mask = 0;
    esp_err_t error = read_registers(handle,
                                     s_irq_enable_registers[TG28_SW_IRQ_BANK1],
                                     &bank_mask, sizeof(bank_mask));
    if (error == ESP_OK) {
        bank_mask = (bank_mask & ~TG28_SW_POWER_KEY_IRQ_MASK) |
                    (enabled_mask & TG28_SW_POWER_KEY_IRQ_MASK);
        error = write_registers(handle,
                                s_irq_enable_registers[TG28_SW_IRQ_BANK1],
                                &bank_mask, sizeof(bank_mask));
    }
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_set_ts_config(tg28_sw_handle_t handle,
                                tg28_sw_ts_mode_t mode,
                                tg28_sw_ts_current_source_t current_source,
                                uint16_t current_ua)
{
    ESP_RETURN_ON_FALSE(mode == TG28_SW_TS_MODE_BATTERY_NTC ||
                        mode == TG28_SW_TS_MODE_EXTERNAL_FIXED,
                        ESP_ERR_INVALID_ARG, TAG, "invalid TS mode");
    ESP_RETURN_ON_FALSE(current_source >= TG28_SW_TS_CURRENT_SOURCE_OFF &&
                        current_source <= TG28_SW_TS_CURRENT_SOURCE_ALWAYS_ON,
                        ESP_ERR_INVALID_ARG, TAG, "invalid TS current source");
    uint8_t current_code = 0;
    ESP_RETURN_ON_ERROR(tg28_sw_encode_ts_current(current_ua, &current_code),
                        TAG, "unsupported TS current");
    const uint8_t value = ((uint8_t)mode << 4) |
                          ((uint8_t)current_source << 2) | current_code;
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = update_bits(handle, TG28_SW_REG_TS_CONFIG,
                                        TG28_SW_TS_CONFIG_MASK, value);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_read_adc_channel(tg28_sw_handle_t handle,
                                   tg28_sw_adc_channel_t channel,
                                   uint16_t *millivolts)
{
    ESP_RETURN_ON_FALSE(adc_channel_is_valid(channel) && millivolts != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid ADC channel request");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = read_adc_channel_locked(handle, channel, millivolts);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_set_adc_channel_enable(tg28_sw_handle_t handle,
        tg28_sw_adc_channel_t channel, bool enable)
{
    ESP_RETURN_ON_FALSE(adc_channel_is_valid(channel), ESP_ERR_INVALID_ARG,
                        TAG, "invalid ADC channel");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const uint8_t mask = s_adc_channels[channel].enable_mask;
    const esp_err_t error = update_bits(handle, TG28_SW_REG_ADC_CHANNEL_ENABLE,
                                        mask, enable ? mask : 0);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_adc_channel_enable(tg28_sw_handle_t handle,
        tg28_sw_adc_channel_t channel, bool *enabled)
{
    ESP_RETURN_ON_FALSE(adc_channel_is_valid(channel) && enabled != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid ADC channel request");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t value = 0;
    const esp_err_t error = read_registers(handle, TG28_SW_REG_ADC_CHANNEL_ENABLE,
                                           &value, sizeof(value));
    if (error == ESP_OK) {
        *enabled = (value & s_adc_channels[channel].enable_mask) != 0;
    }
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_set_charge_current(tg28_sw_handle_t handle, uint16_t milliamps)
{
    uint8_t code = 0;
    ESP_RETURN_ON_ERROR(tg28_sw_encode_charge_current(milliamps, &code),
                        TAG, "unsupported charge current");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = update_bits(handle, TG28_SW_REG_CHARGE_CURRENT,
                                        TG28_SW_CHARGE_CURRENT_MASK, code);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_charge_current(tg28_sw_handle_t handle, uint16_t *milliamps)
{
    ESP_RETURN_ON_FALSE(milliamps != NULL, ESP_ERR_INVALID_ARG,
                        TAG, "charge current is NULL");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t code = 0;
    esp_err_t error = read_registers(handle, TG28_SW_REG_CHARGE_CURRENT,
                                     &code, sizeof(code));
    if (error == ESP_OK) {
        code &= TG28_SW_CHARGE_CURRENT_MASK;
        if (code > 21) {
            ESP_LOGE(TAG, "reserved charge-current code: 0x%02x", code);
            error = ESP_ERR_INVALID_RESPONSE;
        } else {
            *milliamps = tg28_sw_decode_charge_current(code);
        }
    }
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_set_input_current_limit(tg28_sw_handle_t handle,
        uint16_t milliamps)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle is NULL");
    uint8_t code = 0;
    ESP_RETURN_ON_ERROR(tg28_sw_encode_input_current_limit(milliamps, &code),
                        TAG, "unsupported input current limit");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = update_bits(handle, TG28_SW_REG_INPUT_CURRENT_LIMIT,
                                        TG28_SW_INPUT_CURRENT_LIMIT_MASK, code);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_input_current_limit(tg28_sw_handle_t handle,
        uint16_t *milliamps)
{
    ESP_RETURN_ON_FALSE(handle != NULL && milliamps != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid limit request");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t code = 0;
    esp_err_t error = read_registers(handle, TG28_SW_REG_INPUT_CURRENT_LIMIT,
                                     &code, sizeof(code));
    if (error == ESP_OK) {
        code &= TG28_SW_INPUT_CURRENT_LIMIT_MASK;
        if (code >= TG28_SW_INPUT_CURRENT_LIMIT_COUNT) {
            ESP_LOGE(TAG, "reserved input-current-limit code: 0x%02x", code);
            error = ESP_ERR_INVALID_RESPONSE;
        } else {
            *milliamps = tg28_sw_decode_input_current_limit(code);
        }
    }
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_set_charge_voltage(tg28_sw_handle_t handle,
                                     uint16_t millivolts)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle is NULL");
    uint8_t code = 0;
    ESP_RETURN_ON_ERROR(tg28_sw_encode_charge_voltage(millivolts, &code),
                        TAG, "unsupported charge voltage");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = update_bits(handle, TG28_SW_REG_CHARGE_VOLTAGE,
                                        TG28_SW_CHARGE_VOLTAGE_MASK, code);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_charge_voltage(tg28_sw_handle_t handle,
                                     uint16_t *millivolts)
{
    ESP_RETURN_ON_FALSE(handle != NULL && millivolts != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid voltage request");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t code = 0;
    esp_err_t error = read_registers(handle, TG28_SW_REG_CHARGE_VOLTAGE,
                                     &code, sizeof(code));
    if (error == ESP_OK) {
        code &= TG28_SW_CHARGE_VOLTAGE_MASK;
        const uint16_t decoded = tg28_sw_decode_charge_voltage(code);
        if (decoded == 0) {
            ESP_LOGE(TAG, "reserved charge-voltage code: 0x%02x", code);
            error = ESP_ERR_INVALID_RESPONSE;
        } else {
            *millivolts = decoded;
        }
    }
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_set_vindpm(tg28_sw_handle_t handle, uint16_t millivolts)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle is NULL");
    uint8_t code = 0;
    ESP_RETURN_ON_ERROR(tg28_sw_encode_vindpm(millivolts, &code),
                        TAG, "unsupported VINDPM voltage");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = update_bits(handle, TG28_SW_REG_VINDPM,
                                        TG28_SW_VINDPM_MASK, code);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_vindpm(tg28_sw_handle_t handle, uint16_t *millivolts)
{
    ESP_RETURN_ON_FALSE(handle != NULL && millivolts != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid VINDPM request");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t code = 0;
    esp_err_t error = read_registers(handle, TG28_SW_REG_VINDPM,
                                     &code, sizeof(code));
    if (error == ESP_OK) {
        *millivolts = tg28_sw_decode_vindpm(code);
    }
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_program_battery_model_io(const tg28_sw_register_io_t *io,
        const uint8_t *model, size_t size)
{
    ESP_RETURN_ON_FALSE(io != NULL && io->read != NULL && io->write != NULL &&
                        io->delay_ms != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid register transport");
    ESP_RETURN_ON_FALSE(model != NULL, ESP_ERR_INVALID_ARG,
                        TAG, "battery model is NULL");
    ESP_RETURN_ON_FALSE(size == TG28_SW_BATTERY_MODEL_SIZE, ESP_ERR_INVALID_SIZE,
                        TAG, "battery model must be exactly %u bytes", TG28_SW_BATTERY_MODEL_SIZE);

    uint8_t gauge_control = 0;
    esp_err_t error = io->read(io->context, TG28_SW_REG_FUEL_GAUGE_CONTROL,
                               &gauge_control, sizeof(gauge_control));
    if (error != ESP_OK) {
        return error;
    }
    ESP_RETURN_ON_FALSE((gauge_control & TG28_SW_BROM_WRITER_ENABLE_MASK) == 0,
                        ESP_ERR_INVALID_STATE, TAG,
                        "BROM writer was enabled before model programming");

    uint8_t module_enable = 0;
    error = io->read(io->context, TG28_SW_REG_MODULE_ENABLE,
                     &module_enable, sizeof(module_enable));
    if (error != ESP_OK) {
        return error;
    }

    /* The hardware guide assumes the gauge is running while BROM is
     * accessed. Pause the charger and watchdog, but force the gauge module
     * on so source selection and the reset pulse take effect. */
    const uint8_t modules_prepared =
        (module_enable | TG28_SW_GAUGE_ENABLE_MASK) &
        ~(TG28_SW_CHARGER_ENABLE_MASK | TG28_SW_WATCHDOG_ENABLE_MASK);
    bool modules_touched = modules_prepared != module_enable;
    bool gauge_control_touched = false;
    bool reset_released = true;
    if (modules_touched) {
        error = io_write_register_verified(io, TG28_SW_REG_MODULE_ENABLE,
                                           modules_prepared,
                                           TG28_SW_MODULE_ENABLE_WRITE_MASK);
    }
    if (error == ESP_OK) {
        io->delay_ms(io->context, TG28_SW_CHARGER_SETTLE_MS);
        error = io_reset_gauge_mcu(io, &reset_released);
    }

    /* The design guide programs and verifies the BROM window with bit4
     * clear (REGA2 0x00 -> 0x01). Preserve the undocumented/RW bit5 from
     * the entry snapshot while forcing ROM/source bit4 low during access. */
    const uint8_t brom_control = gauge_control &
                                 ~(TG28_SW_BROM_UPDATE_MARK_MASK |
                                   TG28_SW_BROM_WRITER_ENABLE_MASK);
    if (error == ESP_OK) {
        gauge_control_touched = true;
        error = io_set_gauge_control(io, brom_control);
    }
    if (error == ESP_OK) {
        error = io_set_gauge_control(io, brom_control |
                                     TG28_SW_BROM_WRITER_ENABLE_MASK);
    }
    const bool model_write_started = error == ESP_OK;
    for (size_t i = 0; error == ESP_OK && i < size; ++i) {
        error = io->write(io->context, TG28_SW_REG_BATTERY_MODEL,
                          &model[i], sizeof(model[i]));
    }

    if (error == ESP_OK) {
        error = io_set_gauge_control(io, brom_control);
    }
    if (error == ESP_OK) {
        error = io_set_gauge_control(io, brom_control |
                                     TG28_SW_BROM_WRITER_ENABLE_MASK);
    }
    for (size_t i = 0; error == ESP_OK && i < size; ++i) {
        uint8_t value = 0;
        error = io->read(io->context, TG28_SW_REG_BATTERY_MODEL,
                         &value, sizeof(value));
        if (error == ESP_OK && value != model[i]) {
            ESP_LOGE(TAG, "battery model verification failed at byte %u",
                     (unsigned)i);
            error = ESP_ERR_INVALID_RESPONSE;
        }
    }

    /* Once any model byte may have been written, an error must fall back to
     * ROM. Restoring an entry SRAM selection could activate a partial or
     * unverified model on the cleanup reset. */
    const uint8_t final_gauge_control = error == ESP_OK ?
                                        gauge_control |
                                        TG28_SW_BROM_UPDATE_MARK_MASK :
                                        (model_write_started ?
                                         gauge_control &
                                         ~TG28_SW_BROM_UPDATE_MARK_MASK :
                                         gauge_control);
    return finish_battery_model_io(io, final_gauge_control,
                                   gauge_control_touched, module_enable,
                                   modules_touched, reset_released, error);
}

esp_err_t tg28_sw_program_battery_model(tg28_sw_handle_t handle,
                                        const uint8_t *model, size_t size)
{
    ESP_RETURN_ON_FALSE(model != NULL, ESP_ERR_INVALID_ARG,
                        TAG, "battery model is NULL");
    ESP_RETURN_ON_FALSE(size == TG28_SW_BATTERY_MODEL_SIZE, ESP_ERR_INVALID_SIZE,
                        TAG, "battery model must be exactly %u bytes", TG28_SW_BATTERY_MODEL_SIZE);
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const tg28_sw_register_io_t io = {
        .context = handle,
        .read = device_io_read,
        .write = device_io_write,
        .delay_ms = device_io_delay_ms,
    };
    const esp_err_t error = tg28_sw_program_battery_model_io(&io, model, size);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_regulator_set_voltage(tg28_sw_handle_t handle,
                                        tg28_sw_regulator_t regulator,
                                        uint16_t millivolts)
{
    ESP_RETURN_ON_FALSE(regulator_is_valid(regulator), ESP_ERR_INVALID_ARG,
                        TAG, "invalid regulator");
    uint8_t code = 0;
    ESP_RETURN_ON_ERROR(tg28_sw_encode_regulator_voltage(regulator, millivolts,
                        &code),
                        TAG, "unsupported voltage");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const regulator_config_t *config = &s_regulators[regulator];
    const esp_err_t error = update_bits(handle, config->voltage_register,
                                        config->voltage_mask, code);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_regulator_get_voltage(tg28_sw_handle_t handle,
                                        tg28_sw_regulator_t regulator,
                                        uint16_t *millivolts)
{
    ESP_RETURN_ON_FALSE(regulator_is_valid(regulator) && millivolts != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid voltage request");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const regulator_config_t *config = &s_regulators[regulator];
    uint8_t code = 0;
    const esp_err_t error = read_registers(handle, config->voltage_register,
                                           &code, sizeof(code));
    if (error == ESP_OK) {
        *millivolts = tg28_sw_decode_regulator_voltage(regulator, code);
    }
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_regulator_enable(tg28_sw_handle_t handle,
                                   tg28_sw_regulator_t regulator,
                                   bool enable)
{
    ESP_RETURN_ON_FALSE(regulator_is_valid(regulator), ESP_ERR_INVALID_ARG,
                        TAG, "invalid regulator");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const regulator_config_t *config = &s_regulators[regulator];
    const esp_err_t error = update_bits(handle, config->enable_register,
                                        config->enable_mask,
                                        enable ? config->enable_mask : 0);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_regulator_is_enabled(tg28_sw_handle_t handle,
                                       tg28_sw_regulator_t regulator,
                                       bool *enabled)
{
    ESP_RETURN_ON_FALSE(regulator_is_valid(regulator) && enabled != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid state request");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const regulator_config_t *config = &s_regulators[regulator];
    uint8_t value = 0;
    const esp_err_t error = read_registers(handle, config->enable_register,
                                           &value, sizeof(value));
    if (error == ESP_OK) {
        *enabled = (value & config->enable_mask) != 0;
    }
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_and_clear_interrupts(tg28_sw_handle_t handle,
        uint8_t status[3])
{
    ESP_RETURN_ON_FALSE(status != NULL, ESP_ERR_INVALID_ARG, TAG, "status is NULL");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    esp_err_t error = read_registers(handle, TG28_SW_REG_INT_STATUS0, status, 3);
    if (error == ESP_OK) {
        error = write_registers(handle, TG28_SW_REG_INT_STATUS0, status, 3);
    }
    unlock_device(handle);
    return error;
}

const char *tg28_sw_regulator_name(tg28_sw_regulator_t regulator)
{
    return regulator_is_valid(regulator) ? s_regulators[regulator].name : "invalid";
}

esp_err_t tg28_sw_switch_enable(tg28_sw_handle_t handle,
                                tg28_sw_power_switch_t sw, bool enable)
{
    ESP_RETURN_ON_FALSE(switch_is_valid(sw), ESP_ERR_INVALID_ARG,
                        TAG, "invalid power switch");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const power_switch_config_t *config = &s_power_switches[sw];
    const esp_err_t error = update_bits(handle, config->enable_register,
                                        config->enable_mask,
                                        enable ? config->enable_mask : 0);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_switch_is_enabled(tg28_sw_handle_t handle,
                                    tg28_sw_power_switch_t sw, bool *enabled)
{
    ESP_RETURN_ON_FALSE(switch_is_valid(sw) && enabled != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid switch state request");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const power_switch_config_t *config = &s_power_switches[sw];
    uint8_t value = 0;
    const esp_err_t error = read_registers(handle, config->enable_register,
                                           &value, sizeof(value));
    if (error == ESP_OK) {
        *enabled = (value & config->enable_mask) != 0;
    }
    unlock_device(handle);
    return error;
}

const char *tg28_sw_switch_name(tg28_sw_power_switch_t sw)
{
    return switch_is_valid(sw) ? s_power_switches[sw].name : "invalid";
}

esp_err_t tg28_sw_set_irq_enable_bit(tg28_sw_handle_t handle,
                                     tg28_sw_irq_t irq, bool enable)
{
    ESP_RETURN_ON_FALSE(irq_is_valid(irq), ESP_ERR_INVALID_ARG,
                        TAG, "invalid IRQ source");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const uint8_t bit = (uint8_t)(1U << (irq % 8));
    const esp_err_t error = update_bits(handle,
                                        s_irq_enable_registers[irq / 8],
                                        bit, enable ? bit : 0);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_irq_enable_bit(tg28_sw_handle_t handle,
                                     tg28_sw_irq_t irq, bool *enabled)
{
    ESP_RETURN_ON_FALSE(irq_is_valid(irq) && enabled != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid IRQ bit request");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t value = 0;
    const esp_err_t error = read_registers(handle,
                                           s_irq_enable_registers[irq / 8],
                                           &value, sizeof(value));
    if (error == ESP_OK) {
        *enabled = (value & (1U << (irq % 8))) != 0;
    }
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_set_precharge_current(tg28_sw_handle_t handle,
                                        uint16_t milliamps)
{
    uint8_t code = 0;
    ESP_RETURN_ON_ERROR(tg28_sw_encode_precharge_current(milliamps, &code),
                        TAG, "unsupported precharge current");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = update_bits(handle, TG28_SW_REG_PRECHARGE_CURRENT,
                                        TG28_SW_PRECHARGE_CURRENT_MASK, code);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_precharge_current(tg28_sw_handle_t handle,
                                        uint16_t *milliamps)
{
    ESP_RETURN_ON_FALSE(milliamps != NULL, ESP_ERR_INVALID_ARG,
                        TAG, "precharge current is NULL");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t code = 0;
    esp_err_t error = read_registers(handle, TG28_SW_REG_PRECHARGE_CURRENT,
                                     &code, sizeof(code));
    if (error == ESP_OK) {
        code &= TG28_SW_PRECHARGE_CURRENT_MASK;
        if (code > 8) {
            ESP_LOGE(TAG, "reserved precharge-current code: 0x%02x", code);
            error = ESP_ERR_INVALID_RESPONSE;
        } else {
            *milliamps = tg28_sw_decode_precharge_current(code);
        }
    }
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_set_termination_current(tg28_sw_handle_t handle,
        uint16_t milliamps, bool enable)
{
    uint8_t code = 0;
    ESP_RETURN_ON_ERROR(tg28_sw_encode_termination_current(milliamps, &code),
                        TAG, "unsupported termination current");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    esp_err_t error = ESP_OK;
    if (enable) {
        error = update_bits(handle, TG28_SW_REG_TERMINATION_CURRENT,
                            TG28_SW_TERMINATION_ENABLE_MASK |
                            TG28_SW_TERMINATION_CURRENT_MASK,
                            TG28_SW_TERMINATION_ENABLE_MASK | code);
    } else {
        /* Disabling only clears the enable bit; the current code is kept. */
        error = update_bits(handle, TG28_SW_REG_TERMINATION_CURRENT,
                            TG28_SW_TERMINATION_ENABLE_MASK, 0);
    }
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_termination_current(tg28_sw_handle_t handle,
        uint16_t *milliamps, bool *enabled)
{
    ESP_RETURN_ON_FALSE(milliamps != NULL && enabled != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid termination request");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t value = 0;
    esp_err_t error = read_registers(handle, TG28_SW_REG_TERMINATION_CURRENT,
                                     &value, sizeof(value));
    if (error == ESP_OK) {
        *enabled = (value & TG28_SW_TERMINATION_ENABLE_MASK) != 0;
        const uint8_t code = value & TG28_SW_TERMINATION_CURRENT_MASK;
        if (code > 8) {
            ESP_LOGE(TAG, "reserved termination-current code: 0x%02x", code);
            error = ESP_ERR_INVALID_RESPONSE;
        } else {
            *milliamps = tg28_sw_decode_termination_current(code);
        }
    }
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_set_low_battery_warning(tg28_sw_handle_t handle,
        uint8_t level1_percent, uint8_t level2_percent)
{
    uint8_t value = 0;
    ESP_RETURN_ON_ERROR(tg28_sw_encode_low_battery_warning(level1_percent,
                        level2_percent, &value),
                        TAG, "unsupported warning threshold");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = write_registers(handle,
                                            TG28_SW_REG_LOW_BATTERY_WARNING,
                                            &value, sizeof(value));
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_low_battery_warning(tg28_sw_handle_t handle,
        uint8_t *level1_percent, uint8_t *level2_percent)
{
    ESP_RETURN_ON_FALSE(level1_percent != NULL && level2_percent != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid threshold request");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t value = 0;
    const esp_err_t error = read_registers(handle,
                                           TG28_SW_REG_LOW_BATTERY_WARNING,
                                           &value, sizeof(value));
    if (error == ESP_OK) {
        tg28_sw_decode_low_battery_warning(value, level1_percent,
                                           level2_percent);
    }
    unlock_device(handle);
    return error;
}

/* ------------------------------------------------------------------------
 * Feature-block APIs: every setter/getter below holds the device lock and
 * uses read-modify-write so unrelated bits (including reserved POR-1 bits)
 * are preserved. Datasheet section numbers are cited per function.
 * --------------------------------------------------------------------- */

/* The caller must already hold the device lock. */
static esp_err_t get_reg_bit_locked(tg28_sw_handle_t handle, uint8_t reg,
                                    uint8_t mask, bool *enabled)
{
    uint8_t value = 0;
    ESP_RETURN_ON_ERROR(read_registers(handle, reg, &value, sizeof(value)),
                        TAG, "register read failed");
    *enabled = (value & mask) != 0;
    return ESP_OK;
}

/* Encode helpers: exact-step validation, ESP_ERR_INVALID_ARG when the value
 * is not representable. */
static esp_err_t encode_step_mv(uint16_t millivolts, uint16_t base_mv,
                                uint16_t step_mv, uint8_t max_code, uint8_t *code)
{
    if (millivolts < base_mv) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint16_t delta = millivolts - base_mv;
    if (delta % step_mv != 0) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint16_t encoded = delta / step_mv;
    if (encoded > max_code) {
        return ESP_ERR_INVALID_ARG;
    }
    *code = (uint8_t)encoded;
    return ESP_OK;
}

esp_err_t tg28_sw_set_charge_enable(tg28_sw_handle_t handle, bool enable)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid handle");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = update_bits(handle, TG28_SW_REG_MODULE_ENABLE,
                                        TG28_SW_CHARGER_ENABLE_MASK,
                                        enable ? TG28_SW_CHARGER_ENABLE_MASK : 0);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_charge_enable(tg28_sw_handle_t handle, bool *enabled)
{
    ESP_RETURN_ON_FALSE(handle != NULL && enabled != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = get_reg_bit_locked(handle, TG28_SW_REG_MODULE_ENABLE,
                            TG28_SW_CHARGER_ENABLE_MASK, enabled);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_set_watchdog(tg28_sw_handle_t handle,
                               const tg28_sw_watchdog_config_t *config)
{
    ESP_RETURN_ON_FALSE(handle != NULL && config != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(config->period >= TG28_SW_WATCHDOG_PERIOD_1S &&
                        config->period <= TG28_SW_WATCHDOG_PERIOD_128S &&
                        config->action >= TG28_SW_WATCHDOG_ACTION_IRQ_ONLY &&
                        config->action <= TG28_SW_WATCHDOG_ACTION_IRQ_RESET_PWRCYCLE,
                        ESP_ERR_INVALID_ARG, TAG, "invalid watchdog configuration");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    /* REG19 bits5:4 action, bits2:0 period (datasheet 6.13.2.15). */
    esp_err_t error = update_bits(handle, TG28_SW_REG_WATCHDOG,
                                  TG28_SW_WATCHDOG_ACTION_MASK | TG28_SW_WATCHDOG_PERIOD_MASK,
                                  ((uint8_t)config->action << 4) | (uint8_t)config->period);
    if (error == ESP_OK) {
        /* REG18 bit0 watchdog module enable (datasheet 6.13.2.14). */
        error = update_bits(handle, TG28_SW_REG_MODULE_ENABLE,
                            TG28_SW_WATCHDOG_ENABLE_MASK,
                            config->enable ? TG28_SW_WATCHDOG_ENABLE_MASK : 0);
    }
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_watchdog(tg28_sw_handle_t handle,
                               tg28_sw_watchdog_config_t *config)
{
    ESP_RETURN_ON_FALSE(handle != NULL && config != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t control = 0;
    esp_err_t error = read_registers(handle, TG28_SW_REG_WATCHDOG, &control, sizeof(control));
    if (error == ESP_OK) {
        config->action = (tg28_sw_watchdog_action_t)((control & TG28_SW_WATCHDOG_ACTION_MASK) >> 4);
        config->period = (tg28_sw_watchdog_period_t)(control & TG28_SW_WATCHDOG_PERIOD_MASK);
        error = get_reg_bit_locked(handle, TG28_SW_REG_MODULE_ENABLE,
                                   TG28_SW_WATCHDOG_ENABLE_MASK, &config->enable);
    }
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_feed_watchdog(tg28_sw_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid handle");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    /* REG19 bit3 is the RWAC clear signal (datasheet 6.13.2.15). */
    const esp_err_t error = update_bits(handle, TG28_SW_REG_WATCHDOG,
                                        TG28_SW_WATCHDOG_CLEAR_MASK,
                                        TG28_SW_WATCHDOG_CLEAR_MASK);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_set_gauge_enable(tg28_sw_handle_t handle, bool enable)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid handle");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = update_bits(handle, TG28_SW_REG_MODULE_ENABLE,
                                        TG28_SW_GAUGE_ENABLE_MASK,
                                        enable ? TG28_SW_GAUGE_ENABLE_MASK : 0);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_gauge_enable(tg28_sw_handle_t handle, bool *enabled)
{
    ESP_RETURN_ON_FALSE(handle != NULL && enabled != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = get_reg_bit_locked(handle, TG28_SW_REG_MODULE_ENABLE,
                            TG28_SW_GAUGE_ENABLE_MASK, enabled);
    unlock_device(handle);
    return error;
}

/* TS thresholds (datasheet 6.13.2.48-53): REG52 hyst L2N 16mV/step,
 * REG53 hyst H2L 4mV/step, REG54/56 VLTF 32mV/step, REG55/57 VHTF 2mV/step. */
esp_err_t tg28_sw_set_ts_thresholds(tg28_sw_handle_t handle,
                                    const tg28_sw_ts_thresholds_t *thresholds)
{
    ESP_RETURN_ON_FALSE(handle != NULL && thresholds != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    uint8_t hyst_low = 0, hyst_high = 0, vltf_chg = 0, vhtf_chg = 0, vltf_work = 0, vhtf_work = 0;
    ESP_RETURN_ON_ERROR(encode_step_mv(thresholds->hyst_low_to_normal_mv, 0, 16, 0xFF, &hyst_low),
                        TAG, "invalid low-temperature hysteresis");
    ESP_RETURN_ON_ERROR(encode_step_mv(thresholds->hyst_high_to_normal_mv, 0, 4, 0xFF, &hyst_high),
                        TAG, "invalid high-temperature hysteresis");
    ESP_RETURN_ON_ERROR(encode_step_mv(thresholds->vltf_charge_mv, 0, 32, 0xFF, &vltf_chg),
                        TAG, "invalid VLTF charge threshold");
    ESP_RETURN_ON_ERROR(encode_step_mv(thresholds->vhtf_charge_mv, 0, 2, 0xFF, &vhtf_chg),
                        TAG, "invalid VHTF charge threshold");
    ESP_RETURN_ON_ERROR(encode_step_mv(thresholds->vltf_work_mv, 0, 32, 0xFF, &vltf_work),
                        TAG, "invalid VLTF work threshold");
    ESP_RETURN_ON_ERROR(encode_step_mv(thresholds->vhtf_work_mv, 0, 2, 0xFF, &vhtf_work),
                        TAG, "invalid VHTF work threshold");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const uint8_t images[6] = {hyst_low, hyst_high, vltf_chg, vhtf_chg, vltf_work, vhtf_work};
    esp_err_t error = ESP_OK;
    for (int i = 0; i < 6 && error == ESP_OK; i++) {
        error = write_registers(handle, TG28_SW_REG_TS_HYST_LOW + i, &images[i], 1);
    }
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_ts_thresholds(tg28_sw_handle_t handle,
                                    tg28_sw_ts_thresholds_t *thresholds)
{
    ESP_RETURN_ON_FALSE(handle != NULL && thresholds != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t images[6] = {0};
    const esp_err_t error = read_registers(handle, TG28_SW_REG_TS_HYST_LOW, images, sizeof(images));
    unlock_device(handle);
    ESP_RETURN_ON_ERROR(error, TAG, "TS threshold read failed");
    thresholds->hyst_low_to_normal_mv = (uint16_t)images[0] * 16;
    thresholds->hyst_high_to_normal_mv = (uint16_t)images[1] * 4;
    thresholds->vltf_charge_mv = (uint16_t)images[2] * 32;
    thresholds->vhtf_charge_mv = (uint16_t)images[3] * 2;
    thresholds->vltf_work_mv = (uint16_t)images[4] * 32;
    thresholds->vhtf_work_mv = (uint16_t)images[5] * 2;
    return ESP_OK;
}

/* JEITA (datasheet 6.13.2.54-57): REG58 bit0 enable; REG59 bit6 warm current
 * half, bit4 cool current half, bits3:2 warm CV gears, bits1:0 cool CV gears;
 * REG5A cool(T2) 16mV/step; REG5B warm(T3) 8mV/step. */
esp_err_t tg28_sw_set_jeita(tg28_sw_handle_t handle,
                            const tg28_sw_jeita_config_t *config)
{
    ESP_RETURN_ON_FALSE(handle != NULL && config != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(config->cool_cv_adjust >= TG28_SW_JEITA_CV_ADJUST_NONE &&
                        config->cool_cv_adjust <= TG28_SW_JEITA_CV_ADJUST_TWO_GEARS &&
                        config->warm_cv_adjust >= TG28_SW_JEITA_CV_ADJUST_NONE &&
                        config->warm_cv_adjust <= TG28_SW_JEITA_CV_ADJUST_TWO_GEARS,
                        ESP_ERR_INVALID_ARG, TAG, "invalid CV adjustment");
    uint8_t cool = 0, warm = 0;
    ESP_RETURN_ON_ERROR(encode_step_mv(config->cool_mv, 0, 16, 0xFF, &cool),
                        TAG, "invalid cool boundary");
    ESP_RETURN_ON_ERROR(encode_step_mv(config->warm_mv, 0, 8, 0xFF, &warm),
                        TAG, "invalid warm boundary");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    esp_err_t error = write_registers(handle, TG28_SW_REG_JEITA_COOL, &cool, 1);
    if (error == ESP_OK) {
        error = write_registers(handle, TG28_SW_REG_JEITA_WARM, &warm, 1);
    }
    if (error == ESP_OK) {
        const uint8_t cv_image = (config->warm_current_half ? (1U << 6) : 0) |
                                 (config->cool_current_half ? (1U << 4) : 0) |
                                 ((uint8_t)config->warm_cv_adjust << 2) |
                                 (uint8_t)config->cool_cv_adjust;
        error = write_registers(handle, TG28_SW_REG_JEITA_CV, &cv_image, 1);
    }
    if (error == ESP_OK) {
        error = update_bits(handle, TG28_SW_REG_JEITA_ENABLE, 0x01,
                            config->enable ? 0x01 : 0);
    }
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_jeita(tg28_sw_handle_t handle,
                            tg28_sw_jeita_config_t *config)
{
    ESP_RETURN_ON_FALSE(handle != NULL && config != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t cool = 0, warm = 0, cv = 0, enable_reg = 0;
    esp_err_t error = read_registers(handle, TG28_SW_REG_JEITA_COOL, &cool, 1);
    if (error == ESP_OK) {
        error = read_registers(handle, TG28_SW_REG_JEITA_WARM, &warm, 1);
    }
    if (error == ESP_OK) {
        error = read_registers(handle, TG28_SW_REG_JEITA_CV, &cv, 1);
    }
    if (error == ESP_OK) {
        error = read_registers(handle, TG28_SW_REG_JEITA_ENABLE, &enable_reg, 1);
    }
    unlock_device(handle);
    ESP_RETURN_ON_ERROR(error, TAG, "JEITA register read failed");
    config->cool_mv = (uint16_t)cool * 16;
    config->warm_mv = (uint16_t)warm * 8;
    config->warm_current_half = (cv & (1U << 6)) != 0;
    config->cool_current_half = (cv & (1U << 4)) != 0;
    config->warm_cv_adjust = (tg28_sw_jeita_cv_adjust_t)((cv >> 2) & 0x03);
    config->cool_cv_adjust = (tg28_sw_jeita_cv_adjust_t)(cv & 0x03);
    config->enable = (enable_reg & 0x01) != 0;
    /* A reserved CV code (11b) reads back as two gears, the strongest
     * defined adjustment; the setter never produces it. */
    if (config->warm_cv_adjust > TG28_SW_JEITA_CV_ADJUST_TWO_GEARS) {
        config->warm_cv_adjust = TG28_SW_JEITA_CV_ADJUST_TWO_GEARS;
    }
    if (config->cool_cv_adjust > TG28_SW_JEITA_CV_ADJUST_TWO_GEARS) {
        config->cool_cv_adjust = TG28_SW_JEITA_CV_ADJUST_TWO_GEARS;
    }
    return ESP_OK;
}

/* ts_cfg_data (datasheet 6.13.2.58-59): 14-bit TS voltage code, 0.5 mV/LSB
 * like the TS ADC channel. REG5C bits5:0 hold code bits 13:8; REG5D holds
 * bits 7:0. The per-register table marks REG5D RO, but the 6.13.1 register
 * list marks it RW and the description says "configured by MCU"; the RW
 * treatment follows the register list. */
esp_err_t tg28_sw_set_ts_fixed_threshold(tg28_sw_handle_t handle, uint16_t raw_code)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid handle");
    ESP_RETURN_ON_FALSE(raw_code <= 0x3FFF, ESP_ERR_INVALID_ARG, TAG,
                        "ts_cfg_data is a 14-bit code");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const uint8_t high = (uint8_t)((raw_code >> 8) & TG28_SW_TS_FIXED_H_MASK);
    esp_err_t error = update_bits(handle, TG28_SW_REG_TS_FIXED_H,
                                  TG28_SW_TS_FIXED_H_MASK, high);
    if (error == ESP_OK) {
        const uint8_t low = (uint8_t)(raw_code & 0xFF);
        error = write_registers(handle, TG28_SW_REG_TS_FIXED_L, &low, 1);
    }
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_ts_fixed_threshold(tg28_sw_handle_t handle, uint16_t *raw_code)
{
    ESP_RETURN_ON_FALSE(handle != NULL && raw_code != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t high = 0, low = 0;
    esp_err_t error = read_registers(handle, TG28_SW_REG_TS_FIXED_H, &high, 1);
    if (error == ESP_OK) {
        error = read_registers(handle, TG28_SW_REG_TS_FIXED_L, &low, 1);
    }
    unlock_device(handle);
    ESP_RETURN_ON_ERROR(error, TAG, "ts_cfg_data read failed");
    *raw_code = ((uint16_t)(high & TG28_SW_TS_FIXED_H_MASK) << 8) | low;
    return ESP_OK;
}

esp_err_t tg28_sw_set_thermal_regulation(tg28_sw_handle_t handle,
        tg28_sw_thermal_regulation_t threshold)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid handle");
    ESP_RETURN_ON_FALSE(threshold >= TG28_SW_THERMAL_REGULATION_60C &&
                        threshold <= TG28_SW_THERMAL_REGULATION_120C,
                        ESP_ERR_INVALID_ARG, TAG, "invalid threshold");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = update_bits(handle, TG28_SW_REG_THERMAL_REGULATION,
                                        TG28_SW_THERMAL_REGULATION_MASK, (uint8_t)threshold);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_thermal_regulation(tg28_sw_handle_t handle,
        tg28_sw_thermal_regulation_t *threshold)
{
    ESP_RETURN_ON_FALSE(handle != NULL && threshold != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t value = 0;
    const esp_err_t error = read_registers(handle, TG28_SW_REG_THERMAL_REGULATION,
                                           &value, sizeof(value));
    unlock_device(handle);
    ESP_RETURN_ON_ERROR(error, TAG, "thermal regulation read failed");
    *threshold = (tg28_sw_thermal_regulation_t)(value & TG28_SW_THERMAL_REGULATION_MASK);
    return ESP_OK;
}

/* Charger safety timers (datasheet 6.13.2.65): REG67 bit7 timer slow in
 * DPM/thermal, bit6 charge-done timer enable, bits5:4 charge timer,
 * bit2 pre-charge timer enable, bits1:0 pre-charge timer. */
esp_err_t tg28_sw_set_charge_timers(tg28_sw_handle_t handle,
                                    const tg28_sw_charge_timers_t *timers)
{
    ESP_RETURN_ON_FALSE(handle != NULL && timers != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(timers->precharge_timer >= TG28_SW_PRECHARGE_TIMER_40MIN &&
                        timers->precharge_timer <= TG28_SW_PRECHARGE_TIMER_70MIN &&
                        timers->charge_timer >= TG28_SW_CHARGE_TIMER_5H &&
                        timers->charge_timer <= TG28_SW_CHARGE_TIMER_20H,
                        ESP_ERR_INVALID_ARG, TAG, "invalid timer selection");
    const uint8_t image = (timers->timer_slow_during_dpm ? TG28_SW_TIMER_SLOW_DPM_MASK : 0) |
                          (timers->charge_timer_enable ? TG28_SW_CHARGE_TIMER_ENABLE_MASK : 0) |
                          ((uint8_t)timers->charge_timer << 4) |
                          (timers->precharge_timer_enable ? TG28_SW_PRECHARGE_TIMER_ENABLE_MASK : 0) |
                          (uint8_t)timers->precharge_timer;
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = update_bits(handle, TG28_SW_REG_CHARGE_TIMERS,
                                        TG28_SW_TIMER_SLOW_DPM_MASK |
                                        TG28_SW_CHARGE_TIMER_ENABLE_MASK |
                                        TG28_SW_CHARGE_TIMER_MASK |
                                        TG28_SW_PRECHARGE_TIMER_ENABLE_MASK |
                                        TG28_SW_PRECHARGE_TIMER_MASK,
                                        image);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_charge_timers(tg28_sw_handle_t handle,
                                    tg28_sw_charge_timers_t *timers)
{
    ESP_RETURN_ON_FALSE(handle != NULL && timers != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t value = 0;
    const esp_err_t error = read_registers(handle, TG28_SW_REG_CHARGE_TIMERS,
                                           &value, sizeof(value));
    unlock_device(handle);
    ESP_RETURN_ON_ERROR(error, TAG, "charge timers read failed");
    timers->timer_slow_during_dpm = (value & TG28_SW_TIMER_SLOW_DPM_MASK) != 0;
    timers->charge_timer_enable = (value & TG28_SW_CHARGE_TIMER_ENABLE_MASK) != 0;
    timers->charge_timer = (tg28_sw_charge_timer_t)((value & TG28_SW_CHARGE_TIMER_MASK) >> 4);
    timers->precharge_timer_enable = (value & TG28_SW_PRECHARGE_TIMER_ENABLE_MASK) != 0;
    timers->precharge_timer = (tg28_sw_precharge_timer_t)(value & TG28_SW_PRECHARGE_TIMER_MASK);
    return ESP_OK;
}

esp_err_t tg28_sw_set_battery_detect_enable(tg28_sw_handle_t handle, bool enable)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid handle");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = update_bits(handle, TG28_SW_REG_BATTERY_DETECT,
                                        TG28_SW_BATTERY_DETECT_MASK,
                                        enable ? TG28_SW_BATTERY_DETECT_MASK : 0);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_battery_detect_enable(tg28_sw_handle_t handle, bool *enabled)
{
    ESP_RETURN_ON_FALSE(handle != NULL && enabled != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = get_reg_bit_locked(handle, TG28_SW_REG_BATTERY_DETECT,
                            TG28_SW_BATTERY_DETECT_MASK, enabled);
    unlock_device(handle);
    return error;
}

/* CHGLED (datasheet 6.13.2.67): REG69 bits5:4 manual drive, bits2:1 display
 * mode, bit0 pin enable. */
esp_err_t tg28_sw_set_charge_led(tg28_sw_handle_t handle,
                                 const tg28_sw_charge_led_config_t *config)
{
    ESP_RETURN_ON_FALSE(handle != NULL && config != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(config->mode >= TG28_SW_CHARGE_LED_TYPE_A &&
                        config->mode <= TG28_SW_CHARGE_LED_MANUAL &&
                        config->manual_mode >= TG28_SW_CHARGE_LED_MANUAL_HIZ &&
                        config->manual_mode <= TG28_SW_CHARGE_LED_MANUAL_LOW,
                        ESP_ERR_INVALID_ARG, TAG, "invalid CHGLED configuration");
    const uint8_t image = ((uint8_t)config->manual_mode << 4) |
                          ((uint8_t)config->mode << 1) |
                          (config->enable ? TG28_SW_CHARGE_LED_ENABLE_MASK : 0);
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = update_bits(handle, TG28_SW_REG_CHARGE_LED,
                                        TG28_SW_CHARGE_LED_MANUAL_MASK |
                                        TG28_SW_CHARGE_LED_MODE_MASK |
                                        TG28_SW_CHARGE_LED_ENABLE_MASK,
                                        image);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_charge_led(tg28_sw_handle_t handle,
                                 tg28_sw_charge_led_config_t *config)
{
    ESP_RETURN_ON_FALSE(handle != NULL && config != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t value = 0;
    const esp_err_t error = read_registers(handle, TG28_SW_REG_CHARGE_LED, &value, sizeof(value));
    unlock_device(handle);
    ESP_RETURN_ON_ERROR(error, TAG, "CHGLED read failed");
    config->manual_mode = (tg28_sw_charge_led_manual_mode_t)((value & TG28_SW_CHARGE_LED_MANUAL_MASK) >> 4);
    config->mode = (tg28_sw_charge_led_mode_t)((value & TG28_SW_CHARGE_LED_MODE_MASK) >> 1);
    config->enable = (value & TG28_SW_CHARGE_LED_ENABLE_MASK) != 0;
    return ESP_OK;
}

/* Backup/button battery charging (datasheet 6.13.2.68 and 6.12.2): REG18
 * bit2 enable, REG6A bits2:0 termination 2.6-3.3V in 100mV steps. */
esp_err_t tg28_sw_set_backup_charge(tg28_sw_handle_t handle, bool enable,
                                    uint16_t termination_mv)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid handle");
    uint8_t code = 0;
    ESP_RETURN_ON_ERROR(encode_step_mv(termination_mv, TG28_SW_VOFF_VOLTAGE_BASE_MV,
                                       TG28_SW_VOFF_VOLTAGE_STEP_MV,
                                       TG28_SW_BACKUP_VOLTAGE_MASK, &code),
                        TAG, "invalid backup termination voltage");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    esp_err_t error = update_bits(handle, TG28_SW_REG_BACKUP_CHARGE,
                                  TG28_SW_BACKUP_VOLTAGE_MASK, code);
    if (error == ESP_OK) {
        error = update_bits(handle, TG28_SW_REG_MODULE_ENABLE,
                            TG28_SW_BACKUP_CHARGE_ENABLE_MASK,
                            enable ? TG28_SW_BACKUP_CHARGE_ENABLE_MASK : 0);
    }
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_backup_charge(tg28_sw_handle_t handle, bool *enabled,
                                    uint16_t *termination_mv)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid handle");
    ESP_RETURN_ON_FALSE(enabled != NULL || termination_mv != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "no output requested");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    esp_err_t error = ESP_OK;
    if (termination_mv != NULL) {
        uint8_t value = 0;
        error = read_registers(handle, TG28_SW_REG_BACKUP_CHARGE, &value, sizeof(value));
        if (error == ESP_OK) {
            *termination_mv = TG28_SW_VOFF_VOLTAGE_BASE_MV +
                              (value & TG28_SW_BACKUP_VOLTAGE_MASK) * TG28_SW_VOFF_VOLTAGE_STEP_MV;
        }
    }
    if (error == ESP_OK && enabled != NULL) {
        error = get_reg_bit_locked(handle, TG28_SW_REG_MODULE_ENABLE,
                                   TG28_SW_BACKUP_CHARGE_ENABLE_MASK, enabled);
    }
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_set_min_sys_voltage(tg28_sw_handle_t handle, uint16_t millivolts)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid handle");
    uint8_t code = 0;
    ESP_RETURN_ON_ERROR(encode_step_mv(millivolts, TG28_SW_MIN_SYS_VOLTAGE_BASE_MV,
                                       TG28_SW_MIN_SYS_VOLTAGE_STEP_MV,
                                       TG28_SW_MIN_SYS_VOLTAGE_MASK, &code),
                        TAG, "invalid minimum system voltage");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = update_bits(handle, TG28_SW_REG_MIN_SYS_VOLTAGE,
                                        TG28_SW_MIN_SYS_VOLTAGE_MASK, code);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_min_sys_voltage(tg28_sw_handle_t handle, uint16_t *millivolts)
{
    ESP_RETURN_ON_FALSE(handle != NULL && millivolts != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t value = 0;
    const esp_err_t error = read_registers(handle, TG28_SW_REG_MIN_SYS_VOLTAGE,
                                           &value, sizeof(value));
    unlock_device(handle);
    ESP_RETURN_ON_ERROR(error, TAG, "minimum system voltage read failed");
    *millivolts = TG28_SW_MIN_SYS_VOLTAGE_BASE_MV +
                  (value & TG28_SW_MIN_SYS_VOLTAGE_MASK) * TG28_SW_MIN_SYS_VOLTAGE_STEP_MV;
    return ESP_OK;
}

/* Power-off policy (datasheet 6.13.2.20-22,25). REG23 bit4 is reserved with
 * a POR-1 value; the read-modify-write below preserves it. */
esp_err_t tg28_sw_set_poweroff_config(tg28_sw_handle_t handle,
                                      const tg28_sw_poweroff_config_t *config)
{
    ESP_RETURN_ON_FALSE(handle != NULL && config != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(config->dcdc_uvp_shutdown_mask <= 0x0F,
                        ESP_ERR_INVALID_ARG, TAG, "invalid UVP mask");
    uint8_t levels = 0, voff = 0;
    ESP_RETURN_ON_ERROR(tg28_sw_encode_powerkey_levels(config->irqlevel_ms,
                        config->offlevel_ms,
                        config->onlevel_ms, &levels),
                        TAG, "invalid power-key timing window");
    ESP_RETURN_ON_ERROR(encode_step_mv(config->voff_mv, TG28_SW_VOFF_VOLTAGE_BASE_MV,
                                       TG28_SW_VOFF_VOLTAGE_STEP_MV,
                                       TG28_SW_VOFF_VOLTAGE_MASK, &voff),
                        TAG, "invalid VOFF threshold");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    /* Order matters: the immediate power-off sources are cleared first and
     * enabled again only after the new thresholds land. Writing the
     * PWRON>OFFLEVEL enable before REG27 could shut the board down under
     * the old window while the key happens to be pressed. A failure in the
     * middle leaves the sources disabled - the safe direction - and the
     * caller sees the error. */
    esp_err_t error = update_bits(handle, TG28_SW_REG_PWROFF_EN,
                                  TG28_SW_PWROFF_DIE_OT2_MASK |
                                  TG28_SW_PWROFF_OFFLEVEL_MASK |
                                  TG28_SW_PWROFF_BUTTON_RESTART_MASK,
                                  (config->button_off_as_restart ? TG28_SW_PWROFF_BUTTON_RESTART_MASK : 0));
    if (error == ESP_OK) {
        error = update_bits(handle, TG28_SW_REG_DCDC_PWROFF_EN,
                            TG28_SW_DCDC_OVP_PWROFF_MASK | TG28_SW_DCDC_UVP_PWROFF_MASK,
                            (config->dcdc_ovp_shutdown ? TG28_SW_DCDC_OVP_PWROFF_MASK : 0) |
                            (config->dcdc_uvp_shutdown_mask & TG28_SW_DCDC_UVP_PWROFF_MASK));
    }
    if (error == ESP_OK) {
        error = update_bits(handle, TG28_SW_REG_VOFF_THRESHOLD,
                            TG28_SW_VOFF_VOLTAGE_MASK, voff);
    }
    if (error == ESP_OK) {
        error = update_bits(handle, TG28_SW_REG_POWER_KEY_LEVELS,
                            TG28_SW_IRQLEVEL_MASK | TG28_SW_OFFLEVEL_MASK | TG28_SW_ONLEVEL_MASK,
                            levels);
    }
    if (error == ESP_OK) {
        /* Enable the requested power-off sources only now that every
         * threshold they act on holds its new value. */
        error = update_bits(handle, TG28_SW_REG_PWROFF_EN,
                            TG28_SW_PWROFF_DIE_OT2_MASK | TG28_SW_PWROFF_OFFLEVEL_MASK,
                            (config->die_ot_level2_off_enable ? TG28_SW_PWROFF_DIE_OT2_MASK : 0) |
                            (config->pwron_offlevel_off_enable ? TG28_SW_PWROFF_OFFLEVEL_MASK : 0));
    }
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_poweroff_config(tg28_sw_handle_t handle,
                                      tg28_sw_poweroff_config_t *config)
{
    ESP_RETURN_ON_FALSE(handle != NULL && config != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t pwroff_en = 0, dcdc_pwroff = 0, voff_reg = 0, levels = 0;
    esp_err_t error = read_registers(handle, TG28_SW_REG_PWROFF_EN, &pwroff_en, 1);
    if (error == ESP_OK) {
        error = read_registers(handle, TG28_SW_REG_DCDC_PWROFF_EN, &dcdc_pwroff, 1);
    }
    if (error == ESP_OK) {
        error = read_registers(handle, TG28_SW_REG_VOFF_THRESHOLD, &voff_reg, 1);
    }
    if (error == ESP_OK) {
        error = read_registers(handle, TG28_SW_REG_POWER_KEY_LEVELS, &levels, 1);
    }
    unlock_device(handle);
    ESP_RETURN_ON_ERROR(error, TAG, "power-off policy read failed");
    config->die_ot_level2_off_enable = (pwroff_en & TG28_SW_PWROFF_DIE_OT2_MASK) != 0;
    config->pwron_offlevel_off_enable = (pwroff_en & TG28_SW_PWROFF_OFFLEVEL_MASK) != 0;
    config->button_off_as_restart = (pwroff_en & TG28_SW_PWROFF_BUTTON_RESTART_MASK) != 0;
    config->dcdc_ovp_shutdown = (dcdc_pwroff & TG28_SW_DCDC_OVP_PWROFF_MASK) != 0;
    config->dcdc_uvp_shutdown_mask = dcdc_pwroff & TG28_SW_DCDC_UVP_PWROFF_MASK;
    config->voff_mv = TG28_SW_VOFF_VOLTAGE_BASE_MV +
                      (voff_reg & TG28_SW_VOFF_VOLTAGE_MASK) * TG28_SW_VOFF_VOLTAGE_STEP_MV;
    tg28_sw_decode_powerkey_levels(levels, &config->irqlevel_ms,
                                   &config->offlevel_ms, &config->onlevel_ms);
    return ESP_OK;
}

/* Sleep/wakeup (datasheet 6.13.2.24): REG26 bit4 IRQ wakeup, bit3 PWROK
 * low-level on wakeup, bit2 wakeup voltage select, bit1 wakeup enable
 * (RWLC), bit0 sleep enable (RWLC). */
esp_err_t tg28_sw_set_sleep_config(tg28_sw_handle_t handle,
                                   const tg28_sw_sleep_config_t *config)
{
    ESP_RETURN_ON_FALSE(handle != NULL && config != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    const uint8_t image = (config->irq_wakeup_enable ? TG28_SW_WAKEUP_IRQ_MASK : 0) |
                          (config->pwrok_low_level_wakeup_enable ? TG28_SW_WAKEUP_PWROK_LOW_MASK : 0) |
                          (config->wakeup_voltage_from_before_sleep ? TG28_SW_WAKEUP_VOLTAGE_MASK : 0) |
                          (config->wakeup_enable ? TG28_SW_WAKEUP_ENABLE_MASK : 0) |
                          (config->sleep_enable ? TG28_SW_SLEEP_ENABLE_MASK : 0);
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = update_bits(handle, TG28_SW_REG_SLEEP_WAKEUP,
                                        TG28_SW_WAKEUP_IRQ_MASK |
                                        TG28_SW_WAKEUP_PWROK_LOW_MASK |
                                        TG28_SW_WAKEUP_VOLTAGE_MASK |
                                        TG28_SW_WAKEUP_ENABLE_MASK |
                                        TG28_SW_SLEEP_ENABLE_MASK,
                                        image);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_sleep_config(tg28_sw_handle_t handle,
                                   tg28_sw_sleep_config_t *config)
{
    ESP_RETURN_ON_FALSE(handle != NULL && config != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t value = 0;
    const esp_err_t error = read_registers(handle, TG28_SW_REG_SLEEP_WAKEUP,
                                           &value, sizeof(value));
    unlock_device(handle);
    ESP_RETURN_ON_ERROR(error, TAG, "sleep/wakeup read failed");
    config->irq_wakeup_enable = (value & TG28_SW_WAKEUP_IRQ_MASK) != 0;
    config->pwrok_low_level_wakeup_enable = (value & TG28_SW_WAKEUP_PWROK_LOW_MASK) != 0;
    config->wakeup_voltage_from_before_sleep = (value & TG28_SW_WAKEUP_VOLTAGE_MASK) != 0;
    config->wakeup_enable = (value & TG28_SW_WAKEUP_ENABLE_MASK) != 0;
    config->sleep_enable = (value & TG28_SW_SLEEP_ENABLE_MASK) != 0;
    return ESP_OK;
}

esp_err_t tg28_sw_set_gpio1_config(tg28_sw_handle_t handle,
                                   tg28_sw_gpio1_output_t output)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid handle");
    ESP_RETURN_ON_FALSE(output >= TG28_SW_GPIO1_OUTPUT_HIZ &&
                        output <= TG28_SW_GPIO1_OUTPUT_LOW,
                        ESP_ERR_INVALID_ARG, TAG, "reserved GPIO1 output code");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = update_bits(handle, TG28_SW_REG_GPIO1_CONFIG,
                                        TG28_SW_GPIO1_OUTPUT_MASK,
                                        (uint8_t)output << 2);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_gpio1_config(tg28_sw_handle_t handle,
                                   tg28_sw_gpio1_output_t *output)
{
    ESP_RETURN_ON_FALSE(handle != NULL && output != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t value = 0;
    const esp_err_t error = read_registers(handle, TG28_SW_REG_GPIO1_CONFIG,
                                           &value, sizeof(value));
    unlock_device(handle);
    ESP_RETURN_ON_ERROR(error, TAG, "GPIO1 config read failed");
    const uint8_t code = (value & TG28_SW_GPIO1_OUTPUT_MASK) >> 2;
    /* Codes 10b/11b are reserved; report them as Hi-z, the safe state. */
    *output = code <= TG28_SW_GPIO1_OUTPUT_LOW ? (tg28_sw_gpio1_output_t)code
              : TG28_SW_GPIO1_OUTPUT_HIZ;
    return ESP_OK;
}

/* DCDC modes (datasheet 6.13.2.69-70): REG80 bit6 force CCM, bit5 DVM ramp;
 * REG81 bit7 spread enable, bit6 spread range, bits5:2 force-PWM per rail
 * (bit2 = DCDC1 ... bit5 = DCDC4). */
esp_err_t tg28_sw_set_dcdc_mode(tg28_sw_handle_t handle,
                                const tg28_sw_dcdc_mode_t *mode)
{
    ESP_RETURN_ON_FALSE(handle != NULL && mode != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    esp_err_t error = update_bits(handle, TG28_SW_REG_DCDC_ENABLE,
                                  TG28_SW_DCDC_FORCE_CCM_MASK | TG28_SW_DCDC_DVM_RAMP_MASK,
                                  (mode->force_ccm ? TG28_SW_DCDC_FORCE_CCM_MASK : 0) |
                                  (mode->dvm_ramp_slow ? TG28_SW_DCDC_DVM_RAMP_MASK : 0));
    if (error == ESP_OK) {
        uint8_t pwm_bits = 0;
        for (int rail = 0; rail < 4; rail++) {
            if (mode->force_pwm[rail]) {
                pwm_bits |= (uint8_t)(1U << (2 + rail));
            }
        }
        error = update_bits(handle, TG28_SW_REG_DCDC_FORCE_PWM,
                            TG28_SW_DCDC_SPREAD_ENABLE_MASK |
                            TG28_SW_DCDC_SPREAD_RANGE_MASK |
                            TG28_SW_DCDC_FORCE_PWM_MASK,
                            (mode->spread_spectrum_enable ? TG28_SW_DCDC_SPREAD_ENABLE_MASK : 0) |
                            (mode->spread_range_100khz ? TG28_SW_DCDC_SPREAD_RANGE_MASK : 0) |
                            pwm_bits);
    }
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_dcdc_mode(tg28_sw_handle_t handle,
                                tg28_sw_dcdc_mode_t *mode)
{
    ESP_RETURN_ON_FALSE(handle != NULL && mode != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    uint8_t reg80 = 0, reg81 = 0;
    esp_err_t error = read_registers(handle, TG28_SW_REG_DCDC_ENABLE, &reg80, 1);
    if (error == ESP_OK) {
        error = read_registers(handle, TG28_SW_REG_DCDC_FORCE_PWM, &reg81, 1);
    }
    unlock_device(handle);
    ESP_RETURN_ON_ERROR(error, TAG, "DCDC mode read failed");
    mode->force_ccm = (reg80 & TG28_SW_DCDC_FORCE_CCM_MASK) != 0;
    mode->dvm_ramp_slow = (reg80 & TG28_SW_DCDC_DVM_RAMP_MASK) != 0;
    mode->spread_spectrum_enable = (reg81 & TG28_SW_DCDC_SPREAD_ENABLE_MASK) != 0;
    mode->spread_range_100khz = (reg81 & TG28_SW_DCDC_SPREAD_RANGE_MASK) != 0;
    for (int rail = 0; rail < 4; rail++) {
        mode->force_pwm[rail] = (reg81 & (1U << (2 + rail))) != 0;
    }
    return ESP_OK;
}

esp_err_t tg28_sw_soft_reset(tg28_sw_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid handle");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    /* REG10 bit1 is an RWAC command bit (datasheet 6.13.2.7): once the write
     * is acknowledged the whole PMU restarts, every rail cycles, and the
     * system registers reset to defaults, so the call often does not
     * return. The lock is still released unconditionally: the board can
     * survive the reset, and a held mutex would deadlock every later TG28
     * call. */
    const esp_err_t error = update_bits(handle, TG28_SW_REG_COMMON_CONFIG,
                                        TG28_SW_SOFTWARE_RESET_MASK,
                                        TG28_SW_SOFTWARE_RESET_MASK);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_set_batfet_off_state_enable(tg28_sw_handle_t handle, bool enable)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid handle");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = update_bits(handle, TG28_SW_REG_BATFET_CONTROL,
                                        TG28_SW_BATFET_OFF_ENABLE_MASK,
                                        enable ? TG28_SW_BATFET_OFF_ENABLE_MASK : 0);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_get_batfet_off_state_enable(tg28_sw_handle_t handle, bool *enabled)
{
    ESP_RETURN_ON_FALSE(handle != NULL && enabled != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = get_reg_bit_locked(handle, TG28_SW_REG_BATFET_CONTROL,
                            TG28_SW_BATFET_OFF_ENABLE_MASK, enabled);
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_write_register(tg28_sw_handle_t handle, uint8_t register_address,
                                 uint8_t value)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid handle");
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const esp_err_t error = write_registers(handle, register_address, &value, sizeof(value));
    unlock_device(handle);
    return error;
}

esp_err_t tg28_sw_read_battery_model_io(const tg28_sw_register_io_t *io,
                                        uint8_t *model, size_t size)
{
    ESP_RETURN_ON_FALSE(io != NULL && io->read != NULL && io->write != NULL,
                        ESP_ERR_INVALID_ARG, TAG,
                        "invalid register transport");
    ESP_RETURN_ON_FALSE(model != NULL, ESP_ERR_INVALID_ARG, TAG, "model buffer is NULL");
    ESP_RETURN_ON_FALSE(size == TG28_SW_BATTERY_MODEL_SIZE, ESP_ERR_INVALID_SIZE,
                        TAG, "battery model must be exactly %u bytes", TG28_SW_BATTERY_MODEL_SIZE);
    uint8_t gauge_control = 0;
    esp_err_t error = io->read(io->context, TG28_SW_REG_FUEL_GAUGE_CONTROL,
                               &gauge_control, sizeof(gauge_control));
    if (error != ESP_OK) {
        return error;
    }
    ESP_RETURN_ON_FALSE((gauge_control & TG28_SW_BROM_WRITER_ENABLE_MASK) == 0,
                        ESP_ERR_INVALID_STATE, TAG,
                        "BROM writer was enabled before model read");

    uint8_t module_enable = 0;
    error = io->read(io->context, TG28_SW_REG_MODULE_ENABLE,
                     &module_enable, sizeof(module_enable));
    if (error != ESP_OK) {
        return error;
    }

    /* A disabled gauge returns a repeated byte from REGA1 on real TG28
     * hardware. Temporarily enable it and pause the watchdog, preserving the
     * charger and every other module bit. */
    const uint8_t modules_prepared =
        (module_enable | TG28_SW_GAUGE_ENABLE_MASK) &
        ~TG28_SW_WATCHDOG_ENABLE_MASK;
    const bool modules_touched = modules_prepared != module_enable;
    bool gauge_control_touched = false;
    bool reset_released = true;
    if (modules_touched) {
        error = io_write_register_verified(io, TG28_SW_REG_MODULE_ENABLE,
                                           modules_prepared,
                                           TG28_SW_MODULE_ENABLE_WRITE_MASK);
    }

    /* REGA2 bit4 chooses the model source used by the gauge MCU after reset;
     * it does not choose a different REGA1 read window. Keep the entry source
     * unchanged, reset the gauge, then perform the documented 0 -> 1 BROM
     * transition before streaming REGA1. */
    if (error == ESP_OK) {
        error = io_reset_gauge_mcu(io, &reset_released);
    }
    if (error == ESP_OK) {
        gauge_control_touched = true;
        error = io_set_gauge_control(io, gauge_control);
    }
    if (error == ESP_OK) {
        error = io_set_gauge_control(io, gauge_control |
                                     TG28_SW_BROM_WRITER_ENABLE_MASK);
    }
    for (size_t i = 0; error == ESP_OK && i < size; ++i) {
        error = io->read(io->context, TG28_SW_REG_BATTERY_MODEL,
                         &model[i], sizeof(model[i]));
    }

    return finish_battery_model_io(io, gauge_control,
                                   gauge_control_touched, module_enable,
                                   modules_touched, reset_released, error);
}

esp_err_t tg28_sw_read_battery_model(tg28_sw_handle_t handle,
                                     uint8_t *model, size_t size)
{
    ESP_RETURN_ON_FALSE(model != NULL, ESP_ERR_INVALID_ARG, TAG, "model buffer is NULL");
    ESP_RETURN_ON_FALSE(size == TG28_SW_BATTERY_MODEL_SIZE, ESP_ERR_INVALID_SIZE,
                        TAG, "battery model must be exactly %u bytes", TG28_SW_BATTERY_MODEL_SIZE);
    ESP_RETURN_ON_ERROR(lock_device(handle), TAG, "device lock failed");
    const tg28_sw_register_io_t io = {
        .context = handle,
        .read = device_io_read,
        .write = device_io_write,
        .delay_ms = device_io_delay_ms,
    };
    const esp_err_t error = tg28_sw_read_battery_model_io(&io, model, size);
    unlock_device(handle);
    return error;
}
