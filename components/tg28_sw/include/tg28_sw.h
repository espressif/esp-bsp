/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Driver for the switch-charger variant of the TG28 PMIC.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TG28_SW_I2C_ADDRESS_DEFAULT  0x34
#define TG28_SW_I2C_CLOCK_HZ         400000

/** Byte size of the REGA1 fuel-gauge battery-model window. The hardware
 * design guide (section 6.2) writes exactly 128 battery parameters to
 * regA1H, one byte per write, in order; datasheet 6.13.2.86. Models must
 * be exactly this size: shorter models leave the gauge with a truncated
 * table, longer ones write past the window. */
#define TG28_SW_BATTERY_MODEL_SIZE   128

/* Power-key interrupt enable bits in IRQ bank 1 (REG41 bits 3:0). They map
 * one-to-one onto TG28_SW_IRQ_PONPE/PONNE/PONLP/PONSP of tg28_sw_irq_t. */
#define TG28_SW_POWER_KEY_IRQ_POSITIVE_EDGE  (1U << 0)
#define TG28_SW_POWER_KEY_IRQ_NEGATIVE_EDGE  (1U << 1)
#define TG28_SW_POWER_KEY_IRQ_LONG_PRESS      (1U << 2)
#define TG28_SW_POWER_KEY_IRQ_SHORT_PRESS     (1U << 3)
#define TG28_SW_POWER_KEY_IRQ_ALL             0x0F

/* Latched power-off source bits of REG21 (datasheet 6.13.2.19), as
 * returned by tg28_sw_get_power_off_source(). */
#define TG28_SW_POWER_OFF_SOURCE_PWRON_MODE  (1U << 0) /**< PWRON low beyond OFFLEVEL in PWRON mode */
#define TG28_SW_POWER_OFF_SOURCE_SOFTWARE    (1U << 1) /**< Software soft-PWROFF command */
#define TG28_SW_POWER_OFF_SOURCE_EN_MODE     (1U << 2) /**< EN held low in EN mode */
#define TG28_SW_POWER_OFF_SOURCE_VSYS_UV     (1U << 3) /**< VSYS under-voltage */
#define TG28_SW_POWER_OFF_SOURCE_VBUS_OV     (1U << 4) /**< VBUS over-voltage */
#define TG28_SW_POWER_OFF_SOURCE_DCDC_UV     (1U << 5) /**< DCDC under-voltage */
#define TG28_SW_POWER_OFF_SOURCE_DCDC_OV     (1U << 6) /**< DCDC over-voltage */
#define TG28_SW_POWER_OFF_SOURCE_DIE_OT      (1U << 7) /**< Die over-temperature */

/** Opaque TG28 switch-charger device handle. */
typedef struct tg28_sw_device_t *tg28_sw_handle_t;

/** TG28 regulators exposed by this driver. */
typedef enum {
    TG28_SW_DCDC1 = 0,
    TG28_SW_DCDC2,
    TG28_SW_DCDC3,
    TG28_SW_DCDC4,
    TG28_SW_ALDO1,
    TG28_SW_ALDO2,
    TG28_SW_ALDO3,
    TG28_SW_ALDO4,
    TG28_SW_BLDO1,
    TG28_SW_BLDO2,
    TG28_SW_CPUSLDO,
    /** DLDO1 pin. Voltage programming applies only when the chip OTP straps
     * the pin as an LDO; on switch-mode (DC1SW) boards the voltage register
     * is inert and tg28_sw_switch_enable() controls the output instead. */
    TG28_SW_DLDO1,
    /** DLDO2 pin; same OTP LDO/switch caveat as TG28_SW_DLDO1 (DC4SW). */
    TG28_SW_DLDO2,
    TG28_SW_REGULATOR_COUNT,
} tg28_sw_regulator_t;

/**
 * Load-switch outputs (DC1SW/DC4SW, datasheet 6.13.2.75-76).
 *
 * The TG28 OTP straps each DLDO pin either as a programmable LDO or as a
 * load switch that passes its input rail straight through (DC1SW input =
 * DCDC1, DC4SW input = DCDC4). No register reports the OTP mode, so the
 * caller must know the board's OTP configuration. The switch enable bit is
 * the same physical bit as the LDO enable bit; the datasheet defines no
 * separate switch control bit.
 */
typedef enum {
    TG28_SW_SWITCH_DC1SW = 0,   /**< DLDO1 pin as a switch, input = DCDC1 */
    TG28_SW_SWITCH_DC4SW,       /**< DLDO2 pin as a switch, input = DCDC4 */
    TG28_SW_SWITCH_COUNT,
} tg28_sw_power_switch_t;

/** Interrupt enable banks (REG40/REG41/REG42, datasheet 6.13.2.41-43). */
typedef enum {
    TG28_SW_IRQ_BANK0 = 0,
    TG28_SW_IRQ_BANK1,
    TG28_SW_IRQ_BANK2,
    TG28_SW_IRQ_BANK_COUNT,
} tg28_sw_irq_bank_t;

/**
 * Named interrupt sources (datasheet 6.13.2.41-46). The numeric value encodes
 * the bank and bit of both the enable registers REG40-REG42 and the status
 * registers REG48-REG4A: bank = irq / 8, bit = irq % 8. Value 21 (REG42
 * bit5) is a reserved read-only bit and intentionally has no symbol; the
 * Linux vendor header names it tg28_IRQ_BOCP, which the datasheet does not
 * support.
 */
typedef enum {
    TG28_SW_IRQ_BWUT = 0,   /**< REG40 bit0: battery under-temperature in work mode */
    TG28_SW_IRQ_BWOT,       /**< REG40 bit1: battery over-temperature in work mode */
    TG28_SW_IRQ_BCUT,       /**< REG40 bit2: battery under-temperature in charge mode */
    TG28_SW_IRQ_BCOT,       /**< REG40 bit3: battery over-temperature in charge mode */
    TG28_SW_IRQ_LOWSOC,     /**< REG40 bit4: gauge new SOC (lowsoc_irq) */
    TG28_SW_IRQ_GWDT,       /**< REG40 bit5: gauge watchdog timeout */
    TG28_SW_IRQ_SOCWL1,     /**< REG40 bit6: SOC dropped to warning level 1 */
    TG28_SW_IRQ_SOCWL2,     /**< REG40 bit7: SOC dropped to warning level 2 */
    TG28_SW_IRQ_PONPE = 8,  /**< REG41 bit0: power-key positive edge */
    TG28_SW_IRQ_PONNE,      /**< REG41 bit1: power-key negative edge */
    TG28_SW_IRQ_PONLP,      /**< REG41 bit2: power-key long press */
    TG28_SW_IRQ_PONSP,      /**< REG41 bit3: power-key short press */
    TG28_SW_IRQ_BREMOVE,    /**< REG41 bit4: battery removed */
    TG28_SW_IRQ_BINSERT,    /**< REG41 bit5: battery inserted */
    TG28_SW_IRQ_VREMOVE,    /**< REG41 bit6: VBUS removed */
    TG28_SW_IRQ_VINSERT,    /**< REG41 bit7: VBUS inserted */
    TG28_SW_IRQ_BOVP = 16,  /**< REG42 bit0: battery over-voltage protection */
    TG28_SW_IRQ_CHGTE,      /**< REG42 bit1: charger safety timer expire */
    TG28_SW_IRQ_DOTL1,      /**< REG42 bit2: die over-temperature level 1 */
    TG28_SW_IRQ_CHGST,      /**< REG42 bit3: charger start */
    TG28_SW_IRQ_CHGDN,      /**< REG42 bit4: battery charge done */
    /* 21 (REG42 bit5) is reserved and has no symbol by design. */
    TG28_SW_IRQ_LDOOC = 22, /**< REG42 bit6: LDO over-current */
    TG28_SW_IRQ_WDEXP,      /**< REG42 bit7: watchdog expire */
    TG28_SW_IRQ_COUNT = 24, /**< Bit-slot count, including the reserved slot */
} tg28_sw_irq_t;

/** TS pin function selection (REG50 bit4, datasheet 6.13.2.47). */
typedef enum {
    /** TS pin senses the battery NTC resistor and affects the charger. */
    TG28_SW_TS_MODE_BATTERY_NTC = 0,
    /** TS pin is a fixed external input and does not affect the charger. */
    TG28_SW_TS_MODE_EXTERNAL_FIXED = 1,
} tg28_sw_ts_mode_t;

/** TS current-source switch (REG50 bits3:2, datasheet 6.13.2.47). */
typedef enum {
    TG28_SW_TS_CURRENT_SOURCE_OFF = 0,
    /** On whenever the TS channel of the ADC is enabled. */
    TG28_SW_TS_CURRENT_SOURCE_ON_WITH_ADC = 1,
    /** On only while the TS channel is converting, off otherwise. */
    TG28_SW_TS_CURRENT_SOURCE_ON_WHILE_CONVERTING = 2,
    TG28_SW_TS_CURRENT_SOURCE_ALWAYS_ON = 3,
} tg28_sw_ts_current_source_t;

/**
 * Channels of the 14-bit SAR ADC (datasheet 6.10, Table 6-7). The TDIE
 * channel reports the die-temperature sensor voltage, not a temperature.
 */
typedef enum {
    TG28_SW_ADC_CHANNEL_VBAT = 0,
    TG28_SW_ADC_CHANNEL_TS,
    TG28_SW_ADC_CHANNEL_VBUS,
    TG28_SW_ADC_CHANNEL_VSYS,
    TG28_SW_ADC_CHANNEL_TDIE,
    TG28_SW_ADC_CHANNEL_COUNT,
} tg28_sw_adc_channel_t;

/** Watchdog timeout behavior on expiry (REG19 bits5:4, datasheet 6.13.2.15). */
typedef enum {
    /** Send the WDEXP IRQ only. */
    TG28_SW_WATCHDOG_ACTION_IRQ_ONLY = 0,
    /** Send the WDEXP IRQ and restart the system. */
    TG28_SW_WATCHDOG_ACTION_IRQ_RESET = 1,
    /** Send the WDEXP IRQ, restart the system, and pull PWROK low for 1 s. */
    TG28_SW_WATCHDOG_ACTION_IRQ_RESET_PWROK = 2,
    /** Send the WDEXP IRQ, restart the system, and power off then re-power
     * the DCDC/LDO rails. */
    TG28_SW_WATCHDOG_ACTION_IRQ_RESET_PWRCYCLE = 3,
} tg28_sw_watchdog_action_t;

/** Watchdog timer period (REG19 bits2:0, datasheet 6.13.2.15). */
typedef enum {
    TG28_SW_WATCHDOG_PERIOD_1S = 0,
    TG28_SW_WATCHDOG_PERIOD_2S,
    TG28_SW_WATCHDOG_PERIOD_4S,
    TG28_SW_WATCHDOG_PERIOD_8S,
    TG28_SW_WATCHDOG_PERIOD_16S,
    TG28_SW_WATCHDOG_PERIOD_32S,
    TG28_SW_WATCHDOG_PERIOD_64S,
    TG28_SW_WATCHDOG_PERIOD_128S,
} tg28_sw_watchdog_period_t;

/** Watchdog configuration (REG18 bit0 enable, REG19, datasheet 6.13.2.14-15). */
typedef struct {
    bool enable;
    tg28_sw_watchdog_period_t period;
    tg28_sw_watchdog_action_t action;
} tg28_sw_watchdog_config_t;

/**
 * Battery temperature-sense thresholds (REG52-REG57, datasheet 6.13.2.48-53;
 * the charger reacts to them per 6.7.4.4). The four limit fields hold the TS
 * pin voltage in millivolts. With the usual NTC driven by a constant
 * current the TS voltage rises as the battery gets colder, so charging
 * pauses above vltf_charge_mv (over-cold) or below vhtf_charge_mv
 * (over-hot), and discharge pauses above vltf_work_mv or below
 * vhtf_work_mv. Their mapping to temperatures follows the NTC network
 * transfer function (default codes place the charge window around 0-45 C
 * with a 10 kOhm NTC driven by 50 uA, datasheet Table 6-5).
 *
 * Encoding: vltf_charge_mv and vltf_work_mv take 32 mV steps (REG54/REG56),
 * vhtf_charge_mv and vhtf_work_mv take 2 mV steps (REG55/REG57);
 * hyst_low_to_normal_mv takes 16 mV steps (REG52) and
 * hyst_high_to_normal_mv takes 4 mV steps (REG53). Any value that is not
 * exactly representable returns ESP_ERR_INVALID_ARG.
 */
typedef struct {
    uint16_t vltf_charge_mv;
    uint16_t vhtf_charge_mv;
    uint16_t vltf_work_mv;
    uint16_t vhtf_work_mv;
    uint16_t hyst_low_to_normal_mv;
    uint16_t hyst_high_to_normal_mv;
} tg28_sw_ts_thresholds_t;

/** JEITA CV adjustment step (REG59 bits3:2/1:0, datasheet 6.13.2.55). */
typedef enum {
    /** Keep the programmed charge voltage. */
    TG28_SW_JEITA_CV_ADJUST_NONE = 0,
    /** One CV gear lower than the programmed charge voltage. */
    TG28_SW_JEITA_CV_ADJUST_ONE_GEAR = 1,
    /** Two CV gears lower than the programmed charge voltage. */
    TG28_SW_JEITA_CV_ADJUST_TWO_GEARS = 2,
} tg28_sw_jeita_cv_adjust_t;

/**
 * JEITA standard configuration (REG58-REG5B, datasheet 6.13.2.54-57).
 *
 * cool_mv (REG5A) is the T2 boundary in 16 mV steps and warm_mv (REG5B) the
 * T3 boundary in 8 mV steps; both are TS pin voltages, so their mapping to
 * temperatures depends on the NTC network. In the cool zone the charge
 * current may be halved and the CV lowered per cool_current_half /
 * cool_cv_adjust; the warm zone applies warm_current_half / warm_cv_adjust.
 * The T1/T4 window limits live in tg28_sw_ts_thresholds_t.
 */
typedef struct {
    bool enable;
    bool cool_current_half;
    tg28_sw_jeita_cv_adjust_t cool_cv_adjust;
    bool warm_current_half;
    tg28_sw_jeita_cv_adjust_t warm_cv_adjust;
    uint16_t cool_mv;
    uint16_t warm_mv;
} tg28_sw_jeita_config_t;

/** Thermal regulation threshold (REG65 bits1:0, datasheet 6.13.2.64). */
typedef enum {
    TG28_SW_THERMAL_REGULATION_60C = 0,
    TG28_SW_THERMAL_REGULATION_80C,
    TG28_SW_THERMAL_REGULATION_100C,
    TG28_SW_THERMAL_REGULATION_120C,
} tg28_sw_thermal_regulation_t;

/** Pre-charge safety timer duration (REG67 bits1:0, datasheet 6.13.2.65). */
typedef enum {
    TG28_SW_PRECHARGE_TIMER_40MIN = 0,
    TG28_SW_PRECHARGE_TIMER_50MIN,
    TG28_SW_PRECHARGE_TIMER_60MIN,
    TG28_SW_PRECHARGE_TIMER_70MIN,
} tg28_sw_precharge_timer_t;

/** CC/CV charge-cycle safety timer duration (REG67 bits5:4, datasheet 6.13.2.65). */
typedef enum {
    TG28_SW_CHARGE_TIMER_5H = 0,
    TG28_SW_CHARGE_TIMER_8H,
    TG28_SW_CHARGE_TIMER_12H,
    TG28_SW_CHARGE_TIMER_20H,
} tg28_sw_charge_timer_t;

/**
 * Charger safety-timer configuration (REG67, datasheet 6.13.2.65 and
 * 6.7.4.1). A timer expiry drops the charger into battery safe mode and
 * raises TG28_SW_IRQ_CHGTE. timer_slow_during_dpm halves the timer speed
 * while input DPM or thermal regulation limits the current (datasheet
 * 6.7.3.3).
 */
typedef struct {
    bool precharge_timer_enable;
    tg28_sw_precharge_timer_t precharge_timer;
    bool charge_timer_enable;
    tg28_sw_charge_timer_t charge_timer;
    bool timer_slow_during_dpm;
} tg28_sw_charge_timers_t;

/** CHGLED pin display mode (REG69 bits2:1, datasheet 6.13.2.67 and Table 6-6). */
typedef enum {
    /** Type A charging indication. */
    TG28_SW_CHARGE_LED_TYPE_A = 0,
    /** Type B charging indication. */
    TG28_SW_CHARGE_LED_TYPE_B = 1,
    /** Manual output through tg28_sw_charge_led_manual_mode_t. */
    TG28_SW_CHARGE_LED_MANUAL = 2,
} tg28_sw_charge_led_mode_t;

/** CHGLED pin manual drive mode (REG69 bits5:4, datasheet 6.13.2.67). */
typedef enum {
    TG28_SW_CHARGE_LED_MANUAL_HIZ = 0,
    /** Blink: low/Hi-z, 25%/75% duty at 1 Hz. */
    TG28_SW_CHARGE_LED_MANUAL_BLINK_1HZ = 1,
    /** Blink: low/Hi-z, 25%/75% duty at 4 Hz. */
    TG28_SW_CHARGE_LED_MANUAL_BLINK_4HZ = 2,
    TG28_SW_CHARGE_LED_MANUAL_LOW = 3,
} tg28_sw_charge_led_manual_mode_t;

/**
 * CHGLED configuration (REG69, datasheet 6.13.2.67). manual_mode only takes
 * effect while mode is TG28_SW_CHARGE_LED_MANUAL; enable gates the whole
 * pin function.
 */
typedef struct {
    tg28_sw_charge_led_mode_t mode;
    tg28_sw_charge_led_manual_mode_t manual_mode;
    bool enable;
} tg28_sw_charge_led_config_t;

/**
 * Power-off policy (REG22/REG23/REG24/REG27, datasheet 6.13.2.20-22,25).
 *
 * WARNING: these bits decide when the PMIC shuts itself off. A wrong
 * combination (for example a VOFF threshold above the battery voltage with
 * the button source enabled) can make the board impossible to power on or
 * cause unexpected shutdowns. Restore the OTP/POR defaults unless the
 * hardware behavior is fully understood.
 *
 * voff_mv (REG24) is the battery-voltage power-off threshold: 2600-3300 mV
 * in 100 mV steps. dcdc_ovp_shutdown (REG23 bit5) gates the 120%/130%
 * DCDC over-voltage shutdown; dcdc_uvp_shutdown_mask (REG23 bits3:0) gates
 * the 85% under-voltage shutdown per rail (bit0 = DCDC1 ... bit3 = DCDC4).
 * The button_pwrkey_off bits (REG22) control whether the die
 * over-temperature level-2 and the PWRON-beyond-OFFLEVEL sources are
 * allowed to power off. irqlevel/offlevel/onlevel (REG27) set the PWRON
 * key timing windows and only take the discrete datasheet values:
 * irqlevel_ms 1000/1500/2000/2500 ms, offlevel_ms 4000/6000/8000/10000 ms,
 * and onlevel_ms 128/512/1000/2000 ms.
 */
typedef struct {
    uint16_t voff_mv;
    bool dcdc_ovp_shutdown;
    uint8_t dcdc_uvp_shutdown_mask;
    bool die_ot_level2_off_enable;
    bool pwron_offlevel_off_enable;
    bool button_off_as_restart;
    uint16_t irqlevel_ms;
    uint16_t offlevel_ms;
    uint16_t onlevel_ms;
} tg28_sw_poweroff_config_t;

/**
 * Sleep/wakeup policy (REG26, datasheet 6.13.2.24 and 6.5.4.4). The wakeup
 * and sleep enable bits are write-one-to-clear style (RWLC): setting them
 * starts the transition, reading reflects the state.
 */
typedef struct {
    bool irq_wakeup_enable;
    bool pwrok_low_level_wakeup_enable;
    bool wakeup_voltage_from_before_sleep;
    bool wakeup_enable;
    bool sleep_enable;
} tg28_sw_sleep_config_t;

/** GPIO1 output state (REG1B bits3:2, datasheet 6.13.2.17). */
typedef enum {
    TG28_SW_GPIO1_OUTPUT_HIZ = 0,
    TG28_SW_GPIO1_OUTPUT_LOW = 1,
} tg28_sw_gpio1_output_t;

/**
 * DCDC converter operating mode (REG80 bits6:5, REG81 bits7:2, datasheet
 * 6.13.2.69-70). force_ccm keeps all DCDCs in continuous-conduction mode;
 * dvm_ramp selects the DVM voltage-ramp speed; force_pwm per rail overrides
 * the automatic PWM/PFM switching; spread_spectrum_enable and
 * spread_range_khz gate the frequency-spreading feature.
 */
typedef struct {
    bool force_ccm;
    bool dvm_ramp_slow;
    /** Force always-PWM per rail (index 0 = DCDC1 ... 3 = DCDC4); false
     * keeps the automatic PWM/PFM switching. */
    bool force_pwm[4];
    bool spread_spectrum_enable;
    /** false = 50 kHz spread range, true = 100 kHz. */
    bool spread_range_100khz;
} tg28_sw_dcdc_mode_t;

/** I2C configuration used when creating a TG28 device. */
typedef struct {
    uint8_t device_address;
    uint32_t scl_speed_hz;
    /** Optional battery-specific model downloaded through REGA1 on create. */
    const uint8_t *battery_model;
    size_t battery_model_size;
} tg28_sw_config_t;

/** Default TG28 switch-charger I2C configuration. */
#define TG28_SW_CONFIG_DEFAULT()                    \
    {                                               \
        .device_address = TG28_SW_I2C_ADDRESS_DEFAULT, \
        .scl_speed_hz = TG28_SW_I2C_CLOCK_HZ,       \
        .battery_model = NULL,                      \
        .battery_model_size = 0,                    \
    }

/** Power, battery, and charger snapshot. */
typedef struct {
    uint8_t chip_id;
    uint8_t common_status0;
    uint8_t common_status1;
    uint16_t battery_mv;
    uint8_t battery_percent;
    bool battery_present;
    bool vbus_present;
    /** True in every active charge phase: trickle (REG01 2:0 = 0),
     * pre-charge (1), constant current (2), or constant voltage (3). */
    bool charging;
    /** True when the charge cycle finished (REG01 2:0 = 4); 5 means no
     * charging is in progress and leaves both flags false. */
    bool charge_done;
} tg28_sw_status_t;

/**
 * @brief Read contiguous raw TG28 registers.
 *
 * This diagnostic API performs no field decoding and no writeback. Register
 * semantics and side effects remain owned by the PMIC; callers must not use
 * it to replace the typed APIs.
 */
esp_err_t tg28_sw_read_registers(tg28_sw_handle_t handle, uint8_t register_address,
                                 uint8_t *values, size_t count);

/**
 * @brief Create a TG28 switch-charger device on an existing I2C bus.
 *
 * The I2C bus remains owned by the caller. The driver verifies that the chip
 * ID register can be read but accepts unknown revisions so unrecognized
 * silicon can still be diagnosed.
 */
esp_err_t tg28_sw_create(i2c_master_bus_handle_t bus,
                         const tg28_sw_config_t *config,
                         tg28_sw_handle_t *ret_handle);

/** Delete a TG28 device and remove it from the I2C bus. */
esp_err_t tg28_sw_delete(tg28_sw_handle_t handle);

/** Read the chip ID register. */
esp_err_t tg28_sw_get_chip_id(tg28_sw_handle_t handle, uint8_t *chip_id);

/** Return true for chip IDs recognized by the vendor driver. */
bool tg28_sw_is_supported_chip_id(uint8_t chip_id);

/** Read power, battery, and charger state. */
esp_err_t tg28_sw_get_status(tg28_sw_handle_t handle, tg28_sw_status_t *status);

/** Read the raw power-on source bitmap from REG20. */
esp_err_t tg28_sw_get_power_on_source(tg28_sw_handle_t handle, uint8_t *source);

/**
 * Power off the board through the soft-PWROFF command (REG10 bit0, datasheet
 * 6.5.4.3 and 6.13.2.7). Once the write is acknowledged the PMU enters its
 * off state: every DCDC and LDO but the RTCLDO shuts down, the main system
 * rails collapse, and the call does not return in practice. Flush any
 * pending log output before calling. The device lock is released before
 * returning either way, so later TG28 calls stay usable on a board that
 * survives the command (external supply, delayed rail collapse).
 */
esp_err_t tg28_sw_power_off(tg28_sw_handle_t handle);

/**
 * Read the latched power-off source bitmap from REG21 (datasheet
 * 6.13.2.19). Each set bit reports one shutdown source via the
 * TG28_SW_POWER_OFF_SOURCE_* macros; several sources may be latched at
 * once. The latch survives only while the PMIC stays supplied, so after an
 * unexpected power cut revive the PMIC (PWRON key, VBUS still attached)
 * before reading.
 */
esp_err_t tg28_sw_get_power_off_source(tg28_sw_handle_t handle, uint8_t *source);

/**
 * Write the full enable mask of one IRQ enable bank (REG40-REG42).
 * Status bits keep their latched value; only the enables are replaced.
 */
esp_err_t tg28_sw_set_irq_enable(tg28_sw_handle_t handle,
                                 tg28_sw_irq_bank_t bank, uint8_t mask);

/** Read the enable mask of one IRQ enable bank (REG40-REG42). */
esp_err_t tg28_sw_get_irq_enable(tg28_sw_handle_t handle,
                                 tg28_sw_irq_bank_t bank, uint8_t *mask);

/**
 * Enable or disable one named interrupt source (tg28_sw_irq_t) with a
 * read-modify-write on its bank, leaving the other enables untouched.
 */
esp_err_t tg28_sw_set_irq_enable_bit(tg28_sw_handle_t handle,
                                     tg28_sw_irq_t irq, bool enable);

/** Read the enable state of one named interrupt source. */
esp_err_t tg28_sw_get_irq_enable_bit(tg28_sw_handle_t handle,
                                     tg28_sw_irq_t irq, bool *enabled);

/**
 * Configure the four power-key interrupt enable bits (IRQ bank 1, bits 3:0)
 * without disturbing the other enables in the bank.
 */
esp_err_t tg28_sw_configure_power_key_interrupts(tg28_sw_handle_t handle,
        uint8_t enabled_mask);

/**
 * Configure the TS pin (REG50, datasheet 6.13.2.47).
 *
 * Use TG28_SW_TS_MODE_BATTERY_NTC when the battery NTC resistor is connected
 * between the TS pin and GND: the charger then applies the battery
 * temperature protection. The current source must match the resistor
 * (50 uA for the suggested 10 kOhm at 25 C, datasheet 6.7.4.4).
 *
 * Use TG28_SW_TS_MODE_EXTERNAL_FIXED for batteries without an NTC resistor,
 * with the TS pin tied to a fixed external input (for example 10 kOhm to
 * GND); the charger then ignores the TS pin and the current source is
 * normally switched off.
 *
 * current_ua accepts exactly 20/40/50/60 uA. The value is still programmed
 * when the current source is off and takes effect once it is switched on.
 */
esp_err_t tg28_sw_set_ts_config(tg28_sw_handle_t handle,
                                tg28_sw_ts_mode_t mode,
                                tg28_sw_ts_current_source_t current_source,
                                uint16_t current_ua);

/**
 * Read one ADC channel in millivolts (REG34-REG3D, datasheet 6.10).
 *
 * VBAT/VBUS/VSYS convert at 1 mV/LSB, TS at 0.5 mV/LSB, and TDIE at
 * 0.1 mV/LSB; sub-millivolt resolution is truncated. TDIE is the
 * die-temperature sensor voltage, not a temperature. A disabled channel
 * (REG30) returns stale data; enable it first if needed.
 */
esp_err_t tg28_sw_read_adc_channel(tg28_sw_handle_t handle,
                                   tg28_sw_adc_channel_t channel,
                                   uint16_t *millivolts);

/** Enable or disable one ADC channel (REG30). */
esp_err_t tg28_sw_set_adc_channel_enable(tg28_sw_handle_t handle,
        tg28_sw_adc_channel_t channel, bool enable);

/** Read the enable state of one ADC channel (REG30). */
esp_err_t tg28_sw_get_adc_channel_enable(tg28_sw_handle_t handle,
        tg28_sw_adc_channel_t channel, bool *enabled);

/** Set an exactly representable REG62 constant-current charge limit. */
esp_err_t tg28_sw_set_charge_current(tg28_sw_handle_t handle, uint16_t milliamps);

/** Read the REG62 constant-current charge limit. */
esp_err_t tg28_sw_get_charge_current(tg28_sw_handle_t handle, uint16_t *milliamps);

/**
 * Set the REG61 precharge current, 0-200 mA in 25 mA steps (datasheet
 * 6.13.2.60); anything else returns ESP_ERR_INVALID_ARG.
 */
esp_err_t tg28_sw_set_precharge_current(tg28_sw_handle_t handle,
                                        uint16_t milliamps);

/** Read the REG61 precharge current. */
esp_err_t tg28_sw_get_precharge_current(tg28_sw_handle_t handle,
                                        uint16_t *milliamps);

/**
 * Set the REG16 input current limit. Only the discrete vendor levels
 * 100/500/900/1000/1500/2000 mA are accepted; anything else returns
 * ESP_ERR_INVALID_ARG.
 */
esp_err_t tg28_sw_set_input_current_limit(tg28_sw_handle_t handle,
        uint16_t milliamps);

/** Read the REG16 input current limit. */
esp_err_t tg28_sw_get_input_current_limit(tg28_sw_handle_t handle,
        uint16_t *milliamps);

/**
 * Set the REG64 charge termination voltage. On the switch-charger variant
 * only the discrete levels 4000/4100/4200/4350/4400 mV are accepted (codes
 * 1-5; code 0 and codes 6-7 are reserved, datasheet 6.13.2.63); anything
 * else returns ESP_ERR_INVALID_ARG.
 */
esp_err_t tg28_sw_set_charge_voltage(tg28_sw_handle_t handle,
                                     uint16_t millivolts);

/** Read the REG64 charge termination voltage. */
esp_err_t tg28_sw_get_charge_voltage(tg28_sw_handle_t handle,
                                     uint16_t *millivolts);

/**
 * Set the REG63 charge-termination current, 0-200 mA in 25 mA steps, and the
 * termination-enable bit (datasheet 6.13.2.62). When enable is false only
 * the enable bit is cleared; the programmed current code keeps its value.
 */
esp_err_t tg28_sw_set_termination_current(tg28_sw_handle_t handle,
        uint16_t milliamps, bool enable);

/** Read the REG63 termination current and the termination-enable state. */
esp_err_t tg28_sw_get_termination_current(tg28_sw_handle_t handle,
        uint16_t *milliamps, bool *enabled);

/**
 * Set the REG15 VINDPM threshold to an exactly representable value
 * (3880 + 80 * code, codes 0-15, i.e. 3880-5080 mV in 80 mV steps,
 * datasheet 6.13.2.11); anything else returns ESP_ERR_INVALID_ARG.
 */
esp_err_t tg28_sw_set_vindpm(tg28_sw_handle_t handle, uint16_t millivolts);

/** Read the REG15 VINDPM threshold. */
esp_err_t tg28_sw_get_vindpm(tg28_sw_handle_t handle, uint16_t *millivolts);

/**
 * Set the REG1A low-battery warning thresholds (datasheet 6.13.2.16):
 * level1 accepts 0-15 % and level2 accepts 5-20 %, both in 1 % steps;
 * out-of-range values return ESP_ERR_INVALID_ARG. The thresholds raise
 * TG28_SW_IRQ_SOCWL1 and TG28_SW_IRQ_SOCWL2 respectively.
 */
esp_err_t tg28_sw_set_low_battery_warning(tg28_sw_handle_t handle,
        uint8_t level1_percent, uint8_t level2_percent);

/** Read the REG1A low-battery warning thresholds. */
esp_err_t tg28_sw_get_low_battery_warning(tg28_sw_handle_t handle,
        uint8_t *level1_percent, uint8_t *level2_percent);

/**
 * Download and verify a battery-specific fuel-gauge model through REGA1.
 *
 * The sequence follows the vendor reference driver: temporarily disable the
 * charger and the PMIC watchdog, temporarily enable the gauge if necessary,
 * reset the gauge MCU, open BROM, stream the model, reopen BROM for
 * verification, atomically close BROM and select SRAM, reset the gauge MCU,
 * and restore the original REG18 module-enable state. Call once per boot, or
 * pass the model in the create configuration to have the driver do so
 * automatically.
 *
 * size must equal TG28_SW_BATTERY_MODEL_SIZE (128 bytes); any other size
 * returns ESP_ERR_INVALID_SIZE. The model content is battery-specific and
 * must come from the battery supplier. If any model byte may have been
 * written but the operation cannot be verified, cleanup deliberately selects
 * ROM instead of restoring an entry SRAM selection. If safe BROM close/reset
 * release cannot be confirmed, the function leaves the charger and watchdog
 * disabled and the gauge enabled for explicit recovery and returns the
 * cleanup error.
 */
esp_err_t tg28_sw_program_battery_model(tg28_sw_handle_t handle,
                                        const uint8_t *model, size_t size);

/**
 * @brief Read the fuel-gauge BROM parameter image (REGA1 window, datasheet
 * 6.13.2.86; procedure per hardware design guide 6.2).
 *
 * Temporarily enables the gauge and pauses its watchdog, resets the gauge
 * MCU, opens BROM, reads 128 bytes from REGA1, atomically closes BROM while
 * preserving the entry REGA2 source-selection bit, resets the gauge MCU, and
 * restores the original module-enable state. size must equal
 * TG28_SW_BATTERY_MODEL_SIZE. The charger state is preserved, and a disabled
 * gauge is returned to the disabled state.
 *
 * REGA2 bit4 selects ROM or SRAM for the gauge MCU after reset; it does not
 * select a different REGA1 read window. This function therefore leaves bit4
 * unchanged and returns the BROM parameter image exposed by REGA1. If safe
 * BROM close/reset release cannot be confirmed, the function leaves the
 * watchdog disabled and the gauge enabled rather than restoring a state that
 * could reset an unsafe gauge.
 */
esp_err_t tg28_sw_read_battery_model(tg28_sw_handle_t handle,
                                     uint8_t *model, size_t size);

/** Set one regulator to an exactly representable voltage. */
esp_err_t tg28_sw_regulator_set_voltage(tg28_sw_handle_t handle,
                                        tg28_sw_regulator_t regulator,
                                        uint16_t millivolts);

/** Read the programmed voltage of one regulator. */
esp_err_t tg28_sw_regulator_get_voltage(tg28_sw_handle_t handle,
                                        tg28_sw_regulator_t regulator,
                                        uint16_t *millivolts);

/** Enable or disable one regulator without changing its voltage. */
esp_err_t tg28_sw_regulator_enable(tg28_sw_handle_t handle,
                                   tg28_sw_regulator_t regulator,
                                   bool enable);

/** Read the enable state of one regulator. */
esp_err_t tg28_sw_regulator_is_enabled(tg28_sw_handle_t handle,
                                       tg28_sw_regulator_t regulator,
                                       bool *enabled);

/**
 * Open or close a load-switch output. In switch mode the output passes its
 * input rail straight through and no voltage programming applies. The OTP
 * mode cannot be read back: call this only for pins the board's OTP straps
 * as switches. The enable bit is the same physical bit as the LDO enable
 * (REG90 bit7 for DC1SW, REG91 bit0 for DC4SW), so on an LDO-strapped part
 * this simply turns that LDO on or off.
 */
esp_err_t tg28_sw_switch_enable(tg28_sw_handle_t handle,
                                tg28_sw_power_switch_t sw, bool enable);

/** Read the enable state of a load-switch output. */
esp_err_t tg28_sw_switch_is_enabled(tg28_sw_handle_t handle,
                                    tg28_sw_power_switch_t sw, bool *enabled);

/**
 * Read the three interrupt status registers (REG48-REG4A) and attempt to
 * clear every set bit by writing it back (write-one-to-clear).
 *
 * Call this from task context only: it takes the device lock for as long
 * as the blocking I2C traffic needs, so a PMIC IRQ GPIO ISR must not call
 * it directly - the ISR should only notify a task.
 *
 * Several REG48-REG4A bits are condition-latched and clear only after the
 * condition itself clears (VBUS removal, LDO overcurrent recovery,
 * temperature recovery); those bits re-assert while the condition persists.
 */
esp_err_t tg28_sw_get_and_clear_interrupts(tg28_sw_handle_t handle,
        uint8_t status[3]);

/**
 * Enable or disable the cell-battery charger (REG18 bit1, datasheet
 * 6.13.2.14). Disabling stops charging while VBUS bookkeeping stays active;
 * the fuel gauge and watchdog modules have their own enables.
 */
esp_err_t tg28_sw_set_charge_enable(tg28_sw_handle_t handle, bool enable);

/** Read the REG18 bit1 charger-enable state. */
esp_err_t tg28_sw_get_charge_enable(tg28_sw_handle_t handle, bool *enabled);

/**
 * Configure the PMIC watchdog (REG18 bit0 enable, REG19, datasheet
 * 6.13.2.14-15). The period is 1-128 s; on expiry the chip raises
 * TG28_SW_IRQ_WDEXP and optionally restarts the system per the action.
 * Feeding is required once enabled: call tg28_sw_feed_watchdog() more
 * often than the period.
 */
esp_err_t tg28_sw_set_watchdog(tg28_sw_handle_t handle,
                               const tg28_sw_watchdog_config_t *config);

/** Read the watchdog configuration. */
esp_err_t tg28_sw_get_watchdog(tg28_sw_handle_t handle,
                               tg28_sw_watchdog_config_t *config);

/**
 * Feed (clear) the watchdog by pulsing the REG19 bit3 clear signal
 * (RWAC, datasheet 6.13.2.15). Restart the countdown; call periodically
 * while the watchdog is enabled.
 */
esp_err_t tg28_sw_feed_watchdog(tg28_sw_handle_t handle);

/**
 * Enable or disable the fuel-gauge module (REG18 bit3, datasheet 6.13.2.14
 * and 6.11). Disabling stops SOC updates (REGA4 reads stale data).
 */
esp_err_t tg28_sw_set_gauge_enable(tg28_sw_handle_t handle, bool enable);

/** Read the REG18 bit3 fuel-gauge-enable state. */
esp_err_t tg28_sw_get_gauge_enable(tg28_sw_handle_t handle, bool *enabled);

/**
 * Set the battery temperature-sense thresholds and hystereses
 * (REG52-REG57, datasheet 6.13.2.48-53). All fields are TS pin voltages
 * with fixed steps per tg28_sw_ts_thresholds_t; values that are not
 * exactly representable return ESP_ERR_INVALID_ARG.
 */
esp_err_t tg28_sw_set_ts_thresholds(tg28_sw_handle_t handle,
                                    const tg28_sw_ts_thresholds_t *thresholds);

/** Read the battery temperature-sense thresholds and hystereses. */
esp_err_t tg28_sw_get_ts_thresholds(tg28_sw_handle_t handle,
                                    tg28_sw_ts_thresholds_t *thresholds);

/**
 * Configure the JEITA standard (REG58-REG5B, datasheet 6.13.2.54-57):
 * enable, cool/warm current halving, cool/warm CV adjustment, and the T2/T3
 * TS-voltage boundaries. cool_mv must be a multiple of 16 mV and warm_mv a
 * multiple of 8 mV; anything else returns ESP_ERR_INVALID_ARG.
 */
esp_err_t tg28_sw_set_jeita(tg28_sw_handle_t handle,
                            const tg28_sw_jeita_config_t *config);

/** Read the JEITA standard configuration. */
esp_err_t tg28_sw_get_jeita(tg28_sw_handle_t handle,
                            tg28_sw_jeita_config_t *config);

/**
 * Set the external-fixed TS comparison voltage ts_cfg_data (REG5C/REG5D,
 * datasheet 6.13.2.58-59), used together with TG28_SW_TS_MODE_EXTERNAL_FIXED
 * when the battery has no NTC. The value is a 14-bit TS voltage code in
 * 0.5 mV/LSB terms (same scale as the TS ADC channel), 0-16383; anything
 * else returns ESP_ERR_INVALID_ARG.
 */
esp_err_t tg28_sw_set_ts_fixed_threshold(tg28_sw_handle_t handle,
        uint16_t raw_code);

/** Read the external-fixed TS comparison voltage ts_cfg_data code. */
esp_err_t tg28_sw_get_ts_fixed_threshold(tg28_sw_handle_t handle,
        uint16_t *raw_code);

/**
 * Set the thermal-regulation threshold (REG65 bits1:0, datasheet 6.13.2.64
 * and 6.7.4.3). When the die exceeds this temperature the charger reduces
 * current until the die cools.
 */
esp_err_t tg28_sw_set_thermal_regulation(tg28_sw_handle_t handle,
        tg28_sw_thermal_regulation_t threshold);

/** Read the thermal-regulation threshold. */
esp_err_t tg28_sw_get_thermal_regulation(tg28_sw_handle_t handle,
        tg28_sw_thermal_regulation_t *threshold);

/**
 * Configure the charger safety timers (REG67, datasheet 6.13.2.65 and
 * 6.7.4.1): pre-charge timer, CC/CV charge-cycle timer, their enables, and
 * the DPM/thermal slow-down bit. An expiry raises TG28_SW_IRQ_CHGTE and
 * drops the charger into battery safe mode.
 */
esp_err_t tg28_sw_set_charge_timers(tg28_sw_handle_t handle,
                                    const tg28_sw_charge_timers_t *timers);

/** Read the charger safety-timer configuration. */
esp_err_t tg28_sw_get_charge_timers(tg28_sw_handle_t handle,
                                    tg28_sw_charge_timers_t *timers);

/**
 * Enable or disable battery detection (REG68 bit0, datasheet 6.13.2.66 and
 * 6.7.3.5). When disabled the PMU reports the battery as always present.
 */
esp_err_t tg28_sw_set_battery_detect_enable(tg28_sw_handle_t handle, bool enable);

/** Read the battery-detection enable state. */
esp_err_t tg28_sw_get_battery_detect_enable(tg28_sw_handle_t handle, bool *enabled);

/**
 * Configure the CHGLED charging-indication pin (REG69, datasheet 6.13.2.67
 * and Table 6-6): type A/B display mode, manual drive mode, and the pin
 * enable.
 */
esp_err_t tg28_sw_set_charge_led(tg28_sw_handle_t handle,
                                 const tg28_sw_charge_led_config_t *config);

/** Read the CHGLED configuration. */
esp_err_t tg28_sw_get_charge_led(tg28_sw_handle_t handle,
                                 tg28_sw_charge_led_config_t *config);

/**
 * Configure backup/button-battery charging (REG18 bit2 enable, REG6A
 * termination voltage, datasheet 6.13.2.68 and 6.12.2). Charging runs in
 * linear mode at 100 uA; termination_mv accepts 2600-3300 mV in 100 mV
 * steps, anything else returns ESP_ERR_INVALID_ARG.
 */
esp_err_t tg28_sw_set_backup_charge(tg28_sw_handle_t handle, bool enable,
                                    uint16_t termination_mv);

/** Read the backup-battery charge enable and termination voltage. */
esp_err_t tg28_sw_get_backup_charge(tg28_sw_handle_t handle, bool *enabled,
                                    uint16_t *termination_mv);

/**
 * Set the minimum system voltage (REG14 bits2:0, datasheet 6.13.2.10).
 * Accepts 3200-3900 mV in 100 mV steps; anything else returns
 * ESP_ERR_INVALID_ARG.
 */
esp_err_t tg28_sw_set_min_sys_voltage(tg28_sw_handle_t handle, uint16_t millivolts);

/** Read the minimum system voltage. */
esp_err_t tg28_sw_get_min_sys_voltage(tg28_sw_handle_t handle, uint16_t *millivolts);

/**
 * Configure the power-off policy group (REG22/REG23/REG24/REG27, datasheet
 * 6.13.2.20-22,25). WARNING: incorrect settings can make the board
 * unbootable or trigger unexpected shutdowns; see
 * tg28_sw_poweroff_config_t. irqlevel_ms accepts 1000/1500/2000/2500,
 * offlevel_ms accepts 4000/6000/8000/10000, onlevel_ms accepts
 * 128/512/1000/2000; anything else returns ESP_ERR_INVALID_ARG. The driver
 * clears the power-off sources first and enables the requested ones only
 * after the new thresholds land, so a power key held across the update
 * cannot trip the old OFFLEVEL window.
 */
esp_err_t tg28_sw_set_poweroff_config(tg28_sw_handle_t handle,
                                      const tg28_sw_poweroff_config_t *config);

/** Read the power-off policy group. */
esp_err_t tg28_sw_get_poweroff_config(tg28_sw_handle_t handle,
                                      tg28_sw_poweroff_config_t *config);

/**
 * Configure sleep and wakeup behavior (REG26, datasheet 6.13.2.24 and
 * 6.5.4.4). The wakeup/sleep enable bits are RWLC command bits: setting
 * wakeup_enable starts a wakeup transition and setting sleep_enable starts
 * a sleep transition; both self-report through the getter.
 */
esp_err_t tg28_sw_set_sleep_config(tg28_sw_handle_t handle,
                                   const tg28_sw_sleep_config_t *config);

/** Read the sleep/wakeup configuration. */
esp_err_t tg28_sw_get_sleep_config(tg28_sw_handle_t handle,
                                   tg28_sw_sleep_config_t *config);

/**
 * Drive the GPIO1 open-drain output (REG1B bits3:2, datasheet 6.13.2.17).
 * Only high-impedance and driven-low are defined; the other two codes are
 * reserved. GPIO1 may be OTP-strapped as the RTCLDO2 output on some parts;
 * use it as a GPIO only when the board design confirms the strap.
 */
esp_err_t tg28_sw_set_gpio1_config(tg28_sw_handle_t handle,
                                   tg28_sw_gpio1_output_t output);

/** Read the GPIO1 output state. */
esp_err_t tg28_sw_get_gpio1_config(tg28_sw_handle_t handle,
                                   tg28_sw_gpio1_output_t *output);

/**
 * Configure DCDC operating modes (REG80 bits6:5 force-CCM/DVM ramp, REG81
 * force-PWM per rail and spread spectrum, datasheet 6.13.2.69-70). Only the
 * four DCDC rails of the switch-charger variant are addressable.
 */
esp_err_t tg28_sw_set_dcdc_mode(tg28_sw_handle_t handle,
                                const tg28_sw_dcdc_mode_t *mode);

/** Read the DCDC operating-mode configuration. */
esp_err_t tg28_sw_get_dcdc_mode(tg28_sw_handle_t handle,
                                tg28_sw_dcdc_mode_t *mode);

/**
 * Software-reset the whole PMIC (REG10 bit1, RWAC, datasheet 6.13.2.7 and
 * 6.5.4.5). WARNING: this restarts the entire PMU; all rails cycle and
 * system registers reset to defaults. The call does not return in practice
 * on success; the device lock is released before returning either way.
 */
esp_err_t tg28_sw_soft_reset(tg28_sw_handle_t handle);

/**
 * Keep the BATFET enabled while the PMU is powered off and only the battery
 * supplies it (REG12 bit3, datasheet 6.13.2.8). Clearing it gives the
 * lowest ship-mode current (<40 uA); enabling keeps the battery connected
 * to SYS in the off state. The POR value is EFUSE-defined per part.
 */
esp_err_t tg28_sw_set_batfet_off_state_enable(tg28_sw_handle_t handle, bool enable);

/** Read the REG12 bit3 BATFET off-state enable. */
esp_err_t tg28_sw_get_batfet_off_state_enable(tg28_sw_handle_t handle, bool *enabled);

/**
 * Write one raw register byte (diagnostic escape hatch paired with
 * tg28_sw_read_registers). WARNING: prefer the typed APIs; writing the
 * wrong register can affect charging safety, rail sequencing, or reset the
 * PMU. No field decoding or side-effect protection is applied.
 */
esp_err_t tg28_sw_write_register(tg28_sw_handle_t handle, uint8_t register_address,
                                 uint8_t value);

/** Return a stable lowercase regulator name. */
const char *tg28_sw_regulator_name(tg28_sw_regulator_t regulator);

/** Return a stable lowercase switch name ("dc1sw"/"dc4sw"). */
const char *tg28_sw_switch_name(tg28_sw_power_switch_t sw);

#ifdef __cplusplus
}
#endif
