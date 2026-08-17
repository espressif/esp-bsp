/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Driver for the Epson RX8130CE real-time clock.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RX8130CE_I2C_ADDRESS_DEFAULT 0x32
#define RX8130CE_I2C_CLOCK_HZ        400000

#define RX8130CE_FLAG_VBLF           (1U << 7)
#define RX8130CE_FLAG_UF             (1U << 5)
#define RX8130CE_FLAG_TF             (1U << 4)
#define RX8130CE_FLAG_AF             (1U << 3)
#define RX8130CE_FLAG_RSF            (1U << 2)
#define RX8130CE_FLAG_VLF            (1U << 1)
#define RX8130CE_FLAG_VBFF           (1U << 0)

/** Opaque RX8130CE device handle. */
typedef struct rx8130ce_device_t *rx8130ce_handle_t;

/**
 * Backup charge cutoff voltage (BFVSEL1-0, appman Table 39).
 *
 * The values match the BFVSEL1-BFVSEL0 encoding of the control 1 register,
 * so they can be applied to the register image directly. 11b (charging
 * without a voltage cutoff) is deliberately not selectable: without the
 * cutoff the backup source depends entirely on the external current limit.
 */
typedef enum {
    RX8130CE_CHARGE_CUTOFF_3_02V = 0, /**< Stop charging at 3.02 V (POR default). */
    RX8130CE_CHARGE_CUTOFF_3_08V = 1, /**< Stop charging at 3.08 V. */
    RX8130CE_CHARGE_CUTOFF_2_92V = 2, /**< Stop charging at 2.92 V. */
} rx8130ce_charge_cutoff_t;

/** I2C and backup supply configuration used when creating a device. */
typedef struct {
    uint8_t device_address;
    uint32_t scl_speed_hz;
    /**
     * Backup battery charge policy applied on create (CHGEN bit, appman
     * 14.7.2). Automatic supply switchover (INIEN) is always enabled.
     * Keep false for a primary (non-rechargeable) backup cell; set true when
     * the board carries a rechargeable backup source (secondary cell or
     * supercapacitor) that must be charged from VDD.
     */
    bool backup_charge_enable;
    /**
     * Full-charge detection voltage that stops backup charging (BFVSEL1-0,
     * appman Table 39), applied together with the charge policy in both the
     * normal power-up path and the VLF=1 full-register initialization.
     * When charging is enabled, the board must limit the charge current
     * externally to 40 mA max (appman Table 33).
     */
    rx8130ce_charge_cutoff_t backup_charge_cutoff;
    /**
     * Enable the backup low-voltage detection while charging is disabled
     * (VBLFE, appman Table 42). Keep true for a primary (non-rechargeable)
     * backup cell: with VBLFE=0 and CHGEN=0 the VBLF flag never asserts,
     * so rx8130ce_status_t.backup_voltage_low would stay false regardless
     * of the backup voltage.
     */
    bool backup_voltage_low_detect;
} rx8130ce_config_t;

/** Default RX8130CE configuration: I2C defaults, backup charging off. */
#define RX8130CE_CONFIG_DEFAULT()                 \
    {                                             \
        .device_address = RX8130CE_I2C_ADDRESS_DEFAULT, \
        .scl_speed_hz = RX8130CE_I2C_CLOCK_HZ,    \
        .backup_charge_enable = false,            \
        .backup_charge_cutoff = RX8130CE_CHARGE_CUTOFF_3_02V, \
        .backup_voltage_low_detect = true,        \
    }

/** Calendar time represented by the RX8130CE. */
typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t weekday;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} rx8130ce_time_t;

/** Decoded status from the RX8130CE flag register. */
typedef struct {
    uint8_t raw;
    bool time_valid;
    bool alarm;
    bool timer;
    bool update;
    bool reset;
    bool backup_voltage_low;
    bool backup_battery_full;
} rx8130ce_status_t;

/**
 * Alarm compare settings for the RX8130CE.
 *
 * Each *_en flag selects whether the field participates in the alarm
 * comparison; it maps to the active-low AE bit of the alarm register.
 * Fields with a cleared *_en flag are ignored. The hardware compares either
 * the day of month or the weekday, never both, so day_en and weekday_en are
 * mutually exclusive.
 */
typedef struct {
    bool minute_en;  /**< Compare the minute field. */
    uint8_t minute;  /**< Minute, 0-59. */
    bool hour_en;    /**< Compare the hour field. */
    uint8_t hour;    /**< Hour, 0-23. */
    bool day_en;     /**< Compare the day-of-month field. */
    uint8_t day;     /**< Day of month, 1-31. */
    bool weekday_en; /**< Compare the weekday field. */
    uint8_t weekday; /**< Weekday, 0 (Sunday) - 6 (Saturday). */
} rx8130ce_alarm_t;

/**
 * Source clock of the fixed-cycle (wake-up) timer.
 *
 * The values match the TSEL2-TSEL0 encoding of the extension register, so
 * they can be applied to the register image directly.
 */
typedef enum {
    RX8130CE_TIMER_SOURCE_4096HZ = 0,   /**< 4096 Hz: 244.14 us per count. */
    RX8130CE_TIMER_SOURCE_64HZ = 1,     /**< 64 Hz: 15.625 ms per count. */
    RX8130CE_TIMER_SOURCE_1HZ = 2,      /**< 1 Hz: 1 s per count. */
    RX8130CE_TIMER_SOURCE_1_60HZ = 3,   /**< 1/60 Hz: 1 min per count. */
    RX8130CE_TIMER_SOURCE_1_3600HZ = 4, /**< 1/3600 Hz: 1 h per count. */
} rx8130ce_timer_source_t;

/**
 * Fixed-cycle (wake-up) timer settings for the RX8130CE.
 *
 * The 16-bit counter counts down from count at the source clock rate and
 * latches TF when it reaches zero; with TIE set the /IRQ pin pulses low for
 * the auto reset time. The period ranges from 244.14 us (one count at
 * 4096 Hz) to 65535 hours.
 */
typedef struct {
    bool enable; /**< Start or stop the countdown using the timer-enable bit. */
    rx8130ce_timer_source_t source_clock; /**< Countdown rate (TSEL). */
    uint16_t count; /**< Down-counter preset, 1-65535. */
} rx8130ce_timer_t;

/**
 * Furthest future target rx8130ce_alarm_from_time() accepts, in days.
 *
 * The alarm hardware compares day-of-month, hour, and minute only, so the
 * same compare pattern recurs every month (and day 29-31 patterns can skip
 * short months). A target no more than 27 days out is always the first
 * recurrence of its pattern, because the previous recurrence is at least
 * 28 days earlier; farther targets could match an earlier recurrence.
 */
#define RX8130CE_ALARM_MAX_FUTURE_DAYS 27

/**
 * Power-on / backup-domain health check result.
 *
 * Reported by rx8130ce_check_power(); the driver only reads the flags and
 * derives advice, it does not modify the device.
 */
typedef struct {
    bool voltage_low;      /**< VLF reads 1: register contents were lost. */
    bool reset_detected;   /**< RSF reads 1: a reset-voltage event latched. */
    bool init_recommended; /**< Driver advice: re-initialize and re-set the
                                calendar (mirrors voltage_low, appman 14.5.1:
                                VLF=1 requires initializing all registers). */
} rx8130ce_power_check_t;

/** Return true when a calendar value can be represented by the device. */
bool rx8130ce_time_is_valid(const rx8130ce_time_t *time);

/** Return true when an alarm value can be represented by the device. */
bool rx8130ce_alarm_is_valid(const rx8130ce_alarm_t *alarm);

/**
 * Derive alarm compare settings that fire at an absolute future time.
 *
 * Pure helper for the "wake at a specific moment" use case (for example
 * deep-sleep wake-up on a board without a 32.768 kHz crystal, where the RTC
 * alarm is the only timed wake-up path): read the current time with
 * rx8130ce_get_time(), pick the target, and pass both here.
 *
 * The result compares minute, hour, and day-of-month; the hardware then
 * latches AF (and asserts /IRQ when AIE is set) at the start of the target
 * minute. The target must be at least one minute ahead of now and no more
 * than RX8130CE_ALARM_MAX_FUTURE_DAYS days out, otherwise the call fails
 * with ESP_ERR_INVALID_ARG.
 *
 * @param now Current calendar time; must pass rx8130ce_time_is_valid().
 * @param target Wake-up moment; must pass rx8130ce_time_is_valid() and lie
 *        strictly after now at minute granularity.
 * @param out_alarm Receives the compare settings for rx8130ce_set_alarm().
 */
esp_err_t rx8130ce_alarm_from_time(const rx8130ce_time_t *now,
                                   const rx8130ce_time_t *target,
                                   rx8130ce_alarm_t *out_alarm);

/**
 * Encode an alarm into the raw 17h-19h register image.
 *
 * @param alarm Alarm value; must pass rx8130ce_alarm_is_valid().
 * @param registers Receives the MIN/HOUR/WEEK-DAY alarm register values
 *        with the active-low AE bits applied.
 * @param use_day_alarm Receives the required WADA bit state: true selects
 *        the day-of-month register layout, false the weekday layout.
 */
void rx8130ce_alarm_encode(const rx8130ce_alarm_t *alarm,
                           uint8_t registers[3], bool *use_day_alarm);

/** Return true when a timer value can be represented by the device. */
bool rx8130ce_timer_is_valid(const rx8130ce_timer_t *timer);

/**
 * Encode a timer preset into the raw 1Ah-1Bh register image.
 *
 * @param timer Timer value; must pass rx8130ce_timer_is_valid().
 * @param registers Receives the Timer Counter 0/1 register values, low byte
 *        first.
 * @param tsel_bits Receives the raw TSEL2-TSEL0 bit field for the extension
 *        register.
 */
void rx8130ce_timer_encode(const rx8130ce_timer_t *timer,
                           uint8_t registers[2], uint8_t *tsel_bits);

/**
 * Create a device on an existing I2C bus and verify register access.
 *
 * A NULL config selects RX8130CE_CONFIG_DEFAULT(). The configured backup
 * charge policy, charge cutoff, and backup low-voltage detection setting
 * are applied after normal power-up and as part of the VLF=1 full-register
 * initialization.
 */
esp_err_t rx8130ce_create(i2c_master_bus_handle_t bus,
                          const rx8130ce_config_t *config,
                          rx8130ce_handle_t *ret_handle);

/** Delete the device and remove it from the I2C bus. */
esp_err_t rx8130ce_delete(rx8130ce_handle_t handle);

/** Read the calendar and optionally return the retained status flags. */
esp_err_t rx8130ce_get_time(rx8130ce_handle_t handle,
                            rx8130ce_time_t *time,
                            rx8130ce_status_t *status);

/** Set the calendar while the time counter is protected by the STOP bit. */
esp_err_t rx8130ce_set_time(rx8130ce_handle_t handle,
                            const rx8130ce_time_t *time);

/** Read and decode the flag register without clearing it. */
esp_err_t rx8130ce_get_status(rx8130ce_handle_t handle,
                              rx8130ce_status_t *status);

/**
 * Check the power-on / backup-domain health flags (VLF, RSF).
 *
 * Advisory and read-only: nothing is cleared or reconfigured. When
 * init_recommended comes back true, register contents are untrustworthy;
 * recover by re-setting the calendar with rx8130ce_set_time() (which clears
 * VLF) and re-applying alarm/timer settings. Note that rx8130ce_create()
 * already performs the full initialization when it finds VLF=1, so a true
 * result at runtime means the backup supply dropped out after creation.
 */
esp_err_t rx8130ce_check_power(rx8130ce_handle_t handle,
                               rx8130ce_power_check_t *out_check);

/**
 * Configure backup battery charging at runtime (CHGEN/INIEN, appman 14.7.2).
 *
 * Automatic supply switchover (INIEN=1, the recommended setting) is always
 * kept enabled; enable selects whether VDD also charges the backup source
 * (CHGEN). Keep disabled for a primary (non-rechargeable) backup cell;
 * enable only when the board carries a rechargeable backup source. The
 * default after rx8130ce_create() comes from
 * rx8130ce_config_t.backup_charge_enable (off unless configured otherwise).
 * Every call re-applies the configured charge cutoff and VBLFE setting, so
 * charging can never (re-)start with the cutoff-less BFVSEL setting.
 */
esp_err_t rx8130ce_set_backup_charge(rx8130ce_handle_t handle, bool enable);

/** Read back whether backup battery charging is enabled (CHGEN). */
esp_err_t rx8130ce_get_backup_charge(rx8130ce_handle_t handle,
                                     bool *out_enabled);

/**
 * Program the alarm compare registers.
 *
 * AIE is held cleared while the registers change, as recommended by the
 * application manual, and any latched alarm flag is cleared before the
 * previous AIE state is restored. Use rx8130ce_alarm_irq_enable() to route
 * the alarm event to the /IRQ pin.
 */
esp_err_t rx8130ce_set_alarm(rx8130ce_handle_t handle,
                             const rx8130ce_alarm_t *alarm);

/** Read back the alarm compare registers. */
esp_err_t rx8130ce_get_alarm(rx8130ce_handle_t handle,
                             rx8130ce_alarm_t *out_alarm);

/** Enable or disable the alarm interrupt output on the /IRQ pin (AIE). */
esp_err_t rx8130ce_alarm_irq_enable(rx8130ce_handle_t handle, bool enable);

/**
 * Disarm the alarm: gate /IRQ (AIE=0), ignore all compare fields, and clear
 * any latched alarm flag.
 *
 * With every compare field ignored the hardware matches once per minute
 * (appman 14.3.1 note *3), so AIE stays cleared to keep /IRQ released and AF
 * may re-latch afterwards; both are harmless while the alarm is disarmed.
 * Combine with rx8130ce_alarm_irq_enable() to re-arm after
 * rx8130ce_set_alarm().
 */
esp_err_t rx8130ce_clear_alarm(rx8130ce_handle_t handle);

/**
 * Report and clear the latched alarm flag (AF).
 *
 * Selective variant of rx8130ce_get_and_clear_interrupts() for the shared
 * /IRQ line: only AF is cleared, UF/TF and the remaining flags are left
 * untouched. Per the datasheet the flag clears when written 0 and ignores
 * writes of 1, so the read-modify-write used here cannot disturb other
 * flags. alarm_flag may be NULL to just clear.
 */
esp_err_t rx8130ce_get_and_clear_alarm_flag(rx8130ce_handle_t handle,
        bool *alarm_flag);

/**
 * Program the fixed-cycle (wake-up) timer.
 *
 * The timer-enable bit is held cleared while the preset and source clock change, as required by
 * the application manual, and any latched timer flag is cleared. The
 * countdown starts from the preset when timer.enable is true; false leaves
 * the timer stopped. Use rx8130ce_timer_irq_enable() to route the timer event
 * to the /IRQ pin.
 */
esp_err_t rx8130ce_set_timer(rx8130ce_handle_t handle,
                             const rx8130ce_timer_t *timer);

/**
 * Read back the fixed-cycle timer settings.
 *
 * While the timer runs (enable is true), count reads back as the live
 * down-count, which is not latched during the read; read twice until two
 * reads agree, or stop the timer first, for an exact value. count may read
 * back 0 before the timer is first programmed.
 */
esp_err_t rx8130ce_get_timer(rx8130ce_handle_t handle,
                             rx8130ce_timer_t *out_timer);

/** Enable or disable the timer interrupt output on the /IRQ pin (TIE). */
esp_err_t rx8130ce_timer_irq_enable(rx8130ce_handle_t handle, bool enable);

/** Enable or disable the time update interrupt on the /IRQ pin (UIE). */
esp_err_t rx8130ce_update_irq_enable(rx8130ce_handle_t handle, bool enable);

/** Read and clear the update, timer, and alarm interrupt flags. */
esp_err_t rx8130ce_get_and_clear_interrupts(rx8130ce_handle_t handle,
        uint8_t *flags);

/**
 * FOUT clock output selection (FSEL1/FSEL0, appman 14.6.2).
 *
 * The values match the FSEL1-FSEL0 encoding of the extension register, so
 * they can be applied to the register image directly. FOUT stops (Hi-z)
 * while the main supply is below VDET1 (appman 14.6).
 */
typedef enum {
    RX8130CE_FOUT_32768HZ = 0, /**< 32.768 kHz output (POR default). */
    RX8130CE_FOUT_1024HZ = 1,  /**< 1024 Hz output. */
    RX8130CE_FOUT_1HZ = 2,     /**< 1 Hz output; disabled while STOP=1
                                    (appman 13.3.5 7) note 3). */
    RX8130CE_FOUT_OFF = 3,     /**< FOUT output disabled. */
} rx8130ce_fout_t;

/** Time update interrupt period selection (USEL, appman 14.4.1). */
typedef enum {
    RX8130CE_UPDATE_IRQ_PER_SECOND = 0, /**< Interrupt once per second (POR default). */
    RX8130CE_UPDATE_IRQ_PER_MINUTE = 1, /**< Interrupt once per minute. */
} rx8130ce_update_irq_mode_t;

/**
 * Long-Timer accumulation mode (TBKE/TBKON, appman 14.2.2 6)).
 *
 * Selects on which supply the fixed-cycle timer keeps counting, so the
 * counter can be used as an operation-time (usage) accumulator across power
 * modes.
 */
typedef enum {
    RX8130CE_TIMER_COUNT_ALWAYS = 0, /**< Count on both main and backup supply (TBKE=0, POR default). */
    RX8130CE_TIMER_COUNT_MAIN = 1,   /**< Count only during main-supply (VDD) operation (TBKE=1, TBKON=0). */
    RX8130CE_TIMER_COUNT_BACKUP = 2, /**< Count only during backup-supply (VBAT) operation (TBKE=1, TBKON=1). */
} rx8130ce_timer_count_mode_t;

/**
 * Digital offset step size, in parts-per-million (appman 14.10).
 *
 * The offset register applies a signed correction in 3.05e-6 steps;
 * positive values make the clock run faster. The representable range is
 * -64..+63 steps (-195.31e-6 to +192.26e-6).
 */
#define RX8130CE_DIGITAL_OFFSET_STEP_PPM  3.05f
#define RX8130CE_DIGITAL_OFFSET_MIN_STEPS (-64)
#define RX8130CE_DIGITAL_OFFSET_MAX_STEPS (63)
/** User RAM size in bytes (registers 20h-23h, appman 13.3.2). */
#define RX8130CE_USER_RAM_SIZE            4

/**
 * Select the FOUT clock output frequency.
 *
 * A 32.768 kHz output is unaffected by the digital offset function; 1 Hz
 * and 1024 Hz outputs show jitter from it (appman 14.10.1).
 */
esp_err_t rx8130ce_set_fout(rx8130ce_handle_t handle, rx8130ce_fout_t frequency);

/** Read back the FOUT clock output selection. */
esp_err_t rx8130ce_get_fout(rx8130ce_handle_t handle, rx8130ce_fout_t *out_frequency);

/**
 * Select the time update interrupt period (second or minute update).
 *
 * The /IRQ routing itself is controlled by rx8130ce_update_irq_enable().
 */
esp_err_t rx8130ce_set_update_irq_mode(rx8130ce_handle_t handle, rx8130ce_update_irq_mode_t mode);

/** Read back the time update interrupt period selection. */
esp_err_t rx8130ce_get_update_irq_mode(rx8130ce_handle_t handle, rx8130ce_update_irq_mode_t *out_mode);

/**
 * Select on which supply the fixed-cycle timer counts (Long-Timer mode).
 *
 * With a main- or backup-only selection the timer doubles as an
 * operation-time accumulator for the selected power domain.
 */
esp_err_t rx8130ce_set_timer_count_mode(rx8130ce_handle_t handle, rx8130ce_timer_count_mode_t mode);

/** Read back the Long-Timer accumulation mode. */
esp_err_t rx8130ce_get_timer_count_mode(rx8130ce_handle_t handle, rx8130ce_timer_count_mode_t *out_mode);

/**
 * Pause or resume the fixed-cycle timer countdown (TSTP, appman 14.2.2 7)).
 *
 * Pausing keeps the current count value; resuming restarts from it. TSTP is
 * only effective while the timer runs (timer enable is true), the time counter is not
 * stopped (STOP=0) and the count mode accumulates both supplies (TBKE=0);
 * otherwise the bit is ignored by the hardware.
 */
esp_err_t rx8130ce_timer_pause(rx8130ce_handle_t handle, bool pause);

/**
 * Write the battery-backed user RAM (registers 20h-23h).
 *
 * @param offset Byte offset into the 4-byte user RAM, 0-3.
 * @param length Byte count; offset + length must not exceed
 *        RX8130CE_USER_RAM_SIZE.
 */
esp_err_t rx8130ce_write_user_ram(rx8130ce_handle_t handle, uint8_t offset,
                                  const uint8_t *data, size_t length);

/** Read the battery-backed user RAM (registers 20h-23h). */
esp_err_t rx8130ce_read_user_ram(rx8130ce_handle_t handle, uint8_t offset,
                                 uint8_t *data, size_t length);

/**
 * Encode a digital offset value into the raw 30h register image.
 *
 * Pure helper; validates the step range. The correction is applied by the
 * hardware every 10 seconds when enabled.
 *
 * @param enable DTE bit: apply the offset when true.
 * @param offset_steps Signed correction in 3.05e-6 steps,
 *        RX8130CE_DIGITAL_OFFSET_MIN_STEPS..RX8130CE_DIGITAL_OFFSET_MAX_STEPS;
 *        positive makes the clock run faster.
 * @param out_register Receives the register value.
 */
esp_err_t rx8130ce_digital_offset_encode(bool enable, int8_t offset_steps,
        uint8_t *out_register);

/** Decode a raw 30h register image into enable state and signed steps. */
void rx8130ce_digital_offset_decode(uint8_t register_value, bool *out_enabled,
                                    int8_t *out_offset_steps);

/** Program the digital offset register (30h). See rx8130ce_digital_offset_encode(). */
esp_err_t rx8130ce_set_digital_offset(rx8130ce_handle_t handle, bool enable,
                                      int8_t offset_steps);

/** Read back the digital offset register (30h). */
esp_err_t rx8130ce_get_digital_offset(rx8130ce_handle_t handle, bool *out_enabled,
                                      int8_t *out_offset_steps);

/**
 * Raw register read fallback (user registers 10h-23h, 30h and 31h only).
 *
 * The application manual restricts access to the documented user registers
 * (13.2.1 note *6); the complete [start, start + length) window is
 * validated against them, so a transfer crossing into a reserved register
 * is rejected. Reads follow the device auto-increment wrap-around (1Fh
 * wraps to 10h, 2Fh to 20h, 3Fh to 30h; appman 14.12.4). Use this for
 * feature blocks without a structured API (SMPTSEL/RSVSEL power-detection
 * tuning and similar).
 */
esp_err_t rx8130ce_read_registers(rx8130ce_handle_t handle, uint8_t start_register,
                                  uint8_t *data, size_t length);

/**
 * Raw register write fallback (user registers 10h-23h, 30h and 31h only).
 *
 * See rx8130ce_read_registers() for the address policy. Writes a single
 * register; no driver state is updated, so prefer the structured APIs
 * whenever one exists for the target field. In register 31h only bit 0
 * (VBLFE) is defined; keep the remaining manufacturer test bits at 0.
 */
esp_err_t rx8130ce_write_register(rx8130ce_handle_t handle, uint8_t reg,
                                  uint8_t value);

#ifdef __cplusplus
}
#endif
