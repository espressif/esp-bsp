# ChangeLog

## v0.4.0 - 2026-08-22

### Features

\* Expand the datasheet feature coverage of the switch-charger variant: charger enable (`tg28_sw_set/get_charge_enable`, REG18 bit1), fuel-gauge module enable (`tg28_sw_set/get_gauge_enable`, REG18 bit3), PMIC watchdog (`tg28_sw_set/get_watchdog` + `tg28_sw_feed_watchdog`, REG18 bit0 + REG19), TS/JEITA threshold set (`tg28_sw_set/get_ts_thresholds`, REG52-57), JEITA standard configuration (`tg28_sw_set/get_jeita`, REG58-5B), external-fixed TS comparison (`tg28_sw_set/get_ts_fixed_threshold`, REG5C/5D), thermal regulation threshold (`tg28_sw_set/get_thermal_regulation`, REG65), charger safety timers (`tg28_sw_set/get_charge_timers`, REG67), battery detection enable (REG68 bit0), CHGLED pin control (`tg28_sw_set/get_charge_led`, REG69), backup/button-battery charging (`tg28_sw_set/get_backup_charge`, REG18 bit2 + REG6A), minimum system voltage (REG14), power-off policy group (`tg28_sw_set/get_poweroff_config`, REG22/23/24/27), sleep/wakeup control (`tg28_sw_set/get_sleep_config`, REG26), GPIO1 output (REG1B), DCDC operating modes (`tg28_sw_set/get_dcdc_mode`, REG80/81), whole-PMIC software reset (`tg28_sw_soft_reset`, REG10 bit1), and BATFET off-state keep (REG12 bit3, ship mode)
* Add `tg28_sw_write_register` as the raw-write escape hatch paired with `tg28_sw_read_registers`
* Add `tg28_sw_read_battery_model` to dump the 128-byte BROM parameter image exposed by REGA1
* Add `TG28_SW_BATTERY_MODEL_SIZE` (128) and enforce it in `tg28_sw_program_battery_model` and the create-time config: the gauge BROM window takes exactly 128 bytes (hardware design guide 6.2), any other size returns ESP_ERR_INVALID_SIZE
* Add `TG28_SW_POWER_OFF_SOURCE_*` bit names for the full REG21 power-off source byte
* Add load-switch channels `TG28_SW_SWITCH_DC1SW`/`TG28_SW_SWITCH_DC4SW` with `tg28_sw_switch_enable`/`tg28_sw_switch_is_enabled`/`tg28_sw_switch_name` (REG90 bit7 / REG91 bit0, the same physical bits as the DLDO1/DLDO2 LDO enables)
* Add 23 named interrupt sources `tg28_sw_irq_t` covering REG40-REG42 / REG48-REG4A and per-bit `tg28_sw_set_irq_enable_bit`/`tg28_sw_get_irq_enable_bit`; slot 21 (REG42 bit5) is reserved and has no symbol
* Add `tg28_sw_set_precharge_current`/`tg28_sw_get_precharge_current` (REG61, 0-200 mA in 25 mA steps)
* Add `tg28_sw_set_termination_current`/`tg28_sw_get_termination_current` (REG63, 0-200 mA in 25 mA steps plus the termination-enable bit)
* Add `tg28_sw_set_low_battery_warning`/`tg28_sw_get_low_battery_warning` (REG1A, level1 0-15 % and level2 5-20 % in 1 % steps)
* Widen the REG15 VINDPM window to the full hardware range 3880-5080 mV (codes 0-15)
* Add `tg28_sw_power_off` (REG10 bit0 soft-PWROFF command, datasheet 6.5.4.3) and `tg28_sw_get_power_off_source` (REG21, datasheet 6.13.2.19)

### Fixes

* Correct the REG64 charge-voltage table for the switch-charger variant: code 0 is reserved, so the valid levels are 4000/4100/4200/4350/4400 mV and the linear-variant 3900 mV level is dropped
* Cap DLDO1 at 3300 mV per the REG99 enumeration (codes 29-31 reserved); the "0.5-3.4V" headline is contradicted by the enumeration
* Mask VINDPM to bits 3:0 so the read-only REG15 bits 7:4 are never written or fed into the decode
* Release the device mutex on the success path of `tg28_sw_power_off` and `tg28_sw_soft_reset` as well, so the next TG28 call cannot deadlock on a board that survives the command (external supply, delayed rail collapse)
* Preserve the original REGA2 model-source selection in `tg28_sw_read_battery_model`; REGA2 bit4 selects the gauge MCU's runtime source, not a different REGA1 read window
* Temporarily enable a disabled gauge during model access and make BROM close/source restore a verified atomic REGA2 write; failed cleanup no longer resets on an unconfirmed source or uses an invalid entry snapshot
* Fall back to ROM after any failed or unverified battery-model write, so cleanup cannot activate a partial SRAM image
* Require a confirmed REG17 assert/deassert pulse before restoring charger/watchdog state after model cleanup; merely observing reset released no longer activates a potentially partial SRAM image
* Recover an already-open BROM window through ROM in both model APIs, allowing a later call in the same power session to repair an interrupted access
* Close an already-open BROM window and select ROM before REG18 can enable a disabled gauge; even if the REG18 snapshot then fails, pulse the gauge MCU reset on the verified ROM selection instead of leaving a potentially partial SRAM runtime active
* Retry failed REG18 preparation during abnormal open-BROM cleanup and never restore the saved module state unless gauge-on/watchdog-off preparation and the recovery reset were both confirmed
* Use the hardware-guide REG17 `0x04 -> 0x00` gauge-MCU reset sequence so the independent bit3 RWAC full-gauge reset command cannot be replayed from a sampled register value
* Reorder `tg28_sw_set_poweroff_config`: the thresholds (REG24/REG27) now land before the PWRON>OFFLEVEL source enable (REG22), so a key pressed across the update cannot shut the board down under the old OFFLEVEL window
* Treat REG01 charge state 0 (trickle charge) as charging; only state 4 (charge done) and 5 (not charging) report `charging == false`
* Pause both the charger and the watchdog (REG18) for the whole battery-model download and restore the original REG18 value afterwards, so a 1 s watchdog cannot fire mid-download and leave a partial model

### Breaking changes

* `tg28_sw_get_power_off_source` now returns the raw REG21 byte (all eight latched power-off sources) instead of only the software power-off bit; use the `TG28_SW_POWER_OFF_SOURCE_*` bit names to decode it
* `tg28_sw_set_charge_voltage` no longer accepts 3900 mV (reserved code on the switch-charger variant)
* `tg28_sw_regulator_set_voltage(TG28_SW_DLDO1, ...)` rejects values above 3300 mV
* Widen `tg28_sw_poweroff_config_t::irqlevel_ms`/`offlevel_ms`/`onlevel_ms` from `uint8_t` to `uint16_t`: the selectable windows (1000-2500/4000-10000/128-2000 ms) never fit a byte, so `tg28_sw_set_poweroff_config` always returned ESP_ERR_INVALID_ARG and the getters truncated every value above 255 ms
* Remove the source argument from `tg28_sw_read_battery_model`; TG28 exposes one BROM parameter read window, while REGA2 bit4 only selects the model used by the gauge MCU after reset

## v0.3.0 - 2026-07-31

### Features

* Add CPUSLDO to the regulator enum (REG98, 500-1400 mV in 50 mV steps, enable REG90 bit6)
* Add `tg28_sw_set_ts_config` (REG50 TS mode, current-source switch, and 20/40/50/60 uA current selection)
* Add `tg28_sw_set_irq_enable`/`tg28_sw_get_irq_enable` covering the REG40-REG42 IRQ enable banks
* Add `tg28_sw_read_adc_channel` and `tg28_sw_set_adc_channel_enable`/`tg28_sw_get_adc_channel_enable` for the VBAT/TS/VBUS/VSYS/TDIE ADC channels (REG34-REG3D, REG30)
* Move the pure conversion helper declarations into `priv_include/tg28_sw_priv.h`

### Breaking changes

* Remove `tg28_sw_configure_external_fixed_ts`; use `tg28_sw_set_ts_config` instead
* Rename the internal interrupt register macros to the datasheet numbering (`TG28_SW_REG_INT_ENABLE0/1/2`, `TG28_SW_REG_INT_STATUS0`)

## v0.2.0 - 2026-07-31

### Features

* Add DLDO1/DLDO2 to the regulator enum in place of DCDC5
* Add `tg28_sw_set_input_current_limit`/`tg28_sw_get_input_current_limit` (REG16, discrete vendor levels 100-2000 mA)
* Add `tg28_sw_set_charge_voltage`/`tg28_sw_get_charge_voltage` (REG64, discrete vendor levels 3900-4400 mV)
* Add `tg28_sw_set_vindpm`/`tg28_sw_get_vindpm` (REG15, 4040-4680 mV in 80 mV steps)
