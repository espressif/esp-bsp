# ChangeLog

## v0.4.0 - 2026-08-22

### Features

* Complete the datasheet feature coverage: FOUT clock output selection (`rx8130ce_set_fout`/`rx8130ce_get_fout`, 32.768 kHz/1024 Hz/1 Hz/OFF), time update interrupt period selection (`rx8130ce_set_update_irq_mode`/`rx8130ce_get_update_irq_mode`, second or minute), Long-Timer accumulation mode (`rx8130ce_set_timer_count_mode`/`rx8130ce_get_timer_count_mode`) and timer pause (`rx8130ce_timer_pause`), battery-backed user RAM access (`rx8130ce_write_user_ram`/`rx8130ce_read_user_ram`, 20h-23h), and digital offset calibration (`rx8130ce_set_digital_offset`/`rx8130ce_get_digital_offset` with the pure `rx8130ce_digital_offset_encode`/`rx8130ce_digital_offset_decode` helpers, 30h)
* Add raw register fallback accessors `rx8130ce_read_registers`/`rx8130ce_write_register`, restricted to the documented user registers (10h-23h, 30h, 31h) per appman 13.2.1 note *6; these cover the power-detection tuning fields (SMPTSEL/RSVSEL) that have no structured API
* Add `rx8130ce_config_t.backup_charge_cutoff` (`rx8130ce_charge_cutoff_t`: 2.92 V/3.02 V/3.08 V, default 3.02 V): a defined `BFVSEL` full-charge cutoff is programmed together with the charge policy in both configuration paths, and the cutoff-less 11b encoding is rejected in `rx8130ce_create()`; boards that enable charging must still limit the charge current externally to 40 mA max (application manual)
* Add `rx8130ce_config_t.backup_voltage_low_detect` (default `true`), programming the `VBLFE` bit (31h, appman Table 42) in both configuration paths so `status.backup_voltage_low` can assert even while charging stays disabled

### Bug Fixes

* Add a per-device mutex so concurrent calls from application tasks and the shared interrupt service cannot interleave the read-modify-write sequences of `set_time`/`set_alarm`/`set_timer` and the IRQ helpers, matching the concurrency model of the tg28_sw driver
* Reject `scl_speed_hz` above the 400 kHz device limit in `rx8130ce_create()` instead of accepting an out-of-spec bus speed
* Restore the previous AIE state in `rx8130ce_set_alarm()` on every exit path: a failed write no longer leaves the alarm interrupt permanently gated off after it was disabled in CONTROL0 for the register update window
* Make `rx8130ce_set_time()` recover a runtime `VLF` with the same complete register initialization used at creation, using the requested calendar and latest charge policy; normal time updates no longer touch FLAGS, and full recovery clears `VLF` only after the counter restart succeeds
* Clear W0C flags with target masks instead of stale read-modify-write images, preserving unrelated hardware events that arrive between I2C transactions
* Clear `TSTP` when programming a new fixed-cycle timer so `timer.enable=true` resumes a previously paused countdown, and gate `UIE` while changing the update interrupt period
* Validate the complete register window in `rx8130ce_read_registers()` against the user registers (10h-23h, 30h, 31h) and reject reads that cross a 16-byte hardware wrap boundary
* Guarantee the 35 ms backup recovery and oscillator-start waits despite FreeRTOS tick rounding and scheduler phase

## v0.3.0 - 2026-07-31

### Features

* Make the backup battery charge policy configurable: new `rx8130ce_config_t.backup_charge_enable` field (default `false`, keeping the previous primary-cell behavior); `INIEN` is always set and `CHGEN` now follows the configured policy in both the normal power-up path and the `VLF=1` full-register initialization
* Accept `NULL` as the config of `rx8130ce_create`, falling back to `RX8130CE_CONFIG_DEFAULT()`
* Add fixed-cycle wake-up timer support: `rx8130ce_timer_t`, `rx8130ce_timer_source_t`, `rx8130ce_timer_is_valid`, `rx8130ce_timer_encode`, `rx8130ce_set_timer`, `rx8130ce_get_timer`, and `rx8130ce_timer_irq_enable` (`TIE`)
* Add `rx8130ce_update_irq_enable` for the time update interrupt (`UIE`)

### Fixes

* Declare the missing `freertos` private dependency in CMakeLists.txt

## v0.2.0 - 2026-07-31

### Features

* Add alarm support: `rx8130ce_alarm_t`, `rx8130ce_set_alarm`, `rx8130ce_get_alarm`, and `rx8130ce_alarm_irq_enable`
