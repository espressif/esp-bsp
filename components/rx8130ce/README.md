# RX8130CE RTC driver

This component provides an ESP-IDF C driver for the Epson RX8130CE real-time
clock. It uses an existing `i2c_master` bus and does not own the bus, interrupt
GPIO, backup supply, or board power policy.

The API covers the complete user-facing feature set of the application
manual:

- create and remove an RX8130CE device on an existing I2C bus;
- initialize every user register after a voltage-loss (`VLF`) event;
- validate, read, and set calendar time from 2000 through 2099;
- program and read back the minute/hour/day/weekday alarm and switch its
  `/IRQ` output (`AIE`);
- program, start, stop, pause, and read back the fixed-cycle wake-up timer,
  switch its `/IRQ` output (`TIE`), and select the Long-Timer accumulation
  mode (count on main supply, backup supply, or both);
- switch the time update interrupt output (`UIE`) and select its period
  (one second or one minute, `USEL`);
- select the FOUT clock output (32.768 kHz, 1024 Hz, 1 Hz, or off);
- read and write the battery-backed 4-byte user RAM (20h-23h);
- program the digital offset register for clock-accuracy calibration (30h,
  signed 3.05 ppm steps);
- decode retained voltage, reset, alarm, timer, and update flags;
- read and clear the three interrupt flags that can assert `/IRQ`;
- raw read/write access to the documented user registers (10h-23h, 30h,
  31h) for the few fields without a structured API (the power-detection
  tuning bits SMPTSEL/RSVSEL).

The alarm hardware compares either a day of month or one weekday; matching
multiple weekdays at once (supported by the hardware's weekday bit map) is
not expressible through `rx8130ce_alarm_t` and can be reached through the
raw register accessors. `rx8130ce_delete()` must not race with calls from
other tasks: finish or join every concurrent user before deleting.

The backup supply policy is a board-level choice made at device creation.
The driver always sets `INIEN` (automatic supply switchover is enabled),
while `CHGEN` follows `rx8130ce_config_t.backup_charge_enable`: keep the
default `false` for a primary (non-rechargeable) backup cell so charging
stays off, or set it to `true` when the board carries a rechargeable backup
source (secondary cell or supercapacitor) that must be charged from VDD.
When charging is enabled, `rx8130ce_config_t.backup_charge_cutoff` selects
the full-charge voltage that stops charging (2.92 V, 3.02 V, or 3.08 V,
default 3.02 V); the cutoff-less datasheet setting is not selectable, and
the board must limit the charge current externally to 40 mA max (application
manual, switching-element characteristics). Independently,
`rx8130ce_config_t.backup_voltage_low_detect` (default `true`) sets `VBLFE`
so the low-backup-voltage flag can assert even while charging stays
disabled. The
configured policy is applied after normal power-up and as part of the `VLF=1`
full-register initialization. A successful runtime charge-policy change is
also retained for a later recovery. After `VLF=1`, the driver waits for
oscillator startup and initializes all documented user registers. When this
happens during `rx8130ce_create()`, the calendar starts at the explicit
recovery epoch 2000-01-01 00:00:00 (Saturday), and the application should set
real time before relying on timestamps. Runtime recovery through
`rx8130ce_set_time()` uses the caller's requested time instead.

## Basic use

```c
#include "driver/i2c_master.h"
#include "rx8130ce.h"

rx8130ce_handle_t rtc = NULL;
rx8130ce_config_t config = RX8130CE_CONFIG_DEFAULT();

ESP_ERROR_CHECK(rx8130ce_create(i2c_bus, &config, &rtc));

rx8130ce_time_t time;
rx8130ce_status_t status;
ESP_ERROR_CHECK(rx8130ce_get_time(rtc, &time, &status));

ESP_ERROR_CHECK(rx8130ce_delete(rtc));
```

The fixed 7-bit address is `0x32`. `RX8130CE_CONFIG_DEFAULT()` selects a
400 kHz I2C clock, keeps backup charging off with the 3.02 V charge cutoff,
and leaves backup low-voltage detection on. Passing `NULL` as the config
selects the same defaults.

## Setting time

`rx8130ce_set_time()` validates the complete calendar value, sets the device
`STOP` bit, writes all seven calendar registers in one transaction, and
restarts the counter. Weekday uses `0` for Sunday through `6` for Saturday.

If `VLF` is already set or appears during the update, all register contents
are invalid. The call waits for oscillator startup and performs a complete
recovery with the requested calendar and the most recent backup-charge policy;
only after the counter runs again is `VLF` cleared. Alarm, timer, FOUT/update
mode, digital offset, and user RAM are reset to safe defaults and must be
configured again after the call succeeds.

The RTC application manual warns that voltage detection and supply switching
are affected while `STOP` is set. The driver therefore keeps that interval to
the calendar write and always attempts to restore the original control value
if an intermediate transaction fails.

## Alarms

`rx8130ce_set_alarm()` programs the compare fields selected by the `*_en`
flags of `rx8130ce_alarm_t`; each flag maps to the active-low `AE` bit of the
matching alarm register, so a disabled field never blocks a match. With every
field disabled the alarm fires once per minute. Day-of-month and weekday share
register 19h (selected by the `WADA` bit), so `day_en` and `weekday_en` are
mutually exclusive. Weekday uses `0` for Sunday through `6` for Saturday.

While the settings change, the driver holds `AIE` cleared as recommended by
the application manual, clears any latched `AF`, and restores the previous
`AIE` state. `rx8130ce_alarm_irq_enable()` switches the `/IRQ` output without
touching the compare settings, and the latched alarm flag is cleared through
`rx8130ce_get_and_clear_interrupts()`.

## Wake-up timer

`rx8130ce_set_timer()` programs the fixed-cycle timer: the 16-bit down-counter
preset (registers 1Ah-1Bh, 1-65535 counts) and the source clock selected by
`rx8130ce_timer_source_t`, from 4096 Hz (244.14 us per count) down to
1/3600 Hz (one hour per count). The driver holds the timer-enable bit cleared while the
settings change, as the application manual requires, and clears any latched
`TF`. Programming a new timer also clears a prior `rx8130ce_timer_pause(true)`
state. With `timer.enable` set, the countdown then starts from the preset; with
it cleared, the timer stays stopped and the counter registers hold the preset
for readback. For the 4096 Hz, 64 Hz, and 1 Hz source clocks the
first countdown can be up to one period of the selected source clock
shorter than configured; for the 1/60 Hz and 1/3600 Hz source clocks it can
be up to one second shorter (application manual 14.2.2).

`rx8130ce_get_timer()` reads the settings back. While the timer runs, `count`
is the live down-count and is not latched during the read, so read twice until
two reads agree, or stop the timer first, for an exact value. Before the
timer is first programmed, `count` can read back as `0`.

`rx8130ce_timer_irq_enable()` switches the timer interrupt on the `/IRQ` pin
(`TIE`) without touching the countdown, and `rx8130ce_update_irq_enable()`
does the same for the time update interrupt (`UIE`). The latched timer flag
is cleared through `rx8130ce_get_and_clear_interrupts()`.

## Status and interrupts

`status.time_valid` is false while the `VLF` flag is set. A successful I2C read
does not by itself prove that retained time is valid.

`status.backup_voltage_low` reports only the `VBLF` flag (low backup
battery). While charging is disabled, that flag can only assert when the
detection is enabled through `rx8130ce_config_t.backup_voltage_low_detect`
(default on, `VBLFE` bit).

`status.backup_battery_full` reports `VBFF` (backup battery charge threshold
reached); it never sets while charging stays disabled on a primary cell.

`rx8130ce_get_and_clear_interrupts()` returns the complete flag register and
clears only the observed `UF`, `TF`, and `AF`. Its W0C writes preserve every
other flag, including a different event latched after the read transaction.
Voltage-loss, backup, and reset evidence is left unchanged. GPIO interrupt registration belongs to the board or
application because the RX8130CE `/IRQ` output may be shared with another
device.

Raw multi-register reads are restricted to one 16-byte auto-increment page.
The device wraps `1Fh` to `10h` (and similarly for the other pages), so callers
must issue a new read instead of crossing `1Fh`, `2Fh`, or `3Fh`.

## References

- [RX8130CE product page](https://www.epsondevice.com/crystal/en/products/rtc/rx8130ce.html)
- [RX8130CE application manual](https://download.epsondevice.com/td/pdf/app/RX8130CE_en.pdf)

No datasheet is copied into the component archive.
