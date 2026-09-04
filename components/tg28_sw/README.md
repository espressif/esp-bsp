# TG28 switch-charger driver

This component provides an I2C driver for the switch-charger variant of the
TG28 power-management IC. It owns only the I2C device handle; the application
or board support package remains responsible for the I2C bus and for deciding
which regulator powers each peripheral.

The implemented interface covers the switch-charger variant's charger,
fuel-gauge, regulator, load-switch, ADC, and interrupt control:

- chip identification;
- VBUS, battery, charge-state, voltage, and state-of-charge readings;
- raw power-on source and full REG21 power-off source bytes (named with
  `TG28_SW_POWER_OFF_SOURCE_*`), plus explicit power-key interrupt
  configuration;
- exact REG62 charge-current control and REG50 TS pin configuration
  (battery NTC or external fixed input, with current-source control);
- REG61 precharge-current and REG63 termination-current/enable control;
- per-bank (REG40-REG42) and per-bit IRQ enable control with 23 named
  interrupt sources (`tg28_sw_irq_t`);
- 14-bit ADC channel reads for VBAT, TS, VBUS, VSYS, and TDIE with
  REG30 channel-enable management;
- verified per-boot download of a battery-specific REGA1 fuel-gauge model
  (exactly `TG28_SW_BATTERY_MODEL_SIZE` bytes);
- DCDC1-DCDC4, ALDO1-ALDO4, BLDO1-BLDO2, CPUSLDO, and DLDO1-DLDO2 voltage
  and enable control (the switch-charger variant has no DCDC5 rail);
- DC1SW/DC4SW load-switch control for boards whose OTP straps the DLDO
  pins as switches instead of programmable LDOs;
- discrete REG16 input-current-limit and REG64 charge-voltage control;
- exact REG15 VINDPM threshold control;
- REG1A low-battery warning thresholds;
- charger, fuel-gauge, and watchdog module enables (REG18) with the full
  watchdog configuration (REG19 period and expiry action) and feeding;
- battery temperature-sense thresholds and hystereses (REG52-REG57) and
  JEITA standard configuration (REG58-REG5B), including the external-fixed
  TS comparison code (REG5C/REG5D) for batteries without an NTC;
- thermal-regulation threshold (REG65), charger safety timers (REG67),
  battery detection enable (REG68), and CHGLED pin control (REG69);
- backup/button-battery charging (REG18 bit2, REG6A termination voltage);
- minimum system voltage (REG14);
- power-off policy group (REG22/REG23/REG24/REG27) and sleep/wakeup control
  (REG26) - both carry board-locking risk and are documented with warnings;
- GPIO1 output (REG1B), DCDC operating modes (force CCM, DVM ramp,
  per-rail force PWM, spread spectrum, REG80/REG81), whole-PMIC software
  reset (REG10 bit1), and the BATFET off-state keep for ship mode (REG12);
- interrupt status read and write-one-to-clear handling, to be called from
  task context only (the PMIC IRQ GPIO ISR must just notify a task), with
  condition-latched bits re-asserting until the condition clears;
- raw register read (`tg28_sw_read_registers`) and single-register write
  (`tg28_sw_write_register`) escape hatches for anything not yet wrapped.

Voltage setters reject values that cannot be represented exactly. This keeps a
board power sequence from silently selecting a different voltage.
Charge-current setters use the same rule: 0-200 mA is selectable in 25 mA
steps, followed by 300-1500 mA in 100 mA steps. The input current limit
accepts only 100/500/900/1000/1500/2000 mA, the charge voltage only
4000/4100/4200/4350/4400 mV, and the VINDPM threshold only exact 80 mV
steps across the full 3880-5080 mV hardware range.

```c
#include "tg28_sw.h"

tg28_sw_handle_t pmic = NULL;
tg28_sw_config_t config = TG28_SW_CONFIG_DEFAULT();

ESP_ERROR_CHECK(tg28_sw_create(i2c_bus, &config, &pmic));

tg28_sw_status_t status;
ESP_ERROR_CHECK(tg28_sw_get_status(pmic, &status));

ESP_ERROR_CHECK(tg28_sw_delete(pmic));
```

Set `config.battery_model` and `config.battery_model_size` to download the
supplier-generated model on every device creation (normally once per boot).
The model is battery-specific and is deliberately not guessed by this
component. It may also be downloaded explicitly with
`tg28_sw_program_battery_model()`.

The driver follows the supplier register description and Linux reference
driver. Electrical behavior must still be verified on the target board.
