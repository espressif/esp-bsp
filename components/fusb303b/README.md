# FUSB303B USB Type-C controller driver

This component provides an ESP-IDF C driver for the onsemi FUSB303B autonomous
USB Type-C port controller. It uses an existing `i2c_master` bus and leaves the
enable GPIO, VBUS switch, USB host stack, and board power sequence to its
caller.

The driver supports both I2C addresses selected by the `ADDR/ORIENT` pin:

| Pin state | 7-bit address |
|---|---:|
| Low | `0x21` |
| High | `0x31` |

GPIO mode, selected when `ADDR/ORIENT` is floating, does not expose an I2C
device and cannot be used by this component.

## Basic use

```c
#include "driver/i2c_master.h"
#include "fusb303b.h"

fusb303b_handle_t controller = NULL;
fusb303b_config_t config = FUSB303B_CONFIG_DEFAULT();

ESP_ERROR_CHECK(fusb303b_create(i2c_bus, &config, &controller));
ESP_ERROR_CHECK(fusb303b_set_role(controller,
                                  FUSB303B_ROLE_DRP,
                                  FUSB303B_CURRENT_DEFAULT));
ESP_ERROR_CHECK(fusb303b_set_enabled(controller, true));

fusb303b_status_t status;
ESP_ERROR_CHECK(fusb303b_get_status(controller, &status, false));

ESP_ERROR_CHECK(fusb303b_delete(controller));
```

`fusb303b_create()` verifies the device version and type registers but does not
change role, interrupt, or enable settings. In I2C mode the caller must assert
the external active-low enable input (`EN_N`), wait
`FUSB303B_ENABLE_TO_I2C_DELAY_MS` before the first transfer, configure the
role with `fusb303b_set_role()`, and only then enable the autonomous state
machine with `fusb303b_set_enabled()`: enabling first would briefly run the
strap-sampled role.

## Role and VBUS ownership

`fusb303b_set_role()` selects sink, source, dual-role, or disabled operation.
The current argument programs the source advertisement; it never enables a
board VBUS regulator or boost converter. A board implementation must enable
source power only after its CC, protection, and power-path requirements are
satisfied.

Leaving the disabled state requires clearing `MANUAL.DISABLED` before changing
the Portrole register. The driver performs that sequence and preserves
unrelated role and timing fields.

## Status and interrupts

`fusb303b_get_status()` reads connection, type, and both interrupt registers
in one snapshot and reports the identity registers verified and cached at
create time. Passing `clear_interrupts=true` writes the values back to the
two write-one-to-clear registers after the snapshot has been decoded.

The global interrupt mask is controlled separately by
`fusb303b_set_global_interrupt_mask()`, and the per-event Mask (0Eh) and
Mask1 (0Fh) registers are programmed with `fusb303b_set_interrupt_mask()`.
Masking only keeps an event off the INT_N pin; the interrupt bit itself
still sets. `fusb303b_get_and_clear_interrupts()` is task-context only (device
lock plus synchronous I2C): a GPIO ISR on INT_N must only notify a task.
GPIO interrupt registration and USB data-role switching remain
board or application policy. `fusb303b_read_register()` and
`fusb303b_write_register()` give board implementations raw register access
for configuration the typed API does not cover; regular applications should
not need them.

## Wiring notes

- `INT_N` is open-drain: add a board pull-up.
- `EN_N` is active low; after asserting it, wait
  `FUSB303B_ENABLE_TO_I2C_DELAY_MS` (100 ms) before the first I2C access.
- The I2C clock is capped at 400 kHz (`FUSB303B_I2C_CLOCK_HZ`); higher
  `scl_speed_hz` values are rejected by `fusb303b_create()`.
- VBUS_DET, CC1 and CC2 routing follow the datasheet reference design and
  stay board responsibilities.

## Reference

- [FUSB303B datasheet](https://www.onsemi.com/pdf/datasheet/fusb303b-d.pdf)

No datasheet is copied into the component archive.
