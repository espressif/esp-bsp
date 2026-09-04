# ChangeLog

## v1.2.0 - 2026-08-22

### Features

* Add explicit monitor-mode API: `esp_lcd_touch_cst820_enter_monitor_mode()` / `esp_lcd_touch_cst820_exit_monitor_mode()` for the standby tier (~10 uA) in which a touch pulses INT and wakes the host. The previously planned deep-sleep API was dropped: the candidate sleep commands reconstructed from public CST816-family sources (`0xA5 <- 0x03`, `0xE5 <- 0x03`) were verified on hardware to leave the controller answering I2C unchanged
* Add raw register accessors `esp_lcd_touch_cst820_read_reg()` / `esp_lcd_touch_cst820_write_reg()`: the public CST820 datasheet does not document the register map, so features without a structured API (gesture report bytes, auto-standby control, multi-key reports) can be reached directly
* Test apps: add a hardware-free NULL-handle argument check and a monitor-mode light-sleep touch wake-up case

### Documentation

* Document the datasheet standby-mode behavior (auto-standby 2 s after the last touch, INT wake of the host) and a typical board low-power sequence
* Document the datasheet feature blocks that have no structured API because their registers or report bytes are not published (single-tap and standby gestures, auto-standby register control, multi-key reports); the raw register accessors are the supported fallback

## v1.1.1 - 2026-07-31

### Bug Fixes

* Clamp the `get_track_id` loop to `CONFIG_ESP_LCD_TOUCH_MAX_POINTS` so a larger caller-supplied array size cannot read past the internal coordinate buffer

### Documentation

* Remove the Component Registry badge from the README until the component is published
* Document the test-board wiring behind the hardcoded test app pins and clarify the H_RES/V_RES defines

## v1.1.0 - 2026-07-31

### Features

* Add deep-sleep support: `enter_sleep`/`exit_sleep` through the 0xA5 sleep-mode register, with wake-up via a hardware reset cycle

## v1.0.0

* Initial release: CST820 capacitive touch controller driver for `esp_lcd_touch`
