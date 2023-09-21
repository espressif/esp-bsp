# SSD1681 e-Paper Driver

[![Component Registry](https://components.espressif.com/components/espressif/esp_lcd_ssd1681/badge.svg)](https://components.espressif.com/components/espressif/esp_lcd_ssd1681)

Implementation of the SSD1681 e-Paper controller with esp_lcd component.

| LCD controller | Communication interface | Component name | Link to datasheet |
| :------------: | :---------------------: | :------------: | :---------------: |
| SSD1681        | SPI                     | esp_lcd_ssd1681| [Specification](https://cdn-learn.adafruit.com/assets/assets/000/099/573/original/SSD1681.pdf) |

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependency`, e.g.

```bash
compote manifest add-dependency espressif/esp_lcd_ssd1681==0.1.0
```

Alternatively, you can create `idf_component.yml`. More is in [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## Additional Description for `esp_lcd_panel_interface`

The SSD1681 e-Paper driver fully implements the `esp_lcd_panel_interface` interface. Due to the specificity of this hardware, please carefully read the description for the following API.

### `esp_lcd_panel_mirror`

Original function prototype:
```c
/**
 * @brief Mirror the LCD panel on specific axis
 *
 * @note Combined with `esp_lcd_panel_swap_xy()`, one can realize screen rotation
 *
 * @param[in] panel LCD panel handle, which is created by other factory API like `esp_lcd_new_panel_st7789()`
 * @param[in] mirror_x Whether the panel will be mirrored about the x axis
 * @param[in] mirror_y Whether the panel will be mirrored about the y axis
 * @return
 *          - ESP_OK on success
 *          - ESP_ERR_NOT_SUPPORTED if this function is not supported by the panel
 */
esp_err_t esp_lcd_panel_mirror(esp_lcd_panel_handle_t panel, bool mirror_x, bool mirror_y);
```
Please note that `mirror_y` has to be false if you enabled the `non_copy_mode` when constructing the panel, otherwise the function will return `ESP_ERR_INVALID_ARG`.

### `esp_lcd_panel_swap_xy`

Original function prototype:
```c
/**
 * @brief Swap/Exchange x and y axis
 *
 * @note Combined with `esp_lcd_panel_mirror()`, one can realize screen rotation
 *
 * @param[in] panel LCD panel handle, which is created by other factory API like `esp_lcd_new_panel_st7789()`
 * @param[in] swap_axes Whether to swap the x and y axis
 * @return
 *          - ESP_OK on success
 *          - ESP_ERR_NOT_SUPPORTED if this function is not supported by the panel
 */
esp_err_t esp_lcd_panel_swap_xy(esp_lcd_panel_handle_t panel, bool swap_axes);
```
Please note that `swap_axes` has to be false if you enabled the `non_copy_mode` when constructing the panel, otherwise the function will return `ESP_ERR_INVALID_ARG`.

### `esp_lcd_panel_draw_bitmap`

Original function prototype:
```c
/**
 * @brief Draw bitmap on LCD panel
 *
 * @param[in] panel LCD panel handle, which is created by other factory API like `esp_lcd_new_panel_st7789()`
 * @param[in] x_start Start index on x-axis (x_start included)
 * @param[in] y_start Start index on y-axis (y_start included)
 * @param[in] x_end End index on x-axis (x_end not included)
 * @param[in] y_end End index on y-axis (y_end not included)
 * @param[in] color_data RGB color data that will be dumped to the specific window range
 * @return
 *          - ESP_OK on success
 */
esp_err_t esp_lcd_panel_draw_bitmap(esp_lcd_panel_handle_t panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
```
Please note that this function will draw the bitmap but do not refresh the panel. You have to call the `epaper_panel_refresh_screen()` manually to refresh the bitmaps to the panel.


### `esp_lcd_panel_disp_on_off`

Original function prototype:
```c
/**
 * @brief Turn on or off the display
 *
 * @param[in] panel LCD panel handle, which is created by other factory API like `esp_lcd_new_panel_st7789()`
 * @param[in] on_off True to turns on display, False to turns off display
 * @return
 *          - ESP_OK on success
 *          - ESP_ERR_NOT_SUPPORTED if this function is not supported by the panel
 */
esp_err_t esp_lcd_panel_disp_on_off(esp_lcd_panel_handle_t panel, bool on_off);
```
Call with parameter `on_off` set to false will have the e-paper panel enter sleep mode. BUSY pin will stay HIGH in sleep mode and a `esp_lcd_panel_init()` call is needed to resume the panel. Call with parameter `on_off` set to true will load the panel built-in waveform LUT, it is useful if you had set a custom waveform LUT.

## Service Life Optimization

- The screen should not be powered on for extended periods of time. Please use the `disp_on_off` API to put the screen into sleep mode or cut down the power when the screen is not refreshing.
- Do not refresh the screen at maximum speed for a long time. E-Paper panels are not made to show dynamic contents quickly. The refresh interval should be at least 3 minutes if you want to refresh the screen continuously.