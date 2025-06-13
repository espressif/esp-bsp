## :pager: Display and Touch

### Initialization

ESP-BSP provides two ways to initialize the **display**, **touch** and **LVGL**.

Simple method:

```
/* Initialize display, touch, and LVGL */
lv_display_t display = bsp_display_start();
```

Configurable method:

```
bsp_display_cfg_t cfg = {
    .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),   /* See LVGL Port for more info */
    .buffer_size = BSP_LCD_V_RES * BSP_LCD_H_RES,   /* Screen buffer size in pixels */
    .double_buffer = true,                          /* Allocate two buffers if true */
    .flags = {
        .buff_dma = true,                           /* Use DMA-capable LVGL buffer */
        .buff_spiram = false,                       /* Allocate buffer in PSRAM if true */
    }
};
cfg.lvgl_port_cfg.task_stack = 10000;   /* Example: change LVGL task stack size */
/* Initialize display, touch, and LVGL */
lv_display_t display = bsp_display_start_with_config(&cfg);
```

After initialization, you can use the [LVGL](https://docs.lvgl.io/master/) API or [LVGL Port](../components/esp_lvgl_port/README.md) API.

### Initialization without LVGL - NoGLIB BSP

To initialize the LCD without LVGL, use:

```
esp_lcd_panel_handle_t panel_handle;
esp_lcd_panel_io_handle_t io_handle;
const bsp_display_config_t bsp_disp_cfg = {
    .max_transfer_sz = (BSP_LCD_H_RES * 100) * sizeof(uint16_t),
};
BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new(&bsp_disp_cfg, &panel_handle, &io_handle));
```

To initialize the LCD touch without LVGL, use:

```
esp_lcd_touch_handle_t tp;
bsp_touch_new(NULL, &tp);
```

After initialization, you can use the [ESP-LCD](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/lcd/index.html) API and [ESP-LCD Touch](../components/lcd_touch/README.md) API.

### Set Brightness

```
/* Set display brightness to 100% */
bsp_display_backlight_on();

/* Set display brightness to 0% */
bsp_display_backlight_off();

/* Set display brightness to 50% */
bsp_display_brightness_set(50);
```

> [!NOTE]
> Some boards do not support changing brightness. They return an `ESP_ERR_NOT_SUPPORTED` error.

### LVGL API Usage (only when initialized with LVGL)

All LVGL calls must be protected using lock/unlock:

```
/* Wait until other tasks finish screen operations */
bsp_display_lock(0);
...
lv_obj_t * screen = lv_disp_get_scr_act(disp_handle);
lv_obj_t * obj = lv_label_create(screen);
...
/* Unlock after screen operations are done */
bsp_display_unlock();
```

### Screen rotation (only when initialized with LVGL)

```
bsp_display_lock(0);
/* Rotate display to 90 */
bsp_display_rotate(display, LV_DISPLAY_ROTATION_90);
bsp_display_unlock();
```

> [!NOTE]
> Some LCDs do not support hardware rotation and instead use software rotation, which consumes more memory.

### Available constants

Constants like screen resolution, pin configuration, and other options are defined in the BSP header files (`{bsp_name}.h`, `display.h`, `touch.h`).
Below are some of the most relevant predefined constants:

- `BSP_LCD_H_RES` - Horizontal resolution in pixels
- `BSP_LCD_V_RES` - Vertical resolution in pixels
- `BSP_LCD_SPI_NUM` - SPI bus used by the LCD (if applicable)

