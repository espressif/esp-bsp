# LVGL ESP Portation

This component helps with using LVGL with Espressif's LCD and touch drivers. It can be used with any project with LCD display. 

## Features
* Initialization of the LVGL
    * Create task and timer
    * Handle rotating
* Add/remove display (using [`esp_lcd`](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/lcd.html))
* Add/remove touch input (using [`esp_lcd_touch`](https://github.com/espressif/esp-bsp/tree/master/components/lcd_touch))

## Usage

### Initialization
``` c
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    esp_err_t err = lvgl_port_init(&lvgl_cfg);
```

### Add screen

Add an LCD screen to the LVGL. It can be called multiple times for adding multiple LCD screens. 

This part is necessary only in IDF 5.0 and older:

``` c
    static lv_disp_t * disp;
    /* The component calls esp_lcd_panel_draw_bitmap API for send data to the screen. There must be called 
    lvgl_port_flush_ready(disp) after each transaction to display. The best way is to use on_color_trans_done 
    callback from esp_lcd IO config structure. */
    static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
    {
        lv_disp_t ** disp = (lv_disp_t **)user_ctx;
        lvgl_port_flush_ready(*disp);
        return false;
    }
    
    /* LCD IO */
    esp_lcd_panel_io_spi_config_t io_config =
	{
		...
		.on_color_trans_done = notify_lvgl_flush_ready,
		.user_ctx = &disp
	};
	esp_lcd_panel_io_handle_t io_handle = NULL;
	ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t) 1, &io_config, &io_handle));

    /* LCD driver initialization */
    ...
    esp_lcd_panel_handle_t lcd_panel_handle;
    esp_err_t err = esp_lcd_new_panel_st7789(io_handle, &panel_config, &lcd_panel_handle);
```

Main part of the code (IDF version independent):

``` c
    /* Add LCD screen */
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = lcd_panel_handle,
        .buffer_size = DISP_WIDTH*DISP_HEIGHT,
        .double_buffer = true,
        .hres = DISP_WIDTH,
        .vres = DISP_HEIGHT,
        .monochrome = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
        }
    };
    disp = lvgl_port_add_disp(&disp_cfg);
```

**Note:** The screens added in this function are not removed in `lvgl_port_deinit`. They must be removed by `lvgl_port_remove_disp` before deinitialization. Otherwise, there can be memory leaks!

### Add touch input

Add touch input to the LVGL. It can be called more times for adding more touch inputs. 
``` c
    /* Touch driver initialization */
    ...
    esp_lcd_touch_handle_t tp;
    esp_err_t err = esp_lcd_touch_new_i2c_gt911(io_handle, &tp_cfg, &tp);

    /* Add touch input (for selected screen) */
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = disp_spi,
        .handle = tp,
    };
    lvgl_port_add_touch(&touch_cfg);
```

**Note:** The inputs added in this function are not removed in `lvgl_port_deinit`. They must be removed by `lvgl_port_remove_touch` before deinitialization. Otherwise, there can be memory leaks!

### LVGL API usage

Every LVGL calls must be protected with these lock/unlock commands:
``` c
	/* Wait for the other task done the screen operation */
    lvgl_port_lock(0);
    ...
    lv_obj_t * screen = lv_disp_get_scr_act(disp);
    lv_obj_t * obj = lv_label_create(screen);
    ...
    /* Screen operation done -> release for the other task */
    lvgl_port_unlock();
```

### Rotating screen
``` c
    lv_disp_set_rotation(disp, LV_DISP_ROT_90);
```

**Note:** During the rotating, the component call [`esp_lcd`](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/lcd.html) API.
