idf_component_register(
    SRCS "src/esp32_c3_lcdkit.c" "src/led_blink_defaults.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES esp_driver_gpio esp_driver_i2s spiffs fatfs vfs 
    PRIV_REQUIRES esp_psram esp_lcd esp_driver_ledc 
)