idf_component_register(
    SRCS "src/esp32_p4_eye.c" "src/led_blink_defaults.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_i2s esp_lcd esp_driver_ledc 
    PRIV_REQUIRES esp_psram spiffs fatfs esp_driver_sdspi esp_driver_sdmmc 
)
