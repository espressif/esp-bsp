idf_component_register(
    SRCS "esp32_s2_kaluga_kit.c" "esp32_s2_kaluga_kit_idf5.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES esp_lcd esp_driver_gpio esp_driver_i2s driver
    PRIV_REQUIRES spiffs esp_driver_i2c esp_driver_spi esp_psram
)
