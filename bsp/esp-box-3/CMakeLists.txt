idf_component_register(
    SRCS "esp-box-3.c" "esp-box-3_idf5.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES esp_driver_i2s esp_driver_gpio esp_driver_sdmmc spiffs
    PRIV_REQUIRES fatfs esp_lcd esp_driver_spi esp_driver_i2c
)
