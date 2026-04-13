if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.3")
    set(REQ esp_driver_i2c esp_driver_gpio esp_driver_i2s esp_driver_sdmmc esp_driver_sdspi)
    set(PRIV_REQ esp_driver_spi esp_driver_ledc)
else()
    set(REQ driver)
    set(PRIV_REQ "")
endif()

idf_component_register(
    SRCS "esp32_s3_korvo_1.c" "esp32_s3_korvo_1_idf5.c" "led_blink_defaults.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES ${REQ} fatfs esp_adc
    PRIV_REQUIRES ${PRIV_REQ} spiffs esp_psram
)
