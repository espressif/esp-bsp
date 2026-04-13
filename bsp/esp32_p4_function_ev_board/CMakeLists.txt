if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.3")
    set(REQ esp_driver_gpio esp_driver_i2s esp_driver_sdmmc esp_driver_sdspi esp_driver_i2c)
    set(PRIV_REQ esp_driver_spi esp_driver_ledc)
else()
    set(REQ driver)
    set(PRIV_REQ "")
endif()

if(${IDF_VERSION_MAJOR} LESS 6)
    list(APPEND PRIV_REQ usb)
endif()

idf_component_register(
    SRCS "esp32_p4_function_ev_board.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES ${REQ} fatfs
    PRIV_REQUIRES ${PRIV_REQ} esp_lcd spiffs esp_psram
)
