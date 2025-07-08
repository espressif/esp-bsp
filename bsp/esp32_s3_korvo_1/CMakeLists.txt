#IDF version is less than IDF5.0
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_LESS "5.0")
    set(SRC_VER "esp32_s3_korvo_1_idf4.c")
    set(REQ driver spiffs fatfs)
else()
    set(SRC_VER "esp32_s3_korvo_1_idf5.c")
    set(REQ driver spiffs fatfs esp_adc vfs)
endif()

idf_component_register(
    SRCS "esp32_s3_korvo_1.c" ${SRC_VER} "led_blink_defaults.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    PRIV_REQUIRES esp_psram
    REQUIRES ${REQ}
)
