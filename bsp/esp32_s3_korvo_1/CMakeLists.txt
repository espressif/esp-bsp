#IDF version is less than IDF5.0
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_LESS "5.0")
    set(SRC_VER "esp32_s3_korvo_1_idf4.c")
    set(REQ spiffs)
else()
    set(SRC_VER "esp32_s3_korvo_1_idf5.c")
    set(REQ spiffs esp_adc)
endif()

idf_component_register(
    SRCS "esp32_s3_korvo_1.c" ${SRC_VER} "led_blink_defaults.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES ${REQ}
)
