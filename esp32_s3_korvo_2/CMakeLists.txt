#IDF version is less than IDF5.0
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_LESS "5.0")
    set(SRC_VER "esp32_s3_korvo_2_idf4.c")
    set(REQ driver esp_lcd)
else()
    set(SRC_VER "esp32_s3_korvo_2_idf5.c")
    set(REQ driver esp_lcd esp_adc)
endif()

idf_component_register(
    SRCS "esp32_s3_korvo_2.c" ${SRC_VER}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES ${REQ}
    PRIV_REQUIRES spiffs fatfs
)
