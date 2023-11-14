#IDF version is less than IDF5.0
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_LESS "5.0")
    set(SRC_VER "esp32_s3_usb_otg_idf4.c")
    set(REQ "")
else()
    set(SRC_VER "esp32_s3_usb_otg_idf5.c")
    set(REQ esp_adc)
endif()

idf_component_register(
    SRCS "esp32_s3_usb_otg.c" ${SRC_VER}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver esp_lcd
    PRIV_REQUIRES fatfs usb spiffs ${REQ}
)
