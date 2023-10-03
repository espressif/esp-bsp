#IDF version is less than IDF5.0
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_LESS "5.0")
    set(SRC_VER "esp32_s2_kaluga_kit_idf4.c")
else()
    set(SRC_VER "esp32_s2_kaluga_kit_idf5.c")
endif()

idf_component_register(
    SRCS "esp32_s2_kaluga_kit.c" ${SRC_VER}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver
    PRIV_REQUIRES esp_lcd spiffs
)
