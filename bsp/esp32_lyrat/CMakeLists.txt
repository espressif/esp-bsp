#IDF version is less than IDF5.0
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_LESS "5.0")
    set(SRC_VER "esp32_lyrat_idf4.c")
else()
    set(SRC_VER "esp32_lyrat_idf5.c")
endif()

idf_component_register(
    SRCS "esp32_lyrat.c" ${SRC_VER}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    PRIV_REQUIRES esp_psram
    REQUIRES driver spiffs fatfs 
)
