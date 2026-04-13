file(GLOB_RECURSE SRCS src/*.c)

if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.3")
    set(REQ esp_driver_i2c esp_driver_sdmmc esp_driver_gpio)
else()
    set(REQ driver)
endif()

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES ${REQ} spiffs fatfs
)
