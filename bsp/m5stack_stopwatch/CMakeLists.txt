
file(GLOB_RECURSE SRC src/*.c)

if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.3")
    set(REQ esp_driver_gpio esp_driver_i2c esp_driver_i2s spiffs fatfs vfs )
    set(PRIV_REQ esp_lcd esp_driver_ledc )
else()
    set(REQ driver fatfs)
    set(PRIV_REQ esp_lcd spiffs)
endif()


idf_component_register(
    SRCS ${SRC}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES ${REQ}
    PRIV_REQUIRES esp_psram ${PRIV_REQ}
)
