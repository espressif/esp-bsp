
file(GLOB_RECURSE SRC src/*.c)

    set(REQ esp_driver_gpio esp_driver_i2c esp_driver_i2s spiffs fatfs vfs esp_driver_sdspi esp_driver_sdmmc )
    set(PRIV_REQ esp_lcd esp_driver_ledc )

idf_component_register(
    SRCS ${SRC}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES ${REQ}
    PRIV_REQUIRES esp_psram ${PRIV_REQ}
)

# BSP Name Alias
add_library(esp_bsp::core ALIAS ${COMPONENT_LIB})
