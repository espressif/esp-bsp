idf_component_register(
    SRCS "tg28_sw.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES esp_driver_i2c
    PRIV_REQUIRES freertos
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra)
