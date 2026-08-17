idf_component_register(
    SRCS "fusb303b.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c
    PRIV_REQUIRES freertos
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra)
