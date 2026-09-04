idf_component_register(
    SRCS "rx8130ce.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c
    PRIV_REQUIRES freertos
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra)
