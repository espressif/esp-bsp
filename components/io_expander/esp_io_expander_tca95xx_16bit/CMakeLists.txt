set(req)
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.2.6")
    list(APPEND req "esp_driver_i2c")
else()
    list(APPEND req "driver")
endif()

idf_component_register(SRCS "esp_io_expander_tca95xx_16bit.c" INCLUDE_DIRS "include" REQUIRES ${req})
