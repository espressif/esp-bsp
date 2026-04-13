if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.3")
    set(REQ esp_driver_i2c esp_driver_spi)
else()
    set(REQ driver)
endif()

idf_component_register(SRCS src/bmi270.c src/bmi270_sensor_hub.c INCLUDE_DIRS "include" PRIV_INCLUDE_DIRS "include_priv" REQUIRES ${REQ})

target_link_libraries(${COMPONENT_LIB} INTERFACE "-u bmi270_impl_init")