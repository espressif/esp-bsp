if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.3")
    set(REQ esp_driver_i2c)
else()
    set(REQ driver)
endif()

idf_component_register(SRCS "icm42670.c" INCLUDE_DIRS "include" REQUIRES ${REQ} esp_timer)

target_link_libraries(${COMPONENT_LIB} INTERFACE "-u icm42670_impl_init")