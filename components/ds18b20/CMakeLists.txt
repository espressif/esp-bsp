set(SRCS "src/ds18b20.c")

if(CONFIG_DS18B20_SENSOR_HUB)
    list(APPEND SRCS "src/ds18b20_sensor_hub.c")
endif()

idf_component_register(SRCS ${SRCS} INCLUDE_DIRS "include")

if(CONFIG_DS18B20_SENSOR_HUB)
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-u ds18b20_impl_init")
endif()
