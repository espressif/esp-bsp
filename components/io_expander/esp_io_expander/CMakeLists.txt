if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.3")
    set(PRIV_REQ "esp_driver_gpio")
else()
    set(PRIV_REQ "driver")
endif()

idf_component_register(
    SRCS "esp_io_expander.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES ${PRIV_REQ}
)

if(CONFIG_IO_EXPANDER_ENABLE_GPIO_API_WRAPPER)
    target_sources(${COMPONENT_LIB} PRIVATE "esp_io_expander_gpio_wrapper.c")

    target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=gpio_set_level"
                                                     "-Wl,--wrap=gpio_get_level"
                                                     "-Wl,--wrap=gpio_set_direction"
                                                     "-Wl,--wrap=gpio_set_pull_mode"
                                                    )
endif()
