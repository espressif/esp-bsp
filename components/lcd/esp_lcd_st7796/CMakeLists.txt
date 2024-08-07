set(srcs "esp_lcd_st7796.c"
         "esp_lcd_st7796_general.c")
if(CONFIG_SOC_MIPI_DSI_SUPPORTED)
    list(APPEND srcs "esp_lcd_st7796_mipi.c")
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "priv_include"
                       REQUIRES "esp_lcd"
                       PRIV_REQUIRES "driver")

include(package_manager)
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})
