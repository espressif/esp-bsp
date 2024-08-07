set(srcs "")
if(CONFIG_SOC_MIPI_DSI_SUPPORTED)
    list(APPEND srcs "esp_lcd_ili9881c.c")
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       REQUIRES "esp_lcd"
                       PRIV_REQUIRES "esp_driver_gpio")
