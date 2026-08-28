idf_component_register(SRCS "esp_lcd_lt9611.c"
                       INCLUDE_DIRS "include"
                       REQUIRES "esp_lcd" "esp_driver_i2c"
                       PRIV_REQUIRES "esp_driver_gpio")
