
idf_component_register(
    SRCS "m5stack_tab5.c"
    INCLUDE_DIRS "include"
    INCLUDE_DIRS "include/bsp"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES 
        driver 
        esp_lcd
        esp_codec_dev
        esp_lvgl_port
	esp_lcd_ili9881c
    PRIV_REQUIRES usb spiffs fatfs
)
