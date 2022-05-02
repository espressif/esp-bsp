idf_component_register(SRCS "esp_wrover_kit.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver
                    PRIV_REQUIRES fatfs esp_lcd esp_timer)
