idf_component_register(SRCS "esp_wrover_kit.c"
                    INCLUDE_DIRS "include"
                    PRIV_INCLUDE_DIRS "priv_include"
                    REQUIRES driver
                    PRIV_REQUIRES fatfs esp_lcd)

