if( NOT (CONFIG_IDF_TARGET_ESP32 OR CONFIG_IDF_TARGET_ESP32S3))
    message(WARNING "This test app is intended only for esp32 and esp32s3")
endif()

idf_component_register(SRCS "test_app_main.c" "lv_fill_common.c" "test_lv_fill_benchmark.c"
                      REQUIRES unity
                      WHOLE_ARCHIVE)
