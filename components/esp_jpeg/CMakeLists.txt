set(sources "jpeg_decoder.c")
set(includes "include")

# ESP32 and ESP32-S3 has tjpgd in ROM
if((NOT ${IDF_TARGET_ESP32} MATCHES "esp32s3") AND 
(NOT ${IDF_TARGET_ESP32} MATCHES "esp32") AND 
(NOT ${IDF_TARGET_ESP32} MATCHES "esp32h2"))
    list(APPEND sources "tjpgd/tjpgd.c")
    list(APPEND includes "tjpgd")
endif()

idf_component_register(SRCS ${sources} INCLUDE_DIRS ${includes})
