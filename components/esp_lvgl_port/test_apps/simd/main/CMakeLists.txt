# Include SIMD assembly source code for rendering
if(CONFIG_IDF_TARGET_ESP32 OR CONFIG_IDF_TARGET_ESP32S3 OR CONFIG_IDF_TARGET_ESP32S2)
    message(VERBOSE "Compiling SIMD")
    set(PORT_PATH "../../../src/lvgl9")

    if(CONFIG_IDF_TARGET_ESP32S3)
        file(GLOB_RECURSE ASM_SOURCES ${PORT_PATH}/simd/*_xtensa_pie.S)       # Select Xtensa PIE, for esp32s3 target
    else()
        file(GLOB_RECURSE ASM_SOURCES ${PORT_PATH}/simd/*_xtensa_base.S)      # Select Xtensa Base for esp32, esp32s2 targets
    endif()

    file(GLOB_RECURSE ASM_MACROS ${PORT_PATH}/simd/lv_macro_*.S)        # Explicitly add all assembler macro files

else()
    message(WARNING "This test app is intended only for Xtensa targets (esp32, esp32s2, esp32s3")
endif()

# Hard copy of LV files
file(GLOB_RECURSE BLEND_SRCS lv_blend/src/*.c)

idf_component_register(SRCS "test_app_main.c"
                            "test_lv_fill_functionality.c"      # memset tests
                            "test_lv_fill_benchmark.c"
                            "test_lv_image_functionality.c"     # memcpy tests
                            "test_lv_image_benchmark.c"
                            ${BLEND_SRCS}                       # Hard copy of LVGL's blend API, to simplify testing
                            ${ASM_SOURCES}                      # Assembly src files
                            ${ASM_MACROS}                       # Assembly macro files
                      INCLUDE_DIRS "lv_blend/include" "../../../include"
                      REQUIRES unity
                      WHOLE_ARCHIVE)
