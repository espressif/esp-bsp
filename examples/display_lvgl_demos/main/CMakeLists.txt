idf_component_register(
    SRCS "main.c" ${LV_DEMOS_SOURCES}
    INCLUDE_DIRS "." ${LV_DEMO_DIR})

if(CONFIG_LV_USE_DEMO_BENCHMARK)
set_source_files_properties(
    ${LV_DEMOS_SOURCES}
    PROPERTIES COMPILE_OPTIONS
    -DLV_LVGL_H_INCLUDE_SIMPLE)
endif()
