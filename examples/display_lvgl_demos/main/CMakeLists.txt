set(LV_DEMO_DIR "")
set(LV_DEMOS_SOURCES "")
if(CONFIG_LV_USE_DEMO_BENCHMARK)
    list(APPEND LV_DEMO_DIR ../managed_components/lvgl__lvgl/demos)
    file(GLOB_RECURSE LV_DEMOS_SOURCES ${LV_DEMO_DIR}/*.c)
endif()

idf_component_register(
    SRCS "dispaly_lvgl_demos_main.c" ${LV_DEMOS_SOURCES}
    INCLUDE_DIRS "." ${LV_DEMO_DIR})

if(CONFIG_LV_USE_DEMO_BENCHMARK)
set_source_files_properties(
    ${LV_DEMOS_SOURCES}
    PROPERTIES COMPILE_OPTIONS
    -DLV_LVGL_H_INCLUDE_SIMPLE)
endif()
