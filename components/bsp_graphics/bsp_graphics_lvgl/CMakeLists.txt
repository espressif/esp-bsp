idf_component_register(
    SRCS "src/bsp_graphics_lvgl.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
)

# Add BSP from project YML
target_link_libraries(${COMPONENT_LIB} PUBLIC esp_bsp::core)
target_compile_definitions(${COMPONENT_LIB} PUBLIC BSP_GRAPHICS)
target_compile_definitions(${COMPONENT_LIB} PUBLIC BSP_GRAPHICS_LVGL)
