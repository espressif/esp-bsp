#Add sources from ui directory
file(GLOB_RECURSE SRC_UI ${CMAKE_SOURCE_DIR} "ui/*.c")

idf_component_register(SRCS "main.c" ${SRC_UI}
                    INCLUDE_DIRS "." "ui")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-variable -DLV_LVGL_H_INCLUDE_SIMPLE)
