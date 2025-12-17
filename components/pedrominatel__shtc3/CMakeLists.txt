idf_component_register(
    SRCS "shtc3.c"
    INCLUDE_DIRS "include"
    REQUIRES "driver"
)

target_link_libraries(${COMPONENT_LIB} INTERFACE "-u shtc3_impl_init")