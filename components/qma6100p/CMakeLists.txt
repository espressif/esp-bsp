idf_component_register(
    SRCS "qma6100p.c"
    INCLUDE_DIRS "include"
    REQUIRES "driver"
)

include(package_manager)
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})