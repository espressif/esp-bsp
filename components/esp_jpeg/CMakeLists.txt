set(sources "jpeg_decoder.c")
set(includes "include")

# Compile only when cannot use ROM code
if(CONFIG_JD_USE_EXT)
    list(APPEND sources "tjpgd/tjpgd.c")
    list(APPEND includes "tjpgd")
endif()

idf_component_register(SRCS ${sources} INCLUDE_DIRS ${includes})
