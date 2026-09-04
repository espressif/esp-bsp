idf_component_register(SRCS "test_tg28_sw.c")

# The test app needs tg28_sw_priv.h, which lives in the component's private
# include directory. Attach it to the target directly instead of listing it in
# INCLUDE_DIRS: the component validation check warns about include directories
# that belong to another component (components/tg28_sw/priv_include), and this
# keeps the header private to the test app.
target_include_directories(${COMPONENT_LIB} PRIVATE "${COMPONENT_DIR}/../../priv_include")
