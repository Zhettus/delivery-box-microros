idf_component_register(SRCS "ina228.c" "ina228_conv.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver)

# Stricter warnings for project code
target_compile_options(${COMPONENT_LIB} PRIVATE -Wextra -Wno-unused-parameter)
