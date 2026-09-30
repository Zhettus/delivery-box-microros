idf_component_register(SRCS "wm8960.c" "wm8960_clock.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver)

# Stricter warnings for project code
target_compile_options(${COMPONENT_LIB} PRIVATE -Wextra -Wno-unused-parameter)
