idf_component_register(SRCS "tail_lights.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver
                       PRIV_REQUIRES pwm_util)

# Stricter warnings for project code
target_compile_options(${COMPONENT_LIB} PRIVATE -Wextra -Wno-unused-parameter)
