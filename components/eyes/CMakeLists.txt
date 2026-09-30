idf_component_register(SRCS "eyes.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver
                       PRIV_REQUIRES pwm_util esp_timer)

# Stricter warnings for project code
target_compile_options(${COMPONENT_LIB} PRIVATE -Wextra -Wno-unused-parameter)
