idf_component_register(SRCS "led_strips.c" "led_effects.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver
                       PRIV_REQUIRES esp_timer)

# Stricter warnings for project code
target_compile_options(${COMPONENT_LIB} PRIVATE -Wextra -Wno-unused-parameter)
