idf_component_register(SRCS "door.c" "door_fsm.c" "door_lock.c" "motor_control.c" "current_guard.c"
                            "bldc_motor.c"
                            "dri0042.c" "encoder.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver
                       PRIV_REQUIRES pwm_util esp_timer)

# Stricter warnings for project code
target_compile_options(${COMPONENT_LIB} PRIVATE -Wextra -Wno-unused-parameter)
