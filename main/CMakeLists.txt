# Which program to build (the choice is remembered in the build folder):
#
#   idf.py -DMOTOR_TEST=0 build   -> the Delivery Box firmware with micro-ROS
#   idf.py -DMOTOR_TEST=1 build   -> main/motor_test.c: motor wire check +
#                                    speed test (+5 / stop / -5 rpm loop)
option(MOTOR_TEST "Build main/motor_test.c instead of the firmware" OFF)

if(MOTOR_TEST)
    message(STATUS "*** Building the MOTOR TEST (motor_test.c) ***")
    set(app_srcs "motor_test.c")
else()
    set(app_srcs "main.c" "robot_state.c" "commands.c" "power_monitor.c" "demo.c"
                 "ros_interface.c" "usb_jtag_transport.c")
endif()

idf_component_register(SRCS ${app_srcs}
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver esp_timer esp_psram vfs pwm_util
                                     door tail_lights eyes led_strips audio ina228
                                     micro_ros_espidf_component)

# Stricter warnings for project code
target_compile_options(${COMPONENT_LIB} PRIVATE -Wextra -Wno-unused-parameter)
