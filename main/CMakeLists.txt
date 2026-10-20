idf_component_register(
    SRCS "main.c"
         "states.c"
         "demo.c"
         "motor.c"
         "led.c"
         "face.c"
         "tail_light.c"
         "sound.c"
         "mic_stream.c"
         "ros_interface.c"
         "usb_jtag_transport.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver esp_timer led_strip pwm_util door ina228 audio
                  micro_ros_espidf_component
    EMBED_FILES "gm.wav")

# Stricter warnings for project code
target_compile_options(${COMPONENT_LIB} PRIVATE -Wextra -Wno-unused-parameter)
