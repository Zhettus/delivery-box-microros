idf_component_register(SRCS "audio.c" "mic_dsp.c" "audio_resample.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver)

# Stricter warnings for project code
target_compile_options(${COMPONENT_LIB} PRIVATE -Wextra -Wno-unused-parameter)
