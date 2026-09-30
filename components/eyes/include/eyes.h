/*
 * eyes.h — Two eye LEDs, each behind a JZ-MOS driver (TRIG/PWM input).
 *
 * Both eyes always show the same brightness. A 100 Hz task renders the
 * current "emotion" with gamma correction and smooth transitions:
 *
 *   EYE_OFF        fade to dark over fade_s
 *   EYE_CONSTANT   fade to full (brightness cap) over fade_s
 *   EYE_THINKING   slow cosine "breathing", period = 2 * fade_s. Entering
 *                  it continues from the current brightness, no jump.
 *
 * eyes_set_level() takes manual control (e.g. to sync with an LED effect)
 * until the next eyes_set_emotion().
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EYE_FADE_MIN_S  0.01f

typedef enum {
    EYE_OFF = 0,
    EYE_THINKING,
    EYE_CONSTANT,
} eye_emotion_t;

typedef struct {
    gpio_num_t     left_gpio;
    gpio_num_t     right_gpio;
    ledc_timer_t   timer;
    ledc_channel_t left_channel;
    ledc_channel_t right_channel;
    uint32_t       pwm_freq_hz;     /* JZ-MOS PWM frequency                    */
    uint32_t       min_pulse_us;    /* shortest pulse the JZ-MOS reacts to     */
    float          gamma;           /* perceptual curve, 2.2 is typical        */
    float          brightness_cap;  /* 0..1, maximum brightness ever output    */
    bool           invert;          /* true if the driver lights on LOW       */
} eyes_config_t;

esp_err_t eyes_init(const eyes_config_t *cfg);

void eyes_set_emotion(eye_emotion_t e, float fade_s);
void eyes_set_level(float level);            /* manual 0..1 */
void eyes_set_brightness_cap(float cap);     /* 0..1 */
void eyes_set_min_pulse_us(uint32_t us);

eye_emotion_t eyes_get_emotion(void);
float         eyes_get_brightness_cap(void);
bool          eyes_is_transitioning(void);

const char *eye_emotion_name(eye_emotion_t e);
bool        eye_emotion_from_name(const char *name, eye_emotion_t *out);

#ifdef __cplusplus
}
#endif
