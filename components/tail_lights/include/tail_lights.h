/*
 * tail_lights.h — 4-function tail light (brake / run / left / right).
 *
 * Hardware (per channel, wiring doc §6.1):
 *   GPIO --1k--> 2N7000 gate; 2N7000 pulls the P-MOSFET gate low -> lamp on.
 *   GPIO HIGH = lamp ON (active high). A 10k pull-up keeps the lamp off at reset.
 *
 * Each channel is PWM-dimmable (LEDC) with hardware fades. Level 255 is a
 * true 100 % duty (constant DC), so lamps with their own sweep electronics
 * see a clean steady supply.
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

typedef enum {
    TAIL_LIGHT_BRAKE = 0,
    TAIL_LIGHT_RUN,
    TAIL_LIGHT_LEFT,
    TAIL_LIGHT_RIGHT,
    TAIL_LIGHT_COUNT,
} tail_light_t;

typedef struct {
    gpio_num_t     gpio[TAIL_LIGHT_COUNT];     /* indexed by tail_light_t */
    ledc_channel_t channel[TAIL_LIGHT_COUNT];
    ledc_timer_t   timer;
    uint32_t       pwm_freq_hz;
} tail_lights_config_t;

/* Brightness of every function at once (0 = off, 255 = full). */
typedef struct {
    uint8_t brake;
    uint8_t run;
    uint8_t left;
    uint8_t right;
} tail_lights_pattern_t;

esp_err_t tail_lights_init(const tail_lights_config_t *cfg);

/** Set one function immediately (0..255). */
void tail_lights_set(tail_light_t which, uint8_t level);

/** Fade one function to `level` over `ms` (runs in hardware, returns at once). */
void tail_lights_fade(tail_light_t which, uint8_t level, uint32_t ms);

/** Set all four functions at once. */
void tail_lights_apply(const tail_lights_pattern_t *p);

void    tail_lights_off(void);
uint8_t tail_lights_get(tail_light_t which);

const char *tail_light_name(tail_light_t which);

/** Parse "brake" / "run" / "left" / "right". Returns false if unknown. */
bool tail_light_from_name(const char *name, tail_light_t *out);

#ifdef __cplusplus
}
#endif
