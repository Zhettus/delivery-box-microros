/*
 * led_strips.h — Four WS2815 (12 V) strips, one RMT channel each.
 *
 * One 100 Hz task renders the current effect (see led_effects.h) on all
 * strips. Changing effect fades the old one out over `transition_s`, then
 * starts the new one from dark, so there are never hard jumps.
 *
 * Output pipeline per frame:
 *   effect pattern -> global brightness (perceptual curve, max_brightness)
 *   -> 12 V current limit -> colour correction -> temporal dithering
 *   -> wire byte order -> RMT
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#include "led_effects.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_num_t   gpio[LED_MAX_STRIPS];   /* data pin of strip 1..4               */
    led_layout_t layout;                 /* pixel counts + loop around the box   */
    bool         wire_order_rgb;         /* true: WS2815 expects R,G,B bytes     */
    uint8_t      max_brightness;         /* 0..255 ceiling for everything        */
    rgb_t        color_correction;       /* per-channel scale, e.g. 0xFF,0xB0,0xF0 */
    float        transition_s;           /* fade-out between effects; 0 = over the
                                            new effect's time (original behaviour) */
    uint32_t     frame_ms;               /* render period                        */
    uint32_t     power_budget_ma;        /* 12 V current the strips may draw     */
    float        ma_per_pixel_white;     /* full-white current of one pixel      */
    float        ma_per_pixel_idle;      /* quiescent current of one pixel       */
    float        comet_tail_px;
    float        diverge_speed_px_s;
    float        diverge_tail_px;
} led_strips_config_t;

esp_err_t led_strips_init(const led_strips_config_t *cfg);

/**
 * Show `effect` in `color`. `time_s` meaning depends on the effect
 * (see led_effects.h). Re-sending the identical request is a no-op.
 */
void led_strips_set(led_effect_t effect, rgb_t color, float time_s);

/** Fade to dark over `fade_s`. */
void led_strips_off(float fade_s);

void    led_strips_set_max_brightness(uint8_t max);
uint8_t led_strips_get_max_brightness(void);

void    led_strips_set_diverge(float speed_px_s, float tail_px);

/** Currently requested effect/colour/time. */
void led_strips_get(led_effect_t *effect, rgb_t *color, float *time_s);

/** Seconds since the current effect started (to sync e.g. the eyes). */
float led_strips_effect_time_s(void);

/** Estimated 12 V current of the last frame (mA, after limiting). */
uint32_t led_strips_estimated_ma(void);

#ifdef __cplusplus
}
#endif
