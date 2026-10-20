/*
 * led.h — The WS2815 LED strip around the box (one strip, LED_PIXELS long).
 *
 * Two ways to use it:
 *
 *   led_set(mode, color, fade_s)   the whole strip shows one colour; a
 *                                  background task fades between requests
 *   leds_comet_at() / leds_fill_to()   you draw one animation frame yourself
 *                                  (see charging() in states.c). The next
 *                                  led_set() hands control back to the task.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

typedef struct {
    uint8_t r, g, b;
} rgb_t;

#define LED_RGB(rr, gg, bb)  ((rgb_t){ (rr), (gg), (bb) })

#define RGB_RED     LED_RGB(0xFF, 0x00, 0x00)
#define RGB_GREEN   LED_RGB(0x00, 0xFF, 0x00)
#define RGB_BLUE    LED_RGB(0x00, 0x00, 0xFF)
#define RGB_YELLOW  LED_RGB(0xFF, 0xFF, 0x00)
#define RGB_CYAN    LED_RGB(0x00, 0xFF, 0xFF)
#define RGB_WHITE   LED_RGB(0xFF, 0xFF, 0xFF)
#define RGB_BLACK   LED_RGB(0x00, 0x00, 0x00)

#define LED_FADE_MIN  0.01f     /* shortest fade time, seconds */

typedef enum {
    LED_OFF = 0,    /* fade to dark over fade_s                     */
    LED_SOLID,      /* fade in over fade_s, then stay on            */
    LED_PULSE,      /* fade in over fade_s, out over fade_s, repeat */
    LED_BLINK,      /* on for fade_s, off for fade_s, repeat        */
} led_mode_t;

/** Set up the strip (dark) and start the background fade task. */
esp_err_t led_init(void);

/** Show `mode` in `color` on the whole strip. Whatever is lit now fades out
 *  over `fade_s` first, then the new mode starts. Returns at once. */
void led_set(led_mode_t mode, rgb_t color, float fade_s);

/** Fade to dark over `fade_s`. */
void led_set_off(float fade_s);

/** Overall brightness limit, 0..255 (start-up value: LED_MAX_BRIGHTNESS). */
void led_set_max_brightness(uint8_t max);

/**
 * Draw one frame of a comet running along the strip. `progress` 0..1 is the
 * head's position from the first LED to the last (LED_REVERSED in config.h
 * flips the direction); the comet wraps from the end back to the start. Call
 * it every ~10 ms with a growing `progress` to make the comet move.
 */
void leds_comet_at(rgb_t color, float progress);

/** Like leds_comet_at(), but every pixel the head has passed stays lit. */
void leds_fill_to(rgb_t color, float progress);
