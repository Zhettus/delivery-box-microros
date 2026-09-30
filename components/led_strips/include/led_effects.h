/*
 * led_effects.h — Pixel math for the WS2815 strips (pure logic, no ESP-IDF).
 *
 * Every effect is a pure function of time, so the LED task just asks
 * "what does effect X look like at time t?" each frame. Unit-tested in
 * test/host/test_led_effects.c.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LED_MAX_STRIPS  4

typedef struct {
    uint8_t r, g, b;
} rgb_t;

#define LED_RGB(rr, gg, bb)  ((rgb_t){ (rr), (gg), (bb) })

#define RGB_BLACK   LED_RGB(0x00, 0x00, 0x00)
#define RGB_WHITE   LED_RGB(0xFF, 0xFF, 0xFF)
#define RGB_RED     LED_RGB(0xFF, 0x00, 0x00)
#define RGB_GREEN   LED_RGB(0x00, 0xFF, 0x00)
#define RGB_BLUE    LED_RGB(0x00, 0x00, 0xFF)
#define RGB_YELLOW  LED_RGB(0xFF, 0xFF, 0x00)
#define RGB_CYAN    LED_RGB(0x00, 0xFF, 0xFF)
#define RGB_MAGENTA LED_RGB(0xFF, 0x00, 0xFF)

/*
 * What each effect does, and what its `time_s` parameter means:
 *
 *   LED_EFFECT_OFF       all dark                    time_s = fade-out time
 *   LED_EFFECT_SOLID     fade in, then stay on       time_s = fade-in time
 *   LED_EFFECT_PULSE     smooth 0 -> 1 -> 0 breathing time_s = half period
 *   LED_EFFECT_BLINK     hard on/off                 time_s = half period
 *   LED_EFFECT_COMET     comet racing around the box time_s = one lap
 *   LED_EFFECT_FILL      sweep that fills the loop   time_s = sweep time
 *   LED_EFFECT_DIVERGE   comet pairs born at the middle of every strip,
 *                        flying outwards            time_s = spawn interval
 *   LED_EFFECT_IDENTIFY  setup aid: strip 1 red, 2 green, 3 blue, 4 white,
 *                        pixel 0 of every strip magenta   (time_s unused)
 */
typedef enum {
    LED_EFFECT_OFF = 0,
    LED_EFFECT_SOLID,
    LED_EFFECT_PULSE,
    LED_EFFECT_BLINK,
    LED_EFFECT_COMET,
    LED_EFFECT_FILL,
    LED_EFFECT_DIVERGE,
    LED_EFFECT_IDENTIFY,
    LED_EFFECT_COUNT,
} led_effect_t;

#define LED_TIME_MIN_S  0.01f

/* One strip's place in the loop around the box. */
typedef struct {
    uint8_t strip;     /* strip index 0..3 (strip 1 = index 0)                 */
    bool    reversed;  /* true if pixel 0 is at the END of this segment when   */
                       /* walking the loop in its forward direction            */
} led_loop_segment_t;

/* Physical layout: how many pixels each strip has and how they form a loop. */
typedef struct {
    uint8_t            num_strips;
    uint16_t           pixels[LED_MAX_STRIPS];
    uint8_t            loop_segments;          /* segments in `loop`           */
    led_loop_segment_t loop[LED_MAX_STRIPS];   /* walk order around the box    */
} led_layout_t;

/* A frame: one pixel array per strip (fb[s][i]). */
typedef rgb_t *led_frame_t[LED_MAX_STRIPS];

/* ---- 8-bit colour helpers (FastLED-compatible maths) ------------------- */

static inline uint8_t led_scale8(uint8_t i, uint8_t scale)
{
    return (uint8_t)(((uint16_t)i * (uint16_t)scale + i) >> 8);
}

static inline uint8_t led_scale8_video(uint8_t i, uint8_t scale)
{
    uint8_t v = (uint8_t)(((uint16_t)i * (uint16_t)scale) >> 8);
    return (uint8_t)(v + ((i && scale) ? 1 : 0));
}

/* Perceptual dimming curve (squares the value, never rounds lit to zero). */
static inline uint8_t led_dim8_video(uint8_t x)
{
    return led_scale8_video(x, x);
}

/* ---- Layout -------------------------------------------------------------- */

/** Total pixels around the loop. */
uint16_t led_loop_length(const led_layout_t *l);

/** Map loop position `pos` to (strip, pixel). Returns false if out of range. */
bool led_loop_pixel(const led_layout_t *l, uint16_t pos,
                    uint8_t *strip, uint16_t *pixel);

/* ---- Brightness envelopes ------------------------------------------------ */

/**
 * Global brightness 0..1 of `effect` at `t_s` seconds after it started.
 * Moving-pattern effects (COMET, FILL, DIVERGE, IDENTIFY) return 1.
 */
float led_fx_envelope(led_effect_t effect, float t_s, float time_s);

/* ---- Patterns (write full-intensity colours into `fb`) ------------------- */

void led_fx_fill(const led_layout_t *l, led_frame_t fb, rgb_t c);

/** Comet with its head at loop position `head` and a linear tail. */
void led_fx_comet(const led_layout_t *l, led_frame_t fb, rgb_t c,
                  float head, float tail_px);

/** Everything behind loop position `head` is lit (anti-aliased edge). */
void led_fx_sweep(const led_layout_t *l, led_frame_t fb, rgb_t c, float head);

/**
 * Comet pairs are born at the centre of each strip every `spawn_s`
 * seconds and fly outwards at `speed_px_s`, mirrored left/right.
 */
void led_fx_diverge(const led_layout_t *l, led_frame_t fb, rgb_t c,
                    double t_s, float spawn_s, float speed_px_s, float tail_px);

void led_fx_identify(const led_layout_t *l, led_frame_t fb);

/**
 * Render `effect` at time `t_s` (seconds since it started) into `fb`.
 * `comet_tail_px`, `diverge_speed_px_s`, `diverge_tail_px` tune the shapes.
 */
void led_fx_render(const led_layout_t *l, led_frame_t fb,
                   led_effect_t effect, rgb_t c, float time_s, double t_s,
                   float comet_tail_px, float diverge_speed_px_s,
                   float diverge_tail_px);

const char *led_effect_name(led_effect_t e);

/** Parse an effect name ("pulse", "comet", ...). Returns false if unknown. */
bool led_effect_from_name(const char *name, led_effect_t *out);

#ifdef __cplusplus
}
#endif
