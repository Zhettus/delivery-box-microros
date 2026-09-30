/*
 * led_effects.c — Pixel math for the WS2815 strips. See led_effects.h.
 */
#include <math.h>
#include <string.h>

#include "led_effects.h"

static float clamp01(float x)
{
    return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
}

static rgb_t scale_rgb(rgb_t c, float k)
{
    k = clamp01(k);
    return LED_RGB((uint8_t)((float)c.r * k + 0.5f),
                   (uint8_t)((float)c.g * k + 0.5f),
                   (uint8_t)((float)c.b * k + 0.5f));
}

/* ---- Layout -------------------------------------------------------------- */

uint16_t led_loop_length(const led_layout_t *l)
{
    uint16_t n = 0;
    for (int i = 0; i < l->loop_segments; i++) {
        n += l->pixels[l->loop[i].strip];
    }
    return n;
}

bool led_loop_pixel(const led_layout_t *l, uint16_t pos,
                    uint8_t *strip, uint16_t *pixel)
{
    for (int i = 0; i < l->loop_segments; i++) {
        uint8_t  s = l->loop[i].strip;
        uint16_t n = l->pixels[s];
        if (pos < n) {
            *strip = s;
            *pixel = l->loop[i].reversed ? (uint16_t)(n - 1 - pos) : pos;
            return true;
        }
        pos -= n;
    }
    return false;
}

/* ---- Envelopes ------------------------------------------------------------- */

float led_fx_envelope(led_effect_t effect, float t_s, float time_s)
{
    if (!(time_s >= LED_TIME_MIN_S)) {
        time_s = LED_TIME_MIN_S;
    }
    if (t_s < 0.0f) {
        t_s = 0.0f;
    }

    switch (effect) {
    case LED_EFFECT_SOLID:
        return (t_s < time_s) ? (t_s / time_s) : 1.0f;

    case LED_EFFECT_PULSE: {
        float p = fmodf(t_s, 2.0f * time_s);
        return (p < time_s) ? (p / time_s) : (2.0f - p / time_s);
    }

    case LED_EFFECT_BLINK: {
        float p = fmodf(t_s, 2.0f * time_s);
        return (p < time_s) ? 1.0f : 0.0f;
    }

    case LED_EFFECT_COMET:
    case LED_EFFECT_FILL:
    case LED_EFFECT_DIVERGE:
    case LED_EFFECT_IDENTIFY:
        return 1.0f;

    case LED_EFFECT_OFF:
    default:
        return 0.0f;
    }
}

/* ---- Patterns -------------------------------------------------------------- */

void led_fx_fill(const led_layout_t *l, led_frame_t fb, rgb_t c)
{
    for (int s = 0; s < l->num_strips; s++) {
        for (int i = 0; i < l->pixels[s]; i++) {
            fb[s][i] = c;
        }
    }
}

static void set_loop_pixel(const led_layout_t *l, led_frame_t fb,
                           uint16_t pos, rgb_t c)
{
    uint8_t  s;
    uint16_t i;
    if (led_loop_pixel(l, pos, &s, &i)) {
        fb[s][i] = c;
    }
}

void led_fx_comet(const led_layout_t *l, led_frame_t fb, rgb_t c,
                  float head, float tail_px)
{
    const uint16_t len = led_loop_length(l);
    const float    L   = (float)len;
    if (tail_px < 1.0f) {
        tail_px = 1.0f;
    }

    led_fx_fill(l, fb, RGB_BLACK);

    for (uint16_t p = 0; p < len; p++) {
        /* g = how far pixel p sits behind the head, wrapped to [0, L). */
        float g = fmodf(head - (float)p, L);
        if (g < 0.0f) {
            g += L;
        }

        float k;
        if (g < tail_px) {
            k = 1.0f - g / tail_px;       /* head + fading tail            */
        } else if (g > L - 1.0f) {
            k = g - (L - 1.0f);           /* anti-aliased leading pixel    */
        } else {
            continue;
        }
        set_loop_pixel(l, fb, p, scale_rgb(c, k));
    }
}

void led_fx_sweep(const led_layout_t *l, led_frame_t fb, rgb_t c, float head)
{
    const uint16_t len = led_loop_length(l);

    for (uint16_t p = 0; p < len; p++) {
        /* >= 1: fully lit, (0, 1): partially lit edge, <= 0: dark */
        set_loop_pixel(l, fb, p, scale_rgb(c, head - (float)p));
    }
}

void led_fx_diverge(const led_layout_t *l, led_frame_t fb, rgb_t c,
                    double t_s, float spawn_s, float speed_px_s, float tail_px)
{
    if (spawn_s < LED_TIME_MIN_S) {
        spawn_s = LED_TIME_MIN_S;
    }
    if (speed_px_s < 0.1f) {
        speed_px_s = 0.1f;
    }
    if (tail_px < 1.0f) {
        tail_px = 1.0f;
    }

    /* Comet k is born at k * spawn_s; only the youngest few are visible. */
    const double t_mod  = fmod(t_s, (double)spawn_s);
    const long   born   = (long)floor(t_s / (double)spawn_s) + 1;  /* comets born so far */

    for (int s = 0; s < l->num_strips; s++) {
        const int   n      = l->pixels[s];
        const float middle = (float)(n - 1) / 2.0f;     /* exact mirror centre */
        const float max_d  = middle + tail_px + 1.0f;   /* fully off the end   */
        const long  alive  = (long)ceilf(max_d / speed_px_s / spawn_s) + 1;
        const long  count  = born < alive ? born : alive;

        for (int i = 0; i < n; i++) {
            float a    = fabsf((float)i - middle);      /* distance from centre */
            float best = 0.0f;

            for (long k = 0; k < count; k++) {
                float age = (float)(t_mod + (double)k * spawn_s);
                float g   = speed_px_s * age - a;       /* pixel behind the head */
                float v;
                if (g >= 0.0f && g < tail_px) {
                    v = 1.0f - g / tail_px;             /* head -> tail          */
                } else if (g < 0.0f && g > -1.0f) {
                    v = 1.0f + g;                       /* sub-pixel leading edge */
                } else {
                    continue;
                }
                if (v > best) {
                    best = v;
                }
            }

            uint8_t lvl = led_dim8_video((uint8_t)(best * 255.0f + 0.5f));
            fb[s][i] = LED_RGB(led_scale8(c.r, lvl), led_scale8(c.g, lvl),
                               led_scale8(c.b, lvl));
        }
    }
}

void led_fx_identify(const led_layout_t *l, led_frame_t fb)
{
    static const rgb_t colours[LED_MAX_STRIPS] = {
        RGB_RED, RGB_GREEN, RGB_BLUE, RGB_WHITE,
    };

    for (int s = 0; s < l->num_strips; s++) {
        for (int i = 0; i < l->pixels[s]; i++) {
            fb[s][i] = scale_rgb(colours[s], 0.35f);
        }
        if (l->pixels[s] > 0) {
            fb[s][0] = RGB_MAGENTA;   /* marks where the data wire enters */
        }
    }
}

void led_fx_render(const led_layout_t *l, led_frame_t fb,
                   led_effect_t effect, rgb_t c, float time_s, double t_s,
                   float comet_tail_px, float diverge_speed_px_s,
                   float diverge_tail_px)
{
    if (!(time_s >= LED_TIME_MIN_S)) {
        time_s = LED_TIME_MIN_S;
    }
    const float loop_len = (float)led_loop_length(l);

    switch (effect) {
    case LED_EFFECT_SOLID:
    case LED_EFFECT_PULSE:
    case LED_EFFECT_BLINK:
        led_fx_fill(l, fb, c);
        break;

    case LED_EFFECT_COMET: {
        float progress = (float)fmod(t_s / (double)time_s, 1.0);
        led_fx_comet(l, fb, c, progress * loop_len, comet_tail_px);
        break;
    }

    case LED_EFFECT_FILL: {
        float progress = clamp01((float)(t_s / (double)time_s));
        led_fx_sweep(l, fb, c, progress * loop_len);
        break;
    }

    case LED_EFFECT_DIVERGE:
        led_fx_diverge(l, fb, c, t_s, time_s, diverge_speed_px_s, diverge_tail_px);
        break;

    case LED_EFFECT_IDENTIFY:
        led_fx_identify(l, fb);
        break;

    case LED_EFFECT_OFF:
    default:
        led_fx_fill(l, fb, RGB_BLACK);
        break;
    }
}

static const char *const s_names[LED_EFFECT_COUNT] = {
    [LED_EFFECT_OFF]      = "off",
    [LED_EFFECT_SOLID]    = "solid",
    [LED_EFFECT_PULSE]    = "pulse",
    [LED_EFFECT_BLINK]    = "blink",
    [LED_EFFECT_COMET]    = "comet",
    [LED_EFFECT_FILL]     = "fill",
    [LED_EFFECT_DIVERGE]  = "diverge",
    [LED_EFFECT_IDENTIFY] = "identify",
};

const char *led_effect_name(led_effect_t e)
{
    return (e < LED_EFFECT_COUNT) ? s_names[e] : "?";
}

bool led_effect_from_name(const char *name, led_effect_t *out)
{
    for (int e = 0; e < LED_EFFECT_COUNT; e++) {
        if (strcmp(name, s_names[e]) == 0) {
            *out = (led_effect_t)e;
            return true;
        }
    }
    return false;
}
