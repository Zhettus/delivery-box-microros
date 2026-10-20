/*
 * led.c — One WS2815 LED strip (LED_PIXELS long) over RMT. See led.h.
 *
 * One background task (led_task) owns the strip and redraws it every
 * LED_FRAME_MS. The public functions only store a request; the task picks
 * it up on its next frame.
 */
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "led_strip.h"

#include "board.h"
#include "config.h"
#include "led.h"

static const char *TAG = "led";

/* Below this level a new mode starts at once (nothing visible to fade out). */
#define FADE_SKIP_LEVEL     0.002f

/* Only one RMT channel is used, so it can take all 4 memory blocks of the
 * ESP32-S3 (4 x 48 symbols): fewer refills while a long strip is sent. */
#define RMT_SYMBOLS              (4 * 48)
#define RMT_RESOLUTION_HZ        (10 * 1000 * 1000)

static led_strip_handle_t s_strip;
static rgb_t              s_leds[LED_PIXELS];     /* the picture, at full brightness */
static rgb_t              s_dither[LED_PIXELS];   /* rounding leftovers, for smooth fades */

/* ---- the request: what the public functions ask the task to draw -------- */

typedef struct {
    led_mode_t mode;
    rgb_t      color;
    float      fade_s;
} look_t;

typedef enum {
    DRAW_LOOK = 0,      /* led_set(): one colour with a fade/pulse/blink */
    DRAW_COMET,         /* leds_comet_at() */
    DRAW_FILL,          /* leds_fill_to()  */
} draw_t;

typedef struct {
    draw_t draw;
    look_t look;        /* DRAW_LOOK */
    rgb_t  color;       /* DRAW_COMET / DRAW_FILL */
    float  progress;    /* DRAW_COMET / DRAW_FILL: 0..1 along the strip */
} request_t;

static portMUX_TYPE     s_lock    = portMUX_INITIALIZER_UNLOCKED;
static request_t        s_request = { .look = { LED_OFF, { 0, 0, 0 }, 1.0f } };
static volatile uint8_t s_max_bright = LED_MAX_BRIGHTNESS;
static bool             s_inited;

/* ---- brightness maths (same as FastLED) ---------------------------------- */

static inline uint8_t scale8(uint8_t i, uint8_t scale)
{
    return (uint8_t)(((uint16_t)i * (uint16_t)scale + i) >> 8);
}

/* Like scale8, but never scales a non-zero value down to zero. */
static inline uint8_t scale8_video(uint8_t i, uint8_t scale)
{
    uint8_t v = (uint8_t)(((uint16_t)i * (uint16_t)scale) >> 8);
    return v + ((i && scale) ? 1 : 0);
}

/* Perceptual dimming curve: makes a linear fade LOOK linear. */
static inline uint8_t dim8_video(uint8_t x)
{
    return scale8_video(x, x);
}

/* scale8 that carries the rounding error into the next frame. */
static inline uint8_t scale8_dither(uint8_t i, uint8_t scale, uint8_t *residual)
{
    uint16_t v = (uint16_t)i * (uint16_t)scale + i + *residual;
    *residual = (uint8_t)(v & 0xFF);
    return (uint8_t)(v >> 8);
}

/* ---- output ---------------------------------------------------------------- */

/* Lower `brightness` if the current picture would draw more than the budget. */
static uint8_t power_limited_brightness(uint8_t brightness)
{
    uint32_t sum = 0;
    for (int i = 0; i < LED_PIXELS; i++) {
        sum += s_leds[i].r + s_leds[i].g + s_leds[i].b;
    }
    float full_ma = (float)sum / 765.0f * LED_MA_PER_PIXEL_WHITE +
                    (float)LED_PIXELS * LED_MA_PER_PIXEL_IDLE;
    float want_ma = full_ma * (float)brightness / 255.0f;

    if (want_ma <= (float)LED_POWER_BUDGET_MA) {
        return brightness;
    }
    return (uint8_t)((float)brightness * (float)LED_POWER_BUDGET_MA / want_ma);
}

/* Send the picture in s_leds[] to the strip at `brightness` (0..255). */
static void led_show(uint8_t brightness)
{
    uint8_t scale = power_limited_brightness(brightness);

    uint8_t sr = scale8(LED_CORRECTION_R, scale);
    uint8_t sg = scale8(LED_CORRECTION_G, scale);
    uint8_t sb = scale8(LED_CORRECTION_B, scale);

    for (int i = 0; i < LED_PIXELS; i++) {
        uint8_t r = scale8_dither(s_leds[i].r, sr, &s_dither[i].r);
        uint8_t g = scale8_dither(s_leds[i].g, sg, &s_dither[i].g);
        uint8_t b = scale8_dither(s_leds[i].b, sb, &s_dither[i].b);

        /* led_strip sends its arguments as G,R,B. WS2815 wants R,G,B, so
         * pass red in the "green" slot and green in the "red" slot. */
#if LED_WIRE_ORDER_RGB
        led_strip_set_pixel(s_strip, i, g, r, b);
#else
        led_strip_set_pixel(s_strip, i, r, g, b);
#endif
    }
    led_strip_refresh(s_strip);
}

static void fill_solid(rgb_t color)
{
    for (int i = 0; i < LED_PIXELS; i++) {
        s_leds[i] = color;
    }
}

static void clear_dither(void)
{
    memset(s_dither, 0, sizeof(s_dither));
}

/* ---- comet / fill along the strip ------------------------------------------ */

/* Position p along the animation -> LED index (LED_REVERSED flips it). */
static void set_pixel(int p, rgb_t color, float intensity)
{
    int led = LED_REVERSED ? (LED_PIXELS - 1 - p) : p;
    s_leds[led].r = (uint8_t)(color.r * intensity);
    s_leds[led].g = (uint8_t)(color.g * intensity);
    s_leds[led].b = (uint8_t)(color.b * intensity);
}

/* Draw a comet whose head is at fractional position `head`: a bright head
 * with a linearly fading tail behind it. The strip is treated as a closed
 * loop, so the comet runs off the last LED and comes back in at LED 0. The
 * head straddles two pixels (anti-aliased) so sub-pixel positions render
 * smoothly. */
static void render_comet(rgb_t color, float head)
{
    fill_solid(RGB_BLACK);

    for (int p = 0; p < LED_PIXELS; p++) {
        /* g = how far pixel p sits behind the head, wrapped to [0, LED_PIXELS). */
        float g = head - (float)p;
        if (g < 0.0f) {
            g += (float)LED_PIXELS;
        }

        if (g < (float)LED_COMET_TAIL_PX) {
            set_pixel(p, color, 1.0f - g / (float)LED_COMET_TAIL_PX);   /* head + tail */
        } else if (g > (float)LED_PIXELS - 1.0f) {
            set_pixel(p, color, 1.0f - ((float)LED_PIXELS - g));        /* leading edge */
        }
    }
}

/* Fill sweep: every pixel the head has already passed is solid `color`, the
 * pixel the head is currently on is partially lit, the rest are dark. */
static void render_fill(rgb_t color, float head)
{
    for (int p = 0; p < LED_PIXELS; p++) {
        float lit = head - (float)p;      /* >=1 full, in (0,1) partial, else off */
        set_pixel(p, color, (lit >= 1.0f) ? 1.0f : (lit > 0.0f ? lit : 0.0f));
    }
}

/* ---- fade / pulse / blink ---------------------------------------------------- */

/* Brightness 0..1 of `mode`, `t` seconds after the mode started. */
static float mode_level(led_mode_t mode, float t, float fade)
{
    switch (mode) {

    case LED_SOLID:
        return (t < fade) ? (t / fade) : 1.0f;

    case LED_PULSE: {
        float p = fmodf(t, 2.0f * fade);
        return (p < fade) ? (p / fade)
                          : (2.0f - p / fade);
    }

    case LED_BLINK: {
        float p = fmodf(t, 2.0f * fade);
        return (p < fade) ? 1.0f : 0.0f;
    }

    case LED_OFF:
    default:
        return 0.0f;
    }
}

static bool same_look(look_t a, look_t b)
{
    return a.mode == b.mode && a.fade_s == b.fade_s &&
           a.color.r == b.color.r && a.color.g == b.color.g && a.color.b == b.color.b;
}

static void led_task(void *arg)
{
    look_t  active     = { LED_OFF, { 0, 0, 0 }, 1.0f };   /* being shown now */
    int64_t active_t0  = esp_timer_get_time();             /* when it started */
    float   last_level = 0.0f;

    bool    fading_out = false;     /* old look fading to dark before the new one */
    int64_t fade_t0    = 0;
    float   fade_from  = 0.0f;
    float   fade_dur   = 0.0f;

    bool    was_animating = false;  /* last frame was a comet / fill */

    const TickType_t period = pdMS_TO_TICKS(LED_FRAME_MS);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        xTaskDelayUntil(&last_wake, period);

        taskENTER_CRITICAL(&s_lock);
        request_t req = s_request;
        taskEXIT_CRITICAL(&s_lock);

        int64_t now_us = esp_timer_get_time();

        /* ---- comet / fill: draw the requested frame at full brightness ---- */
        if (req.draw != DRAW_LOOK) {
            if (req.draw == DRAW_COMET) {
                render_comet(req.color, req.progress * (float)LED_PIXELS);
            } else {
                render_fill(req.color, req.progress * (float)LED_PIXELS);
            }
            led_show(s_max_bright);
            was_animating = true;
            continue;
        }

        /* An animation just ended: start the requested look fresh. */
        if (was_animating) {
            was_animating = false;
            fading_out    = false;
            last_level    = 0.0f;
            clear_dither();
            active    = req.look;
            active_t0 = now_us;
        }

        /* ---- new look requested: fade the current one out first ----------- */
        if (!fading_out && !same_look(req.look, active)) {
            if (last_level <= FADE_SKIP_LEVEL) {
                active    = req.look;           /* already dark: start at once */
                active_t0 = now_us;
            } else {
                fading_out = true;
                fade_t0    = now_us;
                fade_from  = last_level;
                fade_dur   = req.look.fade_s;   /* over the NEW look's fade time */
            }
        }

        float level;
        if (fading_out) {
            float f = (float)(now_us - fade_t0) / 1e6f;
            if (f >= fade_dur) {
                fading_out = false;
                active     = req.look;          /* the latest request wins */
                active_t0  = now_us;
                level      = mode_level(active.mode, 0.0f, active.fade_s);
            } else {
                level = fade_from * (1.0f - f / fade_dur);
            }
        } else {
            float t = (float)(now_us - active_t0) / 1e6f;
            level = mode_level(active.mode, t, active.fade_s);
        }

        if (level <= 0.0f && last_level > 0.0f) {
            clear_dither();             /* no leftover dither sparkle when dark */
        }
        last_level = level;

        fill_solid(active.color);
        led_show(dim8_video((uint8_t)(level * (float)s_max_bright)));
    }
}

/* ---- public API ---------------------------------------------------------------- */

esp_err_t led_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    led_strip_config_t strip_config = {
        .strip_gpio_num   = PIN_LED_STRIP,
        .max_leds         = LED_PIXELS,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model        = LED_MODEL_WS2812,   /* WS2815 uses WS2812 bit timing */
        .flags.invert_out = false,
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = RMT_RESOLUTION_HZ,
        .mem_block_symbols = RMT_SYMBOLS,
        .flags.with_dma    = false,
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "strip (GPIO%d) init failed: %s", PIN_LED_STRIP, esp_err_to_name(err));
        return err;
    }
    led_strip_clear(s_strip);

    if (xTaskCreate(led_task, "led", 4096, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed");
        return ESP_ERR_NO_MEM;
    }

    s_inited = true;
    ESP_LOGI(TAG, "init ok: 1 strip on GPIO%d, %d pixels, max brightness %u",
             PIN_LED_STRIP, LED_PIXELS, s_max_bright);
    return ESP_OK;
}

void led_set(led_mode_t mode, rgb_t color, float fade_s)
{
    if (!(fade_s >= LED_FADE_MIN)) {
        fade_s = LED_FADE_MIN;
    }

    request_t req = { .draw = DRAW_LOOK, .look = { mode, color, fade_s } };
    taskENTER_CRITICAL(&s_lock);
    s_request = req;
    taskEXIT_CRITICAL(&s_lock);
}

void led_set_off(float fade_s)
{
    led_set(LED_OFF, RGB_BLACK, fade_s);
}

void led_set_max_brightness(uint8_t max)
{
    s_max_bright = max;
    ESP_LOGI(TAG, "max brightness -> %u", max);
}

static void request_animation(draw_t draw, rgb_t color, float progress)
{
    taskENTER_CRITICAL(&s_lock);
    s_request.draw     = draw;      /* s_request.look is kept, but ignored */
    s_request.color    = color;
    s_request.progress = progress;
    taskEXIT_CRITICAL(&s_lock);
}

void leds_comet_at(rgb_t color, float progress)
{
    request_animation(DRAW_COMET, color, progress);
}

void leds_fill_to(rgb_t color, float progress)
{
    request_animation(DRAW_FILL, color, progress);
}
