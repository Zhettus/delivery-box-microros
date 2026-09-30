/*
 * led_strips.c — WS2815 strips over RMT. See led_strips.h.
 */
#include <string.h>

#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

#include "led_strips.h"

static const char *TAG = "leds";

/* Below this global level a new effect starts at once (nothing to fade). */
#define FADE_SKIP_LEVEL   0.002f

/*
 * RMT memory: the ESP32-S3 has 4 TX channels with 48 symbols each. Asking
 * for more than 48 per channel would steal a neighbour's block and leave
 * too few channels for 4 strips.
 */
#define RMT_SYMBOLS_PER_CHANNEL  48
#define RMT_RESOLUTION_HZ        (10 * 1000 * 1000)

typedef struct {
    led_effect_t effect;
    rgb_t        color;
    float        time_s;
} led_request_t;

static led_strips_config_t s_cfg;
static led_strip_handle_t  s_handle[LED_MAX_STRIPS];
static rgb_t              *s_pixels[LED_MAX_STRIPS];   /* pattern (full scale) */
static rgb_t              *s_dither[LED_MAX_STRIPS];   /* dithering residuals  */

static portMUX_TYPE        s_lock = portMUX_INITIALIZER_UNLOCKED;
static led_request_t       s_request;                   /* guarded by s_lock */
static volatile uint8_t    s_max_bright;
static volatile float      s_div_speed, s_div_tail;
static volatile uint32_t   s_last_ma;
static volatile int64_t    s_active_t0;                  /* start of the active effect */

/* Moving patterns (comet, fill, diverge, identify) behave like the original
 * leds_comet_at(): they start and end instantly (no fade) and run at the
 * full max_brightness, without the perceptual dimming curve. */
static bool is_pattern(led_effect_t e)
{
    return e == LED_EFFECT_COMET || e == LED_EFFECT_FILL ||
           e == LED_EFFECT_DIVERGE || e == LED_EFFECT_IDENTIFY;
}

/* ---- output ------------------------------------------------------------ */

static bool same_request(const led_request_t *a, const led_request_t *b)
{
    return a->effect == b->effect && a->time_s == b->time_s &&
           a->color.r == b->color.r && a->color.g == b->color.g &&
           a->color.b == b->color.b;
}

/* Estimated 12 V current at full scale (before global brightness). */
static float pattern_ma(void)
{
    uint32_t sum = 0, count = 0;
    for (int s = 0; s < s_cfg.layout.num_strips; s++) {
        for (int i = 0; i < s_cfg.layout.pixels[s]; i++) {
            sum += s_pixels[s][i].r + s_pixels[s][i].g + s_pixels[s][i].b;
        }
        count += s_cfg.layout.pixels[s];
    }
    return (float)sum / 765.0f * s_cfg.ma_per_pixel_white +
           (float)count * s_cfg.ma_per_pixel_idle;
}

static void clear_dither(void)
{
    for (int s = 0; s < s_cfg.layout.num_strips; s++) {
        memset(s_dither[s], 0, s_cfg.layout.pixels[s] * sizeof(rgb_t));
    }
}

static inline uint8_t scale8_dither(uint8_t v, uint8_t scale, uint8_t *residual)
{
    uint16_t x = (uint16_t)v * scale + v + *residual;
    *residual = (uint8_t)(x & 0xFF);
    return (uint8_t)(x >> 8);
}

static void show(uint8_t brightness)
{
    /* Current limit: scale brightness down if the frame would exceed budget. */
    float full_ma = pattern_ma();
    float want_ma = full_ma * (float)brightness / 255.0f;
    if (s_cfg.power_budget_ma > 0 && want_ma > (float)s_cfg.power_budget_ma) {
        brightness = (uint8_t)((float)brightness * (float)s_cfg.power_budget_ma / want_ma);
        want_ma = (float)s_cfg.power_budget_ma;
    }
    s_last_ma = (uint32_t)want_ma;

    uint8_t sr = led_scale8(s_cfg.color_correction.r, brightness);
    uint8_t sg = led_scale8(s_cfg.color_correction.g, brightness);
    uint8_t sb = led_scale8(s_cfg.color_correction.b, brightness);

    for (int s = 0; s < s_cfg.layout.num_strips; s++) {
        for (int i = 0; i < s_cfg.layout.pixels[s]; i++) {
            rgb_t p = s_pixels[s][i];
            uint8_t r = scale8_dither(p.r, sr, &s_dither[s][i].r);
            uint8_t g = scale8_dither(p.g, sg, &s_dither[s][i].g);
            uint8_t b = scale8_dither(p.b, sb, &s_dither[s][i].b);

            /* led_strip sends its arguments as G,R,B. WS2815 wants R,G,B,
             * so pass red in the "green" slot and green in the "red" slot. */
            if (s_cfg.wire_order_rgb) {
                led_strip_set_pixel(s_handle[s], i, g, r, b);
            } else {
                led_strip_set_pixel(s_handle[s], i, r, g, b);
            }
        }
    }
    for (int s = 0; s < s_cfg.layout.num_strips; s++) {
        led_strip_refresh(s_handle[s]);
    }
}

/* ---- render task --------------------------------------------------------- */

static void led_task(void *arg)
{
    led_request_t active     = { .effect = LED_EFFECT_OFF, .time_s = 1.0f };
    int64_t       active_t0  = esp_timer_get_time();
    bool          fading_out = false;
    int64_t       fade_t0    = 0;
    float         fade_from  = 0.0f;
    float         fade_dur   = 0.0f;
    float         last_level = 0.0f;

    const TickType_t period = pdMS_TO_TICKS(s_cfg.frame_ms);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        int64_t now = esp_timer_get_time();

        led_request_t req;
        taskENTER_CRITICAL(&s_lock);
        req = s_request;
        taskEXIT_CRITICAL(&s_lock);

        /* New request: fade the current output out first, like the original
         * led_task: over the NEW effect's time (or transition_s if set).
         * Moving patterns switch instantly in and out. */
        if (!fading_out && !same_request(&req, &active)) {
            if (last_level <= FADE_SKIP_LEVEL || is_pattern(active.effect) ||
                is_pattern(req.effect)) {
                if (is_pattern(active.effect)) {
                    last_level = 0.0f;         /* next effect fades in fresh */
                    clear_dither();
                }
                active    = req;
                active_t0 = now;
            } else {
                fading_out = true;
                fade_t0    = now;
                fade_from  = last_level;
                fade_dur   = (s_cfg.transition_s > 0.0f && req.effect != LED_EFFECT_OFF)
                             ? s_cfg.transition_s : req.time_s;
                if (fade_dur < LED_TIME_MIN_S) {
                    fade_dur = LED_TIME_MIN_S;
                }
            }
        }
        s_active_t0 = active_t0;

        float level;
        if (fading_out) {
            float f = (float)(now - fade_t0) / 1e6f;
            if (f >= fade_dur) {
                fading_out = false;
                active     = req;          /* the latest request wins */
                active_t0  = now;
                level      = led_fx_envelope(active.effect, 0.0f, active.time_s);
            } else {
                level = fade_from * (1.0f - f / fade_dur);
            }
        } else {
            float t = (float)(now - active_t0) / 1e6f;
            level = led_fx_envelope(active.effect, t, active.time_s);
        }

        s_active_t0 = active_t0;

        if (level <= 0.0f && last_level > 0.0f) {
            clear_dither();            /* no leftover dither sparkle when dark */
        }
        last_level = level;

        double t_s = (double)(now - active_t0) / 1e6;
        led_fx_render(&s_cfg.layout, s_pixels, active.effect, active.color,
                      active.time_s, t_s, s_cfg.comet_tail_px,
                      s_div_speed, s_div_tail);

        if (is_pattern(active.effect) && !fading_out) {
            show(s_max_bright);                /* original: full MAX_BRIGHTNESS */
        } else {
            show(led_dim8_video((uint8_t)(level * (float)s_max_bright + 0.5f)));
        }

        xTaskDelayUntil(&last_wake, period);
    }
}

/* ---- public API ------------------------------------------------------------ */

esp_err_t led_strips_init(const led_strips_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg->layout.num_strips > 0 &&
                        cfg->layout.num_strips <= LED_MAX_STRIPS,
                        ESP_ERR_INVALID_ARG, TAG, "num_strips");
    ESP_RETURN_ON_FALSE(cfg->frame_ms > 0, ESP_ERR_INVALID_ARG, TAG, "frame_ms");

    s_cfg        = *cfg;
    s_max_bright = cfg->max_brightness;
    s_div_speed  = cfg->diverge_speed_px_s;
    s_div_tail   = cfg->diverge_tail_px;
    s_request    = (led_request_t){ .effect = LED_EFFECT_OFF, .time_s = 1.0f };

    for (int s = 0; s < cfg->layout.num_strips; s++) {
        uint16_t n = cfg->layout.pixels[s];
        s_pixels[s] = heap_caps_calloc(n, sizeof(rgb_t), MALLOC_CAP_INTERNAL);
        s_dither[s] = heap_caps_calloc(n, sizeof(rgb_t), MALLOC_CAP_INTERNAL);
        ESP_RETURN_ON_FALSE(s_pixels[s] && s_dither[s], ESP_ERR_NO_MEM, TAG, "buffers");

        led_strip_config_t strip_cfg = {
            .strip_gpio_num   = cfg->gpio[s],
            .max_leds         = n,
            .led_pixel_format = LED_PIXEL_FORMAT_GRB,
            .led_model        = LED_MODEL_WS2812,   /* WS2815 uses WS2812 bit timing */
            .flags.invert_out = false,
        };
        led_strip_rmt_config_t rmt_cfg = {
            .clk_src           = RMT_CLK_SRC_DEFAULT,
            .resolution_hz     = RMT_RESOLUTION_HZ,
            .mem_block_symbols = RMT_SYMBOLS_PER_CHANNEL,
            .flags.with_dma    = false,
        };
        ESP_RETURN_ON_ERROR(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_handle[s]),
                            TAG, "strip %d (GPIO%d)", s + 1, cfg->gpio[s]);
        led_strip_clear(s_handle[s]);
        ESP_LOGI(TAG, "strip %d: GPIO%d, %u pixels", s + 1, cfg->gpio[s], n);
    }

    if (xTaskCreate(led_task, "leds", 4096, NULL, 3, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "ready: loop %u px, max brightness %u, budget %lu mA",
             led_loop_length(&cfg->layout), cfg->max_brightness,
             (unsigned long)cfg->power_budget_ma);
    return ESP_OK;
}

void led_strips_set(led_effect_t effect, rgb_t color, float time_s)
{
    if (!(time_s >= LED_TIME_MIN_S)) {
        time_s = LED_TIME_MIN_S;
    }
    taskENTER_CRITICAL(&s_lock);
    s_request = (led_request_t){ .effect = effect, .color = color, .time_s = time_s };
    taskEXIT_CRITICAL(&s_lock);
}

void led_strips_off(float fade_s)
{
    led_strips_set(LED_EFFECT_OFF, RGB_BLACK, fade_s);
}

void led_strips_set_max_brightness(uint8_t max)
{
    s_max_bright = max;
}

uint8_t led_strips_get_max_brightness(void)
{
    return s_max_bright;
}

void led_strips_set_diverge(float speed_px_s, float tail_px)
{
    s_div_speed = speed_px_s;
    s_div_tail  = tail_px;
}

void led_strips_get(led_effect_t *effect, rgb_t *color, float *time_s)
{
    taskENTER_CRITICAL(&s_lock);
    led_request_t r = s_request;
    taskEXIT_CRITICAL(&s_lock);
    if (effect) *effect = r.effect;
    if (color)  *color  = r.color;
    if (time_s) *time_s = r.time_s;
}

float led_strips_effect_time_s(void)
{
    return (float)((double)(esp_timer_get_time() - s_active_t0) / 1e6);
}

uint32_t led_strips_estimated_ma(void)
{
    return s_last_ma;
}
