/*
 * face.c — The face (eyes + mouth, one MOSFET) over LEDC PWM. See face.h.
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "board.h"
#include "config.h"
#include "face.h"
#include "pwm_util.h"

static const char *TAG = "face";

#define FACE_TICK_HZ  100
#define TWO_PI_F      6.2831853f


static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

/* Requested by the API (guarded by s_lock). */
static face_emotion_t s_target       = FACE_OFF;
static float          s_target_fade  = 1.0f;
static bool           s_manual       = false;
static float          s_manual_level = 0.0f;
static bool           s_resync       = false;   /* manual mode ended: re-enter the emotion */
static volatile float s_cap          = FACE_BRIGHTNESS_CAP;

/* Owned by the face task. */
static face_emotion_t s_active      = FACE_OFF;
static float          s_active_fade = 1.0f;
static int64_t        s_t0;                     /* start of the ramp / breathing cycle */
static float          s_from;                   /* level the ramp started at */
static float          s_ramp_dur;
static float          s_last_level;
static bool           s_rising;                 /* brightness was last going up */

static uint32_t       s_duty_full;              /* LEDC duty for 100 % */
static float          s_min_frac;               /* shortest visible pulse, as a duty fraction */

static float clamp01(float x)
{
    return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
}

/* 0..1 level -> LEDC duty, between the module's minimum pulse and the cap. */
static uint32_t level_to_duty(float level, bool lit)
{
    if (!lit) {
        return 0;
    }

    float top = (float)s_duty_full * s_cap;
    float min = (float)s_duty_full * s_min_frac;
    if (min > top) {
        min = top;
    }

    float d = min + powf(clamp01(level), FACE_GAMMA) * (top - min);
    uint32_t duty = (uint32_t)(d + 0.5f);
    return duty > s_duty_full ? s_duty_full : duty;
}

/* Smooth start and end of a ramp (smoothstep). */
static float ease(float x)
{
    x = clamp01(x);
    return x * x * (3.0f - 2.0f * x);
}

static int64_t breath_period_us(float fade)
{
    int64_t p = (int64_t)(2.0f * fade * 1e6f);
    return (p > 0) ? p : 1;
}

/* FACE_THINKING: cosine breathing, one full cycle every 2 x fade. */
static float breathe(int64_t now_us)
{
    int64_t period = breath_period_us(s_active_fade);
    int64_t ph = (now_us - s_t0) % period;
    if (ph < 0) {
        ph += period;
    }
    return 0.5f - 0.5f * cosf(TWO_PI_F * (float)ph / (float)period);
}

/* FACE_CONSTANT / FACE_OFF: ramp from the start level to `to`. */
static float ramp(int64_t now_us, float to)
{
    if (s_ramp_dur <= 0.0f) {
        return to;
    }
    float x = ((float)(now_us - s_t0) / 1e6f) / s_ramp_dur;
    if (x >= 1.0f) {
        return to;
    }
    return s_from + (to - s_from) * ease(x);
}

static void enter_emotion(face_emotion_t e, float fade, int64_t now_us)
{
    float from = s_last_level;

    s_active      = e;
    s_active_fade = fade;
    s_from        = from;
    s_t0          = now_us;

    switch (e) {

    case FACE_THINKING: {
        /* Start the breathing cycle at the phase matching the current level,
         * moving in the same direction, so there is no visible jump. */
        float theta = acosf(1.0f - 2.0f * clamp01(from));
        if (!s_rising) {
            theta = TWO_PI_F - theta;
        }
        s_t0 = now_us - (int64_t)(theta / TWO_PI_F * (float)breath_period_us(fade));
        s_ramp_dur = 0.0f;
        break;
    }

    case FACE_CONSTANT:
        s_ramp_dur = fade * (1.0f - from);      /* only the part still to go */
        break;

    case FACE_OFF:
    default:
        s_ramp_dur = fade * from;
        break;
    }
}

static void face_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(1000 / FACE_TICK_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t   last_duty = 0;

    for (;;) {
        int64_t now_us = esp_timer_get_time();

        taskENTER_CRITICAL(&s_lock);
        face_emotion_t target       = s_target;
        float          target_fade  = s_target_fade;
        bool           manual       = s_manual;
        float          manual_level = s_manual_level;
        bool           resync       = s_resync;
        s_resync = false;
        taskEXIT_CRITICAL(&s_lock);

        if (resync || target != s_active || target_fade != s_active_fade) {
            enter_emotion(target, target_fade, now_us);
        }

        float level;
        switch (s_active) {
        case FACE_THINKING: level = breathe(now_us);    break;
        case FACE_CONSTANT: level = ramp(now_us, 1.0f); break;
        case FACE_OFF:
        default:            level = ramp(now_us, 0.0f); break;
        }

        bool lit;
        if (manual) {
            level = manual_level;
            lit   = (level > 0.0f);
        } else {
            lit = (s_active != FACE_OFF) || (level > 0.0f);
        }

        if (level > s_last_level) {
            s_rising = true;
        } else if (level < s_last_level) {
            s_rising = false;
        }
        s_last_level = level;

        uint32_t duty = level_to_duty(level, lit);
        if (duty != last_duty) {
            ledc_set_duty(PWM_UTIL_LEDC_MODE, LEDC_CH_FACE, duty);
            ledc_update_duty(PWM_UTIL_LEDC_MODE, LEDC_CH_FACE);
            last_duty = duty;
        }

        xTaskDelayUntil(&last_wake, period);
    }
}

esp_err_t face_init(void)
{
    esp_err_t err = pwm_util_timer_init(LEDC_TIMER_FACE, FACE_PWM_HZ, &s_duty_full);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer failed: %s", esp_err_to_name(err));
        return err;
    }
    s_min_frac = clamp01((float)FACE_MIN_PULSE_US * (float)FACE_PWM_HZ / 1e6f);

    ledc_channel_config_t ch = {
        .speed_mode = PWM_UTIL_LEDC_MODE,
        .channel    = LEDC_CH_FACE,
        .timer_sel  = LEDC_TIMER_FACE,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = PIN_FACE,
        .duty       = 0,
        .hpoint     = 0,
        .flags.output_invert = FACE_INVERT,
    };
    err = ledc_channel_config(&ch);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "face (GPIO%d) failed: %s", PIN_FACE, esp_err_to_name(err));
        return err;
    }
    s_t0 = esp_timer_get_time();

    if (xTaskCreate(face_task, "face", 3072, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "init ok: GPIO%d, %d Hz, floor %.1f%%, cap %.0f%%",
             PIN_FACE, FACE_PWM_HZ, s_min_frac * 100.0f, s_cap * 100.0f);
    return ESP_OK;
}

void face_set_emotion(face_emotion_t e, float fade_s)
{
    if (!(fade_s >= FACE_FADE_MIN)) {
        fade_s = FACE_FADE_MIN;
    }

    taskENTER_CRITICAL(&s_lock);
    if (s_manual) {
        s_resync = true;        /* continue from wherever manual mode left the level */
    }
    s_manual      = false;
    s_target_fade = fade_s;
    s_target      = e;
    taskEXIT_CRITICAL(&s_lock);
}

void face_set_level(float level)
{
    taskENTER_CRITICAL(&s_lock);
    s_manual_level = clamp01(level);
    s_manual       = true;
    taskEXIT_CRITICAL(&s_lock);
}

void face_set_brightness_cap(float cap)
{
    s_cap = clamp01(cap);
    ESP_LOGI(TAG, "brightness cap -> %.0f%%", s_cap * 100.0f);
}
