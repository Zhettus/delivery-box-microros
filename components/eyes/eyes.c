/*
 * eyes.c — Eye LEDs over LEDC. See eyes.h.
 */
#include <math.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "eyes.h"
#include "pwm_util.h"

static const char *TAG = "eyes";

#define EYE_TICK_HZ     100
#define TWO_PI_F        6.2831853f

static eyes_config_t s_cfg;
static portMUX_TYPE  s_lock = portMUX_INITIALIZER_UNLOCKED;

/* Requested by the API (guarded by s_lock). */
static eye_emotion_t s_target       = EYE_OFF;
static float         s_target_fade  = 1.0f;
static bool          s_manual       = false;
static float         s_manual_level = 0.0f;
static bool          s_resync       = false;
static volatile float s_cap;
static volatile float s_min_frac;

/* Owned by the eyes task. */
static volatile eye_emotion_t s_active        = EYE_OFF;
static volatile float         s_active_fade   = 1.0f;
static volatile bool          s_transitioning = false;
static int64_t  s_t0;
static float    s_from;
static float    s_ramp_dur;
static float    s_last_level;
static bool     s_rising;
static uint32_t s_duty_full;
static uint32_t s_last_duty = UINT32_MAX;

static float clamp01(float x)
{
    return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
}

static float pulse_to_frac(uint32_t us)
{
    return clamp01((float)us * (float)s_cfg.pwm_freq_hz / 1e6f);
}

/* 0..1 level -> LEDC duty, between the JZ-MOS minimum pulse and the cap. */
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
    float d = min + powf(clamp01(level), s_cfg.gamma) * (top - min);
    uint32_t duty = (uint32_t)(d + 0.5f);
    return duty > s_duty_full ? s_duty_full : duty;
}

static float smoothstep(float x)
{
    x = clamp01(x);
    return x * x * (3.0f - 2.0f * x);
}

static int64_t breath_period_us(float fade)
{
    int64_t p = (int64_t)(2.0f * fade * 1e6f);
    return p > 0 ? p : 1;
}

static float breathe(int64_t now_us)
{
    int64_t period = breath_period_us(s_active_fade);
    int64_t ph = (now_us - s_t0) % period;
    if (ph < 0) {
        ph += period;
    }
    return 0.5f - 0.5f * cosf(TWO_PI_F * (float)ph / (float)period);
}

static float ramp(int64_t now_us, float to)
{
    float x = 1.0f;
    if (s_ramp_dur > 0.0f) {
        x = ((float)(now_us - s_t0) / 1e6f) / s_ramp_dur;
    }
    if (x >= 1.0f) {
        s_transitioning = false;
        return to;
    }
    return s_from + (to - s_from) * smoothstep(x);
}

static void enter_state(eye_emotion_t e, float fade, int64_t now_us)
{
    float from = s_last_level;

    s_active      = e;
    s_active_fade = fade;
    s_from        = from;
    s_t0          = now_us;

    switch (e) {
    case EYE_THINKING: {
        /* Start the breathing cycle at the phase matching the current level,
         * moving in the same direction, so there is no visible jump. */
        float c = clamp01(from) * -2.0f + 1.0f;
        float theta = acosf(c);
        if (!s_rising) {
            theta = TWO_PI_F - theta;
        }
        s_t0 = now_us - (int64_t)(theta / TWO_PI_F * (float)breath_period_us(fade));
        s_ramp_dur      = 0.0f;
        s_transitioning = false;
        break;
    }
    case EYE_CONSTANT:
        s_ramp_dur      = fade * (1.0f - from);
        s_transitioning = (s_ramp_dur > 0.0f);
        break;

    case EYE_OFF:
    default:
        s_ramp_dur      = fade * from;
        s_transitioning = (s_ramp_dur > 0.0f);
        break;
    }
}

static void write_duty(uint32_t duty)
{
    ledc_set_duty(PWM_UTIL_LEDC_MODE, s_cfg.left_channel, duty);
    ledc_update_duty(PWM_UTIL_LEDC_MODE, s_cfg.left_channel);
    ledc_set_duty(PWM_UTIL_LEDC_MODE, s_cfg.right_channel, duty);
    ledc_update_duty(PWM_UTIL_LEDC_MODE, s_cfg.right_channel);
}

static void eyes_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(1000 / EYE_TICK_HZ);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        int64_t now_us = esp_timer_get_time();

        taskENTER_CRITICAL(&s_lock);
        eye_emotion_t target       = s_target;
        float         target_fade  = s_target_fade;
        bool          manual       = s_manual;
        float         manual_level = s_manual_level;
        bool          resync       = s_resync;
        s_resync = false;
        taskEXIT_CRITICAL(&s_lock);

        if (resync || target != s_active || target_fade != s_active_fade) {
            enter_state(target, target_fade, now_us);
        }

        float level;
        switch (s_active) {
        case EYE_THINKING: level = breathe(now_us);    break;
        case EYE_CONSTANT: level = ramp(now_us, 1.0f); break;
        case EYE_OFF:
        default:           level = ramp(now_us, 0.0f); break;
        }

        bool lit;
        if (manual) {
            level = manual_level;
            lit   = level > 0.0f;
        } else {
            lit = (s_active != EYE_OFF) || (level > 0.0f);
        }

        if (level > s_last_level) {
            s_rising = true;
        } else if (level < s_last_level) {
            s_rising = false;
        }
        s_last_level = level;

        uint32_t duty = level_to_duty(level, lit);
        if (duty != s_last_duty) {
            write_duty(duty);
            s_last_duty = duty;
        }

        xTaskDelayUntil(&last_wake, period);
    }
}

esp_err_t eyes_init(const eyes_config_t *cfg)
{
    s_cfg      = *cfg;
    s_cap      = clamp01(cfg->brightness_cap);
    s_min_frac = pulse_to_frac(cfg->min_pulse_us);

    ESP_RETURN_ON_ERROR(pwm_util_timer_init(cfg->timer, cfg->pwm_freq_hz, &s_duty_full),
                        TAG, "LEDC timer");
    const gpio_num_t     gpios[2] = { cfg->left_gpio, cfg->right_gpio };
    const ledc_channel_t chans[2] = { cfg->left_channel, cfg->right_channel };
    for (int i = 0; i < 2; i++) {
        ledc_channel_config_t ch = {
            .gpio_num   = gpios[i],
            .speed_mode = PWM_UTIL_LEDC_MODE,
            .channel    = chans[i],
            .timer_sel  = cfg->timer,
            .intr_type  = LEDC_INTR_DISABLE,
            .duty       = 0,
            .hpoint     = 0,
            .flags.output_invert = cfg->invert,     /* original EYE_INVERT */
        };
        ESP_RETURN_ON_ERROR(ledc_channel_config(&ch), TAG, "channel %d", i);
    }

    s_last_duty = 0;
    s_t0 = esp_timer_get_time();

    if (xTaskCreate(eyes_task, "eyes", 3072, NULL, 4, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "ready: left=GPIO%d right=GPIO%d, %lu Hz, floor %lu us (%.2f%%), cap %.0f%%%s",
             cfg->left_gpio, cfg->right_gpio, (unsigned long)cfg->pwm_freq_hz,
             (unsigned long)cfg->min_pulse_us, s_min_frac * 100.0f, s_cap * 100.0f,
             cfg->invert ? ", inverted" : "");
    if (s_min_frac > 0.0f && s_cap <= s_min_frac * 1.5f) {
        ESP_LOGW(TAG, "brightness cap is close to the minimum-pulse floor: "
                      "breathing will look flat, raise the cap");
    }
    return ESP_OK;
}

void eyes_set_emotion(eye_emotion_t e, float fade_s)
{
    if (!(fade_s >= EYE_FADE_MIN_S)) {
        fade_s = EYE_FADE_MIN_S;
    }
    taskENTER_CRITICAL(&s_lock);
    if (s_manual) {
        s_resync = true;   /* restart from the manual level */
    }
    s_manual      = false;
    s_target_fade = fade_s;
    s_target      = e;
    taskEXIT_CRITICAL(&s_lock);
}

void eyes_set_level(float level)
{
    taskENTER_CRITICAL(&s_lock);
    s_manual_level = clamp01(level);
    s_manual       = true;
    taskEXIT_CRITICAL(&s_lock);
}

void eyes_set_brightness_cap(float cap)
{
    s_cap = clamp01(cap);
    ESP_LOGI(TAG, "brightness cap -> %.1f%%", s_cap * 100.0f);
}

void eyes_set_min_pulse_us(uint32_t us)
{
    s_min_frac = pulse_to_frac(us);
}

eye_emotion_t eyes_get_emotion(void)       { return s_active; }
float         eyes_get_brightness_cap(void) { return s_cap; }
bool          eyes_is_transitioning(void)  { return s_transitioning; }

static const char *const s_names[] = {
    [EYE_OFF]      = "off",
    [EYE_THINKING] = "thinking",
    [EYE_CONSTANT] = "constant",
};

const char *eye_emotion_name(eye_emotion_t e)
{
    return (e <= EYE_CONSTANT) ? s_names[e] : "?";
}

bool eye_emotion_from_name(const char *name, eye_emotion_t *out)
{
    for (int e = 0; e <= EYE_CONSTANT; e++) {
        if (strcmp(name, s_names[e]) == 0) {
            *out = (eye_emotion_t)e;
            return true;
        }
    }
    return false;
}
