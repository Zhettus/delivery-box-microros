
#include <stdbool.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"

#include "pwm_util.h"
#include "tail_lights.h"

static const char *TAG = "tail";

static const char *const s_names[TAIL_LIGHT_COUNT] = {
    [TAIL_LIGHT_BRAKE] = "brake",
    [TAIL_LIGHT_RUN]   = "run",
    [TAIL_LIGHT_LEFT]  = "left",
    [TAIL_LIGHT_RIGHT] = "right",
};

static tail_lights_config_t s_cfg;
static uint32_t             s_duty_full;
static uint8_t              s_level[TAIL_LIGHT_COUNT];
static bool                 s_ready;

static uint32_t level_to_duty(uint8_t level)
{
    return (uint32_t)(((uint64_t)level * s_duty_full + 127) / 255);
}

esp_err_t tail_lights_init(const tail_lights_config_t *cfg)
{
    s_cfg = *cfg;

    ESP_RETURN_ON_ERROR(pwm_util_timer_init(cfg->timer, cfg->pwm_freq_hz, &s_duty_full),
                        TAG, "LEDC timer");
    for (int i = 0; i < TAIL_LIGHT_COUNT; i++) {
        ESP_RETURN_ON_ERROR(pwm_util_channel_init(cfg->channel[i], cfg->timer, cfg->gpio[i]),
                            TAG, "LEDC channel %d", i);
        s_level[i] = 0;
    }
    ESP_RETURN_ON_ERROR(pwm_util_fade_service(), TAG, "fade service");

    s_ready = true;
    ESP_LOGI(TAG, "ready: brake=GPIO%d run=GPIO%d left=GPIO%d right=GPIO%d, %lu Hz",
             cfg->gpio[TAIL_LIGHT_BRAKE], cfg->gpio[TAIL_LIGHT_RUN],
             cfg->gpio[TAIL_LIGHT_LEFT], cfg->gpio[TAIL_LIGHT_RIGHT],
             (unsigned long)cfg->pwm_freq_hz);
    return ESP_OK;
}

void tail_lights_set(tail_light_t which, uint8_t level)
{
    if (!s_ready || which >= TAIL_LIGHT_COUNT) {
        return;
    }
    ledc_channel_t ch = s_cfg.channel[which];
    ledc_fade_stop(PWM_UTIL_LEDC_MODE, ch);          /* cancel a running fade */
    ledc_set_duty_and_update(PWM_UTIL_LEDC_MODE, ch, level_to_duty(level), 0);
    s_level[which] = level;
}

void tail_lights_fade(tail_light_t which, uint8_t level, uint32_t ms)
{
    if (!s_ready || which >= TAIL_LIGHT_COUNT) {
        return;
    }
    if (ms == 0) {
        tail_lights_set(which, level);
        return;
    }
    ledc_channel_t ch = s_cfg.channel[which];
    ledc_fade_stop(PWM_UTIL_LEDC_MODE, ch);
    ledc_set_fade_time_and_start(PWM_UTIL_LEDC_MODE, ch, level_to_duty(level), ms,
                                 LEDC_FADE_NO_WAIT);
    s_level[which] = level;
}

void tail_lights_apply(const tail_lights_pattern_t *p)
{
    tail_lights_set(TAIL_LIGHT_BRAKE, p->brake);
    tail_lights_set(TAIL_LIGHT_RUN,   p->run);
    tail_lights_set(TAIL_LIGHT_LEFT,  p->left);
    tail_lights_set(TAIL_LIGHT_RIGHT, p->right);
}

void tail_lights_off(void)
{
    tail_lights_apply(&(tail_lights_pattern_t){ 0 });
}

uint8_t tail_lights_get(tail_light_t which)
{
    return (which < TAIL_LIGHT_COUNT) ? s_level[which] : 0;
}

const char *tail_light_name(tail_light_t which)
{
    return (which < TAIL_LIGHT_COUNT) ? s_names[which] : "?";
}

bool tail_light_from_name(const char *name, tail_light_t *out)
{
    for (int i = 0; i < TAIL_LIGHT_COUNT; i++) {
        if (strcmp(name, s_names[i]) == 0) {
            *out = (tail_light_t)i;
            return true;
        }
    }
    return false;
}
