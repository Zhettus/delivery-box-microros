/*
 * dri0042.c — Low-level DFRobot DRI0042 driver. See dri0042.h.
 */
#include "esp_check.h"
#include "esp_log.h"

#include "dri0042.h"
#include "pwm_util.h"

static const char *TAG = "dri0042";

static dri0042_config_t s_cfg;
static uint32_t         s_duty_full;
static uint32_t         s_min_counts;
static int8_t           s_pin_dir;     /* direction currently on IN1/IN2 */

static void set_pwm(uint32_t counts)
{
    ledc_set_duty(PWM_UTIL_LEDC_MODE, s_cfg.channel, counts);
    ledc_update_duty(PWM_UTIL_LEDC_MODE, s_cfg.channel);
}

/* Change IN1/IN2 so that both are never high, even for an instant. */
static void set_direction_pins(int8_t dir)
{
    if (dir == s_pin_dir) {
        return;
    }
    gpio_set_level(s_cfg.in1_gpio, 0);
    gpio_set_level(s_cfg.in2_gpio, 0);
    if (dir > 0) {
        gpio_set_level(s_cfg.in1_gpio, 1);
    } else if (dir < 0) {
        gpio_set_level(s_cfg.in2_gpio, 1);
    }
    s_pin_dir = dir;
}

esp_err_t dri0042_init(const dri0042_config_t *cfg)
{
    s_cfg     = *cfg;
    s_pin_dir = 0;

    /* Latch LOW before switching the pins to output: no glitch at boot. */
    gpio_set_level(cfg->in1_gpio, 0);
    gpio_set_level(cfg->in2_gpio, 0);
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << cfg->in1_gpio) | (1ULL << cfg->in2_gpio),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "IN1/IN2");
    gpio_set_level(cfg->in1_gpio, 0);
    gpio_set_level(cfg->in2_gpio, 0);

    ESP_RETURN_ON_ERROR(pwm_util_timer_init(cfg->timer, cfg->freq_hz, &s_duty_full),
                        TAG, "LEDC timer");
    ESP_RETURN_ON_ERROR(pwm_util_channel_init(cfg->channel, cfg->timer, cfg->pwm_gpio),
                        TAG, "LEDC channel");

    /* Shortest non-zero pulse the DRI0042 accepts, in LEDC counts. */
    uint64_t period_ns = 1000000000ULL / cfg->freq_hz;
    s_min_counts = (uint32_t)(((uint64_t)cfg->min_pulse_ns * s_duty_full + period_ns - 1)
                              / period_ns);

    ESP_LOGI(TAG, "ready: IN1=GPIO%d IN2=GPIO%d PWM=GPIO%d @ %lu Hz (%lu steps)",
             cfg->in1_gpio, cfg->in2_gpio, cfg->pwm_gpio,
             (unsigned long)cfg->freq_hz, (unsigned long)s_duty_full);
    return ESP_OK;
}

void dri0042_stop(void)
{
    set_pwm(0);
    set_direction_pins(0);
}

void dri0042_drive(int8_t dir, float duty)
{
    if (dir == 0) {
        dri0042_stop();
        return;
    }
    if (s_cfg.invert) {
        dir = (int8_t)-dir;
    }

    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    uint32_t counts = (uint32_t)(duty * (float)s_duty_full + 0.5f);
    if (counts > 0 && counts < s_min_counts) {
        counts = s_min_counts;
    }

    if (dir != s_pin_dir) {
        set_pwm(0);   /* never switch direction pins with power applied */
    }
    set_direction_pins(dir);
    set_pwm(counts);
}
