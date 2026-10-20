/*
 * bldc_motor.c — Brushless motor with a built-in driver. See bldc_motor.h.
 */
#include "esp_check.h"
#include "esp_log.h"

#include "bldc_motor.h"
#include "pwm_util.h"

static const char *TAG = "bldc";

static bldc_motor_config_t s_cfg;
static uint32_t            s_duty_full;
static int8_t              s_dir_set;       /* direction currently on DIR (+1/-1) */

static bool has_brake(void)
{
    return s_cfg.pins.brake_gpio != GPIO_NUM_NC;
}

static void set_pwm(uint32_t counts)
{
    ledc_set_duty(PWM_UTIL_LEDC_MODE, s_cfg.channel, counts);
    ledc_update_duty(PWM_UTIL_LEDC_MODE, s_cfg.channel);
}

static void set_brake(bool on)
{
    if (has_brake()) {
        uint8_t lvl = s_cfg.pins.brake_level_on;
        gpio_set_level(s_cfg.pins.brake_gpio, on ? lvl : !lvl);
    }
}

static void set_dir(int8_t dir)
{
    uint8_t open  = s_cfg.pins.dir_level_open;
    int     level = dir > 0 ? open : !open;
    gpio_set_level(s_cfg.pins.dir_gpio, level);
    s_dir_set = dir;
    ESP_EARLY_LOGD(TAG, "DIR GPIO%d -> %d (%s)", s_cfg.pins.dir_gpio, level,
                   dir > 0 ? "open" : "close");
}

esp_err_t bldc_motor_init(const bldc_motor_config_t *cfg)
{
    s_cfg = *cfg;

    /* Latch safe levels before the pins become outputs: brake on, DIR = open. */
    uint64_t mask = 1ULL << cfg->pins.dir_gpio;
    if (has_brake()) {
        mask |= 1ULL << cfg->pins.brake_gpio;
    }
    set_brake(true);
    set_dir(+1);
    gpio_config_t io = {
        .pin_bit_mask = mask,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "DIR/BRAKE");
    if (cfg->pins.dir_open_drain) {
        ESP_RETURN_ON_ERROR(gpio_set_direction(cfg->pins.dir_gpio, GPIO_MODE_OUTPUT_OD),
                            TAG, "DIR open-drain");
    }
    set_brake(true);
    set_dir(+1);

    /* PWM: duty 0 = stop. pwm_inverted flips the pin so duty still means speed. */
    ESP_RETURN_ON_ERROR(pwm_util_timer_init(cfg->timer, cfg->freq_hz, &s_duty_full),
                        TAG, "LEDC timer");
    ledc_channel_config_t ch = {
        .gpio_num   = cfg->pins.pwm_gpio,
        .speed_mode = PWM_UTIL_LEDC_MODE,
        .channel    = cfg->channel,
        .timer_sel  = cfg->timer,
        .intr_type  = LEDC_INTR_DISABLE,
        .duty       = 0,
        .hpoint     = 0,
        .flags.output_invert = cfg->pins.pwm_inverted,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch), TAG, "LEDC channel");

    ESP_LOGI(TAG, "ready: PWM=GPIO%d @ %lu Hz%s, DIR=GPIO%d (open=%d%s), BRAKE=%s",
             cfg->pins.pwm_gpio, (unsigned long)cfg->freq_hz,
             cfg->pins.pwm_inverted ? " (inverted)" : "",
             cfg->pins.dir_gpio, cfg->pins.dir_level_open,
             cfg->pins.dir_open_drain ? ", open-drain" : "",
             has_brake() ? "on" : "none");
    return ESP_OK;
}

void bldc_motor_stop(void)
{
    set_pwm(0);
    set_brake(true);
}

void bldc_motor_drive(int8_t dir, float duty)
{
    if (dir == 0) {
        bldc_motor_stop();
        return;
    }
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    if (dir != s_dir_set) {
        set_pwm(0);           /* never change direction with speed applied */
        set_dir(dir);
    }
    set_brake(false);
    set_pwm((uint32_t)(duty * (float)s_duty_full + 0.5f));
}
