/*
 * pwm_util.h — LEDC helpers shared by the tail lights, eyes and door motor.
 */
#pragma once

#include <stdint.h>

#include "driver/ledc.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Every LEDC timer in this project runs from the 80 MHz APB clock. On the
 * ESP32-S3 all LEDC timers must share one clock source, so keep it uniform. */
#define PWM_UTIL_LEDC_CLK      LEDC_USE_APB_CLK
#define PWM_UTIL_LEDC_CLK_HZ   80000000UL
#define PWM_UTIL_LEDC_MODE     LEDC_LOW_SPEED_MODE   /* the only mode on S3 */

/*
 * Never use the chip's maximum resolution (14 bits on the ESP32-S3): at the
 * maximum bit width a duty of 2^bits (100 %) is not reachable and the output
 * stays LOW instead of HIGH. One bit less keeps 100 % = steady high.
 */
#define PWM_UTIL_MAX_BITS  (SOC_LEDC_TIMER_BIT_WIDTH - 1)

/**
 * Highest usable duty resolution (bits) at `freq_hz`.
 * LEDC needs clk / (freq * 2^bits) >= 1.
 *   20 kHz  -> 11 bits (motor)       1.5 kHz -> 13 bits (eyes)
 */
static inline ledc_timer_bit_t pwm_util_max_resolution(uint32_t freq_hz)
{
    uint32_t bits = 1;
    while (bits < PWM_UTIL_MAX_BITS &&
           ((uint64_t)freq_hz << (bits + 1)) <= PWM_UTIL_LEDC_CLK_HZ) {
        bits++;
    }
    return (ledc_timer_bit_t)bits;
}

/** Configure one LEDC timer at `freq_hz` with the best resolution.
 *  Writes the full-scale duty (2^bits = 100 %) to `duty_full`. */
static inline esp_err_t pwm_util_timer_init(ledc_timer_t timer, uint32_t freq_hz,
                                            uint32_t *duty_full)
{
    ledc_timer_bit_t bits = pwm_util_max_resolution(freq_hz);
    ledc_timer_config_t cfg = {
        .speed_mode      = PWM_UTIL_LEDC_MODE,
        .timer_num       = timer,
        .duty_resolution = bits,
        .freq_hz         = freq_hz,
        .clk_cfg         = PWM_UTIL_LEDC_CLK,
    };
    if (duty_full) {
        *duty_full = 1UL << bits;
    }
    return ledc_timer_config(&cfg);
}

/** Attach `gpio` to LEDC `channel` driven by `timer`, starting at duty 0. */
static inline esp_err_t pwm_util_channel_init(ledc_channel_t channel,
                                              ledc_timer_t timer, int gpio)
{
    ledc_channel_config_t cfg = {
        .gpio_num   = gpio,
        .speed_mode = PWM_UTIL_LEDC_MODE,
        .channel    = channel,
        .timer_sel  = timer,
        .intr_type  = LEDC_INTR_DISABLE,
        .duty       = 0,
        .hpoint     = 0,
    };
    return ledc_channel_config(&cfg);
}

/** Install the LEDC fade service once (safe to call repeatedly). */
static inline esp_err_t pwm_util_fade_service(void)
{
    esp_err_t err = ledc_fade_func_install(0);
    return (err == ESP_ERR_INVALID_STATE) ? ESP_OK : err;   /* already installed */
}

#ifdef __cplusplus
}
#endif
