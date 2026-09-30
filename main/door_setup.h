/*
 * door_setup.h — Build the door_config_t from board.h + app_config.h.
 * Used by the normal firmware (main.c).
 */
#pragma once

#include "app_config.h"
#include "board.h"
#include "door.h"
#include "ina228.h"

static inline door_config_t app_door_config(void)
{
    door_config_t c = {
        .pwm_timer   = LEDC_TIMER_MOTOR,
        .pwm_channel = LEDC_CH_MOTOR,

        .kp                = APP_DOOR_KP,
        .ki                = APP_DOOR_KI,
        .integral_limit    = APP_DOOR_INTEGRAL_LIMIT,
        .speed_window_ms   = APP_DOOR_SPEED_WINDOW_MS,
        .control_period_ms = APP_DOOR_CONTROL_MS,

        .open_angle_deg    = APP_DOOR_OPEN_DEG,
        .tolerance_deg     = APP_DOOR_TOLERANCE_DEG,
        .slowdown_deg      = APP_DOOR_SLOWDOWN_DEG,
        .cruise_rpm        = APP_DOOR_CRUISE_RPM,
        .approach_rpm      = APP_DOOR_APPROACH_RPM,
        .homing_rpm        = APP_DOOR_HOMING_RPM,
        .spinup_grace_ms   = APP_DOOR_SPINUP_GRACE_MS,
        .obstruction_ms    = APP_DOOR_OBSTRUCTION_MS,
        .obstruction_ratio = APP_DOOR_OBSTRUCTION_RATIO,
        .retry_wait_ms     = APP_DOOR_RETRY_WAIT_MS,
        .max_retries       = APP_DOOR_MAX_RETRIES,
        .move_timeout_ms   = APP_DOOR_MOVE_TIMEOUT_MS,
        .home_at_boot      = APP_DOOR_HOME_AT_BOOT,
        .current = {
            .limit_a         = APP_DOOR_CURRENT_LIMIT_A,
            .rise_a          = APP_DOOR_CURRENT_RISE_A,
            .confirm_ms      = APP_DOOR_CURRENT_CONFIRM_MS,
            .ignore_start_ms = APP_DOOR_CURRENT_IGNORE_MS,
        },
        .sense_period_ms   = 5,
        .hold_open_ms      = APP_DOOR_HOLD_OPEN_MS,
        .auto_close_rpm    = APP_DOOR_CLOSE_RPM,
        .latch = {
            .gpio        = APP_LATCH_ENABLED ? PIN_DOOR_LATCH : GPIO_NUM_NC,
            .timer       = LEDC_TIMER_LATCH,
            .channel     = LEDC_CH_LATCH,
            .pwm_freq_hz = APP_LATCH_PWM_HZ,
            .timing = {
                .unlock_ms  = APP_LATCH_UNLOCK_MS,
                .settle_ms  = APP_LATCH_SETTLE_MS,
                .pull_in_ms = APP_LATCH_PULL_IN_MS,
                .hold_duty  = APP_LATCH_HOLD_DUTY,
            },
        },
    };

#if APP_DOOR_MOTOR_TYPE == 0
    c.motor_type = DOOR_MOTOR_BLDC;
    c.bldc = (door_bldc_config_t){
        .pwm_gpio       = PIN_MOTOR_PWM,
        .dir_gpio       = PIN_MOTOR_DIR,
        .brake_gpio     = APP_BLDC_HAS_BRAKE ? PIN_MOTOR_BRAKE : GPIO_NUM_NC,
        .fg_gpio        = PIN_MOTOR_FG,
        .dir_level_open = APP_BLDC_DIR_LEVEL_OPEN,
        .brake_level_on = APP_BLDC_BRAKE_LEVEL_ON,
        .pwm_inverted   = APP_BLDC_PWM_INVERTED,
    };
    c.pwm_freq_hz             = APP_BLDC_PWM_HZ;
    c.counts_per_output_rev   = APP_BLDC_FG_PER_OUTPUT_REV;
    c.output_rpm_at_full_duty = APP_BLDC_OUTPUT_RPM_FULL;
    c.min_duty                = APP_BLDC_MIN_DUTY;
    c.stop_before_reverse_ms  = APP_BLDC_STOP_BEFORE_REV_MS;
#else
    c.motor_type = DOOR_MOTOR_DRI0042;
    c.dri0042 = (door_dri0042_config_t){
        .in1_gpio       = PIN_MOTOR_DIR,
        .in2_gpio       = PIN_MOTOR_BRAKE,
        .pwm_gpio       = PIN_MOTOR_PWM,
        .hall_a_gpio    = PIN_MOTOR_FG,
        .hall_b_gpio    = PIN_MOTOR_AUX,
        .min_pulse_ns   = APP_DRI0042_MIN_PULSE_NS,
        .invert_motor   = APP_DRI0042_INVERT_MOTOR,
        .invert_encoder = APP_DRI0042_INVERT_ENCODER,
    };
    c.pwm_freq_hz             = APP_DRI0042_PWM_HZ;
    c.counts_per_output_rev   = 4.0f * APP_DRI0042_ENCODER_PPR * APP_DRI0042_GEAR_RATIO;
    c.output_rpm_at_full_duty = APP_DRI0042_MOTOR_RPM_FULL / APP_DRI0042_GEAR_RATIO;
    c.min_duty                = APP_DRI0042_MIN_DUTY;
    c.stop_before_reverse_ms  = APP_DRI0042_STOP_BEFORE_REV_MS;
#endif
    return c;
}

/* door_current_reader_t for the INA228 (pass the handle as ctx). */
static inline bool app_door_read_ina228(float *amps, void *ctx)
{
    return ina228_read_current((ina228_handle_t)ctx, amps) == ESP_OK;
}
