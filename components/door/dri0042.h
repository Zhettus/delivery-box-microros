/*
 * dri0042.h — Low-level DFRobot DRI0042 driver (private to the door component).
 *
 * Only knows how to put IN1/IN2/PWM into a legal state. The caller (door.c)
 * is responsible for the brake-before-reverse timing via motor_interlock.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

typedef struct {
    gpio_num_t     in1_gpio;
    gpio_num_t     in2_gpio;
    gpio_num_t     pwm_gpio;
    ledc_timer_t   timer;
    ledc_channel_t channel;
    uint32_t       freq_hz;
    uint32_t       min_pulse_ns;
    bool           invert;
} dri0042_config_t;

/** Configure pins with the motor stopped (IN1 = IN2 = 0, PWM = 0). */
esp_err_t dri0042_init(const dri0042_config_t *cfg);

/** Drive in `dir` (+1 forward / -1 reverse) at `duty` (0..1). dir 0 = stop. */
void dri0042_drive(int8_t dir, float duty);

/** STOP: IN1 = 0, IN2 = 0, PWM = 0. */
void dri0042_stop(void);
