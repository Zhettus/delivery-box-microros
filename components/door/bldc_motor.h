/*
 * bldc_motor.h — Brushless motor with a built-in driver (private to the door component).
 *
 * The motor's own electronics commutate the coils; the ESP32 only sets:
 *   PWM   duty 0 = stop, higher duty = faster (or the reverse if pwm_inverted)
 *   DIR   rotation direction
 *   BRAKE brake (optional)
 * The caller (door.c) guarantees the motor is at standstill before DIR changes.
 */
#pragma once

#include <stdint.h>

#include "driver/ledc.h"
#include "esp_err.h"

#include "door.h"

typedef struct {
    door_bldc_config_t pins;
    ledc_timer_t       timer;
    ledc_channel_t     channel;
    uint32_t           freq_hz;
} bldc_motor_config_t;

/** Configure the outputs with the motor stopped and braked. */
esp_err_t bldc_motor_init(const bldc_motor_config_t *cfg);

/** Run in `dir` (+1 open / -1 close) at `duty` (0..1). dir 0 = stop. */
void bldc_motor_drive(int8_t dir, float duty);

/** Stop: speed 0 and brake on (if there is a brake wire). */
void bldc_motor_stop(void);
