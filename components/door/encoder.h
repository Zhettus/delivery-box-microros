/*
 * encoder.h — Motor position feedback on PCNT (private to door).
 *
 * Two modes, both accumulate past the 16-bit hardware limit:
 *   quadrature  2-channel Hall encoder, x4 decoding, signed count
 *   pulses      single pulse wire (BLDC "FG"), rising edges, always counts up
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

esp_err_t encoder_init(gpio_num_t a, gpio_num_t b, bool invert);

/** Count rising edges of one pulse wire (internal pull-up enabled). */
esp_err_t encoder_init_pulses(gpio_num_t pin);

/** Accumulated count since start (quadrature: 4 per pulse, signed; pulses: 1 per pulse). */
int32_t encoder_get_count(void);
