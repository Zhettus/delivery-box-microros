/*
 * power_monitor.h — Periodic INA228 readings with low-voltage / over-current warnings.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "ina228.h"

esp_err_t power_monitor_start(ina228_handle_t sensor);

/** Latest reading. Returns false if none yet. `age_ms` may be NULL. */
bool power_monitor_latest(ina228_reading_t *out, uint32_t *age_ms);
