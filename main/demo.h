/*
 * demo.h — Cycle through the robot states automatically (bench demo).
 *
 * Order = the original final.c:
 *   startup -> moving_with_task -> success_waiting -> success_open_door
 *   -> success_close_door
 *   -> moving_without_task -> charging -> failure -> failure_not_critical
 *   -> (again), 5 s each
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/**
 * Start cycling, one state every `step_ms`. `move_door` = false keeps the
 * door still during the demo (lights, eyes, tail, sound only).
 */
esp_err_t demo_start(uint32_t step_ms, bool move_door);
void      demo_stop(void);
bool      demo_running(void);
