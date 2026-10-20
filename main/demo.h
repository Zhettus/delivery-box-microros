/*
 * demo.h — Bench demo: cycle through all robot states (DEMO_MODE, config.h)
 * until the first command arrives from ROS.
 */
#pragma once

#include <stdbool.h>

void demo_start(void);
void demo_stop(void);
bool demo_running(void);
