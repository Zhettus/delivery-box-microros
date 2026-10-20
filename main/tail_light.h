/*
 * tail_light.h — The four tail-light functions, plain on/off outputs.
 */
#pragma once

#include <stdbool.h>

void tail_lights_init(void);    /* all off */

void tail_lights_set_running(bool enable);
void tail_lights_set_brake(bool enable);
void tail_lights_set_left_turn(bool enable);
void tail_lights_set_right_turn(bool enable);

void tail_lights_off_all(void);
