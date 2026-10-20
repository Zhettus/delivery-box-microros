/*
 * states.h — The robot states. Setting one switches the whole box: LED strip,
 * face, tail lights, sound and the door.
 *
 *   state                  strip           face      tail    door    sound
 *   startup                white pulse     thinking  off     -       startup
 *   moving_with_task       blue pulse      on        off     closes  -
 *   moving_without_task    white pulse     on        off     closes  -
 *   success_waiting        green pulse     on        off     -       arrived
 *   success_open_door      green solid     on        off     opens   greeting (gm.wav)
 *   success_close_door     green solid     on        off     closes  -
 *   failure                red pulse       on        brake   stops   failure
 *   failure_not_critical   yellow pulse    on        off     -       warning
 *   charging               cyan comet      follows   off     -       -
 *
 * states_set() returns at once: a background task applies the state (and
 * runs the charging animation until the next state). Safe from any task,
 * including the micro-ROS one. If someone holds the door while it moves, it
 * goes back (closing -> open, opening -> closed) and the box beeps.
 */
#pragma once

#include <stdbool.h>

typedef enum {
    STATE_STARTUP = 0,
    STATE_MOVING_WITH_TASK,
    STATE_MOVING_WITHOUT_TASK,
    STATE_SUCCESS_WAITING,
    STATE_SUCCESS_OPEN_DOOR,
    STATE_SUCCESS_CLOSE_DOOR,
    STATE_FAILURE,
    STATE_FAILURE_NOT_CRITICAL,
    STATE_CHARGING,
    STATE_COUNT,
} robot_state_t;

/** Start the state task (call after the LEDs, face, motor and sound). */
void states_init(void);

/** Switch to `s` (the newest request wins). */
void states_set(robot_state_t s);

/** The state last applied (or requested, if not applied yet). */
robot_state_t states_get(void);

const char *state_name(robot_state_t s);
bool        state_from_name(const char *name, robot_state_t *out);
