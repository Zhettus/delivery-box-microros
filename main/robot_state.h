/*
 * robot_state.h — High-level states of the delivery robot, as seen on the box.
 *
 * Setting a state updates everything at once: LED strips, eyes, tail light,
 * sound, and (for some states) the door. The table in robot_state.c
 * defines what each state looks like.
 */
#pragma once

#include <stdbool.h>

typedef enum {
    ROBOT_STATE_STARTUP = 0,          /* booting / initialising                 */
    ROBOT_STATE_MOVING_WITH_TASK,     /* driving to deliver a package           */
    ROBOT_STATE_MOVING_WITHOUT_TASK,  /* driving, box empty                     */
    ROBOT_STATE_SUCCESS_WAITING,      /* arrived, waiting for the customer      */
    ROBOT_STATE_SUCCESS_OPEN_DOOR,    /* customer verified: door opens, stays open */
    ROBOT_STATE_SUCCESS_CLOSE_DOOR,   /* package taken: door closes + locks     */
    ROBOT_STATE_FAILURE,              /* critical failure: robot must stop      */
    ROBOT_STATE_FAILURE_NOT_CRITICAL, /* problem, but robot can continue        */
    ROBOT_STATE_CHARGING,             /* docked and charging                    */
    ROBOT_STATE_COUNT,
} robot_state_t;

/**
 * @param door_actions  false = never move the door automatically
 *                      (bench demo without the mechanism)
 */
void robot_state_init(bool door_actions);

void          robot_state_set(robot_state_t s);

/** Allow / forbid the states to move the door (e.g. demo without door). */
void          robot_state_set_door_actions(bool on);
bool          robot_state_door_actions(void);
robot_state_t robot_state_get(void);

const char *robot_state_name(robot_state_t s);
bool        robot_state_from_name(const char *name, robot_state_t *out);
