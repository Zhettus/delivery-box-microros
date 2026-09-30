/*
 * door_lock.h — Latch solenoid sequencing (pure logic, no ESP-IDF;
 * unit-tested in test/host/test_door_fsm.c).
 *
 *   LOCKED     solenoid OFF (latch extended), motor must not move
 *      | the door wants to move
 *      v
 *   UNLOCKING  solenoid ON (retracting), motor still held for unlock_ms
 *      v
 *   UNLOCKED   solenoid ON, motor free (opening, open, closing, ...)
 *      | door back at the CLOSED position and stopped
 *      v
 *   LOCKING    motor off, wait settle_ms for the door to seat
 *      v          (a new move during this wait -> back to UNLOCKED)
 *   LOCKED     solenoid OFF
 *
 * Power: full power for pull_in_ms after switching on, then hold_duty
 * (1.0 = keep full power).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DOOR_LOCK_LOCKED = 0,
    DOOR_LOCK_UNLOCKING,
    DOOR_LOCK_UNLOCKED,
    DOOR_LOCK_LOCKING,
} door_lock_state_t;

typedef struct {
    uint32_t unlock_ms;     /* solenoid ON -> latch fully retracted (100-200 ms) */
    uint32_t settle_ms;     /* door seated -> extend the latch (~100 ms)         */
    uint32_t pull_in_ms;    /* full power this long after switching on           */
    float    hold_duty;     /* power after pull-in, 0..1 (1 = full)               */
} door_lock_config_t;

typedef struct {
    door_lock_state_t state;
    uint32_t          since_ms;     /* entered the current state            */
    uint32_t          on_since_ms;  /* solenoid switched on (for pull-in)   */
    bool              on;           /* solenoid currently powered           */
} door_lock_t;

typedef struct {
    float solenoid;         /* 0 = off (locked), otherwise power 0..1          */
    bool  motion_allowed;   /* false: keep the motor stopped this tick         */
    bool  changed;          /* lock state changed this tick                    */
} door_lock_output_t;

/** Start LOCKED (door assumed closed at power-up) or UNLOCKED. */
void door_lock_init(door_lock_t *l, bool locked, uint32_t now_ms);

/**
 * @param wants_motion   the door state machine wants the motor to turn
 * @param at_closed_rest door at the closed position, motor stopped
 */
door_lock_output_t door_lock_step(door_lock_t *l, const door_lock_config_t *c,
                                  uint32_t now_ms, bool wants_motion,
                                  bool at_closed_rest);

const char *door_lock_state_name(door_lock_state_t s);

#ifdef __cplusplus
}
#endif
