/*
 * motor.h — The door motor, made easy.
 *
 * The motor is a brushless gear motor with a built-in driver. This file is
 * the simple front door to the `door` component (components/door), which
 * does the careful work underneath: speed control from the motor's FG
 * pulses, the latch solenoid, stopping before reversing, and going back
 * when someone holds the door (motor current from the INA228):
 *
 *   closing + door held -> opens again, ends OPEN
 *   opening + door held -> closes again, ends CLOSED (latch locks)
 *
 * Angles are degrees of the motor's OUTPUT SHAFT: 0 = door closed (where the
 * shaft was at power-up), positive = opening direction.
 * Speeds are output-shaft rpm, up to MOTOR_RPM_AT_FULL_DUTY (config.h).
 *
 *   motor_set_open_angle(120);      // "open" now means 120 degrees
 *   motor_set_speed(10, 5);         // open at 10 rpm, close at 5 rpm
 *   motor_open_door();              // start opening, returns at once
 *   motor_wait_done(10000);         // ...wait until it has arrived
 *
 *   motor_rotate(45, 6);            // turn 45 degrees further open at 6 rpm
 *   motor_rotate(-45, 6);           // and back
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

/** How a door move ended (motor_wait()). */
typedef enum {
    MOTOR_DONE = 0,       /* reached open / closed                                */
    MOTOR_WENT_BACK,      /* someone held the door: back in the previous state    */
    MOTOR_FAILED,         /* blocked both ways, timed out, stopped, sensor lost... */
    MOTOR_STILL_MOVING,   /* the wait timed out                                   */
} motor_result_t;

/** Motor stopped + braked, door locked, control task started, INA228 on the
 *  shared I2C `bus`. The door must be CLOSED at power-up: that position
 *  becomes 0 degrees. */
esp_err_t motor_init(i2c_master_bus_handle_t bus);

/* ---- Settings (start-up values are in config.h) --------------------------- */

/** Shaft rotation that means "door open", in degrees. Used by the next
 *  motor_open_door(). */
void motor_set_open_angle(float degrees);

/** Shaft speed for opening and for closing, in rpm. Used by the next move. */
void motor_set_speed(float open_rpm, float close_rpm);

/* ---- Door ------------------------------------------------------------------ */
/* These return at once; the door moves in the background. Calling one again
 * while the door is already there (or on its way) changes nothing. */

void motor_open_door(void);     /* go to the open angle at the opening speed */
void motor_close_door(void);    /* go to 0 degrees at the closing speed      */
void motor_stop(void);          /* stop where it is, brake on                */

/** Wait until the last started move has ended.
 *  @return true if the door reached its target, false if it went back
 *          (door held), was blocked, stopped, or still moving after `timeout_ms`. */
bool motor_wait_done(uint32_t timeout_ms);

/** Like motor_wait_done(), but tells how the move ended. */
motor_result_t motor_wait(uint32_t timeout_ms);

/* ---- Free rotation --------------------------------------------------------- */

/**
 * Turn the shaft by `degrees` from where it is now, at `rpm`, and wait until
 * it is done. Positive = opening direction, negative = closing direction.
 * @return true if the shaft reached the target.
 */
bool motor_rotate(float degrees, float rpm);

/** Current shaft angle in degrees (0 = closed). */
float motor_get_angle(void);

/* ---- More door control (used by the micro-ROS interface) ------------------- */

/** Go TO `degrees` (0 = closed) at the open / close speed. Held on the way:
 *  goes back to where the move started. */
void motor_go_to(float degrees);

/** Close slowly until the end stop and take that as 0 degrees. */
void motor_home(void);

/** true = keep the latch retracted even with the door closed (to open the
 *  box by hand); false = automatic again. */
void motor_latch_hold_unlocked(bool hold);

/** Motor supply voltage + current from the INA228. false if it is missing. */
bool motor_read_supply(float *volts, float *amps);

/** `callback` runs when the door gives up (stuck both ways, timed out, or
 *  not closed). Keep it short; do not call motor_wait_done()/motor_rotate() in it. */
void motor_on_fault(void (*callback)(void));

/** `callback` runs when someone held the door and it went back to its
 *  previous state. `now_open`: true = it is open again (was closing),
 *  false = closed again (was opening). Same rules as motor_on_fault(). */
void motor_on_door_held(void (*callback)(bool now_open));
