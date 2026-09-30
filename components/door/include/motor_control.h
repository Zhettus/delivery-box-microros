/*
 * motor_control.h — direction interlock, pulse position tracker, speed controller
 * (pure logic, no ESP-IDF; unit-tested in test/host/test_door_fsm.c).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* Direction interlock                                                      */
/*                                                                          */
/* Before switching direction the motor must be stopped: the DRI0042 needs  */
/* >= 100 ms in STOP, a BLDC motor's built-in driver must see the rotor at  */
/* standstill. Whatever direction is requested, the returned direction      */
/* never jumps straight from +1 to -1 (or back), and never (re)starts       */
/* before the brake time has elapsed AND the motor reports standstill.      */
/* ------------------------------------------------------------------------ */

typedef struct {
    int8_t   dir;            /* direction currently driven: -1, 0, +1 */
    int64_t  stopped_at_us;  /* when the driver last entered STOP     */
    uint32_t brake_us;       /* minimum STOP time before a new start  */
} motor_interlock_t;

void motor_interlock_init(motor_interlock_t *m, uint32_t brake_ms);

/**
 * Returns the direction that may be driven right now (-1, 0 or +1).
 * @param standstill  the motor is not turning (speed ~ 0). Pass true if the
 *                    feedback can't tell (time-only interlock).
 */
int8_t motor_interlock_step(motor_interlock_t *m, int8_t wanted_dir, int64_t now_us,
                            bool standstill);

/* ------------------------------------------------------------------------ */
/* Position from a direction-less pulse output (BLDC "FG" wire)             */
/*                                                                          */
/* FG gives pulses but no direction. The direction comes from how the motor */
/* is driven; pulses that arrive while it coasts after a stop keep counting */
/* in the LAST driven direction (the interlock forbids reversing before     */
/* standstill, so that is always correct).                                  */
/* ------------------------------------------------------------------------ */

typedef struct {
    int32_t last_raw;      /* raw pulse count at the previous update   */
    int8_t  last_dir;      /* last non-zero driven direction           */
    int32_t position;      /* signed position, in pulses               */
} pulse_tracker_t;

void    pulse_tracker_init(pulse_tracker_t *t, int32_t raw_now);
int32_t pulse_tracker_update(pulse_tracker_t *t, int32_t raw_now, int8_t driven_dir);

/* ------------------------------------------------------------------------ */
/* Speed controller: feed-forward + PI on motor RPM                          */
/*                                                                          */
/* Output is a signed duty in [-1, 1] that always has the sign of the       */
/* target, so the controller never asks the driver to reverse on its own.   */
/* ------------------------------------------------------------------------ */

typedef struct {
    float kp;              /* duty per RPM of error                    */
    float ki;              /* duty per (RPM * s) of error              */
    float integral_limit;  /* anti-windup clamp, in duty (0..1)        */
    float ff_per_rpm;      /* feed-forward: 1 / motor RPM at 100% duty */
    float min_duty;        /* smallest duty that actually turns the motor */
} speed_ctrl_config_t;

typedef struct {
    float integral;
} speed_ctrl_t;

void  speed_ctrl_reset(speed_ctrl_t *s);
float speed_ctrl_step(speed_ctrl_t *s, const speed_ctrl_config_t *c,
                      float target_rpm, float measured_rpm, float dt_s);

#ifdef __cplusplus
}
#endif
