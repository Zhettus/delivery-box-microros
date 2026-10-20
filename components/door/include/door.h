/*
 * door.h — Delivery-box door: gear motor + position feedback + safe motion.
 *
 * Two motor types are supported (door_config_t.motor_type):
 *
 *   DOOR_MOTOR_BLDC     brushless motor WITH ITS OWN BUILT-IN DRIVER
 *                       (red/black = power, straight from the supply).
 *                       The ESP32 drives its control wires:
 *                         PWM   speed (duty = speed)
 *                         DIR   rotation direction (CW / CCW)
 *                         BRAKE brake (optional)
 *                       and counts its FG (speed pulse) output for position.
 *
 *   DOOR_MOTOR_DRI0042  brushed DC motor + DFRobot DRI0042 H-bridge
 *                       (IN1 / IN2 / PWM) + 2-channel Hall encoder.
 *
 * A 50 Hz control task (highest application priority) runs:
 *
 *   feedback --> position/speed --> door_fsm (where to go, how fast)
 *            --> motor_interlock (stop + standstill before any reversal)
 *            --> speed controller (feed-forward + PI) --> motor
 *
 * Optional latch solenoid (door_config_t.latch): every move away from the
 * closed position first retracts the latch (motor waits unlock_ms), the
 * latch stays retracted while the door moves or is open, and it extends
 * again settle_ms after the door is back at closed and stopped:
 *
 *   LOCKED -> UNLOCKING -> OPENING -> OPEN (hold_open_ms) -> CLOSING
 *          -> LOCKING -> LOCKED
 *
 * All calls are non-blocking and safe from any task.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

#include "door_fsm.h"
#include "door_lock.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DOOR_MOTOR_BLDC = 0,     /* brushless motor with built-in driver (PWM/DIR/BRAKE/FG) */
    DOOR_MOTOR_DRI0042,      /* brushed motor + DRI0042 H-bridge + quadrature encoder    */
} door_motor_type_t;

/* Brushless motor with built-in driver. */
typedef struct {
    gpio_num_t pwm_gpio;         /* speed input                                   */
    gpio_num_t dir_gpio;         /* direction input (CW/CCW)                      */
    gpio_num_t brake_gpio;       /* brake input, or GPIO_NUM_NC if there is none   */
    gpio_num_t fg_gpio;          /* FG speed-pulse output of the motor            */
    uint8_t    dir_level_open;   /* DIR level (0/1) that turns the door OPEN      */
    uint8_t    brake_level_on;   /* BRAKE level (0/1) that brakes                 */
    bool       pwm_inverted;     /* true: motor runs faster with LESS high time   */
    bool       dir_open_drain;   /* true: DIR "high" = let go of the wire, for a
                                    motor that pulls DIR up to 5 V itself        */
} door_bldc_config_t;

/* Brushed motor + DFRobot DRI0042 + 2-channel Hall encoder. */
typedef struct {
    gpio_num_t in1_gpio;
    gpio_num_t in2_gpio;
    gpio_num_t pwm_gpio;
    gpio_num_t hall_a_gpio;
    gpio_num_t hall_b_gpio;
    uint32_t   min_pulse_ns;     /* DRI0042 minimum valid pulse (200 ns)          */
    bool       invert_motor;     /* door opens the "wrong" way? set true          */
    bool       invert_encoder;   /* counts go down when opening? set true         */
} door_dri0042_config_t;

/* Resistance detection from the motor current. 0 switches a limit off. */
typedef struct {
    float    limit_a;          /* resistance above this current, always             */
    float    rise_a;           /* ...or this much above the learned running current */
    uint32_t confirm_ms;       /* the current must stay high this long              */
    uint32_t ignore_start_ms;  /* ignore the start-up spike of every move            */
} door_current_limits_t;

/* Door latch solenoid through a MOSFET module (e.g. XY-MOS). */
typedef struct {
    gpio_num_t         gpio;          /* GPIO_NUM_NC = no latch fitted            */
    ledc_timer_t       timer;         /* used for the reduced hold power          */
    ledc_channel_t     channel;
    uint32_t           pwm_freq_hz;
    door_lock_config_t timing;        /* unlock / settle / pull-in / hold power   */
} door_latch_config_t;

/* Reads the motor current in amperes; returns false if the read failed. */
typedef bool (*door_current_reader_t)(float *amps, void *ctx);

typedef struct {
    door_motor_type_t     motor_type;
    door_bldc_config_t    bldc;      /* used when motor_type == DOOR_MOTOR_BLDC    */
    door_dri0042_config_t dri0042;   /* used when motor_type == DOOR_MOTOR_DRI0042 */
    ledc_timer_t   pwm_timer;
    ledc_channel_t pwm_channel;
    uint32_t       pwm_freq_hz;

    /* ---- Feedback + speed control, all at the OUTPUT shaft ------------- */
    float    counts_per_output_rev;      /* BLDC: FG pulses per output rev;
                                            DRI0042: 4 x encoder PPR x gear ratio */
    float    output_rpm_at_full_duty;    /* feed-forward                          */
    float    min_duty;                   /* 0..1, least duty that turns the motor */
    float    kp, ki, integral_limit;     /* speed PI, duty per output rpm         */
    uint32_t stop_before_reverse_ms;     /* minimum stop time before reversing    */
    uint32_t speed_window_ms;            /* speed = counts over this window       */
    uint32_t control_period_ms;

    /* ---- Door motion (output shaft) ------------------------------------ */
    float    open_angle_deg;             /* closed -> open travel                */
    float    tolerance_deg;              /* "arrived" window                     */
    float    slowdown_deg;               /* start slowing this far from target   */
    float    cruise_rpm;
    float    approach_rpm;
    float    homing_rpm;
    uint32_t spinup_grace_ms;
    uint32_t obstruction_ms;
    float    obstruction_ratio;          /* <= 0: no speed-based detection      */
    uint32_t retry_wait_ms;
    uint8_t  max_retries;
    uint32_t move_timeout_ms;
    bool     home_at_boot;               /* false: assume the door is closed at power-up */
    uint32_t hold_open_ms;               /* after reaching a not-closed position,
                                            close again after this (0 = never)   */
    float    auto_close_rpm;             /* speed of that automatic close        */
    /* ---- Door-closed switch (optional) ------------------------------------
     * With a switch, closing (a move TO 0) runs until the switch closes:
     *   switch closes -> latch_lock_delay_ms later the latch solenoid
     *   switches OFF (locks) -> the motor keeps closing until at least seat_ms
     *   after the switch AND stop_after_latch_ms after the solenoid went off,
     *   then stops.
     *   Once the latch is locked and the shaft has stopped moving, that spot
     *   becomes the new 0 (nothing adds up over cycles).
     * close_overtravel_deg is how far past 0 it may go looking for the switch;
     * if the switch never closes the move ends with DOOR_RESULT_NOT_CLOSED and
     * the latch stays retracted. The latch only ever locks with the switch
     * closed. GPIO_NUM_NC = no switch: closing simply goes to 0.            */
    gpio_num_t closed_switch_gpio;
    uint8_t    closed_switch_level;      /* input level when the door is closed  */
    float      close_overtravel_deg;
    uint32_t   seat_ms;
    uint32_t   latch_lock_delay_ms;
    uint32_t   stop_after_latch_ms;

    /* ---- Nudge before opening (optional) ----------------------------------
     * An open command while the door is closed first turns the shaft
     * open_nudge_deg the CLOSING way, for at most open_nudge_ms, while the
     * solenoid retracts the latch (the push takes the load off the bolt).
     * Then the door opens. 0 = no nudge.                                    */
    float      open_nudge_deg;
    uint32_t   open_nudge_ms;

    /* ---- Latch solenoid (optional) --------------------------------------- */
    door_latch_config_t latch;

    /* ---- Motor current (optional) ---------------------------------------- */
    door_current_reader_t read_current;  /* NULL = no current-based detection   */
    void                 *read_current_ctx;
    door_current_limits_t current;       /* default limits (open/close, defaults) */
    uint32_t              sense_period_ms; /* current sampling period (e.g. 5)    */
} door_config_t;

/*
 * One door move: turn by `degrees` from WHEREVER the door is now.
 * On resistance (motor current rise, or the door too slow) the door stops
 * and goes back to the move's ANCHOR (counted in feedback pulses, not
 * seconds): the start position, or `anchor_deg` if `anchor_set`. Then it
 * waits `retry_wait_ms` and tries again, up to `max_retries` times; with no
 * retries left it stays at the anchor, in that position's resting state
 * (fault OBSTRUCTED, result RETURNED).
 *
 * Going back to closed (anchor 0) with a door switch ends like a normal
 * close: it seats at the switch and the latch locks.
 */
typedef struct {
    float                 degrees;       /* + = opening direction, - = closing;
                                            the TARGET angle if `absolute`       */
    bool                  absolute;      /* true: go TO `degrees` (0 = closed)   */
    float                 rpm;           /* output-shaft speed                    */
    float                 return_rpm;    /* speed back to the anchor (0 = rpm)    */
    door_current_limits_t current;       /* resistance limits for THIS move       */
    bool                  anchor_set;    /* true: on resistance go back to...     */
    float                 anchor_deg;    /* ...this angle (e.g. 0 = closed)       */
    uint8_t               max_retries;   /* 0 = go back once and stay there       */
    uint32_t              retry_wait_ms; /* wait at the anchor before retrying    */
    uint32_t              timeout_ms;    /* per attempt; 0 = automatic            */
} door_move_t;

typedef struct {
    door_state_t state;
    door_fault_t fault;
    uint8_t      retries;
    float        position_deg;           /* 0 = closed                           */
    float        speed_rpm;              /* output shaft, + = opening            */
    float        duty;                   /* last duty sent to the driver (0..1)  */
    int8_t       direction;              /* -1, 0, +1 actually driven            */
    door_result_t result;                /* how the last move ended (BUSY = running) */
    float        current_a;              /* latest motor current                 */
    float        normal_a;               /* learned running current (this direction) */
    float        peak_a;                 /* highest current in the current move  */
    float        limit_now_a;            /* resistance threshold right now       */
    door_lock_state_t lock;              /* latch: locked / unlocking / ...      */
    float        solenoid;               /* latch solenoid power 0..1 (0 = locked) */
    uint32_t     auto_close_in_ms;       /* 0 = no automatic close pending       */
    bool         closed_switch;          /* door-closed switch reads "closed"    */
} door_status_t;

/** Called from the door task whenever the state or fault changes. Keep it short.
 *  (Latch changes are logged, not reported here: see door_status_t.lock.) */
typedef void (*door_event_cb_t)(door_state_t state, door_fault_t fault, void *ctx);

/** Initialise GPIOs (motor stopped), encoder and the control task. */
esp_err_t door_init(const door_config_t *cfg);

void door_open(void);
void door_close(void);
void door_stop(void);
void door_home(void);

/**
 * Another sensor (e.g. the INA228 motor current) sees resistance right now.
 * Call it repeatedly while the condition lasts; the next control tick treats
 * it like any obstruction: go back to the move's anchor (see door_move_t).
 * Ignored while the door is not moving and during the spin-up grace period.
 */
void door_report_obstruction(void);

/**
 * Keep the latch retracted even while the door is closed (true), e.g. to
 * open the box by hand or to test the solenoid. false = automatic again.
 */
void door_latch_hold_unlocked(bool hold);

/* ---- Moves by angle ------------------------------------------------------- */
/* The blocking functions wait until the move ends and return its result.
 * Do NOT call them from the door event callback. */

/**
 * Start current-based resistance detection (e.g. with the INA228) after
 * door_init. `reader` is called every sense_period_ms from a door task.
 */
esp_err_t door_attach_current_sensor(door_current_reader_t reader, void *ctx);

/** A door_move_t filled with the config's default limits and retry settings. */
door_move_t door_move_defaults(float degrees, float rpm);

/** Open by `degrees` at `rpm`; resistance above `current_limit_a` -> go back. */
door_result_t door_open_by(float degrees, float rpm, float current_limit_a);

/** Close by `degrees` at `rpm`; resistance above `current_limit_a` -> go back. */
door_result_t door_close_by(float degrees, float rpm, float current_limit_a);

/** Full control: start `move` and wait for its result. */
door_result_t door_rotate(const door_move_t *move);

/** Non-blocking: start `move` and return at once (see door_wait). */
esp_err_t door_rotate_start(const door_move_t *move);

/**
 * Go TO an absolute angle (0 = closed, APP_DOOR_OPEN_DEG = open). Safe to
 * repeat: asking again for the target it is already heading to changes
 * nothing, and at the target it returns DONE at once.
 */
door_result_t door_move_to(float target_deg, float rpm, float current_limit_a);

/** Non-blocking door_move_to (for robot states, callbacks, ...). */
esp_err_t door_move_to_start(float target_deg, float rpm, float current_limit_a);

/** Wait for the last started move/open/close to end. BUSY on timeout. */
door_result_t door_wait(uint32_t timeout_ms);

door_status_t door_get_status(void);
void          door_set_event_callback(door_event_cb_t cb, void *ctx);

#ifdef __cplusplus
}
#endif
