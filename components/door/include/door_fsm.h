/*
 * door_fsm.h — Door motion state machine (pure logic, no ESP-IDF).
 *
 * Decides WHERE the door goes and HOW FAST, from the measured position and
 * speed. It never touches hardware, so it is unit-tested on the PC
 * (test/host/test_door_fsm.c).
 *
 * Units:
 *   position  feedback counts, 0 = closed, positive = opening direction
 *   speed     output-shaft RPM, positive = opening direction
 *   time      milliseconds (uint32_t, wrap-safe)
 *
 * Two kinds of move:
 *   OPEN / CLOSE   go to the fixed open / closed position
 *   MOVE           turn by `delta` counts from WHEREVER the door is now (or
 *                  go TO an absolute position), at a chosen speed, with its
 *                  own retry settings
 *
 * Every move has an ANCHOR: the state the door goes back to on resistance
 * (someone holding the door, a package in the way). OPEN goes back to
 * closed, CLOSE goes back to open; a MOVE goes back to where it started, or
 * to `cmd.anchor` if `cmd.has_anchor`. Resistance handling:
 *
 *   moving --resistance--> BACKING_OFF (back to the anchor, by counts)
 *            --at anchor--> RETRY_WAIT --wait--> moving again (same target)
 *   no retries left (max_retries = 0: at once): stays at the anchor in its
 *       resting state (CLOSED / OPEN / IDLE), fault OBSTRUCTED -> RETURNED
 *   resistance while backing off too:   stops where it is  -> BLOCKED
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DOOR_STATE_CLOSED = 0,
    DOOR_STATE_OPENING,      /* moving in the opening direction               */
    DOOR_STATE_OPEN,
    DOOR_STATE_CLOSING,      /* moving in the closing direction               */
    DOOR_STATE_STOPPED,      /* halted by a stop command                      */
    DOOR_STATE_BACKING_OFF,  /* resistance: returning to the move's anchor    */
    DOOR_STATE_RETRY_WAIT,   /* at the anchor, waiting before the next try    */
    DOOR_STATE_HOMING,       /* closing slowly until the end stop (sets zero) */
    DOOR_STATE_FAULT,        /* gave up; see the result                       */
    DOOR_STATE_IDLE,         /* at rest after a MOVE, not at open/closed      */
} door_state_t;

typedef enum {
    DOOR_FAULT_NONE = 0,
    DOOR_FAULT_OBSTRUCTED,   /* resistance on every try: went back to the anchor */
    DOOR_FAULT_BLOCKED,      /* resistance while backing off (stuck both ways) */
    DOOR_FAULT_TIMEOUT,      /* a move took longer than its timeout            */
    DOOR_FAULT_HOMING,       /* homing never found the end stop                */
    DOOR_FAULT_NOT_CLOSED,   /* closed all the way, but the door switch stayed open */
} door_fault_t;

/* How a move ended. BUSY while it is still running. */
typedef enum {
    DOOR_RESULT_BUSY = 0,
    DOOR_RESULT_DONE,        /* reached the target                             */
    DOOR_RESULT_RETURNED,    /* resistance every try: back at the anchor
                              * (the previous state), fault OBSTRUCTED        */
    DOOR_RESULT_BLOCKED,     /* resistance on the way back too: stopped there  */
    DOOR_RESULT_STOPPED,     /* stopped by a stop command                      */
    DOOR_RESULT_TIMEOUT,     /* took too long                                  */
    DOOR_RESULT_SENSOR_FAIL, /* the current sensor stopped answering           */
    DOOR_RESULT_NOT_CLOSED,  /* closing ended, but the door switch never closed */
} door_result_t;

typedef enum {
    DOOR_CMD_NONE = 0,
    DOOR_CMD_OPEN,
    DOOR_CMD_CLOSE,
    DOOR_CMD_STOP,
    DOOR_CMD_HOME,
    DOOR_CMD_MOVE,
} door_cmd_t;

/* A command. For OPEN/CLOSE/STOP/HOME only `type` is used. */
typedef struct {
    door_cmd_t type;
    int32_t    delta;          /* MOVE: counts to turn, + = opening direction,
                                *       or the TARGET position if `absolute`     */
    bool       absolute;       /* MOVE: `delta` is a target position (0 = closed) */
    float      rpm;            /* MOVE: constant speed (output rpm)             */
    float      return_rpm;     /* MOVE: speed back to the anchor (0 = rpm)      */
    uint8_t    max_retries;    /* MOVE: retries after resistance                */
    uint32_t   retry_wait_ms;  /* MOVE: wait at the anchor before retrying      */
    uint32_t   timeout_ms;     /* MOVE: per attempt, 0 = config move_timeout_ms */
    bool       has_anchor;     /* MOVE: on resistance go back to `anchor`...    */
    int32_t    anchor;         /*       ...instead of where the move started    */
} door_fsm_cmd_t;

typedef struct {
    int32_t  open_pos;           /* counts: fully open position (> 0)              */
    int32_t  tolerance;          /* counts: |target - pos| <= this = "arrived"     */
    int32_t  slowdown_dist;      /* counts: open/close slow down this far out      */
    float    cruise_rpm;         /* open/close: speed for most of the travel       */
    float    approach_rpm;       /* open/close: speed when arriving                */
    float    homing_rpm;
    uint32_t spinup_grace_ms;    /* no resistance check right after (re)starting   */
    uint32_t obstruction_ms;     /* speed check: too slow this long = resistance   */
    float    obstruction_ratio;  /* speed check: too slow = < ratio x commanded;
                                  * <= 0 turns the speed check off (current only)   */
    uint32_t retry_wait_ms;      /* open/close: pause before retrying              */
    uint8_t  max_retries;        /* open/close: retries before giving up           */
    uint32_t move_timeout_ms;    /* open/close: longest time for one attempt       */
} door_fsm_config_t;

typedef struct {
    door_state_t  state;
    door_fault_t  fault;
    door_result_t result;        /* result of the current / last move             */
    door_state_t  goal_state;    /* state on arrival: OPEN, CLOSED or IDLE        */
    int32_t       goal_pos;      /* counts: where the move wants to end           */
    int32_t       move_target;   /* counts: target of the current leg             */
    int32_t       move_origin;   /* counts: the move's ANCHOR (where it started)  */
    uint8_t       retries;       /* retries used so far                           */
    /* per-move settings (from the command or the config) */
    float         mv_rpm;        /* 0 = use the open/close speed profile          */
    float         mv_return_rpm; /* 0 = same as the forward speed                 */
    uint8_t       mv_retries;
    uint32_t      mv_wait_ms;
    uint32_t      mv_timeout_ms;
    /* timers */
    uint32_t      state_since_ms;
    uint32_t      move_start_ms;
    uint32_t      motion_since_ms;
    uint32_t      slow_since_ms;
    bool          slow_active;
    int8_t        last_dir;
    bool          changed;       /* internal */
    bool          finished;      /* internal */
} door_fsm_t;

typedef struct {
    float target_rpm;            /* signed output RPM; 0 = stop                    */
    bool  zero_position;         /* homing done: caller must re-zero the position */
    bool  state_changed;         /* state or fault changed during this step        */
    bool  finished;              /* a move ended this step (see f->result)         */
} door_fsm_output_t;

/** Start in `initial` (normally DOOR_STATE_CLOSED) at time `now_ms`. */
void door_fsm_init(door_fsm_t *f, door_state_t initial, uint32_t now_ms);

/**
 * Advance the state machine by one control tick.
 *
 * @param cmd        new command, or NULL
 * @param now_ms     current time
 * @param pos        position (counts, 0 = closed)
 * @param speed_rpm  measured output-shaft speed (signed)
 * @param external_obstruction  another sensor (e.g. motor current) reports
 *                   resistance right now. Ignored during the spin-up grace.
 */
door_fsm_output_t door_fsm_step(door_fsm_t *f, const door_fsm_config_t *cfg,
                                const door_fsm_cmd_t *cmd, uint32_t now_ms,
                                int32_t pos, float speed_rpm,
                                bool external_obstruction);

/** Command without parameters (OPEN, CLOSE, STOP, HOME). */
static inline door_fsm_cmd_t door_fsm_cmd(door_cmd_t type)
{
    door_fsm_cmd_t c = { .type = type };
    return c;
}

const char *door_state_name(door_state_t s);
const char *door_fault_name(door_fault_t f);
const char *door_result_name(door_result_t r);

#ifdef __cplusplus
}
#endif
