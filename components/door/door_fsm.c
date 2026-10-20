/*
 * door_fsm.c — Door motion state machine. See door_fsm.h.
 */
#include <math.h>

#include <stddef.h>

#include "door_fsm.h"

static int32_t iabs32(int32_t v) { return v < 0 ? -v : v; }

static bool is_moving(door_state_t s)
{
    return s == DOOR_STATE_OPENING || s == DOOR_STATE_CLOSING ||
           s == DOOR_STATE_BACKING_OFF || s == DOOR_STATE_RETRY_WAIT ||
           s == DOOR_STATE_HOMING;
}

static void enter(door_fsm_t *f, door_state_t s, uint32_t now_ms)
{
    if (f->state != s) {
        f->changed = true;
    }
    f->state          = s;
    f->state_since_ms = now_ms;
}

static void finish(door_fsm_t *f, door_result_t r)
{
    f->result   = r;
    f->finished = true;
}

static void fail(door_fsm_t *f, door_fault_t why, uint32_t now_ms)
{
    f->fault   = why;
    f->changed = true;
    enter(f, DOOR_STATE_FAULT, now_ms);
    switch (why) {
    case DOOR_FAULT_OBSTRUCTED: finish(f, DOOR_RESULT_RETURNED); break;
    case DOOR_FAULT_BLOCKED:    finish(f, DOOR_RESULT_BLOCKED);  break;
    default:                    finish(f, DOOR_RESULT_TIMEOUT);  break;
    }
}

/* Resistance on every try and back at the anchor: the door stays there, in
 * the resting state of that position (the state it had before the move). */
static void settle_at_anchor(door_fsm_t *f, const door_fsm_config_t *c, uint32_t now_ms)
{
    door_state_t rest = DOOR_STATE_IDLE;
    if (iabs32(f->move_origin) <= c->tolerance) {
        rest = DOOR_STATE_CLOSED;
    } else if (iabs32(f->move_origin - c->open_pos) <= c->tolerance) {
        rest = DOOR_STATE_OPEN;
    }
    f->fault   = DOOR_FAULT_OBSTRUCTED;
    f->changed = true;
    enter(f, rest, now_ms);
    finish(f, DOOR_RESULT_RETURNED);
}

/* Direction state for travelling from `from` to `target`. */
static door_state_t leg_state(int32_t target, int32_t from)
{
    return target >= from ? DOOR_STATE_OPENING : DOOR_STATE_CLOSING;
}

/* Start one leg of travel. Resets the timeout, grace period and stall timer. */
static void begin_leg(door_fsm_t *f, door_state_t moving_state,
                      int32_t target, int32_t origin, uint32_t now_ms)
{
    f->move_target     = target;
    f->move_origin     = origin;
    f->move_start_ms   = now_ms;
    f->motion_since_ms = now_ms;
    f->slow_active     = false;
    f->last_dir        = 0;
    enter(f, moving_state, now_ms);
}

/* A new move: clears the fault and the retry counter, records the anchor. */
static void start_move(door_fsm_t *f, door_state_t goal_state, int32_t goal_pos,
                       int32_t anchor, uint32_t now_ms, int32_t pos)
{
    if (f->fault != DOOR_FAULT_NONE) {
        f->changed = true;
    }
    f->goal_state = goal_state;
    f->goal_pos   = goal_pos;
    f->retries    = 0;
    f->fault      = DOOR_FAULT_NONE;
    f->result     = DOOR_RESULT_BUSY;
    begin_leg(f, leg_state(goal_pos, pos), goal_pos, anchor, now_ms);
}

/* Open/close speed profile: cruise far away, ramp down to approach speed. */
static float profile_rpm(const door_fsm_config_t *c, int32_t remaining_abs)
{
    if (c->slowdown_dist <= 0 || remaining_abs >= c->slowdown_dist) {
        return c->cruise_rpm;
    }
    float x = (float)remaining_abs / (float)c->slowdown_dist;
    return c->approach_rpm + (c->cruise_rpm - c->approach_rpm) * x;
}

/*
 * Resistance detector. After a spin-up grace period (restarted on every
 * direction change), resistance is either reported by another sensor
 * (`external`, e.g. motor current), or the door moves slower than
 * `obstruction_ratio` x commanded speed for `obstruction_ms`.
 */
static bool obstructed(door_fsm_t *f, const door_fsm_config_t *c,
                       uint32_t now_ms, float cmd_rpm, float speed_rpm,
                       bool external)
{
    if (cmd_rpm == 0.0f) {
        f->slow_active = false;
        return false;
    }

    int8_t dir = (cmd_rpm > 0.0f) ? 1 : -1;
    if (dir != f->last_dir) {
        f->last_dir        = dir;
        f->motion_since_ms = now_ms;
        f->slow_active     = false;
    }
    if ((uint32_t)(now_ms - f->motion_since_ms) < c->spinup_grace_ms) {
        return false;
    }
    if (external) {
        return true;
    }
    if (c->obstruction_ratio <= 0.0f) {
        return false;                    /* speed check switched off */
    }

    bool too_slow = (speed_rpm * (float)dir) < c->obstruction_ratio * fabsf(cmd_rpm);
    if (!too_slow) {
        f->slow_active = false;
        return false;
    }
    if (!f->slow_active) {
        f->slow_active   = true;
        f->slow_since_ms = now_ms;
    }
    return (uint32_t)(now_ms - f->slow_since_ms) >= c->obstruction_ms;
}

void door_fsm_init(door_fsm_t *f, door_state_t initial, uint32_t now_ms)
{
    *f = (door_fsm_t){
        .state          = initial,
        .fault          = DOOR_FAULT_NONE,
        .result         = DOOR_RESULT_DONE,
        .goal_state     = (initial == DOOR_STATE_OPEN) ? DOOR_STATE_OPEN : DOOR_STATE_CLOSED,
        .state_since_ms = now_ms,
    };
}

static void handle_command(door_fsm_t *f, const door_fsm_config_t *c,
                           const door_fsm_cmd_t *cmd, uint32_t now_ms, int32_t pos)
{
    if (cmd == NULL) {
        return;
    }
    switch (cmd->type) {
    case DOOR_CMD_OPEN:
    case DOOR_CMD_CLOSE: {
        door_state_t goal     = (cmd->type == DOOR_CMD_OPEN) ? DOOR_STATE_OPEN
                                                             : DOOR_STATE_CLOSED;
        int32_t      goal_pos = (cmd->type == DOOR_CMD_OPEN) ? c->open_pos : 0;
        bool already_there = (f->state == goal);
        bool already_going = is_moving(f->state) && f->state != DOOR_STATE_HOMING &&
                             f->goal_pos == goal_pos && f->mv_rpm == 0.0f;
        if (already_there || already_going) {
            break;
        }
        /* Anchor = the other end: resistance while opening goes back to
         * closed, while closing back to open (also when the move started
         * half way, e.g. a close that interrupted an open). */
        int32_t anchor = (cmd->type == DOOR_CMD_OPEN) ? 0 : c->open_pos;
        f->mv_rpm        = 0.0f;               /* use the speed profile */
        f->mv_return_rpm = 0.0f;
        f->mv_retries    = c->max_retries;
        f->mv_wait_ms    = c->retry_wait_ms;
        f->mv_timeout_ms = c->move_timeout_ms;
        start_move(f, goal, goal_pos, anchor, now_ms, pos);
        break;
    }

    case DOOR_CMD_MOVE: {
        f->mv_rpm        = fabsf(cmd->rpm);
        f->mv_return_rpm = fabsf(cmd->return_rpm);
        f->mv_retries    = cmd->max_retries;
        f->mv_wait_ms    = cmd->retry_wait_ms;
        f->mv_timeout_ms = cmd->timeout_ms ? cmd->timeout_ms : c->move_timeout_ms;
        int32_t target = cmd->absolute ? cmd->delta        /* go TO a position     */
                                       : pos + cmd->delta; /* turn from HERE       */
        door_state_t goal = (target == 0) ? DOOR_STATE_CLOSED
                          : (target == c->open_pos) ? DOOR_STATE_OPEN : DOOR_STATE_IDLE;

        /* Absolute target we are already heading for: keep going (retries,
         * anchor and timers stay), so repeating a command is harmless. */
        if (cmd->absolute && is_moving(f->state) && f->state != DOOR_STATE_HOMING &&
            f->goal_pos == target) {
            break;
        }
        /* Already there (or nothing to do): done at once. */
        if (f->mv_rpm <= 0.0f || iabs32(target - pos) <= c->tolerance) {
            if (!is_moving(f->state) || f->goal_pos != target) {
                enter(f, goal, now_ms);        /* also stops a move elsewhere */
            }
            f->fault = DOOR_FAULT_NONE;
            finish(f, DOOR_RESULT_DONE);
            break;
        }
        start_move(f, goal, target, cmd->has_anchor ? cmd->anchor : pos,  /* default: here */
                   now_ms, pos);
        break;
    }

    case DOOR_CMD_STOP:
        if (is_moving(f->state)) {
            enter(f, DOOR_STATE_STOPPED, now_ms);
            finish(f, DOOR_RESULT_STOPPED);
        }
        break;

    case DOOR_CMD_HOME:
        f->goal_state    = DOOR_STATE_CLOSED;
        f->goal_pos      = 0;
        f->retries       = 0;
        f->fault         = DOOR_FAULT_NONE;
        f->result        = DOOR_RESULT_BUSY;
        f->mv_timeout_ms = c->move_timeout_ms;
        begin_leg(f, DOOR_STATE_HOMING, 0, pos, now_ms);
        f->changed = true;
        break;

    case DOOR_CMD_NONE:
    default:
        break;
    }
}

door_fsm_output_t door_fsm_step(door_fsm_t *f, const door_fsm_config_t *c,
                                const door_fsm_cmd_t *cmd, uint32_t now_ms,
                                int32_t pos, float speed_rpm,
                                bool external_obstruction)
{
    door_fsm_output_t out = { 0 };
    f->changed  = false;
    f->finished = false;

    handle_command(f, c, cmd, now_ms, pos);

    switch (f->state) {

    case DOOR_STATE_OPENING:
    case DOOR_STATE_CLOSING:
    case DOOR_STATE_BACKING_OFF: {
        int32_t remaining = f->move_target - pos;

        if (iabs32(remaining) <= c->tolerance) {
            if (f->state == DOOR_STATE_BACKING_OFF) {
                /* Back at the anchor: wait and retry, or stay here. */
                if (f->retries >= f->mv_retries) {
                    settle_at_anchor(f, c, now_ms);
                } else {
                    enter(f, DOOR_STATE_RETRY_WAIT, now_ms);
                }
            } else {
                f->retries = 0;
                enter(f, f->goal_state, now_ms);
                finish(f, DOOR_RESULT_DONE);
            }
            break;
        }

        if ((uint32_t)(now_ms - f->move_start_ms) > f->mv_timeout_ms) {
            fail(f, DOOR_FAULT_TIMEOUT, now_ms);
            break;
        }

        float rpm;
        if (f->state == DOOR_STATE_BACKING_OFF && f->mv_return_rpm > 0.0f) {
            rpm = f->mv_return_rpm;
        } else if (f->mv_rpm > 0.0f) {
            rpm = f->mv_rpm;                            /* MOVE: constant speed */
        } else {
            rpm = profile_rpm(c, iabs32(remaining));    /* OPEN/CLOSE profile   */
        }
        if (remaining < 0) {
            rpm = -rpm;
        }

        if (obstructed(f, c, now_ms, rpm, speed_rpm, external_obstruction)) {
            if (f->state == DOOR_STATE_BACKING_OFF) {
                fail(f, DOOR_FAULT_BLOCKED, now_ms);    /* stuck both ways: stop */
            } else {
                /* Go back to the anchor, counted in feedback counts. */
                begin_leg(f, DOOR_STATE_BACKING_OFF,
                          f->move_origin, f->move_origin, now_ms);
            }
            break;   /* stop for this tick; the interlock brakes before reversing */
        }

        out.target_rpm = rpm;
        break;
    }

    case DOOR_STATE_RETRY_WAIT:
        if ((uint32_t)(now_ms - f->state_since_ms) >= f->mv_wait_ms) {
            f->retries++;
            begin_leg(f, leg_state(f->goal_pos, pos), f->goal_pos,
                      f->move_origin, now_ms);
        }
        break;

    case DOOR_STATE_HOMING: {
        if ((uint32_t)(now_ms - f->move_start_ms) > f->mv_timeout_ms) {
            fail(f, DOOR_FAULT_HOMING, now_ms);
            break;
        }
        float rpm = -c->homing_rpm;
        if (obstructed(f, c, now_ms, rpm, speed_rpm, external_obstruction)) {
            out.zero_position = true;               /* the closed end stop = zero */
            enter(f, DOOR_STATE_CLOSED, now_ms);
            finish(f, DOOR_RESULT_DONE);
            break;
        }
        out.target_rpm = rpm;
        break;
    }

    case DOOR_STATE_CLOSED:
    case DOOR_STATE_OPEN:
    case DOOR_STATE_IDLE:
    case DOOR_STATE_STOPPED:
    case DOOR_STATE_FAULT:
    default:
        break;   /* motor stopped */
    }

    out.state_changed = f->changed;
    out.finished      = f->finished;
    return out;
}

const char *door_state_name(door_state_t s)
{
    switch (s) {
    case DOOR_STATE_CLOSED:      return "closed";
    case DOOR_STATE_OPENING:     return "opening";
    case DOOR_STATE_OPEN:        return "open";
    case DOOR_STATE_CLOSING:     return "closing";
    case DOOR_STATE_STOPPED:     return "stopped";
    case DOOR_STATE_BACKING_OFF: return "backing_off";
    case DOOR_STATE_RETRY_WAIT:  return "retry_wait";
    case DOOR_STATE_HOMING:      return "homing";
    case DOOR_STATE_FAULT:       return "fault";
    case DOOR_STATE_IDLE:        return "idle";
    default:                     return "?";
    }
}

const char *door_fault_name(door_fault_t f)
{
    switch (f) {
    case DOOR_FAULT_NONE:       return "none";
    case DOOR_FAULT_OBSTRUCTED: return "obstructed";
    case DOOR_FAULT_BLOCKED:    return "blocked";
    case DOOR_FAULT_TIMEOUT:    return "timeout";
    case DOOR_FAULT_HOMING:     return "homing_failed";
    case DOOR_FAULT_NOT_CLOSED: return "not_closed";
    default:                    return "?";
    }
}

const char *door_result_name(door_result_t r)
{
    switch (r) {
    case DOOR_RESULT_BUSY:        return "busy";
    case DOOR_RESULT_DONE:        return "done";
    case DOOR_RESULT_RETURNED:    return "returned";
    case DOOR_RESULT_BLOCKED:     return "blocked";
    case DOOR_RESULT_STOPPED:     return "stopped";
    case DOOR_RESULT_TIMEOUT:     return "timeout";
    case DOOR_RESULT_SENSOR_FAIL: return "sensor_fail";
    case DOOR_RESULT_NOT_CLOSED:  return "not_closed";
    default:                      return "?";
    }
}
