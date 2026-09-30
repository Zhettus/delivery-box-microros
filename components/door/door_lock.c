/*
 * door_lock.c — Latch solenoid sequencing. See door_lock.h.
 */
#include "door_lock.h"

static void enter(door_lock_t *l, door_lock_state_t s, uint32_t now_ms,
                  door_lock_output_t *o)
{
    if (l->state != s) {
        o->changed = true;
    }
    l->state    = s;
    l->since_ms = now_ms;
}

void door_lock_init(door_lock_t *l, bool locked, uint32_t now_ms)
{
    *l = (door_lock_t){
        .state       = locked ? DOOR_LOCK_LOCKED : DOOR_LOCK_UNLOCKED,
        .since_ms    = now_ms,
        .on_since_ms = now_ms,
        .on          = !locked,
    };
}

door_lock_output_t door_lock_step(door_lock_t *l, const door_lock_config_t *c,
                                  uint32_t now_ms, bool wants_motion,
                                  bool at_closed_rest)
{
    door_lock_output_t o = { 0 };
    uint32_t in_state = now_ms - l->since_ms;

    switch (l->state) {
    case DOOR_LOCK_LOCKED:
        if (wants_motion) {
            enter(l, DOOR_LOCK_UNLOCKING, now_ms, &o);   /* retract first */
        }
        break;

    case DOOR_LOCK_UNLOCKING:
        if (in_state >= c->unlock_ms) {
            enter(l, DOOR_LOCK_UNLOCKED, now_ms, &o);
        }
        break;

    case DOOR_LOCK_UNLOCKED:
        if (at_closed_rest && !wants_motion) {
            enter(l, DOOR_LOCK_LOCKING, now_ms, &o);     /* let the door seat */
        }
        break;

    case DOOR_LOCK_LOCKING:
        if (wants_motion || !at_closed_rest) {
            enter(l, DOOR_LOCK_UNLOCKED, now_ms, &o);    /* moving again: stay open */
        } else if (in_state >= c->settle_ms) {
            enter(l, DOOR_LOCK_LOCKED, now_ms, &o);      /* extend the latch */
        }
        break;
    }

    bool on = (l->state != DOOR_LOCK_LOCKED);
    if (on && !l->on) {
        l->on_since_ms = now_ms;                          /* pull-in starts now */
    }
    l->on = on;

    if (on) {
        bool pulling_in = (uint32_t)(now_ms - l->on_since_ms) < c->pull_in_ms;
        float hold = c->hold_duty <= 0.0f ? 1.0f : (c->hold_duty > 1.0f ? 1.0f : c->hold_duty);
        o.solenoid = pulling_in ? 1.0f : hold;
    }
    o.motion_allowed = (l->state == DOOR_LOCK_UNLOCKED);
    return o;
}

const char *door_lock_state_name(door_lock_state_t s)
{
    switch (s) {
    case DOOR_LOCK_LOCKED:    return "locked";
    case DOOR_LOCK_UNLOCKING: return "unlocking";
    case DOOR_LOCK_UNLOCKED:  return "unlocked";
    case DOOR_LOCK_LOCKING:   return "locking";
    default:                  return "?";
    }
}
