/*
 * motor_control.c — DRI0042 direction interlock + speed controller.
 */
#include <stdbool.h>

#include "motor_control.h"

void motor_interlock_init(motor_interlock_t *m, uint32_t brake_ms)
{
    m->dir           = 0;
    m->brake_us      = brake_ms * 1000U;
    m->stopped_at_us = INT64_MIN / 2;   /* allow an immediate first start */
}

int8_t motor_interlock_step(motor_interlock_t *m, int8_t wanted_dir, int64_t now_us,
                            bool standstill)
{
    if (wanted_dir > 0) {
        wanted_dir = 1;
    } else if (wanted_dir < 0) {
        wanted_dir = -1;
    }

    if (wanted_dir == m->dir) {
        return m->dir;
    }

    if (m->dir != 0) {
        /* Running and asked to stop or reverse: STOP first, start the clock. */
        m->dir           = 0;
        m->stopped_at_us = now_us;
        return 0;
    }

    /* Stopped and asked to start: only after the brake time AND at standstill. */
    if (now_us - m->stopped_at_us >= (int64_t)m->brake_us && standstill) {
        m->dir = wanted_dir;
    }
    return m->dir;
}

void pulse_tracker_init(pulse_tracker_t *t, int32_t raw_now)
{
    t->last_raw = raw_now;
    t->last_dir = 0;
    t->position = 0;
}

int32_t pulse_tracker_update(pulse_tracker_t *t, int32_t raw_now, int8_t driven_dir)
{
    if (driven_dir != 0) {
        t->last_dir = driven_dir > 0 ? 1 : -1;
    }
    int32_t delta = raw_now - t->last_raw;
    t->last_raw   = raw_now;
    t->position  += delta * t->last_dir;   /* before any drive: last_dir 0, ignored */
    return t->position;
}

void speed_ctrl_reset(speed_ctrl_t *s)
{
    s->integral = 0.0f;
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

float speed_ctrl_step(speed_ctrl_t *s, const speed_ctrl_config_t *c,
                      float target_rpm, float measured_rpm, float dt_s)
{
    if (target_rpm == 0.0f) {
        speed_ctrl_reset(s);
        return 0.0f;
    }

    float err = target_rpm - measured_rpm;

    s->integral += c->ki * err * dt_s;
    s->integral  = clampf(s->integral, -c->integral_limit, c->integral_limit);

    float u = target_rpm * c->ff_per_rpm + c->kp * err + s->integral;

    /* Only drive in the target's direction; "negative" effort means coast. */
    float dir = (target_rpm > 0.0f) ? 1.0f : -1.0f;
    float mag = clampf(u * dir, 0.0f, 1.0f);
    if (mag > 0.0f && mag < c->min_duty) {
        mag = c->min_duty;
    }
    return dir * mag;
}
