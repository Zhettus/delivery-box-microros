/*
 * current_guard.c — Resistance detection from motor current. See current_guard.h.
 */
#include <math.h>

#include "current_guard.h"

static int idx(int8_t dir)
{
    return dir > 0 ? 1 : 0;
}

void current_guard_init(current_guard_t *g)
{
    *g = (current_guard_t){ 0 };
}

float current_guard_baseline(const current_guard_t *g, int8_t dir)
{
    int i = idx(dir);
    return g->learned[i] ? g->baseline[i] : 0.0f;
}

bool current_guard_update(current_guard_t *g, const current_guard_config_t *c,
                          uint32_t now_ms, int8_t motor_dir, float current_a)
{
    float i_abs = fabsf(current_a);
    uint32_t dt = now_ms - g->last_ms;
    g->last_ms = now_ms;

    /* Motor stopped: nothing to guard. */
    if (motor_dir == 0) {
        g->dir  = 0;
        g->over = false;
        return false;
    }

    /* Started or reversed: restart the inrush window. */
    if (motor_dir != g->dir) {
        g->dir             = motor_dir;
        g->moving_since_ms = now_ms;
        g->over            = false;
        return false;
    }
    if ((uint32_t)(now_ms - g->moving_since_ms) < c->settle_ms) {
        return false;
    }

    int    k        = idx(motor_dir);
    float  base     = g->baseline[k];
    bool   learned  = g->learned[k];

    /* First free-running sample in this direction: take it as the baseline,
     * unless it is already above the absolute limit (started against a block). */
    if (!learned && i_abs < c->max_a) {
        g->baseline[k] = base = i_abs;
        g->learned[k]  = learned = true;
    }

    float threshold = c->max_a;
    if (learned) {
        float relative = fmaxf(base + c->margin_a, base * c->ratio);
        threshold = fminf(relative, c->max_a);
    }
    g->threshold = threshold;

    bool high = i_abs > threshold;

    /* Follow slow changes only while running normally. */
    if (!high && learned && c->baseline_tau_ms > 0) {
        float a = (float)dt / (float)c->baseline_tau_ms;
        if (a > 1.0f) {
            a = 1.0f;
        }
        g->baseline[k] += a * (i_abs - g->baseline[k]);
    }

    if (!high) {
        g->over = false;
        return false;
    }
    if (!g->over) {
        g->over          = true;
        g->over_since_ms = now_ms;
    }
    return (uint32_t)(now_ms - g->over_since_ms) >= c->confirm_ms;
}
