/*
 * current_guard.h — Detects resistance on the door from the motor current
 * (pure logic, no ESP-IDF; unit-tested in test/host/test_current_guard.c).
 *
 * A DC motor draws more current the harder it has to push, so a hand, a
 * package edge or a jammed hinge shows up as a current rise. The guard
 * combines two of the methods from the project notes:
 *
 *   1. Baseline profiling: it learns the normal running current separately
 *      for opening and closing (gravity/springs make them differ). The
 *      baseline follows slow changes (temperature, wear).
 *   2. Absolute threshold: above `max_a` is always resistance, even if the
 *      door started against an obstacle and never ran freely.
 *
 * Resistance = |current| above max(baseline + margin_a, baseline * ratio)
 * (or above max_a) continuously for `confirm_ms`. The first `settle_ms` after
 * the motor starts or reverses are ignored (inrush current).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float    margin_a;        /* rise above baseline that counts as resistance   */
    float    ratio;           /* ... or this multiple of the baseline            */
    float    max_a;           /* always resistance above this                    */
    uint32_t settle_ms;       /* ignore inrush after (re)starting                */
    uint32_t confirm_ms;      /* must stay high this long (filters spikes)       */
    uint32_t baseline_tau_ms; /* how slowly the baseline follows the current     */
} current_guard_config_t;

typedef struct {
    float    baseline[2];     /* [0] = closing, [1] = opening                    */
    bool     learned[2];
    int8_t   dir;             /* direction seen on the last update               */
    uint32_t moving_since_ms;
    uint32_t last_ms;
    bool     over;
    uint32_t over_since_ms;
    float    threshold;       /* last threshold used (for logging)               */
} current_guard_t;

void current_guard_init(current_guard_t *g);

/**
 * Feed one current sample.
 *
 * @param motor_dir  direction the motor is driven right now (-1, 0, +1)
 * @param current_a  measured current (sign ignored)
 * @return true while resistance is detected
 */
bool current_guard_update(current_guard_t *g, const current_guard_config_t *c,
                          uint32_t now_ms, int8_t motor_dir, float current_a);

/** Learned running current for a direction (+1 opening, -1 closing), or 0. */
float current_guard_baseline(const current_guard_t *g, int8_t dir);

#ifdef __cplusplus
}
#endif
