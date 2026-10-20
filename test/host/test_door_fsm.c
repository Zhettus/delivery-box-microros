/*
 * test_door_fsm.c — door state machine + motor interlock, closed loop with a
 * simple simulated door (first-order motor, optional obstacle).
 */
#include "check.h"
#include "current_guard.h"
#include "door_lock.h"
#include "door_fsm.h"
#include "motor_control.h"

#define TICK_MS        20
#define OPEN_POS       2096     /* 90 deg with 64 CPR x 131:1 */

static const door_fsm_config_t CFG = {
    .open_pos          = OPEN_POS,
    .tolerance         = 46,
    .slowdown_dist     = 466,
    .cruise_rpm        = 8.0f,
    .approach_rpm      = 3.0f,
    .homing_rpm        = 3.0f,
    .spinup_grace_ms   = 500,
    .obstruction_ms    = 300,
    .obstruction_ratio = 0.35f,
    .retry_wait_ms     = 3000,
    .max_retries       = 2,
    .move_timeout_ms   = 10000,
};

/* counts per output revolution */
#define CPR_OUT  (64.0f * 131.0f)

typedef struct {
    door_fsm_t        fsm;
    motor_interlock_t lock;
    uint32_t          now_ms;
    float             pos;          /* counts */
    float             speed_rpm;    /* output RPM */
    int32_t           zero;         /* encoder offset */
    /* obstacle: blocks motion beyond `block_pos` in the blocked direction */
    int               block_dir;    /* +1 blocks opening, -1 blocks closing, 0 none */
    float             block_pos;
    /* trace */
    int               reversals_without_brake;
    int8_t            last_dir;
    uint32_t          last_dir_change_ms;
    int               saw_state[16];
    /* current-sensor-only mode */
    bool              current_mode;
    door_fsm_config_t cfg;
    current_guard_t   guard;
    int               guard_hits;
} sim_t;

static const current_guard_config_t GUARD = {
    .margin_a = 0.25f, .ratio = 1.8f, .max_a = 2.5f,
    .settle_ms = 400, .confirm_ms = 120, .baseline_tau_ms = 3000,
};

static void sim_init(sim_t *s)
{
    *s = (sim_t){ 0 };
    door_fsm_init(&s->fsm, DOOR_STATE_CLOSED, 0);
    motor_interlock_init(&s->lock, 100);
    s->cfg = CFG;
    current_guard_init(&s->guard);
}

/* Door that only knows about resistance through the motor current. */
static void sim_init_current_mode(sim_t *s)
{
    sim_init(s);
    s->current_mode = true;
    s->cfg.obstruction_ratio = 0.0f;        /* speed check off */
    s->cfg.spinup_grace_ms   = 400;
    s->cfg.cruise_rpm = s->cfg.approach_rpm = 2.0f;
    s->cfg.slowdown_dist = 0;
}

static void sim_tick_cmd(sim_t *s, const door_fsm_cmd_t *cmd)
{
    int32_t pos = (int32_t)s->pos - s->zero;
    /* Simulated motor current: 0.3 A running, 1.5 A when pushing an obstacle. */
    bool ext = false;
    if (s->current_mode) {
        bool pushing = (s->block_dir > 0 && s->last_dir > 0 && s->pos >= s->block_pos - 1) ||
                       (s->block_dir < 0 && s->last_dir < 0 && s->pos <= s->block_pos + 1);
        float amps = s->last_dir == 0 ? 0.02f : (pushing ? 1.5f : 0.30f);
        ext = current_guard_update(&s->guard, &GUARD, s->now_ms, s->last_dir, amps);
        s->guard_hits += ext;
    }
    door_fsm_output_t o = door_fsm_step(&s->fsm, &s->cfg, cmd, s->now_ms, pos,
                                        s->speed_rpm, ext);
    if (o.zero_position) {
        s->zero = (int32_t)s->pos;
    }
    s->saw_state[s->fsm.state] = 1;

    int8_t want = (o.target_rpm > 0) - (o.target_rpm < 0);
    int8_t dir  = motor_interlock_step(&s->lock, want, (int64_t)s->now_ms * 1000,
                                       fabsf(s->speed_rpm) < 0.2f);

    /* Safety property: never jump straight from +1 to -1 or back. */
    if (dir != 0 && s->last_dir != 0 && dir != s->last_dir) {
        s->reversals_without_brake++;
    }
    if (dir != s->last_dir) {
        s->last_dir = dir;
        s->last_dir_change_ms = s->now_ms;
    }

    /* Plant: speed follows the command with a ~60 ms time constant. */
    float cmd_rpm = dir ? o.target_rpm : 0.0f;
    s->speed_rpm += (cmd_rpm - s->speed_rpm) * 0.33f;

    float next = s->pos + s->speed_rpm / 60.0f * CPR_OUT * (TICK_MS / 1000.0f);
    if (s->block_dir > 0 && next > s->block_pos && s->speed_rpm > 0) {
        next = s->block_pos; s->speed_rpm = 0;
    }
    if (s->block_dir < 0 && next < s->block_pos && s->speed_rpm < 0) {
        next = s->block_pos; s->speed_rpm = 0;
    }
    s->pos = next;
    s->now_ms += TICK_MS;
}

static void sim_tick(sim_t *s, door_cmd_t type)
{
    door_fsm_cmd_t c = door_fsm_cmd(type);
    sim_tick_cmd(s, type == DOOR_CMD_NONE ? NULL : &c);
}

static void sim_run(sim_t *s, uint32_t ms)
{
    for (uint32_t t = 0; t < ms; t += TICK_MS) {
        sim_tick(s, DOOR_CMD_NONE);
    }
}

static void test_open_then_close(void)
{
    sim_t s; sim_init(&s);
    sim_tick(&s, DOOR_CMD_OPEN);
    CHECK(s.fsm.state == DOOR_STATE_OPENING);
    sim_run(&s, 5000);
    CHECK(s.fsm.state == DOOR_STATE_OPEN);
    CHECK_NEAR(s.pos, OPEN_POS, CFG.tolerance + 5);

    sim_tick(&s, DOOR_CMD_CLOSE);
    CHECK(s.fsm.state == DOOR_STATE_CLOSING);
    sim_run(&s, 5000);
    CHECK(s.fsm.state == DOOR_STATE_CLOSED);
    CHECK_NEAR(s.pos, 0, CFG.tolerance + 5);
    CHECK(s.reversals_without_brake == 0);
}

static void test_repeated_command_is_ignored(void)
{
    sim_t s; sim_init(&s);
    sim_tick(&s, DOOR_CMD_OPEN);
    sim_run(&s, 400);
    uint32_t start = s.fsm.move_start_ms;
    sim_tick(&s, DOOR_CMD_OPEN);            /* same goal: must not restart */
    CHECK(s.fsm.move_start_ms == start);
    sim_tick(&s, DOOR_CMD_CLOSE);           /* already closed? no: we are opening */
    CHECK(s.fsm.state == DOOR_STATE_CLOSING);
}

static void test_interference_then_success(void)
{
    /* Obstacle half way while opening, removed during the 3 s wait. */
    sim_t s; sim_init(&s);
    s.block_dir = +1; s.block_pos = OPEN_POS / 2;
    sim_tick(&s, DOOR_CMD_OPEN);

    for (int i = 0; i < 400 && s.fsm.state != DOOR_STATE_RETRY_WAIT; i++) {
        sim_tick(&s, DOOR_CMD_NONE);
    }
    CHECK(s.saw_state[DOOR_STATE_BACKING_OFF]);
    CHECK(s.fsm.state == DOOR_STATE_RETRY_WAIT);
    CHECK_NEAR(s.pos, 0, CFG.tolerance + 5);          /* went back to closed */

    uint32_t wait_start = s.now_ms;
    s.block_dir = 0;                                   /* obstacle removed */
    while (s.fsm.state == DOOR_STATE_RETRY_WAIT) {
        sim_tick(&s, DOOR_CMD_NONE);
    }
    CHECK(s.now_ms - wait_start >= CFG.retry_wait_ms);
    CHECK(s.fsm.state == DOOR_STATE_OPENING);
    CHECK(s.fsm.retries == 1);

    sim_run(&s, 5000);
    CHECK(s.fsm.state == DOOR_STATE_OPEN);
    CHECK(s.fsm.retries == 0);
    CHECK(s.fsm.fault == DOOR_FAULT_NONE);
    CHECK(s.reversals_without_brake == 0);
}

static void test_persistent_interference_goes_back_closed(void)
{
    sim_t s; sim_init(&s);
    s.block_dir = +1; s.block_pos = OPEN_POS / 2;
    sim_tick(&s, DOOR_CMD_OPEN);
    sim_run(&s, 40000);
    CHECK(s.fsm.state == DOOR_STATE_CLOSED);          /* the previous state, no FAULT */
    CHECK(s.fsm.fault == DOOR_FAULT_OBSTRUCTED);
    CHECK(s.fsm.result == DOOR_RESULT_RETURNED);
    CHECK(s.fsm.retries == CFG.max_retries);
    CHECK_NEAR(s.pos, 0, CFG.tolerance + 5);          /* ends back at closed */
    CHECK(s.reversals_without_brake == 0);

    /* A new command clears the fault. */
    s.block_dir = 0;
    sim_tick(&s, DOOR_CMD_OPEN);
    CHECK(s.fsm.fault == DOOR_FAULT_NONE);
    sim_run(&s, 5000);
    CHECK(s.fsm.state == DOOR_STATE_OPEN);
}

static void test_closing_on_obstacle_reopens(void)
{
    sim_t s; sim_init(&s);
    sim_tick(&s, DOOR_CMD_OPEN);
    sim_run(&s, 5000);
    CHECK(s.fsm.state == DOOR_STATE_OPEN);

    s.block_dir = -1; s.block_pos = OPEN_POS / 2;     /* e.g. a hand */
    sim_tick(&s, DOOR_CMD_CLOSE);
    for (int i = 0; i < 400 && s.fsm.state != DOOR_STATE_RETRY_WAIT; i++) {
        sim_tick(&s, DOOR_CMD_NONE);
    }
    CHECK(s.fsm.state == DOOR_STATE_RETRY_WAIT);
    CHECK_NEAR(s.pos, OPEN_POS, CFG.tolerance + 5);   /* went back to open */
}

static void test_blocked_both_ways(void)
{
    sim_t s; sim_init(&s);
    s.block_dir = +1; s.block_pos = OPEN_POS / 2;
    sim_tick(&s, DOOR_CMD_OPEN);
    while (s.fsm.state != DOOR_STATE_BACKING_OFF) {
        sim_tick(&s, DOOR_CMD_NONE);
    }
    s.block_dir = -1; s.block_pos = s.pos;             /* now stuck closing too */
    sim_run(&s, 3000);
    CHECK(s.fsm.state == DOOR_STATE_FAULT);
    CHECK(s.fsm.fault == DOOR_FAULT_BLOCKED);
    CHECK_NEAR(s.speed_rpm, 0, 0.5);
}

static void test_stop_command(void)
{
    sim_t s; sim_init(&s);
    sim_tick(&s, DOOR_CMD_OPEN);
    sim_run(&s, 600);
    sim_tick(&s, DOOR_CMD_STOP);
    CHECK(s.fsm.state == DOOR_STATE_STOPPED);
    sim_run(&s, 500);
    CHECK_NEAR(s.speed_rpm, 0, 0.1);
    float p = s.pos;
    sim_run(&s, 500);
    CHECK_NEAR(s.pos, p, 1);
}

static void test_reverse_mid_travel_brakes_first(void)
{
    sim_t s; sim_init(&s);
    sim_tick(&s, DOOR_CMD_OPEN);
    sim_run(&s, 800);
    CHECK(s.last_dir == 1);
    sim_tick(&s, DOOR_CMD_CLOSE);
    uint32_t t_cmd = s.now_ms;
    while (s.last_dir != -1) {
        sim_tick(&s, DOOR_CMD_NONE);
    }
    CHECK(s.now_ms - t_cmd >= 100);                    /* >= 100 ms stopped */
    sim_run(&s, 5000);
    CHECK(s.fsm.state == DOOR_STATE_CLOSED);
    CHECK(s.reversals_without_brake == 0);
}

static void test_timeout(void)
{
    door_fsm_config_t slow = CFG;
    slow.move_timeout_ms = 300;
    sim_t s; sim_init(&s);
    door_fsm_cmd_t open = door_fsm_cmd(DOOR_CMD_OPEN);
    door_fsm_step(&s.fsm, &slow, &open, 0, 0, 0, false);
    door_fsm_step(&s.fsm, &slow, NULL, 400, 100, 8.0f, false);
    CHECK(s.fsm.state == DOOR_STATE_FAULT);
    CHECK(s.fsm.fault == DOOR_FAULT_TIMEOUT);
}

static void test_homing_sets_zero(void)
{
    sim_t s; sim_init(&s);
    s.pos = 900;                                       /* door left half open */
    s.block_dir = -1; s.block_pos = -150;              /* true end stop */
    sim_tick(&s, DOOR_CMD_HOME);
    CHECK(s.fsm.state == DOOR_STATE_HOMING);
    sim_run(&s, 8000);
    CHECK(s.fsm.state == DOOR_STATE_CLOSED);
    CHECK(s.zero == -150);

    s.block_dir = 0;
    sim_tick(&s, DOOR_CMD_OPEN);
    sim_run(&s, 5000);
    CHECK(s.fsm.state == DOOR_STATE_OPEN);
    CHECK_NEAR(s.pos - s.zero, OPEN_POS, CFG.tolerance + 5);
}

static void test_interlock_rules(void)
{
    motor_interlock_t m;
    motor_interlock_init(&m, 100);
    CHECK(motor_interlock_step(&m, +1, 0, true) == +1);          /* first start: at once */
    CHECK(motor_interlock_step(&m, -1, 10000, false) == 0);      /* reverse: stop first  */
    CHECK(motor_interlock_step(&m, -1, 60000, true) == 0);       /* 50 ms: still braking */
    CHECK(motor_interlock_step(&m, -1, 110000, false) == 0);     /* time ok, still coasting */
    CHECK(motor_interlock_step(&m, -1, 130000, true) == -1);     /* stopped: go          */
    CHECK(motor_interlock_step(&m, 0, 140000, true) == 0);
    CHECK(motor_interlock_step(&m, +1, 170000, true) == 0);      /* restart waits too    */
    CHECK(motor_interlock_step(&m, +1, 240000, true) == +1);
}

static void test_pulse_tracker(void)
{
    pulse_tracker_t t;
    pulse_tracker_init(&t, 1000);
    CHECK(pulse_tracker_update(&t, 1005, 0) == 0);               /* noise before any drive */
    CHECK(pulse_tracker_update(&t, 1105, +1) == 100);            /* opening */
    CHECK(pulse_tracker_update(&t, 1115, 0) == 110);             /* coasting: still + */
    CHECK(pulse_tracker_update(&t, 1165, -1) == 60);             /* closing */
    CHECK(pulse_tracker_update(&t, 1170, 0) == 55);              /* coasting: still - */
}

static void test_speed_ctrl(void)
{
    speed_ctrl_config_t c = { .kp = 1e-4f, .ki = 5e-4f, .integral_limit = 0.4f,
                              .ff_per_rpm = 1.0f / 4500.0f, .min_duty = 0.15f };
    speed_ctrl_t s = { 0 };
    CHECK(speed_ctrl_step(&s, &c, 0, 100, 0.02f) == 0.0f);
    float u = speed_ctrl_step(&s, &c, 1000, 0, 0.02f);
    CHECK(u > 0.15f && u <= 1.0f);
    u = speed_ctrl_step(&s, &c, -1000, 0, 0.02f);
    CHECK(u < 0.0f);
    speed_ctrl_reset(&s);
    u = speed_ctrl_step(&s, &c, 100, 5000, 0.02f);         /* way too fast: coast */
    CHECK(u == 0.0f);
    u = speed_ctrl_step(&s, &c, 10, 10, 0.02f);            /* tiny: at least min duty */
    CHECK(u == 0.0f || u >= 0.15f);
}

/* ---- current sensor only ------------------------------------------------ */

static void test_current_mode_free_cycle(void)
{
    sim_t s; sim_init_current_mode(&s);
    for (int cycle = 0; cycle < 3; cycle++) {
        sim_tick(&s, DOOR_CMD_OPEN);
        sim_run(&s, 12000);
        CHECK(s.fsm.state == DOOR_STATE_OPEN);
        sim_tick(&s, DOOR_CMD_CLOSE);
        sim_run(&s, 12000);
        CHECK(s.fsm.state == DOOR_STATE_CLOSED);
    }
    CHECK(s.guard_hits == 0);                         /* no false alarms */
    CHECK(s.reversals_without_brake == 0);
}

static void test_current_mode_hand_while_opening(void)
{
    sim_t s; sim_init_current_mode(&s);
    s.block_dir = +1; s.block_pos = OPEN_POS / 2;
    sim_tick(&s, DOOR_CMD_OPEN);
    for (int i = 0; i < 1000 && s.fsm.state != DOOR_STATE_RETRY_WAIT; i++) {
        sim_tick(&s, DOOR_CMD_NONE);
    }
    CHECK(s.guard_hits > 0);
    CHECK(s.saw_state[DOOR_STATE_BACKING_OFF]);
    CHECK(s.fsm.state == DOOR_STATE_RETRY_WAIT);
    CHECK_NEAR(s.pos, 0, CFG.tolerance + 5);          /* back to closed, slowly */
    s.block_dir = 0;
    sim_run(&s, 20000);
    CHECK(s.fsm.state == DOOR_STATE_OPEN);            /* retried and made it */
    CHECK(s.reversals_without_brake == 0);
}

static void test_current_mode_hand_while_closing(void)
{
    sim_t s; sim_init_current_mode(&s);
    sim_tick(&s, DOOR_CMD_OPEN);
    sim_run(&s, 12000);
    CHECK(s.fsm.state == DOOR_STATE_OPEN);
    s.block_dir = -1; s.block_pos = OPEN_POS / 3;
    sim_tick(&s, DOOR_CMD_CLOSE);
    for (int i = 0; i < 1500 && s.fsm.state != DOOR_STATE_RETRY_WAIT; i++) {
        sim_tick(&s, DOOR_CMD_NONE);
    }
    CHECK(s.fsm.state == DOOR_STATE_RETRY_WAIT);
    CHECK_NEAR(s.pos, OPEN_POS, CFG.tolerance + 5);   /* back to open */
}

static void test_current_mode_ignored_when_idle(void)
{
    sim_t s; sim_init_current_mode(&s);
    door_fsm_output_t o = door_fsm_step(&s.fsm, &s.cfg, NULL, 0, 0, 0, true);
    CHECK(o.target_rpm == 0.0f && s.fsm.state == DOOR_STATE_CLOSED);
    /* During the spin-up grace an external report is ignored (inrush). */
    door_fsm_cmd_t open = door_fsm_cmd(DOOR_CMD_OPEN);
    door_fsm_step(&s.fsm, &s.cfg, &open, 0, 0, 0, false);
    door_fsm_step(&s.fsm, &s.cfg, NULL, 20, 0, 0, true);
    CHECK(s.fsm.state == DOOR_STATE_OPENING);
    door_fsm_step(&s.fsm, &s.cfg, NULL, 500, 50, 2.0f, true);
    CHECK(s.fsm.state == DOOR_STATE_BACKING_OFF);
}

static void test_guard_unit(void)
{
    current_guard_t g; current_guard_init(&g);
    uint32_t t = 0;
    /* stopped: never alarms */
    CHECK(!current_guard_update(&g, &GUARD, t, 0, 5.0f));
    /* start: inrush spike ignored during settle */
    CHECK(!current_guard_update(&g, &GUARD, t += 5, +1, 3.0f));
    for (; t < 400; t += 5) CHECK(!current_guard_update(&g, &GUARD, t, +1, 3.0f));
    /* free running learns ~0.3 A */
    for (int i = 0; i < 200; i++) CHECK(!current_guard_update(&g, &GUARD, t += 5, +1, 0.30f));
    CHECK_NEAR(current_guard_baseline(&g, +1), 0.30, 0.02);
    CHECK(current_guard_baseline(&g, -1) == 0.0f);
    /* short spike (< confirm) is filtered */
    CHECK(!current_guard_update(&g, &GUARD, t += 5, +1, 1.0f));
    CHECK(!current_guard_update(&g, &GUARD, t += 50, +1, 1.0f));
    CHECK(!current_guard_update(&g, &GUARD, t += 5, +1, 0.30f));
    /* sustained rise -> resistance after confirm_ms, baseline not dragged up */
    bool hit = false;
    for (int i = 0; i < 40; i++) hit |= current_guard_update(&g, &GUARD, t += 5, +1, 0.80f);
    CHECK(hit);
    CHECK_NEAR(current_guard_baseline(&g, +1), 0.30, 0.03);
    /* negative current readings count by magnitude */
    hit = false;
    for (int i = 0; i < 40; i++) hit |= current_guard_update(&g, &GUARD, t += 5, +1, -0.80f);
    CHECK(hit);
    /* starting against a block (never free): absolute limit still catches it */
    current_guard_t g2; current_guard_init(&g2);
    uint32_t t2 = 0;
    current_guard_update(&g2, &GUARD, t2, -1, 3.0f);
    hit = false;
    for (int i = 0; i < 200; i++) hit |= current_guard_update(&g2, &GUARD, t2 += 5, -1, 3.0f);
    CHECK(hit);
}

/* ---- relative moves (door_rotate) ------------------------------------------ */

static door_fsm_cmd_t move_cmd(int32_t delta, float rpm, uint8_t retries)
{
    door_fsm_cmd_t c = { .type = DOOR_CMD_MOVE, .delta = delta, .rpm = rpm,
                         .max_retries = retries, .retry_wait_ms = 1000 };
    return c;
}

static void run_until_finished(sim_t *s, uint32_t max_ms)
{
    for (uint32_t t = 0; t < max_ms; t += TICK_MS) {
        sim_tick(s, DOOR_CMD_NONE);
        if (s->fsm.result != DOOR_RESULT_BUSY) {
            return;
        }
    }
}

static void test_move_relative_from_anywhere(void)
{
    sim_t s; sim_init_current_mode(&s);
    s.pos = 700;                                   /* somewhere in the middle */
    door_fsm_cmd_t c = move_cmd(+1048, 5.0f, 2);   /* +180 deg worth of counts */
    sim_tick_cmd(&s, &c);
    CHECK(s.fsm.result == DOOR_RESULT_BUSY);
    CHECK(s.fsm.move_origin == 700);               /* anchor = where it started */
    run_until_finished(&s, 30000);
    CHECK(s.fsm.result == DOOR_RESULT_DONE);
    CHECK(s.fsm.state == DOOR_STATE_IDLE);         /* not at open/closed */
    CHECK_NEAR(s.pos, 700 + 1048, CFG.tolerance + 5);

    c = move_cmd(-1048, 5.0f, 2);                  /* and back the same amount */
    sim_tick_cmd(&s, &c);
    run_until_finished(&s, 30000);
    CHECK(s.fsm.result == DOOR_RESULT_DONE);
    CHECK_NEAR(s.pos, 700, CFG.tolerance + 5);
    CHECK(s.reversals_without_brake == 0);
}

static void test_move_speed_is_used(void)
{
    sim_t s; sim_init_current_mode(&s);
    door_fsm_cmd_t c = move_cmd(+2000, 12.0f, 0);
    sim_tick_cmd(&s, &c);
    sim_run(&s, 1000);
    CHECK_NEAR(s.speed_rpm, 12.0, 0.5);            /* constant requested speed */
}

static void test_move_resistance_returns_to_anchor(void)
{
    sim_t s; sim_init_current_mode(&s);
    s.pos = 500;
    s.block_dir = +1; s.block_pos = 900;           /* hand in the way */
    door_fsm_cmd_t c = move_cmd(+1000, 5.0f, 1);
    sim_tick_cmd(&s, &c);
    for (int i = 0; i < 2000 && s.fsm.state != DOOR_STATE_RETRY_WAIT; i++) {
        sim_tick(&s, DOOR_CMD_NONE);
    }
    CHECK(s.fsm.state == DOOR_STATE_RETRY_WAIT);
    CHECK_NEAR(s.pos, 500, CFG.tolerance + 5);     /* back at ITS anchor, not 0 */
    run_until_finished(&s, 60000);                 /* still blocked: gives up */
    CHECK(s.fsm.result == DOOR_RESULT_RETURNED);
    CHECK(s.fsm.retries == 1);
    CHECK_NEAR(s.pos, 500, CFG.tolerance + 5);
    CHECK(s.reversals_without_brake == 0);
}

static void test_move_retry_succeeds(void)
{
    sim_t s; sim_init_current_mode(&s);
    s.block_dir = +1; s.block_pos = 300;
    door_fsm_cmd_t c = move_cmd(+800, 5.0f, 2);
    sim_tick_cmd(&s, &c);
    for (int i = 0; i < 2000 && s.fsm.state != DOOR_STATE_RETRY_WAIT; i++) {
        sim_tick(&s, DOOR_CMD_NONE);
    }
    s.block_dir = 0;                               /* hand removed during the wait */
    run_until_finished(&s, 60000);
    CHECK(s.fsm.result == DOOR_RESULT_DONE);
    CHECK_NEAR(s.pos, 800, CFG.tolerance + 5);
}

static void test_move_return_speed(void)
{
    sim_t s; sim_init_current_mode(&s);
    s.block_dir = +1; s.block_pos = 1500;
    door_fsm_cmd_t c = move_cmd(+2000, 10.0f, 0);
    c.return_rpm = 3.0f;                           /* go back slowly */
    sim_tick_cmd(&s, &c);
    for (int i = 0; i < 3000 && s.fsm.state != DOOR_STATE_BACKING_OFF; i++) {
        sim_tick(&s, DOOR_CMD_NONE);
    }
    sim_run(&s, 1500);
    CHECK(s.fsm.state == DOOR_STATE_BACKING_OFF);
    CHECK_NEAR(s.speed_rpm, -3.0, 0.5);
}

static void test_move_stop_and_zero(void)
{
    sim_t s; sim_init_current_mode(&s);
    door_fsm_cmd_t c = move_cmd(+4000, 5.0f, 0);   /* ~5.7 s long */
    sim_tick_cmd(&s, &c);
    sim_run(&s, 1500);
    sim_tick(&s, DOOR_CMD_STOP);
    CHECK(s.fsm.result == DOOR_RESULT_STOPPED);
    CHECK(s.fsm.state == DOOR_STATE_STOPPED);

    door_fsm_cmd_t z = move_cmd(0, 5.0f, 0);       /* zero degrees: done at once */
    door_fsm_step(&s.fsm, &s.cfg, &z, s.now_ms, (int32_t)s.pos, 0, false);
    CHECK(s.fsm.result == DOOR_RESULT_DONE);
}

static void test_move_to_absolute(void)
{
    sim_t s; sim_init_current_mode(&s);
    door_fsm_cmd_t c = { .type = DOOR_CMD_MOVE, .absolute = true, .delta = OPEN_POS,
                         .rpm = 8.0f, .max_retries = 2, .retry_wait_ms = 1000 };
    sim_tick_cmd(&s, &c);
    sim_run(&s, 500);
    uint32_t started = s.fsm.move_start_ms;
    sim_tick_cmd(&s, &c);                          /* repeated: must not restart */
    CHECK(s.fsm.move_start_ms == started);
    run_until_finished(&s, 30000);
    CHECK(s.fsm.result == DOOR_RESULT_DONE);
    CHECK(s.fsm.state == DOOR_STATE_OPEN);         /* target = open_pos -> OPEN */
    CHECK_NEAR(s.pos, OPEN_POS, CFG.tolerance + 5);

    sim_tick_cmd(&s, &c);                          /* already there: done at once */
    CHECK(s.fsm.result == DOOR_RESULT_DONE && s.fsm.state == DOOR_STATE_OPEN);

    door_fsm_cmd_t z = c; z.delta = 0;             /* go to closed */
    sim_tick_cmd(&s, &z);
    run_until_finished(&s, 30000);
    CHECK(s.fsm.state == DOOR_STATE_CLOSED);
    CHECK_NEAR(s.pos, 0, CFG.tolerance + 5);
}

static void test_move_to_reverses_mid_way(void)
{
    sim_t s; sim_init_current_mode(&s);
    door_fsm_cmd_t o = { .type = DOOR_CMD_MOVE, .absolute = true, .delta = OPEN_POS,
                         .rpm = 8.0f, .max_retries = 1, .retry_wait_ms = 500 };
    sim_tick_cmd(&s, &o);
    sim_run(&s, 1000);                             /* half way open */
    door_fsm_cmd_t z = o; z.delta = 0;             /* robot starts moving: close! */
    sim_tick_cmd(&s, &z);
    CHECK(s.fsm.move_origin == (int32_t)s.pos || fabsf(s.fsm.move_origin - s.pos) < 60);
    run_until_finished(&s, 30000);
    CHECK(s.fsm.state == DOOR_STATE_CLOSED);
    CHECK(s.reversals_without_brake == 0);
}

/* ---- latch solenoid ------------------------------------------------------------ */

static const door_lock_config_t LOCK = { .unlock_ms = 200, .settle_ms = 100,
                                         .pull_in_ms = 300, .hold_duty = 0.5f };

static void test_lock_sequence(void)
{
    door_lock_t l;
    door_lock_init(&l, true, 0);
    door_lock_output_t o = door_lock_step(&l, &LOCK, 0, false, true);
    CHECK(l.state == DOOR_LOCK_LOCKED && o.solenoid == 0.0f && !o.motion_allowed);

    /* open requested: solenoid on, motor held for unlock_ms */
    o = door_lock_step(&l, &LOCK, 10, true, true);
    CHECK(l.state == DOOR_LOCK_UNLOCKING && o.solenoid == 1.0f && !o.motion_allowed);
    o = door_lock_step(&l, &LOCK, 150, true, true);
    CHECK(!o.motion_allowed);
    o = door_lock_step(&l, &LOCK, 210, true, true);
    CHECK(l.state == DOOR_LOCK_UNLOCKED && o.motion_allowed);

    /* moving + open: stays on; pull-in full power, then hold power */
    o = door_lock_step(&l, &LOCK, 250, true, false);
    CHECK(o.solenoid == 1.0f);
    o = door_lock_step(&l, &LOCK, 400, false, false);          /* holding open */
    CHECK(o.solenoid == 0.5f && l.state == DOOR_LOCK_UNLOCKED);

    /* back at closed and stopped: settle, then lock */
    o = door_lock_step(&l, &LOCK, 5000, false, true);
    CHECK(l.state == DOOR_LOCK_LOCKING && o.solenoid > 0.0f && !o.motion_allowed);
    o = door_lock_step(&l, &LOCK, 5050, false, true);
    CHECK(l.state == DOOR_LOCK_LOCKING);
    o = door_lock_step(&l, &LOCK, 5110, false, true);
    CHECK(l.state == DOOR_LOCK_LOCKED && o.solenoid == 0.0f);
}

static void test_lock_new_move_while_locking(void)
{
    door_lock_t l;
    door_lock_init(&l, false, 0);                              /* start unlocked */
    door_lock_step(&l, &LOCK, 0, false, true);                 /* -> locking */
    CHECK(l.state == DOOR_LOCK_LOCKING);
    door_lock_output_t o = door_lock_step(&l, &LOCK, 50, true, true);  /* open again */
    CHECK(l.state == DOOR_LOCK_UNLOCKED && o.solenoid > 0.0f);
    o = door_lock_step(&l, &LOCK, 60, true, true);
    CHECK(o.motion_allowed);                                   /* no extra unlock wait */
}

static void test_door_with_lock(void)
{
    /* Full loop: the door never turns while the latch is still extended. */
    sim_t s; sim_init_current_mode(&s);
    door_lock_t l; door_lock_init(&l, true, 0);
    door_fsm_cmd_t c = { .type = DOOR_CMD_MOVE, .absolute = true, .delta = OPEN_POS,
                         .rpm = 8.0f, .max_retries = 1, .retry_wait_ms = 500 };
    bool moved_while_locked = false;
    for (int i = 0; i < 1500; i++) {
        door_fsm_output_t o = door_fsm_step(&s.fsm, &s.cfg, i == 0 ? &c : NULL,
                                            s.now_ms, (int32_t)s.pos, s.speed_rpm, false);
        bool closed_rest = o.target_rpm == 0.0f && fabsf(s.pos) <= CFG.tolerance &&
                           fabsf(s.speed_rpm) < 0.2f;
        door_lock_output_t lo = door_lock_step(&l, &LOCK, s.now_ms,
                                               o.target_rpm != 0.0f, closed_rest);
        float rpm = lo.motion_allowed ? o.target_rpm : 0.0f;
        if (lo.solenoid == 0.0f && rpm != 0.0f) moved_while_locked = true;
        s.speed_rpm += (rpm - s.speed_rpm) * 0.33f;
        s.pos += s.speed_rpm / 60.0f * CPR_OUT * (TICK_MS / 1000.0f);
        s.now_ms += TICK_MS;
        if (i == 800) {                                        /* then close again */
            c.delta = 0;
            door_fsm_step(&s.fsm, &s.cfg, &c, s.now_ms, (int32_t)s.pos, s.speed_rpm, false);
        }
    }
    CHECK(!moved_while_locked);
    CHECK(s.fsm.state == DOOR_STATE_CLOSED);
    CHECK(l.state == DOOR_LOCK_LOCKED);                        /* locked again at the end */
}

/* ---- door held (final firmware: no retries, back to the previous state) ----- */

/* Current-sensor door with max_retries = 0, like config.h. */
static void sim_init_held(sim_t *s)
{
    sim_init_current_mode(s);
    s->cfg.max_retries = 0;
}

static void test_held_while_closing_goes_back_open(void)
{
    sim_t s; sim_init_held(&s);
    sim_tick(&s, DOOR_CMD_OPEN);
    run_until_finished(&s, 15000);
    CHECK(s.fsm.state == DOOR_STATE_OPEN);

    s.block_dir = -1; s.block_pos = OPEN_POS / 2;     /* a hand holds the door */
    sim_tick(&s, DOOR_CMD_CLOSE);
    run_until_finished(&s, 30000);
    CHECK(s.guard_hits > 0);                          /* seen by the motor current */
    CHECK(s.saw_state[DOOR_STATE_BACKING_OFF]);
    CHECK(!s.saw_state[DOOR_STATE_RETRY_WAIT]);       /* no retry */
    CHECK(s.fsm.state == DOOR_STATE_OPEN);
    CHECK(s.fsm.fault == DOOR_FAULT_OBSTRUCTED);
    CHECK(s.fsm.result == DOOR_RESULT_RETURNED);
    CHECK_NEAR(s.pos, OPEN_POS, CFG.tolerance + 5);
    CHECK(s.reversals_without_brake == 0);

    /* It stays open... */
    sim_run(&s, 5000);
    CHECK(s.fsm.state == DOOR_STATE_OPEN);
    CHECK_NEAR(s.speed_rpm, 0, 0.2);
    /* ...until the next close; hand gone, it closes and the fault clears. */
    s.block_dir = 0;
    sim_tick(&s, DOOR_CMD_CLOSE);
    CHECK(s.fsm.fault == DOOR_FAULT_NONE);
    run_until_finished(&s, 30000);
    CHECK(s.fsm.state == DOOR_STATE_CLOSED);
    CHECK(s.fsm.result == DOOR_RESULT_DONE);
}

static void test_held_while_opening_goes_back_closed(void)
{
    sim_t s; sim_init_held(&s);
    s.block_dir = +1; s.block_pos = OPEN_POS / 3;
    sim_tick(&s, DOOR_CMD_OPEN);
    run_until_finished(&s, 30000);
    CHECK(s.guard_hits > 0);
    CHECK(s.fsm.state == DOOR_STATE_CLOSED);
    CHECK(s.fsm.fault == DOOR_FAULT_OBSTRUCTED);
    CHECK(s.fsm.result == DOOR_RESULT_RETURNED);
    CHECK_NEAR(s.pos, 0, CFG.tolerance + 5);
    CHECK(s.reversals_without_brake == 0);
}

/* A close that interrupts an open half way still goes back to OPEN when held
 * (the previous state is "open", not the spot where the close began). */
static void test_held_after_mid_way_reversal_goes_to_other_end(void)
{
    sim_t s; sim_init_held(&s);
    sim_tick(&s, DOOR_CMD_OPEN);
    while (s.pos < OPEN_POS / 2) {
        sim_tick(&s, DOOR_CMD_NONE);
    }
    float mid = s.pos;
    s.block_dir = -1; s.block_pos = mid - 300;        /* held on the way down */
    sim_tick(&s, DOOR_CMD_CLOSE);
    CHECK(s.fsm.move_origin == OPEN_POS);
    run_until_finished(&s, 30000);
    CHECK(s.fsm.state == DOOR_STATE_OPEN);
    CHECK(s.fsm.result == DOOR_RESULT_RETURNED);
    CHECK_NEAR(s.pos, OPEN_POS, CFG.tolerance + 5);
    CHECK(s.reversals_without_brake == 0);
}

/* MOVE commands (what the firmware uses) with an explicit anchor. */
static void test_move_with_anchor(void)
{
    sim_t s; sim_init_held(&s);
    door_fsm_cmd_t open = { .type = DOOR_CMD_MOVE, .absolute = true, .delta = OPEN_POS,
                            .rpm = 3.0f, .has_anchor = true, .anchor = 0 };
    sim_tick_cmd(&s, &open);
    run_until_finished(&s, 30000);
    CHECK(s.fsm.state == DOOR_STATE_OPEN && s.fsm.result == DOOR_RESULT_DONE);

    /* close (aimed past 0, like the door switch search), held -> open */
    door_fsm_cmd_t close = { .type = DOOR_CMD_MOVE, .absolute = true, .delta = -100,
                             .rpm = 3.0f, .has_anchor = true, .anchor = OPEN_POS };
    s.block_dir = -1; s.block_pos = OPEN_POS / 2;
    sim_tick_cmd(&s, &close);
    run_until_finished(&s, 40000);
    CHECK(s.fsm.state == DOOR_STATE_OPEN);
    CHECK(s.fsm.result == DOOR_RESULT_RETURNED);
    CHECK_NEAR(s.pos, OPEN_POS, CFG.tolerance + 5);

    /* from half way, opening, held -> back to the anchor 0 = CLOSED */
    s.block_dir = 0;
    door_fsm_cmd_t half = { .type = DOOR_CMD_MOVE, .absolute = true, .delta = OPEN_POS / 2,
                            .rpm = 3.0f };
    sim_tick_cmd(&s, &half);
    run_until_finished(&s, 40000);
    CHECK(s.fsm.state == DOOR_STATE_IDLE);
    s.block_dir = +1; s.block_pos = OPEN_POS * 3 / 4;
    sim_tick_cmd(&s, &open);
    run_until_finished(&s, 40000);
    CHECK(s.fsm.state == DOOR_STATE_CLOSED);
    CHECK(s.fsm.result == DOOR_RESULT_RETURNED);
    CHECK_NEAR(s.pos, 0, CFG.tolerance + 5);
    CHECK(s.reversals_without_brake == 0);
}

/* No anchor given: a MOVE still goes back to where it started. */
static void test_move_without_anchor_returns_to_start(void)
{
    sim_t s; sim_init_held(&s);
    s.pos = 600;
    s.block_dir = +1; s.block_pos = 1200;
    door_fsm_cmd_t c = { .type = DOOR_CMD_MOVE, .absolute = true, .delta = OPEN_POS,
                         .rpm = 3.0f };
    sim_tick_cmd(&s, &c);
    run_until_finished(&s, 40000);
    CHECK(s.fsm.state == DOOR_STATE_IDLE);
    CHECK(s.fsm.result == DOOR_RESULT_RETURNED);
    CHECK_NEAR(s.pos, 600, CFG.tolerance + 5);
}

/* Held while going back too: stops where it is (stuck both ways).
 * Speed check on, as in the firmware: a door that is already blocked when a
 * leg starts makes the current guard learn the blocked current as "normal",
 * so only the absolute limit or the speed check can catch it. */
static void test_held_both_ways_stops(void)
{
    sim_t s; sim_init(&s);
    s.cfg.max_retries = 0;
    s.block_dir = +1; s.block_pos = OPEN_POS / 2;
    sim_tick(&s, DOOR_CMD_OPEN);
    while (s.fsm.state != DOOR_STATE_BACKING_OFF) {
        sim_tick(&s, DOOR_CMD_NONE);
    }
    s.block_dir = -1; s.block_pos = s.pos;
    run_until_finished(&s, 10000);
    CHECK(s.fsm.state == DOOR_STATE_FAULT);
    CHECK(s.fsm.fault == DOOR_FAULT_BLOCKED);
    CHECK(s.fsm.result == DOOR_RESULT_BLOCKED);
}

int main(void)
{
    printf("door_fsm / motor_control\n");
    RUN(test_open_then_close);
    RUN(test_repeated_command_is_ignored);
    RUN(test_interference_then_success);
    RUN(test_persistent_interference_goes_back_closed);
    RUN(test_closing_on_obstacle_reopens);
    RUN(test_blocked_both_ways);
    RUN(test_stop_command);
    RUN(test_reverse_mid_travel_brakes_first);
    RUN(test_timeout);
    RUN(test_homing_sets_zero);
    RUN(test_interlock_rules);
    RUN(test_pulse_tracker);
    RUN(test_speed_ctrl);
    RUN(test_current_mode_free_cycle);
    RUN(test_current_mode_hand_while_opening);
    RUN(test_current_mode_hand_while_closing);
    RUN(test_current_mode_ignored_when_idle);
    RUN(test_guard_unit);
    RUN(test_move_relative_from_anywhere);
    RUN(test_move_speed_is_used);
    RUN(test_move_resistance_returns_to_anchor);
    RUN(test_move_retry_succeeds);
    RUN(test_move_return_speed);
    RUN(test_move_stop_and_zero);
    RUN(test_move_to_absolute);
    RUN(test_move_to_reverses_mid_way);
    RUN(test_lock_sequence);
    RUN(test_lock_new_move_while_locking);
    RUN(test_door_with_lock);
    RUN(test_held_while_closing_goes_back_open);
    RUN(test_held_while_opening_goes_back_closed);
    RUN(test_held_after_mid_way_reversal_goes_to_other_end);
    RUN(test_move_with_anchor);
    RUN(test_move_without_anchor_returns_to_start);
    RUN(test_held_both_ways_stops);
    return check_summary("door");
}
