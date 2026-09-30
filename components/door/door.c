/*
 * door.c — Door control task: feedback -> door_fsm -> interlock -> PI -> motor.
 * See door.h.
 */
#include <math.h>
#include <stdlib.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "bldc_motor.h"
#include "current_guard.h"
#include "door.h"
#include "door_fsm.h"
#include "dri0042.h"
#include "encoder.h"
#include "motor_control.h"
#include "pwm_util.h"

static const char *TAG = "door";

#define DOOR_TASK_PRIORITY  10     /* above LEDs/eyes/audio: safety first */
#define EVENT_TASK_PRIORITY 3      /* logging + callbacks, off the control loop */
#define EVENT_QUEUE_LEN     16
#define SPEED_WINDOW_MAX    32     /* samples kept for the speed window   */
#define MIN_GRACE_AFTER_BRAKE_MS 250   /* spin-up time left after the brake */
#define SENSE_TASK_PRIORITY 9      /* just below the control loop          */
#define SENSOR_FAIL_READS   20     /* this many failed reads in a row = sensor lost */
#define DONE_BIT            BIT0   /* s_done: a move has ended             */
#define GUARD_BASELINE_TAU_MS 3000
#define UNLIMITED_A         1.0e6f

typedef struct {
    door_state_t      state;
    door_fault_t      fault;
    bool              homed;
    bool              door_changed;   /* state/fault changed -> callback      */
    bool              lock_changed;   /* latch changed -> log only            */
    door_lock_state_t lock;
} door_event_t;

static door_config_t       s_cfg;
static door_fsm_config_t   s_fsm_cfg;
static speed_ctrl_config_t s_pid_cfg;
static float               s_counts_per_deg;

static portMUX_TYPE        s_lock = portMUX_INITIALIZER_UNLOCKED;
static door_fsm_cmd_t      s_pending;                   /* latest command wins */
static bool                s_has_pending;
static uint32_t            s_posted_seq;                /* last command posted    */
static uint32_t            s_done_seq;                  /* last command finished  */
static EventGroupHandle_t  s_done;
static door_current_limits_t s_limits;                  /* limits of the active move */
static volatile bool       s_sensor_failed;
static bool                s_ext_obstruction;           /* door_report_obstruction() */
static door_status_t       s_status;
static door_event_cb_t     s_cb;
static void               *s_cb_ctx;
static QueueHandle_t       s_events;
static volatile bool       s_latch_hold;                /* door_latch_hold_unlocked() */
static uint32_t            s_latch_duty_full;

/* ---- motor backend ----------------------------------------------------------- */

static bool is_bldc(void)
{
    return s_cfg.motor_type == DOOR_MOTOR_BLDC;
}

static void motor_drive(int8_t dir, float duty)
{
    if (is_bldc()) {
        bldc_motor_drive(dir, duty);
    } else {
        dri0042_drive(dir, duty);
    }
}

static void motor_stop(void)
{
    if (is_bldc()) {
        bldc_motor_stop();
    } else {
        dri0042_stop();
    }
}

static esp_err_t motor_init(void)
{
    if (is_bldc()) {
        bldc_motor_config_t m = {
            .pins    = s_cfg.bldc,
            .timer   = s_cfg.pwm_timer,
            .channel = s_cfg.pwm_channel,
            .freq_hz = s_cfg.pwm_freq_hz,
        };
        ESP_RETURN_ON_ERROR(bldc_motor_init(&m), TAG, "BLDC motor");
        return encoder_init_pulses(s_cfg.bldc.fg_gpio);
    }
    dri0042_config_t d = {
        .in1_gpio     = s_cfg.dri0042.in1_gpio,
        .in2_gpio     = s_cfg.dri0042.in2_gpio,
        .pwm_gpio     = s_cfg.dri0042.pwm_gpio,
        .timer        = s_cfg.pwm_timer,
        .channel      = s_cfg.pwm_channel,
        .freq_hz      = s_cfg.pwm_freq_hz,
        .min_pulse_ns = s_cfg.dri0042.min_pulse_ns,
        .invert       = s_cfg.dri0042.invert_motor,
    };
    ESP_RETURN_ON_ERROR(dri0042_init(&d), TAG, "DRI0042");
    return encoder_init(s_cfg.dri0042.hall_a_gpio, s_cfg.dri0042.hall_b_gpio,
                        s_cfg.dri0042.invert_encoder);
}

/* ---- latch solenoid ------------------------------------------------------------ */

static bool has_latch(void)
{
    return s_cfg.latch.gpio != GPIO_NUM_NC;
}

/* power 0 = off (latch extended = locked), 1 = full (retracted). */
static void latch_write(float power)
{
    if (!has_latch()) {
        return;
    }
    uint32_t duty = (uint32_t)lroundf(power * (float)s_latch_duty_full);
    if (duty > s_latch_duty_full) {
        duty = s_latch_duty_full;
    }
    ledc_set_duty(PWM_UTIL_LEDC_MODE, s_cfg.latch.channel, duty);
    ledc_update_duty(PWM_UTIL_LEDC_MODE, s_cfg.latch.channel);
}

static esp_err_t latch_init(void)
{
    if (!has_latch()) {
        return ESP_OK;
    }
    /* Drive LOW (locked) at once, before the PWM is set up. */
    gpio_config_t io = { .pin_bit_mask = 1ULL << s_cfg.latch.gpio, .mode = GPIO_MODE_OUTPUT };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "latch GPIO");
    gpio_set_level(s_cfg.latch.gpio, 0);
    ESP_RETURN_ON_ERROR(pwm_util_timer_init(s_cfg.latch.timer, s_cfg.latch.pwm_freq_hz,
                                            &s_latch_duty_full), TAG, "latch LEDC timer");
    return pwm_util_channel_init(s_cfg.latch.channel, s_cfg.latch.timer, s_cfg.latch.gpio);
}

/* Would this command turn the motor, starting at `pos`? */
static bool cmd_moves(const door_fsm_cmd_t *c, int32_t pos)
{
    int32_t target;
    switch (c->type) {
    case DOOR_CMD_OPEN:  target = s_fsm_cfg.open_pos;                   break;
    case DOOR_CMD_CLOSE: target = 0;                                    break;
    case DOOR_CMD_MOVE:  target = c->absolute ? c->delta : pos + c->delta; break;
    case DOOR_CMD_HOME:  return true;
    default:             return false;
    }
    return labs((long)target - pos) > s_fsm_cfg.tolerance;
}

/* Worst-case time for one leg of `travel_deg` at `rpm`: 2 x nominal + 3 s. */
static uint32_t auto_timeout_ms(float travel_deg, float rpm)
{
    return (uint32_t)(fabsf(travel_deg) / 360.0f * 60.0f / rpm * 2000.0f) + 3000;
}

/* Hand a command to the control task. Returns its sequence number. */
static uint32_t post_cmd(const door_fsm_cmd_t *cmd, const door_current_limits_t *limits)
{
    xEventGroupClearBits(s_done, DONE_BIT);
    taskENTER_CRITICAL(&s_lock);
    s_pending     = *cmd;
    s_has_pending = true;
    if (limits) {
        s_limits = *limits;
    }
    uint32_t seq = ++s_posted_seq;
    taskEXIT_CRITICAL(&s_lock);
    return seq;
}

static void post(door_cmd_t type)
{
    door_fsm_cmd_t c = door_fsm_cmd(type);
    post_cmd(&c, type == DOOR_CMD_STOP ? NULL : &s_cfg.current);
}

/* ---- motor current sensing (optional) ---------------------------------------- */

static float s_cur_a, s_normal_a, s_peak_a, s_limit_now_a;

/*
 * Samples the motor current every sense_period_ms and runs the resistance
 * detector with the ACTIVE move's limits. It never touches the motor: it only
 * raises the obstruction flag, which the control task acts on.
 */
static void sense_task(void *arg)
{
    current_guard_t guard;
    current_guard_init(&guard);
    int        fails    = 0;
    int8_t     last_dir = 0;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(s_cfg.sense_period_ms));

        float amps = 0.0f;
        if (!s_cfg.read_current(&amps, s_cfg.read_current_ctx)) {
            if (++fails >= SENSOR_FAIL_READS) {
                s_sensor_failed = true;        /* control task stops the motor */
            }
            continue;
        }
        fails = 0;
        s_sensor_failed = false;
        amps = fabsf(amps);

        taskENTER_CRITICAL(&s_lock);
        door_current_limits_t lim = s_limits;
        int8_t dir = s_status.direction;
        taskEXIT_CRITICAL(&s_lock);

        current_guard_config_t gc = {
            .margin_a        = lim.rise_a  > 0.0f ? lim.rise_a  : UNLIMITED_A,
            .ratio           = 1.0f,
            .max_a           = lim.limit_a > 0.0f ? lim.limit_a : UNLIMITED_A,
            .settle_ms       = lim.ignore_start_ms,
            .confirm_ms      = lim.confirm_ms,
            .baseline_tau_ms = GUARD_BASELINE_TAU_MS,
        };
        bool resisting = current_guard_update(&guard, &gc,
                                              (uint32_t)(esp_timer_get_time() / 1000),
                                              dir, amps);

        taskENTER_CRITICAL(&s_lock);
        if (resisting) {
            s_ext_obstruction = true;
        }
        if (dir != last_dir && dir != 0 && last_dir == 0) {
            s_peak_a = 0.0f;                    /* a new movement starts */
        }
        if (dir != 0 && amps > s_peak_a) {
            s_peak_a = amps;
        }
        s_cur_a       = amps;
        s_normal_a    = current_guard_baseline(&guard, dir ? dir : 1);
        s_limit_now_a = guard.threshold;
        taskEXIT_CRITICAL(&s_lock);
        last_dir = dir;
    }
}

/*
 * The control loop never logs, prints or takes a mutex: anything slow is
 * handed to event_task through a queue, so a busy serial port can never
 * leave the motor running unsupervised.
 */
static void door_task(void *arg)
{
    door_fsm_t        fsm;
    motor_interlock_t lock;
    speed_ctrl_t      pid = { 0 };
    pulse_tracker_t   fg;

    door_fsm_init(&fsm, DOOR_STATE_CLOSED, (uint32_t)(esp_timer_get_time() / 1000));
    motor_interlock_init(&lock, s_cfg.stop_before_reverse_ms);

    /* Position in counts: quadrature is signed already; FG pulses get their
     * sign from the driven direction (pulse_tracker). */
    int32_t raw0 = encoder_get_count();
    pulse_tracker_init(&fg, raw0);
    int32_t zero = is_bldc() ? 0 : raw0;        /* power-up position = closed */
    int8_t  dir  = 0;
    uint32_t active_seq = 0;                 /* command the running move belongs to */
    bool     was_standstill = true;

    /* Latch: power-up = door closed = LOCKED. A command that would move the
     * door while the latch is not fully retracted waits in `deferred`. */
    door_lock_t    latch;
    door_fsm_cmd_t deferred = { 0 };
    bool           has_deferred = false;
    door_lock_init(&latch, has_latch(), (uint32_t)(esp_timer_get_time() / 1000));

    /* Automatic close after the door has stood open for hold_open_ms. */
    bool     close_pending = false;
    uint32_t close_at_ms   = 0;

    /* Speed = counts moved over the last `speed_window_ms` (works with the
     * few pulses per turn an FG output gives at low speed). */
    int32_t win_pos[SPEED_WINDOW_MAX];
    int64_t win_us[SPEED_WINDOW_MAX];
    int     win_len = (int)(s_cfg.speed_window_ms / s_cfg.control_period_ms) + 1;
    if (win_len < 2) win_len = 2;
    if (win_len > SPEED_WINDOW_MAX) win_len = SPEED_WINDOW_MAX;
    int     win_i = 0;
    int64_t t0 = esp_timer_get_time();
    for (int i = 0; i < win_len; i++) {
        win_pos[i] = 0;
        win_us[i]  = t0;
    }

    esp_task_wdt_add(NULL);                   /* a hung loop resets the chip */
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(s_cfg.control_period_ms));
        esp_task_wdt_reset();

        int64_t  now_us = esp_timer_get_time();
        uint32_t now_ms = (uint32_t)(now_us / 1000);

        /* ---- measure --------------------------------------------------- */
        int32_t raw = encoder_get_count();
        int32_t abs_pos = is_bldc() ? pulse_tracker_update(&fg, raw, dir) : raw;
        int32_t pos = abs_pos - zero;

        int oldest = (win_i + 1) % win_len;           /* ring buffer */
        win_pos[win_i] = abs_pos;
        win_us[win_i]  = now_us;
        float span_s = (float)(now_us - win_us[oldest]) / 1e6f;
        float out_rpm = 0.0f;
        if (span_s > 0.0f) {
            out_rpm = (float)(abs_pos - win_pos[oldest]) / s_cfg.counts_per_output_rev
                      / span_s * 60.0f;
        }
        bool standstill = (abs_pos == win_pos[oldest]);  /* no count in the window */
        win_i = oldest;

        /* ---- decide ---------------------------------------------------- */
        taskENTER_CRITICAL(&s_lock);
        door_fsm_cmd_t cmd = s_pending;
        bool has_cmd = s_has_pending;
        s_has_pending = false;
        uint32_t cmd_seq = s_posted_seq;
        bool ext_obstruction = s_ext_obstruction;
        s_ext_obstruction = false;
        taskEXIT_CRITICAL(&s_lock);
        if (has_cmd) {
            active_seq = cmd_seq;
        }

        /* ---- latch: retract it before anything moves --------------------- */
        if (has_cmd) {
            close_pending = false;                 /* any command cancels auto-close */
            has_deferred  = false;                 /* ...and replaces a waiting one  */
            if (has_latch() && latch.state != DOOR_LOCK_UNLOCKED && cmd_moves(&cmd, pos)) {
                deferred     = cmd;                /* hold it until the latch is clear */
                has_deferred = true;
                has_cmd      = false;
            }
        }
        if (has_deferred && latch.state == DOOR_LOCK_UNLOCKED) {
            cmd          = deferred;               /* latch clear: go */
            has_cmd      = true;
            has_deferred = false;
        }

        /* Door stood open long enough: close it (the latch is already clear). */
        if (close_pending && !has_cmd && !has_deferred &&
            (int32_t)(now_ms - close_at_ms) >= 0) {
            close_pending = false;
            door_move_t m = door_move_defaults(0.0f, s_cfg.auto_close_rpm);
            cmd = (door_fsm_cmd_t){
                .type          = DOOR_CMD_MOVE,
                .delta         = 0,
                .absolute      = true,
                .rpm           = m.rpm,
                .max_retries   = m.max_retries,
                .retry_wait_ms = m.retry_wait_ms,
                .timeout_ms    = auto_timeout_ms((float)pos / s_counts_per_deg, m.rpm),
            };
            has_cmd = true;
            taskENTER_CRITICAL(&s_lock);
            s_limits = s_cfg.current;
            taskEXIT_CRITICAL(&s_lock);
        }

        /* Current sensor lost while moving: stop at once, report SENSOR_FAIL. */
        bool sensor_abort = false;
        if (s_cfg.read_current && s_sensor_failed && dir != 0) {
            cmd = door_fsm_cmd(DOOR_CMD_STOP);
            has_cmd = true;
            sensor_abort = true;
        }

        door_fsm_output_t o = door_fsm_step(&fsm, &s_fsm_cfg, has_cmd ? &cmd : NULL,
                                            now_ms, pos, out_rpm, ext_obstruction);
        if (sensor_abort) {
            fsm.result = DOOR_RESULT_SENSOR_FAIL;
        }
        if (o.zero_position) {
            zero = abs_pos;
            pos  = 0;
        }

        if (o.finished && s_cfg.hold_open_ms > 0 && fsm.result == DOOR_RESULT_DONE &&
            labs(pos) > s_fsm_cfg.tolerance) {
            close_pending = true;                  /* reached open: start the hold */
            close_at_ms   = now_ms + s_cfg.hold_open_ms;
        }

        /* ---- latch ----------------------------------------------------- */
        bool busy = (fsm.result == DOOR_RESULT_BUSY);
        door_lock_output_t lk = { .motion_allowed = true };
        if (has_latch()) {
            bool closed_rest = !busy && labs(pos) <= s_fsm_cfg.tolerance &&
                               dir == 0 && was_standstill && standstill;
            lk = door_lock_step(&latch, &s_cfg.latch.timing, now_ms,
                                busy || has_deferred || s_latch_hold, closed_rest);
            latch_write(lk.solenoid);
        }
        was_standstill = standstill;

        /* ---- act ------------------------------------------------------- */
        int8_t wanted = (o.target_rpm > 0.0f) - (o.target_rpm < 0.0f);
        if (!lk.motion_allowed) {
            wanted = 0;                            /* never drive into the latch */
        }
        dir = motor_interlock_step(&lock, wanted, now_us, standstill);
        float duty = 0.0f;

        if (dir == 0) {
            speed_ctrl_reset(&pid);
            motor_stop();
        } else {
            duty = fabsf(speed_ctrl_step(&pid, &s_pid_cfg, o.target_rpm, out_rpm,
                                         (float)s_cfg.control_period_ms / 1000.0f));
            motor_drive(dir, duty);
        }

        /* ---- report ---------------------------------------------------- */
        taskENTER_CRITICAL(&s_lock);
        s_status = (door_status_t){
            .state        = fsm.state,
            .fault        = fsm.fault,
            .retries      = fsm.retries,
            .position_deg = (float)pos / s_counts_per_deg,
            .speed_rpm    = out_rpm,
            .duty         = duty,
            .direction    = dir,
            .result       = fsm.result,
            .current_a    = s_cur_a,
            .normal_a     = s_normal_a,
            .peak_a       = s_peak_a,
            .limit_now_a  = s_limit_now_a,
            .lock         = has_latch() ? latch.state : DOOR_LOCK_UNLOCKED,
            .solenoid     = lk.solenoid,
            .auto_close_in_ms = !close_pending ? 0
                              : (int32_t)(close_at_ms - now_ms) > 0 ? close_at_ms - now_ms : 1,
        };
        if (o.finished) {
            s_done_seq = active_seq;
        }
        taskEXIT_CRITICAL(&s_lock);
        if (o.finished) {
            xEventGroupSetBits(s_done, DONE_BIT);
        }

        if (o.state_changed || o.zero_position || lk.changed) {
            door_event_t ev = {
                .state        = fsm.state,
                .fault        = fsm.fault,
                .homed        = o.zero_position,
                .door_changed = o.state_changed || o.zero_position,
                .lock_changed = lk.changed,
                .lock         = latch.state,
            };
            xQueueSend(s_events, &ev, 0);     /* never blocks; drops if full */
        }
    }
}

/* Low-priority: logs door events and runs the user callback. */
static void event_task(void *arg)
{
    door_event_t ev;
    for (;;) {
        if (xQueueReceive(s_events, &ev, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (ev.lock_changed) {
            ESP_LOGI(TAG, "latch -> %s", door_lock_state_name(ev.lock));
        }
        if (!ev.door_changed) {
            continue;
        }
        if (ev.homed) {
            ESP_LOGI(TAG, "homed: closed position set");
        }
        if (ev.fault != DOOR_FAULT_NONE) {
            ESP_LOGW(TAG, "-> %s (%s)", door_state_name(ev.state), door_fault_name(ev.fault));
        } else {
            ESP_LOGI(TAG, "-> %s", door_state_name(ev.state));
        }

        taskENTER_CRITICAL(&s_lock);
        door_event_cb_t cb  = s_cb;
        void           *ctx = s_cb_ctx;
        taskEXIT_CRITICAL(&s_lock);
        if (cb) {
            cb(ev.state, ev.fault, ctx);
        }
    }
}

static void shutdown_stop(void)
{
    motor_stop();
    latch_write(0.0f);                        /* lock on restart */
}

esp_err_t door_init(const door_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg->control_period_ms > 0 && cfg->counts_per_output_rev > 0.0f &&
                        cfg->output_rpm_at_full_duty > 0.0f,
                        ESP_ERR_INVALID_ARG, TAG, "motor parameters");
    ESP_RETURN_ON_FALSE(cfg->motor_type != DOOR_MOTOR_DRI0042 ||
                        cfg->stop_before_reverse_ms >= 100, ESP_ERR_INVALID_ARG, TAG,
                        "DRI0042 needs >= 100 ms stop before reversing");
    s_cfg = *cfg;

    /* Motor and latch first, so the motor is stopped (and braked) and the
     * door locked as early as possible. */
    ESP_RETURN_ON_ERROR(motor_init(), TAG, "motor");
    ESP_RETURN_ON_ERROR(latch_init(), TAG, "latch");
    if (s_cfg.auto_close_rpm <= 0.0f) {
        s_cfg.auto_close_rpm = cfg->cruise_rpm;
    }

    s_counts_per_deg = cfg->counts_per_output_rev / 360.0f;

    s_fsm_cfg = (door_fsm_config_t){
        .open_pos          = (int32_t)lroundf(cfg->open_angle_deg * s_counts_per_deg),
        .tolerance         = (int32_t)lroundf(cfg->tolerance_deg * s_counts_per_deg),
        .slowdown_dist     = (int32_t)lroundf(cfg->slowdown_deg * s_counts_per_deg),
        .cruise_rpm        = cfg->cruise_rpm,
        .approach_rpm      = cfg->approach_rpm,
        .homing_rpm        = cfg->homing_rpm,
        .spinup_grace_ms   = cfg->spinup_grace_ms,
        .obstruction_ms    = cfg->obstruction_ms,
        .obstruction_ratio = cfg->obstruction_ratio,
        .retry_wait_ms     = cfg->retry_wait_ms,
        .max_retries       = cfg->max_retries,
        .move_timeout_ms   = cfg->move_timeout_ms,
    };
    if (s_fsm_cfg.tolerance < 1) {
        s_fsm_cfg.tolerance = 1;
    }
    s_pid_cfg = (speed_ctrl_config_t){
        .kp             = cfg->kp,
        .ki             = cfg->ki,
        .integral_limit = cfg->integral_limit,
        .ff_per_rpm     = 1.0f / cfg->output_rpm_at_full_duty,
        .min_duty       = cfg->min_duty,
    };

    /* The interlock holds STOP at every reversal; the interference check
     * must not start before the motor could spin up again. */
    uint32_t min_grace = cfg->stop_before_reverse_ms + MIN_GRACE_AFTER_BRAKE_MS;
    if (s_fsm_cfg.spinup_grace_ms < min_grace) {
        ESP_LOGW(TAG, "spin-up grace raised to %lu ms (stop %lu ms + spin-up)",
                 (unsigned long)min_grace, (unsigned long)cfg->stop_before_reverse_ms);
        s_fsm_cfg.spinup_grace_ms = min_grace;
    }

    s_status = (door_status_t){
        .state = DOOR_STATE_CLOSED,
        .lock  = has_latch() ? DOOR_LOCK_LOCKED : DOOR_LOCK_UNLOCKED,
    };

    /* Stop the motor on any orderly restart (esp_restart, reboot command). */
    esp_register_shutdown_handler(shutdown_stop);

    s_limits = cfg->current;
    s_done   = xEventGroupCreate();
    s_events = xQueueCreate(EVENT_QUEUE_LEN, sizeof(door_event_t));
    if (!s_events || !s_done ||
        xTaskCreate(event_task, "door_evt", 3072, NULL, EVENT_TASK_PRIORITY, NULL) != pdPASS ||
        xTaskCreate(door_task, "door", 4096, NULL, DOOR_TASK_PRIORITY, NULL) != pdPASS) {
        motor_stop();
        return ESP_ERR_NO_MEM;
    }
    if (cfg->read_current) {
        ESP_RETURN_ON_ERROR(door_attach_current_sensor(cfg->read_current, cfg->read_current_ctx),
                            TAG, "current sensor");
    }

    ESP_LOGI(TAG, "ready (%s): open = %.0f deg (%ld counts, %.1f counts/deg), cruise %.1f rpm, "
                  "retry after %lu ms, max %u retries",
             is_bldc() ? "BLDC + FG" : "DRI0042 + encoder",
             cfg->open_angle_deg, (long)s_fsm_cfg.open_pos, s_counts_per_deg,
             cfg->cruise_rpm, (unsigned long)cfg->retry_wait_ms, cfg->max_retries);
    if (s_counts_per_deg < 2.0f) {
        ESP_LOGW(TAG, "only %.2f feedback counts per degree: positioning will be coarse",
                 s_counts_per_deg);
    }

    if (has_latch()) {
        ESP_LOGI(TAG, "latch on GPIO%d: unlock %lu ms, settle %lu ms, hold power %.0f%%",
                 s_cfg.latch.gpio, (unsigned long)s_cfg.latch.timing.unlock_ms,
                 (unsigned long)s_cfg.latch.timing.settle_ms,
                 s_cfg.latch.timing.hold_duty * 100.0f);
    }
    if (s_cfg.hold_open_ms > 0) {
        ESP_LOGI(TAG, "auto-close %lu ms after opening", (unsigned long)s_cfg.hold_open_ms);
    }

    if (cfg->home_at_boot) {
        door_home();
    } else {
        ESP_LOGI(TAG, "assuming the door is CLOSED at power-up (use 'door home' if not)");
    }
    return ESP_OK;
}

void door_open(void)  { post(DOOR_CMD_OPEN);  }
void door_close(void) { post(DOOR_CMD_CLOSE); }
void door_stop(void)  { post(DOOR_CMD_STOP);  }
void door_home(void)  { post(DOOR_CMD_HOME);  }

void door_latch_hold_unlocked(bool hold)
{
    s_latch_hold = hold;
}

void door_report_obstruction(void)
{
    taskENTER_CRITICAL(&s_lock);
    s_ext_obstruction = true;
    taskEXIT_CRITICAL(&s_lock);
}

door_status_t door_get_status(void)
{
    taskENTER_CRITICAL(&s_lock);
    door_status_t s = s_status;
    taskEXIT_CRITICAL(&s_lock);
    return s;
}

void door_set_event_callback(door_event_cb_t cb, void *ctx)
{
    taskENTER_CRITICAL(&s_lock);
    s_cb     = cb;
    s_cb_ctx = ctx;
    taskEXIT_CRITICAL(&s_lock);
}

/* ---- moves by angle -------------------------------------------------------------- */

door_move_t door_move_defaults(float degrees, float rpm)
{
    return (door_move_t){
        .degrees       = degrees,
        .rpm           = rpm,
        .return_rpm    = 0.0f,
        .current       = s_cfg.current,
        .max_retries   = s_cfg.max_retries,
        .retry_wait_ms = s_cfg.retry_wait_ms,
        .timeout_ms    = 0,
    };
}

esp_err_t door_rotate_start(const door_move_t *m)
{
    ESP_RETURN_ON_FALSE(s_done != NULL, ESP_ERR_INVALID_STATE, TAG, "door_init first");
    ESP_RETURN_ON_FALSE(m && m->rpm > 0.0f, ESP_ERR_INVALID_ARG, TAG, "rpm must be > 0");

    uint32_t timeout = m->timeout_ms;
    if (timeout == 0) {                       /* automatic: 2 x travel time + 3 s */
        float slowest = (m->return_rpm > 0.0f && m->return_rpm < m->rpm) ? m->return_rpm
                                                                          : m->rpm;
        float travel  = m->absolute ? m->degrees - door_get_status().position_deg
                                    : m->degrees;
        timeout = auto_timeout_ms(travel, slowest);
    }
    door_fsm_cmd_t c = {
        .type          = DOOR_CMD_MOVE,
        .delta         = (int32_t)lroundf(m->degrees * s_counts_per_deg),
        .absolute      = m->absolute,
        .rpm           = m->rpm,
        .return_rpm    = m->return_rpm,
        .max_retries   = m->max_retries,
        .retry_wait_ms = m->retry_wait_ms,
        .timeout_ms    = timeout,
    };
    post_cmd(&c, &m->current);
    return ESP_OK;
}

door_result_t door_wait(uint32_t timeout_ms)
{
    if (s_done == NULL) {
        return DOOR_RESULT_BUSY;
    }
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    for (;;) {
        taskENTER_CRITICAL(&s_lock);
        bool done = (s_done_seq == s_posted_seq);
        door_result_t r = s_status.result;
        taskEXIT_CRITICAL(&s_lock);
        if (done && r != DOOR_RESULT_BUSY) {
            return r;
        }
        int64_t left_us = deadline - esp_timer_get_time();
        if (left_us <= 0) {
            return DOOR_RESULT_BUSY;
        }
        xEventGroupWaitBits(s_done, DONE_BIT, pdTRUE, pdFALSE,
                            pdMS_TO_TICKS(left_us / 1000 + 1));
    }
}

door_result_t door_rotate(const door_move_t *m)
{
    if (door_rotate_start(m) != ESP_OK) {
        return DOOR_RESULT_BUSY;
    }
    /* longest possible: every attempt + every return + every wait, plus margin */
    return door_wait(UINT32_MAX / 2);
}

door_result_t door_open_by(float degrees, float rpm, float current_limit_a)
{
    door_move_t m = door_move_defaults(fabsf(degrees), rpm);
    m.current.limit_a = current_limit_a;
    return door_rotate(&m);
}

static door_move_t move_to(float target_deg, float rpm, float current_limit_a)
{
    door_move_t m = door_move_defaults(target_deg, rpm);
    m.absolute        = true;
    m.current.limit_a = current_limit_a;
    return m;
}

door_result_t door_move_to(float target_deg, float rpm, float current_limit_a)
{
    door_move_t m = move_to(target_deg, rpm, current_limit_a);
    return door_rotate(&m);
}

esp_err_t door_move_to_start(float target_deg, float rpm, float current_limit_a)
{
    door_move_t m = move_to(target_deg, rpm, current_limit_a);
    return door_rotate_start(&m);
}

door_result_t door_close_by(float degrees, float rpm, float current_limit_a)
{
    door_move_t m = door_move_defaults(-fabsf(degrees), rpm);
    m.current.limit_a = current_limit_a;
    return door_rotate(&m);
}

esp_err_t door_attach_current_sensor(door_current_reader_t reader, void *ctx)
{
    ESP_RETURN_ON_FALSE(reader != NULL, ESP_ERR_INVALID_ARG, TAG, "reader");
    ESP_RETURN_ON_FALSE(s_done != NULL, ESP_ERR_INVALID_STATE, TAG, "door_init first");
    static bool started;
    if (started) {
        return ESP_ERR_INVALID_STATE;
    }
    s_cfg.read_current     = reader;
    s_cfg.read_current_ctx = ctx;
    if (s_cfg.sense_period_ms == 0) {
        s_cfg.sense_period_ms = 5;
    }
    if (xTaskCreate(sense_task, "door_sense", 3072, NULL, SENSE_TASK_PRIORITY, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    started = true;
    ESP_LOGI(TAG, "motor current guard on: limit %.2f A, rise %.2f A, confirm %lu ms, "
                  "ignore first %lu ms", s_cfg.current.limit_a, s_cfg.current.rise_a,
             (unsigned long)s_cfg.current.confirm_ms, (unsigned long)s_cfg.current.ignore_start_ms);
    return ESP_OK;
}
