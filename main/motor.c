/*
 * motor.c — Door motor front end. See motor.h.
 */
#include <math.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board.h"
#include "config.h"
#include "door.h"
#include "ina228.h"
#include "motor.h"

static const char *TAG = "motor";

#define MIN_RPM  0.5f

static float s_open_deg  = DOOR_OPEN_ANGLE_DEG;
static float s_open_rpm  = DOOR_OPEN_RPM;
static float s_close_rpm = DOOR_CLOSE_RPM;

static ina228_handle_t s_ina228;
static void          (*s_fault_callback)(void);
static void          (*s_held_callback)(bool now_open);

/* Keep a requested speed inside what the motor can do. */
static float limit_rpm(float rpm)
{
    rpm = fabsf(rpm);
    if (rpm < MIN_RPM) {
        ESP_LOGW(TAG, "%.1f rpm is too slow, using %.1f rpm", rpm, MIN_RPM);
        return MIN_RPM;
    }
    if (rpm > MOTOR_RPM_AT_FULL_DUTY) {
        ESP_LOGW(TAG, "%.1f rpm is more than the motor's top speed, using %.1f rpm",
                 rpm, MOTOR_RPM_AT_FULL_DUTY);
        return MOTOR_RPM_AT_FULL_DUTY;
    }
    return rpm;
}

/* ---- setup -------------------------------------------------------------------- */

/* Everything the door component needs to know, from board.h + config.h. */
static door_config_t door_config(void)
{
    return (door_config_t){
        .motor_type = DOOR_MOTOR_BLDC,
        .bldc = {
            .pwm_gpio       = PIN_MOTOR_PWM,
            .dir_gpio       = PIN_MOTOR_DIR,
            .brake_gpio     = MOTOR_HAS_BRAKE ? PIN_MOTOR_BRAKE : GPIO_NUM_NC,
            .fg_gpio        = PIN_MOTOR_FG,
            .dir_level_open = MOTOR_DIR_LEVEL_OPEN,
            .brake_level_on = MOTOR_BRAKE_LEVEL_ON,
            .pwm_inverted   = MOTOR_PWM_INVERTED,
            .dir_open_drain = MOTOR_DIR_OPEN_DRAIN,
        },
        .pwm_timer   = LEDC_TIMER_MOTOR,
        .pwm_channel = LEDC_CH_MOTOR,
        .pwm_freq_hz = MOTOR_PWM_HZ,

        .counts_per_output_rev   = MOTOR_FG_PER_OUTPUT_REV,
        .output_rpm_at_full_duty = MOTOR_RPM_AT_FULL_DUTY,
        .min_duty                = MOTOR_MIN_DUTY,
        .kp                      = MOTOR_KP,
        .ki                      = MOTOR_KI,
        .integral_limit          = MOTOR_INTEGRAL_LIMIT,
        .stop_before_reverse_ms  = MOTOR_STOP_BEFORE_REVERSE_MS,
        .speed_window_ms         = MOTOR_SPEED_WINDOW_MS,
        .control_period_ms       = MOTOR_CONTROL_MS,

        .open_angle_deg    = DOOR_OPEN_ANGLE_DEG,
        .tolerance_deg     = DOOR_TOLERANCE_DEG,
        .slowdown_deg      = DOOR_SLOWDOWN_DEG,
        .cruise_rpm        = DOOR_OPEN_RPM,
        .approach_rpm      = DOOR_APPROACH_RPM,
        .homing_rpm        = DOOR_HOMING_RPM,
        .spinup_grace_ms   = DOOR_SPINUP_GRACE_MS,
        .obstruction_ms    = DOOR_OBSTRUCTION_MS,
        .obstruction_ratio = DOOR_OBSTRUCTION_RATIO,
        .retry_wait_ms     = DOOR_RETRY_WAIT_MS,
        .max_retries       = DOOR_MAX_RETRIES,
        .move_timeout_ms   = DOOR_MOVE_TIMEOUT_MS,
        .home_at_boot      = false,             /* assume closed at power-up */
        .hold_open_ms      = DOOR_AUTO_CLOSE_MS,
        .auto_close_rpm    = DOOR_CLOSE_RPM,
        .closed_switch_gpio   = PIN_DOOR_SWITCH,
        .closed_switch_level  = DOOR_SWITCH_CLOSED_LEVEL,
        .close_overtravel_deg = DOOR_CLOSE_EXTRA_DEG,
        .seat_ms              = DOOR_SEAT_MS,
        .latch_lock_delay_ms  = DOOR_LATCH_DELAY_MS,
        .stop_after_latch_ms  = DOOR_STOP_AFTER_LATCH_MS,
        .open_nudge_deg       = DOOR_OPEN_NUDGE_DEG,
        .open_nudge_ms        = DOOR_OPEN_NUDGE_MS,

        .latch = {
            .gpio        = LATCH_ENABLED ? PIN_DOOR_LATCH : GPIO_NUM_NC,
            .timer       = LEDC_TIMER_LATCH,
            .channel     = LEDC_CH_LATCH,
            .pwm_freq_hz = LATCH_PWM_HZ,
            .timing = {
                .unlock_ms  = LATCH_UNLOCK_MS,
                .settle_ms  = LATCH_SETTLE_MS,
                .pull_in_ms = LATCH_PULL_IN_MS,
                .hold_duty  = LATCH_HOLD_DUTY,
            },
        },

        .current = {
            .limit_a         = DOOR_CURRENT_LIMIT_A,
            .rise_a          = DOOR_CURRENT_RISE_A,
            .confirm_ms      = DOOR_CURRENT_CONFIRM_MS,
            .ignore_start_ms = DOOR_CURRENT_IGNORE_MS,
        },
        .sense_period_ms = 5,
    };
}

/* The door component calls this every 5 ms while it watches for resistance. */
static bool read_motor_current(float *amps, void *ctx)
{
    return ina228_read_current(s_ina228, amps) == ESP_OK;
}

/* INA228 on the shared I2C bus: it sits in the motor's supply wire. */
static esp_err_t start_current_sensor(i2c_master_bus_handle_t bus)
{
    ESP_RETURN_ON_FALSE(bus != NULL, ESP_ERR_INVALID_ARG, TAG, "no I2C bus");
    ina228_config_t ina_cfg = {
        .bus           = bus,
        .address       = INA228_ADDRESS,
        .i2c_speed_hz  = I2C_SPEED_HZ,
        .shunt_ohms    = INA228_SHUNT_OHMS,
        .max_current_a = INA228_MAX_CURRENT_A,
        .mode          = INA228_MODE_FAST_CURRENT,  /* ~9 ms: fast enough for the door */
    };
    ESP_RETURN_ON_ERROR(ina228_create(&ina_cfg, &s_ina228), TAG, "INA228");

    return door_attach_current_sensor(read_motor_current, NULL);
}

/* Runs in the door component's event task whenever the door state changes. */
static void on_door_event(door_state_t state, door_fault_t fault, void *ctx)
{
    if (state == DOOR_STATE_FAULT) {
        ESP_LOGW(TAG, "door gave up: %s", door_fault_name(fault));
        if (s_fault_callback) {
            s_fault_callback();
        }
    } else if (fault == DOOR_FAULT_OBSTRUCTED) {
        bool now_open = (state != DOOR_STATE_CLOSED);
        ESP_LOGW(TAG, "door held while %s: back %s", now_open ? "closing" : "opening",
                 now_open ? "OPEN" : "CLOSED");
        if (s_held_callback) {
            s_held_callback(now_open);
        }
    }
}

#if MOTOR_LOG_EVERY_MS > 0
/*
 * Status line while the door moves. It also watches the FG pin directly
 * (independent of the pulse counter): "FG edges" must grow while the shaft
 * turns. If it stays the same, no FG pulses reach the ESP32.
 */
static void log_task(void *arg)
{
    int      fg_level = gpio_get_level(PIN_MOTOR_FG);
    uint32_t fg_edges = 0;
    uint32_t ms       = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1));
        int level = gpio_get_level(PIN_MOTOR_FG);
        if (level != fg_level) {
            fg_level = level;
            fg_edges++;
        }
        if (++ms < MOTOR_LOG_EVERY_MS) {
            continue;
        }
        ms = 0;

        door_status_t d = door_get_status();
        if (d.direction != 0 || d.result == DOOR_RESULT_BUSY) {
            float supply_v = 0.0f;              /* motor supply, measured by the INA228 */
            if (s_ina228) {
                ina228_read_bus_voltage(s_ina228, &supply_v);
            }
            ESP_LOGI(TAG, "%-11s %6.1f deg %5.1f rpm duty %3.0f%% dir %+d | switch %s | "
                          "solenoid %s | %.1f V %.3f A (limit %.3f) | FG pin %d, %lu edges",
                     door_state_name(d.state), d.position_deg, d.speed_rpm, d.duty * 100.0f,
                     d.direction, d.closed_switch ? "CLOSED" : "open  ",
                     d.solenoid > 0.0f ? "ON " : "off", supply_v, d.current_a,
                     d.limit_now_a, fg_level, (unsigned long)fg_edges);
        }
    }
}
#endif

esp_err_t motor_init(i2c_master_bus_handle_t bus)
{
    door_config_t cfg = door_config();
    ESP_RETURN_ON_ERROR(door_init(&cfg), TAG, "door");
    door_set_event_callback(on_door_event, NULL);
#if MOTOR_LOG_EVERY_MS > 0
    xTaskCreate(log_task, "motor_log", 4096, NULL, 2, NULL);
#endif

    /* The door still works without the current sensor (it then only notices
     * a blocked door from the motor slowing down), so keep going. */
    if (start_current_sensor(bus) != ESP_OK) {
        ESP_LOGW(TAG, "INA228 not available: NO current-based detection of a held door");
    }

    if (s_ina228) {
        float supply_v = 0.0f;
        vTaskDelay(pdMS_TO_TICKS(50));          /* first INA228 conversion */
        ina228_read_bus_voltage(s_ina228, &supply_v);
        ESP_LOGI(TAG, "motor supply %.1f V, FG pin (GPIO%d) reads %d",
                 supply_v, PIN_MOTOR_FG, gpio_get_level(PIN_MOTOR_FG));
    }
    ESP_LOGI(TAG, "door switch (GPIO%d): %s", PIN_DOOR_SWITCH,
             door_get_status().closed_switch ? "CLOSED" : "open - door is not closed!");

    ESP_LOGI(TAG, "ready: open = %.0f deg, open at %.1f rpm, close at %.1f rpm",
             s_open_deg, s_open_rpm, s_close_rpm);
    return ESP_OK;
}

/* ---- settings ------------------------------------------------------------------ */

void motor_set_open_angle(float degrees)
{
    s_open_deg = fabsf(degrees);
    ESP_LOGI(TAG, "open angle -> %.1f deg", s_open_deg);
}

void motor_set_speed(float open_rpm, float close_rpm)
{
    s_open_rpm  = limit_rpm(open_rpm);
    s_close_rpm = limit_rpm(close_rpm);
    ESP_LOGI(TAG, "speed -> open %.1f rpm, close %.1f rpm", s_open_rpm, s_close_rpm);
}

/* ---- door ---------------------------------------------------------------------- */

/* Go TO `target_deg`; if someone holds the door, go back to `back_deg`. */
static void door_go(float target_deg, float rpm, float back_deg)
{
    door_move_t m = door_move_defaults(target_deg, rpm);
    m.absolute        = true;
    m.current.limit_a = DOOR_CURRENT_LIMIT_A;
    m.anchor_set      = true;
    m.anchor_deg      = back_deg;
    door_rotate_start(&m);
}

/* Held while opening: closes again (and locks). */
void motor_open_door(void)
{
    door_go(s_open_deg, s_open_rpm, 0.0f);
}

/* Closes until the door switch closes, seats DOOR_SEAT_MS, stops (that spot
 * becomes the new 0); the latch locks DOOR_LATCH_DELAY_MS after the switch.
 * Held while closing: opens again. */
void motor_close_door(void)
{
    door_go(0.0f, s_close_rpm, s_open_deg);
}

void motor_stop(void)
{
    door_stop();
}

bool motor_wait_done(uint32_t timeout_ms)
{
    return motor_wait(timeout_ms) == MOTOR_DONE;
}

motor_result_t motor_wait(uint32_t timeout_ms)
{
    switch (door_wait(timeout_ms)) {
    case DOOR_RESULT_DONE:     return MOTOR_DONE;
    case DOOR_RESULT_RETURNED: return MOTOR_WENT_BACK;
    case DOOR_RESULT_BUSY:     return MOTOR_STILL_MOVING;
    default:                   return MOTOR_FAILED;
    }
}

/* ---- free rotation ------------------------------------------------------------- */

bool motor_rotate(float degrees, float rpm)
{
    door_move_t move = door_move_defaults(degrees, limit_rpm(rpm));
    door_result_t result = door_rotate(&move);
    if (result != DOOR_RESULT_DONE) {
        ESP_LOGW(TAG, "rotate %.1f deg ended with: %s", degrees, door_result_name(result));
    }
    return result == DOOR_RESULT_DONE;
}

void motor_go_to(float degrees)
{
    float now = door_get_status().position_deg;
    door_move_t m = door_move_defaults(degrees, degrees > now ? s_open_rpm : s_close_rpm);
    m.absolute        = true;
    m.current.limit_a = DOOR_CURRENT_LIMIT_A;
    door_rotate_start(&m);                       /* held: back to where it started */
}

void motor_home(void)
{
    door_home();
}

void motor_latch_hold_unlocked(bool hold)
{
    door_latch_hold_unlocked(hold);
}

bool motor_read_supply(float *volts, float *amps)
{
    if (!s_ina228 || ina228_read_bus_voltage(s_ina228, volts) != ESP_OK) {
        return false;
    }
    *amps = door_get_status().current_a;         /* sampled every 5 ms by the door */
    return true;
}

float motor_get_angle(void)
{
    return door_get_status().position_deg;
}

void motor_on_fault(void (*callback)(void))
{
    s_fault_callback = callback;
}

void motor_on_door_held(void (*callback)(bool now_open))
{
    s_held_callback = callback;
}
