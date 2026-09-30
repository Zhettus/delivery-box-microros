/*
 * motor_test.c — Door motor test for the brushless (BLDC) motor with its own
 * built-in driver. Same logic as the old ZK-BM1 + encoder program:
 *
 *   set_motor_speed_rpm(+5) ... 5 s ... stop ... 1 s ... (-5) ... 5 s ... stop
 *   PI speed control with feed-forward, stall detection.
 *
 * What is different for this motor:
 *   - No H-bridge: the ESP32 drives the motor's control wires directly:
 *       SPEED  PWM duty = speed
 *       DIR    level  = rotation direction
 *       BRAKE  level  = brake on / off
 *   - No quadrature encoder: the yellow FG wire gives pulses WITHOUT
 *     direction. The direction is the one we command on DIR.
 *   - The direction is only changed while the motor stands still.
 *
 * Wiring (motor red/black straight to the 12 V supply, black also to ESP32 GND):
 *   yellow -> GPIO38   (FG pulses)
 *   blue   -> GPIO40   \
 *   green  -> GPIO18    > SPEED, DIR, BRAKE in an order found by the wire check
 *   white  -> GPIO21   /
 *
 * WIRE CHECK (WIRE_CHECK 1): at start-up the program finds out which of
 * blue/green/white is SPEED, DIR and BRAKE, and at which level. Run it
 * with the motor NOT attached to the door: it spins at full speed for
 * about 1 s at a time. It prints the result as #define lines; paste them
 * below and set WIRE_CHECK 0 to skip the check from then on.
 *
 * Build + run:
 *     idf.py -DMOTOR_TEST=1 -p /dev/ttyACM0 build flash monitor
 * Back to the normal firmware:
 *     idf.py -DMOTOR_TEST=0 -p /dev/ttyACM0 build flash monitor
 * Leave the monitor with Ctrl+]  (the motor keeps running: switch its supply off).
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ######################################################################## */
/* ##########  SETTINGS  ################################################## */
/* ######################################################################## */

/* ---- Wiring ------------------------------------------------------------ */
#define PIN_FG          GPIO_NUM_38    /* yellow: FG speed pulses              */
#define PIN_BLUE        GPIO_NUM_40
#define PIN_GREEN       GPIO_NUM_18
#define PIN_WHITE       GPIO_NUM_21

/* 1 = find SPEED/DIR/BRAKE at every start-up (motor NOT on the door).
 * 0 = use the fixed values below (paste them from the wire check result). */
#define WIRE_CHECK      0

/* Found by the wire check (runs only with green HIGH + blue LOW):          */
#define PIN_SPEED       GPIO_NUM_18    /* green                                */
#define SPEED_INVERTED  0              /* 1 = motor runs faster with LESS duty */
#define PIN_DIR         GPIO_NUM_21    /* white                                */
#define PIN_BRAKE       GPIO_NUM_40    /* blue; GPIO_NUM_NC if there is none   */
#define BRAKE_ON_LEVEL  1              /* level that brakes                    */

#define DIR_LEVEL_FORWARD 1            /* DIR level for positive rpm; flip it
                                          if "forward" turns the wrong way     */

/* ---- PWM ----------------------------------------------------------------- */
#define PWM_FREQ_HZ     20000          /* built-in BLDC drivers: 1..25 kHz     */
#define PWM_RES         LEDC_TIMER_10_BIT
#define PWM_FULL        1024           /* 2^10 = 100 % duty                    */
#define LEDC_MODE       LEDC_LOW_SPEED_MODE   /* the only mode on the ESP32-S3 */
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_CH_SPEED   LEDC_CHANNEL_0

/* ---- Motor (OUTPUT shaft) --------------------------------------------------- */
#define FG_PER_OUTPUT_REV     480.0f   /* FG pulses per output-shaft turn       */
#define OUTPUT_RPM_AT_MAX     48.0f    /* output rpm at 100 % duty (feed-forward) */
#define MIN_DUTY              0.10f    /* least duty that turns the motor       */

/* ---- Speed control ------------------------------------------------------------ */
#define CONTROL_DT_MS         20
#define SPEED_WINDOW_MS       200      /* speed = FG pulses over this window    */
#define KP                    0.010f   /* duty per output rpm of error          */
#define KI                    0.050f
#define INTEG_LIMIT           0.40f    /* duty                                  */

/* ---- Safety ----------------------------------------------------------------- */
#define STALL_DUTY            0.20f    /* stall = at least this duty ...        */
#define STALL_RPM             0.5f     /* ... but slower than this ...          */
#define STALL_MS              600      /* ... for this long -> stop             */
#define STANDSTILL_MS         200      /* no FG pulse this long = stopped       */
#define STOP_BEFORE_REVERSE_MS 300     /* minimum stop before changing DIR      */

/* ######################################################################## */

#define WINDOW_N  (SPEED_WINDOW_MS / CONTROL_DT_MS)

/* The motor's control wires, as found by the wire check (or the settings). */
static gpio_num_t g_pin_speed    = PIN_SPEED;
static bool       g_speed_inv    = SPEED_INVERTED;
static gpio_num_t g_pin_dir      = PIN_DIR;
static gpio_num_t g_pin_brake    = PIN_BRAKE;
static int        g_brake_on_lvl = BRAKE_ON_LEVEL;

static volatile float g_target_output_rpm = 0.0f;
static volatile bool  g_drive_enabled     = false;
static volatile bool  g_stalled           = false;

/* for printing */
static volatile float   g_meas_rpm, g_duty;
static volatile int     g_dir;           /* -1, 0, +1 actually driven */
static volatile int32_t g_position;      /* FG pulses, signed          */

static pcnt_unit_handle_t g_pcnt = NULL;

/* ---- FG pulse counter --------------------------------------------------- */

static void fg_init(void)
{
    pcnt_unit_config_t ucfg = { .high_limit = 30000, .low_limit = -30000 };
    pcnt_new_unit(&ucfg, &g_pcnt);
    pcnt_glitch_filter_config_t fcfg = { .max_glitch_ns = 1000 };
    pcnt_unit_set_glitch_filter(g_pcnt, &fcfg);

    /* count rising edges only: FG has no direction */
    pcnt_chan_config_t ccfg = { .edge_gpio_num = PIN_FG, .level_gpio_num = -1 };
    pcnt_channel_handle_t ch = NULL;
    pcnt_new_channel(g_pcnt, &ccfg, &ch);
    pcnt_channel_set_edge_action(ch, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                     PCNT_CHANNEL_EDGE_ACTION_HOLD);
    gpio_set_pull_mode(PIN_FG, GPIO_PULLUP_ONLY);   /* FG is open-collector */

    pcnt_unit_enable(g_pcnt);
    pcnt_unit_clear_count(g_pcnt);
    pcnt_unit_start(g_pcnt);
}

/* Pulses since the last call. */
static int fg_take(void)
{
    int count = 0;
    pcnt_unit_get_count(g_pcnt, &count);
    pcnt_unit_clear_count(g_pcnt);
    return count;
}

/* ---- motor outputs ------------------------------------------------------------ */

static void pin_out(gpio_num_t pin, int level)
{
    if (pin == GPIO_NUM_NC) {
        return;
    }
    gpio_set_level(pin, level);
    gpio_config_t io = { .pin_bit_mask = 1ULL << pin, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&io);                 /* also detaches a previous PWM signal */
    gpio_set_level(pin, level);
}

static void pwm_timer_init(void)
{
    ledc_timer_config_t tcfg = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = PWM_RES,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&tcfg);
}

/* Put the LEDC PWM on `pin` with `duty` (0..1, raw, no inversion). */
static void pwm_attach(gpio_num_t pin, float duty)
{
    ledc_channel_config_t ccfg = {
        .gpio_num   = pin,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CH_SPEED,
        .timer_sel  = LEDC_TIMER,
        .duty       = (uint32_t)(duty * PWM_FULL + 0.5f),
        .hpoint     = 0,
        .intr_type  = LEDC_INTR_DISABLE,
    };
    ledc_channel_config(&ccfg);
}

/* speed 0..1 (0 = stop), taking an inverted speed input into account */
static void speed_output(float speed)
{
    if (speed < 0.0f) speed = 0.0f;
    if (speed > 1.0f) speed = 1.0f;
    float duty = g_speed_inv ? 1.0f - speed : speed;
    ledc_set_duty(LEDC_MODE, LEDC_CH_SPEED, (uint32_t)(duty * PWM_FULL + 0.5f));
    ledc_update_duty(LEDC_MODE, LEDC_CH_SPEED);
}

static void brake(bool on)
{
    if (g_pin_brake != GPIO_NUM_NC) {
        gpio_set_level(g_pin_brake, on ? g_brake_on_lvl : !g_brake_on_lvl);
    }
}

static void set_dir(int sign)
{
    gpio_set_level(g_pin_dir, sign > 0 ? DIR_LEVEL_FORWARD : !DIR_LEVEL_FORWARD);
}

/* like zk_coast(): no drive, brake on */
static void motor_stop(void)
{
    speed_output(0.0f);
    brake(true);
}

/* ---- wire check ---------------------------------------------------------------------- */

static const gpio_num_t WIRE_PIN[3]  = { PIN_BLUE, PIN_GREEN, PIN_WHITE };
static const char      *WIRE_NAME[3] = { "blue", "green", "white" };

static void wc_levels(int combo)                  /* bit i = level of WIRE_PIN[i] */
{
    ledc_stop(LEDC_MODE, LEDC_CH_SPEED, 0);
    for (int i = 0; i < 3; i++) {
        pin_out(WIRE_PIN[i], (combo >> i) & 1);
    }
}

static float wc_measure_hz(uint32_t settle_ms, uint32_t meas_ms)
{
    vTaskDelay(pdMS_TO_TICKS(settle_ms));
    fg_take();
    vTaskDelay(pdMS_TO_TICKS(meas_ms));
    return fg_take() * 1000.0f / meas_ms;
}

/* back to the resting combination and wait until FG stops */
static void wc_rest(int rest)
{
    wc_levels(rest);
    for (int i = 0; i < 30; i++) {                /* up to 3 s */
        vTaskDelay(pdMS_TO_TICKS(100));
        if (fg_take() == 0) {
            vTaskDelay(pdMS_TO_TICKS(STOP_BEFORE_REVERSE_MS));
            return;
        }
    }
    printf("   (motor did not stop within 3 s)\n");
}

static float wc_try_combo(int combo, int rest)
{
    wc_levels(combo);
    float hz = wc_measure_hz(300, 500);
    wc_rest(rest);
    return hz;
}

/* PWM `effort` on WIRE_PIN[w], the other wires at `combo` levels. */
static float wc_try_pwm(int w, int combo, bool inverted, float effort, int rest)
{
    wc_levels(combo);
    pwm_attach(WIRE_PIN[w], inverted ? 1.0f - effort : effort);
    float hz = wc_measure_hz(500, 500);
    wc_rest(rest);
    return hz;
}

static bool wire_check(void)
{
    printf("\n=== WIRE CHECK: motor must NOT be on the door. It will run at full "
           "speed for ~1 s at a time. ===\n");
    const float RUN_HZ = 5.0f;
    float hz[8];
    bool  known[8] = { 0 };

    /* find a combination in which the motor rests */
    int rest = -1;
    for (int c = 0; c < 8 && rest < 0; c++) {
        wc_levels(c);
        hz[c] = wc_measure_hz(300, 500);
        known[c] = true;
        if (hz[c] < RUN_HZ) {
            rest = c;
        }
    }
    if (rest < 0) {
        wc_levels(0);
        printf("RESULT: the motor runs with EVERY combination: check the wiring.\n");
        return false;
    }
    wc_rest(rest);

    printf("  blue(40) green(18) white(21) -> FG Hz\n");
    for (int c = 0; c < 8; c++) {
        if (!known[c]) {
            hz[c] = wc_try_combo(c, rest);
        }
        printf("     %d         %d         %d     -> %6.1f %s\n",
               c & 1, (c >> 1) & 1, (c >> 2) & 1, hz[c], hz[c] >= RUN_HZ ? "RUNS" : "");
    }

    int runs = 0, run_combo = -1;
    float max_hz = 0.0f;
    for (int c = 0; c < 8; c++) {
        if (hz[c] >= RUN_HZ) {
            runs++;
            if (hz[c] > max_hz) { max_hz = hz[c]; run_combo = c; }
        }
    }
    if (runs == 0) {
        printf("RESULT: the motor never turned. Check: red/black power, black to ESP32 "
               "GND, yellow on GPIO38, 5 V logic on the control wires.\n");
        return false;
    }

    /* A wire whose level never matters for running is DIR (or unused). */
    bool relevant[3];
    int  n_rel = 0;
    for (int w = 0; w < 3; w++) {
        relevant[w] = false;
        for (int c = 0; c < 8; c++) {
            bool r1 = hz[c] >= RUN_HZ, r2 = hz[c ^ (1 << w)] >= RUN_HZ;
            if (r1 != r2) relevant[w] = true;
        }
        n_rel += relevant[w];
    }

    int speed_w = -1, brake_w = -1, dir_w = -1;
    if (n_rel == 2) {
        int a = -1, b = -1;
        for (int w = 0; w < 3; w++) {
            if (!relevant[w])   dir_w = w;
            else if (a < 0)     a = w;
            else                b = w;
        }
        /* the one whose PWM changes the speed smoothly is SPEED */
        float score[2];
        int   cand[2] = { a, b };
        for (int k = 0; k < 2; k++) {
            int  w   = cand[k];
            bool inv = !((run_combo >> w) & 1);          /* runs when LOW */
            float f30 = wc_try_pwm(w, run_combo, inv, 0.30f, rest) / max_hz;
            float f70 = wc_try_pwm(w, run_combo, inv, 0.70f, rest) / max_hz;
            printf("  PWM on %-5s: 30%% -> %3.0f%% speed, 70%% -> %3.0f%% speed\n",
                   WIRE_NAME[w], f30 * 100.0f, f70 * 100.0f);
            score[k] = (f30 > 0.05f && f30 < 0.8f) ? f70 - f30 : -1.0f;
        }
        if (score[0] <= 0.1f && score[1] <= 0.1f) {
            printf("RESULT: neither wire controls the speed smoothly - send me this log.\n");
            return false;
        }
        /* Both respond alike when the motor needs BOTH wires in a state
         * (PWM on the brake also chops the drive). Then prefer the usual
         * arrangement: speed runs when HIGH, brake brakes when HIGH. */
        if (fabsf(score[0] - score[1]) < 0.1f) {
            bool a_high = (run_combo >> a) & 1;
            speed_w = a_high ? a : b;
            printf("  (both respond alike: taking the wire that runs when HIGH as SPEED)\n");
        } else {
            speed_w = score[0] > score[1] ? a : b;
        }
        brake_w = speed_w == a ? b : a;
    } else if (n_rel == 1) {
        for (int w = 0; w < 3; w++) {
            if (relevant[w]) speed_w = w;
            else if (dir_w < 0) dir_w = w;
        }
        printf("NOTE: only one wire matters for running; the brake did nothing. "
               "Assuming %s = DIR: watch that the motor changes direction.\n",
               WIRE_NAME[dir_w]);
    } else {
        printf("RESULT: unclear pattern - send me this log.\n");
        return false;
    }

    g_pin_speed = WIRE_PIN[speed_w];
    g_speed_inv = !((run_combo >> speed_w) & 1);
    g_pin_dir   = WIRE_PIN[dir_w];
    if (brake_w >= 0) {
        g_pin_brake    = WIRE_PIN[brake_w];
        g_brake_on_lvl = !((run_combo >> brake_w) & 1);  /* opposite of "runs" */
    } else {
        g_pin_brake = GPIO_NUM_NC;
    }

    printf("\n==== WIRE CHECK RESULT ====\n");
    printf("  SPEED (PWM) = %-5s on GPIO%d%s\n", WIRE_NAME[speed_w], g_pin_speed,
           g_speed_inv ? "  (inverted: runs when LOW)" : "");
    printf("  DIR         = %-5s on GPIO%d\n", WIRE_NAME[dir_w], g_pin_dir);
    if (brake_w >= 0) {
        printf("  BRAKE       = %-5s on GPIO%d  (brake on = %s)\n", WIRE_NAME[brake_w],
               g_pin_brake, g_brake_on_lvl ? "HIGH" : "LOW");
    }
    printf("  full speed  = %.0f Hz FG = %.1f output rpm\n", max_hz,
           max_hz * 60.0f / FG_PER_OUTPUT_REV);
    printf("Paste into motor_test.c and set WIRE_CHECK 0 to skip this check:\n");
    printf("  #define PIN_SPEED       GPIO_NUM_%d\n", g_pin_speed);
    printf("  #define SPEED_INVERTED  %d\n", g_speed_inv);
    printf("  #define PIN_DIR         GPIO_NUM_%d\n", g_pin_dir);
    if (brake_w >= 0) {
        printf("  #define PIN_BRAKE       GPIO_NUM_%d\n", g_pin_brake);
        printf("  #define BRAKE_ON_LEVEL  %d\n", g_brake_on_lvl);
    } else {
        printf("  #define PIN_BRAKE       GPIO_NUM_NC\n");
    }
    printf("===========================\n\n");
    return true;
}

/* ---- speed control ---------------------------------------------------------------- */

static void control_task(void *arg)
{
    const float dt = CONTROL_DT_MS / 1000.0f;
    float integral = 0.0f;
    int   window[WINDOW_N] = { 0 };
    int   wi = 0, win_sum = 0;
    int   dir = 0;                   /* direction currently set on DIR, 0 = stopped */
    int   stall_ms = 0, still_ms = 0, run_ms = 0;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_DT_MS));

        /* ---- measure: pulses over the last SPEED_WINDOW_MS -------------- */
        int count = fg_take();
        win_sum += count - window[wi];
        window[wi] = count;
        wi = (wi + 1) % WINDOW_N;
        float meas_rpm = (float)win_sum / FG_PER_OUTPUT_REV
                         / (SPEED_WINDOW_MS / 1000.0f) * 60.0f;
        g_position += dir >= 0 ? count : -count;   /* FG has no sign: use DIR */
        still_ms = win_sum == 0 ? still_ms + CONTROL_DT_MS : 0;

        float target = g_target_output_rpm;
        int   want   = !g_drive_enabled ? 0 : (target > 0.0f) - (target < 0.0f);
        float out    = 0.0f;

        if (want != dir) {
            /* stop first; change DIR only after a real standstill */
            motor_stop();
            integral = 0.0f;
            stall_ms = run_ms = 0;
            if (want != 0 && still_ms >= STANDSTILL_MS &&
                still_ms >= STOP_BEFORE_REVERSE_MS) {
                set_dir(want);
                brake(false);
                dir = want;
            } else if (want == 0 && still_ms >= STANDSTILL_MS) {
                dir = 0;
            }
        } else if (dir != 0) {
            /* ---- PI with feed-forward (as the old program) --------------- */
            float target_abs = fabsf(target);
            float err = target_abs - meas_rpm;
            float ff  = target_abs / OUTPUT_RPM_AT_MAX;

            integral += KI * err * dt;
            if (integral >  INTEG_LIMIT) integral =  INTEG_LIMIT;
            if (integral < -INTEG_LIMIT) integral = -INTEG_LIMIT;

            out = ff + KP * err + integral;
            if (out > 1.0f) out = 1.0f;
            if (out < 0.0f) out = 0.0f;
            if (out > 0.001f && out < MIN_DUTY) out = MIN_DUTY;

            /* ---- stall detection: pushing hard but not moving ------------ */
            run_ms += CONTROL_DT_MS;
            if (run_ms > SPEED_WINDOW_MS && out > STALL_DUTY && meas_rpm < STALL_RPM) {
                stall_ms += CONTROL_DT_MS;
                if (stall_ms >= STALL_MS) {
                    g_drive_enabled     = false;     /* obstacle: shut down */
                    g_target_output_rpm = 0.0f;
                    g_stalled           = true;
                    out = 0.0f;
                }
            } else {
                stall_ms = 0;
            }
            speed_output(out);
        } else {
            motor_stop();
            integral = 0.0f;
        }

        g_meas_rpm = dir < 0 ? -meas_rpm : meas_rpm;
        g_duty     = out;
        g_dir      = dir;
    }
}

/* ---- the same functions as the old program ------------------------------------------- */

static void stopMotor(void)
{
    g_target_output_rpm = 0.0f;
    g_drive_enabled = false;
}

static void set_motor_speed_rpm(float target_rpm)
{
    if (target_rpm == 0.0f) {
        stopMotor();
    } else {
        g_stalled = false;
        g_target_output_rpm = target_rpm;
        g_drive_enabled = true;
    }
}

static void moveForward(float seconds, float outputRpm)
{
    set_motor_speed_rpm(+outputRpm);
    vTaskDelay(pdMS_TO_TICKS((uint32_t)(seconds * 1000.0f)));
    stopMotor();
}

static void moveBackward(float seconds, float outputRpm)
{
    set_motor_speed_rpm(-outputRpm);
    vTaskDelay(pdMS_TO_TICKS((uint32_t)(seconds * 1000.0f)));
    stopMotor();
}

/* ---- start-up + status print ---------------------------------------------------------- */

static void print_task(void *arg)
{
    bool reported = false;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(250));
        printf("target %+5.1f rpm | speed %+5.1f rpm | duty %3.0f%% | dir %+d | "
               "pos %+7.1f deg\n",
               g_target_output_rpm, g_meas_rpm, g_duty * 100.0f, g_dir,
               g_position * 360.0f / FG_PER_OUTPUT_REV);
        if (g_stalled && !reported) {
            printf("!!! STALL: the motor pushed but did not move - stopped\n");
        }
        reported = g_stalled;
    }
}

static bool drive_init(void)
{
    /* All three control wires LOW first, so nothing floats. */
    for (int i = 0; i < 3; i++) {
        pin_out(WIRE_PIN[i], 0);
    }
    fg_init();
    pwm_timer_init();

    if (WIRE_CHECK && !wire_check()) {
        return false;
    }

    /* final wiring: DIR + BRAKE plain outputs, PWM on SPEED, motor stopped */
    pin_out(g_pin_dir, !DIR_LEVEL_FORWARD);
    if (g_pin_brake != GPIO_NUM_NC) {
        pin_out(g_pin_brake, g_brake_on_lvl);
    }
    pwm_attach(g_pin_speed, g_speed_inv ? 1.0f : 0.0f);
    motor_stop();
    fg_take();

    /* Safety: stopped + braked, the motor must not turn now. */
    vTaskDelay(pdMS_TO_TICKS(1000));
    int pulses = fg_take();
    if (pulses > 5) {
        printf("!!! MOTOR TURNS WHILE STOPPED (%d FG pulses in 1 s): the SPEED/BRAKE "
               "settings are wrong. Switch the motor supply OFF. Test halted.\n", pulses);
        return false;
    }

    xTaskCreate(control_task, "motor_ctl", 4096, NULL, 10, NULL);
    xTaskCreate(print_task, "motor_print", 3072, NULL, 3, NULL);
    return true;
}

static void test_task(void *arg)
{
    if (!drive_init()) {
        for (int i = 0; i < 3; i++) {
            pin_out(WIRE_PIN[i], 0);
        }
        printf("motor test stopped.\n");
        vTaskDelete(NULL);
    }
    printf("motor test: +5 rpm 5 s, stop 1 s, -5 rpm 5 s, stop 1 s, repeat\n");

    for (;;) {
        set_motor_speed_rpm(5.0f);
        vTaskDelay(pdMS_TO_TICKS(5000));
        set_motor_speed_rpm(0.0f);
        vTaskDelay(pdMS_TO_TICKS(1000));

        set_motor_speed_rpm(-5.0f);
        vTaskDelay(pdMS_TO_TICKS(5000));
        set_motor_speed_rpm(0.0f);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* also available, as in the old program: */
    moveForward(5.0f, 5.0f);
    moveBackward(5.0f, 5.0f);
}

void app_main(void)
{
    esp_log_level_set("gpio", ESP_LOG_WARN);      /* no line per pin change */
    /* own task: the default main task stack is too small for this printing */
    xTaskCreate(test_task, "motor_test", 8192, NULL, 5, NULL);
}
