/*
 * commands.c — Serial command interface. See commands.h.
 *
 * To add a command: write a `static void cmd_xxx(int argc, char **argv)`
 * and add a row to COMMANDS[] at the bottom.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_vfs_dev.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "app.h"
#include "app_config.h"
#include "audio.h"
#include "commands.h"
#include "demo.h"
#include "door.h"
#include "eyes.h"
#include "led_strips.h"
#include "power_monitor.h"
#include "robot_state.h"
#include "tail_lights.h"

static const char *TAG = "cmd";

#define CMD_LINE_MAX       128
#define CMD_ARGS_MAX       8

static SemaphoreHandle_t s_print_lock;

/* ---- output ---------------------------------------------------------------- */

static void print_line(const char *prefix, const char *fmt, va_list ap)
{
    if (s_print_lock) {
        xSemaphoreTake(s_print_lock, portMAX_DELAY);
    }
    fputs(prefix, stdout);
    vprintf(fmt, ap);
    fputc('\n', stdout);
    fflush(stdout);
    if (s_print_lock) {
        xSemaphoreGive(s_print_lock);
    }
}

static void __attribute__((format(printf, 1, 2))) ok(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    print_line("OK ", fmt, ap);
    va_end(ap);
}

static void __attribute__((format(printf, 1, 2))) err(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    print_line("ERR ", fmt, ap);
    va_end(ap);
}

void app_event(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    print_line("EVT ", fmt, ap);
    va_end(ap);
}

/* ---- argument parsing -------------------------------------------------------- */

static bool parse_long(const char *s, long lo, long hi, long *out)
{
    char *end;
    long v = strtol(s, &end, 0);
    if (*s == '\0' || *end != '\0' || v < lo || v > hi) {
        return false;
    }
    *out = v;
    return true;
}

static bool parse_float(const char *s, float lo, float hi, float *out)
{
    char *end;
    float v = strtof(s, &end);
    if (*s == '\0' || *end != '\0' || !(v >= lo && v <= hi)) {
        return false;
    }
    *out = v;
    return true;
}

/* ---- commands ------------------------------------------------------------------ */

static void cmd_help(int argc, char **argv);

static void cmd_status(int argc, char **argv)
{
    door_status_t d = door_get_status();
    ina228_reading_t p;
    bool have_power = g_app.power_ok && power_monitor_latest(&p, NULL);

    char power[40] = "missing";
    if (have_power) {
        snprintf(power, sizeof(power), "%.2fV/%.2fA", p.bus_voltage_v, p.current_a);
    }

    ok("state=%s door=%s latch=%s fault=%s pos=%.1fdeg leds=%s eyes=%s tail=%s mic=%s "
       "speaker=%s power=%s",
       robot_state_name(robot_state_get()),
       door_state_name(d.state), door_lock_state_name(d.lock), door_fault_name(d.fault),
       d.position_deg, g_app.leds_ok ? "ok" : "missing", g_app.eyes_ok ? "ok" : "missing",
       g_app.tail_ok ? "ok" : "missing", g_app.audio_ok ? "ok" : "missing",
       g_app.speaker_ok ? "ok" : "missing", power);
}

static void cmd_states(int argc, char **argv)
{
    char buf[200] = "";
    for (int s = 0; s < ROBOT_STATE_COUNT; s++) {
        strlcat(buf, robot_state_name((robot_state_t)s), sizeof(buf));
        strlcat(buf, " ", sizeof(buf));
    }
    ok("%s", buf);
}

static void cmd_state(int argc, char **argv)
{
    if (argc == 1) {
        ok("%s", robot_state_name(robot_state_get()));
        return;
    }
    robot_state_t s;
    if (!robot_state_from_name(argv[1], &s)) {
        err("unknown state '%s' (try: states)", argv[1]);
        return;
    }
    robot_state_set(s);
    ok("state %s", robot_state_name(s));
}

/* next / prev: step through the states in list order (see `states`). */
static void step_state(int dir)
{
    int n = ROBOT_STATE_COUNT;
    robot_state_t s = (robot_state_t)(((int)robot_state_get() + dir + n) % n);
    robot_state_set(s);
    ok("state %s (%d/%d)", robot_state_name(s), (int)s + 1, n);
}

static void cmd_next(int argc, char **argv) { step_state(+1); }
static void cmd_prev(int argc, char **argv) { step_state(-1); }

static void cmd_demo(int argc, char **argv)
{
    if (argc >= 2 && strcmp(argv[1], "off") == 0) {
        demo_stop();
        ok("demo off");
        return;
    }
    if (argc < 2 || strcmp(argv[1], "on") != 0) {
        ok("demo %s (usage: demo on [seconds] [door] | demo off)",
           demo_running() ? "running" : "off");
        return;
    }
    float secs = 5.0f;
    bool  door = false;
    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "door") == 0) {
            door = true;
        } else if (!parse_float(argv[i], 0.5f, 600.0f, &secs)) {
            err("usage: demo on [seconds 0.5-600] [door]");
            return;
        }
    }
    if (demo_start((uint32_t)(secs * 1000.0f), door) != ESP_OK) {
        err("demo already running (demo off first)");
        return;
    }
    ok("demo on: every %.1f s, door %s", secs, door ? "MOVES" : "stays still");
}

static void cmd_door(int argc, char **argv)
{
    const char *sub = argc > 1 ? argv[1] : "status";

    if (!g_app.door_ok && strcmp(sub, "status") != 0) {
        err("door driver not initialised");
        return;
    }
    if (strcmp(sub, "rotate") == 0) {
        float deg, rpm, limit = APP_DOOR_CURRENT_LIMIT_A;
        if (argc < 4 || !parse_float(argv[2], -3600.0f, 3600.0f, &deg) ||
            !parse_float(argv[3], 0.5f, 60.0f, &rpm) ||
            (argc > 4 && !parse_float(argv[4], 0.0f, 20.0f, &limit))) {
            err("usage: door rotate <degrees (+open/-close)> <rpm> [current_limit_A]");
            return;
        }
        door_move_t m = door_move_defaults(deg, rpm);
        m.current.limit_a = limit;
        if (door_rotate_start(&m) != ESP_OK) {
            err("door rotate failed");
            return;
        }
        ok("door rotate %.1f deg at %.1f rpm, limit %.2f A (EVT door ... when done)",
           deg, rpm, limit);
        return;
    }
    if      (strcmp(sub, "open") == 0)
        door_move_to_start(APP_DOOR_OPEN_DEG, APP_DOOR_OPEN_RPM, APP_DOOR_OPEN_LIMIT_A);
    else if (strcmp(sub, "close") == 0)
        door_move_to_start(0.0f, APP_DOOR_CLOSE_RPM, APP_DOOR_CLOSE_LIMIT_A);
    else if (strcmp(sub, "stop") == 0)  door_stop();
    else if (strcmp(sub, "home") == 0)  door_home();
    else if (strcmp(sub, "unlock") == 0) door_latch_hold_unlocked(true);   /* keep latch in */
    else if (strcmp(sub, "lock") == 0)   door_latch_hold_unlocked(false);  /* automatic    */
    else if (strcmp(sub, "status") != 0) {
        err("usage: door open|close|stop|home|lock|unlock|status|rotate <deg> <rpm> [limit_A]");
        return;
    }

    if (strcmp(sub, "status") == 0) {
        door_status_t d = door_get_status();
        ok("door %s result=%s fault=%s pos=%.1fdeg speed=%.2frpm duty=%.0f%% dir=%d "
           "retries=%u current=%.3fA normal=%.3fA peak=%.3fA latch=%s solenoid=%.0f%% "
           "auto_close_in=%.1fs",
           door_state_name(d.state), door_result_name(d.result), door_fault_name(d.fault),
           d.position_deg, d.speed_rpm, d.duty * 100.0f, d.direction, d.retries,
           d.current_a, d.normal_a, d.peak_a, door_lock_state_name(d.lock),
           d.solenoid * 100.0f, d.auto_close_in_ms / 1000.0f);
    } else {
        ok("door %s", sub);
    }
}

static void cmd_leds(int argc, char **argv)
{
    if (argc < 2) {
        err("usage: leds <effect> [r g b] [time_s] | leds max <0-255>");
        return;
    }
    if (strcmp(argv[1], "max") == 0) {
        long v;
        if (argc != 3 || !parse_long(argv[2], 0, 255, &v)) {
            err("usage: leds max <0-255>");
            return;
        }
        led_strips_set_max_brightness((uint8_t)v);
        ok("leds max %ld", v);
        return;
    }

    led_effect_t effect;
    if (!led_effect_from_name(argv[1], &effect)) {
        err("effects: off solid pulse blink comet fill diverge identify");
        return;
    }
    rgb_t color = RGB_WHITE;
    float time_s = 1.0f;
    int   next = 2;
    if (argc >= 5) {
        long r, g, b;
        if (!parse_long(argv[2], 0, 255, &r) || !parse_long(argv[3], 0, 255, &g) ||
            !parse_long(argv[4], 0, 255, &b)) {
            err("colour must be three numbers 0-255");
            return;
        }
        color = LED_RGB(r, g, b);
        next  = 5;
    }
    if (argc > next && !parse_float(argv[next], 0.01f, 600.0f, &time_s)) {
        err("time_s must be 0.01..600");
        return;
    }
    led_strips_set(effect, color, time_s);
    ok("leds %s %u %u %u %.2f", led_effect_name(effect), color.r, color.g, color.b, time_s);
}

static void cmd_eyes(int argc, char **argv)
{
    if (argc < 2) {
        err("usage: eyes off|thinking|constant [fade_s] | eyes cap <0-1>");
        return;
    }
    if (strcmp(argv[1], "cap") == 0) {
        float cap;
        if (argc != 3 || !parse_float(argv[2], 0.0f, 1.0f, &cap)) {
            err("usage: eyes cap <0.0-1.0>");
            return;
        }
        eyes_set_brightness_cap(cap);
        ok("eyes cap %.3f", cap);
        return;
    }
    eye_emotion_t e;
    float fade = 1.0f;
    if (!eye_emotion_from_name(argv[1], &e)) {
        err("emotions: off thinking constant");
        return;
    }
    if (argc > 2 && !parse_float(argv[2], 0.01f, 60.0f, &fade)) {
        err("fade_s must be 0.01..60");
        return;
    }
    eyes_set_emotion(e, fade);
    ok("eyes %s %.2f", eye_emotion_name(e), fade);
}

static void cmd_tail(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "off") == 0) {
        tail_lights_off();
        ok("tail off");
        return;
    }
    tail_light_t which;
    long level, fade_ms = 0;
    if (argc < 3 || !tail_light_from_name(argv[1], &which) ||
        !parse_long(argv[2], 0, 255, &level) ||
        (argc > 3 && !parse_long(argv[3], 0, 60000, &fade_ms))) {
        err("usage: tail brake|run|left|right <0-255> [fade_ms] | tail off");
        return;
    }
    tail_lights_fade(which, (uint8_t)level, (uint32_t)fade_ms);
    ok("tail %s %ld", tail_light_name(which), level);
}

static void cmd_volume(int argc, char **argv)
{
    if (!audio_speaker_available()) {
        err("speaker (WM8960) not available");
        return;
    }
    long v;
    if (argc == 1) {
        ok("volume %u", audio_get_volume());
        return;
    }
    if (!parse_long(argv[1], 0, 100, &v) || audio_set_volume((uint8_t)v) != ESP_OK) {
        err("usage: volume <0-100>");
        return;
    }
    ok("volume %ld", v);
}

static void cmd_beep(int argc, char **argv)
{
    long freq = 1000, ms = 200;
    if ((argc > 1 && !parse_long(argv[1], 20, 20000, &freq)) ||
        (argc > 2 && !parse_long(argv[2], 1, 10000, &ms))) {
        err("usage: beep [freq_hz 20-20000] [ms 1-10000]");
        return;
    }
    if (audio_play_tone((uint16_t)freq, (uint16_t)ms) != ESP_OK) {
        err("speaker (WM8960) not available");
        return;
    }
    ok("beep %ld Hz %ld ms", freq, ms);
}

static void cmd_mic(int argc, char **argv)
{
    long ms = 500;
    if (!g_app.audio_ok) {
        err("I2S not initialised");
        return;
    }
    if (argc > 1 && !parse_long(argv[1], 10, 10000, &ms)) {
        err("usage: mic [window_ms 10-10000]");
        return;
    }
    float rms, peak;
    if (audio_mic_level((uint32_t)ms, &rms, &peak) != ESP_OK) {
        err("microphone read failed");
        return;
    }
    ok("mic rms=%.1fdBFS peak=%.1fdBFS", rms, peak);
}

static void cmd_power(int argc, char **argv)
{
    if (!g_app.power_ok) {
        err("INA228 not available");
        return;
    }
    if (argc > 1 && strcmp(argv[1], "reset") == 0) {
        ina228_reset_accumulators(g_app.ina228);
        ok("power accumulators reset");
        return;
    }
    ina228_reading_t r;
    uint32_t age;
    if (!power_monitor_latest(&r, &age)) {
        err("no reading yet");
        return;
    }
    ok("power %.3fV %.4fA %.3fW shunt=%.3fmV temp=%.1fC energy=%.4fWh charge=%.4fAh age=%lums",
       r.bus_voltage_v, r.current_a, r.power_w, r.shunt_voltage_mv, r.die_temp_c,
       r.energy_wh, r.charge_ah, (unsigned long)age);
}

static void cmd_i2c(int argc, char **argv)
{
    if (argc < 2 || strcmp(argv[1], "scan") != 0 || !g_app.i2c_bus) {
        err("usage: i2c scan");
        return;
    }
    char buf[120] = "";
    int found = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (i2c_master_probe(g_app.i2c_bus, a, 20) == ESP_OK) {
            char item[8];
            snprintf(item, sizeof(item), "0x%02X ", a);
            strlcat(buf, item, sizeof(buf));
            found++;
        }
    }
    ok("i2c %d device(s): %s(expect 0x1A WM8960, 0x40 INA228)", found, buf);
}

static void cmd_reboot(int argc, char **argv)
{
    ok("rebooting");
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
}

typedef struct {
    const char *name;
    const char *usage;
    void (*fn)(int argc, char **argv);
} command_t;

static const command_t COMMANDS[] = {
    { "help",   "                                      this list",           cmd_help   },
    { "status", "                                      one-line overview",    cmd_status },
    { "states", "                                      list robot states",    cmd_states },
    { "state",  "[name]                                get / set robot state", cmd_state },
    { "door",   "open|close|stop|home|status|lock|unlock | rotate <deg> <rpm> [limit_A]",  cmd_door   },
    { "leds",   "<effect> [r g b] [time_s] | max <0-255>",                      cmd_leds   },
    { "eyes",   "off|thinking|constant [fade_s] | cap <0-1>",                   cmd_eyes   },
    { "tail",   "brake|run|left|right <0-255> [fade_ms] | off",                 cmd_tail   },
    { "volume", "[0-100]",                                                      cmd_volume },
    { "beep",   "[freq_hz] [ms]",                                               cmd_beep   },
    { "mic",    "[window_ms]                           microphone level",     cmd_mic    },
    { "power",  "[reset]                               INA228 reading",       cmd_power  },
    { "i2c",    "scan",                                                         cmd_i2c    },
    { "next",   "                                      go to the next state",  cmd_next   },
    { "prev",   "                                      go to the previous state", cmd_prev },
    { "demo",   "on [seconds] [door] | off             cycle through states",  cmd_demo   },
    { "reboot", "",                                                             cmd_reboot },
};

static void cmd_help(int argc, char **argv)
{
    xSemaphoreTake(s_print_lock, portMAX_DELAY);
    printf("Commands:\n");
    for (size_t i = 0; i < sizeof(COMMANDS) / sizeof(COMMANDS[0]); i++) {
        printf("  %-7s %s\n", COMMANDS[i].name, COMMANDS[i].usage);
    }
    fflush(stdout);
    xSemaphoreGive(s_print_lock);
    ok("help");
}

/* ---- input loop -------------------------------------------------------------- */

static void execute(char *line)
{
    char *argv[CMD_ARGS_MAX];
    int   argc = 0;
    for (char *tok = strtok(line, " \t"); tok && argc < CMD_ARGS_MAX; tok = strtok(NULL, " \t")) {
        argv[argc++] = tok;
    }
    if (argc == 0) {
        return;
    }
    for (size_t i = 0; i < sizeof(COMMANDS) / sizeof(COMMANDS[0]); i++) {
        if (strcmp(argv[0], COMMANDS[i].name) == 0) {
            COMMANDS[i].fn(argc, argv);
            return;
        }
    }
    err("unknown command '%s' (try: help)", argv[0]);
}

/*
 * Console port: whichever one sdkconfig selects as the primary console.
 *   CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG -> the native "USB" port (/dev/ttyACM*)
 *   CONFIG_ESP_CONSOLE_UART            -> the "UART"/"COM" port (GPIO 43/44)
 */
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#define CONSOLE_NAME "native USB"

static esp_err_t console_install(void)
{
    usb_serial_jtag_driver_config_t cfg = { .rx_buffer_size = 512, .tx_buffer_size = 1024 };
    esp_err_t e = usb_serial_jtag_driver_install(&cfg);
    if (e == ESP_OK) {
        /* printf/logs go through the driver too, so they don't fight over the FIFO.
         * With no PC attached, output is dropped instead of blocking. */
        esp_vfs_usb_serial_jtag_use_driver();
    }
    return e;
}

static int console_read(uint8_t *buf, size_t len, TickType_t wait)
{
    return usb_serial_jtag_read_bytes(buf, len, wait);
}

static void console_write(const char *s, size_t n)
{
    usb_serial_jtag_write_bytes(s, n, pdMS_TO_TICKS(20));
}
#else
#define CONSOLE_NAME "UART"

static esp_err_t console_install(void)
{
    return uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 512, 0, 0, NULL, 0);
}

static int console_read(uint8_t *buf, size_t len, TickType_t wait)
{
    return uart_read_bytes(CONFIG_ESP_CONSOLE_UART_NUM, buf, len, wait);
}

static void console_write(const char *s, size_t n)
{
    uart_write_bytes(CONFIG_ESP_CONSOLE_UART_NUM, s, n);
}
#endif

static void echo(const char *s, size_t n)
{
#if APP_SERIAL_ECHO
    console_write(s, n);
#endif
}

static void command_task(void *arg)
{
    char    line[CMD_LINE_MAX];
    size_t  len = 0;
    uint8_t chunk[32];

    for (;;) {
        int n = console_read(chunk, sizeof(chunk), pdMS_TO_TICKS(50));
        for (int i = 0; i < n; i++) {
            char c = (char)chunk[i];
            if (c == '\r' || c == '\n') {
                if (len > 0) {
                    echo("\r\n", 2);
                    line[len] = '\0';
                    execute(line);
                    len = 0;
                }
            } else if (c == '\b' || c == 0x7F) {
                if (len > 0) {
                    len--;
                    echo("\b \b", 3);
                }
            } else if (len < CMD_LINE_MAX - 1 && c >= 0x20) {
                line[len++] = c;
                echo(&c, 1);
            }
        }
    }
}

esp_err_t commands_start(void)
{
    s_print_lock = xSemaphoreCreateMutex();
    if (!s_print_lock) {
        return ESP_ERR_NO_MEM;
    }

    /* Install the console driver so we can block on input. Logs keep working. */
    esp_err_t e = console_install();
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "%s console driver: %s", CONSOLE_NAME, esp_err_to_name(e));
        return e;
    }
    ESP_LOGI(TAG, "serial commands on the %s port", CONSOLE_NAME);
    if (xTaskCreate(command_task, "commands", 6144, NULL, 2, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
