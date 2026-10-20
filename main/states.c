/*
 * states.c — The robot states. See states.h.
 */
#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "config.h"
#include "face.h"
#include "led.h"
#include "motor.h"
#include "sound.h"
#include "states.h"
#include "tail_light.h"

static const char *TAG = "state";

#define PULSE_S          1.0f    /* fade-in (and fade-out) time of the looks */
#define CHARGING_LAP_MS  2000    /* one lap of the charging comet */
#define ANIM_FRAME_MS    10

static const char *const NAMES[STATE_COUNT] = {
    [STATE_STARTUP]              = "startup",
    [STATE_MOVING_WITH_TASK]     = "moving_with_task",
    [STATE_MOVING_WITHOUT_TASK]  = "moving_without_task",
    [STATE_SUCCESS_WAITING]      = "success_waiting",
    [STATE_SUCCESS_OPEN_DOOR]    = "success_open_door",
    [STATE_SUCCESS_CLOSE_DOOR]   = "success_close_door",
    [STATE_FAILURE]              = "failure",
    [STATE_FAILURE_NOT_CRITICAL] = "failure_not_critical",
    [STATE_CHARGING]             = "charging",
};

static QueueHandle_t          s_requests;     /* length 1: the newest request wins */
static volatile robot_state_t s_state = STATE_STARTUP;

/* The door moves only if STATES_MOVE_DOOR is 1 (config.h). */
static void door_open(void)  { if (STATES_MOVE_DOOR) motor_open_door();  }
static void door_close(void) { if (STATES_MOVE_DOOR) motor_close_door(); }
static void door_stop(void)  { if (STATES_MOVE_DOOR) motor_stop();       }

/* Sounds play only if STATE_SOUNDS is 1 (config.h). */
static void say(sound_t s)   { if (STATE_SOUNDS) sound_play(s); }

static void apply(robot_state_t s)
{
    face_set_emotion(s == STATE_STARTUP ? FACE_THINKING : FACE_CONSTANT, PULSE_S);
    tail_lights_off_all();

    switch (s) {
    case STATE_STARTUP:
        led_set(LED_PULSE, RGB_WHITE, PULSE_S);
        say(SOUND_STARTUP);
        break;
    case STATE_MOVING_WITH_TASK:
        led_set(LED_PULSE, RGB_BLUE, PULSE_S);
        door_close();                       /* never drive around with the door open */
        break;
    case STATE_MOVING_WITHOUT_TASK:
        led_set(LED_PULSE, RGB_WHITE, PULSE_S);
        door_close();
        break;
    case STATE_SUCCESS_WAITING:
        led_set(LED_PULSE, RGB_GREEN, PULSE_S);
        say(SOUND_ARRIVED);
        break;
    case STATE_SUCCESS_OPEN_DOOR:
        led_set(LED_SOLID, RGB_GREEN, PULSE_S);
        say(SOUND_GREETING);
        door_open();
        break;
    case STATE_SUCCESS_CLOSE_DOOR:          /* package taken: close + lock */
        led_set(LED_SOLID, RGB_GREEN, PULSE_S);
        door_close();
        break;
    case STATE_FAILURE:
        led_set(LED_PULSE, RGB_RED, PULSE_S);
        tail_lights_set_brake(true);
        door_stop();                        /* critical failure: freeze the door */
        say(SOUND_FAILURE);
        break;
    case STATE_FAILURE_NOT_CRITICAL:
        led_set(LED_PULSE, RGB_YELLOW, PULSE_S);
        say(SOUND_WARNING);
        break;
    case STATE_CHARGING:                    /* drawn frame by frame: see below */
    default:
        break;
    }
}

/*
 * Charging: a cyan comet laps the strip; the face fades in during the first
 * half of each lap and out during the second, from the same clock. Runs until
 * the next request arrives (returned).
 */
static robot_state_t charging_until_next(void)
{
    int64_t       start = esp_timer_get_time();
    robot_state_t next;

    while (xQueueReceive(s_requests, &next, pdMS_TO_TICKS(ANIM_FRAME_MS)) != pdTRUE) {
        float elapsed  = (float)(esp_timer_get_time() - start) / 1000.0f;
        float progress = fmodf(elapsed / (float)CHARGING_LAP_MS, 1.0f);
        leds_comet_at(RGB_CYAN, progress);
        face_set_level(progress < 0.5f ? progress * 2.0f : (1.0f - progress) * 2.0f);
    }
    return next;
}

static void state_task(void *arg)
{
    robot_state_t s;
    xQueueReceive(s_requests, &s, portMAX_DELAY);
    for (;;) {
        s_state = s;
        ESP_LOGI(TAG, "-> %s", NAMES[s]);
        apply(s);
        if (s == STATE_CHARGING) {
            s = charging_until_next();
        } else {
            xQueueReceive(s_requests, &s, portMAX_DELAY);
        }
    }
}

void states_init(void)
{
    s_requests = xQueueCreate(1, sizeof(robot_state_t));
    xTaskCreate(state_task, "states", 4096, NULL, 5, NULL);
}

void states_set(robot_state_t s)
{
    if (s < STATE_COUNT && s_requests) {
        s_state = s;
        xQueueOverwrite(s_requests, &s);
    }
}

robot_state_t states_get(void)
{
    return s_state;
}

const char *state_name(robot_state_t s)
{
    return s < STATE_COUNT ? NAMES[s] : "?";
}

bool state_from_name(const char *name, robot_state_t *out)
{
    for (int s = 0; s < STATE_COUNT; s++) {
        if (strcmp(name, NAMES[s]) == 0) {
            *out = (robot_state_t)s;
            return true;
        }
    }
    return false;
}
