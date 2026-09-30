#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "app_config.h"
#include "audio.h"
#include "commands.h"
#include "door.h"
#include "eyes.h"
#include "led_strips.h"
#include "robot_state.h"
#include "tail_lights.h"

static const char *TAG = "state";

#define ON  APP_TAIL_LEVEL_ON

typedef enum {
    DOOR_ACTION_NONE = 0,   /* leave the door as it is                    */
    DOOR_ACTION_OPEN,
    DOOR_ACTION_CLOSE,      /* never drive around with the door open      */
    DOOR_ACTION_STOP,       /* critical failure: freeze the door          */
} door_action_t;

/* What a state does with the door: go TO open / closed at a speed, with a
 * current limit (resistance -> back to where the move started, retry). */
typedef struct {
    door_action_t action;
    float         rpm;
    float         limit_a;
} door_plan_t;

#define DOOR_KEEP     { DOOR_ACTION_NONE, 0, 0 }
#define DOOR_OPEN     { DOOR_ACTION_OPEN,  APP_DOOR_OPEN_RPM,  APP_DOOR_OPEN_LIMIT_A  }
#define DOOR_CLOSE    { DOOR_ACTION_CLOSE, APP_DOOR_CLOSE_RPM, APP_DOOR_CLOSE_LIMIT_A }
#define DOOR_FREEZE   { DOOR_ACTION_STOP,  0, 0 }

typedef struct {
    const char           *name;
    eye_emotion_t         eyes;
    float                 eyes_time_s;   /* fade / half breathing period   */
    led_effect_t          leds;
    rgb_t                 color;
    float                 leds_time_s;   /* see led_effects.h              */
    tail_lights_pattern_t tail;
    door_plan_t           door;
    const audio_melody_t *sound;         /* NULL = silent                  */
} state_look_t;

/* ---- Sounds (notes in Hz, durations in ms) ------------------------------- */
static const audio_note_t NOTES_STARTUP[]  = { { 523, 120 }, { 659, 120 }, { 784, 200 } };
static const audio_note_t NOTES_ARRIVED[]  = { { 784, 150 }, { 1047, 250 } };
static const audio_note_t NOTES_OPEN[]     = { { 1047, 100 }, { 1319, 100 }, { 1568, 200 } };
static const audio_note_t NOTES_WARNING[]  = { { 660, 150 }, { 0, 100 }, { 660, 150 } };
static const audio_note_t NOTES_FAILURE[]  = { { 440, 300 }, { 330, 300 }, { 0, 150 },
                                               { 440, 300 }, { 330, 300 } };

static const audio_melody_t SOUND_STARTUP = AUDIO_MELODY(NOTES_STARTUP);
static const audio_melody_t SOUND_ARRIVED = AUDIO_MELODY(NOTES_ARRIVED);
static const audio_melody_t SOUND_OPEN    = AUDIO_MELODY(NOTES_OPEN);
static const audio_melody_t SOUND_WARNING = AUDIO_MELODY(NOTES_WARNING);
static const audio_melody_t SOUND_FAILURE = AUDIO_MELODY(NOTES_FAILURE);

/* ---- The state table ------------------------------------------------------ */
static const state_look_t STATES[ROBOT_STATE_COUNT] = {
    [ROBOT_STATE_STARTUP] = {
        .name = "startup",
        .eyes = EYE_THINKING,  .eyes_time_s = 1.0f,
        .leds = LED_EFFECT_PULSE, .color = RGB_WHITE, .leds_time_s = 1.0f,
        .tail = { 0 },
        .door = DOOR_KEEP,
        .sound = &SOUND_STARTUP,
    },
    [ROBOT_STATE_MOVING_WITH_TASK] = {
        .name = "moving_with_task",
        .eyes = EYE_CONSTANT,  .eyes_time_s = 1.0f,
        .leds = LED_EFFECT_PULSE, .color = RGB_BLUE, .leds_time_s = 1.0f,
        .tail = { .run = ON },
        .door = DOOR_CLOSE,
    },
    [ROBOT_STATE_MOVING_WITHOUT_TASK] = {
        .name = "moving_without_task",
        .eyes = EYE_CONSTANT,  .eyes_time_s = 1.0f,
        .leds = LED_EFFECT_PULSE, .color = RGB_WHITE, .leds_time_s = 1.0f,
        .tail = { .run = ON },
        .door = DOOR_CLOSE,
    },
    [ROBOT_STATE_SUCCESS_WAITING] = {
        .name = "success_waiting",
        .eyes = EYE_CONSTANT,  .eyes_time_s = 1.0f,
        .leds = LED_EFFECT_PULSE, .color = RGB_GREEN, .leds_time_s = 1.0f,
        .tail = { .run = ON, .brake = ON },
        .door = DOOR_KEEP,
        .sound = &SOUND_ARRIVED,
    },
    [ROBOT_STATE_SUCCESS_OPEN_DOOR] = {
        .name = "success_open_door",
        .eyes = EYE_CONSTANT,  .eyes_time_s = 1.0f,
        .leds = LED_EFFECT_SOLID, .color = RGB_GREEN, .leds_time_s = 1.0f,
        .tail = { .run = ON, .brake = ON },
        .door = DOOR_OPEN,
        .sound = &SOUND_OPEN,
    },
    [ROBOT_STATE_SUCCESS_CLOSE_DOOR] = {        /* same look, door closes */
        .name = "success_close_door",
        .eyes = EYE_CONSTANT,  .eyes_time_s = 1.0f,
        .leds = LED_EFFECT_SOLID, .color = RGB_GREEN, .leds_time_s = 1.0f,
        .tail = { .run = ON, .brake = ON },
        .door = DOOR_CLOSE,
        .sound = &SOUND_OPEN,
    },
    [ROBOT_STATE_FAILURE] = {
        .name = "failure",
        .eyes = EYE_CONSTANT,  .eyes_time_s = 1.0f,
        .leds = LED_EFFECT_PULSE, .color = RGB_RED, .leds_time_s = 1.0f,
        .tail = { .run = ON, .brake = ON, .left = ON, .right = ON },  /* hazards */
        .door = DOOR_FREEZE,
        .sound = &SOUND_FAILURE,
    },
    [ROBOT_STATE_FAILURE_NOT_CRITICAL] = {
        .name = "failure_not_critical",
        .eyes = EYE_CONSTANT,  .eyes_time_s = 1.0f,
        .leds = LED_EFFECT_PULSE, .color = RGB_YELLOW, .leds_time_s = 1.0f,
        .tail = { .run = ON, .brake = ON },
        .door = DOOR_KEEP,
        .sound = &SOUND_WARNING,
    },
    [ROBOT_STATE_CHARGING] = {
        .name = "charging",
        /* Eyes constant, then driven by the comet (see charging_task),
         * exactly like the original charging(). */
        .eyes = EYE_CONSTANT,  .eyes_time_s = 1.0f,
        .leds = LED_EFFECT_COMET, .color = RGB_CYAN, .leds_time_s = 2.0f,
        .tail = { 0 },
        .door = DOOR_KEEP,
    },
};

static SemaphoreHandle_t s_mutex;
static robot_state_t     s_state = ROBOT_STATE_STARTUP;
static bool              s_door_actions;

static void apply(robot_state_t s)
{
    const state_look_t *look = &STATES[s];

    led_strips_set(look->leds, look->color, look->leds_time_s);
    eyes_set_emotion(look->eyes, look->eyes_time_s);
    tail_lights_apply(&look->tail);

    if (s_door_actions) {
        const door_plan_t *d = &look->door;
        switch (d->action) {
        case DOOR_ACTION_OPEN:                 /* absolute: safe to repeat */
            door_move_to_start(APP_DOOR_OPEN_DEG, d->rpm, d->limit_a);
            break;
        case DOOR_ACTION_CLOSE:
            door_move_to_start(0.0f, d->rpm, d->limit_a);
            break;
        case DOOR_ACTION_STOP:
            door_stop();
            break;
        case DOOR_ACTION_NONE:
        default:
            break;
        }
    }

#if APP_AUDIO_STATE_SOUNDS
    if (look->sound && audio_speaker_available()) {
        audio_stop();
        audio_play_melody(look->sound);
    }
#endif
}

/* Door problems are reported, and escalate to "failure_not_critical". */
static void on_door_event(door_state_t state, door_fault_t fault, void *ctx)
{
    app_event("door %s %s %s", door_state_name(state), door_fault_name(fault),
              door_result_name(door_get_status().result));

    if (state == DOOR_STATE_FAULT) {
        robot_state_t now = robot_state_get();
        if (now != ROBOT_STATE_FAILURE && now != ROBOT_STATE_FAILURE_NOT_CRITICAL) {
            ESP_LOGW(TAG, "door fault (%s) -> failure_not_critical", door_fault_name(fault));
            robot_state_set(ROBOT_STATE_FAILURE_NOT_CRITICAL);
        }
    }
}

/*
 * Charging, as in the original states.c: while the cyan comet laps the box
 * (one lap = the state's leds_time_s), the eyes fade in during the first half
 * of each lap and out during the second half, driven from the comet's own
 * clock so both start, peak and finish together.
 */
#define ANIM_FRAME_MS  10

static void charging_task(void *arg)
{
    const float lap_s = STATES[ROBOT_STATE_CHARGING].leds_time_s;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(ANIM_FRAME_MS));
        /* Hold the state lock so a state change can't slip in between. */
        if (xSemaphoreTake(s_mutex, 0) != pdTRUE) {
            continue;
        }
        if (s_state == ROBOT_STATE_CHARGING) {
            float progress = fmodf(led_strips_effect_time_s() / lap_s, 1.0f);
            float level = (progress < 0.5f) ? (progress * 2.0f)
                                            : ((1.0f - progress) * 2.0f);
            eyes_set_level(level);
        }
        xSemaphoreGive(s_mutex);
    }
}

void robot_state_init(bool door_actions)
{
    s_mutex        = xSemaphoreCreateMutex();
    xTaskCreate(charging_task, "charging", 2048, NULL, 3, NULL);
    s_door_actions = door_actions;
    door_set_event_callback(on_door_event, NULL);
}

void robot_state_set(robot_state_t s)
{
    if (s >= ROBOT_STATE_COUNT) {
        return;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_state = s;
    apply(s);
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "-> %s", STATES[s].name);
    app_event("state %s", STATES[s].name);
}

void robot_state_set_door_actions(bool on)
{
    s_door_actions = on;
}

bool robot_state_door_actions(void)
{
    return s_door_actions;
}

robot_state_t robot_state_get(void)
{
    return s_state;
}

const char *robot_state_name(robot_state_t s)
{
    return (s < ROBOT_STATE_COUNT) ? STATES[s].name : "?";
}

bool robot_state_from_name(const char *name, robot_state_t *out)
{
    for (int s = 0; s < ROBOT_STATE_COUNT; s++) {
        if (strcmp(name, STATES[s].name) == 0) {
            *out = (robot_state_t)s;
            return true;
        }
    }
    return false;
}
