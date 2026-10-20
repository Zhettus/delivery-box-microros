/*
 * demo.c — Bench demo. See demo.h.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "demo.h"
#include "states.h"

static volatile bool s_running;

static void demo_task(void *arg)
{
    static const robot_state_t ORDER[] = {
        STATE_STARTUP, STATE_MOVING_WITH_TASK, STATE_SUCCESS_WAITING,
        STATE_SUCCESS_OPEN_DOOR, STATE_SUCCESS_CLOSE_DOOR, STATE_MOVING_WITHOUT_TASK,
        STATE_CHARGING, STATE_FAILURE, STATE_FAILURE_NOT_CRITICAL,
    };
    for (size_t i = 0; s_running; i = (i + 1) % (sizeof(ORDER) / sizeof(ORDER[0]))) {
        states_set(ORDER[i]);
        for (int t = 0; t < STATE_HOLD_MS / 100 && s_running; t++) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    vTaskDelete(NULL);
}

void demo_start(void)
{
    if (!s_running) {
        s_running = true;
        xTaskCreate(demo_task, "demo", 3072, NULL, 3, NULL);
    }
}

void demo_stop(void)
{
    s_running = false;
}

bool demo_running(void)
{
    return s_running;
}
