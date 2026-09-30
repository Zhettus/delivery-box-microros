/*
 * demo.c — Cycle through the robot states. See demo.h.
 */
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "commands.h"
#include "demo.h"
#include "robot_state.h"

static const char *TAG = "demo";

/* Same order as the original final.c. */
static const robot_state_t ORDER[] = {
    ROBOT_STATE_STARTUP,
    ROBOT_STATE_MOVING_WITH_TASK,
    ROBOT_STATE_SUCCESS_WAITING,
    ROBOT_STATE_SUCCESS_OPEN_DOOR,
    ROBOT_STATE_SUCCESS_CLOSE_DOOR,
    ROBOT_STATE_MOVING_WITHOUT_TASK,
    ROBOT_STATE_CHARGING,
    ROBOT_STATE_FAILURE,
    ROBOT_STATE_FAILURE_NOT_CRITICAL,
};
#define ORDER_LEN  (sizeof(ORDER) / sizeof(ORDER[0]))

static TaskHandle_t  s_task;
static volatile bool s_run;
static uint32_t      s_step_ms;
static bool          s_door_before;

static void demo_task(void *arg)
{
    for (size_t i = 0; s_run; i = (i + 1) % ORDER_LEN) {
        robot_state_set(ORDER[i]);
        /* wait in small slices so "demo off" reacts quickly */
        for (uint32_t t = 0; t < s_step_ms && s_run; t += 100) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    robot_state_set_door_actions(s_door_before);   /* restore */
    app_event("demo stopped");
    s_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t demo_start(uint32_t step_ms, bool move_door)
{
    if (s_task) {
        return ESP_ERR_INVALID_STATE;              /* already running */
    }
    s_step_ms     = step_ms < 500 ? 500 : step_ms;
    s_door_before = robot_state_door_actions();
    robot_state_set_door_actions(move_door);
    s_run = true;
    if (xTaskCreate(demo_task, "demo", 4096, NULL, 2, &s_task) != pdPASS) {
        s_run = false;
        robot_state_set_door_actions(s_door_before);
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGW(TAG, "cycling states every %lu ms, door %s",
             (unsigned long)s_step_ms, move_door ? "MOVES" : "stays still");
    return ESP_OK;
}

/* Returns once the demo has really ended, so a state set right after this
 * call can't be overwritten by the demo's last step. */
void demo_stop(void)
{
    s_run = false;
    for (int i = 0; i < 50 && s_task != NULL; i++) {      /* <= 0.5 s */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool demo_running(void)
{
    return s_task != NULL;
}
