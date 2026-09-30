/*
 * power_monitor.c — Periodic INA228 readings. See power_monitor.h.
 */
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "commands.h"
#include "power_monitor.h"

static const char *TAG = "power";

static ina228_handle_t  s_sensor;
static portMUX_TYPE     s_lock = portMUX_INITIALIZER_UNLOCKED;
static ina228_reading_t s_latest;
static int64_t          s_latest_us;
static bool             s_have;

static void power_task(void *arg)
{
    bool     low_v = false, high_a = false;
    uint32_t ticks = 0;
    uint32_t good  = 0;     /* valid readings so far */

    /* Give the INA228 time for its first complete conversions: a reading
     * taken right after start-up is 0 V and would raise a false alarm. */
    vTaskDelay(pdMS_TO_TICKS(500));
    const uint32_t log_every = APP_POWER_LOG_EVERY_S * 1000 / APP_POWER_PERIOD_MS;

    for (;;) {
        ina228_reading_t r;
        if (ina228_read(s_sensor, &r) == ESP_OK) {
            taskENTER_CRITICAL(&s_lock);
            s_latest    = r;
            s_latest_us = esp_timer_get_time();
            s_have      = true;
            taskEXIT_CRITICAL(&s_lock);

            /* Warn once when a limit is crossed, again after recovery.
             * Skip the first readings (sensor still settling). */
            if (++good < 3) {
                vTaskDelay(pdMS_TO_TICKS(APP_POWER_PERIOD_MS));
                continue;
            }
            if (!low_v && r.bus_voltage_v < APP_POWER_WARN_LOW_V) {
                low_v = true;
                ESP_LOGW(TAG, "LOW VOLTAGE: %.2f V", r.bus_voltage_v);
                app_event("power low_voltage %.2f", r.bus_voltage_v);
            } else if (low_v && r.bus_voltage_v > APP_POWER_WARN_LOW_V + 0.3f) {
                low_v = false;
            }
            if (!high_a && r.current_a > APP_POWER_WARN_HIGH_A) {
                high_a = true;
                ESP_LOGW(TAG, "HIGH CURRENT: %.2f A", r.current_a);
                app_event("power high_current %.2f", r.current_a);
            } else if (high_a && r.current_a < APP_POWER_WARN_HIGH_A * 0.9f) {
                high_a = false;
            }

            if (log_every && ++ticks >= log_every) {
                ticks = 0;
                ESP_LOGI(TAG, "%.2f V  %.3f A  %.2f W  %.1f C  %.3f Wh",
                         r.bus_voltage_v, r.current_a, r.power_w,
                         r.die_temp_c, r.energy_wh);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(APP_POWER_PERIOD_MS));
    }
}

esp_err_t power_monitor_start(ina228_handle_t sensor)
{
    s_sensor = sensor;
    return xTaskCreate(power_task, "power", 3072, NULL, 1, NULL) == pdPASS
               ? ESP_OK : ESP_ERR_NO_MEM;
}

bool power_monitor_latest(ina228_reading_t *out, uint32_t *age_ms)
{
    taskENTER_CRITICAL(&s_lock);
    bool have = s_have;
    *out = s_latest;
    int64_t t = s_latest_us;
    taskEXIT_CRITICAL(&s_lock);

    if (age_ms) {
        *age_ms = (uint32_t)((esp_timer_get_time() - t) / 1000);
    }
    return have;
}
