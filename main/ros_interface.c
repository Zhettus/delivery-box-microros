/*
 * ros_interface.c — micro-ROS node "delivery_box". See ros_interface.h.
 *
 * Connection life cycle (reconnects forever):
 *
 *   WAITING_AGENT --ping ok--> create node/topics --> CONNECTED
 *         ^                                              |
 *         +---- destroy node/topics <-- ping lost -------+
 *
 * All micro-ROS calls happen in this one task (rcl is not thread-safe).
 * Other tasks change the robot state/door; this task notices the change on
 * its 100 ms timer and publishes it.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/battery_state.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>

#include "app.h"
#include "app_config.h"
#include "demo.h"
#include "door.h"
#include "power_monitor.h"
#include "robot_state.h"
#include "ros_interface.h"
#include "usb_jtag_transport.h"

static const char *TAG = "ros";

#define TOPIC_SET_STATE    "/delivery_box/set_state"
#define TOPIC_DOOR_CMD     "/delivery_box/door_cmd"
#define TOPIC_DOOR_GOTO    "/delivery_box/door_goto"
#define TOPIC_STATE        "/delivery_box/state"
#define TOPIC_DOOR_STATE   "/delivery_box/door_state"
#define TOPIC_DOOR_RESULT  "/delivery_box/door_result"
#define TOPIC_LATCH        "/delivery_box/latch"
#define TOPIC_DOOR_ANGLE   "/delivery_box/door_angle"
#define TOPIC_POWER        "/delivery_box/power"

#define TIMER_PERIOD_MS    100    /* change detection + publishing           */
#define REPUBLISH_TICKS    10     /* state/door_state repeated every 1 s      */
#define ANGLE_TICKS        2      /* door angle at 5 Hz                       */
#define POWER_TICKS        10     /* power at 1 Hz                            */
#define PING_PERIOD_MS     1000   /* agent liveness check while connected     */
#define STR_CAPACITY       48

/* Returns false from the enclosing function when an rcl call fails. */
#define RC_TRY(fn) do { rcl_ret_t _rc = (fn); if (_rc != RCL_RET_OK) {          \
    ESP_LOGE(TAG, "%s failed (%d) line %d", #fn, (int)_rc, __LINE__);         \
    rcl_reset_error(); return false; } } while (0)
#define RC_SOFT(fn) do { rcl_ret_t _rc = (fn); (void)_rc; } while (0)

/* ---- micro-ROS entities (only touched by the ros task) ---------------------- */
static rcl_allocator_t    s_alloc;
static rclc_support_t     s_support;
static rcl_node_t         s_node;
/* app-colcon.meta limits: 4 subscriptions, 6 publishers. */
static rcl_subscription_t s_sub_state, s_sub_door, s_sub_goto;
static rcl_publisher_t    s_pub_state, s_pub_door_state, s_pub_door_result, s_pub_latch,
                          s_pub_angle, s_pub_power;
static rcl_timer_t        s_timer;
static rclc_executor_t    s_executor;

/* ---- messages + their string storage ---------------------------------------- */
static std_msgs__msg__String        s_in_state, s_in_door;
static std_msgs__msg__String        s_out_state, s_out_door, s_out_result, s_out_latch;
static std_msgs__msg__Float32       s_in_goto, s_out_angle;
static sensor_msgs__msg__BatteryState s_out_power;
static char s_in_state_buf[STR_CAPACITY], s_in_door_buf[STR_CAPACITY];
static char s_out_state_buf[STR_CAPACITY], s_out_door_buf[STR_CAPACITY];
static char s_out_result_buf[STR_CAPACITY], s_out_latch_buf[STR_CAPACITY];

/* ---- change detection ----------------------------------------------------------- */
static robot_state_t s_last_state;
static door_state_t  s_last_door;
static door_fault_t  s_last_fault;
static door_result_t s_last_result;
static door_lock_state_t s_last_lock;
static uint32_t      s_ticks;

static void bind_string(std_msgs__msg__String *m, char *buf, size_t cap)
{
    m->data.data     = buf;
    m->data.capacity = cap;
    m->data.size     = 0;
    buf[0]           = '\0';
}

static void set_string(std_msgs__msg__String *m, const char *text)
{
    strlcpy(m->data.data, text, m->data.capacity);
    m->data.size = strlen(m->data.data);
}

/* Copy an incoming ROS string into a C string (it is not always 0-terminated). */
static void get_string(const std_msgs__msg__String *m, char *out, size_t cap)
{
    size_t n = m->data.size < cap - 1 ? m->data.size : cap - 1;
    memcpy(out, m->data.data, n);
    out[n] = '\0';
}

/* ---- subscriptions ------------------------------------------------------------ */

/* A command from the robot computer always wins over a running demo. */
static void take_over_from_demo(void)
{
    if (demo_running()) {
        demo_stop();
        ESP_LOGI(TAG, "demo stopped: ROS is in control");
    }
}

static void on_set_state(const void *msgin)
{
    char name[STR_CAPACITY];
    get_string((const std_msgs__msg__String *)msgin, name, sizeof(name));

    robot_state_t s;
    if (!robot_state_from_name(name, &s)) {
        ESP_LOGW(TAG, "set_state: unknown state '%s'", name);
        return;
    }
    ESP_LOGI(TAG, "set_state: %s", name);
    take_over_from_demo();
    robot_state_set(s);
}

static void on_door_cmd(const void *msgin)
{
    char cmd[STR_CAPACITY];
    get_string((const std_msgs__msg__String *)msgin, cmd, sizeof(cmd));

    if (!g_app.door_ok) {
        ESP_LOGW(TAG, "door_cmd '%s' ignored: door driver not running", cmd);
        return;
    }
    take_over_from_demo();
    /* open/close: same moves as the robot states (speed + current limit
     * from app_config.h, latch unlocked first, locked again when closed) */
    if      (strcmp(cmd, "open") == 0)
        door_move_to_start(APP_DOOR_OPEN_DEG, APP_DOOR_OPEN_RPM, APP_DOOR_OPEN_LIMIT_A);
    else if (strcmp(cmd, "close") == 0)
        door_move_to_start(0.0f, APP_DOOR_CLOSE_RPM, APP_DOOR_CLOSE_LIMIT_A);
    else if (strcmp(cmd, "stop") == 0)   door_stop();
    else if (strcmp(cmd, "home") == 0)   door_home();
    else if (strcmp(cmd, "unlock") == 0) door_latch_hold_unlocked(true);
    else if (strcmp(cmd, "lock") == 0)   door_latch_hold_unlocked(false);
    else {
        ESP_LOGW(TAG, "door_cmd: unknown '%s' (open|close|stop|home|unlock|lock)", cmd);
        return;
    }
    ESP_LOGI(TAG, "door_cmd: %s", cmd);
}

static void on_door_goto(const void *msgin)
{
    float deg = ((const std_msgs__msg__Float32 *)msgin)->data;
    if (!g_app.door_ok) {
        ESP_LOGW(TAG, "door_goto ignored: door driver not running");
        return;
    }
    if (!(deg >= 0.0f && deg <= APP_DOOR_OPEN_DEG)) {     /* also rejects NaN */
        ESP_LOGW(TAG, "door_goto %.1f ignored: must be 0..%.0f deg", deg, APP_DOOR_OPEN_DEG);
        return;
    }
    take_over_from_demo();
    float rpm   = deg > door_get_status().position_deg ? APP_DOOR_OPEN_RPM : APP_DOOR_CLOSE_RPM;
    float limit = deg > door_get_status().position_deg ? APP_DOOR_OPEN_LIMIT_A
                                                       : APP_DOOR_CLOSE_LIMIT_A;
    door_move_to_start(deg, rpm, limit);
    ESP_LOGI(TAG, "door_goto: %.1f deg", deg);
}

/* ---- publishing ---------------------------------------------------------------- */

static void publish_state(void)
{
    set_string(&s_out_state, robot_state_name(s_last_state));
    RC_SOFT(rcl_publish(&s_pub_state, &s_out_state, NULL));
}

static void publish_door_state(void)
{
    char text[STR_CAPACITY];
    if (s_last_door == DOOR_STATE_FAULT) {
        snprintf(text, sizeof(text), "fault:%s", door_fault_name(s_last_fault));
    } else {
        strlcpy(text, door_state_name(s_last_door), sizeof(text));
    }
    set_string(&s_out_door, text);
    RC_SOFT(rcl_publish(&s_pub_door_state, &s_out_door, NULL));
}

static void publish_text(rcl_publisher_t *pub, std_msgs__msg__String *m, const char *text)
{
    set_string(m, text);
    RC_SOFT(rcl_publish(pub, m, NULL));
}

static void publish_power(void)
{
    ina228_reading_t r;
    if (!g_app.power_ok || !power_monitor_latest(&r, NULL)) {
        return;
    }
    if (rmw_uros_epoch_synchronized()) {
        int64_t ns = rmw_uros_epoch_nanos();
        s_out_power.header.stamp.sec     = (int32_t)(ns / 1000000000LL);
        s_out_power.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
    }
    s_out_power.voltage     = r.bus_voltage_v;
    s_out_power.current     = -r.current_a;  /* ROS convention: negative = discharging */
    s_out_power.temperature = r.die_temp_c;  /* INA228 die, not the battery */
    s_out_power.charge      = NAN;
    s_out_power.capacity    = NAN;
    s_out_power.design_capacity = NAN;
    s_out_power.percentage  = NAN;
    s_out_power.present     = true;
    RC_SOFT(rcl_publish(&s_pub_power, &s_out_power, NULL));
}

static void on_timer(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;
    s_ticks++;

    /* Robot state: publish at once on change, and every second anyway. */
    robot_state_t st = robot_state_get();
    if (st != s_last_state || s_ticks % REPUBLISH_TICKS == 0) {
        s_last_state = st;
        publish_state();
    }

    door_status_t d = door_get_status();
    if (d.state != s_last_door || d.fault != s_last_fault || s_ticks % REPUBLISH_TICKS == 0) {
        s_last_door  = d.state;
        s_last_fault = d.fault;
        publish_door_state();
    }

    /* A move just ended: tell how (done, returned after resistance, ...). */
    if (d.result != s_last_result) {
        if (d.result != DOOR_RESULT_BUSY && s_last_result != (door_result_t)-1) {
            publish_text(&s_pub_door_result, &s_out_result, door_result_name(d.result));
        }
        s_last_result = d.result;
    }

    if (d.lock != s_last_lock || s_ticks % REPUBLISH_TICKS == 0) {
        s_last_lock = d.lock;
        publish_text(&s_pub_latch, &s_out_latch, door_lock_state_name(d.lock));
    }

    if (s_ticks % ANGLE_TICKS == 0) {
        s_out_angle.data = d.position_deg;
        RC_SOFT(rcl_publish(&s_pub_angle, &s_out_angle, NULL));
    }

    if (s_ticks % POWER_TICKS == 0) {
        publish_power();
    }
}

/* ---- entity life cycle ---------------------------------------------------------- */

static bool create_entities(void)
{
    s_alloc = rcl_get_default_allocator();
    RC_TRY(rclc_support_init(&s_support, 0, NULL, &s_alloc));
    RC_TRY(rclc_node_init_default(&s_node, APP_ROS_NODE_NAME, "", &s_support));

    /* Commands: reliable, so a state change is never silently lost. */
    RC_TRY(rclc_subscription_init_default(&s_sub_state, &s_node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), TOPIC_SET_STATE));
    RC_TRY(rclc_subscription_init_default(&s_sub_door, &s_node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), TOPIC_DOOR_CMD));
    RC_TRY(rclc_subscription_init_default(&s_sub_goto, &s_node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), TOPIC_DOOR_GOTO));

    /* Status: reliable. Streams (angle, power): best effort. */
    RC_TRY(rclc_publisher_init_default(&s_pub_state, &s_node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), TOPIC_STATE));
    RC_TRY(rclc_publisher_init_default(&s_pub_door_state, &s_node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), TOPIC_DOOR_STATE));
    RC_TRY(rclc_publisher_init_default(&s_pub_door_result, &s_node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), TOPIC_DOOR_RESULT));
    RC_TRY(rclc_publisher_init_default(&s_pub_latch, &s_node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), TOPIC_LATCH));
    RC_TRY(rclc_publisher_init_best_effort(&s_pub_angle, &s_node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), TOPIC_DOOR_ANGLE));
    RC_TRY(rclc_publisher_init_best_effort(&s_pub_power, &s_node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), TOPIC_POWER));

    RC_TRY(rclc_timer_init_default2(&s_timer, &s_support,
           RCL_MS_TO_NS(TIMER_PERIOD_MS), on_timer, true));

    /* 4 handles = 3 subscriptions + 1 timer (publishers don't count). */
    s_executor = rclc_executor_get_zero_initialized_executor();
    RC_TRY(rclc_executor_init(&s_executor, &s_support.context, 4, &s_alloc));
    RC_TRY(rclc_executor_add_subscription(&s_executor, &s_sub_state, &s_in_state,
                                          on_set_state, ON_NEW_DATA));
    RC_TRY(rclc_executor_add_subscription(&s_executor, &s_sub_door, &s_in_door,
                                          on_door_cmd, ON_NEW_DATA));
    RC_TRY(rclc_executor_add_subscription(&s_executor, &s_sub_goto, &s_in_goto,
                                          on_door_goto, ON_NEW_DATA));
    RC_TRY(rclc_executor_add_timer(&s_executor, &s_timer));

    /* Align message time stamps with the PC clock. */
    rmw_uros_sync_session(1000);

    /* Publish everything once right after connecting. */
    s_last_state = (robot_state_t)-1;
    s_last_door  = (door_state_t)-1;
    s_last_lock  = (door_lock_state_t)-1;
    s_last_result = (door_result_t)-1;      /* no stale result right after connecting */
    return true;
}

static void destroy_entities(void)
{
    /* The agent is gone: don't wait for it to confirm each deletion. */
    rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&s_support.context);
    if (rmw_ctx) {
        (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);
    }

    RC_SOFT(rcl_subscription_fini(&s_sub_state, &s_node));
    RC_SOFT(rcl_subscription_fini(&s_sub_door, &s_node));
    RC_SOFT(rcl_subscription_fini(&s_sub_goto, &s_node));
    RC_SOFT(rcl_publisher_fini(&s_pub_state, &s_node));
    RC_SOFT(rcl_publisher_fini(&s_pub_door_state, &s_node));
    RC_SOFT(rcl_publisher_fini(&s_pub_door_result, &s_node));
    RC_SOFT(rcl_publisher_fini(&s_pub_latch, &s_node));
    RC_SOFT(rcl_publisher_fini(&s_pub_angle, &s_node));
    RC_SOFT(rcl_publisher_fini(&s_pub_power, &s_node));
    RC_SOFT(rcl_timer_fini(&s_timer));
    RC_SOFT(rclc_executor_fini(&s_executor));
    RC_SOFT(rcl_node_fini(&s_node));
    RC_SOFT(rclc_support_fini(&s_support));
}

/* What the box does when the robot computer disappears. */
static void on_agent_lost(void)
{
#if APP_ROS_STOP_DOOR_WHEN_LOST
    door_state_t d = door_get_status().state;
    if (d == DOOR_STATE_OPENING || d == DOOR_STATE_CLOSING ||
        d == DOOR_STATE_BACKING_OFF || d == DOOR_STATE_RETRY_WAIT ||
        d == DOOR_STATE_HOMING) {
        ESP_LOGW(TAG, "agent lost while the door was moving: stopping it");
        door_stop();
    }
#endif
#if APP_ROS_WARN_WHEN_LOST
    if (robot_state_get() != ROBOT_STATE_FAILURE) {
        robot_state_set(ROBOT_STATE_FAILURE_NOT_CRITICAL);
    }
#endif
}

static void ros_task(void *arg)
{
    bind_string(&s_in_state,  s_in_state_buf,  sizeof(s_in_state_buf));
    bind_string(&s_in_door,   s_in_door_buf,   sizeof(s_in_door_buf));
    bind_string(&s_out_state, s_out_state_buf, sizeof(s_out_state_buf));
    bind_string(&s_out_door,  s_out_door_buf,  sizeof(s_out_door_buf));
    bind_string(&s_out_result, s_out_result_buf, sizeof(s_out_result_buf));
    bind_string(&s_out_latch, s_out_latch_buf, sizeof(s_out_latch_buf));
    sensor_msgs__msg__BatteryState__init(&s_out_power);
    s_out_power.power_supply_status     = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_UNKNOWN;
    s_out_power.power_supply_health     = sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;
    s_out_power.power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_UNKNOWN;

    for (;;) {
        /* 1. Wait for the agent. */
        ESP_LOGI(TAG, "waiting for the micro-ROS agent on the native USB port...");
        while (rmw_uros_ping_agent(200, 1) != RMW_RET_OK) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        /* 2. Create the node and topics. */
        if (!create_entities()) {
            ESP_LOGE(TAG, "could not create the ROS entities, retrying");
            destroy_entities();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        ESP_LOGI(TAG, "connected: node '%s' is up", APP_ROS_NODE_NAME);

        /* 3. Serve until the agent stops answering. */
        TickType_t last_ping = xTaskGetTickCount();
        for (;;) {
            rclc_executor_spin_some(&s_executor, RCL_MS_TO_NS(10));
            if (xTaskGetTickCount() - last_ping >= pdMS_TO_TICKS(PING_PERIOD_MS)) {
                last_ping = xTaskGetTickCount();
                if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        /* 4. Lost: clean up and start over. */
        ESP_LOGW(TAG, "agent lost");
        on_agent_lost();
        destroy_entities();
    }
}

esp_err_t ros_interface_start(void)
{
    /* Same custom transport as hub-motor-microros: framed stream over USB. */
    rmw_ret_t rc = rmw_uros_set_custom_transport(true, NULL,
                                                 usb_jtag_open, usb_jtag_close,
                                                 usb_jtag_write, usb_jtag_read);
    if (rc != RMW_RET_OK) {
        ESP_LOGE(TAG, "rmw_uros_set_custom_transport failed (%d)", (int)rc);
        return ESP_FAIL;
    }
    /* Core 0, below the door (10) and audio (5) tasks; big stack like hub-motor. */
    if (xTaskCreatePinnedToCore(ros_task, "ros", 16384, NULL, 4, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
