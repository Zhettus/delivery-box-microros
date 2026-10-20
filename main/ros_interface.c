/*
 * ros_interface.c — micro-ROS node "delivery_box". See ros_interface.h.
 *
 * Connection life cycle (reconnects forever):
 *
 *   WAITING_AGENT --ping ok--> create node/topics --> CONNECTED
 *         ^                                              |
 *         +---- destroy node/topics <-- ping lost -------+
 *
 * All micro-ROS calls happen in this one task (rcl is not thread-safe). It
 * spins every few ms (the speaker stream arrives 100 times a second),
 * publishes microphone chunks as soon as they are ready, and notices state
 * changes of the box on a 100 ms timer.
 */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
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
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/string.h>

#include "audio.h"
#include "config.h"
#include "demo.h"
#include "door.h"
#include "mic_stream.h"
#include "motor.h"
#include "ros_interface.h"
#include "sound.h"
#include "states.h"
#include "usb_jtag_transport.h"

static const char *TAG = "ros";

#define T(name)            ROS_TOPIC_PREFIX "/" name
#define TIMER_PERIOD_MS    100    /* change detection + publishing           */
#define REPUBLISH_TICKS    10     /* status topics repeated every 1 s         */
#define ANGLE_TICKS        2      /* door angle at 5 Hz                       */
#define POWER_TICKS        10     /* power at 1 Hz                            */
#define PING_PERIOD_MS     1000   /* agent liveness check while connected     */
#define SPIN_WAIT_MS       2      /* longest pause between spins              */
#define STATS_PERIOD_MS    5000   /* stream statistics in the log             */
#define STR_CAPACITY       48
#define SPEAKER_MAX_SAMPLES 256   /* biggest speaker/audio message accepted   */
#define DIM_CAPACITY       2      /* layout.dim entries accepted (normally 0) */

/* Returns false from the enclosing function when an rcl call fails. */
#define RC_TRY(fn) do { rcl_ret_t _rc = (fn); if (_rc != RCL_RET_OK) {          \
    ESP_LOGE(TAG, "%s failed (%d) line %d", #fn, (int)_rc, __LINE__);         \
    rcl_reset_error(); return false; } } while (0)
#define RC_SOFT(fn) do { rcl_ret_t _rc = (fn); (void)_rc; } while (0)

/* ---- micro-ROS entities (only touched by the ros task) ---------------------- */
/* app-colcon.meta limits: 10 publishers, 6 subscriptions. */
static rcl_allocator_t    s_alloc;
static rclc_support_t     s_support;
static rcl_node_t         s_node;
static rcl_subscription_t s_sub_state, s_sub_door, s_sub_goto, s_sub_sound,
                          s_sub_mic_enable, s_sub_speaker;
static rcl_publisher_t    s_pub_state, s_pub_door_state, s_pub_door_result, s_pub_latch,
                          s_pub_angle, s_pub_power, s_pub_mic_status, s_pub_mic_audio,
                          s_pub_speaker_status;
static rcl_timer_t        s_timer;
static rclc_executor_t    s_executor;

/* ---- messages + their storage ------------------------------------------------- */
static std_msgs__msg__String  s_in_state, s_in_door, s_in_sound;
static std_msgs__msg__String  s_out_state, s_out_door, s_out_result, s_out_latch, s_out_speaker;
static std_msgs__msg__Float32 s_in_goto, s_out_angle;
static std_msgs__msg__Bool    s_in_mic_enable, s_out_mic_status;
static std_msgs__msg__Int16MultiArray s_in_speaker, s_out_mic;
static sensor_msgs__msg__BatteryState s_out_power;

static char    s_in_state_buf[STR_CAPACITY], s_in_door_buf[STR_CAPACITY];
static char    s_in_sound_buf[STR_CAPACITY];
static char    s_out_state_buf[STR_CAPACITY], s_out_door_buf[STR_CAPACITY];
static char    s_out_result_buf[STR_CAPACITY], s_out_latch_buf[STR_CAPACITY];
static char    s_out_speaker_buf[96];
static int16_t s_speaker_buf[SPEAKER_MAX_SAMPLES];
static std_msgs__msg__MultiArrayDimension s_speaker_dims[DIM_CAPACITY];
static char    s_speaker_dim_labels[DIM_CAPACITY][16];
static int16_t s_mic_buf[MIC_CHUNK_SAMPLES];

/* ---- change detection ----------------------------------------------------------- */
static robot_state_t     s_last_state;
static door_state_t      s_last_door;
static door_fault_t      s_last_fault;
static door_result_t     s_last_result;
static door_lock_state_t s_last_lock;
static int               s_last_mic;
static int               s_last_speaker;
static uint32_t          s_ticks;

/* ---- strings ------------------------------------------------------------------ */

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

static void publish_text(rcl_publisher_t *pub, std_msgs__msg__String *m, const char *text)
{
    set_string(m, text);
    RC_SOFT(rcl_publish(pub, m, NULL));
}

/* ---- subscriptions -------------------------------------------------------------- */

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
    if (!state_from_name(name, &s)) {
        ESP_LOGW(TAG, "set_state: unknown state '%s'", name);
        return;
    }
    ESP_LOGI(TAG, "set_state: %s", name);
    take_over_from_demo();
    states_set(s);
}

static void on_door_cmd(const void *msgin)
{
    char cmd[STR_CAPACITY];
    get_string((const std_msgs__msg__String *)msgin, cmd, sizeof(cmd));
    take_over_from_demo();

    /* open/close: the same moves as the robot states (held -> goes back) */
    if      (strcmp(cmd, "open") == 0)   motor_open_door();
    else if (strcmp(cmd, "close") == 0)  motor_close_door();
    else if (strcmp(cmd, "stop") == 0)   motor_stop();
    else if (strcmp(cmd, "home") == 0)   motor_home();
    else if (strcmp(cmd, "unlock") == 0) motor_latch_hold_unlocked(true);
    else if (strcmp(cmd, "lock") == 0)   motor_latch_hold_unlocked(false);
    else {
        ESP_LOGW(TAG, "door_cmd: unknown '%s' (open|close|stop|home|unlock|lock)", cmd);
        return;
    }
    ESP_LOGI(TAG, "door_cmd: %s", cmd);
}

static void on_door_goto(const void *msgin)
{
    float deg = ((const std_msgs__msg__Float32 *)msgin)->data;
    if (!(deg >= 0.0f && deg <= DOOR_OPEN_ANGLE_DEG)) {     /* also rejects NaN */
        ESP_LOGW(TAG, "door_goto %.1f ignored: must be 0..%.0f deg", deg, DOOR_OPEN_ANGLE_DEG);
        return;
    }
    take_over_from_demo();
    motor_go_to(deg);
    ESP_LOGI(TAG, "door_goto: %.1f deg", deg);
}

static void on_sound(const void *msgin)
{
    char cmd[STR_CAPACITY];
    get_string((const std_msgs__msg__String *)msgin, cmd, sizeof(cmd));

    sound_t s;
    if (strcmp(cmd, "stop") == 0) {
        sound_stop();
    } else if (strncmp(cmd, "volume ", 7) == 0) {
        char *end;
        long v = strtol(cmd + 7, &end, 10);
        if (end == cmd + 7 || *end != '\0' || v < 0 || v > 100) {
            ESP_LOGW(TAG, "sound: 'volume' needs 0..100, got '%s'", cmd + 7);
            return;
        }
        sound_set_volume((uint8_t)v);
    } else if (sound_from_name(cmd, &s)) {
        sound_play(s);
    } else {
        ESP_LOGW(TAG, "sound: unknown '%s' (startup|arrived|greeting|warning|failure|"
                      "door_held|stop|volume N)", cmd);
        return;
    }
    ESP_LOGI(TAG, "sound: %s", cmd);
}

static void on_mic_enable(const void *msgin)
{
    bool on = ((const std_msgs__msg__Bool *)msgin)->data;
    mic_stream_enable(on);
    s_last_mic = -1;                          /* publish the status at once */
}

static void on_speaker_audio(const void *msgin)
{
    const std_msgs__msg__Int16MultiArray *m = msgin;
    audio_stream_push(m->data.data, m->data.size, m->layout.data_offset);
}

/* ---- publishing ---------------------------------------------------------------- */

static void publish_door_state(void)
{
    char text[STR_CAPACITY];
    if (s_last_door == DOOR_STATE_FAULT) {
        snprintf(text, sizeof(text), "fault:%s", door_fault_name(s_last_fault));
    } else if (s_last_fault == DOOR_FAULT_OBSTRUCTED) {
        snprintf(text, sizeof(text), "%s:held", door_state_name(s_last_door));
    } else {
        strlcpy(text, door_state_name(s_last_door), sizeof(text));
    }
    publish_text(&s_pub_door_state, &s_out_door, text);
}

static void publish_power(void)
{
    float volts, amps;
    if (!motor_read_supply(&volts, &amps)) {
        return;
    }
    if (rmw_uros_epoch_synchronized()) {
        int64_t ns = rmw_uros_epoch_nanos();
        s_out_power.header.stamp.sec     = (int32_t)(ns / 1000000000LL);
        s_out_power.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
    }
    s_out_power.voltage         = volts;
    s_out_power.current         = -amps;    /* ROS convention: negative = discharging */
    s_out_power.temperature     = NAN;
    s_out_power.charge          = NAN;
    s_out_power.capacity        = NAN;
    s_out_power.design_capacity = NAN;
    s_out_power.percentage      = NAN;
    s_out_power.present         = true;
    RC_SOFT(rcl_publish(&s_pub_power, &s_out_power, NULL));
}

static void publish_speaker_status(const audio_stream_stats_t *st)
{
    char text[sizeof(s_out_speaker_buf)];
    snprintf(text, sizeof(text), "%s buffered=%" PRIu32 "ms lost=%" PRIu32 " dropped=%" PRIu32
                                 " underruns=%" PRIu32,
             st->playing ? "playing" : "idle", st->buffered_ms, st->lost, st->dropped,
             st->underruns);
    publish_text(&s_pub_speaker_status, &s_out_speaker, text);
}

static void on_timer(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;
    s_ticks++;
    bool every_second = (s_ticks % REPUBLISH_TICKS == 0);

    /* Robot state: at once on change, and every second anyway. */
    robot_state_t st = states_get();
    if (st != s_last_state || every_second) {
        s_last_state = st;
        publish_text(&s_pub_state, &s_out_state, state_name(st));
    }

    door_status_t d = door_get_status();
    if (d.state != s_last_door || d.fault != s_last_fault || every_second) {
        s_last_door  = d.state;
        s_last_fault = d.fault;
        publish_door_state();
    }

    /* A move just ended: tell how (done, returned after the door was held, ...). */
    if (d.result != s_last_result) {
        if (d.result != DOOR_RESULT_BUSY && s_last_result != (door_result_t)-1) {
            publish_text(&s_pub_door_result, &s_out_result, door_result_name(d.result));
        }
        s_last_result = d.result;
    }

    if (d.lock != s_last_lock || every_second) {
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

    int mic = mic_stream_enabled();
    if (mic != s_last_mic || every_second) {
        s_last_mic = mic;
        s_out_mic_status.data = mic;
        RC_SOFT(rcl_publish(&s_pub_mic_status, &s_out_mic_status, NULL));
    }

    audio_stream_stats_t sp = audio_stream_stats();
    if ((int)sp.playing != s_last_speaker || every_second) {
        s_last_speaker = sp.playing;
        publish_speaker_status(&sp);
    }
}

/* ---- entity life cycle ---------------------------------------------------------- */

static bool create_entities(void)
{
    s_alloc = rcl_get_default_allocator();
    RC_TRY(rclc_support_init(&s_support, 0, NULL, &s_alloc));
    RC_TRY(rclc_node_init_default(&s_node, ROS_NODE_NAME, "", &s_support));

    /* Commands: reliable, so a state change is never silently lost. */
    RC_TRY(rclc_subscription_init_default(&s_sub_state, &s_node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), T("set_state")));
    RC_TRY(rclc_subscription_init_default(&s_sub_door, &s_node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), T("door_cmd")));
    RC_TRY(rclc_subscription_init_default(&s_sub_goto, &s_node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), T("door_goto")));
    RC_TRY(rclc_subscription_init_default(&s_sub_sound, &s_node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), T("sound")));
    RC_TRY(rclc_subscription_init_default(&s_sub_mic_enable, &s_node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), T("mic/enable")));
    /* Audio: best effort, like the microphone. A late chunk is useless. */
    RC_TRY(rclc_subscription_init_best_effort(&s_sub_speaker, &s_node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray), T("speaker/audio")));

    /* Status: reliable. Streams (angle, power, audio): best effort. */
    RC_TRY(rclc_publisher_init_default(&s_pub_state, &s_node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), T("state")));
    RC_TRY(rclc_publisher_init_default(&s_pub_door_state, &s_node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), T("door_state")));
    RC_TRY(rclc_publisher_init_default(&s_pub_door_result, &s_node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), T("door_result")));
    RC_TRY(rclc_publisher_init_default(&s_pub_latch, &s_node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), T("latch")));
    RC_TRY(rclc_publisher_init_best_effort(&s_pub_angle, &s_node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), T("door_angle")));
    RC_TRY(rclc_publisher_init_best_effort(&s_pub_power, &s_node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), T("power")));
    RC_TRY(rclc_publisher_init_default(&s_pub_mic_status, &s_node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), T("mic/status")));
    RC_TRY(rclc_publisher_init_best_effort(&s_pub_mic_audio, &s_node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray), T("mic/audio")));
    RC_TRY(rclc_publisher_init_default(&s_pub_speaker_status, &s_node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), T("speaker/status")));

    RC_TRY(rclc_timer_init_default2(&s_timer, &s_support,
           RCL_MS_TO_NS(TIMER_PERIOD_MS), on_timer, true));

    /* 7 handles = 6 subscriptions + 1 timer (publishers don't count). */
    s_executor = rclc_executor_get_zero_initialized_executor();
    RC_TRY(rclc_executor_init(&s_executor, &s_support.context, 7, &s_alloc));
    RC_TRY(rclc_executor_add_subscription(&s_executor, &s_sub_state, &s_in_state,
                                          on_set_state, ON_NEW_DATA));
    RC_TRY(rclc_executor_add_subscription(&s_executor, &s_sub_door, &s_in_door,
                                          on_door_cmd, ON_NEW_DATA));
    RC_TRY(rclc_executor_add_subscription(&s_executor, &s_sub_goto, &s_in_goto,
                                          on_door_goto, ON_NEW_DATA));
    RC_TRY(rclc_executor_add_subscription(&s_executor, &s_sub_sound, &s_in_sound,
                                          on_sound, ON_NEW_DATA));
    RC_TRY(rclc_executor_add_subscription(&s_executor, &s_sub_mic_enable, &s_in_mic_enable,
                                          on_mic_enable, ON_NEW_DATA));
    RC_TRY(rclc_executor_add_subscription(&s_executor, &s_sub_speaker, &s_in_speaker,
                                          on_speaker_audio, ON_NEW_DATA));
    RC_TRY(rclc_executor_add_timer(&s_executor, &s_timer));

    /* Align message time stamps with the computer's clock. */
    rmw_uros_sync_session(1000);

    /* Publish everything once right after connecting. */
    s_last_state   = (robot_state_t)-1;
    s_last_door    = (door_state_t)-1;
    s_last_lock    = (door_lock_state_t)-1;
    s_last_result  = (door_result_t)-1;     /* no stale result right after connecting */
    s_last_mic     = -1;
    s_last_speaker = -1;
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
    RC_SOFT(rcl_subscription_fini(&s_sub_sound, &s_node));
    RC_SOFT(rcl_subscription_fini(&s_sub_mic_enable, &s_node));
    RC_SOFT(rcl_subscription_fini(&s_sub_speaker, &s_node));
    RC_SOFT(rcl_publisher_fini(&s_pub_state, &s_node));
    RC_SOFT(rcl_publisher_fini(&s_pub_door_state, &s_node));
    RC_SOFT(rcl_publisher_fini(&s_pub_door_result, &s_node));
    RC_SOFT(rcl_publisher_fini(&s_pub_latch, &s_node));
    RC_SOFT(rcl_publisher_fini(&s_pub_angle, &s_node));
    RC_SOFT(rcl_publisher_fini(&s_pub_power, &s_node));
    RC_SOFT(rcl_publisher_fini(&s_pub_mic_status, &s_node));
    RC_SOFT(rcl_publisher_fini(&s_pub_mic_audio, &s_node));
    RC_SOFT(rcl_publisher_fini(&s_pub_speaker_status, &s_node));
    RC_SOFT(rcl_timer_fini(&s_timer));
    RC_SOFT(rclc_executor_fini(&s_executor));
    RC_SOFT(rcl_node_fini(&s_node));
    RC_SOFT(rclc_support_fini(&s_support));
}

/* What the box does when the robot computer disappears. */
static void on_agent_lost(void)
{
    mic_stream_enable(false);
#if ROS_STOP_DOOR_WHEN_LOST
    door_state_t d = door_get_status().state;
    if (d == DOOR_STATE_OPENING || d == DOOR_STATE_CLOSING ||
        d == DOOR_STATE_BACKING_OFF || d == DOOR_STATE_RETRY_WAIT ||
        d == DOOR_STATE_HOMING) {
        ESP_LOGW(TAG, "agent lost while the door was moving: stopping it");
        motor_stop();
    }
#endif
#if ROS_WARN_WHEN_LOST
    if (states_get() != STATE_FAILURE) {
        states_set(STATE_FAILURE_NOT_CRITICAL);
    }
#endif
}

static void init_messages(void)
{
    bind_string(&s_in_state,    s_in_state_buf,    sizeof(s_in_state_buf));
    bind_string(&s_in_door,     s_in_door_buf,     sizeof(s_in_door_buf));
    bind_string(&s_in_sound,    s_in_sound_buf,    sizeof(s_in_sound_buf));
    bind_string(&s_out_state,   s_out_state_buf,   sizeof(s_out_state_buf));
    bind_string(&s_out_door,    s_out_door_buf,    sizeof(s_out_door_buf));
    bind_string(&s_out_result,  s_out_result_buf,  sizeof(s_out_result_buf));
    bind_string(&s_out_latch,   s_out_latch_buf,   sizeof(s_out_latch_buf));
    bind_string(&s_out_speaker, s_out_speaker_buf, sizeof(s_out_speaker_buf));

    /* Incoming audio: room for SPEAKER_MAX_SAMPLES and a few (unused) dims. */
    memset(&s_in_speaker, 0, sizeof(s_in_speaker));
    s_in_speaker.data.data     = s_speaker_buf;
    s_in_speaker.data.capacity = SPEAKER_MAX_SAMPLES;
    for (int i = 0; i < DIM_CAPACITY; i++) {
        s_speaker_dims[i].label.data     = s_speaker_dim_labels[i];
        s_speaker_dims[i].label.capacity = sizeof(s_speaker_dim_labels[i]);
    }
    s_in_speaker.layout.dim.data     = s_speaker_dims;
    s_in_speaker.layout.dim.capacity = DIM_CAPACITY;

    /* Outgoing microphone chunk: no dims, no labels. */
    memset(&s_out_mic, 0, sizeof(s_out_mic));
    s_out_mic.data.data     = s_mic_buf;
    s_out_mic.data.size     = MIC_CHUNK_SAMPLES;
    s_out_mic.data.capacity = MIC_CHUNK_SAMPLES;

    sensor_msgs__msg__BatteryState__init(&s_out_power);
    s_out_power.power_supply_status     = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_UNKNOWN;
    s_out_power.power_supply_health     = sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;
    s_out_power.power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
}

static void log_stream_stats(uint32_t mic_published)
{
    uint32_t captured, dropped;
    mic_stream_counters(&captured, &dropped);
    audio_stream_stats_t sp = audio_stream_stats();
    if (mic_stream_enabled()) {
        ESP_LOGI(TAG, "mic: published=%" PRIu32 " captured=%" PRIu32 " dropped=%" PRIu32,
                 mic_published, captured, dropped);
    }
    if (sp.playing) {
        ESP_LOGI(TAG, "speaker: received=%" PRIu32 " lost=%" PRIu32 " dropped=%" PRIu32
                      " underruns=%" PRIu32 " buffered=%" PRIu32 "ms",
                 sp.received, sp.lost, sp.dropped, sp.underruns, sp.buffered_ms);
    }
}

static void ros_task(void *arg)
{
    init_messages();

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
        mic_stream_enable(false);              /* OFF until asked */
        ESP_LOGI(TAG, "connected: node '%s' is up", ROS_NODE_NAME);

        /* 3. Serve until the agent stops answering. */
        TickType_t  last_ping  = xTaskGetTickCount();
        TickType_t  last_stats = last_ping;
        uint32_t    mic_published = 0;
        mic_chunk_t chunk;
        for (;;) {
            rclc_executor_spin_some(&s_executor, 0);

            /* Wake up as soon as a microphone chunk is ready, else after 2 ms
             * to keep spinning (the speaker stream needs it). */
            if (mic_stream_take(&chunk, SPIN_WAIT_MS) && mic_stream_enabled()) {
                memcpy(s_mic_buf, chunk.samples, sizeof(s_mic_buf));
                s_out_mic.layout.data_offset = chunk.offset;
                RC_SOFT(rcl_publish(&s_pub_mic_audio, &s_out_mic, NULL));
                mic_published++;
            }

            TickType_t now = xTaskGetTickCount();
            if (now - last_stats >= pdMS_TO_TICKS(STATS_PERIOD_MS)) {
                last_stats = now;
                log_stream_stats(mic_published);
            }
            if (now - last_ping >= pdMS_TO_TICKS(PING_PERIOD_MS)) {
                last_ping = now;
                if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
                    break;
                }
            }
        }

        /* 4. Lost: clean up and start over. */
        ESP_LOGW(TAG, "agent lost: microphone OFF");
        on_agent_lost();
        destroy_entities();
    }
}

esp_err_t ros_interface_start(void)
{
    rmw_ret_t rc = rmw_uros_set_custom_transport(true, NULL,
                                                 usb_jtag_open, usb_jtag_close,
                                                 usb_jtag_write, usb_jtag_read);
    if (rc != RMW_RET_OK) {
        ESP_LOGE(TAG, "rmw_uros_set_custom_transport failed (%d)", (int)rc);
        return ESP_FAIL;
    }
    /* Core 0, below the door (10), its sensing (9), mic capture (6), states (5). */
    if (xTaskCreatePinnedToCore(ros_task, "ros", 16384, NULL, 4, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
