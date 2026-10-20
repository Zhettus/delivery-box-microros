/*
 * main.c — Start everything, then the robot computer is in control over
 * micro-ROS (ros_interface.h).
 *
 * START WITH THE DOOR CLOSED: the shaft position at power-up is "closed".
 */
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "audio.h"
#include "board.h"
#include "config.h"
#include "demo.h"
#include "face.h"
#include "led.h"
#include "mic_stream.h"
#include "motor.h"
#include "ros_interface.h"
#include "sound.h"
#include "states.h"
#include "tail_light.h"

static const char *TAG = "main";

/* Someone held the door and it went back: beep (the door task calls this). */
static void on_door_held(bool now_open)
{
    if (DOOR_HELD_SOUND) {
        sound_play(SOUND_DOOR_HELD);
    }
}

/* The door gave up (stuck both ways, timed out, not closed): show it. */
static void on_door_fault(void)
{
    if (states_get() != STATE_FAILURE) {
        states_set(STATE_FAILURE_NOT_CRITICAL);
    }
}

/* One I2C bus for the INA228 (motor current) and the WM8960 (audio HAT). */
static i2c_master_bus_handle_t start_i2c_bus(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_PORT_BOARD,
        .sda_io_num        = PIN_I2C_SDA,
        .scl_io_num        = PIN_I2C_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,   /* backup; the 4.7k externals do the work */
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));
    ESP_LOGI(TAG, "I2C: INA228 0x%02X %s, WM8960 0x%02X %s", INA228_ADDRESS,
             i2c_master_probe(bus, INA228_ADDRESS, 100) == ESP_OK ? "found" : "MISSING",
             AUDIO_WM8960_ADDRESS,
             i2c_master_probe(bus, AUDIO_WM8960_ADDRESS, 100) == ESP_OK ? "found" : "MISSING");
    return bus;
}

void app_main(void)
{
    /* A failed first I2C attempt now and then is normal on the shared bus and
     * is retried (motor-audio-check): keep the driver's own messages quiet. */
    esp_log_level_set("i2c.master", ESP_LOG_NONE);

    i2c_master_bus_handle_t bus = start_i2c_bus();
    ESP_ERROR_CHECK(motor_init(bus));   /* first: motor stopped, door locked */
    /* Audio after the INA228: the codec is set up before the I2S clocks start. */
    esp_err_t audio = sound_init(bus);
    if (audio == ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "no audio HAT (or AUDIO_ENABLED 0): the box stays silent");
    } else if (audio != ESP_OK) {
        ESP_LOGE(TAG, "I2S failed (%s): no sound, no microphone", esp_err_to_name(audio));
    }
    if ((audio == ESP_OK || audio == ESP_ERR_NOT_FOUND) && mic_stream_start() != ESP_OK) {
        ESP_LOGE(TAG, "microphone not started");
    }
    tail_lights_init();
    ESP_ERROR_CHECK(face_init());
    ESP_ERROR_CHECK(led_init());

    motor_on_fault(on_door_fault);
    motor_on_door_held(on_door_held);
    states_init();
    states_set(STATE_STARTUP);

    /* ROS 2: states, door, sound, microphone and speaker over the native USB port. */
    if (ros_interface_start() != ESP_OK) {
        ESP_LOGE(TAG, "micro-ROS FAILED to start");
    }
    if (DEMO_MODE) {
        demo_start();
    }
}
