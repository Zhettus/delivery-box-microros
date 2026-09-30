#include "driver/i2c_master.h"
#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app.h"
#include "app_config.h"
#include "audio.h"
#include "board.h"
#include "commands.h"
#include "demo.h"
#include "door.h"
#include "door_setup.h"
#include "eyes.h"
#include "ina228.h"
#include "led_strips.h"
#include "power_monitor.h"
#include "robot_state.h"
#include "ros_interface.h"
#include "tail_lights.h"

static const char *TAG = "main";

app_t g_app;


static esp_err_t start_door(void)
{
    door_config_t cfg = app_door_config();
    return door_init(&cfg);
}

static esp_err_t start_tail_lights(void)
{
    tail_lights_config_t cfg = {
        .gpio = {
            [TAIL_LIGHT_BRAKE] = PIN_TAIL_BRAKE,
            [TAIL_LIGHT_RUN]   = PIN_TAIL_RUN,
            [TAIL_LIGHT_LEFT]  = PIN_TAIL_LEFT,
            [TAIL_LIGHT_RIGHT] = PIN_TAIL_RIGHT,
        },
        .channel = {
            [TAIL_LIGHT_BRAKE] = LEDC_CH_TAIL_BRAKE,
            [TAIL_LIGHT_RUN]   = LEDC_CH_TAIL_RUN,
            [TAIL_LIGHT_LEFT]  = LEDC_CH_TAIL_LEFT,
            [TAIL_LIGHT_RIGHT] = LEDC_CH_TAIL_RIGHT,
        },
        .timer       = LEDC_TIMER_TAIL,
        .pwm_freq_hz = APP_TAIL_PWM_HZ,
    };
    return tail_lights_init(&cfg);
}

static esp_err_t start_eyes(void)
{
    eyes_config_t cfg = {
        .left_gpio      = PIN_EYE_LEFT,
        .right_gpio     = PIN_EYE_RIGHT,
        .timer          = LEDC_TIMER_EYES,
        .left_channel   = LEDC_CH_EYE_LEFT,
        .right_channel  = LEDC_CH_EYE_RIGHT,
        .pwm_freq_hz    = APP_EYES_PWM_HZ,
        .min_pulse_us   = APP_EYES_MIN_PULSE_US,
        .gamma          = APP_EYES_GAMMA,
        .brightness_cap = APP_EYES_BRIGHTNESS_CAP,
        .invert         = APP_EYES_INVERT,
    };
    return eyes_init(&cfg);
}

static esp_err_t start_leds(void)
{
    led_strips_config_t cfg = {
        .gpio = { PIN_LED_STRIP_1, PIN_LED_STRIP_2, PIN_LED_STRIP_3, PIN_LED_STRIP_4 },
        .layout = {
            .num_strips    = 4,
            .pixels        = { APP_LED_STRIP1_PIXELS, APP_LED_STRIP2_PIXELS,
                               APP_LED_STRIP3_PIXELS, APP_LED_STRIP4_PIXELS },
            .loop_segments = 4,
            .loop          = APP_LED_LOOP,
        },
        .wire_order_rgb     = APP_LED_WIRE_ORDER_RGB,
        .max_brightness     = APP_LED_MAX_BRIGHTNESS,
        .color_correction   = APP_LED_COLOR_CORRECTION,
        .transition_s       = APP_LED_TRANSITION_S,
        .frame_ms           = APP_LED_FRAME_MS,
        .power_budget_ma    = APP_LED_POWER_BUDGET_MA,
        .ma_per_pixel_white = APP_LED_MA_PER_PIXEL_WHITE,
        .ma_per_pixel_idle  = APP_LED_MA_PER_PIXEL_IDLE,
        .comet_tail_px      = APP_LED_COMET_TAIL_PX,
        .diverge_speed_px_s = APP_LED_DIVERGE_SPEED_PX_S,
        .diverge_tail_px    = APP_LED_DIVERGE_TAIL_PX,
    };
    return led_strips_init(&cfg);
}

static esp_err_t start_i2c(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port          = I2C_PORT_BOARD,
        .sda_io_num        = PIN_I2C_SDA,
        .scl_io_num        = PIN_I2C_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,   /* backup; the 4.7k externals do the work */
    };
    return i2c_new_master_bus(&cfg, &g_app.i2c_bus);
}

static esp_err_t start_power_monitor(void)
{
    ina228_config_t cfg = {
        .bus           = g_app.i2c_bus,
        .address       = APP_INA228_ADDRESS,
        .i2c_speed_hz  = APP_I2C_SPEED_HZ,
        .shunt_ohms    = APP_INA228_SHUNT_OHMS,
        .max_current_a = APP_INA228_MAX_CURRENT_A,
        .mode          = INA228_MODE_FAST_CURRENT,  /* ~9 ms: fast enough for the door */
    };
    esp_err_t err = ina228_create(&cfg, &g_app.ina228);
    if (err == ESP_OK) {
        err = power_monitor_start(g_app.ina228);
    }
    /* The INA228 sits in the door motor's supply: use it to detect resistance. */
    if (err == ESP_OK && g_app.door_ok) {
        err = door_attach_current_sensor(app_door_read_ina228, g_app.ina228);
    }
    return err;
}

static esp_err_t start_audio(void)
{
    audio_config_t cfg = {
        .bclk_gpio          = PIN_I2S_BCLK,
        .ws_gpio            = PIN_I2S_WS,
        .dout_gpio          = PIN_I2S_DOUT,
        .din_gpio           = PIN_I2S_DIN,
        .sample_rate_hz     = APP_AUDIO_SAMPLE_RATE_HZ,
        .i2c_bus            = g_app.i2c_bus,
        .codec_address      = APP_AUDIO_CODEC_ADDRESS,
        .codec_i2c_speed_hz = APP_I2C_SPEED_HZ,
        .codec_mclk_hz      = APP_AUDIO_CODEC_MCLK_HZ,
        .volume_percent     = APP_AUDIO_VOLUME_PERCENT,
        .tone_amplitude     = APP_AUDIO_TONE_AMPLITUDE,
    };
    esp_err_t err = audio_init(&cfg);
    g_app.speaker_ok = audio_speaker_available();
    return err;
}

/* Run one start-up step; log failures but keep going. */
static bool step(const char *what, esp_err_t (*fn)(void))
{
    esp_err_t err = fn();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%-13s FAILED: %s", what, esp_err_to_name(err));
        return false;
    }
    return true;
}

static void print_banner(void)
{
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, " Delivery Box — ESP32-S3 N16R8 (wiring rev 4)");
    ESP_LOGI(TAG, " chip rev v%d.%d, %d cores, PSRAM %u KB",
             chip.revision / 100, chip.revision % 100, chip.cores,
             (unsigned)(esp_psram_get_size() / 1024));
    ESP_LOGI(TAG, "==============================================");
}

static void print_summary(void)
{
    ESP_LOGI(TAG, "subsystems: door=%s tail=%s eyes=%s leds=%s i2c=%s power=%s "
                  "mic=%s speaker=%s",
             g_app.door_ok ? "OK" : "--", g_app.tail_ok ? "OK" : "--",
             g_app.eyes_ok ? "OK" : "--", g_app.leds_ok ? "OK" : "--",
             g_app.i2c_bus ? "OK" : "--", g_app.power_ok ? "OK" : "--",
             g_app.audio_ok ? "OK" : "--", g_app.speaker_ok ? "OK" : "--");
}


void app_main(void)
{
    print_banner();

    g_app.door_ok  = step("door motor", start_door);   /* first: motor to STOP */
    g_app.tail_ok  = step("tail light", start_tail_lights);
    g_app.eyes_ok  = step("eyes", start_eyes);
    g_app.leds_ok  = step("LED strips", start_leds);
    if (step("I2C bus", start_i2c)) {
        g_app.power_ok = step("INA228", start_power_monitor);
    }
    g_app.audio_ok = step("audio", start_audio);
    print_summary();

    robot_state_init(true);
    robot_state_set(ROBOT_STATE_STARTUP);

    if (commands_start() == ESP_OK) {
        app_event("ready (type 'help')");
    }

    /* ROS 2: states and door commands arrive as topics over the native USB port. */
    if (ros_interface_start() != ESP_OK) {
        ESP_LOGE(TAG, "micro-ROS FAILED to start; serial commands still work");
    }

    if (APP_DEMO_MODE) {
        demo_start(APP_DEMO_STEP_MS, APP_DEMO_MOVES_DOOR);
    }
}
