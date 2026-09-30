/*
 * encoder.c — Hall quadrature encoder on PCNT. See encoder.h.
 */
#include "driver/pulse_cnt.h"
#include "esp_check.h"
#include "esp_log.h"

#include "encoder.h"

static const char *TAG = "encoder";

#define PCNT_LIMIT        30000
#define GLITCH_FILTER_NS  1000    /* ignore spikes shorter than 1 us */

static pcnt_unit_handle_t s_unit;
static bool               s_invert;

esp_err_t encoder_init(gpio_num_t a, gpio_num_t b, bool invert)
{
    s_invert = invert;

    pcnt_unit_config_t unit_cfg = {
        .high_limit = PCNT_LIMIT,
        .low_limit  = -PCNT_LIMIT,
        .flags.accum_count = 1,     /* keep counting past the 16-bit limits */
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_cfg, &s_unit), TAG, "unit");

    pcnt_glitch_filter_config_t filter = { .max_glitch_ns = GLITCH_FILTER_NS };
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(s_unit, &filter), TAG, "filter");

    /* Channel A: count A's edges, direction from B's level. */
    pcnt_chan_config_t ca = { .edge_gpio_num = a, .level_gpio_num = b };
    pcnt_channel_handle_t ch_a;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(s_unit, &ca, &ch_a), TAG, "channel A");
    pcnt_channel_set_edge_action(ch_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(ch_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    /* Channel B: count B's edges, direction from A's level. */
    pcnt_chan_config_t cb = { .edge_gpio_num = b, .level_gpio_num = a };
    pcnt_channel_handle_t ch_b;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(s_unit, &cb, &ch_b), TAG, "channel B");
    pcnt_channel_set_edge_action(ch_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(ch_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    /* Accumulation happens in the limit watch-point interrupts. */
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(s_unit, PCNT_LIMIT), TAG, "wp+");
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(s_unit, -PCNT_LIMIT), TAG, "wp-");

    ESP_RETURN_ON_ERROR(pcnt_unit_enable(s_unit), TAG, "enable");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(s_unit), TAG, "clear");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(s_unit), TAG, "start");

    ESP_LOGI(TAG, "ready: A=GPIO%d B=GPIO%d%s", a, b, invert ? " (inverted)" : "");
    return ESP_OK;
}

esp_err_t encoder_init_pulses(gpio_num_t pin)
{
    s_invert = false;

    pcnt_unit_config_t unit_cfg = {
        .high_limit = PCNT_LIMIT,
        .low_limit  = -PCNT_LIMIT,
        .flags.accum_count = 1,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_cfg, &s_unit), TAG, "unit");

    pcnt_glitch_filter_config_t filter = { .max_glitch_ns = GLITCH_FILTER_NS };
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(s_unit, &filter), TAG, "filter");

    /* Count rising edges only; the driver enables the pin's pull-up. */
    pcnt_chan_config_t cc = { .edge_gpio_num = pin, .level_gpio_num = -1 };
    pcnt_channel_handle_t ch;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(s_unit, &cc, &ch), TAG, "channel");
    pcnt_channel_set_edge_action(ch, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                 PCNT_CHANNEL_EDGE_ACTION_HOLD);

    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(s_unit, PCNT_LIMIT), TAG, "wp+");
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(s_unit), TAG, "enable");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(s_unit), TAG, "clear");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(s_unit), TAG, "start");

    ESP_LOGI(TAG, "ready: pulse input GPIO%d (rising edges)", pin);
    return ESP_OK;
}

int32_t encoder_get_count(void)
{
    int count = 0;
    pcnt_unit_get_count(s_unit, &count);
    return s_invert ? -count : count;
}
