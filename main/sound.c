/*
 * sound.c — The box's sounds. See sound.h.
 */
#include <string.h>

#include "esp_log.h"

#include "audio.h"
#include "board.h"
#include "config.h"
#include "sound.h"

static const char *TAG = "sound";

extern const uint8_t gm_wav_start[] asm("_binary_gm_wav_start");
extern const uint8_t gm_wav_end[]   asm("_binary_gm_wav_end");

static const audio_note_t NOTES_STARTUP[]   = { { 523, 120 }, { 659, 120 }, { 784, 200 } };
static const audio_note_t NOTES_ARRIVED[]   = { { 784, 150 }, { 1047, 250 } };
static const audio_note_t NOTES_WARNING[]   = { { 660, 150 }, { 0, 100 }, { 660, 150 } };
static const audio_note_t NOTES_FAILURE[]   = { { 440, 300 }, { 330, 300 }, { 0, 150 },
                                                { 262, 500 } };
static const audio_note_t NOTES_DOOR_HELD[] = { { 1319, 80 }, { 0, 60 }, { 1319, 80 },
                                                { 0, 60 }, { 1319, 80 } };

#define MELODY(n)  audio_play_melody((n), sizeof(n) / sizeof((n)[0]))

esp_err_t sound_init(i2c_master_bus_handle_t bus)
{
    /* The I2S starts even without the HAT (or with AUDIO_ENABLED 0): the
     * microphone needs its clocks. */
    audio_config_t cfg = {
        .bus              = AUDIO_ENABLED ? bus : NULL,
        .address          = AUDIO_WM8960_ADDRESS,
        .i2c_speed_hz     = I2C_SPEED_HZ,
        .bclk_gpio        = PIN_I2S_BCLK,
        .ws_gpio          = PIN_I2S_WS,
        .dout_gpio        = PIN_I2S_DOUT,
        .din_gpio         = PIN_I2S_DIN,
        .sample_rate_hz   = AUDIO_SAMPLE_RATE_HZ,
        .volume_percent   = AUDIO_VOLUME_PERCENT,
        .tone_level       = AUDIO_TONE_LEVEL,
        .amp_off_after_ms = AUDIO_AMP_OFF_AFTER_MS,
        .stream_rate_hz      = SPEAKER_STREAM_RATE_HZ,
        .stream_buffer_ms    = SPEAKER_BUFFER_MS,
        .stream_prebuffer_ms = SPEAKER_PREBUFFER_MS,
    };
    return audio_init(&cfg);
}

void sound_play(sound_t s)
{
    if (!audio_speaker_available()) {
        return;
    }
    audio_stop();                           /* the newest sound wins */
    esp_err_t err = ESP_OK;
    switch (s) {
    case SOUND_STARTUP:   err = MELODY(NOTES_STARTUP);   break;
    case SOUND_ARRIVED:   err = MELODY(NOTES_ARRIVED);   break;
    case SOUND_GREETING:  err = audio_play_wav(gm_wav_start, gm_wav_end); break;
    case SOUND_WARNING:   err = MELODY(NOTES_WARNING);   break;
    case SOUND_FAILURE:   err = MELODY(NOTES_FAILURE);   break;
    case SOUND_DOOR_HELD: err = MELODY(NOTES_DOOR_HELD); break;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "sound %d not played: %s", (int)s, esp_err_to_name(err));
    }
}

static const char *const NAMES[] = {
    [SOUND_STARTUP]   = "startup",
    [SOUND_ARRIVED]   = "arrived",
    [SOUND_GREETING]  = "greeting",
    [SOUND_WARNING]   = "warning",
    [SOUND_FAILURE]   = "failure",
    [SOUND_DOOR_HELD] = "door_held",
};

bool sound_from_name(const char *name, sound_t *out)
{
    for (size_t i = 0; i < sizeof(NAMES) / sizeof(NAMES[0]); i++) {
        if (strcmp(name, NAMES[i]) == 0) {
            *out = (sound_t)i;
            return true;
        }
    }
    return false;
}

void sound_stop(void)
{
    if (audio_speaker_available()) {
        audio_stop();
    }
}

void sound_set_volume(uint8_t percent)
{
    if (audio_speaker_available()) {
        audio_set_volume(percent);
    }
}
