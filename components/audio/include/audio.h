/*
 * audio.h — WM8960 Audio HAT speaker + ICS43434 microphone on ONE I2S port.
 *
 *   GPIO14 BCLK  --+--> WM8960 P12 (BCLK)
 *                  +--> ICS43434 BCLK
 *   GPIO15 WS    --+--> WM8960 P35 (LRCLK)
 *                  +--> ICS43434 LRCL
 *   GPIO16 DOUT  -----> WM8960 P40 (DACDAT)      ESP32 -> speaker
 *   GPIO17 DIN   <----- ICS43434 DOUT            microphone -> ESP32 (SEL = GND: left)
 *
 * The codec setup is the one that worked in motor-audio-check together with
 * the door motor and the INA228 on the same I2C bus:
 *
 *   I2C  (shared with the INA228, 100 kHz)  WM8960 at 0x1A, write-only
 *   I2S  ESP32-S3 = clock master, codec = slave, Philips, stereo, 32-bit
 *        slots (the ICS43434 needs 64 BCLK per frame; the codec reads the
 *        first 16 bits of each slot, where the samples go)
 *
 * Order matters (found in motor-audio-check): the codec is configured
 * BEFORE the I2S clocks start (clocks on the wires disturbed the I2C setup),
 * and the class-D speaker amp is only switched on while something plays.
 *
 * Sounds are queued and played by a background task, so every call returns
 * at once. A sound is a WAV clip (16-bit PCM, mono or stereo, AT THE I2S
 * RATE: the microphone shares the clocks) or a tone / melody made on the fly.
 *
 * Audio can also stream in from the computer (audio_stream_push, e.g. 16 kHz
 * from micro-ROS): it is upsampled (audio_resample.h) and played.
 *
 * The microphone works even without the HAT (the ESP32 makes the clocks).
 * mic_dsp.h turns its 24-bit samples into usable 16-bit audio.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AUDIO_WM8960_ADDRESS  0x1A

typedef struct {
    i2c_master_bus_handle_t bus;       /* shared I2C bus (already created)     */
    uint8_t    address;                /* AUDIO_WM8960_ADDRESS                  */
    uint32_t   i2c_speed_hz;           /* 100 kHz: what the HAT works at        */
    gpio_num_t bclk_gpio;
    gpio_num_t ws_gpio;
    gpio_num_t dout_gpio;
    gpio_num_t din_gpio;               /* microphone data in                    */
    uint32_t   sample_rate_hz;         /* speaker AND microphone                */
    uint8_t    volume_percent;         /* 0..100, 100 = +6 dB speaker volume    */
    float      tone_level;             /* 0..1 digital level of generated tones */
    uint32_t   amp_off_after_ms;       /* amp off this long after the last sound */
    /* Audio streamed from the computer (audio_stream_push). 0 = no stream. */
    uint32_t   stream_rate_hz;         /* e.g. 16000; must divide sample_rate_hz */
    uint32_t   stream_buffer_ms;       /* buffer size, e.g. 500                 */
    uint32_t   stream_prebuffer_ms;    /* buffered before playing starts, e.g. 100 */
} audio_config_t;

/* Counters of the computer's audio stream (samples at stream_rate_hz). */
typedef struct {
    bool     playing;
    uint32_t buffered_ms;              /* waiting to be played                  */
    uint32_t received;                 /* samples that arrived                  */
    uint32_t lost;                     /* samples missing (gaps in the index)   */
    uint32_t dropped;                  /* arrived but no room / out of order    */
    uint32_t underruns;                /* the buffer ran empty mid-stream       */
} audio_stream_stats_t;

/* One note of a melody. freq_hz = 0 is a rest (silence). */
typedef struct {
    uint16_t freq_hz;
    uint16_t duration_ms;
} audio_note_t;

/** Configure the codec, then start I2S (both ways) and the playback task.
 *  ESP_ERR_NOT_FOUND if the HAT does not answer: the microphone still works,
 *  playback calls fail. Other errors: nothing works. */
esp_err_t audio_init(const audio_config_t *cfg);

/** true if the WM8960 answered (sounds can play). */
bool audio_speaker_available(void);

/** The I2S sample rate (speaker and microphone). */
uint32_t audio_sample_rate(void);

/**
 * Read microphone samples (mono, signed 24-bit values in int32_t).
 * Blocks until `count` samples are read or `timeout_ms` passes.
 */
esp_err_t audio_mic_read(int32_t *samples, size_t count, size_t *got, uint32_t timeout_ms);

/** Queue a WAV clip in flash (e.g. from EMBED_FILES). Checked when queued. */
esp_err_t audio_play_wav(const uint8_t *wav_start, const uint8_t *wav_end);

/** Queue a tone. */
esp_err_t audio_play_tone(uint16_t freq_hz, uint16_t duration_ms);

/** Queue a melody. `notes` must stay valid while it plays (static const). */
esp_err_t audio_play_melody(const audio_note_t *notes, size_t count);

/** Drop everything queued and stop the sound that is playing (also the stream). */
void audio_stop(void);

/**
 * Audio from the computer: `n` mono int16 samples at stream_rate_hz, the
 * first one being sample number `first_index` of the stream (0 = a new
 * stream). Gaps in the index are played as silence. Call from ONE task only.
 * Returns at once; playback starts after stream_prebuffer_ms.
 */
esp_err_t audio_stream_push(const int16_t *samples, size_t n, uint32_t first_index);

audio_stream_stats_t audio_stream_stats(void);

/** Speaker + headphone volume, 0 = mute, 100 = +6 dB. */
esp_err_t audio_set_volume(uint8_t percent);
uint8_t   audio_get_volume(void);

/** true while a sound plays or is queued. */
bool audio_busy(void);

#ifdef __cplusplus
}
#endif
