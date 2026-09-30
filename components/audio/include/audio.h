/*
 * audio.h — Speaker (WM8960) + microphone (ICS43434) on ONE full-duplex I2S port.
 *
 *   GPIO14 BCLK  --+--> WM8960 P12 (BCLK)
 *                  +--> ICS43434 BCLK
 *   GPIO15 WS    --+--> WM8960 P35 (LRCLK)
 *                  +--> ICS43434 LRCL
 *   GPIO16 DOUT  -----> WM8960 P40 (DACDAT)      ESP32 -> speaker
 *   GPIO17 DIN   <----- ICS43434 DOUT            microphone -> ESP32
 *
 * Clocks: the WM8960 is the I2S clock MASTER (from its own crystal), and the
 * ESP32 and the ICS43434 follow its BCLK/WS. The DAC and the samples therefore
 * run on one clock (no drift clicks). If the WM8960 is missing, the ESP32
 * becomes clock master so the microphone keeps working; playback calls then
 * return ESP_ERR_INVALID_STATE.
 *
 * Both directions MUST use an identical I2S configuration, otherwise the
 * ESP-IDF driver cannot share the clocks (full duplex): 32-bit stereo slots,
 * Philips I2S. The ICS43434 (SEL = GND) sends 24-bit samples in the LEFT slot.
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

typedef struct {
    gpio_num_t bclk_gpio;
    gpio_num_t ws_gpio;
    gpio_num_t dout_gpio;
    gpio_num_t din_gpio;
    uint32_t   sample_rate_hz;

    /* WM8960 codec (on the shared I2C bus). Set i2c_bus = NULL to skip it. */
    i2c_master_bus_handle_t i2c_bus;
    uint8_t    codec_address;
    uint32_t   codec_i2c_speed_hz;
    uint32_t   codec_mclk_hz;
    uint8_t    volume_percent;
    float      tone_amplitude;     /* 0..1 digital level of generated tones */
} audio_config_t;

/* One note of a melody. freq_hz = 0 is a rest (silence). */
typedef struct {
    uint16_t freq_hz;
    uint16_t duration_ms;
} audio_note_t;

typedef struct {
    const audio_note_t *notes;
    uint8_t             count;
} audio_melody_t;

#define AUDIO_MELODY(arr)  ((audio_melody_t){ (arr), (uint8_t)(sizeof(arr) / sizeof((arr)[0])) })

esp_err_t audio_init(const audio_config_t *cfg);

bool audio_speaker_available(void);

/** Queue a melody (non-blocking). `m->notes` must stay valid (use static const). */
esp_err_t audio_play_melody(const audio_melody_t *m);

/** Queue a single tone (non-blocking). */
esp_err_t audio_play_tone(uint16_t freq_hz, uint16_t duration_ms);

/** Drop everything queued and stop the current sound. */
void audio_stop(void);

esp_err_t audio_set_volume(uint8_t percent);
uint8_t   audio_get_volume(void);

/**
 * Read microphone samples (mono, signed 24-bit values in int32_t).
 * Blocks until `count` samples are read or `timeout_ms` passes.
 */
esp_err_t audio_mic_read(int32_t *samples, size_t count, size_t *got, uint32_t timeout_ms);

/** Measure the microphone RMS level over `window_ms`, in dBFS (0 = full scale). */
esp_err_t audio_mic_level(uint32_t window_ms, float *rms_dbfs, float *peak_dbfs);

#ifdef __cplusplus
}
#endif
