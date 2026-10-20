/*
 * mic_stream.h — Microphone capture for streaming (from mic_ros_handoff).
 *
 *   I2S (AUDIO_SAMPLE_RATE_HZ) -> mic_dsp (high-pass, AGC, limiter)
 *   -> decimation to MIC_STREAM_RATE_HZ -> 160-sample chunks (10 ms)
 *
 * The capture task runs only while enabled. ros_interface.c takes the chunks
 * and publishes them; `offset` lets the receiver find lost chunks.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define MIC_CHUNK_SAMPLES 160

typedef struct {
    uint32_t offset;                       /* first sample's index in the stream */
    int16_t  samples[MIC_CHUNK_SAMPLES];
} mic_chunk_t;

/** Create the buffer and the capture task (off). The audio I2S must run. */
esp_err_t mic_stream_start(void);

/** Switch capture on (fresh stream, index 0) or off. Drops waiting chunks. */
void mic_stream_enable(bool on);
bool mic_stream_enabled(void);

/** Next chunk, waiting up to `timeout_ms`. false if none. */
bool mic_stream_take(mic_chunk_t *chunk, uint32_t timeout_ms);

/** Chunks made / dropped because nobody took them in time. */
void mic_stream_counters(uint32_t *captured, uint32_t *dropped);
