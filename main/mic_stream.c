/*
 * mic_stream.c — Microphone capture. See mic_stream.h. The capture loop is
 * the one from mic_ros_handoff/component_tests/mic_ros/main/mic_ros_test.c.
 */
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/message_buffer.h"
#include "freertos/task.h"

#include "audio.h"
#include "config.h"
#include "mic_dsp.h"
#include "mic_stream.h"

static const char *TAG = "mic";

#define MIC_RATE_HZ       AUDIO_SAMPLE_RATE_HZ
#define DECIM_FACTOR      (MIC_RATE_HZ / MIC_STREAM_RATE_HZ)
#define READ_FRAMES       (MIC_CHUNK_SAMPLES * DECIM_FACTOR)
#define STALE_FRAMES      2048            /* thrown away when the mic is switched on */
#define DECIM_CUTOFF_HZ   7000.0f
#define BUFFER_BYTES      (16 * 1024)     /* ~0.5 s of chunks between capture and ros */

_Static_assert(MIC_STREAM_RATE_HZ == 16000, "listen_live.py plays 16 kHz");
_Static_assert(MIC_RATE_HZ % MIC_STREAM_RATE_HZ == 0,
               "AUDIO_SAMPLE_RATE_HZ must be a multiple of MIC_STREAM_RATE_HZ");

static MessageBufferHandle_t s_chunks;
static volatile bool         s_enabled;
static volatile uint32_t     s_dropped;
static volatile uint32_t     s_captured;

static void flush(void)
{
    mic_chunk_t discard;
    while (xMessageBufferReceive(s_chunks, &discard, sizeof(discard), 0) > 0) {
    }
}

static void capture_task(void *arg)
{
    static int32_t in24[READ_FRAMES];
    static int16_t proc[READ_FRAMES];
    static int16_t dec[READ_FRAMES / DECIM_FACTOR + 1];
    mic_dsp_t     dsp;
    mic_decim_t   decim;
    mic_chunk_t   chunk;
    size_t        fill = 0;
    uint32_t      stream_pos = 0;
    bool          was_enabled = false;

    mic_dsp_config_t cfg;
    mic_dsp_default_config(&cfg, MIC_RATE_HZ);
    cfg.mode    = MIC_PROC_MODE;
    cfg.gain_db = MIC_GAIN_DB;
    cfg.hpf_hz  = MIC_HPF_HZ;

    for (;;) {
        if (!s_enabled) {
            was_enabled = false;
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (!was_enabled) {                    /* just switched on: fresh state */
            was_enabled = true;
            mic_dsp_init(&dsp, &cfg);
            mic_decim_init(&decim, DECIM_CUTOFF_HZ, MIC_RATE_HZ, DECIM_FACTOR);
            fill = 0;
            stream_pos = 0;
            for (size_t left = STALE_FRAMES; left > 0; ) {   /* old DMA content */
                size_t want = left > READ_FRAMES ? READ_FRAMES : left, got = 0;
                if (audio_mic_read(in24, want, &got, 100) != ESP_OK || got == 0) {
                    break;
                }
                left -= got;
            }
        }

        size_t got = 0;
        esp_err_t err = audio_mic_read(in24, READ_FRAMES, &got, 100);
        if (err != ESP_OK || got == 0) {
            ESP_LOGW(TAG, "mic read: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        mic_dsp_process(&dsp, in24, proc, got);
        size_t n = mic_decim_process(&decim, proc, got, dec);

        for (size_t i = 0; i < n; i++) {
            if (fill == 0) {
                chunk.offset = stream_pos;
            }
            chunk.samples[fill++] = dec[i];
            if (fill == MIC_CHUNK_SAMPLES) {
                s_captured++;
                /* whole message or nothing: the position still advances, so the
                 * receiver sees the gap in the offset */
                if (xMessageBufferSend(s_chunks, &chunk, sizeof(chunk), 0) != sizeof(chunk)) {
                    s_dropped++;
                }
                stream_pos += MIC_CHUNK_SAMPLES;
                fill = 0;
            }
        }
    }
}

esp_err_t mic_stream_start(void)
{
    if (audio_sample_rate() != MIC_RATE_HZ) {
        ESP_LOGE(TAG, "the I2S is not running at %d Hz: no microphone", MIC_RATE_HZ);
        return ESP_ERR_INVALID_STATE;
    }
    s_chunks = xMessageBufferCreate(BUFFER_BYTES);
    if (!s_chunks) {
        return ESP_ERR_NO_MEM;
    }
    /* Core 1, below the door control (10) and its current sensing (9). */
    if (xTaskCreatePinnedToCore(capture_task, "mic", 8192, NULL, 6, NULL, 1) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "ready: %d Hz -> %d Hz, 10 ms chunks (off until enabled)",
             MIC_RATE_HZ, MIC_STREAM_RATE_HZ);
    return ESP_OK;
}

void mic_stream_enable(bool on)
{
    if (!s_chunks || on == s_enabled) {
        return;
    }
    if (on) {
        flush();
        s_enabled = true;
    } else {
        s_enabled = false;
        flush();
    }
    ESP_LOGI(TAG, "microphone %s", on ? "ON" : "OFF");
}

bool mic_stream_enabled(void)
{
    return s_enabled;
}

bool mic_stream_take(mic_chunk_t *chunk, uint32_t timeout_ms)
{
    if (!s_chunks) {
        vTaskDelay(pdMS_TO_TICKS(timeout_ms));
        return false;
    }
    return xMessageBufferReceive(s_chunks, chunk, sizeof(*chunk),
                                 pdMS_TO_TICKS(timeout_ms)) == sizeof(*chunk);
}

void mic_stream_counters(uint32_t *captured, uint32_t *dropped)
{
    *captured = s_captured;
    *dropped  = s_dropped;
}
