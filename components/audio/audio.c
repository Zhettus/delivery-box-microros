/*
 * audio.c — Full-duplex I2S: tone/melody playback + microphone capture.
 * See audio.h.
 */
#include <math.h>
#include <string.h>

#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "audio.h"
#include "wm8960.h"

static const char *TAG = "audio";

#define SLOT_BITS          32
#define DMA_DESC_NUM       6
#define DMA_FRAME_NUM      240
#define CHUNK_FRAMES       128                /* frames generated per write   */
#define RAMP_MS            5                  /* fade in/out: no clicks       */
#define QUEUE_LEN          32
#define TWO_PI_F           6.2831853f

typedef struct {
    uint16_t freq_hz;
    uint16_t duration_ms;
    uint32_t generation;   /* audio_stop() bumps it: older notes are dropped */
} tone_msg_t;

static audio_config_t    s_cfg;
static i2s_chan_handle_t s_tx, s_rx;
static bool              s_speaker_ok;
static QueueHandle_t     s_queue;
static SemaphoreHandle_t s_rx_mutex;
static volatile uint32_t s_generation;

/* ---- I2S -------------------------------------------------------------- */

/*
 * `role`: I2S_ROLE_SLAVE when the WM8960 drives BCLK/WS (normal case: one
 * clock for DAC and data), I2S_ROLE_MASTER when the codec is missing and the
 * microphone still needs clocks.
 */
static esp_err_t i2s_duplex_init(i2s_role_t role)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, role);
    chan_cfg.dma_desc_num  = DMA_DESC_NUM;
    chan_cfg.dma_frame_num = DMA_FRAME_NUM;
    chan_cfg.auto_clear    = true;            /* send silence when idle */
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &s_tx, &s_rx), TAG, "channels");

    /* ONE config for both directions: identical config = shared clocks. */
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(s_cfg.sample_rate_hz),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = s_cfg.bclk_gpio,
            .ws   = s_cfg.ws_gpio,
            .dout = s_cfg.dout_gpio,
            .din  = s_cfg.din_gpio,
            .invert_flags = { 0 },
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_tx, &std_cfg), TAG, "tx init");
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_rx, &std_cfg), TAG, "rx init");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_tx), TAG, "tx enable");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_rx), TAG, "rx enable");
    return ESP_OK;
}

/* ---- playback task ------------------------------------------------------- */

static void play_tone(uint16_t freq_hz, uint16_t duration_ms, uint32_t generation)
{
    static int32_t frames[CHUNK_FRAMES * 2];   /* stereo, 32-bit slots */

    const uint32_t fs      = s_cfg.sample_rate_hz;
    const uint32_t total   = (uint32_t)duration_ms * fs / 1000;
    const uint32_t ramp    = RAMP_MS * fs / 1000;
    const float    step    = TWO_PI_F * (float)freq_hz / (float)fs;
    const float    amp     = s_cfg.tone_amplitude * 2147483647.0f;
    float          phase   = 0.0f;

    for (uint32_t done = 0; done < total && generation == s_generation; ) {
        uint32_t n = total - done;
        if (n > CHUNK_FRAMES) {
            n = CHUNK_FRAMES;
        }
        for (uint32_t i = 0; i < n; i++, done++) {
            float env = 1.0f;                            /* click-free edges */
            if (done < ramp) {
                env = (float)done / (float)ramp;
            } else if (total - done < ramp) {
                env = (float)(total - done) / (float)ramp;
            }
            int32_t v = (freq_hz == 0) ? 0 : (int32_t)(amp * env * sinf(phase));
            frames[2 * i]     = v;                       /* left  */
            frames[2 * i + 1] = v;                       /* right */
            phase += step;
            if (phase >= TWO_PI_F) {
                phase -= TWO_PI_F;
            }
        }
        size_t written;
        i2s_channel_write(s_tx, frames, n * 2 * sizeof(int32_t), &written,
                          pdMS_TO_TICKS(1000));
    }
}

static void audio_task(void *arg)
{
    tone_msg_t msg;
    for (;;) {
        if (xQueueReceive(s_queue, &msg, portMAX_DELAY) == pdTRUE &&
            msg.generation == s_generation) {
            play_tone(msg.freq_hz, msg.duration_ms, msg.generation);
        }
    }
}

/* ---- public API ------------------------------------------------------------ */

esp_err_t audio_init(const audio_config_t *cfg)
{
    s_cfg = *cfg;
    s_rx_mutex = xSemaphoreCreateMutex();
    s_queue    = xQueueCreate(QUEUE_LEN, sizeof(tone_msg_t));
    ESP_RETURN_ON_FALSE(s_rx_mutex && s_queue, ESP_ERR_NO_MEM, TAG, "alloc");

    /* Codec first: when it answers it becomes the I2S clock master. */
    if (cfg->i2c_bus) {
        wm8960_config_t codec = {
            .bus            = cfg->i2c_bus,
            .address        = cfg->codec_address,
            .i2c_speed_hz   = cfg->codec_i2c_speed_hz,
            .mclk_hz        = cfg->codec_mclk_hz,
            .sample_rate_hz = cfg->sample_rate_hz,
            .word_bits      = SLOT_BITS,
            .volume_percent = cfg->volume_percent,
        };
        esp_err_t err = wm8960_init(&codec);
        s_speaker_ok = (err == ESP_OK);
        if (!s_speaker_ok) {
            ESP_LOGW(TAG, "speaker unavailable (%s); ESP32 drives the mic clocks",
                     esp_err_to_name(err));
        }
    }

    i2s_role_t role = s_speaker_ok ? I2S_ROLE_SLAVE : I2S_ROLE_MASTER;
    ESP_RETURN_ON_ERROR(i2s_duplex_init(role), TAG, "I2S");
    ESP_LOGI(TAG, "I2S0 full duplex (%s): BCLK=GPIO%d WS=GPIO%d DOUT=GPIO%d DIN=GPIO%d, %lu Hz",
             s_speaker_ok ? "clocks from WM8960" : "ESP32 is clock master",
             cfg->bclk_gpio, cfg->ws_gpio, cfg->dout_gpio, cfg->din_gpio,
             (unsigned long)cfg->sample_rate_hz);

    if (xTaskCreate(audio_task, "audio", 4096, NULL, 5, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

bool audio_speaker_available(void)
{
    return s_speaker_ok;
}

esp_err_t audio_play_tone(uint16_t freq_hz, uint16_t duration_ms)
{
    if (!s_speaker_ok) {
        return ESP_ERR_INVALID_STATE;
    }
    tone_msg_t msg = { freq_hz, duration_ms, s_generation };
    return xQueueSend(s_queue, &msg, 0) == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t audio_play_melody(const audio_melody_t *m)
{
    if (!s_speaker_ok) {
        return ESP_ERR_INVALID_STATE;
    }
    for (uint8_t i = 0; i < m->count; i++) {
        ESP_RETURN_ON_ERROR(audio_play_tone(m->notes[i].freq_hz, m->notes[i].duration_ms),
                            TAG, "queue full");
    }
    return ESP_OK;
}

void audio_stop(void)
{
    s_generation++;          /* the current and all queued notes are now stale */
    if (s_queue) {
        xQueueReset(s_queue);
    }
}

esp_err_t audio_set_volume(uint8_t percent)
{
    return s_speaker_ok ? wm8960_set_volume(percent) : ESP_ERR_INVALID_STATE;
}

uint8_t audio_get_volume(void)
{
    return s_speaker_ok ? wm8960_get_volume() : 0;
}

esp_err_t audio_mic_read(int32_t *samples, size_t count, size_t *got, uint32_t timeout_ms)
{
    int32_t frames[CHUNK_FRAMES * 2];
    size_t  n = 0;
    esp_err_t err = ESP_OK;

    if (!s_rx || !s_rx_mutex) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_rx_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    while (n < count) {
        size_t want = count - n;
        if (want > CHUNK_FRAMES) {
            want = CHUNK_FRAMES;
        }
        size_t bytes = 0;
        err = i2s_channel_read(s_rx, frames, want * 2 * sizeof(int32_t), &bytes,
                               pdMS_TO_TICKS(timeout_ms));
        if (err != ESP_OK) {
            break;
        }
        size_t frames_read = bytes / (2 * sizeof(int32_t));
        for (size_t i = 0; i < frames_read; i++) {
            samples[n++] = frames[2 * i] >> 8;   /* left slot, 24-bit signed */
        }
    }
    xSemaphoreGive(s_rx_mutex);

    if (got) {
        *got = n;
    }
    return err;
}

esp_err_t audio_mic_level(uint32_t window_ms, float *rms_dbfs, float *peak_dbfs)
{
    const float full_scale = 8388608.0f;   /* 2^23 */
    int32_t buf[256];

    /* Throw away what piled up in the DMA buffers while nobody listened. */
    size_t got;
    uint32_t stale = DMA_DESC_NUM * DMA_FRAME_NUM;
    while (stale > 0) {
        size_t n = stale > 256 ? 256 : stale;
        ESP_RETURN_ON_ERROR(audio_mic_read(buf, n, &got, 200), TAG, "mic read");
        stale -= n;
    }

    uint32_t total = window_ms * s_cfg.sample_rate_hz / 1000;
    double   sum   = 0.0;
    int32_t  peak  = 0;
    for (uint32_t done = 0; done < total; ) {
        size_t n = (total - done) > 256 ? 256 : (total - done);
        ESP_RETURN_ON_ERROR(audio_mic_read(buf, n, &got, 200), TAG, "mic read");
        for (size_t i = 0; i < got; i++) {
            int32_t a = buf[i] < 0 ? -buf[i] : buf[i];
            if (a > peak) {
                peak = a;
            }
            sum += (double)buf[i] * (double)buf[i];
        }
        done += got;
    }

    float rms = (float)sqrt(sum / (double)(total ? total : 1));
    if (rms_dbfs) {
        *rms_dbfs = 20.0f * log10f(fmaxf(rms, 1.0f) / full_scale);
    }
    if (peak_dbfs) {
        *peak_dbfs = 20.0f * log10f(fmaxf((float)peak, 1.0f) / full_scale);
    }
    return ESP_OK;
}
