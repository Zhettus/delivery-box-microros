/*
 * audio.c — WM8960 playback + ICS43434 capture on one I2S port. See audio.h.
 *
 * The register values, the retried setup writes and the order (codec, then
 * I2S clocks, amp only while playing) come from motor-audio-check/main/main.c.
 * The microphone read comes from the mic hand-off (mic_ros_handoff).
 */
#include <math.h>
#include <string.h>

#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"

#include "audio.h"
#include "audio_resample.h"

static const char *TAG = "audio";

#define AUDIO_TASK_PRIORITY  5        /* below the door (10), its sensing (9), mic (6) */
#define AUDIO_QUEUE_LEN      8
#define FRAMES_PER_CHUNK     256      /* stereo 32-bit frames per I2S transfer   */
#define TONE_FADE_MS         5        /* fade in/out of a tone: no clicks        */
#define AMP_ON_SETTLE_MS     20
#define DMA_DESC_NUM         6        /* as the mic hand-off                     */
#define DMA_FRAME_NUM        240
#define STREAM_CHUNK         160      /* samples taken from the stream at a time */
#define STREAM_WAIT_MS       20       /* wait for stream data before counting a gap */
#define STREAM_END_MS        300      /* no data this long: the stream has ended */
#define STREAM_MAX_GAP_S     1        /* longer gaps are not filled with silence */
#define STREAM_CUTOFF_HZ     7000.0f

/* ---- WM8960 ----------------------------------------------------------------- */

#define R_LOUT1_VOL     0x02
#define R_ROUT1_VOL     0x03
#define R_LSPK_VOL      0x28
#define R_RSPK_VOL      0x29
#define R_CLASSD_CTRL1  0x31
#define WM_CLASSD_ON    0x0F7         /* both speaker outputs on                  */
#define WM_CLASSD_OFF   0x037         /* reset value: speaker outputs disabled    */
#define WM_VOL_UPDATE   (1 << 8)
#define WM_VOL_ZC       (1 << 7)
#define WM_SPK_VOL_MIN  0x30          /* -73 dB; below this is mute               */
#define WM_SPK_VOL_MAX  0x7F          /* +6 dB                                     */

static const struct { uint8_t reg; uint16_t val; } WM_INIT[] = {
    { 0x0F, 0x000 },  /* reset                        */
    { 0x19, 0x0C0 },  /* PWR1: VREF + VMID            */
    { 0x1A, 0x1F8 },  /* PWR2: DAC + outputs + spk    */
    { 0x2F, 0x00C },  /* PWR3: output mixers          */
    { 0x04, 0x000 },  /* Clocking1: MCLK direct       */
    { 0x08, 0x1C0 },  /* Clocking2: class-D clock     */
    { 0x07, 0x002 },  /* IFACE1: I2S 16-bit, slave    */
    { 0x05, 0x000 },  /* DAC un-mute                  */
    { 0x0A, 0x1FF },  /* LDACVOL 0 dB                 */
    { 0x0B, 0x1FF },  /* RDACVOL 0 dB                 */
    { 0x22, 0x100 },  /* LOUTMIX: DAC -> mixer        */
    { 0x25, 0x100 },  /* ROUTMIX: DAC -> mixer        */
    { 0x28, 0x1FF },  /* SPKLVOL +6 dB                */
    { 0x29, 0x1FF },  /* SPKRVOL +6 dB                */
    { 0x02, 0x179 },  /* LOUT1VOL 0 dB (headphone)    */
    { 0x03, 0x179 },  /* ROUT1VOL 0 dB (headphone)    */
    { 0x33, 0x02D },  /* Class-D DC/AC gain (~+5 dB)  */
    /* The class-D amp is switched on only while playing (amp_set). */
};

static i2c_master_dev_handle_t s_wm;
static uint8_t                 s_volume = 100;

static esp_err_t wm_write_once(uint8_t reg, uint16_t val)
{
    uint8_t buf[2] = { (uint8_t)((reg << 1) | ((val >> 8) & 1)), (uint8_t)(val & 0xFF) };
    return i2c_master_transmit(s_wm, buf, sizeof(buf), 100);
}

/* Retried: a first attempt failing now and then is normal on this bus. */
static esp_err_t wm_write(uint8_t reg, uint16_t val)
{
    esp_err_t err = ESP_FAIL;
    for (int attempt = 1; attempt <= 5; ++attempt) {
        if ((err = wm_write_once(reg, val)) == ESP_OK) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    ESP_LOGE(TAG, "WM8960 reg 0x%02X not written (%s)", reg, esp_err_to_name(err));
    return err;
}

static esp_err_t codec_init(const audio_config_t *cfg)
{
    ESP_RETURN_ON_ERROR(i2c_master_probe(cfg->bus, cfg->address, 100), TAG,
                        "WM8960 not found at 0x%02X (HAT connected and powered?)", cfg->address);
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = cfg->address,
        .scl_speed_hz    = cfg->i2c_speed_hz,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(cfg->bus, &dev_cfg, &s_wm), TAG, "add WM8960");

    int failed = 0;
    for (size_t i = 0; i < sizeof(WM_INIT) / sizeof(WM_INIT[0]); ++i) {
        failed += wm_write(WM_INIT[i].reg, WM_INIT[i].val) != ESP_OK;
        uint8_t r = WM_INIT[i].reg;
        vTaskDelay(pdMS_TO_TICKS(r == 0x0F || r == 0x19 || r == 0x1A ? 20 : 2));
    }
    if (failed) {
        ESP_LOGW(TAG, "WM8960 configured WITH %d failed writes: sound may be wrong", failed);
    }
    return ESP_OK;
}

static void amp_set(bool on)
{
    wm_write(R_CLASSD_CTRL1, on ? WM_CLASSD_ON : WM_CLASSD_OFF);
}

esp_err_t audio_set_volume(uint8_t percent)
{
    ESP_RETURN_ON_FALSE(s_wm != NULL, ESP_ERR_INVALID_STATE, TAG, "audio_init first");
    if (percent > 100) {
        percent = 100;
    }
    uint16_t v = 0;                                   /* 0 % = mute */
    if (percent > 0) {
        v = WM_VOL_ZC | (uint16_t)(WM_SPK_VOL_MIN +
                                   ((WM_SPK_VOL_MAX - WM_SPK_VOL_MIN) * percent + 50) / 100);
    }
    /* Left without, right with the update bit: both change together. */
    ESP_RETURN_ON_ERROR(wm_write(R_LSPK_VOL, v), TAG, "speaker L");
    ESP_RETURN_ON_ERROR(wm_write(R_RSPK_VOL, v | WM_VOL_UPDATE), TAG, "speaker R");
    s_volume = percent;
    return ESP_OK;
}

uint8_t audio_get_volume(void)
{
    return s_volume;
}

/* ---- WAV --------------------------------------------------------------------- */

typedef struct {
    const uint8_t *pcm;
    size_t         len;
    uint32_t       rate;
    uint16_t       channels;
} wav_t;

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* 16-bit PCM, mono or stereo. */
static bool parse_wav(const uint8_t *start, const uint8_t *end, wav_t *w)
{
    if (end - start < 12 || memcmp(start, "RIFF", 4) || memcmp(start + 8, "WAVE", 4)) {
        return false;
    }
    bool fmt = false;
    const uint8_t *p = start + 12;
    while (p + 8 <= end) {
        uint32_t sz = rd32(p + 4);
        if (!memcmp(p, "fmt ", 4) && sz >= 16) {
            w->channels = rd16(p + 10);
            w->rate     = rd32(p + 12);
            fmt = rd16(p + 8) == 1 && rd16(p + 22) == 16 &&
                  (w->channels == 1 || w->channels == 2);
        } else if (!memcmp(p, "data", 4)) {
            w->pcm = p + 8;
            w->len = (p + 8 + sz <= end) ? sz : (size_t)(end - (p + 8));
            return fmt;
        }
        p += 8 + ((sz + 1) & ~1u);
    }
    return false;
}

/* ---- I2S (full duplex) + playback task ---------------------------------------------- */

typedef enum { ITEM_WAV, ITEM_TONE, ITEM_MELODY } item_type_t;

typedef struct {
    item_type_t type;
    wav_t       wav;
    uint16_t    freq_hz, duration_ms;          /* ITEM_TONE   */
    const audio_note_t *notes;                 /* ITEM_MELODY */
    size_t      count;
} item_t;

static audio_config_t     s_cfg;
static i2s_chan_handle_t  s_tx, s_rx;
static SemaphoreHandle_t  s_rx_mutex;
static QueueHandle_t      s_queue;             /* NULL = no speaker */
static uint32_t           s_rate;
static volatile uint32_t  s_generation;        /* audio_stop() bumps it */
static volatile bool      s_playing;
static int32_t            s_buf[FRAMES_PER_CHUNK * 2];

/* Speaker stream from the computer: ros task (writer) -> audio task (reader). */
static StreamBufferHandle_t s_stream;          /* int16 samples at stream_rate_hz */
static size_t             s_stream_prebuf;     /* bytes needed before playing     */
static uint32_t           s_stream_next;       /* next expected sample index      */
static bool               s_stream_seen;       /* s_stream_next is valid          */
static volatile bool      s_stream_flush;      /* audio_stop(): reader empties it */
static volatile bool      s_streaming;
static audio_stream_stats_t s_stats;

/*
 * One I2S port, both directions, ESP32 = clock master (as in motor-audio-check
 * and in the mic hand-off when the codec is not the master). Both directions
 * MUST use the same config to share BCLK/WS. 32-bit slots: the ICS43434 needs
 * 64 BCLK per frame. The WM8960 is set to 16-bit words and takes the first 16
 * bits of each slot, which is where the samples are written.
 */
static esp_err_t i2s_start(uint32_t rate)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num  = DMA_DESC_NUM;
    chan_cfg.dma_frame_num = DMA_FRAME_NUM;
    chan_cfg.auto_clear    = true;             /* silence when nothing is written */
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &s_tx, &s_rx), TAG, "I2S channels");
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED, .bclk = s_cfg.bclk_gpio, .ws = s_cfg.ws_gpio,
            .dout = s_cfg.dout_gpio, .din = s_cfg.din_gpio,
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_tx, &std_cfg), TAG, "I2S tx");
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_rx, &std_cfg), TAG, "I2S rx");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_tx), TAG, "I2S tx enable");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_rx), TAG, "I2S rx enable");
    s_rate = rate;
    return ESP_OK;
}

static bool stopped(uint32_t gen)
{
    return gen != s_generation;
}

/* 16-bit sample into the top of a 32-bit slot. */
static int32_t slot(int16_t v)
{
    return (int32_t)((uint32_t)(uint16_t)v << 16);
}

static void write_frames(size_t frames)
{
    size_t written = 0;
    i2s_channel_write(s_tx, s_buf, frames * 2 * sizeof(int32_t), &written, pdMS_TO_TICKS(500));
}

static void play_wav(const wav_t *w, uint32_t gen)
{
    size_t frame_bytes = w->channels * sizeof(int16_t);
    size_t frames      = w->len / frame_bytes;
    for (size_t f = 0; f < frames && !stopped(gen);) {
        size_t n = frames - f < FRAMES_PER_CHUNK ? frames - f : FRAMES_PER_CHUNK;
        const uint8_t *src = w->pcm + f * frame_bytes;
        for (size_t i = 0; i < n; i++) {
            int16_t l = (int16_t)rd16(src + i * frame_bytes);
            int16_t r = w->channels == 2 ? (int16_t)rd16(src + i * frame_bytes + 2) : l;
            s_buf[2 * i]     = slot(l);
            s_buf[2 * i + 1] = slot(r);
        }
        write_frames(n);
        f += n;
    }
}

/* Fade-in/out gain of frame `f` of `total` (no clicks at the edges). */
static float tone_envelope(size_t f, size_t total, size_t fade)
{
    if (f < fade) {
        return (float)f / (float)fade;
    }
    if (total - f < fade) {
        return (float)(total - f) / (float)fade;
    }
    return 1.0f;
}

/* Kept out of line: inlined into play_tone() it crashes the Xtensa GCC
 * 13.2 of IDF v5.2.7 (internal compiler error). */
static __attribute__((noinline)) int16_t tone_sample(float amp, float phase)
{
    return (int16_t)lrintf(amp * sinf(phase));
}

static void play_tone(uint16_t freq_hz, uint16_t duration_ms, uint32_t gen)
{
    const float two_pi = 2.0f * (float)M_PI;
    size_t total = (size_t)s_rate * duration_ms / 1000;
    size_t fade  = (size_t)s_rate * TONE_FADE_MS / 1000;
    float  level = freq_hz ? s_cfg.tone_level * 32767.0f : 0.0f;   /* 0 Hz = rest */
    float  step  = two_pi * (float)freq_hz / (float)s_rate;
    float  phase = 0.0f;

    for (size_t f = 0; f < total && !stopped(gen);) {
        size_t n = total - f < FRAMES_PER_CHUNK ? total - f : FRAMES_PER_CHUNK;
        for (size_t i = 0; i < n; i++) {
            int32_t v = slot(tone_sample(level * tone_envelope(f + i, total, fade), phase));
            s_buf[2 * i]     = v;
            s_buf[2 * i + 1] = v;
            phase += step;
            if (phase > two_pi) {
                phase -= two_pi;
            }
        }
        write_frames(n);
        f += n;
    }
}

static bool s_amp_on;

static void amp_on(void)
{
    if (!s_amp_on) {
        amp_set(true);
        s_amp_on = true;
        vTaskDelay(pdMS_TO_TICKS(AMP_ON_SETTLE_MS));
    }
}

/*
 * One step of the computer's audio stream. Starts once stream_prebuffer_ms
 * are buffered (absorbs USB/ROS jitter), plays 10 ms at a time, and ends
 * after STREAM_END_MS without data. Returns true while the stream is active.
 */
static bool stream_step(void)
{
    static int16_t        in[STREAM_CHUNK];
    static int16_t        up[STREAM_CHUNK * 6];
    static audio_interp_t interp;
    static TickType_t     last_data;
    static bool           starved;
    const uint32_t factor = s_rate / s_cfg.stream_rate_hz;

    if (s_stream_flush) {
        xStreamBufferReset(s_stream);
        s_stream_flush = false;
        s_streaming    = false;
        return false;
    }
    if (!s_streaming) {
        if (xStreamBufferBytesAvailable(s_stream) < s_stream_prebuf) {
            return false;
        }
        audio_interp_init(&interp, STREAM_CUTOFF_HZ, s_rate, factor);
        amp_on();
        s_streaming = true;
        starved     = false;
        last_data   = xTaskGetTickCount();
    }

    size_t bytes = xStreamBufferReceive(s_stream, in, sizeof(in), pdMS_TO_TICKS(STREAM_WAIT_MS));
    size_t n     = bytes / sizeof(int16_t);
    if (n == 0) {
        if (!starved) {
            starved = true;                     /* the computer fell behind */
            s_stats.underruns++;
        }
        if (xTaskGetTickCount() - last_data >= pdMS_TO_TICKS(STREAM_END_MS)) {
            s_streaming = false;                /* finished (or the computer stopped) */
            s_stats.underruns--;                /* the end is not an underrun */
        }
        return s_streaming;
    }
    starved   = false;
    last_data = xTaskGetTickCount();
    size_t m = audio_interp_process(&interp, in, n, up);
    for (size_t done = 0; done < m;) {
        size_t k = m - done < FRAMES_PER_CHUNK ? m - done : FRAMES_PER_CHUNK;
        for (size_t i = 0; i < k; i++) {
            int32_t v = slot(up[done + i]);
            s_buf[2 * i]     = v;
            s_buf[2 * i + 1] = v;
        }
        write_frames(k);
        done += k;
    }
    return true;
}

static void audio_task(void *arg)
{
    TickType_t last_end = 0;
    item_t     it;

    for (;;) {
        /* Built-in sounds first; while the computer streams, do not wait. */
        if (xQueueReceive(s_queue, &it, s_streaming ? 0 : pdMS_TO_TICKS(10)) != pdTRUE) {
            if (s_stream && stream_step()) {
                last_end = xTaskGetTickCount();
                continue;
            }
            if (s_amp_on && xTaskGetTickCount() - last_end >= pdMS_TO_TICKS(s_cfg.amp_off_after_ms)) {
                amp_set(false);
                s_amp_on = false;
            }
            continue;
        }
        uint32_t gen = s_generation;
        s_playing = true;
        amp_on();
        switch (it.type) {
        case ITEM_WAV:
            play_wav(&it.wav, gen);
            break;
        case ITEM_TONE:
            play_tone(it.freq_hz, it.duration_ms, gen);
            break;
        case ITEM_MELODY:
            for (size_t i = 0; i < it.count && !stopped(gen); i++) {
                play_tone(it.notes[i].freq_hz, it.notes[i].duration_ms, gen);
            }
            break;
        }
        s_playing = false;
        last_end  = xTaskGetTickCount();
    }
}

/* ---- public ------------------------------------------------------------------------- */

esp_err_t audio_init(const audio_config_t *cfg)
{
    s_cfg = *cfg;
    if (s_cfg.amp_off_after_ms == 0) {
        s_cfg.amp_off_after_ms = 500;
    }
    s_rx_mutex = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_rx_mutex != NULL, ESP_ERR_NO_MEM, TAG, "mutex");

    /* 1. Codec, before any I2S clock is on the wires. Without it the I2S
     *    still starts: the microphone needs the clocks. */
    bool speaker = cfg->bus != NULL && codec_init(cfg) == ESP_OK;
    if (speaker && cfg->volume_percent != 100) {
        audio_set_volume(cfg->volume_percent);
    }

    /* 2. I2S both ways (the ESP32 drives BCLK/WS), then the playback task. */
    ESP_RETURN_ON_ERROR(i2s_start(cfg->sample_rate_hz), TAG, "I2S");
    if (speaker) {
        s_queue = xQueueCreate(AUDIO_QUEUE_LEN, sizeof(item_t));
        ESP_RETURN_ON_FALSE(s_queue != NULL, ESP_ERR_NO_MEM, TAG, "queue");
        if (cfg->stream_rate_hz > 0) {
            ESP_RETURN_ON_FALSE(s_rate % cfg->stream_rate_hz == 0 &&
                                s_rate / cfg->stream_rate_hz <= 6, ESP_ERR_INVALID_ARG, TAG,
                                "stream rate must divide the I2S rate (at most 1:6)");
            size_t bytes_per_ms = cfg->stream_rate_hz * sizeof(int16_t) / 1000;
            s_stream = xStreamBufferCreate(cfg->stream_buffer_ms * bytes_per_ms, 1);
            ESP_RETURN_ON_FALSE(s_stream != NULL, ESP_ERR_NO_MEM, TAG, "stream buffer");
            s_stream_prebuf = cfg->stream_prebuffer_ms * bytes_per_ms;
        }
        ESP_RETURN_ON_FALSE(xTaskCreate(audio_task, "audio", 4096, NULL, AUDIO_TASK_PRIORITY,
                                        NULL) == pdPASS, ESP_ERR_NO_MEM, TAG, "task");
    }

    ESP_LOGI(TAG, "I2S %lu Hz, 32-bit slots, ESP32 clock master: speaker %s (volume %u%%), "
                  "mic on GPIO%d, stream from the computer %lu Hz", (unsigned long)s_rate,
             speaker ? "OK" : "MISSING", s_volume, cfg->din_gpio,
             (unsigned long)(s_stream ? cfg->stream_rate_hz : 0));
    return speaker ? ESP_OK : ESP_ERR_NOT_FOUND;
}

bool audio_speaker_available(void)
{
    return s_queue != NULL;
}

static esp_err_t enqueue(const item_t *it)
{
    ESP_RETURN_ON_FALSE(s_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "no speaker");
    return xQueueSend(s_queue, it, 0) == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t audio_play_wav(const uint8_t *wav_start, const uint8_t *wav_end)
{
    item_t it = { .type = ITEM_WAV };
    ESP_RETURN_ON_FALSE(parse_wav(wav_start, wav_end, &it.wav), ESP_ERR_INVALID_ARG, TAG,
                        "not a 16-bit mono/stereo PCM WAV");
    /* The microphone shares the clocks, so the rate cannot follow the clip. */
    ESP_RETURN_ON_FALSE(it.wav.rate == s_rate, ESP_ERR_INVALID_ARG, TAG,
                        "WAV is %lu Hz, the I2S runs at %lu Hz: convert the file",
                        (unsigned long)it.wav.rate, (unsigned long)s_rate);
    return enqueue(&it);
}

esp_err_t audio_play_tone(uint16_t freq_hz, uint16_t duration_ms)
{
    item_t it = { .type = ITEM_TONE, .freq_hz = freq_hz, .duration_ms = duration_ms };
    return enqueue(&it);
}

esp_err_t audio_play_melody(const audio_note_t *notes, size_t count)
{
    item_t it = { .type = ITEM_MELODY, .notes = notes, .count = count };
    return enqueue(&it);
}

void audio_stop(void)
{
    if (s_queue) {
        xQueueReset(s_queue);
        s_generation++;
    }
    if (s_stream) {
        s_stream_flush = true;                 /* the audio task empties it */
        s_stream_seen  = false;
    }
}

/* ---- speaker stream from the computer -------------------------------------------- */

esp_err_t audio_stream_push(const int16_t *samples, size_t n, uint32_t first_index)
{
    if (!s_stream) {
        return ESP_ERR_INVALID_STATE;
    }
    s_stats.received += n;

    /* A new stream (index 0, or far behind what we expected): start over. */
    if (!s_stream_seen || first_index == 0 ||
        first_index + s_cfg.stream_rate_hz * STREAM_MAX_GAP_S < s_stream_next) {
        s_stream_next = first_index;
        s_stream_seen = true;
    }
    if (first_index < s_stream_next) {          /* old or repeated chunk */
        s_stats.dropped += n;
        return ESP_OK;
    }

    /* Lost chunks: silence in their place (keeps the timing), up to 1 s. */
    uint32_t gap = first_index - s_stream_next;
    if (gap > 0) {
        s_stats.lost += gap;
        static const int16_t zeros[STREAM_CHUNK];
        uint32_t fill = gap < s_cfg.stream_rate_hz * STREAM_MAX_GAP_S
                      ? gap : s_cfg.stream_rate_hz * STREAM_MAX_GAP_S;
        while (fill > 0) {
            size_t k = fill < STREAM_CHUNK ? fill : STREAM_CHUNK;
            if (xStreamBufferSend(s_stream, zeros, k * sizeof(int16_t), 0) != k * sizeof(int16_t)) {
                break;
            }
            fill -= k;
        }
    }

    /* Whole chunk or nothing: a full buffer means the computer sends too fast. */
    if (xStreamBufferSpacesAvailable(s_stream) < n * sizeof(int16_t)) {
        s_stats.dropped += n;
    } else {
        xStreamBufferSend(s_stream, samples, n * sizeof(int16_t), 0);
    }
    s_stream_next = first_index + n;
    return ESP_OK;
}

audio_stream_stats_t audio_stream_stats(void)
{
    audio_stream_stats_t st = s_stats;
    st.playing     = s_streaming;
    st.buffered_ms = s_stream ? (uint32_t)(xStreamBufferBytesAvailable(s_stream) * 1000 /
                                           (s_cfg.stream_rate_hz * sizeof(int16_t)))
                              : 0;
    return st;
}

bool audio_busy(void)
{
    return s_playing || s_streaming || (s_queue && uxQueueMessagesWaiting(s_queue) > 0);
}

uint32_t audio_sample_rate(void)
{
    return s_rate;
}

/* From the mic hand-off (delivery-box-esp32s3 audio.c): left slot, 24-bit. */
esp_err_t audio_mic_read(int32_t *samples, size_t count, size_t *got, uint32_t timeout_ms)
{
    static int32_t frames[FRAMES_PER_CHUNK * 2];
    size_t    n   = 0;
    esp_err_t err = ESP_OK;

    if (!s_rx || !s_rx_mutex) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_rx_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    while (n < count) {
        size_t want = count - n;
        if (want > FRAMES_PER_CHUNK) {
            want = FRAMES_PER_CHUNK;
        }
        size_t bytes = 0;
        err = i2s_channel_read(s_rx, frames, want * 2 * sizeof(int32_t), &bytes,
                               pdMS_TO_TICKS(timeout_ms));
        if (err != ESP_OK) {
            break;
        }
        size_t frames_read = bytes / (2 * sizeof(int32_t));
        for (size_t i = 0; i < frames_read; i++) {
            samples[n++] = frames[2 * i] >> 8;   /* left slot (SEL = GND), 24-bit signed */
        }
    }
    xSemaphoreGive(s_rx_mutex);

    if (got) {
        *got = n;
    }
    return err;
}
