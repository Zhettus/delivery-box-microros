/*
 * mic_dsp.c — see mic_dsp.h.
 */
#include "mic_dsp.h"

#include <math.h>

#define FULL_SCALE_24   8388608.0f   /* 2^23 */
#define ENV_TIME_S      0.020f       /* RMS window (symmetric: no peak bias) */
#define GAIN_DOWN_S     0.010f       /* gain comes down fast on loud input  */
#define GAIN_UP_S       0.700f       /* ... and recovers slowly             */
#define PI_F            3.14159265f

static float db_to_amp(float db)  { return powf(10.0f, db / 20.0f); }
static float db_to_pow(float db)  { return powf(10.0f, db / 10.0f); }
static float time_coeff(float seconds, uint32_t fs)
{
    return 1.0f - expf(-1.0f / (seconds * (float)fs));
}

void mic_dsp_default_config(mic_dsp_config_t *cfg, uint32_t sample_rate_hz)
{
    cfg->mode              = MIC_DSP_AGC;
    cfg->sample_rate_hz    = sample_rate_hz;
    cfg->hpf_hz            = 120.0f;
    cfg->gain_db           = 24.0f;
    cfg->agc_target_dbfs   = -20.0f;
    cfg->agc_max_gain_db   = 30.0f;
    cfg->agc_gate_dbfs     = -70.0f;
    cfg->limiter_knee_dbfs = -3.0f;
}

void mic_dsp_init(mic_dsp_t *st, const mic_dsp_config_t *cfg)
{
    st->cfg = *cfg;
    const uint32_t fs = cfg->sample_rate_hz;

    /* RBJ 2nd-order Butterworth high-pass (Q = 1/sqrt(2)). */
    const float w0    = 2.0f * PI_F * cfg->hpf_hz / (float)fs;
    const float alpha = sinf(w0) / (2.0f * 0.70710678f);
    const float c     = cosf(w0);
    const float a0    = 1.0f + alpha;
    st->b0 = ((1.0f + c) / 2.0f) / a0;
    st->b1 = -(1.0f + c) / a0;
    st->b2 = st->b0;
    st->a1 = (-2.0f * c) / a0;
    st->a2 = (1.0f - alpha) / a0;
    st->x1 = st->x2 = st->y1 = st->y2 = 0.0f;

    st->gain       = db_to_amp(cfg->gain_db);
    st->env_pow    = 0.0f;
    st->env_coeff  = time_coeff(ENV_TIME_S, fs);
    st->gain_down  = time_coeff(GAIN_DOWN_S, fs);
    st->gain_up    = time_coeff(GAIN_UP_S, fs);
    st->target_lin = db_to_amp(cfg->agc_target_dbfs);
    st->max_gain   = db_to_amp(cfg->agc_max_gain_db);
    st->gate_pow   = db_to_pow(cfg->agc_gate_dbfs);
    st->knee       = db_to_amp(cfg->limiter_knee_dbfs);
    st->limited    = 0;
}

/* Linear up to the knee, then tanh towards (but never reaching) full scale. */
static float soft_limit(mic_dsp_t *st, float y)
{
    const float a = fabsf(y);
    if (a <= st->knee) {
        return y;
    }
    st->limited++;
    const float room = 1.0f - st->knee;
    const float out  = st->knee + room * tanhf((a - st->knee) / room);
    return y < 0.0f ? -out : out;
}

static int16_t to_i16(float v)
{
    float s = v * 32767.0f;
    if (s > 32767.0f)  { s = 32767.0f; }
    if (s < -32768.0f) { s = -32768.0f; }
    return (int16_t)lrintf(s);
}

void mic_dsp_process(mic_dsp_t *st, const int32_t *in24, int16_t *out16, size_t n)
{
    if (st->cfg.mode == MIC_DSP_RAW) {
        for (size_t i = 0; i < n; i++) {
            int32_t v = in24[i] >> 8;
            out16[i] = (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
        }
        return;
    }

    for (size_t i = 0; i < n; i++) {
        const float x = (float)in24[i] / FULL_SCALE_24;

        /* high-pass */
        const float h = st->b0 * x + st->b1 * st->x1 + st->b2 * st->x2
                      - st->a1 * st->y1 - st->a2 * st->y2;
        st->x2 = st->x1; st->x1 = x;
        st->y2 = st->y1; st->y1 = h;

        /* automatic gain control */
        if (st->cfg.mode == MIC_DSP_AGC) {
            const float p = h * h;
            st->env_pow += st->env_coeff * (p - st->env_pow);
            if (st->env_pow > st->gate_pow) {
                float want = st->target_lin / sqrtf(st->env_pow);
                if (want > st->max_gain) { want = st->max_gain; }
                if (want < 1.0f)         { want = 1.0f; }
                const float k = want < st->gain ? st->gain_down : st->gain_up;
                st->gain += k * (want - st->gain);
            }                       /* else: freeze, do not pump up the noise */
        }

        out16[i] = to_i16(soft_limit(st, h * st->gain));
    }
}

float mic_dsp_gain_db(const mic_dsp_t *st)
{
    return 20.0f * log10f(st->gain);
}

uint32_t mic_dsp_limited_count(const mic_dsp_t *st)
{
    return st->limited;
}

/* ---- N:1 decimator --------------------------------------------------------- */

void mic_decim_init(mic_decim_t *d, float cutoff_hz, uint32_t input_rate_hz,
                    uint32_t factor)
{
    const float fc   = cutoff_hz / (float)input_rate_hz;      /* cycles/sample */
    const int   mid  = (MIC_DECIM_TAPS - 1) / 2;
    float       sum  = 0.0f;

    for (int k = 0; k < MIC_DECIM_TAPS; k++) {
        const float x    = (float)(k - mid);
        const float sinc = (k == mid) ? 2.0f * fc
                                      : sinf(2.0f * PI_F * fc * x) / (PI_F * x);
        const float win  = 0.54f - 0.46f * cosf(2.0f * PI_F * (float)k
                                                 / (float)(MIC_DECIM_TAPS - 1));
        d->h[k] = sinc * win;
        sum    += d->h[k];
    }
    for (int k = 0; k < MIC_DECIM_TAPS; k++) {
        d->h[k] /= sum;                                        /* DC gain = 1 */
        d->hist[k] = 0.0f;
    }
    d->pos    = 0;
    d->factor = factor ? factor : 1;
    d->phase  = 0;
}

size_t mic_decim_process(mic_decim_t *d, const int16_t *in, size_t n, int16_t *out)
{
    size_t produced = 0;

    for (size_t i = 0; i < n; i++) {
        d->hist[d->pos] = (float)in[i];
        d->pos = (d->pos + 1) % MIC_DECIM_TAPS;

        if (d->phase == 0) {
            float    acc = 0.0f;
            uint32_t idx = d->pos;                 /* oldest sample */
            for (int k = MIC_DECIM_TAPS - 1; k >= 0; k--) {
                acc += d->h[k] * d->hist[idx];
                idx  = (idx + 1) % MIC_DECIM_TAPS;
            }
            if (acc > 32767.0f)  { acc = 32767.0f; }
            if (acc < -32768.0f) { acc = -32768.0f; }
            out[produced++] = (int16_t)lrintf(acc);
        }
        d->phase = (d->phase + 1) % d->factor;
    }
    return produced;
}
