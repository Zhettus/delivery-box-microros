/*
 * audio_resample.c — see audio_resample.h.
 */
#include "audio_resample.h"

#include <math.h>

#define PI_F 3.14159265f

void audio_interp_init(audio_interp_t *it, float cutoff_hz, uint32_t output_rate_hz,
                       uint32_t factor)
{
    const float fc  = cutoff_hz / (float)output_rate_hz;      /* cycles/output sample */
    const int   mid = (AUDIO_INTERP_TAPS - 1) / 2;
    float       sum = 0.0f;

    it->factor = factor ? factor : 1;
    for (int k = 0; k < AUDIO_INTERP_TAPS; k++) {
        const float x    = (float)(k - mid);
        const float sinc = (k == mid) ? 2.0f * fc
                                      : sinf(2.0f * PI_F * fc * x) / (PI_F * x);
        const float win  = 0.54f - 0.46f * cosf(2.0f * PI_F * (float)k
                                                 / (float)(AUDIO_INTERP_TAPS - 1));
        it->h[k] = sinc * win;
        sum     += it->h[k];
    }
    for (int k = 0; k < AUDIO_INTERP_TAPS; k++) {
        it->h[k] = it->h[k] / sum * (float)it->factor;        /* zero-stuffing loses x factor */
        it->hist[k] = 0.0f;
    }
    it->pos = 0;
}

size_t audio_interp_process(audio_interp_t *it, const int16_t *in, size_t n, int16_t *out)
{
    size_t produced = 0;

    for (size_t i = 0; i < n; i++) {
        for (uint32_t p = 0; p < it->factor; p++) {
            it->hist[it->pos] = (p == 0) ? (float)in[i] : 0.0f;
            it->pos = (it->pos + 1) % AUDIO_INTERP_TAPS;

            float    acc = 0.0f;
            uint32_t idx = it->pos;                    /* oldest sample */
            for (int k = AUDIO_INTERP_TAPS - 1; k >= 0; k--) {
                acc += it->h[k] * it->hist[idx];
                idx  = (idx + 1) % AUDIO_INTERP_TAPS;
            }
            if (acc > 32767.0f)  { acc = 32767.0f; }
            if (acc < -32768.0f) { acc = -32768.0f; }
            out[produced++] = (int16_t)lrintf(acc);
        }
    }
    return produced;
}
