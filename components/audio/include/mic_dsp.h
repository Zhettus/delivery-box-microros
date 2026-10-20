/*
 * mic_dsp.h — microphone conditioning: high-pass, gain (fixed or AGC), soft limiter.
 *
 * The ICS43434 is a low-sensitivity part (-26 dBFS at 94 dB SPL), so speech a
 * few tens of centimetres away arrives around -60 dBFS. This turns the 24-bit
 * samples from audio_mic_read() into usable 16-bit ones. Pure arithmetic: no
 * hardware access, host-tested in test/host/test_mic_dsp.c.
 *
 * Chain: high-pass -> gain (fixed, or AGC with a noise gate) -> soft limiter.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MIC_DSP_RAW   = 0,   /* just 24 -> 16 bit, nothing else            */
    MIC_DSP_FIXED = 1,   /* high-pass + fixed gain + limiter           */
    MIC_DSP_AGC   = 2,   /* high-pass + automatic gain + limiter       */
} mic_dsp_mode_t;

typedef struct {
    mic_dsp_mode_t mode;
    uint32_t sample_rate_hz;
    float    hpf_hz;              /* high-pass corner, e.g. 120            */
    float    gain_db;             /* fixed gain; initial gain in AGC mode  */
    float    agc_target_dbfs;     /* RMS level the AGC aims for, e.g. -20  */
    float    agc_max_gain_db;     /* AGC never amplifies more than this    */
    float    agc_gate_dbfs;       /* below this level the AGC gain freezes */
    float    limiter_knee_dbfs;   /* soft limiting starts here, e.g. -3    */
} mic_dsp_config_t;

typedef struct {
    mic_dsp_config_t cfg;
    /* biquad high-pass */
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
    /* gain / AGC */
    float gain;                   /* linear                                */
    float env_pow;                /* mean-square envelope (full scale = 1) */
    float env_coeff;              /* envelope follower coefficient         */
    float gain_down, gain_up;     /* gain smoothing coefficients           */
    float target_lin, max_gain, gate_pow, knee;
    uint32_t limited;             /* samples the limiter had to reduce     */
} mic_dsp_t;

/** Fill `cfg` with the recommended defaults (AGC, +24 dB start, 120 Hz). */
void mic_dsp_default_config(mic_dsp_config_t *cfg, uint32_t sample_rate_hz);

void mic_dsp_init(mic_dsp_t *st, const mic_dsp_config_t *cfg);

/** Convert `n` signed 24-bit samples (int32_t) to 16-bit, applying the chain. */
void mic_dsp_process(mic_dsp_t *st, const int32_t *in24, int16_t *out16, size_t n);

/** Current gain in dB (AGC state, or the fixed gain). */
float    mic_dsp_gain_db(const mic_dsp_t *st);

/** Samples the limiter has reduced since init. */
uint32_t mic_dsp_limited_count(const mic_dsp_t *st);

/* ---- N:1 decimator (e.g. 48 kHz -> 16 kHz for streaming) ----------------- */

#define MIC_DECIM_TAPS 47

typedef struct {
    float    h[MIC_DECIM_TAPS];       /* low-pass taps (unity gain at DC) */
    float    hist[MIC_DECIM_TAPS];    /* circular input history           */
    uint32_t pos;
    uint32_t factor;                  /* keep 1 of every `factor` samples */
    uint32_t phase;                   /* 0: this input sample is kept     */
} mic_decim_t;

/** `cutoff_hz`: low-pass corner, below half the OUTPUT rate (e.g. 7000 for 16 kHz).
 *  `factor`: input rate / output rate (2 for 32 -> 16 kHz, 3 for 48 -> 16 kHz). */
void   mic_decim_init(mic_decim_t *d, float cutoff_hz, uint32_t input_rate_hz,
                      uint32_t factor);

/** Low-pass and keep every `factor`-th sample. `out` must hold n/factor + 1
 *  samples. State persists across calls, so any chunk size works. Returns
 *  samples written. */
size_t mic_decim_process(mic_decim_t *d, const int16_t *in, size_t n, int16_t *out);

#ifdef __cplusplus
}
#endif
