/*
 * audio_resample.h — 1:N interpolator (e.g. 16 kHz -> 48 kHz) for audio that
 * arrives from the computer and is played on the speaker. The mirror image of
 * the microphone's decimator (mic_dsp.h). Pure arithmetic: host-tested in
 * test/host/test_mic_dsp.c.
 *
 * Each input sample becomes `factor` output samples: zero-stuffing followed by
 * a windowed-sinc low-pass (gain x factor, so the level stays the same).
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AUDIO_INTERP_TAPS 47

typedef struct {
    float    h[AUDIO_INTERP_TAPS];      /* low-pass taps, DC gain = factor  */
    float    hist[AUDIO_INTERP_TAPS];   /* circular zero-stuffed history    */
    uint32_t pos;
    uint32_t factor;
} audio_interp_t;

/** `cutoff_hz`: below half the INPUT rate (e.g. 7000 for 16 kHz input).
 *  `output_rate_hz` = input rate x `factor`. */
void audio_interp_init(audio_interp_t *it, float cutoff_hz, uint32_t output_rate_hz,
                       uint32_t factor);

/** Writes n x factor samples to `out`. State persists across calls. */
size_t audio_interp_process(audio_interp_t *it, const int16_t *in, size_t n, int16_t *out);

#ifdef __cplusplus
}
#endif
