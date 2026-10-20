/*
 * test_mic_dsp.c — high-pass, fixed gain, limiter and AGC behaviour.
 */
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "check.h"
#include "audio_resample.h"
#include "mic_dsp.h"

#define FS 32000
#define M_PI    3.14159265358979323846
#define M_SQRT2 1.41421356237309504880
#define FULL24 8388608.0

static int32_t *tone(double hz, double dbfs, size_t n)
{
    int32_t *b = malloc(n * sizeof *b);
    const double amp = pow(10.0, dbfs / 20.0) * (FULL24 - 1);
    for (size_t i = 0; i < n; i++) {
        b[i] = (int32_t)lrint(amp * sin(2.0 * M_PI * hz * (double)i / FS));
    }
    return b;
}

/* RMS of the last `tail` samples, as a fraction of 16-bit full scale. */
static double rms16(const int16_t *o, size_t n, size_t tail)
{
    double acc = 0;
    for (size_t i = n - tail; i < n; i++) {
        acc += (double)o[i] * o[i];
    }
    return sqrt(acc / (double)tail) / 32768.0;
}

static double dbfs(double lin) { return 20.0 * log10(lin); }

static mic_dsp_t make(mic_dsp_mode_t mode, double gain_db)
{
    mic_dsp_config_t c;
    mic_dsp_default_config(&c, FS);
    c.mode = mode;
    c.gain_db = (float)gain_db;
    mic_dsp_t st;
    mic_dsp_init(&st, &c);
    return st;
}

static void run(mic_dsp_t *st, const int32_t *in, int16_t *out, size_t n)
{
    mic_dsp_process(st, in, out, n);
}

static void test_raw_is_top_16_bits(void)
{
    mic_dsp_t st = make(MIC_DSP_RAW, 0);
    int32_t in[4]  = { 0x7FFFFF, -0x800000, 256, -256 };
    int16_t out[4];
    run(&st, in, out, 4);
    CHECK(out[0] == 32767 && out[1] == -32768 && out[2] == 1 && out[3] == -1);
}

static void test_highpass(void)
{
    const size_t n = FS;
    int16_t *o = malloc(n * sizeof *o);
    int32_t *dc = malloc(n * sizeof *dc);
    for (size_t i = 0; i < n; i++) { dc[i] = 400000; }

    mic_dsp_t st = make(MIC_DSP_FIXED, 0);
    run(&st, dc, o, n);
    CHECK(rms16(o, n, 4000) < 0.001);            /* DC removed */
    free(dc);

    int32_t *t50 = tone(50, -20, n);
    st = make(MIC_DSP_FIXED, 0);
    run(&st, t50, o, n);
    CHECK(dbfs(rms16(o, n, 8000) * M_SQRT2) < -20 - 10);   /* >10 dB down */
    free(t50);

    int32_t *t1k = tone(1000, -20, n);
    st = make(MIC_DSP_FIXED, 0);
    run(&st, t1k, o, n);
    CHECK_NEAR(dbfs(rms16(o, n, 8000) * M_SQRT2), -20, 1.0);
    free(t1k);
    free(o);
}

static void test_fixed_gain(void)
{
    const size_t n = FS;
    int16_t *o = malloc(n * sizeof *o);
    int32_t *t = tone(1000, -50, n);
    mic_dsp_t st = make(MIC_DSP_FIXED, 20);
    run(&st, t, o, n);
    CHECK_NEAR(dbfs(rms16(o, n, 8000) * M_SQRT2), -30, 1.0);
    CHECK(mic_dsp_limited_count(&st) == 0);
    CHECK_NEAR(mic_dsp_gain_db(&st), 20, 0.01);
    free(t); free(o);
}

static void test_limiter_never_overflows(void)
{
    const size_t n = FS / 2;
    int16_t *o = malloc(n * sizeof *o);
    int32_t *t = tone(1000, 0, n);
    for (size_t i = 0; i < n; i += 97) {          /* extremes of the 24-bit range */
        t[i] = (i & 1) ? 0x7FFFFF : -0x800000;
    }
    mic_dsp_t st = make(MIC_DSP_FIXED, 40);
    run(&st, t, o, n);
    int peak = 0;
    for (size_t i = 0; i < n; i++) {
        int a = abs(o[i]);
        if (a > peak) { peak = a; }
    }
    CHECK(peak <= 32767);
    CHECK(peak > 30000);                          /* loud, but not wrapped */
    CHECK(mic_dsp_limited_count(&st) > 0);
    free(t); free(o);
}

static void test_agc_converges_and_caps(void)
{
    const size_t n = FS * 6;
    int16_t *o = malloc(n * sizeof *o);

    /* -60 dBFS RMS-ish tone: needs 40 dB, gain is capped at +30 dB. */
    int32_t *q = tone(1000, -60, n);
    mic_dsp_t st = make(MIC_DSP_AGC, 0);
    run(&st, q, o, n);
    CHECK(mic_dsp_gain_db(&st) <= 30.01);
    CHECK(mic_dsp_gain_db(&st) > 29.0);
    free(q);

    /* -40 dBFS peak tone: AGC should bring RMS to about -20 dBFS. */
    int32_t *m = tone(1000, -40, n);
    st = make(MIC_DSP_AGC, 0);
    run(&st, m, o, n);
    CHECK_NEAR(dbfs(rms16(o, n, FS)), -20, 1.5);
    free(m);
    free(o);
}

static void test_agc_gate_freezes_gain(void)
{
    const size_t n = FS * 3;
    int16_t *o = malloc(n * sizeof *o);
    int32_t *q = tone(1000, -90, n);              /* far below the -70 dBFS gate */
    mic_dsp_t st = make(MIC_DSP_AGC, 10);
    run(&st, q, o, n);
    CHECK_NEAR(mic_dsp_gain_db(&st), 10, 0.01);   /* did not climb */
    free(q); free(o);
}

static void test_agc_drops_on_loud_burst(void)
{
    const size_t n = FS;
    int16_t *o = malloc(n * sizeof *o);
    mic_dsp_t st = make(MIC_DSP_AGC, 30);
    int32_t *loud = tone(1000, -6, n);
    run(&st, loud, o, n);
    CHECK(mic_dsp_gain_db(&st) < 3.0);            /* ~0 dB after 1 s of loud */
    free(loud); free(o);
}

/* Decimator: run `hz` through `fs` -> `fs / factor`, return RMS gain in dB (steady state). */
static double decim_gain_db_at(double hz, size_t chunk, uint32_t fs, uint32_t factor)
{
    const size_t n = fs;
    int16_t *in  = malloc(n * sizeof *in);
    int16_t *out = malloc((n / factor + 8) * sizeof *out);
    for (size_t i = 0; i < n; i++) {
        in[i] = (int16_t)lrint(10000.0 * sin(2.0 * M_PI * hz * (double)i / fs));
    }
    mic_decim_t d;
    mic_decim_init(&d, 7000.0f, fs, factor);
    size_t got = 0;
    for (size_t i = 0; i < n; i += chunk) {
        size_t m = n - i < chunk ? n - i : chunk;
        got += mic_decim_process(&d, in + i, m, out + got);
    }
    CHECK(got == n / factor);
    double acc = 0;
    for (size_t i = got / 2; i < got; i++) { acc += (double)out[i] * out[i]; }
    double rms = sqrt(acc / (double)(got - got / 2));
    free(in); free(out);
    return 20.0 * log10(rms / (10000.0 / M_SQRT2));
}

/* 32 kHz -> 16 kHz (the hand-off test project). */
static double decim_gain_db(double hz, size_t chunk)
{
    return decim_gain_db_at(hz, chunk, FS, 2);
}

/* 48 kHz -> 16 kHz (the final firmware: the speaker runs at 48 kHz). */
static void test_decimator_48k(void)
{
    CHECK_NEAR(decim_gain_db_at(1000, 480, 48000, 3), 0.0, 1.0);
    CHECK_NEAR(decim_gain_db_at(3000, 480, 48000, 3), 0.0, 1.0);
    CHECK(decim_gain_db_at(12000, 480, 48000, 3) < -30.0);   /* would alias to 4 kHz */
    CHECK(decim_gain_db_at(20000, 480, 48000, 3) < -30.0);
    CHECK_NEAR(decim_gain_db_at(1000, 77, 48000, 3), decim_gain_db_at(1000, 480, 48000, 3), 0.01);
}

static void test_decimator(void)
{
    CHECK_NEAR(decim_gain_db(1000, 320), 0.0, 1.0);        /* speech band kept   */
    CHECK_NEAR(decim_gain_db(3000, 320), 0.0, 1.0);
    CHECK(decim_gain_db(12000, 320) < -30.0);              /* would alias to 4 kHz */
    CHECK(decim_gain_db(15000, 320) < -30.0);
    /* odd chunk sizes must give the same result: state/phase survive calls */
    CHECK_NEAR(decim_gain_db(1000, 77), decim_gain_db(1000, 320), 0.01);
}

/* Interpolator (computer -> speaker): 16 kHz -> 48 kHz, RMS gain in dB of the
 * output at `hz`, and the level of an image that would appear at 16k - hz. */
static double interp_gain_db(double hz, size_t chunk, double *image_db)
{
    const uint32_t fin = 16000, factor = 3, fout = fin * factor;
    const size_t n = fin;                                  /* 1 s */
    int16_t *in  = malloc(n * sizeof *in);
    int16_t *out = malloc(n * factor * sizeof *out);
    for (size_t i = 0; i < n; i++) {
        in[i] = (int16_t)lrint(10000.0 * sin(2.0 * M_PI * hz * (double)i / fin));
    }
    audio_interp_t it;
    audio_interp_init(&it, 7000.0f, fout, factor);
    size_t got = 0;
    for (size_t i = 0; i < n; i += chunk) {
        size_t m = n - i < chunk ? n - i : chunk;
        got += audio_interp_process(&it, in + i, m, out + got);
    }
    CHECK(got == n * factor);
    /* Goertzel-style correlation over the second half: level at hz and at 16k-hz */
    double re = 0, im = 0, re2 = 0, im2 = 0;
    size_t start = got / 2, len = got - start;
    for (size_t i = start; i < got; i++) {
        double t = (double)i / fout;
        re  += out[i] * cos(2 * M_PI * hz * t);            im  += out[i] * sin(2 * M_PI * hz * t);
        re2 += out[i] * cos(2 * M_PI * (fin - hz) * t);    im2 += out[i] * sin(2 * M_PI * (fin - hz) * t);
    }
    double amp  = 2.0 * sqrt(re * re + im * im) / (double)len;
    double amp2 = 2.0 * sqrt(re2 * re2 + im2 * im2) / (double)len;
    if (image_db) { *image_db = 20.0 * log10(amp2 / 10000.0 + 1e-12); }
    free(in); free(out);
    return 20.0 * log10(amp / 10000.0);
}

static void test_interpolator(void)
{
    double image;
    CHECK_NEAR(interp_gain_db(1000, 160, &image), 0.0, 1.0);   /* level kept   */
    CHECK(image < -30.0);                                       /* 15 kHz image removed */
    CHECK_NEAR(interp_gain_db(3000, 160, &image), 0.0, 1.0);
    CHECK(image < -30.0);
    /* odd chunk sizes: same result */
    CHECK_NEAR(interp_gain_db(1000, 37, NULL), interp_gain_db(1000, 160, NULL), 0.01);
}

int main(void)
{
    printf("mic_dsp\n");
    RUN(test_raw_is_top_16_bits);
    RUN(test_highpass);
    RUN(test_fixed_gain);
    RUN(test_limiter_never_overflows);
    RUN(test_agc_converges_and_caps);
    RUN(test_agc_gate_freezes_gain);
    RUN(test_agc_drops_on_loud_burst);
    RUN(test_decimator);
    RUN(test_decimator_48k);
    RUN(test_interpolator);
    return check_summary("mic_dsp");
}
