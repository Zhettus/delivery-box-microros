/*
 * wm8960_clock.h — WM8960 clock-tree maths (pure logic, no ESP-IDF).
 *
 * The ESP32-S3 does not send MCLK to the WM8960 (no pin for it in the
 * wiring), so the codec runs from the crystal on the Audio HAT. Its internal
 * PLL turns that crystal frequency into the SYSCLK the DAC needs:
 *
 *     MCLK --[/1 or /2]--> f1 --[x N.K]--> f2 (90..100 MHz) --[/4]--[/2]--> SYSCLK
 *
 *     SYSCLK = 12.288 MHz for 8/12/16/24/32/48 kHz
 *     SYSCLK = 11.2896 MHz for 11.025/22.05/44.1 kHz
 *
 * As I2S clock master the WM8960 also divides SYSCLK down to
 * BCLK = 64 x fs (two 32-bit slots) and LRCLK = fs.
 *
 * Unit-tested in test/host/test_codecs.c.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool     prescale;   /* true: MCLK is divided by 2 before the PLL  */
    uint8_t  n;          /* integer part of the PLL ratio (6..12)      */
    uint32_t k;          /* 24-bit fractional part                     */
    bool     fractional; /* k != 0 (sets the SDM bit)                  */
} wm8960_pll_t;

typedef struct {
    uint32_t     sysclk_hz;  /* 12288000 or 11289600                    */
    wm8960_pll_t pll;
    uint8_t      dacdiv;     /* R4 DACDIV/ADCDIV field (0..6)            */
    uint8_t      bclkdiv;    /* R8 BCLKDIV field for BCLK = 64 x fs      */
} wm8960_clocking_t;

/**
 * Work out PLL + divider settings for `mclk_hz` (HAT crystal) and
 * `sample_rate_hz`. Returns false if the combination is unsupported.
 */
bool wm8960_clocking_calc(uint32_t mclk_hz, uint32_t sample_rate_hz,
                          wm8960_clocking_t *out);

/** PLL settings producing f2 = `pll_out_hz` from `mclk_hz`. */
bool wm8960_pll_calc(uint32_t mclk_hz, uint32_t pll_out_hz, wm8960_pll_t *out);

#ifdef __cplusplus
}
#endif
