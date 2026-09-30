/*
 * wm8960_clock.c — WM8960 clock-tree maths. See wm8960_clock.h.
 * Follows the WM8960 datasheet "PLL" section (same maths as Linux wm8960.c).
 */
#include <stddef.h>

#include "wm8960_clock.h"

bool wm8960_pll_calc(uint32_t mclk_hz, uint32_t pll_out_hz, wm8960_pll_t *out)
{
    if (mclk_hz == 0 || out == NULL) {
        return false;
    }

    bool     prescale = false;
    uint32_t f1       = mclk_hz;
    uint32_t n        = pll_out_hz / f1;

    if (n < 6) {                 /* ratio too small: halve MCLK first */
        prescale = true;
        f1 >>= 1;
        n = pll_out_hz / f1;
    }
    if (n < 6 || n > 12) {
        return false;
    }

    /* K = fractional part * 2^24, rounded to nearest. */
    uint64_t rem = pll_out_hz % f1;
    uint64_t k   = ((rem << 24) + f1 / 2) / f1;

    out->prescale   = prescale;
    out->n          = (uint8_t)n;
    out->k          = (uint32_t)(k & 0xFFFFFF);
    out->fractional = (out->k != 0);
    return true;
}

/* DACDIV codes: SYSCLK / (256 * div) = sample rate. div x2 to stay integral. */
static const struct {
    uint8_t code;
    uint8_t div_x2;
} s_divs[] = {
    { 0, 2 },   /* /1   */
    { 1, 3 },   /* /1.5 */
    { 2, 4 },   /* /2   */
    { 3, 6 },   /* /3   */
    { 4, 8 },   /* /4   */
    { 5, 11 },  /* /5.5 */
    { 6, 12 },  /* /6   */
};

/* BCLKDIV codes (R8 bits 3:0): BCLK = SYSCLK / div. div x2 to stay integral. */
static const uint8_t s_bclk_div_x2[] = {
    2, 3, 4, 6, 8, 11, 12, 16, 22, 24, 32, 44, 48, 64,   /* codes 0..13 */
};

static bool bclkdiv_code(uint8_t div_x2, uint8_t *code)
{
    for (unsigned i = 0; i < sizeof(s_bclk_div_x2); i++) {
        if (s_bclk_div_x2[i] == div_x2) {
            *code = (uint8_t)i;
            return true;
        }
    }
    return false;
}

bool wm8960_clocking_calc(uint32_t mclk_hz, uint32_t sample_rate_hz,
                          wm8960_clocking_t *out)
{
    static const uint32_t sysclks[] = { 12288000u, 11289600u };

    if (sample_rate_hz == 0 || out == NULL) {
        return false;
    }

    for (unsigned s = 0; s < sizeof(sysclks) / sizeof(sysclks[0]); s++) {
        for (unsigned d = 0; d < sizeof(s_divs) / sizeof(s_divs[0]); d++) {
            /* sysclk / (256 * div_x2 / 2) == fs  <=>  sysclk * 2 == fs * 256 * div_x2 */
            uint64_t lhs = (uint64_t)sysclks[s] * 2u;
            uint64_t rhs = (uint64_t)sample_rate_hz * 256u * s_divs[d].div_x2;
            if (lhs != rhs) {
                continue;
            }
            /* f2 = SYSCLK * 2 (SYSCLKDIV) * 4 (fixed post-divider) */
            if (!wm8960_pll_calc(mclk_hz, sysclks[s] * 8u, &out->pll)) {
                return false;
            }
            /* SYSCLK / BCLK = 256 * div / 64 = 4 * div */
            if (!bclkdiv_code((uint8_t)(4u * s_divs[d].div_x2), &out->bclkdiv)) {
                return false;
            }
            out->sysclk_hz = sysclks[s];
            out->dacdiv    = s_divs[d].code;
            return true;
        }
    }
    return false;
}
