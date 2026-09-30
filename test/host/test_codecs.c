/*
 * test_codecs.c — WM8960 clock maths and INA228 conversions.
 */
#include "check.h"
#include "ina228_conv.h"
#include "wm8960_clock.h"

static void test_pll_12mhz(void)
{
    /* Datasheet example: MCLK 12 MHz -> SYSCLK 12.288 MHz: N = 8, K = 0x3126E9 */
    wm8960_pll_t p;
    CHECK(wm8960_pll_calc(12000000, 98304000, &p));
    CHECK(!p.prescale);
    CHECK(p.n == 8);
    CHECK(p.k == 0x3126E9);
    CHECK(p.fractional);
}

static void test_pll_24mhz_uses_prescaler(void)
{
    wm8960_pll_t p;
    CHECK(wm8960_pll_calc(24000000, 98304000, &p));
    CHECK(p.prescale);
    CHECK(p.n == 8);
    CHECK(p.k == 0x3126E9);
}

static void test_pll_24576_is_integer(void)
{
    wm8960_pll_t p;
    CHECK(wm8960_pll_calc(24576000, 98304000, &p));
    CHECK(p.prescale && p.n == 8 && p.k == 0 && !p.fractional);
}

static void test_pll_rejects_nonsense(void)
{
    wm8960_pll_t p;
    CHECK(!wm8960_pll_calc(0, 98304000, &p));
    CHECK(!wm8960_pll_calc(1000000, 98304000, &p));   /* ratio > 12 */
}

static void test_clocking(void)
{
    wm8960_clocking_t c;
    /* BCLK must be 64 x fs: 12.288 MHz / 1.024 MHz = /12 -> code 9 */
    CHECK(wm8960_clocking_calc(24000000, 16000, &c));
    CHECK(c.sysclk_hz == 12288000 && c.dacdiv == 3 && c.bclkdiv == 9);
    /* 32 kHz: DACDIV /1.5, BCLK 2.048 MHz = /6 -> code 6 */
    CHECK(wm8960_clocking_calc(24000000, 32000, &c));
    CHECK(c.dacdiv == 1 && c.bclkdiv == 6);
    /* 48 kHz: /1, BCLK 3.072 MHz = /4 -> code 4 */
    CHECK(wm8960_clocking_calc(24000000, 48000, &c));
    CHECK(c.dacdiv == 0 && c.bclkdiv == 4);
    CHECK(wm8960_clocking_calc(24000000, 8000, &c));
    CHECK(c.dacdiv == 6 && c.bclkdiv == 12);
    CHECK(wm8960_clocking_calc(24000000, 44100, &c));
    CHECK(c.sysclk_hz == 11289600 && c.dacdiv == 0);
    CHECK(!wm8960_clocking_calc(24000000, 12345, &c));
}

static void test_ina228(void)
{
    /* 15 mOhm shunt, 10 A range (Adafruit INA228 breakout) */
    double lsb = ina228_current_lsb(10.0);
    CHECK_NEAR(lsb, 1.9073486e-5, 1e-11);
    CHECK(ina228_shunt_cal(lsb, 0.015, false) == 3750);
    CHECK(ina228_shunt_cal(lsb, 0.015, true) == 15000);

    CHECK(ina228_sext20(0x000010) == 1);
    CHECK(ina228_sext20(0xFFFFF0) == -1);
    CHECK(ina228_sext20(0x7FFFF0) == 524287);
    CHECK(ina228_sext20(0x800000) == -524288);
    CHECK(ina228_sext40(0xFFFFFFFFFFull) == -1);

    /* 12 V bus: 12 / 195.3125e-6 = 61440 counts -> register value << 4 */
    CHECK_NEAR(ina228_vbus_volts(61440u << 4), 12.0, 1e-9);
    /* 1 A: 1 / lsb = 52428.8 counts */
    CHECK_NEAR(ina228_current_amps(52429u << 4, lsb), 1.0, 1e-4);
    CHECK_NEAR(ina228_current_amps((uint32_t)(-52429 * 16) & 0xFFFFFF, lsb), -1.0, 1e-4);
    CHECK_NEAR(ina228_die_temp_c(0x0C80), 25.0, 1e-9);   /* 3200 * 7.8125 m°C */
    CHECK_NEAR(ina228_power_watts(1000, lsb), 1000 * 3.2 * lsb, 1e-12);
}

int main(void)
{
    printf("wm8960_clock / ina228_conv\n");
    RUN(test_pll_12mhz);
    RUN(test_pll_24mhz_uses_prescaler);
    RUN(test_pll_24576_is_integer);
    RUN(test_pll_rejects_nonsense);
    RUN(test_clocking);
    RUN(test_ina228);
    return check_summary("codecs");
}
