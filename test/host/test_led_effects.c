/*
 * test_led_effects.c — loop mapping and effect shapes.
 */
#include <string.h>

#include "check.h"
#include "led_effects.h"

/* Same layout as main/app_config.h: left 29, right 29, front 19, back 19. */
static const led_layout_t L = {
    .num_strips    = 4,
    .pixels        = { 29, 29, 19, 19 },
    .loop_segments = 4,
    .loop          = { { 0, false }, { 2, false }, { 1, true }, { 3, true } },
};

static rgb_t s0[29], s1[29], s2[19], s3[19];
static led_frame_t fb = { s0, s1, s2, s3 };

static void clear(void)
{
    memset(s0, 0, sizeof s0); memset(s1, 0, sizeof s1);
    memset(s2, 0, sizeof s2); memset(s3, 0, sizeof s3);
}

static void test_loop_covers_every_pixel_once(void)
{
    int hits[4][29] = { { 0 } };
    CHECK(led_loop_length(&L) == 96);
    for (uint16_t p = 0; p < 96; p++) {
        uint8_t s; uint16_t i;
        CHECK(led_loop_pixel(&L, p, &s, &i));
        hits[s][i]++;
    }
    for (int s = 0; s < 4; s++) {
        for (int i = 0; i < L.pixels[s]; i++) {
            CHECK(hits[s][i] == 1);
        }
    }
    uint8_t s; uint16_t i;
    CHECK(!led_loop_pixel(&L, 96, &s, &i));
    /* reversed segment: first loop pixel of strip 2 (index 1) is its last pixel */
    CHECK(led_loop_pixel(&L, 29 + 19, &s, &i) && s == 1 && i == 28);
}

static void test_envelopes(void)
{
    CHECK_NEAR(led_fx_envelope(LED_EFFECT_SOLID, 0.5f, 1.0f), 0.5, 1e-6);
    CHECK_NEAR(led_fx_envelope(LED_EFFECT_SOLID, 3.0f, 1.0f), 1.0, 1e-6);
    CHECK_NEAR(led_fx_envelope(LED_EFFECT_PULSE, 1.0f, 1.0f), 1.0, 1e-5);
    CHECK_NEAR(led_fx_envelope(LED_EFFECT_PULSE, 1.5f, 1.0f), 0.5, 1e-5);
    CHECK_NEAR(led_fx_envelope(LED_EFFECT_PULSE, 2.0f, 1.0f), 0.0, 1e-5);
    CHECK(led_fx_envelope(LED_EFFECT_BLINK, 0.2f, 0.5f) == 1.0f);
    CHECK(led_fx_envelope(LED_EFFECT_BLINK, 0.7f, 0.5f) == 0.0f);
    CHECK(led_fx_envelope(LED_EFFECT_OFF, 1.0f, 1.0f) == 0.0f);
    CHECK(led_fx_envelope(LED_EFFECT_COMET, 1.0f, 1.0f) == 1.0f);
}

static void test_comet_head_is_brightest(void)
{
    clear();
    led_fx_comet(&L, fb, RGB_WHITE, 10.0f, 7.0f);
    uint8_t s; uint16_t i;
    led_loop_pixel(&L, 10, &s, &i);
    CHECK(fb[s][i].r == 255);
    led_loop_pixel(&L, 5, &s, &i);
    CHECK(fb[s][i].r > 0 && fb[s][i].r < 255);         /* tail */
    led_loop_pixel(&L, 20, &s, &i);
    CHECK(fb[s][i].r == 0);                            /* ahead of the head */

    /* Wrap-around: head at 1 -> tail continues at the end of the loop. */
    clear();
    led_fx_comet(&L, fb, RGB_WHITE, 1.0f, 7.0f);
    led_loop_pixel(&L, 95, &s, &i);
    CHECK(fb[s][i].r > 0);
}

static void test_sweep(void)
{
    clear();
    led_fx_sweep(&L, fb, RGB_GREEN, 50.5f);
    uint8_t s; uint16_t i;
    led_loop_pixel(&L, 0, &s, &i);   CHECK(fb[s][i].g == 255);
    led_loop_pixel(&L, 49, &s, &i);  CHECK(fb[s][i].g == 255);
    led_loop_pixel(&L, 50, &s, &i);  CHECK(fb[s][i].g > 100 && fb[s][i].g < 160);
    led_loop_pixel(&L, 51, &s, &i);  CHECK(fb[s][i].g == 0);
}

static void test_diverge_is_mirrored(void)
{
    for (double t = 0.0; t < 5.0; t += 0.37) {
        clear();
        led_fx_diverge(&L, fb, RGB_WHITE, t, 1.0f, 12.0f, 7.0f);
        for (int s = 0; s < 4; s++) {
            int n = L.pixels[s];
            for (int i = 0; i < n / 2; i++) {
                CHECK(fb[s][i].r == fb[s][n - 1 - i].r);
            }
        }
    }
    /* Right at birth only the centre is lit. */
    clear();
    led_fx_diverge(&L, fb, RGB_WHITE, 0.0, 1.0f, 12.0f, 7.0f);
    CHECK(s0[14].r > 0);
    CHECK(s0[0].r == 0);
}

static void test_names(void)
{
    led_effect_t e;
    CHECK(led_effect_from_name("comet", &e) && e == LED_EFFECT_COMET);
    CHECK(!led_effect_from_name("nope", &e));
    CHECK(strcmp(led_effect_name(LED_EFFECT_PULSE), "pulse") == 0);
}

int main(void)
{
    printf("led_effects\n");
    RUN(test_loop_covers_every_pixel_once);
    RUN(test_envelopes);
    RUN(test_comet_head_is_brightest);
    RUN(test_sweep);
    RUN(test_diverge_is_mirrored);
    RUN(test_names);
    return check_summary("led_effects");
}
