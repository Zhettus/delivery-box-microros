/*
 * wm8960.h — WM8960 Audio HAT, playback path only (DAC -> speakers + headphones).
 *
 * The HAT has no usable microphone (wiring doc §6.4); recording is done by
 * the separate ICS43434 on the same I2S bus.
 *
 * Control: I2C (shared bus with the INA228), address 0x1A. The WM8960's
 * control port is write-only, so this driver keeps a shadow copy of every
 * register it writes.
 * Audio:   I2S CLOCK MASTER. SYSCLK comes from the HAT crystal through the
 *          WM8960 PLL, and the codec derives BCLK/LRCLK from it (see
 *          wm8960_clock.h). The ESP32-S3 and the ICS43434 follow those clocks,
 *          so the DAC and the I2S data can never drift apart (no clicks).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WM8960_I2C_ADDRESS  0x1A

typedef struct {
    i2c_master_bus_handle_t bus;
    uint8_t  address;          /* WM8960_I2C_ADDRESS                          */
    uint32_t i2c_speed_hz;
    uint32_t mclk_hz;          /* crystal on the HAT (check its marking!)     */
    uint32_t sample_rate_hz;   /* must match the I2S sample rate              */
    uint8_t  word_bits;        /* 16, 20, 24 or 32 bits per sample            */
    uint8_t  volume_percent;   /* 0..100 initial speaker/headphone volume     */
} wm8960_config_t;

/** Probe, reset and configure the codec. ESP_ERR_NOT_FOUND if it doesn't answer. */
esp_err_t wm8960_init(const wm8960_config_t *cfg);

/** Speaker + headphone volume, 0 = mute, 100 = +6 dB (1 dB steps). */
esp_err_t wm8960_set_volume(uint8_t percent);
uint8_t   wm8960_get_volume(void);

/** Soft-mute the DAC (smooth ramp, no click). */
esp_err_t wm8960_set_mute(bool mute);

#ifdef __cplusplus
}
#endif
