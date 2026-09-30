/*
 * ina228.h — TI INA228 20-bit power monitor on the shared I2C bus.
 *
 * Only VCC/SDA/SCL/GND go to the ESP32. The shunt (Current+/-) and the
 * V+/V- taps sit in the monitored rail (e.g. the 12 V feed), see §6.7.
 *
 * Several sensors can share the bus (different A0/A1 addresses), so this
 * driver is handle-based.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define INA228_ADDRESS_DEFAULT  0x40    /* A0 = A1 = GND */

typedef enum {
    INA228_MODE_MONITOR = 0,   /* bus + shunt + temperature, ~200 ms per result */
    INA228_MODE_FAST_CURRENT,  /* current + bus voltage, ~9 ms per result      */
} ina228_mode_t;

typedef struct {
    i2c_master_bus_handle_t bus;
    uint8_t  address;
    uint32_t i2c_speed_hz;
    float    shunt_ohms;       /* e.g. 0.015 on the Adafruit breakout      */
    float    max_current_a;    /* largest current you expect to measure    */
    ina228_mode_t mode;        /* default (0) = INA228_MODE_MONITOR        */
} ina228_config_t;

typedef struct {
    float  bus_voltage_v;
    float  shunt_voltage_mv;
    float  current_a;
    float  power_w;
    float  die_temp_c;
    double energy_wh;          /* since power-up or the last reset          */
    double charge_ah;
} ina228_reading_t;

typedef struct ina228 *ina228_handle_t;

/** Probe, verify the chip ID, reset and configure continuous conversion. */
esp_err_t ina228_create(const ina228_config_t *cfg, ina228_handle_t *out);

esp_err_t ina228_read(ina228_handle_t h, ina228_reading_t *out);

/** Read only the CURRENT register (one short I2C transfer). */
esp_err_t ina228_read_current(ina228_handle_t h, float *amps);

/** Read only the VBUS register (the monitored rail voltage, via V+ / VBUS). */
esp_err_t ina228_read_bus_voltage(ina228_handle_t h, float *volts);

/** Zero the energy and charge accumulators. */
esp_err_t ina228_reset_accumulators(ina228_handle_t h);

#ifdef __cplusplus
}
#endif
