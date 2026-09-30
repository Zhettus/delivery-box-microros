/*
 * ina228_conv.h — INA228 register <-> engineering-unit conversions
 * (pure logic, no ESP-IDF; unit-tested in test/host/test_ina228_conv.c).
 *
 * Formulas from the TI INA228 datasheet, section 8.1.2 "Current and Power
 * Calculations".
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** CURRENT_LSB [A/bit] = max expected current / 2^19. */
double ina228_current_lsb(double max_current_a);

/**
 * SHUNT_CAL register = 13107.2e6 * CURRENT_LSB * R_SHUNT
 * (x4 when ADCRANGE = 1, the +/-40.96 mV range). Clamped to 15 bits.
 */
uint16_t ina228_shunt_cal(double current_lsb, double shunt_ohms, bool adc_range_40mv);

/** Sign-extend the 20-bit value stored in bits [23:4] of a 24-bit register. */
int32_t ina228_sext20(uint32_t reg24);

/** Sign-extend a 40-bit register value. */
int64_t ina228_sext40(uint64_t reg40);

double ina228_vbus_volts(uint32_t reg24);                       /* 195.3125 uV/LSB */
double ina228_vshunt_volts(uint32_t reg24, bool adc_range_40mv); /* 312.5 / 78.125 nV/LSB */
double ina228_current_amps(uint32_t reg24, double current_lsb);
double ina228_power_watts(uint32_t reg24, double current_lsb);  /* 3.2 * CURRENT_LSB */
double ina228_energy_joules(uint64_t reg40, double current_lsb);/* 16 * 3.2 * CURRENT_LSB */
double ina228_charge_coulombs(uint64_t reg40, double current_lsb);
double ina228_die_temp_c(uint16_t reg16);                       /* 7.8125 m°C/LSB */

#ifdef __cplusplus
}
#endif
