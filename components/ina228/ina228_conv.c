/*
 * ina228_conv.c — INA228 conversions. See ina228_conv.h.
 */
#include "ina228_conv.h"

double ina228_current_lsb(double max_current_a)
{
    return max_current_a / 524288.0;   /* 2^19 */
}

uint16_t ina228_shunt_cal(double current_lsb, double shunt_ohms, bool adc_range_40mv)
{
    double cal = 13107.2e6 * current_lsb * shunt_ohms;
    if (adc_range_40mv) {
        cal *= 4.0;
    }
    cal += 0.5;
    if (cal > 32767.0) {
        cal = 32767.0;
    }
    if (cal < 0.0) {
        cal = 0.0;
    }
    return (uint16_t)cal;
}

int32_t ina228_sext20(uint32_t reg24)
{
    /* Move bit 23 to bit 31, then arithmetic-shift back down by 12. */
    return (int32_t)(reg24 << 8) >> 12;
}

int64_t ina228_sext40(uint64_t reg40)
{
    return (int64_t)(reg40 << 24) >> 24;
}

double ina228_vbus_volts(uint32_t reg24)
{
    return (double)ina228_sext20(reg24) * 195.3125e-6;
}

double ina228_vshunt_volts(uint32_t reg24, bool adc_range_40mv)
{
    double lsb = adc_range_40mv ? 78.125e-9 : 312.5e-9;
    return (double)ina228_sext20(reg24) * lsb;
}

double ina228_current_amps(uint32_t reg24, double current_lsb)
{
    return (double)ina228_sext20(reg24) * current_lsb;
}

double ina228_power_watts(uint32_t reg24, double current_lsb)
{
    return (double)(reg24 & 0xFFFFFF) * 3.2 * current_lsb;
}

double ina228_energy_joules(uint64_t reg40, double current_lsb)
{
    return (double)(reg40 & 0xFFFFFFFFFFull) * 16.0 * 3.2 * current_lsb;
}

double ina228_charge_coulombs(uint64_t reg40, double current_lsb)
{
    return (double)ina228_sext40(reg40) * current_lsb;
}

double ina228_die_temp_c(uint16_t reg16)
{
    return (double)(int16_t)reg16 * 7.8125e-3;
}
