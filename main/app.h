/*
 * app.h — What main.c brought up, shared with the other main/ modules.
 */
#pragma once

#include <stdbool.h>

#include "driver/i2c_master.h"
#include "ina228.h"

typedef struct {
    i2c_master_bus_handle_t i2c_bus;   /* NULL if the bus failed           */
    ina228_handle_t         ina228;    /* NULL if the sensor is missing    */

    /* Which subsystems initialised successfully */
    bool leds_ok;
    bool eyes_ok;
    bool tail_ok;
    bool door_ok;
    bool audio_ok;                     /* I2S + microphone                 */
    bool speaker_ok;                   /* WM8960 answered                  */
    bool power_ok;                     /* INA228 answered                  */
} app_t;

extern app_t g_app;
