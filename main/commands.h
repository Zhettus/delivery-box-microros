/*
 * commands.h — Line-based text protocol on the serial port (UART0 / USB-C "UART").
 *
 * Every command gets exactly one reply line:
 *     OK <details>        or        ERR <reason>
 * Asynchronous events are separate lines:
 *     EVT <what> <details>
 *
 * Type `help` in `idf.py monitor` for the list. Full reference: README.md.
 */
#pragma once

#include "esp_err.h"

esp_err_t commands_start(void);

/** Print an asynchronous "EVT ..." line (thread-safe). */
void app_event(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
