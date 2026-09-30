#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

bool   usb_jtag_open (struct uxrCustomTransport *transport);
bool   usb_jtag_close(struct uxrCustomTransport *transport);
size_t usb_jtag_write(struct uxrCustomTransport *transport,
                      const uint8_t *buf, size_t len, uint8_t *errcode);
size_t usb_jtag_read (struct uxrCustomTransport *transport,
                      uint8_t *buf, size_t len, int timeout_ms, uint8_t *errcode);
