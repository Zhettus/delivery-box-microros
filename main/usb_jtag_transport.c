/*
 * usb_jtag_transport.c
 *
 * micro-ROS custom transport over the ESP32-S3's native USB Serial/JTAG
 * peripheral (GPIO19/20 -> the board's only USB-C connector).
 *
 * The Waveshare ESP32-S3-RS485-CAN has NO USB-UART bridge chip: the Type-C
 * connector goes straight to USB_N/USB_P. So UART0 is only on header pins,
 * and the native USB controller is the only USB path off the board.
 *
 * REQUIRED sdkconfig changes (otherwise the console will inject ASCII log
 * text into the XRCE byte stream and the agent will see corrupt frames):
 *
 *   Component config -> ESP System Settings
 *     Channel for console output            : Default: UART0   (or None)
 *     Channel for console secondary output  : No secondary console  <-- CRITICAL
 *
 * The secondary console defaults to USB_SERIAL_JTAG on the S3. That is what
 * has been carrying your idf.py monitor output. It MUST be turned off.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_idf_version.h"

#include <uxr/client/profile/transport/custom/custom_transport.h>

#define USB_JTAG_RX_BUF   1024
#define USB_JTAG_TX_BUF   1024

bool usb_jtag_open(struct uxrCustomTransport *transport)
{
    (void)transport;

    usb_serial_jtag_driver_config_t cfg = {
        .rx_buffer_size = USB_JTAG_RX_BUF,
        .tx_buffer_size = USB_JTAG_TX_BUF,
    };

    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    /* Already installed is fine - e.g. after an agent reconnect. */
    return (err == ESP_OK || err == ESP_ERR_INVALID_STATE);
}

bool usb_jtag_close(struct uxrCustomTransport *transport)
{
    (void)transport;
    usb_serial_jtag_driver_uninstall();
    return true;
}

size_t usb_jtag_write(struct uxrCustomTransport *transport,
                      const uint8_t *buf, size_t len, uint8_t *errcode)
{
    (void)transport;

    int written = usb_serial_jtag_write_bytes(buf, len, pdMS_TO_TICKS(100));
    if (written < 0) {
        *errcode = 1;
        return 0;
    }
    /* Push it out now rather than waiting for the driver's flush timer -
     * XRCE is request/response and latency here shows up as agent timeouts.
     * usb_serial_jtag_wait_tx_done() only exists from IDF v5.3; on v5.2 the
     * driver's ISR drains the FIFO on its own, just without a sync point. */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
    usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(50));
#endif
    return (size_t)written;
}

size_t usb_jtag_read(struct uxrCustomTransport *transport,
                     uint8_t *buf, size_t len, int timeout_ms, uint8_t *errcode)
{
    (void)transport;

    if (timeout_ms < 0) timeout_ms = 0;

    int rx = usb_serial_jtag_read_bytes(buf, len, pdMS_TO_TICKS(timeout_ms));
    if (rx < 0) {
        *errcode = 1;
        return 0;
    }
    return (size_t)rx;
}
