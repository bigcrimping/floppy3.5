// SPDX-License-Identifier: Unlicense
// USB transport backend: TinyUSB CDC0. TinyUSB's FIFOs are the ring buffers.
#include "transport.h"

#include "tusb.h"

static int usb_rx_ready(void) { return (int)tud_cdc_n_available(0); }

static void usb_read(void *buf, uint32_t n) { tud_cdc_n_read(0, buf, n); }

static int usb_tx_ready(void) { return (int)tud_cdc_n_write_available(0); }

static void usb_write(const void *buf, uint32_t n) {
  tud_cdc_n_write(0, buf, n);
  // Replies are small and gw.exe waits on each one: favour latency.
  tud_cdc_n_write_flush(0);
}

volatile bool g_usb_host_active;

static void usb_pump(void) {
  tud_cdc_n_write_flush(0);
  // gw.exe raises DTR on open and drops it on close, so this stays true
  // across idle gaps that release ownership. Ownership covers hosts that
  // never raise DTR.
  g_usb_host_active = tud_cdc_n_connected(0) || transport_owner() == TRANSPORT_USB;
}

static void usb_notify_owner(bool owned) { (void)owned; }

const transport_backend_t transport_usb_backend = {
    .rx_ready = usb_rx_ready,
    .read = usb_read,
    .tx_ready = usb_tx_ready,
    .write = usb_write,
    .pump = usb_pump,
    .notify_owner = usb_notify_owner,
};
