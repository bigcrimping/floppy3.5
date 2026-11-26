// SPDX-License-Identifier: Unlicense
#include "pico/stdlib.h"
#include "tusb.h"

#include "board_rp2354.h"
#include "diag.h"
#include "floppy_drive.h"
#include "flux.h"
#include "gw_floppy.h"
#include "transport.h"

extern const transport_backend_t transport_usb_backend;
extern const transport_backend_t transport_spi_backend;

int main(void) {
  // Keep the default 150 MHz clock tree. The PL022 slave needs
  // SSPCLK >= 12 x SCLK (caps the link at ~12.5 MHz; it runs at 11.43), and the 25 MHz flux tick
  // is derived from 150 MHz by integer PIO dividers.

  tusb_init(); // no stdio-over-USB: CDC0/CDC1 are ours

  diag_init();
  drive_init();
  flux_init();
  transport_init(&transport_usb_backend, &transport_spi_backend);
  // Commandable over SPI straight away, with or without a USB host.
  gw_floppy_init();

  for (;;) {
    tud_task();
    transport_pump();
    gw_floppy_process(); // blocks only inside flux ops
    diag_poll();
  }
}
