// SPDX-License-Identifier: Unlicense
// Transport seam between USB CDC0 and the GWSPI link. The first channel to
// present command bytes owns the session until TRANSPORT_IDLE_MS without RX;
// USB wins ties. The command engine talks only to these functions.
#pragma once
#include <stdbool.h>
#include <stdint.h>

enum { TRANSPORT_NONE = 0, TRANSPORT_USB = 1, TRANSPORT_SPI = 2 };

typedef struct transport_backend {
  int (*rx_ready)(void);                      // bytes available
  void (*read)(void *buf, uint32_t n);        // requires rx_ready() >= n
  int (*tx_ready)(void);                      // writable space
  void (*write)(const void *buf, uint32_t n); // requires tx_ready() >= n
  void (*pump)(void);                         // per-loop housekeeping
  void (*notify_owner)(bool owned);
} transport_backend_t;

void transport_init(const transport_backend_t *usb,
                    const transport_backend_t *spi);

// Call each pass while in command-wait: latches or releases the owner.
void transport_arbitrate(void);

int transport_owner(void); // TRANSPORT_*

// Owner-directed I/O. With no owner, rx_ready/read arbitrate first.
int transport_rx_ready(void);
void transport_read(void *buf, uint32_t n);
bool transport_tx_ready(uint32_t need);
void transport_write(const void *buf, uint32_t n);

// Pumps both backends regardless of owner. Call every superloop pass.
void transport_pump(void);

// A USB host holds the gw port (CDC0 DTR) or owns the transport. Advertised
// to the ESP as GWSPI_FLAG_USB so it keeps the drive rail up and stays off
// the link.
extern volatile bool g_usb_host_active;
