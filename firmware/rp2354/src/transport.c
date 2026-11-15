// SPDX-License-Identifier: Unlicense
#include "transport.h"

#include "pico/time.h"

#include "board_rp2354.h"
#include "diag.h"

static const transport_backend_t *be[3]; // indexed by TRANSPORT_USB/SPI
static int owner = TRANSPORT_NONE;
static absolute_time_t owner_last; // refreshed on latch + nonzero reads

void transport_init(const transport_backend_t *usb,
                    const transport_backend_t *spi) {
  be[TRANSPORT_USB] = usb;
  be[TRANSPORT_SPI] = spi;
  owner = TRANSPORT_NONE;
}

void transport_arbitrate(void) {
  if (owner != TRANSPORT_NONE) {
    if (absolute_time_diff_us(owner_last, get_absolute_time()) <=
        (int64_t)TRANSPORT_IDLE_MS * 1000)
      return;
    // Idle release, then re-arbitrate in the same pass.
    be[owner]->notify_owner(false);
    owner = TRANSPORT_NONE;
    g_diag.arb_idle_releases++;
  }
  // USB is checked first so it wins a tie.
  if (be[TRANSPORT_USB]->rx_ready() > 0) {
    owner = TRANSPORT_USB;
    g_diag.arb_to_usb++;
  } else if (be[TRANSPORT_SPI]->rx_ready() > 0) {
    owner = TRANSPORT_SPI;
    g_diag.arb_to_spi++;
  }
  if (owner != TRANSPORT_NONE) {
    owner_last = get_absolute_time();
    be[owner]->notify_owner(true);
  }
}

int transport_owner(void) { return owner; }

int transport_rx_ready(void) {
  if (owner == TRANSPORT_NONE) {
    transport_arbitrate();
    if (owner == TRANSPORT_NONE)
      return 0;
  }
  return be[owner]->rx_ready();
}

void transport_read(void *buf, uint32_t n) {
  if (owner == TRANSPORT_NONE) {
    transport_arbitrate();
    if (owner == TRANSPORT_NONE)
      return;
  }
  be[owner]->read(buf, n);
  if (n)
    owner_last = get_absolute_time();
}

bool transport_tx_ready(uint32_t need) {
  if (owner == TRANSPORT_NONE)
    return true; // nothing to send to; transport_write() will drop it
  return be[owner]->tx_ready() >= (int)need;
}

void transport_write(const void *buf, uint32_t n) {
  if (owner == TRANSPORT_NONE)
    return;
  be[owner]->write(buf, n);
}

void transport_pump(void) {
  // Both backends, regardless of owner: the SPI link must keep exchanging.
  be[TRANSPORT_USB]->pump();
  be[TRANSPORT_SPI]->pump();
}
