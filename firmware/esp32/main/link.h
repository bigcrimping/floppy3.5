// Byte pipe between gw_client.c and the GWSPI link (link_spi.c,
// docs/spi-link-protocol.md).
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// SPI bus/device + DRDY ISR, then SYNC-poll until the peer answers (up to
// GW_SYNC_TIMEOUT_MS). Blocking.
esp_err_t link_up(void);
void link_down(void);

// False once bring-up failed or a verb exhausted its deadline; every verb then
// fails fast until the next link_up.
bool link_is_up(void);

// Returns as soon as `want` bytes are available, or whatever arrived by the
// deadline (0 is a normal timeout). Negative only on a hard link error.
int link_read(uint8_t *dst, size_t want, int to_ms);

// Blocks until every byte is acked by the peer. Returns len, or negative.
// Handles a single call of up to ~120 KB from PSRAM.
int link_write(const uint8_t *src, size_t len);

// No-op under GWSPI (link_write already waits for acks); returns true.
bool link_wait_tx_done(int to_ms);

void link_flush_rx(void);

// Two header-only polls; returns GWSPI_FLAG_USB from the second (1 = a USB
// host has the controller), or -1 if no valid header came back. Sends no
// command, so it is safe while USB owns the controller. Link-owning task only.
int link_probe_peer_usb(void);

// Pulse the peer's RUN line and wait GW_BOOT_SETTLE_MS.
void link_peer_reset(void);

// One-line counters for logs and /api/status.
const char *link_stats_str(void);
