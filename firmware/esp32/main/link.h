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

// Master-side link profile (speed-up phase 2a): free-running, diff two reads.
typedef struct {
    uint32_t ex;            // exchanges
    uint32_t ex_hdr;        // ... that clocked a header only (nothing granted or sent)
    uint32_t ex_full;       // ... that clocked a full-size frame
    uint32_t rx_frames;     // exchanges that delivered new payload to us
    uint32_t rx_small;      // ... less than half a frame of it
    uint32_t tx_frames;     // exchanges carrying our payload
    uint64_t clocked_bytes; // bytes on the wire
    uint64_t rx_bytes, tx_bytes;
    uint64_t build_us;      // header/CRC/copy before the gap
    uint64_t gap_us;        // waiting out GW_SPI_GAP_US
    uint64_t xfer_us;       // the SPI transaction itself
    uint64_t parse_us;      // CRC + ring push after it
    uint64_t drdy_us;       // link_read asleep waiting for DRDY
    uint64_t backoff_us;    // retry back-off sleeps
} link_prof_t;
void link_prof_get(link_prof_t *out);
