// SPDX-License-Identifier: Unlicense
// GWSPI v1 slave: the SPI side of the transport seam.
#pragma once
#include <stdint.h>

#include "transport.h"

extern const transport_backend_t transport_spi_backend;
const transport_backend_t *spi_transport_backend(void);

// PL022 slave, DMA channels, CS-edge IRQ, first armed frame. Idempotent; the
// backend's pump() calls it on first use.
void spi_link_init(void);

// Bracket every flux op (up to ~2.2 s). The link keeps running (IRQs stay
// on), but the command engine isn't draining the rings, so prepare advertises
// avail=0 and drops DRDY and end re-advertises what queued up. These are
// hints; correctness doesn't depend on the master honouring them.
void spi_link_prepare_busy(void);
void spi_link_end_busy(void);

// Link counters kept out of gw_diag_t, whose layout is fixed by GetInfo 0x88.
typedef struct {
  uint32_t exchanges;         // CS-framed transactions
  uint32_t short_frames;      // clocked less than the declared frame
  uint32_t dups;              // duplicate payloads dropped (normal after retry)
  uint32_t sync_resets;       // SYNC frames honoured
  uint32_t tx_residue_resets; // stale TX FIFO bytes found after the SSE
                              // toggle; nonzero means SSE=0 doesn't flush TX
} spi_link_stats_t;
const spi_link_stats_t *spi_link_stats(void);

// The ESP's 5 V drive rail (GWSPI_FLAG_RAIL): 1 on, 0 off, -1 no ESP frame
// seen since boot.
int spi_rail_state(void);
// True once after the rail was seen off (head position is now unknown).
bool spi_take_rail_dropped(void);
