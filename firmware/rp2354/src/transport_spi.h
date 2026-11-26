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

// GETINFO_GW_LINKPROF payload: free-running counters (the host diffs two
// reads). Must match struct gw_linkprof in the ESP's gw_proto.h.
typedef struct __attribute__((packed)) {
  uint32_t isr_n;        // CS-rise ISRs
  uint32_t isr_us_sum;   // time spent in them
  uint32_t isr_us_max;   // longest one (must stay under the master's gap)
  uint32_t tx_frames;    // new payload frames built
  uint32_t tx_bytes;     // their payload bytes
  uint32_t lim_queue;    // new frames cut short (or empty) by what was queued
  uint32_t lim_credit;   // ... by the master's credit
  uint32_t win_full;     // header-only frames because both window slots were
                         // in flight with data still queued
} spi_link_prof_t;
_Static_assert(sizeof(spi_link_prof_t) == 32, "linkprof must be 32 bytes");
const spi_link_prof_t *spi_link_prof(void);

// The ESP's 5 V drive rail (GWSPI_FLAG_RAIL): 1 on, 0 off, -1 no ESP frame
// seen since boot.
int spi_rail_state(void);
// True once after the rail was seen off (head position is now unknown).
bool spi_take_rail_dropped(void);
