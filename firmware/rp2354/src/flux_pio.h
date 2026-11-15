// SPDX-License-Identifier: Unlicense
// PIO flux engine (flux_read.pio / flux_write.pio + DMA). DMA decouples the
// CPU from both FIFOs, so IRQs stay enabled and the GWSPI link keeps running
// during captures and writes.
#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef enum {
  FLUX_CAP_OK = 0,
  FLUX_CAP_NO_INDEX, // cue timed out, or one-rev mode saw no closing edge
  FLUX_CAP_OVERFLOW, // dst full (or raw-sample ring overrun) before the bound
} flux_cap_status_t;

typedef enum {
  FLUX_WRITE_OK = 0,
  FLUX_WRITE_NO_INDEX,  // cue timed out (no disk / motor off)
  FLUX_WRITE_UNDERFLOW, // FIFO ran dry, or stream ended before the index
} flux_write_status_t;

void flux_pio_init(void);

// Capture Greaseweazle-coded flux into dst[0..cap).
//  - capture_counts: stop after this many ticks (0 = stop at the next index).
//  - cue_ms: if nonzero, first wait up to cue_ms for an index edge.
//  - cue_marker: emit an index marker for the cue edge itself.
// Index edges during the capture are emitted inline as FLUXOP_INDEX.
// *len is valid on every return.
flux_cap_status_t flux_pio_capture(uint8_t *dst, uint32_t cap,
                                   uint32_t capture_counts, uint32_t cue_ms,
                                   bool cue_marker, uint32_t *len);

// Write stream[0..len) (terminator excluded).
//  - cue_at_index: wait up to cue_timeout_ms for an index edge first.
//  - terminate_at_index: stop at the next index edge, dropping any surplus.
flux_write_status_t flux_pio_write(const uint8_t *stream, uint32_t len,
                                   bool cue_at_index, bool terminate_at_index,
                                   uint32_t cue_timeout_ms);
