// SPDX-License-Identifier: Unlicense
// Whole-track flux staging over the PIO engine. Returns gw_ack codes.
#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "flux_pio.h"

// FLUX_BUF_SZ bytes. Also scratch for SOURCE/SINK_BYTES.
extern uint8_t flux_buf[];

void flux_init(void);

// Capture into flux_buf: capture_counts 25 MHz ticks, or one index-to-index
// revolution if 0. See flux_pio_capture() for cue_ms/cue_marker.
// Returns ACK_OKAY, ACK_NO_INDEX (cue timeout, silent RDDATA, unclosed
// revolution) or ACK_FLUX_OVERFLOW.
// With `live`, the capture's bytes are handed out as they are encoded (see
// flux_pio_capture()) and the link stays fully open: no busy bracket.
uint8_t flux_capture(uint32_t capture_counts, uint32_t cue_ms, bool cue_marker,
                     flux_live_fn live, uint32_t *len);

// Write flux_buf[0..len), terminator excluded. Returns ACK_OKAY,
// ACK_NO_INDEX or ACK_FLUX_UNDERFLOW.
uint8_t flux_write(uint32_t len, bool cue_at_index, bool terminate_at_index);

// False if the stream holds anything we can't write (INDEX/ASTABLE/unknown
// opcodes, embedded terminators, truncated codes).
bool flux_stream_valid(const uint8_t *s, uint32_t len);
