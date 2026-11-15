// SPDX-License-Identifier: Unlicense
// Greaseweazle flux byte-stream codec, written from the grammar in keirf's
// public-domain firmware. Pure C so the host tests can run it against the
// ESP's decoder.
//
// Grammar (ticks at GW_SAMPLE_FREQ):
//   0x00                     stream terminator
//   1..249                   one-byte interval
//   250..254, b1             250 + (b0-250)*255 + (b1-1), covering 250..1524
//   0xFF FLUXOP_SPACE u28    quiet ticks, added to the next interval
//   0xFF FLUXOP_INDEX u28    index marker, u28 ticks after the last sample
// Intervals >= 1525 are SPACE(ticks-249) + 249, as upstream does. A u28 is 4
// bytes of 7 bits each in bits 1..7; bit 0 is always set so no byte is 0x00.
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define GW_STREAM_ENC_MAX 7   // longest encoded interval
#define GW_STREAM_INDEX_LEN 6 // index marker length
#define GW_STREAM_U28_MAX ((1u << 28) - 1)

// Returns bytes written (1, 2 or 7). 0 ticks is encoded as 1.
uint32_t gw_stream_enc_interval(uint8_t *dst, uint32_t ticks);

// ticks_ago: from the last emitted sample to the index edge.
uint32_t gw_stream_enc_index(uint8_t *dst, uint32_t ticks_ago);

// Write-path decoder over a stream body (terminator excluded).
typedef struct {
  const uint8_t *p, *end;
  uint32_t pending; // SPACE ticks carried into the next interval
} gw_stream_dec_t;

void gw_stream_dec_init(gw_stream_dec_t *d, const uint8_t *s, uint32_t len);
// 1 = *ticks holds the next interval; 0 = end of stream (a trailing SPACE is
// dropped, as upstream does); -1 = unwritable opcode or truncated code.
int gw_stream_dec_next(gw_stream_dec_t *d, uint32_t *ticks);
