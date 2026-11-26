// SPDX-License-Identifier: Unlicense
// Wraps the PIO engine: owns the buffer and the pre/post-write delays, and
// maps engine results to gw_ack codes. The busy brackets just let the master
// idle during a flux op; the link keeps working without them.
#include "flux.h"

#include "pico/stdlib.h"

#include "board_rp2354.h"
#include "diag.h"
#include "flux_pio.h"
#include "gw_proto.h"
#include "transport_spi.h"

// Aligned: SOURCE/SINK reuse it as a uint32_t timestamp ring.
uint8_t __attribute__((aligned(4))) flux_buf[FLUX_BUF_SZ];

void flux_init(void) { flux_pio_init(); }

uint8_t flux_capture(uint32_t capture_counts, uint32_t cue_ms, bool cue_marker,
                     flux_live_fn live, uint32_t *len) {
  if (!live)
    spi_link_prepare_busy();
  flux_cap_status_t st = flux_pio_capture(flux_buf, FLUX_BUF_SZ,
                                          capture_counts, cue_ms, cue_marker,
                                          live, len);
  if (!live)
    spi_link_end_busy();
  g_diag.flux_reads++; // counts captures (revolutions), not host commands

  switch (st) {
  case FLUX_CAP_NO_INDEX:
    g_diag.flux_no_index++;
    return ACK_NO_INDEX;
  case FLUX_CAP_OVERFLOW:
    g_diag.flux_overflow++;
    return ACK_FLUX_OVERFLOW;
  case FLUX_CAP_OK:
  default:
    break;
  }
  // Ticks mode with no flux at all: no disk or dead drive. Report NO_INDEX as
  // upstream does. Any partial capture is still returned as data.
  if (!cue_ms && *len == 0) {
    g_diag.flux_no_index++;
    return ACK_NO_INDEX;
  }
  return ACK_OKAY;
}

uint8_t flux_write(uint32_t len, bool cue_at_index, bool terminate_at_index) {
  g_diag.flux_writes++;

  sleep_us(DELAY_PRE_WRITE_US);

  spi_link_prepare_busy();
  flux_write_status_t st = flux_pio_write(flux_buf, len, cue_at_index,
                                          terminate_at_index,
                                          WRITE_CUE_TIMEOUT_MS);
  spi_link_end_busy();

  sleep_us(DELAY_POST_WRITE_US);

  switch (st) {
  case FLUX_WRITE_OK:
    return ACK_OKAY;
  case FLUX_WRITE_NO_INDEX:
    g_diag.flux_no_index++;
    return ACK_NO_INDEX;
  case FLUX_WRITE_UNDERFLOW:
  default:
    g_diag.flux_underflow++;
    return ACK_FLUX_UNDERFLOW;
  }
}

bool flux_stream_valid(const uint8_t *s, uint32_t len) {
  // Accept 1-byte and 2-byte intervals and 0xFF FLUXOP_SPACE <u28>. Anything
  // else means a host encoding bug, so fail rather than write it.
  uint32_t i = 0;
  while (i < len) {
    uint8_t b = s[i];
    if (b == 0)
      return false;
    if (b < 250) {
      i += 1;
    } else if (b < 255) {
      if (i + 2 > len)
        return false;
      i += 2;
    } else {
      if (i + 6 > len)
        return false;
      if (s[i + 1] != FLUXOP_SPACE)
        return false;
      i += 6;
    }
  }
  return true;
}
