// SPDX-License-Identifier: Unlicense
// Diagnostic counters (GetInfo 0x88 + CDC1 console). Every loss or error path
// must increment one.
#pragma once
#include <stdint.h>

// The first 32 bytes are served raw by GetInfo 0x88 and mirrored in the ESP's
// gw_proto.h (checked by test/host/test_rp_flash.c). Don't reorder.
typedef struct __attribute__((packed)) {
  uint32_t spi_rx_overruns; // ring full; must stay 0
  uint32_t spi_frame_errors; // bad magic/version/hcrc/pcrc
  uint32_t spi_retransmits;
  uint32_t spi_seq_errs;
  uint32_t proto_bad_cmd;
  uint32_t proto_bad_len;
  uint32_t arb_to_usb;
  uint32_t arb_to_spi;
  uint32_t arb_idle_releases;
  uint32_t flux_reads; // captures (revolutions), not commands
  uint32_t flux_writes;
  uint32_t flux_overflow;
  uint32_t flux_underflow;
  uint32_t flux_no_index;
  uint32_t wdt_quiesces;
} gw_diag_t;

extern gw_diag_t g_diag;

void diag_init(void);
// CDC1 console: 's' stats, 'z' zero, 'v' version. Call every superloop pass.
void diag_poll(void);
const char *diag_version(void);
