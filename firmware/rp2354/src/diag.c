// SPDX-License-Identifier: Unlicense
// CDC1 debug console. Output is dropped, never blocked on, when no terminal is
// draining it.
#include "diag.h"

#include <stdio.h>
#include <string.h>

#include "tusb.h"

#include "board_rp2354.h"
#include "transport_spi.h"

gw_diag_t g_diag;

#ifndef GW_BUILD_ID
#define GW_BUILD_ID "dev"
#endif

const char *diag_version(void) { return GW_BUILD_ID; }

void diag_init(void) { memset(&g_diag, 0, sizeof(g_diag)); }

static void cons_puts(const char *s) {
  uint32_t n = (uint32_t)strlen(s);
  if (!tud_cdc_n_connected(1) || tud_cdc_n_write_available(1) < n)
    return;
  tud_cdc_n_write(1, s, n);
  tud_cdc_n_write_flush(1);
}

static void cons_stats(void) {
  char line[192];
  snprintf(line, sizeof(line),
           "spi: ovr=%lu ferr=%lu rt=%lu seq=%lu | proto: cmd=%lu len=%lu\r\n",
           (unsigned long)g_diag.spi_rx_overruns,
           (unsigned long)g_diag.spi_frame_errors,
           (unsigned long)g_diag.spi_retransmits,
           (unsigned long)g_diag.spi_seq_errs,
           (unsigned long)g_diag.proto_bad_cmd,
           (unsigned long)g_diag.proto_bad_len);
  cons_puts(line);
  snprintf(line, sizeof(line),
           "arb: usb=%lu spi=%lu idle=%lu | flux: rd=%lu wr=%lu ovf=%lu "
           "unf=%lu noidx=%lu | wdt=%lu\r\n",
           (unsigned long)g_diag.arb_to_usb, (unsigned long)g_diag.arb_to_spi,
           (unsigned long)g_diag.arb_idle_releases,
           (unsigned long)g_diag.flux_reads, (unsigned long)g_diag.flux_writes,
           (unsigned long)g_diag.flux_overflow,
           (unsigned long)g_diag.flux_underflow,
           (unsigned long)g_diag.flux_no_index,
           (unsigned long)g_diag.wdt_quiesces);
  cons_puts(line);
  const spi_link_stats_t *ls = spi_link_stats();
  snprintf(line, sizeof(line),
           "link: xchg=%lu short=%lu dup=%lu sync=%lu txres=%lu\r\n",
           (unsigned long)ls->exchanges, (unsigned long)ls->short_frames,
           (unsigned long)ls->dups, (unsigned long)ls->sync_resets,
           (unsigned long)ls->tx_residue_resets);
  cons_puts(line);
}

void diag_poll(void) {
  while (tud_cdc_n_available(1)) {
    uint8_t c;
    tud_cdc_n_read(1, &c, 1);
    switch (c) {
    case 's':
      cons_stats();
      break;
    case 'z':
      memset(&g_diag, 0, sizeof(g_diag));
      cons_puts("counters zeroed\r\n");
      break;
    case 'v': {
      char line[96];
      snprintf(line, sizeof(line),
               "gw-rp2354 fw %u.%u build %s sample=%u Hz\r\n",
               (unsigned)GW_FW_MAJOR, (unsigned)GW_FW_MINOR, diag_version(),
               (unsigned)GW_SAMPLE_FREQ);
      cons_puts(line);
      break;
    }
    default:
      break;
    }
  }
}
