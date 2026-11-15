// GWSPI link to the RP2354, at the protocol level.
//
// bu_test_rp.c proves the five wires can carry bits (it drives them through
// the RP's SIO over SWD, so it works on a blank chip). This file proves the
// two firmwares actually talk: SYNC handshake, then real Greaseweazle
// commands through the byte pipe.
//
// Must run AFTER bu_test_rp, and it pulses RUN first: bu_test_rp halts the
// RP's core over SWD, so the chip has to be handed back to its own firmware
// before it can answer anything.
#include "bu_tests.h"
#include "bu_report.h"
#include "link.h"
#include "gw_proto.h"
#include "app_config.h"

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "t_link";

// [cmd, total_len, payload...] — total_len counts the cmd and len bytes.
static void send_cmd(uint8_t cmd, const uint8_t *payload, uint8_t plen)
{
    uint8_t hdr[2] = { cmd, (uint8_t)(plen + 2) };
    link_write(hdr, 2);
    if (plen) link_write(payload, plen);
}

// Returns the ack byte, -1 on timeout, -2 on cmd echo mismatch.
static int read_ack(uint8_t expect_cmd, int to_ms)
{
    uint8_t r[2];
    int n = link_read(r, 2, to_ms);
    if (n != 2) return -1;
    if (r[0] != expect_cmd) return -2;
    return r[1];
}

// Fetch the controller's diag counters. false = nacked or short read.
static bool read_diag(struct gw_diag_counters *d)
{
    uint8_t sub = GETINFO_GW_DIAG;
    send_cmd(CMD_GET_INFO, &sub, 1);
    if (read_ack(CMD_GET_INFO, 300) != ACK_OKAY) return false;
    if (link_read((uint8_t *)d, sizeof(*d), 300) != (int)sizeof(*d)) {
        link_flush_rx();   // don't leave stream bytes to be read as the next ack
        return false;
    }
    return true;
}

// One GetInfo(FIRMWARE) round-trip; true if it completed cleanly.
static bool ping(void)
{
    struct gw_info_firmware fw;
    uint8_t sub = GETINFO_FIRMWARE;
    send_cmd(CMD_GET_INFO, &sub, 1);
    if (read_ack(CMD_GET_INFO, 500) != ACK_OKAY) return false;
    return link_read((uint8_t *)&fw, sizeof(fw), 500) == (int)sizeof(fw);
}


// ---- bulk transfer: SOURCE_BYTES / SINK_BYTES ----
//
// GetInfo only moves 32 bytes, so it barely stretches the 4096-byte framing.
// These two push real volume and verify every byte against the controller's
// PRNG, in both directions, with a throughput number. Protocol (from
// firmware/rp2354/src/gw_floppy.c): payload is {nr_bytes u32, seed u32}.
// SOURCE ends with no status byte; SINK ends with ONE raw status byte.
#define BULK_N      (128u * 1024u)
#define BULK_CHUNK  4096u
#define BULK_SEED   0x12345678u

// Must match ss_rand_next() in gw_floppy.c exactly.
static uint32_t ss_rand_next(uint32_t x)
{
    return (x & 1u) ? (x >> 1) ^ 0x80000062u : x >> 1;
}

static void put_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static uint8_t s_bulk[BULK_CHUNK];   // static: too big for the main task stack

// Controller -> ESP. Every byte checked against the PRNG.
static void bulk_source(void)
{
    uint8_t pl[8];
    uint32_t r = BULK_SEED, got = 0, bad = 0;
    int64_t t0;
    int kbps;

    put_le32(pl, BULK_N);
    put_le32(pl + 4, BULK_SEED);
    send_cmd(CMD_SOURCE_BYTES, pl, 8);
    if (read_ack(CMD_SOURCE_BYTES, 500) != ACK_OKAY) {
        bu_report("GWSPI SOURCE_BYTES", BU_FAIL, "no ack (controller lacks cmd 18?)");
        return;
    }

    t0 = esp_timer_get_time();
    while (got < BULK_N) {
        uint32_t want = BULK_N - got;
        int n;
        if (want > BULK_CHUNK) want = BULK_CHUNK;
        n = link_read(s_bulk, want, 3000);
        if (n <= 0) break;                       // stalled
        for (int i = 0; i < n; i++) {
            if (s_bulk[i] != (uint8_t)r) bad++;
            r = ss_rand_next(r);
        }
        got += (uint32_t)n;
    }
    kbps = (int)((int64_t)got * 1000000 / (esp_timer_get_time() - t0) / 1024);

    if (got != BULK_N)
        bu_report("GWSPI SOURCE_BYTES", BU_FAIL, "stalled at %lu/%lu bytes",
                  (unsigned long)got, (unsigned long)BULK_N);
    else if (bad)
        bu_report("GWSPI SOURCE_BYTES", BU_FAIL, "%lu of %lu bytes corrupted",
                  (unsigned long)bad, (unsigned long)BULK_N);
    else
        bu_report("GWSPI SOURCE_BYTES", BU_PASS,
                  "%lu KB in, byte-exact, %d KB/s (target >=500)",
                  (unsigned long)(BULK_N / 1024), kbps);
}

// ESP -> controller. The controller checks and replies one status byte.
static void bulk_sink(void)
{
    uint8_t pl[8], st = 0xFF;
    uint32_t r = BULK_SEED, sent = 0;
    int64_t t0;
    int kbps, n;

    put_le32(pl, BULK_N);
    put_le32(pl + 4, BULK_SEED);
    send_cmd(CMD_SINK_BYTES, pl, 8);
    if (read_ack(CMD_SINK_BYTES, 500) != ACK_OKAY) {
        bu_report("GWSPI SINK_BYTES", BU_FAIL, "no ack (controller lacks cmd 19?)");
        return;
    }

    t0 = esp_timer_get_time();
    while (sent < BULK_N) {
        uint32_t want = BULK_N - sent;
        if (want > BULK_CHUNK) want = BULK_CHUNK;
        for (uint32_t i = 0; i < want; i++) {
            s_bulk[i] = (uint8_t)r;
            r = ss_rand_next(r);
        }
        if (link_write(s_bulk, want) != (int)want) break;
        sent += want;
    }
    kbps = (int)((int64_t)sent * 1000000 / (esp_timer_get_time() - t0) / 1024);

    n = link_read(&st, 1, 2000);   // one RAW status byte, not [cmd,ack]
    if (sent != BULK_N)
        bu_report("GWSPI SINK_BYTES", BU_FAIL, "stalled at %lu/%lu bytes",
                  (unsigned long)sent, (unsigned long)BULK_N);
    else if (n != 1)
        bu_report("GWSPI SINK_BYTES", BU_FAIL, "no status byte after %lu KB",
                  (unsigned long)(BULK_N / 1024));
    else if (st != ACK_OKAY)
        bu_report("GWSPI SINK_BYTES", BU_FAIL,
                  "controller reports status %u - it saw corrupted bytes", st);
    else
        bu_report("GWSPI SINK_BYTES", BU_PASS,
                  "%lu KB out, controller verified byte-exact, %d KB/s (target >=500)",
                  (unsigned long)(BULK_N / 1024), kbps);
}

void bu_test_link(void)
{
    enum { PING_N = 16 };
    struct gw_info_firmware info;
    struct gw_diag_counters d0, d1;
    uint32_t dferr, dovr, drt, dseq;
    uint8_t sub;
    int ack, n, i, pings_ok;
    bool stable;

    ESP_LOGI(TAG, "--- GWSPI link to RP2354 (SPI3 SCLK%d MOSI%d MISO%d CS%d DRDY%d) ---",
             GW_SPI_SCLK_GPIO, GW_SPI_MOSI_GPIO, GW_SPI_MISO_GPIO,
             GW_SPI_CS_GPIO, GW_SPI_DRDY_GPIO);

    // Hand the chip back to its own firmware after bu_test_rp's SWD session.
    link_peer_reset();

    if (link_up() != ESP_OK) {
        bu_report("GWSPI link up", BU_FAIL,
                  "SYNC never answered in %d ms — RP firmware running? (blank RP = expected FAIL)",
                  GW_SYNC_TIMEOUT_MS);
        return;
    }
    bu_report("GWSPI link up", BU_PASS,
              "SYNC handshake OK, mode 1 @ %d MHz, gap %d us",
              GW_SPI_CLK_HZ / 1000000, GW_SPI_GAP_US);

    // ---- GetInfo(FIRMWARE): first real round-trip through the pipe ----
    sub = GETINFO_FIRMWARE;
    send_cmd(CMD_GET_INFO, &sub, 1);
    ack = read_ack(CMD_GET_INFO, 500);
    if (ack != ACK_OKAY) {
        bu_report("GWSPI GetInfo", BU_FAIL, "no ack (%d) — link up but command engine silent", ack);
        goto done;
    }
    n = link_read((uint8_t *)&info, sizeof(info), 500);
    if (n != (int)sizeof(info)) {
        bu_report("GWSPI GetInfo", BU_FAIL, "short read %d/%u bytes", n, (unsigned)sizeof(info));
        goto done;
    }
    bu_report("GWSPI GetInfo", info.is_main_firmware ? BU_PASS : BU_WARN,
              "fw %u.%u, %s, max_cmd %u, sample %lu Hz",
              info.fw_ver_major, info.fw_ver_minor,
              info.is_main_firmware ? "main firmware" : "BOOTLOADER/not ready",
              info.max_cmd, (unsigned long)info.sample_freq);

    // ---- Link stability: counters must not move under traffic ----
    // A single frame error at SYNC is an artefact of the slave arming its DMA
    // while the master is already polling; one that grows per command is a
    // real link fault. So measure the DELTA over N round-trips, not the
    // absolute value.
    if (!read_diag(&d0)) {
        bu_report("GWSPI diag counters", BU_WARN,
                  "controller nacked 0x88 or short read - no first-party diag");
        goto done;
    }
    pings_ok = 0;
    for (i = 0; i < PING_N; i++) if (ping()) pings_ok++;

    if (!read_diag(&d1)) {
        bu_report("GWSPI link stability", BU_WARN, "diag unreadable after traffic");
        goto done;
    }
    dferr = d1.spi_frame_errors  - d0.spi_frame_errors;
    dovr  = d1.spi_rx_overruns   - d0.spi_rx_overruns;
    drt   = d1.spi_retransmits   - d0.spi_retransmits;
    dseq  = d1.spi_seq_errs      - d0.spi_seq_errs;
    stable = (dferr == 0 && dovr == 0 && drt == 0 && dseq == 0);

    bu_report("GWSPI diag counters", BU_INFO,
              "at rest: ovr=%lu ferr=%lu rt=%lu seq=%lu badcmd=%lu badlen=%lu | esp: %s",
              (unsigned long)d1.spi_rx_overruns, (unsigned long)d1.spi_frame_errors,
              (unsigned long)d1.spi_retransmits, (unsigned long)d1.spi_seq_errs,
              (unsigned long)d1.proto_bad_cmd, (unsigned long)d1.proto_bad_len,
              link_stats_str());

    bu_report("GWSPI link stability", (stable && pings_ok == PING_N) ? BU_PASS : BU_FAIL,
              "%d/%d GetInfo round-trips OK; deltas over that traffic: ovr=+%lu ferr=+%lu rt=+%lu seq=+%lu",
              pings_ok, PING_N, (unsigned long)dovr, (unsigned long)dferr,
              (unsigned long)drt, (unsigned long)dseq);

    // ---- bulk, both directions, then confirm the counters still held ----
    bulk_source();
    bulk_sink();
    if (read_diag(&d1)) {
        bu_report("GWSPI counters after bulk", BU_INFO,
                  "ovr=%lu ferr=%lu rt=%lu seq=%lu badcmd=%lu badlen=%lu | esp: %s",
                  (unsigned long)d1.spi_rx_overruns, (unsigned long)d1.spi_frame_errors,
                  (unsigned long)d1.spi_retransmits, (unsigned long)d1.spi_seq_errs,
                  (unsigned long)d1.proto_bad_cmd, (unsigned long)d1.proto_bad_len,
                  link_stats_str());
    }

done:
    link_down();   // release SPI3 so an S1 re-run can bring it up cleanly
}
