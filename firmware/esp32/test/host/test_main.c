// Host tests for the offline decode pipeline: crc16, flux_decode, mfm_decode.
// Build with test/host/build.ps1 (or build.sh). No hardware required.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "crc16.h"
#include "flux_decode.h"
#include "mfm_decode.h"
#include "gw_proto.h"
#include "disk_image.h"
#include "mfm_encode.h"

static int g_fail = 0, g_checks = 0;
#define CHECK(cond, ...) do { g_checks++; if (!(cond)) { \
    g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

#define SF 72000000u   // sample clock used throughout the tests

// ---- deterministic sector payload ----
static void fill_sector(uint8_t *b, int cyl, int head, int sec)
{
    for (int i = 0; i < SECTOR_SIZE; i++)
        b[i] = (uint8_t)(sec * 7 + i * 13 + cyl * 3 + head * 5 + (i >> 4));
}

// ---- 28-bit opcode value encoder (inverse of read_28bit) ----
static void enc28(uint32_t v, uint8_t *b)
{
    b[0] = (uint8_t)(((v      ) & 0x7F) << 1) | 1;
    b[1] = (uint8_t)(((v >>  7) & 0x7F) << 1) | 1;
    b[2] = (uint8_t)(((v >> 14) & 0x7F) << 1) | 1;
    b[3] = (uint8_t)(((v >> 21) & 0x7F) << 1) | 1;
}

// =============================== tests ===============================

static void test_crc16(void)
{
    printf("test_crc16\n");
    // Canonical CRC-16/CCITT-FALSE check value for "123456789".
    CHECK(crc16_ccitt(0xFFFF, (const uint8_t *)"123456789", 9) == 0x29B1,
          "check value != 0x29B1");
    // The documented residual after three A1 sync bytes.
    uint8_t a1[3] = { 0xA1, 0xA1, 0xA1 };
    CHECK(crc16_ccitt(0xFFFF, a1, 3) == CRC16_AFTER_A1A1A1,
          "A1A1A1 residual != 0xCDB4 (got 0x%04X)", crc16_ccitt(0xFFFF, a1, 3));
}

static void test_flux_units(void)
{
    printf("test_flux_units\n");
    uint32_t ticks[16]; uint32_t idx[8];
    flux_stream_t fs = { .ticks = ticks, .ticks_cap = 16, .index = idx, .index_cap = 8 };

    // single-byte intervals
    uint8_t s1[] = { 10, 20, 0 };
    CHECK(flux_decode(s1, sizeof(s1), &fs) == 0 && fs.ticks_len == 2 &&
          ticks[0] == 10 && ticks[1] == 20, "single-byte decode");

    // two-byte interval, high=0: 250 + 0*255 + (39-1) = 288
    uint8_t s2[] = { 250, 39, 0 };
    CHECK(flux_decode(s2, sizeof(s2), &fs) == 0 && fs.ticks_len == 1 &&
          ticks[0] == 288, "two-byte decode high=0 (got %u)", ticks[0]);

    // two-byte interval, high>=1 (where 255- vs 250-base diverge):
    // byte0=251 -> high=1; 250 + 1*255 + (40-1) = 544
    uint8_t s2b[] = { 251, 40, 0 };
    CHECK(flux_decode(s2b, sizeof(s2b), &fs) == 0 && fs.ticks_len == 1 &&
          ticks[0] == 544, "two-byte decode high=1 (got %u)", ticks[0]);

    // SPACE opcode folds into the next interval: 100 + 30 = 130
    uint8_t s3[8]; int p = 0;
    s3[p++] = 255; s3[p++] = FLUXOP_SPACE; enc28(100, &s3[p]); p += 4;
    s3[p++] = 30;  s3[p++] = 0;
    CHECK(flux_decode(s3, p, &fs) == 0 && fs.ticks_len == 1 && ticks[0] == 130,
          "SPACE fold (len=%zu t0=%u)", fs.ticks_len, ticks[0]);

    // INDEX opcode is recorded, not emitted as a transition
    uint8_t s4[10]; p = 0;
    s4[p++] = 40;                                   // one transition at pos 40
    s4[p++] = 255; s4[p++] = FLUXOP_INDEX; enc28(5, &s4[p]); p += 4;
    s4[p++] = 60; s4[p++] = 0;
    CHECK(flux_decode(s4, p, &fs) == 0 && fs.ticks_len == 2 && fs.index_len == 1,
          "INDEX recorded (idx_len=%zu)", fs.index_len);

    // INDEX must NOT consume the pending accumulator ("sample cursor is
    // unaffected"): SPACE(100), INDEX(5), 30 -> ONE interval of 130 ticks,
    // index recorded at 100+5=105. (Review finding #1: the old decoder
    // flushed the pending SPACE at INDEX and emitted 30.)
    uint8_t s5[16]; p = 0;
    s5[p++] = 255; s5[p++] = FLUXOP_SPACE; enc28(100, &s5[p]); p += 4;
    s5[p++] = 255; s5[p++] = FLUXOP_INDEX; enc28(5, &s5[p]); p += 4;
    s5[p++] = 30; s5[p++] = 0;
    CHECK(flux_decode(s5, p, &fs) == 0 && fs.ticks_len == 1 &&
          ticks[0] == 130 && fs.index_len == 1 && idx[0] == 105,
          "INDEX preserves pending SPACE (t0=%u idx0=%u)", ticks[0], idx[0]);

    // round-trip random intervals through the GW encoder
    srand(1234);
    uint32_t rt_in[500], rt_out[520]; uint8_t gw[2048];
    for (int i = 0; i < 500; i++) rt_in[i] = 144 + rand() % 145; // 2..4 cells @72
    size_t glen = enc_ticks_to_gw(rt_in, 500, gw, sizeof(gw));
    flux_stream_t fs2 = { .ticks = rt_out, .ticks_cap = 520, .index = idx, .index_cap = 8 };
    int rc = flux_decode(gw, glen, &fs2);
    int match = (rc == 0 && fs2.ticks_len == 500);
    for (size_t i = 0; match && i < 500; i++) if (rt_out[i] != rt_in[i]) match = 0;
    CHECK(match, "GW round-trip (rc=%d len=%zu)", rc, fs2.ticks_len);
}

static uint8_t g_sect[DISK_SECTORS][SECTOR_SIZE];

// Shared cell buffer for the production encoder (enc_bind API). Sized like the
// firmware's WCELLS_CAP: a GAP4-tailed track is ~205k cells.
static uint8_t g_cells[300000];

// Cell index of sector s's first DATA byte in the production encoder's layout:
// 80-byte lead-in, then per sector { 12 sync + 10 ID + 22 gap2 + 12 sync +
// 4 mark } = 60 bytes before the 512-byte data field; sector stride
// 60 + 512 + 2 (CRC) + 84 (gap3) = 658 bytes; 16 cells per byte.
// Self-validating: test_mfm_error fails if this drifts from the encoder.
static size_t data_cell_start(int s)
{
    return (size_t)(80 + s * 658 + 60) * 16;
}

static void test_mfm_roundtrip(void)
{
    printf("test_mfm_track_roundtrip\n");
    const int cyl = 3, head = 1;
    for (int s = 0; s < DISK_SECTORS; s++) fill_sector(g_sect[s], cyl, head, s);

    enc_track_t tr; enc_bind(&tr, g_cells, sizeof(g_cells));
    enc_build_track(&tr, cyl, head, g_sect);
    CHECK(!tr.overflow, "encoder overflow (cap %zu)", sizeof(g_cells));

    static uint32_t ticks[300000];
    size_t n = enc_cells_to_ticks(&tr, ticks, 300000, SF);

    disk_image_reset();
    mfm_track_ctx_t ctx = { .sample_freq = SF, .exp_cyl = cyl, .exp_head = head };
    mfm_result_t r;
    mfm_decode_track(ticks, n, &ctx, &r);

    CHECK(r.sectors_ok == DISK_SECTORS, "ok=%d (want 18)", r.sectors_ok);
    CHECK(r.sectors_bad == 0, "bad=%d (want 0)", r.sectors_bad);
    CHECK(r.sectors_missing == 0, "missing=%d (want 0)", r.sectors_missing);

    int data_ok = 1;
    for (int s = 0; s < DISK_SECTORS; s++) {
        uint8_t got[SECTOR_SIZE];
        uint32_t lba = disk_lba(cyl, head, s + 1);
        disk_image_read(lba, got, 1);
        if (memcmp(got, g_sect[s], SECTOR_SIZE) != 0) data_ok = 0;
        if (!disk_sector_valid(lba)) data_ok = 0;
    }
    CHECK(data_ok, "all sector payloads match & valid");
}

// Read-back after a write: verify_compare counts a sector that reads clean
// but differs from the image as bad, and never changes the image.
static void test_mfm_verify_compare(void)
{
    printf("test_mfm_verify_compare\n");
    const int cyl = 5, head = 0;
    for (int s = 0; s < DISK_SECTORS; s++) fill_sector(g_sect[s], cyl, head, s);
    enc_track_t tr; enc_bind(&tr, g_cells, sizeof(g_cells));
    enc_build_track(&tr, cyl, head, g_sect);
    static uint32_t ticks[300000];
    size_t n = enc_cells_to_ticks(&tr, ticks, 300000, SF);

    // The image holds what was "written".
    disk_image_reset();
    for (int s = 0; s < DISK_SECTORS; s++)
        disk_image_write(disk_lba(cyl, head, s + 1), g_sect[s], 1);

    mfm_track_ctx_t ctx = { .sample_freq = SF, .exp_cyl = cyl, .exp_head = head,
                            .verify_only = 1, .verify_compare = 1 };
    mfm_result_t r;
    mfm_decode_track(ticks, n, &ctx, &r);
    CHECK(r.sectors_ok == DISK_SECTORS && r.sectors_bad == 0 && r.sectors_missing == 0,
          "matching track verifies (ok=%d bad=%d missing=%d)",
          r.sectors_ok, r.sectors_bad, r.sectors_missing);

    // The disk still has the old data for one sector (the write didn't land).
    uint8_t other[SECTOR_SIZE], got[SECTOR_SIZE];
    memset(other, 0x5A, sizeof other);
    uint32_t lba = disk_lba(cyl, head, 7);
    disk_image_write(lba, other, 1);
    mfm_decode_track(ticks, n, &ctx, &r);
    CHECK(r.sectors_ok == DISK_SECTORS - 1 && r.sectors_bad == 1,
          "a sector that differs is bad (ok=%d bad=%d)", r.sectors_ok, r.sectors_bad);
    disk_image_read(lba, got, 1);
    CHECK(memcmp(got, other, SECTOR_SIZE) == 0, "verify leaves the image alone");
    CHECK(r.sectors_stale == 1, "the differing sector counts as stale (stale=%d)",
          r.sectors_stale);

    // Every sector differs: the whole write didn't take.
    for (int s = 0; s < DISK_SECTORS; s++)
        disk_image_write(disk_lba(cyl, head, s + 1), other, 1);
    mfm_decode_track(ticks, n, &ctx, &r);
    CHECK(r.sectors_stale == DISK_SECTORS && r.sectors_bad == DISK_SECTORS,
          "all-old track: stale=%d bad=%d", r.sectors_stale, r.sectors_bad);

    // Without the compare, a clean read of the wrong data passes (CRC only).
    ctx.verify_compare = 0;
    mfm_decode_track(ticks, n, &ctx, &r);
    CHECK(r.sectors_ok == DISK_SECTORS && r.sectors_bad == 0, "CRC-only verify can't tell");
}

static void test_mfm_error(void)
{
    printf("test_mfm_error_injection\n");
    const int cyl = 10, head = 0, bad = 5;
    for (int s = 0; s < DISK_SECTORS; s++) fill_sector(g_sect[s], cyl, head, s);

    enc_track_t tr; enc_bind(&tr, g_cells, sizeof(g_cells));
    enc_build_track(&tr, cyl, head, g_sect);

    // Corrupt one DATA cell of sector `bad`: flip a 0-valued data (odd) cell to
    // 1. Odd cells carry data bits; flipping 0->1 splits an interval (never
    // exceeds the 8-cell clamp) and changes the decoded byte -> CRC fails.
    size_t base = data_cell_start(bad);
    for (size_t k = 1; ; k += 2) {
        if (tr.cells[base + k] == 0) { tr.cells[base + k] = 1; break; }
    }

    static uint32_t ticks[300000];
    size_t n = enc_cells_to_ticks(&tr, ticks, 300000, SF);

    disk_image_reset();
    mfm_track_ctx_t ctx = { .sample_freq = SF, .exp_cyl = cyl, .exp_head = head };
    mfm_result_t r;
    mfm_decode_track(ticks, n, &ctx, &r);

    CHECK(r.sectors_bad >= 1, "corrupted sector flagged bad (bad=%d)", r.sectors_bad);
    CHECK(!disk_sector_valid(disk_lba(cyl, head, bad + 1)),
          "sector %d marked invalid", bad + 1);
    int others_ok = 1;
    for (int s = 0; s < DISK_SECTORS; s++)
        if (s != bad && !disk_sector_valid(disk_lba(cyl, head, s + 1))) others_ok = 0;
    CHECK(others_ok, "other 17 sectors still valid");
}

static void test_mfm_wrong_cylinder(void)
{
    printf("test_mfm_wrong_cylinder\n");
    // Simulate a mis-seek (the MPF920 Track0 gremlin): the track's IDAMs say
    // cylinder 3, but the caller expected cylinder 4. Every sector is CRC-valid
    // yet none may reach the image (review finding #2).
    const int cyl = 3, head = 1;
    for (int s = 0; s < DISK_SECTORS; s++) fill_sector(g_sect[s], cyl, head, s);

    enc_track_t tr; enc_bind(&tr, g_cells, sizeof(g_cells));
    enc_build_track(&tr, cyl, head, g_sect);

    static uint32_t ticks[300000];
    size_t n = enc_cells_to_ticks(&tr, ticks, 300000, SF);

    disk_image_reset();
    mfm_track_ctx_t ctx = { .sample_freq = SF, .exp_cyl = cyl + 1, .exp_head = head };
    mfm_result_t r;
    mfm_decode_track(ticks, n, &ctx, &r);

    CHECK(r.sectors_ok == 0, "wrong-cyl sectors not counted ok (ok=%d)", r.sectors_ok);
    CHECK(r.sectors_missing == DISK_SECTORS,
          "wrong-cyl sectors counted missing (missing=%d)", r.sectors_missing);
    CHECK(disk_sectors_done() == 0,
          "nothing written to the image (done=%u)", disk_sectors_done());
}

// Timing margin: exact cells score 100; cell-time noise lowers it, roughly
// as 100 * (0.5 - worst error) / 0.5; the track still decodes.
static void test_mfm_margin(void)
{
    printf("test_mfm_margin\n");
    const int cyl = 9, head = 1;
    for (int s = 0; s < DISK_SECTORS; s++) fill_sector(g_sect[s], cyl, head, s);
    enc_track_t tr; enc_bind(&tr, g_cells, sizeof(g_cells));
    enc_build_track(&tr, cyl, head, g_sect);
    static uint32_t ticks[300000];
    size_t n = enc_cells_to_ticks(&tr, ticks, 300000, SF);

    mfm_track_ctx_t ctx = { .sample_freq = SF, .exp_cyl = cyl, .exp_head = head,
                            .want_margin = 1 };
    mfm_result_t r;
    disk_image_reset();
    mfm_decode_track(ticks, n, &ctx, &r);
    CHECK(r.margin >= 96, "clean flux: margin %d (want ~100)", r.margin);
    CHECK(r.sectors_ok == DISK_SECTORS, "clean flux decodes (ok=%d)", r.sectors_ok);

    // Uniform noise on every interval (72 ticks per cell): +/-0.1 cell, then
    // +/-0.2. The PLL's own wobble adds to it, so the score is below
    // 100 * (0.5 - noise) / 0.5, and must fall as the noise grows.
    static uint32_t clean[300000];
    memcpy(clean, ticks, n * sizeof ticks[0]);
    int m[2];
    const int amp[2] = { 7, 14 };
    for (int k = 0; k < 2; k++) {
        srand(7);
        for (size_t i = 0; i < n; i++)
            ticks[i] = clean[i] + (uint32_t)(rand() % (2 * amp[k] + 1)) - (uint32_t)amp[k];
        disk_image_reset();
        mfm_decode_track(ticks, n, &ctx, &r);
        m[k] = r.margin;
        CHECK(r.sectors_ok == DISK_SECTORS, "noise +/-%d ticks still decodes (ok=%d)",
              amp[k], r.sectors_ok);
    }
    printf("  margins: clean 100, +/-0.1 cell %d, +/-0.2 cell %d\n", m[0], m[1]);
    CHECK(m[0] < 96 && m[0] > m[1] && m[1] > 20, "margin falls with noise (%d, %d)",
          m[0], m[1]);
    CHECK(m[0] <= 80 && m[1] <= 61, "margin within the noise bound (%d, %d)", m[0], m[1]);

    // Not asked for: stays 0.
    ctx.want_margin = 0;
    mfm_decode_track(ticks, n, &ctx, &r);
    CHECK(r.margin == 0, "margin off: %d", r.margin);
}

int main(void)
{
    if (!disk_image_init()) { printf("disk_image_init failed\n"); return 2; }
    test_crc16();
    test_flux_units();
    test_mfm_roundtrip();
    test_mfm_verify_compare();
    test_mfm_error();
    test_mfm_wrong_cylinder();
    test_mfm_margin();

    printf("\n%d checks, %d failures\n", g_checks, g_fail);
    if (g_fail == 0) printf("ALL PASS\n");
    return g_fail ? 1 : 0;
}
