// flux intervals -> PLL -> raw cells -> A1 sync hunt -> ID/DATA fields -> CRC
// -> disk image.
#include "mfm_decode.h"
#include "disk_image.h"
#include "crc16.h"
#include "app_config.h"
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "mfm";

#if GW_PROFILE
uint64_t g_mfm_pll_us = 0;   // pll_to_raw time per dump, for app_main
#endif

// 500 kbit/s HD data = 1 MHz raw cells.
#define MFM_CELL_HZ         1000000u
// A 220 ms capture is ~220k cells; anything past this is dropped.
#define MAX_RAW_BITS        220000
// MFM address marks.
#define MARK_IDAM           0xFE
#define MARK_DAM            0xFB
#define MARK_DDAM           0xF8   // deleted data — treat like DAM
// Max gap from an ID field's CRC to its data field's A1: 34 bytes on the IBM
// format, doubled for odd formatters. The next sector's data is ~600 bytes on.
#define IDAM_TO_DAM_MAX_CELLS (64 * 16)

static uint8_t *s_raw;             // raw cells, MSB-first, PSRAM

static bool ensure_buf(void)
{
    if (!s_raw) s_raw = heap_caps_malloc(MAX_RAW_BITS / 8, MALLOC_CAP_SPIRAM);
    return s_raw != NULL;
}

static inline void raw_set(uint32_t i, int v)
{
    if (v) s_raw[i >> 3] |=  (uint8_t)(1u << (7 - (i & 7)));
    else   s_raw[i >> 3] &= (uint8_t)~(1u << (7 - (i & 7)));
}
static inline int raw_get(uint32_t i)
{
    return (s_raw[i >> 3] >> (7 - (i & 7))) & 1;
}

// -------- PLL: flux intervals -> raw cells --------
// Margin histogram: |phase error| in 0.01-cell bins, 0..0.50 (see mfm_result_t).
#define MARGIN_BINS 51

// Cells go into a 64-bit accumulator and out to s_raw 32 at a time, MSB first
// (the layout raw_get() reads): far cheaper than a read-modify-write per bit.
// `hist` (MARGIN_BINS, zeroed by the caller) is filled if non-NULL.
static uint32_t pll_to_raw(const uint32_t *ticks, size_t n, uint32_t sample_freq,
                           uint32_t *hist)
{
    // float only: the S3 does double in software, and this runs per transition.
    float nominal = (float)(sample_freq ? sample_freq : 72000000u) / MFM_CELL_HZ;
    float cell = nominal;
    const float lo = nominal * 0.85f, hi = nominal * 1.15f;
    static const float inv_n[9] = { 0, 1.0f, 1 / 2.0f, 1 / 3.0f, 1 / 4.0f,
                                    1 / 5.0f, 1 / 6.0f, 1 / 7.0f, 1 / 8.0f };
    uint64_t acc = 0;
    int      nacc = 0;                    // bits waiting in acc (< 32 between steps)
    uint32_t out = 0;                     // bits produced

    for (size_t i = 0; i < n; i++) {
        float t = (float)ticks[i];
        float x = t / cell;
        int cells = (int)(x + 0.5f);
        if (cells < 1) cells = 1;
        if (cells > 8) cells = 8;              // dropouts
        if (out + (uint32_t)cells > MAX_RAW_BITS) break;
        if (hist) {
            float e = x - (float)cells;
            if (e < 0) e = -e;
            int b = (cells >= 2 && cells <= 4) ? (int)(e * 100.0f + 0.5f) : MARGIN_BINS - 1;
            hist[b < MARGIN_BINS ? b : MARGIN_BINS - 1]++;
        }
        // cells-1 zeros then a one.
        acc = (acc << cells) | 1u;
        nacc += cells;
        out  += (uint32_t)cells;
        if (nacc >= 32) {
            uint32_t w = (uint32_t)(acc >> (nacc - 32));
            uint8_t *d = &s_raw[(out - (uint32_t)nacc) >> 3];
            d[0] = (uint8_t)(w >> 24); d[1] = (uint8_t)(w >> 16);
            d[2] = (uint8_t)(w >> 8);  d[3] = (uint8_t)w;
            nacc -= 32;
            acc &= nacc ? ((1ull << nacc) - 1) : 0;
        }
        // Nudge the cell estimate toward the observed spacing, within +/-15%.
        float err = t * inv_n[cells] - cell;
        cell += err * 0.15f;
        if (cell < lo) cell = lo;
        if (cell > hi) cell = hi;
    }
    // The tail, bit by bit.
    for (int k = nacc - 1; k >= 0; k--)
        raw_set(out - 1 - (uint32_t)k, (int)((acc >> k) & 1u));
    return out;
}

// 100 * (0.5 - p99.9 error) / 0.5 from the histogram (see mfm_result_t).
static int margin_from_hist(const uint32_t *hist)
{
    uint32_t total = 0;
    for (int b = 0; b < MARGIN_BINS; b++) total += hist[b];
    if (!total) return 0;
    uint32_t tail = total / 1000, seen = 0;   // the worst 0.1%
    int b = MARGIN_BINS - 1;
    for (; b > 0; b--) {
        seen += hist[b];
        if (seen > tail) break;
    }
    int m = (50 - b) * 2;                     // bin b = b/100 of a cell
    return m < 0 ? 0 : m > 100 ? 100 : m;
}

// One byte from 16 raw cells; data bits are the odd cells.
static inline uint8_t mfm_byte(uint32_t p)
{
    uint8_t b = 0;
    for (int j = 0; j < 8; j++) b = (b << 1) | raw_get(p + 2 * j + 1);
    return b;
}

// Start of the next A1 sync (raw 0x4489) at or after `from`, or `nbits`.
static uint32_t next_sync(uint32_t from, uint32_t nbits)
{
    if (from + 16 > nbits) return nbits;
    uint16_t w = 0;
    uint32_t q = from;
    for (int j = 0; j < 15; j++) w = (uint16_t)((w << 1) | raw_get(q++));
    while (q < nbits) {
        w = (uint16_t)((w << 1) | raw_get(q++));   // w = raw bits [q-16, q)
        if (w == 0x4489) return q - 16;
    }
    return nbits;
}

void mfm_decode_track(const uint32_t *ticks, size_t ticks_len,
                      const mfm_track_ctx_t *ctx, mfm_result_t *res)
{
    memset(res, 0, sizeof(*res));
    if (!ensure_buf()) { ESP_LOGE(TAG, "no PSRAM for raw buffer"); return; }

#if GW_PROFILE
    int64_t pll_t0 = esp_timer_get_time();
#endif
    static uint32_t hist[MARGIN_BINS];
    if (ctx->want_margin) memset(hist, 0, sizeof hist);
    uint32_t nbits = pll_to_raw(ticks, ticks_len, ctx->sample_freq,
                                ctx->want_margin ? hist : NULL);
    if (ctx->want_margin) res->margin = margin_from_hist(hist);
#if GW_PROFILE
    g_mfm_pll_us += (uint64_t)(esp_timer_get_time() - pll_t0);
#endif

    bool     have_id = false;
    bool     warned_mismatch = false;      // one wrong-track warning per call
    uint8_t  id_cyl = 0, id_head = 0, id_sec = 0;
    uint32_t id_end = 0;                   // raw-cell position just past the ID CRC
    int      recovered_mask[DISK_SECTORS + 1]; // dedupe by sector number 1..18
    memset(recovered_mask, 0, sizeof(recovered_mask));

    uint32_t p = next_sync(0, nbits);
    while (p + 16 <= nbits) {
        // Walk the A1 run, feeding it to the CRC.
        uint16_t crc = 0xFFFF;
        uint32_t q  = p;
        int a1s = 0;
        while (q + 16 <= nbits && mfm_byte(q) == 0xA1 && a1s < 4) {
            uint8_t a1 = 0xA1;
            crc = crc16_ccitt(crc, &a1, 1);
            q += 16; a1s++;
        }
        if (a1s < 1 || q + 16 > nbits) { p = next_sync(p + 1, nbits); continue; }

        uint8_t mark = mfm_byte(q);
        crc = crc16_ccitt(crc, &mark, 1);
        q += 16;

        if (mark == MARK_IDAM) {
            if (q + 6 * 16 > nbits) { p++; continue; }
            uint8_t hdr[4];
            for (int k = 0; k < 4; k++) { hdr[k] = mfm_byte(q); q += 16; }
            uint8_t chi = mfm_byte(q); q += 16;
            uint8_t clo = mfm_byte(q); q += 16;
            crc = crc16_ccitt(crc, hdr, 4);
            uint16_t stored = (uint16_t)(chi << 8) | clo;
            if (crc == stored) {
                id_cyl = hdr[0]; id_head = hdr[1]; id_sec = hdr[2];
                have_id = (id_sec >= 1 && id_sec <= DISK_SECTORS);
                id_end = q;
            } else {
                have_id = false;
            }
            p = next_sync(q, nbits);
            continue;
        }

        if (mark == MARK_DAM || mark == MARK_DDAM) {
            if (q + (SECTOR_SIZE + 2) * 16 > nbits) { p++; continue; }
            static uint8_t data[SECTOR_SIZE];
            for (int k = 0; k < SECTOR_SIZE; k++) { data[k] = mfm_byte(q); q += 16; }
            uint8_t chi = mfm_byte(q); q += 16;
            uint8_t clo = mfm_byte(q); q += 16;
            crc = crc16_ccitt(crc, data, SECTOR_SIZE);
            uint16_t stored = (uint16_t)(chi << 8) | clo;
            bool ok = (crc == stored);

            // Too far from the last ID: both that ID's data and this data's ID
            // were missed, so don't file this under the wrong sector number.
            if (have_id && p - id_end > IDAM_TO_DAM_MAX_CELLS)
                have_id = false;

            if (have_id) {
                if (id_cyl == ctx->exp_cyl && id_head == ctx->exp_head) {
                    uint32_t lba = disk_lba(id_cyl, id_head, id_sec);
                    bool stale = false;
                    if (!ctx->verify_only) {
                        disk_image_put(lba, data, ok);
                    } else if (ok && ctx->verify_compare) {
                        static uint8_t want[SECTOR_SIZE];
                        if (!disk_image_read(lba, want, 1) ||
                            memcmp(want, data, SECTOR_SIZE) != 0) {
                            ok = false;
                            stale = true;
                        }
                    }
                    if (!recovered_mask[id_sec]) {
                        recovered_mask[id_sec] = 1;
                        if (ok) res->sectors_ok++; else res->sectors_bad++;
                        if (stale) res->sectors_stale++;
                    }
                } else if (!warned_mismatch) {
                    // Mis-seek: never store sectors from the wrong track.
                    warned_mismatch = true;
                    ESP_LOGW(TAG, "IDAM c%u h%u != expected c%u h%u: dropping",
                             (unsigned)id_cyl, (unsigned)id_head,
                             (unsigned)ctx->exp_cyl, (unsigned)ctx->exp_head);
                }
                have_id = false;
            }
            p = next_sync(q, nbits);
            continue;
        }

        p = next_sync(p + 1, nbits);     // unknown mark
    }

    int seen = 0;
    for (int s = 1; s <= DISK_SECTORS; s++) if (recovered_mask[s]) seen++;
    res->sectors_missing = DISK_SECTORS - seen;
}
