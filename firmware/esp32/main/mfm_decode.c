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
static uint32_t pll_to_raw(const uint32_t *ticks, size_t n, uint32_t sample_freq)
{
    // float only: the S3 does double in software, and this runs per transition.
    float nominal = (float)(sample_freq ? sample_freq : 72000000u) / MFM_CELL_HZ;
    float cell = nominal;
    const float lo = nominal * 0.85f, hi = nominal * 1.15f;
    uint32_t out = 0;

    for (size_t i = 0; i < n && out < MAX_RAW_BITS; i++) {
        int cells = (int)((float)ticks[i] / cell + 0.5f);
        if (cells < 1) cells = 1;
        if (cells > 8) cells = 8;              // dropouts
        for (int z = 0; z < cells - 1 && out < MAX_RAW_BITS; z++) raw_set(out++, 0);
        if (out < MAX_RAW_BITS) raw_set(out++, 1);
        // Nudge the cell estimate toward the observed spacing, within +/-15%.
        float err = (float)ticks[i] / cells - cell;
        cell += err * 0.15f;
        if (cell < lo) cell = lo;
        if (cell > hi) cell = hi;
    }
    return out;
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
    uint32_t nbits = pll_to_raw(ticks, ticks_len, ctx->sample_freq);
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
                    if (!ctx->verify_only) {
                        disk_image_put(lba, data, ok);
                    } else if (ok && ctx->verify_compare) {
                        static uint8_t want[SECTOR_SIZE];
                        if (!disk_image_read(lba, want, 1) ||
                            memcmp(want, data, SECTOR_SIZE) != 0)
                            ok = false;
                    }
                    if (!recovered_mask[id_sec]) {
                        recovered_mask[id_sec] = 1;
                        if (ok) res->sectors_ok++; else res->sectors_bad++;
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
