#include "mfm_encode.h"
#include "crc16.h"
#include <string.h>

#define A1_RAW 0x4489   // A1 with its missing clock

static void push(enc_track_t *t, int bit)
{
    if (t->ncells >= t->cap) { t->overflow = true; return; }
    t->cells[t->ncells++] = (uint8_t)(bit & 1);
}

// One MFM data byte: 8 (clock,data) cell pairs, MSB first.
static void enc_byte(enc_track_t *t, int *prev, uint8_t b)
{
    for (int i = 7; i >= 0; i--) {
        int d = (b >> i) & 1;
        int c = !(*prev | d);          // MFM clock rule
        push(t, c);
        push(t, d);
        *prev = d;
    }
}

static void enc_bytes(enc_track_t *t, int *prev, uint8_t v, int count)
{
    for (int i = 0; i < count; i++) enc_byte(t, prev, v);
}

// Raw 0x4489 verbatim; the LSB of 0xA1 is 1.
static void enc_a1(enc_track_t *t, int *prev)
{
    for (int i = 15; i >= 0; i--) push(t, (A1_RAW >> i) & 1);
    *prev = 1;
}

void enc_bind(enc_track_t *t, uint8_t *cells, size_t cap)
{
    t->cells = cells; t->cap = cap; t->ncells = 0; t->overflow = false;
}

void enc_build_track(enc_track_t *t, uint8_t cyl, uint8_t head,
                     const uint8_t sect[][SECTOR_SIZE])
{
    // Standard IBM 1.44 MB gaps; tighter ones read back badly from real media.
    int prev = 0;
    enc_bytes(t, &prev, 0x4E, 80);                 // post-index gap (GAP4a/GAP1)

    for (int s = 0; s < DISK_SECTORS; s++) {
        // ---- ID field ----
        enc_bytes(t, &prev, 0x00, 12);             // sync
        uint8_t idf[8] = { 0xA1, 0xA1, 0xA1, 0xFE, cyl, head, (uint8_t)(s + 1), 2 };
        uint16_t crc = crc16_ccitt(0xFFFF, idf, 8);
        enc_a1(t, &prev); enc_a1(t, &prev); enc_a1(t, &prev);
        enc_byte(t, &prev, 0xFE);
        enc_byte(t, &prev, cyl); enc_byte(t, &prev, head);
        enc_byte(t, &prev, (uint8_t)(s + 1)); enc_byte(t, &prev, 2);
        enc_byte(t, &prev, crc >> 8); enc_byte(t, &prev, crc & 0xFF);

        enc_bytes(t, &prev, 0x4E, 22);             // gap2

        // ---- DATA field ----
        enc_bytes(t, &prev, 0x00, 12);             // sync
        uint8_t dmark[4] = { 0xA1, 0xA1, 0xA1, 0xFB };
        crc = crc16_ccitt(0xFFFF, dmark, 4);
        crc = crc16_ccitt(crc, sect[s], SECTOR_SIZE);
        enc_a1(t, &prev); enc_a1(t, &prev); enc_a1(t, &prev);
        enc_byte(t, &prev, 0xFB);
        for (int i = 0; i < SECTOR_SIZE; i++) enc_byte(t, &prev, sect[s][i]);
        enc_byte(t, &prev, crc >> 8); enc_byte(t, &prev, crc & 0xFF);

        enc_bytes(t, &prev, 0x4E, 84);             // gap3 (standard for 512B/18-sector)
    }
    // GAP4 runs past one revolution (~205 ms vs 203 ms for a slow spindle) so
    // the controller cuts the write at the index, in the gap.
    enc_bytes(t, &prev, 0x4E, 900);
}

size_t enc_cells_to_ticks(const enc_track_t *t, uint32_t *ticks, size_t cap,
                          uint32_t sample_freq)
{
    uint32_t ct = sample_freq / 1000000u;          // ticks per cell
    size_t n = 0;
    long last = -1;
    for (size_t i = 0; i < t->ncells; i++) {
        if (!t->cells[i]) continue;
        if (last >= 0) {
            if (n >= cap) break;
            ticks[n++] = (uint32_t)((i - last) * ct);
        }
        last = (long)i;
    }
    return n;
}

size_t enc_ticks_to_gw(const uint32_t *ticks, size_t n, uint8_t *out, size_t cap)
{
    size_t len = 0;
    for (size_t i = 0; i < n; i++) {
        uint32_t v = ticks[i];
        if (v < 1) v = 1;
        if (v < 250) {
            if (len + 1 >= cap) break;
            out[len++] = (uint8_t)v;
        } else {
            // MFM intervals never exceed the two-byte range (1524).
            if (len + 2 >= cap) break;
            out[len++] = (uint8_t)(250 + (v - 250) / 255);
            out[len++] = (uint8_t)((v - 250) % 255 + 1);
        }
    }
    if (len < cap) out[len++] = 0;                 // stream terminator
    return len;
}
