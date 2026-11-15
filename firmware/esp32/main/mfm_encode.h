// IBM MFM track encoder (the inverse of mfm_decode) for writing a track back
// as Greaseweazle flux.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "app_config.h"

typedef struct {
    uint8_t *cells;     // raw MFM cells, 1 byte each (0/1); caller-allocated
    size_t   ncells;    // cells produced so far
    size_t   cap;       // capacity of `cells`
    bool     overflow;  // set if a push exceeded cap (result is unusable)
} enc_track_t;

// Bind a preallocated cell buffer and reset the encoder.
void   enc_bind(enc_track_t *t, uint8_t *cells, size_t cap);

// A full 18-sector track, sect[s] as sector s+1 (1:1 interleave).
void   enc_build_track(enc_track_t *t, uint8_t cyl, uint8_t head,
                       const uint8_t sect[][SECTOR_SIZE]);

// Cells -> intervals at sample_freq/1e6 ticks per cell (integer MHz only).
// Stops silently at cap; returns the count.
size_t enc_cells_to_ticks(const enc_track_t *t, uint32_t *ticks, size_t cap,
                          uint32_t sample_freq);

// Intervals -> 0-terminated GW flux stream. Stops silently at cap (still
// terminated); returns bytes including the terminator.
size_t enc_ticks_to_gw(const uint32_t *ticks, size_t n, uint8_t *out, size_t cap);
