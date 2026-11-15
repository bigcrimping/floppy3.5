// Minimal IBM MFM decoder for 1.44 MB HD floppies (18 x 512-byte sectors):
// one track's flux intervals -> sectors in the disk image.
#pragma once
#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint32_t sample_freq;   // flux tick clock (Hz), from GetInfo
    uint8_t  exp_cyl;       // sectors from another track are dropped
    uint8_t  exp_head;
    uint8_t  verify_only;   // tally CRCs without touching the image
    uint8_t  verify_compare;// with verify_only: a sector that differs from the
                            // image counts as bad (read-back after a write)
} mfm_track_ctx_t;

typedef struct {
    int sectors_ok;         // sectors whose ID+data CRC both passed
    int sectors_bad;        // sectors seen but with a CRC failure
    int sectors_missing;    // 18 - (distinct sectors recovered)
} mfm_result_t;

// `ticks` are intervals from flux_decode. Sectors go to disk_image_put().
void mfm_decode_track(const uint32_t *ticks, size_t ticks_len,
                      const mfm_track_ctx_t *ctx, mfm_result_t *res);
