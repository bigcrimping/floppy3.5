// Greaseweazle flux byte stream -> transition intervals in sample ticks, with
// index positions reported separately.
#pragma once
#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint32_t *ticks;      // caller-provided: interval (ticks) between transitions
    size_t    ticks_cap;  // capacity of ticks[]
    size_t    ticks_len;  // filled by decoder

    uint32_t *index;      // caller-provided: cumulative tick position of each index
    size_t    index_cap;  // markers past this are dropped, not an error
    size_t    index_len;
} flux_stream_t;

// `raw` must end with the 0x00 terminator. Returns 0, or negative on malformed
// input or a full ticks[].
int flux_decode(const uint8_t *raw, size_t raw_len, flux_stream_t *fs);
