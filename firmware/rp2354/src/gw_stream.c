// SPDX-License-Identifier: Unlicense
#include "gw_stream.h"

#include "gw_proto.h"

static uint8_t *w28(uint8_t *p, uint32_t x) {
  *p++ = 1 | (uint8_t)(x << 1);
  *p++ = 1 | (uint8_t)(x >> 6);
  *p++ = 1 | (uint8_t)(x >> 13);
  *p++ = 1 | (uint8_t)(x >> 20);
  return p;
}

uint32_t gw_stream_enc_interval(uint8_t *dst, uint32_t ticks) {
  if (ticks == 0)
    ticks = 1;
  if (ticks < 250) {
    dst[0] = (uint8_t)ticks;
    return 1;
  }
  if (ticks <= 1524) {
    uint32_t high = (ticks - 250) / 255;
    dst[0] = (uint8_t)(250 + high);
    dst[1] = (uint8_t)(1 + (ticks - 250) % 255);
    return 2;
  }
  // Clamped at the u28 limit (10.7 s), far beyond any real capture.
  uint32_t space = ticks - 249;
  if (space > GW_STREAM_U28_MAX)
    space = GW_STREAM_U28_MAX;
  dst[0] = 0xFF;
  dst[1] = FLUXOP_SPACE;
  w28(dst + 2, space);
  dst[6] = 249;
  return 7;
}

uint32_t gw_stream_enc_index(uint8_t *dst, uint32_t ticks_ago) {
  if (ticks_ago > GW_STREAM_U28_MAX)
    ticks_ago = GW_STREAM_U28_MAX;
  dst[0] = 0xFF;
  dst[1] = FLUXOP_INDEX;
  w28(dst + 2, ticks_ago);
  return GW_STREAM_INDEX_LEN;
}

void gw_stream_dec_init(gw_stream_dec_t *d, const uint8_t *s, uint32_t len) {
  d->p = s;
  d->end = s + len;
  d->pending = 0;
}

int gw_stream_dec_next(gw_stream_dec_t *d, uint32_t *ticks) {
  while (d->p < d->end) {
    uint8_t b = *d->p++;
    if (b == 0)
      return -1;
    if (b < 250) {
      *ticks = d->pending + b;
      d->pending = 0;
      return 1;
    }
    if (b < 255) {
      if (d->p >= d->end)
        return -1;
      *ticks = d->pending + 250 + (uint32_t)(b - 250) * 255 + *d->p++ - 1;
      d->pending = 0;
      return 1;
    }
    // 0xFF, opcode, u28
    if (d->end - d->p < 5)
      return -1;
    uint8_t op = *d->p++;
    if (op != FLUXOP_SPACE)
      return -1;
    d->pending += ((d->p[0] & 0xFEu) >> 1) | ((uint32_t)(d->p[1] & 0xFEu) << 6) |
                  ((uint32_t)(d->p[2] & 0xFEu) << 13) |
                  ((uint32_t)(d->p[3] & 0xFEu) << 20);
    d->p += 4;
  }
  return 0;
}
