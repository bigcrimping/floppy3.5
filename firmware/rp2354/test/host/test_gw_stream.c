// SPDX-License-Identifier: Unlicense
// Host unit test for the first-party Greaseweazle stream codec
// (src/gw_stream.c) — the piece that replaced the vendored Adafruit
// greasepack coder in this tree. Three independent implementations meet:
//   1. gw_stream.c            (this tree: capture emitter + write decoder)
//   2. mfm_encode.c contract  (ESP write-path emission, replicated verbatim)
//   3. flux_decode.c          (the ESP's REAL read-path decoder, compiled in
//                              — what actually consumes our captures)
// Properties:
//   P1  enc_interval -> dec_next is the exact inverse for 1..70000 (covers
//       the 1-, 2- and 7-byte forms; the 7-byte form closes itself with the
//       bare 249, so the round trip is exact everywhere).
//   P2  byte-identical to the ESP encoder over its range (1..1524).
//   P3  the ESP's flux_decode() recovers intervals AND index positions from
//       a composed capture stream (inline markers, keirf semantics).
//   P4  write-decoder hardening: SPACE folding, {250,1}==250, truncation,
//       unwritable opcodes, trailing space.
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "flux_decode.h" // ESP header (test/host/build.ps1 sets -I paths)
#include "gw_stream.h"

static int failures;

#define CHECK(cond, ...)                                                       \
  do {                                                                         \
    if (!(cond)) {                                                             \
      if (failures++ < 20) {                                                   \
        printf("FAIL %s:%d: ", __FILE__, __LINE__);                            \
        printf(__VA_ARGS__);                                                   \
        printf("\n");                                                          \
      }                                                                        \
    }                                                                          \
  } while (0)

// Reference emission, replicated verbatim from
// firmware/esp32/main/mfm_encode.c enc_ticks_to_gw() (the host contract).
// Only valid to 1524 — the ESP never emits longer forms ("values above the
// two-byte range don't occur in a valid MFM track").
static size_t esp_encode(uint32_t v, uint8_t out[2]) {
  if (v < 1)
    v = 1;
  if (v < 250) {
    out[0] = (uint8_t)v;
    return 1;
  }
  out[0] = (uint8_t)(250 + (v - 250) / 255);
  out[1] = (uint8_t)((v - 250) % 255 + 1);
  return 2;
}

int main(void) {
  // ---- P1: exact round trip, 1..70000 ----
  for (uint32_t v = 1; v <= 70000; v++) {
    uint8_t buf[8];
    uint32_t n = gw_stream_enc_interval(buf, v);
    uint32_t want_n = v < 250 ? 1 : v <= 1524 ? 2 : 7;
    CHECK(n == want_n, "v=%u: encoded %u bytes, want %u", v, n, want_n);

    gw_stream_dec_t d;
    gw_stream_dec_init(&d, buf, n);
    uint32_t got;
    CHECK(gw_stream_dec_next(&d, &got) == 1, "v=%u: no interval", v);
    CHECK(got == v, "v=%u: round trip gave %u", v, got);
    CHECK(gw_stream_dec_next(&d, &got) == 0, "v=%u: trailing junk", v);
  }

  // ---- P2: byte-identical to the ESP encoder, 1..1524 ----
  for (uint32_t v = 1; v <= 1524; v++) {
    uint8_t ours[8], theirs[2];
    uint32_t n = gw_stream_enc_interval(ours, v);
    size_t en = esp_encode(v, theirs);
    CHECK(n == (uint32_t)en, "v=%u: length %u vs ESP %u", v, n, (unsigned)en);
    CHECK(memcmp(ours, theirs, en) == 0, "v=%u: bytes differ (%02X%02X)", v,
          ours[0], n > 1 ? ours[1] : 0);
  }

  // ---- boundary bytes, stated literally ----
  {
    uint8_t b[8];
    CHECK(gw_stream_enc_interval(b, 249) == 1 && b[0] == 249, "249");
    CHECK(gw_stream_enc_interval(b, 250) == 2 && b[0] == 250 && b[1] == 1,
          "250 -> {250,1}");
    CHECK(gw_stream_enc_interval(b, 1524) == 2 && b[0] == 254 && b[1] == 255,
          "1524 -> {254,255}");
    CHECK(gw_stream_enc_interval(b, 1525) == 7 && b[0] == 0xFF && b[1] == 2 &&
              b[6] == 249,
          "1525 -> SPACE(1276)+249");
    // u28 layout: each byte 1 | bits<<1 (keirf _write_28bit). 1276 = 0x4FC.
    CHECK(b[2] == (1 | ((1276u << 1) & 0xFF)), "u28 b0");
    CHECK(b[3] == (1 | ((1276u >> 6) & 0xFF)), "u28 b1");
    CHECK(b[4] == 1 && b[5] == 1, "u28 b2/b3 empty");
    // index marker: 6 bytes, op 1, u28(0) = all 0x01
    CHECK(gw_stream_enc_index(b, 0) == 6 && b[0] == 0xFF && b[1] == 1 &&
              b[2] == 1 && b[3] == 1 && b[4] == 1 && b[5] == 1,
          "index marker layout");
  }

  // ---- P3: the ESP's real decoder consumes a composed capture stream ----
  // Shape a one-revolution index-mode capture the way flux_pio.c emits it:
  // cue-edge marker, intervals (spanning all three encodings), a mid-track
  // index marker, more intervals, the stop-edge marker, terminator.
  {
    static const uint32_t iv[] = {100, 249,  250, 1524, 1525, 40000,
                                  7,   1200, 333, 50,   50,   50};
    enum { NIV = sizeof(iv) / sizeof(iv[0]), MID = 6 }; // marker before iv[6]
    uint8_t raw[256];
    uint32_t n = 0;
    n += gw_stream_enc_index(raw + n, 0); // cue edge
    uint32_t expect_idx1 = 0;
    for (uint32_t i = 0; i < NIV; i++) {
      if (i == MID)
        n += gw_stream_enc_index(raw + n, 0); // mid-capture edge
      n += gw_stream_enc_interval(raw + n, iv[i]);
      if (i < MID)
        expect_idx1 += iv[i];
    }
    uint32_t expect_idx2 = 0;
    for (uint32_t i = 0; i < NIV; i++)
      expect_idx2 += iv[i];
    n += gw_stream_enc_index(raw + n, 0); // stop edge
    raw[n++] = 0x00;                      // terminator

    uint32_t ticks[NIV + 4];
    uint32_t index[8];
    flux_stream_t fs = {.ticks = ticks,
                        .ticks_cap = NIV + 4,
                        .index = index,
                        .index_cap = 8};
    int r = flux_decode(raw, n, &fs);
    CHECK(r == 0, "flux_decode returned %d", r);
    CHECK(fs.ticks_len == NIV, "decoded %u intervals, want %u",
          (unsigned)fs.ticks_len, (unsigned)NIV);
    for (uint32_t i = 0; i < NIV && i < fs.ticks_len; i++)
      CHECK(ticks[i] == iv[i], "interval %u: %u != %u", i, ticks[i], iv[i]);
    CHECK(fs.index_len == 3, "decoded %u markers, want 3",
          (unsigned)fs.index_len);
    CHECK(index[0] == 0, "cue marker at %u, want 0", index[0]);
    CHECK(index[1] == expect_idx1, "mid marker at %u, want %u", index[1],
          expect_idx1);
    CHECK(index[2] == expect_idx2, "stop marker at %u, want %u", index[2],
          expect_idx2);
  }

  // ---- P4: write-decoder hardening ----
  {
    gw_stream_dec_t d;
    uint32_t got;

    // {250,1} decodes to 250 — the off-by-one class the vendored coder's
    // upstream shipped (patch P2 in the RP2040 tree); stated literally.
    uint8_t two[2] = {250, 1};
    gw_stream_dec_init(&d, two, 2);
    CHECK(gw_stream_dec_next(&d, &got) == 1 && got == 250, "{250,1} != 250");

    // SPACE folds into the NEXT interval.
    uint8_t sp[16];
    uint32_t sn = gw_stream_enc_interval(sp, 100000); // SPACE(99751)+249
    gw_stream_dec_init(&d, sp, sn);
    CHECK(gw_stream_dec_next(&d, &got) == 1 && got == 100000,
          "SPACE fold gave %u", got);

    // Trailing SPACE with no closing interval: clean end (keirf discard).
    gw_stream_dec_init(&d, sp, sn - 1); // drop the 249
    CHECK(gw_stream_dec_next(&d, &got) == 0, "trailing SPACE not discarded");

    // FLUXOP_INDEX is not writable.
    uint8_t ix[8];
    uint32_t in = gw_stream_enc_index(ix, 0);
    gw_stream_dec_init(&d, ix, in);
    CHECK(gw_stream_dec_next(&d, &got) == -1, "INDEX accepted in write path");

    // Truncated two-byte and truncated escape are errors, not garbage.
    uint8_t t1[1] = {252};
    gw_stream_dec_init(&d, t1, 1);
    CHECK(gw_stream_dec_next(&d, &got) == -1, "truncated 2-byte accepted");
    uint8_t t2[3] = {0xFF, 2, 1};
    gw_stream_dec_init(&d, t2, 3);
    CHECK(gw_stream_dec_next(&d, &got) == -1, "truncated escape accepted");

    // Embedded terminator inside the body is an error.
    uint8_t t3[2] = {0, 5};
    gw_stream_dec_init(&d, t3, 2);
    CHECK(gw_stream_dec_next(&d, &got) == -1, "embedded 0x00 accepted");
  }

  if (failures) {
    printf("test_gw_stream: %d FAILURES\n", failures);
    return 1;
  }
  printf("test_gw_stream: all checks pass (round trip 1..70000, ESP encoder "
         "cross-check 1..1524, flux_decode.c composed-stream check)\n");
  return 0;
}
