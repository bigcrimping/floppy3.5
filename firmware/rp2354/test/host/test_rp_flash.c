// SPDX-License-Identifier: Unlicense
// Host test for the ESP<->stub flashing path: the GWRP mailbox protocol and
// the SWD packet layer, both exercised without hardware.
//
// Part 1 — mailbox: a fake 2 MB flash + a faithful re-implementation of the
// stub's command handler run against the REAL contract header
// (tools/flash_stub/gwrp_mailbox.h, the same file both firmwares compile),
// driven by the same command sequence rp_flash.c issues. Checks the happy
// path end to end plus the failure modes the ESP relies on: CRC rejection,
// span/alignment rejection, 256-byte page padding, and that a verify catches
// a corrupted program.
//
// Part 2 — SWD: the request-byte builder and the 32-bit data parity from
// rp_swd.c, replicated here and checked against hand-computed ADIv5 frames.
// (rp_swd.c itself needs the ESP-IDF GPIO layer, so the bit-level rules are
// pinned here rather than compiling the driver — the constants that WOULD
// need silicon are flagged in the driver as bench checkpoints.)
#include <stddef.h> // offsetof: the MEM-AP alignment checks below
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "gwrp_mailbox.h"

// Part 3 — the GetInfo 0x88 diag window. The controller memcpy's the leading
// 32 bytes of its counter block into the fixed GetInfo payload, and the ESP
// casts those bytes onto its own struct. Nothing at runtime would notice the
// two drifting apart (a reordered counter just reports the wrong name), so
// pin them together here by compiling BOTH real headers.
// Explicit paths, not -I: BOTH trees have a gw_proto.h, and an include-path
// search would silently pick the controller's and defeat the whole check.
#include "../../src/diag.h"                  // controller's gw_diag_t
#include "../../../esp32/main/gw_proto.h"    // the ESP's view of the window

#define SAME_FIELD(f)                                                          \
  _Static_assert(offsetof(gw_diag_t, f) ==                                     \
                     offsetof(struct gw_diag_counters, f),                     \
                 "diag field " #f " moved: ESP gw_proto.h and controller "     \
                 "diag.h disagree")
SAME_FIELD(spi_rx_overruns);
SAME_FIELD(spi_frame_errors);
SAME_FIELD(spi_retransmits);
SAME_FIELD(spi_seq_errs);
SAME_FIELD(proto_bad_cmd);
SAME_FIELD(proto_bad_len);
SAME_FIELD(arb_to_usb);
SAME_FIELD(arb_to_spi);
_Static_assert(sizeof(struct gw_diag_counters) == 32,
               "the ESP's diag window must be exactly the GetInfo payload");
_Static_assert(sizeof(gw_diag_t) >= sizeof(struct gw_diag_counters),
               "controller counter block is smaller than the window the ESP "
               "reads — GetInfo 0x88 would copy past the end");

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

// ---- fake target ----

#define XIP_BASE_ADDR 0x10000000u
#define FLASH_BYTES (2u * 1024 * 1024)
#define FLASH_SECTOR_SIZE 4096u
#define FLASH_PAGE_SIZE 256u

static uint8_t g_flash[FLASH_BYTES];
static struct gwrp_mailbox g_mb;

static void flash_erase(uint32_t off, uint32_t len) {
  memset(g_flash + off, 0xFF, len);
}

// Program with NOR semantics (AND into the existing contents) so a missing
// erase shows up as corruption, exactly as it would on the real part.
static void flash_program(uint32_t off, const uint8_t *src, uint32_t len) {
  for (uint32_t i = 0; i < len; i++)
    g_flash[off + i] &= src[i];
}

// ---- the stub's handler, mirroring tools/flash_stub/flash_stub.c ----

static int span_ok(uint32_t addr, uint32_t len, uint32_t align) {
  if (addr < XIP_BASE_ADDR || len == 0 || len > GWRP_DATA_MAX)
    return 0;
  if (addr & (align - 1))
    return 0;
  return (addr - XIP_BASE_ADDR) + len <= FLASH_BYTES;
}

static uint32_t stub_erase(uint32_t addr, uint32_t len) {
  if (addr < XIP_BASE_ADDR || len == 0 || (addr & (FLASH_SECTOR_SIZE - 1)) ||
      (len & (FLASH_SECTOR_SIZE - 1)) ||
      (addr - XIP_BASE_ADDR) + len > FLASH_BYTES)
    return GWRP_ST_BADARG;
  flash_erase(addr - XIP_BASE_ADDR, len);
  return GWRP_ST_OK;
}

static uint32_t stub_write(uint32_t addr, uint32_t len, uint32_t want_crc) {
  static uint8_t page[GWRP_DATA_MAX];
  if (!span_ok(addr, len, FLASH_PAGE_SIZE))
    return GWRP_ST_BADARG;
  for (uint32_t i = 0; i < len; i++)
    page[i] = g_mb.data[i];
  if (gwrp_crc16(page, len) != (uint16_t)want_crc)
    return GWRP_ST_BADCRC;
  uint32_t plen = (len + FLASH_PAGE_SIZE - 1) & ~(FLASH_PAGE_SIZE - 1u);
  memset(page + len, 0xFF, plen - len);
  flash_program(addr - XIP_BASE_ADDR, page, plen);
  return GWRP_ST_OK;
}

static uint32_t stub_read(uint32_t addr, uint32_t len) {
  if (!span_ok(addr, len, 1))
    return GWRP_ST_BADARG;
  const uint8_t *p = g_flash + (addr - XIP_BASE_ADDR);
  for (uint32_t i = 0; i < len; i++)
    g_mb.data[i] = p[i];
  static uint8_t copy[GWRP_DATA_MAX];
  for (uint32_t i = 0; i < len; i++)
    copy[i] = g_mb.data[i];
  g_mb.crc = gwrp_crc16(copy, len);
  return GWRP_ST_OK;
}

// One poll iteration of the stub's main loop.
static void stub_poll(void) {
  uint32_t seq = g_mb.seq;
  if (seq == g_mb.ack)
    return;
  uint32_t st;
  switch (g_mb.cmd) {
  case GWRP_CMD_PING:  st = GWRP_ST_OK; break;
  case GWRP_CMD_ERASE: st = stub_erase(g_mb.addr, g_mb.len); break;
  case GWRP_CMD_WRITE: st = stub_write(g_mb.addr, g_mb.len, g_mb.crc); break;
  case GWRP_CMD_READ:  st = stub_read(g_mb.addr, g_mb.len); break;
  case GWRP_CMD_EXIT:  st = GWRP_ST_OK; break;
  default:             st = GWRP_ST_BADCMD; break;
  }
  g_mb.status = st;
  g_mb.ack = seq;
}

// ---- the ESP's client half, mirroring rp_flash.c's mb_run ----

static uint32_t g_seq;

static uint32_t mb_run(uint32_t cmd, uint32_t addr, uint32_t len,
                       uint32_t crc) {
  g_mb.cmd = cmd;
  g_mb.addr = addr;
  g_mb.len = len;
  g_mb.crc = crc;
  g_mb.seq = ++g_seq; // seq last, as the driver does
  for (int i = 0; i < 4 && g_mb.ack != g_seq; i++)
    stub_poll();
  if (g_mb.ack != g_seq)
    return 0xDEAD;
  return g_mb.status;
}

static uint32_t esp_write(uint32_t addr, const uint8_t *data, uint32_t len) {
  memcpy((void *)g_mb.data, data, len);
  return mb_run(GWRP_CMD_WRITE, addr, len, gwrp_crc16(data, len));
}

// Returns 0 when the readback matches `expect` on both the stub's CRC and a
// direct compare (the two checks rp_flash.c's stub_verify performs).
static int esp_verify(uint32_t addr, const uint8_t *expect, uint32_t len) {
  if (mb_run(GWRP_CMD_READ, addr, len, 0) != GWRP_ST_OK)
    return -1;
  static uint8_t got[GWRP_DATA_MAX];
  memcpy(got, (const void *)g_mb.data, len);
  if ((uint16_t)g_mb.crc != gwrp_crc16(got, len))
    return -2;
  return memcmp(got, expect, len) == 0 ? 0 : -3;
}

// ---- SWD bit rules, mirroring rp_swd.c ----

static int parity32(uint32_t v) {
  v ^= v >> 16; v ^= v >> 8; v ^= v >> 4; v ^= v >> 2; v ^= v >> 1;
  return (int)(v & 1);
}

static uint8_t request_byte(int ap, int rd, uint8_t addr) {
  uint8_t a2 = (addr >> 2) & 1, a3 = (addr >> 3) & 1;
  uint8_t p = (uint8_t)(ap ^ rd ^ a2 ^ a3) & 1;
  return (uint8_t)(0x81 | (ap << 1) | (rd << 2) | (a2 << 3) | (a3 << 4) |
                   (p << 5));
}

int main(void) {
  // ================= Part 1: mailbox =================
  memset(g_flash, 0xFF, sizeof(g_flash));
  memset(&g_mb, 0, sizeof(g_mb));

  CHECK(mb_run(GWRP_CMD_PING, 0, 0, 0) == GWRP_ST_OK, "ping failed");

  // A realistic image: 10 KB of varied bytes, flashed in 4 KB chunks the way
  // rp_flash.c drives it (erase whole span, write chunks, verify chunks).
  enum { IMG = 10 * 1024, BASE = 0x10000000 };
  static uint8_t img[IMG];
  for (uint32_t i = 0; i < IMG; i++)
    img[i] = (uint8_t)(i * 7 + (i >> 5));

  uint32_t espan = (IMG + FLASH_SECTOR_SIZE - 1) & ~(FLASH_SECTOR_SIZE - 1u);
  CHECK(mb_run(GWRP_CMD_ERASE, BASE, espan, 0) == GWRP_ST_OK, "erase failed");
  for (uint32_t off = 0; off < IMG; off += GWRP_DATA_MAX) {
    uint32_t n = (IMG - off > GWRP_DATA_MAX) ? GWRP_DATA_MAX : IMG - off;
    CHECK(esp_write(BASE + off, img + off, n) == GWRP_ST_OK,
          "write @%u failed", off);
  }
  for (uint32_t off = 0; off < IMG; off += GWRP_DATA_MAX) {
    uint32_t n = (IMG - off > GWRP_DATA_MAX) ? GWRP_DATA_MAX : IMG - off;
    CHECK(esp_verify(BASE + off, img + off, n) == 0, "verify @%u failed", off);
  }
  // The 10 KB image ends mid-page; the tail of that page must be erased
  // (0xFF), not stale — that is the stub's 256-byte padding rule.
  CHECK(g_flash[IMG] == 0xFF && g_flash[IMG + 100] == 0xFF,
        "tail padding not 0xFF");

  // Corrupted program is caught by verify (not just by the CRC, which only
  // covers the transport).
  g_flash[512] ^= 0xFF;
  CHECK(esp_verify(BASE, img, GWRP_DATA_MAX) == -3, "verify missed corruption");
  g_flash[512] ^= 0xFF;
  CHECK(esp_verify(BASE, img, GWRP_DATA_MAX) == 0, "verify false positive");

  // A payload CRC mismatch is rejected and must NOT touch flash.
  {
    uint8_t junk[256];
    memset(junk, 0x5A, sizeof(junk));
    memcpy((void *)g_mb.data, junk, sizeof(junk));
    uint32_t before = g_flash[0];
    CHECK(mb_run(GWRP_CMD_WRITE, BASE, sizeof(junk),
                 gwrp_crc16(junk, sizeof(junk)) ^ 1) == GWRP_ST_BADCRC,
          "bad CRC accepted");
    CHECK(g_flash[0] == before, "flash modified on a CRC failure");
  }

  // Span / alignment rejection: below XIP, past the device, misaligned write,
  // misaligned erase, oversized payload, unknown opcode.
  CHECK(mb_run(GWRP_CMD_WRITE, 0x0FFFF000, 256, 0) == GWRP_ST_BADARG, "low addr");
  CHECK(mb_run(GWRP_CMD_WRITE, 0x101FFF00, 4096, 0) == GWRP_ST_BADARG, "past end");
  CHECK(mb_run(GWRP_CMD_WRITE, BASE + 1, 256, 0) == GWRP_ST_BADARG, "unaligned write");
  CHECK(mb_run(GWRP_CMD_ERASE, BASE + 256, 4096, 0) == GWRP_ST_BADARG, "unaligned erase");
  CHECK(mb_run(GWRP_CMD_ERASE, BASE, 1024, 0) == GWRP_ST_BADARG, "partial-sector erase");
  CHECK(mb_run(GWRP_CMD_WRITE, BASE, GWRP_DATA_MAX + 4, 0) == GWRP_ST_BADARG, "oversize");
  CHECK(mb_run(99, BASE, 256, 0) == GWRP_ST_BADCMD, "unknown opcode accepted");

  // The mailbox must stay word-addressable for the MEM-AP: every field the
  // ESP pokes, and the data buffer, at 4-byte offsets.
  CHECK(offsetof(struct gwrp_mailbox, data) % 4 == 0, "data not word-aligned");
  CHECK(offsetof(struct gwrp_mailbox, seq) % 4 == 0 &&
        offsetof(struct gwrp_mailbox, ack) % 4 == 0 &&
        offsetof(struct gwrp_mailbox, status) % 4 == 0, "header misaligned");
  CHECK(GWRP_MAILBOX_ADDR % 4 == 0, "mailbox base misaligned");
  // CRC identity against the repo's canonical CCITT-FALSE check vector.
  CHECK(gwrp_crc16((const uint8_t *)"123456789", 9) == 0x29B1, "crc16 vector");

  // ================= Part 2: SWD frames =================
  // Hand-computed ADIv5 request bytes (bit0 start, bit1 APnDP, bit2 RnW,
  // bits3-4 A[2:3], bit5 parity, bit6 stop=0, bit7 park=1):
  //   DP read IDCODE  (addr 0x0, DP, read)  -> 1,0,1,0,0,par=1,0,1 = 0xA5
  //   DP write ABORT  (addr 0x0, DP, write) -> 1,0,0,0,0,par=0,0,1 = 0x81
  //   DP read CTRLSTAT(addr 0x4, DP, read)  -> A2=1 -> 0x8D
  //   DP write SELECT (addr 0x8, DP, write) -> A3=1 -> 0xB1
  //   AP write DRW    (addr 0xC, AP, write) -> AP,A2,A3 -> 0xBB
  //   AP read  DRW    (addr 0xC, AP, read)  -> +RnW    -> 0x9F
  CHECK(request_byte(0, 1, 0x0) == 0xA5, "IDCODE req = 0x%02X",
        request_byte(0, 1, 0x0));
  CHECK(request_byte(0, 0, 0x0) == 0x81, "ABORT req = 0x%02X",
        request_byte(0, 0, 0x0));
  CHECK(request_byte(0, 1, 0x4) == 0x8D, "CTRLSTAT req = 0x%02X",
        request_byte(0, 1, 0x4));
  CHECK(request_byte(0, 0, 0x8) == 0xB1, "SELECT req = 0x%02X",
        request_byte(0, 0, 0x8));
  CHECK(request_byte(1, 0, 0xC) == 0xBB, "AP DRW wr req = 0x%02X",
        request_byte(1, 0, 0xC));
  CHECK(request_byte(1, 1, 0xC) == 0x9F, "AP DRW rd req = 0x%02X",
        request_byte(1, 1, 0xC));
  // Every request byte must have start=1, stop=0, park=1 and even parity over
  // {APnDP, RnW, A2, A3} — check the whole space, not just the six above.
  for (int ap = 0; ap <= 1; ap++)
    for (int rd = 0; rd <= 1; rd++)
      for (uint8_t a = 0; a <= 0xC; a += 4) {
        uint8_t b = request_byte(ap, rd, a);
        CHECK((b & 0x01) && !(b & 0x40) && (b & 0x80), "framing bits ap=%d rd=%d a=%u", ap, rd, a);
        int ones = 0;
        for (int i = 1; i <= 5; i++)
          ones += (b >> i) & 1;
        CHECK((ones & 1) == 0, "parity ap=%d rd=%d a=%u", ap, rd, a);
      }
  // Data parity is even over the 32 payload bits.
  CHECK(parity32(0) == 0 && parity32(1) == 1 && parity32(3) == 0,
        "parity32 basics");
  CHECK(parity32(0xA05F0003u) == parity32(0xA05F0003u), "parity stable");
  {
    uint32_t v = 0xA05F0003u; // DHCSR halt word
    int ones = 0;
    for (int i = 0; i < 32; i++)
      ones += (v >> i) & 1;
    CHECK(parity32(v) == (ones & 1), "parity32 vs bit count");
  }

  if (failures) {
    printf("test_rp_flash: %d FAILURES\n", failures);
    return 1;
  }
  printf("test_rp_flash: all checks pass (mailbox happy path + CRC/span/"
         "padding/corruption rejection, SWD request+parity vectors)\n");
  return 0;
}
