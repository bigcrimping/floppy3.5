// SPDX-License-Identifier: Unlicense
// GWSPI v1 slave (docs/spi-link-protocol.md). The PL022 runs as a mode-1
// hardware slave, two DMA channels move full-size frame buffers each way, and
// link state changes only in the CS rising-edge ISR. A torn transaction
// ingests zero payload bytes and is repaired by seq/ack (window 2,
// go-back-N), so correctness never depends on timing; timing only buys
// throughput.
//
// Ownership (all on core 0):
//   ISR writes:    rx_prod, rx_last, tx_sent, tx_acked, win[], seq state,
//                  the armed frame buffers.
//   thread writes: rx_cons, tx_prod.
// Cursors are free-running 32-bit words (atomic on the M33), so the rings need
// no locks. The busy swaps touch ISR state from the thread only with IRQs off
// and CS high.
#include "transport_spi.h"

#include <string.h>

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/structs/resets.h"
#include "hardware/structs/spi.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "pico/platform.h"

#include "board_rp2354.h"
#include "crc16.h"
#include "diag.h"

// Acks lag one transaction (§2.3): the first missing ack is normal, the
// second means loss.
#define ACK_MISS_LIMIT 2

// Free-running cursors wrap by masking, so ring sizes must be powers of two.
_Static_assert((SPI_RX_RING_SZ & (SPI_RX_RING_SZ - 1)) == 0, "pow2");
_Static_assert((SPI_TX_RING_SZ & (SPI_TX_RING_SZ - 1)) == 0, "pow2");

static uint8_t rx_ring[SPI_RX_RING_SZ];
static volatile uint32_t rx_prod;
static volatile uint32_t rx_cons;

static uint8_t tx_ring[SPI_TX_RING_SZ];
static volatile uint32_t tx_prod;
static volatile uint32_t tx_sent;  // sliced into frames, maybe unacked
static volatile uint32_t tx_acked; // [tx_acked, tx_sent) is the retransmit
                                   // store; no separate copies are kept

// SYNC must flush the rx ring, but rx_cons belongs to the thread, so the ISR
// posts a cut point (rx_prod at SYNC time) that the thread applies before its
// next ring access.
static volatile uint32_t rx_flush_req, rx_flush_done;
static volatile uint32_t rx_flush_upto;

static uint8_t rx_frame[GWSPI_FRAME_MAX] __attribute__((aligned(4)));
static uint8_t tx_frame[GWSPI_FRAME_MAX] __attribute__((aligned(4)));

// ---- Reliability state (ISR-owned, §2.3) ----
typedef struct {
  uint8_t seq;
  uint8_t acks_missed; // valid master frames since last sent, without its ack
  uint16_t len;
} txwin_t;
static txwin_t win[2];     // unacked payload frames, win[0] oldest
static uint8_t win_n;
static uint8_t resend_idx; // [0,resend_idx) sent since the last rewind;
                           // [resend_idx,win_n) await retransmission
static uint8_t tx_next_seq;
static uint8_t rx_last;        // last in-order seq accepted (0xFF = none)
static uint16_t grant_self;    // avail in our last frame; caps our next len
static uint16_t credit_master; // master's rx-ring free space

// What the armed (not yet clocked) frame consumed, so a busy swap can undo it
// without losing bytes or burning a seq. Cleared at ISR entry.
enum { PB_NONE, PB_NEW, PB_RESEND };
static uint8_t pb_kind, pb_idx;
static bool pb_sync;

static volatile bool busy_mode;
static bool sync_reply_pending;
static bool inited;
static bool sniff_ok; // DMA sniffer CRC matches crc16_ccitt_tab
// Master's 5 V drive rail from GWSPI_FLAG_RAIL. -1 = no master frame yet.
static volatile int8_t rail_state = -1;
static volatile bool rail_dropped; // rail seen off since the last take
static int ch_rx = -1, ch_tx = -1, ch_m2m = -1;
static spi_link_stats_t st;

static inline uint16_t get16(const uint8_t *p) {
  return (uint16_t)(p[0] | (p[1] << 8));
}
static inline void put16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

// ---- Copy + CRC ------------------------------------------------------------
// The ISR must validate, rebuild and re-arm inside the master's
// inter-transaction gap (GW_SPI_GAP_US, 400 us). Two table CRCs of 4 KB plus
// two memcpys don't fit, so copy and CRC are fused in one pass through the DMA
// sniffer (~65 us per 4 KB). If the sniffer fails its init self-test we use
// the CPU path, which may miss the gap on full frames: that costs a
// retransmit, not correctness.

static uint16_t dma_copy_crc_raw(uint8_t *dst, const uint8_t *src, uint32_t n,
                                 uint16_t crc) {
  // Writing SNIFF_DATA seeds the CRC (chains across ring-wrap segments).
  dma_hw->sniff_data = crc;
  dma_channel_set_read_addr((uint)ch_m2m, src, false);
  dma_channel_set_write_addr((uint)ch_m2m, dst, false);
  dma_channel_set_trans_count((uint)ch_m2m, n, true); // trigger
  dma_channel_wait_for_finish_blocking((uint)ch_m2m);
  return (uint16_t)dma_hw->sniff_data;
}

static uint16_t __not_in_flash_func(copy_crc)(uint8_t *dst, const uint8_t *src,
                                              uint32_t n, uint16_t crc) {
  if (n == 0)
    return crc;
  if (sniff_ok)
    return dma_copy_crc_raw(dst, src, n, crc);
  memcpy(dst, src, n);
  return crc16_ccitt_tab(crc, src, n);
}

// Ring<->linear copies, CRC chained across the wrap seam (<=2 segments).
static uint16_t __not_in_flash_func(ring_to_buf_crc)(uint8_t *dst,
                                                     const uint8_t *ring,
                                                     uint32_t ring_sz,
                                                     uint32_t pos, uint32_t n) {
  uint32_t o = pos & (ring_sz - 1);
  uint32_t first = ring_sz - o;
  if (first > n)
    first = n;
  uint16_t crc = copy_crc(dst, ring + o, first, 0xFFFF);
  if (n > first)
    crc = copy_crc(dst + first, ring, n - first, crc);
  return crc;
}

static uint16_t __not_in_flash_func(buf_to_ring_crc)(uint8_t *ring,
                                                     uint32_t ring_sz,
                                                     uint32_t pos,
                                                     const uint8_t *src,
                                                     uint32_t n) {
  uint32_t o = pos & (ring_sz - 1);
  uint32_t first = ring_sz - o;
  if (first > n)
    first = n;
  uint16_t crc = copy_crc(ring + o, src, first, 0xFFFF);
  if (n > first)
    crc = copy_crc(ring, src + first, n - first, crc);
  return crc;
}

static void sniff_selftest(void) {
  // Two chained segments prove both the polynomial and the seed/readback
  // format match the software CRC.
  static const uint8_t pat[13] = {0x5A, 0x01, 0x00, 0xFF, 0x10, 0x00, 0x34,
                                  0x12, 0xEE, 0x01, 0x80, 0x7F, 0xA5};
  uint8_t out[13];
  uint16_t hw = dma_copy_crc_raw(out, pat, 7, 0xFFFF);
  hw = dma_copy_crc_raw(out + 7, pat + 7, sizeof(pat) - 7, hw);
  sniff_ok = (hw == crc16_ccitt_tab(0xFFFF, pat, sizeof(pat))) &&
             (memcmp(out, pat, sizeof(pat)) == 0);
}

// ---- DMA arm/quiesce -------------------------------------------------------
static void __not_in_flash_func(dma_quiesce)(uint ch) {
  // Erratum RP2040-E13: aborting a channel while its DREQ is active can leave
  // a stale transfer credit, and the SSP TX DREQ is almost always active.
  // Clear EN first (al1_ctrl doesn't trigger), then abort.
  hw_clear_bits(&dma_hw->ch[ch].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
  dma_channel_abort(ch);
}

static void __not_in_flash_func(dma_rearm)(uint ch) {
  // Always a full frame: the master picks the length, and the leftover count
  // at CS rise tells us what it was.
  hw_set_bits(&dma_hw->ch[ch].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
  if ((int)ch == ch_rx)
    dma_channel_set_write_addr(ch, rx_frame, false);
  else
    dma_channel_set_read_addr(ch, tx_frame, false);
  dma_channel_set_trans_count(ch, GWSPI_FRAME_MAX, true); // trigger
}

// ---- PL022 block reset + reprogram ------------------------------------
// Resets SPI0 and programs it as a slave, leaving SSE=0. The block reset is
// the only guaranteed TX-FIFO flush: the TRM documents none, and whether
// SSE=0 clears the FIFOs is disputed (pico-sdk #713/#1757). Cheap enough for
// the ISR. Raw registers: the SDK reset helpers were renamed between 1.5 and
// 2.x, and spi_init()/spi_set_baudrate() are master-only and would clobber
// CR0/CPSR.
static void __not_in_flash_func(pl022_reset_reprogram)(void) {
  hw_set_bits(&resets_hw->reset, RESETS_RESET_SPI0_BITS);
  hw_clear_bits(&resets_hw->reset, RESETS_RESET_SPI0_BITS);
  while (!(resets_hw->reset_done & RESETS_RESET_SPI0_BITS))
    tight_loop_contents();
  spi0_hw->cr1 = 0; // SSE=0 while configuring
  spi0_hw->cr0 = (7u << SPI_SSPCR0_DSS_LSB)   // 8-bit frames
                 | (0u << SPI_SSPCR0_FRF_LSB) // Motorola SPI
                 // Mode 1 (SPH=1): the only phase in which a PL022 slave
                 // accepts back-to-back bytes with CS held low.
                 | SPI_SSPCR0_SPH_BITS | (0u << SPI_SSPCR0_SCR_LSB);
  spi0_hw->cpsr = 12; // must be programmed even in slave mode
  spi0_hw->imsc = 0;  // DMA + the CS-edge IRQ do the work
  spi0_hw->dmacr = SPI_SSPDMACR_TXDMAE_BITS | SPI_SSPDMACR_RXDMAE_BITS;
  hw_set_bits(&spi0_hw->cr1, SPI_SSPCR1_MS_BITS); // slave; needs SSE=0
}

// ---- TX-FIFO residue guard ---------------------------------------------
// Re-arming assumes SSE=0 flushed the TX FIFO, which is unverified on this
// silicon and the TX FIFO can't be read back. So check by effect: after
// arming, the DMA should push 8 new bytes to fill the empty FIFO. If fewer
// left the channel, stale bytes are queued ahead of the new frame and would
// shift every later frame into an hcrc failure, SYNC replies included. Recover
// with a block reset. Call with IRQs off, right after dma_rearm(ch_tx).
// TODO(bench): measure whether SSE=0 flushes TX. If it never does, do the
// block reset unconditionally in the ISR and drop this double re-arm.
#define SSP_TX_FIFO_DEPTH 8u
static void __not_in_flash_func(tx_arm_verify)(void) {
  // A timeout errs toward recovery, which a stalled DMA needs anyway.
  for (uint32_t spin = 0; (spi0_hw->sr & SPI_SSPSR_TNF_BITS) && spin < 1024;
       spin++)
    tight_loop_contents();
  if (dma_hw->ch[ch_tx].transfer_count <=
      (uint32_t)(GWSPI_FRAME_MAX - SSP_TX_FIFO_DEPTH))
    return;
  st.tx_residue_resets++;
  dma_quiesce((uint)ch_rx); // a DREQ glitch during reset mustn't move RX
  dma_quiesce((uint)ch_tx);
  pl022_reset_reprogram();
  dma_rearm((uint)ch_rx);
  hw_set_bits(&spi0_hw->cr1, SPI_SSPCR1_SSE_BITS);
  dma_rearm((uint)ch_tx);
}

// ---- Link state reset (SYNC, §2.4; also bring-up) ----
static void __not_in_flash_func(link_reset)(void) {
  win_n = 0;
  resend_idx = 0;
  pb_kind = PB_NONE;
  pb_sync = false;
  tx_next_seq = 0;
  rx_last = 0xFF;    // next accepted seq is 0
  grant_self = 0;    // first frame after SYNC is header-only
  credit_master = 0;
  // Drop stale replies. A racing write() may leave bytes queued; the master
  // flushes its RX after SYNC, so they can't leak into the new session.
  uint32_t p = tx_prod;
  tx_sent = p;
  tx_acked = p;
  rx_flush_upto = rx_prod;
  __compiler_memory_barrier(); // upto must be visible before the request
  rx_flush_req++;
}

// Read cursor with any pending flush applied, so the SYNC reply already
// advertises an empty ring.
static inline uint32_t rx_cons_eff(void) {
  uint32_t c = rx_cons;
  if (rx_flush_req != rx_flush_done) {
    uint32_t u = rx_flush_upto;
    if ((int32_t)(u - c) > 0)
      c = u;
  }
  return c;
}

// ---- Ack processing (sender side of §2.3) ----
static void __not_in_flash_func(process_ack)(uint8_t a) {
  if (win_n == 0)
    return;
  uint8_t k = 0; // entries released by this ack
  if (win_n == 2 && a == win[1].seq)
    k = 2;
  else if (a == win[0].seq)
    k = 1;
  else if (a != (uint8_t)(win[0].seq - 1u))
    g_diag.spi_seq_errs++; // not no-progress, w0 or w1: release nothing
  for (uint8_t i = 0; i < k; i++)
    tx_acked += win[i].len;
  if (k) {
    win[0] = win[1]; // when k==2 this copy is unused
    win_n = (uint8_t)(win_n - k);
    resend_idx = (resend_idx > k) ? (uint8_t)(resend_idx - k) : 0;
  }
  // Only entries sent since the last rewind can miss acks.
  bool rewind = false;
  for (uint8_t i = 0; i < resend_idx; i++)
    if (++win[i].acks_missed >= ACK_MISS_LIMIT)
      rewind = true;
  if (rewind)
    resend_idx = 0; // go-back-N from tx_acked
}

// ---- Inbound frame validation (§2.2/§2.3) ----
static void __not_in_flash_func(process_rx_frame)(uint32_t got) {
  st.exchanges++;
  const uint8_t *f = rx_frame;
  if (got < GWSPI_HDR_LEN) {
    st.short_frames++; // aborted transaction or our missed re-arm
    goto hard_invalid;
  }
  if (f[GWSPI_OFF_MAGIC] != GWSPI_MAGIC_M2S)
    goto hard_invalid; // includes 0x00/0xFF floods
  if ((f[GWSPI_OFF_FLAGS] & 0x0Fu) != GWSPI_VERSION)
    goto hard_invalid;
  if (get16(f + GWSPI_OFF_HCRC) != crc16_ccitt_tab(0xFFFF, f, GWSPI_OFF_HCRC))
    goto hard_invalid;
  uint16_t len = get16(f + GWSPI_OFF_LEN);
  if (len > GWSPI_PAYLOAD_MAX)
    goto hard_invalid;

  // Before the SYNC branch: SYNC frames at bring-up carry the rail state too.
  {
    bool on = (f[GWSPI_OFF_FLAGS] & GWSPI_FLAG_RAIL) != 0;
    if (!on && rail_state != 0)
      rail_dropped = true;
    rail_state = on ? 1 : 0;
  }
  // The header is trusted from here even if the payload turns out short or
  // corrupt; taking ack/credit from it stops a master with a stale grant
  // from livelocking on 12-byte polls.
  if (f[GWSPI_OFF_FLAGS] & GWSPI_FLAG_SYNC) {
    // seq/ack belong to the dead session; only credit carries over.
    link_reset();
    sync_reply_pending = true;
    credit_master = get16(f + GWSPI_OFF_CREDIT);
    st.sync_resets++;
    return;
  }
  credit_master = get16(f + GWSPI_OFF_CREDIT);
  // The master's avail field is unused: only the master sizes transactions.
  process_ack(f[GWSPI_OFF_ACK]);
  if (len == 0)
    return; // header-only poll
  if (got < (uint32_t)GWSPI_HDR_LEN + len + 2u) {
    // Payload not fully clocked; the unadvanced ack makes the master resend.
    st.short_frames++;
    g_diag.spi_frame_errors++;
    return;
  }
  uint8_t seq = f[GWSPI_OFF_SEQ];
  if (seq == (uint8_t)(rx_last + 1u)) {
    // All or nothing: the master can't repair a half-taken frame.
    uint32_t space = SPI_RX_RING_SZ - (rx_prod - rx_cons_eff());
    if (space < len) {
      g_diag.spi_rx_overruns++; // stays 0 while both sides honour credit
      return;
    }
    // Bytes land past rx_prod and only become visible if the pcrc matches.
    uint16_t pcrc =
        buf_to_ring_crc(rx_ring, SPI_RX_RING_SZ, rx_prod, f + GWSPI_HDR_LEN, len);
    if (pcrc != get16(f + GWSPI_HDR_LEN + len)) {
      g_diag.spi_frame_errors++;
      return;
    }
    __compiler_memory_barrier(); // payload bytes land before occupancy grows
    rx_prod += len;
    rx_last = seq;
  } else if (seq == rx_last || seq == (uint8_t)(rx_last - 1u)) {
    st.dups++;
  } else {
    g_diag.spi_seq_errs++; // out of window: treat as duplicate
  }
  return;

hard_invalid:
  g_diag.spi_frame_errors++;
  // We can't tell whether our in-flight payloads arrived: resend them. The
  // master drops duplicates by seq.
  if (win_n)
    resend_idx = 0;
}

// ---- Outbound frame builder ------------------------------------------------
// Builds the next frame into tx_frame with the TX DMA quiesced (CS ISR, init,
// or a busy swap with IRQs off).
static void __not_in_flash_func(build_tx_frame)(void) {
  uint8_t *f = tx_frame;
  pb_kind = PB_NONE;
  pb_sync = false;

  uint8_t flags = GWSPI_VERSION;
  if (g_usb_host_active)
    flags |= GWSPI_FLAG_USB;
  if (sync_reply_pending) {
    flags |= GWSPI_FLAG_SYNC;
    sync_reply_pending = false; // the master re-SYNCs if this reply is lost
    pb_sync = true;
  }

  uint16_t len = 0;
  uint8_t seq = (uint8_t)(tx_next_seq - 1u); // ignored on len==0 frames
  uint32_t src = 0;
  if (!busy_mode) {
    if (resend_idx < win_n) {
      // Resend the exact original slice. A longer one would put new bytes in
      // a frame the master may drop as a duplicate.
      uint8_t i = resend_idx;
      src = tx_acked;
      for (uint8_t j = 0; j < i; j++)
        src += win[j].len;
      len = win[i].len;
      seq = win[i].seq;
      win[i].acks_missed = 0;
      pb_kind = PB_RESEND;
      pb_idx = i;
      resend_idx = (uint8_t)(i + 1);
      g_diag.spi_retransmits++;
    } else if (win_n < 2) {
      // New payload, capped by what's queued, the frame size, our previous
      // avail (so the master clocked a long enough transaction, §2.1) and the
      // master's credit. Retransmits skip the last two: they were checked when
      // first sliced.
      uint32_t cap = tx_prod - tx_sent;
      if (cap > GWSPI_PAYLOAD_MAX)
        cap = GWSPI_PAYLOAD_MAX;
      if (cap > grant_self)
        cap = grant_self;
      // credit_master was computed before the master took our unacked
      // slices, so subtract them or we'd double-book its ring.
      uint32_t inflight = 0;
      for (uint8_t j = 0; j < win_n; j++)
        inflight += win[j].len;
      uint32_t eff_credit =
          (credit_master > inflight) ? credit_master - inflight : 0;
      if (cap > eff_credit)
        cap = eff_credit;
      if (cap) {
        len = (uint16_t)cap;
        seq = tx_next_seq;
        src = tx_sent;
        win[win_n].seq = seq;
        win[win_n].len = len;
        win[win_n].acks_missed = 0;
        win_n++;
        resend_idx = win_n;
        tx_sent += len;
        tx_next_seq = (uint8_t)(tx_next_seq + 1u);
        pb_kind = PB_NEW;
      }
    }
    // else: window full, header-only frame
  }

  // avail counts every undelivered byte, including unacked slices: any of
  // them may be retransmitted, and the master must clock long enough to carry
  // it. Counting only unsent bytes would livelock after a loss (avail=0,
  // 12-byte polls, every resend truncated). Over-advertising is harmless.
  // Busy forces 0 so the master stops granting, and our first frame after
  // busy is header-only.
  uint32_t avail32 = busy_mode ? 0 : (tx_prod - tx_acked);
  uint16_t avail = (avail32 > 0xFFFFu) ? 0xFFFF : (uint16_t)avail32;
  uint32_t space = SPI_RX_RING_SZ - (rx_prod - rx_cons_eff());
  uint16_t credit = (space > 0xFFFFu) ? 0xFFFF : (uint16_t)space;

  f[GWSPI_OFF_MAGIC] = GWSPI_MAGIC_S2M;
  f[GWSPI_OFF_FLAGS] = flags;
  f[GWSPI_OFF_SEQ] = seq;
  f[GWSPI_OFF_ACK] = rx_last;
  put16(f + GWSPI_OFF_LEN, len);
  put16(f + GWSPI_OFF_AVAIL, avail);
  put16(f + GWSPI_OFF_CREDIT, credit);
  put16(f + GWSPI_OFF_HCRC, crc16_ccitt_tab(0xFFFF, f, GWSPI_OFF_HCRC));
  if (len) {
    uint16_t pcrc =
        ring_to_buf_crc(f + GWSPI_HDR_LEN, tx_ring, SPI_TX_RING_SZ, src, len);
    put16(f + GWSPI_HDR_LEN + len, pcrc);
  }
  grant_self = avail;
}

// Undo an armed frame that was never clocked (busy swaps only). A PB_NEW entry
// is always win[win_n-1]; its bytes and seq are handed back.
static void unbuild_prebuilt(void) {
  if (pb_kind == PB_NEW) {
    win_n--;
    tx_sent -= win[win_n].len;
    tx_next_seq = win[win_n].seq;
    resend_idx = win_n;
  } else if (pb_kind == PB_RESEND) {
    resend_idx = pb_idx;
  }
  if (pb_sync)
    sync_reply_pending = true;
  pb_kind = PB_NONE;
  pb_sync = false;
}

// ---- CS rising edge = end of transaction (§2.4) ----
static void __not_in_flash_func(spi_cs_irq)(void) {
  // IO_IRQ_BANK0 raw handlers are shared: filter for our event.
  uint32_t ev = gpio_get_irq_event_mask(PIN_SPI_CS);
  if (!(ev & GPIO_IRQ_EDGE_RISE))
    return;
  gpio_acknowledge_irq(PIN_SPI_CS, GPIO_IRQ_EDGE_RISE);

  // The armed frame went out (fully or not); seq/ack recovers it from here.
  pb_kind = PB_NONE;
  pb_sync = false;

  // 1. Let the RX DMA drain the SSP FIFO tail (bounded, in case the channel
  // already used its full count).
  for (uint32_t spin = 0; (spi0_hw->sr & SPI_SSPSR_RNE_BITS) && spin < 256;
       spin++)
    tight_loop_contents();

  // 2. Freeze both channels and see how much the master clocked.
  dma_quiesce((uint)ch_rx);
  dma_quiesce((uint)ch_tx);
  uint32_t got = GWSPI_FRAME_MAX - dma_hw->ch[ch_rx].transfer_count;

  // 3. SSE off to flush the FIFOs, so a torn transaction can't leak into the
  // next one.
  hw_clear_bits(&spi0_hw->cr1, SPI_SSPCR1_SSE_BITS);
  while (spi0_hw->sr & SPI_SSPSR_RNE_BITS)
    (void)spi0_hw->dr;

  // 4. Validate the received frame, then build our next one.
  process_rx_frame(got);
  build_tx_frame();

  // 5. Re-arm RX first so no byte is missed, then enable, then TX (which
  // preloads the FIFO before the next CS fall).
  dma_rearm((uint)ch_rx);
  hw_set_bits(&spi0_hw->cr1, SPI_SSPCR1_SSE_BITS);
  dma_rearm((uint)ch_tx);
  tx_arm_verify();

  // 6. DRDY last; it must not change while CS is low. Unacked bytes count.
  gpio_put(PIN_DRDY, !busy_mode && (tx_prod != tx_acked));
}

// ---- Bring-up --------------------------------------------------------------
void spi_link_init(void) {
  if (inited)
    return;
  inited = true;

  gpio_init(PIN_DRDY);
  gpio_set_dir(PIN_DRDY, GPIO_OUT);
  gpio_put(PIN_DRDY, 0);

  // Pulls make a reset or hi-Z master look like an idle bus. GPIO edge IRQs
  // still fire on a pin muxed to SPI, which is how hardware CS and the CS
  // interrupt coexist.
  gpio_set_function(PIN_SPI_SCK, GPIO_FUNC_SPI);
  gpio_pull_down(PIN_SPI_SCK);
  gpio_set_function(PIN_SPI_RX, GPIO_FUNC_SPI);
  gpio_pull_down(PIN_SPI_RX);
  gpio_set_function(PIN_SPI_TX, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SPI_CS, GPIO_FUNC_SPI);
  gpio_pull_up(PIN_SPI_CS);

  pl022_reset_reprogram();

  // One DMA channel per direction plus a mem-to-mem channel for the fused
  // copy+CRC. Only ch_m2m uses the sniffer.
  ch_rx = dma_claim_unused_channel(true);
  ch_tx = dma_claim_unused_channel(true);
  ch_m2m = dma_claim_unused_channel(true);

  dma_channel_config c = dma_channel_get_default_config((uint)ch_rx);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);
  channel_config_set_dreq(&c, DREQ_SPI0_RX);
  dma_channel_configure((uint)ch_rx, &c, rx_frame, &spi0_hw->dr,
                        GWSPI_FRAME_MAX, false);

  c = dma_channel_get_default_config((uint)ch_tx);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  channel_config_set_read_increment(&c, true);
  channel_config_set_write_increment(&c, false);
  channel_config_set_dreq(&c, DREQ_SPI0_TX);
  dma_channel_configure((uint)ch_tx, &c, &spi0_hw->dr, tx_frame,
                        GWSPI_FRAME_MAX, false);

  c = dma_channel_get_default_config((uint)ch_m2m);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  channel_config_set_read_increment(&c, true);
  channel_config_set_write_increment(&c, true);
  channel_config_set_sniff_enable(&c, true);
  dma_channel_set_config((uint)ch_m2m, &c, false);

  // CALC=0x2 is CRC-16-CCITT (raw value: the SDK enum name changed between
  // releases).
  dma_hw->sniff_ctrl = ((uint32_t)ch_m2m << DMA_SNIFF_CTRL_DMACH_LSB) |
                       (0x2u << DMA_SNIFF_CTRL_CALC_LSB) |
                       DMA_SNIFF_CTRL_EN_BITS;
  sniff_selftest();

  link_reset();
  build_tx_frame();
  dma_rearm((uint)ch_rx);
  hw_set_bits(&spi0_hw->cr1, SPI_SSPCR1_SSE_BITS);
  dma_rearm((uint)ch_tx);

  // Above the default 0x80 priority so the USB ISR can't push the CS work
  // past the inter-transaction gap.
  gpio_add_raw_irq_handler(PIN_SPI_CS, spi_cs_irq);
  gpio_set_irq_enabled(PIN_SPI_CS, GPIO_IRQ_EDGE_RISE, true);
  irq_set_priority(IO_IRQ_BANK0, 0x40);
  irq_set_enabled(IO_IRQ_BANK0, true);
}

// DRDY must not change while CS is low. The ISR sets it on every CS rise, so
// a skipped thread-side hint loses nothing.
static inline void drdy_hint(bool level) {
  if (gpio_get(PIN_SPI_CS))
    gpio_put(PIN_DRDY, level);
}

// ---- Busy windows (§2.3) ----
// Rebuild the armed TX frame from thread context, only while the bus is idle.
// Wait up to 6 ms for CS high (a max transaction is ~4.1 ms at 8 MHz), then
// recheck with IRQs off. On timeout leave the frame alone: busy_mode steers
// the next ISR build anyway.
//
// A pending CS-rise IRQ also means hands off: that frame has just gone out,
// and unbuilding it would reuse its seq for different bytes, which the master
// would drop as a duplicate and ack, losing data.
static void swap_armed_frame(void) {
  uint32_t t0 = time_us_32();
  while (!gpio_get(PIN_SPI_CS) && (time_us_32() - t0) < 6000u)
    tight_loop_contents();
  uint32_t s = save_and_disable_interrupts();
  if (gpio_get(PIN_SPI_CS) &&
      !(gpio_get_irq_event_mask(PIN_SPI_CS) & GPIO_IRQ_EDGE_RISE)) {
    dma_quiesce((uint)ch_tx);
    // The old frame's first 8 bytes are already preloaded in the TX FIFO and
    // would corrupt the new frame, so flush and verify as the ISR does.
    hw_clear_bits(&spi0_hw->cr1, SPI_SSPCR1_SSE_BITS);
    while (spi0_hw->sr & SPI_SSPSR_RNE_BITS)
      (void)spi0_hw->dr;
    unbuild_prebuilt();
    build_tx_frame();
    hw_set_bits(&spi0_hw->cr1, SPI_SSPCR1_SSE_BITS);
    dma_rearm((uint)ch_tx);
    tx_arm_verify();
  }
  restore_interrupts(s);
}

void spi_link_prepare_busy(void) {
  if (!inited)
    return;
  busy_mode = true;
  swap_armed_frame(); // the already-armed frame must say avail=0 too
  drdy_hint(false);
}

void spi_link_end_busy(void) {
  if (!inited)
    return;
  busy_mode = false;
  swap_armed_frame(); // advertise what queued up during the flux op
  drdy_hint(tx_prod != tx_acked);
}

// ---- Thread-side ring access (transport_backend_t) ----
static void rx_apply_flush(void) {
  if (rx_flush_req == rx_flush_done)
    return;
  // IRQs off so the ISR can't post a new cut point between the two reads.
  uint32_t s = save_and_disable_interrupts();
  uint32_t upto = rx_flush_upto;
  rx_flush_done = rx_flush_req;
  if ((int32_t)(upto - rx_cons) > 0)
    rx_cons = upto;
  restore_interrupts(s);
}

static int spi_rx_ready(void) {
  rx_apply_flush();
  return (int)(rx_prod - rx_cons);
}

static void spi_read(void *buf, uint32_t n) {
  rx_apply_flush();
  uint8_t *p = buf;
  uint32_t cons = rx_cons;
  uint32_t o = cons & (SPI_RX_RING_SZ - 1);
  uint32_t first = SPI_RX_RING_SZ - o;
  if (first > n)
    first = n;
  memcpy(p, rx_ring + o, first);
  memcpy(p + first, rx_ring, n - first);
  __compiler_memory_barrier(); // consume bytes before publishing freed space
  rx_cons = cons + n;
}

static int spi_tx_ready(void) {
  // Unacked bytes are the retransmit store: free space ends at tx_acked.
  return (int)(SPI_TX_RING_SZ - (tx_prod - tx_acked));
}

static void spi_write(const void *buf, uint32_t n) {
  const uint8_t *p = buf;
  uint32_t prod = tx_prod;
  uint32_t o = prod & (SPI_TX_RING_SZ - 1);
  uint32_t first = SPI_TX_RING_SZ - o;
  if (first > n)
    first = n;
  memcpy(tx_ring + o, p, first);
  memcpy(tx_ring, p + first, n - first);
  __compiler_memory_barrier(); // publish bytes before advancing prod
  tx_prod = prod + n;
  // Wake hint only. The armed frame still has the old avail, so the master's
  // next poll learns the new one. Rebuilding here could race a CS fall.
  if (!busy_mode)
    drdy_hint(true);
}

static void spi_pump(void) {
  if (!inited)
    spi_link_init(); // the transport seam has no per-backend init hook
  rx_apply_flush();
  drdy_hint(!busy_mode && (tx_prod != tx_acked));
}

static void spi_notify_owner(bool owned) { (void)owned; }

const transport_backend_t transport_spi_backend = {
    .rx_ready = spi_rx_ready,
    .read = spi_read,
    .tx_ready = spi_tx_ready,
    .write = spi_write,
    .pump = spi_pump,
    .notify_owner = spi_notify_owner,
};

const transport_backend_t *spi_transport_backend(void) {
  return &transport_spi_backend;
}

const spi_link_stats_t *spi_link_stats(void) { return &st; }

int spi_rail_state(void) { return rail_state; }

bool spi_take_rail_dropped(void) {
  if (!rail_dropped)
    return false;
  rail_dropped = false;
  return true;
}
