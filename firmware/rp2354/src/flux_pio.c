// SPDX-License-Identifier: Unlicense
// PIO flux engine. Design notes: docs/flux-engine.md.
//
// Capture: flux_read.pio times RDDATA intervals, DMA drains them into a raw
// ring, and the CPU encodes them (gw_stream.c) while polling INDEX for inline
// markers. Write: the stream is decoded into interval words and DMA feeds
// flux_write.pio from two 4 KB buffers, refilled in small slices so the INDEX
// poll never waits more than ~10 us. IRQs are never masked.
#include "flux_pio.h"

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include "board_rp2354.h"
#include "gw_stream.h"

#include "flux_read.pio.h"
#include "flux_write.pio.h"

#define FLUX_PIO pio0
#define SM_READ 0
#define SM_WRITE 1

// Read SM at 50 MHz (2 cycles/tick), write SM at 25 MHz (1 cycle/tick).
#define CLKDIV_READ 3
#define CLKDIV_WRITE 6

// Cycles per interval the programs don't count (see the .pio audits).
#define FLUX_READ_BIAS_TICKS 2u
#define FLUX_WRITE_BIAS_TICKS 13u

// 512-sample DMA ring (2^11 bytes): ~0.5 ms of slack at the fastest real flux
// rate (~1 sample/us).
#define CAP_RING_WORDS 512u
#define CAP_TRANS_MAX 0x0FFFFFFFu
static uint32_t cap_ring[CAP_RING_WORDS] __attribute__((aligned(2048)));

// Two 4 KB chunks of interval words (each >= 2 ms of disk time), refilled 32
// intervals (~10 us of decode) at a time between INDEX polls.
#define WR_CHUNK_WORDS 1024u
#define WR_STAGE_SLICE 32u
static uint32_t wr_buf[2][WR_CHUNK_WORDS];

// After a cue edge the closing edge is due within one revolution (~203 ms).
#define ONE_REV_DEADLINE_MS 500
// Slack on ticks-bounded captures for samples in flight.
#define TICKS_BOUND_MARGIN_US 50000

static uint off_read, off_write;
static int ch_cap = -1, ch_wr = -1;

// True once per falling edge of the active-low INDEX line.
static inline bool index_falling(bool *prev) {
  bool now = gpio_get(PIN_INDEX);
  bool fell = *prev && !now;
  *prev = now;
  return fell;
}

static bool cue_wait(bool *idx_prev, uint32_t timeout_ms) {
  absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
  while (!index_falling(idx_prev)) {
    if (time_reached(deadline))
      return false;
  }
  return true;
}

void flux_pio_init(void) {
  off_read = pio_add_program(FLUX_PIO, &flux_read_program);
  off_write = pio_add_program(FLUX_PIO, &flux_write_program);

  // JMP pin reads the raw pad, so RDDATA can stay an SIO input.
  pio_sm_config c = flux_read_program_get_default_config(off_read);
  sm_config_set_jmp_pin(&c, PIN_RDDATA);
  sm_config_set_clkdiv_int_frac(&c, CLKDIV_READ, 0);
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
  pio_sm_init(FLUX_PIO, SM_READ, off_read, &c);

  // Joined 8-deep TX FIFO (>= 16 us at MFM rates) covers a DMA re-arm gap.
  // WRDATA is muxed to PIO only during a write; it is SIO-high otherwise.
  c = flux_write_program_get_default_config(off_write);
  sm_config_set_sideset_pins(&c, PIN_WRDATA);
  sm_config_set_clkdiv_int_frac(&c, CLKDIV_WRITE, 0);
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
  pio_sm_init(FLUX_PIO, SM_WRITE, off_write, &c);
  pio_sm_set_consecutive_pindirs(FLUX_PIO, SM_WRITE, PIN_WRDATA, 1, true);

  ch_cap = dma_claim_unused_channel(true);
  ch_wr = dma_claim_unused_channel(true);

  dma_channel_config dc = dma_channel_get_default_config((uint)ch_cap);
  channel_config_set_read_increment(&dc, false);
  channel_config_set_write_increment(&dc, true);
  channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
  channel_config_set_dreq(&dc, pio_get_dreq(FLUX_PIO, SM_READ, false));
  channel_config_set_ring(&dc, true /* write */, 11); // 2048-byte ring
  dma_channel_configure((uint)ch_cap, &dc, cap_ring,
                        &FLUX_PIO->rxf[SM_READ], CAP_TRANS_MAX, false);

  dc =dma_channel_get_default_config((uint)ch_wr);
  channel_config_set_read_increment(&dc, true);
  channel_config_set_write_increment(&dc, false);
  channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
  channel_config_set_dreq(&dc, pio_get_dreq(FLUX_PIO, SM_WRITE, true));
  dma_channel_configure((uint)ch_wr, &dc, &FLUX_PIO->txf[SM_WRITE], wr_buf[0],
                        0, false);
}

// ---- capture ----

flux_cap_status_t flux_pio_capture(uint8_t *dst, uint32_t cap,
                                   uint32_t capture_counts, uint32_t cue_ms,
                                   bool cue_marker, uint32_t *len) {
  uint32_t out = 0;
  *len = 0;

  bool idx_prev = gpio_get(PIN_INDEX);
  if (cue_ms && !cue_wait(&idx_prev, cue_ms))
    return FLUX_CAP_NO_INDEX;

  if (cue_marker && cap >= GW_STREAM_INDEX_LEN)
    out += gw_stream_enc_index(dst + out, 0);

  // The first interval counts from here, a few us after the cue edge.
  pio_sm_set_enabled(FLUX_PIO, SM_READ, false);
  pio_sm_clear_fifos(FLUX_PIO, SM_READ);
  pio_sm_restart(FLUX_PIO, SM_READ);
  pio_sm_exec(FLUX_PIO, SM_READ, pio_encode_jmp(off_read));
  dma_channel_abort((uint)ch_cap);
  dma_channel_set_write_addr((uint)ch_cap, cap_ring, false);
  dma_channel_set_trans_count((uint)ch_cap, CAP_TRANS_MAX, true);
  pio_sm_set_enabled(FLUX_PIO, SM_READ, true);

  // Wall-clock bound so a silent RDDATA (no disk, dead drive, blank track)
  // still ends the capture.
  absolute_time_t deadline =
      capture_counts
          ? make_timeout_time_us((uint64_t)capture_counts * 1000000u /
                                     GW_SAMPLE_FREQ +
                                 TICKS_BOUND_MARGIN_US)
          : make_timeout_time_ms(ONE_REV_DEADLINE_MS);

  uint32_t consumed = 0;
  uint64_t total_ticks = 0;
  flux_cap_status_t st = FLUX_CAP_OK;
  bool done = false;

  while (!done) {
    uint32_t produced =
        CAP_TRANS_MAX - dma_hw->ch[ch_cap].transfer_count;
    if (produced - consumed > CAP_RING_WORDS) {
      // Overwritten before read. Shouldn't happen, but never return corrupt
      // flux.
      st = FLUX_CAP_OVERFLOW;
      break;
    }
    while (consumed != produced) {
      uint32_t ticks =
          ~cap_ring[consumed % CAP_RING_WORDS] + FLUX_READ_BIAS_TICKS;
      consumed++;
      if (out + GW_STREAM_ENC_MAX + GW_STREAM_INDEX_LEN > cap) {
        st = FLUX_CAP_OVERFLOW; // headroom for this code + a pending marker
        done = true;
        break;
      }
      out += gw_stream_enc_interval(dst + out, ticks);
      total_ticks += ticks;
      if (capture_counts && total_ticks >= capture_counts) {
        done = true;
        break;
      }
    }
    if (done)
      break;

    // A marker for every falling edge, within a couple of samples of the
    // true edge (docs/flux-engine.md).
    if (index_falling(&idx_prev)) {
      if (out + GW_STREAM_INDEX_LEN > cap) {
        st = FLUX_CAP_OVERFLOW;
        break;
      }
      out += gw_stream_enc_index(dst + out, 0);
      if (capture_counts == 0)
        done = true; // one-revolution mode
    }

    if (!done && time_reached(deadline)) {
      if (capture_counts == 0)
        st = FLUX_CAP_NO_INDEX;
      done = true; // ticks mode returns the partial capture
    }
  }

  pio_sm_set_enabled(FLUX_PIO, SM_READ, false);
  dma_channel_abort((uint)ch_cap);
  pio_sm_clear_fifos(FLUX_PIO, SM_READ);

  *len = out;
  return st;
}

// ---- write ----

// Decode up to `max` intervals. Sets *ended at end of stream (a decode error
// also ends it; flux_stream_valid() already vetted the stream).
static uint32_t stage_some(gw_stream_dec_t *dec, uint32_t *dst, uint32_t max,
                           bool *ended) {
  uint32_t n = 0;
  while (n < max) {
    uint32_t t;
    int r = gw_stream_dec_next(dec, &t);
    if (r != 1) {
      *ended = true;
      break;
    }
    if (t < FLUX_WRITE_BIAS_TICKS + 1)
      t = FLUX_WRITE_BIAS_TICKS + 1;
    dst[n++] = t - FLUX_WRITE_BIAS_TICKS;
  }
  return n;
}

flux_write_status_t flux_pio_write(const uint8_t *stream, uint32_t len,
                                   bool cue_at_index, bool terminate_at_index,
                                   uint32_t cue_timeout_ms) {
  gw_stream_dec_t dec;
  gw_stream_dec_init(&dec, stream, len);

  bool stream_end = false;
  uint32_t n0 = stage_some(&dec, wr_buf[0], WR_CHUNK_WORDS, &stream_end);
  uint32_t staged =
      stream_end ? 0 : stage_some(&dec, wr_buf[1], WR_CHUNK_WORDS, &stream_end);
  if (n0 == 0)
    return terminate_at_index ? FLUX_WRITE_UNDERFLOW : FLUX_WRITE_OK;

  bool idx_prev = gpio_get(PIN_INDEX);
  if (cue_at_index && !cue_wait(&idx_prev, cue_timeout_ms))
    return FLUX_WRITE_NO_INDEX;

  // The exec'd jmp needs `side 1`: side-set is mandatory in this program, so
  // it drives WRDATA. This latches the PIO output high before the pin is
  // muxed to PIO.
  pio_sm_set_enabled(FLUX_PIO, SM_WRITE, false);
  pio_sm_clear_fifos(FLUX_PIO, SM_WRITE);
  pio_sm_restart(FLUX_PIO, SM_WRITE);
  pio_sm_exec(FLUX_PIO, SM_WRITE,
              pio_encode_jmp(off_write) | pio_encode_sideset(1, 1));
  gpio_set_function(PIN_WRDATA, GPIO_FUNC_PIO0);
  pio_sm_set_enabled(FLUX_PIO, SM_WRITE, true);

  gpio_put(PIN_WRGATE, 0);
  uint run = 0;
  dma_channel_set_read_addr((uint)ch_wr, wr_buf[0], false);
  dma_channel_set_trans_count((uint)ch_wr, n0, true);

  // The SM stalled before DMA started: wait for the first word, then clear
  // TXSTALL so from here it means the FIFO ran dry.
  const uint32_t stall_bit = 1u << (PIO_FDEBUG_TXSTALL_LSB + SM_WRITE);
  for (uint32_t spin = 0;
       pio_sm_get_tx_fifo_level(FLUX_PIO, SM_WRITE) == 0 && spin < 10000;
       spin++)
    tight_loop_contents();
  FLUX_PIO->fdebug = stall_bit;

  // DMA owns wr_buf[run]; wr_buf[run^1] is being refilled.
  //
  // TXSTALL is sticky: "the SM has idled at pull since the write began". While
  // intervals are still pending that is an underflow (a stretched interval on
  // disk); once everything is queued it is the normal end signal.
  flux_write_status_t st = FLUX_WRITE_OK;
  for (;;) {
    if (index_falling(&idx_prev) && terminate_at_index) {
      st = FLUX_WRITE_OK;
      break;
    }
    bool stalled = (FLUX_PIO->fdebug & stall_bit) != 0; // sample before DMA
    bool dma_idle = !dma_channel_is_busy((uint)ch_wr);
    if (dma_idle && !staged && stream_end) {
      // All queued, and the previous pass saw no stall, so a stall now means
      // the SM finished.
      if (stalled && pio_sm_get_tx_fifo_level(FLUX_PIO, SM_WRITE) == 0) {
        st = terminate_at_index ? FLUX_WRITE_UNDERFLOW : FLUX_WRITE_OK;
        break;
      }
      continue;
    }
    if (stalled) {
      // Ran dry mid-stream: an ISR outlasted the FIFO at a buffer swap, or
      // decode fell behind.
      st = FLUX_WRITE_UNDERFLOW;
      break;
    }
    if (dma_idle && staged) {
      if (FLUX_PIO->fdebug & stall_bit) {
        st = FLUX_WRITE_UNDERFLOW;
        break;
      }
      run ^= 1;
      dma_channel_set_read_addr((uint)ch_wr, wr_buf[run], false);
      dma_channel_set_trans_count((uint)ch_wr, staged, true);
      staged = 0;
    }
    if (!stream_end && staged < WR_CHUNK_WORDS) {
      uint32_t want = WR_CHUNK_WORDS - staged;
      if (want > WR_STAGE_SLICE)
        want = WR_STAGE_SLICE;
      staged += stage_some(&dec, wr_buf[run ^ 1] + staged, want, &stream_end);
    }
  }

  // WRGATE off first, then park WRDATA on SIO, driven high.
  gpio_put(PIN_WRGATE, 1);
  dma_channel_abort((uint)ch_wr);
  pio_sm_set_enabled(FLUX_PIO, SM_WRITE, false);
  pio_sm_clear_fifos(FLUX_PIO, SM_WRITE);
  gpio_put(PIN_WRDATA, 1);
  gpio_set_dir(PIN_WRDATA, GPIO_OUT);
  gpio_set_function(PIN_WRDATA, GPIO_FUNC_SIO);

  return st;
}
