// SPDX-License-Identifier: Unlicense
// Greaseweazle command engine. A command-wait state dispatches at most one
// command per pass; anything longer runs as its own state so the superloop
// keeps pumping the transports. Flux captures and writes block for the
// duration of the disk operation (store-and-forward). All I/O goes through
// the transport seam; USB state is never consulted.
#include "gw_floppy.h"

#include <string.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include "board_rp2354.h"
#include "diag.h"
#include "floppy_drive.h"
#include "flux.h"
#include "gw_proto.h"
#include "transport.h"
#include "transport_spi.h"

#define CHUNK 64

// The superloop stops for the whole capture and the busy bracket advertises
// avail=0, so spend a few ms in the superloop first to let the master (polling
// DRDY every <=5 ms) collect the READ_FLUX ack inside its 500 ms budget.
#define READ_ACK_DRAIN_MS 15

// The motor is already spinning (MOTOR blocks until spun up), so an index edge
// is due within one revolution (~203 ms worst case). Timeout -> ACK_NO_INDEX.
// Upstream allows 2 s from read start.
#define READ_CUE_MS 250

// ---- engine state ----

enum {
  ST_command_wait,
  ST_read_stream,
  ST_write_rx,
  ST_erase,
  ST_source_bytes,
  ST_sink_bytes,
  ST_status_reply, // 1 raw status byte pending (write/erase/sink completion)
};

// ---- drive power (the ESP's 5 V rail, reported via GWSPI_FLAG_RAIL) ----
//  - If the rail went off, the drive was power-cycled: forget the head
//    position so the next seek recalibrates.
//  - A USB host's first commands can arrive before the ESP notices
//    GWSPI_FLAG_USB (it polls every 250 ms), so wait for the rail and let the
//    drive come out of power-on reset. Skipped if no ESP has ever spoken
//    (rail -1), so an RP on USB alone doesn't stall 2 s per command.
#define RAIL_WAIT_MS 2000
#define RAIL_SETTLE_MS 300
static void drive_power_gate(void) {
  if (transport_owner() == TRANSPORT_USB && spi_rail_state() == 0) {
    absolute_time_t dl = make_timeout_time_ms(RAIL_WAIT_MS);
    while (spi_rail_state() == 0 && !time_reached(dl))
      sleep_ms(1);
    if (spi_rail_state() == 1)
      sleep_ms(RAIL_SETTLE_MS);
  }
  if (spi_take_rail_dropped())
    drive_forget_position();
}

static int state;

// Frames are [cmd, total_len, payload...]; total_len includes the header.
static uint8_t cmd_buf[64];
static uint32_t cmd_prod;
static uint32_t cmd_discard; // oversized-frame bytes still to swallow
static uint8_t discard_cmd;  // echo byte for the deferred bad reply

static uint8_t flux_status = ACK_OKAY; // persisted for GET_FLUX_STATUS
static uint8_t pending_status;         // payload of ST_status_reply
static uint8_t bus_type = BUS_NONE;

static inline uint32_t umin(uint32_t a, uint32_t b) { return a < b ? a : b; }
static inline uint16_t get_le16(const uint8_t *p) {
  return (uint16_t)(p[0] | (p[1] << 8));
}
static inline uint32_t get_le32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

// ---- watchdog ----

static struct {
  bool armed;
  absolute_time_t deadline;
} wd;

static void watchdog_kick(void) {
  wd.deadline = make_timeout_time_ms(drive_delays()->watchdog);
}

static void watchdog_arm(void) {
  wd.armed = true;
  watchdog_kick();
}

// Abort whatever is in flight and park the drive. Re-armed by the next command.
static void watchdog_quiesce(void) {
  gpio_put(PIN_WRGATE, 1); // a quiesced ERASE_FLUX must not keep erasing
  drive_motor(false);
  drive_select(false);
  state = ST_command_wait;
  cmd_prod = 0;
  cmd_discard = 0;
  wd.armed = false;
  g_diag.wdt_quiesces++;
}

static void end_command(const void *reply, uint32_t n) {
  watchdog_arm();
  transport_write(reply, n);
  cmd_prod = 0;
}

// ---- READ_FLUX ----
// Index markers are already inline in the captured stream (flux_pio.c).

static struct {
  enum { RD_DRAIN, RD_CAPTURE, RD_STREAM, RD_TERM } phase;
  absolute_time_t drain_until;
  uint32_t ticks;      // ticks bound (0 = none)
  uint16_t revs_max;   // index-mode revolutions wanted (0 = ticks mode)
  uint16_t revs_done;
  bool tick_bound;     // both bounds given: ticks acts as a wall-clock cap
  absolute_time_t deadline;
  uint8_t status;
  uint32_t pos, len;  // streaming cursor over flux_buf
  bool first_capture; // index mode: cue-edge marker wanted (first rev only)
} rd;

static void read_stream_pass(void) {
  switch (rd.phase) {

  case RD_DRAIN:
    if (!time_reached(rd.drain_until))
      return;
    rd.phase = RD_CAPTURE;
    return;

  case RD_CAPTURE: {
    // With both bounds, ticks is a wall-clock deadline checked between
    // revolutions, so it can overshoot by up to one revolution (upstream cuts
    // mid-revolution).
    if (rd.revs_max && rd.tick_bound && time_reached(rd.deadline)) {
      rd.phase = RD_TERM;
      return;
    }
    uint32_t len;
    uint8_t st;
    if (rd.revs_max == 0) {
      // Ticks mode: one bounded, uncued capture.
      st = flux_capture(rd.ticks, 0, false, &len);
    } else {
      // Index mode, one index-to-index revolution per capture. Only the first
      // capture marks its cue edge; otherwise the host would decode a
      // zero-length revolution between captures. Revolutions are streamed
      // between captures, so they aren't contiguous on disk, but each is
      // complete.
      st = flux_capture(0, READ_CUE_MS, rd.first_capture, &len);
      rd.first_capture = false;
    }
    if (st != ACK_OKAY) {
      // Drop the data; the host reads the reason from GET_FLUX_STATUS.
      rd.status = st;
      rd.phase = RD_TERM;
      return;
    }
    rd.pos = 0;
    rd.len = len;
    rd.phase = RD_STREAM;
    return;
  }

  case RD_STREAM:
    // Return whenever the transport is full so the superloop keeps pumping.
    while (transport_tx_ready(CHUNK)) {
      if (rd.pos < rd.len) {
        uint32_t n = umin(CHUNK, rd.len - rd.pos);
        transport_write(&flux_buf[rd.pos], n);
        rd.pos += n;
      } else {
        rd.revs_done++;
        bool more = rd.revs_max && (rd.revs_done < rd.revs_max) &&
                    !(rd.tick_bound && time_reached(rd.deadline));
        rd.phase = more ? RD_CAPTURE : RD_TERM;
        return;
      }
      watchdog_kick();
    }
    return;

  case RD_TERM:
  default:
    if (!transport_tx_ready(CHUNK))
      return;
    {
      const uint8_t term = 0x00;
      transport_write(&term, 1);
    }
    flux_status = rd.status;
    watchdog_arm();
    state = ST_command_wait;
    return;
  }
}

// ---- WRITE_FLUX ----

static struct {
  uint32_t len;       // stream bytes staged (terminator excluded)
  uint8_t status;
  bool discard;       // overflow: keep consuming, stop storing
  uint32_t discarded;
  bool cue, term;
  absolute_time_t stall;
} wr;

static void write_rx_pass(void) {
  int avail = transport_rx_ready();
  if (avail <= 0) {
    if (time_reached(wr.stall)) {
      wr.status = ACK_BAD_COMMAND;
      goto reply;
    }
    return;
  }
  wr.stall = make_timeout_time_ms(WRITE_RX_STALL_MS);
  watchdog_kick();

  while (avail > 0) {
    uint32_t n;
    if (!wr.discard) {
      uint32_t space = FLUX_BUF_SZ - wr.len;
      if (space == 0) {
        // Keep consuming to the terminator so the stream stays in sync.
        wr.status = ACK_FLUX_OVERFLOW;
        g_diag.flux_overflow++;
        wr.discard = true;
        continue;
      }
      n = umin((uint32_t)avail, space);
      transport_read(&flux_buf[wr.len], n);
      for (uint32_t i = 0; i < n; i++) {
        if (flux_buf[wr.len + i] == 0x00) {
          wr.len += i;
          goto have_stream;
        }
      }
      wr.len += n;
    } else {
      uint8_t junk[CHUNK];
      n = umin((uint32_t)avail, CHUNK);
      transport_read(junk, n);
      for (uint32_t i = 0; i < n; i++) {
        if (junk[i] == 0x00)
          goto have_stream; // status already ACK_FLUX_OVERFLOW
      }
      wr.discarded += n;
      if (wr.discarded > FLUX_BUF_SZ)
        goto reply; // bounded discard: no sane host sends this much extra
    }
    avail -= (int)n;
  }
  return;

have_stream:
  if (wr.status == ACK_OKAY) {
    if (!flux_stream_valid(flux_buf, wr.len)) {
      wr.status = ACK_BAD_COMMAND; // unwritable opcodes: skip the write
    } else {
      // Whole stream is buffered, so the link can't fall behind the disk.
      // Blocks for up to ~2.2 s (cue timeout + revolution); IRQs stay on.
      wr.status = flux_write(wr.len, wr.cue, wr.term);
    }
  }
reply:
  flux_status = wr.status;
  pending_status = wr.status;
  state = ST_status_reply;
}

// ---- ERASE_FLUX ----

static struct {
  absolute_time_t end;
} er;

static void erase_pass(void) {
  if (!time_reached(er.end)) {
    // An erase can outlast the watchdog period.
    watchdog_kick();
    return;
  }
  gpio_put(PIN_WRGATE, 1);
  sleep_us(DELAY_POST_WRITE_US);
  pending_status = ACK_OKAY;
  state = ST_status_reply;
}

// ---- SOURCE_BYTES / SINK_BYTES (link soak tests) ----

// Bandwidth stats: timestamp every 4th byte position in a ring the size of
// flux_buf (which holds the timestamps) and report min/max time per lap.
#define BW_WIN ((uint32_t)FLUX_BUF_SZ)
_Static_assert((FLUX_BUF_SZ & (FLUX_BUF_SZ - 1)) == 0,
               "BW ring wraps by masking: FLUX_BUF_SZ must be a power of two");

static struct {
  uint32_t todo;
  uint32_t rand;
  uint8_t status;
  uint32_t p;               // byte position (drives the timestamp ring)
  uint32_t min_us, max_us;  // GETINFO_BW_STATS source
} ss = {.min_us = UINT32_MAX};

static uint32_t ss_rand_next(uint32_t x) {
  return (x & 1) ? (x >> 1) ^ 0x80000062 : x >> 1;
}

static void ss_update_deltas(uint32_t n) {
  uint32_t *times = (uint32_t *)(void *)flux_buf;
  uint32_t now = time_us_32();
  while (n--) {
    if (ss.p++ & 3)
      continue;
    uint32_t slot = (ss.p & (BW_WIN - 1)) >> 2;
    uint32_t delta = now - times[slot];
    times[slot] = now;
    if (ss.p >= BW_WIN) { // first lap only primes the ring
      if (delta > ss.max_us)
        ss.max_us = delta;
      if (delta < ss.min_us)
        ss.min_us = delta;
    }
  }
}

static void source_pass(void) {
  while (ss.todo && transport_tx_ready(CHUNK)) {
    uint8_t data[CHUNK];
    uint32_t n = umin(ss.todo, CHUNK);
    for (uint32_t i = 0; i < n; i++) {
      data[i] = (uint8_t)ss.rand;
      ss.rand = ss_rand_next(ss.rand);
    }
    transport_write(data, n);
    ss.todo -= n;
    ss_update_deltas(n);
    watchdog_kick(); // soaks can run for minutes
  }
  if (!ss.todo) {
    watchdog_arm();
    cmd_prod = 0;
    state = ST_command_wait; // no status byte after SOURCE
  }
}

static void sink_pass(void) {
  if (!ss.todo) {
    pending_status = ss.status;
    state = ST_status_reply;
    return;
  }
  int avail = transport_rx_ready();
  while (avail > 0 && ss.todo) {
    uint8_t data[CHUNK];
    uint32_t n = umin(umin((uint32_t)avail, CHUNK), ss.todo);
    transport_read(data, n);
    for (uint32_t i = 0; i < n; i++) {
      if (data[i] != (uint8_t)ss.rand)
        ss.status = ACK_BAD_COMMAND;
      ss.rand = ss_rand_next(ss.rand);
    }
    ss.todo -= n;
    avail -= (int)n;
    ss_update_deltas(n);
    watchdog_kick();
  }
}

// ---- status-byte reply (write / erase / sink completion) ----

static void status_reply_pass(void) {
  if (!transport_tx_ready(1))
    return;
  // One raw byte, not [cmd, ack]: the host reads exactly one.
  transport_write(&pending_status, 1);
  watchdog_arm();
  cmd_prod = 0;
  state = ST_command_wait;
}

// On-chip temperature sensor (ADC input 4), initialised on first use.
// Datasheet: T = 27 - (V - 0.706) / 0.001721, V = raw * 3.3 / 4096.
// Only accurate to a few degrees; good for trends and overheat warnings.
static bool die_temp_c100(int *out_c100) {
  static bool adc_ready = false;
  if (!adc_ready) {
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_ready = true;
    busy_wait_us(200); // let the sensor settle before the first conversion
  }
  adc_select_input(4);
  uint32_t raw = adc_read();
  int v_uv = (int)((raw * 3300000ull) / 4096ull);
  // In microvolts: T_c100 = 2700 - (V_uV - 706000) * 100 / 1721.
  // One ADC count is ~0.47 C, so readings move in steps of ~47.
  *out_c100 = 2700 - (int)(((int64_t)(v_uv - 706000) * 100) / 1721);
  return true;
}

// ---- command dispatch ----

static void process_command(void) {
  uint8_t cmd = cmd_buf[0];
  uint8_t len = cmd_buf[1];
  uint8_t reply[2 + 32];
  uint32_t resp_sz = 2;

  watchdog_arm();

  reply[0] = cmd;
  reply[1] = ACK_OKAY;

  switch (cmd) {

  case CMD_GET_INFO: {
    if (len != 3)
      goto bad_length;
    uint8_t sub = cmd_buf[2];
    memset(&reply[2], 0, 32);
    if (sub == GETINFO_FIRMWARE) {
      // sample_freq drives the ESP's MFM PLL; keep every field truthful.
      gw_info_t info = {
          .fw_major = GW_FW_MAJOR,
          .fw_minor = GW_FW_MINOR,
          .is_main_firmware = 1,
          .max_cmd = GW_MAX_CMD,
          .sample_freq = GW_SAMPLE_FREQ,
          .hw_model = GW_HW_MODEL,
          .hw_submodel = GW_HW_SUBMODEL,
          .usb_speed = 0, // full speed
          .mcu_id = 0,    // no keirf-assigned id for RP2 parts
          .mcu_mhz = GW_MCU_MHZ,
          .mcu_sram_kb = GW_MCU_SRAM_KB,
          .usb_buf_kb = GW_USB_BUF_KB,
      };
      memcpy(&reply[2], &info, sizeof(info));
    } else if (sub == GETINFO_BW_STATS) {
      // {min_bw, max_bw}: min is the slowest lap, max the fastest.
      struct __attribute__((packed)) {
        uint32_t bytes, usecs;
      } bw[2] = {{BW_WIN, ss.max_us}, {BW_WIN, ss.min_us}};
      memcpy(&reply[2], bw, sizeof(bw));
    } else if (sub == GETINFO_GW_DIAG) {
      // First 32 bytes of g_diag (the SPI/proto counters). The full set is on
      // the CDC1 console.
      memcpy(&reply[2], &g_diag, umin(32, sizeof(g_diag)));
    } else if (sub == GETINFO_GW_HEALTH) {
      gw_health_t h = {
          .die_temp_c100 = 0,
          .temp_valid = 0,
          .uptime_ms = (uint32_t)(time_us_64() / 1000u),
      };
      int t100;
      if (die_temp_c100(&t100)) {
        h.die_temp_c100 = (int16_t)t100;
        h.temp_valid = 1;
      }
      memcpy(&reply[2], &h, sizeof(h));
    } else {
      goto bad_command;
    }
    resp_sz += 32;
    break;
  }

  case CMD_SEEK: {
    // len 3 = int8 cylinder, len 4 = int16.
    int cyl;
    if (len == 3)
      cyl = (int8_t)cmd_buf[2];
    else if (len == 4)
      cyl = (int16_t)get_le16(&cmd_buf[2]);
    else
      goto bad_length;
    drive_power_gate();
    reply[1] = drive_seek(cyl);
    break;
  }

  case CMD_HEAD: {
    if (len != 3)
      goto bad_length;
    if (cmd_buf[2] > 1)
      goto bad_command;
    drive_head(cmd_buf[2]);
    break;
  }

  case CMD_SET_PARAMS: {
    // Index 0 = delays. A host may set just the leading fields.
    if (len < 3 || cmd_buf[2] != 0 || len > 3 + sizeof(gw_delay_t))
      goto bad_length;
    memcpy(drive_delays(), &cmd_buf[3], len - 3);
    break;
  }

  case CMD_GET_PARAMS: {
    uint8_t nr = cmd_buf[3];
    if (len != 4 || cmd_buf[2] != 0 || nr > sizeof(gw_delay_t))
      goto bad_length;
    memcpy(&reply[2], drive_delays(), nr);
    resp_sz += nr;
    break;
  }

  case CMD_MOTOR: {
    // {unit, on}. drive_motor blocks for motor_delay, so the ack tells the
    // host the drive is spun up (host budget 1500 ms).
    if (len != 4 || (cmd_buf[3] & ~1))
      goto bad_length;
    if (bus_type == BUS_NONE) {
      reply[1] = ACK_NO_BUS;
      break;
    }
    if (cmd_buf[2] > 1) {
      reply[1] = ACK_BAD_UNIT;
      break;
    }
    if (cmd_buf[3] & 1)
      drive_power_gate();
    drive_motor(cmd_buf[3] & 1);
    break;
  }

  case CMD_READ_FLUX: {
    // {ticks u32, max_index u16, linger u32}, linger optional.
    struct __attribute__((packed)) {
      uint32_t ticks;
      uint16_t max_index;
      uint32_t linger;
    } rf = {0};
    if (len < 8 || len > 12)
      goto bad_length;
    memcpy(&rf, &cmd_buf[2], len - 2);
    // linger is ignored: captures end exactly on an index edge or tick count.
    (void)rf.linger;
    if (!rf.ticks && !rf.max_index)
      goto bad_command; // an unbounded read can't be buffered
    memset(&rd, 0, sizeof(rd));
    rd.phase = RD_DRAIN;
    rd.drain_until = make_timeout_time_ms(READ_ACK_DRAIN_MS);
    rd.ticks = rf.ticks;
    // Upstream's stream carries max_index index markers. N captures here
    // carry N+1 (the first also marks its cue edge), so capture max_index-1
    // revolutions, minimum one.
    rd.revs_max = rf.max_index > 1 ? (uint16_t)(rf.max_index - 1) : rf.max_index;
    rd.status = ACK_OKAY;
    rd.first_capture = (rf.max_index != 0);
    rd.tick_bound = (rf.max_index && rf.ticks);
    if (rd.tick_bound)
      rd.deadline = make_timeout_time_us((uint64_t)rf.ticks * 1000000u /
                                         GW_SAMPLE_FREQ);
    state = ST_read_stream;
    break;
  }

  case CMD_WRITE_FLUX: {
    struct __attribute__((packed)) {
      uint8_t cue, term;
      uint32_t hard_sector_ticks;
    } wf = {0};
    if (len < 4 || len > 8)
      goto bad_length;
    memcpy(&wf, &cmd_buf[2], len - 2);
    if (wf.hard_sector_ticks)
      goto bad_command; // hard-sectored media unsupported
    if (drive_wrprot()) {
      reply[1] = ACK_WRPROT; // host won't send the stream after a nack
      break;
    }
    memset(&wr, 0, sizeof(wr));
    wr.status = ACK_OKAY;
    wr.cue = wf.cue != 0;
    wr.term = wf.term != 0;
    wr.stall = make_timeout_time_ms(WRITE_RX_STALL_MS);
    state = ST_write_rx;
    break;
  }

  case CMD_GET_FLUX_STATUS:
    if (len != 2)
      goto bad_length;
    reply[1] = flux_status;
    break;

  case CMD_SELECT: {
    if (len != 3)
      goto bad_length;
    if (bus_type == BUS_NONE) {
      reply[1] = ACK_NO_BUS;
      break;
    }
    // Units 0 and 1 both use the single SELECT line (the ESP sends 1, gw.exe
    // usually 0).
    if (cmd_buf[2] > 1) {
      reply[1] = ACK_BAD_UNIT;
      break;
    }
    drive_select(true);
    break;
  }

  case CMD_DESELECT:
    if (len != 2)
      goto bad_length;
    drive_select(false);
    break;

  case CMD_SET_BUS_TYPE: {
    if (len != 3)
      goto bad_length;
    uint8_t t = cmd_buf[2];
    if (t > BUS_SHUGART)
      goto bad_command;
    if (t != bus_type) {
      drive_motor(false);
      drive_select(false);
      bus_type = t;
    }
    break;
  }

  case CMD_SET_PIN: {
    if (len != 4)
      goto bad_length;
    if (cmd_buf[3] & ~1)
      goto bad_command;
    if (cmd_buf[2] != 2) { // pin 2 = density select, the only settable pin
      reply[1] = ACK_BAD_PIN;
      break;
    }
    drive_set_density(cmd_buf[3] & 1);
    break;
  }

  case CMD_GET_PIN: {
    // Replies are raw bus levels (inputs are active-low). Ids 8/26/28/34 are
    // ribbon pins; 100/101 are our own ids for the slimline-only lines.
    if (len != 3)
      goto bad_length;
    uint8_t level;
    switch (cmd_buf[2]) {
    case 8:
      level = drive_index() ? 0 : 1;
      break;
    case 26:
      level = drive_trk0() ? 0 : 1;
      break;
    case 28:
      level = drive_wrprot() ? 0 : 1;
      break;
    case 34: // disk change
      if (!drive_has_dskchg()) {
        reply[1] = ACK_BAD_PIN;
        goto send;
      }
      level = drive_dskchg() ? 0 : 1;
      break;
    case 100: // media density sense
      if (!drive_has_dens_sense()) {
        reply[1] = ACK_BAD_PIN;
        goto send;
      }
      level = drive_dens_sense() ? 0 : 1;
      break;
    case 101: // drive ready
      if (!drive_has_rdy()) {
        reply[1] = ACK_BAD_PIN;
        goto send;
      }
      level = drive_rdy() ? 0 : 1;
      break;
    default:
      reply[1] = ACK_BAD_PIN;
      goto send; // no payload on error
    }
    reply[2] = level;
    resp_sz += 1;
    break;
  }

  case CMD_RESET:
    if (len != 2)
      goto bad_length;
    drive_reset();
    bus_type = BUS_NONE;
    break;

  case CMD_ERASE_FLUX: {
    // Ack, hold WRGATE for the tick count as its own state (it can be
    // minutes), then send one raw status byte.
    if (len != 6)
      goto bad_length;
    uint32_t ticks = get_le32(&cmd_buf[2]);
    if (drive_wrprot()) {
      reply[1] = ACK_WRPROT;
      break;
    }
    sleep_us(DELAY_PRE_WRITE_US);
    gpio_set_dir(PIN_WRGATE, GPIO_OUT);
    gpio_put(PIN_WRGATE, 0);
    er.end = make_timeout_time_us((uint64_t)ticks * 1000000u / GW_SAMPLE_FREQ);
    flux_status = ACK_OKAY;
    state = ST_erase;
    break;
  }

  case CMD_SOURCE_BYTES:
  case CMD_SINK_BYTES: {
    // {nr_bytes u32, seed u32}
    if (len != 10)
      goto bad_length;
    ss.todo = get_le32(&cmd_buf[2]);
    ss.rand = get_le32(&cmd_buf[6]);
    ss.status = ACK_OKAY;
    ss.p = 0;
    ss.min_us = UINT32_MAX;
    ss.max_us = 0;
    state = (cmd == CMD_SOURCE_BYTES) ? ST_source_bytes : ST_sink_bytes;
    break;
  }

  case CMD_UPDATE:          // the ESP flashes over SWD instead
  case CMD_GET_INDEX_TIMES: // deprecated upstream
  case CMD_SWITCH_FW_MODE:
  case CMD_TEST_MODE:
  case CMD_NO_CLICK_STEP:
  default:
    goto bad_command;
  }

send:
  end_command(reply, resp_sz);
  return;

bad_command:
  g_diag.proto_bad_cmd++;
  reply[1] = ACK_BAD_COMMAND;
  resp_sz = 2;
  goto send;

bad_length:
  g_diag.proto_bad_len++;
  reply[1] = ACK_BAD_COMMAND;
  resp_sz = 2;
  goto send;
}

// ---- command-wait ----

static void command_wait_pass(void) {
  transport_arbitrate();

  // Oversized frame: swallow the declared remainder, then nack.
  if (cmd_discard) {
    int avail = transport_rx_ready();
    while (avail > 0 && cmd_discard) {
      uint8_t junk[CHUNK];
      uint32_t n = umin(umin((uint32_t)avail, CHUNK), cmd_discard);
      transport_read(junk, n);
      cmd_discard -= n;
      avail -= (int)n;
    }
    if (cmd_discard || !transport_tx_ready(CHUNK))
      return;
    uint8_t r[2] = {discard_cmd, ACK_BAD_COMMAND};
    end_command(r, 2);
    return;
  }

  int avail = transport_rx_ready();
  if (avail > 0) {
    uint32_t n = umin((uint32_t)avail, sizeof(cmd_buf) - cmd_prod);
    if (n) {
      transport_read(&cmd_buf[cmd_prod], n);
      cmd_prod += n;
    }
  }

  // Longer than any real command: count it and discard (at most 255 bytes).
  if (cmd_prod >= 2 && cmd_buf[1] > sizeof(cmd_buf)) {
    g_diag.proto_bad_len++;
    discard_cmd = cmd_buf[0];
    cmd_discard = cmd_buf[1] - cmd_prod;
    cmd_prod = 0;
    return;
  }

  // Bytes past the frame are dropped by end_command(); hosts never pipeline.
  if (cmd_prod >= 2 && cmd_prod >= cmd_buf[1] && transport_tx_ready(CHUNK))
    process_command();
}

// ---- public entry points ----

void gw_floppy_init(void) {
  state = ST_command_wait;
  cmd_prod = 0;
  cmd_discard = 0;
  wd.armed = false; // armed by the first command; an idle boot never quiesces
  flux_status = ACK_OKAY;
  bus_type = BUS_NONE;
}

void gw_floppy_process(void) {
  if (wd.armed && time_reached(wd.deadline))
    watchdog_quiesce();

  switch (state) {
  case ST_command_wait:
    command_wait_pass();
    break;
  case ST_read_stream:
    read_stream_pass();
    break;
  case ST_write_rx:
    write_rx_pass();
    break;
  case ST_erase:
    erase_pass();
    break;
  case ST_source_bytes:
    source_pass();
    break;
  case ST_sink_bytes:
    sink_pass();
    break;
  case ST_status_reply:
    status_reply_pass();
    break;
  default:
    state = ST_command_wait;
    break;
  }
}
