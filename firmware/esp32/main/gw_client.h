// Greaseweazle host-side client: the stock GW command protocol over link.h.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "gw_proto.h"

// Pulse the peer's RUN line, then bring the link up. Call once, early. A dead
// link here is non-fatal.
esp_err_t gw_client_init(void);

// Exclusive ownership of the controller (link, RUN line, drive). Held by
// gw_task for every drive session and idle poll, and by the SWD reflash.
// wait_ms UINT32_MAX = forever. Not recursive.
bool gw_claim(uint32_t wait_ms);
void gw_release(void);

// GetInfo: checks main firmware is running and latches sample_freq.
bool gw_handshake(void);
uint32_t gw_sample_freq(void);

// Reset the peer over RUN and bring the link back up; retry gw_handshake()
// afterwards. False if SYNC times out.
bool gw_relink(void);

// Reason for the last failure, for /api/status. Empty until one occurs.
const char *gw_last_error(void);

// Controller counters (GetInfo 0x88) and health (0x89). refresh talks to the
// controller, so gw task only; get reads the cached copy from any task and
// returns false until the first successful sample.
bool gw_refresh_diag(void);
bool gw_get_diag(struct gw_diag_counters *out, uint32_t *age_ms);
bool gw_get_health(struct gw_health *out, uint32_t *age_ms);
// Controller link profile, refreshed with the diag counters after each drive
// session. False if never read (or the controller lacks GetInfo 0x8A).
bool gw_get_linkprof(struct gw_linkprof *out, uint32_t *age_ms);
bool gw_get_capprof(struct gw_capprof *out);

// One idle tick: refresh health and, if the controller has gone silent,
// reset it and relink. Blocks for seconds when it does. gw task only, between
// drive sessions. Gives up on controllers without 0x89.
void gw_idle_poll(void);

// USB host on the controller, from GWSPI_FLAG_USB. gw_usb_watch() runs every
// idle tick (gw_claim held) and raises/drops the drive rail. While active,
// nothing may put a command on the link.
void gw_usb_watch(void);
bool gw_usb_active(void);

bool gw_drive_prepare(void);   // rail on, bus type, select, motor on
void gw_drive_release(void);   // motor off, deselect, rail off; also the abort path

// Raw flux buffer size for one ReadFlux. Must be >= the controller's staging
// buffer (GetInfo usb_buf_kb, 256 KB on the RP2354A): a larger capture would
// overflow read_flux_stream() and leave bytes in the link. gw_handshake()
// logs an error if the controller reports more.
#define GW_FLUX_RAW_MAX (272 * 1024)

// Per-read phase timing in microseconds. Zeroed on entry.
typedef struct {
    uint32_t seek_us;      // SEEK + HEAD
    uint32_t readxfer_us;  // READ_FLUX until the stream terminator
    uint32_t ttfb_us;      // READ_FLUX until the first flux byte
    uint32_t ackwait_us;   // READ_FLUX until its ack
    uint32_t status_us;    // GET_FLUX_STATUS
} gw_read_timing_t;

// Seek, read GW_READ_WINDOW_MS of raw flux into `raw` (including the 0x00
// terminator) and fetch the flux status. Decoding is left to the caller so
// it can run on another core. Returns 0 or negative; `t` may be NULL.
int gw_read_flux_raw(uint8_t cyl, uint8_t head, uint8_t *raw, size_t raw_cap,
                     size_t *raw_len, gw_read_timing_t *t);

// gw_read_flux_raw() result for ACK_NO_TRK0 on SEEK: no drive, unpowered or a
// reversed cable. Not a link failure, so don't reset the controller for it.
#define GW_RD_NO_DRIVE (-7)

// GET_FLUX_STATUS of the last read, or 0xFF if it never got that far.
// ACK_NO_INDEX on a successful read means no flux at all (no disk).
uint8_t gw_last_flux_status(void);

// After a failed command: drain any late reply and verify alignment with
// GetInfo. False means relink. Link-owning task only.
bool gw_resync(void);

// Write one track of 0x00-terminated flux, cued and terminated at index.
// Returns 0 on success, negative otherwise (see gw_last_error()). The drive
// must be prepared.
int gw_write_flux(uint8_t cyl, uint8_t head, const uint8_t *gw, size_t len);
