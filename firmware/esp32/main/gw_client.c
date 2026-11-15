#include "gw_client.h"
#include "gw_proto.h"
#include "flux_decode.h"
#include "app_config.h"
#include "link.h"
#include "power.h"
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "gw";

static char s_last_err[64] = "";
static void gw_set_err(const char *msg) { strlcpy(s_last_err, msg, sizeof(s_last_err)); }
const char *gw_last_error(void) { return s_last_err; }

static uint32_t s_sample_freq;
static uint8_t  s_flux_status = 0xFF;   // last GET_FLUX_STATUS; 0xFF = none
static int      s_last_ack;             // ack of the last failed cmd_ok

uint8_t gw_last_flux_status(void) { return s_flux_status; }

// ---- low-level framing ----

// [cmd, total_len, payload...], total_len including the header. One
// link_write per frame, since each write waits for an ack.
static void send_cmd(uint8_t cmd, const uint8_t *payload, uint8_t plen)
{
    uint8_t f[2 + 16];
    if (plen > sizeof f - 2) plen = sizeof f - 2;
    f[0] = cmd;
    f[1] = (uint8_t)(plen + 2);
    if (plen) memcpy(&f[2], payload, plen);
    link_write(f, (size_t)plen + 2);
}

// Returns the ack byte of a [cmd, ack] reply, or -1 on timeout or mismatch.
static int read_ack(uint8_t expect_cmd, int to_ms)
{
    uint8_t r[2];
    int n = link_read(r, 2, to_ms);
    if (n != 2) { ESP_LOGW(TAG, "ack timeout cmd=%u", (unsigned)expect_cmd); return -1; }
    if (r[0] != expect_cmd) { ESP_LOGW(TAG, "ack cmd %u!=%u", (unsigned)r[0], (unsigned)expect_cmd); return -1; }
    return r[1];
}

static bool cmd_ok(uint8_t cmd, const uint8_t *payload, uint8_t plen, int to_ms)
{
    send_cmd(cmd, payload, plen);
    int ack = read_ack(cmd, to_ms);
    if (ack != ACK_OKAY) {
        s_last_ack = ack;
        ESP_LOGW(TAG, "cmd %u ack=%d", (unsigned)cmd, ack);
        snprintf(s_last_err, sizeof(s_last_err),
                 ack < 0 ? "cmd %u no ack (link/timeout)" : "cmd %u nack=%d",
                 (unsigned)cmd, ack);
        return false;
    }
    return true;
}

// ---- controller ownership ----
// The SWD reflash and gw_task's relink both pulse RUN, so neither may land
// inside the other or inside a drive session. Each holds this throughout.
static SemaphoreHandle_t s_owner;

bool gw_claim(uint32_t wait_ms)
{
    return xSemaphoreTake(s_owner, wait_ms == UINT32_MAX ? portMAX_DELAY
                                                         : pdMS_TO_TICKS(wait_ms)) == pdTRUE;
}

void gw_release(void) { xSemaphoreGive(s_owner); }

esp_err_t gw_client_init(void)
{
    s_owner = xSemaphoreCreateMutex();
    if (!s_owner) return ESP_ERR_NO_MEM;

    link_peer_reset();
    esp_err_t e = link_up();
    if (e != ESP_OK) {
        // Non-fatal: the read path relinks on demand.
        ESP_LOGW(TAG, "SPI link not up (%s) — gw_relink will retry on demand",
                 esp_err_to_name(e));
        gw_set_err("SPI link down at boot (peer not answering SYNC)");
        return ESP_OK;
    }
    ESP_LOGI(TAG, "GWSPI link up: sclk=%d mosi=%d miso=%d cs=%d drdy=%d @ %d Hz",
             GW_SPI_SCLK_GPIO, GW_SPI_MOSI_GPIO, GW_SPI_MISO_GPIO,
             GW_SPI_CS_GPIO, GW_SPI_DRDY_GPIO, GW_SPI_CLK_HZ);
    return ESP_OK;
}

static void idle_poll_rearm(void);

bool gw_relink(void)
{
    link_down();
    link_peer_reset();
    // The peer may come back as a different build (one with GetInfo 0x89).
    idle_poll_rearm();
    if (link_up() != ESP_OK) return false;
    ESP_LOGW(TAG, "GW relinked via reset line");
    return true;
}

// ---- handshake ----

bool gw_handshake(void)
{
    uint8_t sub = GETINFO_FIRMWARE;
    send_cmd(CMD_GET_INFO, &sub, 1);
    if (read_ack(CMD_GET_INFO, 500) != ACK_OKAY) {
        gw_set_err("no GetInfo ack (link down: wiring/power/link?)");
        return false;
    }

    struct gw_info_firmware info;
    int n = link_read((uint8_t *)&info, sizeof(info), 500);
    if (n != sizeof(info)) {
        ESP_LOGW(TAG, "GetInfo short read %d", n);
        snprintf(s_last_err, sizeof(s_last_err),
                 "GetInfo short read (%d/%u bytes)", n, (unsigned)sizeof(info));
        return false;
    }
    if (!info.is_main_firmware) {
        ESP_LOGE(TAG, "peer in bootloader/not ready!");
        gw_set_err("peer in bootloader/not ready (reset it?)");
        return false;
    }

    s_sample_freq = info.sample_freq;
    // u_buf must hold a whole write track (buffered write).
    ESP_LOGI(TAG, "GW fw %u.%u, sample_freq=%u Hz, max_cmd=%u, mcu_id=%u %u MHz, sram=%u KB, u_buf=%u KB",
             (unsigned)info.fw_ver_major, (unsigned)info.fw_ver_minor,
             (unsigned)s_sample_freq, (unsigned)info.max_cmd,
             (unsigned)info.mcu_id, (unsigned)info.mcu_mhz,
             (unsigned)info.mcu_sram_kb, (unsigned)info.usb_buf_kb);
    if (s_sample_freq == 0) {
        gw_set_err("sample_freq=0 (GW not fully booted)");
        return false;
    }
    // See GW_FLUX_RAW_MAX in gw_client.h.
    if ((uint32_t)info.usb_buf_kb * 1024u > GW_FLUX_RAW_MAX) {
        ESP_LOGE(TAG, "controller staging buffer %u KB exceeds our raw buffer "
                      "%u KB — raise GW_FLUX_RAW_MAX",
                 (unsigned)info.usb_buf_kb, (unsigned)(GW_FLUX_RAW_MAX / 1024));
    }
    return true;
}

uint32_t gw_sample_freq(void) { return s_sample_freq; }

// ---- getting the command pipe back in step ----
// The protocol has no framing, so a late reply would be read as the next
// command's. Drain until quiet, then prove alignment with GetInfo.
bool gw_resync(void)
{
    uint8_t junk[256];
    int64_t dl = esp_timer_get_time() + 2000000;  // a late reply gets 2 s to land
    int quiet_ms = 0;
    while (quiet_ms < 200 && esp_timer_get_time() < dl) {
        int n = link_read(junk, sizeof junk, 50);
        if (n < 0) return false;
        quiet_ms = n ? 0 : quiet_ms + 50;
    }
    link_flush_rx();
    return link_is_up() && gw_handshake();
}

// ---- controller diagnostics (GetInfo 0x88) ----
// Sampled only by the link-owning task; /api/status serves the cached copy.
static struct gw_diag_counters s_diag;
static bool s_diag_valid;
static int64_t s_diag_us;

bool gw_refresh_diag(void)
{
    uint8_t sub = GETINFO_GW_DIAG;
    send_cmd(CMD_GET_INFO, &sub, 1);
    // Best-effort: never overwrites s_last_err.
    if (read_ack(CMD_GET_INFO, 300) != ACK_OKAY)
        return false;

    struct gw_diag_counters d;
    if (link_read((uint8_t *)&d, sizeof(d), 300) != sizeof(d)) {
        link_flush_rx();
        return false;
    }
    s_diag = d;
    s_diag_valid = true;
    s_diag_us = esp_timer_get_time();
    return true;
}

bool gw_get_diag(struct gw_diag_counters *out, uint32_t *age_ms)
{
    if (!s_diag_valid)
        return false;
    if (out) *out = s_diag;
    if (age_ms) *age_ms = (uint32_t)((esp_timer_get_time() - s_diag_us) / 1000);
    return true;
}

// ---- controller health (GetInfo 0x89) ----
// Same ownership rule as diag, but also polled while idle.
static struct gw_health s_health;
static bool s_health_valid;
static int64_t s_health_us;

// A nack means old firmware (healthy); silence means a wedged link.
typedef enum {
    HEALTH_OK,          // answered with a health block
    HEALTH_UNSUPPORTED, // answered, but nacked the sub-command
    HEALTH_NO_ANSWER,   // no ack, wrong ack, or a short payload: link suspect
} health_result_t;

static health_result_t refresh_health(void)
{
    uint8_t sub = GETINFO_GW_HEALTH;
    send_cmd(CMD_GET_INFO, &sub, 1);
    int ack = read_ack(CMD_GET_INFO, 300);
    if (ack < 0)
        return HEALTH_NO_ANSWER;
    if (ack != ACK_OKAY)
        return HEALTH_UNSUPPORTED;

    struct gw_health h;
    if (link_read((uint8_t *)&h, sizeof(h), 300) != sizeof(h)) {
        link_flush_rx();
        return HEALTH_NO_ANSWER;
    }
    s_health = h;
    s_health_valid = true;
    s_health_us = esp_timer_get_time();
    return HEALTH_OK;
}

bool gw_get_health(struct gw_health *out, uint32_t *age_ms)
{
    if (!s_health_valid)
        return false;
    if (out) *out = s_health;
    if (age_ms) *age_ms = (uint32_t)((esp_timer_get_time() - s_health_us) / 1000);
    return true;
}

// ---- idle keepalive / link self-heal ----
// A controller reset (e.g. SWD reflash) desyncs the link. The read path
// relinks on demand; this gives the idle path the same recovery. Relinking
// pulses RUN, so call only between drive sessions.

// ~30 s of silence at GW_IDLE_POLL_MS before relinking.
#define HEALTH_FAILS_BEFORE_RELINK 3

// A failed relink costs ~3.3 s; back off geometrically (to ~16 min) so an
// absent controller isn't held in a reset loop.
#define RELINK_BACKOFF_MAX 32

static int  s_health_fails;
static int  s_relink_scale = 1;
static bool s_health_unsupported;

// Leaves s_relink_scale and s_health_fails alone: gw_idle_poll() reaches this
// via gw_relink() and manages both itself.
static void idle_poll_rearm(void)
{
    s_health_unsupported = false;
}

// ---- USB host on the controller ----
// The RP reports a USB host (gw.exe on its own USB-C) in its link headers.
// While set, keep the drive rail up and stay off the link: the RP serves only
// the owning transport, so our commands would time out and eventually trigger
// a relink that kills the USB session.
//
// Clear after 2 s of explicit "no USB", or 5 s with no valid header (headers
// are invalid for a few hundred ms during a flux op; an absent RP must not
// hold the rail up forever).
#define USB_CLEAR_MS    2000
#define USB_SILENT_MS   5000

static bool    s_usb_active;
static int64_t s_usb_seen_us, s_usb_answer_us;

bool gw_usb_active(void) { return s_usb_active; }

void gw_usb_watch(void)
{
    int64_t now = esp_timer_get_time();
    int u = link_probe_peer_usb();
    if (u >= 0) s_usb_answer_us = now;
    if (u == 1) {
        s_usb_seen_us = now;
        if (!s_usb_active) {
            s_usb_active = true;
            power_boost_usb(true);
            ESP_LOGW(TAG, "USB host on the controller: drive rail up, link commands paused");
        }
        return;
    }
    if (!s_usb_active) return;
    bool cleared = (u == 0 && now - s_usb_seen_us > (int64_t)USB_CLEAR_MS * 1000) ||
                   (now - s_usb_answer_us > (int64_t)USB_SILENT_MS * 1000);
    if (cleared) {
        s_usb_active = false;
        power_boost_usb(false);
        s_health_fails = 0;
        ESP_LOGW(TAG, "USB host gone: drive rail down, link commands resumed");
    }
}

void gw_idle_poll(void)
{
    // Old firmware without 0x89: no idle liveness check.
    if (s_health_unsupported)
        return;

    // USB host, or a manual 5 V hold (the fallback for controllers too old to
    // report USB): stay off the link.
    if (s_usb_active || power_boost_hold_left()) {
        s_health_fails = 0;
        return;
    }

    switch (refresh_health()) {
    case HEALTH_OK:
        s_health_fails = 0;
        s_relink_scale = 1;
        return;

    case HEALTH_UNSUPPORTED:
        ESP_LOGI(TAG, "controller nacks GetInfo 0x89 (pre-health firmware) — "
                      "idle polling disabled");
        s_health_unsupported = true;
        s_health_fails = 0;
        return;

    case HEALTH_NO_ANSWER:
        break;
    }

    if (++s_health_fails < HEALTH_FAILS_BEFORE_RELINK * s_relink_scale)
        return;
    s_health_fails = 0;

    ESP_LOGW(TAG, "controller silent for %d polls — relinking",
             HEALTH_FAILS_BEFORE_RELINK * s_relink_scale);

    // The handshake proves the command layer works and re-latches sample_freq.
    if (gw_relink() && gw_handshake()) {
        ESP_LOGI(TAG, "link recovered on idle poll");
        s_relink_scale = 1;
    } else if (s_relink_scale < RELINK_BACKOFF_MAX) {
        s_relink_scale *= 2;
    }
}

// ---- drive control ----

bool gw_drive_prepare(void)
{
    power_boost_5v(true);
    vTaskDelay(pdMS_TO_TICKS(10));        // boost settle

    uint8_t bus = GW_BUS_SHUGART;
    if (!cmd_ok(CMD_SET_BUS_TYPE, &bus, 1, 200)) { power_boost_5v(false); return false; }

    uint8_t unit = GW_DRIVE_UNIT;
    if (!cmd_ok(CMD_SELECT, &unit, 1, 200)) { power_boost_5v(false); return false; }

    uint8_t motor[2] = { GW_DRIVE_UNIT, 1 };   // {unit, on}
    // The controller waits motor_delay (750 ms default) before acking.
    if (!cmd_ok(CMD_MOTOR, motor, 2, 1500)) { gw_drive_release(); return false; }

    vTaskDelay(pdMS_TO_TICKS(500));            // motor spin-up settle
    return true;
}

void gw_drive_release(void)
{
    uint8_t motor[2] = { GW_DRIVE_UNIT, 0 };
    send_cmd(CMD_MOTOR, motor, 2);   (void)read_ack(CMD_MOTOR, 200);
    send_cmd(CMD_DESELECT, NULL, 0); (void)read_ack(CMD_DESELECT, 200);
    (void)gw_refresh_diag();         // safe point: we own the link and it's idle
    power_boost_5v(false);
}

// ---- flux read ----

// Read a 0x00-terminated flux stream. Returns bytes including the terminator,
// or negative on stall/overflow. Reads in short slices so it can stop as soon
// as the terminator arrives.
static int read_flux_stream(uint8_t *buf, size_t cap, int64_t *t_first)
{
    size_t len = 0;
    int64_t last_rx = esp_timer_get_time();
    while (len < cap) {
        size_t want = cap - len;
        if (want > 4096) want = 4096;
        int n = link_read(buf + len, want, 20);
        if (n < 0) return -1;
        if (n == 0) {
            if (esp_timer_get_time() - last_rx > 1000000) return -1; // 1 s stall
            continue;
        }
        if (t_first && len == 0) *t_first = esp_timer_get_time();
        last_rx = esp_timer_get_time();
        for (int i = 0; i < n; i++) {
            if (buf[len + i] == 0) return (int)(len + i + 1);
        }
        len += n;
    }
    return -2;                                 // overflow (see GW_FLUX_RAW_MAX)
}

int gw_read_flux_raw(uint8_t cyl, uint8_t head, uint8_t *raw, size_t raw_cap,
                     size_t *raw_len, gw_read_timing_t *t)
{
    if (t) *t = (gw_read_timing_t){0};
    s_flux_status = 0xFF;
    int64_t ph = esp_timer_get_time();

    uint8_t c = cyl;
    // SEEK acks after the move. A cold seek recalibrates (<= 300 ms) and then
    // steps at 10 ms/cyl: ~1.15 s worst case.
    if (!cmd_ok(CMD_SEEK, &c, 1, 2000)) {
        if (s_last_ack != ACK_NO_TRK0) return -1;
        gw_set_err("no drive? the controller never saw track 0");
        return GW_RD_NO_DRIVE;
    }
    uint8_t h = head;
    if (!cmd_ok(CMD_HEAD, &h, 1, 200)) return -2;
    if (t) t->seek_us = (uint32_t)(esp_timer_get_time() - ph);

    // ReadFlux {u32 ticks, u16 max_index, u32 linger}: a tick-bounded capture
    // of GW_READ_WINDOW_MS.
    uint32_t rev_ticks = (uint32_t)((uint64_t)s_sample_freq * GW_READ_WINDOW_MS / 1000);
    uint8_t p[10];
    memcpy(&p[0], &rev_ticks, 4);
    uint16_t max_index = 0; memcpy(&p[4], &max_index, 2);
    uint32_t linger = 0;    memcpy(&p[6], &linger, 4);

    ph = esp_timer_get_time();
    send_cmd(CMD_READ_FLUX, p, sizeof(p));
    if (read_ack(CMD_READ_FLUX, 500) != ACK_OKAY) return -3;

    int64_t t_ack = esp_timer_get_time();
    int64_t t_first = 0;
    int n = read_flux_stream(raw, raw_cap, &t_first);
    if (t) {
        t->readxfer_us = (uint32_t)(esp_timer_get_time() - ph);
        t->ttfb_us = (t_first > 0) ? (uint32_t)(t_first - ph) : 0;
        t->ackwait_us = (uint32_t)(t_ack - ph);
    }
    if (n < 0) { ESP_LOGW(TAG, "flux read err %d", n); return -4; }
    *raw_len = (size_t)n;

    ph = esp_timer_get_time();
    send_cmd(CMD_GET_FLUX_STATUS, NULL, 0);
    int st = read_ack(CMD_GET_FLUX_STATUS, 500);
    if (t) t->status_us = (uint32_t)(esp_timer_get_time() - ph);
    if (st < 0) {
        gw_set_err("GetFluxStatus no reply");
        return -5;
    }
    s_flux_status = (uint8_t)st;
    if (st != ACK_OKAY)
        // Non-fatal: the caller decodes what arrived and checks
        // gw_last_flux_status() to spot "no disk".
        ESP_LOGW(TAG, "flux status %d (c%u h%u)", st, (unsigned)cyl, (unsigned)head);

    return 0;
}

// ---- flux write (Stage B) ----

int gw_write_flux(uint8_t cyl, uint8_t head, const uint8_t *gw, size_t len)
{
    if (!len || gw[len - 1] != 0) { gw_set_err("write flux not 0-terminated"); return -1; }

    uint8_t c = cyl;
    if (!cmd_ok(CMD_SEEK, &c, 1, 2000)) return -2;   // see gw_read_flux_raw
    uint8_t h = head;
    if (!cmd_ok(CMD_HEAD, &h, 1, 200)) return -3;

    // {cue_at_index, terminate_at_index}. The track is encoded longer than one
    // revolution and cut at the next index, so the splice lands in gap flux.
    uint8_t wf[2] = { 1, 1 };
    send_cmd(CMD_WRITE_FLUX, wf, sizeof(wf));
    int ack = read_ack(CMD_WRITE_FLUX, 500);
    if (ack != ACK_OKAY) {
        if (ack == ACK_WRPROT)  gw_set_err("disk is write-protected");
        else if (ack < 0)       gw_set_err("WriteFlux no ack (link/timeout)");
        else snprintf(s_last_err, sizeof(s_last_err), "WriteFlux nack=%d", ack);
        return -4;
    }

    // One burst: the controller buffers the whole stream before writing.
    int w = link_write(gw, len);
    if (w != (int)len) { gw_set_err("WriteFlux short link write"); return -5; }
    link_wait_tx_done(1000);

    // One status byte when the write completes (after up to a revolution's cue).
    uint8_t st = 0xFF;
    int n = link_read(&st, 1, 3000);
    if (n != 1) { gw_set_err("WriteFlux no status byte (timeout)"); return -6; }
    if (st != ACK_OKAY) {
        snprintf(s_last_err, sizeof(s_last_err), "WriteFlux status=%u (c%u h%u)",
                 (unsigned)st, (unsigned)cyl, (unsigned)head);
        return -7;
    }
    return 0;
}
