// Application entry: subsystem bring-up and the gw task (disk read, write,
// save) (architecture doc §3, §4).
#include "app_config.h"
#include "disk_image.h"
#include "gw_client.h"
#include "mfm_decode.h"
#include "mfm_encode.h"
#include "flux_decode.h"
#include "fat_scan.h"
#include "fat12_diskio.h"
#include "provision.h"
#include "web.h"
#include "ui.h"
#include "wifi_pwr.h"
#include "storage.h"
#include "sd_index.h"
#include "display.h"
#include "leds.h"
#include "ota.h"
#include "power.h"
#include "wallclock.h"
#include "serialno.h"
#include "temps.h"
#include "ff.h"
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_system.h"
#include "esp_random.h"
#include "esp_netif_sntp.h"
#include <time.h>

static const char *TAG = "app";

// Per-track flux intervals. A 220 ms window at the 2 us minimum HD interval
// is 110k transitions; the margin covers a pathological track.
#define FLUX_TICKS_MAX 150000
static uint32_t *s_ticks;

// Write scratch (PSRAM): one track's MFM cells and its GW flux stream. A
// >1-revolution track is ~205k cells / ~100 KB of flux.
#define WCELLS_CAP 300000
#define WGW_CAP    200000
static uint8_t *s_wcells;
static uint8_t *s_wgw[2];         // [1]: the pipelined test's second encode buffer
static uint8_t  s_wsect[DISK_SECTORS][SECTOR_SIZE];

#if GW_PROFILE
// Per-dump phase timing in microseconds. Reading and decoding overlap in pass
// 1, so the report shows saved = sum - wall.
typedef struct {
    uint64_t seek_us, readxfer_us, ttfb_us, ackwait_us, status_us, decode_us, fluxdec_us, rawbytes;
    uint32_t tracks, attempts;
} dump_stats_t;
static dump_stats_t s_stats;
extern uint64_t g_mfm_pll_us;   // mfm_decode.c
#endif

// ---- the two-core pipeline: the gw task (core 1) drives the drive while
// pipe_task on core 0 decodes (and, for the disk test, fills and encodes).
// Buffers change hands only through the queues. ----
static uint8_t  *s_raw[2];        // ping-pong raw flux buffers (PSRAM)
static uint32_t *s_index;         // flux_decode index scratch (unused downstream)
#define IDX_MAX 8

enum { PK_END, PK_DEC, PK_ENC };
typedef struct {
    int8_t   kind;                // PK_*
    int8_t   slot;                // PK_DEC: s_raw buffer; PK_ENC: s_wgw buffer
    uint8_t  cyl, head;
    uint8_t  attempt;             // health check: which of the track's reads
    uint8_t  fstat;               // PK_DEC: that read's GET_FLUX_STATUS (0xFF = none)
    int      rc;                  // PK_DEC: gw_read_flux_raw() result (0 = ok);
                                  // PK_ENC (reply): encode result (0 = ok)
    size_t   raw_len;             // PK_DEC: flux bytes read; PK_ENC (reply): to write
} pipe_item_t;
static QueueHandle_t     s_q_filled;   // gw task -> pipe task (pipe_item_t)
static QueueHandle_t     s_q_empty;    // pipe task -> gw task (int raw slot)
static QueueHandle_t     s_q_enc;      // pipe task -> gw task (encoded PK_ENC items)
static SemaphoreHandle_t s_decode_done;
static void (*s_pipe_fn)(pipe_item_t *it);   // the job's per-item work (core 0)
static mfm_track_ctx_t   s_ctx;        // shared read-only after init (sample_freq)

// Tracks still bad after pass 1, for the pass-2 retry. Written by the decode
// task, read only after it has joined.
static struct { uint8_t cyl, head; } s_retry[TOTAL_TRACKS];
static int s_retry_n;
static bool s_no_drive;          // a SEEK returned GW_RD_NO_DRIVE

// Consecutive link-failed tracks before aborting, so a wedged link never
// leaves the drive spinning.
#define LINK_FAIL_ABORT 8

// ---------------- WiFi station ----------------
static EventGroupHandle_t s_wifi_eg;
#define WIFI_GOT_IP BIT0
static int s_auth_fails;                 // consecutive auth/handshake failures

// ---- network time ----
// FatFs stamps saved files with time(NULL). SNTP corrects the RTC-seeded
// clock and writes it back to the RTC.
//
// SNTP starts only once we have an address: started before DHCP it fails DNS
// and waits an hour to retry. sntp_register() must run after wifi_start()
// (which creates the event loop), but wifi_start() usually waits for GOT_IP,
// so start immediately if already connected and let the handler cover later
// reconnects.
static bool s_sntp_ready, s_sntp_running;

static void time_synced(struct timeval *tv)
{
    (void)tv;
    wallclock_store();
}

static void sntp_register(void)
{
    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    cfg.start                     = false;
    cfg.server_from_dhcp          = true;
    cfg.renew_servers_after_new_IP = true;
    cfg.sync_cb                   = time_synced;
    esp_err_t e = esp_netif_sntp_init(&cfg);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "SNTP init failed: %s", esp_err_to_name(e));
        return;
    }
    s_sntp_ready = true;

    if (s_wifi_eg && (xEventGroupGetBits(s_wifi_eg) & WIFI_GOT_IP) &&
        !s_sntp_running && esp_netif_sntp_start() == ESP_OK)
        s_sntp_running = true;
}

static void wifi_evt(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *d = data;
        int reason = d ? d->reason : -1;
        // Repeated auth failures mean a wrong password: clear the creds and
        // reboot into the setup portal. "AP not found" just keeps retrying.
        if (reason == WIFI_REASON_AUTH_FAIL ||
            reason == WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT ||
            reason == WIFI_REASON_HANDSHAKE_TIMEOUT ||
            reason == WIFI_REASON_AUTH_EXPIRE) {
            if (++s_auth_fails >= 5) {
                ESP_LOGE(TAG, "WiFi auth keeps failing (reason %d) — clearing "
                         "creds, rebooting to setup portal", reason);
                provision_clear_creds();
                vTaskDelay(pdMS_TO_TICKS(300));
                esp_restart();
            }
        }
        if (wifi_pwr_radio_paused()) return;   // stopped on purpose for a drive session
        ESP_LOGW(TAG, "wifi disconnected (reason %d), retrying", reason);
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        s_auth_fails = 0;
        xEventGroupSetBits(s_wifi_eg, WIFI_GOT_IP);
        if (s_sntp_ready && !s_sntp_running && esp_netif_sntp_start() == ESP_OK)
            s_sntp_running = true;
    }
}

// WiFi creds from NVS ("wifi"/"ssid","pass"), else the build defaults.
// Returns true if the SSID came from NVS.
static bool wifi_load_creds(char *ssid, size_t ssid_sz, char *pass, size_t pass_sz)
{
    strlcpy(ssid, WIFI_SSID, ssid_sz);
    strlcpy(pass, WIFI_PASS, pass_sz);
    nvs_handle_t h;
    if (nvs_open("wifi", NVS_READONLY, &h) != ESP_OK)
        return false;
    size_t n = ssid_sz;
    bool from_nvs = (nvs_get_str(h, "ssid", ssid, &n) == ESP_OK);
    n = pass_sz;
    nvs_get_str(h, "pass", pass, &n);          // absent = open network
    nvs_close(h);
    return from_nvs;
}

static void wifi_start(void)
{
    s_wifi_eg = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&ic));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_evt, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_evt, NULL));

    char ssid[33] = { 0 }, pass[65] = { 0 };
    bool from_nvs = wifi_load_creds(ssid, sizeof(ssid), pass, sizeof(pass));

    wifi_config_t wc = { 0 };
    strlcpy((char *)wc.sta.ssid, ssid, sizeof(wc.sta.ssid));
    strlcpy((char *)wc.sta.password, pass, sizeof(wc.sta.password));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    wifi_pwr_sta_ready();

    ESP_LOGI(TAG, "connecting to SSID '%s' (creds from %s)...",
             ssid, from_nvs ? "NVS" : "build default");
    // Bounded wait: the read path doesn't need WiFi, and the handler keeps
    // retrying in the background.
    EventBits_t bits = xEventGroupWaitBits(s_wifi_eg, WIFI_GOT_IP, pdFALSE, pdTRUE,
                                           pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));
    if (bits & WIFI_GOT_IP)
        ESP_LOGI(TAG, "wifi connected");
    else
        ESP_LOGW(TAG, "wifi not connected after %d ms; continuing (read path is "
                 "offline-capable, web UI will attach on connect)",
                 WIFI_CONNECT_TIMEOUT_MS);
}

// ---------------- whole-disk read ----------------

// Decode one raw track into the disk image. s_ticks/s_index are shared, but the
// synchronous paths and pipe_task never run at the same time. (The test's
// encoder uses s_ticks too, but it runs in pipe_task, between decodes.)
static void decode_and_store(const uint8_t *raw, size_t raw_len,
                             const mfm_track_ctx_t *ctx, mfm_result_t *res)
{
#if GW_PROFILE
    int64_t d0 = esp_timer_get_time();
#endif
    *res = (mfm_result_t){ .sectors_missing = DISK_SECTORS };
    flux_stream_t fs = { .ticks = s_ticks, .ticks_cap = FLUX_TICKS_MAX,
                         .index = s_index, .index_cap = IDX_MAX };
    int fd = flux_decode(raw, raw_len, &fs);
#if GW_PROFILE
    s_stats.fluxdec_us += (uint64_t)(esp_timer_get_time() - d0);
#endif
    if (fd == 0)
        mfm_decode_track(s_ticks, fs.ticks_len, ctx, res);
#if GW_PROFILE
    s_stats.decode_us += (uint64_t)(esp_timer_get_time() - d0);
#endif
}

// After a protocol failure, get the command pipe back in step (a late reply
// would otherwise be read as the next command's). Try resync first; failing
// that, reset the controller and re-prepare the drive. False only if the
// controller can't be brought back.
static bool recover_link(void)
{
    if (gw_resync()) return true;
    ESP_LOGW(TAG, "command pipe out of step — resetting the controller");
    if (!gw_relink() || !gw_handshake()) return false;
    return gw_drive_prepare();
}

// The last read attempt that came back deficient, for /api/linkprof.
char g_last_bad_read[128];

// Longest gap between flux transitions in a raw Greaseweazle stream, and
// where it starts, in microseconds from the start of the capture.
static void longest_gap(const uint8_t *s, size_t n, uint32_t *gap_us, uint32_t *at_us)
{
    uint64_t t = 0, best = 0, best_at = 0, pending = 0;
    const uint64_t f = gw_sample_freq() ? gw_sample_freq() : 25000000;
    for (size_t i = 0; i < n && s[i]; ) {
        uint64_t v;
        if (s[i] < 250) {
            v = pending + s[i]; i += 1;
        } else if (s[i] < 255) {
            if (i + 1 >= n) break;
            v = pending + 250 + (uint32_t)(s[i] - 250) * 255 + s[i + 1] - 1; i += 2;
        } else {
            // 0xFF, opcode, u28 (7 bits per byte). FLUXOP_SPACE (2) extends
            // the next interval; FLUXOP_INDEX (1) is only a marker.
            if (i + 5 >= n) break;
            if (s[i + 1] == 2)
                pending += (uint32_t)((s[i + 2] & 0xFE) >> 1) | ((uint32_t)(s[i + 3] & 0xFE) << 6) |
                           ((uint32_t)(s[i + 4] & 0xFE) << 13) | ((uint32_t)(s[i + 5] & 0xFE) << 20);
            i += 6;
            continue;
        }
        pending = 0;
        if (v > best) { best = v; best_at = t; }
        t += v;
    }
    *gap_us = (uint32_t)(best * 1000000 / f);
    *at_us  = (uint32_t)(best_at * 1000000 / f);
}

// fstat: the read's own flux status. The pipelined jobs pass the one the gw
// task stored with the item: by the time core 0 decodes, gw_last_flux_status()
// already belongs to a later read.
static void note_bad_read(uint8_t cyl, uint8_t head, int attempt, int fstat,
                          const uint8_t *raw, size_t raw_len, const mfm_result_t *r)
{
    uint32_t gap, at;
    longest_gap(raw, raw_len, &gap, &at);
    snprintf(g_last_bad_read, sizeof g_last_bad_read,
             "c%02uh%u try %d: %d ok %d bad %d missing, st %d, %u B, gap %u us at %u us",
             (unsigned)cyl, (unsigned)head, attempt, r->sectors_ok, r->sectors_bad,
             r->sectors_missing, fstat, (unsigned)raw_len,
             (unsigned)gap, (unsigned)at);
}

// Best-of-TRACK_RETRIES read+decode, for cyl 0 and the pass-2 retry. Returns
// true if the link responded. *no_flux (optional) is set if every read came
// back ACK_NO_INDEX (no disk, or not spinning).
static bool read_track_sync(uint8_t cyl, uint8_t head, const mfm_track_ctx_t *ctx,
                            bool is_retry, bool *no_flux)
{
    mfm_track_ctx_t c = *ctx; c.exp_cyl = cyl; c.exp_head = head;
    mfm_result_t best = { .sectors_missing = DISK_SECTORS };
    bool link_ok = false;
    bool all_no_flux = true;
    int  clean_attempt = -1;

    for (int attempt = 0; attempt < TRACK_RETRIES; attempt++) {
        size_t raw_len = 0;
#if GW_PROFILE
        gw_read_timing_t t;
        int rc = gw_read_flux_raw(cyl, head, s_raw[0], GW_FLUX_RAW_MAX, &raw_len, &t);
        s_stats.attempts++;
        s_stats.seek_us += t.seek_us; s_stats.readxfer_us += t.readxfer_us;
        s_stats.status_us += t.status_us; s_stats.rawbytes += raw_len;
        s_stats.ttfb_us += t.ttfb_us; s_stats.ackwait_us += t.ackwait_us;
#else
        int rc = gw_read_flux_raw(cyl, head, s_raw[0], GW_FLUX_RAW_MAX, &raw_len, NULL);
#endif
        if (rc == GW_RD_NO_DRIVE) {
            s_no_drive = true;
            break;
        }
        if (rc != 0) {
            if (!recover_link()) break;
            continue;
        }
        link_ok = true;
        if (gw_last_flux_status() != ACK_NO_INDEX) all_no_flux = false;
        mfm_result_t r;
        decode_and_store(s_raw[0], raw_len, &c, &r);
        if (r.sectors_missing < best.sectors_missing) best = r;
        if (r.sectors_missing == 0 && r.sectors_bad == 0) { clean_attempt = attempt; break; }
        note_bad_read(cyl, head, attempt, gw_last_flux_status(), s_raw[0], raw_len, &r);
    }
#if GW_PROFILE
    s_stats.tracks++;
#endif
    // Disk map: clean first time = GOOD, clean after a retry (or in pass 2) =
    // RETRY, never clean = BAD.
    disk_trackmap_set(cyl, head,
        (clean_attempt >= 0) ? ((is_retry || clean_attempt > 0) ? TRK_RETRY : TRK_GOOD)
                             : TRK_BAD);
    if (link_ok && (best.sectors_missing || best.sectors_bad))
        ESP_LOGW(TAG, "c%02u h%u: %d ok, %d bad, %d missing", (unsigned)cyl,
                 (unsigned)head, best.sectors_ok, best.sectors_bad, best.sectors_missing);
    if (no_flux) *no_flux = link_ok && all_no_flux;
    return link_ok;
}

// Pass-1 decoder for the whole-disk read. Records deficient tracks for pass 2.
static void read_pipe_fn(pipe_item_t *it)
{
    bool deficient = true;
    if (it->rc == 0) {
        mfm_track_ctx_t c = s_ctx; c.exp_cyl = it->cyl; c.exp_head = it->head;
        mfm_result_t r;
        decode_and_store(s_raw[it->slot], it->raw_len, &c, &r);
        deficient = (r.sectors_missing || r.sectors_bad);
        if (deficient)
            note_bad_read(it->cyl, it->head, 0, it->fstat, s_raw[it->slot], it->raw_len, &r);
        if (deficient)
            ESP_LOGW(TAG, "c%02u h%u: %d ok, %d bad, %d missing", (unsigned)it->cyl,
                     (unsigned)it->head, r.sectors_ok, r.sectors_bad, r.sectors_missing);
    }
    // Provisionally BAD; pass 2 upgrades it to RETRY if it recovers.
    disk_trackmap_set(it->cyl, it->head, deficient ? TRK_BAD : TRK_GOOD);
    if (deficient && s_retry_n < TOTAL_TRACKS) {
        s_retry[s_retry_n].cyl  = it->cyl;
        s_retry[s_retry_n].head = it->head;
        s_retry_n++;
    }
}

// Core 0: run s_pipe_fn on each item in order; hand raw buffers back, and
// encoded tracks to the gw task.
static void pipe_task(void *arg)
{
    (void)arg;
    pipe_item_t it;
    for (;;) {
        xQueueReceive(s_q_filled, &it, portMAX_DELAY);
        if (it.kind == PK_END) break;
        s_pipe_fn(&it);
        if (it.kind == PK_DEC) {
            int slot = it.slot;
            xQueueSend(s_q_empty, &slot, portMAX_DELAY);
        } else {
            xQueueSend(s_q_enc, &it, portMAX_DELAY);
        }
    }
    xSemaphoreGive(s_decode_done);
    vTaskDelete(NULL);
}

static void pipe_free(void)
{
    if (s_q_empty)     vQueueDelete(s_q_empty);
    if (s_q_filled)    vQueueDelete(s_q_filled);
    if (s_q_enc)       vQueueDelete(s_q_enc);
    if (s_decode_done) vSemaphoreDelete(s_decode_done);
    s_q_empty = NULL; s_q_filled = NULL; s_q_enc = NULL; s_decode_done = NULL;
}

// Start pipe_task with `fn`. False (nothing running) if out of memory: the
// caller must not queue anything then, or it would block forever, motor on.
static bool pipe_start(void (*fn)(pipe_item_t *it))
{
    s_pipe_fn     = fn;
    s_q_empty     = xQueueCreate(4, sizeof(int));
    s_q_filled    = xQueueCreate(8, sizeof(pipe_item_t));
    s_q_enc       = xQueueCreate(4, sizeof(pipe_item_t));   // at most 2 in flight
    s_decode_done = xSemaphoreCreateBinary();
    if (s_q_empty && s_q_filled && s_q_enc && s_decode_done &&
        xTaskCreatePinnedToCore(pipe_task, "decode", 8192, NULL, 4, NULL, 0) == pdPASS) {
        for (int i = 0; i < 2; i++) { int sl = i; xQueueSend(s_q_empty, &sl, 0); }
        return true;
    }
    pipe_free();
    return false;
}

// Let pipe_task finish what's queued, then stop it.
static void pipe_stop(void)
{
    pipe_item_t end = { .kind = PK_END };
    xQueueSend(s_q_filled, &end, portMAX_DELAY);
    xSemaphoreTake(s_decode_done, portMAX_DELAY);
    pipe_free();
}

static int pipe_take_slot(void)
{
    int slot;
    xQueueReceive(s_q_empty, &slot, portMAX_DELAY);
    return slot;
}

static void pipe_put(const pipe_item_t *it) { xQueueSend(s_q_filled, it, portMAX_DELAY); }

// Handshake (relinking once if needed) and spin the drive up. On failure the
// detail line says why; the caller picks the state.
static bool drive_start(void)
{
    if (!gw_handshake()) {
        if (!gw_relink() || !gw_handshake()) {
            disk_set_detail(gw_last_error());
            return false;
        }
        ESP_LOGI(TAG, "link recovered after GW reset");
    }
    if (!gw_drive_prepare()) {
        disk_set_detail(gw_last_error());
        return false;
    }
    return true;
}

// Mount the image and publish it as READY or READY_DIRTY. False, with the
// state at ERROR, if it doesn't mount.
static bool publish_image(void)
{
    uint32_t bad = disk_bad_sector_count();
    if (!fat12_mount()) {
        disk_set_detail("read done but FAT12 mount failed (unformatted/bad boot sector?)");
        disk_set_state(DISK_ERROR);
        return false;
    }
    // Count files for the LCD now, before READY lets the web task use volume 0.
    // Hash them too, so the first /api/list or SD save doesn't have to on the
    // web task with the volume locked.
    {
        int nf = 0; FF_DIR d; FILINFO fi;
        if (f_opendir(&d, FAT12_ROOT) == FR_OK) {
            while (f_readdir(&d, &fi) == FR_OK && fi.fname[0]) {
                if (fi.fattrib & (AM_DIR | AM_SYS | AM_HID)) continue;
                nf++;
                uint8_t sha[SD_SHA_LEN];
                sd_sha_image_file_cached(fi.fname, (uint32_t)fi.fsize, sha);
            }
            f_closedir(&d);
        }
        display_set_files(nf);
    }
    disk_set_state(bad ? DISK_READY_DIRTY : DISK_READY);
    ESP_LOGI(TAG, "image ready: %u/%u sectors bad",
             (unsigned)bad, (unsigned)TOTAL_SECTORS);
    return true;
}

static void read_whole_disk(void)
{
    ESP_LOGI(TAG, "disk read requested");
    // Leave READY under the volume lock, so no web handler is mid-file when
    // the volume is unmounted.
    disk_volume_lock();
    disk_set_state(DISK_READING);
    sd_sha_image_cache_clear();
    disk_volume_unlock();
    fat12_unmount();
    disk_image_reset();
    disk_trackmap_reset();
    disk_set_detail("");
    display_note_saved(NULL);
    disk_set_state(DISK_READING);
    g_last_bad_read[0] = '\0';

    if (!drive_start()) {
        disk_set_state(DISK_ERROR);
        return;
    }

    s_ctx = (mfm_track_ctx_t){ .sample_freq = gw_sample_freq() };
    s_retry_n = 0;
    s_no_drive = false;
    int consec_link_fail = 0;
#if GW_PROFILE
    memset(&s_stats, 0, sizeof s_stats);
    g_mfm_pll_us = 0;
    int64_t dump_t0 = esp_timer_get_time();
#endif

    // Cylinder 0 first: it holds the FAT the worklist needs, and it is the
    // cheap place to fail fast on a dead link or a missing disk (which would
    // otherwise cost minutes of retries).
    int cyl0_dead = 0, cyl0_noflux = 0;
    for (uint8_t head = 0; head < DISK_HEADS; head++) {
        bool no_flux = false;
        if (!read_track_sync(0, head, &s_ctx, false, &no_flux)) cyl0_dead++;
        else if (no_flux)                                       cyl0_noflux++;
    }
    if (s_no_drive) {
        ESP_LOGE(TAG, "SEEK refused (no TRK0) — no drive?");
        disk_set_detail(gw_last_error());
        gw_drive_release(); disk_set_state(DISK_ERROR); return;
    }
    if (cyl0_dead == DISK_HEADS) {
        ESP_LOGE(TAG, "link dead at c00 — aborting read");
        disk_set_detail("link dead reading cylinder 0 (no good tracks)");
        gw_drive_release(); disk_set_state(DISK_ERROR); return;
    }
    if (cyl0_noflux == DISK_HEADS) {
        ESP_LOGE(TAG, "no flux on cylinder 0 — no disk?");
        disk_set_detail("no disk? the drive returned no flux on cylinder 0");
        gw_drive_release(); disk_set_state(DISK_ERROR); return;
    }

    bool used[TOTAL_TRACKS];
    for (int i = 0; i < TOTAL_TRACKS; i++) used[i] = true;
    if (READ_FAT_AWARE) {
        if (fat_used_tracks(used)) {
            uint32_t n = 0;
            for (int i = 0; i < TOTAL_TRACKS; i++) if (used[i]) n++;
            ESP_LOGI(TAG, "FAT-aware: %u/%u tracks in use — skipping %u free",
                     (unsigned)n, TOTAL_TRACKS, (unsigned)(TOTAL_TRACKS - n));
        } else {
            ESP_LOGW(TAG, "FAT parse failed — falling back to whole-disk read");
            for (int i = 0; i < TOTAL_TRACKS; i++) used[i] = true;
        }
    }

    // Mark skipped tracks' sectors valid (zero) so only in-use tracks count
    // toward completion and bad sectors. Progress leaves them out, or the
    // percentage would jump here by however empty the disk is.
    {
        uint32_t skipped = 0;
        for (int i = 0; i < TOTAL_TRACKS; i++) if (!used[i]) skipped += DISK_SECTORS;
        disk_progress_set_skipped(skipped);
        uint8_t zero_sec[SECTOR_SIZE] = { 0 };
        for (uint8_t cyl = 0; cyl < DISK_CYLS; cyl++)
            for (uint8_t head = 0; head < DISK_HEADS; head++)
                if (!used[(uint32_t)cyl * DISK_HEADS + head])
                    for (uint8_t s = 1; s <= DISK_SECTORS; s++)
                        disk_image_put(disk_lba(cyl, head, s), zero_sec, true);
    }

    // ---- Pass 1: pipelined single read of cyl 1..79 ----
    bool aborted = false, no_pipeline = false, cancelled = false;
    if (pipe_start(read_pipe_fn)) {
        for (uint8_t cyl = 1; cyl < DISK_CYLS && !aborted; cyl++) {
            for (uint8_t head = 0; head < DISK_HEADS; head++) {
                if (!used[(uint32_t)cyl * DISK_HEADS + head]) continue;
                if (ui_cancelled()) { aborted = cancelled = true; break; }
                int slot = pipe_take_slot();
                pipe_item_t it = { .kind = PK_DEC, .slot = (int8_t)slot,
                                   .cyl = cyl, .head = head };
#if GW_PROFILE
                gw_read_timing_t t;
                it.rc = gw_read_flux_raw(cyl, head, s_raw[slot], GW_FLUX_RAW_MAX,
                                         &it.raw_len, &t);
                it.fstat = gw_last_flux_status();
                s_stats.attempts++; s_stats.tracks++;
                s_stats.seek_us += t.seek_us; s_stats.readxfer_us += t.readxfer_us;
                s_stats.status_us += t.status_us; s_stats.rawbytes += it.raw_len;
                s_stats.ttfb_us += t.ttfb_us; s_stats.ackwait_us += t.ackwait_us;
#else
                it.rc = gw_read_flux_raw(cyl, head, s_raw[slot], GW_FLUX_RAW_MAX,
                                         &it.raw_len, NULL);
                it.fstat = gw_last_flux_status();
#endif
                pipe_put(&it);
                // A failed track goes to pass 2 like a bad one.
                if (it.rc == 0) consec_link_fail = 0;
                else if (!recover_link() || ++consec_link_fail >= LINK_FAIL_ABORT)
                    aborted = true;
                if (aborted) break;
            }
        }
        pipe_stop();
    } else {
        aborted = no_pipeline = true;
    }

    if (aborted) {
        const char *why = no_pipeline ? "out of memory starting the read pipeline"
                        : cancelled   ? "read cancelled"
                                      : "link dead mid-read (consecutive track failures)";
        ESP_LOGE(TAG, "%s — aborting", why);
        disk_set_detail(why);
        gw_drive_release(); disk_set_state(DISK_ERROR); return;
    }

    // ---- Pass 2: synchronous retry of tracks still bad ----
    int retry_n = s_retry_n;
    for (int i = 0; i < retry_n && !ui_cancelled(); i++)
        read_track_sync(s_retry[i].cyl, s_retry[i].head, &s_ctx, true, NULL);

    gw_drive_release();

#if GW_PROFILE
    {
        int64_t wall_us = esp_timer_get_time() - dump_t0;
        uint64_t work_us = s_stats.seek_us + s_stats.readxfer_us +
                           s_stats.status_us + s_stats.decode_us;
        int64_t saved_us = (int64_t)work_us - wall_us;
        if (saved_us < 0) saved_us = 0;
        // %u only: nano-printf has no %llu or float. Static because
        // disk_set_detail() keeps the pointer.
        static char prof[256];
        snprintf(prof, sizeof prof,
                 "dump(ms) wall=%u seek=%u xfer=%u(ttfb=%u ack=%u) status=%u decode=%u(flux=%u pll=%u) saved=%u raw=%uKB | %u trk %u att %u retry",
                 (unsigned)(wall_us / 1000),
                 (unsigned)(s_stats.seek_us / 1000),
                 (unsigned)(s_stats.readxfer_us / 1000),
                 (unsigned)(s_stats.ttfb_us / 1000),
                 (unsigned)(s_stats.ackwait_us / 1000),
                 (unsigned)(s_stats.status_us / 1000),
                 (unsigned)(s_stats.decode_us / 1000),
                 (unsigned)(s_stats.fluxdec_us / 1000),
                 (unsigned)(g_mfm_pll_us / 1000),
                 (unsigned)((uint64_t)saved_us / 1000),
                 (unsigned)(s_stats.rawbytes / 1024),
                 (unsigned)s_stats.tracks, (unsigned)s_stats.attempts, (unsigned)retry_n);
        ESP_LOGI(TAG, "%s", prof);
        disk_set_detail(prof);
    }
#endif

    publish_image();
}

// RETRY: go back for the tracks the last read left bad and merge what comes
// out into the same image (disk_image_put() only ever replaces a bad sector).
// Static message buffer: disk_set_detail() keeps it.
static char s_retry_msg[112];
static void retry_bad_tracks(void)
{
    const uint8_t *map = disk_trackmap();
    int n = 0;
    for (int i = 0; i < TOTAL_TRACKS; i++) if (map[i] == TRK_BAD) n++;
    // Read everything if there is no partial image to merge into, or if
    // cylinder 0 was bad: its FAT chose which tracks were read at all.
    bool cyl0_bad = false;
    for (int h = 0; h < DISK_HEADS; h++) if (map[h] == TRK_BAD) cyl0_bad = true;
    if (disk_get_state() != DISK_READY_DIRTY || n == 0 || cyl0_bad) {
        read_whole_disk();
        return;
    }

    ESP_LOGI(TAG, "retrying %d bad track%s", n, n == 1 ? "" : "s");
    uint32_t bad_before = disk_bad_sector_count();
    // As read_whole_disk(): leave READY under the volume lock. File contents
    // may change, so the hashes go too.
    disk_volume_lock();
    disk_set_state(DISK_READING);
    sd_sha_image_cache_clear();
    disk_volume_unlock();
    fat12_unmount();
    disk_set_detail("");
    display_note_saved(NULL);
    disk_progress_override(0);

    bool started = drive_start();
    if (started) {
        s_ctx = (mfm_track_ctx_t){ .sample_freq = gw_sample_freq() };
        s_no_drive = false;
        int done = 0;
        for (int i = 0; i < TOTAL_TRACKS && !ui_cancelled(); i++) {
            if (map[i] != TRK_BAD) continue;
            if (!read_track_sync((uint8_t)(i / DISK_HEADS), (uint8_t)(i % DISK_HEADS),
                                 &s_ctx, true, NULL))
                break;                              // link dead or no drive
            disk_progress_override(++done * TOTAL_SECTORS / n);
        }
        gw_drive_release();
    }
    disk_progress_override(-1);

    // The image is no worse than before either way, so it goes back up.
    const char *why = started ? "" : gw_last_error();
    if (!publish_image()) return;
    uint32_t bad = disk_bad_sector_count();
    if (started)
        snprintf(s_retry_msg, sizeof s_retry_msg,
                 "retry: %u bad sector%s left (was %u)",
                 (unsigned)bad, bad == 1 ? "" : "s", (unsigned)bad_before);
    else
        snprintf(s_retry_msg, sizeof s_retry_msg,
                 "retry did not start: %s - image unchanged", why);
    disk_set_detail(s_retry_msg);
    ESP_LOGI(TAG, "%s", s_retry_msg);
}

// Where a write/verify job's time goes (the disk test reports it). Summed
// over every track write and verify read until job_timing_reset().
typedef struct {
    uint64_t enc_us, wr_us, rd_us, dec_us;
    uint32_t n_wr, n_rd;
} job_timing_t;
static job_timing_t s_jt;
static void job_timing_reset(void) { memset(&s_jt, 0, sizeof s_jt); }

// Encode one track from the image into s_wgw[buf]: flux bytes, or negative.
// Uses s_wsect/s_wcells/s_ticks, so only one encode or decode at a time.
static int encode_track(uint8_t cyl, uint8_t head, int buf)
{
    int64_t t0 = esp_timer_get_time();
    for (int s = 0; s < DISK_SECTORS; s++) {
        uint32_t lba = disk_lba(cyl, head, (uint8_t)(s + 1));
        if (!disk_image_read(lba, s_wsect[s], 1)) return -20;
    }
    enc_track_t enc;
    enc_bind(&enc, s_wcells, WCELLS_CAP);
    enc_build_track(&enc, cyl, head, s_wsect);
    if (enc.overflow) return -21;
    size_t nt = enc_cells_to_ticks(&enc, s_ticks, FLUX_TICKS_MAX, gw_sample_freq());
    size_t ng = enc_ticks_to_gw(s_ticks, nt, s_wgw[buf], WGW_CAP);
    ESP_LOGD(TAG, "write c%uh%u: %u cells -> %u ticks -> %u flux bytes",
             (unsigned)cyl, (unsigned)head, (unsigned)enc.ncells,
             (unsigned)nt, (unsigned)ng);
    // The encoders truncate silently (and still terminate the stream), which
    // would write a partial track that reports success.
    if (nt == FLUX_TICKS_MAX) return -23;
    if (ng == WGW_CAP)        return -22;
    if (ng > 120 * 1024)      return -24;      // fits even a 128 KB controller buffer
    s_jt.enc_us += (uint64_t)(esp_timer_get_time() - t0);
    return (int)ng;
}

// Write s_wgw[buf] (len bytes) to a track, timed. 0 or negative.
static int write_track_buf(uint8_t cyl, uint8_t head, int buf, size_t len)
{
    int64_t t0 = esp_timer_get_time();
    int rc = gw_write_flux(cyl, head, s_wgw[buf], len);
    s_jt.wr_us += (uint64_t)(esp_timer_get_time() - t0);
    s_jt.n_wr++;
    return rc;
}

// Re-encode one track from the image and write it. 0 or negative.
static int write_track_from_image(uint8_t cyl, uint8_t head)
{
    int ng = encode_track(cyl, head, 0);
    return ng < 0 ? ng : write_track_buf(cyl, head, 0, (size_t)ng);
}

// Read a track back and compare it with the image, without touching the
// image (a bad read mustn't spoil a sector we may rewrite). A sector that
// reads clean but differs counts as bad, so a write that never reached the
// disk can't pass on the old contents. Returns bad+missing, or -1 on a link
// failure. *out (optional) gets the decode result, timing margin included.
// Decode a read-back in s_raw[slot] and compare it with the image: bad+missing.
static int verify_decode(uint8_t cyl, uint8_t head, int slot, size_t raw_len,
                         mfm_result_t *r)
{
    int64_t t0 = esp_timer_get_time();
    mfm_track_ctx_t c = { .sample_freq = gw_sample_freq(),
                          .exp_cyl = cyl, .exp_head = head,
                          .verify_only = 1, .verify_compare = 1, .want_margin = 1 };
    decode_and_store(s_raw[slot], raw_len, &c, r);
    s_jt.dec_us += (uint64_t)(esp_timer_get_time() - t0);
    return r->sectors_bad + r->sectors_missing;
}

// Read a track into s_raw[slot], timed. gw_read_flux_raw()'s result.
static int read_track_raw(uint8_t cyl, uint8_t head, int slot, size_t *raw_len)
{
    int64_t t0 = esp_timer_get_time();
    int rc = gw_read_flux_raw(cyl, head, s_raw[slot], GW_FLUX_RAW_MAX, raw_len, NULL);
    s_jt.rd_us += (uint64_t)(esp_timer_get_time() - t0);
    s_jt.n_rd++;
    return rc;
}

static int verify_track(uint8_t cyl, uint8_t head, mfm_result_t *out)
{
    mfm_result_t r = { .sectors_missing = DISK_SECTORS };
    if (out) *out = r;
    size_t raw_len = 0;
    if (read_track_raw(cyl, head, 0, &raw_len) != 0) return -1;
    int bad = verify_decode(cyl, head, 0, raw_len, &r);
    if (out) *out = r;
    return bad;
}

// Verify, rewriting up to max_rewrites times on failure. Returns the final
// bad+missing count, or <0 on a link failure. *rewrites (optional) counts the
// rewrites; *last (optional) gets the final read-back's result.
static int verify_and_retry_n(uint8_t cyl, uint8_t head, int max_rewrites, int *rewrites,
                              mfm_result_t *last)
{
    int bad = verify_track(cyl, head, last);
    // A cancel skips the rewrites (the caller stops at the next track).
    for (int tries = 0; bad > 0 && tries < max_rewrites && !ui_cancelled(); tries++) {
        ESP_LOGW(TAG, "verify c%uh%u: %d bad/missing, rewrite try %d",
                 (unsigned)cyl, (unsigned)head, bad, tries + 1);
        if (rewrites) (*rewrites)++;
        if (write_track_from_image(cyl, head) != 0) break;
        bad = verify_track(cyl, head, last);
    }
    return bad;
}

static int verify_and_retry(uint8_t cyl, uint8_t head, int *rewrites, mfm_result_t *last)
{
    return verify_and_retry_n(cyl, head, 2, rewrites, last);
}

// "ok/18", or "ERR" for a link failure.
static const char *verify_frac(int bad, char *buf, size_t n)
{
    if (bad < 0) snprintf(buf, n, "ERR");
    else         snprintf(buf, n, "%d/%d", DISK_SECTORS - bad, DISK_SECTORS);
    return buf;
}

// Write cylinder 0 (FAT + root directory, all a delete touches) from the image
// to the disk; also the manual write test. Static because disk_set_detail()
// keeps the pointer.
static char s_commit_msg[128];

// Nothing reached the disk: restore the cyl0 checkpoint a delete took and
// remount (FatFs caches FAT state). Returns a suffix for the status line.
static const char *undo_uncommitted_delete(void)
{
    disk_volume_lock();
    bool undone = disk_image_rollback_cyl0();
    bool ok = !undone || fat12_mount();
    if (!ok) disk_set_state(DISK_ERROR);
    if (undone) disk_bump_generation();     // the deleted file is back in the list
    disk_volume_unlock();
    if (!undone) return "";
    return ok ? " - delete undone, disk unchanged"
              : " - delete undone but remount failed, re-read the disk";
}

static void commit_cyl0_locked(const char *tag);

// Holds the volume lock throughout, so a web delete can't change cyl0 while
// it is being written.
static void commit_cyl0(const char *tag)
{
    if (!disk_volume_acquire()) {
        disk_image_discard_cyl0();
        snprintf(s_commit_msg, sizeof s_commit_msg, "%s: no disk loaded", tag);
        disk_set_detail(s_commit_msg); return;
    }
    commit_cyl0_locked(tag);
    disk_volume_unlock();
}

static void commit_cyl0_locked(const char *tag)
{
    ESP_LOGW(TAG, "%s: writing cyl0 h0+h1 to disk", tag);
    if ((!gw_handshake() && (!gw_relink() || !gw_handshake())) || !gw_drive_prepare()) {
        const char *undo = undo_uncommitted_delete();
        snprintf(s_commit_msg, sizeof s_commit_msg, "%s: %s%s", tag, gw_last_error(), undo);
        disk_set_detail(s_commit_msg);
        ESP_LOGW(TAG, "%s", s_commit_msg);
        return;
    }

    int r0 = write_track_from_image(0, 0);
    int r1 = (r0 == 0) ? write_track_from_image(0, 1) : 99;

    // Verify with the motor still on; rewrites should be rare (media wear).
    int v0 = (r0 == 0) ? verify_and_retry(0, 0, NULL, NULL) : -1;
    int v1 = (r1 == 0) ? verify_and_retry(0, 1, NULL, NULL) : -1;
    gw_drive_release();

    if (r0 == 0 && r1 == 0) {
        char b0[8], b1[8];
        snprintf(s_commit_msg, sizeof s_commit_msg,
                 "%s c0h0=0 c0h1=0 verify h0=%s h1=%s %s", tag,
                 verify_frac(v0, b0, sizeof b0), verify_frac(v1, b1, sizeof b1),
                 (v0 == 0 && v1 == 0) ? "OK" : "FAILED");
    } else {
        // r0 failed: usually before any flux was sent, so restore the old
        // image (a mid-stream failure may leave head 0 partial; the message
        // says to re-read). r0 ok but r1 failed: head 0 has the new FAT, so
        // keep the edited image; retrying rewrites both heads.
        const char *undo = (r0 != 0) ? undo_uncommitted_delete() : "";
        snprintf(s_commit_msg, sizeof s_commit_msg, "%s c0h0=%d c0h1=%d %s%s",
                 tag, r0, r1, gw_last_error(), undo);
    }
    if (r0 == 0) disk_image_discard_cyl0();
    disk_set_detail(s_commit_msg);
    ESP_LOGW(TAG, "%s", s_commit_msg);
}

// WRITE_DISK: write every track of an image to the floppy, each one read back
// and compared. The image is the staged upload, or the loaded one (read a
// disk, swap disks, write: a copy). Static message buffer: disk_set_detail()
// keeps it.
static char s_write_msg[224];

// ---------------- disk test (destructive) and health check (read-only) ----------------
//
// TEST: is this floppy still fit to use? Every track is written and read back
// with each pattern in turn, then (full test) the whole disk is read again:
//   RAND  pseudo-random data       mixed flux spacings; can't pass on stale data
//   INV   its bitwise complement   every bit cell written both ways (full only)
//   FMT   a blank 1.44 MB FAT12    mostly zeros, the densest flux; the disk is
//         format                   left formatted and empty, ready to use
//   CHK   read every track again   after all the writing: catches a track that
//                                  fades or was clobbered by its neighbour's
//                                  write (full only)
// The quick test is RAND then FMT. A write pass rewrites a track up to twice
// if the read-back is wrong; CHK re-reads (up to TRACK_RETRIES) instead. Clean
// first time = GOOD, clean only after a retry = weak (TRK_RETRY), never clean
// = BAD; the track map keeps each track's worst result over all passes, and
// each track's timing margin comes from the last pass.
//
// If track 0 reads back clean but still holding its old data after every
// rewrite, the writes aren't reaching the disk (write-protect sensor or write
// circuit): the test stops there rather than fail 160 tracks. With bad tracks
// but a good cylinder 0, their clusters are marked bad in the FAT, as DOS
// FORMAT did, so the disk is still usable at a smaller size.
//
// HEALTH: read-only. Every track is read twice (more if a read has errors),
// graded the same way, and its timing margin measured. The sectors go into the
// image, so it is also a full-surface read of a disk with data on it.
// TEST_WEAK_MARGINAL and MARGIN_LOW live in app_config.h (the web page's
// margin view shows them too).
// FAT12 1.44 MB layout: boot, 2 x 9 FAT sectors, 14 root-directory sectors.
#define FAT12_FIRST_DATA   33
#define FAT12_CLUSTERS     (TOTAL_SECTORS - FAT12_FIRST_DATA)   // 2847, from #2

// TP_IMG: write the image as loaded (the image write), no test pattern.
enum { TP_RAND, TP_INV, TP_FMT, TP_CHK, TP_IMG };
static const char *const s_tp_name[] = { "RAND", "INV", "FMT", "CHK", "IMG" };
static const uint8_t s_tp_full[]  = { TP_RAND, TP_INV, TP_FMT, TP_CHK };
static const uint8_t s_tp_quick[] = { TP_RAND, TP_FMT };

static uint32_t test_rand_seed(uint32_t lba)
{
    uint32_t x = lba * 2654435761u ^ 0x5EED1234u;
    return x ? x : 1;
}

// Sector `lba` of a blank, standard (DOS 1.44 MB BPB) FAT12 disk.
static void blank_fat12_sector(uint32_t lba, uint8_t *b, uint32_t vol_serial)
{
    memset(b, 0, SECTOR_SIZE);
    if (lba == 0) {
        static const uint8_t bpb[] = {
            0xEB, 0x3C, 0x90, 'M', 'S', 'D', 'O', 'S', '5', '.', '0',
            0x00, 0x02,   // 512 bytes per sector
            0x01,         // 1 sector per cluster
            0x01, 0x00,   // 1 reserved sector
            0x02,         // 2 FATs
            0xE0, 0x00,   // 224 root entries
            0x40, 0x0B,   // 2880 sectors
            0xF0,         // media: 3.5" HD
            0x09, 0x00,   // 9 sectors per FAT
            0x12, 0x00,   // 18 sectors per track
            0x02, 0x00,   // 2 heads
        };
        memcpy(b, bpb, sizeof bpb);
        b[0x26] = 0x29;                                  // extended boot signature
        memcpy(&b[0x27], &vol_serial, 4);
        memcpy(&b[0x2B], "NO NAME    FAT12   ", 19);     // label + fs type
        b[0x3E] = 0xCD; b[0x3F] = 0x18;                  // not bootable: int 18h
        b[0x1FE] = 0x55; b[0x1FF] = 0xAA;
    } else if (lba == 1 || lba == 10) {                  // FAT 1 and FAT 2
        b[0] = 0xF0; b[1] = 0xFF; b[2] = 0xFF;
    }
}

// Put a write pass's data for one track into the image (write_track_from_image()
// writes it, verify_track() compares against it).
static void test_fill_track(int tp, uint8_t cyl, uint8_t head, uint32_t vol_serial)
{
    uint8_t buf[SECTOR_SIZE];
    for (uint8_t s = 1; s <= DISK_SECTORS; s++) {
        uint32_t lba = disk_lba(cyl, head, s);
        if (tp == TP_FMT) {
            blank_fat12_sector(lba, buf, vol_serial);
        } else {
            uint32_t x = test_rand_seed(lba);           // xorshift32
            for (int i = 0; i < SECTOR_SIZE; i += 4) {
                x ^= x << 13; x ^= x >> 17; x ^= x << 5;
                uint32_t v = (tp == TP_INV) ? ~x : x;
                memcpy(&buf[i], &v, 4);
            }
        }
        disk_image_write(lba, buf, 1);
    }
}

// CHK pass re-reads: read a track up to `tries` times against the image.
// *last gets the final read's result. bad+missing, or <0.
static int test_reread_track(uint8_t cyl, uint8_t head, int tries, mfm_result_t *last)
{
    int bad = -1;
    for (int a = 0; a < tries; a++) {
        bad = verify_track(cyl, head, last);
        if (bad <= 0 || ui_cancelled()) break;
    }
    return bad;
}

static void fat12_set(uint8_t *fat, uint32_t n, uint16_t v)
{
    uint32_t o = n + n / 2;
    if (n & 1) {
        fat[o]     = (uint8_t)((fat[o] & 0x0F) | ((v & 0x0F) << 4));
        fat[o + 1] = (uint8_t)(v >> 4);
    } else {
        fat[o]     = (uint8_t)v;
        fat[o + 1] = (uint8_t)((fat[o + 1] & 0xF0) | ((v >> 8) & 0x0F));
    }
}

// Mark the clusters on BAD tracks as bad (0xFF7) in both FATs of the blank
// format in the image. Returns the number of clusters marked.
static int test_mark_bad_clusters(const uint8_t *worst)
{
    static uint8_t fat[9 * SECTOR_SIZE];
    disk_image_read(1, fat, 9);
    int n = 0;
    for (int t = 0; t < TOTAL_TRACKS; t++) {
        if (worst[t] != TRK_BAD) continue;
        for (uint8_t s = 1; s <= DISK_SECTORS; s++) {
            uint32_t lba = disk_lba((uint8_t)(t / DISK_HEADS), (uint8_t)(t % DISK_HEADS), s);
            if (lba < FAT12_FIRST_DATA) continue;
            fat12_set(fat, lba - FAT12_FIRST_DATA + 2, 0xFF7);
            n++;
        }
    }
    disk_image_write(1, fat, 9);
    disk_image_write(10, fat, 9);
    return n;
}

// Margin summary over the tracks that have one: worst (and where), median,
// and how many are under MARGIN_LOW. False if none were measured.
typedef struct { int worst, worst_t, median, n_low; } margin_sum_t;
static bool margin_summary(margin_sum_t *m)
{
    const uint8_t *mg = disk_margins();
    int hist[101] = { 0 }, n = 0;
    *m = (margin_sum_t){ .worst = 101, .worst_t = -1 };
    for (int t = 0; t < TOTAL_TRACKS; t++) {
        if (mg[t] == DISK_MARGIN_NONE) continue;
        hist[mg[t]]++; n++;
        if (mg[t] < m->worst) { m->worst = mg[t]; m->worst_t = t; }
        if (mg[t] < MARGIN_LOW) m->n_low++;
    }
    if (!n) return false;
    for (int v = 0, seen = 0; v <= 100; v++) {
        seen += hist[v];
        if (seen * 2 >= n) { m->median = v; break; }
    }
    return true;
}

// " margin 72 (worst 41 c12h1, 3 low)" for the detail line.
static void margin_text(char *b, size_t n)
{
    margin_sum_t m;
    if (!margin_summary(&m)) { b[0] = '\0'; return; }
    int o = snprintf(b, n, ", margin %d (worst %d c%02dh%d", m.median, m.worst,
                     m.worst_t / DISK_HEADS, m.worst_t % DISK_HEADS);
    if (m.n_low) o += snprintf(b + o, n - o, ", %d under %d", m.n_low, MARGIN_LOW);
    snprintf(b + o, n - o, ")");
}

// " | ms/trk enc 40 wr 310 rd 420 dec 60" for the detail line.
static void timing_text(char *b, size_t n)
{
    uint32_t w = s_jt.n_wr ? s_jt.n_wr : 1, r = s_jt.n_rd ? s_jt.n_rd : 1;
    snprintf(b, n, " | ms/trk enc %u wr %u rd %u dec %u",
             (unsigned)(s_jt.enc_us / w / 1000), (unsigned)(s_jt.wr_us / w / 1000),
             (unsigned)(s_jt.rd_us / r / 1000), (unsigned)(s_jt.dec_us / r / 1000));
}

static char s_test_msg[512];   // detail line: verdict, margin, timing
static char s_test_note[64];

// Disk test state shared by test_disk() (gw task, core 1: writes and reads)
// and test_pipe_fn() (core 0: fills, encodes, decodes). A track's entries are
// written by one side only (redo[]: the gw task when it never read the track,
// else the pipe task), and read by test_disk() after pipe_stop().
static struct {
    int      tp;                         // this pass's pattern (TP_*)
    bool     last_pass;
    uint32_t vol_serial;
    uint8_t  worst[TOTAL_TRACKS];        // worst grade over the passes
    uint8_t  stale[TOTAL_TRACKS];        // a read-back that kept old data
    uint8_t  redo[TOTAL_TRACKS];         // failed in the pipe: redo after the pass
} s_tst;

static void test_grade(int t, int bad, bool weak, const mfm_result_t *res)
{
    uint8_t cyl = (uint8_t)(t / DISK_HEADS), head = (uint8_t)(t % DISK_HEADS);
    track_state_t st = bad == 0 ? (weak ? TRK_RETRY : TRK_GOOD) : TRK_BAD;
    if (st > s_tst.worst[t]) s_tst.worst[t] = (uint8_t)st;
    disk_trackmap_set(cyl, head, (track_state_t)s_tst.worst[t]);
    if (s_tst.last_pass && bad >= 0) disk_margin_set(cyl, head, res->margin);
    if (bad > 0 && res->sectors_stale && s_tst.tp != TP_CHK) s_tst.stale[t] = 1;
}

// Core 0. PK_ENC: fill the track's pattern into the image and encode it.
// PK_DEC: compare a read-back with the image; a clean one is graded now, any
// other goes on the redo list (rewritten or re-read after the pass).
static void test_pipe_fn(pipe_item_t *it)
{
    int t = it->cyl * DISK_HEADS + it->head;
    if (it->kind == PK_ENC) {
        if (s_tst.tp != TP_IMG)
            test_fill_track(s_tst.tp, it->cyl, it->head, s_tst.vol_serial);
        int n = encode_track(it->cyl, it->head, it->slot);
        it->rc = n < 0 ? n : 0;
        it->raw_len = n < 0 ? 0 : (size_t)n;
        return;
    }
    mfm_result_t r = { .sectors_missing = DISK_SECTORS };
    int bad = it->rc == 0 ? verify_decode(it->cyl, it->head, it->slot, it->raw_len, &r) : -1;
    if (bad == 0) test_grade(t, 0, false, &r);
    else          s_tst.redo[t] = 1;
}

static void test_enc_req(int t, int buf)
{
    pipe_item_t e = { .kind = PK_ENC, .slot = (int8_t)buf,
                      .cyl = (uint8_t)(t / DISK_HEADS), .head = (uint8_t)(t % DISK_HEADS) };
    pipe_put(&e);
}

static void test_job_pos(int t)
{
    if (s_tst.tp != TP_IMG) display_set_job_pos(s_tp_name[s_tst.tp], t);
}

static void test_progress(int pi, int npass, int t)
{
    disk_progress_override((int32_t)(((uint64_t)pi * TOTAL_TRACKS + t + 1) *
                                     TOTAL_SECTORS / ((uint64_t)npass * TOTAL_TRACKS)));
}

// One pass over tracks t0.. with the work split across the cores: while this
// task writes and reads back track N, core 0 compares N-1 and encodes N+1
// (write passes), or decodes N-1 (CHK). NULL, or why it stopped and *stop_t
// at the track.
static const char *test_pass_pipelined(int t0, int pi, int npass, int *link_fails,
                                       int *stop_t, bool *cancelled)
{
    const bool wr = s_tst.tp != TP_CHK;
    const char *why = NULL;
    int t = t0;
    if (!pipe_start(test_pipe_fn)) { *stop_t = t0; return "out of memory starting the pipeline"; }
    if (wr)
        for (int k = 0; k < 2 && t0 + k < TOTAL_TRACKS; k++) test_enc_req(t0 + k, k);
    for (; t < TOTAL_TRACKS; t++) {
        if (ui_cancelled()) { *cancelled = true; why = "cancelled"; break; }
        uint8_t cyl = (uint8_t)(t / DISK_HEADS), head = (uint8_t)(t % DISK_HEADS);
        test_job_pos(t);
        int rc = 0;
        bool enc_fail = false;
        if (wr) {
            pipe_item_t e;
            xQueueReceive(s_q_enc, &e, portMAX_DELAY);       // in order: track t
            if (e.rc == 0) rc = write_track_buf(cyl, head, e.slot, e.raw_len);
            else           enc_fail = true;
            if (t + 2 < TOTAL_TRACKS) test_enc_req(t + 2, e.slot);
        }
        if (!enc_fail && rc == 0) {
            pipe_item_t it = { .kind = PK_DEC, .slot = (int8_t)pipe_take_slot(),
                               .cyl = cyl, .head = head };
            it.rc = read_track_raw(cyl, head, it.slot, &it.raw_len);
            it.fstat = gw_last_flux_status();
            rc = it.rc;
            pipe_put(&it);                 // a failed read comes back as a redo
        } else {
            s_tst.redo[t] = 1;
        }
        if (enc_fail || rc == 0) {
            if (rc == 0) *link_fails = 0;
        } else if (!recover_link() || ++*link_fails >= LINK_FAIL_ABORT) {
            why = "link dead mid-test";
            break;
        }
        test_progress(pi, npass, t);
    }
    *stop_t = t;
    pipe_stop();
    return why;
}

// After a pipelined pass: the tracks that failed in it, done the old way. A
// write pass rewrites the track (and once more if still bad, so two rewrites
// as before); CHK re-reads it. Either way a pass here counts as weak.
static const char *test_redo(int *link_fails, int *rewrites_total, int *rereads_total,
                             int *stop_t, bool *cancelled)
{
    const bool wr = s_tst.tp != TP_CHK;
    for (int t = 0; t < TOTAL_TRACKS; t++) {
        if (!s_tst.redo[t]) continue;
        *stop_t = t;
        if (ui_cancelled()) { *cancelled = true; return "cancelled"; }
        uint8_t cyl = (uint8_t)(t / DISK_HEADS), head = (uint8_t)(t % DISK_HEADS);
        test_job_pos(t);
        mfm_result_t res = { .sectors_missing = DISK_SECTORS };
        int bad;
        if (wr) {
            int rewrites = 0;
            int w = write_track_from_image(cyl, head);
            bad = (w == 0) ? verify_and_retry_n(cyl, head, 1, &rewrites, &res) : -1;
            *rewrites_total += 1 + rewrites;
        } else {
            bad = test_reread_track(cyl, head, TRACK_RETRIES - 1, &res);
            (*rereads_total)++;
        }
        test_grade(t, bad, true, &res);
        if (bad != 0)
            ESP_LOGW(TAG, "test %s c%02u h%u: %s (stale %d)", s_tp_name[s_tst.tp],
                     (unsigned)cyl, (unsigned)head,
                     bad < 0 ? "write/link failure" : "bad after retries", res.sectors_stale);
        if (bad == 0) *link_fails = 0;
        else if (bad < 0 && (!recover_link() || ++*link_fails >= LINK_FAIL_ABORT))
            return "link dead mid-test";
    }
    return NULL;
}

static void write_whole_disk(void)
{
    uint8_t *img = disk_stage_take();
    disk_state_t st = disk_get_state();
    if (!img && st != DISK_READY && st != DISK_READY_DIRTY) {
        disk_set_detail("write: no image loaded");
        return;
    }
    ESP_LOGW(TAG, "writing the whole disk from %s image",
             img ? "an uploaded" : "the loaded");

    disk_volume_lock();
    disk_set_state(DISK_READING);
    sd_sha_image_cache_clear();
    disk_volume_unlock();
    fat12_unmount();
    if (img) {
        disk_image_reset();
        disk_image_write(0, img, TOTAL_SECTORS);
        heap_caps_free(img);
        disk_set_state(DISK_READING);
    } else {
        disk_image_discard_cyl0();      // every track is about to match the image
    }
    disk_trackmap_reset();
    disk_set_detail("");
    display_note_saved(NULL);
    disk_progress_override(0);
    job_timing_reset();
    int64_t t_start = esp_timer_get_time();

    // The disk test's pipeline with the image as the "pattern": core 0
    // encodes the next track and compares the last read-back while this task
    // writes and reads back the current one; failed tracks are rewritten
    // after the pass, two rewrites as before (speed-up phase 4).
    s_tst.tp = TP_IMG;
    s_tst.last_pass = true;
    memset(s_tst.worst, TRK_UNREAD, sizeof s_tst.worst);
    memset(s_tst.stale, 0, sizeof s_tst.stale);
    memset(s_tst.redo, 0, sizeof s_tst.redo);
    int link_fails = 0, rewrites_total = 0, rereads = 0, t = 0;
    bool cancelled = false;
    const char *abort_why = drive_start() ? NULL : gw_last_error();
    if (!abort_why) {
        // Track 0 alone first: a refused write (write-protect, no disk, no
        // drive) stops here rather than failing 160 times.
        mfm_result_t res = { .sectors_missing = DISK_SECTORS };
        int rewrites = 0;
        int w   = write_track_from_image(0, 0);
        int bad = (w == 0) ? verify_and_retry(0, 0, &rewrites, &res) : -1;
        rewrites_total += rewrites;
        if (w != 0) {
            abort_why = gw_last_error();
        } else {
            test_grade(0, bad, rewrites > 0, &res);
            if (bad < 0 && (!recover_link() || ++link_fails >= LINK_FAIL_ABORT))
                abort_why = "link dead mid-write";
        }
        test_progress(0, 1, 0);
    }
    if (!abort_why) {
        int stop_t = 1;
        abort_why = test_pass_pipelined(1, 0, 1, &link_fails, &stop_t, &cancelled);
        if (!abort_why)
            abort_why = test_redo(&link_fails, &rewrites_total, &rereads, &stop_t,
                                  &cancelled);
        if (abort_why) t = stop_t;
    }
    if (abort_why && !strcmp(abort_why, "link dead mid-test"))
        abort_why = "link dead mid-write";
    gw_drive_release();
    disk_progress_override(-1);
    display_set_job_pos(NULL, -1);
    unsigned secs = (unsigned)((esp_timer_get_time() - t_start) / 1000000);

    if (abort_why) {
        // The image in memory is no longer what is on the disk.
        snprintf(s_write_msg, sizeof s_write_msg,
                 "write failed at track %d of %d: %s%s", t, TOTAL_TRACKS, abort_why,
                 t > 1 ? " - the disk is part-written, write it again" : "");
        disk_image_reset();
        disk_trackmap_reset();
        disk_set_detail(s_write_msg);
        disk_set_state(DISK_ERROR);
        ESP_LOGE(TAG, "%s", s_write_msg);
        return;
    }

    int failed = 0;
    for (int i = 0; i < TOTAL_TRACKS; i++)
        if (s_tst.worst[i] == TRK_BAD) failed++;
    char tim[64];
    timing_text(tim, sizeof tim);

    // An image need not be FAT12; then there is just no file list to show.
    bool mounted = publish_image();
    if (!mounted) disk_image_reset();
    if (failed)
        snprintf(s_write_msg, sizeof s_write_msg,
                 "write: %d of %d tracks FAILED verify - not a good copy, %us%s",
                 failed, TOTAL_TRACKS, secs, tim);
    else
        snprintf(s_write_msg, sizeof s_write_msg,
                 "write OK: %d tracks written and verified%s, %us%s", TOTAL_TRACKS,
                 mounted ? "" : " (not a FAT12 disk, so no file list)", secs, tim);
    display_note(failed ? "WRITE: SOME TRACKS FAILED VERIFY" : "DISK WRITTEN AND VERIFIED");
    disk_set_detail(s_write_msg);
    ESP_LOGW(TAG, "%s", s_write_msg);
}

// Leave READY under the volume lock, as a read does, and start a fresh image
// and track map for a job that fills them track by track.
static void job_begin_fresh(void)
{
    disk_volume_lock();
    disk_set_state(DISK_READING);
    sd_sha_image_cache_clear();
    disk_volume_unlock();
    fat12_unmount();
    disk_image_reset();              // also disarms the cyl0 checkpoint
    disk_set_state(DISK_READING);
    disk_trackmap_reset();
    disk_set_detail("");
    display_note_saved(NULL);
    disk_progress_override(0);
    job_timing_reset();
    g_last_bad_read[0] = '\0';
}

static void test_disk(bool quick)
{
    const uint8_t *passes = quick ? s_tp_quick : s_tp_full;
    const int npass = quick ? (int)sizeof s_tp_quick : (int)sizeof s_tp_full;
    const char *kind = quick ? "quick test" : "test";
    ESP_LOGW(TAG, "disk %s requested: overwriting every track", kind);
    job_begin_fresh();

    uint8_t *worst = s_tst.worst, *stale = s_tst.stale;
    memset(worst, TRK_UNREAD, TOTAL_TRACKS);
    memset(stale, 0, TOTAL_TRACKS);
    int rewrites_total = 0, rereads_total = 0, link_fails = 0;
    int fail_pi = -1, fail_t = 0;
    bool cancelled = false;
    s_tst.vol_serial = esp_random();
    int64_t t_start = esp_timer_get_time();
    const char *abort_why = drive_start() ? NULL : gw_last_error();
    bool nothing_written = abort_why != NULL;

    for (int pi = 0; !abort_why && pi < npass; pi++) {
        s_tst.tp = passes[pi];
        s_tst.last_pass = pi == npass - 1;
        memset(s_tst.redo, 0, sizeof s_tst.redo);
        int t0 = 0;
        if (pi == 0) {
            // Track 0 alone, before anything else is written: a refused write
            // (write-protect, no disk, no drive) or writes that never reach
            // the disk stop the test here with the disk unchanged.
            display_set_job_pos(s_tp_name[s_tst.tp], 0);
            test_fill_track(s_tst.tp, 0, 0, s_tst.vol_serial);
            mfm_result_t res = { .sectors_missing = DISK_SECTORS };
            int rewrites = 0;
            int w = write_track_from_image(0, 0);
            int bad = (w == 0) ? verify_and_retry(0, 0, &rewrites, &res) : -1;
            rewrites_total += rewrites;
            if (w != 0) {
                abort_why = gw_last_error();
                nothing_written = true;
            } else if (bad > 0 && res.sectors_stale == DISK_SECTORS) {
                // Every sector read back clean - with its OLD data, after
                // three writes.
                abort_why = "writes are not reaching the disk: track 0 still reads "
                            "back its old data (write-protect sensor or write circuit?)";
                nothing_written = true;
            } else {
                test_grade(0, bad, rewrites > 0, &res);
                if (bad != 0)
                    ESP_LOGW(TAG, "test %s c00 h0: %s (stale %d)", s_tp_name[s_tst.tp],
                             bad < 0 ? "write/link failure" : "bad after retries",
                             res.sectors_stale);
                if (bad < 0 && (!recover_link() || ++link_fails >= LINK_FAIL_ABORT))
                    abort_why = "link dead mid-test";
            }
            test_progress(0, npass, 0);
            if (abort_why) { fail_pi = 0; fail_t = 0; break; }
            t0 = 1;
        }
        int stop_t = t0;
        abort_why = test_pass_pipelined(t0, pi, npass, &link_fails, &stop_t, &cancelled);
        if (!abort_why)
            abort_why = test_redo(&link_fails, &rewrites_total, &rereads_total, &stop_t,
                                  &cancelled);
        if (abort_why) { fail_pi = pi; fail_t = stop_t; }
    }

    // Bad tracks but a good cylinder 0: mark their clusters bad in the FAT and
    // rewrite cylinder 0, so the blank disk never puts a file there.
    int n_bad = 0, n_weak = 0, first_bad = -1, bad_clusters = 0;
    bool cyl0_bad = false, fat_ok = true;
    for (int i = 0; i < TOTAL_TRACKS; i++) {
        if (worst[i] == TRK_BAD) {
            n_bad++;
            if (first_bad < 0) first_bad = i;
            if (i < DISK_HEADS) cyl0_bad = true;
        } else if (worst[i] == TRK_RETRY) {
            n_weak++;
        }
    }
    if (!abort_why && n_bad && !cyl0_bad) {
        display_set_job_pos("FAT", 0);
        bad_clusters = test_mark_bad_clusters(worst);
        for (uint8_t h = 0; h < DISK_HEADS && fat_ok; h++)
            fat_ok = write_track_from_image(0, h) == 0 &&
                     verify_and_retry(0, h, NULL, NULL) == 0;
    }
    gw_drive_release();
    disk_progress_override(-1);
    display_set_job_pos(NULL, -1);
    unsigned secs = (unsigned)((esp_timer_get_time() - t_start) / 1000000);
    char tim[64];
    timing_text(tim, sizeof tim);

    if (abort_why) {
        int fp = fail_pi, ft = fail_t;
        if (fp < 0)
            snprintf(s_test_msg, sizeof s_test_msg, "%s did not start: %s - disk not changed",
                     kind, abort_why);
        else
            snprintf(s_test_msg, sizeof s_test_msg, "%s %s (%s pass, track %d): %s%s",
                     kind, cancelled ? "cancelled" : "aborted", s_tp_name[passes[fp]],
                     ft, cancelled ? "by request" : abort_why,
                     nothing_written ? " - disk not changed" : " - disk is part-written");
        disk_image_reset();
        disk_set_detail(s_test_msg);
        disk_set_state(DISK_ERROR);
        ESP_LOGE(TAG, "%s", s_test_msg);
        return;
    }

    // Sectors on bad tracks: the image holds the format, but the disk can't be
    // trusted to, so show them as bad (READY_DIRTY) rather than "clean".
    for (int i = 0; i < TOTAL_TRACKS; i++)
        if (worst[i] == TRK_BAD)
            disk_image_invalidate(disk_lba((uint8_t)(i / DISK_HEADS),
                                           (uint8_t)(i % DISK_HEADS), 1), DISK_SECTORS);

    char mtxt[64], extra[96] = "";
    margin_text(mtxt, sizeof mtxt);
    int stale_tracks = 0;
    for (int i = 0; i < TOTAL_TRACKS; i++) stale_tracks += stale[i];
    if (stale_tracks)
        snprintf(extra, sizeof extra, "; %d track%s kept old data (write didn't take)",
                 stale_tracks, stale_tracks == 1 ? "" : "s");
    const char *verdict;
    if (n_bad) {
        verdict = "BAD";
        char fat[96];
        if (cyl0_bad)
            snprintf(fat, sizeof fat, "; cylinder 0 is bad, so it can't hold a filesystem");
        else if (fat_ok)
            snprintf(fat, sizeof fat, "; %d KB marked bad in the FAT, %d KB free",
                     bad_clusters / 2, (FAT12_CLUSTERS - bad_clusters) / 2);
        else
            snprintf(fat, sizeof fat, "; marking the bad tracks in the FAT FAILED");
        snprintf(s_test_msg, sizeof s_test_msg,
                 "%s BAD: %d bad track%s (first c%02d h%d), %d weak%s%s%s - don't trust this disk, %us%s",
                 kind, n_bad, n_bad == 1 ? "" : "s", first_bad / DISK_HEADS,
                 first_bad % DISK_HEADS, n_weak, mtxt, extra, fat, secs, tim);
        snprintf(s_test_note, sizeof s_test_note, "TEST: BAD - %d BAD TRACK%s - BIN IT",
                 n_bad, n_bad == 1 ? "" : "S");
    } else if (n_weak >= TEST_WEAK_MARGINAL) {
        verdict = "MARGINAL";
        snprintf(s_test_msg, sizeof s_test_msg,
                 "%s MARGINAL: no bad tracks but %d weak (%d rewrites, %d re-reads)%s%s - use for nothing you need, %us%s",
                 kind, n_weak, rewrites_total, rereads_total, mtxt, extra, secs, tim);
        snprintf(s_test_note, sizeof s_test_note, "TEST: MARGINAL - %d WEAK TRACKS", n_weak);
    } else {
        verdict = "GOOD";
        snprintf(s_test_msg, sizeof s_test_msg,
                 "%s GOOD: %d x %d tracks written and read back, %d weak (%d rewrites, %d re-reads)%s%s, %us%s",
                 kind, quick ? 2 : 3, TOTAL_TRACKS, n_weak, rewrites_total, rereads_total,
                 mtxt, extra, secs, tim);
        if (n_weak)
            snprintf(s_test_note, sizeof s_test_note, "TEST: GOOD (%d WEAK) - DISK NOW BLANK", n_weak);
        else
            snprintf(s_test_note, sizeof s_test_note, "TEST: GOOD - DISK NOW BLANK");
    }

    // The disk now holds an empty FAT12 volume (bad tracks permitting).
    bool mounted = publish_image();
    if (!mounted) disk_image_reset();
    display_note(s_test_note);
    disk_set_detail(s_test_msg);
    ESP_LOGW(TAG, "disk %s %s: %s", kind, verdict, s_test_msg);
}

// HEALTH: read every track at least twice without writing; grade it and
// measure its margin. The image is filled as a full-surface read.
//
// Per-track read counts, written by health_pipe_fn() (core 0) during the
// pipelined pass and by health_check() after pipe_stop().
static struct { uint8_t reads, clean, dirty, margin; } s_hc[TOTAL_TRACKS];

static void health_count(int t, const mfm_result_t *r)
{
    s_hc[t].reads++;
    if (r->sectors_ok == DISK_SECTORS) s_hc[t].clean++; else s_hc[t].dirty++;
    if (r->margin < s_hc[t].margin) s_hc[t].margin = (uint8_t)r->margin;
}

static track_state_t health_state(int t)
{
    return !s_hc[t].clean ? TRK_BAD : s_hc[t].dirty ? TRK_RETRY : TRK_GOOD;
}

// Core 0: decode one of a track's reads; after its second, show a provisional
// grade on the disk map.
static void health_pipe_fn(pipe_item_t *it)
{
    int t = it->cyl * DISK_HEADS + it->head;
    if (it->rc == 0) {
        int64_t d0 = esp_timer_get_time();
        mfm_track_ctx_t c = s_ctx; c.exp_cyl = it->cyl; c.exp_head = it->head;
        mfm_result_t r;
        decode_and_store(s_raw[it->slot], it->raw_len, &c, &r);
        s_jt.dec_us += (uint64_t)(esp_timer_get_time() - d0);
        health_count(t, &r);
        if (r.sectors_ok != DISK_SECTORS)
            note_bad_read(it->cyl, it->head, it->attempt, it->fstat, s_raw[it->slot],
                          it->raw_len, &r);
    }
    if (it->attempt == 1) disk_trackmap_set(it->cyl, it->head, health_state(t));
}

static void health_check(void)
{
    ESP_LOGI(TAG, "health check requested");
    job_begin_fresh();
    int64_t t_start = esp_timer_get_time();
    const char *abort_why = drive_start() ? NULL : gw_last_error();
    s_ctx = (mfm_track_ctx_t){ .sample_freq = gw_sample_freq(), .want_margin = 1 };
    int n_bad = 0, n_weak = 0, first_bad = -1, extra_reads = 0, link_fails = 0, t = 0;
    bool cancelled = false;
    for (int i = 0; i < TOTAL_TRACKS; i++) s_hc[i] = (typeof(s_hc[0])){ .margin = 101 };

    // Pass 1, pipelined: this task reads every track twice while core 0
    // decodes the read before.
    if (!abort_why && !pipe_start(health_pipe_fn))
        abort_why = "out of memory starting the pipeline";
    else if (!abort_why) {
        for (t = 0; !abort_why && t < TOTAL_TRACKS; t++) {
            if (ui_cancelled()) { cancelled = true; abort_why = "cancelled"; break; }
            uint8_t cyl = (uint8_t)(t / DISK_HEADS), head = (uint8_t)(t % DISK_HEADS);
            display_set_job_pos("HLTH", t);
            int ok = 0, no_flux = 0;
            for (int a = 0; a < 2; a++) {
                pipe_item_t it = { .kind = PK_DEC, .slot = (int8_t)pipe_take_slot(),
                                   .cyl = cyl, .head = head, .attempt = (uint8_t)a };
                it.rc = read_track_raw(cyl, head, it.slot, &it.raw_len);
                it.fstat = gw_last_flux_status();
                if (it.rc == 0) {
                    ok++;
                    if (gw_last_flux_status() == ACK_NO_INDEX) no_flux++;
                }
                pipe_put(&it);
                if (it.rc == GW_RD_NO_DRIVE) { abort_why = gw_last_error(); break; }
                if (it.rc == 0) {
                    link_fails = 0;
                } else if (!recover_link() || ++link_fails >= LINK_FAIL_ABORT) {
                    abort_why = "link dead mid-check";
                    break;
                }
            }
            if (abort_why) break;
            if (t < DISK_HEADS && ok && no_flux == ok) {
                abort_why = "no disk? the drive returned no flux on cylinder 0";
                break;
            }
            disk_progress_override((t + 1) * DISK_SECTORS);
        }
        pipe_stop();
    }

    // Pass 2: up to TRACK_RETRIES more reads of each track not yet read
    // clean, then the final grades.
    for (int i = 0; !abort_why && i < TOTAL_TRACKS; i++) {
        uint8_t cyl = (uint8_t)(i / DISK_HEADS), head = (uint8_t)(i % DISK_HEADS);
        mfm_track_ctx_t c = s_ctx; c.exp_cyl = cyl; c.exp_head = head;
        for (int a = 0; !s_hc[i].clean && a < TRACK_RETRIES; a++) {
            if (ui_cancelled()) { cancelled = true; abort_why = "cancelled"; break; }
            display_set_job_pos("HLTH", i);
            size_t raw_len = 0;
            int rc = read_track_raw(cyl, head, 0, &raw_len);
            if (rc == GW_RD_NO_DRIVE) { abort_why = gw_last_error(); break; }
            if (rc != 0) {
                if (!recover_link() || ++link_fails >= LINK_FAIL_ABORT) {
                    abort_why = "link dead mid-check"; break;
                }
                continue;
            }
            link_fails = 0;
            extra_reads++;
            int64_t d0 = esp_timer_get_time();
            mfm_result_t r;
            decode_and_store(s_raw[0], raw_len, &c, &r);
            s_jt.dec_us += (uint64_t)(esp_timer_get_time() - d0);
            health_count(i, &r);
        }
        if (abort_why) { t = i; break; }
        track_state_t st = health_state(i);
        if (st == TRK_BAD) { n_bad++; if (first_bad < 0) first_bad = i; }
        if (st == TRK_RETRY) n_weak++;
        disk_trackmap_set(cyl, head, st);
        if (s_hc[i].reads) disk_margin_set(cyl, head, s_hc[i].margin);
        if (st != TRK_GOOD)
            ESP_LOGW(TAG, "health c%02u h%u: %d/%d reads clean, margin %d", (unsigned)cyl,
                     (unsigned)head, s_hc[i].clean, s_hc[i].reads, s_hc[i].margin);
    }
    gw_drive_release();
    disk_progress_override(-1);
    display_set_job_pos(NULL, -1);
    unsigned secs = (unsigned)((esp_timer_get_time() - t_start) / 1000000);

    if (abort_why) {
        snprintf(s_test_msg, sizeof s_test_msg, "health check %s at track %d: %s",
                 cancelled ? "cancelled" : "aborted", t, cancelled ? "by request" : abort_why);
        disk_image_reset();
        disk_set_detail(s_test_msg);
        disk_set_state(DISK_ERROR);
        ESP_LOGE(TAG, "%s", s_test_msg);
        return;
    }

    char mtxt[64], tim[64];
    margin_text(mtxt, sizeof mtxt);
    timing_text(tim, sizeof tim);
    const char *verdict = n_bad ? "BAD" : n_weak >= TEST_WEAK_MARGINAL ? "MARGINAL" : "GOOD";
    int o = snprintf(s_test_msg, sizeof s_test_msg,
                     "health %s: %d bad, %d weak of %d tracks read twice (+%d re-reads)%s",
                     verdict, n_bad, n_weak, TOTAL_TRACKS, extra_reads, mtxt);
    if (n_bad)
        o += snprintf(s_test_msg + o, sizeof s_test_msg - o, ", first bad c%02d h%d",
                      first_bad / DISK_HEADS, first_bad % DISK_HEADS);
    snprintf(s_test_msg + o, sizeof s_test_msg - o, ", %us%s", secs, tim);
    if (n_bad)
        snprintf(s_test_note, sizeof s_test_note, "HEALTH: BAD - %d UNREADABLE TRACK%s",
                 n_bad, n_bad == 1 ? "" : "S");
    else if (n_weak >= TEST_WEAK_MARGINAL)
        snprintf(s_test_note, sizeof s_test_note, "HEALTH: MARGINAL - %d WEAK - COPY IT", n_weak);
    else if (n_weak)
        snprintf(s_test_note, sizeof s_test_note, "HEALTH: GOOD (%d WEAK)", n_weak);
    else
        snprintf(s_test_note, sizeof s_test_note, "HEALTH: GOOD");

    // A disk need not be FAT12: then there's just no file list.
    if (!publish_image()) {
        disk_image_reset();
        disk_set_state(DISK_ERROR);
    }
    display_note(s_test_note);
    disk_set_detail(s_test_msg);
    ESP_LOGW(TAG, "%s", s_test_msg);
}

// SAVE: archive the disk to /sdcard/DISKnnnn/, reading it first if needed, so
// one press per floppy. Separate folders because Mavica cameras restart file
// numbering after a wipe. A disk already on the card is refused unless
// `force` (long press). Static message buffer: disk_set_detail() keeps it.
static char s_saveall_msg[80];
static void save_all_to_sd(bool force)
{
    if (!sd_present()) {
        disk_set_detail("save: no SD card");
        display_note("NO SD CARD - NOTHING SAVED");
        return;
    }

    disk_state_t st = disk_get_state();
    if (st != DISK_READY && st != DISK_READY_DIRTY) {
        ESP_LOGI(TAG, "save: no disk loaded — reading first");
        read_whole_disk();
        st = disk_get_state();
        if (st != DISK_READY && st != DISK_READY_DIRTY)
            return;                  // read_whole_disk() set the detail line
    }

    if (!disk_volume_acquire()) return;
    sd_save_result_t r;
    sd_save_status_t sst = sd_save_disk(force, &r);
    disk_volume_unlock();
    switch (sst) {
    case SD_SAVE_OK:
        // Files already on the card are skipped by hash, so say when the
        // folder is only part of the disk.
        if (r.skipped)
            snprintf(s_saveall_msg, sizeof s_saveall_msg,
                     "%d new file%s to %s (%d already on card)%s",
                     r.copied, r.copied == 1 ? "" : "s", r.dir, r.skipped,
                     r.failed ? " — some failed" : "");
        else
            snprintf(s_saveall_msg, sizeof s_saveall_msg, "saved %d file%s to %s%s",
                     r.copied, r.copied == 1 ? "" : "s", r.dir,
                     r.failed ? " (some failed)" : "");
        display_note_saved(r.dir);
        break;
    case SD_SAVE_DUPLICATE: {
        snprintf(s_saveall_msg, sizeof s_saveall_msg,
                 "already saved as %s — hold SAVE to copy again", r.dir);
        char note[64];
        snprintf(note, sizeof note, "ALREADY ON CARD: %.12s HOLD SAVE TO COPY", r.dir);
        display_note(note);
        break;
    }
    case SD_SAVE_NO_IMAGE:
        snprintf(s_saveall_msg, sizeof s_saveall_msg, "save: nothing to copy");
        display_note("NOTHING TO COPY");
        break;
    case SD_SAVE_MKDIR_FAIL:
        snprintf(s_saveall_msg, sizeof s_saveall_msg, "save: cannot create folder");
        display_note("SD WRITE FAILED - NOTHING SAVED");
        break;
    case SD_SAVE_CARD_FULL:
        snprintf(s_saveall_msg, sizeof s_saveall_msg, "save: DISK9999 reached");
        display_note("DISK9999 REACHED - NOTHING SAVED");
        break;
    case SD_SAVE_NO_SPACE:
        snprintf(s_saveall_msg, sizeof s_saveall_msg,
                 "save: not enough space on the SD card");
        display_note("SD CARD FULL - NOTHING SAVED");
        break;
    case SD_SAVE_NO_CARD:
    default:
        snprintf(s_saveall_msg, sizeof s_saveall_msg, "save: no SD card");
        display_note("NO SD CARD - NOTHING SAVED");
        break;
    }
    disk_set_detail(s_saveall_msg);
    ESP_LOGI(TAG, "%s", s_saveall_msg);
    ESP_LOGI(TAG, "save-all: gw stack headroom %u B",
             (unsigned)uxTaskGetStackHighWaterMark(NULL));
}

// Idle health poll: slow, since each one is a command on the link.
#define GW_IDLE_POLL_MS 10000
// USB-host check (two header-only polls): fast enough that the drive rail is
// up before gw.exe's first spin-up (the RP also waits up to 2 s for it).
#define GW_USB_WATCH_MS 250

static void gw_task(void *arg)
{
    (void)arg;
    int64_t next_health_us = 0;
    while (1) {
        // Finite wait so the idle work below gets a turn.
        int op;
        if (ui_wait_for_op(&op, pdMS_TO_TICKS(GW_USB_WATCH_MS))) {
            // An OTA ends in a reboot, which must not land mid-job; the hold
            // also makes ota_begin() refuse until the job is done.
            if (!ota_hold()) {
                disk_set_detail("busy: firmware update in progress");
                disk_stage_set(NULL);
                ui_op_done();
                continue;
            }
            // Waits out an RP reflash.
            gw_claim(UINT32_MAX);
            // Refuse drive work while a USB host has the controller, and undo
            // a pending delete. SAVE of a loaded disk only needs the SD card.
            bool loaded = disk_get_state() == DISK_READY ||
                          disk_get_state() == DISK_READY_DIRTY;
            bool needs_drive = !((op == UI_OP_SAVEALL || op == UI_OP_SAVEALL_FORCE) && loaded);
            if (needs_drive) gw_usb_watch();
            if (needs_drive && gw_usb_active()) {
                gw_release();
                ota_unhold();
                disk_stage_set(NULL);
                snprintf(s_commit_msg, sizeof s_commit_msg,
                         "busy: controller in use over USB - close gw first%s",
                         op == UI_OP_COMMIT ? undo_uncommitted_delete() : "");
                disk_set_detail(s_commit_msg);
                ESP_LOGW(TAG, "%s", s_commit_msg);
                ui_op_done();
                continue;
            }
            switch (op) {
            case UI_OP_WTEST:         commit_cyl0("wtest");   break;
            case UI_OP_COMMIT:        commit_cyl0("delete");  break;
            case UI_OP_SAVEALL:       save_all_to_sd(false);  break;
            case UI_OP_SAVEALL_FORCE: save_all_to_sd(true);   break;
            case UI_OP_RETRY:         retry_bad_tracks();     break;
            case UI_OP_WRITE_DISK:    write_whole_disk();     break;
            case UI_OP_TEST:          test_disk(false);       break;
            case UI_OP_TEST_QUICK:    test_disk(true);        break;
            case UI_OP_HEALTH:        health_check();         break;
            default:                  read_whole_disk();      break;
            }
            gw_release();
            ota_unhold();
            ui_op_done();
        } else if (!ota_in_progress() && gw_claim(0)) {
            // Idle: nothing in flight, and a reflash holds the claim, so a
            // relink from gw_idle_poll() is safe here.
            gw_usb_watch();
            int64_t now = esp_timer_get_time();
            if (now >= next_health_us) {
                next_health_us = now + (int64_t)GW_IDLE_POLL_MS * 1000;
                gw_idle_poll();
            }
            gw_release();
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_pwr_init();

    // Everything is UTC (get_fattime() uses localtime_r()); the browser
    // converts for display.
    setenv("TZ", "UTC0", 1);
    tzset();

    if (!disk_image_init()) { ESP_LOGE(TAG, "PSRAM image alloc failed"); return; }
    s_ticks  = heap_caps_malloc(FLUX_TICKS_MAX * sizeof(uint32_t), MALLOC_CAP_SPIRAM);
    s_raw[0] = heap_caps_malloc(GW_FLUX_RAW_MAX, MALLOC_CAP_SPIRAM);
    s_raw[1] = heap_caps_malloc(GW_FLUX_RAW_MAX, MALLOC_CAP_SPIRAM);
    s_index  = heap_caps_malloc(IDX_MAX * sizeof(uint32_t), MALLOC_CAP_SPIRAM);
    s_wcells = heap_caps_malloc(WCELLS_CAP, MALLOC_CAP_SPIRAM);
    s_wgw[0] = heap_caps_malloc(WGW_CAP, MALLOC_CAP_SPIRAM);
    s_wgw[1] = heap_caps_malloc(WGW_CAP, MALLOC_CAP_SPIRAM);
    if (!s_ticks || !s_raw[0] || !s_raw[1] || !s_index || !s_wcells || !s_wgw[0] ||
        !s_wgw[1]) {
        ESP_LOGE(TAG, "flux scratch alloc failed"); return;
    }

    ESP_ERROR_CHECK(gw_client_init());
    fat12_reserve_drive(); // drive 0 = floppy image; must precede sd_mount()
    sd_mount();            // before display_init(): the SD init owns SPI2
    sd_index_load();
    sd_detect_init();
    display_init();
    leds_init();           // before ui_init()
    power_io_init();
    power_init();
    wallclock_init();      // seeds the clock before anything stamps a file
    serialno_init();       // needs wallclock_init()
    temps_init();
    ui_init();             // needs the display

    // Before WiFi, so the buttons work during the connect wait. Core 1;
    // web/ui run on core 0. 12 KB because SAVE archives to SD from this task
    // (see web.c for the stack story).
    xTaskCreatePinnedToCore(gw_task, "gw", 12288, NULL, 5, NULL, 1);

    // No stored WiFi: run the captive setup portal instead. The read path
    // works either way.
    if (provision_have_creds()) {
        display_set_mode(DISPLAY_RUNTIME);
        wifi_start();
        sntp_register();   // after wifi_start(): needs the event loop
        web_start();
    } else {
        display_set_mode(DISPLAY_PROVISIONING);
        ui_set_autooff_enabled(false);   // the portal has no activity hook
        provision_run_portal();
    }

    // Confirm a fresh OTA image only now, after WiFi and the link came up; an
    // image that breaks either rolls back on the next restart.
    ota_mark_valid_if_pending();

    ESP_LOGI(TAG, "ready — press button or POST /api/read to read a disk");
}
