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
static uint8_t *s_wgw;
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

// ---- pipelined read (pass 1): this task reads raw flux while decode_task on
// the other core decodes. Buffers change hands only through the two queues. ----
static uint8_t  *s_raw[2];        // ping-pong raw flux buffers (PSRAM)
static uint32_t *s_index;         // flux_decode index scratch (unused downstream)
#define IDX_MAX 8

typedef struct {
    int      slot;                // raw buffer index (0/1); -1 = end-of-stream
    uint8_t  cyl, head;
    int      rc;                  // gw_read_flux_raw() result (0 = ok)
    size_t   raw_len;
} pipe_item_t;
static QueueHandle_t     s_q_filled;   // producer -> decode task (pipe_item_t)
static QueueHandle_t     s_q_empty;    // decode task -> producer (int slot)
static SemaphoreHandle_t s_decode_done;
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
// synchronous paths and decode_task never run at the same time.
static void decode_and_store(const uint8_t *raw, size_t raw_len,
                             const mfm_track_ctx_t *ctx, mfm_result_t *res)
{
#if GW_PROFILE
    int64_t d0 = esp_timer_get_time();
#endif
    res->sectors_ok = 0; res->sectors_bad = 0; res->sectors_missing = DISK_SECTORS;
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

// Pass-1 decoder (core 0). Records deficient tracks for pass 2.
static void decode_task(void *arg)
{
    (void)arg;
    pipe_item_t it;
    for (;;) {
        xQueueReceive(s_q_filled, &it, portMAX_DELAY);
        if (it.slot < 0) break;                          // end of stream
        bool deficient = true;
        if (it.rc == 0) {
            mfm_track_ctx_t c = s_ctx; c.exp_cyl = it.cyl; c.exp_head = it.head;
            mfm_result_t r;
            decode_and_store(s_raw[it.slot], it.raw_len, &c, &r);
            deficient = (r.sectors_missing || r.sectors_bad);
            if (deficient)
                ESP_LOGW(TAG, "c%02u h%u: %d ok, %d bad, %d missing", (unsigned)it.cyl,
                         (unsigned)it.head, r.sectors_ok, r.sectors_bad, r.sectors_missing);
        }
        // Provisionally BAD; pass 2 upgrades it to RETRY if it recovers.
        disk_trackmap_set(it.cyl, it.head, deficient ? TRK_BAD : TRK_GOOD);
        if (deficient && s_retry_n < TOTAL_TRACKS) {
            s_retry[s_retry_n].cyl  = it.cyl;
            s_retry[s_retry_n].head = it.head;
            s_retry_n++;
        }
        int slot = it.slot;
        xQueueSend(s_q_empty, &slot, portMAX_DELAY);
    }
    xSemaphoreGive(s_decode_done);
    vTaskDelete(NULL);
}

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
    bool aborted = false, no_pipeline = false;
    s_q_empty     = xQueueCreate(4, sizeof(int));
    s_q_filled    = xQueueCreate(4, sizeof(pipe_item_t));
    s_decode_done = xSemaphoreCreateBinary();
    // Without the decode task the producer would block forever, motor on.
    if (s_q_empty && s_q_filled && s_decode_done &&
        xTaskCreatePinnedToCore(decode_task, "decode", 8192, NULL, 4, NULL, 0) == pdPASS) {
        for (int i = 0; i < 2; i++) { int s = i; xQueueSend(s_q_empty, &s, 0); }

        for (uint8_t cyl = 1; cyl < DISK_CYLS && !aborted; cyl++) {
            for (uint8_t head = 0; head < DISK_HEADS; head++) {
                if (!used[(uint32_t)cyl * DISK_HEADS + head]) continue;
                int slot;
                xQueueReceive(s_q_empty, &slot, portMAX_DELAY);
                pipe_item_t it = { .slot = slot, .cyl = cyl, .head = head };
#if GW_PROFILE
                gw_read_timing_t t;
                it.rc = gw_read_flux_raw(cyl, head, s_raw[slot], GW_FLUX_RAW_MAX,
                                         &it.raw_len, &t);
                s_stats.attempts++; s_stats.tracks++;
                s_stats.seek_us += t.seek_us; s_stats.readxfer_us += t.readxfer_us;
                s_stats.status_us += t.status_us; s_stats.rawbytes += it.raw_len;
                s_stats.ttfb_us += t.ttfb_us; s_stats.ackwait_us += t.ackwait_us;
#else
                it.rc = gw_read_flux_raw(cyl, head, s_raw[slot], GW_FLUX_RAW_MAX,
                                         &it.raw_len, NULL);
#endif
                xQueueSend(s_q_filled, &it, portMAX_DELAY);
                // A failed track goes to pass 2 like a bad one.
                if (it.rc == 0) consec_link_fail = 0;
                else if (!recover_link() || ++consec_link_fail >= LINK_FAIL_ABORT)
                    aborted = true;
                if (aborted) break;
            }
        }
        pipe_item_t sentinel = { .slot = -1 };
        xQueueSend(s_q_filled, &sentinel, portMAX_DELAY);
        xSemaphoreTake(s_decode_done, portMAX_DELAY);
    } else {
        aborted = no_pipeline = true;
    }
    if (s_q_empty)     vQueueDelete(s_q_empty);
    if (s_q_filled)    vQueueDelete(s_q_filled);
    if (s_decode_done) vSemaphoreDelete(s_decode_done);
    s_q_empty = NULL; s_q_filled = NULL; s_decode_done = NULL;

    if (aborted) {
        const char *why = no_pipeline ? "out of memory starting the read pipeline"
                                      : "link dead mid-read (consecutive track failures)";
        ESP_LOGE(TAG, "%s — aborting", why);
        disk_set_detail(why);
        gw_drive_release(); disk_set_state(DISK_ERROR); return;
    }

    // ---- Pass 2: synchronous retry of tracks still bad ----
    int retry_n = s_retry_n;
    for (int i = 0; i < retry_n; i++)
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
        for (int i = 0; i < TOTAL_TRACKS; i++) {
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

// Re-encode one track from the image and write it. 0 or negative.
static int write_track_from_image(uint8_t cyl, uint8_t head)
{
    for (int s = 0; s < DISK_SECTORS; s++) {
        uint32_t lba = disk_lba(cyl, head, (uint8_t)(s + 1));
        if (!disk_image_read(lba, s_wsect[s], 1)) return -20;
    }
    enc_track_t enc;
    enc_bind(&enc, s_wcells, WCELLS_CAP);
    enc_build_track(&enc, cyl, head, s_wsect);
    if (enc.overflow) return -21;
    size_t nt = enc_cells_to_ticks(&enc, s_ticks, FLUX_TICKS_MAX, gw_sample_freq());
    size_t ng = enc_ticks_to_gw(s_ticks, nt, s_wgw, WGW_CAP);
    ESP_LOGW(TAG, "write c%uh%u: %u cells -> %u ticks -> %u flux bytes",
             (unsigned)cyl, (unsigned)head, (unsigned)enc.ncells,
             (unsigned)nt, (unsigned)ng);
    // The encoders truncate silently (and still terminate the stream), which
    // would write a partial track that reports success.
    if (nt == FLUX_TICKS_MAX) return -23;
    if (ng == WGW_CAP)        return -22;
    if (ng > 120 * 1024)      return -24;      // fits even a 128 KB controller buffer
    return gw_write_flux(cyl, head, s_wgw, ng);
}

// Read a track back and compare it with the image, without touching the
// image (a bad read mustn't spoil a sector we may rewrite). A sector that
// reads clean but differs counts as bad, so a write that never reached the
// disk can't pass on the old contents. Returns bad+missing, or -1 on a link
// failure.
static int verify_track(uint8_t cyl, uint8_t head)
{
    size_t raw_len = 0;
    if (gw_read_flux_raw(cyl, head, s_raw[0], GW_FLUX_RAW_MAX, &raw_len, NULL) != 0)
        return -1;
    mfm_track_ctx_t c = { .sample_freq = gw_sample_freq(),
                          .exp_cyl = cyl, .exp_head = head,
                          .verify_only = 1, .verify_compare = 1 };
    mfm_result_t r;
    decode_and_store(s_raw[0], raw_len, &c, &r);
    return r.sectors_bad + r.sectors_missing;
}

// Verify, rewriting up to twice on failure. Returns the final bad+missing
// count, or <0 on a link failure. *rewrites (optional) counts the rewrites.
static int verify_and_retry(uint8_t cyl, uint8_t head, int *rewrites)
{
    int bad = verify_track(cyl, head);
    for (int tries = 0; bad > 0 && tries < 2; tries++) {
        ESP_LOGW(TAG, "verify c%uh%u: %d bad/missing, rewrite try %d",
                 (unsigned)cyl, (unsigned)head, bad, tries + 1);
        if (rewrites) (*rewrites)++;
        if (write_track_from_image(cyl, head) != 0) break;
        bad = verify_track(cyl, head);
    }
    return bad;
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
    int v0 = (r0 == 0) ? verify_and_retry(0, 0, NULL) : -1;
    int v1 = (r1 == 0) ? verify_and_retry(0, 1, NULL) : -1;
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
static char s_write_msg[144];
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

    int failed = 0, link_fails = 0, t = 0;
    const char *abort_why = drive_start() ? NULL : gw_last_error();
    for (; !abort_why && t < TOTAL_TRACKS; t++) {
        uint8_t cyl = (uint8_t)(t / DISK_HEADS), head = (uint8_t)(t % DISK_HEADS);
        int rewrites = 0;
        int w   = write_track_from_image(cyl, head);
        int bad = (w == 0) ? verify_and_retry(cyl, head, &rewrites) : -1;
        if (bad == 0) {
            link_fails = 0;
            disk_trackmap_set(cyl, head, rewrites ? TRK_RETRY : TRK_GOOD);
        } else {
            disk_trackmap_set(cyl, head, TRK_BAD);
            failed++;
            // The first track refusing the write is write-protect, no disk or
            // no drive: stop rather than fail 160 times.
            if (t == 0 && w != 0)
                abort_why = gw_last_error();
            else if ((w != 0 || bad < 0) &&
                     (!recover_link() || ++link_fails >= LINK_FAIL_ABORT))
                abort_why = "link dead mid-write";
        }
        disk_progress_override((t + 1) * DISK_SECTORS);
    }
    gw_drive_release();
    disk_progress_override(-1);

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

    // An image need not be FAT12; then there is just no file list to show.
    bool mounted = publish_image();
    if (!mounted) disk_image_reset();
    if (failed)
        snprintf(s_write_msg, sizeof s_write_msg,
                 "write: %d of %d tracks FAILED verify - not a good copy",
                 failed, TOTAL_TRACKS);
    else
        snprintf(s_write_msg, sizeof s_write_msg,
                 "write OK: %d tracks written and verified%s", TOTAL_TRACKS,
                 mounted ? "" : " (not a FAT12 disk, so no file list)");
    display_note(failed ? "WRITE: SOME TRACKS FAILED VERIFY" : "DISK WRITTEN AND VERIFIED");
    disk_set_detail(s_write_msg);
    ESP_LOGW(TAG, "%s", s_write_msg);
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
    s_wgw    = heap_caps_malloc(WGW_CAP, MALLOC_CAP_SPIRAM);
    if (!s_ticks || !s_raw[0] || !s_raw[1] || !s_index || !s_wcells || !s_wgw) {
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
