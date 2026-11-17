#include "web.h"
#include "wallclock.h"
#include "temps.h"
#include "app_config.h"
#include "disk_image.h"
#include "fat12_diskio.h"
#include "ui.h"
#include "web_util.h"
#include "netinfo.h"
#include "storage.h"
#include "display.h"
#include "power.h"
#include "provision.h"
#include "link.h"
#include "gw_client.h"   // gw_get_diag: controller counters for /api/status
#include "rp_flash.h"
#include "sd_index.h"
#include "ota.h"
#include "serialno.h"
#include "lastoff.h"
#include "wifi_pwr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <time.h>
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_netif.h"
#include "mdns.h"
#include "ff.h"

static const char *TAG = "web";

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

// A percent-decoded query value (the GUI encodeURIComponent's every name and
// path). ESP_FAIL if the key is missing or decodes to a NUL.
static esp_err_t query_str(const char *query, const char *key, char *out, size_t n)
{
    if (httpd_query_key_value(query, key, out, n) != ESP_OK) return ESP_FAIL;
    return web_url_decode(out) ? ESP_OK : ESP_FAIL;
}

// httpd_req_recv() that rides out a short stall but not a client that has
// gone: after RECV_STALLS timeouts in a row (5 s each, the server's default
// recv_wait_timeout) it gives up, so a dead upload can't hold the one httpd
// task, or an OTA's hold on the drive, for ever.
#define RECV_STALLS 6
static int recv_some(httpd_req_t *req, char *buf, size_t len)
{
    for (int stalls = 0; stalls < RECV_STALLS; stalls++) {
        int n = httpd_req_recv(req, buf, len);
        if (n != HTTPD_SOCK_ERR_TIMEOUT) return n;
    }
    return HTTPD_SOCK_ERR_TIMEOUT;
}

static bool recv_all(httpd_req_t *req, uint8_t *buf, size_t len)
{
    for (size_t got = 0; got < len; ) {
        int n = recv_some(req, (char *)buf + got, len - got);
        if (n <= 0) return false;
        got += (size_t)n;
    }
    return true;
}

// GET /  -> single-page UI
static esp_err_t h_index(httpd_req_t *req)
{
    ui_activity();      // a person is using the web page
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)index_html_start,
                           index_html_end - index_html_start - 1);
}

static const char *state_str(disk_state_t s)
{
    switch (s) {
    case DISK_EMPTY: return "empty";       case DISK_READING: return "reading";
    case DISK_READY: return "ready";       case DISK_READY_DIRTY: return "dirty";
    default:         return "error";
    }
}

// Why the chip last reset: tells a brownout from a panic or watchdog.
static const char *reset_reason_str(void)
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "poweron";
    case ESP_RST_SW:       return "sw";
    case ESP_RST_PANIC:    return "panic";
    case ESP_RST_INT_WDT:  return "int_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt";
    case ESP_RST_WDT:      return "wdt";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_EXT:      return "ext";
    default:               return "other";
    }
}

// GET /api/status -> progress JSON
static esp_err_t h_status(httpd_req_t *req)
{
    char ip[16];
    net_sta_ip_str(ip, sizeof(ip));

    // "null" when no power-path IC is fitted.
    char batt[680] = "null";
    power_state_t ps;
    if (power_get(&ps)) {
        char rate_b[24] = "null";   // signed, negative = discharging
        if (ps.crate_valid)
            snprintf(rate_b, sizeof(rate_b), "%.2f", ps.crate_cpph / 100.0f);
        snprintf(batt, sizeof(batt),
            "{\"soc\":%d,\"mv\":%u,\"usb\":%s,\"chg\":%s,\"state\":\"%s\","
            "\"rate_pph\":%s,\"ichg_ma\":%u,\"vreg_mv\":%u,\"thermreg\":%s,"
            "\"settling\":%s,\"low\":%d,"
            // Sticky gauge alerts plus the three worth acting on, unpacked,
            // and the thresholds they were judged against.
            "\"alerts\":%u,\"alert_n\":%u,\"low_v\":%s,\"low_soc\":%s,"
            "\"over_v\":%s,\"warn_mv\":%u,\"athd_pct\":%u,"
            // Charger configuration (0 = disabled for vindpm_mv and safety_h).
            "\"chg_disabled\":%s,"
            "\"cfg\":{\"ilim_ma\":%u,\"vindpm_mv\":%u,\"iterm_pct\":%u,"
            "\"buvlo_mv\":%u,\"ocp_ma\":%u,\"safety_h\":%u,\"treg_c\":%u,"
            "\"jeita\":{\"cold_c\":%d,\"cool_c\":%d,\"warm_c\":%d,\"hot_c\":%d,"
            "\"derate_pct\":%u},"
            "\"raw\":\"cc0=%02X cc1=%02X ic=%02X tmr=%02X ts=%02X\"},"
            // flag0 accumulated this boot; boot_flag0 latched before it
            // (bit 0 BAT_OCP, bit 1 BUVLO: why the board last went off).
            "\"flag0\":%u,\"boot_flag0\":%d}",
            ps.soc, (unsigned)ps.mv,
            ps.usb_present ? "true" : "false",
            ps.charging   ? "true" : "false",
            power_chg_state_str(ps.chg_state), rate_b,
            (unsigned)ps.ichg_ma, (unsigned)ps.vbatreg_mv,
            ps.thermreg ? "true" : "false",
            ps.gauge_reset ? "true" : "false",              // SOC still settling
            power_low_level(),
            (unsigned)ps.gauge_alerts, (unsigned)ps.alert_count,
            (ps.gauge_alerts & 0x04) ? "true" : "false",   // VL
            (ps.gauge_alerts & 0x10) ? "true" : "false",   // HD
            (ps.gauge_alerts & 0x02) ? "true" : "false",   // VH
            (unsigned)MAX17048_VALRT_MIN_MV, (unsigned)MAX17048_ATHD_PCT,
            ps.chg_disabled ? "true" : "false",
            (unsigned)ps.ilim_ma, (unsigned)ps.vindpm_mv,
            (unsigned)ps.iterm_pct, (unsigned)ps.buvlo_mv,
            (unsigned)ps.ibat_ocp_ma, (unsigned)ps.safety_timer_h,
            (unsigned)(ps.treg_known ? ps.treg_c : 0),
            (int)ps.ts_cold_c, (int)ps.ts_cool_c, (int)ps.ts_warm_c,
            (int)ps.ts_hot_c, (unsigned)ps.ts_derate_pct,
            ps.cc0_raw, ps.cc1_raw, ps.ic_ctrl_raw, ps.tmr_ilim_raw,
            ps.ts_ctrl_raw, (unsigned)ps.flag0_raw,
            ps.boot_flag0_valid ? (int)ps.boot_flag0 : -1);
    }

    // Controller counters cached by the gw task after each drive session.
    char gwd[200] = "null";
    struct gw_diag_counters dc;
    uint32_t dc_age = 0;
    if (gw_get_diag(&dc, &dc_age))
        snprintf(gwd, sizeof(gwd),
            "{\"age_ms\":%u,\"ovr\":%u,\"ferr\":%u,\"rt\":%u,\"seq\":%u,"
            "\"badcmd\":%u,\"badlen\":%u,\"arb_usb\":%u,\"arb_spi\":%u}",
            (unsigned)dc_age, (unsigned)dc.spi_rx_overruns,
            (unsigned)dc.spi_frame_errors, (unsigned)dc.spi_retransmits,
            (unsigned)dc.spi_seq_errs, (unsigned)dc.proto_bad_cmd,
            (unsigned)dc.proto_bad_len, (unsigned)dc.arb_to_usb,
            (unsigned)dc.arb_to_spi);

    // UTC epoch and its source. "boot"/"backup" are latched at boot: the only
    // evidence the RTC kept time through a power cycle (see wallclock.h).
    char tm_s[128];
    snprintf(tm_s, sizeof(tm_s),
             "{\"epoch\":%lld,\"src\":\"%s\",\"rtc\":%s,"
             "\"boot\":\"%s\",\"backup\":%u}",
             (long long)time(NULL), wallclock_source_str(),
             wallclock_rtc_present() ? "true" : "false",
             wallclock_boot_str(), (unsigned)wallclock_backup_reg());

    // Each source is optional; the battery is a JEITA zone, not degrees.
    temps_t tp;
    temps_get(&tp);
    char temp_s[176];
    {
        char esp_b[24] = "null", rp_b[64] = "null", bat_b[48] = "null";
        if (tp.esp_valid) snprintf(esp_b, sizeof(esp_b), "%.1f", tp.esp_c);
        if (tp.rp_valid)
            snprintf(rp_b, sizeof(rp_b), "{\"c\":%.1f,\"age_ms\":%u}",
                     tp.rp_c, (unsigned)tp.rp_age_ms);
        if (tp.batt_zone_valid)
            snprintf(bat_b, sizeof(bat_b), "{\"zone\":\"%s\",\"ts_open\":%s}",
                     power_ts_zone_str(tp.batt_zone),
                     tp.batt_ts_open ? "true" : "false");
        snprintf(temp_s, sizeof(temp_s),
                 "{\"esp_c\":%s,\"rp\":%s,\"batt\":%s}", esp_b, rp_b, bat_b);
    }

    // Worst case is ~2300 bytes (incl. lastoff); grow buf if any block above grows, since
    // truncation would emit malformed JSON. n is clamped below because
    // snprintf returns the untruncated length.
    const esp_app_desc_t *app = esp_app_get_description();
    char lastoff_s[300];
    lastoff_json(lastoff_s, sizeof lastoff_s);
    char wifipwr_s[140];
    wifi_pwr_json(wifipwr_s, sizeof wifipwr_s);
    // The op the gw task is running ("" = idle) and those waiting behind it.
    char queued_s[80] = "";
    {
        uint32_t qm = ui_queued_mask();
        size_t o = 0;
        for (int op = 0; op < UI_OP_N && o < sizeof(queued_s) - 12; op++)
            if (qm & (1u << op))
                o += snprintf(queued_s + o, sizeof(queued_s) - o, "%s\"%s\"",
                              o ? "," : "", ui_op_name(op));
    }
    char buf[3100];
    int n = snprintf(buf, sizeof(buf),
        "{\"serial\":\"%s\",\"state\":\"%s\",\"op\":\"%s\",\"queued\":[%s],\"disk_gen\":%u,\"done\":%u,\"total\":%u,\"bad\":%u,\"detail\":\"%s\","
        "\"link\":\"%s\",\"reset\":\"%s\",\"heap\":%u,\"heap_min\":%u,\"psram\":%u,\"ip\":\"%s\","
        "\"sd_cd\":%s,\"batt\":%s,\"gwdiag\":%s,\"time\":%s,\"temps\":%s,"
        "\"ota\":{\"slot\":\"%s\",\"pending\":%s,\"ver\":\"%s\",\"built\":\"%s %s\"},\"drive5v\":{\"on\":%s,\"usb\":%s,\"hold_s\":%u},\"lastoff\":%s,\"wifipwr\":%s,\"up_s\":%lu,\"autooff_s\":%d}",
        serialno_get(), state_str(disk_get_state()), ui_op_name(ui_running_op()),
        queued_s, (unsigned)disk_generation(),
        (unsigned)disk_sectors_done(),
        (unsigned)disk_sectors_total(), (unsigned)disk_bad_sector_count(),
        disk_get_detail(), link_stats_str(), reset_reason_str(),
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
        (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM), ip,
        sd_card_detected() ? "true" : "false", batt, gwd, tm_s, temp_s,
        ota_running_label(), ota_pending_verify() ? "true" : "false",
        app->version, app->date, app->time,
        power_boost_is_on() ? "true" : "false", gw_usb_active() ? "true" : "false",
        (unsigned)power_boost_hold_left(), lastoff_s, wifipwr_s,
        (unsigned long)(esp_timer_get_time() / 1000000), ui_autooff_left_s());
    if (n < 0) n = 0;
    else if (n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, n);
}

// GET /api/trackmap -> {"cyls":80,"heads":2,"map":"..."}, one char per track
// (cyl*heads+head): '0'=unread '1'=good '2'=retry '3'=bad.
static esp_err_t h_trackmap(httpd_req_t *req)
{
    const uint8_t *m = disk_trackmap();
    char buf[TOTAL_TRACKS + 48];
    int n = snprintf(buf, sizeof(buf), "{\"cyls\":%u,\"heads\":%u,\"map\":\"",
                     (unsigned)DISK_CYLS, (unsigned)DISK_HEADS);
    for (int i = 0; i < TOTAL_TRACKS && n < (int)sizeof(buf) - 3; i++)
        buf[n++] = (char)('0' + (m[i] & 3));
    buf[n++] = '"'; buf[n++] = '}';
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, n);
}

// GET /api/list -> [{"name":..,"size":..,"dirty":bool,"sha":"<64 hex>"}, ...]
// ("sha" added for floppy_screen; the content hash, same as the SD index.
// Left out if the file couldn't be hashed; the client then hashes it itself.)
// On a dirty disk the hash covers the image as read, bad sectors zero-filled.
static esp_err_t h_list(httpd_req_t *req)
{
    // Polled every second by the page: not activity (auto-off).
    httpd_resp_set_type(req, "application/json");
    if (!disk_volume_acquire())
        return httpd_resp_sendstr(req, "[]");
    // TODO: per-file dirty, by testing disk_sector_valid() over each file's
    // cluster chain.
    bool disk_dirty = (disk_get_state() == DISK_READY_DIRTY);

    FF_DIR dir; FILINFO fno;
    if (f_opendir(&dir, FAT12_ROOT) != FR_OK) {
        disk_volume_unlock();
        return httpd_resp_sendstr(req, "[]");
    }

    httpd_resp_sendstr_chunk(req, "[");
    bool first = true;
    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0]) {
        if (fno.fattrib & (AM_DIR | AM_SYS | AM_HID)) continue;
        char item[400], sha_kv[80] = "";
        uint8_t sha[SD_SHA_LEN];
        if (sd_sha_image_file_cached(fno.fname, (uint32_t)fno.fsize, sha)) {
            char hex[65];
            sd_sha_hex(sha, hex);
            snprintf(sha_kv, sizeof(sha_kv), ",\"sha\":\"%s\"", hex);
        }
        int n = snprintf(item, sizeof(item),
            "%s{\"name\":\"%s\",\"size\":%llu,\"dirty\":%s%s}",
            first ? "" : ",", fno.fname, (unsigned long long)fno.fsize,
            disk_dirty ? "true" : "false", sha_kv);
        if (n >= (int)sizeof(item)) n = (int)sizeof(item) - 1;
        httpd_resp_send_chunk(req, item, n);
        first = false;
    }
    f_closedir(&dir);
    disk_volume_unlock();
    httpd_resp_sendstr_chunk(req, "]");
    return httpd_resp_send_chunk(req, NULL, 0);
}

// GET /file?name=DSC00001.JPG -> raw bytes from the FAT
static esp_err_t h_file(httpd_req_t *req)
{
    ui_activity();      // a person is using the web page
    char query[300], name[256];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        query_str(query, "name", name, sizeof(name)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing name");
        return ESP_FAIL;
    }
    if (!web_name_is_safe(name)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad name");
        return ESP_FAIL;
    }

    if (!disk_volume_acquire()) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no disk mounted");
        return ESP_FAIL;
    }
    char path[300];
    snprintf(path, sizeof(path), "%s%s", FAT12_ROOT, name);
    FIL f;
    if (f_open(&f, path, FA_READ) != FR_OK) {
        disk_volume_unlock();
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no such file");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, web_mime_for(name));

    static uint8_t chunk[4096];
    UINT br;
    esp_err_t e = ESP_OK;
    while (f_read(&f, chunk, sizeof(chunk), &br) == FR_OK && br > 0) {
        if (httpd_resp_send_chunk(req, (const char *)chunk, br) != ESP_OK) {
            e = ESP_FAIL;
            break;
        }
    }
    f_close(&f);
    disk_volume_unlock();
    return e == ESP_OK ? httpd_resp_send_chunk(req, NULL, 0) : ESP_FAIL;
}

// GET /sdfile?path=DISK0007/MVC-00001.JPG -> raw bytes from the SD card.
static esp_err_t h_sdfile(httpd_req_t *req)
{
    ui_activity();      // a person is using the web page
    char query[360], path[220];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        query_str(query, "path", path, sizeof(path)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing path");
        return ESP_FAIL;
    }
    if (path[0] == '\0' || !sd_path_is_safe(path)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad path");
        return ESP_FAIL;
    }
    if (!sd_present()) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no SD card");
        return ESP_FAIL;
    }

    char full[300];
    snprintf(full, sizeof(full), "%s/%s", SD_MOUNT_POINT, path);
    FILE *f = fopen(full, "rb");
    if (!f) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no such file");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, web_mime_for(path));
    // Archived photos never change.
    httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=31536000");

    static uint8_t chunk[4096];
    size_t br;
    while ((br = fread(chunk, 1, sizeof(chunk), f)) > 0) {
        if (httpd_resp_send_chunk(req, (const char *)chunk, br) != ESP_OK) {
            fclose(f);
            return ESP_FAIL;
        }
    }
    fclose(f);
    return httpd_resp_send_chunk(req, NULL, 0);
}

// POST /api/delete?name=FOO.JPG -> delete from the image, then the gw task
// writes cyl0 (FAT + root dir) back to the floppy.
static esp_err_t h_delete(httpd_req_t *req)
{
    ui_activity();      // a person is using the web page
    char query[300], name[256];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        query_str(query, "name", name, sizeof(name)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing name");
        return ESP_FAIL;
    }
    if (!web_name_is_safe(name)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad name");
        return ESP_FAIL;
    }
    if (!disk_volume_acquire()) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no disk mounted");
        return ESP_FAIL;
    }
    char path[300];
    snprintf(path, sizeof(path), "%s%s", FAT12_ROOT, name);
    // If the commit can't write the disk (e.g. write-protected), commit_cyl0
    // rolls back to this.
    disk_image_checkpoint_cyl0();
    FRESULT fr = f_unlink(path);
    if (fr != FR_OK) {
        disk_volume_unlock();
        ESP_LOGW(TAG, "f_unlink(%s) failed: %d", name, fr);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "delete failed");
        return ESP_FAIL;
    }
    // Mavica cameras index the disk from a hidden .411 thumbnail per picture;
    // an orphaned one keeps the photo on the camera's index screen.
    sd_sha_image_cache_forget(name);
    char thumb[sizeof(name)];
    if (web_thumb_for(name, thumb, sizeof(thumb))) {
        sd_sha_image_cache_forget(thumb);
        snprintf(path, sizeof(path), "%s%s", FAT12_ROOT, thumb);
        FRESULT tr = f_unlink(path);
        if (tr == FR_OK)
            ESP_LOGI(TAG, "deleted thumbnail %s too", thumb);
        else if (tr != FR_NO_FILE)
            ESP_LOGW(TAG, "f_unlink(%s) failed: %d", path, tr);
    }
    disk_bump_generation();          // the list changed; polling clients resync
    disk_volume_unlock();
    ui_trigger_op(UI_OP_COMMIT);
    ESP_LOGI(TAG, "deleted %s; committing cyl0 to disk", name);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

// POST /api/charge?en=0|1 -> bench aid to toggle charging (see
// power_request_charge_enable()).
static esp_err_t h_charge(httpd_req_t *req)
{
    ui_activity();      // a person is using the web page
    char query[32], ens[8];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "en", ens, sizeof(ens)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "need ?en=0 or ?en=1");
        return ESP_FAIL;
    }
    power_state_t ps;
    if (!power_get(&ps) || !ps.charger_ok) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no charger on this board");
        return ESP_FAIL;
    }
    bool en = (ens[0] != '0');
    power_request_charge_enable(en);

    // Applied asynchronously, so report the request; /api/status confirms.
    httpd_resp_set_type(req, "application/json");
    char resp[160];
    snprintf(resp, sizeof(resp),
             "{\"ok\":true,\"requested\":\"%s\",\"ichg_ma\":%u,\"ilim_ma\":%u,"
             "\"note\":\"confirm via batt.chg_disabled; auto-reverts in 10 min\"}",
             en ? "enabled" : "disabled",
             (unsigned)ps.ichg_ma, (unsigned)ps.ilim_ma);
    return httpd_resp_sendstr(req, resp);
}

// POST /api/drive5v?secs=N -> hold the 5 V drive rail up for N s (0 = release,
// max 1800): bench metering, or gw.exe on a controller too old to report USB.
static esp_err_t h_drive5v(httpd_req_t *req)
{
    ui_activity();      // a person is using the web page
    char query[32], ss[12];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "secs", ss, sizeof(ss)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "need ?secs=N (0 = off, max 1800)");
        return ESP_FAIL;
    }
    unsigned long secs = strtoul(ss, NULL, 10);
    if (secs > 1800) secs = 1800;
    power_boost_hold((uint32_t)secs);
    char resp[64];
    snprintf(resp, sizeof(resp), "{\"ok\":true,\"on\":%s,\"hold_s\":%u}",
             power_boost_is_on() ? "true" : "false", (unsigned)power_boost_hold_left());
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, resp);
}

// POST /api/wifipwr?mode=0..3[&dbm=2..20] -> WiFi policy during drive
// sessions (wifi_pwr.h). Saved in NVS.
static esp_err_t h_wifipwr(httpd_req_t *req)
{
    ui_activity();      // a person is using the web page
    char query[48], ms[8], ds[8];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "mode", ms, sizeof(ms)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
            "need ?mode=0..3 (normal/lowtx/off/spinup) [&dbm=2..20]");
        return ESP_FAIL;
    }
    int dbm = wifi_pwr_low_dbm();
    if (httpd_query_key_value(query, "dbm", ds, sizeof(ds)) == ESP_OK)
        dbm = atoi(ds);
    if (!wifi_pwr_set(atoi(ms), dbm)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "mode 0..3, dbm 2..20");
        return ESP_FAIL;
    }
    char resp[160];
    wifi_pwr_json(resp, sizeof resp);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, resp);
}

// POST /api/rpflash?addr=0x10000000 with a raw .bin body -> reprogram the
// RP2354A over SWD (rp_flash.c). Runs on the httpd task for tens of seconds
// with the controller halted. addr is the XIP base for build/gw_rp2354.bin.
static esp_err_t h_rpflash(httpd_req_t *req)
{
    ui_activity();      // a person is using the web page
    char query[64], addrs[16];
    uint32_t addr = 0;
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "addr", addrs, sizeof(addrs)) == ESP_OK)
        addr = (uint32_t)strtoul(addrs, NULL, 0);
    if (!addr || req->content_len == 0 ||
        req->content_len > RP_FLASH_IMG_MAX) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "need ?addr=0x10xxxxxx and a .bin body <= 512 KB");
        return ESP_FAIL;
    }

    uint8_t *img = heap_caps_malloc(req->content_len, MALLOC_CAP_SPIRAM);
    if (!img) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no memory");
        return ESP_FAIL;
    }
    size_t got = req->content_len;
    if (!recv_all(req, img, got)) {
        free(img);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "body truncated");
        return ESP_FAIL;
    }

    // Claimed after the upload so a slow upload doesn't block the drive; 5 s
    // covers an idle relink in progress.
    if (!gw_claim(5000)) {
        free(img);
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"controller busy "
                                       "(disk read/write in progress) - try again after it finishes\"}");
    }
    char err[128];
    esp_err_t e = rp_flash_program(img, got, addr, err, sizeof(err));
    gw_release();
    free(img);

    httpd_resp_set_type(req, "application/json");
    char resp[224];
    if (e == ESP_OK) {
        snprintf(resp, sizeof(resp),
                 "{\"ok\":true,\"bytes\":%u,\"addr\":\"0x%08lX\"}",
                 (unsigned)got, (unsigned long)addr);
    } else {
        snprintf(resp, sizeof(resp), "{\"ok\":false,\"error\":\"%s\"}", err);
    }
    return httpd_resp_sendstr(req, resp);
}

// POST /api/sd/reindex -> rehash the whole card (slow). Needed after adding
// files from a PC; deletions self-heal on mount.
static esp_err_t h_sd_reindex(httpd_req_t *req)
{
    ui_activity();      // a person is using the web page
    if (!sd_present()) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no SD card");
        return ESP_FAIL;
    }
    int n = sd_reindex();
    httpd_resp_set_type(req, "application/json");
    char buf[96];
    int m = (n >= 0)
        ? snprintf(buf, sizeof(buf), "{\"ok\":true,\"files\":%d}", n)
        : snprintf(buf, sizeof(buf), "{\"ok\":false,\"error\":\"rebuild failed\"}");
    return httpd_resp_send(req, buf, m);
}

// POST /api/ota with a raw app .bin -> stream into the inactive slot and
// reboot. The new build must confirm itself or the bootloader rolls back.
static esp_err_t h_ota(httpd_req_t *req)
{
    ui_activity();      // a person is using the web page
    char err[128] = "";
    if (req->content_len == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "empty body");
        return ESP_FAIL;
    }
    if (ota_begin(req->content_len, err, sizeof(err)) != ESP_OK) {
        httpd_resp_set_type(req, "application/json");
        char resp[200];
        int n = snprintf(resp, sizeof(resp),
                         "{\"ok\":false,\"error\":\"%s\"}", err);
        return httpd_resp_send(req, resp, n);
    }

    static char buf[4096];
    size_t got = 0;
    while (got < req->content_len) {
        int n = recv_some(req, buf, sizeof(buf));
        if (n <= 0) {
            ota_abort();
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "upload truncated");
            return ESP_FAIL;
        }
        if (ota_write(buf, (size_t)n, err, sizeof(err)) != ESP_OK) {
            httpd_resp_set_type(req, "application/json");
            char resp[200];
            int m = snprintf(resp, sizeof(resp),
                             "{\"ok\":false,\"error\":\"%s\"}", err);
            return httpd_resp_send(req, resp, m);
        }
        got += (size_t)n;
    }

    esp_err_t e = ota_end(err, sizeof(err));
    httpd_resp_set_type(req, "application/json");
    char resp[224];
    int n;
    if (e == ESP_OK) {
        n = snprintf(resp, sizeof(resp),
                     "{\"ok\":true,\"bytes\":%u,\"reboot\":true}",
                     (unsigned)got);
    } else {
        n = snprintf(resp, sizeof(resp), "{\"ok\":false,\"error\":\"%s\"}", err);
    }
    esp_err_t sent = httpd_resp_send(req, resp, n);

    if (e == ESP_OK) {
        // Let the reply reach the browser first.
        ESP_LOGW(TAG, "rebooting into the new firmware");
        vTaskDelay(pdMS_TO_TICKS(1200));
        esp_restart();
    }
    return sent;
}

// Reply to a trigger: "queued" says it is waiting behind a running job.
static esp_err_t send_triggered(httpd_req_t *req, bool busy)
{
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, busy ? "{\"ok\":true,\"queued\":true}"
                                        : "{\"ok\":true,\"queued\":false}");
}

// POST /api/read -> trigger a disk read
static esp_err_t h_read(httpd_req_t *req)
{
    ui_activity();      // a person is using the web page
    bool busy = ui_running_op() >= 0;
    ui_trigger();
    return send_triggered(req, busy);
}

// POST /api/retry -> re-read only the tracks that came out bad, into the
// image already loaded.
static esp_err_t h_retry(httpd_req_t *req)
{
    ui_activity();      // a person is using the web page
    if (disk_get_state() != DISK_READY_DIRTY) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no partial disk to retry");
        return ESP_FAIL;
    }
    bool busy = ui_running_op() >= 0;
    ui_trigger_op(UI_OP_RETRY);
    return send_triggered(req, busy);
}

// POST /api/image/write with a raw 1.44 MB .img body -> write it to the floppy
// in the drive, every track read back and compared. With ?src=loaded and no
// body, writes the image already in memory instead (read a disk, swap disks,
// write). Progress and the result are in /api/status.
static esp_err_t h_image_write(httpd_req_t *req)
{
    ui_activity();      // a person is using the web page
    char query[32], src[12] = "";
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK)
        httpd_query_key_value(query, "src", src, sizeof(src));
    bool from_loaded = strcmp(src, "loaded") == 0;
    disk_state_t st = disk_get_state();

    if (st == DISK_READING) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"a disk job is running\"}");
    }
    if (from_loaded) {
        if (req->content_len != 0 || (st != DISK_READY && st != DISK_READY_DIRTY)) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                "src=loaded needs a loaded disk and no body");
            return ESP_FAIL;
        }
    } else {
        if (req->content_len != DISK_IMAGE_BYTES) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                "need a raw 1.44 MB image (1474560 bytes)");
            return ESP_FAIL;
        }
        uint8_t *img = heap_caps_malloc(DISK_IMAGE_BYTES, MALLOC_CAP_SPIRAM);
        if (!img) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no memory");
            return ESP_FAIL;
        }
        if (!recv_all(req, img, DISK_IMAGE_BYTES)) {
            heap_caps_free(img);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "body truncated");
            return ESP_FAIL;
        }
        disk_stage_set(img);
    }
    bool busy = ui_running_op() >= 0;
    ui_trigger_op(UI_OP_WRITE_DISK);
    ESP_LOGW(TAG, "whole-disk write requested (%s image)",
             from_loaded ? "loaded" : "uploaded");
    return send_triggered(req, busy);
}

// POST /api/wtest -> write self-test: rewrite cyl0 unchanged and report in
// /api/status "detail".
static esp_err_t h_wtest(httpd_req_t *req)
{
    ui_activity();      // a person is using the web page
    if (disk_get_state() != DISK_READY && disk_get_state() != DISK_READY_DIRTY) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no disk loaded");
        return ESP_FAIL;
    }
    ui_trigger_op(UI_OP_WTEST);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

// GET /api/sd[?path=DISK0007] -> {"present","total","free","path",
// "files":[{name,size,dir}]}. No path = the card root.
static esp_err_t h_sd(httpd_req_t *req)
{
    // Polled every second by the page: not activity (auto-off).
    char query[200], path[160] = "";
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK)
        (void)query_str(query, "path", path, sizeof(path));

    httpd_resp_set_type(req, "application/json");
    if (!sd_present())
        return httpd_resp_sendstr(req,
            "{\"present\":false,\"total\":0,\"free\":0,\"files\":[]}");
    if (!sd_path_is_safe(path)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad path");
        return ESP_FAIL;
    }

    uint64_t total = 0, freeb = 0;
    sd_usage(&total, &freeb);
    char head[320];
    int n = snprintf(head, sizeof(head),
        "{\"present\":true,\"total\":%llu,\"free\":%llu,\"path\":\"%s\",\"files\":[",
        (unsigned long long)total, (unsigned long long)freeb, path);
    httpd_resp_send_chunk(req, head, n);

    // A page at a time: the root can hold thousands of DISKnnnn folders.
    static sd_entry_t ents[64];
    uint32_t cursor = 0;
    bool first = true;
    int cnt;
    while ((cnt = sd_list_path(path, &cursor, ents, 64)) > 0) {
        for (int i = 0; i < cnt; i++) {
            char item[128];
            int m = snprintf(item, sizeof(item),
                             "%s{\"name\":\"%s\",\"size\":%u,\"dir\":%s}",
                             first ? "" : ",", ents[i].name, (unsigned)ents[i].size,
                             ents[i].is_dir ? "true" : "false");
            first = false;
            if (httpd_resp_send_chunk(req, item, m) != ESP_OK) return ESP_FAIL;
        }
    }
    httpd_resp_sendstr_chunk(req, "]}");
    return httpd_resp_send_chunk(req, NULL, 0);
}

// SD saves need a card and a mounted disk image. On success the volume lock is
// held; the caller releases it.
static bool sd_save_ready(httpd_req_t *req)
{
    if (!sd_present()) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no SD card");
        return false;
    }
    if (!disk_volume_acquire()) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no disk mounted");
        return false;
    }
    return true;
}

// POST /api/sd/save?name=FOO.JPG -> copy one file from the image to the SD card.
static esp_err_t h_sd_save(httpd_req_t *req)
{
    ui_activity();      // a person is using the web page
    char query[300], name[256];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        query_str(query, "name", name, sizeof(name)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing name");
        return ESP_FAIL;
    }
    if (!web_name_is_safe(name)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad name");
        return ESP_FAIL;
    }
    if (!sd_save_ready(req)) return ESP_FAIL;
    long r = sd_copy_from_image(name);
    disk_volume_unlock();
    httpd_resp_set_type(req, "application/json");
    if (r < 0) {
        char e[48];
        int n = snprintf(e, sizeof(e), "{\"ok\":false,\"err\":%ld}", r);
        return httpd_resp_send(req, e, n);
    }
    display_note_saved(name);
    char ok[48];
    int n = snprintf(ok, sizeof(ok), "{\"ok\":true,\"bytes\":%ld}", r);
    return httpd_resp_send(req, ok, n);
}

// POST /api/sd/save-all[?force=1] -> archive the disk into a new DISKnnnn
// folder (see sd_save_disk()).
static esp_err_t h_sd_save_all(httpd_req_t *req)
{
    ui_activity();      // a person is using the web page
    char query[48], forces[8];
    bool force = false;
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "force", forces, sizeof(forces)) == ESP_OK)
        force = (forces[0] == '1' || forces[0] == 't');

    if (!sd_save_ready(req)) return ESP_FAIL;
    sd_save_result_t r;
    sd_save_status_t st = sd_save_disk(force, &r);
    disk_volume_unlock();
    ESP_LOGI(TAG, "save-all: httpd stack headroom %u B",
             (unsigned)uxTaskGetStackHighWaterMark(NULL));

    char buf[160];
    int n;
    if (st == SD_SAVE_OK) {
        display_note_saved(r.dir);
        n = snprintf(buf, sizeof(buf),
                     "{\"ok\":%d,\"fail\":%d,\"skipped\":%d,\"dir\":\"%s\"}",
                     r.copied, r.failed, r.skipped, r.dir);
    } else if (st == SD_SAVE_DUPLICATE) {
        n = snprintf(buf, sizeof(buf),
                     "{\"ok\":0,\"fail\":0,\"duplicate\":true,\"dir\":\"%s\","
                     "\"error\":\"already archived as %s\"}", r.dir, r.dir);
    } else {
        static const char *why[] = {
            [SD_SAVE_NO_CARD]    = "no SD card",
            [SD_SAVE_NO_IMAGE]   = "no disk loaded",
            [SD_SAVE_MKDIR_FAIL] = "could not create folder",
            [SD_SAVE_CARD_FULL]  = "DISK9999 reached",
            [SD_SAVE_NO_SPACE]   = "not enough space on the SD card",
        };
        const char *msg = (st < sizeof(why) / sizeof(why[0]) && why[st])
                              ? why[st] : "save failed";
        n = snprintf(buf, sizeof(buf), "{\"ok\":0,\"fail\":0,\"error\":\"%s\"}",
                     msg);
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, n);
}

// POST /api/serial?sn=GW4-0001[&force=1] -> store the board serial in the RTC
// EEPROM. Won't overwrite a different number unless forced.
static esp_err_t h_serial(httpd_req_t *req)
{
    ui_activity();      // a person is using the web page
    char query[96], sn[SERIALNO_MAX + 2] = "", force[4] = "";
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        // Oversized values truncate to 33 chars and then fail validation.
        query_str(query, "sn", sn, sizeof(sn));
        httpd_query_key_value(query, "force", force, sizeof(force));
    }
    char err[96] = "";
    esp_err_t e = serialno_set(sn, force[0] == '1', err, sizeof(err));
    char resp[160];
    if (e == ESP_OK) {
        snprintf(resp, sizeof(resp), "{\"ok\":true,\"serial\":\"%s\"}", serialno_get());
    } else {
        if (e == ESP_ERR_INVALID_STATE) httpd_resp_set_status(req, "409 Conflict");
        else if (e == ESP_ERR_INVALID_ARG) httpd_resp_set_status(req, "400 Bad Request");
        else httpd_resp_set_status(req, "500 Internal Server Error");
        snprintf(resp, sizeof(resp), "{\"ok\":false,\"error\":\"%s\"}", err);
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, resp);
}

// POST /api/wifi/forget -> erase WiFi creds and reboot into the setup portal.
static esp_err_t h_wifi_forget(httpd_req_t *req)
{
    ui_activity();      // a person is using the web page
    provision_clear_creds();
    ESP_LOGW(TAG, "WiFi credentials cleared via web — rebooting into setup portal");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    vTaskDelay(pdMS_TO_TICKS(800));
    esp_restart();
    return ESP_OK;
}

static void mdns_start(void)
{
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(MDNS_HOSTNAME));       // -> floppy.local
    ESP_ERROR_CHECK(mdns_instance_name_set(MDNS_INSTANCE));
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    ESP_LOGI(TAG, "mDNS: http://%s.local/", MDNS_HOSTNAME);
}

void web_start(void)
{
    mdns_start();

    // max_uri_handlers is derived from this table.
    static const httpd_uri_t routes[] = {
        { .uri = "/",            .method = HTTP_GET,  .handler = h_index  },
        { .uri = "/api/status",  .method = HTTP_GET,  .handler = h_status },
        { .uri = "/api/trackmap",.method = HTTP_GET,  .handler = h_trackmap },
        { .uri = "/api/list",    .method = HTTP_GET,  .handler = h_list   },
        { .uri = "/api/read",    .method = HTTP_POST, .handler = h_read   },
        { .uri = "/api/delete",  .method = HTTP_POST, .handler = h_delete },
        { .uri = "/api/wtest",   .method = HTTP_POST, .handler = h_wtest  },
        { .uri = "/api/retry",   .method = HTTP_POST, .handler = h_retry  },
        { .uri = "/api/image/write", .method = HTTP_POST, .handler = h_image_write },
        { .uri = "/api/sd",      .method = HTTP_GET,  .handler = h_sd     },
        { .uri = "/api/sd/save", .method = HTTP_POST, .handler = h_sd_save },
        { .uri = "/api/sd/save-all", .method = HTTP_POST, .handler = h_sd_save_all },
        { .uri = "/api/sd/reindex", .method = HTTP_POST, .handler = h_sd_reindex },
        { .uri = "/api/wifi/forget", .method = HTTP_POST, .handler = h_wifi_forget },
        { .uri = "/api/charge",  .method = HTTP_POST, .handler = h_charge  },
        { .uri = "/api/drive5v", .method = HTTP_POST, .handler = h_drive5v },
        { .uri = "/api/wifipwr", .method = HTTP_POST, .handler = h_wifipwr },
        { .uri = "/api/rpflash", .method = HTTP_POST, .handler = h_rpflash },
        { .uri = "/api/ota",     .method = HTTP_POST, .handler = h_ota     },
        { .uri = "/api/serial",  .method = HTTP_POST, .handler = h_serial  },
        { .uri = "/file",        .method = HTTP_GET,  .handler = h_file   },
        { .uri = "/sdfile",      .method = HTTP_GET,  .handler = h_sdfile },
    };
    const size_t nroutes = sizeof(routes) / sizeof(routes[0]);

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.core_id = 0;                 // gw/decode run on core 1
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.max_uri_handlers = nroutes;
    // 4 KB overflowed in h_file and 8 KB in h_sd_save_all (paths, FIL, VFS,
    // FatFs and the SD driver). Headroom is logged after every save.
    cfg.stack_size = 12288;

    httpd_handle_t srv = NULL;
    ESP_ERROR_CHECK(httpd_start(&srv, &cfg));

    // A dropped route only shows up as a 404, so log it.
    for (size_t i = 0; i < nroutes; i++) {
        esp_err_t e = httpd_register_uri_handler(srv, &routes[i]);
        if (e != ESP_OK)
            ESP_LOGE(TAG, "route %s not registered: %s", routes[i].uri, esp_err_to_name(e));
    }

    ESP_LOGI(TAG, "HTTP server up");
}
