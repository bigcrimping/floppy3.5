#include "lastoff.h"
#include "disk_image.h"
#include "power.h"
#include "wifi_pwr.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

static const char *TAG = "lastoff";

#define LASTOFF_NS      "lastoff"
#define LASTOFF_KEY     "rec"
#define LASTOFF_MAGIC   0x4C4F           // "LO"
#define LASTOFF_VER     3
// Each save is a short flash write (cache off for a few ms), so keep them
// sparse: a full read is a minute or two, which gives ~6-12 points.
#define LASTOFF_SAVE_MS 10000
#define LASTOFF_SAMPLE_MS 1000

enum { C_NONE = 0, C_SESSION = 1, C_KILL = 2, C_AUTO = 3 };
static bool s_auto_kill;     // the next KILL is the idle auto-off

typedef struct {
    uint16_t magic;
    uint8_t  ver;
    uint8_t  cause;       // C_SESSION while open, C_KILL after a KILL, C_NONE closed
    uint32_t epoch;       // wall-clock time the session opened (0 = clock not set)
    uint32_t secs;        // seconds into the session at the last save
    uint16_t mv_start;    // cell mV when the session opened
    uint16_t mv_min;      // lowest cell mV seen (gauge, ~250 ms+ resolution)
    uint16_t mv_last;     // cell mV at the last save
    uint16_t done;        // sectors decoded at the last save
    uint16_t saves;       // number of saves this session (shows how long it lived)
    uint8_t  disk_state;  // disk_state_t at the last save
    uint8_t  usb;         // USB power present at the last save
    uint8_t  wifi_mode;   // wifi_pwr policy for this session
    uint8_t  low_dbm;     // its LOWTX cap
    int8_t   tx_dbm;      // live TX power at the last sample (-1 = radio off)
    int8_t   rssi;        // AP RSSI at the last sample with the radio on
    uint8_t  radio_on;    // radio running at the last sample
    uint8_t  phase;       // last LO_PH_* reached
    uint32_t phase_ms;    // ms after the session opened that it was reached
} rec_t;

static volatile uint8_t s_phase_req;      // set from the gw task
static volatile int64_t s_phase_us;

static rec_t   s_prev;          // previous boot's record, published read-only
static bool    s_prev_valid;
static rec_t   s_cur;           // live record while the drive rail is on
static bool    s_open;
static int64_t s_t0_us, s_last_save_us, s_last_sample_us;
static bool    s_dirty;

static void save(void)
{
    nvs_handle_t h;
    if (nvs_open(LASTOFF_NS, NVS_READWRITE, &h) != ESP_OK) return;
    if (nvs_set_blob(h, LASTOFF_KEY, &s_cur, sizeof s_cur) == ESP_OK)
        nvs_commit(h);
    nvs_close(h);
    s_last_save_us = esp_timer_get_time();
    s_dirty = false;
}

static const char *cause_str(uint8_t c)
{
    switch (c) {
    case C_SESSION: return "lost_in_session";
    case C_KILL:    return "kill";
    case C_AUTO:    return "auto_off";
    default:        return "none";
    }
}

static const char *state_str(uint8_t s)
{
    switch ((disk_state_t)s) {
    case DISK_EMPTY:       return "empty";
    case DISK_READING:     return "reading";
    case DISK_READY:       return "ready";
    case DISK_READY_DIRTY: return "dirty";
    case DISK_ERROR:       return "error";
    default:               return "?";
    }
}

static const char *phase_str(uint8_t p)
{
    static const char *const n[] = { "none", "boost_on", "motor_cmd", "motor_ack",
                                      "spun_up", "radio_on", "tracks" };
    return p < sizeof n / sizeof n[0] ? n[p] : "?";
}

void lastoff_boot(void)
{
    nvs_handle_t h;
    if (nvs_open(LASTOFF_NS, NVS_READWRITE, &h) != ESP_OK) return;
    size_t len = sizeof s_prev;
    if (nvs_get_blob(h, LASTOFF_KEY, &s_prev, &len) == ESP_OK &&
        len == sizeof s_prev && s_prev.magic == LASTOFF_MAGIC &&
        s_prev.ver == LASTOFF_VER) {
        s_prev_valid = true;
        if (s_prev.cause == C_SESSION)
            ESP_LOGE(TAG, "POWER LOST with the drive rail on: %s, %lus in, %u sectors, "
                     "cell %u -> min %u / last %u mV, usb=%u",
                     state_str(s_prev.disk_state), (unsigned long)s_prev.secs,
                     s_prev.done, s_prev.mv_start, s_prev.mv_min, s_prev.mv_last,
                     s_prev.usb);
        else
            ESP_LOGI(TAG, "last power-off: %s", cause_str(s_prev.cause));
        nvs_erase_key(h, LASTOFF_KEY);
        nvs_commit(h);
    }
    nvs_close(h);
}

static uint16_t cell_mv(bool *usb)
{
    power_state_t ps;
    if (!power_get(&ps)) { *usb = false; return 0; }
    *usb = ps.usb_present;
    return ps.mv;
}

void lastoff_tick(void)
{
    int64_t now = esp_timer_get_time();
    bool on = power_boost_is_on();

    if (on && !s_open) {
        bool usb;
        memset(&s_cur, 0, sizeof s_cur);
        s_cur.magic = LASTOFF_MAGIC;
        s_cur.ver   = LASTOFF_VER;
        s_cur.cause = C_SESSION;
        time_t t = time(NULL);
        s_cur.epoch = t > 1600000000 ? (uint32_t)t : 0;
        s_cur.mv_start = s_cur.mv_min = s_cur.mv_last = cell_mv(&usb);
        s_cur.usb = usb;
        s_cur.disk_state = (uint8_t)disk_get_state();
        s_cur.wifi_mode = (uint8_t)wifi_pwr_mode();
        s_cur.low_dbm = (uint8_t)wifi_pwr_low_dbm();
        s_cur.tx_dbm = -1;
        s_open = true;
        s_t0_us = s_last_sample_us = now;
        s_cur.radio_on = !wifi_pwr_radio_paused();
        s_cur.tx_dbm = (int8_t)wifi_pwr_tx_dbm();
        save();
        return;
    }
    if (!on && s_open) {
        s_open = false;
        s_phase_req = LO_PH_NONE;
        s_cur.cause = C_NONE;       // closed cleanly
        save();
        return;
    }
    if (!s_open) return;

    uint8_t ph = s_phase_req;
    if (ph != s_cur.phase) {
        s_cur.phase = ph;
        int64_t d = s_phase_us - s_t0_us;
        s_cur.phase_ms = d > 0 ? (uint32_t)(d / 1000) : 0;
        s_cur.radio_on = !wifi_pwr_radio_paused();
        s_cur.tx_dbm = (int8_t)wifi_pwr_tx_dbm();
        s_cur.secs = (uint32_t)((now - s_t0_us) / 1000000);
        save();                             // at once: this is the point of it
    }

    if (now - s_last_sample_us >= LASTOFF_SAMPLE_MS * 1000LL) {
        s_last_sample_us = now;
        bool usb;
        uint16_t mv = cell_mv(&usb);
        if (mv) {
            s_cur.mv_last = mv;
            if (!s_cur.mv_min || mv < s_cur.mv_min) { s_cur.mv_min = mv; s_dirty = true; }
        }
        if (usb != s_cur.usb) { s_cur.usb = usb; s_dirty = true; }
        int tx = wifi_pwr_tx_dbm(), rssi = wifi_pwr_rssi();
        uint8_t ron = !wifi_pwr_radio_paused();
        if (tx != s_cur.tx_dbm || ron != s_cur.radio_on) {
            s_cur.tx_dbm = (int8_t)tx; s_cur.radio_on = ron; s_dirty = true;
        }
        if (rssi) s_cur.rssi = (int8_t)rssi;
        uint16_t done = (uint16_t)disk_sectors_done();
        if (done && s_phase_req < LO_PH_TRACKS) lastoff_phase(LO_PH_TRACKS);
        uint8_t  ds   = (uint8_t)disk_get_state();
        if (done != s_cur.done || ds != s_cur.disk_state) {
            s_cur.done = done; s_cur.disk_state = ds; s_dirty = true;
        }
    }
    if (s_dirty && now - s_last_save_us >= LASTOFF_SAVE_MS * 1000LL) {
        s_cur.secs = (uint32_t)((now - s_t0_us) / 1000000);
        s_cur.saves++;
        save();
    }
}

void lastoff_phase(int phase)
{
    s_phase_us = esp_timer_get_time();
    s_phase_req = (uint8_t)phase;
}

void lastoff_mark_kill(void)
{
    if (!s_open) {
        memset(&s_cur, 0, sizeof s_cur);
        s_cur.magic = LASTOFF_MAGIC;
        s_cur.ver   = LASTOFF_VER;
        bool usb;
        s_cur.mv_last = s_cur.mv_start = s_cur.mv_min = cell_mv(&usb);
        s_cur.usb = usb;
        s_cur.disk_state = (uint8_t)disk_get_state();
    } else {
        s_cur.secs = (uint32_t)((esp_timer_get_time() - s_t0_us) / 1000000);
    }
    s_cur.cause = s_auto_kill ? C_AUTO : C_KILL;
    save();
}

void lastoff_json(char *buf, size_t len)
{
    if (!s_prev_valid) { snprintf(buf, len, "{\"cause\":\"none\"}"); return; }
    snprintf(buf, len,
             "{\"cause\":\"%s\",\"state\":\"%s\",\"secs\":%lu,\"done\":%u,"
             "\"mv_start\":%u,\"mv_min\":%u,\"mv_last\":%u,\"usb\":%s,"
             "\"epoch\":%lu,\"saves\":%u,\"wifi_mode\":%u,\"low_dbm\":%u,"
             "\"tx_dbm\":%d,\"rssi\":%d,\"radio_on\":%s,\"phase\":\"%s\",\"phase_ms\":%lu}",
             cause_str(s_prev.cause), state_str(s_prev.disk_state),
             (unsigned long)s_prev.secs, s_prev.done, s_prev.mv_start,
             s_prev.mv_min, s_prev.mv_last, s_prev.usb ? "true" : "false",
             (unsigned long)s_prev.epoch, s_prev.saves, s_prev.wifi_mode,
             s_prev.low_dbm, s_prev.tx_dbm, s_prev.rssi,
             s_prev.radio_on ? "true" : "false", phase_str(s_prev.phase),
             (unsigned long)s_prev.phase_ms);
}

const char *lastoff_prev_short(void)
{
    if (!s_prev_valid) return "NORMAL";
    switch (s_prev.cause) {
    case C_SESSION: return "LOST IN DRIVE USE";
    case C_KILL:    return "POWER BUTTON";
    case C_AUTO:    return "AUTO OFF (IDLE)";
    default:        return "NORMAL";
    }
}

void lastoff_set_auto(void) { s_auto_kill = true; }
