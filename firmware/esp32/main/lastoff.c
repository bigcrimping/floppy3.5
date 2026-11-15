#include "lastoff.h"
#include "disk_image.h"
#include "power.h"
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
#define LASTOFF_VER     1
// Each save is a short flash write (cache off for a few ms), so keep them
// sparse: a full read is a minute or two, which gives ~6-12 points.
#define LASTOFF_SAVE_MS 10000
#define LASTOFF_SAMPLE_MS 1000

enum { C_NONE = 0, C_SESSION = 1, C_KILL = 2 };

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
} rec_t;

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
        s_open = true;
        s_t0_us = s_last_sample_us = now;
        save();
        return;
    }
    if (!on && s_open) {
        s_open = false;
        s_cur.cause = C_NONE;       // closed cleanly
        save();
        return;
    }
    if (!s_open) return;

    if (now - s_last_sample_us >= LASTOFF_SAMPLE_MS * 1000LL) {
        s_last_sample_us = now;
        bool usb;
        uint16_t mv = cell_mv(&usb);
        if (mv) {
            s_cur.mv_last = mv;
            if (!s_cur.mv_min || mv < s_cur.mv_min) { s_cur.mv_min = mv; s_dirty = true; }
        }
        if (usb != s_cur.usb) { s_cur.usb = usb; s_dirty = true; }
        uint16_t done = (uint16_t)disk_sectors_done();
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
    s_cur.cause = C_KILL;
    save();
}

void lastoff_json(char *buf, size_t len)
{
    if (!s_prev_valid) { snprintf(buf, len, "{\"cause\":\"none\"}"); return; }
    snprintf(buf, len,
             "{\"cause\":\"%s\",\"state\":\"%s\",\"secs\":%lu,\"done\":%u,"
             "\"mv_start\":%u,\"mv_min\":%u,\"mv_last\":%u,\"usb\":%s,"
             "\"epoch\":%lu,\"saves\":%u}",
             cause_str(s_prev.cause), state_str(s_prev.disk_state),
             (unsigned long)s_prev.secs, s_prev.done, s_prev.mv_start,
             s_prev.mv_min, s_prev.mv_last, s_prev.usb ? "true" : "false",
             (unsigned long)s_prev.epoch, s_prev.saves);
}
