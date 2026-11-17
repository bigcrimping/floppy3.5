#include "wifi_pwr.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs.h"
#include <stdio.h>

static const char *TAG = "wifipwr";

#define NS        "wifipwr"
#define FULL_QDBM 80            // 20 dBm, the sdkconfig maximum (0.25 dBm units)
#define DEF_LOW   11            // dBm for LOWTX
#define MIN_DBM   2
#define MAX_DBM   20

static int  s_mode = WIFIPWR_LOWTX;   // default: mode 0 browned out 3/3 at spin-up (2026-11-17)
static int  s_low_dbm = DEF_LOW;
static bool s_sta;              // STA runtime (not the portal), radio started
static volatile bool s_paused;  // we stopped the radio
static bool s_in_session;
static uint32_t s_pauses;       // radio stops since boot

static const char *mode_str(int m)
{
    static const char *const n[WIFIPWR_N] = { "normal", "lowtx", "off", "spinup" };
    return (m >= 0 && m < WIFIPWR_N) ? n[m] : "?";
}

void wifi_pwr_init(void)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) return;
    int8_t m = 0, d = DEF_LOW;
    if (nvs_get_i8(h, "mode", &m) == ESP_OK && m >= 0 && m < WIFIPWR_N) s_mode = m;
    if (nvs_get_i8(h, "dbm", &d) == ESP_OK && d >= MIN_DBM && d <= MAX_DBM) s_low_dbm = d;
    nvs_close(h);
    ESP_LOGI(TAG, "policy %s (low tx %d dBm)", mode_str(s_mode), s_low_dbm);
}

void wifi_pwr_sta_ready(void) { s_sta = true; }

int  wifi_pwr_mode(void)    { return s_mode; }
int  wifi_pwr_low_dbm(void) { return s_low_dbm; }
bool wifi_pwr_radio_paused(void) { return s_paused; }

bool wifi_pwr_set(int mode, int low_dbm)
{
    if (mode < 0 || mode >= WIFIPWR_N) return false;
    if (low_dbm < MIN_DBM || low_dbm > MAX_DBM) return false;
    s_mode = mode;
    s_low_dbm = low_dbm;
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i8(h, "mode", (int8_t)mode);
        nvs_set_i8(h, "dbm", (int8_t)low_dbm);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGI(TAG, "policy set: %s (low tx %d dBm)", mode_str(mode), low_dbm);
    return true;
}

static void radio_stop(void)
{
    if (s_paused) return;
    s_paused = true;            // before the stop, so DISCONNECTED doesn't reconnect
    s_pauses++;
    esp_err_t e = esp_wifi_stop();
    ESP_LOGI(TAG, "radio off for drive (%s)", esp_err_to_name(e));
}

static void radio_start(void)
{
    if (!s_paused) return;
    esp_err_t e = esp_wifi_start();    // STA_START -> esp_wifi_connect()
    s_paused = false;
    esp_wifi_set_max_tx_power(FULL_QDBM);
    ESP_LOGI(TAG, "radio back on (%s)", esp_err_to_name(e));
}

void wifi_pwr_drive_begin(void)
{
    if (!s_sta || s_in_session) return;
    s_in_session = true;
    switch (s_mode) {
    case WIFIPWR_LOWTX:
        esp_wifi_set_max_tx_power((int8_t)(s_low_dbm * 4));
        break;
    case WIFIPWR_OFF:
    case WIFIPWR_SPINUP:
        radio_stop();
        break;
    default: break;
    }
}

void wifi_pwr_spinup_done(void)
{
    if (!s_sta || !s_in_session) return;
    if (s_mode == WIFIPWR_SPINUP) radio_start();
}

void wifi_pwr_drive_end(void)
{
    if (!s_sta || !s_in_session) return;
    s_in_session = false;
    if (s_paused) radio_start();
    else esp_wifi_set_max_tx_power(FULL_QDBM);
}

int wifi_pwr_tx_dbm(void)
{
    if (!s_sta || s_paused) return -1;
    int8_t q;
    if (esp_wifi_get_max_tx_power(&q) != ESP_OK) return -1;
    return q / 4;
}

int wifi_pwr_rssi(void)
{
    if (!s_sta || s_paused) return 0;
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) return 0;
    return ap.rssi;
}

void wifi_pwr_json(char *buf, size_t len)
{
    snprintf(buf, len,
             "{\"mode\":\"%s\",\"mode_n\":%d,\"low_dbm\":%d,\"tx_dbm\":%d,"
             "\"rssi\":%d,\"paused\":%s,\"pauses\":%lu}",
             mode_str(s_mode), s_mode, s_low_dbm, wifi_pwr_tx_dbm(),
             wifi_pwr_rssi(), s_paused ? "true" : "false",
             (unsigned long)s_pauses);
}

bool wifi_pwr_ap(char *ssid, size_t n, int *rssi, int *channel)
{
    if (!s_sta || s_paused) return false;
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) return false;
    snprintf(ssid, n, "%s", (const char *)ap.ssid);
    if (rssi) *rssi = ap.rssi;
    if (channel) *channel = ap.primary;
    return true;
}
