#include "provision.h"
#include "app_config.h"
#include "serialno.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs.h"
#include "lwip/sockets.h"

static const char *TAG = "prov";

#define NVS_WIFI_NS  "wifi"
#define AP_IP        "192.168.4.1"          // default esp_netif SoftAP gateway

// ---------------- NVS credential store ----------------

bool provision_have_creds(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_WIFI_NS, NVS_READONLY, &h) != ESP_OK) return false;
    size_t len = 0;
    esp_err_t e = nvs_get_str(h, "ssid", NULL, &len);
    nvs_close(h);
    return e == ESP_OK && len > 1;          // len includes the NUL
}

void provision_clear_creds(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_WIFI_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_erase_key(h, "ssid");
    nvs_erase_key(h, "pass");
    nvs_commit(h);
    nvs_close(h);
}

static esp_err_t save_creds(const char *ssid, const char *pass)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_WIFI_NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    if (e == ESP_OK) e = nvs_set_str(h, "ssid", ssid);
    if (e == ESP_OK) e = nvs_set_str(h, "pass", pass);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e;
}

// ---------------- form parsing ----------------

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

// Decode application/x-www-form-urlencoded in place (%XX and '+').
static void urldecode(char *s)
{
    char *o = s;
    for (char *p = s; *p; p++) {
        if (*p == '%' && hexval(p[1]) >= 0 && hexval(p[2]) >= 0) {
            *o++ = (char)((hexval(p[1]) << 4) | hexval(p[2]));
            p += 2;
        } else if (*p == '+') {
            *o++ = ' ';
        } else {
            *o++ = *p;
        }
    }
    *o = '\0';
}

// Extract `key` from a urlencoded body ("a=1&b=2") into `out` (decoded).
static bool form_field(const char *body, const char *key, char *out, size_t outsz)
{
    size_t klen = strlen(key);
    for (const char *p = body; p && *p;) {
        if (strncmp(p, key, klen) == 0 && p[klen] == '=') {
            const char *v = p + klen + 1;
            const char *end = strchr(v, '&');
            size_t len = end ? (size_t)(end - v) : strlen(v);
            if (len >= outsz) len = outsz - 1;
            memcpy(out, v, len);
            out[len] = '\0';
            urldecode(out);
            return true;
        }
        p = strchr(p, '&');
        if (p) p++;
    }
    return false;
}

// ---------------- HTTP handlers ----------------

// www/setup.html, embedded by CMake; self-contained (the SoftAP has no
// internet).
extern const uint8_t setup_html_start[] asm("_binary_setup_html_start");
extern const uint8_t setup_html_end[]   asm("_binary_setup_html_end");

static esp_err_t h_portal(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)setup_html_start,
                           setup_html_end - setup_html_start - 1);
}

// ---------------- network scan (for the tuner list) ----------------
//
// APSTA only so the STA side can scan. One scan at portal start; ?fresh=1
// rescans (hopping channels for ~2 s, which the joined phone rides out).

#define SCAN_MAX 20
static wifi_ap_record_t s_aps[SCAN_MAX];
static uint16_t s_nap;

static void do_scan(void)
{
    uint16_t n = SCAN_MAX;
    if (esp_wifi_scan_start(NULL, true) != ESP_OK ||
        esp_wifi_scan_get_ap_records(&n, s_aps) != ESP_OK) {
        ESP_LOGW(TAG, "WiFi scan failed");
        n = 0;
    }
    s_nap = n;
    ESP_LOGI(TAG, "scan: %u networks", (unsigned)n);
}

// SSIDs are arbitrary bytes: escape what JSON cannot carry raw.
static void json_str(char *o, size_t cap, const uint8_t *in)
{
    size_t k = 0;
    for (; *in && k + 7 < cap; in++) {
        uint8_t c = *in;
        if (c == '"' || c == '\\') { o[k++] = '\\'; o[k++] = (char)c; }
        else if (c < 0x20) k += snprintf(o + k, cap - k, "\\u%04x", c);
        else o[k++] = (char)c;
    }
    o[k] = '\0';
}

static esp_err_t h_scan(httpd_req_t *req)
{
    char q[16], v[4];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK &&
        httpd_query_key_value(q, "fresh", v, sizeof(v)) == ESP_OK && v[0] == '1')
        do_scan();

    // Worst case per record: 32 SSID bytes escaped to 6 each + ~64 of framing.
    size_t cap = 96 + (size_t)s_nap * (32 * 6 + 64);
    char *out = malloc(cap);
    if (!out) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no memory");
        return ESP_FAIL;
    }
    size_t k = snprintf(out, cap, "{\"sn\":\"%s\",\"host\":\"%s\",\"nets\":[",
                        serialno_get(), MDNS_HOSTNAME);
    for (uint16_t i = 0; i < s_nap; i++) {
        char name[32 * 6 + 1];
        json_str(name, sizeof(name), s_aps[i].ssid);
        k += snprintf(out + k, cap - k,
                      "%s{\"ssid\":\"%s\",\"rssi\":%d,\"ch\":%u,\"lock\":%s}",
                      i ? "," : "", name, s_aps[i].rssi, (unsigned)s_aps[i].primary,
                      s_aps[i].authmode == WIFI_AUTH_OPEN ? "false" : "true");
    }
    k += snprintf(out + k, cap - k, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t e = httpd_resp_send(req, out, k);
    free(out);
    return e;
}

static esp_err_t h_save(httpd_req_t *req)
{
    char body[256];
    int total = 0, r;
    while (total < (int)sizeof(body) - 1
           && (r = httpd_req_recv(req, body + total, sizeof(body) - 1 - total)) > 0)
        total += r;
    body[total > 0 ? total : 0] = '\0';

    char ssid[33] = { 0 }, pass[65] = { 0 };
    if (!form_field(body, "ssid", ssid, sizeof(ssid)) || ssid[0] == '\0') {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "missing SSID");
    }
    form_field(body, "pass", pass, sizeof(pass));   // absent = open network

    if (save_creds(ssid, pass) != ESP_OK) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, "could not write credentials");
    }

    ESP_LOGI(TAG, "credentials saved for '%s' - rebooting into STA", ssid);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req,
        "<!doctype html><meta charset=utf-8><title>Saved</title>"
        // Only seen without JS; setup.html normally shows this itself.
        "<body style='font-family:ui-monospace,monospace;max-width:24rem;margin:2rem auto;"
        "padding:0 1rem;background:#121315;color:#6CF6E4'>"
        "<h1>Saved</h1><p>Rebooting and joining your network. Reconnect your phone "
        "to your normal WiFi, then open <b>" MDNS_HOSTNAME ".local</b>.</p>");
    vTaskDelay(pdMS_TO_TICKS(1500));
    esp_restart();
    return ESP_OK;
}

// Captive-portal catch-all: redirect any unknown URL to the config page so the
// phone/OS "sign in to WiFi network" detection fires.
static esp_err_t h_captive_redirect(httpd_req_t *req, httpd_err_code_t err)
{
    (void)err;
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://" AP_IP "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static void http_portal_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.core_id = 0;
    cfg.max_uri_handlers = 4;
    httpd_handle_t srv = NULL;
    ESP_ERROR_CHECK(httpd_start(&srv, &cfg));

    httpd_uri_t portal = { .uri = "/", .method = HTTP_GET,  .handler = h_portal };
    httpd_uri_t save   = { .uri = "/save", .method = HTTP_POST, .handler = h_save };
    httpd_uri_t scan   = { .uri = "/scan", .method = HTTP_GET,  .handler = h_scan };
    httpd_register_uri_handler(srv, &portal);
    httpd_register_uri_handler(srv, &save);
    httpd_register_uri_handler(srv, &scan);
    // Everything else -> redirect to the portal (captive-portal detection).
    httpd_register_err_handler(srv, HTTPD_404_NOT_FOUND, h_captive_redirect);
}

// ---------------- captive-portal DNS (answer every A query with AP_IP) --------

static void dns_task(void *arg)
{
    (void)arg;
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { ESP_LOGE(TAG, "DNS socket failed"); vTaskDelete(NULL); return; }
    struct sockaddr_in sa = {
        .sin_family = AF_INET, .sin_port = htons(53),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(sock, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
        ESP_LOGE(TAG, "DNS bind failed"); close(sock); vTaskDelete(NULL); return;
    }

    uint8_t pkt[256];
    for (;;) {
        struct sockaddr_in src;
        socklen_t sl = sizeof(src);
        int n = recvfrom(sock, pkt, sizeof(pkt), 0, (struct sockaddr *)&src, &sl);
        if (n < 12 || n > (int)sizeof(pkt) - 16) continue;     // need room for answer

        pkt[2] = 0x81; pkt[3] = 0x80;          // QR=1, RD copied, RA=1
        pkt[6] = 0; pkt[7] = 1;                // ANCOUNT = 1
        pkt[8] = pkt[9] = pkt[10] = pkt[11] = 0;

        int qtype = (pkt[n - 4] << 8) | pkt[n - 3];  // last 4 bytes = qtype+qclass
        int p = n;
        if (qtype == 1) {                      // A record -> point everything at us
            pkt[p++] = 0xc0; pkt[p++] = 0x0c;  // name pointer to the question
            pkt[p++] = 0x00; pkt[p++] = 0x01;  // TYPE A
            pkt[p++] = 0x00; pkt[p++] = 0x01;  // CLASS IN
            pkt[p++] = 0; pkt[p++] = 0; pkt[p++] = 0; pkt[p++] = 60;  // TTL
            pkt[p++] = 0x00; pkt[p++] = 0x04;  // RDLENGTH
            pkt[p++] = 192; pkt[p++] = 168; pkt[p++] = 4; pkt[p++] = 1;
        } else {
            pkt[6] = 0; pkt[7] = 0;            // no answer for non-A queries
        }
        sendto(sock, pkt, p, 0, (struct sockaddr *)&src, sl);
    }
}

// ---------------- entry point ----------------

void provision_run_portal(void)
{
    ESP_LOGW(TAG, "no WiFi credentials - starting SoftAP '%s' for setup",
             PROVISION_AP_SSID);

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&ic));

    wifi_config_t ap = { 0 };
    strlcpy((char *)ap.ap.ssid, PROVISION_AP_SSID, sizeof(ap.ap.ssid));
    ap.ap.ssid_len = strlen(PROVISION_AP_SSID);
    ap.ap.channel = 1;
    ap.ap.max_connection = 4;
    ap.ap.authmode = WIFI_AUTH_OPEN;           // open AP for easy setup join
    esp_netif_create_default_wifi_sta();       // STA side exists only to scan
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());
    do_scan();                                  // before any phone has joined

    http_portal_start();
    xTaskCreatePinnedToCore(dns_task, "dns", 3072, NULL, 4, NULL, 0);

    ESP_LOGI(TAG, "portal up: join WiFi '%s', browse to http://%s/",
             PROVISION_AP_SSID, AP_IP);
}
