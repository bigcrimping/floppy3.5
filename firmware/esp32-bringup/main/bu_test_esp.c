#include "bu_tests.h"
#include "bu_report.h"

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_mac.h"
#include "esp_flash.h"
#include "esp_psram.h"
#include "esp_heap_caps.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "driver/temperature_sensor.h"
#include "driver/usb_serial_jtag.h"

static const char *TAG = "t_esp";

static void chip_id(void)
{
    esp_chip_info_t ci;
    esp_chip_info(&ci);
    uint8_t mac[6] = { 0 };
    esp_efuse_mac_get_default(mac);
    bu_report("ESP chip", ci.model == CHIP_ESP32S3 ? BU_PASS : BU_FAIL,
              "%s rev v%d.%d, %d cores, MAC %02X:%02X:%02X:%02X:%02X:%02X",
              ci.model == CHIP_ESP32S3 ? "ESP32-S3" : "not an S3?",
              ci.revision / 100, ci.revision % 100, ci.cores,
              mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void flash_size(void)
{
    uint32_t sz = 0;
    esp_err_t e = esp_flash_get_size(NULL, &sz);
    if (e != ESP_OK) { bu_report("ESP flash", BU_FAIL, "%s", esp_err_to_name(e)); return; }
    bu_report("ESP flash", sz == 16u * 1024 * 1024 ? BU_PASS : BU_WARN,
              "%lu MB (N16R8 expects 16)", (unsigned long)(sz >> 20));
}

static void psram(void)
{
    size_t sz = esp_psram_get_size();
    if (sz == 0) {
        bu_report("ESP PSRAM", BU_FAIL, "not detected (octal PSRAM on GPIO33-37 inside the module)");
        return;
    }
    bu_report("ESP PSRAM size", sz == 8u * 1024 * 1024 ? BU_PASS : BU_WARN,
              "%u MB (N16R8 expects 8)", (unsigned)(sz >> 20));

    // Pattern test over 1 MB: address-derived words, so a stuck or aliased
    // line shows up as a mismatch at a reportable offset.
    const size_t n = 1u << 20;
    uint32_t *buf = heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
    if (!buf) { bu_report("ESP PSRAM r/w", BU_FAIL, "1 MB SPIRAM alloc failed"); return; }
    size_t words = n / 4;
    for (size_t i = 0; i < words; i++) buf[i] = (uint32_t)i * 0x9E3779B9u ^ 0xA5A5A5A5u;
    size_t bad = 0, first = 0;
    for (size_t i = 0; i < words; i++)
        if (buf[i] != ((uint32_t)i * 0x9E3779B9u ^ 0xA5A5A5A5u)) { if (!bad) first = i * 4; bad++; }
    heap_caps_free(buf);
    if (bad) bu_report("ESP PSRAM r/w", BU_FAIL, "%u bad words in 1 MB, first at +0x%X", (unsigned)bad, (unsigned)first);
    else     bu_report("ESP PSRAM r/w", BU_PASS, "1 MB pattern OK, free SPIRAM %u KB",
                       (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) >> 10));
}

static void die_temp(void)
{
    temperature_sensor_handle_t h = NULL;
    temperature_sensor_config_t c = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    float t = 0;
    if (temperature_sensor_install(&c, &h) == ESP_OK && temperature_sensor_enable(h) == ESP_OK &&
        temperature_sensor_get_celsius(h, &t) == ESP_OK) {
        bu_report("ESP die temp", (t > 0 && t < 70) ? BU_INFO : BU_WARN, "%.1f C", t);
        temperature_sensor_disable(h);
    } else {
        bu_report("ESP die temp", BU_WARN, "sensor read failed");
    }
    if (h) temperature_sensor_uninstall(h);
}

static void usb(void)
{
    bool c = usb_serial_jtag_is_connected();
    bu_report("ESP USB (J8)", c ? BU_PASS : BU_INFO,
              c ? "USB-Serial/JTAG host connected (GPIO19/20)" : "no USB host seen — using UART0/J3?");
}

// One passive scan: proves the radio, the module antenna and the RF path.
static void wifi_scan(void)
{
    static bool s_stack;    // netif + event loop are create-once
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        e = nvs_flash_init();
    }
    if (e != ESP_OK) { bu_report("ESP WiFi scan", BU_FAIL, "nvs_flash_init: %s", esp_err_to_name(e)); return; }
    if (!s_stack) {
        esp_netif_init();
        esp_event_loop_create_default();
        esp_netif_create_default_wifi_sta();
        s_stack = true;
    }
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    if ((e = esp_wifi_init(&cfg)) != ESP_OK) { bu_report("ESP WiFi scan", BU_FAIL, "esp_wifi_init: %s", esp_err_to_name(e)); return; }
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);
    if ((e = esp_wifi_start()) != ESP_OK) {
        bu_report("ESP WiFi scan", BU_FAIL, "esp_wifi_start: %s", esp_err_to_name(e));
        esp_wifi_deinit();
        return;
    }
    wifi_scan_config_t sc = { 0 };
    e = esp_wifi_scan_start(&sc, true);          // blocking, ~1.5-3 s
    uint16_t n = 0;
    if (e == ESP_OK) esp_wifi_scan_get_ap_num(&n);
    wifi_ap_record_t top = { 0 };
    if (n) {
        uint16_t want = 1;                       // records come RSSI-sorted
        esp_wifi_scan_get_ap_records(&want, &top);
    }
    esp_wifi_scan_stop();
    esp_wifi_stop();
    esp_wifi_deinit();
    if (e != ESP_OK)      bu_report("ESP WiFi scan", BU_FAIL, "scan: %s", esp_err_to_name(e));
    else if (n == 0)      bu_report("ESP WiFi scan", BU_WARN, "radio OK but 0 networks heard (antenna? shielded bench?)");
    else                  bu_report("ESP WiFi scan", BU_PASS, "%u networks, strongest '%.20s' %d dBm ch%u",
                                    n, (const char *)top.ssid, top.rssi, top.primary);
}

void bu_test_esp(void)
{
    ESP_LOGI(TAG, "--- ESP32-S3 module ---");
    chip_id();
    flash_size();
    psram();
    die_temp();
    usb();
    wifi_scan();
}
