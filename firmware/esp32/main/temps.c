#include "temps.h"
#include "gw_client.h"
#include "gw_proto.h"
#include "power.h"

#include <string.h>
#include "esp_log.h"
#include "driver/temperature_sensor.h"

static const char *TAG = "temps";

static temperature_sensor_handle_t s_esp;

void temps_init(void)
{
    // 10..80 C is the most accurate range (+/-1 C).
    temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
    esp_err_t e = temperature_sensor_install(&cfg, &s_esp);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "ESP die sensor install failed: %s", esp_err_to_name(e));
        s_esp = NULL;
        return;
    }
    if ((e = temperature_sensor_enable(s_esp)) != ESP_OK) {
        ESP_LOGW(TAG, "ESP die sensor enable failed: %s", esp_err_to_name(e));
        temperature_sensor_uninstall(s_esp);
        s_esp = NULL;
        return;
    }
    float c;
    if (temperature_sensor_get_celsius(s_esp, &c) == ESP_OK)
        ESP_LOGI(TAG, "ESP32-S3 die %.1f C", c);
}

void temps_get(temps_t *out)
{
    memset(out, 0, sizeof(*out));

    if (s_esp) {
        float c;
        if (temperature_sensor_get_celsius(s_esp, &c) == ESP_OK) {
            out->esp_c = c;
            out->esp_valid = true;
        }
    }

    struct gw_health h;
    uint32_t age;
    if (gw_get_health(&h, &age) && h.temp_valid) {
        out->rp_c = h.die_temp_c100 / 100.0f;
        out->rp_age_ms = age;
        out->rp_valid = true;
    }

    power_state_t ps;
    if (power_get(&ps) && ps.charger_ok) {
        out->batt_zone_valid = true;
        out->batt_zone = ps.ts_zone;
        out->batt_ts_open = ps.ts_open;
    }
}
