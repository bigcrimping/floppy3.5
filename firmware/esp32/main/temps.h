// Board temperatures, for display only:
//   ESP32-S3 die  reads 10-20 C above ambient with the radio up.
//   RP2354A die   GetInfo 0x89, cached by the gw task, so it carries an age.
//   Battery       a JEITA zone from the BQ25180; its NTC reaches no ADC.
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "power.h"

typedef struct {
    bool    esp_valid;
    float   esp_c;          // ESP32-S3 die temperature

    bool    rp_valid;
    float   rp_c;           // RP2354A die temperature
    uint32_t rp_age_ms;     // how stale the controller reading is

    bool      batt_zone_valid;  // charger answered
    ts_zone_t batt_zone;        // JEITA window, not degrees
    bool      batt_ts_open;     // thermistor not connected
} temps_t;

// Non-fatal: the ESP reading is just invalid on failure.
void temps_init(void);

// Safe from any task; never touches the GWSPI link.
void temps_get(temps_t *out);
