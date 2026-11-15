// The four WS2812B status LEDs. STATUS mode shows read progress, then the
// outcome colour; BATTERY mode (DISP button) shows the charge for
// LED_BATTERY_MS and reverts by itself. Safe from any task; no-ops if the
// strip failed to initialise.
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define LED_BATTERY_MS 4000

void leds_init(void);

// disk_state_t value; called every UI poll.
void leds_set_state(int disk_state);

void leds_show_battery(void);

// State keeps tracking while disabled.
void leds_set_enabled(bool on);
bool leds_enabled(void);
