// The 4x WS2812B chain (LED1..LED4 on UI_LED_GPIO). Visual only.
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "bu_report.h"

bool bu_leds_init(void);
void bu_leds_set(int i, uint8_t r, uint8_t g, uint8_t b);   // brightness-capped
void bu_leds_all(uint8_t r, uint8_t g, uint8_t b);
void bu_leds_flush(void);
// Walk R, G, B through each LED in chain order — proves data reaches every
// pixel and that they are in the expected order on the board.
void bu_leds_chase(void);
// Colour each LED by a result status (green/red/amber/blue/white).
void bu_leds_status(bu_status_t led1, bu_status_t led2, bu_status_t led3, bu_status_t led4);
