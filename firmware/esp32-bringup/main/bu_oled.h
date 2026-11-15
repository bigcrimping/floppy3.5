// Minimal SSD1309 (128x64, 4-wire SPI) driver for the bring-up: enough to put
// a test pattern and a few lines of text on the panel. Write-only device, so
// its presence cannot be sensed — it is always a LOOK item.
#pragma once
#include <stdbool.h>

bool bu_oled_init(void);            // SPI2 bus must already be initialised
void bu_oled_clear(void);
void bu_oled_text(int x, int y, const char *s);
void bu_oled_rect(int x, int y, int w, int h, int on);
void bu_oled_flush(void);
void bu_oled_invert(bool inv);
void bu_oled_test_pattern(const char *line2);
