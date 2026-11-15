// 1.9" ST7789V status LCD (170x320 portrait), "Walkman LCD" style: setup QR, home
// screen (IP + web QR), read progress with a track strip, and results. A
// firmware update (ESP32 OTA or controller flash) takes the screen over, and
// wakes the panel, for as long as it runs.
#pragma once
#include <stdbool.h>

typedef enum {
    DISPLAY_PROVISIONING = 0,  // no WiFi creds: show the Floppy-Setup join QR
    DISPLAY_RUNTIME,           // normal operation: state-driven screens
} display_mode_t;

// Non-fatal if the panel is absent.
void display_init(void);

void display_set_mode(display_mode_t m);

// File count for the result screen; <0 = unknown.
void display_set_files(int n);

// "SAVED TO <name>" on the result screen; NULL or "" clears it.
void display_note_saved(const char *name);

// Any short note in the same place (up to three 19-character lines), e.g. why
// a SAVE did nothing. Cleared by display_note_saved(NULL).
void display_note(const char *text);

// DISP: toggle the home screen over the disk screen until the state changes.
void display_toggle_home(void);

// Panel sleep/wake (the backlight stays on). Safe from any task.
void display_set_power(bool on);
void display_toggle_power(void);
