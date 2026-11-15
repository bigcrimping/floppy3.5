// The per-peripheral tests. Each files its results through bu_report() and
// returns nothing; the sequencer (bu_main.c) decides the order.
#pragma once
#include <stdbool.h>

// ESP32-S3 module itself: chip id, MAC, flash size, PSRAM size + pattern test,
// die temperature, USB-Serial/JTAG host presence, WiFi scan (radio + antenna).
void bu_test_esp(void);

// Power-path GPIOs: safe defaults, idle levels of the interrupt lines, buttons.
void bu_test_power_io_init(void);   // once at boot, BEFORE anything else touches GPIO
void bu_test_power_levels(void);
void bu_boost_5v(bool on);          // TPS61022 EN
void bu_ltc_kill(void);             // one-way: assert LTC2954 KILL (board powers off)

// I2C: line idle levels, bus scan, MAX17048 / BQ25180 / RV-3028 identity + RTC tick.
void bu_test_i2c(void);

// microSD on SPI2: card-detect, mount, write/read/delete, unmount.
void bu_test_sd(void);

// RP2354 controller: RUN pulse, SWD attach (DPIDR/AP IDR), CHIP_ID, SRAM
// write/read, GWSPI link continuity wire-by-wire (driven through the RP's
// SIO over SWD, so it works on a BLANK chip), RUN-reset check. Falls back to
// passive idle-level checks when SWD does not attach.
void bu_test_rp(void);

// GWSPI link to the RP2354 at protocol level: peer reset, SYNC handshake,
// GetInfo round-trip, and the controller's own link counters. Needs the RP
// running its firmware, so it runs after bu_test_rp (which halts the core).
void bu_test_link(void);
