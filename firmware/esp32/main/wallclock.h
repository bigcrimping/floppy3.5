// RV-3028-C7 RTC (Y2, I2C 0x52). FatFs stamps saved files with time(NULL), and
// the device may run with no network, so the RTC seeds the system clock at
// boot and SNTP corrects it whenever WiFi is up. Everything is UTC (TZ=UTC0);
// the browser converts for display. If the part doesn't answer, every call is
// a no-op.
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

typedef enum {
    WALLCLOCK_SRC_UNSET = 0,   // clock never set — time is meaningless (1970)
    WALLCLOCK_SRC_RTC,         // seeded from the RV-3028 at boot
    WALLCLOCK_SRC_SNTP,        // corrected from the network
} wallclock_src_t;

// Seed the system clock from the RTC. Call after power_init() (which creates
// the bus) and before anything timestamps a file.
void wallclock_init(void);

// Store the system time in the RTC (on every SNTP sync).
bool wallclock_store(void);

bool wallclock_rtc_present(void);
wallclock_src_t wallclock_source(void);
const char *wallclock_source_str(void);  // "unset" | "rtc" | "sntp"

// ---- boot verdict: did battery backup hold? ----
// Latched at wallclock_init(), since the source soon changes to "sntp". HELD =
// time kept across a power cycle. LOST = PORF set: expected on a board's first
// power-up, a backup fault after that.
typedef enum {
    WALLCLOCK_BOOT_ABSENT = 0,  // the part did not answer sanely
    WALLCLOCK_BOOT_LOST,        // answered, PORF set: time was not retained
    WALLCLOCK_BOOT_HELD,        // answered with a time it had kept
} wallclock_boot_t;

wallclock_boot_t wallclock_boot_state(void);
const char *wallclock_boot_str(void);   // "absent" | "lost" | "held"

// Backup register (0x37) as read at boot, 0xFF if unreadable. BSM (bits 3:2)
// must be 0b11 and TCE (bit 5) 0, since VBACKUP is tied to the LiPo.
uint8_t wallclock_backup_reg(void);

// ---- user EEPROM ----
//
// 43 bytes of EEPROM in the RV-3028 that survive a flat battery and any
// reflash; holds the board serial (serialno.c). False if the RTC is absent or
// the range is out of bounds.
#define WALLCLOCK_EE_SIZE 43
bool wallclock_ee_read(uint8_t off, uint8_t *buf, size_t n);
bool wallclock_ee_write(uint8_t off, const uint8_t *buf, size_t n);
