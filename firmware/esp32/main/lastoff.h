#pragma once
// Shutdown breadcrumb: why did the board last go off?
//
// The LTC2954 cuts every rail at once, so a power loss leaves no trace in
// RAM, and the ESP's reset reason only says "poweron". This keeps a small
// record in NVS while the 5 V drive rail is on (a read, write or drive test,
// i.e. the heavy-load window), and marks a deliberate KILL. At the next boot
// the record is read back, published under "lastoff" in /api/status, and
// cleared.
//
//   cause "lost_in_session": power vanished while the drive rail was on
//                            (no KILL issued) = brown-out / LTC2954 dropout
//   cause "kill":            firmware asserted KILL (S5 shutdown request)
//   cause "none":            no session open and no KILL: idle power loss,
//                            battery pull, or first boot with this firmware

#include <stddef.h>

// Read and clear the previous boot's record. Call once, after nvs_flash_init().
void lastoff_boot(void);

// Call periodically (ui_task). Opens a record when the drive rail turns on,
// refreshes it at most every LASTOFF_SAVE_MS, closes it when the rail turns off.
void lastoff_tick(void);

// Record a deliberate power-off. Call right before asserting KILL.
void lastoff_mark_kill(void);

// JSON object for /api/status, e.g. {"cause":"lost_in_session",...}.
void lastoff_json(char *buf, size_t len);
