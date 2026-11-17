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

// The next lastoff_mark_kill() is the idle auto-off (cause "auto_off").
void lastoff_set_auto(void);

// Record a deliberate power-off. Call right before asserting KILL.
void lastoff_mark_kill(void);

// Drive-session phase marker, saved at once (so a death pins the step).
enum { LO_PH_NONE = 0, LO_PH_BOOST = 1, LO_PH_MOTOR_CMD = 2, LO_PH_MOTOR_ACK = 3,
       LO_PH_SETTLED = 4, LO_PH_RADIO_ON = 5, LO_PH_TRACKS = 6 };
void lastoff_phase(int phase);

// Previous boot's cause for the LCD: "LOST IN READ" / "POWER BUTTON" / "NORMAL".
const char *lastoff_prev_short(void);

// JSON object for /api/status, e.g. {"cause":"lost_in_session",...}.
void lastoff_json(char *buf, size_t len);
