// Over-the-air update of this ESP32-S3 app: POST /api/ota writes the inactive
// slot, which boots on restart and rolls back unless it confirms itself.
// Two 4 MB slots + otadata (../partitions.csv); a device on an older table
// needs one wired `idf.py erase-flash flash` first.
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

// Call once at boot, only after WiFi and the controller link are up: it marks
// a freshly updated image valid and cancels the rollback.
void ota_mark_valid_if_pending(void);

// True while an update is being received.
bool ota_in_progress(void);

// For the LCD: percent received, 100 once the image is accepted and the
// reboot is pending, -1 when no update is under way.
int ota_progress_pct(void);

// Held around work an update reboot must not interrupt (drive and SD jobs).
// ota_hold() fails if an update is already under way; ota_begin() refuses
// while any hold is taken.
bool ota_hold(void);
void ota_unhold(void);

// ota_begin(), ota_write() per chunk, ota_end(). ota_abort() is safe at any
// point.
esp_err_t ota_begin(size_t total_len, char *err, size_t errsz);
esp_err_t ota_write(const void *data, size_t len, char *err, size_t errsz);
esp_err_t ota_end(char *err, size_t errsz);   // validates, sets the boot slot
void      ota_abort(void);

// For /api/status.
const char *ota_running_label(void);
bool        ota_pending_verify(void);
