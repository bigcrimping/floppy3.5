// Board serial number in the RV-3028's user EEPROM, which survives an ESP
// erase-flash and RP reflashes (unlike NVS).
//
// Record at offset 0: 'S' 'N' <ver=1> <len 1..32> <ASCII> <CRC-16-CCITT, BE>.
// A blank or damaged record reads as "not set". Set via POST /api/serial;
// changing it requires force.
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#define SERIALNO_MAX 32

// Call after wallclock_init().
void serialno_init(void);

// The serial number, or "" when unset, damaged, or the RTC is absent.
const char *serialno_get(void);

// 1..32 chars of A-Z a-z 0-9 - _ . ; written and read back to verify.
esp_err_t serialno_set(const char *sn, bool force, char *err, size_t errsz);
