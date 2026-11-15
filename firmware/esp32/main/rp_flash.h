// RP2354A flasher over SWD + RUN using a RAM flash stub. Works on a blank or
// wedged chip. See firmware/rp2354/docs/bringup.md.
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

// Program `img` (the raw .bin from the firmware/rp2354 build) at `addr`
// (0x10000000 for the firmware; 4 KB-aligned), verify it, and reset the chip.
// Blocks for tens of seconds. On failure writes a reason into err; the SWD
// pins are released and the chip reset either way.
esp_err_t rp_flash_program(const uint8_t *img, size_t len, uint32_t addr,
                           char *err, size_t errsz);

// For the LCD: percent through rp_flash_program() (write 0-70, verify
// 70-100; the erase before them shows as 0), -1 when it isn't running.
int rp_flash_progress_pct(void);
