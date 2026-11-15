// SPDX-License-Identifier: Unlicense
// CRC-16-CCITT (poly 0x1021, no reflection, no xorout; callers seed 0xFFFF),
// the GWSPI frame CRC. Must match firmware/esp32/main/crc16.c.
#pragma once
#include <stddef.h>
#include <stdint.h>

uint16_t crc16_ccitt_tab(uint16_t init, const uint8_t *buf, size_t len);
