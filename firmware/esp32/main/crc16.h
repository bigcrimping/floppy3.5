// CRC-16-CCITT (poly 0x1021, no reflection, no xorout; callers seed 0xFFFF).
#pragma once
#include <stdint.h>
#include <stddef.h>

// Bitwise, for MFM.
uint16_t crc16_ccitt(uint16_t crc, const uint8_t *buf, size_t len);

// Same result, table-driven, for the GWSPI link. Must match
// firmware/rp2354/src/crc16.c.
uint16_t crc16_ccitt_tab(uint16_t init, const uint8_t *buf, size_t len);

// CRC after the three 0xA1 sync bytes, from 0xFFFF.
#define CRC16_AFTER_A1A1A1  0xCDB4
