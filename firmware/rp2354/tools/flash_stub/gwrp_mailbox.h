// SPDX-License-Identifier: Unlicense
// GWRP mailbox — the shared-SRAM command block between the ESP (SWD master,
// firmware/esp32/main/rp_flash.c) and the RP2354 RAM flash stub
// (flash_stub.c). ONE definition, compiled into both sides; a change here is
// a change to both ends by construction.
//
// Transport: the ESP has no code running on the RP other than this stub, so
// it pokes the struct through the SWD MEM-AP (32-bit word writes) while the
// stub polls it from the CPU. There is no interrupt and no UART involved.
//
// Protocol (one command in flight):
//   ESP: write data[] (if any), then addr/len/crc, then cmd = <op>, seq++
//   stub: sees seq change -> executes -> writes status, then ack = seq
//   ESP: polls ack == seq, reads status (+ data[] for READ)
// Every field is 32-bit so the ESP only ever needs word accesses (the MEM-AP
// auto-increment path is word-oriented; byte access would need TAR reloads).
#pragma once
#include <stdint.h>

// Where the stub places the mailbox. RP2350 SRAM starts at 0x20000000; the
// stub is loaded at the base and is a few KB, so the mailbox sits well clear
// of it and of the stack (top of SRAM, from the stub's vector table).
#define GWRP_MAILBOX_ADDR 0x20010000u

// Payload bytes per WRITE/READ command. 4 KB = one flash sector erase unit
// and 16 program pages; also the ESP's chunk size.
#define GWRP_DATA_MAX 4096u

// Magic the stub writes once it is running (the ESP's "is it alive" probe).
#define GWRP_MAGIC 0x50525747u // "GWRP" little-endian

enum gwrp_cmd {
    GWRP_CMD_NONE  = 0,
    GWRP_CMD_PING  = 1, // no-op; proves the poll loop is running
    GWRP_CMD_ERASE = 2, // addr/len, both 4096-aligned
    GWRP_CMD_WRITE = 3, // addr (256-aligned), len <= GWRP_DATA_MAX, crc of data
    GWRP_CMD_READ  = 4, // addr/len -> data[] filled, crc set by the stub
    GWRP_CMD_EXIT  = 5, // acknowledge, then spin (the ESP resets the chip)
};

enum gwrp_status {
    GWRP_ST_OK       = 0,
    GWRP_ST_BADARG   = 1, // span/alignment rejected
    GWRP_ST_BADCRC   = 2, // WRITE payload CRC mismatch
    GWRP_ST_BADCMD   = 3,
};

// Field order matters only in that both sides agree; keep it stable.
struct gwrp_mailbox {
    volatile uint32_t magic;   // stub -> ESP: GWRP_MAGIC once alive
    volatile uint32_t seq;     // ESP -> stub: incremented to submit a command
    volatile uint32_t ack;     // stub -> ESP: echoes seq when complete
    volatile uint32_t cmd;     // ESP -> stub: enum gwrp_cmd
    volatile uint32_t addr;    // bus address (0x10xxxxxx), not a flash offset
    volatile uint32_t len;
    volatile uint32_t crc;     // WRITE: ESP's CRC16 of data[0..len)
                               // READ:  stub's CRC16 of data[0..len)
    volatile uint32_t status;  // stub -> ESP: enum gwrp_status
    volatile uint8_t  data[GWRP_DATA_MAX];
};

// CRC-16/CCITT-FALSE (poly 0x1021), the repo-wide convention. Defined inline
// so the stub needs no extra translation unit and the ESP side can check the
// two implementations agree in a host test.
static inline uint16_t gwrp_crc16(const uint8_t *p, uint32_t n)
{
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc ^= (uint16_t)(*p++) << 8;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
    }
    return crc;
}
