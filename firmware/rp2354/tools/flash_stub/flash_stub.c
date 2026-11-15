// SPDX-License-Identifier: Unlicense
// RP2350 RAM flash stub: loaded into SRAM over SWD by the ESP (no bootrom
// UART boot, no BOOTSEL strap — see firmware/rp2354/docs/bringup.md), then
// programs the RP2354's in-package flash on command. The ESP pokes the
// shared mailbox (gwrp_mailbox.h) through the SWD MEM-AP while this loop
// polls it from the CPU; no UART, no interrupts, no flash image.
//
// Everything the stub touches lives in SRAM: it runs with XIP quiesced during
// erase/program (the SDK's flash_range_* handle that), so it must never call
// through to code or constants in flash. Hence: no printf, no SDK init, and
// the mailbox handler is compiled into RAM by the no_flash binary type.
//
// Build (bench, pico-sdk 2.x):  cmake -S . -B build && cmake --build build
// -> build/flash_stub.bin, embedded in firmware/esp32/main/rp_flash_stub_bin.h.
#include <stdint.h>
#include <string.h>

#include "hardware/flash.h"
#include "hardware/sync.h"

#include "gwrp_mailbox.h"

#define XIP_BASE_ADDR 0x10000000u
#define FLASH_BYTES (2u * 1024 * 1024) // RP2354A: 2 MB stacked in-package

// The mailbox at its fixed address (gwrp_mailbox.h). Not in .bss: the ESP
// writes the struct before releasing the core, and a startup .bss clear would
// wipe it — this is a bare pointer to an agreed address instead.
static volatile struct gwrp_mailbox *const mb =
    (volatile struct gwrp_mailbox *)GWRP_MAILBOX_ADDR;

// addr arrives as a bus address (0x10xxxxxx); the flash API wants offsets.
static int span_ok(uint32_t addr, uint32_t len, uint32_t align)
{
    if (addr < XIP_BASE_ADDR || len == 0 || len > GWRP_DATA_MAX)
        return 0;
    if (addr & (align - 1))
        return 0;
    uint32_t off = addr - XIP_BASE_ADDR;
    return off <= FLASH_BYTES - len; // len <= GWRP_DATA_MAX: no wrap
}

static uint32_t do_erase(uint32_t addr, uint32_t len)
{
    // Erase spans may exceed one data buffer, so this one is not bounded by
    // GWRP_DATA_MAX — check it against the device instead.
    // Written so nothing can wrap: (addr - base) + len overflows 32 bits for
    // a huge len and would pass a sum-based check.
    if (addr < XIP_BASE_ADDR || len == 0 ||
        (addr & (FLASH_SECTOR_SIZE - 1)) || (len & (FLASH_SECTOR_SIZE - 1)) ||
        len > FLASH_BYTES || addr - XIP_BASE_ADDR > FLASH_BYTES - len)
        return GWRP_ST_BADARG;
    uint32_t save = save_and_disable_interrupts();
    flash_range_erase(addr - XIP_BASE_ADDR, len);
    restore_interrupts(save);
    return GWRP_ST_OK;
}

static uint32_t do_write(uint32_t addr, uint32_t len, uint32_t want_crc)
{
    static uint8_t page[GWRP_DATA_MAX]; // SRAM copy: flash API needs a stable
                                        // buffer, and the mailbox is volatile
    if (!span_ok(addr, len, FLASH_PAGE_SIZE))
        return GWRP_ST_BADARG;
    for (uint32_t i = 0; i < len; i++)
        page[i] = mb->data[i];
    if (gwrp_crc16(page, len) != (uint16_t)want_crc)
        return GWRP_ST_BADCRC;
    // Pad the tail to the 256-byte program granularity with 0xFF (erased).
    uint32_t plen = (len + FLASH_PAGE_SIZE - 1) & ~(FLASH_PAGE_SIZE - 1u);
    memset(page + len, 0xFF, plen - len);
    uint32_t save = save_and_disable_interrupts();
    flash_range_program(addr - XIP_BASE_ADDR, page, plen);
    restore_interrupts(save);
    return GWRP_ST_OK;
}

static uint32_t do_read(uint32_t addr, uint32_t len)
{
    if (!span_ok(addr, len, 1))
        return GWRP_ST_BADARG;
    const uint8_t *p = (const uint8_t *)addr; // XIP reads are plain loads
    for (uint32_t i = 0; i < len; i++)
        mb->data[i] = p[i];
    // CRC what actually sits in the mailbox (read back through the volatile
    // pointer), without a second 4 KB buffer to hold it.
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= (uint16_t)mb->data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
    }
    mb->crc = crc; // same CRC-16/CCITT-FALSE as gwrp_crc16()
    return GWRP_ST_OK;
}

int main(void)
{
    // Announce liveness before touching anything else: the ESP polls magic to
    // decide the stub loaded and started (its "probe" step).
    mb->cmd = GWRP_CMD_NONE;
    mb->ack = mb->seq;
    mb->magic = GWRP_MAGIC;

    for (;;) {
        uint32_t seq = mb->seq;
        if (seq == mb->ack)
            continue; // no new command

        uint32_t st;
        switch (mb->cmd) {
        case GWRP_CMD_PING:
            st = GWRP_ST_OK;
            break;
        case GWRP_CMD_ERASE:
            st = do_erase(mb->addr, mb->len);
            break;
        case GWRP_CMD_WRITE:
            st = do_write(mb->addr, mb->len, mb->crc);
            break;
        case GWRP_CMD_READ:
            st = do_read(mb->addr, mb->len);
            break;
        case GWRP_CMD_EXIT:
            mb->status = GWRP_ST_OK;
            mb->ack = seq;
            // The ESP resets the chip from its side (RUN pulse) — spin here
            // rather than self-resetting, so it stays in control of timing.
            for (;;)
                __asm volatile("wfi");
        default:
            st = GWRP_ST_BADCMD;
            break;
        }

        mb->status = st;
        mb->ack = seq; // publish LAST: ack is what releases the ESP's poll
    }
}
