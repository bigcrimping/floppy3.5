// SPDX-License-Identifier: Unlicense
// pico-sdk board header for the gen-4 controller PCB: RP2354A = RP2350A die
// (QFN-60, 30 GPIOs) + 2 MB in-package flash. Pin assignments live in
// src/board_rp2354.h.
//
// PCB rule: a 12 MHz crystal is required (the bootrom's USB boot assumes it).
#ifndef _BOARDS_GW_RP2354A_H
#define _BOARDS_GW_RP2354A_H

#define GW_RP2354A

#define PICO_RP2350A 1

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#endif

// The J13 debug UART header (unused by the firmware today).
#ifndef PICO_DEFAULT_UART
#define PICO_DEFAULT_UART 0
#endif
#ifndef PICO_DEFAULT_UART_TX_PIN
#define PICO_DEFAULT_UART_TX_PIN 12
#endif
#ifndef PICO_DEFAULT_UART_RX_PIN
#define PICO_DEFAULT_UART_RX_PIN 13
#endif

#endif
