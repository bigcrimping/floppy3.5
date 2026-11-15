// SPDX-License-Identifier: Unlicense
// TinyUSB: CDC0 = Greaseweazle protocol (transport_usb.c), CDC1 = debug
// console (diag.c).
#pragma once

// pico-sdk normally supplies these; the fallbacks let a bare compile work.
#ifndef CFG_TUSB_MCU
#define CFG_TUSB_MCU OPT_MCU_RP2040
#endif
#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS OPT_OS_PICO
#endif

#define CFG_TUD_ENABLED 1
#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE // older TinyUSB spelling

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif
#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))
#endif

#define CFG_TUD_ENDPOINT0_SIZE 64

#define CFG_TUD_CDC 2
#define CFG_TUD_MSC 0
#define CFG_TUD_HID 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 0

// Commands are <= 12 bytes and read chunks 64, so 512 bytes each way is ample.
#define CFG_TUD_CDC_RX_BUFSIZE 512
#define CFG_TUD_CDC_TX_BUFSIZE 512
#define CFG_TUD_CDC_EP_BUFSIZE 64 // full-speed bulk MPS
