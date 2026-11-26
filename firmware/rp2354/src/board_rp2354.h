// SPDX-License-Identifier: Unlicense
// Pin map and tunables for the gen-4 RP2354A controller PCB.
//
// All pins are in bank-0 GPIO 0..29 so both PIO blocks reach them with the
// default GPIOBASE of 0. This firmware uses GPIO 0..25.
//
// PCB rules (full list in docs/bringup.md):
//  - Erratum RP2350-E9: never rely on internal pull-downs on inputs. Floppy-bus
//    inputs have external pull-ups; SPI SCK/MOSI have external 100k pull-downs
//    so the link rests low when the ESP is unpowered.
//  - SWCLK/SWDIO are shared between the J12 debug header and the ESP (GPIO21/
//    GPIO14 via 100R R59/R60); the ESP programs this chip over SWD. SWCLK has a
//    100k pull-down so a rebooting ESP can't clock the debug port.
#pragma once

// ---- GWSPI link (SPI0 slave; docs/spi-link-protocol.md §1) ----
#define PIN_SPI_RX 0  // <- ESP MOSI (GPIO16)
#define PIN_SPI_CS 1  // <- ESP CS   (GPIO15)
#define PIN_SPI_SCK 2 // <- ESP SCLK (GPIO7)
#define PIN_SPI_TX 3  // -> ESP MISO (GPIO6)
#define PIN_DRDY 4    // -> ESP GPIO5, high = output queued

// ---- Debug UART -> J13 header ----
// Reserved; nothing opens UART0 yet. The ESP programs this chip over SWD, not
// UART.
#define PIN_DBG_UART_TX 12
#define PIN_DBG_UART_RX 13

// ---- Floppy bus, via the SN74LS07 buffers ----
// Scattered GPIOs to ease PCB routing. Every pin is used only through its
// macro (no masks, no adjacency), so a re-pin is header-only.
#define PIN_DENSITY 9  // ribbon 2  (IC2 pin 13 / FDD_DENSITY)
#define PIN_INDEX 7    // ribbon 8  (IC2 pin 10 / FDD_INDEX)
#define PIN_SELECT 8   // ribbon 12 (IC2 pin 12 / FDD_SEL; units 0 and 1 both use it)
#define PIN_MOTOR 18   // ribbon 16 (IC2 pin 29 / FDD_MOTOR)
#define PIN_DIR 17     // ribbon 18 (IC2 pin 28 / FDD_DIR)
#define PIN_STEP 19    // ribbon 20 (IC2 pin 31 / FDD_STEP)
#define PIN_WRDATA 20  // ribbon 22 (IC2 pin 32 / FDD_WRDATA; PIO during writes)
#define PIN_WRGATE 21  // ribbon 24 (IC2 pin 33 / FDD_WRGATE)
#define PIN_TRK0 22    // ribbon 26 (IC2 pin 34 / FDD_TRK0)
#define PIN_PROT 23    // ribbon 28 (IC2 pin 35 / FDD_WPROT)
#define PIN_RDDATA 25  // ribbon 30 (IC2 pin 37 / FDD_RDDATA; PIO JMP pin)
#define PIN_SIDE 24    // ribbon 32 (IC2 pin 36 / FDD_SIDE)
// Carries DISK CHANGE despite the schematic net name FDD_READY. True drive
// ready is PIN_RDY. GET_PIN 34 answers with this line.
#define PIN_DSKCHG 6   // ribbon 34 (IC2 pin 9)

// ---- Slimline-only status inputs (26-pin J1, buffered by IC11) ----
// Read-only via GET_PIN ids 100/101; never used to gate drive operations.
// With IC11 unfitted they read inactive. Not to be confused with PIN_DENSITY,
// which is the density SELECT output.
#define PIN_DENS 14 // <- FD_DENS (J1 pin 9), media density sense, active-low
#define PIN_RDY 15  // <- FD_RDY  (J1 pin 8), drive ready, active-low

// Frame geometry shared with the ESP's link_spi.c.
#include "../../common/gwspi_wire.h"

// ---- Flux staging ----
// Whole-track buffer. An HD read revolution is <= ~110 KB; the ESP sends
// <= 120 KB writes. Must be a power of two (SOURCE/SINK timestamp ring).
#define FLUX_BUF_SZ (256 * 1024)

// SPI link rings. Two 4 KB frames are in flight each way (window 2) and stay
// in the ring until acked, so each ring needs well over 8 KB or new frames
// go out half empty: an 8 KB TX ring averaged 2.4 KB per frame (speed-up
// phase 2a). 32 KB keeps credit (a u16 field) in range.
#define SPI_RX_RING_SZ (32 * 1024)
#define SPI_TX_RING_SZ (32 * 1024)

// ---- Drive timing (Greaseweazle factory defaults; SET_PARAMS can change) ----
#define DELAY_SELECT_US 10
#define DELAY_STEP_US 10000
#define DELAY_SETTLE_MS 15
#define DELAY_MOTOR_MS 750
#define DELAY_WATCHDOG_MS 10000
#define DELAY_PRE_WRITE_US 100
#define DELAY_POST_WRITE_US 1000

#define WRITE_CUE_TIMEOUT_MS 2000 // no index before a write -> ACK_NO_INDEX
#define WRITE_RX_STALL_MS 3000    // host went quiet mid WRITE_FLUX stream
#define TRANSPORT_IDLE_MS 10000   // release transport ownership after this

// ---- Identity (GetInfo) ----
#define GW_FW_MAJOR 1
#define GW_FW_MINOR 0
#define GW_MAX_CMD 20    // CMD_GET_PIN; TEST_MODE and NO_CLICK_STEP are rejected
#define GW_HW_MODEL 8    // gw.exe's generic RP2 category
#define GW_HW_SUBMODEL 1 // 0 = RP2040 tree, 1 = RP2354A
// Read SM: 2 PIO cycles/tick at 50 MHz (clkdiv 3). Write SM: 1 cycle/tick at
// 25 MHz (clkdiv 6). Integer dividers, so no fractional jitter. 25 MHz keeps
// DD 4/6/8 us cells (100/150/200 ticks) in single-byte codes.
#define GW_SAMPLE_FREQ 25000000u
#define GW_MCU_MHZ 150
#define GW_MCU_SRAM_KB 520
#define GW_USB_BUF_KB (FLUX_BUF_SZ / 1024)
