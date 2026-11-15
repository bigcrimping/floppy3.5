// SPDX-License-Identifier: Unlicense
// GWSPI v1 wire format -- ONE definition, compiled into both ends of the link:
// the ESP32 master (firmware/esp32/main/link_spi.c) and the RP2354 slave
// (firmware/rp2354/src/transport_spi.c, via board_rp2354.h). The protocol
// itself is specified in docs/spi-link-protocol.md; this header only pins its
// numbers, so a change here is a change to both ends by construction (the same
// pattern as the SWD mailbox, rp2354/tools/flash_stub/gwrp_mailbox.h).
#pragma once

#define GWSPI_FRAME_MAX   4096 // max clocked transaction = both DMA frame buffers
#define GWSPI_PAYLOAD_MAX 4080 // 12 hdr + 4080 payload + 2 pcrc = 4094, pad to 4096
#define GWSPI_HDR_LEN     12
#define GWSPI_MAGIC_M2S   0xA5
#define GWSPI_MAGIC_S2M   0x5A
#define GWSPI_VERSION     1    // low nibble of the flags byte
#define GWSPI_FLAG_SYNC   0x10
// Sideband status bits in the flags byte, one per direction (spec 2.1a). A
// receiver that predates them checks only the version nibble and SYNC, so
// either end may be older without harm. Both ride header-only polls, so they
// are read without putting a command on the link.
#define GWSPI_FLAG_USB    0x20 // S->M: a USB host holds the gw port (CDC0 DTR)
                               // or owns the transport. The master must stay
                               // off the command channel and keep the drive
                               // rail up.
#define GWSPI_FLAG_RAIL   0x40 // M->S: the master's 5 V drive rail is on.

// Header byte offsets (spec 2.1). Multi-byte fields are little-endian; the
// header CRC covers bytes [0, GWSPI_OFF_HCRC).
#define GWSPI_OFF_MAGIC  0
#define GWSPI_OFF_FLAGS  1
#define GWSPI_OFF_SEQ    2
#define GWSPI_OFF_ACK    3
#define GWSPI_OFF_LEN    4
#define GWSPI_OFF_AVAIL  6
#define GWSPI_OFF_CREDIT 8
#define GWSPI_OFF_HCRC   10

_Static_assert(GWSPI_HDR_LEN + GWSPI_PAYLOAD_MAX + 2 <= GWSPI_FRAME_MAX,
               "max frame must fit the DMA buffer");
