// Greaseweazle protocol constants (upstream keirf/greaseweazle). Must agree
// with firmware/rp2354/src/gw_proto.h.
#pragma once
#include <stdint.h>

// ---- Command bytes ----
enum gw_cmd {
    CMD_GET_INFO       = 0,
    CMD_UPDATE         = 1,
    CMD_SEEK           = 2,
    CMD_HEAD           = 3,
    CMD_SET_PARAMS     = 4,
    CMD_GET_PARAMS     = 5,
    CMD_MOTOR          = 6,
    CMD_READ_FLUX      = 7,
    CMD_WRITE_FLUX     = 8,
    CMD_GET_FLUX_STATUS= 9,
    CMD_GET_INDEX_TIMES= 10,   // deprecated
    CMD_SWITCH_FW_MODE = 11,
    CMD_SELECT         = 12,
    CMD_DESELECT       = 13,
    CMD_SET_BUS_TYPE   = 14,
    CMD_SET_PIN        = 15,
    CMD_RESET          = 16,
    CMD_ERASE_FLUX     = 17,
    CMD_SOURCE_BYTES   = 18,
    CMD_SINK_BYTES     = 19,
    CMD_GET_PIN        = 20,
    CMD_TEST_MODE      = 21,
    CMD_NO_CLICK_STEP  = 22,
};

// ---- Acknowledgement / status bytes ----
enum gw_ack {
    ACK_OKAY           = 0,
    ACK_BAD_COMMAND    = 1,
    ACK_NO_INDEX       = 2,
    ACK_NO_TRK0        = 3,
    ACK_FLUX_OVERFLOW  = 4,
    ACK_FLUX_UNDERFLOW = 5,
    ACK_WRPROT         = 6,
    ACK_NO_UNIT        = 7,
    ACK_NO_BUS         = 8,
    ACK_BAD_UNIT       = 9,
    ACK_BAD_PIN        = 10,
    ACK_BAD_CYLINDER   = 11,
    ACK_OUT_OF_SRAM    = 12,
    ACK_OUT_OF_FLASH   = 13,
};

// GetInfo sub-commands (payload byte after cmd/len).
enum gw_getinfo_sub {
    GETINFO_FIRMWARE      = 0,
    GETINFO_BW_STATS      = 1,
    GETINFO_CURRENT_DRIVE = 7,
    GETINFO_GW_DIAG       = 0x88, // ours: controller link/proto counters
    GETINFO_GW_HEALTH     = 0x89, // ours: die temperature + uptime
    GETINFO_GW_LINKPROF   = 0x8A, // ours: SPI link profile (speed-up phase 2a)
    GETINFO_GW_CAPPROF    = 0x8B, // ours: capture profile (speed-up phase 3)
};

// GETINFO_GW_DIAG payload: the first eight counters of the controller's
// gw_diag_t (firmware/rp2354/src/diag.h; the host test checks the layouts
// match). Controllers without 0x88 nack it.
#pragma pack(push, 1)
struct gw_diag_counters {
    uint32_t spi_rx_overruns;   // ring full; must stay 0
    uint32_t spi_frame_errors;  // bad magic/version/hcrc/pcrc
    uint32_t spi_retransmits;   // frames resent (go-back-N)
    uint32_t spi_seq_errs;
    uint32_t proto_bad_cmd;
    uint32_t proto_bad_len;
    uint32_t arb_to_usb;
    uint32_t arb_to_spi;
};
#pragma pack(pop)
_Static_assert(sizeof(struct gw_diag_counters) == 32,
               "diag window must fill the 32-byte GetInfo payload");

// GETINFO_GW_HEALTH payload (0x88 is already full). Controllers without it
// nack it.
#pragma pack(push, 1)
struct gw_health {
    int16_t  die_temp_c100;   // controller die temperature, hundredths of a degree C
    uint8_t  temp_valid;      // 0 = no sensor / not sampled; die_temp_c100 meaningless
    uint8_t  _pad;
    uint32_t uptime_ms;       // controller uptime, for spotting an unnoticed reset
    uint8_t  _reserved[24];
};
#pragma pack(pop)
_Static_assert(sizeof(struct gw_health) == 32, "gw_health must be 32 bytes");

// GETINFO_GW_LINKPROF payload: the controller's free-running link profile
// (spi_link_prof_t in firmware/rp2354/src/transport_spi.h). Older controllers
// nack it.
#pragma pack(push, 1)
struct gw_linkprof {
    uint32_t isr_n, isr_us_sum, isr_us_max;  // CS-rise ISR count / time / worst
    uint32_t tx_frames, tx_bytes;            // payload frames it built, bytes
    uint32_t lim_queue;    // granted frames cut short by what it had queued
    uint32_t lim_credit;   // ... by our credit
    uint32_t win_full;     // header-only: both window slots in flight
};
#pragma pack(pop)
_Static_assert(sizeof(struct gw_linkprof) == 32, "gw_linkprof must be 32 bytes");

// GETINFO_GW_CAPPROF payload (flux_cap_prof_t in firmware/rp2354/src/flux_pio.h).
#pragma pack(push, 1)
struct gw_capprof {
    uint32_t captures, overflows;
    uint32_t ring_hiwater;            // most samples waiting in the 512-word ring
    uint32_t min_len, live_bytes;     // shortest capture this job; bytes sent while capturing
    uint32_t tail_bytes;              // sent after the capture
    uint32_t last_len, min_len_us;    // last capture's length; the shortest's duration
};
#pragma pack(pop)
_Static_assert(sizeof(struct gw_capprof) == 32, "gw_capprof must be 32 bytes");

// GetInfo(FIRMWARE) payload (upstream struct gw_info).
#pragma pack(push, 1)
struct gw_info_firmware {
    uint8_t  fw_ver_major;
    uint8_t  fw_ver_minor;
    uint8_t  is_main_firmware; // 0 = bootloader/update mode, 1 = main
    uint8_t  max_cmd;
    uint32_t sample_freq;      // flux tick clock, Hz  <-- drives the MFM PLL
    uint8_t  hw_model;
    uint8_t  hw_submodel;
    uint8_t  usb_speed;
    uint8_t  mcu_id;
    uint16_t mcu_mhz;
    uint16_t mcu_sram_kb;
    uint16_t usb_buf_kb;
    uint8_t  _reserved[14];
};
#pragma pack(pop)
_Static_assert(sizeof(struct gw_info_firmware) == 32, "gw_info must be 32 bytes");

// Flux stream opcodes (the byte after an 0xFF escape). See flux_decode.c.
enum gw_fluxop {
    FLUXOP_INDEX = 1,
    FLUXOP_SPACE = 2,
    FLUXOP_ASTABLE = 3,
};

enum gw_bus { BUS_NONE = 0, BUS_IBMPC = 1, BUS_SHUGART = 2 };
