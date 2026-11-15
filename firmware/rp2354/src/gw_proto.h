// SPDX-License-Identifier: Unlicense
// Greaseweazle protocol constants. Must agree with firmware/esp32/main/gw_proto.h.
#pragma once
#include <stdint.h>

enum gw_cmd {
  CMD_GET_INFO = 0,
  CMD_UPDATE = 1,        // unsupported
  CMD_SEEK = 2,
  CMD_HEAD = 3,
  CMD_SET_PARAMS = 4,
  CMD_GET_PARAMS = 5,
  CMD_MOTOR = 6,
  CMD_READ_FLUX = 7,
  CMD_WRITE_FLUX = 8,
  CMD_GET_FLUX_STATUS = 9,
  CMD_GET_INDEX_TIMES = 10, // deprecated upstream; unsupported
  CMD_SWITCH_FW_MODE = 11,  // unsupported
  CMD_SELECT = 12,
  CMD_DESELECT = 13,
  CMD_SET_BUS_TYPE = 14,
  CMD_SET_PIN = 15,
  CMD_RESET = 16,
  CMD_ERASE_FLUX = 17,
  CMD_SOURCE_BYTES = 18,
  CMD_SINK_BYTES = 19,
  CMD_GET_PIN = 20,
  CMD_TEST_MODE = 21,   // unsupported
  CMD_NO_CLICK_STEP = 22, // unsupported
};

enum gw_ack {
  ACK_OKAY = 0,
  ACK_BAD_COMMAND = 1,
  ACK_NO_INDEX = 2,
  ACK_NO_TRK0 = 3,
  ACK_FLUX_OVERFLOW = 4,
  ACK_FLUX_UNDERFLOW = 5,
  ACK_WRPROT = 6,
  ACK_NO_UNIT = 7,
  ACK_NO_BUS = 8,
  ACK_BAD_UNIT = 9,
  ACK_BAD_PIN = 10,
  ACK_BAD_CYLINDER = 11,
  ACK_OUT_OF_SRAM = 12,
  ACK_OUT_OF_FLASH = 13,
};

enum gw_getinfo_sub {
  GETINFO_FIRMWARE = 0,
  GETINFO_BW_STATS = 1,
  GETINFO_CURRENT_DRIVE = 7,
  GETINFO_GW_DIAG = 0x88,   // ours: diag counters
  GETINFO_GW_HEALTH = 0x89, // ours: die temperature + uptime
};

// GetInfo(FIRMWARE) payload, packed little-endian.
typedef struct __attribute__((packed)) {
  uint8_t fw_major;
  uint8_t fw_minor;
  uint8_t is_main_firmware;
  uint8_t max_cmd;
  uint32_t sample_freq;
  uint8_t hw_model;
  uint8_t hw_submodel;
  uint8_t usb_speed; // 0 = full speed
  uint8_t mcu_id;
  uint16_t mcu_mhz;
  uint16_t mcu_sram_kb;
  uint16_t usb_buf_kb;
  uint8_t reserved[14];
} gw_info_t;
_Static_assert(sizeof(gw_info_t) == 32, "gw_info must be 32 bytes");

// GETINFO_GW_HEALTH payload; must match struct gw_health in the ESP's gw_proto.h.
typedef struct __attribute__((packed)) {
  int16_t die_temp_c100; // die temperature, hundredths of a degree C
  uint8_t temp_valid;    // 0 = not sampled; die_temp_c100 meaningless
  uint8_t _pad;
  uint32_t uptime_ms;
  uint8_t _reserved[24];
} gw_health_t;
_Static_assert(sizeof(gw_health_t) == 32, "gw_health must be 32 bytes");

enum gw_fluxop { FLUXOP_INDEX = 1, FLUXOP_SPACE = 2, FLUXOP_ASTABLE = 3 };

enum gw_bus { BUS_NONE = 0, BUS_IBMPC = 1, BUS_SHUGART = 2 };

// SET_PARAMS/GET_PARAMS index 0 (upstream struct gw_delay).
typedef struct __attribute__((packed)) {
  uint16_t select_delay; // us
  uint16_t step_delay;   // us
  uint16_t seek_settle;  // ms
  uint16_t motor_delay;  // ms
  uint16_t watchdog;     // ms
} gw_delay_t;
