// Bit-banged SWD master for the RP2354A (RP2350, ADIv6 debug port). Used by
// rp_flash.c to halt the controller, load a RAM stub and program its flash.
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

// Dormant-to-SWD, TARGETSEL for core 0, DP power-up and MEM-AP setup.
// ESP_ERR_NOT_FOUND if no target answers.
esp_err_t rp_swd_attach(void);

// Both pins back to high-Z so a probe on J12 can take over. Always safe.
void rp_swd_detach(void);

uint32_t rp_swd_idcode(void);

// MEM-AP access. Word-aligned addresses; `len` in bytes, a multiple of 4.
esp_err_t rp_swd_read_mem(uint32_t addr, void *dst, size_t len);
esp_err_t rp_swd_write_mem(uint32_t addr, const void *src, size_t len);
esp_err_t rp_swd_read32(uint32_t addr, uint32_t *out);
esp_err_t rp_swd_write32(uint32_t addr, uint32_t val);

esp_err_t rp_swd_halt(void);
esp_err_t rp_swd_run_stub(uint32_t pc, uint32_t sp); // set SP/PC, then resume
