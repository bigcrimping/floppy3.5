// Bit-banged SWD master for the RP2354 (RP2350 die) — ADIv6 debug port.
//
// Differences from the production ../esp32/main/rp_swd.c, which was written
// ADIv5-style with "verify at bench" placeholders:
//   * The RP2350 has ONE SW-DP (DPIDR 0x4C013477, DPv3 => ADIv6). It is not
//     a 3-DP multidrop chain like the RP2040, so TARGETSEL is optional; its
//     TARGETID is 0x00040927 (OpenOCD rp2350.cfg: -dp-id 0x0040927
//     -instance-id 0), and the RP2040 value 0x01002927 must NOT be written —
//     a mismatching TARGETSEL deselects the DP until the next line reset.
//   * ADIv6 APs are addressed through DP.SELECT[31:4]; the Cortex-M33 core-0
//     AHB-AP sits at 0x2000 and its MEM-AP registers are at the ADIv6
//     offsets (CSW 0xD00, TAR 0xD04, DRW 0xD0C, IDR 0xDFC).
// The connect sequence tries the plausible line protocols in turn and
// reports which one worked, so the production driver can be fixed from the
// bench log with no guessing.
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t   bu_swd_attach(void);      // ESP_OK, or ESP_ERR_NOT_FOUND if nothing answers
void        bu_swd_detach(void);      // both pins back to high-Z
uint32_t    bu_swd_dpidr(void);
uint32_t    bu_swd_ap_idr(void);
const char *bu_swd_connect_how(void); // which line sequence attached ("jtag-to-swd", ...)
const char *bu_swd_csw_how(void);     // "secure" / "non-secure" AP access that worked

esp_err_t bu_swd_read32(uint32_t addr, uint32_t *out);
esp_err_t bu_swd_write32(uint32_t addr, uint32_t val);
esp_err_t bu_swd_halt(void);          // halt core 0 (DHCSR)
