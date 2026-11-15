// Bit-banged SWD master, written from the ADI spec and the RP2350 datasheet.
// The pins are shared with the J12 debug header and go high-Z after each
// flash (rp_swd_detach).
//
// Wire format:
//  - The host changes SWDIO on the falling edge; the target samples on the
//    rising edge. Reads mirror this.
//  - Request byte {start=1, APnDP, RnW, A[2:3], parity, stop=0, park=1},
//    turnaround, 3-bit ACK, then 32 data bits + parity (writes have a
//    turnaround before the data).
//  - ACK 0b001 = OK, 0b010 = WAIT (retry), 0b100 = FAULT (clear via ABORT).
#include "rp_swd.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include "app_config.h"

static const char *TAG = "rpswd";

// ---- DP / AP register addresses (ADIv5) ----
#define DP_ABORT 0x0    // write-only
#define DP_IDCODE 0x0   // read-only
#define DP_CTRLSTAT 0x4 // when DP_SELECT.DPBANKSEL == 0
#define DP_SELECT 0x8   // write-only
#define DP_RDBUFF 0xC   // read-only
#define DP_TARGETSEL 0xC // write-only (multidrop, during line reset)

// ---- ADIv6 AP addressing (RP2350) ----
// The core-0 AHB-AP is at AP address 0x2000; its registers are reached by
// putting the AP register address in DP_SELECT[31:4]. ADIv5 APSEL addressing
// does not reach it.
#define AP_CORE0_BASE 0x2000u
#define AP_CSW 0xD00u
#define AP_TAR 0xD04u
#define AP_DRW 0xD0Cu
#define AP_IDR 0xDFCu

// CSW: MasterType=debugger (b29), HPROT1 privileged (b25), HPROT0 data (b24),
// AddrInc = single (b5:4 = 1), Size = 32-bit (b2:0 = 2). Packed increment
// would need byte-packed DRW handling this driver doesn't do.
#define CSW_SECURE 0x23000012u
#define CSW_NONSEC 0x63000012u

#define CTRLSTAT_PWRUP 0x50000000u // CDBGPWRUPREQ | CSYSPWRUPREQ
#define CTRLSTAT_PWRACK 0xA0000000u // CDBGPWRUPACK | CSYSPWRUPACK
#define ABORT_ALLERR 0x0000001Eu    // STKCMPCLR|STKERRCLR|WDERRCLR|ORUNERRCLR

// ---- ARMv8-M debug registers (RP2350 Cortex-M33) ----
#define DHCSR 0xE000EDF0u
#define DCRSR 0xE000EDF4u
#define DCRDR 0xE000EDF8u
#define DEMCR 0xE000EDFCu
#define DHCSR_KEY 0xA05F0000u
#define DHCSR_C_DEBUGEN 0x1u
#define DHCSR_C_HALT 0x2u
#define DHCSR_S_HALT 0x20000u
#define DHCSR_S_REGRDY 0x10000u

// Verified on the gen-4 board (firmware/esp32-bringup tries every connect
// sequence): dormant-to-SWD, line reset, TARGETSEL 0x00040927, then DPIDR
// reads 0x4C013477 and the AHB-AP IDR 0x34770008.
#define RP_TARGETSEL_CORE0 0x00040927u
#define RP_IDCODE_EXPECT 0x4C013477u
// Off: the top nibble is a silicon revision, so a strict check would reject
// future revs.
#define RP_IDCODE_CHECK_STRICT 0

// ~500 kHz: slow enough for the 100R series resistors; SWD has no minimum.
#define SWD_HALF_US 1

static uint32_t s_idcode;
static bool s_attached;

// ---- pin primitives ----

static inline void clk_low(void) { gpio_set_level(RP_SWCLK_GPIO, 0); }
static inline void clk_high(void) { gpio_set_level(RP_SWCLK_GPIO, 1); }

static inline void dio_out(int level)
{
    gpio_set_direction(RP_SWDIO_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(RP_SWDIO_GPIO, level);
}

static inline void dio_in(void)
{
    gpio_set_direction(RP_SWDIO_GPIO, GPIO_MODE_INPUT);
}

static void wr_bit(int bit)
{
    gpio_set_level(RP_SWDIO_GPIO, bit);
    esp_rom_delay_us(SWD_HALF_US);
    clk_high();
    esp_rom_delay_us(SWD_HALF_US);
    clk_low();
}

// Sample just before the rising edge, after a full low phase.
static int rd_bit(void)
{
    esp_rom_delay_us(SWD_HALF_US);
    int b = gpio_get_level(RP_SWDIO_GPIO);
    clk_high();
    esp_rom_delay_us(SWD_HALF_US);
    clk_low();
    return b;
}

static void wr_bits(uint32_t v, int n)
{
    dio_out(v & 1);
    for (int i = 0; i < n; i++)
        wr_bit((v >> i) & 1);
}

static uint32_t rd_bits(int n)
{
    uint32_t v = 0;
    for (int i = 0; i < n; i++)
        v |= (uint32_t)rd_bit() << i;
    return v;
}

// One clock with nobody driving.
static void turnaround(void)
{
    dio_in();
    esp_rom_delay_us(SWD_HALF_US);
    clk_high();
    esp_rom_delay_us(SWD_HALF_US);
    clk_low();
}

static int parity32(uint32_t v)
{
    v ^= v >> 16;
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return (int)(v & 1);
}

// ---- transactions ----

static uint8_t request_byte(bool ap, bool rd, uint8_t addr)
{
    uint8_t a2 = (addr >> 2) & 1, a3 = (addr >> 3) & 1;
    uint8_t p = (uint8_t)((ap ? 1 : 0) ^ (rd ? 1 : 0) ^ a2 ^ a3) & 1;
    return (uint8_t)(0x81 |            // start (bit0) + park (bit7)
                     ((ap ? 1 : 0) << 1) | ((rd ? 1 : 0) << 2) |
                     (a2 << 3) | (a3 << 4) | (p << 5)); // stop (bit6) = 0
}

#define ACK_OK 1
#define ACK_WAIT 2
#define ACK_FAULT 4

static int xfer(bool ap, bool rd, uint8_t addr, uint32_t *data)
{
    wr_bits(request_byte(ap, rd, addr), 8);
    turnaround();
    int ack = (int)rd_bits(3);

    if (ack != ACK_OK) {
        // No data phase follows WAIT/FAULT.
        turnaround();
        dio_out(0);
        return ack;
    }

    if (rd) {
        uint32_t v = rd_bits(32);
        int p = rd_bit();
        turnaround();
        dio_out(0);
        if (p != parity32(v))
            return ACK_FAULT; // parity error: treat as a bus fault
        *data = v;
    } else {
        turnaround();
        wr_bits(*data, 32);
        wr_bit(parity32(*data));
        dio_out(0);
    }
    // Idle cycles so the target completes the transaction.
    for (int i = 0; i < 8; i++)
        wr_bit(0);
    return ACK_OK;
}

// Retries WAIT; on FAULT clears the sticky errors and retries.
static esp_err_t xfer_retry(bool ap, bool rd, uint8_t addr, uint32_t *data)
{
    for (int attempt = 0; attempt < 3; attempt++) {
        int ack = xfer(ap, rd, addr, data);
        if (ack == ACK_OK)
            return ESP_OK;
        if (ack == ACK_WAIT) {
            esp_rom_delay_us(50);
            continue;
        }
        uint32_t ab = ABORT_ALLERR;
        (void)xfer(false, false, DP_ABORT, &ab);
    }
    return ESP_FAIL;
}

static esp_err_t dp_write(uint8_t addr, uint32_t v)
{
    return xfer_retry(false, false, addr, &v);
}

static esp_err_t dp_read(uint8_t addr, uint32_t *v)
{
    return xfer_retry(false, true, addr, v);
}

// AP reads are posted: the value comes from RDBUFF. DP_SELECT[31:4] holds the
// AP register address, A[3:2] picks the word. s_select caches it to skip
// redundant SELECT writes.
static uint32_t s_select = 0xFFFFFFFFu;

static esp_err_t ap_select(uint32_t reg)
{
    uint32_t sel = (AP_CORE0_BASE + reg) & ~0xFu;
    if (sel == s_select)
        return ESP_OK;
    esp_err_t e = dp_write(DP_SELECT, sel);
    if (e == ESP_OK)
        s_select = sel;
    return e;
}

static esp_err_t ap_write(uint32_t reg, uint32_t v)
{
    esp_err_t e = ap_select(reg);
    if (e != ESP_OK)
        return e;
    return xfer_retry(true, false, (uint8_t)(reg & 0xC), &v);
}

static esp_err_t ap_read(uint32_t reg, uint32_t *v)
{
    esp_err_t e = ap_select(reg);
    if (e != ESP_OK)
        return e;
    uint32_t dummy;
    e = xfer_retry(true, true, (uint8_t)(reg & 0xC), &dummy); // posted
    if (e != ESP_OK)
        return e;
    return dp_read(DP_RDBUFF, v);
}

// ---- line sequences ----

static void line_reset(void)
{
    // >= 50 clocks with SWDIO high, then two idle bits.
    dio_out(1);
    for (int i = 0; i < 56; i++)
        wr_bit(1);
    for (int i = 0; i < 2; i++)
        wr_bit(0);
}

// The RP2350 powers up dormant: 8 clocks high, the 128-bit selection alert
// (LSB first), 4 low clocks, then the SWD activation code 0x1A.
static void dormant_to_swd(void)
{
    static const uint32_t alert[4] = {
        0x6209F392u, 0x86852D95u, 0xE3DDAFE9u, 0x19BC0EA2u,
    };
    dio_out(1);
    for (int i = 0; i < 8; i++)
        wr_bit(1);
    for (int w = 0; w < 4; w++)
        wr_bits(alert[w], 32);
    for (int i = 0; i < 4; i++)
        wr_bit(0);
    wr_bits(0x1A, 8);
}

// Multidrop: line reset, TARGETSEL (no target drives its ACK), read IDCODE.
static esp_err_t select_target(uint32_t targetsel, uint32_t *idcode)
{
    line_reset();

    wr_bits(request_byte(false, false, DP_TARGETSEL), 8);
    turnaround();
    (void)rd_bits(3);
    turnaround();
    wr_bits(targetsel, 32);
    wr_bit(parity32(targetsel));
    dio_out(0);
    for (int i = 0; i < 8; i++)
        wr_bit(0);

    return dp_read(DP_IDCODE, idcode);
}

// ---- public API ----

uint32_t rp_swd_idcode(void) { return s_idcode; }

esp_err_t rp_swd_attach(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << RP_SWCLK_GPIO) | (1ULL << RP_SWDIO_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    clk_low();
    dio_out(1);
    s_attached = true;

    dormant_to_swd();

    esp_err_t e = select_target(RP_TARGETSEL_CORE0, &s_idcode);
    if (e != ESP_OK || s_idcode == 0 || s_idcode == 0xFFFFFFFFu) {
        ESP_LOGE(TAG, "no SWD target (IDCODE=0x%08lX)", (unsigned long)s_idcode);
        rp_swd_detach();
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "SWD attached, IDCODE=0x%08lX", (unsigned long)s_idcode);
#if RP_IDCODE_CHECK_STRICT
    if (s_idcode != RP_IDCODE_EXPECT) {
        ESP_LOGE(TAG, "unexpected IDCODE (want 0x%08lX)",
                 (unsigned long)RP_IDCODE_EXPECT);
        rp_swd_detach();
        return ESP_ERR_NOT_FOUND;
    }
#endif

    // Clear sticky errors, then power up the debug and system domains.
    if (dp_write(DP_ABORT, ABORT_ALLERR) != ESP_OK ||
        dp_write(DP_SELECT, 0) != ESP_OK ||
        dp_write(DP_CTRLSTAT, CTRLSTAT_PWRUP) != ESP_OK) {
        rp_swd_detach();
        return ESP_FAIL;
    }
    s_select = 0;
    for (int i = 0;; i++) {
        uint32_t cs = 0;
        if (dp_read(DP_CTRLSTAT, &cs) != ESP_OK) {
            rp_swd_detach();
            return ESP_FAIL;
        }
        if ((cs & CTRLSTAT_PWRACK) == CTRLSTAT_PWRACK)
            break;
        if (i >= 100) {
            ESP_LOGE(TAG, "debug power-up not acked (CTRL/STAT=0x%08lX)",
                     (unsigned long)cs);
            rp_swd_detach();
            return ESP_FAIL;
        }
        esp_rom_delay_us(100);
    }

    // Try secure access, then non-secure, proving each with a read of ROM[0].
    {
        static const uint32_t CSWS[2] = { CSW_SECURE, CSW_NONSEC };
        static const char *const NAMES[2] = { "secure", "non-secure" };
        bool up = false;
        for (int i = 0; i < 2 && !up; i++) {
            uint32_t probe = 0;
            if (ap_write(AP_CSW, CSWS[i]) != ESP_OK)
                continue;
            if (rp_swd_read32(0x00000000u, &probe) != ESP_OK)
                continue;
            ESP_LOGI(TAG, "MEM-AP up (%s access), ROM[0]=0x%08lX", NAMES[i],
                     (unsigned long)probe);
            up = true;
        }
        if (!up) {
            ESP_LOGE(TAG, "MEM-AP would not come up (tried secure and non-secure)");
            rp_swd_detach();
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

void rp_swd_detach(void)
{
    if (!s_attached)
        return;
    gpio_set_direction(RP_SWDIO_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(RP_SWCLK_GPIO, GPIO_MODE_INPUT);
    s_attached = false;
}

esp_err_t rp_swd_read32(uint32_t addr, uint32_t *out)
{
    esp_err_t e = ap_write(AP_TAR, addr);
    if (e != ESP_OK)
        return e;
    return ap_read(AP_DRW, out);
}

esp_err_t rp_swd_write32(uint32_t addr, uint32_t val)
{
    esp_err_t e = ap_write(AP_TAR, addr);
    if (e != ESP_OK)
        return e;
    return ap_write(AP_DRW, val);
}

// TAR auto-increment only works within 1 KB, so reload it at each boundary.
#define TAR_WRAP 1024u

esp_err_t rp_swd_write_mem(uint32_t addr, const void *src, size_t len)
{
    const uint8_t *p = (const uint8_t *)src;
    if ((addr & 3) || (len & 3))
        return ESP_ERR_INVALID_ARG;
    while (len) {
        esp_err_t e = ap_write(AP_TAR, addr);
        if (e != ESP_OK)
            return e;
        uint32_t room = TAR_WRAP - (addr & (TAR_WRAP - 1));
        uint32_t n = (len < room) ? (uint32_t)len : room;
        for (uint32_t i = 0; i < n; i += 4) {
            uint32_t w;
            memcpy(&w, p + i, 4);
            if ((e = ap_write(AP_DRW, w)) != ESP_OK)
                return e;
        }
        addr += n;
        p += n;
        len -= n;
    }
    return ESP_OK;
}

esp_err_t rp_swd_read_mem(uint32_t addr, void *dst, size_t len)
{
    uint8_t *p = (uint8_t *)dst;
    if ((addr & 3) || (len & 3))
        return ESP_ERR_INVALID_ARG;
    while (len) {
        esp_err_t e = ap_write(AP_TAR, addr);
        if (e != ESP_OK)
            return e;
        uint32_t room = TAR_WRAP - (addr & (TAR_WRAP - 1));
        uint32_t n = (len < room) ? (uint32_t)len : room;
        for (uint32_t i = 0; i < n; i += 4) {
            uint32_t w;
            if ((e = ap_read(AP_DRW, &w)) != ESP_OK)
                return e;
            memcpy(p + i, &w, 4);
        }
        addr += n;
        p += n;
        len -= n;
    }
    return ESP_OK;
}

esp_err_t rp_swd_halt(void)
{
    esp_err_t e = rp_swd_write32(DHCSR, DHCSR_KEY | DHCSR_C_DEBUGEN |
                                            DHCSR_C_HALT);
    if (e != ESP_OK)
        return e;
    for (int i = 0; i < 100; i++) {
        uint32_t v = 0;
        if ((e = rp_swd_read32(DHCSR, &v)) != ESP_OK)
            return e;
        if (v & DHCSR_S_HALT)
            return ESP_OK;
        esp_rom_delay_us(100);
    }
    ESP_LOGE(TAG, "core did not halt");
    return ESP_ERR_TIMEOUT;
}

// Write a core register (13 = SP_main, 15 = PC, 16 = xPSR), waiting for
// DHCSR.S_REGRDY before the next DCRDR/DCRSR access.
static esp_err_t write_core_reg(uint32_t regsel, uint32_t val)
{
    esp_err_t e = rp_swd_write32(DCRDR, val);
    if (e != ESP_OK)
        return e;
    if ((e = rp_swd_write32(DCRSR, 0x00010000u | regsel)) != ESP_OK) // REGWnR
        return e;
    for (int i = 0; i < 100; i++) {
        uint32_t v = 0;
        if ((e = rp_swd_read32(DHCSR, &v)) != ESP_OK)
            return e;
        if (v & DHCSR_S_REGRDY)
            return ESP_OK;
    }
    ESP_LOGE(TAG, "core register %lu write never completed", (unsigned long)regsel);
    return ESP_ERR_TIMEOUT;
}

esp_err_t rp_swd_run_stub(uint32_t pc, uint32_t sp)
{
    esp_err_t e;
    if ((e = write_core_reg(13, sp)) != ESP_OK)
        return e;
    if ((e = write_core_reg(15, pc)) != ESP_OK)
        return e;
    if ((e = write_core_reg(16, 0x01000000u)) != ESP_OK) // xPSR.T, or it faults
        return e;
    // Keep C_DEBUGEN so we can re-halt.
    return rp_swd_write32(DHCSR, DHCSR_KEY | DHCSR_C_DEBUGEN);
}
