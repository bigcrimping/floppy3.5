// Bit-banged SWD master (bu_swd.h). Wire-level primitives follow the ADIv5/6
// SWD protocol: host drives SWDIO on the falling clock edge, target samples on
// the rising edge; reads are the mirror. Request byte = {start=1, APnDP, RnW,
// A[2:3], parity, stop=0, park=1}; then turnaround, 3-bit ACK, 32 data + parity.
#include "bu_swd.h"

#include <string.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "app_config.h"     // RP_SWCLK_GPIO / RP_SWDIO_GPIO

static const char *TAG = "swd";

// ---- DP registers ----
#define DP_ABORT     0x0    // W
#define DP_DPIDR     0x0    // R
#define DP_CTRLSTAT  0x4    // DPBANKSEL 0
#define DP_SELECT    0x8    // W
#define DP_RDBUFF    0xC    // R
#define DP_TARGETSEL 0xC    // W (no ACK)

// ---- RP2350 identity (OpenOCD tcl/target/rp2350.cfg; DPIDR from OpenOCD logs) ----
#define RP2350_TARGETSEL   0x00040927u
#define RP2350_DPIDR       0x4C013477u

// ---- ADIv6 AP addressing ----
#define AP_CORE0_BASE 0x2000u
#define AP_CSW 0xD00u
#define AP_TAR 0xD04u
#define AP_DRW 0xD0Cu
#define AP_IDR 0xDFCu

// CSW: MasterType=debugger (bit29), HPROT1 privileged (25), HPROT0 data (24),
// AddrInc=single (5:4 = 01), Size=32-bit (2:0 = 010). HNONSEC (30) clear =
// secure access, which is what the debugger normally gets on a fresh RP2350.
#define CSW_SECURE  0x23000012u
#define CSW_NONSEC  0x63000012u

#define CTRLSTAT_PWRUP  0x50000000u   // CDBGPWRUPREQ | CSYSPWRUPREQ
#define CTRLSTAT_PWRACK 0xA0000000u   // CDBGPWRUPACK | CSYSPWRUPACK
#define ABORT_ALLERR    0x0000001Eu

// ---- ARMv8-M debug ----
#define DHCSR          0xE000EDF0u
#define DHCSR_KEY      0xA05F0000u
#define DHCSR_C_DEBUGEN 0x1u
#define DHCSR_C_HALT    0x2u
#define DHCSR_S_HALT    0x20000u

#define SWD_HALF_US 1     // ~500 kHz; over 100R series resistors + cm of track

static bool     s_attached;
static uint32_t s_dpidr, s_apidr;
static uint32_t s_select = 0xFFFFFFFFu;   // last DP.SELECT written
static uint32_t s_csw;
static const char *s_how = "none";
static const char *s_csw_how = "none";

// ---------------- pin primitives ----------------

static inline void clk_low(void)  { gpio_set_level(RP_SWCLK_GPIO, 0); }
static inline void clk_high(void) { gpio_set_level(RP_SWCLK_GPIO, 1); }

static inline void dio_out(int level)
{
    gpio_set_direction(RP_SWDIO_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(RP_SWDIO_GPIO, level);
}
static inline void dio_in(void) { gpio_set_direction(RP_SWDIO_GPIO, GPIO_MODE_INPUT); }

static void wr_bit(int bit)
{
    gpio_set_level(RP_SWDIO_GPIO, bit);
    esp_rom_delay_us(SWD_HALF_US);
    clk_high();
    esp_rom_delay_us(SWD_HALF_US);
    clk_low();
}

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
    for (int i = 0; i < n; i++) wr_bit((v >> i) & 1);
}

static uint32_t rd_bits(int n)
{
    uint32_t v = 0;
    for (int i = 0; i < n; i++) v |= (uint32_t)rd_bit() << i;
    return v;
}

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
    v ^= v >> 16; v ^= v >> 8; v ^= v >> 4; v ^= v >> 2; v ^= v >> 1;
    return (int)(v & 1);
}

// ---------------- transactions ----------------

static uint8_t request_byte(bool ap, bool rd, uint8_t addr)
{
    uint8_t a2 = (addr >> 2) & 1, a3 = (addr >> 3) & 1;
    uint8_t p = (uint8_t)((ap ? 1 : 0) ^ (rd ? 1 : 0) ^ a2 ^ a3) & 1;
    return (uint8_t)(0x81 | ((ap ? 1 : 0) << 1) | ((rd ? 1 : 0) << 2) |
                     (a2 << 3) | (a3 << 4) | (p << 5));
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
        turnaround();
        dio_out(0);
        return ack;
    }
    if (rd) {
        uint32_t v = rd_bits(32);
        int p = rd_bit();
        turnaround();
        dio_out(0);
        if (p != parity32(v)) return ACK_FAULT;
        *data = v;
    } else {
        turnaround();
        wr_bits(*data, 32);
        wr_bit(parity32(*data));
        dio_out(0);
    }
    for (int i = 0; i < 8; i++) wr_bit(0);   // idle so the target completes it
    return ACK_OK;
}

static int last_ack;

static esp_err_t xfer_retry(bool ap, bool rd, uint8_t addr, uint32_t *data)
{
    for (int attempt = 0; attempt < 4; attempt++) {
        int ack = xfer(ap, rd, addr, data);
        last_ack = ack;
        if (ack == ACK_OK) return ESP_OK;
        if (ack == ACK_WAIT) { esp_rom_delay_us(50); continue; }
        // FAULT / parity / no target (7): clear sticky errors and retry once.
        uint32_t ab = ABORT_ALLERR;
        (void)xfer(false, false, DP_ABORT, &ab);
    }
    return ESP_FAIL;
}

static esp_err_t dp_write(uint8_t addr, uint32_t v) { return xfer_retry(false, false, addr, &v); }
static esp_err_t dp_read(uint8_t addr, uint32_t *v)  { return xfer_retry(false, true, addr, v); }

// ADIv6: SELECT[31:4] holds the AP register address bits [31:4]; the request's
// A[3:2] picks the word within that 16-byte bank. DPBANKSEL stays 0.
static esp_err_t ap_select(uint32_t reg)
{
    uint32_t sel = (AP_CORE0_BASE + reg) & ~0xFu;
    if (sel == s_select) return ESP_OK;
    esp_err_t e = dp_write(DP_SELECT, sel);
    if (e == ESP_OK) s_select = sel;
    return e;
}

static esp_err_t ap_write(uint32_t reg, uint32_t v)
{
    esp_err_t e = ap_select(reg);
    if (e != ESP_OK) return e;
    return xfer_retry(true, false, (uint8_t)(reg & 0xC), &v);
}

static esp_err_t ap_read(uint32_t reg, uint32_t *v)
{
    esp_err_t e = ap_select(reg);
    if (e != ESP_OK) return e;
    uint32_t dummy;
    e = xfer_retry(true, true, (uint8_t)(reg & 0xC), &dummy);   // posted
    if (e != ESP_OK) return e;
    return dp_read(DP_RDBUFF, v);
}

// ---------------- line sequences ----------------

static void line_reset(void)
{
    dio_out(1);
    for (int i = 0; i < 56; i++) wr_bit(1);
    for (int i = 0; i < 2; i++) wr_bit(0);
}

// Deprecated-but-universal JTAG-to-SWD select: 0xE79E LSB-first between resets.
static void jtag_to_swd(void)
{
    line_reset();
    wr_bits(0xE79E, 16);
    line_reset();
}

// SWD-to-dormant: line reset then 0xE3BC.
static void swd_to_dormant(void)
{
    line_reset();
    wr_bits(0xE3BC, 16);
}

// Dormant-to-SWD (ADIv5.2 B5.3): 8 clocks high, the 128-bit selection alert
// (LSB first), 4 idle clocks, then the SW-DP activation code 0x1A.
static void dormant_to_swd(void)
{
    static const uint32_t alert[4] = { 0x6209F392u, 0x86852D95u, 0xE3DDAFE9u, 0x19BC0EA2u };
    dio_out(1);
    for (int i = 0; i < 8; i++) wr_bit(1);
    for (int w = 0; w < 4; w++) wr_bits(alert[w], 32);
    for (int i = 0; i < 4; i++) wr_bit(0);
    wr_bits(0x1A, 8);
}

// TARGETSEL write: the ACK phase is not driven by any target.
static void targetsel(uint32_t v)
{
    wr_bits(request_byte(false, false, DP_TARGETSEL), 8);
    turnaround();
    (void)rd_bits(3);
    turnaround();
    wr_bits(v, 32);
    wr_bit(parity32(v));
    dio_out(0);
    for (int i = 0; i < 8; i++) wr_bit(0);
}

static bool read_dpidr_ok(void)
{
    uint32_t id = 0;
    int ack = xfer(false, true, DP_DPIDR, &id);
    if (ack != ACK_OK || id == 0 || id == 0xFFFFFFFFu) return false;
    s_dpidr = id;
    return true;
}

// The candidate connect sequences, cheapest and least invasive first. Every
// variant ends with a line reset so a wrong guess can't poison the next one.
typedef struct { const char *name; void (*run)(void); } variant_t;

static void v_jtag(void)        { jtag_to_swd(); }
static void v_dormant_ts(void)  { dormant_to_swd(); line_reset(); targetsel(RP2350_TARGETSEL); }
static void v_dormant(void)     { dormant_to_swd(); line_reset(); }
static void v_jtag_ts(void)     { jtag_to_swd(); targetsel(RP2350_TARGETSEL); }
static void v_via_dormant(void) { swd_to_dormant(); dormant_to_swd(); line_reset(); }

static const variant_t VARIANTS[] = {
    { "jtag-to-swd",                 v_jtag },
    { "dormant-to-swd + TARGETSEL",  v_dormant_ts },
    { "dormant-to-swd",              v_dormant },
    { "jtag-to-swd + TARGETSEL",     v_jtag_ts },
    { "swd-to-dormant-to-swd",       v_via_dormant },
};

// ---------------- public API ----------------

uint32_t    bu_swd_dpidr(void)       { return s_dpidr; }
uint32_t    bu_swd_ap_idr(void)      { return s_apidr; }
const char *bu_swd_connect_how(void) { return s_how; }
const char *bu_swd_csw_how(void)     { return s_csw_how; }

esp_err_t bu_swd_attach(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << RP_SWCLK_GPIO) | (1ULL << RP_SWDIO_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    clk_low();
    dio_out(1);
    s_attached = true;
    s_select   = 0xFFFFFFFFu;
    s_dpidr = s_apidr = 0;
    s_how = "none";
    s_csw_how = "none";

    bool ok = false;
    for (size_t v = 0; v < sizeof(VARIANTS) / sizeof(VARIANTS[0]) && !ok; v++) {
        for (int attempt = 0; attempt < 2 && !ok; attempt++) {
            VARIANTS[v].run();
            for (int i = 0; i < 2; i++) wr_bit(0);
            if (read_dpidr_ok()) { ok = true; s_how = VARIANTS[v].name; }
        }
    }
    if (!ok) {
        ESP_LOGW(TAG, "no SWD target answered (SWDIO idle=%d)", gpio_get_level(RP_SWDIO_GPIO));
        bu_swd_detach();
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "DPIDR=0x%08lX via %s%s", (unsigned long)s_dpidr, s_how,
             s_dpidr == RP2350_DPIDR ? " (RP2350)" : " (UNEXPECTED — not 0x4C013477)");

    // Sticky-error clear, debug + system power-up.
    if (dp_write(DP_ABORT, ABORT_ALLERR) != ESP_OK ||
        dp_write(DP_SELECT, 0) != ESP_OK ||
        dp_write(DP_CTRLSTAT, CTRLSTAT_PWRUP) != ESP_OK) {
        ESP_LOGE(TAG, "DP power-up request failed (ack=%d)", last_ack);
        bu_swd_detach();
        return ESP_FAIL;
    }
    s_select = 0;
    for (int i = 0;; i++) {
        uint32_t cs = 0;
        if (dp_read(DP_CTRLSTAT, &cs) != ESP_OK) { bu_swd_detach(); return ESP_FAIL; }
        if ((cs & CTRLSTAT_PWRACK) == CTRLSTAT_PWRACK) break;
        if (i >= 200) {
            ESP_LOGE(TAG, "debug power-up not acked (CTRL/STAT=0x%08lX)", (unsigned long)cs);
            bu_swd_detach();
            return ESP_FAIL;
        }
        esp_rom_delay_us(100);
    }

    // Core-0 AHB-AP identity, then a CSW that lets us read memory: secure
    // first, non-secure as the fallback. The probe read is the bootrom at 0.
    if (ap_read(AP_IDR, &s_apidr) != ESP_OK) {
        ESP_LOGE(TAG, "AP IDR read failed — no AP at 0x%04X?", AP_CORE0_BASE);
        bu_swd_detach();
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "core-0 AP IDR=0x%08lX", (unsigned long)s_apidr);

    static const struct { uint32_t csw; const char *name; } CSWS[] = {
        { CSW_SECURE, "secure" }, { CSW_NONSEC, "non-secure" },
    };
    for (size_t i = 0; i < 2; i++) {
        uint32_t probe = 0;
        if (ap_write(AP_CSW, CSWS[i].csw) != ESP_OK) continue;
        s_csw = CSWS[i].csw;
        if (bu_swd_read32(0x00000000u, &probe) == ESP_OK) {
            s_csw_how = CSWS[i].name;
            ESP_LOGI(TAG, "MEM-AP up (%s access), ROM[0]=0x%08lX", s_csw_how, (unsigned long)probe);
            return ESP_OK;
        }
        (void)dp_write(DP_ABORT, ABORT_ALLERR);
    }
    ESP_LOGE(TAG, "MEM-AP configured but a memory read faults");
    bu_swd_detach();
    return ESP_FAIL;
}

void bu_swd_detach(void)
{
    if (!s_attached) return;
    gpio_set_direction(RP_SWDIO_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(RP_SWCLK_GPIO, GPIO_MODE_INPUT);
    s_attached = false;
}

esp_err_t bu_swd_read32(uint32_t addr, uint32_t *out)
{
    if (!s_attached) return ESP_ERR_INVALID_STATE;
    esp_err_t e = ap_write(AP_TAR, addr);
    if (e != ESP_OK) return e;
    return ap_read(AP_DRW, out);
}

esp_err_t bu_swd_write32(uint32_t addr, uint32_t val)
{
    if (!s_attached) return ESP_ERR_INVALID_STATE;
    esp_err_t e = ap_write(AP_TAR, addr);
    if (e != ESP_OK) return e;
    return ap_write(AP_DRW, val);
}

esp_err_t bu_swd_halt(void)
{
    esp_err_t e = bu_swd_write32(DHCSR, DHCSR_KEY | DHCSR_C_DEBUGEN | DHCSR_C_HALT);
    if (e != ESP_OK) return e;
    for (int i = 0; i < 100; i++) {
        uint32_t v = 0;
        if ((e = bu_swd_read32(DHCSR, &v)) != ESP_OK) return e;
        if (v & DHCSR_S_HALT) return ESP_OK;
        esp_rom_delay_us(100);
    }
    return ESP_ERR_TIMEOUT;
}
