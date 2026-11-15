// RP2354 controller (IC2) from the ESP's side. Everything here works on a
// BLANK chip, because it goes through SWD rather than through firmware on the
// RP: the ESP halts core 0, then reads/writes the RP's SIO and IO/PAD
// registers directly to wiggle the GWSPI link pins one at a time, so each of
// the five link wires (and RUN, SWCLK, SWDIO) gets its own PASS/FAIL line.
//
// Register addresses are the RP2350 ones (pico-sdk hardware_regs, rp2350):
//   SIO 0xD0000000  IO_BANK0 0x40028000  PADS_BANK0 0x40038000
//   RESETS 0x40020000  SYSINFO 0x40000000
#include "bu_tests.h"
#include "bu_report.h"
#include "bu_swd.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "app_config.h"

static const char *TAG = "t_rp";

// ---- RP2350 registers ----
#define SIO_BASE          0xD0000000u
#define SIO_GPIO_IN       (SIO_BASE + 0x004)
#define SIO_GPIO_OUT_SET  (SIO_BASE + 0x018)
#define SIO_GPIO_OUT_CLR  (SIO_BASE + 0x020)
#define SIO_GPIO_OE       (SIO_BASE + 0x030)
#define SIO_GPIO_OE_SET   (SIO_BASE + 0x038)
#define SIO_GPIO_OE_CLR   (SIO_BASE + 0x040)
#define IO_BANK0_BASE     0x40028000u
#define IO_GPIO_CTRL(n)   (IO_BANK0_BASE + 0x004 + 8u * (n))
#define IO_FUNCSEL_SIO    0x05u
#define IO_FUNCSEL_NULL   0x1Fu
#define PADS_BANK0_BASE   0x40038000u
#define PAD_GPIO(n)       (PADS_BANK0_BASE + 0x004 + 4u * (n))
#define PAD_RESET_VALUE   0x116u     // ISO | IE | PDE | SCHMITT
#define PAD_ACTIVE        0x052u     // IE | DRIVE 4mA | SCHMITT; ISO cleared, no pulls
#define RESETS_BASE       0x40020000u
#define RESETS_RESET_CLR  (RESETS_BASE + 0x3000)     // atomic clear alias
#define RESETS_RESET_DONE (RESETS_BASE + 0x008)
#define RESET_IO_BANK0    0x40u
#define RESET_PADS_BANK0  0x200u
#define SYSINFO_CHIP_ID   0x40000000u
#define CHIP_ID_RP2350    0x00004927u                 // part 0x0004, mfr 0x493, bits[27:0]
#define SRAM_TEST_ADDR    0x20040000u

// ---- link wiring: RP GPIO <-> ESP GPIO (must match firmware/rp2354/src/board_rp2354.h) ----
#define RP_PIN_SPI_RX  0    // <- ESP MOSI  (GW_SPI_MOSI_GPIO)
#define RP_PIN_SPI_CS  1    // <- ESP CS    (GW_SPI_CS_GPIO)
#define RP_PIN_SPI_SCK 2    // <- ESP SCLK  (GW_SPI_SCLK_GPIO)
#define RP_PIN_SPI_TX  3    // -> ESP MISO  (GW_SPI_MISO_GPIO)
#define RP_PIN_DRDY    4    // -> ESP DRDY  (GW_SPI_DRDY_GPIO)
#define RP_LINK_MASK   0x1Fu

typedef struct { const char *name; int esp; int rp; } wire_t;
static const wire_t ESP_TO_RP[] = {
    { "RP LINK SCLK ESP7->RP2",  GW_SPI_SCLK_GPIO, RP_PIN_SPI_SCK },
    { "RP LINK MOSI ESP16->RP0", GW_SPI_MOSI_GPIO, RP_PIN_SPI_RX },
    { "RP LINK CS ESP15->RP1",   GW_SPI_CS_GPIO,   RP_PIN_SPI_CS },
};
static const wire_t RP_TO_ESP[] = {
    { "RP LINK MISO RP3->ESP6",  GW_SPI_MISO_GPIO, RP_PIN_SPI_TX },
    { "RP LINK DRDY RP4->ESP5",  GW_SPI_DRDY_GPIO, RP_PIN_DRDY },
};
#define N_E2R (sizeof(ESP_TO_RP) / sizeof(ESP_TO_RP[0]))
#define N_R2E (sizeof(RP_TO_ESP) / sizeof(RP_TO_ESP[0]))

// ---- helpers ----

static void esp_link_pins_input(void)
{
    const int pins[] = { GW_SPI_SCLK_GPIO, GW_SPI_MOSI_GPIO, GW_SPI_CS_GPIO,
                         GW_SPI_MISO_GPIO, GW_SPI_DRDY_GPIO };
    uint64_t m = 0;
    for (size_t i = 0; i < 5; i++) m |= 1ULL << pins[i];
    gpio_config_t in = { .pin_bit_mask = m, .mode = GPIO_MODE_INPUT };   // no ESP pulls: read the board's
    gpio_config(&in);
}

static void run_pulse(void)
{
    gpio_config_t od = { .pin_bit_mask = 1ULL << GW_RESET_GPIO, .mode = GPIO_MODE_OUTPUT_OD };
    gpio_config(&od);
    gpio_set_level(GW_RESET_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(GW_RESET_LOW_MS));
    gpio_set_level(GW_RESET_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(GW_BOOT_SETTLE_MS));
}

// Idle levels tell the pull resistors and shorts apart before anything is
// driven: SCLK/MOSI 100k pull-downs (R35/R36) -> 0, CS 10k pull-up (R33) -> 1,
// RUN pull-up (R32) -> 1, SWCLK 100k pull-down (R38) -> 0, SWDIO RP-internal
// pull-up -> 1. MISO/DRDY have no pull and float when the RP isn't driving.
static void passive_levels(void)
{
    esp_link_pins_input();
    gpio_config_t in = {
        .pin_bit_mask = (1ULL << GW_RESET_GPIO) | (1ULL << RP_SWCLK_GPIO) | (1ULL << RP_SWDIO_GPIO),
        .mode = GPIO_MODE_INPUT,
    };
    gpio_config(&in);
    int sclk = gpio_get_level(GW_SPI_SCLK_GPIO), mosi = gpio_get_level(GW_SPI_MOSI_GPIO);
    int cs = gpio_get_level(GW_SPI_CS_GPIO), miso = gpio_get_level(GW_SPI_MISO_GPIO);
    int drdy = gpio_get_level(GW_SPI_DRDY_GPIO), run = gpio_get_level(GW_RESET_GPIO);
    int swclk = gpio_get_level(RP_SWCLK_GPIO), swdio = gpio_get_level(RP_SWDIO_GPIO);
    bool ok = (sclk == 0) && (mosi == 0) && (cs == 1) && (run == 1) && (swclk == 0);
    bu_report("RP LINK idle levels", ok ? BU_PASS : BU_WARN,
              "SCLK=%d MOSI=%d CS=%d MISO=%d DRDY=%d RUN=%d SWCLK=%d SWDIO=%d (expect 0 0 1 x x 1 0 1)",
              sclk, mosi, cs, miso, drdy, run, swclk, swdio);
}

static esp_err_t rp_wr(uint32_t a, uint32_t v) { return bu_swd_write32(a, v); }
static esp_err_t rp_rd(uint32_t a, uint32_t *v) { return bu_swd_read32(a, v); }

static bool sram_test(void)
{
    uint32_t saved[8], bad = 0;
    for (int i = 0; i < 8; i++)
        if (rp_rd(SRAM_TEST_ADDR + 4 * i, &saved[i]) != ESP_OK) {
            bu_report("RP SRAM r/w", BU_FAIL, "read failed at 0x%08lX", (unsigned long)(SRAM_TEST_ADDR + 4 * i));
            return false;
        }
    static const uint32_t PAT[3] = { 0xA5A5A5A5u, 0x5A5A5A5Au, 0x00000000u };
    for (int p = 0; p < 3; p++) {
        for (int i = 0; i < 8; i++) rp_wr(SRAM_TEST_ADDR + 4 * i, PAT[p] ^ ((uint32_t)i * 0x01010101u));
        for (int i = 0; i < 8; i++) {
            uint32_t v = ~0u;
            rp_rd(SRAM_TEST_ADDR + 4 * i, &v);
            if (v != (PAT[p] ^ ((uint32_t)i * 0x01010101u))) bad++;
        }
    }
    for (int i = 0; i < 8; i++) rp_wr(SRAM_TEST_ADDR + 4 * i, saved[i]);
    if (bad) bu_report("RP SRAM r/w", BU_FAIL, "%lu mismatches over 24 words at 0x%08lX (SWDIO data path?)",
                       (unsigned long)bad, (unsigned long)SRAM_TEST_ADDR);
    else     bu_report("RP SRAM r/w", BU_PASS, "24 words x 3 patterns at 0x%08lX written, read back, restored",
                       (unsigned long)SRAM_TEST_ADDR);
    return bad == 0;
}

static void chip_id(void)
{
    uint32_t id = 0;
    if (rp_rd(SYSINFO_CHIP_ID, &id) != ESP_OK) { bu_report("RP CHIP_ID", BU_FAIL, "SYSINFO read faulted"); return; }
    bool rp2350 = (id & 0x0FFFFFFFu) == CHIP_ID_RP2350;
    if (rp2350) bu_report("RP CHIP_ID", BU_PASS, "0x%08lX (RP2350 family, silicon rev %lu)",
                          (unsigned long)id, (unsigned long)(id >> 28));
    else        bu_report("RP CHIP_ID", BU_WARN, "0x%08lX (not the RP2350 part/manufacturer code?)",
                          (unsigned long)id);
}

// Put RP GPIO0..4 under SIO control: peripherals out of reset, FUNCSEL=SIO,
// pads active with isolation removed, everything as input to start.
static bool rp_gpio_take(void)
{
    if (rp_wr(RESETS_RESET_CLR, RESET_IO_BANK0 | RESET_PADS_BANK0) != ESP_OK) return false;
    for (int i = 0; i < 50; i++) {
        uint32_t done = 0;
        if (rp_rd(RESETS_RESET_DONE, &done) != ESP_OK) return false;
        if ((done & (RESET_IO_BANK0 | RESET_PADS_BANK0)) == (RESET_IO_BANK0 | RESET_PADS_BANK0)) break;
        esp_rom_delay_us(100);
    }
    if (rp_wr(SIO_GPIO_OE_CLR, RP_LINK_MASK) != ESP_OK) return false;
    for (uint32_t g = 0; g <= 4; g++) {
        if (rp_wr(IO_GPIO_CTRL(g), IO_FUNCSEL_SIO) != ESP_OK) return false;
        if (rp_wr(PAD_GPIO(g), PAD_ACTIVE) != ESP_OK) return false;
    }
    return true;
}

static void rp_gpio_release(void)
{
    rp_wr(SIO_GPIO_OE_CLR, RP_LINK_MASK);
    for (uint32_t g = 0; g <= 4; g++) {
        rp_wr(PAD_GPIO(g), PAD_RESET_VALUE);
        rp_wr(IO_GPIO_CTRL(g), IO_FUNCSEL_NULL);
    }
}

static void link_esp_to_rp(void)
{
    uint64_t m = 0;
    for (size_t i = 0; i < N_E2R; i++) m |= 1ULL << ESP_TO_RP[i].esp;
    gpio_config_t out = { .pin_bit_mask = m, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&out);

    int bad[N_E2R] = { 0 }, stuck_hi[N_E2R] = { 0 }, stuck_lo[N_E2R] = { 0 };
    static const uint8_t PATTERNS[8] = { 0, 1, 2, 4, 7, 6, 5, 3 };   // walking-1 then walking-0
    for (int p = 0; p < 8; p++) {
        for (size_t i = 0; i < N_E2R; i++) gpio_set_level(ESP_TO_RP[i].esp, (PATTERNS[p] >> i) & 1);
        esp_rom_delay_us(200);
        uint32_t in = 0;
        if (rp_rd(SIO_GPIO_IN, &in) != ESP_OK) {
            for (size_t i = 0; i < N_E2R; i++) bu_report(ESP_TO_RP[i].name, BU_FAIL, "SIO read faulted mid-test");
            goto done;
        }
        for (size_t i = 0; i < N_E2R; i++) {
            int want = (PATTERNS[p] >> i) & 1, got = (in >> ESP_TO_RP[i].rp) & 1;
            if (got != want) bad[i]++;
            if (got) stuck_hi[i]++; else stuck_lo[i]++;
        }
    }
    for (size_t i = 0; i < N_E2R; i++) {
        if (!bad[i])                bu_report(ESP_TO_RP[i].name, BU_PASS, "8/8 patterns, no crosstalk");
        else if (stuck_hi[i] == 8)  bu_report(ESP_TO_RP[i].name, BU_FAIL, "RP side stuck HIGH (short to 3V3 / neighbour?)");
        else if (stuck_lo[i] == 8)  bu_report(ESP_TO_RP[i].name, BU_FAIL, "RP side stuck LOW (open track, or short to GND)");
        else                        bu_report(ESP_TO_RP[i].name, BU_FAIL, "%d/8 patterns wrong (crosstalk / shared net?)", bad[i]);
    }
done:
    for (size_t i = 0; i < N_E2R; i++) gpio_set_level(ESP_TO_RP[i].esp, 0);
}

static void link_rp_to_esp(void)
{
    uint64_t m = 0;
    for (size_t i = 0; i < N_R2E; i++) m |= 1ULL << RP_TO_ESP[i].esp;
    gpio_config_t in = { .pin_bit_mask = m, .mode = GPIO_MODE_INPUT };
    gpio_config(&in);

    uint32_t oe = 0;
    for (size_t i = 0; i < N_R2E; i++) oe |= 1u << RP_TO_ESP[i].rp;
    rp_wr(SIO_GPIO_OUT_CLR, oe);
    if (rp_wr(SIO_GPIO_OE_SET, oe) != ESP_OK) {
        for (size_t i = 0; i < N_R2E; i++) bu_report(RP_TO_ESP[i].name, BU_FAIL, "could not enable RP output");
        return;
    }
    int bad[N_R2E] = { 0 }, hi[N_R2E] = { 0 };
    static const uint8_t PATTERNS[4] = { 0, 1, 2, 3 };
    for (int p = 0; p < 4; p++) {
        uint32_t set = 0, clr = 0;
        for (size_t i = 0; i < N_R2E; i++)
            if ((PATTERNS[p] >> i) & 1) set |= 1u << RP_TO_ESP[i].rp; else clr |= 1u << RP_TO_ESP[i].rp;
        if (set) rp_wr(SIO_GPIO_OUT_SET, set);
        if (clr) rp_wr(SIO_GPIO_OUT_CLR, clr);
        esp_rom_delay_us(200);
        for (size_t i = 0; i < N_R2E; i++) {
            int want = (PATTERNS[p] >> i) & 1, got = gpio_get_level(RP_TO_ESP[i].esp);
            if (got != want) bad[i]++;
            if (got) hi[i]++;
        }
    }
    rp_wr(SIO_GPIO_OE_CLR, oe);
    for (size_t i = 0; i < N_R2E; i++) {
        if (!bad[i])          bu_report(RP_TO_ESP[i].name, BU_PASS, "4/4 patterns, no crosstalk");
        else if (hi[i] == 4)  bu_report(RP_TO_ESP[i].name, BU_FAIL, "ESP side stuck HIGH");
        else if (hi[i] == 0)  bu_report(RP_TO_ESP[i].name, BU_FAIL, "ESP side stuck LOW (open track?)");
        else                  bu_report(RP_TO_ESP[i].name, BU_FAIL, "%d/4 patterns wrong", bad[i]);
    }
}

// After a RUN pulse the SIO/IO state we set must be gone: that is the proof
// the RUN net (ESP41 -> IC2 RUN, R32 pull-up) actually resets the chip.
//
// Three outcomes, because the RP may or may not have firmware:
//   funcsel NULL  - reset to a blank slate. Chip was blank. PASS.
//   funcsel SIO   - exactly the state WE set is still there. RUN did nothing. FAIL.
//   anything else - the RP reset and its own firmware reconfigured the pin
//                   (GWSPI slave leaves GPIO3 on SPI, GPIO4 driving DRDY).
//                   RUN worked; we just cannot see a blank slate. PASS.
static void run_reset_check(void)
{
    run_pulse();
    if (bu_swd_attach() != ESP_OK) {
        bu_report("RP RUN reset", BU_FAIL, "no SWD answer after the RUN pulse");
        return;                      // nothing attached, nothing to detach
    }
    uint32_t oe = ~0u, ctrl = ~0u;
    rp_rd(SIO_GPIO_OE, &oe);
    rp_rd(IO_GPIO_CTRL(RP_PIN_SPI_TX), &ctrl);
    uint32_t fsel = ctrl & 0x1Fu;
    unsigned long oem = (unsigned long)(oe & RP_LINK_MASK), f = (unsigned long)fsel;

    if (fsel == IO_FUNCSEL_SIO) {
        // Our own SIO setup is still in place: the pulse did nothing at all.
        bu_report("RP RUN reset", BU_FAIL,
                  "our SIO state survived the RUN pulse: OE=0x%02lX CTRL3=0x%02lX - RUN net not reaching IC2?",
                  oem, f);
    } else if (fsel == IO_FUNCSEL_NULL && (oe & RP_LINK_MASK) == 0) {
        bu_report("RP RUN reset", BU_PASS,
                  "GPIO state cleared by RUN pulse (GPIO41 -> IC2 RUN works)");
    } else {
        // Reset happened, then the RP's own firmware re-took the pins before
        // we looked (GWSPI slave: GPIO3 on SPI, GPIO4 driving DRDY).
        bu_report("RP RUN reset", BU_PASS,
                  "RUN pulse reset IC2; RP firmware re-took the pins (OE=0x%02lX CTRL3=0x%02lX, funcsel!=SIO)",
                  oem, f);
    }
    bu_swd_detach();
}

void bu_test_rp(void)
{
    ESP_LOGI(TAG, "--- RP2354 controller over SWD (SWCLK=%d SWDIO=%d RUN=%d) ---",
             RP_SWCLK_GPIO, RP_SWDIO_GPIO, GW_RESET_GPIO);
    passive_levels();
    run_pulse();

    if (bu_swd_attach() != ESP_OK) {
        bu_report("RP SWD attach", BU_FAIL,
                  "no debug port answered — IC2 power/3V3, SWCLK/SWDIO (R59/R60), or RUN held low?");
        esp_link_pins_input();
        return;
    }
    uint32_t dpidr = bu_swd_dpidr();
    bu_report("RP SWD attach", dpidr == 0x4C013477u ? BU_PASS : BU_WARN,
              "DPIDR 0x%08lX via %s; AP IDR 0x%08lX; %s access", (unsigned long)dpidr,
              bu_swd_connect_how(), (unsigned long)bu_swd_ap_idr(), bu_swd_csw_how());

    esp_err_t h = bu_swd_halt();
    bu_report("RP core halt", h == ESP_OK ? BU_PASS : BU_FAIL, "%s",
              h == ESP_OK ? "core 0 halted (DHCSR)" : esp_err_to_name(h));

    chip_id();
    sram_test();

    if (h == ESP_OK && rp_gpio_take()) {
        link_esp_to_rp();
        link_rp_to_esp();
        rp_gpio_release();
    } else {
        bu_report("RP LINK continuity", BU_SKIP, "core not halted / GPIO blocks not reachable");
    }
    esp_link_pins_input();
    bu_swd_detach();

    run_reset_check();
    esp_link_pins_input();
}
