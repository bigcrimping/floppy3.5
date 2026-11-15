// Sequence: RUN pulse -> SWD attach -> halt core 0 -> load the stub into SRAM
// -> start it -> erase / write / verify through the mailbox (gwrp_mailbox.h)
// -> EXIT -> release SWD -> RUN pulse boots the new image.
#include "rp_flash.h"
#include "app_config.h"
#include "rp_swd.h"
#include <string.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "../../rp2354/tools/flash_stub/gwrp_mailbox.h"
#include "rp_flash_stub_bin.h"

static const char *TAG = "rpflash";

#define XIP_BASE 0x10000000u
#define FLASH_SIZE (2u * 1024 * 1024)
#define ERASE_ALIGN 4096u
#define WRITE_CHUNK GWRP_DATA_MAX

// The mailbox is 64 KB above this, clear of the image; the stack is the top
// of SRAM (rp_flash_stub_sp, from the stub's vector table).
#define STUB_LOAD_ADDR 0x20000000u

#define MB_OFF(field) (GWRP_MAILBOX_ADDR + offsetof(struct gwrp_mailbox, field))

static char *s_err;
static size_t s_errsz;
static uint32_t s_seq;

static void fail(const char *msg)
{
    if (s_err && s_errsz)
        snprintf(s_err, s_errsz, "%s", msg);
    ESP_LOGE(TAG, "%s", msg);
}

// ---- RUN line ----

static void run_pulse(void)
{
    gpio_set_direction(GW_RESET_GPIO, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(GW_RESET_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(GW_RESET_LOW_MS));
    gpio_set_level(GW_RESET_GPIO, 1);
}

// ---- stub load ----

static esp_err_t load_stub(void)
{
    if (rp_flash_stub_bin_len == 0) {
        fail("flash stub not built (firmware/rp2354/tools/flash_stub)");
        return ESP_ERR_INVALID_STATE;
    }

    run_pulse();
    vTaskDelay(pdMS_TO_TICKS(50));

    esp_err_t e = rp_swd_attach();
    if (e != ESP_OK) {
        fail("SWD attach failed — check SWCLK/SWDIO wiring and that no debug "
             "probe is driving J12");
        return e;
    }
    if ((e = rp_swd_halt()) != ESP_OK) {
        fail("core would not halt");
        return e;
    }

    // Padded to a word: the MEM-AP is word-only.
    size_t len = (rp_flash_stub_bin_len + 3u) & ~3u;
    static uint8_t padded[8192];
    if (len > sizeof(padded)) {
        fail("stub image too large");
        return ESP_ERR_INVALID_SIZE;
    }
    memset(padded, 0, len);
    memcpy(padded, rp_flash_stub_bin, rp_flash_stub_bin_len);

    if ((e = rp_swd_write_mem(STUB_LOAD_ADDR, padded, len)) != ESP_OK) {
        fail("stub upload failed (MEM-AP write)");
        return e;
    }

    // Log a readback: a mis-addressed AP can "succeed" without writing.
    {
        uint32_t w0 = 0, w1 = 0;
        rp_swd_read32(STUB_LOAD_ADDR, &w0);
        rp_swd_read32(STUB_LOAD_ADDR + 4, &w1);
        ESP_LOGI(TAG, "stub readback: [0]=0x%08lX (want 0x%08lX) [1]=0x%08lX",
                 (unsigned long)w0, (unsigned long)rp_flash_stub_sp,
                 (unsigned long)w1);
    }

    // Clear any stale magic from a previous run.
    for (uint32_t off = 0; off < 32; off += 4)
        rp_swd_write32(GWRP_MAILBOX_ADDR + off, 0);

    // The resident firmware booted during the 50 ms before the halt, and its
    // NVIC enables survive it. A pending IRQ (e.g. USB with a host attached)
    // would fire into overwritten RAM and re-halt the stub. Disable and
    // un-pend every external IRQ (ICER/ICPR 0..3 cover all 52) and SysTick.
    for (uint32_t i = 0; i < 4; i++) {
        rp_swd_write32(0xE000E180u + 4 * i, 0xFFFFFFFFu);   // NVIC_ICERn
        rp_swd_write32(0xE000E280u + 4 * i, 0xFFFFFFFFu);   // NVIC_ICPRn
    }
    rp_swd_write32(0xE000E010u, 0);                         // SYST_CSR

    // Start at the ELF entry, not the load address: the image begins with a
    // vector table.
    if ((e = rp_swd_run_stub(rp_flash_stub_entry, rp_flash_stub_sp)) != ESP_OK) {
        fail("could not start the stub");
        return e;
    }

    // Log whether the core resumed or faulted straight back into halt.
    {
        uint32_t dhcsr = 0;
        rp_swd_read32(0xE000EDF0u, &dhcsr);
        ESP_LOGI(TAG, "after resume: DHCSR=0x%08lX (S_HALT=%lu) entry=0x%08lX sp=0x%08lX",
                 (unsigned long)dhcsr, (unsigned long)((dhcsr >> 17) & 1),
                 (unsigned long)rp_flash_stub_entry,
                 (unsigned long)rp_flash_stub_sp);
    }

    for (int i = 0; i < 200; i++) {
        uint32_t magic = 0;
        if (rp_swd_read32(MB_OFF(magic), &magic) == ESP_OK &&
            magic == GWRP_MAGIC) {
            s_seq = 0;
            rp_swd_read32(MB_OFF(ack), &s_seq);
            ESP_LOGI(TAG, "stub running (IDCODE=0x%08lX)",
                     (unsigned long)rp_swd_idcode());
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    {
        uint32_t magic = 0, dhcsr = 0;
        rp_swd_read32(MB_OFF(magic), &magic);
        rp_swd_read32(0xE000EDF0u, &dhcsr);
        ESP_LOGE(TAG, "timeout: mailbox magic=0x%08lX (want 0x%08lX), DHCSR=0x%08lX",
                 (unsigned long)magic, (unsigned long)GWRP_MAGIC,
                 (unsigned long)dhcsr);
    }
    fail("stub did not start (no GWRP magic in SRAM)");
    return ESP_ERR_TIMEOUT;
}

// ---- mailbox commands ----

static esp_err_t mb_run(uint32_t cmd, uint32_t addr, uint32_t len,
                        uint32_t crc, int timeout_ms, const char *what)
{
    esp_err_t e;
    if ((e = rp_swd_write32(MB_OFF(cmd), cmd)) != ESP_OK ||
        (e = rp_swd_write32(MB_OFF(addr), addr)) != ESP_OK ||
        (e = rp_swd_write32(MB_OFF(len), len)) != ESP_OK ||
        (e = rp_swd_write32(MB_OFF(crc), crc)) != ESP_OK) {
        fail("mailbox write failed (SWD)");
        return e;
    }
    // seq last: the stub polls it.
    if ((e = rp_swd_write32(MB_OFF(seq), ++s_seq)) != ESP_OK) {
        fail("mailbox submit failed (SWD)");
        return e;
    }

    for (int waited = 0; waited <= timeout_ms; waited += 2) {
        uint32_t ack = 0;
        if ((e = rp_swd_read32(MB_OFF(ack), &ack)) != ESP_OK) {
            fail("mailbox poll failed (SWD)");
            return e;
        }
        if (ack == s_seq) {
            uint32_t st = GWRP_ST_BADCMD;
            rp_swd_read32(MB_OFF(status), &st);
            if (st == GWRP_ST_OK)
                return ESP_OK;
            char msg[80];
            snprintf(msg, sizeof(msg), "%s: stub status %lu @0x%08lX", what,
                     (unsigned long)st, (unsigned long)addr);
            fail(msg);
            return ESP_FAIL;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    {
        char msg[64];
        snprintf(msg, sizeof(msg), "%s: stub timeout", what);
        fail(msg);
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t stub_erase(uint32_t addr, uint32_t len)
{
    // ~45 ms/sector: ~6 s for 512 KB.
    return mb_run(GWRP_CMD_ERASE, addr, len, 0, 30000, "erase");
}

static esp_err_t stub_write(uint32_t addr, const uint8_t *data, uint32_t len)
{
    uint32_t padded = (len + 3u) & ~3u;
    static uint8_t buf[WRITE_CHUNK];
    memset(buf, 0xFF, sizeof(buf));
    memcpy(buf, data, len);
    esp_err_t e = rp_swd_write_mem(MB_OFF(data), buf, padded);
    if (e != ESP_OK) {
        fail("payload upload failed (SWD)");
        return e;
    }
    return mb_run(GWRP_CMD_WRITE, addr, len, gwrp_crc16(data, len), 5000,
                  "write");
}

static esp_err_t stub_verify(uint32_t addr, const uint8_t *expect,
                             uint32_t len)
{
    esp_err_t e = mb_run(GWRP_CMD_READ, addr, len, 0, 5000, "read");
    if (e != ESP_OK)
        return e;

    static uint8_t got[WRITE_CHUNK];
    uint32_t padded = (len + 3u) & ~3u;
    if ((e = rp_swd_read_mem(MB_OFF(data), got, padded)) != ESP_OK) {
        fail("readback failed (SWD)");
        return e;
    }
    uint32_t stub_crc = 0;
    rp_swd_read32(MB_OFF(crc), &stub_crc);

    // The stub's CRC catches a bad SWD readback; the compare catches a bad
    // program.
    if ((uint16_t)stub_crc != gwrp_crc16(got, len) ||
        memcmp(got, expect, len) != 0) {
        char msg[64];
        snprintf(msg, sizeof(msg), "verify mismatch @0x%08lX",
                 (unsigned long)addr);
        fail(msg);
        return ESP_FAIL;
    }
    return ESP_OK;
}

// ---- public entry point ----

static volatile int s_pct = -1;
int rp_flash_progress_pct(void) { return s_pct; }

esp_err_t rp_flash_program(const uint8_t *img, size_t len, uint32_t addr,
                           char *err, size_t errsz)
{
    s_err = err;
    s_errsz = errsz;
    if (err && errsz)
        err[0] = '\0';

    if (!len || len > RP_FLASH_IMG_MAX || addr < XIP_BASE ||
        (addr & (ERASE_ALIGN - 1)) || addr + len > XIP_BASE + FLASH_SIZE) {
        fail("bad image length/address (need 4K-aligned 0x10xxxxxx)");
        return ESP_ERR_INVALID_ARG;
    }

    s_pct = 0;
    esp_err_t e = load_stub();

    if (e == ESP_OK) {
        uint32_t espan = (uint32_t)((len + ERASE_ALIGN - 1) & ~(ERASE_ALIGN - 1));
        e = stub_erase(addr, espan);
    }
    for (size_t off = 0; e == ESP_OK && off < len; off += WRITE_CHUNK) {
        size_t n = (len - off > WRITE_CHUNK) ? WRITE_CHUNK : len - off;
        e = stub_write(addr + (uint32_t)off, img + off, (uint32_t)n);
        s_pct = (int)((off + n) * 70 / len);
    }
    for (size_t off = 0; e == ESP_OK && off < len; off += WRITE_CHUNK) {
        size_t n = (len - off > WRITE_CHUNK) ? WRITE_CHUNK : len - off;
        e = stub_verify(addr + (uint32_t)off, img + off, (uint32_t)n);
        s_pct = 70 + (int)((off + n) * 30 / len);
    }
    if (e == ESP_OK)
        (void)mb_run(GWRP_CMD_EXIT, 0, 0, 0, 1000, "exit");

    // Even on failure, so a probe on J12 can take over.
    rp_swd_detach();
    run_pulse();
    s_pct = -1;

    if (e == ESP_OK)
        ESP_LOGI(TAG, "flashed %u bytes @0x%08lX OK", (unsigned)len,
                 (unsigned long)addr);
    return e;
}
