// microSD (J7, DM3AT holder) on SPI2: CLK 11 / MOSI 3 / MISO 10 / CS 47,
// card-detect 48. The bus is shared with the OLED and initialised once by the
// sequencer. Mount -> write a 4 KB file -> read it back -> delete -> unmount,
// so the card leaves the bench exactly as it arrived.
#include "bu_tests.h"
#include "bu_report.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/gpio.h"
#include "sdmmc_cmd.h"
#include "app_config.h"

static const char *TAG = "t_sd";
#define TEST_FILE SD_MOUNT_POINT "/BRINGUP.TXT"
#define TEST_LEN  4096

static bool detect(void)
{
#ifdef SD_DET_GPIO
    gpio_config_t in = { .pin_bit_mask = 1ULL << SD_DET_GPIO, .mode = GPIO_MODE_INPUT,
                         .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config(&in);
    int lvl = gpio_get_level(SD_DET_GPIO);
    bool present = SD_DET_ACTIVE_LOW ? (lvl == 0) : (lvl != 0);
    bu_report("SD detect (GPIO48)", BU_INFO, "level %d -> %s", lvl, present ? "card seated" : "no card");
    return present;
#else
    return true;
#endif
}

static bool rw_test(void)
{
    static uint8_t wbuf[TEST_LEN], rbuf[TEST_LEN];
    for (int i = 0; i < TEST_LEN; i++) wbuf[i] = (uint8_t)(i * 7 + 13);

    FILE *f = fopen(TEST_FILE, "wb");
    if (!f) { bu_report("SD write/read", BU_FAIL, "fopen for write failed"); return false; }
    size_t w = fwrite(wbuf, 1, TEST_LEN, f);
    fflush(f);
    fsync(fileno(f));
    fclose(f);
    if (w != TEST_LEN) { bu_report("SD write/read", BU_FAIL, "short write %u/%u", (unsigned)w, TEST_LEN); remove(TEST_FILE); return false; }

    f = fopen(TEST_FILE, "rb");
    if (!f) { bu_report("SD write/read", BU_FAIL, "fopen for read failed"); remove(TEST_FILE); return false; }
    memset(rbuf, 0, sizeof(rbuf));
    size_t r = fread(rbuf, 1, TEST_LEN, f);
    fclose(f);
    remove(TEST_FILE);
    if (r != TEST_LEN || memcmp(wbuf, rbuf, TEST_LEN) != 0) {
        bu_report("SD write/read", BU_FAIL, "readback mismatch (%u bytes) — MISO/MOSI/CLK integrity", (unsigned)r);
        return false;
    }
    bu_report("SD write/read", BU_PASS, "%u B written, read back, deleted (%s)", TEST_LEN, TEST_FILE);
    return true;
}

void bu_test_sd(void)
{
    ESP_LOGI(TAG, "--- microSD (SPI2, CS=%d) ---", SD_PIN_CS);
    bool det = detect();

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SD_SPI_HOST;
    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = SD_PIN_CS;
    slot.host_id = SD_SPI_HOST;
    esp_vfs_fat_mount_config_t mcfg = {
        .format_if_mount_failed = false,      // never touch the layout of someone's card
        .max_files              = 2,
        .allocation_unit_size   = 16 * 1024,
    };
    sdmmc_card_t *card = NULL;
    esp_err_t e = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot, &mcfg, &card);
    if (e != ESP_OK) {
        // No card is a normal bench state; detect saying "seated" but no
        // mount is not.
        bu_report("SD mount", det ? BU_FAIL : BU_WARN, "%s%s", esp_err_to_name(e),
                  det ? " — card detected but not talking: CS/CLK/MOSI/MISO or card format"
                      : " — insert a FAT-formatted card to test the SPI lines");
        return;
    }
    uint64_t cap = (uint64_t)card->csd.capacity * card->csd.sector_size;
    bu_report("SD mount", BU_PASS, "%s, %llu MB, %s, %u kHz", card->cid.name, cap >> 20,
              card->ocr & (1u << 30) ? "SDHC/SDXC" : "SDSC", (unsigned)card->max_freq_khz);
    if (!det) bu_report("SD detect vs mount", BU_WARN, "card mounted but detect pin says absent — J7 CD switch / GPIO48");

    rw_test();
    esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, card);
}
