#include "fat_scan.h"
#include "app_config.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "fat";

static inline uint16_t rd16(const uint8_t *b, int o) { return b[o] | (b[o + 1] << 8); }
static inline uint32_t rd32(const uint8_t *b, int o)
{
    return b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | ((uint32_t)b[o + 3] << 24);
}

// Physical geometry (disk_lba), not the BPB's CHS.
static inline uint32_t lba_track(uint32_t lba) { return lba / DISK_SECTORS; }

bool fat_used_tracks(bool *used)
{
    uint8_t boot[SECTOR_SIZE];
    if (!disk_image_read(0, boot, 1)) return false;
    if (!disk_sector_valid(0)) { ESP_LOGW(TAG, "boot sector CRC-bad"); return false; }

    const uint16_t bps      = rd16(boot, 11);
    const uint8_t  spc      = boot[13];
    const uint16_t reserved = rd16(boot, 14);
    const uint8_t  nfats    = boot[16];
    const uint16_t root_ent = rd16(boot, 17);
    const uint16_t totsec16 = rd16(boot, 19);
    const uint16_t spf      = rd16(boot, 22);
    const uint32_t totsec32 = rd32(boot, 32);
    const uint32_t totsec   = totsec16 ? totsec16 : totsec32;

    if (bps != SECTOR_SIZE || spc == 0 || nfats < 1 || nfats > 2
        || spf == 0 || totsec == 0) {
        ESP_LOGW(TAG, "not a FAT12 BPB");
        return false;
    }

    const uint32_t fat_start  = reserved;
    const uint32_t root_start = reserved + (uint32_t)nfats * spf;
    const uint32_t root_sec   = ((uint32_t)root_ent * 32 + bps - 1) / bps;
    const uint32_t data_start = root_start + root_sec;
    if (data_start >= totsec) { ESP_LOGW(TAG, "bad BPB geometry"); return false; }
    const uint32_t total_clus = (totsec - data_start) / spc;

    const int ntracks = TOTAL_TRACKS;
    for (int i = 0; i < ntracks; i++) used[i] = false;

    // Boot sector, FATs and root directory.
    for (uint32_t lba = 0; lba < data_start; lba++) used[lba_track(lba)] = true;

    const size_t fat_bytes = (size_t)spf * bps;
    uint8_t *fat = heap_caps_malloc(fat_bytes, MALLOC_CAP_SPIRAM);
    if (!fat) { ESP_LOGE(TAG, "FAT buffer OOM"); return false; }
    if (!disk_image_read(fat_start, fat, spf)) { heap_caps_free(fat); return false; }
    for (uint32_t s = 0; s < spf; s++) {
        if (!disk_sector_valid(fat_start + s)) {
            ESP_LOGW(TAG, "FAT sector %u CRC-bad", (unsigned)(fat_start + s));
            heap_caps_free(fat);
            return false;
        }
    }

    // Any non-zero entry (data or a bad-cluster mark) counts as used.
    for (uint32_t n = 2; n < total_clus + 2; n++) {
        const uint32_t off = n + n / 2;             // n * 3 / 2
        if (off + 1 >= fat_bytes) break;
        const uint16_t v = fat[off] | (fat[off + 1] << 8);
        const uint16_t ent = (n & 1) ? (v >> 4) : (v & 0x0FFF);
        if (ent != 0x000) {
            const uint32_t first = data_start + (n - 2) * spc;
            for (uint32_t s = 0; s < spc; s++) used[lba_track(first + s)] = true;
        }
    }

    heap_caps_free(fat);
    return true;
}
