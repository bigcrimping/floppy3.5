#include "storage.h"
#include "sd_index.h"
#include "app_config.h"
#include "fat12_diskio.h"
#include "ff.h"                    // BYTE, needed by diskio_sdmmc.h
#include "diskio_sdmmc.h"
#include "web_util.h"
#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include "sdmmc_cmd.h"
#include "ff.h"

static const char *TAG = "sd";

static sdmmc_card_t     *s_card;
static bool              s_mounted;
static SemaphoreHandle_t s_lock;

// Off the task stacks; guarded by s_lock.
static uint8_t s_copy_buf[8192];

// Recursive: sd_reindex() holds it across sd_index_rebuild(), which lists
// folders through sd_list_path().
#define LOCK()   xSemaphoreTakeRecursive(s_lock, portMAX_DELAY)
#define UNLOCK() xSemaphoreGiveRecursive(s_lock)

bool sd_mount(void)
{
    if (s_mounted) return true;
    // Kept across remounts: other tasks may be blocked on it.
    if (!s_lock) s_lock = xSemaphoreCreateRecursiveMutex();

    spi_bus_config_t bus = {
        .mosi_io_num     = SD_PIN_MOSI,
        .miso_io_num     = SD_PIN_MISO,
        .sclk_io_num     = SD_PIN_CLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4096,
    };
    esp_err_t e = spi_bus_initialize(SD_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {   // already initialized by the LCD
        ESP_LOGW(TAG, "SPI bus init failed: %s", esp_err_to_name(e));
        return false;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SD_SPI_HOST;
    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = SD_PIN_CS;
    slot.host_id = SD_SPI_HOST;

    esp_vfs_fat_mount_config_t mcfg = {
        .format_if_mount_failed = false,   // never reformat the user's card
        .max_files              = 5,
        .allocation_unit_size   = 16 * 1024,
    };

    e = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot, &mcfg, &s_card);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "no SD card mounted (%s) — SD features disabled",
                 esp_err_to_name(e));
        return false;
    }
    s_mounted = true;
    ESP_LOGI(TAG, "SD mounted at %s (%lluMB, FatFs drive %u)", SD_MOUNT_POINT,
             ((uint64_t)s_card->csd.capacity * s_card->csd.sector_size) >> 20,
             (unsigned)ff_diskio_get_pdrv_card(s_card));
    // Drive 0 belongs to the floppy image; catches a boot-order regression.
    if (ff_diskio_get_pdrv_card(s_card) == 0)
        ESP_LOGE(TAG, "SD got FatFs drive 0 — fat12_reserve_drive() must run first");
    return true;
}

bool sd_present(void) { return s_mounted; }

// Under the lock, so the card can't be unmounted mid-call. Never waits: the
// LCD polls this, and a save holds the lock for seconds, so a busy lock
// returns the last reading instead.
bool sd_usage(uint64_t *total_bytes, uint64_t *free_bytes)
{
    static uint64_t c_total, c_free;
    static bool     c_valid;
    if (!s_mounted || !s_lock) { c_valid = false; return false; }
    if (xSemaphoreTakeRecursive(s_lock, 0) == pdTRUE) {
        uint64_t t = 0, f = 0;
        bool ok = s_mounted && esp_vfs_fat_info(SD_MOUNT_POINT, &t, &f) == ESP_OK;
        if (ok) { c_total = t; c_free = f; }
        c_valid = ok;
        UNLOCK();
    }
    if (!c_valid) return false;
    if (total_bytes) *total_bytes = c_total;
    if (free_bytes)  *free_bytes  = c_free;
    return true;
}

bool sd_path_is_safe(const char *rel)
{
    if (!rel) return true;                 // NULL == the root
    size_t n = strlen(rel);
    if (n == 0) return true;
    if (n > 128) return false;             // fits every path[] buffer
    if (rel[0] == '/' || rel[0] == '\\') return false;
    if (strstr(rel, "..")) return false;
    if (strchr(rel, '\\') || strchr(rel, ':')) return false;
    if (rel[n - 1] == '/') return false;
    for (size_t i = 1; i < n; i++)
        if (rel[i] == '/' && rel[i - 1] == '/') return false;
    return true;
}

// "/sdcard" or "/sdcard/REL". Caller has checked the path.
static void sd_abspath(char *dst, size_t dstsz, const char *rel)
{
    if (rel && rel[0])
        snprintf(dst, dstsz, "%s/%s", SD_MOUNT_POINT, rel);
    else
        snprintf(dst, dstsz, "%s", SD_MOUNT_POINT);
}

int sd_list_path(const char *rel, uint32_t *cursor, sd_entry_t *out, int max)
{
    if (!s_mounted) return -1;
    if (!sd_path_is_safe(rel)) return -1;
    char base[300];
    sd_abspath(base, sizeof(base), rel);

    LOCK();
    DIR *d = opendir(base);
    if (!d) { UNLOCK(); return -1; }

    // The cursor counts directory entries read, not entries returned: one
    // that can't be stat'ed is skipped either way.
    uint32_t skip = cursor ? *cursor : 0, seen = 0;
    int n = 0;
    struct dirent *ent;
    while (n < max && (ent = readdir(d)) != NULL) {
        if (seen++ < skip) continue;
        char path[300 + 1 + 256];
        snprintf(path, sizeof(path), "%s/%s", base, ent->d_name);
        struct stat st;
        if (stat(path, &st) != 0) continue;
        out[n].is_dir = S_ISDIR(st.st_mode);
        strlcpy(out[n].name, ent->d_name, sizeof(out[n].name));
        out[n].size = out[n].is_dir ? 0u : (uint32_t)st.st_size;
        n++;
    }
    closedir(d);
    UNLOCK();
    if (cursor) *cursor = seen;
    return n;
}

// Copy one image file into `dstdir` (absolute). Caller holds s_lock.
static long copy_one_locked(const char *name, const char *dstdir)
{
    char src[300], dst[300];
    snprintf(src, sizeof(src), "%s%s", FAT12_ROOT, name);
    snprintf(dst, sizeof(dst), "%s/%s", dstdir, name);

    FIL fsrc;
    if (f_open(&fsrc, src, FA_READ) != FR_OK) return -3;
    FILE *fd = fopen(dst, "wb");
    if (!fd) { f_close(&fsrc); return -4; }

    long total = 0;
    for (;;) {
        UINT br = 0;
        if (f_read(&fsrc, s_copy_buf, sizeof(s_copy_buf), &br) != FR_OK) { total = -5; break; }
        if (br == 0) break;
        if (fwrite(s_copy_buf, 1, br, fd) != br) { total = -6; break; }
        total += br;
    }
    // fsync so a power cut can't leave the card's directory corrupt.
    if (total >= 0) {
        fflush(fd);
        fsync(fileno(fd));
    }
    fclose(fd);
    f_close(&fsrc);
    if (total < 0) remove(dst);

    if (total < 0) ESP_LOGW(TAG, "copy %s failed (%ld)", name, total);
    else           ESP_LOGI(TAG, "copied %s -> %s (%ld bytes)", name, dstdir, total);
    return total;
}

long sd_copy_from_image(const char *name)
{
    if (!s_mounted)               return -1;
    if (!name || name[0] == '\0') return -2;

    LOCK();
    long total = copy_one_locked(name, SD_MOUNT_POINT);
    // Index it, so a later disk save skips this file. If it replaced a file
    // of the same name, the old entry stays until a reindex.
    uint8_t sha[SD_SHA_LEN];
    if (total >= 0 && sd_index_is_loaded() &&
        sd_sha_image_file_cached(name, (uint32_t)total, sha) &&
        !sd_index_has_file(sha, NULL, 0))
        sd_index_add_file(sha, (uint32_t)total, name);
    UNLOCK();
    return total;
}

// ---- whole-disk archiving into /sdcard/DISKnnnn/ (storage.h) ----

// A FAT12 root directory holds at most 224 entries.
#define SD_MAX_FILES 224
typedef struct {
    char     name[13];   // 8.3 + NUL
    uint32_t size;
} file_ent_t;

// Static (too big for task stacks); guarded by s_lock.
static file_ent_t s_img_files[SD_MAX_FILES];
static int        s_img_n;
static file_ent_t s_dir_files[SD_MAX_FILES];

// The image's regular files with safe names. -1 if not mounted.
static int scan_image_locked(void)
{
    FF_DIR dir; FILINFO fno;
    if (f_opendir(&dir, FAT12_ROOT) != FR_OK) return -1;
    int n = 0;
    while (n < SD_MAX_FILES && f_readdir(&dir, &fno) == FR_OK && fno.fname[0]) {
        if (fno.fattrib & (AM_DIR | AM_SYS | AM_HID)) continue;
        if (!web_name_is_safe(fno.fname)) continue;
        strlcpy(s_img_files[n].name, fno.fname, sizeof(s_img_files[n].name));
        s_img_files[n].size = (uint32_t)fno.fsize;
        n++;
    }
    f_closedir(&dir);
    // Name order, matching sd_index_rebuild(), so both compute the same disk
    // identity.
    for (int i = 1; i < n; i++) {
        file_ent_t t = s_img_files[i];
        int j = i;
        for (; j > 0 && strcmp(s_img_files[j - 1].name, t.name) > 0; j--)
            s_img_files[j] = s_img_files[j - 1];
        s_img_files[j] = t;
    }
    return n;
}

// An SD folder's regular files. Returns SD_MAX_FILES+1 for a folder too big
// to match any floppy; -1 if it can't be opened.
static int scan_sd_dir_locked(const char *path)
{
    DIR *d = opendir(path);
    if (!d) return -1;
    int n = 0;
    struct dirent *ent;
    while ((ent = readdir(d)) != NULL) {
        char full[300];
        snprintf(full, sizeof(full), "%s/%s", path, ent->d_name);
        struct stat st;
        if (stat(full, &st) != 0 || S_ISDIR(st.st_mode)) continue;
        if (n >= SD_MAX_FILES) { n++; break; }
        strlcpy(s_dir_files[n].name, ent->d_name, sizeof(s_dir_files[n].name));
        s_dir_files[n].size = (uint32_t)st.st_size;
        n++;
    }
    closedir(d);
    return n;
}

// Fallback duplicate check when hashing is unavailable: same file count and
// every image file has a same-name (case-blind), same-size partner.
static bool same_disk(int dir_n)
{
    if (dir_n != s_img_n) return false;
    for (int i = 0; i < s_img_n; i++) {
        bool found = false;
        for (int j = 0; j < dir_n; j++) {
            if (s_img_files[i].size == s_dir_files[j].size &&
                strcasecmp(s_img_files[i].name, s_dir_files[j].name) == 0) {
                found = true;
                break;
            }
        }
        if (!found) return false;
    }
    return true;
}

// "DISK0007" -> 7; -1 if the name isn't one of ours.
static int disk_dir_index(const char *name)
{
    size_t pfx = strlen(SD_DISK_DIR_PREFIX);
    if (strncasecmp(name, SD_DISK_DIR_PREFIX, pfx) != 0) return -1;
    const char *p = name + pfx;
    if (strlen(p) != 4) return -1;
    int v = 0;
    for (int i = 0; i < 4; i++) {
        if (p[i] < '0' || p[i] > '9') return -1;
        v = v * 10 + (p[i] - '0');
    }
    return v;
}

// Per-file hashes of the image being saved, and which are new to the card.
static uint8_t s_img_sha[SD_MAX_FILES][SD_SHA_LEN];
static bool    s_img_new[SD_MAX_FILES];

sd_save_status_t sd_save_disk(bool force, sd_save_result_t *out)
{
    sd_save_result_t r = { .dir = "", .copied = 0, .failed = 0, .skipped = 0,
                           .first_dupe = "" };
    if (out) *out = r;
    if (!s_mounted) return SD_SAVE_NO_CARD;

    LOCK();
    s_img_n = scan_image_locked();
    if (s_img_n <= 0) { UNLOCK(); return SD_SAVE_NO_IMAGE; }

    // Hardware SHA over the PSRAM image: ~100 ms per disk.
    bool hashed = sd_index_is_loaded();
    if (hashed) {
        for (int i = 0; i < s_img_n && hashed; i++)
            hashed = sd_sha_image_file_cached(s_img_files[i].name,
                                              s_img_files[i].size, s_img_sha[i]);
        if (!hashed)
            ESP_LOGW(TAG, "hashing failed — falling back to name/size dedup");
    }

    if (hashed && !force) {
        uint8_t dsha[SD_SHA_LEN];
        char where[40];
        if (sd_sha_disk((const uint8_t (*)[SD_SHA_LEN])s_img_sha, s_img_n, dsha) &&
            sd_index_has_disk(dsha, where, sizeof(where))) {
            strlcpy(r.dir, where, sizeof(r.dir));
            UNLOCK();
            if (out) *out = r;
            ESP_LOGW(TAG, "save: this disk is already archived as %s", r.dir);
            return SD_SAVE_DUPLICATE;
        }
        // Skip files already on the card, whatever they are called there.
        for (int i = 0; i < s_img_n; i++) {
            char w[40];
            if (sd_index_has_file(s_img_sha[i], w, sizeof(w))) {
                s_img_new[i] = false;
                r.skipped++;
                if (!r.first_dupe[0]) strlcpy(r.first_dupe, w, sizeof(r.first_dupe));
            } else {
                s_img_new[i] = true;
            }
        }
        if (!r.skipped) { /* all new */ }
        else if (r.skipped == s_img_n) {
            // Every file already saved (from other disks): nothing to copy.
            strlcpy(r.dir, r.first_dupe, sizeof(r.dir));
            UNLOCK();
            if (out) *out = r;
            ESP_LOGW(TAG, "save: all %d files already on the card", s_img_n);
            return SD_SAVE_DUPLICATE;
        }
    } else {
        for (int i = 0; i < s_img_n; i++) s_img_new[i] = true;
    }

    // Find the highest DISKnnnn (the new folder is max+1) and, without
    // hashes, run the name/size duplicate check.
    int highest = 0;
    DIR *root = opendir(SD_MOUNT_POINT);
    if (root) {
        struct dirent *ent;
        while ((ent = readdir(root)) != NULL) {
            int idx = disk_dir_index(ent->d_name);
            if (idx < 0) continue;
            char path[300];
            snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, ent->d_name);
            struct stat st;
            if (stat(path, &st) != 0 || !S_ISDIR(st.st_mode)) continue;
            if (idx > highest) highest = idx;
            if (!force && !hashed) {
                int dn = scan_sd_dir_locked(path);
                if (dn >= 0 && same_disk(dn)) {
                    strlcpy(r.dir, ent->d_name, sizeof(r.dir));
                    closedir(root);
                    UNLOCK();
                    if (out) *out = r;
                    ESP_LOGW(TAG, "save: this disk is already archived as %s",
                             r.dir);
                    return SD_SAVE_DUPLICATE;
                }
            }
        }
        closedir(root);
    }
    if (highest >= 9999) { UNLOCK(); return SD_SAVE_CARD_FULL; }

    // Refuse up front rather than leave a half-written folder. Pessimistic:
    // each file rounded up to a 32 KB cluster, plus directory slack.
    {
        uint64_t total_b = 0, free_b = 0;
        if (esp_vfs_fat_info(SD_MOUNT_POINT, &total_b, &free_b) == ESP_OK) {
            const uint64_t CLUSTER_GUESS = 32u * 1024u;
            uint64_t need = 64u * 1024u;
            for (int i = 0; i < s_img_n; i++)
                if (s_img_new[i])
                    need += ((uint64_t)s_img_files[i].size + CLUSTER_GUESS - 1) /
                            CLUSTER_GUESS * CLUSTER_GUESS;
            if (need > free_b) {
                UNLOCK();
                if (out) *out = r;
                ESP_LOGE(TAG, "save: needs ~%llu KB, only %llu KB free",
                         (unsigned long long)(need / 1024),
                         (unsigned long long)(free_b / 1024));
                return SD_SAVE_NO_SPACE;
            }
        }
    }

    char dirpath[300];
    snprintf(r.dir, sizeof(r.dir), "%s%04d", SD_DISK_DIR_PREFIX, highest + 1);
    snprintf(dirpath, sizeof(dirpath), "%s/%s", SD_MOUNT_POINT, r.dir);
    if (mkdir(dirpath, 0777) != 0) {
        UNLOCK();
        if (out) *out = r;
        ESP_LOGE(TAG, "save: mkdir %s failed", dirpath);
        return SD_SAVE_MKDIR_FAIL;
    }

    for (int i = 0; i < s_img_n; i++) {
        if (!s_img_new[i]) continue;
        if (copy_one_locked(s_img_files[i].name, dirpath) < 0) {
            r.failed++;
            continue;
        }
        r.copied++;
        // Index only after the fsync: over-reporting would skip a photo that
        // never reached the card.
        if (hashed) {
            char rel[48];
            snprintf(rel, sizeof(rel), "%s/%s", r.dir, s_img_files[i].name);
            sd_index_add_file(s_img_sha[i], s_img_files[i].size, rel);
        }
    }
    // Disk identity only for a complete save, so a partial one can be re-run.
    if (hashed && !r.failed) {
        uint8_t dsha[SD_SHA_LEN];
        if (sd_sha_disk((const uint8_t (*)[SD_SHA_LEN])s_img_sha, s_img_n, dsha))
            sd_index_add_disk(dsha, r.dir, s_img_n);
    }
    UNLOCK();

    if (out) *out = r;
    ESP_LOGI(TAG, "save: %s — %d copied, %d failed", r.dir, r.copied, r.failed);
    return SD_SAVE_OK;
}

int sd_reindex(void)
{
    if (!s_mounted) return -1;
    LOCK();
    int n = sd_index_rebuild();
    UNLOCK();
    return n;
}

// ---- card detect / hot-plug ----

#ifdef SD_DET_GPIO
// Caller holds s_lock.
static void sd_unmount_locked(void)
{
    if (!s_mounted) return;
    esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_card);
    s_card = NULL;
    s_mounted = false;
    ESP_LOGW(TAG, "SD card removed — unmounted");
}
#endif

void sd_detect_init(void)
{
#ifdef SD_DET_GPIO
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << SD_DET_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io);
#endif
}

void sd_detect_poll(void)
{
#ifdef SD_DET_GPIO
    // Debounce: ~5 agreeing 30 ms polls (the switch wipes as the card
    // slides). The ~150 ms also covers card power-up.
    static int last = -1, stable_n;
    int now = sd_card_detected() ? 1 : 0;
    if (now != last) { last = now; stable_n = 0; return; }
    if (stable_n < 100) stable_n++;
    if (stable_n != 5) return;          // once per settled transition

    if (now && !s_mounted) {
        if (sd_mount()) {
            LOCK();
            sd_index_load();            // the index lives on the card
            UNLOCK();
            ESP_LOGI(TAG, "SD card inserted — mounted");
        } else {
            // Not retried until the switch changes again.
            ESP_LOGW(TAG, "SD card inserted but mount failed");
        }
    } else if (!now && s_mounted) {
        // Don't unmount under a copy, and don't block the UI task: if a save
        // holds the lock, retry next poll (that save's writes will fail).
        if (xSemaphoreTakeRecursive(s_lock, pdMS_TO_TICKS(10)) == pdTRUE) {
            sd_unmount_locked();
            sd_index_free();
            xSemaphoreGiveRecursive(s_lock);
        } else {
            stable_n = 4;               // retry next poll
        }
    }
#endif
}

bool sd_card_detected(void)
{
#ifdef SD_DET_GPIO
    int lvl = gpio_get_level(SD_DET_GPIO);
    return SD_DET_ACTIVE_LOW ? (lvl == 0) : (lvl != 0);
#else
    return sd_present();
#endif
}
