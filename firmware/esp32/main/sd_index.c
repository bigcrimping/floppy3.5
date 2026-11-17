// mbedTLS SHA-256 uses the ESP32-S3's accelerator: ~100 ms per floppy.
#include "sd_index.h"

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <sys/stat.h>
#include <dirent.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "mbedtls/sha256.h"
#include "ff.h"

#include "app_config.h"
#include "fat12_diskio.h"
#include "storage.h"

static const char *TAG = "sdindex";

// ~200 disks of photos; 72 B/entry in PSRAM.
#define IDX_MAX 4000
#define PATH_MAX_LEN 40    // "DISK0007/MVC-00001.JPG" + slack

typedef struct {
    uint8_t sha[SD_SHA_LEN];
    char    path[PATH_MAX_LEN];
    bool    is_disk;        // a disk-identity entry rather than a file
} idx_ent_t;

static idx_ent_t *s_ent;
static int s_n;
static bool s_loaded;

static void idx_path(char *dst, size_t n)
{
    snprintf(dst, n, "%s/%s", SD_MOUNT_POINT, SD_INDEX_FILE);
}

// ---- hashing ----

void sd_sha_hex(const uint8_t sha[SD_SHA_LEN], char *hex65)
{
    static const char *H = "0123456789abcdef";
    for (int i = 0; i < SD_SHA_LEN; i++) {
        hex65[i * 2]     = H[sha[i] >> 4];
        hex65[i * 2 + 1] = H[sha[i] & 15];
    }
    hex65[64] = '\0';
}

static bool hex_to_sha(const char *hex, uint8_t out[SD_SHA_LEN])
{
    for (int i = 0; i < SD_SHA_LEN; i++) {
        int hi = hex[i * 2], lo = hex[i * 2 + 1], v = 0;
        for (int k = 0; k < 2; k++) {
            int c = k ? lo : hi;
            if (c >= '0' && c <= '9') c -= '0';
            else if (c >= 'a' && c <= 'f') c -= 'a' - 10;
            else if (c >= 'A' && c <= 'F') c -= 'A' - 10;
            else return false;
            v = (v << 4) | c;
        }
        out[i] = (uint8_t)v;
    }
    return true;
}

bool sd_sha_image_file(const char *name, uint8_t out[SD_SHA_LEN])
{
    char src[300];
    snprintf(src, sizeof(src), "%s%s", FAT12_ROOT, name);
    FIL f;
    if (f_open(&f, src, FA_READ) != FR_OK) return false;

    mbedtls_sha256_context c;
    mbedtls_sha256_init(&c);
    bool ok = mbedtls_sha256_starts(&c, 0) == 0;   // 0 = SHA-256, not -224
    static uint8_t buf[4096];
    UINT br;
    while (ok && f_read(&f, buf, sizeof(buf), &br) == FR_OK && br > 0)
        ok = mbedtls_sha256_update(&c, buf, br) == 0;
    ok = ok && mbedtls_sha256_finish(&c, out) == 0;
    mbedtls_sha256_free(&c);
    f_close(&f);
    return ok;
}

// One slot per possible root-directory entry, full LFN names: ~65 KB, so
// PSRAM, allocated on first use. Without it every call just hashes.
#define IMG_CACHE_N 224
typedef struct {
    char     name[FF_LFN_BUF + 1];
    uint32_t size;
    uint8_t  sha[SD_SHA_LEN];
} img_sha_t;
static img_sha_t *s_img_cache;
static int s_img_cache_n;

bool sd_sha_image_file_cached(const char *name, uint32_t size, uint8_t out[SD_SHA_LEN])
{
    for (int i = 0; i < s_img_cache_n; i++)
        if (s_img_cache[i].size == size && strcmp(s_img_cache[i].name, name) == 0) {
            memcpy(out, s_img_cache[i].sha, SD_SHA_LEN);
            return true;
        }
    if (!sd_sha_image_file(name, out)) return false;
    if (!s_img_cache)
        s_img_cache = heap_caps_malloc(IMG_CACHE_N * sizeof *s_img_cache, MALLOC_CAP_SPIRAM);
    if (s_img_cache && s_img_cache_n < IMG_CACHE_N &&
        strlen(name) < sizeof s_img_cache[0].name) {
        img_sha_t *e = &s_img_cache[s_img_cache_n++];
        strcpy(e->name, name);
        e->size = size;
        memcpy(e->sha, out, SD_SHA_LEN);
    }
    return true;
}

void sd_sha_image_cache_clear(void) { s_img_cache_n = 0; }

// FAT names are case-insensitive and the caller's spelling comes from a URL.
void sd_sha_image_cache_forget(const char *name)
{
    for (int i = 0; i < s_img_cache_n; )
        if (strcasecmp(s_img_cache[i].name, name) == 0)
            s_img_cache[i] = s_img_cache[--s_img_cache_n];
        else
            i++;
}

// SHA-256 of a file already on the SD card (used by the rebuild path).
static bool sha_sd_file(const char *abspath, uint8_t out[SD_SHA_LEN])
{
    FILE *f = fopen(abspath, "rb");
    if (!f) return false;
    mbedtls_sha256_context c;
    mbedtls_sha256_init(&c);
    bool ok = mbedtls_sha256_starts(&c, 0) == 0;
    static uint8_t buf[4096];
    size_t br;
    while (ok && (br = fread(buf, 1, sizeof(buf), f)) > 0)
        ok = mbedtls_sha256_update(&c, buf, br) == 0;
    ok = ok && mbedtls_sha256_finish(&c, out) == 0;
    mbedtls_sha256_free(&c);
    fclose(f);
    return ok;
}

bool sd_sha_disk(const uint8_t (*file_shas)[SD_SHA_LEN], int n,
                 uint8_t out[SD_SHA_LEN])
{
    mbedtls_sha256_context c;
    mbedtls_sha256_init(&c);
    bool ok = mbedtls_sha256_starts(&c, 0) == 0;
    for (int i = 0; ok && i < n; i++)
        ok = mbedtls_sha256_update(&c, file_shas[i], SD_SHA_LEN) == 0;
    ok = ok && mbedtls_sha256_finish(&c, out) == 0;
    mbedtls_sha256_free(&c);
    return ok;
}

// ---- in-memory table ----

static bool ent_add(const uint8_t sha[SD_SHA_LEN], const char *path, bool is_disk)
{
    if (s_n >= IDX_MAX) {
        ESP_LOGW(TAG, "index full at %d entries — dedup will miss new files", IDX_MAX);
        return false;
    }
    memcpy(s_ent[s_n].sha, sha, SD_SHA_LEN);
    strlcpy(s_ent[s_n].path, path, PATH_MAX_LEN);
    s_ent[s_n].is_disk = is_disk;
    s_n++;
    return true;
}

static bool ent_find(const uint8_t sha[SD_SHA_LEN], bool is_disk,
                     char *where, size_t wheresz)
{
    for (int i = 0; i < s_n; i++) {
        if (s_ent[i].is_disk != is_disk) continue;
        if (memcmp(s_ent[i].sha, sha, SD_SHA_LEN) != 0) continue;
        if (where && wheresz) strlcpy(where, s_ent[i].path, wheresz);
        return true;
    }
    return false;
}

bool sd_index_has_file(const uint8_t sha[SD_SHA_LEN], char *w, size_t n)
{ return s_loaded && ent_find(sha, false, w, n); }

bool sd_index_has_disk(const uint8_t sha[SD_SHA_LEN], char *w, size_t n)
{ return s_loaded && ent_find(sha, true, w, n); }

int sd_index_disk_count(void)
{
    int n = 0;
    for (int i = 0; i < s_n; i++) if (s_ent[i].is_disk) n++;
    return n;
}

int sd_index_count(void)
{
    int n = 0;
    for (int i = 0; i < s_n; i++) if (!s_ent[i].is_disk) n++;
    return n;
}

bool sd_index_is_loaded(void) { return s_loaded; }

// ---- file I/O ----

// Extract "key":"value" from a line this file wrote (fixed shapes, so no
// real JSON parser needed).
static bool json_str(const char *line, const char *key, char *out, size_t outsz)
{
    char pat[16];
    snprintf(pat, sizeof(pat), "\"%s\":\"", key);
    const char *p = strstr(line, pat);
    if (!p) return false;
    p += strlen(pat);
    const char *e = strchr(p, '"');
    if (!e || (size_t)(e - p) >= outsz) return false;
    memcpy(out, p, e - p);
    out[e - p] = '\0';
    return true;
}

static bool append_line(const char *line)
{
    char ip[300];
    idx_path(ip, sizeof(ip));
    FILE *f = fopen(ip, "a");
    if (!f) return false;
    bool ok = fputs(line, f) >= 0 && fputc('\n', f) != EOF;
    fflush(f);
    fsync(fileno(f));
    fclose(f);
    return ok;
}

bool sd_index_add_file(const uint8_t sha[SD_SHA_LEN], uint32_t size, const char *path)
{
    if (!s_loaded) return false;
    char hex[65], line[200];
    sd_sha_hex(sha, hex);
    snprintf(line, sizeof(line), "{\"sha\":\"%s\",\"size\":%u,\"path\":\"%s\"}",
             hex, (unsigned)size, path);
    ent_add(sha, path, false);
    return append_line(line);
}

bool sd_index_add_disk(const uint8_t sha[SD_SHA_LEN], const char *dir, int nfiles)
{
    if (!s_loaded) return false;
    char hex[65], line[200];
    sd_sha_hex(sha, hex);
    snprintf(line, sizeof(line), "{\"disk\":\"%s\",\"path\":\"%s\",\"files\":%d}",
             hex, dir, nfiles);
    ent_add(sha, dir, true);
    return append_line(line);
}

// Rewrite the index from memory via a temp file and rename.
static bool rewrite_index(void)
{
    char ip[300], tp[300];
    idx_path(ip, sizeof(ip));
    snprintf(tp, sizeof(tp), "%s/%s.tmp", SD_MOUNT_POINT, SD_INDEX_FILE);

    FILE *f = fopen(tp, "w");
    if (!f) return false;
    char hex[65];
    bool ok = true;
    for (int i = 0; i < s_n && ok; i++) {
        sd_sha_hex(s_ent[i].sha, hex);
        ok = fprintf(f, s_ent[i].is_disk
                          ? "{\"disk\":\"%s\",\"path\":\"%s\"}\n"
                          : "{\"sha\":\"%s\",\"path\":\"%s\"}\n",
                     hex, s_ent[i].path) > 0;
    }
    fflush(f);
    fsync(fileno(f));
    fclose(f);
    if (!ok) { remove(tp); return false; }
    remove(ip);
    return rename(tp, ip) == 0;
}

bool sd_index_load(void)
{
    sd_index_free();
    if (!sd_present()) return false;

    s_ent = heap_caps_malloc(sizeof(idx_ent_t) * IDX_MAX, MALLOC_CAP_SPIRAM);
    if (!s_ent) {
        ESP_LOGE(TAG, "index alloc failed");
        return false;
    }
    s_n = 0;
    s_loaded = true;

    char ip[300];
    idx_path(ip, sizeof(ip));
    FILE *f = fopen(ip, "r");
    if (!f) {
        ESP_LOGI(TAG, "no index on this card yet — starting empty");
        return true;
    }

    char line[256];
    int pruned = 0, disks = 0;
    while (fgets(line, sizeof(line), f)) {
        char hex[80], path[PATH_MAX_LEN];
        bool is_disk = false;
        if (json_str(line, "sha", hex, sizeof(hex))) is_disk = false;
        else if (json_str(line, "disk", hex, sizeof(hex))) is_disk = true;
        else continue;                                   // blank/torn line
        if (strlen(hex) != 64) continue;
        if (!json_str(line, "path", path, sizeof(path))) continue;
        uint8_t sha[SD_SHA_LEN];
        if (!hex_to_sha(hex, sha)) continue;

        // Prune entries whose file is gone (stat only).
        char abs[360];
        snprintf(abs, sizeof(abs), "%s/%s", SD_MOUNT_POINT, path);
        struct stat st;
        if (stat(abs, &st) != 0) { pruned++; continue; }

        ent_add(sha, path, is_disk);
        if (is_disk) disks++;
    }
    fclose(f);

    if (pruned) {
        ESP_LOGW(TAG, "pruned %d index entries whose files are gone", pruned);
        rewrite_index();
    }
    ESP_LOGI(TAG, "index: %d files, %d disks", sd_index_count(), disks);
    return true;
}

void sd_index_free(void)
{
    if (s_ent) heap_caps_free(s_ent);
    s_ent = NULL;
    s_n = 0;
    s_loaded = false;
}

int sd_index_rebuild(void)
{
    if (!sd_present()) return -1;
    sd_index_free();
    s_ent = heap_caps_malloc(sizeof(idx_ent_t) * IDX_MAX, MALLOC_CAP_SPIRAM);
    if (!s_ent) return -1;
    s_n = 0;
    s_loaded = true;

    // The root is paged (it can hold thousands of DISKnnnn folders); a disk
    // folder holds at most a root directory's worth of files.
    static sd_entry_t dirs[64], files[224];
    uint32_t cursor = 0;
    int nd, total = 0;

    while ((nd = sd_list_path(NULL, &cursor, dirs, 64)) > 0)
    for (int i = 0; i < nd; i++) {
        if (!dirs[i].is_dir) {
            // A single-file save (sd_copy_from_image) lands in the root.
            uint8_t sha[SD_SHA_LEN];
            char abs[360];
            snprintf(abs, sizeof(abs), "%s/%s", SD_MOUNT_POINT, dirs[i].name);
            if (dirs[i].size <= 1474560u &&     // bigger than a floppy: not ours
                strncasecmp(dirs[i].name, SD_INDEX_FILE, strlen(SD_INDEX_FILE)) != 0 &&
                sha_sd_file(abs, sha) && ent_add(sha, dirs[i].name, false))
                total++;
            continue;
        }
        int nf = sd_list_path(dirs[i].name, NULL, files, 224);
        if (nf < 0) continue;

        // Filename order, for a stable disk identity.
        for (int a = 0; a < nf; a++)
            for (int b = a + 1; b < nf; b++)
                if (strcmp(files[b].name, files[a].name) < 0) {
                    sd_entry_t t = files[a]; files[a] = files[b]; files[b] = t;
                }

        static uint8_t shas[224][SD_SHA_LEN];
        int nh = 0;
        for (int j = 0; j < nf; j++) {
            if (files[j].is_dir) continue;
            char rel[64 + 1 + 64 + 1], abs[360];
            snprintf(rel, sizeof(rel), "%s/%s", dirs[i].name, files[j].name);
            snprintf(abs, sizeof(abs), "%s/%s", SD_MOUNT_POINT, rel);
            if (!sha_sd_file(abs, shas[nh])) continue;
            ent_add(shas[nh], rel, false);
            nh++;
            total++;
        }
        uint8_t dsha[SD_SHA_LEN];
        if (nh && sd_sha_disk((const uint8_t (*)[SD_SHA_LEN])shas, nh, dsha))
            ent_add(dsha, dirs[i].name, true);
        ESP_LOGI(TAG, "reindexed %s (%d files)", dirs[i].name, nh);
    }

    if (!rewrite_index()) return -1;
    ESP_LOGW(TAG, "index rebuilt: %d files", total);
    return total;
}
