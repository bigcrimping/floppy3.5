// Host tests for sd_index.c, the SD card's content index: hashing, the index
// file (load, append, prune), and the rebuild. The "card" is a folder beside
// the test binary (SD_MOUNT_POINT in shims/host_compat.h); storage.c's two
// entry points are stood in below, with the real paging contract.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <io.h>
#include <direct.h>
#include "ff.h"
#include "mbedtls/sha256.h"
#include "sd_index.h"
#include "storage.h"
#include "disk_image.h"
#include "fat12_diskio.h"
#include "fat12_image.h"

static int g_fail = 0, g_checks = 0;
#define CHECK(cond, ...) do { g_checks++; if (!(cond)) { \
    g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

#define CARD SD_MOUNT_POINT

// ---- storage.c stand-ins ----
bool sd_present(void) { return true; }

int sd_list_path(const char *rel, uint32_t *cursor, sd_entry_t *out, int max)
{
    char pat[300];
    if (rel && rel[0]) snprintf(pat, sizeof pat, "%s/%s/*", CARD, rel);
    else               snprintf(pat, sizeof pat, "%s/*", CARD);
    struct _finddata_t fd;
    intptr_t h = _findfirst(pat, &fd);
    if (h == -1) return -1;
    uint32_t skip = cursor ? *cursor : 0, seen = 0;
    int n = 0;
    do {
        if (!strcmp(fd.name, ".") || !strcmp(fd.name, "..")) continue;
        if (n >= max) break;
        if (seen++ < skip) continue;
        out[n].is_dir = (fd.attrib & _A_SUBDIR) != 0;
        strlcpy(out[n].name, fd.name, sizeof out[n].name);
        out[n].size = out[n].is_dir ? 0u : (uint32_t)fd.size;
        n++;
    } while (_findnext(h, &fd) == 0);
    _findclose(h);
    if (cursor) *cursor = seen;
    return n;
}

// ---- helpers ----
static void sha_of(const void *p, size_t n, uint8_t out[SD_SHA_LEN])
{
    mbedtls_sha256_context c;
    mbedtls_sha256_init(&c);
    mbedtls_sha256_starts(&c, 0);
    mbedtls_sha256_update(&c, p, n);
    mbedtls_sha256_finish(&c, out);
}

static void put_file(const char *rel, const char *text)
{
    char p[300];
    snprintf(p, sizeof p, "%s/%s", CARD, rel);
    FILE *f = fopen(p, "wb");
    if (!f) { printf("  cannot create %s\n", p); exit(2); }
    fwrite(text, 1, strlen(text), f);
    fclose(f);
}

static void make_dir(const char *rel)
{
    char p[300];
    snprintf(p, sizeof p, "%s/%s", CARD, rel);
    _mkdir(p);
}

// Empty the card folder (two levels is all the index ever makes).
static void wipe(const char *dir, int depth)
{
    char pat[300];
    snprintf(pat, sizeof pat, "%s/*", dir);
    struct _finddata_t fd;
    intptr_t h = _findfirst(pat, &fd);
    if (h == -1) return;
    do {
        if (!strcmp(fd.name, ".") || !strcmp(fd.name, "..")) continue;
        char p[300];
        snprintf(p, sizeof p, "%s/%s", dir, fd.name);
        if (fd.attrib & _A_SUBDIR) { if (depth < 2) wipe(p, depth + 1); _rmdir(p); }
        else remove(p);
    } while (_findnext(h, &fd) == 0);
    _findclose(h);
}

static bool has_file(const char *text, const char *want_path)
{
    uint8_t sha[SD_SHA_LEN];
    char where[64] = "";
    sha_of(text, strlen(text), sha);
    if (!sd_index_has_file(sha, where, sizeof where)) return false;
    return !want_path || _stricmp(where, want_path) == 0;
}

int main(int argc, char **argv)
{
    printf("test_sd_index\n");
    // Keep the scratch card next to the binary, wherever the test is run from.
    if (argc > 0) {
        char dir[300];
        strlcpy(dir, argv[0], sizeof dir);
        char *s = strrchr(dir, '\\');
        if (!s) s = strrchr(dir, '/');
        if (s) { *s = '\0'; _chdir(dir); }
    }
    _mkdir(CARD);
    wipe(CARD, 0);

    // --- the SHA-256 stand-in is a real SHA-256 (FIPS 180-2 vectors) ---
    {
        uint8_t sha[SD_SHA_LEN];
        char hex[65];
        sha_of("abc", 3, sha);
        sd_sha_hex(sha, hex);
        CHECK(!strcmp(hex, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"),
              "sha256(abc) = %s", hex);
        const char *two = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
        sha_of(two, strlen(two), sha);
        sd_sha_hex(sha, hex);
        CHECK(!strcmp(hex, "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1"),
              "sha256(two blocks) = %s", hex);
    }

    // --- a card with no index loads empty ---
    CHECK(sd_index_load() && sd_index_is_loaded(), "empty card loads");
    CHECK(sd_index_count() == 0, "and is empty (%d)", sd_index_count());

    // --- rebuild: folders, the disk identity, and loose files in the root ---
    make_dir("DISK0001"); put_file("DISK0001/B.TXT", "bravo"); put_file("DISK0001/A.TXT", "alpha");
    make_dir("DISK0002"); put_file("DISK0002/C.TXT", "charlie");
    put_file("LOOSE.JPG", "a single-file save");
    CHECK(sd_index_rebuild() == 4, "rebuild indexes 4 files");
    CHECK(sd_index_count() == 4, "count is 4 (%d): the index file is not indexed", sd_index_count());
    CHECK(has_file("alpha", "DISK0001/A.TXT"), "a folder file is found, with its path");
    CHECK(has_file("a single-file save", "LOOSE.JPG"), "a root file is found");
    CHECK(!has_file("never saved", NULL), "an unknown file is not");
    {
        // Disk identity = hash of the per-file hashes in name order.
        uint8_t shas[2][SD_SHA_LEN], dsha[SD_SHA_LEN];
        char where[64] = "";
        sha_of("alpha", 5, shas[0]);
        sha_of("bravo", 5, shas[1]);
        sd_sha_disk((const uint8_t (*)[SD_SHA_LEN])shas, 2, dsha);
        CHECK(sd_index_has_disk(dsha, where, sizeof where) && !strcmp(where, "DISK0001"),
              "the disk is known as DISK0001 (%s)", where);
        CHECK(!sd_index_has_file(dsha, NULL, 0), "a disk entry is not a file entry");
    }

    // --- the rebuilt index survives a reload ---
    CHECK(sd_index_load() && sd_index_count() == 4, "reload keeps 4 files (%d)", sd_index_count());
    CHECK(has_file("charlie", "DISK0002/C.TXT"), "reloaded entry found");

    // --- append, reload, then prune an entry whose file has gone ---
    {
        uint8_t sha[SD_SHA_LEN];
        put_file("DISK0002/NEW.TXT", "delta");
        sha_of("delta", 5, sha);
        CHECK(sd_index_add_file(sha, 5, "DISK0002/NEW.TXT"), "append a file");
        CHECK(has_file("delta", "DISK0002/NEW.TXT"), "found at once");
        CHECK(sd_index_load() && has_file("delta", NULL), "and after a reload");
        char p[300];
        snprintf(p, sizeof p, "%s/DISK0002/NEW.TXT", CARD);
        remove(p);
        CHECK(sd_index_load() && !has_file("delta", NULL), "pruned once the file is deleted");
        CHECK(sd_index_count() == 4, "the others stay (%d)", sd_index_count());
        CHECK(sd_index_load() && sd_index_count() == 4, "the pruned index reloads clean");
    }

    // --- rebuild pages through a root with more folders than one page (64) ---
    for (int i = 100; i < 170; i++) {
        char rel[32], text[32];
        snprintf(rel, sizeof rel, "DISK%04d", i);
        make_dir(rel);
        snprintf(rel, sizeof rel, "DISK%04d/P.JPG", i);
        snprintf(text, sizeof text, "photo %d", i);
        put_file(rel, text);
    }
    CHECK(sd_index_rebuild() == 74, "rebuild indexes all 74 files");
    CHECK(has_file("photo 100", "DISK0100/P.JPG"), "first of the 70 found");
    CHECK(has_file("photo 169", "DISK0169/P.JPG"), "last of the 70 found");
    CHECK(has_file("alpha", NULL) && has_file("a single-file save", NULL), "the first ones too");

    // --- no index, nothing found ---
    sd_index_free();
    CHECK(!sd_index_is_loaded() && !has_file("alpha", NULL), "freed index finds nothing");

    // --- hashing a file in the floppy image, and the per-disk cache ---
    {
        static uint8_t img[2880 * 512], body[1500];
        for (int i = 0; i < (int)sizeof body; i++) body[i] = (uint8_t)(i * 31 + 7);
        fat12_format(img);
        fat12_add_file(img, "MVC-0001", "JPG", body, sizeof body);
        if (!disk_image_init()) { printf("disk_image_init failed\n"); return 2; }
        disk_image_write(0, img, 2880);
        disk_set_state(DISK_READY);
        CHECK(fat12_mount(), "image mounts");

        uint8_t want[SD_SHA_LEN], got[SD_SHA_LEN];
        sha_of(body, sizeof body, want);
        CHECK(sd_sha_image_file("MVC-0001.JPG", got) && !memcmp(got, want, SD_SHA_LEN),
              "image file hash matches its content");
        memset(got, 0, sizeof got);
        CHECK(sd_sha_image_file_cached("MVC-0001.JPG", sizeof body, got) &&
              !memcmp(got, want, SD_SHA_LEN), "cached hash matches");
        // A hit is served from the cache: it still answers with the volume gone.
        fat12_unmount();
        memset(got, 0, sizeof got);
        CHECK(sd_sha_image_file_cached("MVC-0001.JPG", sizeof body, got) &&
              !memcmp(got, want, SD_SHA_LEN), "second call is a cache hit");
        sd_sha_image_cache_forget("mvc-0001.jpg");
        CHECK(!sd_sha_image_file_cached("MVC-0001.JPG", sizeof body, got),
              "a forgotten name (any case) is hashed again, and fails unmounted");
        CHECK(!sd_sha_image_file("NOPE.JPG", got), "a missing file fails");
    }

    wipe(CARD, 0);
    _rmdir(CARD);
    printf("\n%d checks, %d failures\n", g_checks, g_fail);
    if (g_fail == 0) printf("ALL PASS\n");
    return g_fail ? 1 : 0;
}
