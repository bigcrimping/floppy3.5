// Host test for the FAT12 read path: build a real 1.44 MB FAT12 image, load it
// into the disk-image store, then mount it with the *real* fat12_diskio.c + IDF
// ff.c and verify directory listing + byte-exact file reads.
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ff.h"
#include "disk_image.h"
#include "fat12_diskio.h"
#include "fat12_image.h"

static int g_fail = 0, g_checks = 0;
#define CHECK(cond, ...) do { g_checks++; if (!(cond)) { \
    g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

#define IMG_BYTES (2880 * 512)

typedef struct { const char *name; uint8_t *data; uint32_t size; } tfile_t;

static uint8_t buf1[1500], buf2[600], buf3[64];

static void load_image_into_store(const uint8_t *img)
{
    disk_image_reset();
    for (uint32_t lba = 0; lba < 2880; lba++)
        disk_image_put(lba, img + (size_t)lba * 512, true);
    disk_set_state(DISK_READY);
}

static int read_whole(const char *path, uint8_t *out, uint32_t cap, uint32_t *outlen)
{
    FIL f;
    if (f_open(&f, path, FA_READ) != FR_OK) return -1;
    UINT br; uint32_t total = 0;
    while (total < cap) {
        if (f_read(&f, out + total, cap - total, &br) != FR_OK) { f_close(&f); return -2; }
        if (br == 0) break;
        total += br;
    }
    f_close(&f);
    *outlen = total;
    return 0;
}

int main(void)
{
    printf("test_fat12\n");

    // Deterministic payloads spanning 3 / 2 / 1 clusters.
    for (int i = 0; i < (int)sizeof(buf1); i++) buf1[i] = (uint8_t)(i * 31 + 7);
    for (int i = 0; i < (int)sizeof(buf2); i++) buf2[i] = (uint8_t)(i * 17 + 3);
    memcpy(buf3, "Floppy bridge host FAT12 test file.\n", 36);

    tfile_t files[] = {
        { "MVC-0001.JPG", buf1, sizeof(buf1) },
        { "MVC-0002.JPG", buf2, sizeof(buf2) },
        { "INFO.TXT",     buf3, 36 },
    };
    const int NF = 3;

    static uint8_t img[IMG_BYTES];
    fat12_format(img);
    CHECK(fat12_add_file(img, "MVC-0001", "JPG", buf1, sizeof(buf1)), "add file 1");
    CHECK(fat12_add_file(img, "MVC-0002", "JPG", buf2, sizeof(buf2)), "add file 2");
    CHECK(fat12_add_file(img, "INFO",     "TXT", buf3, 36),           "add file 3");

    if (!disk_image_init()) { printf("disk_image_init failed\n"); return 2; }
    load_image_into_store(img);

    CHECK(fat12_mount(), "FAT12 volume mounts");

    // --- directory listing ---
    FF_DIR dir; FILINFO fno;
    int seen[3] = { 0 };
    CHECK(f_opendir(&dir, FAT12_ROOT) == FR_OK, "opendir");
    int nlisted = 0;
    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0]) {
        if (fno.fattrib & (AM_DIR | AM_SYS | AM_HID)) continue;  // f_readdir already skips volume labels
        nlisted++;
        for (int k = 0; k < NF; k++) {
            if (strcmp(fno.fname, files[k].name) == 0) {  // SFNs come back uppercase
                seen[k] = 1;
                CHECK(fno.fsize == files[k].size,
                      "%s size %lu (want %u)", fno.fname,
                      (unsigned long)fno.fsize, files[k].size);
            }
        }
    }
    f_closedir(&dir);
    CHECK(nlisted == NF, "listed %d files (want %d)", nlisted, NF);
    CHECK(seen[0] && seen[1] && seen[2], "all three files present in listing");

    // --- byte-exact reads ---
    for (int k = 0; k < NF; k++) {
        char path[64];
        snprintf(path, sizeof(path), "%s%s", FAT12_ROOT, files[k].name);
        static uint8_t rd[2048]; uint32_t got = 0;
        int rc = read_whole(path, rd, sizeof(rd), &got);
        CHECK(rc == 0 && got == files[k].size &&
              memcmp(rd, files[k].data, files[k].size) == 0,
              "%s read back byte-exact (rc=%d got=%u)", files[k].name, rc, got);
    }

    // --- delete rollback: a commit that never reached the disk (write-
    // protected) puts cyl0 back and remounts, so the file is listed again ---
    {
        char path[64];
        snprintf(path, sizeof(path), "%s%s", FAT12_ROOT, files[1].name);
        FILINFO fi;
        disk_image_checkpoint_cyl0();
        CHECK(f_unlink(path) == FR_OK, "delete %s", files[1].name);
        disk_image_checkpoint_cyl0();   // a second arm must keep the older snapshot
        CHECK(f_stat(path, &fi) == FR_NO_FILE, "%s gone after delete", files[1].name);
        CHECK(disk_image_rollback_cyl0(), "rollback restores an armed checkpoint");
        CHECK(!disk_image_rollback_cyl0(), "rollback disarms the checkpoint");
        CHECK(fat12_mount(), "remount after rollback");
        CHECK(f_stat(path, &fi) == FR_OK && fi.fsize == files[1].size,
              "%s listed again after rollback", files[1].name);
        static uint8_t rd[2048]; uint32_t got = 0;
        int rc = read_whole(path, rd, sizeof(rd), &got);
        CHECK(rc == 0 && got == files[1].size &&
              memcmp(rd, files[1].data, files[1].size) == 0,
              "%s byte-exact after rollback (rc=%d got=%u)", files[1].name, rc, got);

        disk_image_checkpoint_cyl0();
        CHECK(f_unlink(path) == FR_OK, "delete again");
        disk_image_discard_cyl0();      // commit reached the disk: keep the delete
        CHECK(!disk_image_rollback_cyl0(), "nothing to roll back after discard");
        CHECK(f_stat(path, &fi) == FR_NO_FILE, "%s stays deleted after discard", files[1].name);
    }

    fat12_unmount();
    printf("\n%d checks, %d failures\n", g_checks, g_fail);
    if (g_fail == 0) printf("ALL PASS\n");
    return g_fail ? 1 : 0;
}
