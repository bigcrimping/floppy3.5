// Host test for fat_scan.c (the FAT12 used-track scanner). Builds synthetic
// FAT12 volumes with fat12_image.c, loads them into the disk_image stub, and
// checks fat_used_tracks() selects exactly the metadata + allocated-cluster
// tracks. Mirrors the logic validated byte-for-byte against fat.py / the
// reads/floppy_* fixtures.
#include "fat_scan.h"      // pulls in disk_image.h + app_config.h
#include "fat12_image.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int checks, failures;
#define CHECK(cond) do { checks++; if (!(cond)) { \
    failures++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)

// Load a raw 1.44 MB image into the stub (all sectors valid), run the scan,
// return the used-track count and (via out params) the cylinder span.
static int scan(const uint8_t *img, int *mincyl, int *maxcyl)
{
    static uint8_t sec[SECTOR_SIZE];
    disk_image_reset();
    for (uint32_t lba = 0; lba < TOTAL_SECTORS; lba++)
        disk_image_put(lba, img + (size_t)lba * SECTOR_SIZE, true);

    bool used[TOTAL_TRACKS];
    if (!fat_used_tracks(used)) return -1;
    int n = 0; *mincyl = 999; *maxcyl = -1;
    for (int i = 0; i < TOTAL_TRACKS; i++) if (used[i]) {
        n++; int cyl = i / DISK_HEADS;
        if (cyl < *mincyl) *mincyl = cyl;
        if (cyl > *maxcyl) *maxcyl = cyl;
    }
    return n;
}

int main(void)
{
    if (!disk_image_init()) { printf("stub init failed\n"); return 2; }
    uint8_t *img = malloc(DISK_IMAGE_BYTES);
    if (!img) { printf("img alloc failed\n"); return 2; }
    int lo, hi;

    // Empty volume: only the metadata tracks (cyl 0, both heads) are "used".
    fat12_format(img);
    CHECK(scan(img, &lo, &hi) == 2);
    CHECK(lo == 0 && hi == 0);

    // Small file (~5 KB = 10 clusters, LBA 33..42) reaches into cyl 1.
    fat12_format(img);
    static uint8_t blob[131072];
    memset(blob, 0xA5, sizeof(blob));
    CHECK(fat12_add_file(img, "SMALL", "BIN", blob, 5000));
    CHECK(scan(img, &lo, &hi) == 3);          // tracks {c0h0, c0h1, c1h0}
    CHECK(lo == 0 && hi == 1);

    // Larger file (100 KB = 196 clusters, LBA 33..228) reaches cyl 6.
    fat12_format(img);
    CHECK(fat12_add_file(img, "BIG", "BIN", blob, 100000));
    CHECK(scan(img, &lo, &hi) == 13);         // linear tracks 0..12
    CHECK(hi == 6);

    // Not FAT12 (zeroed boot sector) -> parse fails, caller falls back.
    memset(img, 0, DISK_IMAGE_BYTES);
    CHECK(scan(img, &lo, &hi) == -1);

    printf("\n%d checks, %d failures\n%s\n", checks, failures,
           failures ? "SOME FAILED" : "ALL PASS");
    return failures ? 1 : 0;
}
