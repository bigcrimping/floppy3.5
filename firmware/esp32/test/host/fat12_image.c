#include "fat12_image.h"
#include <string.h>

// Standard 1.44 MB floppy geometry.
#define SECSZ       512
#define TOTAL_SEC   2880
#define RSVD        1
#define NFATS       2
#define SPF         9            // sectors per FAT
#define ROOT_ENTS   224
#define SPC         1            // sectors per cluster

#define FAT0_SEC    RSVD                          // 1
#define ROOT_SEC    (RSVD + NFATS * SPF)          // 19
#define ROOT_SECS   (ROOT_ENTS * 32 / SECSZ)      // 14
#define DATA_SEC    (ROOT_SEC + ROOT_SECS)        // 33
#define MAX_CLUST   (TOTAL_SEC - DATA_SEC + 2)    // last valid cluster + 1

static inline void wr16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static inline void wr32(uint8_t *p, uint32_t v) { p[0]=v; p[1]=v>>8; p[2]=v>>16; p[3]=v>>24; }

static uint8_t *sec_ptr(uint8_t *img, uint32_t sec) { return img + (size_t)sec * SECSZ; }

// --- FAT12 12-bit entry access (operates on FAT copy #0; mirrored on finish) ---
static void fat_set(uint8_t *fat, uint32_t clust, uint16_t val)
{
    uint32_t off = clust + clust / 2;   // *1.5
    if (clust & 1) {
        fat[off]   = (fat[off] & 0x0F) | ((val << 4) & 0xF0);
        fat[off+1] = (val >> 4) & 0xFF;
    } else {
        fat[off]   = val & 0xFF;
        fat[off+1] = (fat[off+1] & 0xF0) | ((val >> 8) & 0x0F);
    }
}

void fat12_format(uint8_t *img)
{
    memset(img, 0, (size_t)TOTAL_SEC * SECSZ);

    uint8_t *bs = sec_ptr(img, 0);
    bs[0]=0xEB; bs[1]=0x3C; bs[2]=0x90;          // jmp
    memcpy(bs + 3, "MSDOS5.0", 8);
    wr16(bs + 11, SECSZ);
    bs[13] = SPC;
    wr16(bs + 14, RSVD);
    bs[16] = NFATS;
    wr16(bs + 17, ROOT_ENTS);
    wr16(bs + 19, TOTAL_SEC);
    bs[21] = 0xF0;                                // media descriptor
    wr16(bs + 22, SPF);
    wr16(bs + 24, 18);                            // sectors/track
    wr16(bs + 26, 2);                             // heads
    wr32(bs + 28, 0);                            // hidden
    wr32(bs + 32, 0);                            // total32
    bs[36] = 0x00;                               // drive
    bs[38] = 0x29;                               // ext boot sig
    wr32(bs + 39, 0x4D415643);                   // volume id
    memcpy(bs + 43, "FLOPPY     ", 11);
    memcpy(bs + 54, "FAT12   ", 8);
    bs[510] = 0x55; bs[511] = 0xAA;

    // FAT reserved entries: 0xFF0 (media) and 0xFFF (EOC).
    uint8_t *fat = sec_ptr(img, FAT0_SEC);
    fat_set(fat, 0, 0xFF0);
    fat_set(fat, 1, 0xFFF);
    memcpy(sec_ptr(img, FAT0_SEC + SPF), fat, (size_t)SPF * SECSZ); // mirror FAT2
}

static uint32_t next_free_cluster(const uint8_t *img)
{
    const uint8_t *fat = img + (size_t)FAT0_SEC * SECSZ;
    for (uint32_t c = 2; c < MAX_CLUST; c++) {
        uint32_t off = c + c / 2;
        uint16_t v = (c & 1) ? ((fat[off] >> 4) | (fat[off+1] << 4))
                             : (fat[off] | ((fat[off+1] & 0x0F) << 8));
        if ((v & 0xFFF) == 0) return c;
    }
    return 0;
}

static uint8_t *free_dir_entry(uint8_t *img)
{
    uint8_t *root = sec_ptr(img, ROOT_SEC);
    for (int i = 0; i < ROOT_ENTS; i++) {
        uint8_t *e = root + i * 32;
        if (e[0] == 0x00 || e[0] == 0xE5) return e;
    }
    return NULL;
}

bool fat12_add_file(uint8_t *img, const char *name8, const char *ext3,
                    const uint8_t *data, uint32_t size)
{
    uint8_t *fat = sec_ptr(img, FAT0_SEC);
    uint32_t nclust = size ? (size + SECSZ - 1) / SECSZ : 1;

    // Allocate + chain clusters, copying data as we go.
    uint32_t first = 0, prev = 0, remaining = size;
    for (uint32_t i = 0; i < nclust; i++) {
        uint32_t c = next_free_cluster(img);
        if (!c) return false;
        fat_set(fat, c, 0xFFF);                  // claim (temporarily EOC)
        if (prev) fat_set(fat, prev, c);
        else      first = c;
        uint8_t *dst = sec_ptr(img, DATA_SEC + (c - 2) * SPC);
        uint32_t n = remaining < SECSZ ? remaining : SECSZ;
        if (n) memcpy(dst, data + (size - remaining), n);
        remaining -= n;
        prev = c;
    }

    uint8_t *e = free_dir_entry(img);
    if (!e) return false;
    memset(e, ' ', 11);
    for (int i = 0; i < 8 && name8[i]; i++) e[i]     = (uint8_t)name8[i];
    for (int i = 0; i < 3 && ext3[i];  i++) e[8 + i] = (uint8_t)ext3[i];
    e[11] = 0x20;                                // archive attribute
    wr16(e + 20, 0);                             // first cluster hi (FAT12: 0)
    wr16(e + 26, (uint16_t)first);               // first cluster lo
    wr32(e + 28, size);

    // Keep FAT copy #2 in sync.
    memcpy(sec_ptr(img, FAT0_SEC + SPF), fat, (size_t)SPF * SECSZ);
    return true;
}
