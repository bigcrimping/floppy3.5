// Host-test stand-in for disk_image.c: same public API, but plain malloc and no
// FreeRTOS mutex. Lets the MFM decoder write sectors that the test can inspect
// via disk_image_read()/disk_sector_valid().
#include "disk_image.h"
#include <stdlib.h>
#include <string.h>

static uint8_t *s_image;
static uint8_t  s_valid[(TOTAL_SECTORS + 7) / 8];
static uint8_t  s_seen[(TOTAL_SECTORS + 7) / 8];
static disk_state_t s_state = DISK_EMPTY;

static inline void bit_set(uint8_t *bm, uint32_t i) { bm[i >> 3] |=  (1u << (i & 7)); }
static inline int  bit_get(const uint8_t *bm, uint32_t i){ return bm[i >> 3] & (1u << (i & 7)); }

bool disk_image_init(void)
{
    s_image = malloc(DISK_IMAGE_BYTES);
    if (!s_image) return false;
    disk_image_reset();
    return true;
}

void disk_image_reset(void)
{
    memset(s_image, 0, DISK_IMAGE_BYTES);
    memset(s_valid, 0, sizeof(s_valid));
    memset(s_seen, 0, sizeof(s_seen));
    s_state = DISK_EMPTY;
}

void disk_image_put(uint32_t lba, const uint8_t *data512, bool valid)
{
    if (lba >= TOTAL_SECTORS) return;
    if (!bit_get(s_valid, lba)) {   // keep the first good copy, as disk_image.c
        memcpy(s_image + (size_t)lba * SECTOR_SIZE, data512, SECTOR_SIZE);
        bit_set(s_seen, lba);
        if (valid) bit_set(s_valid, lba);
    }
}

bool disk_image_read(uint32_t lba, uint8_t *buf, uint32_t count)
{
    if (lba + count > TOTAL_SECTORS) return false;
    memcpy(buf, s_image + (size_t)lba * SECTOR_SIZE, (size_t)count * SECTOR_SIZE);
    return true;
}

bool disk_image_write(uint32_t lba, const uint8_t *buf, uint32_t count)
{
    if (lba + count > TOTAL_SECTORS) return false;
    memcpy(s_image + (size_t)lba * SECTOR_SIZE, buf, (size_t)count * SECTOR_SIZE);
    for (uint32_t i = 0; i < count; i++) { bit_set(s_seen, lba + i); bit_set(s_valid, lba + i); }
    return true;
}

// Cylinder-0 checkpoint, as disk_image.c (bitmaps copied a bit at a time).
#define CKPT_SECTORS (DISK_HEADS * DISK_SECTORS)
static uint8_t s_ckpt[CKPT_SECTORS * SECTOR_SIZE];
static uint8_t s_ckpt_valid[(CKPT_SECTORS + 7) / 8], s_ckpt_seen[(CKPT_SECTORS + 7) / 8];
static bool    s_ckpt_armed;

static void bit_copy(uint8_t *dst, const uint8_t *src, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        if (bit_get(src, i)) bit_set(dst, i); else dst[i >> 3] &= ~(1u << (i & 7));
    }
}

void disk_image_checkpoint_cyl0(void)
{
    if (s_ckpt_armed) return;
    memcpy(s_ckpt, s_image, sizeof s_ckpt);
    bit_copy(s_ckpt_valid, s_valid, CKPT_SECTORS);
    bit_copy(s_ckpt_seen,  s_seen,  CKPT_SECTORS);
    s_ckpt_armed = true;
}

bool disk_image_rollback_cyl0(void)
{
    if (!s_ckpt_armed) return false;
    memcpy(s_image, s_ckpt, sizeof s_ckpt);
    bit_copy(s_valid, s_ckpt_valid, CKPT_SECTORS);
    bit_copy(s_seen,  s_ckpt_seen,  CKPT_SECTORS);
    s_ckpt_armed = false;
    return true;
}

void disk_image_discard_cyl0(void) { s_ckpt_armed = false; }

bool disk_sector_valid(uint32_t lba) { return lba < TOTAL_SECTORS && bit_get(s_valid, lba); }

uint32_t disk_bad_sector_count(void)
{
    uint32_t bad = 0;
    for (uint32_t i = 0; i < TOTAL_SECTORS; i++) if (!bit_get(s_valid, i)) bad++;
    return bad;
}

uint32_t disk_sectors_done(void)
{
    uint32_t n = 0;
    for (uint32_t i = 0; i < TOTAL_SECTORS; i++) if (bit_get(s_seen, i)) n++;
    return n;
}

disk_state_t disk_get_state(void)           { return s_state; }
void         disk_set_state(disk_state_t s) { s_state = s; }
