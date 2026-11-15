#include "disk_image.h"
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static uint8_t     *s_image;                       // DISK_IMAGE_BYTES in PSRAM
static uint8_t      s_valid[(TOTAL_SECTORS + 7)/8];// per-sector CRC-pass bitmap
static uint8_t      s_seen[(TOTAL_SECTORS + 7)/8]; // per-sector "written" bitmap
static volatile uint32_t s_skipped;                // sectors of tracks the read skips
static volatile disk_state_t s_state = DISK_EMPTY;
static SemaphoreHandle_t s_lock;                   // guards image + bitmaps + checkpoint
static SemaphoreHandle_t s_vol;                    // FatFs volume 0 (recursive)

// Cylinder-0 checkpoint (see disk_image.h): LBAs 0..CKPT_SECTORS-1.
#define CKPT_SECTORS (DISK_HEADS * DISK_SECTORS)
static uint8_t *s_ckpt;                            // CKPT_SECTORS*SECTOR_SIZE in PSRAM
static uint8_t  s_ckpt_valid[(CKPT_SECTORS + 7)/8];
static uint8_t  s_ckpt_seen[(CKPT_SECTORS + 7)/8];
static bool     s_ckpt_armed;

static inline void bit_set(uint8_t *bm, uint32_t i) { bm[i >> 3] |=  (1u << (i & 7)); }
static inline void bit_clr(uint8_t *bm, uint32_t i) { bm[i >> 3] &= ~(1u << (i & 7)); }
static inline bool bit_get(const uint8_t *bm, uint32_t i){ return bm[i >> 3] & (1u << (i & 7)); }
static inline void bit_put(uint8_t *bm, uint32_t i, bool v){ if (v) bit_set(bm, i); else bit_clr(bm, i); }

bool disk_image_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    s_vol  = xSemaphoreCreateRecursiveMutex();
    if (!s_lock || !s_vol) return false;
    s_image = heap_caps_malloc(DISK_IMAGE_BYTES, MALLOC_CAP_SPIRAM);
    if (!s_image) return false;
    s_ckpt = heap_caps_malloc(CKPT_SECTORS * SECTOR_SIZE, MALLOC_CAP_SPIRAM);
    if (!s_ckpt) return false;
    disk_image_reset();
    return true;
}

void disk_image_reset(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    memset(s_image, 0, DISK_IMAGE_BYTES);
    memset(s_valid, 0, sizeof(s_valid));
    memset(s_seen,  0, sizeof(s_seen));
    s_ckpt_armed = false;
    s_state = DISK_EMPTY;
    s_skipped = 0;
    xSemaphoreGive(s_lock);
}

void disk_image_checkpoint_cyl0(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (!s_ckpt_armed) {
        memcpy(s_ckpt, s_image, CKPT_SECTORS * SECTOR_SIZE);
        for (uint32_t i = 0; i < CKPT_SECTORS; i++) {
            bit_put(s_ckpt_valid, i, bit_get(s_valid, i));
            bit_put(s_ckpt_seen,  i, bit_get(s_seen,  i));
        }
        s_ckpt_armed = true;
    }
    xSemaphoreGive(s_lock);
}

bool disk_image_rollback_cyl0(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool armed = s_ckpt_armed;
    if (armed) {
        memcpy(s_image, s_ckpt, CKPT_SECTORS * SECTOR_SIZE);
        for (uint32_t i = 0; i < CKPT_SECTORS; i++) {
            bit_put(s_valid, i, bit_get(s_ckpt_valid, i));
            bit_put(s_seen,  i, bit_get(s_ckpt_seen,  i));
        }
        s_ckpt_armed = false;
    }
    xSemaphoreGive(s_lock);
    return armed;
}

void disk_image_discard_cyl0(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_ckpt_armed = false;
    xSemaphoreGive(s_lock);
}

void disk_image_put(uint32_t lba, const uint8_t *data512, bool valid)
{
    if (lba >= TOTAL_SECTORS) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    // Keep the first good copy; a bad copy is kept only until a good one comes.
    if (!bit_get(s_valid, lba)) {
        memcpy(s_image + (size_t)lba * SECTOR_SIZE, data512, SECTOR_SIZE);
        bit_set(s_seen, lba);
        if (valid) bit_set(s_valid, lba);
    }
    xSemaphoreGive(s_lock);
}

bool disk_image_read(uint32_t lba, uint8_t *buf, uint32_t count)
{
    if (lba + count > TOTAL_SECTORS) return false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    memcpy(buf, s_image + (size_t)lba * SECTOR_SIZE, (size_t)count * SECTOR_SIZE);
    xSemaphoreGive(s_lock);
    return true;
}

bool disk_image_write(uint32_t lba, const uint8_t *buf, uint32_t count)
{
    if (lba + count > TOTAL_SECTORS) return false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    memcpy(s_image + (size_t)lba * SECTOR_SIZE, buf, (size_t)count * SECTOR_SIZE);
    for (uint32_t i = 0; i < count; i++) { bit_set(s_seen, lba + i); bit_set(s_valid, lba + i); }
    xSemaphoreGive(s_lock);
    return true;
}

bool disk_sector_valid(uint32_t lba)
{
    return lba < TOTAL_SECTORS && bit_get(s_valid, lba);
}

uint32_t disk_bad_sector_count(void)
{
    uint32_t bad = 0;
    for (uint32_t i = 0; i < TOTAL_SECTORS; i++)
        if (!bit_get(s_valid, i)) bad++;
    return bad;
}

static volatile int32_t s_progress = -1;
void disk_progress_override(int32_t sectors) { s_progress = sectors; }

static uint8_t *s_staged;
void disk_stage_set(uint8_t *img)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    uint8_t *old = s_staged;
    s_staged = img;
    xSemaphoreGive(s_lock);
    if (old) heap_caps_free(old);
}

uint8_t *disk_stage_take(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    uint8_t *img = s_staged;
    s_staged = NULL;
    xSemaphoreGive(s_lock);
    return img;
}

void disk_progress_set_skipped(uint32_t sectors)
{
    // Never the whole disk: the total is a divisor.
    s_skipped = sectors >= TOTAL_SECTORS ? TOTAL_SECTORS - 1 : sectors;
}

uint32_t disk_sectors_total(void)
{
    return s_progress >= 0 ? TOTAL_SECTORS : TOTAL_SECTORS - s_skipped;
}

uint32_t disk_sectors_done(void)
{
    int32_t p = s_progress;
    if (p >= 0) return p > TOTAL_SECTORS ? TOTAL_SECTORS : (uint32_t)p;
    uint32_t n = 0;
    for (uint32_t i = 0; i < TOTAL_SECTORS; i++)
        if (bit_get(s_seen, i)) n++;
    // The skipped tracks are announced before they are filled in.
    uint32_t skip = s_skipped;
    return n > skip ? n - skip : 0;
}

// Changes whenever the file list a client would see changes (a read finished,
// a delete, a delete undone), so a client polling /api/status can tell when
// to resync (floppy_screen). 0 until the first bump, then counts up from a
// random start: a plain counter would give the first disk after every reboot
// the same number, and a client that saw it before the reboot would skip it.
static volatile uint32_t s_gen;
static bool s_gen_seeded;
static portMUX_TYPE s_gen_mux = portMUX_INITIALIZER_UNLOCKED;

void disk_bump_generation(void)        // gw task (read, undo) and httpd (delete)
{
    uint32_t seed = s_gen_seeded ? 0 : esp_random();
    portENTER_CRITICAL(&s_gen_mux);
    if (!s_gen_seeded) { s_gen = seed; s_gen_seeded = true; }
    s_gen++;
    portEXIT_CRITICAL(&s_gen_mux);
}

disk_state_t disk_get_state(void)          { return s_state; }
void disk_set_state(disk_state_t s)
{
    bool was = (s_state == DISK_READY || s_state == DISK_READY_DIRTY);
    bool now = (s == DISK_READY || s == DISK_READY_DIRTY);
    if (now && !was) disk_bump_generation();
    s_state = s;
}
uint32_t disk_generation(void)             { return s_gen; }

void disk_volume_lock(void)   { xSemaphoreTakeRecursive(s_vol, portMAX_DELAY); }
void disk_volume_unlock(void) { xSemaphoreGiveRecursive(s_vol); }

bool disk_volume_acquire(void)
{
    disk_volume_lock();
    if (s_state == DISK_READY || s_state == DISK_READY_DIRTY) return true;
    disk_volume_unlock();
    return false;
}

static const char *volatile s_detail = "";
void        disk_set_detail(const char *reason){ s_detail = reason ? reason : ""; }
const char *disk_get_detail(void)             { return (const char *)s_detail; }

// Written only by the gw task; readers tolerate a stale value, so no lock.
static uint8_t s_trackmap[TOTAL_TRACKS];
void disk_trackmap_reset(void){ memset(s_trackmap, TRK_UNREAD, sizeof s_trackmap); }
void disk_trackmap_set(uint8_t cyl, uint8_t head, track_state_t st)
{
    if (cyl < DISK_CYLS && head < DISK_HEADS)
        s_trackmap[(uint32_t)cyl * DISK_HEADS + head] = (uint8_t)st;
}
const uint8_t *disk_trackmap(void){ return s_trackmap; }
