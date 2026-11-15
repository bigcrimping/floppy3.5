// The decoded 1.44 MB disk image in PSRAM, with per-sector validity. FatFs
// mounts it as volume 0 (fat12_diskio.c).
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "app_config.h"

#define TOTAL_SECTORS (DISK_CYLS * DISK_HEADS * DISK_SECTORS)
#define TOTAL_TRACKS  (DISK_CYLS * DISK_HEADS)

typedef enum {
    DISK_EMPTY = 0,   // no disk read yet
    DISK_READING,     // read in progress
    DISK_READY,       // fully decoded, all sectors valid
    DISK_READY_DIRTY, // decoded but >=1 sector failed CRC (badge affected files)
    DISK_ERROR,       // fatal (no disk / no track0 / protocol timeout)
} disk_state_t;

// Allocate the image in PSRAM. Call once at boot. Returns false on OOM.
bool disk_image_init(void);

// Reset all sectors to "not yet read" before a fresh disk read.
void disk_image_reset(void);

static inline uint32_t disk_lba(uint8_t cyl, uint8_t head, uint8_t sec /*1..18*/)
{
    return ((uint32_t)cyl * DISK_HEADS + head) * DISK_SECTORS + (sec - 1);
}

// Store one 512-byte sector at the given LBA. `valid` = passed CRC.
void disk_image_put(uint32_t lba, const uint8_t *data512, bool valid);

// FatFs read path: copy `count` sectors starting at `lba` into `buf`.
bool disk_image_read(uint32_t lba, uint8_t *buf, uint32_t count);

// FatFs write path (file delete). Written sectors become valid.
bool disk_image_write(uint32_t lba, const uint8_t *buf, uint32_t count);

// ---- Cylinder-0 rollback checkpoint ----
// A delete edits cyl0 in the image before it is written to the disk. If that
// write fails, roll back so the image matches the disk again. A second
// checkpoint while armed keeps the older snapshot. disk_image_reset() disarms.
void disk_image_checkpoint_cyl0(void);
bool disk_image_rollback_cyl0(void);   // restore + disarm; false if not armed
void disk_image_discard_cyl0(void);    // disarm (commit reached the disk)

bool     disk_sector_valid(uint32_t lba);
uint32_t disk_bad_sector_count(void);

disk_state_t disk_get_state(void);
void         disk_set_state(disk_state_t s);
// Changes whenever the image's file list changes: bumped by disk_set_state()
// on entering READY/READY_DIRTY, and by disk_bump_generation() after a delete
// or its rollback. Starts random per boot, so values don't repeat across
// reboots.
uint32_t     disk_generation(void);
void         disk_bump_generation(void);

// ---- FatFs volume 0 (the mounted image) ----
// Web handlers call disk_volume_acquire() before touching the volume; it fails
// if no disk is mounted. The gw task holds the lock while it changes the state
// away from READY or remounts, so a read can't unmount the volume under a
// handler. Recursive.
bool disk_volume_acquire(void);   // lock; false (and unlocked) unless READY/DIRTY
void disk_volume_lock(void);
void disk_volume_unlock(void);

// Status detail for /api/status and the LCD. Stored by reference, so pass a
// string with static lifetime.
void         disk_set_detail(const char *reason);
const char  *disk_get_detail(void);

// Progress for the LCD, LEDs and /api/status: sectors seen so far, out of
// disk_sectors_total(). Tracks a FAT-aware read skips are filled in at once
// and would make the figure jump, so the read reports them with
// disk_progress_set_skipped() and they count in neither number (a disk with
// 30 tracks in use runs 0..540). disk_image_reset() clears that. A job that
// doesn't fill the image (a retry of a few tracks, a disk write) reports its
// own progress out of TOTAL_SECTORS with the override; <0 ends it.
uint32_t disk_sectors_done(void);
uint32_t disk_sectors_total(void);
void     disk_progress_set_skipped(uint32_t sectors);
void     disk_progress_override(int32_t sectors);

// ---- An uploaded image waiting to be written to a floppy ----
// disk_stage_set() takes ownership of a DISK_IMAGE_BYTES heap_caps buffer
// (freeing one already staged); disk_stage_take() hands it to the gw task,
// which frees it. NULL = nothing staged.
void     disk_stage_set(uint8_t *img);
uint8_t *disk_stage_take(void);

// ---- Per-track read outcome (/api/trackmap, LCD) ----
typedef enum {
    TRK_UNREAD = 0,   // not read this pass (FAT-aware skip, or before any read)
    TRK_GOOD   = 1,   // decoded clean on the first read
    TRK_RETRY  = 2,   // decoded clean only after a retry
    TRK_BAD    = 3,   // still had bad/missing sectors after all retries (or link-dead)
} track_state_t;

void disk_trackmap_reset(void);                                  // all -> TRK_UNREAD
void disk_trackmap_set(uint8_t cyl, uint8_t head, track_state_t st);
const uint8_t *disk_trackmap(void);   // TOTAL_TRACKS bytes; idx = cyl*DISK_HEADS+head
