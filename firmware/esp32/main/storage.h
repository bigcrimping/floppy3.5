// microSD card at SD_MOUNT_POINT, the destination for files copied off the
// floppy image. Everything reports "absent" when no card is mounted.
#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    char     name[64];   // no path
    uint32_t size;       // 0 for a directory
    bool     is_dir;
} sd_entry_t;

// Init the SPI bus and mount. Non-fatal: false if there is no card.
bool sd_mount(void);
bool sd_present(void);
bool sd_usage(uint64_t *total_bytes, uint64_t *free_bytes);

// List a directory relative to the mount point ("" / NULL = root), including
// subdirectories, up to `max` entries a call. To page through a big directory
// start `*cursor` at 0 and call again until it returns 0; NULL = first page
// only. -1 if no card, an unsafe path, or unreadable.
int sd_list_path(const char *rel, uint32_t *cursor, sd_entry_t *out, int max);

// True for a safe card-relative path: no "..", no absolute or backslash
// forms, no empty segments, and short enough for the path buffers.
bool sd_path_is_safe(const char *rel);

// Copy one image file (0:/NAME) to /sdcard/NAME, overwriting, and add it to
// the content index. `name` must pass
// web_name_is_safe(). Returns bytes copied, or -1 no card, -2 bad name,
// -3 source open, -4 dest open, -5 read, -6 write.
long sd_copy_from_image(const char *name);

// ---- whole-disk archiving ----
// Each floppy goes into its own /sdcard/DISKnnnn/ (one past the highest
// existing), because Mavica cameras restart file numbering after every wipe.
#define SD_DISK_DIR_PREFIX "DISK"

typedef struct {
    char dir[16];        // folder written, or the existing one on DUPLICATE
    int  copied;
    int  failed;
    int  skipped;        // already on the card (same SHA-256)
    char first_dupe[40]; // where one skipped file already lives
} sd_save_result_t;

typedef enum {
    SD_SAVE_OK = 0,
    SD_SAVE_DUPLICATE,   // this disk (or every file on it) is already saved
    SD_SAVE_NO_CARD,
    SD_SAVE_NO_IMAGE,    // no image mounted, or no files
    SD_SAVE_MKDIR_FAIL,
    SD_SAVE_CARD_FULL,   // DISK9999 reached
    SD_SAVE_NO_SPACE,    // refused before writing anything
} sd_save_status_t;

// Archive the image into a new DISKnnnn folder. Deduplicates by content:
// files already on the card are skipped, so a folder isn't always the whole
// floppy (skipped/first_dupe say where the rest are). force=true copies
// everything and skips the duplicate check.
sd_save_status_t sd_save_disk(bool force, sd_save_result_t *out);

// Rebuild the content index (sd_index_rebuild) under the SD lock, so it can't
// run under a save. Slow. Returns files indexed, or -1.
int sd_reindex(void);

void sd_detect_init(void);

// Card seated per the CD switch; without SD_DET_GPIO, the mount state.
bool sd_card_detected(void);

// Debounce the CD switch and mount/unmount on change. Call periodically (the
// UI task does, every 30 ms); never blocks. No-op without SD_DET_GPIO.
void sd_detect_poll(void);
