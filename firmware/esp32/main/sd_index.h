// Content index for the SD card: a SHA-256 per archived file plus one per
// archived disk. Content, not names, because camera file numbering restarts
// after every wipe.
//
// Stored on the card as append-only JSON Lines (/sdcard/.gwindex.jsonl), so it
// travels with the card and a power cut loses at most the last line:
//   {"sha":"<64 hex>","size":1234,"path":"DISK0007/MVC-00001.JPG"}
//   {"disk":"<64 hex>","path":"DISK0007","files":20}
//
// Entries whose file is gone are pruned on load. Files added from a PC are
// unknown until sd_index_rebuild(); the worst case is a duplicate save.
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SD_INDEX_FILE ".gwindex.jsonl"
#define SD_SHA_LEN 32

// Load the index, pruning (and rewriting) entries whose file is gone. A
// missing file gives an empty index. False only with no card or no memory.
bool sd_index_load(void);
void sd_index_free(void);

// `where` (optional) receives the card-relative path of the existing copy.
bool sd_index_has_file(const uint8_t sha[SD_SHA_LEN], char *where, size_t wheresz);
bool sd_index_has_disk(const uint8_t sha[SD_SHA_LEN], char *where, size_t wheresz);

// Append to memory and the card (fsync'd). Add a file only once it is safely
// on the card.
bool sd_index_add_file(const uint8_t sha[SD_SHA_LEN], uint32_t size, const char *path);
bool sd_index_add_disk(const uint8_t sha[SD_SHA_LEN], const char *dir, int nfiles);

int sd_index_count(void);      // indexed files (disk entries not counted)
int sd_index_disk_count(void); // disk entries (one per archived DISKnnnn)
bool sd_index_is_loaded(void);

// Rehash every file in every top-level folder and rewrite the index. Slow
// (minutes for a full card). Returns files indexed, or -1. Call through
// sd_reindex(), which holds the SD lock.
int sd_index_rebuild(void);

// ---- hashing (hardware SHA on the ESP32-S3) ----

// SHA-256 of a file in the floppy image.
bool sd_sha_image_file(const char *name, uint8_t out[SD_SHA_LEN]);

// The same, remembered per name+size so /api/list polling and an SD save
// cost one hash pass per disk between them. Caller holds the volume lock (or
// is the gw task before READY). The cache is cleared when a read starts and
// a deleted file is forgotten; nothing else can change a file's content.
bool sd_sha_image_file_cached(const char *name, uint32_t size, uint8_t out[SD_SHA_LEN]);
void sd_sha_image_cache_clear(void);
void sd_sha_image_cache_forget(const char *name);

// Disk identity: SHA-256 over the per-file hashes in filename order (both
// sd_save_disk() and sd_index_rebuild() sort before calling).
bool sd_sha_disk(const uint8_t (*file_shas)[SD_SHA_LEN], int n,
                 uint8_t out[SD_SHA_LEN]);

// 64-char lowercase hex, NUL-terminated (needs 65 bytes).
void sd_sha_hex(const uint8_t sha[SD_SHA_LEN], char *hex65);
