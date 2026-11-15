// Test-only builder for a real 1.44 MB FAT12 floppy image, so the FatFs read
// path (ff.c + fat12_diskio.c) can be exercised on the host.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Start a fresh empty FAT12 volume in `img` (must be 1.44 MB = 2880*512 bytes).
void fat12_format(uint8_t *img);

// Add a root-directory file. `name8`/`ext3` are space-agnostic (padded here),
// e.g. ("MVC-0001","JPG"). Returns false if the root dir or data area is full.
bool fat12_add_file(uint8_t *img, const char *name8, const char *ext3,
                    const uint8_t *data, uint32_t size);
