// FatFs drive 0 over the PSRAM disk image.
#pragma once
#include <stdbool.h>

// Mount after a disk read. False if the image has no valid filesystem.
bool fat12_mount(void);
// Claim drive 0 for the floppy. Call once at boot, before sd_mount().
void fat12_reserve_drive(void);
void fat12_unmount(void);

#define FAT12_ROOT "0:/"
