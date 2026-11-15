// Which tracks hold FAT12 metadata or allocated clusters, from the boot
// sector and FAT already in the image (cylinder 0 must be read first). A C
// port of tools/uart_host/fat.py.
#pragma once
#include <stdbool.h>
#include "disk_image.h"

// `used` is TOTAL_TRACKS bools (cyl*DISK_HEADS+head). False if the image isn't
// FAT12 or the boot sector / FAT is CRC-bad: read the whole disk instead.
bool fat_used_tracks(bool *used);
