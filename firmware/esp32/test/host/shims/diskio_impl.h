// Host-test shim for IDF's diskio_impl.h.
//
// IDF's version re-typedefs BYTE/UINT/DWORD on top of ff.h's typedefs, which
// gcc/C11 tolerates but tcc/C99 rejects as a duplicate. We only need the driver
// struct + registration hook, so declare those and reuse ff.h/diskio.h's types.
#pragma once
#include "diskio.h"     // BYTE, DSTATUS, DRESULT (and transitively ff.h)
#include <stdint.h>

// Signatures use FatFs's own typedefs (BYTE/LBA_t/UINT) so they match
// fat12_diskio.c's functions exactly under tcc's strict pointer checks.
typedef struct {
    DSTATUS (*init)  (BYTE pdrv);
    DSTATUS (*status)(BYTE pdrv);
    DRESULT (*read)  (BYTE pdrv, BYTE *buff, LBA_t sector, UINT count);
    DRESULT (*write) (BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count);
    DRESULT (*ioctl) (BYTE pdrv, BYTE cmd, void *buff);
} ff_diskio_impl_t;

void ff_diskio_register(BYTE pdrv, const ff_diskio_impl_t *discio_impl);
#define ff_diskio_unregister(pdrv_) ff_diskio_register(pdrv_, NULL)
