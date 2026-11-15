// Minimal host diskio dispatcher: the same ff_diskio_register() contract as
// IDF's diskio.c, but without the ESP/sys-time dependencies. Routes FatFs's
// disk_* calls to the registered ff_diskio_impl_t (our fat12_diskio.c).
#include "ff.h"
#include "diskio.h"
#include "diskio_impl.h"

static const ff_diskio_impl_t *s_impl[FF_VOLUMES];

void ff_diskio_register(BYTE pdrv, const ff_diskio_impl_t *impl)
{
    if (pdrv < FF_VOLUMES) s_impl[pdrv] = impl;
}

DSTATUS disk_status(BYTE pdrv)
{
    return (pdrv < FF_VOLUMES && s_impl[pdrv]) ? s_impl[pdrv]->status(pdrv) : STA_NOINIT;
}
DSTATUS disk_initialize(BYTE pdrv)
{
    return (pdrv < FF_VOLUMES && s_impl[pdrv]) ? s_impl[pdrv]->init(pdrv) : STA_NOINIT;
}
DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    return s_impl[pdrv]->read(pdrv, buff, (uint32_t)sector, count);
}
DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    return s_impl[pdrv]->write(pdrv, buff, (uint32_t)sector, count);
}
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    return s_impl[pdrv]->ioctl(pdrv, cmd, buff);
}

// Fixed timestamp (2025-07-19 12:00:00) — read/write build needs this symbol.
DWORD get_fattime(void)
{
    return ((DWORD)(2025 - 1980) << 25) | ((DWORD)7 << 21) | ((DWORD)19 << 16) |
           ((DWORD)12 << 11);
}
