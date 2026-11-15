#include "fat12_diskio.h"
#include "disk_image.h"
#include "app_config.h"
#include "esp_log.h"
#include "ff.h"
#include "diskio_impl.h"

static const char *TAG = "fat12";
static FATFS s_fs;
static bool  s_mounted;
#define PDRV 0

static DSTATUS ram_status(BYTE pdrv) { (void)pdrv; return 0; }
static DSTATUS ram_init(BYTE pdrv)   { (void)pdrv; return 0; }

static DRESULT ram_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    (void)pdrv;
    return disk_image_read((uint32_t)sector, buff, count) ? RES_OK : RES_PARERR;
}

// Edits (file delete) change only the image; commit_cyl0() writes them back.
static DRESULT ram_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    (void)pdrv;
    return disk_image_write((uint32_t)sector, buff, count) ? RES_OK : RES_PARERR;
}

static DRESULT ram_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    (void)pdrv;
    switch (cmd) {
    case CTRL_SYNC:         return RES_OK;
    case GET_SECTOR_COUNT:  *(LBA_t *)buff = TOTAL_SECTORS;  return RES_OK;
    case GET_SECTOR_SIZE:   *(WORD *)buff  = SECTOR_SIZE;    return RES_OK;
    case GET_BLOCK_SIZE:    *(DWORD *)buff = 1;              return RES_OK;
    default:                return RES_PARERR;
    }
}

static const ff_diskio_impl_t s_impl = {
    .init   = ram_init,
    .status = ram_status,
    .read   = ram_read,
    .write  = ram_write,
    .ioctl  = ram_ioctl,
};

// The SD mount takes the first free FatFs drive, so claim drive 0 before it
// runs and never release it; otherwise fat12_mount() would take over the
// card's drive.
void fat12_reserve_drive(void)
{
    ff_diskio_register(PDRV, &s_impl);
}

bool fat12_mount(void)
{
    if (s_mounted) fat12_unmount();
    ff_diskio_register(PDRV, &s_impl);
    FRESULT fr = f_mount(&s_fs, FAT12_ROOT, 1 /* mount now */);
    if (fr != FR_OK) {
        ESP_LOGW(TAG, "f_mount failed: %d", fr);
        return false;
    }
    s_mounted = true;
    ESP_LOGI(TAG, "FAT mounted (type=%d)", s_fs.fs_type);
    return true;
}

void fat12_unmount(void)
{
    if (!s_mounted) return;
    f_mount(NULL, FAT12_ROOT, 0);
    s_mounted = false;   // drive 0 stays registered
}
