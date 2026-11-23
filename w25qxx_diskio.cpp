/**
 * @file     w25qxx_diskio.cpp
 * @brief    W25Qxx FatFs 磁盘 I/O 胶水层实现
 *
 * 功能：disk_* 回调转发到 W25qxx 类；512B 扇区写入经读-改-擦-写合并到 4KB
 * 修改：2026-11-21 由 C 配置指针方式改为 W25qxx 实例挂接
 */
#include "w25qxx_diskio.h"

#include <string.h>

#include "esp_log.h"

static const char *TAG = "w25qxx_diskio";
static W25qxx *s_drv = nullptr;

void w25qxx_diskio_attach(W25qxx &drv)
{
    s_drv = &drv;
}

extern "C" DSTATUS w25qxx_disk_initialize(BYTE pdrv)
{
    if (pdrv != 0 || s_drv == nullptr) return STA_NOINIT;
    if (!s_drv->readInfo(s_drv->info())) return STA_NOINIT;
    ESP_LOGI(TAG, "W25QXX Disk Initialized, Size: %u MB",
             (unsigned int)(s_drv->info().total_size / (1024 * 1024)));
    return 0;
}

extern "C" DSTATUS w25qxx_disk_status(BYTE pdrv)
{
    return (pdrv == 0 && s_drv != nullptr) ? 0 : STA_NOINIT;
}

extern "C" DRESULT w25qxx_disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv != 0 || !s_drv || !buff) return RES_PARERR;
    uint32_t addr = sector * 512;
    uint32_t len  = count * 512;
    return s_drv->read(addr, buff, len) ? RES_OK : RES_ERROR;
}

/* read-modify-write：W25QXX 擦除粒度为 4KB，FatFs 以 512B 扇区请求写入，
 * 必须先读、合并、擦除再整块重写，否则改写现有数据会静默损坏。
 * FatFs 保证每次请求的 sector 是连续的 512B 块，且 count 通常为 1。
 */
extern "C" DRESULT w25qxx_disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv != 0 || !s_drv || !buff) return RES_PARERR;
    if (count == 0) return RES_PARERR;

    /* 验证请求不跨 4KB 边界（防御性：正常 FatFs 不会跨边界，但做检查更安全）*/
    uint32_t req_start = sector * 512;
    uint32_t req_end   = req_start + count * 512;
    if ((req_start & (W25qxx::SECTOR_SIZE - 1)) != 0) return RES_PARERR;
    if (req_end   >  req_start + W25qxx::SECTOR_SIZE) return RES_PARERR;
    if (req_end > s_drv->info().total_size)           return RES_PARERR;

    /* 计算包含该扇区的 4KB 擦除块地址 */
    uint32_t erase_base = sector & ~(W25qxx::SECTOR_SIZE / 512 - 1);
    uint32_t offset_in_block = req_start - erase_base; /* 0~4095，必 < 4096 */

    /* Step 1: 读出整个 4KB 块 */
    uint8_t full_block[W25qxx::SECTOR_SIZE];
    if (!s_drv->read(erase_base, full_block, W25qxx::SECTOR_SIZE)) {
        return RES_ERROR;
    }

    /* Step 2: 用新数据覆盖对应偏移 */
    memcpy(full_block + offset_in_block, buff, count * 512);

    /* Step 3: 擦除整个 4KB 扇区 */
    if (!s_drv->eraseSector(erase_base)) {
        return RES_ERROR;
    }

    /* Step 4: 将合并后的完整 4KB 写回 */
    if (!s_drv->write(erase_base, full_block, W25qxx::SECTOR_SIZE)) {
        return RES_ERROR;
    }

    return RES_OK;
}

extern "C" DRESULT w25qxx_disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    if (pdrv != 0 || !s_drv) return RES_PARERR;
    switch (cmd) {
        case GET_SECTOR_COUNT: *(LBA_t*)buff = s_drv->info().total_size / 512; break;
        case GET_SECTOR_SIZE:  *(WORD*)buff = 512; break;
        case GET_BLOCK_SIZE:   *(DWORD*)buff = 1; break;
        case CTRL_SYNC:        s_drv->waitBusy(); break;
        default: return RES_PARERR;
    }
    return RES_OK;
}
