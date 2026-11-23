/**
 * @file     w25qxx_diskio.h
 * @brief    W25Qxx FatFs 磁盘 I/O 胶水层
 *
 * 功能：把 W25qxx 类挂接到 FatFs 的 disk_* 接口（512B 扇区抽象）
 * 修改：2026-11-21 由 C 配置指针方式改为 w25qxx_diskio_attach(W25qxx&) 挂接
 *
 * @note disk_* 函数由 FatFs（C 代码）回调，必须保持 extern "C" 链接。
 */
#ifndef DISKIO_W25QXX_H
#define DISKIO_W25QXX_H

#include "ff.h"
#include "diskio.h"
#include "w25qxx.h"

#ifdef __cplusplus
extern "C" {
#endif

DSTATUS w25qxx_disk_initialize(BYTE pdrv);
DSTATUS w25qxx_disk_status(BYTE pdrv);
DRESULT w25qxx_disk_read(BYTE pdrv, BYTE* buff, LBA_t sector, UINT count);
DRESULT w25qxx_disk_write(BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count);
DRESULT w25qxx_disk_ioctl(BYTE pdrv, BYTE cmd, void *buff);

#ifdef __cplusplus
}

/** 挂接 W25qxx 驱动实例（W25qxx::init 在 FATFS 功能开关打开时自动调用） */
void w25qxx_diskio_attach(W25qxx &drv);
#endif

#endif /* DISKIO_W25QXX_H */
