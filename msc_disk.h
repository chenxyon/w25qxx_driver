/**
 * @file     msc_disk.h
 * @brief    W25Qxx USB MSC（TinyUSB）磁盘胶水层
 *
 * 功能：把 W25qxx 类挂接到 TinyUSB MSC 回调（512B 扇区抽象）
 * 修改：2026-11-21 由 C 配置指针方式改为 msc_disk_attach(W25qxx&) 挂接
 *
 * @note tud_* 回调由 TinyUSB（C 代码）调用，必须保持 extern "C" 链接。
 */
#ifndef MSC_DISK_H
#define MSC_DISK_H

#include "w25qxx.h"

#ifdef __cplusplus
extern "C" {
#endif

/* MSC 磁盘操作函数 */
int32_t msc_read_sectors(uint32_t lba, uint32_t sector_count, uint8_t *buf);
int32_t msc_write_sectors(uint32_t lba, uint32_t sector_count, uint8_t *buf);
uint32_t msc_get_sector_count(void);
uint32_t msc_get_sector_size(void);
uint32_t msc_get_block_size(void);

/* TinyUSB MSC 回调函数（需要在 tinyusb 配置中启用 MSC） */
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize);
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize);
void tud_msc_write10_complete_cb(uint8_t lun);
int32_t tud_msc_get_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size);
bool tud_msc_is_writable_cb(uint8_t lun);
int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const *cbw, uint8_t *csw);

#ifdef __cplusplus
}

/** 挂接 W25qxx 驱动实例（W25qxx::init 在 MSC 功能开关打开时自动调用） */
void msc_disk_attach(W25qxx &drv);
#endif

#endif /* MSC_DISK_H */
