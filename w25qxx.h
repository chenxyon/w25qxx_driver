/**
 * @file     w25qxx.h
 * @brief    W25Qxx SPI NOR Flash 完整驱动 — C++ class
 *
 * 功能：SPI 软件 CS 收发层、JEDEC 自动识别（W25Q16~W25Q1024）、
 *       3/4 字节地址模式、读/页编程写/扇区擦/块擦/整片擦、写保护清除，
 *       并可选挂接 FatFs 磁盘 I/O 与 USB MSC（见 w25qxx_diskio.h / msc_disk.h）
 * 修改：2026-11-21 由 C API（w25qxx.c + w25qxx_spi.c）重构为 W25qxx 类；
 *       接线与功能开关经 W25qxxConfig 结构体一次性注入
 */
#ifndef W25QXX_H
#define W25QXX_H

#include <stdint.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"

/** 芯片型号（JEDEC ID 容量代码） */
enum class W25qxxModel : uint8_t {
    W25Q16   = 0x15,
    W25Q32   = 0x16,
    W25Q64   = 0x17,
    W25Q128  = 0x18,
    W25Q256  = 0x19,
    W25Q512  = 0x20,
    W25Q1024 = 0x21,
};

/** 芯片信息（readInfo 成功后填充） */
struct W25qxxInfo {
    uint32_t    total_size      = 0;   /*!< 总容量（字节） */
    uint32_t    sector_count    = 0;   /*!< 4KB 扇区数 */
    uint32_t    block_count     = 0;   /*!< 64KB 块数 */
    W25qxxModel model           = W25qxxModel::W25Q16;
    uint8_t     manufacturer_id = 0;   /*!< 厂商 ID，Winbond 为 0xEF */
    bool        addr_4byte      = false; /*!< 是否使用 4 字节地址命令 */
};

/** 附加功能开关（可位或组合） */
enum W25qxxFeature : uint8_t {
    W25QXX_FEATURE_NONE  = 0x00, /*!< 无额外功能 */
    W25QXX_FEATURE_CLI   = 0x01, /*!< 启用串口命令交互（w25qxx_cli 组件，需在 main 中启动） */
    W25QXX_FEATURE_FATFS = 0x02, /*!< 启用 FatFs 文件系统磁盘 I/O */
    W25QXX_FEATURE_MSC   = 0x04, /*!< 启用 USB MSC 大容量存储 */
};

/**
 * W25Qxx 接线与功能配置（init 时一次性传入）
 */
struct W25qxxConfig {
    spi_host_device_t spi_host;  /*!< SPI 控制器 */
    gpio_num_t cs_pin;           /*!< 片选脚（软件 CS，GPIO 手动控制） */
    gpio_num_t sclk_pin;         /*!< 时钟脚 */
    gpio_num_t mosi_pin;         /*!< 数据输出脚 */
    gpio_num_t miso_pin;         /*!< 数据输入脚 */
    int spi_mode = 0;            /*!< SPI 模式：0 或 3，非法值按 3 处理 */
    uint8_t feature_flags = W25QXX_FEATURE_NONE; /*!< 功能标志位，见 W25qxxFeature */
};

/**
 * W25Qxx 驱动类
 *
 * 功能：一个实例对应一片 Flash；init 完成 SPI 初始化与型号识别后，
 *       即可进行裸读写/擦除，或经磁盘 I/O 层挂接 FatFs / USB MSC。
 */
class W25qxx {
public:
    /* ---- 芯片参数 ---- */
    static constexpr uint32_t PAGE_SIZE       = 256;
    static constexpr uint32_t SECTOR_SIZE     = 4096;
    static constexpr uint32_t BLOCK_SIZE      = 65536;
    static constexpr uint8_t  MANUFACTURER_ID = 0xEF; /*!< Winbond */

    W25qxx() = default;

    /**
     * @brief  初始化 SPI、识别型号、清除写保护，并按功能开关挂接磁盘层
     *
     * @param cfg 接线与功能配置
     * @return true 成功（厂商 ID 校验通过）
     */
    bool init(const W25qxxConfig &cfg);

    /** 释放 SPI 设备与总线 */
    void deinit();

    /** 读取 JEDEC ID 并推导容量/扇区数/4 字节地址模式 */
    bool readInfo(W25qxxInfo &info);

    /** 读数据（0x03 / 4 字节 0x13），含越界检查 */
    bool read(uint32_t addr, uint8_t *data, uint32_t len);

    /** 写数据（页编程 0x02 / 4 字节 0x12），内部自动跨页拆分 */
    bool write(uint32_t addr, const uint8_t *data, uint32_t len);

    /** 擦除 4KB 扇区（地址须 4KB 对齐） */
    bool eraseSector(uint32_t addr);

    /** 擦除 64KB 块（地址须 64KB 对齐） */
    bool eraseBlock(uint32_t addr);

    /** 整片擦除（耗时最长约 3 分钟） */
    bool eraseChip();

    /** 查询芯片是否忙（状态寄存器 BUSY 位） */
    bool isBusy();

    /** 阻塞等待芯片空闲（按整片擦除超时上限） */
    void waitBusy();

    /** SPI 回环自检（MOSI<->MISO 短接时应通过），用于接线排查 */
    bool loopbackTest();

    /** 芯片信息访问 */
    const W25qxxInfo &info() const { return m_info; }
    W25qxxInfo &info() { return m_info; }

private:
    /* ---- W25Qxx 命令集 ---- */
    static constexpr uint8_t CMD_WRITE_ENABLE          = 0x06;
    static constexpr uint8_t CMD_READ_STATUS_REG       = 0x05;
    static constexpr uint8_t CMD_WRITE_STATUS_REG      = 0x01;
    static constexpr uint8_t CMD_READ_DATA             = 0x03;
    static constexpr uint8_t CMD_PAGE_PROGRAM          = 0x02;
    static constexpr uint8_t CMD_SECTOR_ERASE          = 0x20;
    static constexpr uint8_t CMD_BLOCK_ERASE_64K       = 0xD8;
    static constexpr uint8_t CMD_CHIP_ERASE            = 0xC7;
    static constexpr uint8_t CMD_JEDEC_ID              = 0x9F;
    static constexpr uint8_t CMD_ENTER_4BYTE_MODE      = 0xB7;
    static constexpr uint8_t CMD_READ_DATA_4BYTE       = 0x13;
    static constexpr uint8_t CMD_PAGE_PROGRAM_4BYTE    = 0x12;
    static constexpr uint8_t CMD_SECTOR_ERASE_4BYTE    = 0x21;
    static constexpr uint8_t CMD_BLOCK_ERASE_64K_4BYTE = 0xDC;

    /* ---- 超时（ms） ---- */
    static constexpr uint32_t PAGE_PROGRAM_TIMEOUT_MS = 100;    /* 页编程最坏 3ms，留余量 */
    static constexpr uint32_t SECTOR_ERASE_TIMEOUT_MS = 3000;
    static constexpr uint32_t BLOCK_ERASE_TIMEOUT_MS  = 10000;
    static constexpr uint32_t CHIP_ERASE_TIMEOUT_MS   = 180000;

    /* ---- SPI 收发层（软件 CS） ---- */
    bool spiInit();
    void spiDeinit();
    /**
     * 先写后读全双工事务。注意必须同时提供 TX/RX 缓冲：
     * 全双工 DMA 模式下 rx_buffer 为 NULL 会关闭 RX DMA 并截断 SCLK，
     * 导致页编程/扇区擦除命令根本没发到芯片却返回 ESP_OK。
     */
    bool spiWriteThenRead(const uint8_t *txData, uint32_t txLen,
                          uint8_t *rxData, uint32_t rxLen);
    void csSelect()   { gpio_set_level(m_cfg.cs_pin, 0); }
    void csDeselect() { gpio_set_level(m_cfg.cs_pin, 1); }

    /* ---- 命令辅助 ---- */
    bool writeCommand(uint8_t cmd);
    bool writeCommandAddr(uint8_t cmd, uint32_t addr);
    bool writeCommandAddrData(uint8_t cmd, uint32_t addr, const uint8_t *data, uint32_t len);
    bool readStatusReg(uint8_t *status);
    bool writeStatusReg(uint8_t val);
    bool writeEnable() { return writeCommand(CMD_WRITE_ENABLE); }
    void clearWriteProtection();
    bool waitBusyTimeout(uint32_t timeoutMs);

    W25qxxConfig        m_cfg  = {};
    spi_device_handle_t m_spi  = nullptr;
    W25qxxInfo          m_info = {};
};

#endif /* W25QXX_H */
