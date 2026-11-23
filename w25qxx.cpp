/**
 * @file     w25qxx.cpp
 * @brief    W25Qxx SPI NOR Flash 完整驱动实现
 *
 * 功能：SPI 软件 CS 收发层 + 命令层 + 读写擦除 + 写保护清除 + 4 字节地址
 * 修改：2026-11-21 由 w25qxx_spi.c / w25qxx.c 合并重构为 W25qxx 类实现
 */
#include "w25qxx.h"

#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_rom_sys.h"

#include "w25qxx_diskio.h"
#include "msc_disk.h"

static const char *TAG = "w25qxx";

/* ================= SPI 收发层（软件 CS） ================= */

bool W25qxx::spiInit()
{
    spi_bus_config_t buscfg = {};
    buscfg.miso_io_num     = m_cfg.miso_pin;
    buscfg.mosi_io_num     = m_cfg.mosi_pin;
    buscfg.sclk_io_num     = m_cfg.sclk_pin;
    buscfg.quadwp_io_num   = -1;
    buscfg.quadhd_io_num   = -1;
    buscfg.max_transfer_sz = 8192;

    esp_err_t ret = spi_bus_initialize(m_cfg.spi_host, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return false;
    }

    int spi_mode = (m_cfg.spi_mode == 0 || m_cfg.spi_mode == 3) ? m_cfg.spi_mode : 3;
    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 1 * 1000 * 1000;
    devcfg.mode           = spi_mode;
    devcfg.spics_io_num   = -1; /* 软件 CS */
    devcfg.queue_size     = 7;
    ESP_LOGI(TAG, "SPI Mode: %d (CPOL=%d, CPHA=%d)", spi_mode, (spi_mode >> 1) & 1, spi_mode & 1);

    ret = spi_bus_add_device(m_cfg.spi_host, &devcfg, &m_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        spi_bus_free(m_cfg.spi_host);
        return false;
    }

    gpio_config_t cs_gpio_config = {};
    cs_gpio_config.pin_bit_mask = (1ULL << m_cfg.cs_pin);
    cs_gpio_config.mode         = GPIO_MODE_OUTPUT;
    cs_gpio_config.pull_up_en   = GPIO_PULLUP_ENABLE;
    cs_gpio_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cs_gpio_config.intr_type    = GPIO_INTR_DISABLE;
    ret = gpio_config(&cs_gpio_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CS GPIO: %s", esp_err_to_name(ret));
        spi_bus_remove_device(m_spi);
        m_spi = nullptr;
        spi_bus_free(m_cfg.spi_host);
        return false;
    }

    csDeselect();
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "SPI init (software CS): host=%d, CS=%d, SCLK=%d, MOSI=%d, MISO=%d",
             (int)m_cfg.spi_host, (int)m_cfg.cs_pin, (int)m_cfg.sclk_pin,
             (int)m_cfg.mosi_pin, (int)m_cfg.miso_pin);
    return true;
}

void W25qxx::spiDeinit()
{
    if (m_spi != nullptr) {
        spi_bus_remove_device(m_spi);
        m_spi = nullptr;
    }
    spi_bus_free(m_cfg.spi_host);
}

bool W25qxx::spiWriteThenRead(const uint8_t *txData, uint32_t txLen,
                              uint8_t *rxData, uint32_t rxLen)
{
    uint32_t total_len = txLen + rxLen;
    if (total_len == 0 || m_spi == nullptr) {
        return total_len == 0;
    }

    /* 必须同时提供 TX/RX 缓冲：全双工 DMA 模式下 rx_buffer 为 NULL 会关闭
     * RX DMA 并截断 SCLK，页编程/扇区擦除命令将发不到芯片却返回 ESP_OK。 */
    uint8_t *tx_buf = (uint8_t *)calloc(1, total_len);
    uint8_t *rx_buf = (uint8_t *)calloc(1, total_len);
    if (tx_buf == nullptr || rx_buf == nullptr) {
        free(tx_buf);
        free(rx_buf);
        ESP_LOGE(TAG, "Failed to allocate memory for SPI transaction");
        return false;
    }
    if (txLen > 0 && txData != nullptr) {
        memcpy(tx_buf, txData, txLen);
    }

    spi_transaction_t t = {};
    t.length    = total_len * 8;
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;

    csSelect();
    esp_rom_delay_us(1);
    esp_err_t ret = spi_device_transmit(m_spi, &t);
    esp_rom_delay_us(1);
    csDeselect();

    if (ret == ESP_OK && rxLen > 0 && rxData != nullptr) {
        memcpy(rxData, rx_buf + txLen, rxLen);
    }
    free(tx_buf);
    free(rx_buf);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transaction failed: %s", esp_err_to_name(ret));
        return false;
    }
    return true;
}

bool W25qxx::loopbackTest()
{
    uint8_t test_data[8] = {0x9F, 0xAA, 0x55, 0xFF, 0x00, 0x12, 0x34, 0x56};
    uint8_t rx_data[8]   = {0};

    ESP_LOGI(TAG, "=== SPI Loopback Test ===");
    ESP_LOGI(TAG, "TX: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
             test_data[0], test_data[1], test_data[2], test_data[3],
             test_data[4], test_data[5], test_data[6], test_data[7]);

    spi_transaction_t t = {};
    t.length    = 8 * 8;
    t.tx_buffer = test_data;
    t.rx_buffer = rx_data;

    csSelect();
    esp_err_t ret = spi_device_transmit(m_spi, &t);
    csDeselect();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmit failed: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "RX: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
             rx_data[0], rx_data[1], rx_data[2], rx_data[3],
             rx_data[4], rx_data[5], rx_data[6], rx_data[7]);

    bool match = (memcmp(rx_data, test_data, 8) == 0);
    if (match) {
        ESP_LOGI(TAG, "Loopback test PASSED! MOSI<->MISO connected.");
    } else {
        ESP_LOGW(TAG, "Loopback test FAILED! MOSI<->MISO NOT connected.");
        ESP_LOGW(TAG, "Possible causes:");
        ESP_LOGW(TAG, "  - MOSI/MISO wires swapped");
        ESP_LOGW(TAG, "  - Bad connection");
        ESP_LOGW(TAG, "  - Wrong GPIO pins configured");
    }
    return match;
}

/* ================= 命令辅助 ================= */

bool W25qxx::writeCommand(uint8_t cmd)
{
    return spiWriteThenRead(&cmd, 1, nullptr, 0);
}

bool W25qxx::writeCommandAddr(uint8_t cmd, uint32_t addr)
{
    uint8_t tx_data[5] = {cmd};
    if (m_info.addr_4byte) {
        tx_data[1] = (addr >> 24) & 0xFF;
        tx_data[2] = (addr >> 16) & 0xFF;
        tx_data[3] = (addr >> 8) & 0xFF;
        tx_data[4] = addr & 0xFF;
        return spiWriteThenRead(tx_data, 5, nullptr, 0);
    }

    tx_data[1] = (addr >> 16) & 0xFF;
    tx_data[2] = (addr >> 8) & 0xFF;
    tx_data[3] = addr & 0xFF;
    return spiWriteThenRead(tx_data, 4, nullptr, 0);
}

bool W25qxx::writeCommandAddrData(uint8_t cmd, uint32_t addr,
                                  const uint8_t *data, uint32_t len)
{
    uint32_t addr_len = m_info.addr_4byte ? 4 : 3;
    uint32_t tx_len   = 1 + addr_len + len;
    uint8_t *tx_data  = (uint8_t *)malloc(tx_len);
    if (tx_data == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate memory for SPI write command");
        return false;
    }

    tx_data[0] = cmd;
    if (m_info.addr_4byte) {
        tx_data[1] = (addr >> 24) & 0xFF;
        tx_data[2] = (addr >> 16) & 0xFF;
        tx_data[3] = (addr >> 8) & 0xFF;
        tx_data[4] = addr & 0xFF;
    } else {
        tx_data[1] = (addr >> 16) & 0xFF;
        tx_data[2] = (addr >> 8) & 0xFF;
        tx_data[3] = addr & 0xFF;
    }
    memcpy(tx_data + 1 + addr_len, data, len);

    bool ok = spiWriteThenRead(tx_data, tx_len, nullptr, 0);
    free(tx_data);
    return ok;
}

bool W25qxx::readStatusReg(uint8_t *status)
{
    const uint8_t cmd = CMD_READ_STATUS_REG;
    return spiWriteThenRead(&cmd, 1, status, 1);
}

bool W25qxx::writeStatusReg(uint8_t val)
{
    if (!writeEnable()) {
        return false;
    }

    uint8_t tx_data[2] = {CMD_WRITE_STATUS_REG, val};
    if (!spiWriteThenRead(tx_data, sizeof(tx_data), nullptr, 0)) {
        return false;
    }
    return waitBusyTimeout(PAGE_PROGRAM_TIMEOUT_MS);
}

void W25qxx::clearWriteProtection()
{
    uint8_t sr;
    if (!readStatusReg(&sr)) {
        ESP_LOGE(TAG, "无法读取状态寄存器，跳过写保护检查");
        return;
    }

    ESP_LOGI(TAG, "状态寄存器: 0x%02X (SRP=%d TB=%d BP2=%d BP1=%d BP0=%d WEL=%d WIP=%d)",
             sr, (sr >> 6) & 1, (sr >> 5) & 1, (sr >> 4) & 1,
             (sr >> 3) & 1, (sr >> 2) & 1, (sr >> 1) & 1, sr & 1);

    if (sr & 0x7C) {
        ESP_LOGW(TAG, "写保护已启用，正在清除...");
        if (!writeStatusReg(0x00) || !readStatusReg(&sr)) {
            ESP_LOGE(TAG, "写保护清除失败：无法更新或读取状态寄存器");
            return;
        }
        ESP_LOGI(TAG, "清除后状态寄存器: 0x%02X", sr);
        if (sr & 0x7C) {
            ESP_LOGE(TAG, "写保护清除失败！SRP位可能锁定了状态寄存器");
        } else {
            ESP_LOGI(TAG, "写保护已成功清除");
        }
    } else {
        ESP_LOGI(TAG, "无写保护，可正常写入");
    }
}

bool W25qxx::isBusy()
{
    uint8_t status;
    return readStatusReg(&status) && (status & 0x01) != 0;
}

bool W25qxx::waitBusyTimeout(uint32_t timeoutMs)
{
    TickType_t start = xTaskGetTickCount();
    while (isBusy()) {
        if ((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(timeoutMs)) {
            ESP_LOGE(TAG, "Flash operation timed out after %" PRIu32 " ms", timeoutMs);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return true;
}

void W25qxx::waitBusy()
{
    waitBusyTimeout(CHIP_ERASE_TIMEOUT_MS);
}

/* ================= 初始化 / 识别 ================= */

bool W25qxx::init(const W25qxxConfig &cfg)
{
    m_cfg = cfg;

    if (!spiInit()) {
        return false;
    }

    if (!readInfo(m_info)) {
        ESP_LOGE(TAG, "Failed to read W25QXX info");
        deinit();
        return false;
    }

    clearWriteProtection();

    if (m_info.model >= W25qxxModel::W25Q512) {
        ESP_LOGI(TAG, "Enabling 4-byte address mode");
        if (!writeCommand(CMD_ENTER_4BYTE_MODE)) {
            ESP_LOGE(TAG, "Failed to enter 4-byte address mode");
            deinit();
            return false;
        }
        m_info.addr_4byte = true;
    }

    /* 根据功能标志位挂接磁盘层 */
    if (m_cfg.feature_flags & W25QXX_FEATURE_FATFS) {
        ESP_LOGI(TAG, "Initializing FatFs disk I/O");
        w25qxx_diskio_attach(*this);
    }

    if (m_cfg.feature_flags & W25QXX_FEATURE_MSC) {
        ESP_LOGI(TAG, "Initializing USB MSC disk");
        msc_disk_attach(*this);
    }

    /* CLI 功能在独立组件 w25qxx_cli，由用户在 main 中启动 */
    if (m_cfg.feature_flags & W25QXX_FEATURE_CLI) {
        ESP_LOGI(TAG, "CLI feature enabled (initialize in main)");
    }

    return true;
}

void W25qxx::deinit()
{
    spiDeinit();
}

bool W25qxx::readInfo(W25qxxInfo &info)
{
    uint8_t tx_data[8] = {CMD_JEDEC_ID, 0, 0, 0, 0, 0, 0, 0};
    uint8_t rx_data[8] = {0};

    spi_transaction_t t = {};
    t.length    = 8 * 8;
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;

    esp_err_t ret = ESP_FAIL;
    int retry_count = 0;
    const int max_retries = 5;

    do {
        memset(rx_data, 0, sizeof(rx_data));
        csDeselect();
        vTaskDelay(pdMS_TO_TICKS(10));

        csSelect();
        ret = spi_device_transmit(m_spi, &t);
        csDeselect();

        retry_count++;
        if (ret == ESP_OK && rx_data[1] != 0x00 && rx_data[1] != 0xFF) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    } while (retry_count < max_retries);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read JEDEC ID failed: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "SPI transmit result: %s, retries=%d", esp_err_to_name(ret), retry_count);
    ESP_LOGI(TAG, "JEDEC ID raw: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
             rx_data[0], rx_data[1], rx_data[2], rx_data[3],
             rx_data[4], rx_data[5], rx_data[6], rx_data[7]);

    /* SPI 全双工：发送命令时 W25QXX 还未响应，数据从第 2 个字节开始。
     * JEDEC ID 格式：制造商ID(1) + 设备类型(1) + 容量代码(1)。
     * 发送 0x9F 时芯片还未响应(dummy)，发送第 2 字节时返回制造商 ID。 */
    info.manufacturer_id = rx_data[1];
    info.model           = (W25qxxModel)rx_data[3];

    switch (info.model) {
        case W25qxxModel::W25Q16:   info.total_size = 2 * 1024 * 1024;   break;
        case W25qxxModel::W25Q32:   info.total_size = 4 * 1024 * 1024;   break;
        case W25qxxModel::W25Q64:   info.total_size = 8 * 1024 * 1024;   break;
        case W25qxxModel::W25Q128:  info.total_size = 16 * 1024 * 1024;  break;
        case W25qxxModel::W25Q256:  info.total_size = 32 * 1024 * 1024;  break;
        case W25qxxModel::W25Q512:  info.total_size = 64 * 1024 * 1024;  break;
        case W25qxxModel::W25Q1024: info.total_size = 128 * 1024 * 1024; break;
        default:
            ESP_LOGE(TAG, "Unknown flash model: 0x%02X", (int)info.model);
            return false;
    }

    info.sector_count = info.total_size / SECTOR_SIZE;
    info.block_count  = info.total_size / BLOCK_SIZE;
    info.addr_4byte   = (info.model >= W25qxxModel::W25Q512);

    ESP_LOGI(TAG, "W25QXX Info: Manufacturer=0x%02X, Model=0x%02X, Size=%" PRIu32 " MB, Sectors=%" PRIu32 ", 4-byte-addr=%s",
             info.manufacturer_id, (int)info.model, info.total_size / (1024 * 1024),
             info.sector_count, info.addr_4byte ? "yes" : "no");

    return info.manufacturer_id == MANUFACTURER_ID;
}

/* ================= 读 / 写 / 擦除 ================= */

bool W25qxx::read(uint32_t addr, uint8_t *data, uint32_t len)
{
    if (data == nullptr || len == 0 ||
        addr > m_info.total_size || len > m_info.total_size - addr) {
        return false;
    }

    uint8_t cmd = m_info.addr_4byte ? CMD_READ_DATA_4BYTE : CMD_READ_DATA;
    uint32_t addr_len = m_info.addr_4byte ? 4 : 3;
    uint32_t tx_len   = 1 + addr_len;
    uint8_t tx_data[5] = {cmd};
    if (m_info.addr_4byte) {
        tx_data[1] = (addr >> 24) & 0xFF;
        tx_data[2] = (addr >> 16) & 0xFF;
        tx_data[3] = (addr >> 8) & 0xFF;
        tx_data[4] = addr & 0xFF;
    } else {
        tx_data[1] = (addr >> 16) & 0xFF;
        tx_data[2] = (addr >> 8) & 0xFF;
        tx_data[3] = addr & 0xFF;
    }

    return spiWriteThenRead(tx_data, tx_len, data, len);
}

bool W25qxx::write(uint32_t addr, const uint8_t *data, uint32_t len)
{
    if (data == nullptr || len == 0 ||
        addr > m_info.total_size || len > m_info.total_size - addr) {
        return false;
    }

    uint32_t bytes_written = 0;
    uint8_t cmd = m_info.addr_4byte ? CMD_PAGE_PROGRAM_4BYTE : CMD_PAGE_PROGRAM;

    while (bytes_written < len) {
        uint32_t offset = addr & (PAGE_SIZE - 1);
        uint32_t bytes_to_write = PAGE_SIZE - offset;
        if (bytes_to_write > len - bytes_written) {
            bytes_to_write = len - bytes_written;
        }

        if (!writeEnable()) {
            ESP_LOGE(TAG, "Write enable command failed");
            return false;
        }

        if (!writeCommandAddrData(cmd, addr, &data[bytes_written], bytes_to_write)) {
            ESP_LOGE(TAG, "Page program command failed at address 0x%08" PRIX32, addr);
            return false;
        }

        /* 状态诊断日志降级为 ESP_LOGD：生产环境串口输出会严重拖慢页编程 */
        uint8_t sr_after_pp = 0;
        readStatusReg(&sr_after_pp);
        ESP_LOGD(TAG, "[DEBUG] SR after PageProgram: 0x%02X (WIP=%d WEL=%d)",
                 sr_after_pp, sr_after_pp & 0x01, (sr_after_pp >> 1) & 1);

        if (!waitBusyTimeout(PAGE_PROGRAM_TIMEOUT_MS)) {
            ESP_LOGE(TAG, "Page program failed at address 0x%08" PRIX32, addr);
            return false;
        }

        uint8_t sr_after_wait = 0;
        readStatusReg(&sr_after_wait);
        ESP_LOGD(TAG, "[DEBUG] SR after wait_busy: 0x%02X (WIP=%d WEL=%d)",
                 sr_after_wait, sr_after_wait & 0x01, (sr_after_wait >> 1) & 1);

        bytes_written += bytes_to_write;
        addr          += bytes_to_write;
    }

    return true;
}

bool W25qxx::eraseSector(uint32_t addr)
{
    if (addr >= m_info.total_size) return false;

    /* 校验 4KB 对齐：不对齐的擦除会破坏相邻扇区，属于编程错误 */
    if (addr % SECTOR_SIZE != 0) {
        ESP_LOGE(TAG, "erase_sector: addr 0x%08" PRIX32 " 未按 %" PRIu32 " 字节对齐",
                 addr, SECTOR_SIZE);
        return false;
    }

    /* 等待上一次操作完成（与官方参考实现一致） */
    if (!waitBusyTimeout(SECTOR_ERASE_TIMEOUT_MS)) {
        ESP_LOGE(TAG, "Previous operation still in progress before sector erase");
        return false;
    }

    uint8_t cmd = m_info.addr_4byte ? CMD_SECTOR_ERASE_4BYTE : CMD_SECTOR_ERASE;
    if (!writeEnable()) {
        ESP_LOGE(TAG, "Write enable failed before sector erase at 0x%08" PRIX32, addr);
        return false;
    }

    /* 校验 WEL 位已置位（官方参考实现的健壮性做法） */
    uint8_t sr;
    if (!readStatusReg(&sr) || !(sr & 0x02)) {
        ESP_LOGE(TAG, "WEL not set after WriteEnable for sector erase! SR=0x%02X", sr);
        return false;
    }

    if (!writeCommandAddr(cmd, addr)) {
        ESP_LOGE(TAG, "Sector erase command failed at 0x%08" PRIX32, addr);
        return false;
    }
    return waitBusyTimeout(SECTOR_ERASE_TIMEOUT_MS);
}

bool W25qxx::eraseBlock(uint32_t addr)
{
    if (addr >= m_info.total_size) return false;

    /* 校验 64KB 对齐 */
    if (addr % BLOCK_SIZE != 0) {
        ESP_LOGE(TAG, "erase_block: addr 0x%08" PRIX32 " 未按 %" PRIu32 " 字节对齐",
                 addr, BLOCK_SIZE);
        return false;
    }

    if (!waitBusyTimeout(BLOCK_ERASE_TIMEOUT_MS)) {
        ESP_LOGE(TAG, "Previous operation still in progress before block erase");
        return false;
    }

    uint8_t cmd = m_info.addr_4byte ? CMD_BLOCK_ERASE_64K_4BYTE : CMD_BLOCK_ERASE_64K;
    if (!writeEnable()) {
        ESP_LOGE(TAG, "Write enable failed before block erase at 0x%08" PRIX32, addr);
        return false;
    }

    uint8_t sr;
    if (!readStatusReg(&sr) || !(sr & 0x02)) {
        ESP_LOGE(TAG, "WEL not set after WriteEnable for block erase! SR=0x%02X", sr);
        return false;
    }

    if (!writeCommandAddr(cmd, addr)) {
        ESP_LOGE(TAG, "Block erase command failed at 0x%08" PRIX32, addr);
        return false;
    }
    return waitBusyTimeout(BLOCK_ERASE_TIMEOUT_MS);
}

bool W25qxx::eraseChip()
{
    if (!waitBusyTimeout(CHIP_ERASE_TIMEOUT_MS)) {
        ESP_LOGE(TAG, "Previous operation still in progress before chip erase");
        return false;
    }

    if (!writeEnable()) {
        ESP_LOGE(TAG, "Write enable failed before chip erase");
        return false;
    }

    uint8_t sr;
    if (!readStatusReg(&sr) || !(sr & 0x02)) {
        ESP_LOGE(TAG, "WEL not set after WriteEnable for chip erase! SR=0x%02X", sr);
        return false;
    }

    if (!writeCommand(CMD_CHIP_ERASE)) {
        ESP_LOGE(TAG, "Chip erase command failed");
        return false;
    }
    return waitBusyTimeout(CHIP_ERASE_TIMEOUT_MS);
}

#include "w25qxx_driver_version.h"
#include "version_manager.h"
COMPONENT_VERSION_REGISTER(w25qxx_driver, W25QXX_DRIVER_VERSION_STRING)
