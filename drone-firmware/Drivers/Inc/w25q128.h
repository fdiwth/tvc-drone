/**
 * @file    w25q128.h
 * @brief   Driver for Winbond W25Q128JV SPI NOR Flash (128Mbit / 16MByte)
 *
 * Supported operations:
 * - Read (fast read, blocking & DMA)
 * - Page Program (blocking & DMA, 256 bytes max per call)
 * - Sector / Block / Chip erase
 * - Write-enable / write-disable helpers
 * - Status register polling
 */

#ifndef W25Q128_H
#define W25Q128_H

#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stddef.h>

/* ─── Geometry ───────────────────────────────────────────────────────────── */
#define W25Q128_PAGE_SIZE           256U          /**< Bytes per page          */
#define W25Q128_SECTOR_SIZE         4096U         /**< Bytes per 4 KB sector   */
#define W25Q128_BLOCK32_SIZE        32768U        /**< Bytes per 32 KB block   */
#define W25Q128_BLOCK64_SIZE        65536U        /**< Bytes per 64 KB block   */
#define W25Q128_TOTAL_SIZE          16777216U     /**< Total bytes (16 MB)     */

/* ─── Instruction Set ────────────────────────────────────────────────────── */
/* Write control */
#define W25Q128_CMD_WRITE_ENABLE            0x06U
#define W25Q128_CMD_WRITE_ENABLE_VSR        0x50U  /**< Volatile status reg     */
#define W25Q128_CMD_WRITE_DISABLE           0x04U

/* Status registers */
#define W25Q128_CMD_READ_SR1                0x05U
#define W25Q128_CMD_READ_SR2                0x35U
#define W25Q128_CMD_READ_SR3                0x15U
#define W25Q128_CMD_WRITE_SR1               0x01U
#define W25Q128_CMD_WRITE_SR2               0x31U
#define W25Q128_CMD_WRITE_SR3               0x11U

/* Read */
#define W25Q128_CMD_READ_DATA               0x03U
#define W25Q128_CMD_FAST_READ               0x0BU

/* Program */
#define W25Q128_CMD_PAGE_PROGRAM            0x02U

/* Erase */
#define W25Q128_CMD_SECTOR_ERASE_4K         0x20U
#define W25Q128_CMD_BLOCK_ERASE_32K         0x52U
#define W25Q128_CMD_BLOCK_ERASE_64K         0xD8U
#define W25Q128_CMD_CHIP_ERASE              0xC7U  /**< Alias: 0x60            */

/* Power */
#define W25Q128_CMD_POWER_DOWN              0xB9U
#define W25Q128_CMD_RELEASE_POWER_DOWN      0xABU

/* Device ID */
#define W25Q128_CMD_READ_JEDEC_ID           0x9FU
#define W25Q128_CMD_READ_DEVICE_ID          0x90U
#define W25Q128_CMD_READ_UNIQUE_ID          0x4BU

/* Reset */
#define W25Q128_CMD_ENABLE_RESET            0x66U
#define W25Q128_CMD_RESET_DEVICE            0x99U

/* ─── Status Register 1 Bit Masks ───────────────────────────────────────── */
#define W25Q128_SR1_BUSY                    0x01U  /**< Erase/Write in progress */
#define W25Q128_SR1_WEL                     0x02U  /**< Write enable latch      */
#define W25Q128_SR1_BP0                     0x04U
#define W25Q128_SR1_BP1                     0x08U
#define W25Q128_SR1_BP2                     0x10U
#define W25Q128_SR1_TB                      0x20U
#define W25Q128_SR1_SEC                     0x40U
#define W25Q128_SR1_SRP0                    0x80U

/* ─── JEDEC Identification ───────────────────────────────────────────────── */
#define W25Q128_MANUFACTURER_ID             0xEFU
#define W25Q128_DEVICE_ID_HIGH              0x40U
#define W25Q128_DEVICE_ID_LOW               0x18U

/* ─── Timeouts (ms) ──────────────────────────────────────────────────────── */
#define W25Q128_TIMEOUT_PAGE_PROGRAM_MS     3U
#define W25Q128_TIMEOUT_SECTOR_ERASE_MS     400U
#define W25Q128_TIMEOUT_BLOCK_ERASE_MS      2000U
#define W25Q128_TIMEOUT_CHIP_ERASE_MS       200000U
#define W25Q128_TIMEOUT_DEFAULT_MS          100U

/* ─── Types ──────────────────────────────────────────────────────────────── */

/**
 * @brief Erase granularity options for w25q128_erase().
 */
typedef enum {
    W25Q128_ERASE_SECTOR  = 0,   /**< 4 KB sector erase   */
    W25Q128_ERASE_BLOCK32 = 1,   /**< 32 KB block erase   */
    W25Q128_ERASE_BLOCK64 = 2,   /**< 64 KB block erase   */
    W25Q128_ERASE_CHIP    = 3,   /**< Full chip erase      */
} w25q128_erase_t;

/**
 * @brief Initialisation configuration.
 */
typedef struct {
    SPI_HandleTypeDef *hspi;     /**< SPI peripheral handle (required)   */
    GPIO_TypeDef      *cs_port;  /**< Chip-select GPIO port              */
    uint16_t           cs_pin;   /**< Chip-select GPIO pin               */
} w25q128_config_t;

/**
 * @brief Data structure for reads and writes.
 */
typedef struct {
    uint8_t *buf;       /**< Pointer to data buffer                      */
    uint32_t len;       /**< Number of valid bytes in buf                */
    uint32_t address;   /**< Flash address related to the operation      */
} w25q128_data_t;

/**
 * @brief DMA transfer-complete callback type.
 * @param status  HAL_OK on success, or HAL error code.
 */
typedef void (*w25q128_dma_callback_t)(HAL_StatusTypeDef status);

/* ─── Public API ─────────────────────────────────────────────────────────── */

HAL_StatusTypeDef w25q128_init(const w25q128_config_t *config);

HAL_StatusTypeDef w25q128_read(uint32_t address, w25q128_data_t *data);

HAL_StatusTypeDef w25q128_read_dma(uint32_t address,
                                    w25q128_data_t *data,
                                    w25q128_dma_callback_t callback);

HAL_StatusTypeDef w25q128_write(uint32_t       address,
                                        const uint8_t *buf,
                                        uint16_t       len);

/**
 * @brief  Non-blocking DMA write up to one page (256 bytes) to flash.
 * The target region MUST already be erased.
 * @param  address  24-bit start address.
 * @param  data     Pointer to w25q128_data_t; caller must set buf and len.
 * @param  callback Called on DMA completion. Note: The SPI transfer finishes
 * before the Flash internal write cycle. Call wait_busy() after!
 * @return HAL_OK if DMA started, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef w25q128_write_dma(uint32_t address,
                                     const w25q128_data_t *data,
                                     w25q128_dma_callback_t callback);

HAL_StatusTypeDef w25q128_erase(w25q128_erase_t type, uint32_t address);

HAL_StatusTypeDef w25q128_read_status(uint8_t *sr1, uint8_t *sr2, uint8_t *sr3);

HAL_StatusTypeDef w25q128_wait_busy(uint32_t timeout_ms);

HAL_StatusTypeDef w25q128_power_down(void);

HAL_StatusTypeDef w25q128_power_up(void);

HAL_StatusTypeDef w25q128_reset(void);

/**
 * @brief  HAL SPI DMA Tx-Rx complete callback — call from your IRQ handler
 * or HAL_SPI_TxRxCpltCallback / HAL_SPI_RxCpltCallback / HAL_SPI_TxCpltCallback.
 */
void w25q128_dma_irq_handler(void);

#endif /* W25Q128_H */
