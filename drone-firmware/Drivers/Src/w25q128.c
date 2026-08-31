/**
 * @file    w25q128.c
 * @brief   Driver implementation for Winbond W25Q128JV SPI NOR Flash
 */

#include "w25q128.h"
#include <string.h>
#include "cmsis_os.h"

/* ─── Private state ──────────────────────────────────────────────────────── */

static SPI_HandleTypeDef  *s_hspi     = NULL;
static GPIO_TypeDef       *s_cs_port  = NULL;
static uint16_t            s_cs_pin   = 0;

/* DMA transfer state */
static w25q128_dma_callback_t s_dma_callback = NULL;

/* Scratch buffer for commands + 3-byte address (never larger than 5 bytes) */
static uint8_t s_cmd_buf[5];

/* ─── Private helpers ─────────────────────────────────────────────────────── */

static inline void cs_assert(void)
{
    HAL_GPIO_WritePin(s_cs_port, s_cs_pin, GPIO_PIN_RESET);
}

static inline void cs_deassert(void)
{
    HAL_GPIO_WritePin(s_cs_port, s_cs_pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef send_cmd(uint8_t cmd)
{
    cs_assert();
    HAL_StatusTypeDef ret = HAL_SPI_Transmit(s_hspi, &cmd, 1,
                                              W25Q128_TIMEOUT_DEFAULT_MS);
    cs_deassert();
    return ret;
}

static void build_addr_cmd(uint8_t *buf, uint8_t cmd, uint32_t address)
{
    buf[0] = cmd;
    buf[1] = (address >> 16) & 0xFF;
    buf[2] = (address >>  8) & 0xFF;
    buf[3] = (address)       & 0xFF;
}

static HAL_StatusTypeDef write_enable(void)
{
    HAL_StatusTypeDef ret = send_cmd(W25Q128_CMD_WRITE_ENABLE);
    if (ret != HAL_OK) return ret;

    uint32_t tick = HAL_GetTick();
    while (HAL_GetTick() - tick < W25Q128_TIMEOUT_DEFAULT_MS)
    {
        uint8_t sr1;
        cs_assert();
        uint8_t req = W25Q128_CMD_READ_SR1;
        ret = HAL_SPI_Transmit(s_hspi, &req, 1, W25Q128_TIMEOUT_DEFAULT_MS);
        if (ret == HAL_OK)
            ret = HAL_SPI_Receive(s_hspi, &sr1, 1, W25Q128_TIMEOUT_DEFAULT_MS);
        cs_deassert();

        if (ret != HAL_OK) return ret;
        if (sr1 & W25Q128_SR1_WEL) return HAL_OK;
    }
    return HAL_TIMEOUT;
}

/* ─── Public API ─────────────────────────────────────────────────────────── */

HAL_StatusTypeDef w25q128_init(const w25q128_config_t *config)
{
    if (!config || !config->hspi || !config->cs_port)
        return HAL_ERROR;

    s_hspi    = config->hspi;
    s_cs_port = config->cs_port;
    s_cs_pin  = config->cs_pin;

    cs_deassert();
    HAL_Delay(5);

    HAL_StatusTypeDef ret = send_cmd(W25Q128_CMD_RELEASE_POWER_DOWN);
    if (ret != HAL_OK) return ret;
    HAL_Delay(1);

    cs_assert();
    uint8_t cmd = W25Q128_CMD_READ_JEDEC_ID;
    ret = HAL_SPI_Transmit(s_hspi, &cmd, 1, W25Q128_TIMEOUT_DEFAULT_MS);
    if (ret != HAL_OK) { cs_deassert(); return ret; }

    uint8_t id[3] = {0};
    ret = HAL_SPI_Receive(s_hspi, id, 3, W25Q128_TIMEOUT_DEFAULT_MS);
    cs_deassert();
    if (ret != HAL_OK) return ret;

    if (id[0] != W25Q128_MANUFACTURER_ID ||
        id[1] != W25Q128_DEVICE_ID_HIGH  ||
        id[2] != W25Q128_DEVICE_ID_LOW)
        return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef w25q128_read(uint32_t address, w25q128_data_t *data)
{
    if (!data || !data->buf || data->len == 0) return HAL_ERROR;

    s_cmd_buf[0] = W25Q128_CMD_FAST_READ;
    s_cmd_buf[1] = (address >> 16) & 0xFF;
    s_cmd_buf[2] = (address >>  8) & 0xFF;
    s_cmd_buf[3] = (address)       & 0xFF;
    s_cmd_buf[4] = 0x00; /* dummy */

    cs_assert();
    HAL_StatusTypeDef ret = HAL_SPI_Transmit(s_hspi, s_cmd_buf, 5,
                                              W25Q128_TIMEOUT_DEFAULT_MS);
    if (ret == HAL_OK)
        ret = HAL_SPI_Receive(s_hspi, data->buf, data->len,
                              W25Q128_TIMEOUT_DEFAULT_MS);
    cs_deassert();

    if (ret == HAL_OK) data->address = address;

    return ret;
}

HAL_StatusTypeDef w25q128_read_dma(uint32_t address,
                                    w25q128_data_t *data,
                                    w25q128_dma_callback_t callback)
{
    if (!data || !data->buf || data->len == 0 || !callback) return HAL_ERROR;

    s_dma_callback = callback;
    data->address  = address;

    s_cmd_buf[0] = W25Q128_CMD_FAST_READ;
    s_cmd_buf[1] = (address >> 16) & 0xFF;
    s_cmd_buf[2] = (address >>  8) & 0xFF;
    s_cmd_buf[3] = (address)       & 0xFF;
    s_cmd_buf[4] = 0x00;

    cs_assert();
    HAL_StatusTypeDef ret = HAL_SPI_Transmit(s_hspi, s_cmd_buf, 5, W25Q128_TIMEOUT_DEFAULT_MS);
    if (ret != HAL_OK)
    {
        cs_deassert();
        s_dma_callback = NULL;
        return ret;
    }

    ret = HAL_SPI_Receive_DMA(s_hspi, data->buf, data->len);
    if (ret != HAL_OK)
    {
        cs_deassert();
        s_dma_callback = NULL;
    }

    return ret;
}

HAL_StatusTypeDef w25q128_write(uint32_t       address,
                                        const uint8_t *buf,
                                        uint16_t       len)
{
    if (!buf || len == 0 || len > W25Q128_PAGE_SIZE) return HAL_ERROR;

    HAL_StatusTypeDef ret = write_enable();
    if (ret != HAL_OK) return ret;

    build_addr_cmd(s_cmd_buf, W25Q128_CMD_PAGE_PROGRAM, address);

    cs_assert();
    ret = HAL_SPI_Transmit(s_hspi, s_cmd_buf, 4, W25Q128_TIMEOUT_DEFAULT_MS);
    if (ret == HAL_OK) {
        /* Drop const locally for the HAL prototype */
        ret = HAL_SPI_Transmit(s_hspi, (uint8_t *)buf, len, W25Q128_TIMEOUT_DEFAULT_MS);
    }
    cs_deassert();

    if (ret != HAL_OK) return ret;

    return w25q128_wait_busy(W25Q128_TIMEOUT_PAGE_PROGRAM_MS);
}

HAL_StatusTypeDef w25q128_write_dma(uint32_t address,
                                     const w25q128_data_t *data,
                                     w25q128_dma_callback_t callback)
{
    if (!data || !data->buf || data->len == 0 || data->len > W25Q128_PAGE_SIZE || !callback)
        return HAL_ERROR;

    HAL_StatusTypeDef ret = write_enable();
    if (ret != HAL_OK) return ret;

    s_dma_callback = callback;

    build_addr_cmd(s_cmd_buf, W25Q128_CMD_PAGE_PROGRAM, address);

    cs_assert();
    /* Transmit the instruction and address synchronously first */
    ret = HAL_SPI_Transmit(s_hspi, s_cmd_buf, 4, W25Q128_TIMEOUT_DEFAULT_MS);
    if (ret != HAL_OK)
    {
        cs_deassert();
        s_dma_callback = NULL;
        return ret;
    }

    /* Start DMA transmission for the payload */
    ret = HAL_SPI_Transmit_DMA(s_hspi, (uint8_t *)data->buf, data->len);
    if (ret != HAL_OK)
    {
        cs_deassert();
        s_dma_callback = NULL;
    }

    return ret;
}

HAL_StatusTypeDef w25q128_erase(w25q128_erase_t type, uint32_t address)
{
    HAL_StatusTypeDef ret = write_enable();
    if (ret != HAL_OK) return ret;

    uint8_t  cmd;
    uint32_t timeout_ms;

    switch (type)
    {
        case W25Q128_ERASE_SECTOR:
            cmd        = W25Q128_CMD_SECTOR_ERASE_4K;
            timeout_ms = W25Q128_TIMEOUT_SECTOR_ERASE_MS;
            break;
        case W25Q128_ERASE_BLOCK32:
            cmd        = W25Q128_CMD_BLOCK_ERASE_32K;
            timeout_ms = W25Q128_TIMEOUT_BLOCK_ERASE_MS;
            break;
        case W25Q128_ERASE_BLOCK64:
            cmd        = W25Q128_CMD_BLOCK_ERASE_64K;
            timeout_ms = W25Q128_TIMEOUT_BLOCK_ERASE_MS;
            break;
        case W25Q128_ERASE_CHIP:
            ret = send_cmd(W25Q128_CMD_CHIP_ERASE);
            if (ret != HAL_OK) return ret;
            return w25q128_wait_busy(W25Q128_TIMEOUT_CHIP_ERASE_MS);
        default:
            return HAL_ERROR;
    }

    build_addr_cmd(s_cmd_buf, cmd, address);
    cs_assert();
    ret = HAL_SPI_Transmit(s_hspi, s_cmd_buf, 4, W25Q128_TIMEOUT_DEFAULT_MS);
    cs_deassert();
    if (ret != HAL_OK) return ret;

    return w25q128_wait_busy(timeout_ms);
}

HAL_StatusTypeDef w25q128_read_status(uint8_t *sr1, uint8_t *sr2, uint8_t *sr3)
{
    HAL_StatusTypeDef ret;
    uint8_t cmd, val;

    /* Bugfix: Avoid bitwise OR'ing HAL_StatusTypeDef enums */
    if (sr1)
    {
        cmd = W25Q128_CMD_READ_SR1;
        cs_assert();
        ret = HAL_SPI_Transmit(s_hspi, &cmd, 1, W25Q128_TIMEOUT_DEFAULT_MS);
        if (ret == HAL_OK) {
            ret = HAL_SPI_Receive(s_hspi, &val, 1, W25Q128_TIMEOUT_DEFAULT_MS);
            if (ret == HAL_OK) *sr1 = val;
        }
        cs_deassert();
        if (ret != HAL_OK) return ret;
    }

    if (sr2)
    {
        cmd = W25Q128_CMD_READ_SR2;
        cs_assert();
        ret = HAL_SPI_Transmit(s_hspi, &cmd, 1, W25Q128_TIMEOUT_DEFAULT_MS);
        if (ret == HAL_OK) {
            ret = HAL_SPI_Receive(s_hspi, &val, 1, W25Q128_TIMEOUT_DEFAULT_MS);
            if (ret == HAL_OK) *sr2 = val;
        }
        cs_deassert();
        if (ret != HAL_OK) return ret;
    }

    if (sr3)
    {
        cmd = W25Q128_CMD_READ_SR3;
        cs_assert();
        ret = HAL_SPI_Transmit(s_hspi, &cmd, 1, W25Q128_TIMEOUT_DEFAULT_MS);
        if (ret == HAL_OK) {
            ret = HAL_SPI_Receive(s_hspi, &val, 1, W25Q128_TIMEOUT_DEFAULT_MS);
            if (ret == HAL_OK) *sr3 = val;
        }
        cs_deassert();
        if (ret != HAL_OK) return ret;
    }
    return HAL_OK;
}

/**
 * @brief 1ms yield used while polling SR1_BUSY.
 *
 * HAL_Delay() (per your TIM7-driven tick) is a busy-poll on HAL_GetTick() --
 * it does NOT let the RTOS scheduler run other tasks. That's fine before
 * osKernelStart() (w25q128_wait_busy() is called from sys_init() in main(),
 * pre-scheduler), but every other caller is a task (LoggerTask erasing
 * during ARM, LoggerTask after every page program) where busy-polling for
 * up to W25Q128_TIMEOUT_CHIP_ERASE_MS at a time starves lower-priority
 * tasks. osDelay() genuinely blocks the calling task and lets the scheduler
 * run everything else during that time.
 */
static inline void w25q128_yield_1ms(void)
{
    if (osKernelGetState() == osKernelRunning) {
        osDelay(1);
    } else {
        HAL_Delay(1);
    }
}

HAL_StatusTypeDef w25q128_wait_busy(uint32_t timeout_ms)

{
    uint32_t tick = HAL_GetTick();
    while (HAL_GetTick() - tick < timeout_ms)
    {
        uint8_t sr1;
        HAL_StatusTypeDef ret = w25q128_read_status(&sr1, NULL, NULL);
        if (ret != HAL_OK) return ret;
        if (!(sr1 & W25Q128_SR1_BUSY)) return HAL_OK;
        w25q128_yield_1ms();
    }
    return HAL_TIMEOUT;
}

HAL_StatusTypeDef w25q128_power_down(void)
{
    return send_cmd(W25Q128_CMD_POWER_DOWN);
}

HAL_StatusTypeDef w25q128_power_up(void)
{
    HAL_StatusTypeDef ret = send_cmd(W25Q128_CMD_RELEASE_POWER_DOWN);
    HAL_Delay(1);
    return ret;
}

HAL_StatusTypeDef w25q128_reset(void)
{
    HAL_StatusTypeDef ret = send_cmd(W25Q128_CMD_ENABLE_RESET);
    if (ret != HAL_OK) return ret;
    ret = send_cmd(W25Q128_CMD_RESET_DEVICE);
    HAL_Delay(1);
    return ret;
}

void w25q128_dma_irq_handler(void)
{
    cs_deassert();

    w25q128_dma_callback_t cb = s_dma_callback;
    s_dma_callback = NULL;

    if (cb) cb(HAL_OK);
}
