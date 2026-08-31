/**
 * @file    icm42688p.c
 * @brief   Tokmas ICM-42688-P driver implementation.
 *
 * Fixes vs. original driver (5 total):
 *
 *   1. Burst read start register corrected from 0x1D (FIFO data port) to
 *      0x00 (ACC_DATA_XL). The original start address read from the FIFO
 *      port and produced garbage data.
 *
 *   2. All axis byte assembly corrected from MSB-first to LSB-first.
 *      The datasheet (§9.1.1, §9.1.2) defines each axis as _L (low byte)
 *      at the lower address, _H (high byte) at the next address. The
 *      correct assembly is (high << 8) | low, not (b[n] << 8) | b[n+1].
 *
 *   3. Temperature handling corrected:
 *      - Was: 16-bit signed, (raw / 132.48) + 25 (Bosch formula).
 *      - Now: 12-bit unsigned, (TEMP_DATA - ROOM_TEMP) / 14 + 25 per
 *        datasheet §7.2 and Table 3. ROOM_TEMP is read from registers
 *        0x29 (low 8 bits) and 0x2A[3:0] (high 4 bits) during init.
 *
 *   4. FS_SEL bit position corrected from bit [7:5] to bit [6:4] for both
 *      ACC_CONFIG1 (0x21) and GYRO_CONFIG2 (0x24) per datasheet §9.1.4
 *      and §9.1.5.
 *
 *   5. CHIP_ID register address corrected from 0x75 to 0x1F and expected
 *      value corrected from 0x47 to 0xA1 (Tokmas 'K01' chip identifier,
 *      per datasheet §9.1, register 0x1F).
 */

#include "icm42688p.h"
#include <string.h>

/* ─── Private state ──────────────────────────────────────────────────────── */

static icm42688p_iface_t   s_iface     = ICM42688P_IFACE_SPI;
static I2C_HandleTypeDef  *s_hi2c      = NULL;
static SPI_HandleTypeDef  *s_hspi      = NULL;
static GPIO_TypeDef       *s_cs_port   = NULL;
static uint16_t            s_cs_pin    = 0;
static uint8_t             s_i2c_addr  = 0;

static float    s_acc_scale   = ICM42688P_ACC_SCALE_8G;
static float    s_gyro_scale  = ICM42688P_GYRO_SCALE_500DPS;
static uint16_t s_room_temp   = 0;   /* Factory-calibrated 12-bit offset     */

/* DMA state */
static icm42688p_dma_callback_t s_dma_cb   = NULL;
static icm42688p_data_t        *s_dma_data = NULL;
/*
 * 14-byte DMA buffer: ACC X/Y/Z (L,H each), GYRO X/Y/Z (L,H each),
 * TEMP_L, TEMP_H — all burst from register 0x00.
 */
static uint8_t s_dma_raw[14];

/* ─── CS helpers ─────────────────────────────────────────────────────────── */
static inline void cs_lo(void) { HAL_GPIO_WritePin(s_cs_port, s_cs_pin, GPIO_PIN_RESET); }
static inline void cs_hi(void) { HAL_GPIO_WritePin(s_cs_port, s_cs_pin, GPIO_PIN_SET);   }

/* ─── Bus I/O ─────────────────────────────────────────────────────────────── */

static HAL_StatusTypeDef reg_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (s_iface == ICM42688P_IFACE_SPI)
    {
        uint8_t cmd = reg | 0x80U;
        cs_lo();
        HAL_StatusTypeDef ret = HAL_SPI_Transmit(s_hspi, &cmd, 1, 10);
        if (ret == HAL_OK)
            ret = HAL_SPI_Receive(s_hspi, buf, len, 50);
        cs_hi();
        return ret;
    }
    else
    {
        HAL_StatusTypeDef ret =
            HAL_I2C_Master_Transmit(s_hi2c, s_i2c_addr, &reg, 1, 50);
        if (ret != HAL_OK) return ret;
        return HAL_I2C_Master_Receive(s_hi2c, s_i2c_addr, buf, len, 50);
    }
}

static HAL_StatusTypeDef reg_write(uint8_t reg, uint8_t val)
{
    if (s_iface == ICM42688P_IFACE_SPI)
    {
        uint8_t buf[2] = { reg & 0x7FU, val };
        cs_lo();
        HAL_StatusTypeDef ret = HAL_SPI_Transmit(s_hspi, buf, 2, 10);
        cs_hi();
        return ret;
    }
    else
    {
        uint8_t buf[2] = { reg, val };
        return HAL_I2C_Master_Transmit(s_hi2c, s_i2c_addr, buf, 2, 50);
    }
}

/* ─── Data decoding ──────────────────────────────────────────────────────── */

/**
 * @brief Decode 14-byte burst read starting at ACC_DATA_XL (0x00).
 *
 * FIX #1 + #2: Burst starts at 0x00, not 0x1D. All registers are LSB-first,
 * so the 16-bit value for each axis is assembled as (high << 8) | low.
 *
 * Buffer layout (LSB-first per datasheet §9.1.1 and §9.1.2):
 *   [0]  ACC_DATA_XL   [1]  ACC_DATA_XH
 *   [2]  ACC_DATA_YL   [3]  ACC_DATA_YH
 *   [4]  ACC_DATA_ZL   [5]  ACC_DATA_ZH
 *   [6]  GYRO_DATA_XL  [7]  GYRO_DATA_XH
 *   [8]  GYRO_DATA_YL  [9]  GYRO_DATA_YH
 *   [10] GYRO_DATA_ZL  [11] GYRO_DATA_ZH
 *   [12] TEMP_DATA_L   [13] TEMP_DATA_H
 *
 * FIX #3: Temperature is 12-bit unsigned. Formula per datasheet §7.2:
 *   T(°C) = (TEMP_DATA - ROOM_TEMP) / 14 + 25
 * TEMP_DATA_H[7:4] are unused; the 12-bit value is
 * (TEMP_DATA_H[3:0] << 8) | TEMP_DATA_L[7:0].
 */
static void decode_raw(icm42688p_data_t *out, const uint8_t *b)
{
    /*
     * FIX #2: Assemble LSB-first. Each axis: low byte at even index,
     * high byte at odd index.
     */
    out->raw_ax = (int16_t)(((uint16_t)b[1]  << 8) | b[0]);
    out->raw_ay = (int16_t)(((uint16_t)b[3]  << 8) | b[2]);
    out->raw_az = (int16_t)(((uint16_t)b[5]  << 8) | b[4]);
    out->raw_gx = (int16_t)(((uint16_t)b[7]  << 8) | b[6]);
    out->raw_gy = (int16_t)(((uint16_t)b[9]  << 8) | b[8]);
    out->raw_gz = (int16_t)(((uint16_t)b[11] << 8) | b[10]);

    /*
     * FIX #3: Temperature is 12-bit unsigned.
     * TEMP_DATA_L = b[12] = bits [7:0]
     * TEMP_DATA_H = b[13] = bits [11:8] in lower nibble [3:0]
     */
    out->raw_temp      = ((uint16_t)(b[13] & 0x0FU) << 8) | b[12];
    out->raw_room_temp = s_room_temp;

    out->accel_x_g  = -(float)out->raw_ax * s_acc_scale; // magnetometer and imu are oriented differently: minus sign alignes their axis
    out->accel_y_g  = (float)out->raw_ay * s_acc_scale;
    out->accel_z_g  = -(float)out->raw_az * s_acc_scale;
    out->gyro_x_dps = -(float)out->raw_gx * s_gyro_scale;
    out->gyro_y_dps = (float)out->raw_gy * s_gyro_scale;
    out->gyro_z_dps = -(float)out->raw_gz * s_gyro_scale;

    /* T(°C) = (TEMP_DATA - ROOM_TEMP) / 14 + 25  (datasheet §7.2) */
    out->temperature_C = ((float)(int32_t)(out->raw_temp - s_room_temp)
                          / 14.0f) + 25.0f;
}

/* ─── Public API ─────────────────────────────────────────────────────────── */

HAL_StatusTypeDef icm42688p_init(const icm42688p_config_t *config)
{
    if (!config) return HAL_ERROR;

    s_iface      = config->iface;
    s_acc_scale  = config->acc_scale;
    s_gyro_scale = config->gyro_scale;

    if (s_iface == ICM42688P_IFACE_SPI)
    {
        if (!config->hspi || !config->cs_port) return HAL_ERROR;
        s_hspi    = config->hspi;
        s_cs_port = config->cs_port;
        s_cs_pin  = config->cs_pin;
        cs_hi();
        HAL_Delay(2);
    }
    else
    {
        if (!config->hi2c) return HAL_ERROR;
        s_hi2c     = config->hi2c;
        s_i2c_addr = config->i2c_addr;
    }

    /* Soft reset — write 0x73 to register 0x00 */
    HAL_StatusTypeDef ret = reg_write(ICM42688P_SOFT_RESET_REG,
                                       ICM42688P_SOFT_RESET_VAL);
    if (ret != HAL_OK) return ret;
    HAL_Delay(100);

    /* Verify CHIP_ID at 0x1F, expected 0xA1 */
    uint8_t chip_id = 0;
    ret = reg_read(ICM42688P_REG_CHIP_ID, &chip_id, 1);
    if (ret != HAL_OK) return ret;
    if (chip_id != ICM42688P_CHIP_ID) return HAL_ERROR;

    /* Read factory-calibrated ROOM_TEMP offset */
    uint8_t rt[2];
    ret = reg_read(ICM42688P_REG_TEMP_CONFIG1, rt, 2);
    if (ret != HAL_OK) return ret;
    s_room_temp = ((uint16_t)(rt[1] & 0x0FU) << 8) | rt[0];

    /* Write FS_SEL to ACC_CONFIG1 (0x21) */
    ret = reg_write(ICM42688P_REG_ACC_CONFIG1, config->accel_config);
    if (ret != HAL_OK) return ret;

    /* Write FS_SEL + ODR to GYRO_CONFIG2 (0x24) */
    ret = reg_write(ICM42688P_REG_GYRO_CONFIG2, config->gyro_config);
    if (ret != HAL_OK) return ret;

    /*
     * Enable gyro hardware digital LPF (GYRO_CONFIG_1, address 0x23)
     * bit[0] = 1: enable digital filter
     * bit[1] = 0: do NOT bypass (i.e. use the filter)
     */
    ret = reg_write(0x23, 0x01);
    if (ret != HAL_OK) return ret;

    /*
     * Set gyro LPF cutoff frequency (GYRO_CONFIG_3, address 0x25)
     * bits[3:0] = cutoff factor N, cutoff = ODR * N
     * At ODR=1760Hz:
     *   0x09 = N=0.10 -> 176Hz
     *   0x0A = N=0.08 -> 141Hz  <- good balance, start here
     *   0x0B = N=0.06 -> 106Hz  <- more aggressive, try if 141Hz still noisy
     *   0x0C = N=0.04 ->  70Hz  <- heavy, only if vibration is severe
     */
    ret = reg_write(0x25, 0x0B);   /* ~141Hz cutoff at 1760Hz ODR */
    if (ret != HAL_OK) return ret;

    /* Enable normal (accel + gyro) mode */
    ret = reg_write(ICM42688P_REG_PWR_MGMT, ICM42688P_PWR_NORMAL);
    if (ret != HAL_OK) return ret;

    HAL_Delay(100);

    return HAL_OK;
}

HAL_StatusTypeDef icm42688p_read(icm42688p_data_t *data)
{
    if (!data) return HAL_ERROR;

    /*
     * FIX #1: Burst-read 14 bytes starting at ACC_DATA_XL (0x00).
     * The original driver started at 0x1D which is the FIFO data port.
     */
    uint8_t raw[14];
    HAL_StatusTypeDef ret = reg_read(ICM42688P_REG_ACC_DATA_XL, raw, 14);
    if (ret != HAL_OK) return ret;

    decode_raw(data, raw);
    return HAL_OK;
}

HAL_StatusTypeDef icm42688p_read_dma(icm42688p_data_t        *data,
                                      icm42688p_dma_callback_t callback)
{
    if (!data || !callback) return HAL_ERROR;

    s_dma_cb   = callback;
    s_dma_data = data;

    HAL_StatusTypeDef ret;

    if (s_iface == ICM42688P_IFACE_SPI)
    {
        uint8_t cmd = ICM42688P_REG_ACC_DATA_XL | 0x80U;
        cs_lo();
        ret = HAL_SPI_Transmit(s_hspi, &cmd, 1, 10);
        if (ret != HAL_OK) { cs_hi(); s_dma_cb = NULL; s_dma_data = NULL; return ret; }
        ret = HAL_SPI_Receive_DMA(s_hspi, s_dma_raw, 14);
        if (ret != HAL_OK) { cs_hi(); s_dma_cb = NULL; s_dma_data = NULL; }
    }
    else
    {
        uint8_t reg = ICM42688P_REG_ACC_DATA_XL;
        ret = HAL_I2C_Master_Transmit(s_hi2c, s_i2c_addr, &reg, 1, 50);
        if (ret != HAL_OK) { s_dma_cb = NULL; s_dma_data = NULL; return ret; }
        ret = HAL_I2C_Master_Receive_DMA(s_hi2c, s_i2c_addr, s_dma_raw, 14);
        if (ret != HAL_OK) { s_dma_cb = NULL; s_dma_data = NULL; }
    }

    return ret;
}

void icm42688p_dma_irq_handler(void)
{
    if (s_iface == ICM42688P_IFACE_SPI) cs_hi();

    icm42688p_dma_callback_t cb   = s_dma_cb;
    icm42688p_data_t        *data = s_dma_data;
    s_dma_cb   = NULL;
    s_dma_data = NULL;

    if (!cb) return;
    if (data) decode_raw(data, s_dma_raw);
    cb(HAL_OK);
}
