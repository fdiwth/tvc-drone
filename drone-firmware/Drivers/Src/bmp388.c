/**
 * @file    bmp388.c
 * @brief   Tokmas BMP388 driver.
 *
 * Fixes vs. original (3 total):
 *   1. c00 is now decoded as SIGNED 20-bit 2's complement.
 *      The datasheet (§4.6.1) explicitly states c00 and c10 are both
 *      20-bit 2's complement. The original driver treated c00 as unsigned,
 *      which corrupts the pressure baseline whenever bit 19 is set.
 *
 *   2. CFG_REG now enables INT_PRS (bit 4) and INT_TMP (bit 5) so that
 *      the INT_STS flags are actually asserted after a command-mode
 *      measurement. Without these bits the polling loop in bmp388_read()
 *      always times out (300 ms) and returns stale data.
 *
 *   3. Raw pressure and temperature are sign-extended from 24-bit 2's
 *      complement to int32_t before compensation. The datasheet (§7.1, §7.2)
 *      states both values are "24 bit 2's complement". Without this, a
 *      negative raw temperature is treated as a huge positive number,
 *      producing completely wrong compensated output.
 *
 * Debug tip:
 *   Set a breakpoint after bmp388_init() and inspect the global array
 *   bmp388_debug_coef_raw[21] in the Watch window. Those 21 bytes are
 *   exactly what reg_read returns for registers 0x10-0x24.
 */

#include "bmp388.h"
#include <string.h>
#include <math.h>

/* ── Debug: raw coef bytes from the last init — inspect in debugger ─────── */
uint8_t bmp388_debug_coef_raw[21];

/* ─── Private state ──────────────────────────────────────────────────────── */

static bmp388_iface_t      s_iface     = BMP388_IFACE_SPI;
static I2C_HandleTypeDef  *s_hi2c      = NULL;
static SPI_HandleTypeDef  *s_hspi      = NULL;
static GPIO_TypeDef       *s_cs_port   = NULL;
static uint16_t            s_cs_pin    = 0;
static uint8_t             s_i2c_addr  = 0;
static uint8_t             s_mode      = BMP388_MODE_NORMAL;
static float               s_sea_level = BMP388_SEA_LEVEL_HPA;

static double s_kt = BMP388_K_OSR_x1;
static double s_kp = BMP388_K_OSR_x1;

static bmp388_calib_t s_calib;

/* DMA state */
static bmp388_dma_callback_t s_dma_cb   = NULL;
static bmp388_data_t        *s_dma_data = NULL;
static uint8_t               s_dma_raw[6];

/* ─── CS helpers ─────────────────────────────────────────────────────────── */
static inline void cs_lo(void) { HAL_GPIO_WritePin(s_cs_port, s_cs_pin, GPIO_PIN_RESET); }
static inline void cs_hi(void) { HAL_GPIO_WritePin(s_cs_port, s_cs_pin, GPIO_PIN_SET);   }

/* ─── Bus I/O ─────────────────────────────────────────────────────────────── */

static HAL_StatusTypeDef reg_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (s_iface == BMP388_IFACE_SPI)
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
    if (s_iface == BMP388_IFACE_SPI)
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

/* ─── Scale factor lookup ────────────────────────────────────────────────── */

static double osr_to_scale(uint8_t osr)
{
    switch (osr & 0x07U)
    {
        case BMP388_OSR_x1:   return BMP388_K_OSR_x1;
        case BMP388_OSR_x2:   return BMP388_K_OSR_x2;
        case BMP388_OSR_x4:   return BMP388_K_OSR_x4;
        case BMP388_OSR_x8:   return BMP388_K_OSR_x8;
        case BMP388_OSR_x16:  return BMP388_K_OSR_x16;
        case BMP388_OSR_x32:  return BMP388_K_OSR_x32;
        case BMP388_OSR_x64:  return BMP388_K_OSR_x64;
        case BMP388_OSR_x128: return BMP388_K_OSR_x128;
        default:              return BMP388_K_OSR_x1;
    }
}

/* ─── Sign-extension helpers ─────────────────────────────────────────────── */

static inline int32_t sign_ext12(uint32_t v)
{
    return (v & 0x0800U) ? (int32_t)(v | 0xFFFFF000UL) : (int32_t)v;
}

static inline int32_t sign_ext20(uint32_t v)
{
    return (v & 0x00080000UL) ? (int32_t)(v | 0xFFF00000UL) : (int32_t)v;
}

/* ─── Calibration loading ────────────────────────────────────────────────── */

static HAL_StatusTypeDef load_calibration(void)
{
    uint8_t r[21];
    HAL_StatusTypeDef ret = reg_read(BMP388_REG_COEF_START, r, 21);
    if (ret != HAL_OK) return ret;

    /* Save for debugger inspection */
    memcpy(bmp388_debug_coef_raw, r, 21);

    /* Temperature: 12-bit signed */
    s_calib.c0  = (double)sign_ext12(((uint32_t)r[0] << 4) | (r[1] >> 4));
    s_calib.c1  = (double)sign_ext12(((uint32_t)(r[1] & 0x0FU) << 8) | r[2]);

    /*
     * FIX #1: c00 is SIGNED 20-bit 2's complement.
     * The datasheet (§4.6.1) explicitly states:
     *   "The coefficients c00 and c10 are 20 bit 2's complement numbers."
     * The original driver treated c00 as unsigned, which is incorrect and
     * corrupts the pressure baseline whenever bit 19 of c00 is set.
     */
    s_calib.c00 = (double)sign_ext20(
        ((uint32_t)r[3] << 12) | ((uint32_t)r[4] << 4) | (r[5] >> 4));

    /* c10: 20-bit SIGNED */
    s_calib.c10 = (double)sign_ext20(
        ((uint32_t)(r[5] & 0x0FU) << 16) | ((uint32_t)r[6] << 8) | r[7]);

    /* 16-bit signed */
    s_calib.c01 = (double)(int16_t)(((uint16_t)r[8]  << 8) | r[9]);
    s_calib.c11 = (double)(int16_t)(((uint16_t)r[10] << 8) | r[11]);
    s_calib.c20 = (double)(int16_t)(((uint16_t)r[12] << 8) | r[13]);
    s_calib.c21 = (double)(int16_t)(((uint16_t)r[14] << 8) | r[15]);
    s_calib.c30 = (double)(int16_t)(((uint16_t)r[16] << 8) | r[17]);

    /* 12-bit signed */
    s_calib.c31 = (double)sign_ext12(((uint32_t)r[18] << 4) | (r[19] >> 4));
    s_calib.c40 = (double)sign_ext12(((uint32_t)(r[19] & 0x0FU) << 8) | r[20]);

    return HAL_OK;
}

/* ─── Compensation ───────────────────────────────────────────────────────── */

static void compensate(bmp388_data_t *out, int32_t raw_p, int32_t raw_t)
{
    out->raw_pressure    = (uint32_t)raw_p;
    out->raw_temperature = (uint32_t)raw_t;

    double Tsc = (double)raw_t / s_kt;
    double Psc = (double)raw_p / s_kp;

    out->temperature_C = (float)(s_calib.c0 * 0.5 + s_calib.c1 * Tsc);

    double Psc2 = Psc  * Psc;
    double Psc3 = Psc2 * Psc;
    double Psc4 = Psc3 * Psc;

    double P = s_calib.c00
             + s_calib.c10 * Psc
             + s_calib.c20 * Psc2
             + s_calib.c30 * Psc3
             + s_calib.c40 * Psc4
             + Tsc * (s_calib.c01
                    + s_calib.c11 * Psc
                    + s_calib.c21 * Psc2
                    + s_calib.c31 * Psc3);

    out->pressure_Pa = (float)P;
    out->altitude_m  = 44330.0f *
                       (1.0f - powf((float)(P / 100.0) / s_sea_level,
                                    1.0f / 5.255f));
}

static void process_raw(bmp388_data_t *out, const uint8_t *buf)
{
    /*
     * FIX #3: Raw pressure and temperature are 24-bit 2's complement (signed).
     * The datasheet §7.1 and §7.2 both state the data registers hold
     * "24 bit 2's complement" values. They must be sign-extended to int32_t
     * before being passed to compensate(), otherwise a negative raw reading
     * (common for temperature at cold conditions) is interpreted as a huge
     * positive number, producing completely wrong compensated output.
     *
     * Register layout from burst read at 0x00:
     *   buf[0] = PSR_B2 = PRS[23:16]  (MSB)
     *   buf[1] = PSR_B1 = PRS[15:8]
     *   buf[2] = PSR_B0 = PRS[7:0]    (LSB)
     *   buf[3] = TMP_B2 = TMP[23:16]  (MSB)
     *   buf[4] = TMP_B1 = TMP[15:8]
     *   buf[5] = TMP_B0 = TMP[7:0]    (LSB)
     */
    uint32_t raw_p_u = ((uint32_t)buf[0] << 16) |
                       ((uint32_t)buf[1] <<  8) | buf[2];
    uint32_t raw_t_u = ((uint32_t)buf[3] << 16) |
                       ((uint32_t)buf[4] <<  8) | buf[5];

    /* Sign-extend 24-bit to 32-bit */
    int32_t raw_p = (raw_p_u & 0x800000U) ? (int32_t)(raw_p_u | 0xFF000000UL)
                                           : (int32_t)raw_p_u;
    int32_t raw_t = (raw_t_u & 0x800000U) ? (int32_t)(raw_t_u | 0xFF000000UL)
                                           : (int32_t)raw_t_u;

    compensate(out, raw_p, raw_t);
}

/* ─── Public API ─────────────────────────────────────────────────────────── */

HAL_StatusTypeDef bmp388_init(const bmp388_config_t *config)
{
    if (!config) return HAL_ERROR;

    s_iface     = config->iface;
    s_mode      = config->mode;
    s_sea_level = (config->sea_level_hpa > 0.0f)
                  ? config->sea_level_hpa : BMP388_SEA_LEVEL_HPA;

    if (s_iface == BMP388_IFACE_SPI)
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

    s_kt = osr_to_scale(config->tmp_cfg & 0x07U);
    s_kp = osr_to_scale(config->prs_cfg & 0x07U);

    /* Soft reset */
    HAL_StatusTypeDef ret = reg_write(BMP388_REG_RESET, BMP388_SOFT_RESET_VAL);
    if (ret != HAL_OK) return ret;
    HAL_Delay(10);

    /* Verify ID */
    uint8_t id = 0;
    ret = reg_read(BMP388_REG_ID, &id, 1);
    if (ret != HAL_OK) return ret;
    if (id != BMP388_ID_RESET_VAL) return HAL_ERROR;

    /* Wait for self-init + coefficients ready */
    uint32_t tick = HAL_GetTick();
    while (HAL_GetTick() - tick < 60)
    {
        uint8_t meas;
        ret = reg_read(BMP388_REG_MEAS_CFG, &meas, 1);
        if (ret != HAL_OK) return ret;
        if ((meas & BMP388_COEF_RDY) && (meas & BMP388_SENSOR_RDY)) break;
        HAL_Delay(1);
    }

    ret = load_calibration();
    if (ret != HAL_OK) return ret;

    ret = reg_write(BMP388_REG_PRS_CFG, config->prs_cfg);
    if (ret != HAL_OK) return ret;

    ret = reg_write(BMP388_REG_TMP_CFG, config->tmp_cfg);
    if (ret != HAL_OK) return ret;

    /*
     * FIX #2: Enable INT_PRS (bit 4) and INT_TMP (bit 5) in CFG_REG.
     *
     * Without these bits the sensor never asserts the INT_STS flags, so the
     * polling loop in bmp388_read() would always time out (300 ms) when using
     * command mode (PRESS_ONCE / TEMP_ONCE) and return stale data.
     *
     * Note: these are status-flag enables only — no physical interrupt pin
     * is used here. The flags are read by software polling INT_STS (0x0A).
     * INT_STS is cleared on read, so polling is safe in normal mode too.
     */
    uint8_t cfg = BMP388_INT_PRS_EN | BMP388_INT_TMP_EN;
    if ((config->prs_cfg & 0x07U) > BMP388_OSR_x8) cfg |= BMP388_P_SHIFT;
    if ((config->tmp_cfg & 0x07U) > BMP388_OSR_x8) cfg |= BMP388_T_SHIFT;
    ret = reg_write(BMP388_REG_CFG_REG, cfg);
    if (ret != HAL_OK) return ret;

    return reg_write(BMP388_REG_MEAS_CFG, config->mode & 0x07U);
}

HAL_StatusTypeDef bmp388_read(bmp388_data_t *data)
{
    if (!data) return HAL_ERROR;
    HAL_StatusTypeDef ret;

    if (s_mode == BMP388_MODE_PRESS_ONCE ||
        s_mode == BMP388_MODE_TEMP_ONCE)
    {
        ret = reg_write(BMP388_REG_MEAS_CFG, s_mode & 0x07U);
        if (ret != HAL_OK) return ret;
        uint32_t tick = HAL_GetTick();
        while (HAL_GetTick() - tick < 300)
        {
            uint8_t sts;
            ret = reg_read(BMP388_REG_INT_STS, &sts, 1);
            if (ret != HAL_OK) return ret;
            if ((sts & (BMP388_INT_PRS | BMP388_INT_TMP)) ==
                       (BMP388_INT_PRS | BMP388_INT_TMP)) break;
            HAL_Delay(1);
        }
    }

    uint8_t raw[6];
    ret = reg_read(BMP388_REG_PSR_B2, raw, 6);
    if (ret != HAL_OK) return ret;

    process_raw(data, raw);
    return HAL_OK;
}

HAL_StatusTypeDef bmp388_read_dma(bmp388_data_t        *data,
                                   bmp388_dma_callback_t callback)
{
    if (!data || !callback) return HAL_ERROR;

    s_dma_cb   = callback;
    s_dma_data = data;

    HAL_StatusTypeDef ret;

    if (s_iface == BMP388_IFACE_SPI)
    {
        uint8_t cmd = BMP388_REG_PSR_B2 | 0x80U;
        cs_lo();
        ret = HAL_SPI_Transmit(s_hspi, &cmd, 1, 10);
        if (ret != HAL_OK) { cs_hi(); s_dma_cb = NULL; s_dma_data = NULL; return ret; }
        ret = HAL_SPI_Receive_DMA(s_hspi, s_dma_raw, 6);
        if (ret != HAL_OK) { cs_hi(); s_dma_cb = NULL; s_dma_data = NULL; }
    }
    else
    {
        uint8_t reg = BMP388_REG_PSR_B2;
        ret = HAL_I2C_Master_Transmit(s_hi2c, s_i2c_addr, &reg, 1, 50);
        if (ret != HAL_OK) { s_dma_cb = NULL; s_dma_data = NULL; return ret; }
        ret = HAL_I2C_Master_Receive_DMA(s_hi2c, s_i2c_addr, s_dma_raw, 6);
        if (ret != HAL_OK) { s_dma_cb = NULL; s_dma_data = NULL; }
    }

    return ret;
}

void bmp388_dma_irq_handler(void)
{
    if (s_iface == BMP388_IFACE_SPI) cs_hi();

    bmp388_dma_callback_t cb   = s_dma_cb;
    bmp388_data_t        *data = s_dma_data;
    s_dma_cb   = NULL;
    s_dma_data = NULL;

    if (!cb) return;
    if (data) process_raw(data, s_dma_raw);
    cb(HAL_OK);
}
