/**
 * @file    bmm150.c
 * @brief   Driver implementation for Bosch BMM150 3-Axis Geomagnetic Sensor
 *
 * Compensation formulas follow Bosch BMM150 API reference (BMM150_API).
 */

#include "bmm150.h"
#include <string.h>

/* ─── Private state ──────────────────────────────────────────────────────── */

static bmm150_iface_t      s_iface    = BMM150_IFACE_I2C;
static I2C_HandleTypeDef  *s_hi2c     = NULL;
static SPI_HandleTypeDef  *s_hspi     = NULL;
static GPIO_TypeDef       *s_cs_port  = NULL;
static uint16_t            s_cs_pin   = 0;
static uint8_t             s_i2c_addr = 0;

static bmm150_trim_t s_trim;

/* DMA state */
static bmm150_dma_callback_t s_dma_callback = NULL;
static bmm150_data_t        *s_dma_data     = NULL;
/* 8 bytes: X_LSB X_MSB Y_LSB Y_MSB Z_LSB Z_MSB RHALL_LSB RHALL_MSB */
static uint8_t s_dma_raw[8];

/* ─── Private: bus helpers ────────────────────────────────────────────────── */

static void cs_assert(void)   { HAL_GPIO_WritePin(s_cs_port, s_cs_pin, GPIO_PIN_RESET); }
static void cs_deassert(void) { HAL_GPIO_WritePin(s_cs_port, s_cs_pin, GPIO_PIN_SET);   }

static HAL_StatusTypeDef reg_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (s_iface == BMM150_IFACE_I2C)
    {
        HAL_StatusTypeDef ret;
        ret = HAL_I2C_Master_Transmit(s_hi2c, s_i2c_addr, &reg, 1, 50);
        if (ret != HAL_OK) return ret;
        return HAL_I2C_Master_Receive(s_hi2c, s_i2c_addr, buf, len, 50);
    }
    else
    {
        uint8_t cmd = reg | 0x80U;
        cs_assert();
        HAL_StatusTypeDef ret = HAL_SPI_Transmit(s_hspi, &cmd, 1, 50);
        if (ret == HAL_OK) ret = HAL_SPI_Receive(s_hspi, buf, len, 50);
        cs_deassert();
        return ret;
    }
}

static HAL_StatusTypeDef reg_write(uint8_t reg, uint8_t value)
{
	if (s_iface == BMM150_IFACE_I2C)
	{
		uint8_t buf[2] = { reg, value };
		HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(s_hi2c, s_i2c_addr, buf, 2, 50);
		return ret;
	}
    else
    {
        uint8_t buf[2] = { reg & 0x7FU, value };
        cs_assert();
        HAL_StatusTypeDef ret = HAL_SPI_Transmit(s_hspi, buf, 2, 50);
        cs_deassert();
        return ret;
    }
}

/* ─── Private: trim data loading ─────────────────────────────────────────── */

static HAL_StatusTypeDef load_trim(void)
{
    uint8_t raw[2];
    HAL_StatusTypeDef ret;

#define RD1(r, dst)  do { ret = reg_read(r, &(dst), 1); if (ret != HAL_OK) return ret; } while(0)
#define RD2(r, dst)  do { ret = reg_read(r, raw,   2); if (ret != HAL_OK) return ret; \
                          (dst) = (uint16_t)((raw[1] << 8) | raw[0]); } while(0)
#define RD2S(r, dst) do { ret = reg_read(r, raw,   2); if (ret != HAL_OK) return ret; \
                          (dst) = (int16_t)((raw[1] << 8) | raw[0]);  } while(0)

    RD1(BMM150_REG_DIG_X1,       s_trim.dig_x1);
    RD1(BMM150_REG_DIG_Y1,       s_trim.dig_y1);
    RD1(BMM150_REG_DIG_X2,       s_trim.dig_x2);
    RD1(BMM150_REG_DIG_Y2,       s_trim.dig_y2);
    RD2(BMM150_REG_DIG_Z1_LSB,   s_trim.dig_z1);
    RD2S(BMM150_REG_DIG_Z2_LSB,  s_trim.dig_z2);
    RD2S(BMM150_REG_DIG_Z3_LSB,  s_trim.dig_z3);
    RD2S(BMM150_REG_DIG_Z4_LSB,  s_trim.dig_z4);
    RD2(BMM150_REG_DIG_XYZ1_LSB, s_trim.dig_xyz1);

    uint8_t xy12[2];
    ret = reg_read(BMM150_REG_DIG_XY2, xy12, 2);
    if (ret != HAL_OK) return ret;
    s_trim.dig_xy2 = (int8_t)xy12[0];
    s_trim.dig_xy1 = xy12[1];

#undef RD1
#undef RD2
#undef RD2S
    return HAL_OK;
}

/* ─── Private: compensation ──────────────────────────────────────────────── */

/**
 * @brief Compensate X-axis raw value using Bosch BMM150 API formula.
 *        Returns value in µT × 16 (divide by 16 for µT).
 */
static float compensate_x(int16_t raw_x, uint16_t raw_rhall)
{
    float retval;
    float process_comp_x0, process_comp_x1, process_comp_x2, process_comp_x3;

    if (raw_x == BMM150_OVERFLOW_ADCVAL_XY)
        return (float)BMM150_OVERFLOW_OUTPUT;

    if (raw_rhall == 0) raw_rhall = s_trim.dig_xyz1;

    process_comp_x0 = ((float)s_trim.dig_xyz1) * 16384.0f / raw_rhall;
    retval = process_comp_x0 - 16384.0f;
    process_comp_x1 = ((float)s_trim.dig_xy2) * (retval * retval / 268435456.0f);
    process_comp_x2 = process_comp_x1 + retval * ((float)s_trim.dig_xy1) / 16384.0f;
    process_comp_x3 = (float)s_trim.dig_x2 + 160.0f;
    retval = (float)raw_x * ((process_comp_x2 + 256.0f) * process_comp_x3);
    retval = (retval / 8192.0f) + (((float)s_trim.dig_x1) * 8.0f);
    return retval / 16.0f; /* µT */
}

/**
 * @brief Compensate Y-axis raw value.
 */
static float compensate_y(int16_t raw_y, uint16_t raw_rhall)
{
    float retval;
    float process_comp_y0, process_comp_y1, process_comp_y2, process_comp_y3;

    if (raw_y == BMM150_OVERFLOW_ADCVAL_XY)
        return (float)BMM150_OVERFLOW_OUTPUT;

    if (raw_rhall == 0) raw_rhall = s_trim.dig_xyz1;

    process_comp_y0 = ((float)s_trim.dig_xyz1) * 16384.0f / raw_rhall;
    retval = process_comp_y0 - 16384.0f;
    process_comp_y1 = ((float)s_trim.dig_xy2) * (retval * retval / 268435456.0f);
    process_comp_y2 = process_comp_y1 + retval * ((float)s_trim.dig_xy1) / 16384.0f;
    process_comp_y3 = (float)s_trim.dig_y2 + 160.0f;
    retval = (float)raw_y * ((process_comp_y2 + 256.0f) * process_comp_y3);
    retval = (retval / 8192.0f) + (((float)s_trim.dig_y1) * 8.0f);
    return retval / 16.0f; /* µT */
}

/**
 * @brief Compensate Z-axis raw value.
 */
static float compensate_z(int16_t raw_z, uint16_t raw_rhall)
{
    float retval;
    float process_comp_z0, process_comp_z1, process_comp_z2;

    if (raw_z == BMM150_OVERFLOW_ADCVAL_Z)
        return (float)BMM150_OVERFLOW_OUTPUT;

    if (s_trim.dig_z2 == 0 || s_trim.dig_z1 == 0 || raw_rhall == 0)
        return (float)BMM150_OVERFLOW_OUTPUT;

    process_comp_z0 = ((float)raw_rhall) - ((float)s_trim.dig_xyz1);
    process_comp_z1 = ((float)s_trim.dig_z3) * process_comp_z0;
    process_comp_z2 = ((float)s_trim.dig_z1) * raw_rhall / 32768.0f;
    retval = ((float)raw_z - ((float)s_trim.dig_z4)) * 131072.0f -
              process_comp_z1;
    retval /= ((float)s_trim.dig_z2 + process_comp_z2) * 4.0f;
    return retval / 16.0f; /* µT */
}

/**
 * @brief Decode 8 raw bytes into the data struct and apply compensation.
 */
static void decode_raw(bmm150_data_t *data, const uint8_t *raw)
{
    /* X: bits [7:3] of LSB + 8 MSB bits → signed 13-bit */
    data->raw_x = (int16_t)((raw[1] << 8) | (raw[0] & 0xF8U));
    data->raw_x >>= 3;

    /* Y: bits [7:3] of LSB + 8 MSB bits → signed 13-bit */
    data->raw_y = (int16_t)((raw[3] << 8) | (raw[2] & 0xF8U));
    data->raw_y >>= 3;

    /* Z: bits [7:1] of LSB + 8 MSB bits → signed 15-bit */
    data->raw_z = (int16_t)((raw[5] << 8) | (raw[4] & 0xFEU));
    data->raw_z >>= 1;

    /* RHALL: bits [7:2] of LSB + 8 MSB bits → unsigned 14-bit */
    data->raw_rhall = (uint16_t)((raw[7] << 8) | (raw[6] & 0xFCU));
    data->raw_rhall >>= 2;

    data->drdy = raw[6] & BMM150_RHALL_LSB_DRDY;

    data->mag_x_uT = compensate_x(data->raw_x, data->raw_rhall);
    data->mag_y_uT = compensate_y(data->raw_y, data->raw_rhall);
    data->mag_z_uT = compensate_z(data->raw_z, data->raw_rhall);
}

/* ─── Public API ─────────────────────────────────────────────────────────── */

HAL_StatusTypeDef bmm150_init(const bmm150_config_t *config)
{
    if (!config) return HAL_ERROR;

    s_iface = config->iface;

    if (s_iface == BMM150_IFACE_I2C)
    {
        if (!config->hi2c) return HAL_ERROR;
        s_hi2c     = config->hi2c;
        s_i2c_addr = config->i2c_addr;
    }
    else
    {
        if (!config->hspi || !config->cs_port) return HAL_ERROR;
        s_hspi    = config->hspi;
        s_cs_port = config->cs_port;
        s_cs_pin  = config->cs_pin;
        cs_deassert();
    }

    /* Soft reset via POWER_CTRL (also clears suspend mode) */
    HAL_StatusTypeDef ret = reg_write(BMM150_REG_POWER_CTRL, BMM150_CMD_SOFT_RESET);
    if (ret != HAL_OK) return ret;
    HAL_Delay(3); /* soft-reset duration */

    /* Set Power Control bit to enter Sleep mode (required to access all regs) */
    ret = reg_write(BMM150_REG_POWER_CTRL, BMM150_POWER_CTRL_POW_BIT);
    if (ret != HAL_OK) return ret;
    HAL_Delay(3);

    /* Verify Chip ID (only readable in sleep/active mode) */
    uint8_t chip_id = 0;
    ret = reg_read(BMM150_REG_CHIP_ID, &chip_id, 1);
    if (ret != HAL_OK) return ret;
    if (chip_id != BMM150_CHIP_ID) return HAL_ERROR;

    /* Load factory-trimmed compensation coefficients */
    ret = load_trim();
    if (ret != HAL_OK) return ret;

    /* Set repetitions for oversampling */
    ret = reg_write(BMM150_REG_REPXY, config->rep_xy);
    if (ret != HAL_OK) return ret;
    ret = reg_write(BMM150_REG_REPZ,  config->rep_z);
    if (ret != HAL_OK) return ret;

    /* Enable all axes and set operating mode */
    ret = reg_write(BMM150_REG_SENS_CTRL, 0x07U); /* channel X/Y/Z enabled */
    if (ret != HAL_OK) return ret;

    return reg_write(BMM150_REG_OP_CTRL, config->op_ctrl);
}

HAL_StatusTypeDef bmm150_read(bmm150_data_t *data)
{
    if (!data) return HAL_ERROR;

    /* Wait for data-ready in RHALL_LSB bit 0 */
    uint32_t tick = HAL_GetTick();
    while (HAL_GetTick() - tick < 200)
    {
        uint8_t rhall_lsb;
        HAL_StatusTypeDef ret = reg_read(BMM150_REG_RHALL_LSB, &rhall_lsb, 1);
        if (ret != HAL_OK) return ret;
        if (rhall_lsb & BMM150_RHALL_LSB_DRDY) break;
        HAL_Delay(1);
    }

    /* Read 8 bytes: 0x42 – 0x49 (X, Y, Z, RHALL) */
    uint8_t raw[8];
    HAL_StatusTypeDef ret = reg_read(BMM150_REG_DATA_X_LSB, raw, 8);
    if (ret != HAL_OK) return ret;

    decode_raw(data, raw);
    return HAL_OK;
}

HAL_StatusTypeDef bmm150_read_dma(bmm150_data_t        *data,
                                   bmm150_dma_callback_t callback)
{
    if (!data || !callback) return HAL_ERROR;

    s_dma_callback = callback;
    s_dma_data     = data;

    HAL_StatusTypeDef ret;

    if (s_iface == BMM150_IFACE_I2C)
    {
        uint8_t reg = BMM150_REG_DATA_X_LSB;
        ret = HAL_I2C_Master_Transmit(s_hi2c, s_i2c_addr, &reg, 1, 50);
        if (ret != HAL_OK) { s_dma_callback = NULL; s_dma_data = NULL; return ret; }
        ret = HAL_I2C_Master_Receive_DMA(s_hi2c, s_i2c_addr, s_dma_raw, 8);
    }
    else /* SPI */
    {
        uint8_t cmd = BMM150_REG_DATA_X_LSB | 0x80U;
        cs_assert();
        ret = HAL_SPI_Transmit(s_hspi, &cmd, 1, 50);
        if (ret != HAL_OK)
        {
            cs_deassert();
            s_dma_callback = NULL;
            s_dma_data     = NULL;
            return ret;
        }
        ret = HAL_SPI_Receive_DMA(s_hspi, s_dma_raw, 8);
    }

    if (ret != HAL_OK)
    {
        s_dma_callback = NULL;
        s_dma_data     = NULL;
    }
    return ret;
}

void bmm150_dma_irq_handler(void)
{
    if (s_iface == BMM150_IFACE_SPI)
        cs_deassert();

    bmm150_dma_callback_t cb   = s_dma_callback;
    bmm150_data_t        *data = s_dma_data;
    s_dma_callback = NULL;
    s_dma_data     = NULL;

    if (!cb) return;

    if (data)
        decode_raw(data, s_dma_raw);

    cb(HAL_OK);
}
