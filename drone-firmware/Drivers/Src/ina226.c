/**
 * @file    ina226.c
 * @brief   Driver implementation for TI INA226 Current/Voltage/Power Monitor
 */

#include "ina226.h"
#include <string.h>

/* ─── Private state ──────────────────────────────────────────────────────── */

static I2C_HandleTypeDef *s_hi2c      = NULL;
static uint8_t            s_i2c_addr  = 0;
static float              s_current_lsb_A = 0.0f;
static float              s_power_lsb_W   = 0.0f;

/* DMA state machine tracking */
static ina226_dma_callback_t s_dma_callback = NULL;
static ina226_data_t        *s_dma_data     = NULL;
static volatile uint8_t      s_dma_state    = 0; /* 0=Idle, 1=Shunt, 2=Bus, 3=Power, 4=Current */

/* Raw DMA receive buffer: 4 registers × 2 bytes */
static uint8_t s_dma_raw[8];

/* ─── Private helpers ─────────────────────────────────────────────────────── */

static HAL_StatusTypeDef write_reg(uint8_t reg, uint16_t value)
{
    uint8_t buf[2] = { (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    return HAL_I2C_Mem_Write(s_hi2c, s_i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, 100);
}

static HAL_StatusTypeDef read_reg(uint8_t reg, uint16_t *value)
{
    uint8_t buf[2];
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(s_hi2c, s_i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, 100);
    if (ret == HAL_OK) {
        *value = (uint16_t)((buf[0] << 8) | buf[1]);
    }
    return ret;
}

static void convert_raw(ina226_data_t *data,
                         int16_t  raw_shunt,
                         uint16_t raw_bus,
                         int16_t  raw_current,
                         uint16_t raw_power)
{
    data->raw_shunt   = raw_shunt;
    data->raw_bus     = raw_bus;
    data->raw_current = raw_current;
    data->raw_power   = raw_power;

    data->shunt_voltage_mV = (float)raw_shunt  * INA226_SHUNT_VOLTAGE_LSB_UV / 1000.0f;
    data->bus_voltage_V    = (float)raw_bus     * INA226_BUS_VOLTAGE_LSB_MV  / 1000.0f;
    data->current_A        = (float)raw_current * s_current_lsb_A;
    data->power_W          = (float)raw_power   * s_power_lsb_W;
}

/* ─── Public API ─────────────────────────────────────────────────────────── */

HAL_StatusTypeDef ina226_init(const ina226_config_t *config)
{
    if (!config || !config->hi2c) return HAL_ERROR;

    s_hi2c     = config->hi2c;
    s_i2c_addr = config->i2c_addr;

    HAL_StatusTypeDef ret = write_reg(INA226_REG_CONFIG, INA226_CFG_RESET);
    if (ret != HAL_OK) return ret;
    HAL_Delay(1);

    ret = write_reg(INA226_REG_CONFIG, config->config_reg);
    if (ret != HAL_OK) return ret;

    if (config->current_lsb_A > 0.0f && config->r_shunt_ohm > 0.0f)
    {
        s_current_lsb_A = config->current_lsb_A;
        s_power_lsb_W   = 25.0f * config->current_lsb_A;

        uint16_t cal = (uint16_t)(0.00512f / (config->current_lsb_A * config->r_shunt_ohm));
        ret = write_reg(INA226_REG_CALIBRATION, cal);
        if (ret != HAL_OK) return ret;
    }

    return HAL_OK;
}

HAL_StatusTypeDef ina226_read(ina226_data_t *data)
{
    if (!data) return HAL_ERROR;

    uint16_t raw[4];
    HAL_StatusTypeDef ret;

    ret = read_reg(INA226_REG_SHUNT_VOLTAGE, &raw[0]); if (ret != HAL_OK) return ret;
    ret = read_reg(INA226_REG_BUS_VOLTAGE,   &raw[1]); if (ret != HAL_OK) return ret;
    ret = read_reg(INA226_REG_POWER,         &raw[2]); if (ret != HAL_OK) return ret;
    ret = read_reg(INA226_REG_CURRENT,       &raw[3]); if (ret != HAL_OK) return ret;

    convert_raw(data, (int16_t)raw[0], raw[1], (int16_t)raw[3], raw[2]);
    return HAL_OK;
}

HAL_StatusTypeDef ina226_read_dma(ina226_data_t *data, ina226_dma_callback_t callback)
{
    if (!data || !callback) return HAL_ERROR;
    if (s_dma_state != 0) return HAL_BUSY; /* Prevent overlapping DMA requests */

    s_dma_callback = callback;
    s_dma_data     = data;
    s_dma_state    = 1; /* Begin state machine */

    /* Step 1: Read Shunt Voltage (Mem_Read automatically writes the pointer first) */
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read_DMA(s_hi2c, s_i2c_addr, INA226_REG_SHUNT_VOLTAGE,
                                                 I2C_MEMADD_SIZE_8BIT, &s_dma_raw[0], 2);
    if (ret != HAL_OK)
    {
        s_dma_state    = 0;
        s_dma_callback = NULL;
        s_dma_data     = NULL;
    }

    return ret;
}

void ina226_dma_irq_handler(void)
{
    HAL_StatusTypeDef ret = HAL_OK;

    if (s_dma_state == 1)
    {
        ret = HAL_I2C_Mem_Read_DMA(s_hi2c, s_i2c_addr, INA226_REG_BUS_VOLTAGE,
                                    I2C_MEMADD_SIZE_8BIT, &s_dma_raw[2], 2);
        if (ret == HAL_OK) s_dma_state = 2;
    }
    else if (s_dma_state == 2)
    {
        ret = HAL_I2C_Mem_Read_DMA(s_hi2c, s_i2c_addr, INA226_REG_POWER,
                                    I2C_MEMADD_SIZE_8BIT, &s_dma_raw[4], 2);
        if (ret == HAL_OK) s_dma_state = 3;
    }
    else if (s_dma_state == 3)
    {
        ret = HAL_I2C_Mem_Read_DMA(s_hi2c, s_i2c_addr, INA226_REG_CURRENT,
                                    I2C_MEMADD_SIZE_8BIT, &s_dma_raw[6], 2);
        if (ret == HAL_OK) s_dma_state = 4;
    }
    else if (s_dma_state == 4)
    {
        s_dma_state = 0;
        ina226_dma_callback_t cb   = s_dma_callback;
        ina226_data_t        *data = s_dma_data;
        s_dma_callback = NULL;
        s_dma_data     = NULL;
        if (cb && data) {
            int16_t  raw_shunt   = (int16_t)((s_dma_raw[0] << 8) | s_dma_raw[1]);
            uint16_t raw_bus     = (uint16_t)((s_dma_raw[2] << 8) | s_dma_raw[3]);
            uint16_t raw_power   = (uint16_t)((s_dma_raw[4] << 8) | s_dma_raw[5]);
            int16_t  raw_current = (int16_t)((s_dma_raw[6] << 8) | s_dma_raw[7]);
            convert_raw(data, raw_shunt, raw_bus, raw_current, raw_power);
            cb(HAL_OK);
        }
        return;
    }

    if (ret != HAL_OK)
    {
        /* A chain step failed to start — recover instead of wedging forever. */
        HAL_I2C_Master_Abort_IT(s_hi2c, s_i2c_addr);
        s_dma_state    = 0;
        s_dma_callback = NULL;
        s_dma_data     = NULL;
    }
}

HAL_StatusTypeDef ina226_set_calibration(uint16_t cal_value)
{
    return write_reg(INA226_REG_CALIBRATION, cal_value);
}

HAL_StatusTypeDef ina226_set_config(uint16_t cfg)
{
    return write_reg(INA226_REG_CONFIG, cfg);
}

HAL_StatusTypeDef ina226_wait_conversion_ready(uint32_t timeout_ms)
{
    uint32_t tick = HAL_GetTick();
    while (HAL_GetTick() - tick < timeout_ms)
    {
        uint16_t mask_en;
        HAL_StatusTypeDef ret = read_reg(INA226_REG_MASK_ENABLE, &mask_en);
        if (ret != HAL_OK) return ret;
        if (mask_en & INA226_MASK_CVRF) return HAL_OK;
        HAL_Delay(1);
    }
    return HAL_TIMEOUT;
}
