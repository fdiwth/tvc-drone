/**
 * @file    ina226.h
 * @brief   Driver for TI INA226 36V 16-bit I2C Current/Voltage/Power Monitor
 *
 * The INA226 measures shunt voltage, bus voltage, current, and power.
 * Current and power readings require the Calibration register to be
 * programmed, which is done via ina226_config_t.
 *
 * All reads are blocking; DMA is provided for high-frequency sampling loops.
 */

#ifndef INA226_H
#define INA226_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

/* ─── Register Addresses ─────────────────────────────────────────────────── */
#define INA226_REG_CONFIG           0x00U  /**< Configuration                 */
#define INA226_REG_SHUNT_VOLTAGE    0x01U  /**< Shunt voltage (±81.92 mV)     */
#define INA226_REG_BUS_VOLTAGE      0x02U  /**< Bus voltage  (0–36 V)         */
#define INA226_REG_POWER            0x03U  /**< Calculated power              */
#define INA226_REG_CURRENT          0x04U  /**< Calculated current            */
#define INA226_REG_CALIBRATION      0x05U  /**< Full-scale calibration        */
#define INA226_REG_MASK_ENABLE      0x06U  /**< Alert config / CVRF flag      */
#define INA226_REG_ALERT_LIMIT      0x07U  /**< Alert threshold               */
#define INA226_REG_MANUFACTURER_ID  0xFEU  /**< Should read 0x5449 ("TI")     */
#define INA226_REG_DIE_ID           0xFFU  /**< Should read 0x2260            */

/* ─── Configuration Register Bit Fields ─────────────────────────────────── */
#define INA226_CFG_RESET            (1U << 15)

/* Averaging count [11:9] */
#define INA226_AVG_1                (0x0U << 9)
#define INA226_AVG_4                (0x1U << 9)
#define INA226_AVG_16               (0x2U << 9)
#define INA226_AVG_64               (0x3U << 9)
#define INA226_AVG_128              (0x4U << 9)
#define INA226_AVG_256              (0x5U << 9)
#define INA226_AVG_512              (0x6U << 9)
#define INA226_AVG_1024             (0x7U << 9)

/* Bus voltage conversion time [8:6] */
#define INA226_VBUS_CT_140US        (0x0U << 6)
#define INA226_VBUS_CT_204US        (0x1U << 6)
#define INA226_VBUS_CT_332US        (0x2U << 6)
#define INA226_VBUS_CT_588US        (0x3U << 6)
#define INA226_VBUS_CT_1100US       (0x4U << 6)
#define INA226_VBUS_CT_2116US       (0x5U << 6)
#define INA226_VBUS_CT_4156US       (0x6U << 6)
#define INA226_VBUS_CT_8244US       (0x7U << 6)

/* Shunt voltage conversion time [5:3] */
#define INA226_VSH_CT_140US         (0x0U << 3)
#define INA226_VSH_CT_204US         (0x1U << 3)
#define INA226_VSH_CT_332US         (0x2U << 3)
#define INA226_VSH_CT_588US         (0x3U << 3)
#define INA226_VSH_CT_1100US        (0x4U << 3)
#define INA226_VSH_CT_2116US        (0x5U << 3)
#define INA226_VSH_CT_4156US        (0x6U << 3)
#define INA226_VSH_CT_8244US        (0x7U << 3)

/* Operating mode [2:0] */
#define INA226_MODE_POWER_DOWN      0x0U
#define INA226_MODE_SHUNT_TRIG      0x1U
#define INA226_MODE_BUS_TRIG        0x2U
#define INA226_MODE_SHUNT_BUS_TRIG  0x3U
#define INA226_MODE_POWER_DOWN2     0x4U
#define INA226_MODE_SHUNT_CONT      0x5U
#define INA226_MODE_BUS_CONT        0x6U
#define INA226_MODE_SHUNT_BUS_CONT  0x7U  /**< Default continuous mode */

/* Mask/Enable register: Conversion Ready Flag */
#define INA226_MASK_CVRF            (1U << 3)

/* ─── I2C Addresses ──────────────────────────────────────────────────────── */
/** Base address (A1=GND, A0=GND). Shift left for 8-bit HAL format. */
#define INA226_I2C_ADDR_BASE        0x40U  /**< 7-bit base address */
/* BUGFIX: A1 represents states 0-3, so it must be shifted by 2, not 1 */
#define INA226_I2C_ADDR(a1, a0)     (uint8_t)((0x40U | ((a1) << 2) | (a0)) << 1)

/* ─── LSB Constants ──────────────────────────────────────────────────────── */
#define INA226_SHUNT_VOLTAGE_LSB_UV  2.5f   /**< 2.5 µV per LSB               */
#define INA226_BUS_VOLTAGE_LSB_MV    1.25f  /**< 1.25 mV per LSB              */

/* ─── Types ──────────────────────────────────────────────────────────────── */

/**
 * @brief Initialisation configuration.
 *
 * current_lsb_A and r_shunt_ohm are used to compute the Calibration register:
 * CAL = 0.00512 / (current_lsb_A * r_shunt_ohm)
 * Set both to 0 to skip calibration (current and power registers will be 0).
 */
typedef struct {
    I2C_HandleTypeDef *hi2c;          /**< I2C peripheral handle              */
    uint8_t            i2c_addr;      /**< 8-bit I2C address (left-shifted)   */
    uint16_t           config_reg;    /**< Initial value for CONFIG register  */
    float              current_lsb_A; /**< Desired current LSB in amperes     */
    float              r_shunt_ohm;   /**< Shunt resistor value in ohms       */
} ina226_config_t;

/**
 * @brief Sensor reading populated by ina226_read() / ina226_read_dma().
 */
typedef struct {
    float    shunt_voltage_mV; /**< Shunt voltage in mV                       */
    float    bus_voltage_V;    /**< Bus voltage in V                          */
    float    current_A;        /**< Current in A (requires calibration)       */
    float    power_W;          /**< Power in W   (requires calibration)       */
    int16_t  raw_shunt;        /**< Raw signed shunt voltage register         */
    uint16_t raw_bus;          /**< Raw unsigned bus voltage register         */
    int16_t  raw_current;      /**< Raw signed current register               */
    uint16_t raw_power;        /**< Raw unsigned power register               */
} ina226_data_t;

/**
 * @brief DMA transfer-complete callback type.
 * @param status  HAL_OK on success, or HAL error code.
 */
typedef void (*ina226_dma_callback_t)(HAL_StatusTypeDef status);

/* ─── Public API ─────────────────────────────────────────────────────────── */

HAL_StatusTypeDef ina226_init(const ina226_config_t *config);
HAL_StatusTypeDef ina226_read(ina226_data_t *data);
HAL_StatusTypeDef ina226_read_dma(ina226_data_t *data, ina226_dma_callback_t callback);
HAL_StatusTypeDef ina226_set_calibration(uint16_t cal_value);
HAL_StatusTypeDef ina226_set_config(uint16_t cfg);
HAL_StatusTypeDef ina226_wait_conversion_ready(uint32_t timeout_ms);

/**
 * @brief  HAL I2C DMA complete callback — call from HAL_I2C_MemRxCpltCallback.
 */
void ina226_dma_irq_handler(void);

#endif /* INA226_H */
