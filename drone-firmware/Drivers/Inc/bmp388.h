/**
 * @file    bmp388.h
 * @brief   Driver for Tokmas BMP388 Barometric Pressure and Temperature Sensor
 *
 * IMPORTANT: Tokmas variant — NOT compatible with Bosch BMP388.
 *
 * Hardware facts (Tokmas datasheet):
 *   - SPI: Mode 3 (CPOL=1, CPHA=1)
 *   - ID register 0x0D resets to 0x11
 *   - Soft reset: write 0x09 to register 0x0C
 *   - Background P+T mode: write 0x07 to register 0x08
 *   - Data burst starts at 0x00 (PSR_B2 = MSB first)
 *
 * Coefficient sign conventions (datasheet §4.6.1, Table 10):
 *   c00       — SIGNED 20-bit 2's complement  (datasheet explicitly states so)
 *   c10       — SIGNED 20-bit 2's complement.
 *   c01..c30  — SIGNED 16-bit 2's complement.
 *   c31, c40  — SIGNED 12-bit 2's complement.
 *   c0,  c1   — SIGNED 12-bit 2's complement (temperature).
 *
 * All compensation arithmetic is done in double (STM32F7 FPv5-D16 handles
 * double in hardware). c00 ~600 000 and c10*Psc nearly cancel it — 32-bit
 * float would lose all significant digits.
 *
 * Known fixes vs. original driver:
 *   1. c00 now decoded as SIGNED 20-bit (was incorrectly unsigned).
 *   2. INT_PRS and INT_TMP enable bits are set in CFG_REG so that command-mode
 *      polling in bmp388_read() actually sees the INT_STS flags.
 */

#ifndef BMP388_H
#define BMP388_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

/* ─── Register Addresses ─────────────────────────────────────────────────── */
#define BMP388_REG_PSR_B2       0x00U
#define BMP388_REG_PSR_B1       0x01U
#define BMP388_REG_PSR_B0       0x02U
#define BMP388_REG_TMP_B2       0x03U
#define BMP388_REG_TMP_B1       0x04U
#define BMP388_REG_TMP_B0       0x05U
#define BMP388_REG_PRS_CFG      0x06U
#define BMP388_REG_TMP_CFG      0x07U
#define BMP388_REG_MEAS_CFG     0x08U
#define BMP388_REG_CFG_REG      0x09U
#define BMP388_REG_INT_STS      0x0AU
#define BMP388_REG_FIFO_STS     0x0BU
#define BMP388_REG_RESET        0x0CU
#define BMP388_REG_ID           0x0DU
#define BMP388_REG_COEF_START   0x10U

/* ─── MEAS_CFG status bits ───────────────────────────────────────────────── */
#define BMP388_COEF_RDY         (1U << 7)
#define BMP388_SENSOR_RDY       (1U << 6)
#define BMP388_TMP_RDY          (1U << 5)
#define BMP388_PRS_RDY          (1U << 4)

/* ─── Operating modes (MEAS_CTRL[2:0]) ──────────────────────────────────── */
#define BMP388_MODE_STANDBY     0x00U   /* Idle                              */
#define BMP388_MODE_PRESS_ONCE  0x01U   /* Command: single pressure          */
#define BMP388_MODE_TEMP_ONCE   0x02U   /* Command: single temperature       */
#define BMP388_MODE_NORMAL      0x07U   /* Background: continuous P+T        */

/* ─── CFG_REG bit positions ──────────────────────────────────────────────── */
#define BMP388_INT_HL           (1U << 7)   /* Interrupt active level        */
#define BMP388_INT_FIFO         (1U << 6)   /* FIFO full interrupt enable    */
#define BMP388_INT_TMP_EN       (1U << 5)   /* Temperature ready int enable  */
#define BMP388_INT_PRS_EN       (1U << 4)   /* Pressure ready int enable     */
#define BMP388_T_SHIFT          (1U << 3)   /* Temperature result bit-shift  */
#define BMP388_P_SHIFT          (1U << 2)   /* Pressure result bit-shift     */
#define BMP388_FIFO_EN          (1U << 1)   /* FIFO enable                   */
#define BMP388_SPI_MODE         (1U << 0)   /* 1 = 3-wire SPI                */

/* ─── INT_STS bits ───────────────────────────────────────────────────────── */
#define BMP388_INT_TMP          (1U << 1)
#define BMP388_INT_PRS          (1U << 0)

/* ─── Special values ─────────────────────────────────────────────────────── */
#define BMP388_SOFT_RESET_VAL   0x09U
#define BMP388_ID_RESET_VAL     0x11U

/* ─── Oversampling bits [3:0] ────────────────────────────────────────────── */
#define BMP388_OSR_x1           0x00U
#define BMP388_OSR_x2           0x01U
#define BMP388_OSR_x4           0x02U
#define BMP388_OSR_x8           0x03U
#define BMP388_OSR_x16          0x04U
#define BMP388_OSR_x32          0x05U
#define BMP388_OSR_x64          0x06U
#define BMP388_OSR_x128         0x07U

/* ─── Output data rate bits [7:4] ────────────────────────────────────────── */
#define BMP388_ODR_1HZ          (0x00U << 4)
#define BMP388_ODR_2HZ          (0x01U << 4)
#define BMP388_ODR_4HZ          (0x02U << 4)
#define BMP388_ODR_8HZ          (0x03U << 4)
#define BMP388_ODR_16HZ         (0x04U << 4)
#define BMP388_ODR_32HZ         (0x05U << 4)
#define BMP388_ODR_64HZ         (0x06U << 4)
#define BMP388_ODR_128HZ        (0x07U << 4)
#define BMP388_ODR_25HZ         (0x0CU << 4)
#define BMP388_ODR_50HZ         (0x0DU << 4)
#define BMP388_ODR_100HZ        (0x0EU << 4)
#define BMP388_ODR_200HZ        (0x0FU << 4)

/* ─── Scale factors — Table 4 ───────────────────────────────────────────── */
#define BMP388_K_OSR_x1         524288.0
#define BMP388_K_OSR_x2         1572864.0
#define BMP388_K_OSR_x4         3670016.0
#define BMP388_K_OSR_x8         7864320.0
#define BMP388_K_OSR_x16        253952.0
#define BMP388_K_OSR_x32        516096.0
#define BMP388_K_OSR_x64        1040384.0
#define BMP388_K_OSR_x128       2088960.0

/* ─── Altitude reference ─────────────────────────────────────────────────── */
#define BMP388_SEA_LEVEL_HPA    1013.25f

/* ─── I2C addresses ──────────────────────────────────────────────────────── */
#define BMP388_I2C_ADDR_PRIMARY    (0x77U << 1)
#define BMP388_I2C_ADDR_SECONDARY  (0x76U << 1)

/* ─── Types ──────────────────────────────────────────────────────────────── */

typedef enum { BMP388_IFACE_I2C = 0, BMP388_IFACE_SPI = 1 } bmp388_iface_t;

typedef struct {
    double c0,  c1;          /* 12-bit signed                                 */
    double c00;              /* 20-bit SIGNED 2's complement (per datasheet)  */
    double c10;              /* 20-bit SIGNED 2's complement                  */
    double c01, c11, c20, c21, c30;  /* 16-bit signed                        */
    double c31, c40;         /* 12-bit signed                                 */
} bmp388_calib_t;

typedef struct {
    bmp388_iface_t     iface;
    I2C_HandleTypeDef *hi2c;
    uint8_t            i2c_addr;
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    uint8_t            prs_cfg;       /* BMP388_ODR_x | BMP388_OSR_x          */
    uint8_t            tmp_cfg;       /* BMP388_ODR_x | BMP388_OSR_x          */
    uint8_t            mode;          /* BMP388_MODE_x                        */
    float              sea_level_hpa; /* 0.0f → use 1013.25 hPa              */
} bmp388_config_t;

typedef struct {
    float    temperature_C;
    float    pressure_Pa;
    float    altitude_m;
    uint32_t raw_pressure;
    uint32_t raw_temperature;
} bmp388_data_t;

typedef void (*bmp388_dma_callback_t)(HAL_StatusTypeDef status);

/* ─── Debug ──────────────────────────────────────────────────────────────── */
extern uint8_t bmp388_debug_coef_raw[21]; /*< Set breakpoint after init, inspect in Watch */

/* ─── Public API ─────────────────────────────────────────────────────────── */
HAL_StatusTypeDef bmp388_init(const bmp388_config_t *config);
HAL_StatusTypeDef bmp388_read(bmp388_data_t *data);
HAL_StatusTypeDef bmp388_read_dma(bmp388_data_t        *data,
                                   bmp388_dma_callback_t callback);
void              bmp388_dma_irq_handler(void);

#endif /* BMP388_H */
