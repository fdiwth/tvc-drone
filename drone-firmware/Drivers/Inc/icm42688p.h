/**
 * @file    icm42688p.h
 * @brief   Driver for Tokmas ICM-42688-P 6-Axis IMU (Accel + Gyro + Temp)
 *
 * IMPORTANT: Tokmas variant — register map differs from Bosch/TDK ICM-42688-P.
 *
 * Hardware facts (Tokmas datasheet):
 *   - SPI: Mode 0 (CPOL=0, CPHA=0) or Mode 3 (CPOL=1, CPHA=1), auto-selected.
 *   - CHIP_ID register 0x1F resets to 0xA1 ('K01').
 *   - Soft reset: write 0x73 to register 0x00.
 *   - Burst data read starts at 0x00 (ACC_DATA_XL), not 0x1D.
 *   - All data registers are LSB-first (little-endian).
 *   - Temperature: 12-bit unsigned, formula: (TEMP_DATA - ROOM_TEMP)/14 + 25°C
 *   - ROOM_TEMP stored at 0x29[7:0] (low) and 0x2A[3:0] (high).
 *
 * Data register layout (burst read from 0x00, 14 bytes):
 *   [0]  ACC_DATA_XL   — Accel X low
 *   [1]  ACC_DATA_XH   — Accel X high
 *   [2]  ACC_DATA_YL   — Accel Y low
 *   [3]  ACC_DATA_YH   — Accel Y high
 *   [4]  ACC_DATA_ZL   — Accel Z low
 *   [5]  ACC_DATA_ZH   — Accel Z high
 *   [6]  GYRO_DATA_XL  — Gyro X low
 *   [7]  GYRO_DATA_XH  — Gyro X high
 *   [8]  GYRO_DATA_YL  — Gyro Y low
 *   [9]  GYRO_DATA_YH  — Gyro Y high
 *   [10] GYRO_DATA_ZL  — Gyro Z low
 *   [11] GYRO_DATA_ZH  — Gyro Z high
 *   [12] TEMP_DATA_L   — Temp low  (0x0C)
 *   [13] TEMP_DATA_H   — Temp high (0x0D)
 *
 * Fixes vs. original driver:
 *   1. Burst start corrected from 0x1D (FIFO port) to 0x00 (ACC_DATA_XL).
 *   2. All axis byte assembly corrected from MSB-first to LSB-first.
 *   3. Temperature: 12-bit unsigned, correct formula using ROOM_TEMP offset.
 *   4. FS_SEL bit shift corrected from << 5 to << 4 (bits [6:4]).
 *   5. CHIP_ID register address corrected to 0x1F, expected value to 0xA1.
 */

#ifndef ICM42688P_H
#define ICM42688P_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

/* ─── Register Addresses ─────────────────────────────────────────────────── */

/* Accelerometer data — LSB first, burst starts here */
#define ICM42688P_REG_ACC_DATA_XL       0x00U
#define ICM42688P_REG_ACC_DATA_XH       0x01U
#define ICM42688P_REG_ACC_DATA_YL       0x02U
#define ICM42688P_REG_ACC_DATA_YH       0x03U
#define ICM42688P_REG_ACC_DATA_ZL       0x04U
#define ICM42688P_REG_ACC_DATA_ZH       0x05U

/* Gyroscope data — LSB first */
#define ICM42688P_REG_GYRO_DATA_XL      0x06U
#define ICM42688P_REG_GYRO_DATA_XH      0x07U
#define ICM42688P_REG_GYRO_DATA_YL      0x08U
#define ICM42688P_REG_GYRO_DATA_YH      0x09U
#define ICM42688P_REG_GYRO_DATA_ZL      0x0AU
#define ICM42688P_REG_GYRO_DATA_ZH      0x0BU

/* Temperature data — LSB first, 12-bit unsigned */
#define ICM42688P_REG_TEMP_DATA_L       0x0CU
#define ICM42688P_REG_TEMP_DATA_H       0x0DU

/* Status and ID */
#define ICM42688P_REG_INT_STATUS_L      0x16U
#define ICM42688P_REG_INT_STATUS_H      0x17U
#define ICM42688P_REG_FIFO_STATUS_L     0x1BU
#define ICM42688P_REG_FIFO_STATUS_H     0x1CU
#define ICM42688P_REG_FIFO_DATA         0x1DU  /* FIFO read port — NOT data  */
#define ICM42688P_REG_CHIP_ID           0x1FU  /* Resets to 0xA1             */

/* Accelerometer configuration */
#define ICM42688P_REG_ACC_CONFIG0       0x20U
#define ICM42688P_REG_ACC_CONFIG1       0x21U  /* FS_SEL[6:4], ODR[3:0]      */
#define ICM42688P_REG_ACC_CONFIG2       0x22U

/* Gyroscope configuration */
#define ICM42688P_REG_GYRO_CONFIG1      0x23U
#define ICM42688P_REG_GYRO_CONFIG2      0x24U  /* FS_SEL[6:4], ODR[3:0]      */
#define ICM42688P_REG_GYRO_CONFIG3      0x25U

/* Temperature / room-temp offset */
#define ICM42688P_REG_TEMP_CONFIG1      0x29U  /* ROOM_TEMP[7:0]             */
#define ICM42688P_REG_TEMP_CONFIG2      0x2AU  /* ROOM_TEMP[11:8] in [3:0]   */

/* Power / FIFO / interrupt control */
#define ICM42688P_REG_PWR_MGMT         0x30U   /* Power mode register        */
#define ICM42688P_REG_FIFO_CONFIG       0x35U
#define ICM42688P_REG_INT_ENABLE_L      0x40U
#define ICM42688P_REG_INT_ENABLE_H      0x41U
#define ICM42688P_REG_INT_CONFIG        0x42U

/* Bank select (register 0x7F bit 0 selects page 1 or 2) */
#define ICM42688P_REG_PAGE_SEL          0x7FU

/* ─── Register Values ────────────────────────────────────────────────────── */

/* CHIP_ID — Tokmas variant */
#define ICM42688P_CHIP_ID               0xA1U

/* Soft reset — write 0x73 to register 0x00 (ACC_DATA_XL doubles as cmd) */
#define ICM42688P_SOFT_RESET_REG        0x00U
#define ICM42688P_SOFT_RESET_VAL        0x73U

/* PWR_MGMT (0x30) mode values */
#define ICM42688P_PWR_NORMAL            0x00U  /* Accel + Gyro normal mode   */
#define ICM42688P_PWR_ACC_ONLY          0x02U
#define ICM42688P_PWR_ACC_LP            0x03U
#define ICM42688P_PWR_GYRO_ONLY         0x05U
#define ICM42688P_PWR_DOWN              0x07U

/* INT_STATUS_H (0x17) data-ready bits */
#define ICM42688P_INT_GYRO_DRDY         (1U << 2)
#define ICM42688P_INT_ACC_DRDY          (1U << 1)

/* ─── ACC_CONFIG1 (0x21): FS_SEL bits [6:4] ─────────────────────────────── */
/* Datasheet §9.1.4: bits [6:4] = range; 000=2g 001=4g 010=8g 011=16g      */
#define ICM42688P_ACCEL_FS_2G           (0x00U << 4)
#define ICM42688P_ACCEL_FS_4G           (0x01U << 4)
#define ICM42688P_ACCEL_FS_8G           (0x02U << 4)
#define ICM42688P_ACCEL_FS_16G          (0x03U << 4)

/* ─── GYRO_CONFIG2 (0x24): FS_SEL bits [6:4] ────────────────────────────── */
/* Datasheet §9.1.5: bits [6:4] = range; 000=31 001=62 010=125 011=250     */
/*                                        100=500 101=1000 110=2000          */
#define ICM42688P_GYRO_FS_31DPS         (0x00U << 4)
#define ICM42688P_GYRO_FS_62DPS         (0x01U << 4)
#define ICM42688P_GYRO_FS_125DPS        (0x02U << 4)
#define ICM42688P_GYRO_FS_250DPS        (0x03U << 4)
#define ICM42688P_GYRO_FS_500DPS        (0x04U << 4)
#define ICM42688P_GYRO_FS_1000DPS       (0x05U << 4)
#define ICM42688P_GYRO_FS_2000DPS       (0x06U << 4)

/* ─── ODR values — bits [3:0] shared by ACC_CONFIG1 and GYRO_CONFIG2 ─────── */
/* Datasheet Table 14 / Table 17 */
#define ICM42688P_ODR_880HZ             0x00U
#define ICM42688P_ODR_440HZ             0x01U
#define ICM42688P_ODR_220HZ             0x02U
#define ICM42688P_ODR_110HZ             0x03U
#define ICM42688P_ODR_1760HZ            0x08U
#define ICM42688P_ODR_3520HZ            0x09U
#define ICM42688P_ODR_7040HZ            0x0AU
#define ICM42688P_ODR_14080HZ           0x0BU  /* Gyro only */

/* ─── Sensitivity scale factors ──────────────────────────────────────────── */
/* Accel: datasheet Table 2 */
#define ICM42688P_ACC_SCALE_16G         (1.0f / 2048.0f)
#define ICM42688P_ACC_SCALE_8G          (1.0f / 4096.0f)
#define ICM42688P_ACC_SCALE_4G          (1.0f / 8192.0f)
#define ICM42688P_ACC_SCALE_2G          (1.0f / 16384.0f)

/* Gyro: datasheet Table 1 */
#define ICM42688P_GYRO_SCALE_2000DPS    (1.0f / 16.4f)
#define ICM42688P_GYRO_SCALE_1000DPS    (1.0f / 32.8f)
#define ICM42688P_GYRO_SCALE_500DPS     (1.0f / 65.5f)
#define ICM42688P_GYRO_SCALE_250DPS     (1.0f / 131.0f)
#define ICM42688P_GYRO_SCALE_125DPS     (1.0f / 262.0f)
#define ICM42688P_GYRO_SCALE_62DPS      (1.0f / 524.3f)
#define ICM42688P_GYRO_SCALE_31DPS      (1.0f / 1048.6f)

/* ─── I2C Addresses ──────────────────────────────────────────────────────── */
#define ICM42688P_I2C_ADDR_LOW          (0x68U << 1)  /* SDO = GND            */
#define ICM42688P_I2C_ADDR_HIGH         (0x69U << 1)  /* SDO = VDDIO          */

/* ─── Types ──────────────────────────────────────────────────────────────── */

typedef enum {
    ICM42688P_IFACE_I2C = 0,
    ICM42688P_IFACE_SPI = 1,
} icm42688p_iface_t;

typedef struct {
    icm42688p_iface_t  iface;
    /* I2C */
    I2C_HandleTypeDef *hi2c;
    uint8_t            i2c_addr;
    /* SPI */
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    /* Measurement config */
    uint8_t  gyro_config;    /* ICM42688P_GYRO_FS_x  | ICM42688P_ODR_x        */
    uint8_t  accel_config;   /* ICM42688P_ACCEL_FS_x | ICM42688P_ODR_x        */
    float    acc_scale;      /* ICM42688P_ACC_SCALE_x                          */
    float    gyro_scale;     /* ICM42688P_GYRO_SCALE_x                         */
} icm42688p_config_t;

typedef struct {
    float    accel_x_g;
    float    accel_y_g;
    float    accel_z_g;
    float    gyro_x_dps;
    float    gyro_y_dps;
    float    gyro_z_dps;
    float    temperature_C;
    int16_t  raw_ax, raw_ay, raw_az;
    int16_t  raw_gx, raw_gy, raw_gz;
    uint16_t raw_temp;        /* 12-bit unsigned                               */
    uint16_t raw_room_temp;   /* 12-bit unsigned factory offset                */
} icm42688p_data_t;

typedef void (*icm42688p_dma_callback_t)(HAL_StatusTypeDef status);

/* ─── Public API ─────────────────────────────────────────────────────────── */
HAL_StatusTypeDef icm42688p_init(const icm42688p_config_t *config);
HAL_StatusTypeDef icm42688p_read(icm42688p_data_t *data);
HAL_StatusTypeDef icm42688p_read_dma(icm42688p_data_t        *data,
                                      icm42688p_dma_callback_t callback);
void              icm42688p_dma_irq_handler(void);

#endif /* ICM42688P_H */
