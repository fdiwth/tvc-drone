/**
 * @file    bmm150.h
 * @brief   Driver for Bosch BMM150 3-Axis Geomagnetic Sensor
 *
 * Supports I2C and SPI (4-wire) interfaces.
 * Magnetic field output: ±1300 µT (X/Y), ±2500 µT (Z)
 * Resolution: ~0.3 µT.
 *
 * IMPORTANT: The device starts in Suspend mode after power-on.
 * bmm150_init() transitions it to Sleep → Normal (Active) mode and reads
 * the factory-trimmed compensation data required by the compensation
 * formulas (stored in NVM registers 0x5D–0x71).
 */

#ifndef BMM150_H
#define BMM150_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

/* ─── Register Addresses ─────────────────────────────────────────────────── */

/* Read-only data registers */
#define BMM150_REG_CHIP_ID          0x40U  /**< Should read 0x32 (in sleep mode)*/
#define BMM150_REG_DATA_X_LSB       0x42U
#define BMM150_REG_DATA_X_MSB       0x43U
#define BMM150_REG_DATA_Y_LSB       0x44U
#define BMM150_REG_DATA_Y_MSB       0x45U
#define BMM150_REG_DATA_Z_LSB       0x46U
#define BMM150_REG_DATA_Z_MSB       0x47U
#define BMM150_REG_RHALL_LSB        0x48U  /**< Hall resistance (for compensation) */
#define BMM150_REG_RHALL_MSB        0x49U
#define BMM150_REG_INT_STATUS       0x4AU  /**< Interrupt status + Data Ready  */

/* Read/write control registers */
#define BMM150_REG_POWER_CTRL       0x4BU  /**< Power control + soft reset     */
#define BMM150_REG_OP_CTRL          0x4CU  /**< OpMode, data rate, self-test   */
#define BMM150_REG_INT_CTRL         0x4DU  /**< Interrupt enable bits          */
#define BMM150_REG_SENS_CTRL        0x4EU  /**< Sensor axis enable, INT pin    */
#define BMM150_REG_LOW_THRESH       0x4FU
#define BMM150_REG_HIGH_THRESH      0x50U
#define BMM150_REG_REPXY            0x51U  /**< XY repetitions for oversampling*/
#define BMM150_REG_REPZ             0x52U  /**< Z repetitions for oversampling */

/* Trim / compensation data registers */
#define BMM150_REG_TRIM_START       0x5DU
#define BMM150_REG_DIG_X1           0x5DU
#define BMM150_REG_DIG_Y1           0x5EU
#define BMM150_REG_DIG_Z4_LSB       0x62U
#define BMM150_REG_DIG_Z4_MSB       0x63U
#define BMM150_REG_DIG_X2           0x64U
#define BMM150_REG_DIG_Y2           0x65U
#define BMM150_REG_DIG_Z2_LSB       0x68U
#define BMM150_REG_DIG_Z2_MSB       0x69U
#define BMM150_REG_DIG_Z1_LSB       0x6AU
#define BMM150_REG_DIG_Z1_MSB       0x6BU
#define BMM150_REG_DIG_XYZ1_LSB     0x6CU
#define BMM150_REG_DIG_XYZ1_MSB     0x6DU
#define BMM150_REG_DIG_Z3_LSB       0x6EU
#define BMM150_REG_DIG_Z3_MSB       0x6FU
#define BMM150_REG_DIG_XY2          0x70U
#define BMM150_REG_DIG_XY1          0x71U

/* ─── Register Bit Definitions ───────────────────────────────────────────── */

/* POWER_CTRL (0x4B) */
#define BMM150_POWER_CTRL_POW_BIT   (1U << 0)  /**< 1 = enter Sleep mode      */
#define BMM150_POWER_CTRL_SOFT_RST1 (1U << 7)
#define BMM150_POWER_CTRL_SOFT_RST2 (1U << 1)
#define BMM150_CMD_SOFT_RESET       (BMM150_POWER_CTRL_SOFT_RST1 | \
                                     BMM150_POWER_CTRL_SOFT_RST2)

/* OP_CTRL (0x4C) operating modes [2:1] */
#define BMM150_OPMODE_NORMAL        (0x0U << 1)
#define BMM150_OPMODE_FORCED        (0x1U << 1)
#define BMM150_OPMODE_SLEEP         (0x3U << 1)

/* OP_CTRL output data rate [4:3] */
#define BMM150_ODR_10HZ             (0x0U << 3)
#define BMM150_ODR_2HZ              (0x1U << 3)
#define BMM150_ODR_6HZ              (0x2U << 3)
#define BMM150_ODR_8HZ              (0x3U << 3)
#define BMM150_ODR_15HZ             (0x4U << 3)
#define BMM150_ODR_20HZ             (0x5U << 3)
#define BMM150_ODR_25HZ             (0x6U << 3)
#define BMM150_ODR_30HZ             (0x7U << 3)

/* INT_STATUS (0x4A) */
#define BMM150_INT_STATUS_DRDY      (1U << 0)  /**< Not in 0x4A — see RHALL_LSB */
#define BMM150_RHALL_LSB_DRDY       (1U << 0)  /**< Data Ready in RHALL_LSB reg */

/* Chip ID */
#define BMM150_CHIP_ID              0x32U

/* Overflow / invalid value sentinels */
#define BMM150_OVERFLOW_ADCVAL_XY   (-4096)
#define BMM150_OVERFLOW_ADCVAL_Z    (-16384)
#define BMM150_OVERFLOW_OUTPUT      (0x8000)

/* ─── I2C Addresses ──────────────────────────────────────────────────────── */
#define BMM150_I2C_ADDR_PRIMARY     (0x13U << 1)  /**< SDO=VDDIO, CSB=VDDIO   */
#define BMM150_I2C_ADDR_ALT1        (0x12U << 1)
#define BMM150_I2C_ADDR_ALT2        (0x11U << 1)
#define BMM150_I2C_ADDR_ALT3        (0x10U << 1)

/* ─── Types ──────────────────────────────────────────────────────────────── */

typedef enum {
    BMM150_IFACE_I2C = 0,
    BMM150_IFACE_SPI = 1,
} bmm150_iface_t;

/**
 * @brief Factory-programmed compensation (trim) coefficients.
 */
typedef struct {
    int8_t   dig_x1;
    int8_t   dig_y1;
    int8_t   dig_x2;
    int8_t   dig_y2;
    uint16_t dig_z1;
    int16_t  dig_z2;
    int16_t  dig_z3;
    int16_t  dig_z4;
    uint8_t  dig_xy1;
    int8_t   dig_xy2;
    uint16_t dig_xyz1;
} bmm150_trim_t;

/**
 * @brief Initialisation configuration.
 */
typedef struct {
    bmm150_iface_t     iface;

    /* I2C */
    I2C_HandleTypeDef *hi2c;
    uint8_t            i2c_addr;

    /* SPI */
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;

    /* Measurement settings */
    uint8_t op_ctrl;   /**< BMM150_OPMODE_x | BMM150_ODR_x */
    uint8_t rep_xy;    /**< XY repetitions  (0 = 1 rep, 1 = 3 reps, …)   */
    uint8_t rep_z;     /**< Z  repetitions  (0 = 1 rep, 1 = 2 reps, …)   */
} bmm150_config_t;

/**
 * @brief Compensated sensor output from bmm150_read() / bmm150_read_dma().
 */
typedef struct {
    float   mag_x_uT;    /**< X-axis magnetic field in µT                 */
    float   mag_y_uT;    /**< Y-axis magnetic field in µT                 */
    float   mag_z_uT;    /**< Z-axis magnetic field in µT                 */
    int16_t raw_x;       /**< Raw 13-bit signed X ADC value               */
    int16_t raw_y;       /**< Raw 13-bit signed Y ADC value               */
    int16_t raw_z;       /**< Raw 15-bit signed Z ADC value               */
    uint16_t raw_rhall;  /**< Raw 14-bit unsigned RHALL value             */
    uint8_t  drdy;       /**< Non-zero when data-ready flag was set       */
} bmm150_data_t;

/**
 * @brief DMA transfer-complete callback type.
 * @param status  HAL_OK on success, or HAL error code.
 */
typedef void (*bmm150_dma_callback_t)(HAL_StatusTypeDef status);

/* ─── Public API ─────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise driver: soft-reset, power up, verify Chip ID, load
 *         trim coefficients, configure operating mode.
 * @param  config  Pointer to populated configuration struct.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef bmm150_init(const bmm150_config_t *config);

/**
 * @brief  Blocking read of X, Y, Z magnetic field and RHALL, then compensate.
 * @param  data  Output struct.
 * @return HAL_OK or HAL_ERROR / HAL_TIMEOUT.
 */
HAL_StatusTypeDef bmm150_read(bmm150_data_t *data);

/**
 * @brief  Non-blocking DMA read of the 8 raw data bytes (0x42–0x49).
 *         Compensation is performed in the callback context.
 * @param  data      Output struct (must remain valid until callback fires).
 * @param  callback  Called with HAL_OK or error on completion.
 * @return HAL_OK if DMA started, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef bmm150_read_dma(bmm150_data_t        *data,
                                   bmm150_dma_callback_t callback);

/**
 * @brief  HAL DMA complete callback — call from HAL_I2C_MasterRxCpltCallback
 *         or HAL_SPI_RxCpltCallback.
 */
void bmm150_dma_irq_handler(void);

#endif /* BMM150_H */
