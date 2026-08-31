/* pmw3901.h */
#ifndef PMW3901_H
#define PMW3901_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

/* ─── Register addresses ─────────────────────────────────────────────────── */
#define PMW3901_REG_PRODUCT_ID          0x00U   /* expect 0x49 */
#define PMW3901_REG_REVISION_ID         0x01U
#define PMW3901_REG_MOTION              0x02U
#define PMW3901_REG_DELTA_X_L           0x03U
#define PMW3901_REG_DELTA_X_H           0x04U
#define PMW3901_REG_DELTA_Y_L           0x05U
#define PMW3901_REG_DELTA_Y_H           0x06U
#define PMW3901_REG_SQUAL               0x07U
#define PMW3901_REG_INVERSE_PRODUCT_ID  0x5FU   /* expect 0xB6 — bitwise-inverse check of 0x00 */
#define PMW3901_REG_MOTION_BURST        0x16U
#define PMW3901_REG_POWER_UP_RESET      0x3AU
#define PMW3901_REG_SHUTDOWN            0x3BU

#define PMW3901_POWER_UP_RESET_VAL      0x5AU
#define PMW3901_PRODUCT_ID_VAL          0x49U
#define PMW3901_INVERSE_PRODUCT_ID_VAL  0xB6U

/* ─── Types ──────────────────────────────────────────────────────────────── */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef       *cs_port;
    uint16_t             cs_pin;
} pmw3901_config_t;

/* Parsed motion-burst result (12-byte burst starting at 0x16) */
typedef struct {
    uint8_t  motion_flags;   /* raw Motion register, bit7 = motion since last read */
    uint8_t  observation;
    int16_t  delta_x;        /* raw sensor counts, sign per datasheet */
    int16_t  delta_y;
    uint8_t  squal;          /* surface quality — low value = poor tracking surface */
    uint8_t  raw_data_sum;
    uint8_t  max_raw_data;
    uint8_t  min_raw_data;
    uint16_t shutter;        /* upper<<8 | lower — rises on dim/close surfaces */
} pmw3901_data_t;

typedef void (*pmw3901_dma_callback_t)(HAL_StatusTypeDef status);

/* ─── Public API ─────────────────────────────────────────────────────────── */
HAL_StatusTypeDef pmw3901_init(const pmw3901_config_t *config);
HAL_StatusTypeDef pmw3901_read(pmw3901_data_t *data);                 /* blocking */
HAL_StatusTypeDef pmw3901_read_dma(pmw3901_data_t *data, pmw3901_dma_callback_t callback);
void              pmw3901_dma_irq_handler(void);   /* call from HAL_SPI_RxCpltCallback */

#endif /* PMW3901_H */
