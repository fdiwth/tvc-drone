/* vl53l1x_platform.c — HAL I2C DMA implementation of ST's ULD platform layer */
#include "main.h"
#include "global.h"
#include "vl53l1_platform.h"
#include "cmsis_os.h"
#include <string.h>

extern I2C_HandleTypeDef  hi2c1;
extern osMutexId_t        i2c1MutexHandle;
extern osSemaphoreId_t    i2c1DmaSemHandle;   /* new semaphore — init count 0, same as your other DMA sems */

#define VL53L1X_I2C_TIMEOUT_MS  50

/*
 * These wrap HAL's DMA transfer + a semaphore block instead of polling,
 * so the calling task yields to the scheduler during the transfer window
 * rather than busy-waiting on the peripheral — same benefit your
 * uart2TxDmaSemHandle pattern gives com_send(). ST's ULD API is a
 * synchronous/blocking API by design (VL53L1X_GetDistance() etc. call
 * these directly and expect data back before returning), so this stays a
 * blocking call from the caller's perspective — DMA here buys you
 * hardware-driven transfer + scheduler-friendly waiting, not true
 * async/non-blocking behavior. A fully non-blocking flow would mean
 * bypassing ULD's wrapper functions and driving VL53L1X_CheckForDataReady
 * yourself — more invasive, not done here.
 *
 * Caller (whichever task polls the sensor) is still responsible for
 * holding i2c1MutexHandle around any VL53L1X_* call, same convention as
 * your other I2C1 peripherals (BMM150, INA226).
 */

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *data, uint32_t count)
{
    uint8_t buf[2 + 64];   /* register index (16-bit, big-endian per ST's protocol) + payload */
    if (count > sizeof(buf) - 2) return -1;

    buf[0] = (uint8_t)(index >> 8);
    buf[1] = (uint8_t)(index & 0xFF);
    memcpy(&buf[2], data, count);

    i2c1_pending = I2C1_PENDING_VL53;
        if (HAL_I2C_Master_Transmit_DMA(&hi2c1, (uint16_t)dev, buf, count + 2) != HAL_OK) {
            i2c1_pending = I2C1_PENDING_NONE;
            return -1;
        }
        if (osSemaphoreAcquire(i2c1DmaSemHandle, VL53L1X_I2C_TIMEOUT_MS) != osOK) {
            i2c1_pending = I2C1_PENDING_NONE;
            return -1;
        }
        return 0;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *data, uint32_t count)
{
    uint8_t idx_buf[2] = { (uint8_t)(index >> 8), (uint8_t)(index & 0xFF) };

    /* Write the register index (blocking — it's 2 bytes, not the bottleneck),
     * then DMA the actual payload read, which is the part that scales with
     * count and is worth offloading. */
    if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)dev, idx_buf, 2, VL53L1X_I2C_TIMEOUT_MS) != HAL_OK) return -1;
    i2c1_pending = I2C1_PENDING_VL53;
    if (HAL_I2C_Master_Receive_DMA(&hi2c1, (uint16_t)dev, data, (uint16_t)count) != HAL_OK) return -1;
    if (osSemaphoreAcquire(i2c1DmaSemHandle, VL53L1X_I2C_TIMEOUT_MS) != osOK) return -1;
    return 0;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data)
{
    return VL53L1_WriteMulti(dev, index, &data, 1);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data)
{
    uint8_t buf[2] = { (uint8_t)(data >> 8), (uint8_t)(data & 0xFF) };
    return VL53L1_WriteMulti(dev, index, buf, 2);
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{
    uint8_t buf[4] = {
        (uint8_t)(data >> 24), (uint8_t)(data >> 16),
        (uint8_t)(data >> 8),  (uint8_t)(data & 0xFF)
    };
    return VL53L1_WriteMulti(dev, index, buf, 4);
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data)
{
    return VL53L1_ReadMulti(dev, index, data, 1);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data)
{
    uint8_t buf[2];
    int8_t ret = VL53L1_ReadMulti(dev, index, buf, 2);
    if (ret == 0) *data = ((uint16_t)buf[0] << 8) | buf[1];
    return ret;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data)
{
    uint8_t buf[4];
    int8_t ret = VL53L1_ReadMulti(dev, index, buf, 4);
    if (ret == 0) *data = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16)
                        | ((uint32_t)buf[2] << 8)  | buf[3];
    return ret;
}

/* ULD uses this internally for its own timing-budget/polling logic, not for RTOS delays */
int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms)
{
    (void)dev;
    osDelay(wait_ms);
}
