/* pmw3901.c */
#include "pmw3901.h"
#include <string.h>

/*
 * Timing note: the PMW3901 needs microsecond-scale gaps between SPI
 * transactions (tSRAD ~35us address-to-data turnaround on burst reads,
 * ~4-20us between consecutive register accesses depending on which pair).
 * HAL_Delay() is millisecond-resolution — too coarse — so this driver uses
 * a DWT-cycle-counter microsecond delay, a standard technique on Cortex-M7.
 */
static void dwt_delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = (SystemCoreClock / 1000000U) * us;
    while ((DWT->CYCCNT - start) < ticks) { }
}

static void dwt_enable(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/* ─── Private state ──────────────────────────────────────────────────────── */
static SPI_HandleTypeDef *s_hspi    = NULL;
static GPIO_TypeDef       *s_cs_port = NULL;
static uint16_t             s_cs_pin  = 0;

static pmw3901_dma_callback_t s_dma_cb   = NULL;
static pmw3901_data_t        *s_dma_data = NULL;
static uint8_t                s_dma_raw[12];

static inline void cs_lo(void) { HAL_GPIO_WritePin(s_cs_port, s_cs_pin, GPIO_PIN_RESET); }
static inline void cs_hi(void) { HAL_GPIO_WritePin(s_cs_port, s_cs_pin, GPIO_PIN_SET);   }

/* ─── Blocking single-register access ────────────────────────────────────── */
static HAL_StatusTypeDef reg_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { (uint8_t)(reg | 0x80U), val };  /* bit7 set = write */
    cs_lo();
    HAL_StatusTypeDef ret = HAL_SPI_Transmit(s_hspi, buf, 2, 10);
    dwt_delay_us(20);   /* tSWW/tSWR minimum inter-write gap */
    cs_hi();
    dwt_delay_us(20);
    return ret;
}

static HAL_StatusTypeDef reg_read(uint8_t reg, uint8_t *val)
{
    uint8_t cmd = reg & 0x7FU;  /* bit7 clear = read */
    cs_lo();
    HAL_StatusTypeDef ret = HAL_SPI_Transmit(s_hspi, &cmd, 1, 10);
    if (ret == HAL_OK) {
        dwt_delay_us(35);   /* tSRAD address-to-data turnaround */
        ret = HAL_SPI_Receive(s_hspi, val, 1, 10);
    }
    cs_hi();
    dwt_delay_us(20);
    return ret;
}

static void parse_burst(pmw3901_data_t *out, const uint8_t *b)
{
    out->motion_flags = b[0];
    out->observation   = b[1];
    out->delta_x        = (int16_t)(((uint16_t)b[3] << 8) | b[2]);
    out->delta_y        = (int16_t)(((uint16_t)b[5] << 8) | b[4]);
    out->squal          = b[6];
    out->raw_data_sum   = b[7];
    out->max_raw_data   = b[8];
    out->min_raw_data   = b[9];
    out->shutter        = ((uint16_t)b[10] << 8) | b[11];
}

/*
 * Vendor "performance optimization" register sequence, verified against
 * Bitcraze's reference Arduino driver (bitcraze/Bitcraze_PMW3901,
 * src/Bitcraze_PMW3901.cpp, initRegisters()) on 2026-07-30. PixArt does
 * not publish the meaning of these registers in the public datasheet.
 *
 * Two classes of bug were found and fixed vs. the original table here:
 *   1. Three wrong byte values: {0x65,0x67}->{0x65,0x60}, {0x63,0x70}->
 *      {0x63,0x78}, {0x48,0x48}->{0x48,0x58} (all within the 0x7F,0x14 /
 *      0x7F,0x15 register banks).
 *   2. The sequence previously stopped right after {0x70,0x00} + the
 *      100ms delay. The reference driver continues for 14 more writes
 *      after that point (down through the final {0x40,0x80}), which is
 *      required to bring the sensor into its actual tuned operating
 *      state. Without this tail, the chip responds over SPI and reports
 *      plausible-looking but weak SQUAL, with motion detection never
 *      triggering — exactly the "responds but stuck at 0" symptom this
 *      fix addresses.
 */
static const uint8_t s_perf_opt_regs[][2] = {
    {0x7F,0x00},{0x61,0xAD},{0x7F,0x03},{0x40,0x00},{0x7F,0x05},
    {0x41,0xB3},{0x43,0xF1},{0x45,0x14},{0x5B,0x32},{0x5F,0x34},
    {0x7B,0x08},{0x7F,0x06},{0x44,0x1B},{0x40,0xBF},{0x4E,0x3F},
    {0x7F,0x08},{0x65,0x20},{0x6A,0x18},{0x7F,0x09},{0x4F,0xAF},
    {0x5F,0x40},{0x48,0x80},{0x49,0x80},{0x57,0x77},{0x60,0x78},
    {0x61,0x78},{0x62,0x08},{0x63,0x50},{0x7F,0x0A},{0x45,0x60},
    {0x7F,0x00},{0x4D,0x11},{0x55,0x80},{0x74,0x1F},{0x75,0x1F},
    {0x4A,0x78},{0x4B,0x78},{0x44,0x08},{0x45,0x50},{0x64,0xFF},
    {0x65,0x1F},{0x7F,0x14},{0x65,0x60},{0x66,0x08},{0x63,0x78},
    {0x7F,0x15},{0x48,0x58},{0x7F,0x07},{0x41,0x0D},{0x43,0x14},
    {0x4B,0x0E},{0x45,0x0F},{0x44,0x42},{0x4C,0x80},{0x7F,0x10},
    {0x5B,0x02},{0x7F,0x07},{0x40,0x41},{0x70,0x00},
    /* ---- tail sequence, previously missing — required after the delay ---- */
    {0x32,0x44},{0x7F,0x07},{0x40,0x40},{0x7F,0x06},{0x62,0xF0},
    {0x63,0x00},{0x7F,0x0D},{0x48,0xC0},{0x6F,0xD5},{0x7F,0x00},
    {0x5B,0xA0},{0x4E,0xA8},{0x5A,0x50},{0x40,0x80},
};

/* ─── Public API ─────────────────────────────────────────────────────────── */
HAL_StatusTypeDef pmw3901_init(const pmw3901_config_t *config)
{
    if (!config || !config->hspi || !config->cs_port) return HAL_ERROR;

    s_hspi    = config->hspi;
    s_cs_port = config->cs_port;
    s_cs_pin  = config->cs_pin;

    dwt_enable();
    cs_hi();
    HAL_Delay(1);

    /* Power-up reset */
    HAL_StatusTypeDef ret = reg_write(PMW3901_REG_POWER_UP_RESET, PMW3901_POWER_UP_RESET_VAL);
    if (ret != HAL_OK) return ret;
    HAL_Delay(5);

    /*
     * Verify chip identity with BOTH the Product_ID register and its
     * bitwise-inverse counterpart at 0x5F, matching the reference driver
     * (Bitcraze checks chipId==0x49 && inverse==0xB6 before proceeding).
     * A single-register check can pass on a marginal/noisy bus even when
     * something's subtly wrong; the dual check is a stronger guarantee
     * the sensor is genuinely present and responding correctly before any
     * further configuration is attempted.
     */
    uint8_t id = 0;
    ret = reg_read(PMW3901_REG_PRODUCT_ID, &id);
    if (ret != HAL_OK) return ret;

    uint8_t inv_id = 0;
    ret = reg_read(PMW3901_REG_INVERSE_PRODUCT_ID, &inv_id);
    if (ret != HAL_OK) return ret;

    if (id != PMW3901_PRODUCT_ID_VAL || inv_id != PMW3901_INVERSE_PRODUCT_ID_VAL) {
        return HAL_ERROR;
    }

    /* Datasheet recommends reading + discarding registers 0x02,0x03,0x04,0x05,0x06 once after reset */
    uint8_t dummy;
    reg_read(PMW3901_REG_MOTION, &dummy);
    reg_read(PMW3901_REG_DELTA_X_L, &dummy);
    reg_read(PMW3901_REG_DELTA_X_H, &dummy);
    reg_read(PMW3901_REG_DELTA_Y_L, &dummy);
    reg_read(PMW3901_REG_DELTA_Y_H, &dummy);
    HAL_Delay(1);

    for (size_t i = 0; i < sizeof(s_perf_opt_regs) / sizeof(s_perf_opt_regs[0]); i++) {
        ret = reg_write(s_perf_opt_regs[i][0], s_perf_opt_regs[i][1]);
        if (ret != HAL_OK) return ret;
        if (s_perf_opt_regs[i][0] == 0x70) HAL_Delay(100); /* known settle point, mid-sequence */
    }

    return HAL_OK;
}

HAL_StatusTypeDef pmw3901_read(pmw3901_data_t *data)
{
    if (!data) return HAL_ERROR;

    uint8_t cmd = PMW3901_REG_MOTION_BURST & 0x7FU;
    uint8_t raw[12];

    cs_lo();
    HAL_StatusTypeDef ret = HAL_SPI_Transmit(s_hspi, &cmd, 1, 10);
    if (ret == HAL_OK) {
        dwt_delay_us(35);
        ret = HAL_SPI_Receive(s_hspi, raw, sizeof(raw), 10);
    }
    cs_hi();
    dwt_delay_us(20);

    if (ret != HAL_OK) return ret;
    parse_burst(data, raw);
    return HAL_OK;
}

HAL_StatusTypeDef pmw3901_read_dma(pmw3901_data_t *data, pmw3901_dma_callback_t callback)
{
    if (!data || !callback) return HAL_ERROR;

    s_dma_cb   = callback;
    s_dma_data = data;

    uint8_t cmd = PMW3901_REG_MOTION_BURST & 0x7FU;
    cs_lo();
    HAL_StatusTypeDef ret = HAL_SPI_Transmit(s_hspi, &cmd, 1, 10);
    if (ret != HAL_OK) { cs_hi(); s_dma_cb = NULL; s_dma_data = NULL; return ret; }

    dwt_delay_us(35);   /* address-to-data turnaround still applies before starting the DMA receive */
    ret = HAL_SPI_Receive_DMA(s_hspi, s_dma_raw, sizeof(s_dma_raw));
    if (ret != HAL_OK) { cs_hi(); s_dma_cb = NULL; s_dma_data = NULL; }

    return ret;
}

/* Call this from HAL_SPI_RxCpltCallback() for the SPI instance PMW3901 is on. */
void pmw3901_dma_irq_handler(void)
{
    cs_hi();

    pmw3901_dma_callback_t cb   = s_dma_cb;
    pmw3901_data_t        *data = s_dma_data;
    s_dma_cb   = NULL;
    s_dma_data = NULL;

    if (!cb) return;
    if (data) parse_burst(data, s_dma_raw);
    cb(HAL_OK);
}
