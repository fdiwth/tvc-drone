#ifndef GLOBALS_H
#define GLOBALS_H

#include "main.h"
#include "pmw3901.h"
#include "VL53L1X_api.h"
#include "Fusion.h"
#include "bmm150.h"
#include "bmp388.h"
#include "icm42688p.h"
#include "ina226.h"
#include "w25q128.h"
#include "servo.h"
#include "signal.h"
#include "com.h"
#include "motor.h"
#include "kalman.h"
#include "lqr.h"
#include "pid.h"
#include "filter.h"
#include "types.h"
#include "cmsis_os2.h"

extern TIM_HandleTypeDef htim6;

extern servo_config_t servox_conf;
extern servo_config_t servoy_conf;
extern motor_config_t motor1_conf;
extern motor_config_t motor2_conf;
extern com_config_t   com_conf;

extern uint16_t vl53l1x_dev;

extern pmw3901_data_t   pmw3901_data;
extern bmm150_data_t    bmm150_data;
extern bmp388_data_t    bmp388_data;
extern icm42688p_data_t icm42688p_data;
extern ina226_data_t    ina226_data;

extern FusionAhrsSettings settings;
extern FusionAhrs ahrs;
extern FusionBias bias;
extern FusionMatrix gyro_misalignment;
extern FusionVector gyro_sensitivity;
extern FusionVector gyro_offset;
extern FusionMatrix accel_misalignement;
extern FusionVector accel_sensitivity;
extern FusionVector accel_offset;
extern FusionMatrix mag_softIronMatrix;
extern FusionVector mag_hardIronOffset;

extern FusionVector gyro, accel, mag;
extern FusionEuler  ori;
extern FusionVector lin_accel;
extern FusionVector vel, pos;

extern kalman_t kf_altitude;
extern kalman_t kf_vel_x;
extern kalman_t kf_vel_y;
extern float    altitude_offset;
extern lpf_t    lpf_altitude;
extern lpf_t    lpf_voltage;
extern lpf_t    lpf_velx;
extern lpf_t    lpf_vely;

extern lqr_t lqr;
extern pid_t pos_p_x, pos_p_y;
extern pid_t vel_pid_x, vel_pid_y;
extern float rz_offset; // offset for rz
extern float ref_rx_offset, ref_ry_offset, ref_rz_offset, ref_pz_offset; // offset for x_ref
extern float rz_prev_wrapped;
extern float rz_unwrapped;
extern volatile bool rz_unwrap_init;
extern float u_min[];
extern float u_max[];
extern float x_ref[];
extern float cmd_ref[];
extern float pz_integral, rx_integral, ry_integral, rz_integral;
extern volatile bool dt_init;

extern drone_state_t drone_state;

typedef enum { SPI2_PENDING_NONE, SPI2_PENDING_ICM, SPI2_PENDING_PMW } spi2_pending_t;
extern volatile spi2_pending_t spi2_pending;
typedef enum { I2C1_PENDING_NONE, I2C1_PENDING_BMM, I2C1_PENDING_VL53 } i2c1_pending_t;
extern volatile i2c1_pending_t i2c1_pending;

void icm_dma_complete_cb(HAL_StatusTypeDef status);
void pmw_dma_complete_cb(HAL_StatusTypeDef status);
void bmm_dma_complete_cb(HAL_StatusTypeDef status);
void bmp_dma_complete_cb(HAL_StatusTypeDef status);
void ina_dma_complete_cb(HAL_StatusTypeDef status);
void flash_dma_complete_cb(HAL_StatusTypeDef status);

extern osSemaphoreId_t flashDmaSemHandle;
extern osMutexId_t comTxMutexHandle;
extern osMutexId_t stateMutexHandle;
extern osMutexId_t spi1MutexHandle;
extern osMutexId_t spi2MutexHandle;
extern osMutexId_t i2c1MutexHandle;
extern osSemaphoreId_t uart2TxDmaSemHandle;
extern osThreadId_t controlTaskHandle;
extern osSemaphoreId_t uart2RxSemHandle;
extern osSemaphoreId_t i2c1DmaSemHandle;
extern osSemaphoreId_t pmwDmaSemHandle;
extern osSemaphoreId_t icmDmaSemHandle;
extern osSemaphoreId_t bmmDmaSemHandle;
extern osSemaphoreId_t bmpDmaSemHandle;
extern osSemaphoreId_t inaDmaSemHandle;

extern float Q[], R[], x0[];
extern float Q_vel[], R_vel[], x0_vel[];

#define CONTROL_100HZ_FLAG   (0x01u)
#define CONTROL_50HZ_FLAG   (0x02u)
#define CONTROL_10HZ_FLAG   (0x08u)
#define CONTROL_1HZ_FLAG   (0x10u)
#define CONTROL_ALL_FLAGS  (CONTROL_100HZ_FLAG | CONTROL_50HZ_FLAG | CONTROL_10HZ_FLAG | CONTROL_1HZ_FLAG)

extern uint32_t flash_write_address;
extern uint16_t flash_buf_idx;
void log_to_flash(flight_log_t *log);
uint8_t transfer_logs_to_sd(void);

void sys_init();
void sys_check(void);


#define LAND_DESCENT_RATE_MPS   0.3f
#define LAND_ALT_THRESHOLD_M    0.06f
#define LAND_THRUST_THRESHOLD   2.6f
#define LAND_HOLD_MS            500
#define DEFAULT_DT 0.01
#define THRUST_LCLAMP (0.2f * G)
#define TELEMETRY_PAYLOAD_LEN (20 * sizeof(float)) // accelx3 + gyrox3 + magx3 + flowx2 + barntofx2 + powx1 + posx3 + orix3 = 20 = 80 bytes
#define SENSOR_DMA_TIMEOUT_MS 5U
#define G 9.81f
#define NOMINAL_THRUST (0.305f * G)
#define HOVER_THRUST   (0.430f * G)
#define LQR_SXSS 220.4f
#define LQR_SYSS 293.3f
#define LQR_SSO  50
#define TOF_MAX_JUMP_M 0.5f
#define RX_INTEGRAL_MAX 0.4f
#define RY_INTEGRAL_MAX 0.4f
#define RZ_INTEGRAL_MAX 30.0f
#define PZ_INTEGRAL_MAX 15.0f
#define DEG_TO_RAD(deg) ((deg) * (3.14159265358979323846f / 180.0f))
#define THRUST_TO_PWM(voltage, thrust) \
    ((1.22799952f) \
    + (1.22700429f) * (voltage) \
    + (413.74137613f) * (thrust) \
    + (-0.05927199f) * ((voltage) * (voltage)) \
    + (-20.72012511f) * ((voltage) * (thrust)) \
    + (-37.85755474f) * ((thrust)  * (thrust)))

#endif
