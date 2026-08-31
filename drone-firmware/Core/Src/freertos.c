/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "global.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for controlTask */
osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for commandTask */
osThreadId_t commandTaskHandle;
const osThreadAttr_t commandTask_attributes = {
  .name = "commandTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for loggerTask */
osThreadId_t loggerTaskHandle;
const osThreadAttr_t loggerTask_attributes = {
  .name = "loggerTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for telemetryTask */
osThreadId_t telemetryTaskHandle;
const osThreadAttr_t telemetryTask_attributes = {
  .name = "telemetryTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for transferTask */
osThreadId_t transferTaskHandle;
const osThreadAttr_t transferTask_attributes = {
  .name = "transferTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for loggerQueue */
osMessageQueueId_t loggerQueueHandle;
const osMessageQueueAttr_t loggerQueue_attributes = {
  .name = "loggerQueue"
};
/* Definitions for telemetryQueue */
osMessageQueueId_t telemetryQueueHandle;
const osMessageQueueAttr_t telemetryQueue_attributes = {
  .name = "telemetryQueue"
};
/* Definitions for transferQueue */
osMessageQueueId_t transferQueueHandle;
const osMessageQueueAttr_t transferQueue_attributes = {
  .name = "transferQueue"
};
/* Definitions for stateMutex */
osMutexId_t stateMutexHandle;
const osMutexAttr_t stateMutex_attributes = {
  .name = "stateMutex"
};
/* Definitions for spi1Mutex */
osMutexId_t spi1MutexHandle;
const osMutexAttr_t spi1Mutex_attributes = {
  .name = "spi1Mutex"
};
/* Definitions for comTxMutex */
osMutexId_t comTxMutexHandle;
const osMutexAttr_t comTxMutex_attributes = {
  .name = "comTxMutex"
};
/* Definitions for spi2Mutex */
osMutexId_t spi2MutexHandle;
const osMutexAttr_t spi2Mutex_attributes = {
  .name = "spi2Mutex"
};
/* Definitions for i2c1Mutex */
osMutexId_t i2c1MutexHandle;
const osMutexAttr_t i2c1Mutex_attributes = {
  .name = "i2c1Mutex"
};
/* Definitions for icmDmaSem */
osSemaphoreId_t icmDmaSemHandle;
const osSemaphoreAttr_t icmDmaSem_attributes = {
  .name = "icmDmaSem"
};
/* Definitions for bmpDmaSem */
osSemaphoreId_t bmpDmaSemHandle;
const osSemaphoreAttr_t bmpDmaSem_attributes = {
  .name = "bmpDmaSem"
};
/* Definitions for bmmDmaSem */
osSemaphoreId_t bmmDmaSemHandle;
const osSemaphoreAttr_t bmmDmaSem_attributes = {
  .name = "bmmDmaSem"
};
/* Definitions for inaDmaSem */
osSemaphoreId_t inaDmaSemHandle;
const osSemaphoreAttr_t inaDmaSem_attributes = {
  .name = "inaDmaSem"
};
/* Definitions for flashDmaSem */
osSemaphoreId_t flashDmaSemHandle;
const osSemaphoreAttr_t flashDmaSem_attributes = {
  .name = "flashDmaSem"
};
/* Definitions for uart2TxDmaSem */
osSemaphoreId_t uart2TxDmaSemHandle;
const osSemaphoreAttr_t uart2TxDmaSem_attributes = {
  .name = "uart2TxDmaSem"
};
/* Definitions for uart2RxSem */
osSemaphoreId_t uart2RxSemHandle;
const osSemaphoreAttr_t uart2RxSem_attributes = {
  .name = "uart2RxSem"
};
/* Definitions for pmwDmaSem */
osSemaphoreId_t pmwDmaSemHandle;
const osSemaphoreAttr_t pmwDmaSem_attributes = {
  .name = "pmwDmaSem"
};
/* Definitions for i2c1DmaSem */
osSemaphoreId_t i2c1DmaSemHandle;
const osSemaphoreAttr_t i2c1DmaSem_attributes = {
  .name = "i2c1DmaSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ControlTask(void *argument);
void CommandTask(void *argument);
void LoggerTask(void *argument);
void TelemetryTask(void *argument);
void TransferTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

// Set by CMD_LAND in CommandTask, read every cycle in ControlTask. Kept as a
// plain file-scope flag (not routed through global.h) since both tasks that
// touch it live in this file — writes go through stateMutexHandle to match
// the existing convention for cross-task flags like dt_init/rz_unwrap_init.
static volatile bool landing_active = false;

#define LAND_DESCENT_RATE_MPS   0.4f
#define LAND_ALT_THRESHOLD_M    0.10f
#define LAND_THRUST_THRESHOLD   2.6f

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of stateMutex */
  stateMutexHandle = osMutexNew(&stateMutex_attributes);

  /* creation of spi1Mutex */
  spi1MutexHandle = osMutexNew(&spi1Mutex_attributes);

  /* creation of comTxMutex */
  comTxMutexHandle = osMutexNew(&comTxMutex_attributes);

  /* creation of spi2Mutex */
  spi2MutexHandle = osMutexNew(&spi2Mutex_attributes);

  /* creation of i2c1Mutex */
  i2c1MutexHandle = osMutexNew(&i2c1Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of icmDmaSem */
  icmDmaSemHandle = osSemaphoreNew(1, 0, &icmDmaSem_attributes);

  /* creation of bmpDmaSem */
  bmpDmaSemHandle = osSemaphoreNew(1, 0, &bmpDmaSem_attributes);

  /* creation of bmmDmaSem */
  bmmDmaSemHandle = osSemaphoreNew(1, 0, &bmmDmaSem_attributes);

  /* creation of inaDmaSem */
  inaDmaSemHandle = osSemaphoreNew(1, 0, &inaDmaSem_attributes);

  /* creation of flashDmaSem */
  flashDmaSemHandle = osSemaphoreNew(1, 0, &flashDmaSem_attributes);

  /* creation of uart2TxDmaSem */
  uart2TxDmaSemHandle = osSemaphoreNew(1, 0, &uart2TxDmaSem_attributes);

  /* creation of uart2RxSem */
  uart2RxSemHandle = osSemaphoreNew(1, 0, &uart2RxSem_attributes);

  /* creation of pmwDmaSem */
  pmwDmaSemHandle = osSemaphoreNew(1, 0, &pmwDmaSem_attributes);

  /* creation of i2c1DmaSem */
  i2c1DmaSemHandle = osSemaphoreNew(1, 0, &i2c1DmaSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of loggerQueue */
  loggerQueueHandle = osMessageQueueNew (16, sizeof(flight_log_t), &loggerQueue_attributes);

  /* creation of telemetryQueue */
  telemetryQueueHandle = osMessageQueueNew (4, sizeof(telemetry_packet_t), &telemetryQueue_attributes);

  /* creation of transferQueue */
  transferQueueHandle = osMessageQueueNew (1, sizeof(uint8_t), &transferQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of controlTask */
  controlTaskHandle = osThreadNew(ControlTask, NULL, &controlTask_attributes);

  /* creation of commandTask */
  commandTaskHandle = osThreadNew(CommandTask, NULL, &commandTask_attributes);

  /* creation of loggerTask */
  loggerTaskHandle = osThreadNew(LoggerTask, NULL, &loggerTask_attributes);

  /* creation of telemetryTask */
  telemetryTaskHandle = osThreadNew(TelemetryTask, NULL, &telemetryTask_attributes);

  /* creation of transferTask */
  transferTaskHandle = osThreadNew(TransferTask, NULL, &transferTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_ControlTask */
/**
  * @brief  Function implementing the controlTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ControlTask */
void ControlTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN ControlTask */
  sys_init();
//  motor_calibrate_escs(&motor1_conf, &motor2_conf);
  HAL_TIM_Base_Start_IT(&htim6);
  static uint32_t pv_tick_ms = 0;
  static float delta_time;
  static float tof_altitude_m = 0.0f;
  static bool prev_sat_rx = true;
  static bool prev_sat_ry = true;
  static bool prev_sat_velx = true;
  static bool prev_sat_vely = true;
  static float prev_thrust = HOVER_THRUST;
  static float baro_tof_offset = 0.0f;
  static float land_ref_z = 0.0f;
  static bool land_ramp_init = false;
  /* Infinite loop */
  for(;;)
		{
		  uint32_t flags = osThreadFlagsWait(CONTROL_ALL_FLAGS, osFlagsWaitAny, osWaitForever);

		  if ((flags & 0x80000000) != 0) {
			  continue;
		  }

		  bool flag_50hz = (flags & CONTROL_50HZ_FLAG) != 0;
		  bool flag_10hz = (flags & CONTROL_10HZ_FLAG) != 0;
		  bool flag_1hz  = (flags & CONTROL_1HZ_FLAG)  != 0;

		  if (!dt_init) {
			  delta_time = DEFAULT_DT;
			  pv_tick_ms = HAL_GetTick();
			  prev_thrust = HOVER_THRUST;
			  prev_sat_rx = true;
			  prev_sat_ry = true;
			  prev_sat_velx = true;
			  prev_sat_vely = true;
			  tof_altitude_m = 0.0f;
			  baro_tof_offset = altitude_offset;
			  osMutexAcquire(stateMutexHandle, osWaitForever);
			  dt_init = true;
			  osMutexRelease(stateMutexHandle);
		  } else {
			  uint32_t now_ms = HAL_GetTick();
			  delta_time = (float)(now_ms - pv_tick_ms) * 0.001f; // convert to seconds
			  pv_tick_ms = now_ms;
		  }

		  float local_x_ref[12];
		  drone_state_t local_state;
		  int local_servox_offset, local_servoy_offset;
		  osMutexAcquire(stateMutexHandle, osWaitForever);
		  memcpy(local_x_ref, x_ref, sizeof(local_x_ref));
		  local_state = drone_state;
		  local_servox_offset = servox_conf.offset;
		  local_servoy_offset = servoy_conf.offset;
		  osMutexRelease(stateMutexHandle);

	  osMutexAcquire(spi2MutexHandle, osWaitForever);
	  spi2_pending = SPI2_PENDING_ICM;
	  icm42688p_read_dma(&icm42688p_data, icm_dma_complete_cb);
	  osMutexRelease(spi2MutexHandle);
	  if (osSemaphoreAcquire(icmDmaSemHandle, SENSOR_DMA_TIMEOUT_MS) == osOK) {
		  gyro  = (FusionVector) {{icm42688p_data.gyro_x_dps,  icm42688p_data.gyro_y_dps,  icm42688p_data.gyro_z_dps}};
		  accel = (FusionVector) {{icm42688p_data.accel_x_g,   icm42688p_data.accel_y_g,   icm42688p_data.accel_z_g}};
		  gyro  = FusionModelInertial(gyro,  gyro_misalignment,  gyro_sensitivity,  gyro_offset);
		  accel = FusionModelInertial(accel, accel_misalignement, accel_sensitivity, accel_offset);
		  gyro  = FusionBiasUpdate(&bias, gyro);
	  }

	  float flow_dx = 0.0f, flow_dy = 0.0f;
	  osMutexAcquire(spi2MutexHandle, osWaitForever);
	  spi2_pending = SPI2_PENDING_PMW;
	  pmw3901_read_dma(&pmw3901_data, pmw_dma_complete_cb);
	  osMutexRelease(spi2MutexHandle);
	  if (osSemaphoreAcquire(pmwDmaSemHandle, SENSOR_DMA_TIMEOUT_MS) == osOK) {
		  const float PMW_MOUNT_COS45 = 0.70710678f;
		  const float PMW_MOUNT_SIN45 = 0.70710678f;
		  float raw_dx = (float)pmw3901_data.delta_x;
		  float raw_dy = (float)pmw3901_data.delta_y;
		  flow_dx = raw_dx * PMW_MOUNT_COS45 - raw_dy * PMW_MOUNT_SIN45;
		  flow_dy = raw_dx * PMW_MOUNT_SIN45 + raw_dy * PMW_MOUNT_COS45;
	  }

	  osMutexAcquire(i2c1MutexHandle, osWaitForever);
	  ina226_read_dma(&ina226_data, ina_dma_complete_cb);
	  osSemaphoreAcquire(inaDmaSemHandle, SENSOR_DMA_TIMEOUT_MS);
	  osMutexRelease(i2c1MutexHandle);
	  float voltage = lpf_update(&lpf_voltage, ina226_data.bus_voltage_V);

	  if (flag_10hz) {
		  osMutexAcquire(i2c1MutexHandle, osWaitForever);
		  i2c1_pending = I2C1_PENDING_BMM;
		  bmm150_read_dma(&bmm150_data, bmm_dma_complete_cb);
		  bool bmm_ok = (osSemaphoreAcquire(bmmDmaSemHandle, SENSOR_DMA_TIMEOUT_MS) == osOK); // FIX: was unchecked
		  osMutexRelease(i2c1MutexHandle);

		  if (bmm_ok) {
			  mag = (FusionVector) {{bmm150_data.mag_x_uT, bmm150_data.mag_y_uT, bmm150_data.mag_z_uT}};
			  mag = FusionModelMagnetic(mag, mag_softIronMatrix, mag_hardIronOffset);
			  FusionAhrsUpdate(&ahrs, gyro, accel, mag, delta_time);
		  } else {
			  FusionAhrsUpdateNoMagnetometer(&ahrs, gyro, accel, delta_time);
		  }
	  } else {
		  FusionAhrsUpdateNoMagnetometer(&ahrs, gyro, accel, delta_time);
	  }

	  ori = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
	  lin_accel = FusionAhrsGetLinearAcceleration(&ahrs);

	  float rx = DEG_TO_RAD(ori.angle.roll);
	  float ry = DEG_TO_RAD(ori.angle.pitch);

	  bool  tof_valid = false;
	  if (flag_10hz) {
		  osMutexAcquire(i2c1MutexHandle, osWaitForever);
		  uint8_t vlx_ready = 0;
		  VL53L1X_CheckForDataReady(vl53l1x_dev, &vlx_ready);
		  if (vlx_ready) {
		      uint16_t vlx_distance_mm = 0;
		      uint8_t  vlx_status = 0;

		      VL53L1X_GetDistance(vl53l1x_dev, &vlx_distance_mm);
		      VL53L1X_GetRangeStatus(vl53l1x_dev, &vlx_status);
		      VL53L1X_ClearInterrupt(vl53l1x_dev);   // Required every read or the sensor stalls

		      if (vlx_status == 0) {
		          float candidate = (vlx_distance_mm / 1000.0f) * cosf(rx) * cosf(ry);
		          if (candidate > 0.04f && candidate < 4.0f) { // sensor ranges from 4cm to 4m
		              tof_altitude_m = candidate;
		              tof_valid = true;
		          } else {
		              tof_valid = false;
		          }
		      } else {
		          tof_valid = false;
		      }
		  }
		  osMutexRelease(i2c1MutexHandle);
	  }

	  float F[KALMAN_MAX_DIM][KALMAN_MAX_DIM] = {
		  {1,          0,  0},
		  {delta_time,         1,  0},
		  {0.5f*delta_time*delta_time, delta_time, 1}
	  };
	  kalman_predict(&kf_altitude, F, NULL, NULL, delta_time);

	  float H[KALMAN_MAX_MEAS][KALMAN_MAX_DIM] = {0};
	  float z[KALMAN_MAX_MEAS] = {0};
	  H[0][0] = 1.0f;
	  z[0] = -lin_accel.axis.z * 9.81f;

	  bool  bmp_ok = false;
	  float baro_filtered = 0.0f;

	  if (flag_50hz) {
		  osMutexAcquire(spi1MutexHandle, osWaitForever);
		  bmp388_read_dma(&bmp388_data, bmp_dma_complete_cb);
		  bmp_ok = (osSemaphoreAcquire(bmpDmaSemHandle, SENSOR_DMA_TIMEOUT_MS) == osOK); // FIX: was unchecked
		  osMutexRelease(spi1MutexHandle);

		  if (bmp_ok) {
			  baro_filtered = lpf_update(&lpf_altitude, bmp388_data.altitude_m);
		  }
	  }

	  if (tof_valid) {
		  // TOF is the authoritative altitude source in range — use it alone
		  // rather than fusing it alongside the noisier barometer, and keep
		  // recalibrating the barometer's offset against it so the fallback
		  // below picks up from the right point instead of an ARM-time value
		  // that's since drifted.
		  H[2][2] = 1.0f;
		  z[2] = tof_altitude_m;

		  if (bmp_ok) {
			  baro_tof_offset = baro_filtered - tof_altitude_m;
		  }
	  } else if (bmp_ok) {
		  H[1][2] = 1.0f;
		  z[1] = baro_filtered - baro_tof_offset;
	  }

	  kalman_update(&kf_altitude, H, z);
	  vel.axis.z = kalman_get_state(&kf_altitude, 1);
	  pos.axis.z = kalman_get_state(&kf_altitude, 2);

	  float pz = pos.axis.z;

	  const float PMW_RAD_PER_COUNT = 0.0023529f;
	  float flow_height_m = pos.axis.z;
	  const uint8_t PMW_SQUAL_MIN = 25; // surface quality minimum
	  float vx_raw = 0.0f, vy_raw = 0.0f;
	  bool  flow_valid = (pmw3901_data.squal >= PMW_SQUAL_MIN) && (flow_height_m > 0.05f) && (delta_time > 0.0f);

	  float rz_wrapped = DEG_TO_RAD(ori.angle.yaw);

	  if (!rz_unwrap_init) {
		  osMutexAcquire(stateMutexHandle, osWaitForever);
		  rz_offset = rz_wrapped;
		  rz_unwrapped = rz_wrapped;
		  rz_prev_wrapped = rz_wrapped;
		  rz_unwrap_init = true;
		  osMutexRelease(stateMutexHandle);
	  } else {
		  float delta = rz_wrapped - rz_prev_wrapped;
		  if (delta > (float)M_PI)       delta -= 2.0f * (float)M_PI; // wrapping boundary is PI/0.001 radians per second
		  else if (delta < -(float)M_PI) delta += 2.0f * (float)M_PI;
		  osMutexAcquire(stateMutexHandle, osWaitForever);
		  rz_unwrapped += delta;
		  rz_prev_wrapped = rz_wrapped;
		  osMutexRelease(stateMutexHandle);
	  }
	  float rz = rz_unwrapped - rz_offset;


	  float Fx[KALMAN_MAX_DIM][KALMAN_MAX_DIM] = {
		  {1,          0},
		  {delta_time, 1}
	  };
	  kalman_predict(&kf_vel_x, Fx, NULL, NULL, delta_time);

	  float Fy[KALMAN_MAX_DIM][KALMAN_MAX_DIM] = {
		  {1,          0},
		  {delta_time, 1}
	  };
	  kalman_predict(&kf_vel_y, Fy, NULL, NULL, delta_time);

	  float Hx[KALMAN_MAX_MEAS][KALMAN_MAX_DIM] = {0};
	  float zx[KALMAN_MAX_MEAS] = {0};
	  Hx[0][0] = 1.0f;

	  float Hy[KALMAN_MAX_MEAS][KALMAN_MAX_DIM] = {0};
	  float zy[KALMAN_MAX_MEAS] = {0};
	  Hy[0][0] = 1.0f;

	  float world_accel_x = lin_accel.axis.x * cosf(ry) - lin_accel.axis.z * sinf(ry);
	  float world_accel_y = lin_accel.axis.y * cosf(rx) + lin_accel.axis.z * sinf(rx);
	  zx[0] = -world_accel_x * 9.81f;
	  zy[0] = -world_accel_y * 9.81f;

	  if (flow_valid) {
		  float gyro_rad_x = DEG_TO_RAD(gyro.axis.x);
		  float gyro_rad_y = DEG_TO_RAD(gyro.axis.y);
		  float omega_x = (flow_dx * PMW_RAD_PER_COUNT) / delta_time - gyro_rad_y;
		  float omega_y = (flow_dy * PMW_RAD_PER_COUNT) / delta_time + gyro_rad_x;

		  vx_raw = omega_x * flow_height_m;
		  vy_raw = omega_y * flow_height_m;

		  Hx[1][1] = 1.0f;
		  zx[1] = vx_raw;

		  Hy[1][1] = 1.0f;
		  zy[1] = vy_raw;
	  }

	  kalman_update(&kf_vel_x, Hx, zx);
	  kalman_update(&kf_vel_y, Hy, zy);

	  vel.axis.x = kalman_get_state(&kf_vel_x, 1);
	  vel.axis.y = kalman_get_state(&kf_vel_y, 1);
//	  vel.axis.x = vx_raw;
//	  vel.axis.y = vy_raw;

	  if (flow_valid) {
		  float cos_yaw = cosf(rz);
		  float sin_yaw = sinf(rz);
		  float vel_world_x = vel.axis.x * cos_yaw - vel.axis.y * sin_yaw;
		  float vel_world_y = vel.axis.x * sin_yaw + vel.axis.y * cos_yaw;

		  pos.axis.x += delta_time * vel_world_x;
		  pos.axis.y += delta_time * vel_world_y;
	  }
	  osMutexAcquire(stateMutexHandle, osWaitForever);

	  float vx_px_ctrl = 0.0f;
	  float vy_py_ctrl = 0.0f;
	  float rx_vy_ctrl = 0.0f;
	  float ry_vx_ctrl = 0.0f;

	  if (local_state == DRONE_FLYING) {
		  bool hold_integral_x = !prev_sat_ry || !prev_sat_velx;
		  bool hold_integral_y = !prev_sat_rx || !prev_sat_vely;
		  if (pos.axis.z <= 0.3) {
			  vx_px_ctrl = pid_update(&pos_p_x, cmd_ref[0], pos.axis.x, delta_time, hold_integral_x);
			  vy_py_ctrl = pid_update(&pos_p_y, cmd_ref[1], pos.axis.y, delta_time, hold_integral_y);
			  ry_vx_ctrl = pid_update(&vel_pid_x, vx_px_ctrl, vel.axis.x, delta_time, hold_integral_x)
						   * NOMINAL_THRUST / fmaxf(prev_thrust, THRUST_LCLAMP);
			  rx_vy_ctrl = pid_update(&vel_pid_y, vy_py_ctrl, vel.axis.y, delta_time, hold_integral_y)
						   * NOMINAL_THRUST / fmaxf(prev_thrust, THRUST_LCLAMP);

			  bool not_sat_velx = (vel_pid_x.output_held > vel_pid_x.output_min * 0.99f) &&
								   (vel_pid_x.output_held < vel_pid_x.output_max * 0.99f);
			  bool not_sat_vely = (vel_pid_y.output_held > vel_pid_y.output_min * 0.99f) &&
								   (vel_pid_y.output_held < vel_pid_y.output_max * 0.99f);
			  prev_sat_velx = not_sat_velx;
			  prev_sat_vely = not_sat_vely;
		  } else {
			  ry_vx_ctrl = pid_update(&vel_pid_x, vx_px_ctrl, vel.axis.x, delta_time, hold_integral_x)
			  						   * NOMINAL_THRUST / fmaxf(prev_thrust, THRUST_LCLAMP);
			  rx_vy_ctrl = pid_update(&vel_pid_y, vy_py_ctrl, vel.axis.y, delta_time, hold_integral_y)
						   * NOMINAL_THRUST / fmaxf(prev_thrust, THRUST_LCLAMP);

			  ry_vx_ctrl = ry_vx_ctrl*0.1;
			  rx_vy_ctrl = rx_vy_ctrl*0.1;
		  }
	  } else {
		  pid_reset(&pos_p_x);
		  pid_reset(&pos_p_y);
		  pid_reset(&vel_pid_x);
		  pid_reset(&vel_pid_y);
	  }

	  x_ref[0] = rx_vy_ctrl + DEG_TO_RAD(ref_rx_offset);
	  x_ref[1] = ry_vx_ctrl + DEG_TO_RAD(ref_ry_offset);
	  x_ref[2] = DEG_TO_RAD(ref_rz_offset);

	  if (landing_active) {
		  if (!land_ramp_init) {
			  land_ref_z = pz;
			  land_ramp_init = true;
		  }
		  if ((pos.axis.x*pos.axis.x + pos.axis.y*pos.axis.y) < 0.2) {
			  land_ref_z -= LAND_DESCENT_RATE_MPS * delta_time;
			  x_ref[3] = land_ref_z;

		  }
	  } else {
		  land_ramp_init = false;
		  x_ref[3] = cmd_ref[2] + ref_pz_offset;
	  }

	  lqr_set_ref(&lqr, x_ref);
	  memcpy(local_x_ref, x_ref, sizeof(local_x_ref));
	  osMutexRelease(stateMutexHandle);

	  float erx = rx - local_x_ref[0];
	  float ery = ry - local_x_ref[1];
	  float erz = rz - local_x_ref[2];
	  float epz = pz - local_x_ref[3];

	  if (local_state == DRONE_ARMED) {
		  if (flag_50hz) {
			  servo_set(&servox_conf, LQR_SSO);
			  servo_set(&servoy_conf, LQR_SSO);
		  }
		  motor_set(&motor1_conf, 0.0f);
		  motor_set(&motor2_conf, 0.0f);

		  rx_integral = 0; ry_integral = 0; rz_integral = 0; pz_integral = 0;
		  prev_thrust = HOVER_THRUST;
	  } else if (local_state == DRONE_FLYING) {
		  float x_state[] = {
			  rx, ry, rz, pz,
			  DEG_TO_RAD(gyro.axis.x), DEG_TO_RAD(gyro.axis.y), DEG_TO_RAD(gyro.axis.z),
			  vel.axis.z,
			  rx_integral, ry_integral, rz_integral, pz_integral,
		  };

		  lqr_compute(&lqr, x_state);

		  float thrust = HOVER_THRUST + lqr.u[3];
		  float clamped_thrust = fmaxf(thrust, THRUST_LCLAMP);


		  float motor1_cmd = 0;
		  float motor2_cmd = 0;
		  bool near_ground = landing_active && (pz < LAND_ALT_THRESHOLD_M);

		  float servo_x_cmd, servo_y_cmd;
		  if (near_ground) {
			  servo_x_cmd = LQR_SSO;
			  servo_y_cmd = LQR_SSO;
			  motor1_cmd = 0;
			  motor2_cmd = 0;

		  } else {
			  if (thrust > 0.0f) {
				  motor1_cmd = THRUST_TO_PWM(voltage, (thrust / G + lqr.u[2] / 2));
				  motor2_cmd = THRUST_TO_PWM(voltage, (thrust / G - lqr.u[2] / 2));
			  }
			  servo_x_cmd = LQR_SXSS * lqr.u[0] * NOMINAL_THRUST / clamped_thrust + LQR_SSO;
			  servo_y_cmd = LQR_SSO - (LQR_SYSS * lqr.u[1] * NOMINAL_THRUST / clamped_thrust);
		  }

		  if (motor1_cmd <   0.0f) motor1_cmd =   0.0f;
		  if (motor1_cmd > 100.0f) motor1_cmd = 100.0f;
		  if (motor2_cmd <   0.0f) motor2_cmd =   0.0f;
		  if (motor2_cmd > 100.0f) motor2_cmd = 100.0f;

		  motor_set(&motor1_conf, motor1_cmd);
		  motor_set(&motor2_conf, motor2_cmd);


		  servo_set(&servox_conf, servo_x_cmd);
		  servo_set(&servoy_conf, servo_y_cmd);

		  float sat_rx = (lqr.u[0] > u_min[0] * 0.99f) && (lqr.u[0] < u_max[0] * 0.99f);
		  float sat_ry = (lqr.u[1] > u_min[1] * 0.99f) && (lqr.u[1] < u_max[1] * 0.99f);
		  float sat_rz = (lqr.u[2] > u_min[2] * 0.99f) && (lqr.u[2] < u_max[2] * 0.99f);
		  float sat_pz = (lqr.u[3] > u_min[3] * 0.99f) && (lqr.u[3] < u_max[3] * 0.99f);

		  prev_sat_rx = sat_rx;
		  prev_sat_ry = sat_ry;
		  prev_thrust = thrust;

		  if (sat_rx) rx_integral += erx * delta_time;
		  if (sat_ry) ry_integral += ery * delta_time;
		  if (sat_rz) rz_integral += erz * delta_time;
		  if (sat_pz) pz_integral += epz * delta_time;

		  if (rx_integral >  RX_INTEGRAL_MAX) rx_integral =  RX_INTEGRAL_MAX;
		  if (rx_integral < -RX_INTEGRAL_MAX) rx_integral = -RX_INTEGRAL_MAX;
		  if (ry_integral >  RY_INTEGRAL_MAX) ry_integral =  RY_INTEGRAL_MAX;
		  if (ry_integral < -RY_INTEGRAL_MAX) ry_integral = -RY_INTEGRAL_MAX;
		  if (rz_integral >  RZ_INTEGRAL_MAX) rz_integral =  RZ_INTEGRAL_MAX;
		  if (rz_integral < -RZ_INTEGRAL_MAX) rz_integral = -RZ_INTEGRAL_MAX;
		  if (pz_integral >  PZ_INTEGRAL_MAX) pz_integral =  PZ_INTEGRAL_MAX;
		  if (pz_integral < -PZ_INTEGRAL_MAX) pz_integral = -PZ_INTEGRAL_MAX;

		  if (landing_active) {
				  bool thrust_low = (thrust < LAND_THRUST_THRESHOLD);

				  if (near_ground && thrust_low) {
					  motor_set(&motor1_conf, 0.0f);
					  motor_set(&motor2_conf, 0.0f);
					  servo_set(&servox_conf, LQR_SSO);
					  servo_set(&servoy_conf, LQR_SSO);

					  osMutexAcquire(stateMutexHandle, osWaitForever);
					  drone_state = DRONE_DISARMED;
					  landing_active = false;
					  osMutexRelease(stateMutexHandle);

					  land_ramp_init = false;
					  rx_integral = 0; ry_integral = 0; rz_integral = 0; pz_integral = 0;

					  uint8_t land_msg[16];
					  strcpy((char*)land_msg, "LANDED");
					  com_send(&com_conf, land_msg, strlen((char*)land_msg), 0);
				  }
		  }

		  flight_log_t current_log = {
			  .timestamp_ms = HAL_GetTick(),
			  .drone_state  = (uint8_t) local_state,
			  .accel = accel,
			  .gyro  = gyro,
			  .mag   = mag,
			  .baro_altitude_m = bmp388_data.altitude_m - altitude_offset,
			  .tof_altitude_m  = tof_altitude_m,
			  .flow_dx = flow_dx,
			  .flow_dy = flow_dy,
			  .voltage = voltage,
			  .rx = rx, .ry = ry, .rz = rz,
			  .pos_x = pos.axis.x, .pos_y = pos.axis.y, .pos_z = pos.axis.z,
			  .pos_p_out_x = vx_px_ctrl, .pos_p_out_y = vy_py_ctrl,
			  .vel_pid_out_x = ry_vx_ctrl, .vel_pid_out_y = rx_vy_ctrl,
			  .lqr_u = { lqr.u[0], lqr.u[1], lqr.u[2], lqr.u[3] },
			  .servo_x = servo_x_cmd,
			  .servo_y = servo_y_cmd,
			  .motor1  = motor1_cmd,
			  .motor2  = motor2_cmd,
		  };

		  osMessageQueuePut(loggerQueueHandle, &current_log, 0, 0);
	  } else {
		  motor_set(&motor1_conf, 0);
		  motor_set(&motor2_conf, 0);
		  prev_sat_rx = true;
		  prev_sat_ry = true;
	  }

	  if (flag_1hz) {
		  led_toggle();
		  telemetry_packet_t pkt = {
			  .accel = accel, .gyro = gyro, .mag = mag,
			  .flow_dx = flow_dx, .flow_dy = flow_dy,
			  .baro_altitude_m = bmp388_data.altitude_m - altitude_offset,
			  .tof_altitude_m  = tof_altitude_m,
			  .bus_voltage_V   = ina226_data.bus_voltage_V,
			  .pos = pos, .ori = ori,
		  };
		  osMessageQueuePut(telemetryQueueHandle, &pkt, 0, 0);
	  }
	}
  /* USER CODE END ControlTask */
}

/* USER CODE BEGIN Header_CommandTask */
/**
* @brief Function implementing the commandTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CommandTask */
void CommandTask(void *argument)
{
  /* USER CODE BEGIN CommandTask */
	/* Infinite loop */
	for(;;)
	{
	com_cmd_t cmd = {0};
	uint8_t msg[64];
	osSemaphoreAcquire(uart2RxSemHandle, osWaitForever);

	if (!com_rx_parse_utf8(&com_conf, &cmd)) {
		continue;
	} else {
		switch (cmd.type) {
			  case CMD_CHECK: {
				  strcpy((char*)msg, "CHECK");
				  com_send(&com_conf, msg, strlen((char*)msg), 0);
				  osDelay(10);
				  sys_check();

				  osMutexAcquire(stateMutexHandle, osWaitForever);
				  drone_state = DRONE_INIT;
				  osMutexRelease(stateMutexHandle);
				  break;
			  }

			  case CMD_STATUS: {
				  osMutexAcquire(stateMutexHandle, osWaitForever);
				  int len = snprintf((char*)msg, sizeof(msg), "STATUS x_ref: rx=%.3f ry=%.3f rz=%.3f pz=%.3f\n", x_ref[0], x_ref[1], x_ref[2], x_ref[3]);
				  osMutexRelease(stateMutexHandle);
				  com_send(&com_conf, msg, len, UTF_ENC);
				  osDelay(10);
				  break;
			  }

			  case CMD_ARM: {
			      strcpy((char*)msg, "ARM");
			      com_send(&com_conf, msg, strlen((char*)msg), 0);
			      osDelay(10);
			      strncpy((char*)msg, "clearing flash logs...\n", sizeof(msg));
			      com_send(&com_conf, msg, strlen((char*)msg), UTF_ENC);
			      osMutexAcquire(spi1MutexHandle, osWaitForever);
			      osMutexAcquire(spi2MutexHandle, osWaitForever);
			      osMutexAcquire(i2c1MutexHandle, osWaitForever);

			      osThreadSuspend(controlTaskHandle);

			      for (uint32_t addr = 0; addr < 0x100000; addr += 4096) {
			          w25q128_erase(W25Q128_ERASE_SECTOR, addr);
			          w25q128_wait_busy(W25Q128_TIMEOUT_SECTOR_ERASE_MS);
			      }

			      int ITERATION = 30;
			      float altitude_sum = 0;
			      for (int i = 0; i < ITERATION; i++) {
					bmp388_read(&bmp388_data);
					altitude_sum += bmp388_data.altitude_m;
					HAL_Delay(200);
			      }

			      osMutexAcquire(stateMutexHandle, osWaitForever);
			      x_ref[0] = DEG_TO_RAD(ref_rx_offset);
				  x_ref[1] = DEG_TO_RAD(ref_ry_offset);
				  x_ref[2] = DEG_TO_RAD(ref_rz_offset);
				  x_ref[3] = ref_pz_offset;
				  pos.axis.x = 0;
				  pos.axis.y = 0;
				  altitude_offset = altitude_sum / ITERATION;
				  lqr_set_ref(&lqr, x_ref);
				  pid_reset(&vel_pid_x);
				  pid_reset(&vel_pid_y);
				  rz_unwrap_init = false;
				  dt_init = false;

				  VL53L1X_SensorInit(vl53l1x_dev);
				  VL53L1X_SetDistanceMode(vl53l1x_dev, 1); // distance mode 1:short, 2:long
				  VL53L1X_SetTimingBudgetInMs(vl53l1x_dev, 20);
				  VL53L1X_SetInterMeasurementInMs(vl53l1x_dev, 20);
				  VL53L1X_StartRanging(vl53l1x_dev);

				  kalman_init(&kf_altitude, 3, 3, Q, R, x0);
				  kalman_init(&kf_vel_x, 2, 2, Q_vel, R_vel, x0_vel);
				  kalman_init(&kf_vel_y, 2, 2, Q_vel, R_vel, x0_vel);

				  drone_state = DRONE_ARMED;

				  osMutexRelease(stateMutexHandle);

			      osThreadResume(controlTaskHandle);

			      osMutexRelease(i2c1MutexHandle);
			      osMutexRelease(spi2MutexHandle);
			      osMutexRelease(spi1MutexHandle);

			      flash_write_address = 0x000000;
			      flash_buf_idx = 0;

			      strcpy((char*)msg, "armed!");
			      com_send(&com_conf, msg, strlen((char*)msg), 0);
			      break;
			  }

			  case CMD_LAUNCH: {
				  strcpy((char*)msg, "LAUNCH");
				  com_send(&com_conf, msg, strlen((char*)msg), 0);

				  osMutexAcquire(stateMutexHandle, osWaitForever);
				  pid_reset(&vel_pid_x);
				  pid_reset(&vel_pid_y);
				  if (drone_state == DRONE_ARMED) drone_state = DRONE_FLYING;
				  osMutexRelease(stateMutexHandle);
				  break;
			  }

			  case CMD_LAND: {
				  strcpy((char*)msg, "LAND");
				  com_send(&com_conf, msg, strlen((char*)msg), 0);

				  osMutexAcquire(stateMutexHandle, osWaitForever);
				  if (drone_state == DRONE_FLYING) {
					  landing_active = true;
				  }
				  osMutexRelease(stateMutexHandle);
				  break;
			  }

			  case CMD_KILL: {
			      osThreadSuspend(controlTaskHandle);
			      osThreadSuspend(loggerTaskHandle);
			      osThreadSuspend(telemetryTaskHandle);
			      osThreadSuspend(transferTaskHandle);

			      motor_set(&motor1_conf, 0);
			      motor_set(&motor2_conf, 0);
			      servo_set(&servox_conf, LQR_SSO);
			      servo_set(&servoy_conf, LQR_SSO);

			      strcpy((char*)msg, "KILL");
			      com_send(&com_conf, msg, strlen((char*)msg), 0);
			      osDelay(10);

			      osMutexAcquire(stateMutexHandle, osWaitForever);
			      drone_state = DRONE_DISARMED;
			      landing_active = false;
			      osMutexRelease(stateMutexHandle);

			      strncpy((char*)msg, "writing flight.csv to SD...\n", sizeof(msg));
			      com_send(&com_conf, msg, strlen((char*)msg), UTF_ENC);
			      osDelay(10);

			      osThreadResume(controlTaskHandle);
			      osThreadResume(loggerTaskHandle);
			      osThreadResume(telemetryTaskHandle);
			      osThreadResume(transferTaskHandle);

				  motor_set(&motor1_conf, 0);
				  motor_set(&motor2_conf, 0);
				  servo_set(&servox_conf, LQR_SSO);
				  servo_set(&servoy_conf, LQR_SSO);

			      uint8_t trigger = 1;
			      osMessageQueuePut(transferQueueHandle, &trigger, 0, 0);
			      break;
			  }

			  case CMD_DATA: {
				  float new_cmd_ref[3] = {0};
				  osMutexAcquire(stateMutexHandle, osWaitForever);
				  new_cmd_ref[0] = cmd.cmd0; // m
				  new_cmd_ref[1] = cmd.cmd1; // m
				  new_cmd_ref[2] = cmd.cmd2; // m
				  memcpy(cmd_ref, new_cmd_ref, sizeof(new_cmd_ref));
				  osMutexRelease(stateMutexHandle);
				  break;
			  }

			  case CMD_TRIM: {
				  osMutexAcquire(stateMutexHandle, osWaitForever);
				  if (cmd.trim_x == cmd.trim_x) servox_conf.offset = (int) cmd.trim_x;
				  if (cmd.trim_y == cmd.trim_y) servoy_conf.offset = (int) cmd.trim_y;
				  osMutexRelease(stateMutexHandle);

				  servo_set(&servox_conf, LQR_SSO);
				  servo_set(&servoy_conf, LQR_SSO);

				  int len = snprintf((char*)msg, sizeof(msg), "TRIM: x=%d y=%d\n", servox_conf.offset, servoy_conf.offset);
				  com_send(&com_conf, msg, len, UTF_ENC);
				  osDelay(10);
				  break;
			  }

			  case CMD_PIDP: {
				  osMutexAcquire(stateMutexHandle, osWaitForever);
				  if (cmd.vkp == cmd.vkp) {
					  vel_pid_x.kp = -cmd.vkp;
					  vel_pid_y.kp = cmd.vkp;
				  }
				  if (cmd.vki == cmd.vki) {
					  vel_pid_x.ki = -cmd.vki;
					  vel_pid_y.ki = cmd.vki;
				  }
				  if (cmd.vkd == cmd.vkd) {
					  vel_pid_x.kd = -cmd.vkd;
					  vel_pid_y.kd = cmd.vkd;
				  }
				  if (cmd.pkp == cmd.pkp) {
					  pos_p_x.kp = cmd.pkp;
					  pos_p_y.kp = cmd.pkp;
				  }
				  pid_reset(&vel_pid_x);
				  pid_reset(&vel_pid_y);

				  osMutexRelease(stateMutexHandle);

				  int len = snprintf((char*)msg, sizeof(msg), "PID: vkp=%.3f vki=%.3f vkd=%.3f pkp=%.3f\n", vel_pid_y.kp, vel_pid_y.ki, vel_pid_y.kd, pos_p_x.kp);
				  com_send(&com_conf, msg, len, UTF_ENC);
				  osDelay(10);
				  break;
			  }

			  case CMD_OFFSET: {
				  osMutexAcquire(stateMutexHandle, osWaitForever);
				  if (cmd.offset_rx == cmd.offset_rx) ref_rx_offset = cmd.offset_rx;
				  if (cmd.offset_ry == cmd.offset_ry) ref_ry_offset = cmd.offset_ry;
				  if (cmd.offset_rz == cmd.offset_rz) ref_rz_offset = cmd.offset_rz;
				  if (cmd.offset_pz == cmd.offset_pz) ref_pz_offset = cmd.offset_pz;

				  int len = snprintf((char*)msg, sizeof(msg), "OFFSET: rx=%.3f ry=%.3f rz=%.2f pz=%.3f\n", ref_rx_offset, ref_ry_offset, ref_rz_offset, ref_pz_offset);
				  osMutexRelease(stateMutexHandle);

				  com_send(&com_conf, msg, len, UTF_ENC);
				  osDelay(10);
				  break;
			  }

			  default:
				  break;
			}
	}
	}
  /* USER CODE END CommandTask */
}

/* USER CODE BEGIN Header_LoggerTask */
/**
* @brief Function implementing the loggerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LoggerTask */
void LoggerTask(void *argument)
{
  /* USER CODE BEGIN LoggerTask */
	/* Infinite loop */
	for(;;)
	{
	flight_log_t entry;

	if (osMessageQueueGet(loggerQueueHandle, &entry, NULL, osWaitForever) == osOK) {
	  osMutexAcquire(spi1MutexHandle, osWaitForever);
	  log_to_flash(&entry);
	  osMutexRelease(spi1MutexHandle);
	}
	}
  /* USER CODE END LoggerTask */
}

/* USER CODE BEGIN Header_TelemetryTask */
/**
* @brief Function implementing the telemetryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TelemetryTask */
void TelemetryTask(void *argument)
{
  /* USER CODE BEGIN TelemetryTask */

	telemetry_packet_t pkt;
	uint8_t buf[TELEMETRY_PAYLOAD_LEN];
	/* Infinite loop */
	for(;;)
	{
		if (osMessageQueueGet(telemetryQueueHandle, &pkt, NULL, osWaitForever) == osOK) {
		  uint8_t idx = 0;
		  memcpy(buf + idx, &pkt.accel.axis.x, 4); idx += 4;
		  memcpy(buf + idx, &pkt.accel.axis.y, 4); idx += 4;
		  memcpy(buf + idx, &pkt.accel.axis.z, 4); idx += 4;
		  memcpy(buf + idx, &pkt.gyro.axis.x,  4); idx += 4;
		  memcpy(buf + idx, &pkt.gyro.axis.y,  4); idx += 4;
		  memcpy(buf + idx, &pkt.gyro.axis.z,  4); idx += 4;
		  memcpy(buf + idx, &pkt.mag.axis.x,   4); idx += 4;
		  memcpy(buf + idx, &pkt.mag.axis.y,   4); idx += 4;
		  memcpy(buf + idx, &pkt.mag.axis.z,   4); idx += 4;
		  memcpy(buf + idx, &pkt.flow_dx, 4); idx += 4;
		  memcpy(buf + idx, &pkt.flow_dy, 4); idx += 4;
		  memcpy(buf + idx, &pkt.baro_altitude_m, 4); idx += 4;
		  memcpy(buf + idx, &pkt.tof_altitude_m,  4); idx += 4;
		  memcpy(buf + idx, &pkt.bus_voltage_V,   4); idx += 4;
		  memcpy(buf + idx, &pkt.pos.axis.x, 4); idx += 4;
		  memcpy(buf + idx, &pkt.pos.axis.y, 4); idx += 4;
		  memcpy(buf + idx, &pkt.pos.axis.z, 4); idx += 4;
		  memcpy(buf + idx, &pkt.ori.angle.roll,  4); idx += 4;
		  memcpy(buf + idx, &pkt.ori.angle.pitch, 4); idx += 4;
		  memcpy(buf + idx, &pkt.ori.angle.yaw,   4); idx += 4;

		  com_send(&com_conf, buf, idx, ACCEL_ENC | GYRO_ENC | MAG_ENC | FLOW_ENC | BARNTOF_ENC | POW_ENC | POS_ENC | ORI_ENC);
		}
	}
  /* USER CODE END TelemetryTask */
}

/* USER CODE BEGIN Header_TransferTask */
/**
* @brief Function implementing the transferTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TransferTask */
void TransferTask(void *argument)
{
  /* USER CODE BEGIN TransferTask */
  uint8_t trigger;
  for (;;)
	{
	  if (osMessageQueueGet(transferQueueHandle, &trigger, NULL, osWaitForever) == osOK) {

		osMutexAcquire(spi1MutexHandle, osWaitForever);

		osThreadSuspend(controlTaskHandle);
		osThreadSuspend(commandTaskHandle);
		osThreadSuspend(loggerTaskHandle);
		osThreadSuspend(telemetryTaskHandle);

		uint8_t success = transfer_logs_to_sd();

		osMutexRelease(spi1MutexHandle);

		// Reset the time delta flag to prevent a massive delta_time calculation
		// on the first loop after resuming the control task.
		osMutexAcquire(stateMutexHandle, osWaitForever);
		dt_init = false;
		rz_unwrap_init = false;
		osMutexRelease(stateMutexHandle);

		osThreadResume(controlTaskHandle);
		osThreadResume(commandTaskHandle);
		osThreadResume(loggerTaskHandle);
		osThreadResume(telemetryTaskHandle);

		uint8_t msg[32];
		uint8_t len = snprintf((char*)msg, sizeof(msg), success ? "write complete!\n" : "write FAILED!\n");
		com_send(&com_conf, msg, len, UTF_ENC);
	  }
	}
  /* USER CODE END TransferTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
