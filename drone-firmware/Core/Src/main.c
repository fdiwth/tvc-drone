/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */

#if 1 // GLOBAL VARIABLES
bmm150_data_t bmm150_data;
bmp388_data_t bmp388_data;
icm42688p_data_t icm42688p_data;
ina226_data_t ina226_data;
FATFS fs;
FIL fil;
FRESULT res;
uint8_t com_receive_buf[128] = {0};
uint8_t com_transmit_buf[128] = {0};
note_t melody[] = {
    {NOTE_C5, EIGHTH}, {NOTE_D5, EIGHTH}, {NOTE_E5, EIGHTH}, {NOTE_G5, EIGHTH},
    {NOTE_E5, EIGHTH}, {NOTE_G5, QUARTER}, {NOTE_C6, HALF},
};
uint8_t melody_len = (sizeof(melody) / sizeof(melody[0]));

FusionAhrs ahrs;
FusionBias bias;
FusionMatrix gyro_misalignment = {{1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}};
FusionVector gyro_sensitivity = {{1.0f, 1.0f, 1.0f}};
FusionVector gyro_offset = {{0.0f, 0.0f, 0.0f}};
FusionMatrix accel_misalignement = {{1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}};
FusionVector accel_sensitivity = {{1.0f, 1.0f, 1.0f}};
FusionVector accel_offset = {{0.0f, 0.0f, 0.0f}};
FusionMatrix mag_softIronMatrix = {{0.940f, 0.041f, 0.0f, 0.041f, 1.027f, 0.006f, 0.0f, 0.006f, 1.047f}};
FusionVector mag_hardIronOffset = {{-7.37f, 3.12f, 16.56f}};

FusionAhrsSettings settings = {
	.convention = FusionConventionNed,
	.gain = 0.5f,
	.gyroscopeRange = 500.0f,
	.accelerationRejection = 8.0f,
	.magneticRejection = 10.0f,
};
FusionVector gyro;
FusionVector accel;
FusionVector mag;
FusionEuler ori;
FusionVector lin_accel;
FusionVector vel = {{0, 0, 0}};
FusionVector pos = {{0, 0, 0}};

kalman_t kf_altitude;
float altitude_offset = 0;

lpf_t lpf_altitude;
lpf_t lpf_voltage;
lpf_t lpf_velx;
lpf_t lpf_vely;

lqr_t lqr;
float u_min[] = {-0.149066f, -0.149066f, -0.1, -HOVER_THRUST};
float u_max[] = { 0.149066f,  0.149066f, 0.1, 0.05f * G};
float x_ref[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float cmd_ref[] = {0.0f, 0.0f, 0.0f};
float pz_integral = 0;
float rx_integral = 0;
float ry_integral = 0;
float rz_integral = 0;
float rz_offset = 0;
float rz_prev_wrapped = 0.0f;
float rz_unwrapped = 0.0f;
volatile bool rz_unwrap_init = false;
volatile bool dt_init = false;

float ref_rx_offset = -4.0f; // hard coded offsets
float ref_ry_offset = 7.0f;  // hard coded offsets
float ref_rz_offset = 0;
float ref_pz_offset = 0;

volatile spi2_pending_t spi2_pending = SPI2_PENDING_NONE;
volatile i2c1_pending_t i2c1_pending = I2C1_PENDING_NONE;

drone_state_t drone_state = DRONE_DISARMED;

uint32_t flash_write_address = 0x000000;
uint32_t flash_max_address = 0x1000000;
#endif // GLOBAL VARIABLES

#if 1 // LOCAL VARIABLES
uint16_t flash_buf_idx = 0;
uint8_t  flash_page_buffer[256];
const float K[LQR_MAX_INPUTS][LQR_MAX_STATES] = {
    {-1.14198845f, 0.00000000f, 0.00000000f, 0.00000000f, -0.29207114f, 0.00000000f, 0.00000000f, 0.00000000f, -0.29505944f, 0.00000000f, 0.00000000f, 0.00000000f},
    {0.00000000f, 1.14198845f, 0.00000000f, 0.00000000f, 0.00000000f, 0.29207114f, 0.00000000f, 0.00000000f, 0.00000000f, 0.29505944f, 0.00000000f, 0.00000000f},
    {0.00000000f, 0.00000000f, 0.06454058f, 0.00000000f, 0.00000000f, 0.00000000f, 0.57378372f, 0.00000000f, 0.00000000f, 0.00000000f, 0.00238777f, 0.00000000f},
	{0.00000000f, 0.00000000f, 0.00000000f, 1.73874813f, 0.00000000f, 0.00000000f, 0.00000000f, 1.23067720f, 0.00000000f, 0.00000000f, 0.00000000f, 0.04408133f},
};
float  Q[] = {0.05f, // acceleration
             0.1f, // velocity
             0.3f}; // position

float R[] = {0.15f, // imu noise
             0.15f, // barometer noise
			 0.005f}; // tof noise


kalman_t kf_vel_x;
kalman_t kf_vel_y;

float Q_vel[] = {0.05f,  // acceleration — reused from kf_altitude's tuning, same physical sensor
                  0.5f}; // velocity

float R_vel[] = {0.15f, // accelerometer noise
                  0.001f}; // optical flow sensor noise

float x0_vel[] = {0.0f, 0.0f};

pid_t vel_pid_x = {
	.kp =  -0.140f,
	.ki = -0.040f,
	.kd = -0.008f,
	.output_min = -0.2f,
	.output_max =  0.2f,
	.integral_min = -2.5f,
	.integral_max =  2.5f,
	.d_filter_alpha = 0.1f,
	.decimation_factor = 5, // division factor from the P and I update loop to the output loop
};
pid_t vel_pid_y = {
	.kp =  0.140f,
	.ki = 0.040f,
	.kd = 0.008f,
	.output_min = -0.2f,
	.output_max =  0.2f,
	.integral_min = -2.5f,
	.integral_max =  2.5f,
	.d_filter_alpha = 0.1f,
	.decimation_factor = 5,
};
pid_t pos_p_x = {
	.kp =  0.5f,
	.output_min = -0.5f,
	.output_max =  0.5f,
	.decimation_factor = 10, // division factor from the P and I update loop to the output loop
};

pid_t pos_p_y = {
	.kp =  0.5f,
	.output_min = -0.5f,
	.output_max =  0.5f,
	.decimation_factor = 10,
};
float x0[] = {0.0f, 0.0f, 0.0f};
#endif // LOCAL VARIABLES

#if 1 // DRIVER CONFIG
pmw3901_data_t pmw3901_data;
pmw3901_config_t pmw3901_conf = {
    .hspi    = &hspi2,
    .cs_port = SPI2_CS2_GPIO_Port,
    .cs_pin  = SPI2_CS2_Pin,
};

uint16_t vl53l1x_dev = 0x52;
servo_config_t servox_conf = {
	.pwm_timer = &htim3,
	.pwm_channel = TIM_CHANNEL_1,
	.offset = 25,
};
servo_config_t servoy_conf = {
	.pwm_timer = &htim3,
	.pwm_channel = TIM_CHANNEL_4,
	.offset = 20,
};
motor_config_t motor1_conf = {
	.pwm_timer = &htim2,
	.pwm_channel = TIM_CHANNEL_1,
};
motor_config_t motor2_conf = {
	.pwm_timer = &htim2,
	.pwm_channel = TIM_CHANNEL_2,
};
bmm150_config_t bmm150_conf = {
	.iface    = BMM150_IFACE_I2C,
	.hi2c     = &hi2c1,
	.i2c_addr = BMM150_I2C_ADDR_ALT3,
	.hspi     = NULL,
	.cs_port  = NULL,
	.cs_pin   = 0,
	.op_ctrl  = BMM150_OPMODE_NORMAL | BMM150_ODR_10HZ,
	.rep_xy   = 0x01U,
	.rep_z    = 0x06U,
};
bmp388_config_t bmp388_conf = {
    .iface    = BMP388_IFACE_SPI,
    .hspi     = &hspi1,
    .cs_port  = SPI1_CS1_GPIO_Port,
    .cs_pin   = SPI1_CS1_Pin,
    .prs_cfg  = BMP388_ODR_50HZ | BMP388_OSR_x8,
    .tmp_cfg  = BMP388_ODR_50HZ | BMP388_OSR_x1,
    .mode     = BMP388_MODE_NORMAL,
    .sea_level_hpa = 1013.25f,
};
icm42688p_config_t icm42688p_conf = {
    .iface         = ICM42688P_IFACE_SPI,
    .hspi          = &hspi2,
    .cs_port       = SPI2_CS1_GPIO_Port,
    .cs_pin        = SPI2_CS1_Pin,
    .hi2c          = NULL,
    .i2c_addr      = 0,
    .gyro_config  = ICM42688P_GYRO_FS_500DPS  | ICM42688P_ODR_1760HZ,
    .accel_config = ICM42688P_ACCEL_FS_8G     | ICM42688P_ODR_1760HZ,
    .acc_scale     = ICM42688P_ACC_SCALE_8G,
    .gyro_scale    = ICM42688P_GYRO_SCALE_500DPS,
};
ina226_config_t ina226_conf = {
	.hi2c          = &hi2c1,
	.i2c_addr      = INA226_I2C_ADDR(0, 0),
	.config_reg    = (INA226_AVG_1 | INA226_VBUS_CT_140US | INA226_VSH_CT_140US | INA226_MODE_SHUNT_BUS_CONT),
	.current_lsb_A = 0.001f,
	.r_shunt_ohm   = 0.002f
};
w25q128_config_t w25q128_conf = {
	.hspi    = &hspi1,
	.cs_port = SPI1_CS2_GPIO_Port,
	.cs_pin  = SPI1_CS2_Pin
};
com_config_t com_conf = {
	.com_method = COM_LORA,
	.m0m1_pin = GPIO_PIN_3,
	.m0m1_port = GPIOB,
	.uart    = &huart2,
	.receive_buf = com_receive_buf,
	.transmit_buf = com_transmit_buf,
	.timeout = 1000,
};
signal_config_t signal_conf = {
	.led_port = GPIOC,
	.led_pin = GPIO_PIN_13,
	.buzzer_htim = &htim1,
	.buzzer_channel = TIM_CHANNEL_3,
};
#endif // DRIVER CONFIG

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t g_log_field_mask = LOG_MASK_ALL;

static uint16_t flight_log_serialize(uint8_t *buf, const flight_log_t *log, uint32_t mask) {
	uint16_t idx = 0;
	memcpy(&buf[idx], &mask, 4); idx += 4;
	memcpy(&buf[idx], &log->timestamp_ms, 4); idx += 4;
	buf[idx++] = log->drone_state;

	if (mask & LOG_FIELD_ACCEL)   { memcpy(&buf[idx], &log->accel, sizeof(FusionVector)); idx += sizeof(FusionVector); }
	if (mask & LOG_FIELD_GYRO)    { memcpy(&buf[idx], &log->gyro,  sizeof(FusionVector)); idx += sizeof(FusionVector); }
	if (mask & LOG_FIELD_MAG)     { memcpy(&buf[idx], &log->mag,   sizeof(FusionVector)); idx += sizeof(FusionVector); }
	if (mask & LOG_FIELD_BARO)    { memcpy(&buf[idx], &log->baro_altitude_m, 4); idx += 4; }
	if (mask & LOG_FIELD_TOF)     { memcpy(&buf[idx], &log->tof_altitude_m,  4); idx += 4; }
	if (mask & LOG_FIELD_FLOW)    { memcpy(&buf[idx], &log->flow_dx, 4); idx += 4;
	                                 memcpy(&buf[idx], &log->flow_dy, 4); idx += 4; }
	if (mask & LOG_FIELD_VOLTAGE) { memcpy(&buf[idx], &log->voltage, 4); idx += 4; }
	if (mask & LOG_FIELD_ORI) {
		memcpy(&buf[idx], &log->rx, 4); idx += 4;
		memcpy(&buf[idx], &log->ry, 4); idx += 4;
		memcpy(&buf[idx], &log->rz, 4); idx += 4;
	}
	if (mask & LOG_FIELD_POS) {
		memcpy(&buf[idx], &log->pos_x, 4); idx += 4;
		memcpy(&buf[idx], &log->pos_y, 4); idx += 4;
		memcpy(&buf[idx], &log->pos_z, 4); idx += 4;
	}
	if (mask & LOG_FIELD_POS_P)   { memcpy(&buf[idx], &log->pos_p_out_x, 4); idx += 4;
	                                 memcpy(&buf[idx], &log->pos_p_out_y, 4); idx += 4; }
	if (mask & LOG_FIELD_VEL_PID) { memcpy(&buf[idx], &log->vel_pid_out_x, 4); idx += 4;
	                                 memcpy(&buf[idx], &log->vel_pid_out_y, 4); idx += 4; }
	if (mask & LOG_FIELD_LQR)     { memcpy(&buf[idx], log->lqr_u, sizeof(log->lqr_u)); idx += sizeof(log->lqr_u); }
	if (mask & LOG_FIELD_SERVO)   { memcpy(&buf[idx], &log->servo_x, 4); idx += 4;
	                                 memcpy(&buf[idx], &log->servo_y, 4); idx += 4; }
	if (mask & LOG_FIELD_MOTOR)   { memcpy(&buf[idx], &log->motor1, 4); idx += 4;
	                                 memcpy(&buf[idx], &log->motor2, 4); idx += 4; }

	return idx;
}

/* Returns false if this record slot is erased/unwritten flash (end of valid
 * data in this page). Otherwise populates *out (fields absent from this
 * record's own mask are left zeroed) and *record_len (bytes consumed). */
static bool flight_log_deserialize(const uint8_t *buf, uint16_t buf_remaining, flight_log_t *out, uint16_t *record_len) {
	if (buf_remaining < 9) return false;

	uint32_t mask;
	memcpy(&mask, &buf[0], 4);
	if (mask == LOG_MASK_ERASED) return false;

	memset(out, 0, sizeof(*out));
	uint16_t idx = 4;
	memcpy(&out->timestamp_ms, &buf[idx], 4); idx += 4;
	out->drone_state = buf[idx++];

	if (mask & LOG_FIELD_ACCEL) { memcpy(&out->accel, &buf[idx], sizeof(FusionVector)); idx += sizeof(FusionVector); }
	if (mask & LOG_FIELD_GYRO)  { memcpy(&out->gyro,  &buf[idx], sizeof(FusionVector)); idx += sizeof(FusionVector); }
	if (mask & LOG_FIELD_MAG)   { memcpy(&out->mag,   &buf[idx], sizeof(FusionVector)); idx += sizeof(FusionVector); }
	if (mask & LOG_FIELD_BARO)  { memcpy(&out->baro_altitude_m, &buf[idx], 4); idx += 4; }
	if (mask & LOG_FIELD_TOF)   { memcpy(&out->tof_altitude_m,  &buf[idx], 4); idx += 4; }
	if (mask & LOG_FIELD_FLOW)  { memcpy(&out->flow_dx, &buf[idx], 4); idx += 4;
	                               memcpy(&out->flow_dy, &buf[idx], 4); idx += 4; }
	if (mask & LOG_FIELD_VOLTAGE) { memcpy(&out->voltage, &buf[idx], 4); idx += 4; }
	if (mask & LOG_FIELD_ORI) {
		memcpy(&out->rx, &buf[idx], 4); idx += 4;
		memcpy(&out->ry, &buf[idx], 4); idx += 4;
		memcpy(&out->rz, &buf[idx], 4); idx += 4;
	}
	if (mask & LOG_FIELD_POS) {
		memcpy(&out->pos_x, &buf[idx], 4); idx += 4;
		memcpy(&out->pos_y, &buf[idx], 4); idx += 4;
		memcpy(&out->pos_z, &buf[idx], 4); idx += 4;
	}
	if (mask & LOG_FIELD_POS_P)   { memcpy(&out->pos_p_out_x, &buf[idx], 4); idx += 4;
	                                 memcpy(&out->pos_p_out_y, &buf[idx], 4); idx += 4; }
	if (mask & LOG_FIELD_VEL_PID) { memcpy(&out->vel_pid_out_x, &buf[idx], 4); idx += 4;
	                                 memcpy(&out->vel_pid_out_y, &buf[idx], 4); idx += 4; }
	if (mask & LOG_FIELD_LQR)     { memcpy(out->lqr_u, &buf[idx], sizeof(out->lqr_u)); idx += sizeof(out->lqr_u); }
	if (mask & LOG_FIELD_SERVO)   { memcpy(&out->servo_x, &buf[idx], 4); idx += 4;
	                                 memcpy(&out->servo_y, &buf[idx], 4); idx += 4; }
	if (mask & LOG_FIELD_MOTOR)   { memcpy(&out->motor1, &buf[idx], 4); idx += 4;
	                                 memcpy(&out->motor2, &buf[idx], 4); idx += 4; }

	*record_len = idx;
	return true;
}

void log_to_flash(flight_log_t *log) {
	if (flash_write_address >= flash_max_address) {
		return;
	}

	uint8_t rec_buf[FLIGHT_LOG_MAX_RECORD_SIZE];
	uint16_t rec_len = flight_log_serialize(rec_buf, log, g_log_field_mask);

	if (flash_buf_idx + rec_len > 256) {
		memset(&flash_page_buffer[flash_buf_idx], 0xFF, 256 - flash_buf_idx);

		w25q128_data_t wdata = { .buf = flash_page_buffer, .len = 256 };
		if (w25q128_write_dma(flash_write_address, &wdata, flash_dma_complete_cb) == HAL_OK) {
			osSemaphoreAcquire(flashDmaSemHandle, W25Q128_TIMEOUT_PAGE_PROGRAM_MS);
		}
		w25q128_wait_busy(W25Q128_TIMEOUT_PAGE_PROGRAM_MS);

		flash_write_address += 256;
		flash_buf_idx = 0;

		if (flash_write_address >= flash_max_address) {
			return;
		}
	}

	memcpy(&flash_page_buffer[flash_buf_idx], rec_buf, rec_len);
	flash_buf_idx += rec_len;
}
uint8_t transfer_logs_to_sd(void) {
	uint32_t read_address = 0x000000;
	uint8_t read_page_buffer[256];
	flight_log_t log_packet;
	char csv_line[320];
	UINT bytes_written;

	if (flash_write_address == 0x000000 && flash_buf_idx == 0) {
		return 0;
	}

	res = f_mount(&fs, "", 1);
	if (res != FR_OK) {
		return 0;
	}

	res = f_open(&fil, "flight.csv", FA_WRITE | FA_CREATE_ALWAYS);
	if (res != FR_OK) {
		f_mount(NULL, "", 0);
		return 0;
	}

	const char* header =
		"timestamp_ms,state,"
		"accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,"
		"baro_alt,tof_alt,flow_dx,flow_dy,voltage,"
		"rx,ry,rz,pos_x,pos_y,pos_z,"
		"pos_p_x,pos_p_y,vel_pid_x,vel_pid_y,"
		"lqr_u0,lqr_u1,lqr_u2,lqr_u3,"
		"servo_x,servo_y,motor1,motor2\n";
	res = f_write(&fil, header, strlen(header), &bytes_written);
	if (res != FR_OK || bytes_written != strlen(header)) {
		f_close(&fil);
		f_mount(NULL, "", 0);
		return 0;
	}

	if (flash_buf_idx > 0) {
		memset(&flash_page_buffer[flash_buf_idx], 0xFF, 256 - flash_buf_idx);

		if (w25q128_write(flash_write_address, flash_page_buffer, 256) != HAL_OK) {
			f_close(&fil);
			f_mount(NULL, "", 0);
			return 0;
		}
		w25q128_wait_busy(W25Q128_TIMEOUT_PAGE_PROGRAM_MS);
		flash_write_address += 256;
	}

	while (read_address < flash_write_address) {
		w25q128_data_t read_data = { .buf = read_page_buffer, .len = 256 };

		if (w25q128_read(read_address, &read_data) != HAL_OK) {
			break;
		}

		uint16_t offset = 0;
		while (offset < 256) {
			uint16_t record_len = 0;
			if (!flight_log_deserialize(&read_page_buffer[offset], 256 - offset, &log_packet, &record_len)) {
				break;   // erased/unwritten from here on within this page
			}

			int len = snprintf(csv_line, sizeof(csv_line),
				"%lu,%u,"
				"%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
				"%.3f,%.3f,%.3f,%.3f,%.3f,"
				"%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,"
				"%.4f,%.4f,%.4f,%.4f,"
				"%.4f,%.4f,%.4f,%.4f,"
				"%.3f,%.3f,%.3f,%.3f\n",
				log_packet.timestamp_ms, log_packet.drone_state,
				log_packet.accel.axis.x, log_packet.accel.axis.y, log_packet.accel.axis.z,
				log_packet.gyro.axis.x,  log_packet.gyro.axis.y,  log_packet.gyro.axis.z,
				log_packet.mag.axis.x,   log_packet.mag.axis.y,   log_packet.mag.axis.z,
				log_packet.baro_altitude_m, log_packet.tof_altitude_m,
				log_packet.flow_dx, log_packet.flow_dy, log_packet.voltage,
				log_packet.rx, log_packet.ry, log_packet.rz,
				log_packet.pos_x, log_packet.pos_y, log_packet.pos_z,
				log_packet.pos_p_out_x, log_packet.pos_p_out_y,
				log_packet.vel_pid_out_x, log_packet.vel_pid_out_y,
				log_packet.lqr_u[0], log_packet.lqr_u[1], log_packet.lqr_u[2], log_packet.lqr_u[3],
				log_packet.servo_x, log_packet.servo_y, log_packet.motor1, log_packet.motor2);

			if (len > 0 && (size_t)len < sizeof(csv_line)) {
				f_write(&fil, csv_line, len, &bytes_written);
			}

			offset += record_len;
		}

		read_address += 256;
	}

	f_close(&fil);
	f_mount(NULL, "", 0);
	return 1;
}


void sys_init() {
	servo_init(&servox_conf);
	servo_init(&servoy_conf);
	motor_init(&motor1_conf);
	motor_init(&motor2_conf);
	com_init(&com_conf);
	com_receive_it_start(&com_conf);
	signal_init(&signal_conf);
	bmm150_init(&bmm150_conf);
	bmp388_init(&bmp388_conf);
	icm42688p_init(&icm42688p_conf);
	ina226_init(&ina226_conf);
	w25q128_init(&w25q128_conf);
	pmw3901_init(&pmw3901_conf);
	VL53L1X_SensorInit(vl53l1x_dev);
	VL53L1X_SetDistanceMode(vl53l1x_dev, 1); // distance mode 1:short, 2:long
	VL53L1X_SetTimingBudgetInMs(vl53l1x_dev, 20);
	VL53L1X_SetInterMeasurementInMs(vl53l1x_dev, 20);
	VL53L1X_StartRanging(vl53l1x_dev);

	FusionAhrsInitialise(&ahrs); // fusion initialization
	FusionAhrsSetSettings(&ahrs, &settings);
	FusionBiasInitialise(&bias);
	FusionBiasSettings bias_settings = fusionBiasDefaultSettings;
	bias_settings.sampleRate = 100;
	FusionBiasSetSettings(&bias, &bias_settings);

	// calibrate accelerometer & gyroscope
	int i = 0;
	int ITERATION = 30;

	FusionVector accel_sum = {0};
	for (i = 0; i < ITERATION; i++) {
		icm42688p_read(&icm42688p_data);
		accel_sum.axis.x += icm42688p_data.accel_x_g;
		accel_sum.axis.y += icm42688p_data.accel_y_g;
		accel_sum.axis.z += icm42688p_data.accel_z_g;
		gyro_offset.axis.x += icm42688p_data.gyro_x_dps;
		gyro_offset.axis.y += icm42688p_data.gyro_y_dps;
		gyro_offset.axis.z += icm42688p_data.gyro_z_dps;
		HAL_Delay(100);
	}

	accel_sum   = FusionVectorScale(accel_sum, 1.0f / ITERATION);

	float accel_mag = sqrtf(accel_sum.axis.x * accel_sum.axis.x + accel_sum.axis.y * accel_sum.axis.y + accel_sum.axis.z * accel_sum.axis.z);
	float scale = 1.0f / accel_mag;

	accel_sensitivity = (FusionVector) {{scale, scale, scale}};
	gyro_offset = FusionVectorScale(gyro_offset, 1.0f / ITERATION);

	const float CONVERGE_DT = 0.01f;
	const int   MAX_CONVERGE_ITERATIONS = 500;

	i = 0;
	while (FusionAhrsGetFlags(&ahrs).startup && (i < MAX_CONVERGE_ITERATIONS)) {
		icm42688p_read(&icm42688p_data);
		bmm150_read(&bmm150_data);

		gyro  = (FusionVector) {{icm42688p_data.gyro_x_dps,  icm42688p_data.gyro_y_dps,  icm42688p_data.gyro_z_dps}};
		accel = (FusionVector) {{icm42688p_data.accel_x_g,   icm42688p_data.accel_y_g,   icm42688p_data.accel_z_g}};
		gyro  = FusionModelInertial(gyro,  gyro_misalignment,  gyro_sensitivity,  gyro_offset);
		accel = FusionModelInertial(accel, accel_misalignement, accel_sensitivity, accel_offset);
		gyro  = FusionBiasUpdate(&bias, gyro);

		mag = (FusionVector) {{bmm150_data.mag_x_uT, bmm150_data.mag_y_uT, bmm150_data.mag_z_uT}};
		mag = FusionModelMagnetic(mag, mag_softIronMatrix, mag_hardIronOffset);

		FusionAhrsUpdate(&ahrs, gyro, accel, mag, CONVERGE_DT);

		HAL_Delay(10);
		i++;
	}

	if (FusionAhrsGetFlags(&ahrs).startup) {
		uint8_t warn_buf[48] = {0};
		strcpy((char*)warn_buf, "warning: ahrs startup did not converge");
		com_send(&com_conf, warn_buf, strlen((char*)warn_buf), 0);
		osDelay(10);
	}

	ori = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

	for (i = 0; i < ITERATION; i++) {
		bmp388_read(&bmp388_data);
		altitude_offset += bmp388_data.altitude_m;
		HAL_Delay(200);
	}

	altitude_offset = altitude_offset / ITERATION;

	rx_integral = 0;
	ry_integral = 0;
	rz_integral = 0;
	pz_integral = 0;

	lpf_init(&lpf_altitude, 0.8f, 0.0f);
	lpf_init(&lpf_voltage, 0.1f, 11.1f);

	kalman_init(&kf_altitude, 3, 3, Q, R, x0); // kalman filter altitude init
	kalman_init(&kf_vel_x, 2, 2, Q_vel, R_vel, x0_vel);
	kalman_init(&kf_vel_y, 2, 2, Q_vel, R_vel, x0_vel);

	lqr_init(&lqr, 12, 4, K, u_min, u_max);
	lqr_set_ref(&lqr, x_ref);

	uint8_t buf[32] = {0};
	strcpy((char*)buf, "initialization complete");
	com_send(&com_conf, buf, strlen((char*)buf), 0);
}

void sys_check(void) {
	int i = 0;
	uint8_t com_buf[64];

	osMutexAcquire(spi1MutexHandle, osWaitForever);
	osMutexAcquire(spi2MutexHandle, osWaitForever);
	osMutexAcquire(i2c1MutexHandle, osWaitForever);

	osThreadSuspend(controlTaskHandle);

	osMutexRelease(i2c1MutexHandle);
	osMutexRelease(spi2MutexHandle);
	osMutexRelease(spi1MutexHandle);

	// init
	strncpy((char*)com_buf, "performing system check\n", sizeof(com_buf));
	com_send(&com_conf, com_buf, strlen((char*)com_buf), UTF_ENC);
	HAL_Delay(1000);

	// led & buzzer test
	strncpy((char*)com_buf, "led & buzzer check\n", sizeof(com_buf));
	com_send(&com_conf, com_buf, strlen((char*)com_buf), UTF_ENC);
	led_on();
	buzzer_play(melody, melody_len);

	// servo-x & servo-y test
	strncpy((char*)com_buf, "servo check\n", sizeof(com_buf));
	com_send(&com_conf, com_buf, strlen((char*)com_buf), UTF_ENC);
	servo_swipe(&servox_conf, 50, 100, 0, 2000);
	servo_swipe(&servoy_conf, 50, 100, 0, 2000);

	// motor1 & motor2 test
	strncpy((char*)com_buf, "motor check\n", sizeof(com_buf));
	com_send(&com_conf, com_buf, strlen((char*)com_buf), UTF_ENC);
	motor_swipe(&motor1_conf, 0, 10, 0, 1000);
	motor_swipe(&motor2_conf, 0, 10, 0, 1000);

	// test bmm150 magnetometer
	strncpy((char*)com_buf, "magnetometer check\n", sizeof(com_buf));
	com_send(&com_conf, com_buf, strlen((char*)com_buf), UTF_ENC);
	for (i = 0; i <=10; i++) {
		bmm150_read(&bmm150_data);
		HAL_Delay(100);
	}

	// test bmp388 barometer
	strncpy((char*)com_buf, "barometer check\n", sizeof(com_buf));
	com_send(&com_conf, com_buf, strlen((char*)com_buf), UTF_ENC);
	for (i = 0; i <=10; i++) {
		bmp388_read(&bmp388_data);
		HAL_Delay(100);
	}

	// test icm42688p inertial measurement unit
	strncpy((char*)com_buf, "inertial measurement unit check\n", sizeof(com_buf));
	com_send(&com_conf, com_buf, strlen((char*)com_buf), UTF_ENC);
	for (i = 0; i <=10; i++) {
		icm42688p_read(&icm42688p_data);
		HAL_Delay(100);
	}

	// test ina226 power sensor
	strncpy((char*)com_buf, "power check\n", sizeof(com_buf));
	com_send(&com_conf, com_buf, strlen((char*)com_buf), UTF_ENC);
	for (i = 0; i <=10; i++) {
		ina226_read(&ina226_data);
		HAL_Delay(100);
	}

	// test w25q128 flash
	strncpy((char*)com_buf, "flash check\n", sizeof(com_buf));
	com_send(&com_conf, com_buf, strlen((char*)com_buf), UTF_ENC);

	uint8_t write_buf[64];
	uint8_t read_buf[64] = {0};
	uint32_t test_addr = 0x000000;

	for(int j = 0; j < 64; j++) write_buf[j] = (uint8_t)(j);

	w25q128_erase(W25Q128_ERASE_SECTOR, test_addr);
	w25q128_wait_busy(W25Q128_TIMEOUT_SECTOR_ERASE_MS);
	w25q128_write(test_addr, write_buf, 64);
	w25q128_data_t read_data = { .buf = read_buf, .len = 64 };
	w25q128_read(test_addr, &read_data);
	HAL_Delay(1000);

	// test sdmmc
	strncpy((char*)com_buf, "sd card check\n", sizeof(com_buf));
	com_send(&com_conf, com_buf, strlen((char*)com_buf), UTF_ENC);

	char buffer[10];
	uint32_t bytes_written;
	res = f_mount(&fs, "", 1);
	res = f_open(&fil, "sys.csv", FA_WRITE | FA_CREATE_ALWAYS);
	for (int k = 0; k <= 63; k++) {
		int len = snprintf(buffer, sizeof(buffer), "%d\n", k);
		f_write(&fil, buffer, len, (UINT *)&bytes_written);
	}
	f_close(&fil);
	f_mount(NULL, "", 0);

	HAL_Delay(1000);
	strncpy((char*)com_buf, "system check complete\n", sizeof(com_buf));
	com_send(&com_conf, com_buf, strlen((char*)com_buf), UTF_ENC);

	HAL_Delay(1000);
	led_off();

	osMutexAcquire(stateMutexHandle, osWaitForever);
	dt_init = false;
	rz_unwrap_init = false;
	osMutexRelease(stateMutexHandle);
	osThreadResume(controlTaskHandle);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SDMMC1_SD_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_FATFS_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        com_rx_callback(&com_conf);
        osSemaphoreRelease(uart2RxSemHandle);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(huart, (uint8_t *)&com_conf.rx_byte, 1);
    }
}

void icm_dma_complete_cb(HAL_StatusTypeDef status)
{
    osSemaphoreRelease(icmDmaSemHandle);
}

void pmw_dma_complete_cb(HAL_StatusTypeDef status)
{
    osSemaphoreRelease(pmwDmaSemHandle);
}

void bmm_dma_complete_cb(HAL_StatusTypeDef status)
{
    osSemaphoreRelease(bmmDmaSemHandle);
}

void bmp_dma_complete_cb(HAL_StatusTypeDef status)
{
    osSemaphoreRelease(bmpDmaSemHandle);
}

void ina_dma_complete_cb(HAL_StatusTypeDef status)
{
    osSemaphoreRelease(inaDmaSemHandle);
}

void flash_dma_complete_cb(HAL_StatusTypeDef status)
{
    osSemaphoreRelease(flashDmaSemHandle);
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        osSemaphoreRelease(uart2TxDmaSemHandle);
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2) {
		if (spi2_pending == SPI2_PENDING_ICM)      icm42688p_dma_irq_handler();
		else if (spi2_pending == SPI2_PENDING_PMW) pmw3901_dma_irq_handler();
		spi2_pending = SPI2_PENDING_NONE;
	} else if (hspi->Instance == SPI1) {
		bmp388_dma_irq_handler();
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        w25q128_dma_irq_handler();
    }
}


void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        if (i2c1_pending == I2C1_PENDING_BMM)       bmm150_dma_irq_handler();
        else if (i2c1_pending == I2C1_PENDING_VL53) osSemaphoreRelease(i2c1DmaSemHandle);
        i2c1_pending = I2C1_PENDING_NONE;
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1 && i2c1_pending == I2C1_PENDING_VL53) {
        osSemaphoreRelease(i2c1DmaSemHandle);
        i2c1_pending = I2C1_PENDING_NONE;
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        ina226_dma_irq_handler();
    }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM6) { // 100Hz
	  static int div50 = 0, div10 = 0, div1 = 0;
	  uint32_t flags = CONTROL_100HZ_FLAG;

	  if (++div50 >= 2)   { flags |= CONTROL_50HZ_FLAG;   div50 = 0; } // 50Hz
	  if (++div10 >= 10)  { flags |= CONTROL_10HZ_FLAG;  div10 = 0; } // 10Hz
	  if (++div1  >= 100) { flags |= CONTROL_1HZ_FLAG; div1  = 0; } // 1Hz

	  osThreadFlagsSet(controlTaskHandle, flags);
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
