#ifndef COM_H
#define COM_H

#include "stm32f7xx_hal.h"
#include "usbd_core.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdlib.h>

// all data sent through UART will be structured with a leading command byte containing various flags

// 1000---- // telemetry: sync
// 1100---- // drone: sync
// 1110---- or 1000****// telemetry: confirm // check with internal timer
// if 1110---- then 1111---- or 00000000 // drone: confirm // check with internal timer
// if all steps are completed, set telemetry to listening mode, finish the drone's init sequence.
// begin every data transmit with 1111----

// second byte is the encoding byte containing flags for how the data being sent should be interpreted
// first bit: acceleration x, y, z (float): total 12 bytes
// second bit: gyroscope x, y, z (float): total 12 bytes
// third bit: magnetometer x, y, z (float): total 12 bytes
// fourth bit: relative gps x, y, z (float): total 12 bytes
// fifth bit: barometer (float): 4 bytes
// sixth bit: power voltage (float): 4 bytes
// seventh bit: filtered position x, y, z (float): total 12 bytes
// eighth bit: filtered orientation x, y, z (float): total 12 bytes
// when each bit is flagged as 1, it means that the incoming data should be interpreted as pure binary with the data bytes in order of the flags
// when all 8 bits are 0, the incoming data should be interpreted as utf-8 as it contains string data.

#define ACCEL_ENC   0b10000000
#define GYRO_ENC    0b01000000
#define MAG_ENC     0b00100000
#define FLOW_ENC    0b00010000
#define BARNTOF_ENC 0b00001000
#define POW_ENC     0b00000100
#define POS_ENC     0b00000010
#define ORI_ENC     0b00000001
#define UTF_ENC     0b00000000

typedef enum {
    COM_USB,
    COM_LORA,
} com_method_t;

typedef enum {
    STATUS_TSDN = 0b10000000, // telemetry sync drone null
    STATUS_TSDS = 0b11000000, // telemetry sync drone sync
    STATUS_TCDS = 0b11100000, // telemetry confirm drone sync
    STATUS_TCDC = 0b11110000, // telemetry confirm drone confirm
    STATUS_NULL = 0b00000000, // NULL
} com_status_t;

#define COM_RX_MAX_DATA  128

typedef struct {
    com_method_t        com_method;
    GPIO_TypeDef       *m0m1_port;
    uint16_t            m0m1_pin;
    UART_HandleTypeDef *uart;
    uint8_t            *receive_buf;
    uint8_t            *transmit_buf;
    uint32_t            timeout;

    // Interrupt receive state — do not modify directly
    // Replace the volatile interrupt fields in com_config_t with:
    volatile uint8_t        rx_byte;
    volatile uint8_t        rx_len;
    volatile uint8_t        rx_idx;
    volatile uint8_t        rx_data[COM_RX_MAX_DATA];
    volatile uint8_t        rx_ready;
} com_config_t;

typedef enum {
    CMD_NONE = 0,
    CMD_CHECK,
    CMD_ARM,
	CMD_TRIM,
	CMD_STATUS,
	CMD_OFFSET,
    CMD_LAUNCH,
    CMD_KILL,
    CMD_LAND,
	CMD_PIDP,
    CMD_DATA,
} com_cmd_type_t;

typedef struct {
    com_cmd_type_t type;
    float cmd0, cmd1, cmd2;
    float vkp, vki, vkd, pkp;
    float trim_x, trim_y;
    float offset_rx, offset_ry, offset_rz, offset_pz;
} com_cmd_t;

// Core functions — unchanged from original
void    com_init    (com_config_t *config);
void    com_send    (com_config_t *config, uint8_t *buf,
                     uint32_t len, uint8_t encoding);
uint8_t com_receive(com_config_t *config, uint8_t *buf, uint32_t len);

// Interrupt-driven receive functions
void    com_receive_it_start (com_config_t *config);
void    com_rx_callback      (com_config_t *config);
uint8_t com_rx_parse_utf8(com_config_t *config, com_cmd_t *cmd);

#endif
