#include "com.h"
#include "usbd_core.h"
#include "cmsis_os.h"

extern osMutexId_t     comTxMutexHandle;
extern osSemaphoreId_t uart2TxDmaSemHandle;

com_status_t com_status  = STATUS_NULL;
uint8_t      signature;
uint32_t     latency      = 1000;
uint32_t     current_time;

void com_init(com_config_t *config) {
    config->rx_idx      = 0;
    config->rx_ready    = 0;
    config->rx_len      = 0;

    if (config->com_method == COM_LORA) {
        HAL_GPIO_WritePin(config->m0m1_port, config->m0m1_pin, GPIO_PIN_RESET);
        while (com_status != STATUS_TCDC) {
            if (HAL_UART_Receive(config->uart, config->receive_buf, 1, config->timeout) == HAL_OK) {

                if ((config->receive_buf[0] & 0b11110000) == STATUS_TSDN) {
                    com_status              = STATUS_TSDS;
                    signature               = config->receive_buf[0] & 0b00001111;
                    config->transmit_buf[0] = com_status | signature;
                    HAL_UART_Transmit(config->uart, config->transmit_buf,
                                      1, config->timeout);
                    current_time = HAL_GetTick();
                }

                if (((config->receive_buf[0] & 0b11110000) == STATUS_TCDS) &&
                    (com_status == STATUS_TSDS)) {
                    if (((HAL_GetTick() - current_time) < latency) &&
                        ((config->receive_buf[0] & 0b00001111) == signature)) {
                        com_status              = STATUS_TCDC;
                        config->transmit_buf[0] = com_status | signature;
                        HAL_UART_Transmit(config->uart, config->transmit_buf,
                                          1, config->timeout);
                    } else {
                        config->transmit_buf[0] = STATUS_NULL;
                        HAL_UART_Transmit(config->uart, config->transmit_buf,
                                          1, config->timeout);
                    }
                }
            }
        }
    } else if (config->com_method == COM_USB) {
        while (com_status != STATUS_TCDC) {
            if (CDC_Read_FS(config->receive_buf, 1) == 0) {

                if ((config->receive_buf[0] & 0b11110000) == STATUS_TSDN) {
                    com_status              = STATUS_TSDS;
                    signature               = config->receive_buf[0] & 0b00001111;
                    config->transmit_buf[0] = com_status | signature;
                    CDC_Transmit_FS(config->transmit_buf, 1);
                    current_time = HAL_GetTick();
                }

                if (((config->receive_buf[0] & 0b11110000) == STATUS_TCDS) &&
                    (com_status == STATUS_TSDS)) {
                    if (((HAL_GetTick() - current_time) < latency) &&
                        ((config->receive_buf[0] & 0b00001111) == signature)) {
                        com_status              = STATUS_TCDC;
                        config->transmit_buf[0] = com_status | signature;
                        CDC_Transmit_FS(config->transmit_buf, 1);
                    } else {
                        config->transmit_buf[0] = STATUS_NULL;
                        CDC_Transmit_FS(config->transmit_buf, 1);
                    }
                }
            }
        }
    }
}

void com_send(com_config_t *config, uint8_t *buf,
              uint32_t len, uint8_t encoding) {
    osMutexAcquire(comTxMutexHandle, osWaitForever);

    config->transmit_buf[0] = com_status | signature;
    config->transmit_buf[1] = encoding;
    config->transmit_buf[2] = (uint8_t)len;
    memcpy(config->transmit_buf + 3, buf, len);

    if (config->com_method == COM_LORA) {
        if (HAL_UART_Transmit_DMA(config->uart, config->transmit_buf, len + 3) == HAL_OK) {
            osSemaphoreAcquire(uart2TxDmaSemHandle, config->timeout);
        }
    } else if (config->com_method == COM_USB) {
        CDC_Transmit_FS(config->transmit_buf, len + 3);
    }

    osMutexRelease(comTxMutexHandle);
}

uint8_t com_receive(com_config_t *config, uint8_t *buf, uint32_t len) {
    uint8_t  byte;
    uint32_t idx = 0;

    while (idx < len - 1) {
        uint8_t ret;

        if (config->com_method == COM_LORA) {
            ret = HAL_UART_Receive(config->uart, &byte, 1, config->timeout);
        } else if (config->com_method == COM_USB) {
            ret = CDC_Read_FS(&byte, 1);
        } else {
            break;
        }

        if (ret != HAL_OK) break;   // timeout or error

        if (byte == '\n') break;    // end of message

        buf[idx++] = byte;
    }

    buf[idx] = '\0';    // null terminate
    return idx;         // return number of bytes received
}

void com_receive_it_start(com_config_t *config) {
    if (config->com_method == COM_LORA) {
        config->rx_idx   = 0;
        config->rx_ready = 0;
        HAL_UART_Receive_IT(config->uart,
                            (uint8_t *)&config->rx_byte, 1);
    }
}

void com_rx_callback(com_config_t *config) {
    if (config->com_method != COM_LORA) return;

    uint8_t byte = config->rx_byte;

    if (byte == '\n') {
        // Newline = end of message
        config->rx_data[config->rx_idx] = '\0';
        config->rx_len   = config->rx_idx;
        config->rx_idx   = 0;
        config->rx_ready = 1;
    } else if (config->rx_idx < COM_RX_MAX_DATA - 1) {
        config->rx_data[config->rx_idx++] = byte;
    } else {
        // Buffer overflow — reset
        config->rx_idx = 0;
    }

    // Re-arm for next byte
    HAL_UART_Receive_IT(config->uart,
                        (uint8_t *)&config->rx_byte, 1);
}

static char *parse_field_or_wild(char *ptr, float *out) {
    if (*ptr == '*') {
        *out = __builtin_nanf("");
        return ptr + 1;
    }
    char *end;
    float v = strtof(ptr, &end);
    if (end == ptr) return NULL;
    *out = v;
    return end;
}

uint8_t com_rx_parse_utf8(com_config_t *config, com_cmd_t *cmd) {
    if (!config->rx_ready) return 0;

    char str[COM_RX_MAX_DATA + 1];
    uint8_t len = config->rx_len < COM_RX_MAX_DATA
                  ? config->rx_len : COM_RX_MAX_DATA;
    for (uint8_t i = 0; i < len; i++)
        str[i] = (char)config->rx_data[i];
    str[len] = '\0';

    config->rx_ready = 0;

    for (int i = len - 1; i >= 0; i--) {
        if (str[i] == '\r' || str[i] == '\n') str[i] = '\0';
        else break;
    }

    if (strcmp(str, "CHECK")   == 0) { cmd->type = CMD_CHECK;   return 1; }
    if (strcmp(str, "STATUS")   == 0) { cmd->type = CMD_STATUS;   return 1; }
    if (strcmp(str, "ARM")    == 0) { cmd->type = CMD_ARM;    return 1; }
    if (strcmp(str, "LAUNCH") == 0) { cmd->type = CMD_LAUNCH; return 1; }
    if (strcmp(str, "KILL")   == 0) { cmd->type = CMD_KILL;   return 1; }
    if (strcmp(str, "LAND")   == 0) { cmd->type = CMD_LAND;   return 1; }
    if (strncmp(str, "TRIM:", 5) == 0) {
		char *ptr = str + 5;
		char *end;

		float trim_x, trim_y;

		end = parse_field_or_wild(ptr, &trim_x);
		if (end == NULL || *end != ',') return 0;
		ptr = end + 1;

		end = parse_field_or_wild(ptr, &trim_y);
		if (end == NULL) return 0;

		// NAN (wildcard) fields skip the clamp — comparisons against NAN
		// are always false, so this naturally leaves them untouched.
		const float TRIM_MAX = 50.0f;
		if (trim_x >  TRIM_MAX) trim_x =  TRIM_MAX;
		if (trim_x < -TRIM_MAX) trim_x = -TRIM_MAX;
		if (trim_y >  TRIM_MAX) trim_y =  TRIM_MAX;
		if (trim_y < -TRIM_MAX) trim_y = -TRIM_MAX;

		cmd->type = CMD_TRIM;
		cmd->trim_x = trim_x;
		cmd->trim_y = trim_y;
		return 1;
	}

    if (strncmp(str, "PIDP:", 5) == 0) {
    		char *ptr = str + 5;
    		char *end;

    		float vkp, vki, vkd, pkp; // velocity kp, ki, kd and position kp

    		end = parse_field_or_wild(ptr, &vkp);
    		if (end == NULL || *end != ',') return 0;
    		ptr = end + 1;

    		end = parse_field_or_wild(ptr, &vki);
			if (end == NULL || *end != ',') return 0;
			ptr = end + 1;

			end = parse_field_or_wild(ptr, &vkd);
			if (end == NULL || *end != ',') return 0;
			ptr = end + 1;

    		end = parse_field_or_wild(ptr, &pkp);
    		if (end == NULL) return 0;

    		cmd->type = CMD_PIDP;
    		cmd->vkp = vkp;
    		cmd->vki = vki;
    		cmd->vkd = vkd;
    		cmd->pkp = pkp;
    		return 1;
    	}

	if (strncmp(str, "OFFSET:", 7) == 0) {
		char *ptr = str + 7;
		char *end;

		float offset_rx, offset_ry, offset_rz, offset_pz;

		end = parse_field_or_wild(ptr, &offset_rx);
		if (end == NULL || *end != ',') return 0;
		ptr = end + 1;

		end = parse_field_or_wild(ptr, &offset_ry);
		if (end == NULL || *end != ',') return 0;
		ptr = end + 1;

		end = parse_field_or_wild(ptr, &offset_rz);
		if (end == NULL || *end != ',') return 0;
		ptr = end + 1;

		end = parse_field_or_wild(ptr, &offset_pz);
		if (end == NULL) return 0;

		cmd->type = CMD_OFFSET;
		cmd->offset_rx = offset_rx;
		cmd->offset_ry = offset_ry;
		cmd->offset_pz = offset_pz;
		return 1;
	}

    float cmd0, cmd1, cmd2;
    char *ptr = str;
    char *end;

    cmd0 = strtof(ptr, &end);
    if (end == ptr || *end != ',') return 0;
    ptr = end + 1;

    cmd1 = strtof(ptr, &end);
    if (end == ptr || *end != ',') return 0;
    ptr = end + 1;

    cmd2 = strtof(ptr, &end);
    if (end == ptr) return 0;

    cmd->type = CMD_DATA;
    cmd->cmd0   = cmd0;
    cmd->cmd1   = cmd1;
    cmd->cmd2   = cmd2;

    return 1;
}
